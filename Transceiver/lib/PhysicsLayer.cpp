//
// Created by locate on 2019/10/29.
//

#include "PhysicsLayer.hpp"
#include <utility>

PhysicsLayer::PhysicsLayer(UHDDevice::sptr &dev, const std::string &id) {

    _dev = dev;
    _device_id = id;

    //生成crc多项式
    _gen = std::make_shared<Generator>(CRC16Generator, 16);

    _syn_detector = DetectSynSeq::make();

    SetPN();//设置PN序列

    ReadSeq0();
}

PhysicsLayer::~PhysicsLayer() {
    Stop();
    LOG(INFO) << "~PhysicsLayer()";
}

std::shared_ptr<PhysicsLayer> PhysicsLayer::make(UHDDevice::sptr &dev, const std::string &id) {
    return std::make_shared<PhysicsLayer>(dev, id);
}

void PhysicsLayer::SetPN() {
    _pn.resize(270, std::complex<double>(0, 0));
    std::vector<std::complex<double>> pn_tmp(64);
    for (size_t j = 0; j < pn_tmp.size(); ++j) {
        pn_tmp.at(j) = std::complex<double>(sreal[j], simag[j]);
        _pn.at(j) = pn_tmp.at(j) * _multiple;
    }

    _pn_detector = DetectSynSeq::make();
    _pn_detector->SetSyn(pn_tmp);
}

void PhysicsLayer::Start() {
    _loop = true;//set loop as true to start work thread
    _work_thread = std::make_shared<std::thread>(std::bind(&PhysicsLayer::Work, this));
    _hop_thread = std::make_shared<std::thread>(std::bind(&PhysicsLayer::HopThread, this));
}

void PhysicsLayer::Stop() {
    _loop = false;

    _start_flag = false;
    _work_ready = true;
    _work_cv.notify_all();

    if (_work_thread.get() && _work_thread->joinable()) {
        _work_thread->join();
    }

    _hop_ready = true;
    _hop_cv.notify_all();
    if (_hop_thread.get() && _hop_thread->joinable()) {
        _hop_thread->join();
    }

    _dev->Stop();
}

void PhysicsLayer::StartWork() {
    if (_start_flag) {
        return;
    }

    if (!_dev) {
        std::string str = "No device";
        LOG(INFO) << str;
        SendException(str);
        return;
    }

    _first_recv = true;

    if (_seq0.empty()) {
        _seq0.resize(_iq_num, std::complex<short>(0, 0));
    }

    //清空预发送缓冲区
    std::unique_lock<std::mutex> mtf_ul(_mtf_mtx);
    while (!_mod_data.empty()) {
        _mod_data.pop();
    }

    //发送缓冲区复位
    PhyFrame::ST st;
    for (size_t j = 0; j < _tx_fifo.size(); ++j) {
        _tx_fifo.at(j).SetVacant(j > 25);//初始化时,前25个置为不空闲状态
        _tx_fifo.at(j).SetTimestamp(0);
        _tx_fifo.at(j).AddSamples(_seq0);
        _tx_fifo.at(j).SetST(st);
    }
    mtf_ul.unlock();

    _dev->SetTxStream();
    _dev->SetRxStream();
    _dev->StartRxStream();

    _timeout.Reset();
    _pst.Reset();
    ResetEmptyFlag();

    //通知线程
    _start_flag = true;
    _work_ready = true;
    _work_cv.notify_all();
}

void PhysicsLayer::StopWork() {

    if (!_dev) {
        std::string str = "No device";
        LOG(INFO) << str;
        SendException(str);
        return;
    }

    _start_flag = false;

    _work_ready = false;
    _work_cv.notify_all();

    _hop_ready = false;
    _hop_cv.notify_all();

    _dev->StopRxStream();
}

void PhysicsLayer::BindCallback(call_back_t cb) {
    _cb = std::move(cb);
}

void PhysicsLayer::SetTrans(EasyConnectIface::sptr &trans) {
    _trans = trans;
}

void PhysicsLayer::SetSyn(const std::vector<std::complex<double>> &syn, bool need_reverse_syn,
                          std::complex<double> multiple) {

    std::lock_guard<std::mutex> lg(_work_mtx);

    //设置同步序列
    _syn_detector->SetSyn(syn, need_reverse_syn);
    _syn_detector->GetSyn(_syn, multiple);
    _syn_num = _syn.size();
    _multiple = multiple;
}

void PhysicsLayer::CopeSetParam(double tx_freq, double rx_freq, double tx_lo_off, double rx_lo_off,
                                double sampling_rate, double tx_gain, double rx_gain, int MTU, int delay, int TSC,
                                int insert_tsc_per_bytes, int tx_fifo_size) {

    std::lock_guard<std::mutex> lg(_work_mtx);

    SetDevParam(tx_freq, rx_freq, tx_lo_off, rx_lo_off, sampling_rate, tx_gain, rx_gain);

    SetTone(round(_dev->GetRxRate() / 4));

    SetTSOffset(delay);

    _MTU = MTU;
    _actual_mtu = _MTU + EXPENSES;
    LOG(INFO) << "MTU: " << _MTU << ", actual MTU: " << _actual_mtu;

    _TSC = TSC;
    LOG(INFO) << "TSC: " << _TSC;

    CalculateIqSamples(insert_tsc_per_bytes);

    uhd::time_spec_t frame_t = uhd::time_spec_t::from_ticks(_samples_per_package, _dev->GetRxRate());
    _one_frame_t = frame_t.get_full_secs() + frame_t.get_frac_secs();
    LOG(INFO) << "_one_frame_t: " << _one_frame_t;

    _hop_ready = false;

    //调制
    _modulator.reset();
    _modulator = ModulatorBase::make(_signal_type, _iq_num, _TSC);
    _demodulator.reset();
    _demodulator = DemodulatorBase::make(_signal_type, _iq_num, _TSC);

    //creat tx fifo
    _tx_fifo_size = tx_fifo_size;
    SetFifo();

    LOG(INFO) << "device started at " << _dev->GetTimeNow().get_real_secs() << "s";
}

void PhysicsLayer::SetDevParam(double tx_freq, double rx_freq, double tx_lo_off, double rx_lo_off, double sampling_rate,
                               double tx_gain, double rx_gain) {

    _dev->SetTxFreq(tx_freq, tx_lo_off);
    _dev->SetRxFreq(rx_freq, rx_lo_off);

    _dev->SetTxRate(sampling_rate);
    _dev->SetRxRate(sampling_rate);

    LOG(INFO) << "clock rate is " << _dev->GetMasterClockRate();

    _dev->SetTxBandwidth(sampling_rate);
    _dev->SetRxBandwidth(sampling_rate);

    _dev->SetTxGain(tx_gain);
    _dev->SetRxGain(rx_gain);
}

void PhysicsLayer::SetTone(double fs) {
    double rate = fs * 4;//采样率为信号频率的4倍
    double dt = 1 / rate;
    _tone.resize(_syn_num, std::complex<short>(0, 0));
    for (int j = 0; j < _syn_num; ++j) {
        _tone.at(j) = (std::complex<double>(cos(2 * M_PI * fs * dt * j), sin(2 * M_PI * fs * dt * j)) *
                       _multiple.real());
    }
}

void PhysicsLayer::SetTSOffset(int delay) {
    _timestamp_offset = TIMESTAMP(std::round(((double) delay / 1e3) * _dev->GetRxRate()));//接收到开始发射之间有延时
    LOG(INFO) << "timestamp offset = " << _timestamp_offset;
}

void PhysicsLayer::CalculateIqSamples(int insert_tsc_per_bytes) {
    //计算插入tsc的数量
    _insert_tsc_per_bytes = insert_tsc_per_bytes;
    _insert_tsc_num = _actual_mtu / _insert_tsc_per_bytes;
    if (_actual_mtu % _insert_tsc_per_bytes != 0) {
        _insert_tsc_num++;
    }
    LOG(INFO) << "insert TSC per bytes: " << _insert_tsc_per_bytes << ", insert TSC times: " << _insert_tsc_num;

    //IQ num = 数据(序号 数据 crc) + TSC + 保护周期
    switch (_signal_type) {
        case BPSK: {
            _iq_num =
                    _actual_mtu * 8 * 2 + _insert_tsc_num * (int) (gTrainingSequence[_TSC].size() + GUARDPERIODLENGTH);
            _iq_num *= 4;
            break;
        }
        case GMSK:
        default: {
            _iq_num =
                    _actual_mtu * 8 * 2 + _insert_tsc_num * (int) (gTrainingSequence[_TSC].size() + GUARDPERIODLENGTH);
            break;
        }
    }
    _samples_per_package = _iq_num + _syn_num + (int) _tone.size();
    LOG(INFO) << "iq num: " << _iq_num << ", samples per package: " << _samples_per_package;
}

void PhysicsLayer::SetFifo() {
    //清空预发送缓冲区
    std::unique_lock<std::mutex> mtf_ul(_mtf_mtx);
    while (!_mod_data.empty()) {
        _mod_data.pop();
    }

    //清空发送缓冲区
    _tx_fifo.clear();
    PhyFrame frame(_tone, _syn, _iq_num);
    frame.AddSamples(_seq0);
    for (int j = 0; j < _tx_fifo_size; ++j) {
        frame.SetVacant(j > 2);
        _tx_fifo.emplace_back(frame);
    }
    mtf_ul.unlock();

    _recv_data.resize(_samples_per_package * 2, std::complex<short>(0, 0));
    _valid_data.resize(_iq_num, std::complex<short>(0, 0));

    _recv_buf._buf.resize(_samples_per_package);
    _recv_buf._timestamp = 0.0;
}

void PhysicsLayer::Work() {
    while (_loop) {
//        auto now = std::chrono::high_resolution_clock::now();
        std::unique_lock<std::mutex> ul(_work_mtx);
        _work_cv.wait(ul, [this]() {
            return _work_ready;
        });

        if (!_start_flag) {
            continue;
        }

        PullBuffer();
        PushBuffer();
//        auto elapse = std::chrono::duration_cast<std::chrono::milliseconds>(
//                std::chrono::high_resolution_clock::now() - now);
//        LOG(INFO) << "per cycle cost: " << elapse.count() << "ms";

//        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }

    LOG(INFO) << "work thread exit...";
}

size_t PhysicsLayer::PullBuffer() {

    size_t recv_size = 0;
    try {
//        auto now = std::chrono::high_resolution_clock::now();
        recv_size = _dev->Recv(_recv_buf._buf, _samples_per_package, &_recv_buf._timestamp, _timeout.rx);
//        auto elapse = std::chrono::duration_cast<std::chrono::milliseconds>(
//                std::chrono::high_resolution_clock::now() - now);
//        LOG(INFO) << "recv cost: " << elapse.count() << "ms";
    } catch (uhd::exception &e) {
        LOG(INFO) << e.what();
        _dev->StopRxStream();

        _start_flag = false;
        SendException(e.what());

        return -1;
    }

    _timeout.rx = 1.0;

    TIMESTAMP current_recv_timestamp = _recv_buf._timestamp.to_ticks(_dev->GetRxRate());
    _next_send_timestamp = current_recv_timestamp + _samples_per_package + _timestamp_offset;//理论上等于下一包接收的时间戳

    if (_first_recv) {//获取第一次启动接收的时间戳
        _first_recv = false;

        //初始化发送缓冲区的时间戳
        //为了确保接收和发送稳定 接收20包后再启动发送
        //_timestamp_offset 考虑到接收到发送需要一些处理
        TIMESTAMP init_send_timestamp = current_recv_timestamp + _samples_per_package * 20 + _timestamp_offset;

        InitSendFifo(init_send_timestamp);
    }

    CopeRecvData();

    return recv_size;
}

void PhysicsLayer::InitSendFifo(TIMESTAMP timestamp) {
    std::unique_lock<std::mutex> mtf_ul(_mtf_mtx);
    for (auto &t : _tx_fifo) {
        t.SetTimestamp(timestamp);
        timestamp += _samples_per_package;

        if (!_mod_data.empty() && t.GetVacant()) {
            t.SetVacant(false);
            t.AddSamples(_mod_data.front().modBuf);
            t.SetST(_mod_data.front().st);
            _mod_data.pop();
        }

    }
}

void PhysicsLayer::CopeRecvData() {
    uint64_t tsp = std::round((_recv_buf._timestamp.get_full_secs() + _recv_buf._timestamp.get_frac_secs()) * 1000);
//    LOG(INFO) << "tsp: " << tsp << "ms";

    //提取接收fifo里面全部的内容
//    auto now = std::chrono::high_resolution_clock::now();
    memcpy(_recv_data.data(), &_recv_data.at(_samples_per_package), sizeof(std::complex<short>) * _samples_per_package);
    memcpy(&_recv_data.at(_samples_per_package), _recv_buf._buf.data(),
           sizeof(std::complex<short>) * _samples_per_package);
    _syn_detector->DetectSynWork(_recv_data);
//    auto elapse = std::chrono::duration_cast<std::chrono::milliseconds>(
//            std::chrono::high_resolution_clock::now() - now);
//    LOG(INFO) << "cope recv data count: " << elapse.count() << "ms";

    double snr = 0.0;
    unsigned syndrome = 1;//不等于0就可以了
    BitVector decode_data;//译码后的数据
    if (_syn_detector->GetFindSyn() &&
        (_syn_detector->GetSymbolCount() <= (_samples_per_package + _syn_num + (int) _tone.size()))) {

        //提取数据
        memcpy(_valid_data.data(), &_recv_data.at(_syn_detector->GetSymbolCount()),
               sizeof(std::complex<short>) * _iq_num);

        SoftVector demod_data;
        int ret = _demodulator->DemodWork(_actual_mtu, _insert_tsc_num, _insert_tsc_per_bytes,
                                          _valid_data, demod_data, snr);

        if (ret != 0) {
//            SendSNR(snr / _insert_tsc_num);
            //回调函数
            _cb((_recv_buf._timestamp.get_full_secs() + _recv_buf._timestamp.get_frac_secs()), decode_data, syndrome, snr);
            SendSNR(0.0, tsp, true);
            return;
        } else {
            SendSNR(snr, tsp, true);
        }
//            LOG(INFO) << "snr: " << snr;

        //viterbi译码
        decode_data = _demodulator->ViterbiDecode(demod_data);

        //crc校验
        //由于crc编码时对crc检验码进行了取反, 接收时也要将附加在数据尾部的crc校验码翻取反
        BitVector crc_check(decode_data.tail(40/*序号 空格 传输类型共40bit*/ + _MTU * 8));
        crc_check.invert();

        //计算余数 余数不等于0说明有误码
        syndrome = decode_data.syndrome(*_gen);

        //每8bit翻转
        decode_data.LSB8MSB();
//            LOG(INFO) << "SNR: " << snr;

        //回调函数
        //_cb((_recv_buf._timestamp.get_full_secs() + _recv_buf._timestamp.get_frac_secs()), _decode_data, syndrome, snr);

        int tc = _syn_detector->GetSymbolCount() - _samples_per_package;
        if (tc > 0 && tc <= int(_syn.size() + _tone.size())) {
            memset(&_recv_data.at(_samples_per_package), 0, sizeof(std::complex<short>) * tc);//防止重复解调同一组数
        }
    } else {
        SendSNR(snr, tsp, false);
    }

    //TODO fix me
    //回调函数
    _cb((_recv_buf._timestamp.get_full_secs() + _recv_buf._timestamp.get_frac_secs()), decode_data, syndrome, snr);
}

void PhysicsLayer::SendSNR(double snr, uint64_t tsp, bool detect_syn) {
    if (_device_id == "transmitter")return;
    flatbuffers::FlatBufferBuilder builder;
    auto snr_f = EasyConnect::CreateSnrFrame(builder, snr, tsp, detect_syn);
    builder.Finish(snr_f);

    auto command_f = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_SNR, false,
                                                     EasyConnect::CommandData_SnrFrame, snr_f.Union());
    builder.Finish(command_f);

    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, command_f.Union(),
                                          builder.CreateString(_device_id));
    builder.Finish(msg);

    _trans->send_to(_Receiver.gui, builder.GetCurrentBufferPointer(), builder.GetSize());
    _trans->send_to(_Receiver.preception, builder.GetCurrentBufferPointer(), builder.GetSize());

//    if (snr != 0) {
//        LOG(INFO) << "SNR: " << snr;
//    }
}

size_t PhysicsLayer::PushBuffer() {
    size_t send_size = 0;

    std::unique_lock<std::mutex> mtf_ul(_mtf_mtx);
    int d_value = (int) (_tx_fifo.front().GetTimestamp() - _next_send_timestamp);//对比时间戳
    if (d_value < 0) {//如果时间戳已过
        LOG(INFO) << "预计发送的时间标签落后设备: " << d_value << "个样值, 将重新调整";
        ResetSendFifoTimestamp(_next_send_timestamp);
    } else if (d_value >= 10) {
        LOG(INFO) << "预计发送的时间标签超前设备： " << d_value << "个样值, 将重新调整";
        ResetSendFifoTimestamp(_next_send_timestamp);
    }

    if (!_tx_fifo.empty()) {
        send_size = _dev->Send(_tx_fifo.front().GetBuffer(), _samples_per_package, _tx_fifo.front().GetTimestamp(),
                               _timeout.tx);

        if (_tx_fifo.front().GetVacant()) {
            std::lock_guard<std::mutex> lg(_empty_mtx);
            ++_emptyFlag;
            if (_emptyFlag.count > _tx_fifo.size()) {
                _emptyFlag.empty = true;
            }
        } else {
            ResetEmptyFlag();
        }
        _pst = _tx_fifo.front().GetST();

        PopPushFrame();
    } else {
        LOG(INFO) << "Not Data to Send!";
    }
    mtf_ul.unlock();

    _timeout.tx = 1.0;

    return send_size;
}

PhyFrame::ST PhysicsLayer::GetPhyFrameST() {
    return _pst;
}

void PhysicsLayer::ResetEmptyFlag() {
    std::lock_guard<std::mutex> lg(_empty_mtx);
    _emptyFlag.Reset();
}

bool PhysicsLayer::GetEmptyFlag() {
    std::lock_guard<std::mutex> lg(_empty_mtx);
    return _emptyFlag.empty;
}

void PhysicsLayer::ResetSendFifoTimestamp(TIMESTAMP timestamp) {
    if (_tx_fifo.empty()) return;
    for (size_t j = 0; j < _tx_fifo.size(); ++j) {
        _tx_fifo.at(j).SetTimestamp(timestamp + _samples_per_package * j);
    }
}

void PhysicsLayer::PopPushFrame() {
    _tx_fifo.pop_front();

    PhyFrame frame(_tone, _syn, _iq_num);
    frame.SetTimestamp(CalculateNextAddSendFrameTimestamp());
    frame.AddSamples(_seq0);

    if (!_mod_data.empty()) {
        frame.SetVacant(false);
        frame.AddSamples(_mod_data.front().modBuf);
        frame.SetST(_mod_data.front().st);
        _mod_data.pop();
    } else {
        frame.SetVacant(true);
    }

    _tx_fifo.emplace_back(frame);
}

TIMESTAMP PhysicsLayer::CalculateNextAddSendFrameTimestamp() {
    TIMESTAMP ts = _tx_fifo.back().GetTimestamp() + _samples_per_package;
    return ts;
}

void PhysicsLayer::RecvFromLinkLayer(const std::list<LinkFrame> &input) {
    if (input.empty()) {
        LOG(INFO) << "No content";
        return;
    }

    //检查调制器
    if (!_modulator) {
        LOG(INFO) << "parameters not configured!!!!";
        return;
    }

    BitVector code;//编码后的数据
    std::vector<std::complex<float>> fc32_mod;//调制后的数据
    std::vector<ModBuffer> modBufVec(input.size());

    auto input_it = input.begin();
    for (size_t j = 0; j < input.size(); ++j) {
        code.clear();
        fc32_mod.clear();

        _modulator->CoderWork(input_it->GetAllBit(), code);//编码
        _modulator->ModWork(_insert_tsc_num, _insert_tsc_per_bytes, code, fc32_mod);//调制

        modBufVec.at(j).st.isAck = (input_it->GetFrameType() == ACK_FRAME);
        modBufVec.at(j).st.ordinal = input_it->GetOrdinal();
        if (input_it->GetTransType() == TRANS_FILE) {
            modBufVec.at(j).st.type = PhyFrame::Type::File;
        } else {
            modBufVec.at(j).st.type = PhyFrame::Type::Str;
        }

        modBufVec.at(j).modBuf.resize(fc32_mod.size());
        for (size_t k = 0; k < fc32_mod.size(); ++k) {
            modBufVec.at(j).modBuf.at(k) = static_cast<std::complex<short>>(fc32_mod.at(k) *
                                                                            static_cast<std::complex<float>>(_multiple));
        }

        input_it++;
    }

    //WriteSeq0(modBufVec.at(0).modBuf.data(), modBufVec.at(0).modBuf.size());

    InsertIQDataToSendFifo(modBufVec);
}

void PhysicsLayer::InsertIQDataToSendFifo(const std::vector<ModBuffer> &iq_data) {

    if (iq_data.empty()) {
        return;
    }

    std::unique_lock<std::mutex> mtf_ul(_mtf_mtx);
    for (const auto &mod : iq_data) {
        _mod_data.emplace(mod);//暂存未发送的IQ
    }

    for (auto &f : _tx_fifo) {
        if (f.GetVacant() && !_mod_data.empty()) {
            f.SetVacant(false);
            f.AddSamples(_mod_data.front().modBuf);
            f.SetST(_mod_data.front().st);
            _mod_data.pop();
            if (_mod_data.empty()) {
                break;
            }
        }
    }
}

uhd::time_spec_t PhysicsLayer::GetSystemTimeUsec() {
    timeval tv{};
    gettimeofday(&tv, nullptr);
    return {tv.tv_sec, tv.tv_usec / 1000000.0};
}

void PhysicsLayer::SetHopFreq(uint64_t time, double freq, HopType type) {
    std::lock_guard<std::mutex> lg(_hop_mtx);

    if (_hop_ready) {
        LOG(INFO) << "上一次的指令还未执行完";
    }

    _hop_param.time = time / 1000.0;//毫秒转换为秒.秒
    _hop_param.freq = freq;
    _hop_type = type;

    _hop_ready = true;
    _hop_cv.notify_all();
}

void PhysicsLayer::HopThread() {

    while (_loop) {

        std::unique_lock<std::mutex> ul(_hop_mtx);
        _hop_cv.wait(ul, [this]() {
            return _hop_ready;
        });

        if (!_start_flag) {
            continue;
        }

        FreqHopping();

    }

    LOG(INFO) << "set freq thread exit...";

}

void PhysicsLayer::FreqHopping() {

    //设备当前时间
    auto dev_time = _dev->GetTimeNow();
    double real_time = dev_time.get_full_secs() + dev_time.get_frac_secs();

    //误差
    double time_diff = _hop_param.time - real_time;

    if (time_diff < -(_one_frame_t / 2.0)) {
        //超过误差
        std::string str = (boost::format("time: %lfs timout, now device time: %lfs") % _hop_param.time %
                           real_time).str();
        LOG(INFO) << str;
        SendException(str);
        _hop_ready = false;
        return;
    }

    if (std::abs(time_diff) <= _one_frame_t) {
        //在误差以内
        switch (_hop_type) {
            case HopType::RX: {
                _dev->SetRxFreq(_hop_param.freq, 0);
                break;
            }
            case HopType::TX: {
                _dev->SetTxFreq(_hop_param.freq, 0);
                break;
            }
            default: {
                _dev->SetRxFreq(_hop_param.freq, 0);
                _dev->SetTxFreq(_hop_param.freq, 0);
                break;
            }
        }
        SendHeart();
        _hop_ready = false;
    }
}

void PhysicsLayer::SendHeart() {
    flatbuffers::FlatBufferBuilder builder;
    auto heart_f = EasyConnect::CreateStateFrame(builder, _dev->GetTxFreq(), _dev->GetRxFreq());
    builder.Finish(heart_f);

    auto command_frame = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_STATE, false,
                                                         EasyConnect::CommandData_StateFrame, heart_f.Union());
    builder.Finish(command_frame);

    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, command_frame.Union(),
                                          builder.CreateString(_device_id));
    builder.Finish(msg);

    _trans->send_to(_Receiver.gui, builder.GetBufferPointer(), builder.GetSize());
}

void PhysicsLayer::ClearPhyscisData() {

    std::vector<std::complex<short>> tmp(_iq_num, std::complex<short>(0, 0));
    std::unique_lock<std::mutex> mtf_ul(_mtf_mtx);
    while (!_mod_data.empty()) {
        _mod_data.pop();
    }

    PhyFrame::ST st;
    for (auto &tf:_tx_fifo) {
        if (!tf.GetVacant()) {
            tf.SetVacant(true);
            tf.AddSamples(tmp);
            tf.SetST(st);
        }
    }
}

void PhysicsLayer::SendPNSeq(uhd::time_spec_t *ts) {
//    std::lock_guard<std::mutex> lg(_work_mtx);//确保收发线程已经停止
    _dev->SetTxStream();
    _dev->Send(_pn, _pn.size(), ts);

    //阻塞
//    while (GetSystemTimeUsec().get_real_secs() >= ts->get_real_secs()) {
//        break;
//    }

    LOG(INFO) << "SendPNSeq()";
}

void PhysicsLayer::DetectPNSeq(uhd::time_spec_t *ts) {
//    std::lock_guard<std::mutex> lg(_work_mtx);//确保收发线程已经停止
    if (!_detect_pn_ready) {
        //防止本次未执行完,新的指令又下来了
        LOG(INFO) << "wait a minute, and try again";
        return;
    }
    std::vector<std::complex<short>> recv_PN(_pn.size(), std::complex<short>(0, 0));
    std::vector<std::complex<short>> recv_data(2 * recv_PN.size());

    auto now = std::chrono::high_resolution_clock::now();

    auto tts = ts->get_full_secs() + ts->get_frac_secs();
    uhd::time_spec_t recv_time;
    _dev->SetRxStream();
    _dev->StartRxStream();
    while (_dev->GetTimeNow().get_real_secs() <= (tts + 1.0)) {

        _detect_pn_ready = false;

        _dev->Recv(recv_PN, recv_PN.size(), &recv_time);//接收数据

        //拼接数据
        memcpy(&recv_data.at(0), &recv_data.at(recv_PN.size()), sizeof(std::complex<short>) * recv_PN.size());
        memcpy(&recv_data.at(recv_PN.size()), recv_PN.data(), sizeof(std::complex<short>) * recv_PN.size());

        _pn_detector->DetectSynWork(recv_data);//检测PN数据
        if (_pn_detector->GetFindSyn()) {
            auto rt = recv_time.get_full_secs() + recv_time.get_frac_secs();//该包第一个样值的时间
            int pos = (_pn_detector->GetSymbolCount() - 64) - (int) recv_PN.size();//找到PN包第一个样值的位置
            double find_t = rt + (pos / _dev->GetRxRate());//确定PN数据包第一个样值的时间
            LOG(INFO) << "find PN Seq: " << pos
                      << (boost::format(", at: %lf, send time: %lf, diff: %lf") %
                          find_t %
                          tts %
                          (find_t - tts)).str();
//                          (tts + ((64 - 1) / _dev->GetTxRate())) %
//                          (find_t - (tts + ((64 - 1) / _dev->GetTxRate())))).str();

            int tc = _pn_detector->GetSymbolCount() - (int) recv_PN.size();
            if (tc > 0 && tc <= 64) {
                memset(&recv_data.at(recv_PN.size()), 0, sizeof(std::complex<short>) * tc);
            } else if (tc > 64) {
                memset(&recv_data.at(recv_PN.size() + (tc - 64)), 0, sizeof(std::complex<short>) * 64);
            }
        }

        //超时退出
        auto elapse = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::high_resolution_clock::now() - now);
        if (elapse.count() > 300) break;
    }

    _dev->StopRxStream();
    _detect_pn_ready = true;
}

void PhysicsLayer::SendException(const std::string &str) {
    flatbuffers::FlatBufferBuilder builder;
    auto err = EasyConnect::CreateStringFrame(builder, builder.CreateString(str));
    builder.Finish(err);
    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Exception, false,
                                               EasyConnect::CommandData_StringFrame, err.Union());
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(_device_id));
    builder.Finish(msg);
    _trans->send_to(_Receiver.gui, builder.GetCurrentBufferPointer(), builder.GetSize());
}

bool PhysicsLayer::CheckLinkDataSize() {
    if (_device_id == "receiver")return false;

    std::unique_lock<std::mutex> mtf_ul(_mtf_mtx);
    size_t lsize = _mod_data.size();///使用前应当注意外部是加锁了
    size_t tsize = _tx_fifo.size();
    mtf_ul.unlock();

    auto full_load = 10 * tsize;

    if (full_load == 0) {
        full_load = 1;
    }

#if 1

    double capacity = lsize / (double) full_load;

    if (capacity >= 0.9) {
        SendCapacity(true, capacity);
        return true;
    } else if (capacity <= 0.2) {
        SendCapacity(false, capacity);
        return false;
    }
#else
    auto allow_num = full_load - lsize;
    auto plan_num = lsize + plan_add;
    double capacity = plan_num / (double) full_load;
    SendCapacity(capacity < 0.9, allow_num);

#endif
    return false;
}

void PhysicsLayer::SendCapacity(bool full_load, double capacity) {
    if (_device_id == "receiver")return;
    flatbuffers::FlatBufferBuilder builder;
    auto full_f = EasyConnect::CreateFullLoadFrame(builder, full_load, capacity);
    builder.Finish(full_f);
    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_FULL_LOAD, true,
                                               EasyConnect::CommandData_DoubleFrame, full_f.Union());
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(_device_id));
    builder.Finish(msg);
    _trans->send_to(_Receiver.gui, builder.GetBufferPointer(), builder.GetSize());
    LOG(INFO) << "send capacity: " << capacity << " to " << _Receiver.gui;
}

void PhysicsLayer::WriteSeq0(std::complex<short> *buf, size_t size) {
    std::ofstream ofs("modSamplesSeq0.dat", std::ios::binary | std::ios::trunc);
    if (ofs.good()) {
        ofs.write((char *) &size, sizeof(uint64_t));
        ofs.write((char *) buf, sizeof(std::complex<short>) * size);
        ofs.close();
        LOG(INFO) << "Write IQ to modSamplesSeq0.dat, size: " << size;
    }
}


void PhysicsLayer::ReadSeq0() {

    uint64_t size = 0;

    std::ifstream ifs("modSamplesSeq0.dat", std::ios::binary);
    if (ifs.good()) {
        ifs.read((char *) &size, sizeof(uint64_t));
        _seq0.resize(size, 0);
        ifs.read((char *) _seq0.data(), sizeof(std::complex<short>) * size);
        ifs.close();
    }
    LOG(INFO) << "read modSamplesSeq0.dat, size: " << size;
}