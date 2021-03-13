//
// Created by locate on 2019/11/21.
//

#include "LinkLayer.hpp"

#include <utility>
#include <flatbuffers/flatbuffers.h>
#include <easy_connect_generated.h>
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/json.h>

LinkLayer::LinkLayer(const Json::Value &value) {

    //device id
    if (!value["device_id"].empty() && value["device_id"].isString()) {
        _device_id = value["device_id"].asString();
    }
    //udp port
    if (!value["udp_port"].empty() && value["udp_port"].isInt()) {
        _udp_port = value["udp_port"].asInt();
    }
    //update file directory
    if (!value["send_file_dir"].empty() && value["send_file_dir"].isString()) {
        _send_file_dir = value["send_file_dir"].asString();
    }
    if (!value["recv_file_dir"].empty() && value["recv_file_dir"].isString()) {
        _recv_file_dir = value["recv_file_dir"].asString();
    }
    //未使用 有需要可在配置文件添加
    if (!value["reverse_syn"].empty()) {
        _physics_params.reverse_syn = value["reverse_syn"].asBool();
    }
    if (!value["multiple"].empty()) {
        _physics_params.multiple = std::complex<double>(value["multiple"].asDouble(), 0);
        if (_physics_params.multiple.real() < 0) {
            _physics_params.multiple = 8192.0;
        }
    }
    if (!value["ntp_server_ip"].empty() && value["ntp_server_ip"].isString()) {
        _ntp_server_ip = value["ntp_server_ip"].asString();
    }
    if (!value["retransmission_gap"].empty() && value["retransmission_gap"].isInt()) {
        _link_params.retransmission_gap = value["retransmission_gap"].asInt();
    }
    LOG(INFO) << "Device id is " << _device_id;
    LOG(INFO) << "Udp port is " << _udp_port;
    LOG(INFO) << "Send file dir: " << _send_file_dir;
    LOG(INFO) << "Recv file dir: " << _recv_file_dir;
    LOG(INFO) << "Multiple: " << _physics_params.multiple;
    LOG(INFO) << "Ntp server ip: " << _ntp_server_ip;


    //创建一个trans需要的配置文件
    EasyConnectIface::Config config{};
    config.id = _device_id;//id是本地身份标识，任意字符串，用于区分不同的设备
    config.udp_port = (short) _udp_port;//udp端口，监听该端口，用于获取主控端广播的UDP包，用于广播UDP的消息配置broker地址

    //create trans
    _trans = EasyConnectIface::make(config, std::bind(&LinkLayer::OnMsg, this,
                                                      std::placeholders::_1,
                                                      std::placeholders::_2,
                                                      std::placeholders::_3));
    _trans->create_broker();

    //create device
    if (!value["device addr"].empty() && value["device addr"].isString()) {
        _dev_addr = value["device addr"].asString();
    }
    if (!CreateDevice(value)) {
        exit(-1);
    }

    _iface = terjin_iface::make(_dev->GetDevice());

    //链路层配置
    ResetSetParam();

    //物理层配置
    _physics = PhysicsLayer::make(_dev, _device_id);
    _physics->SetTrans(_trans);
    _physics->SetSyn(_physics_params.syn, _physics_params.reverse_syn, _physics_params.multiple);//默认使用内部的syn
    _physics->CopeSetParam(_physics_params.tx_freq, _physics_params.rx_freq, _physics_params.tx_lo_off,
                           _physics_params.rx_lo_off,
                           _physics_params.sampling_rate, _physics_params.tx_gain, _physics_params.rx_gain,
                           _MTU, _physics_params.delay, _physics_params.TSC,
                           _physics_params.insert_tsc_per_bytes, _physics_params.tx_fifo_size);
#if 1
    CopeTimeSync(_ntp_server_ip);
#else
    //与ntp服务器同步时间
    _ntp_client = std::make_shared<NTPclient>(_ntp_server_ip);
    NTPTimeSync();//与ntp服务器同步时间
    if (!_use_pps) {
        _dev->SetTimeNow(_physics->GetSystemTimeUsec());//配置设备时间
    } else {
        _dev->SetTimeNextPPS(_physics->GetSystemTimeUsec().get_full_secs() + 1.0);
    }

    _ntp_client->GetNowSystemTime_context();
    for (int j = 0; j < 5; ++j) {
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        LOG(INFO) << (boost::format("System time: %lf, dev time: %lf") %
                      _physics->GetSystemTimeUsec().get_real_secs() %
                      _dev->GetTimeNow().get_real_secs()).str();
    }
#endif

    //开启线程
    Start();
    _physics->Start();

    //绑定回调
    BindCallback(std::bind(&PhysicsLayer::RecvFromLinkLayer, _physics, std::placeholders::_1));
    _physics->BindCallback(
            std::bind(&LinkLayer::RecvFromPhysicsLayer, this, std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3, std::placeholders::_4));
    _make = true;

    DisplayParameters();
    LOG(INFO) << "Link create success";
}

LinkLayer::~LinkLayer() {
    Stop();
    _physics->Stop();
    LOG(INFO) << "~LinkLayer()";
}

LinkLayer::sptr LinkLayer::make(const Json::Value &value) {
    return std::make_shared<LinkLayer>(value);
}

bool LinkLayer::CreateDevice(const Json::Value &value) {
    LOG(INFO) << "start to init a normal device";
    try {
        //try to make a device
        _dev = UHDDevice::make(_dev_addr, _trans, _device_id);
    } catch (uhd::exception &e) {
        //make device failed
        LOG(INFO) << "init a normal device error: " << e.what();

        //send message to upper levels
        SendException((boost::format("init device error: %s") % e.what()).str());
        return false;
    } catch (...) {
        //make device failed
        LOG(INFO) << "init a normal device error";

        //send message to upper levels
        SendException((boost::format("init device error: %s")).str());
        return false;
    }

    if (!value["clock_source"].empty() && value["clock_source"].isString()) {
        auto clos = (value["clock_source"].asString());

        try {
            _dev->SetTimeSource(clos);
        } catch (uhd::exception &e) {
            LOG(INFO) << e.what();
            SendException((boost::format(e.what())).str());
            return false;
        }

        try {
            _dev->SetClockSource(clos);
        } catch (uhd::exception &e) {
            LOG(INFO) << e.what();
            SendException((boost::format(e.what())).str());
            return false;
        }

        if (clos != "internal") {
            ///check for 10Mhz lock
            bool ref_locked = false;
            for (int j = 0; j < 10 and !ref_locked; ++j) {
                LOG(INFO) << "Waiting for reference lock...";
                ref_locked = _dev->GetMboardSensor();
                if (!ref_locked) {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
            _use_pps = ref_locked;
        }
    }

    if (!value["up_freq"].empty() && value["up_freq"].isDouble()) {
        if (_device_id == "transmitter") {
            _physics_params.tx_freq = (value["up_freq"].asDouble());
        } else {
            _physics_params.rx_freq = (value["up_freq"].asDouble());
        }
    }

    if (!value["down_freq"].empty() && value["down_freq"].isDouble()) {
        if (_device_id == "transmitter") {
            _physics_params.rx_freq = (value["down_freq"].asDouble());
        } else {
            _physics_params.tx_freq = (value["down_freq"].asDouble());
        }
    }

    ///配置文件提取 有需要可在配置文件添加 目前未添加
    if (!value["tx_antenna"].empty() && value["tx_antenna"].isString()) {
        _physics_params.tx_antenna = (value["tx_antenna"].asString());
        try {
            _dev->SetTxAntenna(_physics_params.tx_antenna);
        } catch (uhd::exception &e) {
            LOG(INFO) << e.what();
            SendException((boost::format(e.what())).str());
            return false;
        }
    }

    if (!value["rx_antenna"].empty() && value["rx_antenna"].isString()) {
        _physics_params.rx_antenna = (value["rx_antenna"].asString());
        try {
            _dev->SetRxAntenna(_physics_params.rx_antenna);
        } catch (uhd::exception &e) {
            LOG(INFO) << e.what();
            SendException((boost::format(e.what())).str());
            return false;
        }
    }
    if (!value["tx_subdev_spec"].empty() && value["tx_subdev_spec"].isString()) {
        _physics_params.tx_subdev_spec = (value["tx_subdev_spec"].asString());
        try {
            _dev->SetTxSubdev(_physics_params.tx_subdev_spec);
        } catch (uhd::exception &e) {
            LOG(INFO) << e.what();
            SendException((boost::format(e.what())).str());
            return false;
        }
    }

    if (!value["rx_subdev_spec"].empty() && value["rx_subdev_spec"].isString()) {
        _physics_params.rx_subdev_spec = (value["rx_subdev_spec"].asString());
        try {
            _dev->SetRxSubdev(_physics_params.rx_subdev_spec);
        } catch (uhd::exception &e) {
            LOG(INFO) << e.what();
            SendException((boost::format(e.what())).str());
            return false;
        }
    }

    if (!value["master_clock_rate"].empty()) {
        _physics_params.clk_rate = (value["master_clock_rate"].asDouble());
        try {
            _dev->SetMasterClockRate(_physics_params.clk_rate);
        } catch (uhd::exception &e) {
            LOG(INFO) << e.what();
            SendException((boost::format(e.what())).str());
            return false;
        }
    }

    if (!value["tx_lo_off"].empty()) {
        _physics_params.tx_lo_off = value["tx_lo_off"].asDouble();
    }
    if (!value["rx_lo_off"].empty()) {
        _physics_params.rx_lo_off = value["rx_lo_off"].asDouble();
    }

    return true;
}

void LinkLayer::ResetSetParam() {
    std::unique_lock<std::mutex> sf_ul(_sf_mtx);
    _str_fifo.clear();
    sf_ul.unlock();

    std::unique_lock<std::mutex> ff_ul(_ff_mtx);
    _file_fifo.clear();
    ff_ul.unlock();

    std::unique_lock<std::mutex> sfs_ul(_sfs_mtx);
    _send_file_size = 0;
    sfs_ul.unlock();

    _retrans_str_ordinal = 1;
    _retrans_file_ordinal = 1;
    _send_str_ordinal = 1;
    _send_file_ordinal = 1;

    //清空存文件的操作
    _recv_file.clear();
    _file_end_ordinal = 0;
}

bool LinkLayer::NTPTimeSync() {

    int ret = _ntp_client->SetSystemTime();
    if (ret == -999) {
        std::string err = "修改系统时钟失败,原因:没有权限. 如需继续,请尝试使用sudo运行此程序";
        LOG(INFO) << err;
        SendException(err);
        return false;
    } else if (ret == 0) {
        return true;
    } else {
        for (int j = 0; j < 10; ++j) {

            ret = _ntp_client->SetSystemTime();
            if (ret == 0)
                return true;

            if (j == 9) {
                if (ret == -999) {
                    std::string str = "修改系统时钟失败,原因没有权限. 如需继续,请尝试使用sudo运行此程序";
                    LOG(INFO) << str;
                    SendException(str);
                    return false;
                } else {
                    std::stringstream str;
                    str << "ip(" << _ntp_server_ip.c_str() << ")请求失败，可能是因为应答超时...error code: " << ret;
                    LOG(INFO) << str.str();
                    SendException(str.str());
                    return false;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    return true;
}

void LinkLayer::BindCallback(callback_t cb) {
    _cb = std::move(cb);
}

void LinkLayer::DisplayParameters() {
    LOG(INFO) << "Display parameters:\n\n";
    LOG(INFO) << "Device ID: " << _device_id;
    LOG(INFO) << "MTU: " << _MTU << " bytes";
    LOG(INFO) << "TSC: " << _physics_params.TSC;
    LOG(INFO) << "delay: " << _physics_params.delay << " ms";
    LOG(INFO) << "retransmission_gap: " << _link_params.retransmission_gap;
    LOG(INFO) << "max_retransmission_times: " << _link_params.max_retransmission_times;
    LOG(INFO) << "tx_fifo_size: " << _physics_params.tx_fifo_size;
    LOG(INFO) << "tx_freq/rx_freq: " << _dev->GetTxFreq() << " Hz/ " << _dev->GetRxFreq() << " Hz";
    LOG(INFO) << "tx_rate/rx_rate: " << _dev->GetTxRate() << " Hz/ " << _dev->GetRxRate() << " Hz";
    LOG(INFO) << "tx_gain/rx_gain: " << _dev->GetTxGain() << " dB/ " << _dev->GetRxGain() << " dB";
    LOG(INFO) << "syn size: " << _physics_params.syn.size() << "\n\n";
}


void LinkLayer::Start() {
    _loop = true;
    _retrans_thread = std::make_shared<std::thread>(std::bind(&LinkLayer::RetransWork, this));//重传
    _program_state_thread = std::make_shared<std::thread>(std::bind(&LinkLayer::ProgramStateThread, this));//心跳
    _throughput_thread = std::make_shared<std::thread>(std::bind(&LinkLayer::ThoughThread, this));
}

void LinkLayer::Stop() {
    _loop = false;
    _start_flag = false;
    _work_ready = true;
    _cv.notify_all();

    if (_retrans_thread && _retrans_thread->joinable()) {
        _retrans_thread->join();
    }
    if (_program_state_thread && _program_state_thread->joinable()) {
        _program_state_thread->join();
    }
    if (_throughput_thread && _throughput_thread->joinable()) {
        _throughput_thread->join();
    }
}

void LinkLayer::OnMsg(const std::string &id, void *data, size_t size) {
    if (!_make) {
        std::string err = "Program not initialized completed";
        LOG(INFO) << err;
        SendException(err);
        return;
    }

    const uint8_t *buf = (uint8_t *) data;
    flatbuffers::FlatBufferBuilder builder_out;
    builder_out.PushBytes(buf, size);

    flatbuffers::Verifier verifier(builder_out.GetCurrentBufferPointer(), builder_out.GetSize());

    auto flag = EasyConnect::VerifyMessageBuffer(verifier);

    if (!flag) {
        LOG(WARNING) << "wrong frame";
        return;
    }

    auto msg_recv = EasyConnect::GetMessage(buf);

    if (msg_recv->id() == nullptr) {
//        LOG(ERROR) << "message: " << msg_recv->any_type() << " has no id";
        return;
    }
    auto src = msg_recv->id()->str();

    switch (msg_recv->any_type()) {
        case EasyConnect::DataAny_NONE:
            break;
        case EasyConnect::DataAny_SpectrumData:
            break;
        case EasyConnect::DataAny_TestData:
            break;
        case EasyConnect::DataAny_CommandFrame: {
            auto cmd = (EasyConnect::CommandFrame *) msg_recv->any();
            HandleCommand(src, cmd);
            break;
        }
        case EasyConnect::DataAny_FileFrame: {
            if (!_start_flag) {
                std::string str = "Program not started";
                LOG(INFO) << str;
                SendException(str);
                break;
            }

            if (_device_id == "receiver")
                break;

            LOG(INFO) << "recv file from " << src;
#if 0
            auto file_frame = (EasyConnect::FileFrame *) msg_recv->any_as_FileFrame();
            if (file_frame == nullptr)break;
            auto frame_data = file_frame->data()->data();//this is a (uint8_t*)
            auto frame_data_size = file_frame->size();

            std::string fileStr;
            fileStr.resize(frame_data_size);
            std::memcpy((void *) fileStr.data(), &frame_data[0], fileStr.size());

            RecvBinaryFileData(fileStr);
#else
            auto file_frame = (EasyConnect::FileFrame *) msg_recv->any_as_FileFrame();
            if (file_frame == nullptr)break;
            auto frame_data = file_frame->name()->data();//this is a (uint8_t*)
            auto frame_data_size = file_frame->size();

            std::string fileStr;
            fileStr.resize(frame_data_size);
            std::memcpy((void *) fileStr.data(), &frame_data[0], fileStr.size());

            CopeFileCommand(fileStr);
#endif

            break;
        }
    }
}

void LinkLayer::HandleCommand(const std::string &id, EasyConnect::CommandFrame *cmd) {
    if (cmd->is_ack())return;
    switch (cmd->cmd()) {
        case EasyConnect::CommandType_Stop: {
            LOG(INFO) << "recv stop from " << id;
            CopeStopCommand();
            //send ack
            flatbuffers::FlatBufferBuilder builder;
            auto ack = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Stop, true);
            builder.Finish(ack);
            auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, ack.Union(),
                                                  builder.CreateString(_device_id));
            builder.Finish(msg);
            _trans->send_to(id, builder.GetBufferPointer(), builder.GetSize());
            LOG(INFO) << "send stop ack to " << id;
            break;
        }
        case EasyConnect::CommandType_None:
            break;
        case EasyConnect::CommandType_Start: {
            LOG(INFO) << "recv start from " << id;

            bool res = CopeStartCommand();
            if (!res)return;
            //send ack
            flatbuffers::FlatBufferBuilder builder;
            auto ack = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Start, true);
            builder.Finish(ack);
            auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, ack.Union(),
                                                  builder.CreateString(_device_id));
            builder.Finish(msg);
            _trans->send_to(id, builder.GetBufferPointer(), builder.GetSize());
            LOG(INFO) << "send start ack to " << id;
            break;
        }
        case EasyConnect::CommandType_SetParam: {
            LOG(INFO) << "recv set params from " << id;
            auto params = (EasyConnect::ParameterFrame *) cmd->data();

            CopeSetParamsCommand(params);

            //send ack
            flatbuffers::FlatBufferBuilder builder;
            auto ack = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_SetParam, true);
            builder.Finish(ack);
            auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, ack.Union(),
                                                  builder.CreateString(_device_id));
            builder.Finish(msg);
            _trans->send_to(id, builder.GetBufferPointer(), builder.GetSize());
            LOG(INFO) << "send set param ack to " << id;
            break;
        }
        case EasyConnect::CommandType_GetParam: {
//            LOG(INFO) << "recv get params";
//
//
//            //send ack
//            flatbuffers::FlatBufferBuilder builder;
//            auto ack = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_GetParam, true);
//            builder.Finish(ack);
//            auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, ack.Union(),
//                                                  builder.CreateString(_device_id));
//            builder.Finish(msg);
//            _trans->send_to(id, builder.GetBufferPointer(), builder.GetSize());
//            LOG(INFO) << "send get params ack";
            break;
        }
        case EasyConnect::CommandType_Chat: {
            if (_device_id == "receiver")return;
            LOG(INFO) << "recv chat from " << id;
            if (cmd->data_type() != EasyConnect::CommandData_StringFrame) {
                LOG(ERROR) << "chat data error";
                break;
            }
            auto chat_data = cmd->data_as_StringFrame()->content()->str();
            if (chat_data.empty()) {
                LOG(INFO) << "chat data empty";
                break;
            }

            CopeChatCommand(chat_data);

            //send ack
            flatbuffers::FlatBufferBuilder builder;
            auto ack = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Chat, true);
            builder.Finish(ack);
            auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, ack.Union(),
                                                  builder.CreateString(_device_id));
            builder.Finish(msg);
            _trans->send_to(id, builder.GetBufferPointer(), builder.GetSize());
            LOG(INFO) << "send chat ack to " << id;
            break;
        }
        case EasyConnect::CommandType_NtpSync: {
            LOG(INFO) << "recv ntp sync from " << id;
            if (cmd->data_type() != EasyConnect::CommandData_StringFrame) {
                LOG(ERROR) << "ntp sync data error";
                break;
            }
            auto ntp_ip = cmd->data_as_StringFrame()->content()->str();
//            if (ntp_ip.empty()) {
//                LOG(INFO) << "ntp sync data empty";
//                break;
//            }

            CopeTimeSync(ntp_ip);

            auto dev_time = _dev->GetTimeNow();
            uint64_t tsp = std::round((dev_time.get_full_secs() + dev_time.get_frac_secs()) * 1000);

            //send ack
            flatbuffers::FlatBufferBuilder builder;
            auto tsp_f = EasyConnect::CreateLongFrame(builder, tsp);
            auto ack = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_NtpSync, true,
                                                       EasyConnect::CommandData_LongFrame, tsp_f.Union(), tsp);
            builder.Finish(ack);
            auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, ack.Union(),
                                                  builder.CreateString(_device_id));
            builder.Finish(msg);
            _trans->send_to(id, builder.GetBufferPointer(), builder.GetSize());
            LOG(INFO) << "send ntp sync ack to " << id;
            break;
        }
        case EasyConnect::CommandType_NtpSyncErr: {
            break;
        }
        case EasyConnect::CommandType_Exception: {
            break;
        }
        case EasyConnect::CommandType_Hop: {
           // LOG(INFO) << "recv hop from " << id;
            auto hop_params = (EasyConnect::HopFrame *) cmd->data();

            bool res = CopeHopCommand(hop_params);
            if (!res)
                return;

            //send ack
            flatbuffers::FlatBufferBuilder builder;
            auto ack = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Hop, true);
            builder.Finish(ack);
            auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, ack.Union(),
                                                  builder.CreateString(_device_id));
            builder.Finish(msg);
            _trans->send_to(id, builder.GetBufferPointer(), builder.GetSize());
            //LOG(INFO) << "send hop ack to " << id;
            break;
        }
        case EasyConnect::CommandType_TestTime: {
            LOG(INFO) << "recv test time from " << id;
            if (cmd->data_type() != EasyConnect::CommandData_LongFrame) {
                LOG(ERROR) << "test time data error";
                break;
            }
            auto tm = cmd->data_as_LongFrame()->content();

            CopeTestTime(tm);

            LOG(INFO) << "send test time ack to " << id;
            break;
        }
        case EasyConnect::CommandType_CRC_CHECK:
            break;
        case EasyConnect::CommandType_SNR: {
            break;
        }
        case EasyConnect::CommandType_STATE: {
            break;
        }
        case EasyConnect::CommandType_FULL_LOAD: {
            LOG(INFO) << "recv full load query from " << id;
            _physics->CheckLinkDataSize();
            break;
        }
        case EasyConnect::CommandType_Throughput: {
            LOG(INFO) << "recv throughput";
            if (cmd->data_type() != EasyConnect::CommandData_ThroughputFrame) {
                LOG(ERROR) << "data type error";
                break;
            }
            break;
        }
            /*///发送文件
        case : {
            CopeFileCommand();
            break;
        }
            ///取消发送
        case : {
            CopeCancelSendCommand();
            break;
        }*/
    }
}

void LinkLayer::CopeStopCommand() {
    _physics->StopWork();
    StopWork();
}

void LinkLayer::StopWork() {
    _start_flag = false;
    _work_ready = false;
    _cv.notify_all();

    std::unique_lock<std::mutex> sfs_ul(_sfs_mtx);
    _send_file_size = 0;
    sfs_ul.unlock();
}

bool LinkLayer::CopeStartCommand() {
    try {
        _physics->StartWork();
    } catch (uhd::exception &e) {
        LOG(INFO) << e.what();
        SendException((boost::format(e.what())).str());
        return false;
    } catch (...) {
        std::string str("catch an unknown exception");
        LOG(INFO) << str;
        SendException(str);
        return false;
    }

    StartWork();

    return true;
}

void LinkLayer::StartWork() {
    if (_start_flag) {
        return;
    }

    std::unique_lock<std::mutex> sf_ul(_sf_mtx);
    _str_fifo.clear();
    sf_ul.unlock();

    std::unique_lock<std::mutex> ff_ul(_ff_mtx);
    _file_fifo.clear();
    ff_ul.unlock();

    _send_str_ordinal = 1;
    _send_file_ordinal = 1;
    _retrans_str_ordinal = 1;
    _retrans_file_ordinal = 1;

    _recv_file.clear();
    _file_end_ordinal = 0;

    _start_flag = true;
    _work_ready = true;

    std::unique_lock<std::mutex> ck_ul(_ff_mtx);
    _ackCount = 0;
    _nckCount = 0;
    _ackCount_GUI = 0;
    _nckCount_GUI = 0;
    ck_ul.unlock();

    _cv.notify_all();
}

void LinkLayer::RetransmissionThread() {
    while (_loop) {

        std::unique_lock<std::mutex> ul(_retrans_thread_mtx);
        _cv.wait(ul, [this]() {
            return _work_ready;
        });

        if (!_start_flag) {
            continue;
        }

        if (_link_params.retransmission_gap != 0)
            std::this_thread::sleep_for(
                    std::chrono::milliseconds(_link_params.retransmission_gap));//_retransmission_gap ms查询一次

        std::list<LinkFrame> retrans_fifo;
        std::unique_lock<std::mutex> sf_ul(_sf_mtx);
        if (!_str_fifo.empty()) {

            for (auto &cff: _str_fifo) {

                if (cff.GetOrdinal() <= _retrans_str_ordinal) {
                    retrans_fifo.emplace_back(cff);
                    cff.UpdataCount();//更新重传次数
                    if (cff.GetCount() >= _link_params.max_retransmission_times) {//如果重传次数达到预定值 需要通知上层
                        //do something
                        if (_link_params.max_retransmission_times != 0 &&
                            cff.GetCount() % _link_params.max_retransmission_times == 0) {
                            SendException((boost::format("(str)序号[%d]已重传: %d次") % cff.GetOrdinal() %
                                           cff.GetCount()).str());
                        }
                    }
                }
            }

            if (!retrans_fifo.empty()) {
                _cb(retrans_fifo);
            }

            _retrans_str_ordinal = (int) (_str_fifo.begin()->GetOrdinal() + retrans_fifo.size());
        }
        sf_ul.unlock();

        if (_link_params.retransmission_gap != 0 && !retrans_fifo.empty())
            std::this_thread::sleep_for(
                    std::chrono::milliseconds(_link_params.retransmission_gap));//_retransmission_gap ms查询一次

        retrans_fifo.clear();
        std::unique_lock<std::mutex> ff_ul(_ff_mtx);
        if (!_file_fifo.empty()) {

            for (auto &fff: _file_fifo) {

                if (fff.GetOrdinal() <= _retrans_file_ordinal) {
                    retrans_fifo.emplace_back(fff);
                    fff.UpdataCount();//更新重传次数
                    if (fff.GetCount() >= _link_params.max_retransmission_times) {//如果重传次数达到预定值 需要通知上层
                        //do something
                        if (_link_params.max_retransmission_times != 0 &&
                            fff.GetCount() % _link_params.max_retransmission_times == 0) {
                            SendException((boost::format("序号[%d]已重传: %d次") % fff.GetOrdinal() % fff.GetCount()).str());
                        }
                    }
                }
            }

            if (!retrans_fifo.empty()) {
                _cb(retrans_fifo);
            }

            _retrans_file_ordinal = int(_file_fifo.begin()->GetOrdinal() + retrans_fifo.size());
        }
        ff_ul.unlock();

    }

    LOG(INFO) << "retransmission thread exit...";
}

void LinkLayer::RetransWork() {
    PhyFrame::ST _prePst;
    while (_loop) {

        std::unique_lock<std::mutex> ul(_retrans_thread_mtx);
        _cv.wait(ul, [this]() {
            return _work_ready;
        });

        if (!_start_flag) {
            continue;
        }

        //读取当前已发送的内容
        if (_physics->GetEmptyFlag()) {
            std::unique_lock<std::mutex> sf_ul(_sf_mtx);
            if (!_str_fifo.empty()) {
                for (auto &tmp:_str_fifo) {
                    RetransWarning(tmp);
                }
                _cb(_str_fifo);
            }
            sf_ul.unlock();

            std::unique_lock<std::mutex> ff_ul(_ff_mtx);
            if (!_file_fifo.empty()) {
                for (auto &tmp:_file_fifo) {
                    RetransWarning(tmp);
                }
                _cb(_file_fifo);
            }
            ff_ul.unlock();

            _physics->ResetEmptyFlag();
            continue;
        }


        auto pst = _physics->GetPhyFrameST();

        if (pst.isAck) continue;

        if (_prePst == pst) continue;///fix

        long pos = 0;
        long retransNum = 0;
        std::list<LinkFrame> retransBuf;
        bool findTarget = false;

        switch (pst.type) {
            case PhyFrame::Type::Str: {
                long windowsLen = 2;
                std::unique_lock<std::mutex> sf_ul(_sf_mtx);
                if (!_str_fifo.empty()) {

                    long totalSize = _str_fifo.size();
                    if (totalSize > 1) {
                        while (windowsLen >= totalSize) {
                            windowsLen /= 2;
                        }
                    } else {
                        windowsLen = 1;
                    }

                    for (auto &sff: _str_fifo) {
                        pos++;
                        if (sff.GetOrdinal() == pst.ordinal) {
                            findTarget = true;
                            retransNum = (pos - windowsLen);
                            break;
                        }
                    }
                }

                if (!findTarget) break;
                if (retransNum > 0) {
                    for (int j = 0; j < retransNum; ++j) {
                        RetransWarning(_str_fifo.front());

                        retransBuf.emplace_back(_str_fifo.front());
                        _str_fifo.emplace_back(_str_fifo.front());
                        _str_fifo.pop_front();
                    }
                }

                if (!retransBuf.empty()) {
                    _cb(retransBuf);
                }

                break;
            }
            case PhyFrame::Type::File: {
                long windowsLen = 50;
                std::unique_lock<std::mutex> ff_ul(_ff_mtx);
                if (!_file_fifo.empty()) {

                    long totalSize = _file_fifo.size();
                    if (totalSize > 1) {
                        while (windowsLen >= totalSize) {
                            windowsLen /= 2;
                        }
                    } else {
                        windowsLen = 1;
                    }

                    for (auto &sff: _file_fifo) {
                        pos++;
                        if (sff.GetOrdinal() == pst.ordinal) {
                            findTarget = true;
                            retransNum = (pos - windowsLen);
                            break;
                        }
                    }

                }

                if (!findTarget) break;
                if (retransNum > 0) {
                    for (int j = 0; j < retransNum; ++j) {
                        RetransWarning(_file_fifo.front());

                        retransBuf.emplace_back(_file_fifo.front());
                        _file_fifo.emplace_back(_file_fifo.front());
                        _file_fifo.pop_front();
                    }
                }

                if (!retransBuf.empty()) {
                    _cb(retransBuf);
                }
                break;
            }
            default:
                break;
        }
        _prePst = pst;
    }

    LOG(INFO) << "retransmission thread exit...";
}

void LinkLayer::RetransWarning(LinkFrame &obj) {
    obj.UpdataCount();
    if (obj.GetCount() >= _link_params.max_retransmission_times) {//如果重传次数达到预定值 需要通知上层
        //do something
        if (_link_params.max_retransmission_times != 0 &&
            obj.GetCount() % _link_params.max_retransmission_times == 0) {
            SendException((boost::format("序号[%d]已重传: %d次") % obj.GetOrdinal() % obj.GetCount()).str());
        }
    }
}

void LinkLayer::ProgramStateThread() {
    while (_loop) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        SendHeart();
        SendFileProgressBar();
        SendThroughput_GUI(); //1s 发送一次ACK给GUI
    }
    LOG(INFO) << "ProgramStateThread() exit...";
}

void LinkLayer::ThoughThread() { //100ms 发送一次ACK给决策子系统和干扰子系统
    while (_loop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        SendThroughput();
    }
    LOG(INFO) << "PThoughThread() exit...";
}

void LinkLayer::SendHeart() {
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

void LinkLayer::SendFileProgressBar() {

    std::unique_lock<std::mutex> ff_ul(_ff_mtx);
    size_t file_fifo_size = _file_fifo.size();
    ff_ul.unlock();

    std::unique_lock<std::mutex> sfs_ul(_sfs_mtx);
    if (_send_file_size == 0) {
        sfs_ul.unlock();
        return;
    }

    double cal_ret = 1 - (file_fifo_size / (double) _send_file_size);

    //含文件的内容发完了
    if (file_fifo_size == 0) {
        _send_file_size = 0;//清零
    }
    sfs_ul.unlock();

    if (_device_id == "receiver")return;
    flatbuffers::FlatBufferBuilder builder;
    auto full_f = EasyConnect::CreateFullLoadFrame(builder, (cal_ret >= 1.0), cal_ret);
    builder.Finish(full_f);
    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_FULL_LOAD, true,
                                               EasyConnect::CommandData_DoubleFrame, full_f.Union());
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(_device_id));
    builder.Finish(msg);
    _trans->send_to(_Receiver.gui, builder.GetBufferPointer(), builder.GetSize());
    LOG(INFO) << "Send Progress Bar: " << cal_ret * 100 << "% to " << _Receiver.gui;
}

void LinkLayer::CopeSetParamsCommand(EasyConnect::ParameterFrame *params) {
    CopeStopCommand();

    _MTU = params->mtu();
    _physics_params.tx_freq = params->tx_freq();
    _physics_params.tx_gain = params->tx_gain();
    _physics_params.rx_freq = params->rx_freq();
    _physics_params.rx_gain = params->rx_gain();
    _physics_params.sampling_rate = params->sampling_rate();

    ResetSetParam();

    _physics->SetSyn(_physics_params.syn, _physics_params.reverse_syn, _physics_params.multiple);

    _physics->CopeSetParam(_physics_params.tx_freq, _physics_params.rx_freq, _physics_params.tx_lo_off,
                           _physics_params.rx_lo_off, _physics_params.sampling_rate, _physics_params.tx_gain,
                           _physics_params.rx_gain, _MTU, _physics_params.delay, _physics_params.TSC,
                           _physics_params.insert_tsc_per_bytes, _physics_params.tx_fifo_size);

    //开始
    CopeStartCommand();

    DisplayParameters();
}

void LinkLayer::CopeChatCommand(std::string &msg) {
    if (!_start_flag) {
        if (!CopeStartCommand())
            return;
    }
    RecvChatFromUpperLevels(msg);
}

void LinkLayer::RecvChatFromUpperLevels(std::string &msg) {
    if (msg.empty()) {
        return;
    }

    ///计算拆分的包数
    size_t packages_num = msg.size() / _MTU;//包数
    size_t r_size = msg.size() % _MTU;
    size_t append_space = 0;
    if (r_size != 0) {
        packages_num++;

        ///添加的' '字节数 目的是使发送的数据=MTU
        append_space = _MTU - r_size;
        msg.append(append_space, ' ');
    }
    if (packages_num == 0)return;

    std::list<LinkFrame> current_send_fifo;//存储当前需要发送的数据

    //发送的字节小于MTU 不用分组
    if (packages_num == 1) {

        LinkFrame frame_1(_send_str_ordinal++, append_space, MSG_FRAME, TRANS_CHAT, true, msg, 0);
        frame_1.UpdataCount();//发送次数+1

        current_send_fifo.emplace_back(frame_1);

        std::unique_lock<std::mutex> sf_ul(_sf_mtx);
        _str_fifo.emplace_back(frame_1);//存储发送的数据,以便在收到ack时进行删除 未收到ack时进行重发
        sf_ul.unlock();

    } else {//发送的字节大于MTU, 拆包处理

        std::string temp_str;

        std::unique_lock<std::mutex> sf_ul(_sf_mtx);
        for (size_t i = 0; i < packages_num; ++i) {
            temp_str.resize(_MTU);
            memcpy(&temp_str.at(0), &msg.at(i * _MTU), sizeof(char) * _MTU);
            LinkFrame frame_2(_send_str_ordinal++, (i == (packages_num - 1)) ? (int) append_space : 0, MSG_FRAME,
                              TRANS_CHAT,
                              (i == (packages_num - 1)), temp_str, 0);
            frame_2.UpdataCount();
            current_send_fifo.emplace_back(frame_2);
            _str_fifo.emplace_back(frame_2);
        }
        sf_ul.unlock();

    }

    //序号占2字节(编码后16bit)
    if (_send_str_ordinal == (1 << 16) - 1) {
        LOG(INFO) << "序号达到最大,下一包将从1开始";
        _send_str_ordinal = 1;
    }

    if (!current_send_fifo.empty()) {
        _cb(current_send_fifo);
    }

}

bool LinkLayer::CopeTimeSync(std::string &ip) {
#if 1
    try {
        LOG(INFO) << "sync time to GPS";
        if (_dev->GetClockSource() != "gpsdo" && _dev->GetTimeSource() != "gpsdo") {
            return false;
        }

        _dev->SetTimeNow(0.0);
        auto now = _iface->get_gps_time();
        LOG(INFO) << "gps now: " << now.get_full_secs() << " " << now.get_frac_secs();

        _dev->SetTimeNextPPS(now.get_full_secs() + 1);

#if 1
        for (int l = 0; l < 10; ++l) {

            std::this_thread::sleep_for(std::chrono::milliseconds(180));
            auto fpga_time = _dev->GetTimeNow();
            LOG(INFO) << "device time: " << fpga_time.get_full_secs() << " " << fpga_time.get_frac_secs();

        }
#endif

        auto locate = _iface->get_locate_data();
        LOG(INFO) << "num: " << locate.num;
        LOG(INFO) << "gps status: " << locate.status;
        LOG(INFO) << "location: " << locate.longitude << "," << locate.latitude << "," << locate.altitude;
    } catch (std::exception &e) {
        LOG(ERROR) << e.what();
    }

#else
    if (ip != _ntp_server_ip) {
        _ntp_server_ip = ip;
        _ntp_client.reset(new NTPclient(_ntp_server_ip));
    }
    bool res = NTPTimeSync();
    if (!res) {
        return false;
    } else {
        if (!_use_pps) {
            _dev->SetTimeNow(_physics->GetSystemTimeUsec());//配置设备时间
        } else {
            _dev->SetTimeNextPPS(_physics->GetSystemTimeUsec().get_full_secs() + 1.0);
        }

        _ntp_client->GetNowSystemTime_context();
        for (int j = 0; j < 5; ++j) {
            std::this_thread::sleep_for(std::chrono::milliseconds(150));
            LOG(INFO) << (boost::format("System time: %lf, dev time: %lf") %
                          _physics->GetSystemTimeUsec().get_real_secs() %
                          _dev->GetTimeNow().get_real_secs()).str();
        }

        return true;
    }
#endif

    return true;
}

bool LinkLayer::CopeHopCommand(EasyConnect::HopFrame *hopFrame) {
    if (!_start_flag) {
        std::string str("Please start the device first");
        LOG(INFO) << str;
        SendException(str);
        return false;
    }

    auto dt = hopFrame->hop_time() / 1000.0 - _dev->GetTimeNow().get_real_secs();
    if (dt <= 0) {
        std::string str = (boost::format("time out:%lf s") % dt).str();
        LOG(ERROR) << str;
        SendException(str);
        return false;
    } else if (dt > 5 * 60) {
        std::string str("time too long");
        LOG(ERROR) << str;
        SendException(str);
        return false;
    }

    if (_device_id == "transmitter")
        _physics->SetHopFreq(hopFrame->hop_time(), hopFrame->hop_freq(), PhysicsLayer::HopType::TX);
    else if (_device_id == "receiver")
        _physics->SetHopFreq(hopFrame->hop_time(), hopFrame->hop_freq(), PhysicsLayer::HopType::RX);
    else if (_device_id == "transceiver")
        _physics->SetHopFreq(hopFrame->hop_time(), hopFrame->hop_freq(), PhysicsLayer::HopType::ALL);

    return true;
}

void LinkLayer::CopeTestTime(const uint64_t &tm) {
    uhd::time_spec_t timeSpec = tm / 1000.0;
    double dt = timeSpec.get_real_secs() - _dev->GetTimeNow().get_real_secs();
    if (dt <= 0.002) {
        std::string str("time out");
        LOG(ERROR) << str;
        SendException(str);
        return;
    } else if (dt > 5 * 60) {
        std::string str("time too long");
        LOG(ERROR) << str;
        SendException(str);
        return;
    }

    CopeStopCommand();
    if (_device_id == "transmitter")
        _physics->SendPNSeq(&timeSpec);
    else if (_device_id == "receiver")
        _physics->DetectPNSeq(&timeSpec);
    else if (_device_id == "transceiver") {
        _physics->SendPNSeq(&timeSpec);
        _physics->DetectPNSeq(&timeSpec);
    }
}


void LinkLayer::RecvFromPhysicsLayer(double timestamp, const BitVector &input, unsigned crc_check_result, double snr) {
    SendCRCResult(crc_check_result);
    std::unique_lock<std::mutex> ul(_mtxCk);
    if (crc_check_result != 0) {
        _nckCount++;
        _nckCount_GUI++;
        if (crc_check_result != 1)
            LOG(INFO) << "有误码, crc检验结果 = " << crc_check_result;
        return;
    }
    _ackCount++;
    _ackCount_GUI++;
    ul.unlock();

//    LOG(INFO) << (boost::format("%lf") % timestamp).str();
//    LOG(INFO) << "snr: " << snr;

    //提取序号
    int recv_ordinal = -1;
    recv_ordinal = ParseOrdinal(input.head(16));
    if (recv_ordinal < 0) {
        LOG(INFO) << "recv ordinal error!!!";
        return;
    }
    if (recv_ordinal == 0) {
        //test ...
        return;
    }

    //类型
    FrameType frame_type = MSG_FRAME;
    if (input.bit(16)) {
        frame_type = ACK_FRAME;
    }

    //提取传输模式
    TransType recv_trans_type = TRANS_CHAT;
    int ret = ParseTransType(input, recv_trans_type);
    if (ret != 0) {
        return;
    }

    switch (frame_type) {
        case ACK_FRAME: {//收到ack帧就把fifo对应的数据删除
            HandleAckFrame(recv_trans_type, recv_ordinal);
            break;
        }
        case MSG_FRAME: {
            HandleMsgFrame(recv_trans_type, input, recv_ordinal);
            break;
        }
    }
}

void LinkLayer::SendCRCResult(double crc_result) {

    flatbuffers::FlatBufferBuilder builder;

    auto crc_f = EasyConnect::CreateDoubleFrame(builder, crc_result);
    builder.Finish(crc_f);

    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_CRC_CHECK, false,
                                               EasyConnect::CommandData_DoubleFrame, crc_f.Union());
    builder.Finish(cmd);

    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(_device_id));
    builder.Finish(msg);

    _trans->send_to(_Receiver.preception, builder.GetCurrentBufferPointer(), builder.GetSize());
    _trans->send_to(_Receiver.gui, builder.GetCurrentBufferPointer(), builder.GetSize());
}

int LinkLayer::ParseOrdinal(const BitVector &input) {
    std::bitset<16> odbs;
    for (int i = 0; i < 16; ++i) {
        odbs[15 - i] = input.bit(i);
    }
    return (int) odbs.to_ulong();
}

int LinkLayer::ParseTransType(const BitVector &input, TransType &recv_trans_type) {
    BitVector trans_type = input.segment(32, 2);
    std::string check_type = "00";
    check_type.at(0) = trans_type.bit(0) ? '1' : '0';
    check_type.at(1) = trans_type.bit(1) ? '1' : '0';

    if (check_type == FILESTR) {
        recv_trans_type = TRANS_FILE;
    } else if (check_type == CHATSTR) {
        recv_trans_type = TRANS_CHAT;
    } else if (check_type == BINARYFILESTR) {
        recv_trans_type = TRANS_BINARYFILE;
    } else {
        return -1;
    }

    return 0;
}

void LinkLayer::HandleAckFrame(TransType recv_trans_type, int recv_ordinal) {
    switch (recv_trans_type) {
        case TRANS_BINARYFILE:
        case TRANS_CHAT: {
            HandleChatAckFrame(recv_ordinal);
            break;
        }
        case TRANS_FILE: {
            HandleFileAckFrame(recv_ordinal);
            break;
        }
    }
}

void LinkLayer::HandleChatAckFrame(int recv_ordinal) {
    bool find_ordinal = false;
    std::unique_lock<std::mutex> sf_ul(_sf_mtx);
    if (!_str_fifo.empty()) {//此时保证fifo不为空
        auto sf_it = _str_fifo.begin();
        while (sf_it != _str_fifo.end()) {
            if (sf_it->GetOrdinal() == recv_ordinal) {
                find_ordinal = true;
                LOG(INFO) << "Found str[" << recv_ordinal << "], delete...";
                _str_fifo.erase(sf_it);//删除

                break;
            }
            sf_it++;
        }
    }
    LOG(INFO) << "Remaining str num: " << _str_fifo.size();
    sf_ul.unlock();

    if (!find_ordinal) {
        LOG(INFO) << "Not found str[" << recv_ordinal << "], maybe already recv and delete...";
    }

}

void LinkLayer::HandleFileAckFrame(int recv_ordinal) {
    bool find_ordinal = false;
    std::unique_lock<std::mutex> ff_ul(_ff_mtx);
    if (!_file_fifo.empty()) {//此时保证fifo不为空
        auto sf_it = _file_fifo.begin();
        while (sf_it != _file_fifo.end()) {
            if (sf_it->GetOrdinal() == recv_ordinal) {
                find_ordinal = true;
                LOG(INFO) << "Found file[" << recv_ordinal << "], delete...";
                _file_fifo.erase(sf_it);//删除

                break;
            }
            sf_it++;
        }
    }
    LOG(INFO) << "Remaining file num: " << _file_fifo.size();
    ff_ul.unlock();

    if (!find_ordinal) {
        LOG(INFO) << "Not found file[" << recv_ordinal << "], maybe already recv and delete...";
    }

}

void LinkLayer::HandleMsgFrame(TransType recv_trans_type, const BitVector &input, int recv_ordinal) {
    //提取空格数
    int recv_space_num = 0;
    recv_space_num = ParseSpace(input.segment(17, 15));

    //提取内容 并把多余的空格去掉
    std::string recv_data;
    ParseData(input.segment(40, (_MTU - recv_space_num) * 8), recv_data);

    switch (recv_trans_type) {
        case TRANS_CHAT: {
            LOG(INFO) << "recv chat[" << recv_ordinal << "]: " << recv_data;
            SendACK(TRANS_CHAT, recv_ordinal);
            SendChat(recv_data);
            break;
        }
        case TRANS_FILE: {
            //do something
            ParseFileContext(input, recv_data, recv_ordinal);
            break;
        }
        case TRANS_BINARYFILE: {
            LOG(INFO) << "recv binary file[" << recv_ordinal << "]";
            SendACK(TRANS_BINARYFILE, recv_ordinal);
            SendBinaryFile(recv_data);
            break;
        }
    }
}

int LinkLayer::ParseSpace(const BitVector &input) {
    std::bitset<15> odbs;
    for (int i = 0; i < 15; ++i) {
        odbs[14 - i] = input.bit(i);
    }
    return (int) odbs.to_ulong();;
}

void LinkLayer::ParseData(const BitVector &input, std::string &output) {
    output.resize(input.size() / 8);
    input.pack((unsigned char *) output.c_str());
}

void LinkLayer::SendACK(TransType recv_trans_type, int recv_ordinal) {
    //回复ack
    std::string ack_str = " ";
    ack_str.append(_MTU - ack_str.size(), ' ');
    std::list<LinkFrame> ack(1,
                             LinkFrame(recv_ordinal, _MTU - ack_str.size(), ACK_FRAME,
                                       recv_trans_type, true, ack_str, 0));

    switch (recv_trans_type) {
        case TRANS_FILE: {
            LOG(INFO) << "send file[" << recv_ordinal << "] ack";
            break;
        }
        case TRANS_BINARYFILE:
        case TRANS_CHAT: {
            LOG(INFO) << "send str[" << recv_ordinal << "] ack";
            break;
        }
    }

    _cb(ack);
}

void LinkLayer::SendChat(const std::string &chat) {
    flatbuffers::FlatBufferBuilder builder;

    auto chat_f = EasyConnect::CreateStringFrame(builder, builder.CreateString(chat));
    builder.Finish(chat_f);

    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Chat, false,
                                               EasyConnect::CommandData_StringFrame, chat_f.Union());
    builder.Finish(cmd);

    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(_device_id));
    builder.Finish(msg);

    _trans->send_to(_Receiver.gui, builder.GetCurrentBufferPointer(), builder.GetSize());
}

void LinkLayer::ParseFileContext(const BitVector &input, const std::string &recv_data, int recv_ordinal) {

    ///检测结尾 "0"代表结尾
    bool recv_end = !input.bit(34);
    if (recv_end) {
        //判断此时接收recv_ordinal 和 之前确定的_file_end_ordinal
        /*if (recv_ordinal >= _file_end_ordinal) {
            //有可能是对方原先发了大小为n的文件1,但是中断了,
            //然后现在发了大小为m(m>=n)的文件2过来
            //后面有处理 这里就不重复处理了
        } else */if (recv_ordinal < _file_end_ordinal) {
            //有可能是对方原先发了大小为n的文件1,但是中断了,
            //然后现在发了大小为m(m<n)的文件2过来
            //现在要做的是把已接收大于m的序号的内容清除
            if (!_recv_file.empty() && ((int) _recv_file.size() > recv_ordinal)) {

                RemoveExtraElements(_recv_file, recv_ordinal);

                //再次判断一下 这里不应该有问题
                if ((int) _recv_file.size() > recv_ordinal) {
                    LOG(INFO) << "recv file size exception!!!";
                }

            }

        }

        _file_end_ordinal = recv_ordinal;//确定_recv_file大小
    }

    SendACK(TRANS_FILE, recv_ordinal);

    //查询有没有相同序号
    bool same_ordinal = false;
    if (!_recv_file.empty()) {
        for (auto &item : _recv_file) {
            if (item._ordinal == recv_ordinal) {

                if (item._data == recv_data) {
                    //有相同的就不重复接收了
                    return;
                } else {
                    //新的取代旧的
                    item._data.clear();
                    item._data = recv_data;
                    if (!recv_end && _file_end_ordinal != 0)
                        _file_end_ordinal = 0;//可能会有问题
                }

                same_ordinal = true;

                //节省时间,一次只有一帧数据进来, 检测到了就直接break
                break;
            }
        }
    }

    //没有相同序号就接收
    if (!same_ordinal) {
        _recv_file.emplace_back(recv_ordinal, recv_data);
    }

    if (_file_end_ordinal != 0 && (int) _recv_file.size() >= _file_end_ordinal) {

        if ((int) _recv_file.size() > _file_end_ordinal) {

            //查查是不是有问题
            RemoveExtraElements(_recv_file, _file_end_ordinal);

            //再次判断一下 这里不应该有问题
            if ((int) _recv_file.size() > recv_ordinal) {
                _recv_file.clear();
                _file_end_ordinal = 0;
                LOG(INFO) << "recv file size exception!!!";
                return;
            }

        }

        if ((int) _recv_file.size() == _file_end_ordinal) {
            //because 规定了每个文件的序号从1开始排序

            if (_recv_file.size() != 1) {//不止一帧就要排序了
                std::sort(_recv_file.begin(), _recv_file.end());
            }

            //提取文件名长度
            std::bitset<5> name_len;
            for (int i = 0; i < 5; ++i) {
                name_len[4 - i] = input.bit(35 + i);
            }
            int filename_len = (int) name_len.to_ulong();

            std::string file_name;
            int ret = SaveRecvFile(filename_len, &file_name);
            if (ret == 0)
//                SendState("file_recv_over", file_name);
                //TODO
                ;
            else
                return;
        }
    }
}

void LinkLayer::RemoveExtraElements(std::vector<FileFrame> &vec, int threshold) {
    //严谨起见,先排个序
    if (vec.size() != 1) {//不止一帧就要排序了
        std::sort(vec.begin(), vec.end());
    }
    auto rf_it = vec.begin();
    while (rf_it < vec.end()) {
        if (rf_it->_ordinal > threshold) {
            vec.erase(rf_it);//rf_if已经指向被删元素的下一个
        } else {
            rf_it++;
        }
    }
}

int LinkLayer::SaveRecvFile(int filename_len, std::string *file_name) {
    std::string all_data;
    for (auto &recvf : _recv_file) {
        all_data.append(recvf._data);
    }

    std::string filename;
    filename.insert(filename.end(), all_data.begin(), all_data.begin() + filename_len);
    *file_name = filename;
    LOG(INFO) << "recv file: " << filename;

    std::string file_data;
    file_data.insert(file_data.end(), all_data.begin() + filename_len, all_data.end());

#if 0
    std::time(&_nowtime);
    std::strftime(_deal_time, sizeof(_deal_time), "%Y-%m-%d %H:%M:%S", std::localtime(&_nowtime));
    std::vector<std::string> vec;
    boost::split(vec, _deal_time, boost::is_any_of(" "));
    boost::filesystem::path m_path = vec.at(0);
    if (!boost::filesystem::exists(m_path)) {
        boost::filesystem::create_directories(m_path);
    }
    std::string fn((boost::format("%s/%s/%s") % _recv_file_dir % vec.at(0) % filename).str());
    std::ofstream sf(fn, std::ios::binary);
    if (sf.fail()) {
        std::string str = (boost::format("fail to open %s") % fn).str();
        LOG(INFO) << str;
        SendException(str);
        return -1;
    }
    sf.write((char *) file_data.data(), sizeof(char) * file_data.size());
    sf.close();
#else
    std::string fn((boost::format("%s/%s") % _recv_file_dir % filename).str());
    std::ofstream sf(fn, std::ios::binary);
    if (sf.fail()) {
        std::string str = (boost::format("fail to open %s") % fn).str();
        LOG(INFO) << str;
        SendException(str);
        return -1;
    }
    sf.write((char *) file_data.data(), sizeof(char) * file_data.size());
    sf.close();
#endif

    _recv_file.clear();
    _file_end_ordinal = 0;

    return 0;
}

void LinkLayer::SendException(const std::string &str) {
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

void LinkLayer::CopeFileCommand(const std::string &filename) {
    if (filename.empty()) {
        return;
    }

    if (!_start_flag) {
        if (!CopeStartCommand())
            return;
    }
    //目前一次只允许发一个文件, 即_fifo里面最多只允许含一个文件的数据
    std::unique_lock<std::mutex> ff_ul(_ff_mtx);
    if (!_file_fifo.empty()) {
        std::string str = "There's still file data to send, please wait a moment or restart";
        SendException(str);
        ff_ul.unlock();
        return;
    }
    ff_ul.unlock();

    std::string file_addr = (boost::format("%s/%s") % _send_file_dir % filename).str();
    LOG(INFO) << "file_addr: " << file_addr;
    std::string file_data;
    int ret = ReadFile(file_addr, file_data);
    if (ret != 0) {
        return;
    }

    auto tf = filename;
    MergeNameContent(tf, file_data);
    RecvFileFromUpperLevels(file_data, tf.size());
}

int LinkLayer::ReadFile(const std::string &file_name, std::string &output) {
    std::ifstream fread(file_name, std::ios::binary/*二进制*/ | std::ios::ate/*指向文件末尾*/);
    if (fread.fail()) {
        std::string str = (boost::format("Failed to open the %s") % file_name).str();
        LOG(INFO) << str;
        SendException(str);
        return -1;
    }

    int file_size = fread.tellg();//获取文件大小（字节）
    if (file_size >= 10 * 1024 * 1024) {
        std::string str = (boost::format("文件不能大于10M, 该文件大小: %d") % file_size).str();
        LOG(INFO) << str;
        SendException(str);
        output.clear();
        fread.close();
        return -1;
    }

    fread.seekg(0, std::ios::beg);//将fread指向文件开头

    output.resize(file_size);

    fread.read((char *) output.data(), sizeof(char) * file_size);//读取文件内容进入file_data
    fread.close();//关闭文件

    return 0;
}

void LinkLayer::MergeNameContent(std::string &name, std::string &content) {

    //最大只允许31个字节
    size_t max_len = (1 << 5) - 1;
    std::string tmp;
    if (name.size() > max_len) {
        tmp.insert(tmp.end(), name.begin() + (name.size() - max_len), name.end());
        name = tmp;
    }

    content.insert(content.begin(), name.begin(), name.end());
}

void LinkLayer::RecvFileFromUpperLevels(std::string msg, int filename_len) {
    if (msg.empty()) {
        return;
    }

    ///计算拆分的包数
    size_t packages_num = msg.size() / _MTU;//包数
    size_t r_size = msg.size() % _MTU;
    size_t append_space = 0;
    if (r_size != 0) {
        packages_num++;

        if (packages_num > (1 << 16) - 1) {
            std::string str("The file is too large, please set MTU to large");
            LOG(WARNING) << str;
            SendException("The file is too large, please set MTU to large");
            return;
        }

        //添加的' '字节数 目的是使发送的数据=MTU
        append_space = _MTU - r_size;
        msg.append(append_space, ' ');
    }
    if (packages_num == 0)return;

    _send_file_ordinal = 1;

    std::list<LinkFrame> current_send_fifo;//存储当前需要发送的数据

    //发送的字节小于MTU 不用分组
    if (packages_num == 1) {

        LinkFrame frame_1(_send_file_ordinal++, append_space, MSG_FRAME, TRANS_FILE, true, msg, filename_len);
        frame_1.UpdataCount();//发送次数+1

        current_send_fifo.emplace_back(frame_1);

        std::unique_lock<std::mutex> ff_ul(_ff_mtx);
        _file_fifo.emplace_back(frame_1);//存储发送的数据,以便在收到ack时进行删除 未收到ack时进行重发
        ff_ul.unlock();

    } else {//发送的字节大于MTU, 拆包处理
        std::string temp_str;

        std::unique_lock<std::mutex> ff_ul(_ff_mtx);
        for (size_t i = 0; i < packages_num; ++i) {
            temp_str.resize(_MTU);
            memcpy(&temp_str.at(0), &msg.at(i * _MTU), sizeof(char) * _MTU);
            LinkFrame frame_2(_send_file_ordinal++, (i == (packages_num - 1)) ? (int) append_space : 0, MSG_FRAME,
                              TRANS_FILE,
                              (i == (packages_num - 1)), temp_str, filename_len);
            frame_2.UpdataCount();
            current_send_fifo.emplace_back(frame_2);
            _file_fifo.emplace_back(frame_2);
        }
        ff_ul.unlock();

    }

    std::unique_lock<std::mutex> sfs_ul(_sfs_mtx);
    _send_file_size = _send_file_ordinal - 1;
    LOG(INFO) << "send file sum size: " << _send_file_size;
    sfs_ul.unlock();

    if (!current_send_fifo.empty()) {
        _cb(current_send_fifo);
    }

}

void LinkLayer::CopeCancelSendCommand() {
    std::unique_lock<std::mutex> sf_ul(_sf_mtx);
    _str_fifo.clear();
    sf_ul.unlock();

    std::unique_lock<std::mutex> sfs_ul(_sfs_mtx);
    _send_file_size = 0;
    sfs_ul.unlock();

    std::unique_lock<std::mutex> ff_ul(_ff_mtx);
    _file_fifo.clear();
    ff_ul.unlock();

    _retrans_str_ordinal = 1;
    _retrans_file_ordinal = 1;

    _physics->ClearPhyscisData();
}

void LinkLayer::SendMsg(const std::string &id, void *data, size_t size) {
    _trans->send_to(id, data, size);
}

void LinkLayer::RecvBinaryFileData(std::string &msg) {
    if (msg.empty()) {
        return;
    }

    ///计算拆分的包数
    size_t packages_num = msg.size() / _MTU;//包数
    size_t r_size = msg.size() % _MTU;
    size_t append_space = 0;
    if (r_size != 0) {
        packages_num++;

        ///添加的' '字节数 目的是使发送的数据=MTU
        append_space = _MTU - r_size;
        msg.append(append_space, ' ');
    }
    if (packages_num == 0)return;

    std::list<LinkFrame> current_send_fifo;//存储当前需要发送的数据

    //发送的字节小于MTU 不用分组
    if (packages_num == 1) {

        LinkFrame frame_1(_send_str_ordinal++, append_space, MSG_FRAME, TRANS_BINARYFILE, true, msg, 0);
        frame_1.UpdataCount();//发送次数+1

        current_send_fifo.emplace_back(frame_1);

        std::unique_lock<std::mutex> sf_ul(_sf_mtx);
        _str_fifo.emplace_back(frame_1);//存储发送的数据,以便在收到ack时进行删除 未收到ack时进行重发
        sf_ul.unlock();

    } else {//发送的字节大于MTU, 拆包处理

        std::string temp_str;

        std::unique_lock<std::mutex> sf_ul(_sf_mtx);
        for (size_t i = 0; i < packages_num; ++i) {
            temp_str.resize(_MTU);
            memcpy(&temp_str.at(0), &msg.at(i * _MTU), sizeof(char) * _MTU);
            LinkFrame frame_2(_send_str_ordinal++, (i == (packages_num - 1)) ? (int) append_space : 0, MSG_FRAME,
                              TRANS_BINARYFILE,
                              (i == (packages_num - 1)), temp_str, 0);
            frame_2.UpdataCount();
            current_send_fifo.emplace_back(frame_2);
            _str_fifo.emplace_back(frame_2);
        }
        sf_ul.unlock();

    }

    //序号占2字节(编码后16bit)
    if (_send_str_ordinal == (1 << 16) - 1) {
        LOG(INFO) << "序号达到最大,下一包将从1开始";
        _send_str_ordinal = 1;
    }

    if (!current_send_fifo.empty()) {
        _cb(current_send_fifo);
    }
}


void LinkLayer::SendBinaryFile(const std::string &msg) {
    flatbuffers::FlatBufferBuilder builder;

    std::vector<uint8_t> data(msg.size());
    memcpy(data.data(), msg.data(), msg.size());
    auto binary_frame = EasyConnect::CreateFileFrameDirect(builder, "", 0, 0, data.size(), &data);

    builder.Finish(binary_frame);

    auto msgs = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_FileFrame, binary_frame.Union(),
                                           builder.CreateString(_device_id));
    builder.Finish(msgs);

    _trans->send_to(_Receiver.gui, builder.GetCurrentBufferPointer(), builder.GetSize());
}

void LinkLayer::SendThroughput() {
    if (_device_id != "receiver")return;
    flatbuffers::FlatBufferBuilder builder;

    std::unique_lock<std::mutex> ul(_mtxCk);
    int ackNum = _ackCount;
    int nckNum = _nckCount;
    _ackCount = 0;
    _nckCount = 0;
    ul.unlock();

    auto frame = EasyConnect::CreateThroughputFrame(builder, ackNum, nckNum);
    builder.Finish(frame);

    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Throughput, false,
                                               EasyConnect::CommandData_ThroughputFrame, frame.Union());
    builder.Finish(cmd);

    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(_device_id));
    builder.Finish(msg);

    _trans->send_to("disturb", builder.GetCurrentBufferPointer(), builder.GetSize());
    //_trans->send_to(_Receiver.gui, builder.GetCurrentBufferPointer(), builder.GetSize());
    _trans->send_to("strategy", builder.GetCurrentBufferPointer(), builder.GetSize());
    //_trans->send_to("GUIJammer", builder.GetCurrentBufferPointer(), builder.GetSize());
//    if (ackNum != 0 or nckNum != 0)
//        LOG(INFO) << "ack: " << ackNum << ", nck: " << nckNum;

    std::ofstream out1;
    out1.open("ack.txt",std::ios::app);
    out1<<" "<<ackNum;
    out1.close();

    std::ofstream out2;
    out2.open("nck.txt",std::ios::app);
    out2<<" "<<nckNum;
    out2.close();
}

void LinkLayer::SendThroughput_GUI() {
    if (_device_id != "receiver")return;
    flatbuffers::FlatBufferBuilder builder;

    std::unique_lock<std::mutex> ul(_mtxCk);
    int ackNum = _ackCount_GUI;
    int nckNum = _nckCount_GUI;
    _ackCount_GUI = 0;
    _nckCount_GUI = 0;
    ul.unlock();

    auto frame = EasyConnect::CreateThroughputFrame(builder, ackNum, nckNum);
    builder.Finish(frame);

    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Throughput, false,
                                               EasyConnect::CommandData_ThroughputFrame, frame.Union());
    builder.Finish(cmd);

    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(_device_id));
    builder.Finish(msg);

    _trans->send_to(_Receiver.gui, builder.GetCurrentBufferPointer(), builder.GetSize());
    _trans->send_to("GUIJammer", builder.GetCurrentBufferPointer(), builder.GetSize());

    if (ackNum != 0 or nckNum != 0)
        LOG(INFO) << "ack: " << ackNum << ", nck: " << nckNum;

}
