//
// Created by locate on 2019/10/29.
//

#include <string>
#include "UHDDevice.hpp"

#include <boost/format.hpp>

UHDDevice::UHDDevice(const std::string &args, EasyConnectIface::sptr &trans, const std::string &id) {

    _trans = trans;
    _device_id = id;

    // Find UHD devices
#if 1
    uhd::device_addr_t addr(args);
    uhd::device_addrs_t dev_addrs = uhd::device::find(addr);
    if (dev_addrs.empty()) {
        std::string str;
        str = (boost::format("No UHD devices found with address '%s', exit...") % args).str();
        LOG(ERROR) << str;
        SendException(str);
        exit(-1);
    }

    bool find_device = false;
    for (size_t j = 0; j < dev_addrs.size() and not find_device; ++j) {
        // make
        // Use the found device
        LOG(INFO) << "Using discovered UHD device " << dev_addrs[j].to_string();
        try {
            _device = uhd::usrp::multi_usrp::make(dev_addrs[j]);
        } catch (uhd::exception &e) {
            LOG(ERROR) << e.what();
            if (j == dev_addrs.size() - 1) {
                SendException(e.what());
                exit(-1);
            }
            exit(-1);
        } catch (...) {
            std::string str;
            str = (boost::format("UHD make failed, device %s, exit...") % dev_addrs[0].to_string()).str();
            LOG(ERROR) << str;
            if (j == dev_addrs.size() - 1) {
                SendException(str);
                exit(-1);
            }
            exit(-1);
        }
        find_device = true;
    }
#else
    try {
        _device = uhd::usrp::multi_usrp::make(args);
    } catch (uhd::exception &e) {
        LOG(INFO) << e.what();
        SendException(e.what());
        exit(-1);
    } catch (...) {
        std::string str;
        str = (boost::format("No UHD devices found with address '%s', exit...") % args).str();
        LOG(INFO) << str;
        SendException(str);
        exit(-1);
    }

#endif

    _is_make = true;
}

UHDDevice::~UHDDevice() {
//    Stop();
}

UHDDevice::sptr UHDDevice::make(const std::string &args, EasyConnectIface::sptr &trans, const std::string &id) {
    return std::make_shared<UHDDevice>(args, trans, id);
}

void UHDDevice::Start() {

    _device->set_rx_freq(_rx_freq);
    _device->set_tx_freq(_tx_freq);

    _rx_freq = _device->get_rx_freq();
    _tx_freq = _device->get_tx_freq();
    LOG(INFO) << "current rx/tx freq is " << _rx_freq << "/" << _tx_freq;

    _device->set_rx_rate(_rx_rate);
    _device->set_tx_rate(_tx_rate);

    _rx_rate = _device->get_rx_rate();
    _tx_rate = _device->get_tx_rate();
    LOG(INFO) << "current rx/tx rate is " << _rx_rate << "/" << _tx_rate;

    _device->set_rx_bandwidth(_rx_bandwidth);
    _device->set_tx_bandwidth(_tx_bandwidth);

    _rx_bandwidth = _device->get_rx_bandwidth();
    _tx_bandwidth = _device->get_tx_bandwidth();
    LOG(INFO) << "current rx/tx bandwidth is " << _rx_bandwidth << "/" << _tx_bandwidth;

    _device->set_rx_gain(_rx_gain);
    _device->set_tx_gain(_tx_gain);

    _rx_gain = _device->get_rx_gain();
    _tx_gain = _device->get_tx_gain();
    LOG(INFO) << "current rx/tx gain is " << _rx_gain << "/" << _tx_gain;

//    _device->set_master_clock_rate(26e6);
    _clk_rate = _device->get_master_clock_rate();
    LOG(INFO) << "current master clock rate is " << _clk_rate;

    uhd::stream_args_t stream_args("sc16", "sc16");
    stream_args.channels = std::vector<size_t>{0};
    _rx_mtx.lock();
    _rx_stream = _device->get_rx_stream(stream_args);
    _rx_mtx.unlock();
    _tx_mtx.lock();
    _tx_stream = _device->get_tx_stream(stream_args);
    _tx_mtx.unlock();

    _device->set_time_now(0.0);

    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.stream_now = true;
    _rx_mtx.lock();
    _rx_stream->issue_stream_cmd(stream_cmd);
    _rx_mtx.unlock();

    LOG(INFO) << "device started at " << _device->get_time_now().get_real_secs() << "s";
}

void UHDDevice::Stop() {
    uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    cmd.stream_now = true;

    try {
        _device->issue_stream_cmd(cmd);
    } catch (uhd::exception &e) {
        LOG(INFO) << e.what();
        SendException(e.what());
    } catch (...) {
        std::string str("Device Stop error");
        LOG(INFO) << str;
        SendException(str);
    }
}

void UHDDevice::SetRxSubdev(const std::string &rx_subdev) {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }
    _device->set_rx_subdev_spec(rx_subdev);
}

void UHDDevice::SetTxSubdev(const std::string &tx_subdev) {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }
    _device->set_tx_subdev_spec(tx_subdev);
}

void UHDDevice::SetRxStream() {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }
    boost::this_thread::sleep(boost::posix_time::milliseconds(2));
    uhd::stream_args_t rx_stream_args("sc16", "sc16");
    rx_stream_args.channels = std::vector<size_t>{0};
    std::lock_guard<std::mutex> lg(_rx_mtx);
    if (_rx_stream.get()) {
        _rx_stream.reset();
    }
    _rx_stream = _device->get_rx_stream(rx_stream_args);
    LOG(INFO) << "rx stream is built";
}

void UHDDevice::SetTxStream() {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }
    boost::this_thread::sleep(boost::posix_time::milliseconds(2));
    uhd::stream_args_t tx_stream_args("sc16", "sc16");
    tx_stream_args.channels = std::vector<size_t>{0};

    std::lock_guard<std::mutex> lg(_tx_mtx);
    if (_tx_stream.get()) {
        _tx_stream.reset();
    }
    _tx_stream = _device->get_tx_stream(tx_stream_args);
    LOG(INFO) << "tx stream is built";
}

void UHDDevice::SetRxGain(double gain) {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }

    _device->set_rx_gain(gain);
    _rx_gain = _device->get_rx_gain();
    LOG(INFO) << "current rx gain is " << _rx_gain;
}

void UHDDevice::SetTxGain(double gain) {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }

    _device->set_tx_gain(gain);
    _tx_gain = _device->get_tx_gain();
    LOG(INFO) << "current tx gain is " << _tx_gain;
}

void UHDDevice::SetRxRate(double rate) {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }

    _device->set_rx_rate(rate);
    _rx_rate = _device->get_rx_rate();
    LOG(INFO) << "current rx rate is " << _rx_rate;
}

void UHDDevice::SetTxRate(double rate) {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }

    _device->set_tx_rate(rate);
    _tx_rate = _device->get_tx_rate();
    LOG(INFO) << "current tx rate is " << _tx_rate;
}

void UHDDevice::SetRxFreq(double rx_freq, double rx_lo_off) {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }
    if (rx_lo_off != 0.0) {
        _device->set_rx_freq(rx_freq, rx_lo_off);
    } else {
        _device->set_rx_freq(rx_freq);
    }

    _rx_freq = _device->get_rx_freq();
    //LOG(INFO) << "current rx freq is " << _rx_freq;
}

void UHDDevice::SetTxFreq(double tx_freq, double tx_lo_off) {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }

    if (tx_lo_off != 0.0) {
        _device->set_tx_freq(tx_freq, tx_lo_off);
    } else {
        _device->set_tx_freq(tx_freq);
    }
    _tx_freq = _device->get_tx_freq();
    LOG(INFO) << "current tx freq is " << _tx_freq;
}

void UHDDevice::SetRxBandwidth(double bw) {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }

    _device->set_rx_bandwidth(bw);
    _rx_bandwidth = _device->get_rx_bandwidth();
    LOG(INFO) << "current rx bandwidth is " << _rx_bandwidth;
}

void UHDDevice::SetTxBandwidth(double bw) {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }
    _device->set_tx_bandwidth(bw);

    _tx_bandwidth = _device->get_tx_bandwidth();
    LOG(INFO) << "current tx bandwidth is " << _tx_bandwidth;
}

void UHDDevice::SetMasterClockRate(double clk_rate) {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }

    _device->set_master_clock_rate(clk_rate);
    _clk_rate = _device->get_master_clock_rate();
    LOG(INFO) << "current master clock rate is " << _clk_rate;
}

void UHDDevice::SetTimeNow(const uhd::time_spec_t &time_spec) {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }
    try {
        _device->set_time_now(time_spec);
    } catch (std::exception &e) {
        LOG(INFO) << e.what();
    }

}

void UHDDevice::SetTimeNextPPS(const uhd::time_spec_t &time_spec) {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }
    try {
        _device->set_time_next_pps(time_spec);
    } catch (std::exception &e) {
        LOG(INFO) << e.what();
    }

}

uhd::time_spec_t UHDDevice::GetTimeNow() {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return 0.0;
    }
    uhd::time_spec_t dev_time;
    try {
        dev_time = _device->get_time_now();
    } catch (std::exception &e) {
        LOG(ERROR) << e.what();
        return 0.0;
    }
    return dev_time;
}

void UHDDevice::StartRxStream() {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }
    if (!_rx_stream.get()) {
        SetRxStream();
    }
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.stream_now = true;

    try {
        std::lock_guard<std::mutex> lg(_rx_mtx);
        _rx_stream->issue_stream_cmd(stream_cmd);
    } catch (uhd::exception &e) {
        LOG(INFO) << e.what();
        SendException(e.what());
    } catch (...) {
        std::string str("Device start rx stream error");
        LOG(INFO) << str;
        SendException(str);
    }
}

void UHDDevice::StartRxStream(size_t total_samples) {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }
    if (!_rx_stream.get()) {
        SetRxStream();
    }
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.stream_now = true;
    stream_cmd.num_samps = total_samples;
    _rx_mtx.lock();
    _rx_stream->issue_stream_cmd(stream_cmd);
    _rx_mtx.unlock();
}

void UHDDevice::StartRxStream(size_t total_samples, const uhd::time_spec_t &start_time) {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }
    if (!_rx_stream.get()) {
        SetRxStream();
    }
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.stream_now = false;
    stream_cmd.num_samps = total_samples;
    stream_cmd.time_spec = _device->get_time_now() + start_time;
    _rx_mtx.lock();
    _rx_stream->issue_stream_cmd(stream_cmd);
    _rx_mtx.unlock();
}

void UHDDevice::StopRxStream() {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }
    if (!_rx_stream.get()) {
        return;
    }
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    stream_cmd.stream_now = true;

    try {
        std::lock_guard<std::mutex> lg(_rx_mtx);
        _rx_stream->issue_stream_cmd(stream_cmd);
    } catch (uhd::exception &e) {
        LOG(INFO) << e.what();
        SendException(e.what());
    } catch (...) {
        std::string str("Device stop rx stream error");
        LOG(INFO) << str;
        SendException(str);
    }
}


size_t UHDDevice::Send(std::vector<std::complex<short>> &buffer, size_t sample_size, const TIMESTAMP &send_timestamp,
                       double timeout) {

    uhd::tx_metadata_t tx_md;
    tx_md.has_time_spec = true;
    tx_md.time_spec = uhd::time_spec_t::from_ticks(send_timestamp, _tx_rate);

    size_t send_size = 0;
    try {
        std::lock_guard<std::mutex> lg(_tx_mtx);
        send_size = _tx_stream->send(buffer.data(), sample_size, tx_md, timeout);
    } catch (uhd::exception &e) {
        LOG(INFO) << e.what();
        SendException(e.what());
        return 0;
    } catch (...) {
        std::string str("send buffer catch an error");
        LOG(INFO) << str;
        SendException(str);
    }

    if (send_size != sample_size) {
        LOG(INFO) << "Device send time out";
    }

    return send_size;
}

size_t UHDDevice::Recv(std::vector<std::complex<short>> &buffer, size_t sample_size, TIMESTAMP &recv_timestamp,
                       double timeout) {
    uhd::rx_metadata_t rx_md;

    size_t recv_size = 0;
    try {
        std::lock_guard<std::mutex> lg(_rx_mtx);
        recv_size = _rx_stream->recv(buffer.data(), sample_size, rx_md, timeout);
    } catch (uhd::exception &e) {
        LOG(INFO) << e.what();
        SendException(e.what());
        return 0;
    } catch (...) {
        std::string str("recv buffer catch an error");
        LOG(INFO) << str;
        SendException(str);
    }

    // Check for errors
    CheckRxMdErr(rx_md);

    recv_timestamp = rx_md.time_spec.to_ticks(_rx_rate);

//    if (_first_recv_flag) {
//        LOG(INFO) << "recv first buffer time = " << rx_md.time_spec.get_real_secs()
//                  << "s, timestamp = " << rx_md.time_spec.get_real_secs() * _rx_rate
//                  << ", actual timestamp = " << recv_timestamp;
//        _first_recv_flag = false;
//    }

    return recv_size;
}

size_t UHDDevice::Send(std::vector<std::complex<short>> &buffer, size_t sample_size, const uhd::time_spec_t *timeSpec,
                       double timeout) {

    uhd::tx_metadata_t tx_md;
    if (timeSpec != nullptr) {
        tx_md.has_time_spec = true;
        tx_md.time_spec = *timeSpec;
    }

    size_t send_size = 0;
    try {
        std::lock_guard<std::mutex> lg(_tx_mtx);
        send_size = _tx_stream->send(buffer.data(), sample_size, tx_md, timeout);
    } catch (uhd::exception &e) {
        LOG(INFO) << e.what();
        SendException(e.what());
        return 0;
    } catch (...) {
        std::string str("send buffer catch an error");
        LOG(INFO) << str;
        SendException(str);
    }

    if (send_size != sample_size) {
        LOG(INFO) << "Device send time out";
    }

    return send_size;
}

size_t UHDDevice::Recv(std::vector<std::complex<short>> &buffer, size_t sample_size, uhd::time_spec_t *timeSpec,
                       double timeout) {
    uhd::rx_metadata_t rx_md;

    size_t recv_size = 0;
    try {
        std::lock_guard<std::mutex> lg(_rx_mtx);
        recv_size = _rx_stream->recv(buffer.data(), sample_size, rx_md, timeout);
    } catch (uhd::exception &e) {
        LOG(INFO) << e.what();
        SendException(e.what());
        return 0;
    } catch (...) {
        std::string str("recv buffer catch an error");
        LOG(INFO) << str;
        SendException(str);
    }

    // Check for errors
    CheckRxMdErr(rx_md);

    if (timeSpec != nullptr)
        *timeSpec = rx_md.time_spec;

    return recv_size;
}

int UHDDevice::CheckRxMdErr(uhd::rx_metadata_t &md) {

    switch (md.error_code) {
        case uhd::rx_metadata_t::ERROR_CODE_NONE:
//            LOG(ERROR) << "UHD: No error";
            break;
        case uhd::rx_metadata_t::ERROR_CODE_TIMEOUT:
            LOG(ERROR) << "UHD: No packet received, implementation timed-out";
            break;
        case uhd::rx_metadata_t::ERROR_CODE_LATE_COMMAND:
            LOG(ERROR) << "UHD: stream command was issued in the past";
            break;
        case uhd::rx_metadata_t::ERROR_CODE_BROKEN_CHAIN:
            LOG(ERROR) << "UHD: Expected another stream command";
            break;
        case uhd::rx_metadata_t::ERROR_CODE_OVERFLOW:
            LOG(ERROR) << "UHD: An internal receive buffer has filled";
            break;
        case uhd::rx_metadata_t::ERROR_CODE_BAD_PACKET:
            LOG(ERROR) << "UHD: The packet could not be parsed";
            break;
        default:
            LOG(ERROR) << "UHD: Unknown error " << md.error_code;
    }

    // Missing timestamp
    if (!md.has_time_spec) {
        LOG(INFO) << "UHD: Received packet missing timestamp";
        return ERROR_UNRECOVERABLE;
    }

    uhd::time_spec_t ts;
    ts = md.time_spec;

    // Monotonicity check
    if (ts < _prev_ts) {
        LOG(INFO) << "UHD: Loss of monotonic time";
        LOG(INFO) << "Current time: " << ts.get_real_secs() << ", "
                  << "Previous time: " << _prev_ts.get_real_secs();
        _prev_ts = ts;
        return ERROR_TIMING;
    }

    _prev_ts = ts;

    return 0;
}

void UHDDevice::SetRxAntenna(const std::string &rx_antenna) {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }

    _device->set_rx_antenna(rx_antenna);
    LOG(INFO) << "current rx antenna is " << _device->get_rx_antenna();
}

void UHDDevice::SetTxAntenna(const std::string &tx_antenna) {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }

    _device->set_tx_antenna(tx_antenna);
    LOG(INFO) << "current tx antenna is " << _device->get_tx_antenna();
}

void UHDDevice::SetReceiver(const std::string &id) {
    _receiver = id;
}

void UHDDevice::SendException(const std::string &str) {
    flatbuffers::FlatBufferBuilder builder;
    auto err = EasyConnect::CreateStringFrame(builder, builder.CreateString(str));
    builder.Finish(err);
    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Exception, false,
                                               EasyConnect::CommandData_StringFrame, err.Union());
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(_device_id));
    builder.Finish(msg);
    _trans->send_to(_receiver, builder.GetCurrentBufferPointer(), builder.GetSize());
}

void UHDDevice::SetTimeSource(const std::string &source) {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }

    try {
        _device->set_time_source(source);
    } catch (std::exception &e) {
        LOG(INFO) << e.what();
    }

    LOG(INFO) << "current time source is " << _device->get_time_source(0);
}

void UHDDevice::SetClockSource(const std::string &source) {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return;
    }

    try {
        _device->set_clock_source(source);
    } catch (std::exception &e) {
        LOG(INFO) << e.what();
    }

    LOG(INFO) << "current clock source is " << _device->get_clock_source(0);
}

std::string UHDDevice::GetClockSource(const size_t mboard) {

    std::string source = "No";

    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return source;
    }

    try {
        source = _device->get_clock_source(mboard);
    } catch (std::exception &e) {
        LOG(INFO) << e.what();
    }

    LOG(INFO) << "current clock source is " << source;
    return source;
}

std::string UHDDevice::GetTimeSource(const size_t mboard) {
    std::string source = "No";

    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return source;
    }

    try {
        source = _device->get_time_source(mboard);
    } catch (std::exception &e) {
        LOG(INFO) << e.what();
    }

    LOG(INFO) << "current clock source is " << source;
    return source;
}

bool UHDDevice::GetMboardSensor(const size_t mboard) {
    if (!_is_make) {
        LOG(INFO) << "device didn't make";
        return false;
    }

    bool res = false;
    try {
        res = _device->get_mboard_sensor("ref_locked", mboard).to_bool();
    } catch (std::exception &e) {
        LOG(INFO) << e.what();
    }

    return res;
}

uhd::usrp::multi_usrp::sptr &UHDDevice::GetDevice() {
    return _device;
}