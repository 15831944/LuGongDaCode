//
// Created by locate on 2019/10/29.
//

#ifndef DEVICELAYER_HPP
#define DEVICELAYER_HPP

#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/chrono.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/exception.hpp>
#include <uhd/utils/thread.hpp>
#include <glog/logging.h>
#include <chrono>
#include <thread>
#include <mutex>

#include "Frame.hpp"
#include <easy_connect/EasyConnect.hpp>
#include <easy_connect/MessageFrame.hpp>
#include <easy_connect_generated.h>

class UHDDevice {
public:
    typedef std::shared_ptr<UHDDevice> sptr;
    uhd::usrp::multi_usrp::sptr _device;
    uhd::rx_streamer::sptr _rx_stream;
    uhd::tx_streamer::sptr _tx_stream;

    EasyConnectIface::sptr _trans;
    std::string _receiver = "GUI";
    std::string _device_id = "transmitter";

    enum err_code {
        ERROR_TIMING = -1,
        ERROR_UNRECOVERABLE = -2,
        ERROR_UNHANDLED = -3,
    };
public:
    double _rx_freq = 838e6;
    double _tx_freq = 838e6;
//    double _rx_rate = GSMRATE;
    double _rx_rate = 270e3;
    double _tx_rate = _rx_rate;
    double _rx_bandwidth = _rx_rate;
    double _tx_bandwidth = _tx_rate;
    double _rx_gain = 50;
    double _tx_gain = 50;

    double _clk_rate{};

    bool _first_recv_flag = true;

    uhd::time_spec_t _prev_ts{};

    std::mutex _tx_mtx;
    std::mutex _rx_mtx;

    bool _is_make = false;
public:
    explicit UHDDevice(const std::string &args, EasyConnectIface::sptr &trans, const std::string &id);

    ~UHDDevice();

    static sptr make(const std::string &args, EasyConnectIface::sptr &trans, const std::string &id);

    void Start();

    void Stop();

    void SetRxStream();

    void SetTxStream();

    void SetRxGain(double gain);

    void SetTxGain(double gain);

    void SetRxRate(double rate);

    void SetTxRate(double rate);

    void SetRxFreq(double rx_freq, double rx_lo_off);

    void SetTxFreq(double tx_freq, double tx_lo_0ff);

    void SetRxBandwidth(double bw);

    void SetTxBandwidth(double bw);

    void SetMasterClockRate(double clk_rate);

    void SetTimeNow(const uhd::time_spec_t &time_spec = 0.0);

    void SetTimeNextPPS(const uhd::time_spec_t &time_spec = 0.0);

    uhd::time_spec_t GetTimeNow();

    void StartRxStream();

    void StartRxStream(size_t total_samples);

    void StartRxStream(size_t total_samples, const uhd::time_spec_t &start_time);

    void StopRxStream();

    size_t Send(std::vector<std::complex<short>> &buffer, size_t sample_size, const TIMESTAMP &send_timestamp,
                double timeout = 0.5);

    size_t Recv(std::vector<std::complex<short>> &buffer, size_t sample_size, TIMESTAMP &recv_timestamp,
                double timeout = 0.5);

    size_t Send(std::vector<std::complex<short>> &buffer, size_t sample_size, const uhd::time_spec_t *timeSpec,
                double timeout = 0.5);

    size_t Recv(std::vector<std::complex<short>> &buffer, size_t sample_size, uhd::time_spec_t *timeSpec,
                double timeout = 0.5);

    int CheckRxMdErr(uhd::rx_metadata_t &md);

    double GetRxGain() { return _rx_gain; }

    double GetTxGain() { return _tx_gain; }

    double GetRxRate() { return _rx_rate; }

    double GetTxRate() { return _tx_rate; }

    double GetRxFreq() { return _device->get_rx_freq(); }

    double GetTxFreq() { return _device->get_tx_freq(); }

    double GetRxBandwidth() { return _rx_bandwidth; }

    double GetTxBandwidth() { return _tx_bandwidth; }

    double GetMasterClockRate() { return _device->get_master_clock_rate(); }

    uhd::freq_range_t GetTxFreqRange() { return _device->get_tx_freq_range(); }

    uhd::freq_range_t GetRxFreqRange() { return _device->get_rx_freq_range(); }

    uhd::meta_range_t GetTxRateRange() { return _device->get_tx_rates(); }

    uhd::meta_range_t GetRxRateRange() { return _device->get_rx_rates(); }

    uhd::gain_range_t GetTxGainRange() { return _device->get_tx_gain_range(); }

    uhd::gain_range_t GetRxGainRange() { return _device->get_rx_gain_range(); }

    void SetRxSubdev(const std::string &rx_subdev);

    void SetTxSubdev(const std::string &tx_subdev);

    void SetRxAntenna(const std::string &rx_antenna);

    void SetTxAntenna(const std::string &tx_antenna);

    void SetTimeSource(const std::string &source);

    void SetClockSource(const std::string &source);

    std::string GetClockSource(const size_t mboard = 0);

    std::string GetTimeSource(const size_t mboard = 0);

    bool GetMboardSensor(const size_t mboard = 0);

    uhd::usrp::multi_usrp::sptr &GetDevice();

    /**
     * 设置通信对象id
     * @param id
     */
    void SetReceiver(const std::string &id);

private:
    /**
     * 发送异常
     * @param str 异常信息
     */
    void SendException(const std::string &str);
};


#endif //DEVICELAYER_HPP
