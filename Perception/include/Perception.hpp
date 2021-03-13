//
// Created by locate on 4/22/20.
//

#ifndef PERCEPTION_PERCEPTION_HPP
#define PERCEPTION_PERCEPTION_HPP

#include <memory>
#include <thread>
#include <list>
#include <condition_variable>
#include <uhd/usrp/multi_usrp.hpp>
#include <easy_connect/EasyConnect.hpp>
#include <easy_connect/MessageFrame.hpp>
#include <easy_connect_generated.h>
#include <uhd/terjin_iface.hpp>

class Spectrum {
public:
    typedef std::shared_ptr<Spectrum> sptr;

    Spectrum(double *data, size_t size) {
        _data.resize(size);
        memcpy(_data.data(), data, size * sizeof(double));
    }

    static Spectrum::sptr make(double *data, size_t size) {
        return std::make_shared<Spectrum>(data, size);
    }

    static Spectrum::sptr make(double data, size_t size) {
        std::vector<double> temp(size, 0);
        return make(temp.data(), temp.size());
    }

    std::vector<double> &Data() {
        return _data;
    }

private:
    std::vector<double> _data;


};

class Perception {
public:
    typedef std::shared_ptr<Perception> sptr;

    void reset_usb();

    Perception();

    ~Perception();

    static sptr make();

    void start_conn(short port);

    void init();

    void start_sweeper();

    void stop_sweeper();

    //handle msg
    void on_msg(const std::string &src, void *data, size_t size);

    void send_msg(const std::string &id, void *data, size_t size);

    //recv working
    void recv_working();

    void tsp_working();

    void state_working();

    void send_spectrum(const double *data, size_t size);

    void send_tsp(uint64_t tsp);

    void send_hop_command(uint64_t tsp, double hop_freq);

    void handle_command(const std::string &id, EasyConnect::CommandFrame *cmd);

    void handle_set_param(EasyConnect::ParameterFrame *parameter);

    uint64_t time_spec_to_ms(const uhd::time_spec_t &timeSpec);

    bool sync_time(const std::string &id, const std::string &ip);

    void send_exception(const std::string &id, const std::string &str);

    void broadcast_broker(const std::string &host);

private:
    EasyConnectIface::sptr _conn;

    uhd::usrp::multi_usrp::sptr _usrp;

    terjin_iface::sptr _iface;

    std::shared_ptr<std::thread> _recv_thread;
    std::shared_ptr<std::thread> _tsp_thread;

    std::shared_ptr<std::thread> _state_thread;

    struct performance_t {
        double snr;
        uint64_t tsp;
        bool syn;
    };

    std::vector<performance_t> _performance;

    std::list<Spectrum::sptr> _spectrum;


    bool _recv_running = false;
    bool _tsp_running = false;

    struct spec_param_t {
        double rx_freq;
        double rx_rate;
        double gain;
        int fft_size;
        int rx_len;

        double interval_s;
        double trans_band;        //channel bandwidth
        double guard_time_s; //guard time
        int performance_count;    //performance T
        int current_performance_index;
        int current_performance_count;

        performance_t performance_threshold;
        double hop_interval_s;//hop interval

        int spectrum_L;

        uhd::time_spec_t _last_check_need_hop{0, 0};
    } _spec_param{};

    struct {
        std::string Client = "GUI";
        std::string Perception = "perception";
        std::string Transmitter = "transmitter";
        std::string Receiver = "receiver";
        std::string Disturb = "disturb";
        std::string Strategy = "strategy";
    } _dev;


    bool _loop = true;
};


#endif //PERCEPTION_PERCEPTION_HPP
