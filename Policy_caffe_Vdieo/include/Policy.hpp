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
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/lexical_cast.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include "PlayEngine.hpp"
//#include "dqn.hpp"
//#include <dqn>

//Spectrum_Data
using Spectrum_lw = std::array<double , 1024>;
using waterfall_data = std::array<double, 200>;
using SpectrumSp = std::shared_ptr<waterfall_data>;

struct WFcolor
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};
void generateColorMap();
WFcolor bytesTovec3(uint8_t byte);




//class Spectrum {
//public:
//    typedef std::shared_ptr<Spectrum> sptr;
//
//    Spectrum(double *data, size_t size) {
//        _data.resize(size);
//        memcpy(_data.data(), data, size * sizeof(double));
//    }
//
//    static Spectrum::sptr make(double *data, size_t size) {
//        return std::make_shared<Spectrum>(data, size);
//    }
//
//    static Spectrum::sptr make(double data, size_t size) {
//        std::vector<double> temp(size, 0);
//        return make(temp.data(), temp.size());
//    }
//
//    std::vector<double> &Data() {
//        return _data;
//    }
//
//private:
//    std::vector<double> _data;
//
//
//};

#define SPEC_POOL_SIZE 15

class Policy {
public:
    typedef std::shared_ptr<Policy> sptr;

    void reset_usb();

    Policy();

    ~Policy();

    static sptr make();

    void start_conn(short port);
    //Start policy thread
    //void start_policy(EasyConnect::CommandFrame *cmd);

    void stop_policy();
    //handle msg
    void on_msg(const std::string &id, void *data, size_t size);
    //send start command
    void send_msg(const std::string &id, void *data, size_t size);
    // generate hop frequency policy
    void fixed_hop(double freq); //固定跳频
    void sense_hop(); //感知跳频 2s
    void probab_hop(); //概率跳频 0.2 0.7 1-4 0.3 5-8
    void intell_hop(); //dqn智能跳频
    void DQN_hop(); // 换一种写法
    void Auto_hop(int32_t Mode);

    void Play_vdieo();


    void Handle_Spectrum(const std::string &id, EasyConnect::SpectrumData *spec_recv);
    // Send heart state
    void state_working();
    // Send hop frequency to Transmitter
    void send_hop_command(uint64_t tsp, double hop_freq);

    // Handle the command from other device
    void handle_command(const std::string &id, EasyConnect::CommandFrame *cmd);
    // change the param from the Client
   // void handle_set_param(EasyConnect::ParameterFrame *parameter);

    uint64_t time_spec_to_ms(const uhd::time_spec_t &timeSpec);

    //bool sync_time(const std::string &id, const std::string &ip);

    void send_exception(const std::string &id, const std::string &str);

    void broadcast_broker(const std::string &host);

    /****DQN related functions****/
    double CalculateEpsilon(const int iter);


private:
    EasyConnectIface::sptr _conn; //jie ru duan kou

   //uhd::usrp::multi_usrp::sptr _usrp;

    //terjin_iface::sptr _iface;

    std::shared_ptr<std::thread> _generate_thread;

    std::shared_ptr<std::thread> _state_thread;

    std::shared_ptr<std::thread> _vdieo_thread;
    std::shared_ptr<std::thread> _Auto_thread;


    bool _generate_running = false;
    bool _vdieo_running = false;
    bool _auto_running = false;
    //bool _fixed_freq = true;

//    struct  dqn_param_t{
//        double lr = 0.8;
//        double gamma = 0.1;
//    }_dqn_param;

    struct  spec_param_t{
        double guard_time_s;
      double rx_freq;
      double rx_rate;
      int fft_size;
      double trans_band;
        int spectrum_L;
        int spectrum_sense;
    } _spec_param{};
    std::deque<Spectrum_lw> _water;

    std::deque<EasyConnect::SpectrumData*> _spec;



    struct {
        std::string Client = "GUI";
        std::string Perception = "perception";
        std::string Transmitter = "transmitter";
        std::string Receiver = "receiver";
        std::string Disturb = "disturb";
        std::string Strategy= "strategy";
    } _dev;

    bool _loop = true;
    bool _make = false;

    uint64_t _curr_time;

    float _greed=0.95;

    //std::deque<double> _ack_list;
    std::deque<double> _nck_list;

    boost::mutex _poolLocker;
    std::mutex   _ai_loker;
};


#endif //PERCEPTION_PERCEPTION_HPP
