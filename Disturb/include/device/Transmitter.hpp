#ifndef TRANSMITTER_HPP
#define TRANSMITTER_HPP

//UHD
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>

//BOOST
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread.hpp>
#include <boost/format.hpp>

//STD
#include <iostream>
#include <complex>


enum TrxSignalname {
    SET_PARAM = 1,
    SET_SOURCE = 2,
    START_SENDING = 3
};


struct TrxSignal {
    TrxSignalname name;
    char data[1024];
};



//大概100K的消息空间
#define SIGNAL_POOL_SIZE 100
typedef boost::shared_ptr<TrxSignal> TrxSignalSp;
typedef std::deque<TrxSignalSp> SignalPool;
const TrxSignalSp NullSp = TrxSignalSp((TrxSignal *) nullptr);


template<typename datatype>
datatype ParseData(char *data);

template<typename datatype>
void WriteData(char *dest, datatype data);


class Transmitter {
public:

//设备状态
    enum State {
        IDLE = 0,  //空闲
        SENDING = 1, //发送状态
        ERROR = 2 //异常
    };


//设备参数
    struct DeviceParam {
        double txFreq; //发送中心频率
        double txGain; //增益
        double txRate; //速率
    };

//样值数据结构
    typedef std::complex<short> samp_type;

//数据源描述
    struct SourceDesc {
        enum Stype {
            FILE = 0,       //文件传输
            CONST = 1,      //连续波
            GAUSS = 2,      //高斯噪声
            COMB = 3,       //梳状
            SWEEP = 4,      //扫频
            SWEEP_ONE = 5,  //单音扫频
            COMB_SWEEP = 6, //扫频梳状
            SWEEP_DOUBLE= 7,//双音扫频
        } type = Stype(1);
        char description[512]{};
    };

    bool isOpen = false;

private:
    //硬件设备控制模块
    uhd::usrp::multi_usrp::sptr _dev;
    //与设备相关的参数
    DeviceParam _param;
    //设备状态
    State _state;
    //发送控制信号
    bool _stop_sending_called;
    //线程循环状态
    bool workloop=true;
    //主线程
    boost::shared_ptr<boost::thread> _work_thread;
    //线程锁，保护_sigPool
    boost::mutex _poolLocker;
    //消息缓冲器
    SignalPool _sigPool;
    //发送数据流
    uhd::tx_streamer::sptr _tx_stream;
    //每包的文件大小
    size_t _samples_per_unit;
    //发送数据源
    SourceDesc _sd;

public:
    Transmitter();

    ~Transmitter();

    //API 接口函数
    int init(const std::string &args,   // for example "addr=192.168.10.2 types=x300"
              const std::string &band,   //子板名称
              const std::string &time_source //触发脉冲源
    );

    //设置参数
    void setParam(DeviceParam param);
    //设置频率
    void setFreqNow(double freq);
    //设置采样率
    void setTxRate(double rate);
    //设置增益
    void setTxGain(double gain);
    //停止发送
    void stop();
    //开始发送
    void start();

    //设置状态机变成发送状态
    void setSource(SourceDesc sd);
    //设置发送通道
    void setTxBand(std::string band);


    //获取设备参数
    double getFreqNow();
    double getRateNow();
    double getGainNow();


protected:

    //主循环
    void workLoop();
    //状态机相关
    void processIdle();
    //处理发送函数
    void processSending();
    //错误函数
    void processError();
    //空闲函数
    void enterIdleState();

    void enterSendingState();

    void enterErrorState();


    //消息处理函数
    template<typename datatype>
    void addSignal(TrxSignalname name, datatype data) {
        TrxSignalSp sig = TrxSignalSp(new TrxSignal);
        sig->name = name;
        WriteData(sig->data, data);
        //memcpy(&sig->data,&data,sizeof(datatype));
        _poolLocker.lock();
        if (_sigPool.size() >= SIGNAL_POOL_SIZE) _sigPool.pop_front();
        _sigPool.push_back(sig);
        _poolLocker.unlock();
    };
    //将消息放到消息缓冲区
    TrxSignalSp popSignal();

    void applyCurrentParam();



};

#endif

