
#ifndef DISTURB_H
#define DISTURB_H

#include <iostream>
#include <chrono>
#include <string>

#include "device/CtrlUsrp.h"
#include "device/IntelligentDecision.h"

#include <easy_connect_generated.h>
#include <easy_connect/EasyConnect.hpp>
#include <easy_connect/MessageFrame.hpp>
#include <easy_connect_generated.h>

#include <glog/logging.h>
#include <mutex>
#include <random>


#define msleep(N); boost::this_thread::sleep(boost::posix_time::milliseconds(N));

class Disturb {
    enum mode
    {
        BLOCK = 1,  //阻塞是干扰，不调频，固定频点
        TRACK = 2,  //跟踪干扰
        AI    = 3,  //智能干扰
        COMB_1 = 4, //梳状干扰1
        COMB_2 = 5, //梳状干扰2
        SWEEP_1 = 6, //单音扫频
        SWEEP_2 = 7, //多音扫频
        SWEEP_COMB = 8, //梳妆加扫频动态
    };

public:

    ~Disturb()
    {
        _dev.Stop();

        _loop = false;
        _track_running = false;
        _ai_running = false;

        msleep(10);

        if(_heart_thread->joinable())   //线程退出的时候把线程析构
            _heart_thread->interrupt();
        if(_track_thread->joinable())   //线程退出的时候把线程析构
            _track_thread->interrupt();
        if(_ai_thread->joinable())   //线程退出的时候把线程析构
            _ai_thread->interrupt();
    }


    void Start();   //开始


private:

    /**
     * 处理命令函数
     * @param id 发送者
     * @param cmd 命令
     */
    void HandleCommand(const std::string &id, EasyConnect::CommandFrame *cmd);
    /*处理不同mode下的发送决策*/
    void HandleMode(int mode,int channel);
    /*处理收到的消息，回调函数，有消息就调用*/
    void on_msg(const std::string &src, void *data, size_t size);
    /*实时发送心跳状态，证明设备在线*/
    void SendHeart();

    void ModeTrack();
    void ModeAI();
    void ModeComb();

    SIG_TYPE Freq_to_Mode(double freq);
    //生成随机数
    int randm(const int low, const int high) {
        std::random_device rd;
        return rd() % (high - low + 1) + low;
    }

protected:
    std::string             _id;    //当前设备ID

    CtrlUsrp                _dev;   //创建设备控制对象

    int _now_mode = 0;

    EasyConnectIface::sptr _trans;//与其他设备的通信接口

    boost::shared_ptr<boost::thread> _heart_thread; //发送心跳的线程
    boost::shared_ptr<boost::thread> _track_thread; //跟踪干扰跳频专用的线程
    boost::shared_ptr<boost::thread> _ai_thread;    //智能干扰的决策线程

    bool _track_running = false;
    bool _ai_running = false;

    bool _make = false; //设备是否初始化完成

    bool _loop = true;  //线程是否循环标志位


    std::deque<double>   _freq_list;    //跳频频率列表
    std::deque<uint64_t> _time_list;    //表示跳频频率列表里的每个频点需要跳频时间

    std::deque<double>  _nck_list;      //用来存储一段时间内的吞吐量，用来计算平均值的 ,规定长度为3

    std::mutex          _track_loker;   //锁，为了不同时操作上面两个列表，不然可能会出现某种错误
    std::mutex          _ai_loker;      //同理
};


#endif
