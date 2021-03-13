/*
 * 控制板子
 *
 * */


#ifndef CTRLUSRP_H
#define CTRLUSRP_H

#include <iostream>
#include "Transmitter.hpp"

enum SIG_TYPE
{
    GAUSS = 0,      //高斯噪声
    COMB = 1,       //梳状
    SWEEP_ONE = 2,  //单音扫频
    COMB_SWEEP = 3, //扫频梳状
    SWEEP_DOUBLE= 4,//双音扫频



    AM = 5,
    SSB = 6,
    FM = 7,
    BPSK = 8,
    QPSK = 9,
    GMSK = 10,

    CHANNEL_1 = 11,      //信道1干扰型号
    CHANNEL_2 = 12,      //信道2干扰型号
    CHANNEL_3 = 13,      //信道3干扰型号
    CHANNEL_4 = 14,      //信道4干扰型号
    CHANNEL_5 = 15,      //信道5干扰型号
    CHANNEL_6 = 16,      //信道6干扰型号
    CHANNEL_7 = 17,      //信道7干扰型号
    CHANNEL_8 = 18,      //信道8干扰型号
    CHANNEL_9 = 19,      //信道9干扰型号

    COMB1  = 20,         //梳状1
    COMB2  = 21,         //梳状2

//    SWEEP_103_01 = 22,
};


class CtrlUsrp {

public:
    CtrlUsrp();

    ~CtrlUsrp();
    //开始
    void Start();
    //停止
    void Stop();
    //配置参数，频率带宽增益
    void SetParam(double rate,double freq,double gain);
    //配置发送信号类型
    void SetSignalType(SIG_TYPE type);
    //单独配置发送频率接口
    void SetFreq(double freq);

    bool IsOpne();

    //获取当前状态
    double NowFreq() {return _trans->getFreqNow();}
    double NOWGain() {return _trans->getGainNow();}
    double NOWRate() {return _trans->getRateNow();}

private:
    Transmitter *_trans; //发送设备

    std::string _fileAddr="./data/";

    Transmitter::DeviceParam _param{};
};


#endif
