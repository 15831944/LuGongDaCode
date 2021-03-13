
#include "device/CtrlUsrp.h"
#include "glog/logging.h"


CtrlUsrp::CtrlUsrp()
{
    _trans = new Transmitter();
    _trans->init("", "RF", "internal");

    //给参数配置初始化一个值
    _param.txFreq = 838e6;
    _param.txGain = 60;
    _param.txRate = 10e6;
    _trans->setParam(_param);
}

CtrlUsrp::~CtrlUsrp()
{
    delete _trans;
}

void CtrlUsrp::Start()
{
    //开始发送
    _trans->start();
}

void CtrlUsrp::Stop()
{
    //设备停止发送
    _trans->stop();
}

void CtrlUsrp::SetParam(double rate,double freq,double gain)
{

    _param.txRate = rate;
    _param.txFreq = freq;
    _param.txGain = gain;

    _trans->setParam(_param);
}

void CtrlUsrp::SetFreq(double freq)
{
    //更改板子频率接口
    _trans->setFreqNow(freq);
}

/*设置发送信号类型*/
void CtrlUsrp::SetSignalType(SIG_TYPE type)
{
    Transmitter::SourceDesc sd = Transmitter::SourceDesc();
    switch(type)
    {
        case GAUSS:
        {   //阻塞式，高斯噪声
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"GAUSS.dat").c_str());
            break;
        }
        case COMB:
        {   //梳妆干扰
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"COMB.dat").c_str());
            break;
        }
        case COMB1:
        {   //梳妆干扰1
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"COMB1.dat").c_str());
            break;
        }
        case COMB2:
        {   //梳妆干扰2
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"COMB2.dat").c_str());
            break;
        }
        case SWEEP_ONE:
        {   //单音扫频
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"SWEEP_ONE.dat").c_str());
            break;
        }
        case SWEEP_DOUBLE:
        {   //双扫频干扰
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"SWEEP_DOUBLE.dat").c_str());
            break;
        }
//        case SWEEP_103_01:
//        {   //双扫频干扰
//            sd.type = Transmitter::SourceDesc::FILE;
//            strcpy(sd.description,(_fileAddr+"sweep_10e3_20dB.dat").c_str());
//            break;
//        }
        case COMB_SWEEP:
        {
//            sd.type = Transmitter::SourceDesc::FILE;
//            strcpy(sd.description,(_fileAddr+"comb+sweep.dat").c_str());
            break;
        }
        case AM:
        {   //梳妆干扰+扫频干扰
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"AM.dat").c_str());
            break;
        }
        case SSB:
        {
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"SSB.dat").c_str());
            break;
        }
        case FM:
        {
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"FM.dat").c_str());
            break;
        }
        case BPSK:
        {
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"BPSK.dat").c_str());
            break;
        }
        case QPSK:
        {
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"QPSK.dat").c_str());
            break;
        }
        case GMSK:
        {
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"GMSK.dat").c_str());
            break;
        }
        case CHANNEL_1:
        {
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"CHANNEL_1.dat").c_str());
            break;
        }
        case CHANNEL_2:
        {
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"CHANNEL_2.dat").c_str());
            break;
        }
        case CHANNEL_3:
        {
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"CHANNEL_3.dat").c_str());
            break;
        }
        case CHANNEL_4:
        {
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"CHANNEL_4.dat").c_str());
            break;
        }
        case CHANNEL_5:
        {
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"CHANNEL_5.dat").c_str());
            break;
        }
        case CHANNEL_6:
        {
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"CHANNEL_6.dat").c_str());
            break;
        }
        case CHANNEL_7:
        {
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"CHANNEL_7.dat").c_str());
            break;
        }
        case CHANNEL_8:
        {
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"CHANNEL_8.dat").c_str());
            break;
        }
        case CHANNEL_9:
        {
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"CHANNEL_9.dat").c_str());
            break;
        }
        default:
        {   //如果都不是就默认阻塞式，高斯噪声
            sd.type = Transmitter::SourceDesc::FILE;
            strcpy(sd.description,(_fileAddr+"GAUSS.dat").c_str());
            break;
        }
    }

    //目的是为了防止发送干扰信号过程中切换发送信号，先停止再开始，大概1ms-5ms，不影响实际效果
    if(_trans->isOpen)
    {
        Stop();
        _trans->setSource(sd);
        Start();
    }
    else
    {
        _trans->setSource(sd);
    }
}

bool CtrlUsrp::IsOpne() {

    return _trans->isOpen;
}





