#include "device/Transmitter.hpp"
#include <fstream>
#include <glog/logging.h>



Transmitter::Transmitter() {
    _samples_per_unit = 362;

    system("sysctl -w net.core.rmem_max=57600000");
    system("sysctl -w net.core.wmem_max=57600000");
}

Transmitter::~Transmitter() {
    workloop= false;    //对象析构时，让线程退出
    if (_work_thread->joinable()) _work_thread->join();

}

void Transmitter::setTxBand(const std::string) {
    //切换子板
    //_dev->set_tx_subdev_spec(std::string("A:A"));   //设置通道   A A:A  A:B

    //配置发送数据流
    uhd::stream_args_t stream_args("sc16", "sc16"); //complex floats
    _tx_stream = _dev->get_tx_stream(stream_args);
    _samples_per_unit = _tx_stream->get_max_num_samps();

    boost::this_thread::sleep(boost::posix_time::milliseconds(200));

}

int Transmitter::init(const std::string &args,   // for example "addr=192.168.10.2 types=x300"
                       const std::string &band,
                       const std::string &time_source)
{

    _state = ERROR;
    while (true) {
        try {
            //构建母板
            _dev = uhd::usrp::multi_usrp::make(args);
            LOG(INFO) << boost::format("Using Device: %s") % _dev->get_pp_string();

            break;
        }
        catch (std::exception &e)
        {
            LOG(INFO) << e.what()<<"\n没有找到设备";
            enterErrorState();
            exit(1);
        }

    }


    setTxBand(band);

    _work_thread = boost::make_shared<boost::thread>(&Transmitter::workLoop, this);

    _state = IDLE; // 启动线程之后再进入IDLE
}


void Transmitter::workLoop() {
    while (workloop) {
        switch (_state) {
            case IDLE:
                processIdle();
                break;
            case SENDING:
                processSending();
                break;
            case ERROR:
                processError();
                break;
            default:
                processError();
                break;
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(1));

    }
    LOG(INFO)<<"work out";
}

template<typename datatype>
datatype ParseData(char *data) {
    datatype param;
    memcpy(&param, data, sizeof(datatype));
    return param;
}

template<typename datatype>
void WriteData(char *dest, datatype data) {
    memcpy(dest, &data, sizeof(datatype));
}


void Transmitter::processIdle() {

    TrxSignalSp sig = popSignal();
    if (sig == NullSp)
    {//没有任何需要处理的消息，则定时发送状态包
        return;
    }

    switch (sig->name) {
        case SET_PARAM: {//配置参数
            _param = ParseData<DeviceParam>(sig->data);
            applyCurrentParam();
            break;
        }
        case SET_SOURCE: {//配置发送信号类型
            _sd = ParseData<SourceDesc>(sig->data);
//            LOG(INFO) << _sd.description;
            break;
        }
        case START_SENDING: {
            LOG(INFO) << "enter sending state";
            enterSendingState(); //进入到发送状态
            break;
        }
    }
}




void Transmitter::processSending() {
    uhd::tx_metadata_t md; //时间标记
    //复位设备时间
    _dev->set_time_now(uhd::time_spec_t(0.0));
    _stop_sending_called = false;

    md.has_time_spec = false;
    md.end_of_burst = false;

    //创建发送缓冲
    _samples_per_unit = _tx_stream->get_max_num_samps(); //确定每个包数据的大小
    std::vector<samp_type> buff(_samples_per_unit);

    switch (_sd.type) {
        //文件发送
        case Transmitter::SourceDesc::FILE: {
            std::vector<std::vector<std::complex<short> > > big_buffer;
            std::ifstream infile(_sd.description, std::ios::binary);
            if (!infile.good()) {
                LOG(INFO) << _sd.description<<"文件不可用请检查,没有找到文件";
                break;
            }

            //提取文件IQ数据
            while (infile.good() && !infile.eof()) {
                infile.read((char *) buff.data(), buff.size() * sizeof(samp_type));
                big_buffer.push_back(buff);
            }
            infile.close();

            size_t segment_num = big_buffer.size();
            size_t current_index = 0;

            isOpen = true;

            while (not _stop_sending_called)
            {
                md.end_of_burst = false;

                auto &segment = big_buffer.at(current_index);

                _tx_stream->send(segment.data(), segment.size(), md);

                current_index++;
                if (current_index >= segment_num) current_index = 0;
            }

            break;
        }

        default:
            LOG(INFO) << "Unexpected Source";
            break;


    }


    //发送结束标志
    md.end_of_burst = true;
    isOpen = false;
    _tx_stream->send("", 0, md);

    LOG(INFO) << "Stoped";

    enterIdleState();
}





void Transmitter::processError() {
    // ERROR下什么都不干就睡1s
    boost::this_thread::sleep(boost::posix_time::seconds(1));
}


void Transmitter::enterIdleState() {
    LOG(INFO) << "Transmitter Enter IDLE STATE";
    _state = IDLE;
}

void Transmitter::enterSendingState() {
    LOG(INFO) << "Transmitter Enter SENDING STATE";
    _state = SENDING;
}


void Transmitter::enterErrorState() {
    LOG(INFO) << "Transmitter Enter ERROR STATE";
    _state = ERROR;
}


void Transmitter::setParam(const DeviceParam param) {
    addSignal(SET_PARAM, param);
}


void Transmitter::setSource(SourceDesc sd) {
    addSignal(SET_SOURCE, sd);
}


void Transmitter::stop() {
    _poolLocker.lock();
    _sigPool.clear();
    _poolLocker.unlock();

    _stop_sending_called = true;
    isOpen = false;
}


void Transmitter::start() {
    addSignal(START_SENDING, 0);
}


TrxSignalSp Transmitter::popSignal() {
    TrxSignalSp sig;
    _poolLocker.lock();
    if (_sigPool.size() > 0)
    {
        sig = _sigPool.front();
        _sigPool.pop_front();
    }
    else
    {
        sig = NullSp;
    }
    _poolLocker.unlock();
    return sig;
}



void Transmitter::setTxGain(const double gain) {
    _dev->set_tx_gain(gain);
}


void Transmitter::setTxRate(const double rate) {
    _dev->set_tx_rate(rate);
}


void Transmitter::applyCurrentParam() {
    setFreqNow(_param.txFreq);
    setTxRate(_param.txRate);
    setTxGain(_param.txGain);
}


void Transmitter::setFreqNow(double freq) {
    _dev->set_tx_freq(uhd::tune_request_t(freq));
}


double Transmitter::getFreqNow() {
    return _dev->get_tx_freq();
}
double Transmitter::getRateNow() {
    return _dev->get_tx_rate();
}
double Transmitter::getGainNow() {
    return _dev->get_tx_gain();
}

