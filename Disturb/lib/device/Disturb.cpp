
#include "device/Disturb.h"


void Disturb::Start() {
    _id = "disturb";    //设置控制用的标识号，ID

    EasyConnectIface::Config config;//配置参数
    config.id = _id;
    config.udp_port = 8773; //接收信息的端口号
    _trans = EasyConnectIface::make(config, std::bind(&Disturb::on_msg, this,
                                                      std::placeholders::_1,
                                                      std::placeholders::_2,
                                                      std::placeholders::_3));
    _make = true;msleep(500);

    _dev.SetParam(10e6, 838.5e6, 70);   //设备配置默认参数
    _dev.SetSignalType(COMB2);                  //配置默认发送类型

    _heart_thread = boost::make_shared<boost::thread>(&Disturb::SendHeart, this);    //发送心跳开始

   // _track_thread = boost::make_shared<boost::thread>(&Disturb::ModeTrack, this);   //跟踪干扰跳频专用的线程
    //_ai_thread    = boost::make_shared<boost::thread>(&Disturb::ModeAI, this);      //智能干扰的决策线程
}

/*收到消息就回调此函数*/
void Disturb::on_msg(const std::string &id, void *data, size_t size) {
    if (!_make) {
        std::string err = "Program not initialized completed";
        LOG(INFO) << err;
        return;
    }
    //LOG(INFO)<<"msg++";
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

    if (msg_recv->id() == nullptr) return;

    auto src = msg_recv->id()->str();

    switch (msg_recv->any_type()) {
        case EasyConnect::DataAny_NONE:
            break;
        case EasyConnect::DataAny_SpectrumData:
            break;
        case EasyConnect::DataAny_TestData:
            break;
        case EasyConnect::DataAny_CommandFrame: {   //收到指令帧
            auto cmd = (EasyConnect::CommandFrame *) msg_recv->any();
            HandleCommand(src, cmd);
            break;
        }
    }
}


/*处理命令指令帧*/
void Disturb::HandleCommand(const std::string &id, EasyConnect::CommandFrame *cmd) {

    if (cmd->is_ack()) return;

    switch (cmd->cmd()) {
        case EasyConnect::CommandType_Stop: {
            _dev.Stop();
            break;
        }
        case EasyConnect::CommandType_None:
            break;
        case EasyConnect::CommandType_Start: {
            _dev.Start();
            break;
        }
        case EasyConnect::CommandType_SetParam: {
            //LOG(INFO)<<"current Jamming mode is ";

            auto params = (EasyConnect::Disturb *) cmd->data();

            ///处理不同mode下的发送干扰的决策
            HandleMode(params->mode(), params->channel());
            LOG(INFO) << "current Jamming mode is " << params->mode();

            //send ack
            flatbuffers::FlatBufferBuilder builder;
            auto ack = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_SetParam, true);
            builder.Finish(ack);
            auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, ack.Union(),
                                                  builder.CreateString(_id));
            builder.Finish(msg);
            _trans->send_to(id, builder.GetBufferPointer(), builder.GetSize());
            LOG(INFO) << "send set param ack to " << id;
            break;
        }
        case EasyConnect::CommandType_Hop: {
            //处理由通信需要跳频的决策跳频频点，最终可以在获取到跳频决策后，加上一个时间，达到预测性的跟踪干扰

            auto hop = (EasyConnect::HopFrame *) cmd->data();

            if(_now_mode != TRACK) return;    //如果当前使用的模式不是跟踪干扰决策，那么就没必要继续了

            //通信设备的跳频频点，实际上通信设备并不会马上跳频，这个频率只是下个跳频的频点而已
            auto hop_freq = hop->hop_freq();
            int time_out  = 500;    //延时变量，表示接收到该频点500ms后改变本设备的频率（信道）,,,,这个时间一定要比跳频决策时间短,鑫哥要求跳频决策是1s跳一次
            LOG(INFO)<<"Receive CommandType_Hop and current hop_freq is: "<<hop_freq;
            auto millsec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count(); //获取当前系统时间，单位ms


            _track_loker.lock();                          //操作该列表的时候加锁，防止同时操作
            if(_freq_list.size() >= 10){
                _freq_list.pop_front();
            }
            _freq_list.push_back(hop_freq);               //将该跳频频点添加到跳频的列表里边，在其他线程逐一取出来，然后“跟踪”跳频
            auto size = _freq_list.size();
            LOG(INFO)<<"current size is: "<<size<<"\nlast freq is "<<_freq_list[size-1];
            _time_list.push_back(millsec+time_out);     //加time_out是让该跳频频点在time_out ms后跳频
            _track_loker.unlock();                        //操作完了就解锁

            break;
        }
        case EasyConnect::CommandType_Throughput: {
            //提取通信设备当前吞吐量
            if(cmd->data_type() != EasyConnect::CommandData_ThroughputFrame)
                return;

            if(_now_mode != AI) return;    //如果当前使用的模式不是跟踪干扰决策，那么就没必要继续了


            auto nck = cmd->data_as_ThroughputFrame()->nckNum();                    //获得通信设备上一时刻的丢包数
            auto ack = cmd->data_as_ThroughputFrame()->ackNum();                    //获得通信设备上一时刻的吞吐量
            double packet_loss_probability = (double)nck/(double)(nck+ack);         //丢包率
           // LOG(INFO)<<"吞吐量："<<ack<<"  丢包数："<<nck<<"  总体丢包率："<<packet_loss_probability;

            _ai_loker.lock();
            if(_nck_list.size()>1)          //观察3次就够了，能够大概得到一个平均变化
            {
                _nck_list.pop_front();
            }
            _nck_list.push_back(packet_loss_probability);
            _ai_loker.unlock();
           // LOG(INFO)<<"_nck_list size is "<<_nck_list.size();

            break;
        }
        default:
            break;
    }
}
/*
 * 处理不同mode下的发送决策
 * */
void Disturb::HandleMode(int mode, int channel) {

    _now_mode = mode;           //从这里获取当前的模式
    LOG(INFO)<<"current Jamming MODE is "<<_now_mode;

    switch (mode) {
        case BLOCK: {   //固定信道干扰模式
            if(_ai_running){
                _ai_running = false;
                if (_ai_thread && _ai_thread->joinable()) {
                    _ai_thread->join();
                }
            }
            if(_track_running){
                _track_running = false;
                if (_track_thread && _track_thread->joinable()) {
                    _track_thread->join();
                }
            }
            _dev.SetSignalType(SIG_TYPE(channel + 10));
            break;
        }
        case TRACK: {
            //TODO 开启跟踪干扰决策
            if(_ai_running){
                _ai_running = false;
                if (_ai_thread && _ai_thread->joinable()) {
                    _ai_thread->join();
                }
            }
            if(_track_running){
                break;
                }
            _track_running = true;
            _track_thread = boost::make_shared<boost::thread>(&Disturb::ModeTrack, this);   //跟踪干扰跳频专用的线程
            break;
        }
        case AI: {
            //TODO 开启智能干扰决策
            LOG(INFO)<<"AI MODE";
            if(_track_running){
                _track_running = false;
                if (_track_thread && _track_thread->joinable()) {
                    _track_thread->join();
                }
            }
            if(_ai_running){
                break;
            }
            _ai_running = true;
            _ai_thread = boost::make_shared<boost::thread>(&Disturb::ModeAI, this);   //跟踪干扰跳频专用的线程
            break;
        }
        case SWEEP_COMB: {
            //TODO 开启动态干扰决策
            LOG(INFO)<<"SWEEP_COMB MODE";
            if(_track_running){
                _track_running = false;
                if (_track_thread && _track_thread->joinable()) {
                    _track_thread->join();
                }
            }
            if(_ai_running){
                break;
            }
            _ai_running = true;
            _ai_thread = boost::make_shared<boost::thread>(&Disturb::ModeComb, this);   //跟踪干扰跳频专用的线程
            break;
        }
        case COMB_1: {
            if(_ai_running){
                _ai_running = false;
                if (_ai_thread && _ai_thread->joinable()) {
                    _ai_thread->join();
                }
            }
            if(_track_running){
                _track_running = false;
                if (_track_thread && _track_thread->joinable()) {
                    _track_thread->join();
                }
            }
            _dev.SetSignalType(SIG_TYPE(COMB1));
            break;
        }
        case COMB_2: {   //固定信道干扰模式
            if(_ai_running){
                _ai_running = false;
                if (_ai_thread && _ai_thread->joinable()) {
                    _ai_thread->join();
                }
            }
            if(_track_running){
                _track_running = false;
                if (_track_thread && _track_thread->joinable()) {
                    _track_thread->join();
                }
            }
            _dev.SetSignalType(SIG_TYPE(COMB2));
            break;
        }
        case SWEEP_1: {   //固定信道干扰模式
            if(_ai_running){
                _ai_running = false;
                if (_ai_thread && _ai_thread->joinable()) {
                    _ai_thread->join();
                }
            }
            if(_track_running){
                _track_running = false;
                if (_track_thread && _track_thread->joinable()) {
                    _track_thread->join();
                }
            }
            _dev.SetSignalType(SIG_TYPE(SWEEP_ONE));
            break;
        }
        case SWEEP_2: {   //固定信道干扰模式
            if(_ai_running){
                _ai_running = false;
                if (_ai_thread && _ai_thread->joinable()) {
                    _ai_thread->join();
                }
            }
            if(_track_running){
                _track_running = false;
                if (_track_thread && _track_thread->joinable()) {
                    _track_thread->join();
                }
            }
            _dev.SetSignalType(SIG_TYPE(SWEEP_DOUBLE));
            break;
        }
//        case SWEEP_3: {   //固定信道干扰模式
//            if(_ai_running){
//                _ai_running = false;
//                if (_ai_thread && _ai_thread->joinable()) {
//                    _ai_thread->join();
//                }
//            }
//            if(_track_running){
//                _track_running = false;
//                if (_track_thread && _track_thread->joinable()) {
//                    _track_thread->join();
//                }
//            }
//            _dev.SetSignalType(SIG_TYPE(SWEEP_103_01));
//            break;
//        }
        default:
            break;
    }
}

//跟踪干扰决策
void Disturb::ModeTrack()
{
    LOG(INFO)<<"TRACK MODE";
    while (_track_running)
    {
        if(!_dev.IsOpne()) continue;        //如果没有在发送状态，那么就没必要继续循环了，跳过本次循环
        //LOG(INFO)<<"IsOPne";
        if(_now_mode != TRACK) continue;    //如果当前使用的模式不是跟踪干扰决策，那么就没必要继续循环了，跳过本次循环
        //LOG(INFO)<<"TRACK";
        auto millsec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count(); //获取当前系统时间，单位ms

        if (_freq_list.size() >= 1) { //当频率列表中有跳频数据时才发送

            _track_loker.lock();                //操作该列表的时候加锁，防止同时操作

            auto freq = _freq_list.front();     //提取跳频列表的频率
            _freq_list.pop_front();             //提取完就吧头部的丢弃，因为已经使用了

            auto time_out = _time_list.front(); //提取时间列表的时间
            _time_list.pop_front();             //提取完就吧头部的丢弃，因为已经使用了

            _track_loker.unlock();              //操作完了就解锁

            auto time = time_out - millsec;     //设定好了time_out跳频，但是已经超过了当前时刻millsec，超时了就没意义了
            LOG(INFO) << "Time is " << time;
            LOG(INFO) << "freq is " << freq;
            //if(time > 0)
            //{
            msleep(400);                   //睡眠距离设定好的时间
            _dev.SetSignalType(Freq_to_Mode(freq)); //根据不同频率选择不同信道发送
            //}
        }
    }
}

//智能干扰决策
void Disturb::ModeAI()
{
    LOG(INFO)<<"AI MODE";
    SIG_TYPE last_mode = COMB1;
    while (_ai_running)
    {
        auto start = std::chrono::steady_clock::now();

        if(!_dev.IsOpne()) continue;   //如果没有在发送状态，那么就没必要继续循环了，跳过本次循环
       // LOG(INFO)<<"IsOPne";
        if(_now_mode != AI) return;    //如果当前使用的模式不是智能干扰决策，那么就没必要继续循环了，跳过本次循环

        double avr = 0.0;

        _ai_loker.lock();                //操作该列表的时候加锁，防止同时操作
        for (int i = 0; i < _nck_list.size(); ++i) {
            avr += _nck_list[i]/_nck_list.size();   //得到平均值
        }
        _ai_loker.unlock();                //操作该列表的时候加锁，防止同时操作

//        LOG(INFO)<<"avr is "<<avr;

        if(avr < 0.6)               //规定一个值  50%，当丢包率低于这个值就继续“智能”干扰
        {
            if(last_mode == COMB1){
                last_mode = COMB2;
            }
            else{
                last_mode = COMB1;
            }

            //_dev.SetSignalType((SIG_TYPE)randm(20,21));
        }
        _dev.SetSignalType(last_mode);

        msleep(300);        //500ms做一次改变
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::micro> elapsed = end - start; // std::micro 表示以微秒为时间单位
        std::cout<< "time: "  << elapsed.count() << "us" << std::endl;
    }
}

void Disturb::ModeComb() //适用于各类动态切换
{
    LOG(INFO)<<"SWEEPCOMB MODE";
    SIG_TYPE last_mode = COMB1;
    uint8_t  duration = 2500;
    while (_ai_running)
    {
        int mode_index = rand()%4;  //可以添加任意中模式切换
        switch (mode_index){
            case 0:
                last_mode = COMB1;
                duration = 2000;
                break;
            case 1:
                last_mode = COMB2;
                duration = 2000;
                break;
            case 2:
                last_mode = SWEEP_ONE;
                duration = 2000;
                break;
            case 3:
                last_mode = SWEEP_DOUBLE;
                duration = 2000;
                break;
        }

        _dev.SetSignalType(last_mode);

        msleep(duration);        //不同模式对应不同延时
    }
}

SIG_TYPE Disturb::Freq_to_Mode(double f)
{
    double now_f = 838.5e6;

    if(f>=now_f-4.5e6&&f<now_f-3.5e6)   return CHANNEL_1;
    if(f>=now_f-3.5e6&&f<now_f-2.5e6)   return CHANNEL_2;
    if(f>=now_f-2.5e6&&f<now_f-1.5e6)   return CHANNEL_3;
    if(f>=now_f-1.5e6&&f<now_f-0.5e6)   return CHANNEL_4;
    if(f>=now_f-0.5e6&&f<now_f+0.5e6)   return CHANNEL_5;
    if(f>=now_f+0.5e6&&f<now_f+1.5e6)   return CHANNEL_6;
    if(f>=now_f+1.5e6&&f<now_f+2.5e6)   return CHANNEL_7;
    if(f>=now_f+2.5e6&&f<now_f+3.5e6)   return CHANNEL_8;
    //if(f>=now_f+3.5e6&&f<now_f+4.5e6)   return CHANNEL_9;

    return CHANNEL_5;
}

/* 处理收到的频点数据
 * 主要用来做智能干扰决策
 * */
//void Disturb::HandleSpectrum(const std::string &id, EasyConnect::SpectrumData *spec_recv) {
//    LOG(INFO) << "cent_freq:" << spec_recv->cent_freq();
//    LOG(INFO) << spec_recv->freq_index();
//    LOG(INFO) << spec_recv->fft_size();
//    LOG(INFO) << spec_recv->rx_rate();
//
//    for (uint32_t j = 0; j < spec_recv->data()->size(); ++j) {
//        //        LOG(INFO) << spec_recv->data()->data()[j];
//    }
//}


void Disturb::SendHeart() {
    while (_loop) {
        flatbuffers::FlatBufferBuilder builder;
        auto heart_f = EasyConnect::CreateStateFrame(builder, _dev.NowFreq(), 0);
        builder.Finish(heart_f);

        auto command_frame = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_STATE, false,
                                                             EasyConnect::CommandData_StateFrame, heart_f.Union());
        builder.Finish(command_frame);

        auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, command_frame.Union(),
                                              builder.CreateString(_id));
        builder.Finish(msg);

        _trans->send_to("GUI", builder.GetBufferPointer(), builder.GetSize());
        msleep(1000);
    }
}