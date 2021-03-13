//
// Created by locate on 4/22/20.
//

#include <Policy.hpp>
#include <glog/logging.h>
#include <fstream>
#include "DFT.hpp"
#include "GNUPlotIface.hpp"
#include "NTPclient.h"
#include <easy_connect_generated.h>
#include "USB.hpp"
#include "random"
//#include "PlayEngine.hpp"
#include "dqn.hpp"
#include "ctime"

#define msleep(N); boost::this_thread::sleep(boost::posix_time::milliseconds(N));

using terjin::TestFrame;
using terjin::MessageFrame;

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


Policy::Policy() {
    _spec_param.rx_freq = 430e6;
    _spec_param.rx_rate = 10e6;
    _spec_param.fft_size = 1024;
    _spec_param.spectrum_L = 200;
    _spec_param.spectrum_sense = 20;
    _spec_param.trans_band = 1000e3;
    _spec_param.guard_time_s = 100e-3;
    _curr_time = 0;
}

Policy::sptr Policy::make() {
    return std::make_shared<Policy>(); //分配一个感知对象并初始化
}

// Handle Message
void Policy::on_msg(const std::string &id, void *data, size_t size) {
    //LOG(INFO)<<"on_msg++ ";
    if (!_make) {
        std::string err = "Program not initialized completed";
        LOG(INFO) << err;
        return;
    }

    const uint8_t *buf = (uint8_t *) data;
    flatbuffers::FlatBufferBuilder builder_out;
    builder_out.PushBytes(buf, size);

    flatbuffers::Verifier verifier(builder_out.GetCurrentBufferPointer(), builder_out.GetSize());

    auto flag = EasyConnect::VerifyMessageBuffer(verifier); //确认帧是否合法

    if (!flag) {
        LOG(WARNING) << "wrong frame";
        return;
    }
    auto msg_recv = EasyConnect::GetMessage(buf);  //获取发送过来帧
    if (msg_recv->id() == nullptr) {
        LOG(ERROR) << "message: " << msg_recv->any_type() << " have no id";
        return;
    }
    auto src = msg_recv->id()->str();

    switch (msg_recv->any_type()) {
        case EasyConnect::DataAny_NONE:
            break;
        case EasyConnect::DataAny_SpectrumData:
        {
            auto spec_recv = (EasyConnect::SpectrumData*)msg_recv->any();
            Handle_Spectrum(src,spec_recv);  //Handle Spectrum and generate Policy
            break;
        }
        case EasyConnect::DataAny_TestData:
            break;
        case EasyConnect::DataAny_FileFrame:
            break;
        case EasyConnect::DataAny_CommandFrame: {
            auto cmd = (EasyConnect::CommandFrame *) msg_recv->any();

            handle_command(src, cmd);
            break;
        }
    }
}

void Policy::Handle_Spectrum(const std::string &id, EasyConnect::SpectrumData *spec_recv) {
    //LOG(INFO) << "cent_freq:" << spec_recv->cent_freq();

    _spec_param.rx_freq = spec_recv->cent_freq();
    _spec_param.rx_rate = spec_recv->rx_rate();
    _spec_param.fft_size = spec_recv->fft_size();

    Spectrum_lw spec_double;
    for(uint16_t i=0;i<spec_recv->data()->size();++i){
        spec_double[i]=(double)spec_recv->data()->data()[i]/10;
//        std::ofstream out1;
//        out1.open("waterfall.txt",std::ios::app);
//        out1<<spec_double[i]<<" ";
//        if(i == 1023){
//            out1<<"\n";
//        }
//        out1.close();
    }

    _poolLocker.lock();
    if (_water.size() >= _spec_param.spectrum_L)
        _water.pop_front();
    _water.push_back(spec_double);
    _poolLocker.unlock();

    //int fre_size = spec_recv->data()->size(); //1024

    //std::vector<double> spec_doub(fre_size,0);
//    for (uint8_t j = 0; j < fre_size; ++j)
//        spec_doub[j] = (double)spec_recv->data()->data()[j];

   // auto spec_ptr = Spectrum::make(spec_doub.data(),fre_size);

    //_poolLocker.lock();
//    if (_waterfall.size() >= _spec_param.spectrum_L)
//        _waterfall.pop_front();
//    _waterfall.push_back(spec_ptr);
    //_poolLocker.unlock();


//
//    _poolLocker.lock();
//    if (_spec.size() >= SPEC_POOL_SIZE)
//        _spec.pop_front();
//    _spec.push_back(spec_recv);
//    _poolLocker.unlock();

//    LOG(INFO) << "cent_freq: " << spec_recv->cent_freq();
//    LOG(INFO) <<"fre_index: "<<spec_recv->freq_index();
//    LOG(INFO) << "fft_size: "<<spec_recv->fft_size();
//    LOG(INFO) << "rx_rate:  "<<spec_recv->rx_rate();

//    for (uint32_t j = 0; j < spec_recv->data()->size(); ++j) {
//        //        LOG(INFO) << spec_recv->data()->data()[j];
//    }
}


void Policy::handle_command(const std::string &id, EasyConnect::CommandFrame *cmd) {
    switch (cmd->cmd()) {
        case EasyConnect::CommandType_Stop: {
            LOG(INFO) << "Nunmber of the Command " << cmd->cmd();
            this->stop_policy(); // stop the policy thread
            //send ack
            flatbuffers::FlatBufferBuilder builder;
            auto ack = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Stop, true);
            builder.Finish(ack);
            auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, ack.Union(),
                                                  builder.CreateString(_dev.Strategy));
            builder.Finish(msg);
            this->_conn->send_to(id, builder.GetBufferPointer(), builder.GetSize());
            break;
        }
        case EasyConnect::CommandType_None:
            break;
        case EasyConnect::CommandType_Start:
        {
            ///播放传输视频
//            if(!_vdieo_running){
//                _vdieo_running = true;
//                _vdieo_thread = std::make_shared<std::thread>(std::bind(&Policy::Play_vdieo, this));
//            }

            break;
        }
        case EasyConnect::CommandType_SENSE: {
            LOG(INFO) << "Sense command.";
            if (_generate_running)
            {
                this->stop_policy();
            }
            _generate_running = true;
            _generate_thread = std::make_shared<std::thread>(std::bind(&Policy::sense_hop, this)); //sense-based hop frequency thread
            // this->start_policy(cmd); //start the policy thread
            //send ack
            flatbuffers::FlatBufferBuilder builder;
            auto ack = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_SENSE, true);
            builder.Finish(ack);
            auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, ack.Union(),
                                                  builder.CreateString(_dev.Strategy));
            builder.Finish(msg);
            this->_conn->send_to(id, builder.GetBufferPointer(), builder.GetSize());
            break;
        }
        case EasyConnect::CommandType_FAST: {
            LOG(INFO) << "Fast command.";
            if (_generate_running)
            {
                this->stop_policy();
            }
            _generate_running = true;
            _generate_thread = std::make_shared<std::thread>(std::bind(&Policy::probab_hop, this)); //Probab-based hop frequency thread
            // this->start_policy(cmd); //start the policy thread
            //send ack
            flatbuffers::FlatBufferBuilder builder;
            auto ack = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_FAST, true);
            builder.Finish(ack);
            auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, ack.Union(),
                                                  builder.CreateString(_dev.Strategy));
            builder.Finish(msg);
            this->_conn->send_to(id, builder.GetBufferPointer(), builder.GetSize());
            break;
        }
        case EasyConnect::CommandType_INTELL: {
            LOG(INFO) << "Intell command.";
            if (_generate_running)
            {
                this->stop_policy();
            }
            _generate_running = true;
            _generate_thread = std::make_shared<std::thread>(std::bind(&Policy::intell_hop, this)); //DQN-based hop frequency thread
            //_generate_thread = std::make_shared<std::thread>(std::bind(&Policy::DQN_hop, this));
            // this->start_policy(cmd); //start the policy thread
            //send ack
            flatbuffers::FlatBufferBuilder builder;
            auto ack = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_INTELL, true);
            builder.Finish(ack);
            auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, ack.Union(),
                                                  builder.CreateString(_dev.Strategy));
            builder.Finish(msg);
            this->_conn->send_to(id, builder.GetBufferPointer(), builder.GetSize());
            break;
        }
        case EasyConnect::CommandType_FIXED: {
            //LOG(INFO)<<"current Jamming mode is ";
            if(id == "GUI"){
                LOG(INFO) << "fixed command";
                if (_generate_running)
                {
                    this->stop_policy();
                }
                auto params = (EasyConnect::Disturb *) cmd->data();

                LOG(INFO)<<"Channel is "<<params->channel();
                double next_hop = 833.5e6 + (params->channel() - 1)*1e6;
                fixed_hop(next_hop);

                //send ack
                flatbuffers::FlatBufferBuilder builder;
                auto ack = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_FIXED, true);
                builder.Finish(ack);
                auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, ack.Union(),
                                                      builder.CreateString(_dev.Strategy));
                builder.Finish(msg);
                this->_conn->send_to(id, builder.GetBufferPointer(), builder.GetSize());
                LOG(INFO) << "send set param ack to " << id;

                //_generate_running = true;
                break;

            }

        }
        case EasyConnect::CommandType_Hop:{
            //LOG(INFO) << "tsp command";
            if(id == "perception") {
                auto hop_params = (EasyConnect::HopFrame *) cmd->data();   ///接收当前的设备时间
                _poolLocker.lock();
                _curr_time =  hop_params->hop_time();
                //LOG(INFO)<<"current time is "<<_curr_time;
                _poolLocker.unlock();
                break;
            }
            else{
                break;
            }

        }
        case EasyConnect::CommandType_Throughput: {
            if(cmd->data_type() != EasyConnect::CommandData_ThroughputFrame)
                return;
            //if(_now_mode != AI) return;    //如果当前使用的模式不是跟踪干扰决策，那么就没必要继续了


            auto nck = cmd->data_as_ThroughputFrame()->nckNum();                    //获得通信设备上一时刻的丢包数
            auto ack = cmd->data_as_ThroughputFrame()->ackNum();                    //获得通信设备上一时刻的吞吐量
            double packet_loss_probability = (double)nck/(double)(nck+ack);         //丢包率
//            LOG(INFO)<<"吞吐量："<<ack<<"  丢包数："<<nck<<"  总体丢包率："<<packet_loss_probability;

            _ai_loker.lock();
            if(_nck_list.size()>1)          //观察3次就够了，能够大概得到一个平均变化
            {
                _nck_list.pop_front();
            }
            _nck_list.push_back(packet_loss_probability);
            _ai_loker.unlock();
            //LOG(INFO)<<"size is "<<_nck_list.size();
            break;
        }
        case EasyConnect::CommandType_SetParam:
            {

//            LOG(INFO)<<"current  PAram  is ";
            auto params = (EasyConnect::Disturb *) cmd->data();

            auto Jamming_Mode = params->mode();


            if (_auto_running)
            {
                _auto_running = false;
                if (_Auto_thread && _Auto_thread->joinable())
                    _Auto_thread->join();
            }
            _auto_running = true;
            _Auto_thread = std::make_shared<std::thread>(std::bind(&Policy::Auto_hop, this, Jamming_Mode));

            break;
        }
//        case EasyConnect::CommandType_Chat:{  ///测试用
//            LOG(INFO)<<"recv Command from "<< id;
//            if (cmd->data_type() != EasyConnect::CommandData_StringFrame) {
//                LOG(ERROR) << "chat data error";
//                break;
//            }
//            auto chat_data = cmd->data_as_StringFrame()->content()->str();
//            if (chat_data.empty()) {
//                LOG(INFO) << "chat data empty";
//                break;
//            }
//            LOG(INFO)<<"recv Command is "<<chat_data;
//            std::vector<std::string> vec;
//            boost::split(vec, chat_data, boost::is_any_of("+"));
//            LOG(INFO)<<"recv Command1 is "<<vec[0];
//            LOG(INFO)<<"recv Command2 is "<<vec[1];
//
//            break;
//        }
        default:
            break;
    }
}

void Policy::start_conn(short port) {
    EasyConnectIface::Config config;
    config.id = "strategy";
    config.udp_port = port; //8773
    //LOG(INFO)<<"8733\n";
    _conn = EasyConnectIface::make(config,
                                   std::bind(&Policy::on_msg, this, std::placeholders::_1, std::placeholders::_2,
                                             std::placeholders::_3));  //msg处理函数

   _make = true;
   _conn->create_broker();

    msleep(500);

    _state_thread = std::make_shared<std::thread>(std::bind(&Policy::state_working, this)); //心跳线程，运行state_working函数
}


//void Policy::start_policy(EasyConnect::CommandFrame *cmd) {  //activate the policy thread
//
//        _generate_running = true;
//        _generate_thread = std::make_shared<std::thread>(std::bind(&Policy::generate_working, this, cmd)); //生成感知线程
//
//}

void Policy::stop_policy() {
    LOG(INFO) << "stop generating anti-jamming policy";
    _generate_running = false;
    if (_generate_thread && _generate_thread->joinable()) {
        _generate_thread->join();
    }
//    _vdieo_running = false;
//    if (_vdieo_thread && _vdieo_thread->joinable()) {
//        _vdieo_thread->join();
//    }

    LOG(INFO) << "stop ok";
}

// generating anti-jamming policy
void Policy::fixed_hop(double freq){
    LOG(INFO) << "Start fixed hop";

//        const int seg_size = (int) (_spec_param.trans_band / _spec_param.rx_rate * _spec_param.fft_size);
//        const int seg_num = _spec_param.fft_size / seg_size;  //决策频点数
//        srand(time(NULL));
//        int channel_index = (rand() % (seg_num - 2))+ 1; // 这里根据输入的值改变频率
//
//        LOG(INFO)<<"\n channel_index is: "<<channel_index;
//
//        double hop_freq = 0;
//
//        hop_freq = (_spec_param.rx_freq - _spec_param.rx_rate / 2.0) +
//                   ((double) channel_index/ (double) seg_num) * _spec_param.rx_rate +
//                   0.5 * _spec_param.trans_band;  //找到中心频率

        //跳到指定的频率给定的频率
        uint64_t timeSpec = _curr_time;

        send_hop_command(timeSpec + time_spec_to_ms(_spec_param.guard_time_s), freq);
}

//int len_sense = 15;
void Policy::sense_hop(){
    LOG(INFO) << "Start sense-based hop";
    while (_generate_running) {

        Spectrum_lw average_spec;
        average_spec.fill(0);
        //LOG(INFO)<<average_spec[120];
        Spectrum_lw max_spec;
        max_spec.fill(0);

        for (int k=0;k <_spec_param.spectrum_sense;k++) {
            Spectrum_lw data = _water[_water.size()-1-k];
            //data.fill(0);
            //const auto &data = _water[_waterfall.size()-1-k]->Data();
            for (int j = 0; j < data.size(); ++j) {
                //average
                average_spec[j] += data[j];

                //max val
                if (data[j] > max_spec[j]) {
                    max_spec[j] = data[j];
                }
            }

        }

        for (int j=0;j<average_spec.size();j++) {
            average_spec[j] /= _spec_param.spectrum_sense;  //20
        }
        const int seg_size = (int) (_spec_param.trans_band / _spec_param.rx_rate * _spec_param.fft_size); //102
        const int seg_num = _spec_param.fft_size / seg_size;  //决策频点数 10
        //LOG(INFO)<<"seg_size is "<<seg_size<<"\n seg_num is"<<seg_num;

       // auto spec_cal_channel = average_spec;

        //std::array<double, seg_num>;
        std::vector<double> simple_spec(seg_num, 0);


        for (int i = 0; i < seg_num; ++i) {
            for (int j = 0; j < seg_size; ++j) {
                simple_spec[i] += average_spec[i * seg_size + j]; //把子频带内点数相加
            }
        }


        //calculate lowest channel
        double hop_freq = 0;
        int channel_index = -1;
        double min_channel = 1e8;
        for (int i = 1; i < seg_num - 1; ++i) {
            if (simple_spec[i] < min_channel) {
                min_channel = simple_spec[i];
                channel_index = i;
                hop_freq = (_spec_param.rx_freq - _spec_param.rx_rate / 2.0) +
                           ((double) channel_index / (double) seg_num) * _spec_param.rx_rate +
                           0.5 * _spec_param.trans_band;  //找到中心频率
            }
        }
        LOG(INFO)<<"next channel index is "<<channel_index;
        //////Tsp how to time sync
        uint64_t timeSpec = _curr_time;

        send_hop_command(timeSpec + time_spec_to_ms(_spec_param.guard_time_s), hop_freq);

        msleep(1500); //延迟1s
    }

}
void Policy::probab_hop(){  //模式三，快速跳频
    while (_generate_running) {
//        clock_t begin,end;
//        begin=clock();
        //LOG(INFO) << "Start probab_hop: "<<_curr_time<<" "<<_water.size();
        const int seg_size = (int) (_spec_param.trans_band / _spec_param.rx_rate * _spec_param.fft_size);
        const int seg_num = _spec_param.fft_size / seg_size;  //决策频点数
        int channel_index = 0;

        std::random_device e;
        std::uniform_real_distribution<double> p(0, 1); //随机数分布对象
        double hop_freq = 0;
        //int channel_index = 0;
        const int half_seg_num = ceil((seg_num - 2) / 2);

        double prob = p(e); //生成0-1随机数
        //LOG(INFO)<<"probab is "<<prob;
        if (prob < 0.5) {
            srand(time(NULL));
            channel_index = 2*((rand() % (half_seg_num)) + 1) - 1;
            //channel_index = (rand() % (half_seg_num)) + 1;
            hop_freq = (_spec_param.rx_freq - _spec_param.rx_rate / 2.0) +
                       ((double) channel_index / (double) seg_num) * _spec_param.rx_rate +
                       0.5 * _spec_param.trans_band;  //找到中心频率
        } else {
            srand(time(NULL));
            channel_index = 2*((rand() % (half_seg_num)) + 1);
            //channel_index = (rand() % (seg_num - half_seg_num - 2)) + half_seg_num + 1;
            hop_freq = (_spec_param.rx_freq - _spec_param.rx_rate / 2.0) +
                       ((double) channel_index / (double) seg_num) * _spec_param.rx_rate +
                       0.5 * _spec_param.trans_band;  //找到中心频率
        }
        LOG(INFO)<<"next channel index is "<<channel_index;
        //////Tsp how to time sync
        uint64_t timeSpec = _curr_time;

        send_hop_command(timeSpec + time_spec_to_ms(_spec_param.guard_time_s), hop_freq);

//        end = clock();
//        float time = float(end-begin)/float( CLOCKS_PER_SEC);
//        LOG(INFO)<<"time is "<<time;

        //msleep(195); //延迟0.5s
        msleep(300); //延迟0.5s
    }
}

/*********乱序DQN**********/
//void Policy::intell_hop(){ //智能跳频
//    LOG(INFO) << "Start intell_hop";
//
//    const int seg_size = (int) (_spec_param.trans_band / _spec_param.rx_rate * _spec_param.fft_size);
//    const int seg_num = _spec_param.fft_size / seg_size;  //决策频点数 10
//    const int num_actions = seg_num - 2; //只需要8个
//
//    int channel_index = 1;
//    double reward = 0;
//
//    /*********** Q-learning算法***************/
//
//    /*********** DQN算法***************/
//    caffe::Caffe::set_mode(caffe::Caffe::GPU); //caffe工作模式
//    dqn::ActionVect legal_actions;
//    for(int i=0;i<num_actions;i++) legal_actions.push_back(i); //0,1,2,3,4,5,6,7
//
//    dqn::DQN Dqn(legal_actions, "dqn_solver.prototxt", Memory_Size, Gamma, true);
//    Dqn.Initialize(); //初始化网络，这块需要根据送过来的频谱数据修改一下；
//    //remove("record.txt");
//    int size_water = _water.size();
//    if (size_water < 200)
//    {
//        LOG(INFO)<<"Wait time to get more spectrum data, water_size is "<<size_water;
//        int wait_num = (200-size_water + 5)*200;
//        msleep(wait_num);
//    }
//        /**********构建频谱瀑布图，这里可以对频谱数据进行重构处理************/
//        std::deque<waterfall_data> waterfall_current; //1024频点变成200
//        for (int k = 0; k < _water.size(); k++) {
//            waterfall_data flag_200;
//            flag_200.fill(0);
//            //int tab_fre = 0;
//            int tab_two = 0;
//            for (int i = 0; i < flag_200.size(); i++) {
//                int tab_num = 0;
//                while (tab_num < 5) {
//                    flag_200[i] += _water[k][i * 5 + tab_num + tab_two];
//                    tab_num++;
//                }
//                //tab_fre++;
//                flag_200[i] = flag_200[i] / 5;  //求均值
//                if (i % 20 == 0)tab_two += 2;
//            }
//            waterfall_current.push_back(flag_200);
//        }
//
//        std::deque<waterfall_data> waterfall_next = waterfall_current;
//        auto current_frame = std::make_shared<dqn::FrameData>();
//        for (auto i = 0; i < dqn::kFrameHeight; ++i) {
//            for (auto j = 0; j < dqn::kFrameWidth; ++j) {
//                (*current_frame)[i * dqn::kFrameWidth + j] = waterfall_current[i][j];
//            }
//        }
//
//        dqn::Action LastAction = 1; //at
//        dqn::Action LastAction_last = 1; //at
//        int episode = 0;
//
//        /**********DQN抗干扰线程开启************/
//        while (_generate_running) { //该线程一直循环输出跳频决策
////            clock_t begin, end;
////            begin = clock();
//            std::cout << "episode: " << episode << std::endl;
//
//            const auto epsilon = CalculateEpsilon(Dqn.current_iteration());
////            const auto epsilon =0;
//
//            bool update = true;
//            /**********获取当前感知的瀑布图 s_t+1, 通过当前传过来的感知数据进行构造************/
//
//            //waterfall_next.pop_front();
//            for (int k = 0; k < _water.size(); k++) {
//                waterfall_data flag_200;
//                flag_200.fill(0);
//                int tab_two = 0;
//                for (int i = 0; i < flag_200.size(); i++) {
//                    int tab_num = 0;
//                    while (tab_num < 5) {
//                        flag_200[i] += _water[k][i * 5 + tab_num + tab_two];
////                        LOG(INFO)<<" test1 is: "<<_water[k][i * 5 + tab_num + tab_two];
//                        tab_num++;
//                    }
//                    flag_200[i] = flag_200[i] / 5;  //求均值
////                    LOG(INFO)<<" test is: "<<flag_200[i];
//                    if (i % 20 == 0)tab_two += 2;
//                }
//                waterfall_next.pop_front();
//                waterfall_next.push_back(flag_200);
//            }
//
//            auto next_frame = std::make_shared<dqn::FrameData>();
//            for (auto i = 0; i < dqn::kFrameHeight; ++i) {
//                for (auto j = 0; j < dqn::kFrameWidth; ++j) {
//                    (*next_frame)[i * dqn::kFrameWidth + j] = waterfall_next[i][j];
//                }
//            }
//
//            dqn::InputFrames input_frames = next_frame; //动态指针指向 1*40000的数组
//            /**********根据当前输入确定策略************/
//            const auto action = Dqn.SelectAction(input_frames, epsilon); //输出这个 e-greedy贪婪选择跳频动作；
//            channel_index = action + 1;
//
//            /**********执行策略 计算回报  根据前后干扰与决策关系来判断（这里回报值获取和计算可修改）************/
//            /*****通过频谱数据推算出干扰频点来计算汇报值*****/
////            Spectrum_lw average_spec;
////            average_spec.fill(0);
////
////            const Spectrum_lw data = _water[_spec_param.spectrum_L - 1];
////
////            for (size_t j = 0; j < data.size(); ++j) {
////                average_spec[j] += data[j];
////            }
////
////            std::vector<double> simple_spec(seg_num, 0);
////            for (int i = 0; i < seg_num; ++i) {
////                for (int j = 0; j < seg_size; ++j) {
////                    simple_spec[i] += average_spec[i * seg_size + j]; //把子频带内点数相加
////                }
////            }
////
////            int jamming_index = 0;
////            double max_channel = simple_spec[0];  //设置门限值来获得干扰的频率
////            for (int i = 1; i < seg_num; ++i) {
////                if (simple_spec[i] >= max_channel) {
////                    jamming_index = i;
////                    max_channel = simple_spec[i];
////                }
////            }
//            //            if (LastAction == jamming_index + 1)
////                reward = 0;
////            else
////                reward = 10;
////            if (LastAction != channel_index) reward = 0.8 * reward; else reward = 1 * reward;
////            LastAction = channel_index;
//
//            /*****通过发送过来的ack数来计算回报值*****/
//            double av_loss = 0.0;
//           // LOG(INFO)<<"nck list size is "<<_nck_list.size();
//            if(_nck_list.size() > 0){
//                _ai_loker.lock();
//                for (int i = 0; i < _nck_list.size(); ++i) {
//                    av_loss += _nck_list[i]/_nck_list.size();
//                }
//                _ai_loker.unlock();
//            }
//            std::cout << "current avr=" << av_loss << std::endl;
//
//            if(av_loss >= 0.3)
//                reward = 0;
//            else
//                reward = 1;
//            if (LastAction != LastAction_last) reward = 0.8 * reward; else reward = 1 * reward;
//
//
//            std::cout << "current reward=" << reward << std::endl<< std::endl;
//
//            /**********训练阶段，则需要update网络************/
//            if (update) {
//                const auto transition =
//                        dqn::Transition(current_frame, LastAction-1, reward, input_frames);
//                Dqn.AddTransition(transition);
//                // If the size of replay memory is enough, update DQN
//                if (Dqn.memory_size() > Memory_TH) Dqn.Update();
//            }
//            LastAction_last = LastAction;
//            LastAction = channel_index;
//            current_frame = input_frames;
//
//            episode += 1;
//
//            /**********发送跳频指令************/
//            double hop_freq = (_spec_param.rx_freq - _spec_param.rx_rate / 2.0) +
//                              ((double) channel_index / (double) seg_num) * _spec_param.rx_rate +
//                              0.5 * _spec_param.trans_band;  //找到中心频率
//
//            uint64_t timeSpec = _curr_time;
//
//            send_hop_command(timeSpec + time_spec_to_ms(_spec_param.guard_time_s), hop_freq);
//
////            end = clock();
////            float time = float(end - begin) / float(CLOCKS_PER_SEC);
////            LOG(INFO) << "time is " << time;
//
//            msleep(300);
//        }
////    }
//}


/******快速感知策略******/
//void Policy::intell_hop() {
//
//    while (_generate_running) {
//
//        Spectrum_lw average_spec;
//        average_spec.fill(0);
//        //LOG(INFO)<<average_spec[120];
//        Spectrum_lw max_spec;
//        max_spec.fill(0);
//
//        for (int k = 0; k < 1; k++) {
//            Spectrum_lw data = _water[_water.size() - 1 - k];
//            //data.fill(0);
//            //const auto &data = _water[_waterfall.size()-1-k]->Data();
//            for (int j = 0; j < data.size(); ++j) {
//                //average
//                average_spec[j] += data[j];
//
//                //max val
//                if (data[j] > max_spec[j]) {
//                    max_spec[j] = data[j];
//                }
//            }
//
//        }
//
//        for (int j = 0; j < average_spec.size(); j++) {
//            average_spec[j] /= _spec_param.spectrum_sense;  //20
//        }
//        const int seg_size = (int) (_spec_param.trans_band / _spec_param.rx_rate * _spec_param.fft_size); //102
//        const int seg_num = _spec_param.fft_size / seg_size;  //决策频点数 10
//        //LOG(INFO)<<"seg_size is "<<seg_size<<"\n seg_num is"<<seg_num;
//
//        // auto spec_cal_channel = average_spec;
//
//        //std::array<double, seg_num>;
//        std::vector<double> simple_spec(seg_num, 0);
//
//
//        for (int i = 0; i < seg_num; ++i) {
//            for (int j = 0; j < seg_size; ++j) {
//                simple_spec[i] += average_spec[i * seg_size + j]; //把子频带内点数相加
//            }
//        }
//
//
//        //calculate lowest channel
//        double hop_freq = 0;
//        int channel_index = -1;
//        double min_channel = 1e8;
//        for (int i = 1; i < seg_num - 1; ++i) {
//            if (simple_spec[i] < min_channel) {
//                min_channel = simple_spec[i];
//                channel_index = i;
//                hop_freq = (_spec_param.rx_freq - _spec_param.rx_rate / 2.0) +
//                           ((double) channel_index / (double) seg_num) * _spec_param.rx_rate +
//                           0.5 * _spec_param.trans_band;  //找到中心频率
//            }
//        }
//        LOG(INFO) << "next channel index is " << channel_index;
//        //////Tsp how to time sync
//        uint64_t timeSpec = _curr_time;
//
//        send_hop_command(timeSpec + time_spec_to_ms(_spec_param.guard_time_s), hop_freq);
//
//        msleep(300); //延迟1s
//    }
//}

/******有先验信息的智能跳频******/
//void Policy::intell_hop(){
//    LOG(INFO) << "Start intell_hop";
//    const int seg_size = (int) (_spec_param.trans_band / _spec_param.rx_rate * _spec_param.fft_size);
//    const int seg_num = _spec_param.fft_size / seg_size;  //决策频点数
//    int channel_index = 1;
//    int last_channel = 2;
//    double reward = 0;
//    const int half_seg_num = ceil((seg_num - 2) / 2);
//
//    while (_generate_running) {
//
//        double av_loss = 0.0;
//
//        if(_nck_list.size() > 0){
//            _ai_loker.lock();
//            for (int i = 0; i < _nck_list.size(); ++i) {
//                av_loss += _nck_list[i]/_nck_list.size();
//            }
//            _ai_loker.unlock();
//        }
//        std::cout << "current avr=" << av_loss << std::endl;
//
//        if(av_loss >= 0.3)
//            reward = 0;
//        else
//            reward = 1;
//        if (channel_index != last_channel) reward = 0.8 * reward;else reward = 1 * reward;
//
//        std::cout << "current reward=" << reward << std::endl;
//
//
//        if(last_channel%2 == 0){
//            srand(time(NULL));
//            channel_index = 2 * ((rand() % (half_seg_num)) + 1) - 1;
//        }else{
//            srand(time(NULL));
//            channel_index = 2 * ((rand() % (half_seg_num)) + 1);
//        }
//
//        last_channel = channel_index;
//        double hop_freq = 0;
//        hop_freq = (_spec_param.rx_freq - _spec_param.rx_rate / 2.0) +
//                   ((double) channel_index / (double) seg_num) * _spec_param.rx_rate +
//                   0.5 * _spec_param.trans_band;  //找到中心频率
//
//        std::cout << "next channel index is " << channel_index << std::endl <<std::endl;
//
//        uint64_t timeSpec = _curr_time;
//
//        send_hop_command(timeSpec + time_spec_to_ms(_spec_param.guard_time_s), hop_freq);
//
//        std::this_thread::sleep_for(std::chrono::milliseconds(300));
//    }
//}


/*************DQN智能算法*************/
void Policy::intell_hop(){ //智能跳频
    LOG(INFO) << "Start intell_hop";

    const int seg_size = (int) (_spec_param.trans_band / _spec_param.rx_rate * _spec_param.fft_size);
    const int seg_num = _spec_param.fft_size / seg_size;  //决策频点数 10
    const int num_actions = seg_num - 2; //只需要8个

    int channel_index = 1;
    double reward = 0;

    /*********** Q-learning算法***************/

    /*********** DQN算法***************/
    caffe::Caffe::set_mode(caffe::Caffe::GPU); //caffe工作模式
    dqn::ActionVect legal_actions;
    for(int i=0;i<num_actions;i++) legal_actions.push_back(i); //0,1,2,3,4,5,6,7

    dqn::DQN Dqn(legal_actions, "dqn_solver.prototxt", Memory_Size, Gamma, true);
    Dqn.Initialize(); //初始化网络，这块需要根据送过来的频谱数据修改一下；
    //remove("record.txt");
    int size_water = _water.size();
    if (size_water < 200)
    {
        LOG(INFO)<<"Wait time to get more spectrum data, water_size is "<<size_water;
        int wait_num = (200-size_water + 5)*305;
        msleep(wait_num);
    }
    /**********构建频谱瀑布图，这里可以对频谱数据进行重构处理************/
    std::deque<waterfall_data> waterfall_current; //1024频点变成200
    for (int k = 0; k < _water.size(); k++) {
        waterfall_data flag_200;
        flag_200.fill(0);
        //int tab_fre = 0;
        int tab_two = 0;
        for (int i = 0; i < flag_200.size(); i++) {
            int tab_num = 0;
            while (tab_num < 5) {
                flag_200[i] += _water[k][i * 5 + tab_num + tab_two];
                tab_num++;
            }
            //tab_fre++;
            flag_200[i] = flag_200[i] / 5;  //求均值
            if (i % 20 == 0)tab_two += 2;
        }
        waterfall_current.push_back(flag_200);
    }

    std::deque<waterfall_data> waterfall_next = waterfall_current;
    auto current_frame = std::make_shared<dqn::FrameData>();
    for (auto i = 0; i < dqn::kFrameHeight; ++i) {
        for (auto j = 0; j < dqn::kFrameWidth; ++j) {
            (*current_frame)[i * dqn::kFrameWidth + j] = waterfall_current[i][j];
        }
    }

    dqn::Action LastAction = 1; //at
    //dqn::Action LastAction_last = 1; //at
    int episode = 0;

    /**********DQN抗干扰线程开启************/
    while (_generate_running) { //该线程一直循环输出跳频决策
//        auto start = std::chrono::steady_clock::now();

        std::cout << "episode: " << episode << std::endl;

        const auto epsilon = CalculateEpsilon(Dqn.current_iteration());
//            const auto epsilon =0;
        std::cout << " epsilon:" << epsilon << std::endl;

        std::random_device e;
        std::uniform_real_distribution<double> p(0, 1); //随机数分布对象
        double prob = p(e); //生成0-1随机数
        prob = 1;  ///一直学习训练

        if(prob < epsilon){
            bool update = true;

            dqn::InputFrames input_frames = current_frame; //动态指针指向 1*40000的数组
            /**********根据当前输入确定策略************/
            const auto action = Dqn.SelectAction(input_frames, epsilon); //输出这个 e-greedy贪婪选择跳频动作；
            channel_index = action + 1;

            /**********发送跳频指令************/
            double hop_freq = (_spec_param.rx_freq - _spec_param.rx_rate / 2.0) +
                              ((double) channel_index / (double) seg_num) * _spec_param.rx_rate +
                              0.5 * _spec_param.trans_band;  //找到中心频率

            uint64_t timeSpec = _curr_time;

            send_hop_command(timeSpec + time_spec_to_ms(_spec_param.guard_time_s), hop_freq);


//        msleep(300); //延迟300ms 方便计算回报值
            std::this_thread::sleep_for(std::chrono::microseconds(295000));

            /**********执行策略 计算回报  根据前后干扰与决策关系来判断（这里回报值获取和计算可修改）************/
            /*****通过频谱数据推算出干扰频点来计算汇报值*****/
//            Spectrum_lw average_spec;
//            average_spec.fill(0);
//
//            const Spectrum_lw data = _water[_spec_param.spectrum_L - 1];
//
//            for (size_t j = 0; j < data.size(); ++j) {
//                average_spec[j] += data[j];
//            }
//
//            std::vector<double> simple_spec(seg_num, 0);
//            for (int i = 0; i < seg_num; ++i) {
//                for (int j = 0; j < seg_size; ++j) {
//                    simple_spec[i] += average_spec[i * seg_size + j]; //把子频带内点数相加
//                }
//            }
//
//            int jamming_index = 0;
//            double max_channel = simple_spec[0];  //设置门限值来获得干扰的频率
//            for (int i = 1; i < seg_num; ++i) {
//                if (simple_spec[i] >= max_channel) {
//                    jamming_index = i;
//                    max_channel = simple_spec[i];
//                }
//            }
            //            if (LastAction == jamming_index + 1)
//                reward = 0;
//            else
//                reward = 10;
//            if (LastAction != channel_index) reward = 0.8 * reward; else reward = 1 * reward;
//            LastAction = channel_index;

            /*****通过发送过来的ack数来计算回报值*****/
            double av_loss = 0.0;
            // LOG(INFO)<<"nck list size is "<<_nck_list.size();
            if(_nck_list.size() > 0){
                _ai_loker.lock();
                for (int i = 0; i < _nck_list.size(); ++i) {
                    av_loss += _nck_list[i]/_nck_list.size();
                }
                _ai_loker.unlock();
            }
            std::cout << "current avr=" << av_loss << std::endl;

            if(av_loss >= 0.5)
                reward = 0;
            else
                reward = 1;
            if (channel_index != LastAction) reward = 0.8 * reward; else reward = 1 * reward;

            std::cout << "current reward=" << reward << std::endl;
            std::cout << "next channel index is " << channel_index << std::endl <<std::endl;

            /**********获取下一时刻的瀑布图 s_t+1, 通过当前传过来的感知数据进行构造************/
            //waterfall_next.pop_front();
            for (int k = 0; k < _water.size(); k++) {
                waterfall_data flag_200;
                flag_200.fill(0);
                int tab_two = 0;
                for (int i = 0; i < flag_200.size(); i++) {
                    int tab_num = 0;
                    while (tab_num < 5) {
                        flag_200[i] += _water[k][i * 5 + tab_num + tab_two];
//                        LOG(INFO)<<" test1 is: "<<_water[k][i * 5 + tab_num + tab_two];
                        tab_num++;
                    }
                    flag_200[i] = flag_200[i] / 5;  //求均值
//                    LOG(INFO)<<" test is: "<<flag_200[i];
                    if (i % 20 == 0)tab_two += 2;
                }
                waterfall_next.pop_front();
                waterfall_next.push_back(flag_200);
            }

            auto next_frame = std::make_shared<dqn::FrameData>();
            for (auto i = 0; i < dqn::kFrameHeight; ++i) {
                for (auto j = 0; j < dqn::kFrameWidth; ++j) {
                    (*next_frame)[i * dqn::kFrameWidth + j] = waterfall_next[i][j];
                }
            }

            /**********训练阶段，则需要update网络************/
            if (update) {
                const auto transition =
                        dqn::Transition(input_frames, channel_index-1, reward, next_frame);
                Dqn.AddTransition(transition);
                // If the size of replay memory is enough, update DQN
                if (Dqn.memory_size() > Memory_TH) Dqn.Update();
            }
            LastAction = channel_index;
            current_frame = next_frame;
        }
        else{

            double av_loss = 0.0;

            if(_nck_list.size() > 0){
                _ai_loker.lock();
                for (int i = 0; i < _nck_list.size(); ++i) {
                    av_loss += _nck_list[i]/_nck_list.size();
                }
                _ai_loker.unlock();
            }
            std::cout << "current avr=" << av_loss << std::endl;

            if(av_loss >= 0.3)
                reward = 0;
            else
                reward = 1;
            if (channel_index != LastAction) reward = 0.8 * reward;else reward = 1 * reward;

            std::cout << "current reward=" << reward << std::endl;


            if(LastAction%2 == 0){
                srand(time(NULL));
                channel_index = 2 * ((rand() % (LastAction)) + 1) - 1;
            }else{
                srand(time(NULL));
                channel_index = 2 * ((rand() % (LastAction)) + 1);
            }

            LastAction = channel_index;
            double hop_freq = 0;
            hop_freq = (_spec_param.rx_freq - _spec_param.rx_rate / 2.0) +
                       ((double) channel_index / (double) seg_num) * _spec_param.rx_rate +
                       0.5 * _spec_param.trans_band;  //找到中心频率

            std::cout << "next channel index is " << channel_index << std::endl <<std::endl;

            uint64_t timeSpec = _curr_time;

            send_hop_command(timeSpec + time_spec_to_ms(_spec_param.guard_time_s), hop_freq);

            std::this_thread::sleep_for(std::chrono::milliseconds(300));

        }
        episode += 1;
//        auto end = std::chrono::steady_clock::now();
//        std::chrono::duration<double, std::micro> elapsed = end - start; // std::micro 表示以微秒为时间单位
//        std::cout<< "time: "  << elapsed.count() << "us" << std::endl;
    }
//    }
}

void Policy::Auto_hop(int32_t Mode){  ///推理出干扰类型 和 自动切换抗干扰策略
    EasyConnect::CommandType sig_type = EasyConnect::CommandType_SENSE;

    std::string Jam_type = "固定干扰+自适应感知跳频";

    ///处理不同干扰模式
    switch (Mode){
        case BLOCK:
            Jam_type = "固定干扰+自适应感知跳频";
            sig_type = EasyConnect::CommandType_SENSE;
            break;
        case TRACK:
            Jam_type = "跟踪干扰+快速跳频";
            sig_type = EasyConnect::CommandType_FAST;
            break;
        case AI:
            Jam_type = "智能干扰+深度强化学习智能抗干扰";
            sig_type = EasyConnect::CommandType_INTELL;
            break;
        case COMB_1:
            Jam_type = "固定梳妆干扰+自适应感知跳频";
            sig_type = EasyConnect::CommandType_SENSE;
            break;
        case COMB_2:
            Jam_type = "固定梳妆干扰+自适应感知跳频";
            sig_type = EasyConnect::CommandType_SENSE;
            break;
        case SWEEP_1:
            Jam_type = "扫频干扰+快速跳频";
            sig_type = EasyConnect::CommandType_FAST;
            break;
        case SWEEP_2:
            Jam_type = "扫频干扰+快速跳频";
            sig_type = EasyConnect::CommandType_FAST;
            break;
        case SWEEP_COMB:
            Jam_type = "动态组合干扰+深度强化学习智能抗干扰";
            sig_type = EasyConnect::CommandType_INTELL;
            break;
        default:
            break;
    }

    ///将字符发送给GUI界面

    std::this_thread::sleep_for(std::chrono::seconds(3)); ///延迟三秒后发送，吞吐量下降很多才开始执行OODA环

//    while(_auto_running){
        flatbuffers::FlatBufferBuilder builder;
        auto parameter = EasyConnect::CreateStringFrame(builder, builder.CreateString(Jam_type));

        builder.Finish(parameter);

        auto chat_Jam = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType::CommandType_Chat,
                                                        false, EasyConnect::CommandData_StringFrame, parameter.Union());

        builder.Finish(chat_Jam);
        auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, chat_Jam.Union(),
                                              builder.CreateString("strategy"));

        builder.Finish(msg);
        this->_conn->send_to("GUI", builder.GetBufferPointer(), builder.GetSize());  ///发送给界面端
//            this->_conn->send_to("strategy", builder.GetBufferPointer(), builder.GetSize());  ///测试用

        std::this_thread::sleep_for(std::chrono::seconds(9));


        flatbuffers::FlatBufferBuilder builder1;
        auto cmd1 = EasyConnect::CreateCommandFrame(builder1, sig_type, false);
        builder1.Finish(cmd1);
        auto msg1 = EasyConnect::CreateMessage(builder1, EasyConnect::DataAny_CommandFrame, cmd1.Union(),
                                               builder1.CreateString("GUI"));
        builder1.Finish(msg1);
        this->_conn->send_to("strategy", builder1.GetBufferPointer(), builder1.GetSize());  ///自动切换策略
//    }


}

void Policy::Play_vdieo() {
    LOG(INFO)<<"Start Play vdieo";

    ///读取视频并播放
    while(1){
        cv::VideoCapture cap("./Test.mp4");
        assert(cap.isOpened());

        cv::Mat frame;
        while (cap.grab())//下一帧是否为空
        {
            cap >> frame;

            double av_loss = 0.0;
            if(_nck_list.size() > 0){
                _ai_loker.lock();
                for (int i = 0; i < _nck_list.size(); ++i) {
                    av_loss += _nck_list[i]/_nck_list.size();
                }
                _ai_loker.unlock();
            }

            int delay = int(av_loss*200 + 50);  //视频卡顿的延时
//
//            int width = frame.rows;
//            int height = frame.cols;
//            int arr = int(av_loss*10); ///马赛克大小
//            for (int i = 0; i < width; i+=arr) {
//                for (int j = 0; j < height; j+=arr) {
//                    //对矩形区域内的每一个像素值进行遍历
//                    for (int k = i; k < arr + i && k < width; k++) {
//                        for (int m = j; m < arr + j && m < height; m++) {
//                            //在这里进行颜色的修改
//                            frame.at<cv::Vec3b>(k, m)[0] = frame.at<cv::Vec3b>(i, j)[0];
//                            frame.at<cv::Vec3b>(k, m)[1] = frame.at<cv::Vec3b>(i, j)[1];
//                            frame.at<cv::Vec3b>(k, m)[2] = frame.at<cv::Vec3b>(i, j)[2];
//                        }
//                    }
//                }
//            }

            cv::namedWindow("video",cv::WINDOW_NORMAL);
            imshow("video", frame);
            char c = cv::waitKey(delay);
            if (c == 27)break;
        }
    }

}

double Policy::CalculateEpsilon(const int iter) {
    if (iter < Iter_Explore) {
        return 1.0 - _greed * (static_cast<double>(iter) / Iter_Explore); //Iter_Explore 次迭代后 取得随机策略的概率为0.1
    } else {
        return 1 - _greed;
    }
}


uint64_t Policy::time_spec_to_ms(const uhd::time_spec_t &timeSpec) {
    return timeSpec.get_full_secs() * 1000 + timeSpec.get_frac_secs() * 1000.0;
}  // change time_spec to ms

Policy::~Policy() {
    this->stop_policy();

    _loop = false;

    if (_state_thread && _state_thread->joinable())_state_thread->join();  //stop heart thread
    LOG(INFO) << "~Policy()";
}

//Send hop_command to the transmitter and the receiver
void Policy::send_hop_command(uint64_t tsp, double hop_freq) {
   // LOG(INFO)<<"Send hop frequency to TX/RX";
    flatbuffers::FlatBufferBuilder builder;
    auto hop_frame = EasyConnect::CreateHopFrame(builder, tsp, hop_freq);
    builder.Finish(hop_frame);

    auto command_frame = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Hop, false,
                                                         EasyConnect::CommandData_HopFrame, hop_frame.Union());
    builder.Finish(command_frame);

    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, command_frame.Union(),
                                          builder.CreateString(_dev.Strategy));

    builder.Finish(msg);

    _conn->send_to(_dev.Transmitter, builder.GetBufferPointer(), builder.GetSize());
    _conn->send_to(_dev.Receiver, builder.GetBufferPointer(), builder.GetSize());
    _conn->send_to(_dev.Disturb, builder.GetBufferPointer(), builder.GetSize());

}


void send_error_str_as_ntp_sync_err(const std::string &id, const std::string &err_str, EasyConnectIface::sptr &conn,
                                    const std::string &from) {
    LOG(ERROR) << err_str;
    flatbuffers::FlatBufferBuilder builder;
    auto err = EasyConnect::CreateStringFrame(builder, builder.CreateString(err_str));
    builder.Finish(err);
    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_NtpSyncErr, false,
                                               EasyConnect::CommandData_StringFrame, err.Union());
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(from));
    builder.Finish(msg);
    conn->send_to(id, builder.GetBufferPointer(), builder.GetSize());
}

void send_sync_ack(int64_t tsp, const std::string &id, EasyConnectIface::sptr &conn, const std::string &from) {
    flatbuffers::FlatBufferBuilder builder;
    auto tsp_frame = EasyConnect::CreateLongFrame(builder, tsp);
    builder.Finish(tsp_frame);
    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_NtpSync, true,
                                               EasyConnect::CommandData_LongFrame, tsp_frame.Union(), tsp);
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(from));
    builder.Finish(msg);
    conn->send_to(id, builder.GetBufferPointer(), builder.GetSize());

}

//bool Policy::sync_time(const std::string &id, const std::string &ip) {
//#if 1
//    return true;
//#else
//    //有gpsdo使用
//    LOG(INFO) << "sync time to GPS";
//
//    if (not _usrp) {
//        this->init();
//    }
//
//    int status = 0;
//    try {
//        auto locate = _iface->get_locate_data();
//        status = locate.status;
//    } catch (std::exception &e) {
//
//    } catch (...) {
//    }
//
//    if (status == 0) {
//        LOG(INFO) << "GPS not lock";
//    }
//
//    _usrp->set_time_now(uhd::time_spec_t{0, 0});
//    auto now = _iface->get_gps_time();
//
//    LOG(INFO) << "gps now: " << now.get_full_secs() << " " << now.get_frac_secs();
//
//    time_t t = now.get_full_secs();
//    LOG(INFO) << std::ctime(&t);
//
//    _usrp->set_time_next_pps(now.get_full_secs() + 1);
//
//    uhd::time_spec_t fpga_time;
//    for (int l = 0; l < 10; ++l) {
//
//        std::this_thread::sleep_for(std::chrono::milliseconds(180));
//        fpga_time = _usrp->get_time_now();
//        LOG(INFO) << "" << fpga_time.get_full_secs() << " " << fpga_time.get_frac_secs();
//
//    }
//
//
//    auto locate = _iface->get_locate_data();
//    LOG(INFO) << "num: " << locate.num;
//    LOG(INFO) << "gps status: " << locate.status;
//    LOG(INFO) << "location: " << locate.longitude << "," << locate.latitude << "," << locate.altitude;
//
//    send_sync_ack(fpga_time.get_full_secs() * 1000, _dev.Client, _conn, _dev.Perception);
////    const std::string err_str = "Set system time failed, please try again via 'sudo'";
////    auto *ntp_client = new NTPclient(ip);
////    int ret;
////    for (int j = 0; j < 10; ++j) {
////        ret = ntp_client->SetSystemTime();
////        if (ret == 0) {
////            LOG(INFO) << "ntp ok";
////            send_ntp_sync_ack(id, _conn, _dev.Perception); //send ack
////            return;
////        }
////
////        if (ret == -999) {
////            send_error_str_as_ntp_sync_err(id, err_str, _conn, _dev.Perception);
////            return;
////        }
////
////        std::this_thread::sleep_for(std::chrono::milliseconds(100));
////    }
//
//
//    auto now_epoch = _usrp->get_time_now().get_full_secs();
//    auto cpu_time = std::chrono::duration_cast<std::chrono::seconds>(
//            std::chrono::system_clock::now().time_since_epoch()).count();
//    LOG(INFO) << "now epoch: " << now_epoch;
//    LOG(INFO) << "cpu epoch: " << cpu_time;
//    if (std::abs(now_epoch - cpu_time) < 3600) {
//        return true;
//    }
//#endif
//}

void Policy::send_exception(const std::string &id, const std::string &str) {
    flatbuffers::FlatBufferBuilder builder;
    auto err = EasyConnect::CreateStringFrame(builder, builder.CreateString(str));
    builder.Finish(err);
    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Exception, false,
                                               EasyConnect::CommandData_StringFrame, err.Union());
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(_dev.Strategy));
    builder.Finish(msg);

    _conn->send_to(id, builder.GetBufferPointer(), builder.GetSize());
}

void Policy::broadcast_broker(const std::string &host) {
    _conn->broadcast_host(host);
}

void Policy::send_msg(const std::string &id, void *data, size_t size) {  //GUI sends the msg to the Policy
    _conn->send_to(id, data, size);
}

void Policy::reset_usb() {
    auto usb = terjin::USB::make();
    usb->reset_usb(0x2500, 0x0020);

}

void Policy::state_working() {
    while (_loop) {
        if (_conn) {
            flatbuffers::FlatBufferBuilder builder;
            auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_STATE);
            builder.Finish(cmd);
            auto ms = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                                 builder.CreateString(_dev.Strategy));
            builder.Finish(ms);

            _conn->send_to(_dev.Client, builder.GetBufferPointer(), builder.GetSize());
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
