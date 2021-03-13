//
// Created by locate on 4/22/20.
//

#include <Policy.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/lexical_cast.hpp>

const std::string id = "GUI";


const std::string ntp_server = "127.0.0.1"; //时间同步 广播地址

enum mode{
    BLOCK = 1,
    TRACK = 2,
    AI = 3,
};

void start_command(Policy::sptr &policy) {
    flatbuffers::FlatBufferBuilder builder;
    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Start, false);
    //auto cmd1 = EasyConnect::CreateCommandFrame(builder,EasyConnect::CommandType_Start, false, )
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(id));
    builder.Finish(msg);
    policy->send_msg("strategy", builder.GetBufferPointer(), builder.GetSize());
}


//void send_ntp_sync(const std::string &dest, Policy::sptr &policy) {
//    flatbuffers::FlatBufferBuilder builder;
//    auto err = EasyConnect::CreateStringFrame(builder, builder.CreateString(ntp_server));
//    builder.Finish(err);
//    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_NtpSync, false,
//                                               EasyConnect::CommandData_StringFrame, err.Union());
//    builder.Finish(cmd);
//    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
//                                          builder.CreateString(id));
//    builder.Finish(msg);
//
//    policy->send_msg(dest, builder.GetBufferPointer(), builder.GetSize());
//
//}

//CommandType_Hop = 9,
//        CommandType_TestTime = 10,
//        CommandType_CRC_CHECK = 11,

void Hop_one(Policy::sptr &policy, int mode, int channel) {
    flatbuffers::FlatBufferBuilder builder;
    auto parameter = EasyConnect::CreateDisturb(builder,mode,channel);

    builder.Finish(parameter);

    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType::CommandType_FIXED,
                                               false, EasyConnect::CommandData_Disturb, parameter.Union());

    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(id));
    builder.Finish(msg);
    policy->send_msg("strategy", builder.GetBufferPointer(), builder.GetSize());

}

void Hop_two(Policy::sptr &policy) {
    flatbuffers::FlatBufferBuilder builder;
    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_SENSE, false);
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(id));
    builder.Finish(msg);
    policy->send_msg("strategy", builder.GetBufferPointer(), builder.GetSize());
}

void Hop_three(Policy::sptr &policy) {
    flatbuffers::FlatBufferBuilder builder;
    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_FAST, false);
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(id));
    builder.Finish(msg);
    policy->send_msg("strategy", builder.GetBufferPointer(), builder.GetSize());
}


void Hop_four(Policy::sptr &policy) {
    flatbuffers::FlatBufferBuilder builder;
    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_INTELL, false);
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(id));
    builder.Finish(msg);
    policy->send_msg("strategy", builder.GetBufferPointer(), builder.GetSize());
}

//发送给干扰机，调整干扰模式
void Jamming_one(Policy::sptr &policy, int mo, int channel) {
    flatbuffers::FlatBufferBuilder builder;
    auto parameter = EasyConnect::CreateDisturb(builder,mo,channel);

    builder.Finish(parameter);

    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType::CommandType_SetParam,
                                               false, EasyConnect::CommandData_Disturb, parameter.Union());

    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(id));
    builder.Finish(msg);
    policy->send_msg("disturb", builder.GetBufferPointer(), builder.GetSize());
    policy->send_msg("strategy", builder.GetBufferPointer(), builder.GetSize());
}


template<typename T>
T ChangeType(const std::string &vec, T dv) {
    try {
        return boost::lexical_cast<T>(vec);
    } catch (std::exception &e) {
        LOG(INFO) << e.what();
        return dv;
    }

}

bool loop = true;

void handle_sigint(int) {
    loop = false;
}

//void sync(const std::string &cmd, Policy::sptr &policy) { //给发射机、接收机和感知机 and policy
//    LOG(INFO) << "Send Sync";
//    send_ntp_sync("transmitter", policy);
//    send_ntp_sync("receiver", policy);
//    send_ntp_sync("perception", policy);
//    send_ntp_sync("strategy",policy);
//}


int main(int, char *[]) {
    remove("waterfall.txt");
    auto policy = Policy::make();

    policy->start_conn(8773);

    std::thread cmd_thread([&]() {  //子线程读取命令
        for (std::string line; std::getline(std::cin, line);) {
            LOG(INFO)<<"input command is "<<line;
            std::vector<std::string> vec;
            boost::split(vec, line, boost::is_any_of(" "));

            if (vec.empty()){
                LOG(WARNING) << "error input";
            }

            std::string cmd = vec.at(0);

            if (cmd == "exit") {
                loop = false;
                break;
            }
            if (cmd == "ip") {
                LOG(INFO) << "broadcast broker ip";
                std::string ip = "127.0.0.1";
                if(vec.size() > 1){
                    ip = vec.at(1);
                }
                LOG(INFO) << "broker: " << ip;
                policy->broadcast_broker(ip);
            }

            if (cmd == "start"){
                LOG(INFO) << "start device!";
                start_command(policy);
            }

            if (cmd == "reset") {
                LOG(INFO) << "reset";
                policy->reset_usb();
                continue;
            }

            if (cmd == "stop") {
                policy->stop_policy();
                continue;
            }

            //Jamming 5 5
            if (cmd == "Jamming") {
                LOG(INFO) << "Anti-Jamming mode one: Fixed Frequency";
                int channel = 1;
                int mo = 1;
                if (vec.size() > 1){
                    mo = ChangeType<int>(vec.at(1),1);
                    channel = ChangeType<int>(vec.at(2),1);
                }
                LOG(INFO) << "next channel index is: " <<channel;
                Jamming_one(policy,mo,channel);
                continue;
            }

            // fixed 2 5
            if (cmd == "fixed") {
                LOG(INFO) << "Anti-Jamming mode one: Fixed Frequency";
                int next_freq = 1;
                int mod = 1;
                if (vec.size() > 1){
                    mod = ChangeType<double>(vec.at(1),1);
                    next_freq = ChangeType<double>(vec.at(2),1);
                }
                LOG(INFO) << "next channel index is: " <<next_freq;
                Hop_one(policy,mod,next_freq);
                continue;
            }
            if (cmd == "sense"){
                LOG(INFO) << "Anti-Jamming mode two: Sense-based Hop Frequency";
                Hop_two(policy);
            }
            if (cmd == "fast"){
                LOG(INFO) << "Anti-Jamming mode three: Sense-based Hop Frequency";
                Hop_three(policy);
            }
            if (cmd == "intell"){
                LOG(INFO) << "Anti-Jamming mode four: DQN-based Hop Frequency";
                Hop_four(policy);
            }

        }
    });

    while (loop) {  //循环等待指令
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    cmd_thread.detach();

    return 0;
}