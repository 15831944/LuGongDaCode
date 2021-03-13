//
// Created by locate on 4/22/20.
//

#include <Perception.hpp>
#include <glog/logging.h>
#include <glog/log_severity.h>

const std::string id = "GUI";


const std::string ntp_server = "127.0.0.1";

void start_command(Perception::sptr &perception) {
    flatbuffers::FlatBufferBuilder builder;
    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Start, false);
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(id));
    builder.Finish(msg);
    perception->send_msg("perception", builder.GetBufferPointer(), builder.GetSize());
}


void send_ntp_sync(const std::string &dest, Perception::sptr &perception) {
    flatbuffers::FlatBufferBuilder builder;
    auto err = EasyConnect::CreateStringFrame(builder, builder.CreateString(ntp_server));
    builder.Finish(err);
    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_NtpSync, false,
                                               EasyConnect::CommandData_StringFrame, err.Union());
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(id));
    builder.Finish(msg);

    perception->send_msg(dest, builder.GetBufferPointer(), builder.GetSize());

}


bool loop = true;

void handle_sigint(int) {
    loop = false;
}

void sync(const std::string &cmd, Perception::sptr &perception) {
    LOG(INFO) << "Send Sync";
    send_ntp_sync("transmitter", perception);
    send_ntp_sync("receiver", perception);
    send_ntp_sync("perception", perception);
}


int main(int argc, char *argv[]) {

    google::InitGoogleLogging(argv[0]);
    //FLAGS_logtostderr = true;//设置日志消息是否转到标准输出而不是日志文件
    FLAGS_alsologtostderr = true;//设置日志消息除了日志文件之外是否去标准输出
    FLAGS_colorlogtostderr = true;//设置记录到标准输出的颜色消息（如果终端支持）
    system("mkdir logs > /dev/null");
    FLAGS_logbufsecs = 0;//多少秒打印一次日志 0为实时刷新

    google::SetLogDestination(google::GLOG_INFO, "./logs/INFO_");
    google::SetLogDestination(google::GLOG_WARNING, "./logs/WARNING_");
    google::SetLogDestination(google::GLOG_ERROR, "./logs/ERROR_");
    google::SetLogDestination(google::GLOG_FATAL, "./logs/FATAL_");


    auto perception = Perception::make();

    perception->start_conn(8773);

    for (int j = 0; j < 10; ++j) {
        LOG(INFO) << "auto sync...";
        bool ret = perception->sync_time("GUI", "");
        if (ret)break;
    }


    //for tests
//    perception->broadcast_broker("127.0.0.1"); //set zmq broker for 127.0.0.1

//    send_ntp_sync(id, perception);//ntp sync

//    perception->start_sweeper();

    std::thread cmd_thread([&]() {
        for (std::string line; std::getline(std::cin, line);) {
            if (line.substr(0, 4) == "exit") {
                loop = false;
                break;
            }

            std::stringstream ss;
            ss << line;

            std::string cmd;
            ss >> cmd;

            //handle cmd
            if (cmd == "sync") {
                sync(line, perception);
                continue;
            }

            if (cmd == "ip") {
                LOG(INFO) << "broadcast broker ip";
                std::string ip = "127.0.0.1";
                ss >> ip;
                LOG(INFO) << "broker: " << ip;
                perception->broadcast_broker(ip);
            }


            if (cmd == "start") {
                LOG(INFO) << "start sweeper";
                start_command(perception);
                continue;
            }

            if (cmd == "reset") {
                LOG(INFO) << "reset";
                perception->reset_usb();
                continue;
            }

            if (cmd == "stop") {
                perception->stop_sweeper();
                continue;
            }

        }
    });

    while (loop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    cmd_thread.detach();

    return 0;
}