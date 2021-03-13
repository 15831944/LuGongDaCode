//
// Created by locate on 2020/5/10.
//

#include <string>
#include <glog/logging.h>
#include <csignal>
#include <jsoncpp/json/json.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <easy_connect/version.hpp>

#include "LinkLayer.hpp"

static bool stop_signal_called = false;

void sig_int_handler(int) {
    stop_signal_called = true;
    LOG(INFO) << "Ctrl + C Press...";
}

bool ReadConfigFile(const std::string &filename, Json::Value &value) {
    Json::Reader reader;

    std::ifstream rf(filename, std::ios::binary);
    if (rf.fail()) {
        LOG(INFO) << "failed to open " << filename;
        return false;
    }
    if (!reader.parse(rf, value)) {
        LOG(INFO) << "failed to parse " << filename;
        return false;
    }
    rf.close();
    return true;
}

void ErrorInput() {
    LOG(WARNING) << "error input";
}

enum Command {
    START = 0,
    STOP = 1,
    SETPARAM = 2,
    EXIT = 3,
    CHATDATA = 4,
    FILEDATA = 5,
    DEVICE = 6,
    PARAMS = 7,
    HOP = 8,
    SYNC = 9,
    CANCEL = 10,
    TESTTIME = 11,
    IP = 12,
    UNKNOWN
};

Command StringToMod(const std::string &cmd) {
    if (cmd == "start") return START;
    if (cmd == "stop") return STOP;
    if (cmd == "setparam") return SETPARAM;
    if (cmd == "exit") return EXIT;
    if (cmd == "chat") return CHATDATA;
    if (cmd == "file") return FILEDATA;
    if (cmd == "device") return DEVICE;
    if (cmd == "updata") return PARAMS;
    if (cmd == "hop") return HOP;
    if (cmd == "sync") return SYNC;
    if (cmd == "cancel") return CANCEL;
    if (cmd == "time") return TESTTIME;
    if (cmd == "ip") return IP;
    return UNKNOWN;
}


std::string owner = "GUI";

struct OTHER {
    std::string tx = "transmitter";
    std::string rx = "receiver";
};

OTHER other;

void Start(LinkLayer::sptr &link_layer, bool start) {
    flatbuffers::FlatBufferBuilder builder;

    auto cmd = EasyConnect::CreateCommandFrame(builder,
                                               start ? EasyConnect::CommandType_Start : EasyConnect::CommandType_Stop,
                                               false);
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(owner));
    builder.Finish(msg);

    link_layer->SendMsg(other.tx, builder.GetBufferPointer(), builder.GetSize());
    link_layer->SendMsg(other.rx, builder.GetBufferPointer(), builder.GetSize());
    if (start)
        LOG(INFO) << "send start command to " << other.tx << " and " << other.rx;
    else
        LOG(INFO) << "send stop command to " << other.tx << " and " << other.rx;
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

void SetParam(LinkLayer::sptr &link_layer, const std::vector<std::string> &vec) {
    double tx_freq = 838e6;
    double rx_freq = 838e6;
    double sample_rate = 270e3;
    double gain = 50;
    int mtu = 150;
    if (vec.size() > 1) {
        tx_freq = ChangeType<double>(vec.at(1), 838e6);
    }
    if (vec.size() > 2) {
        rx_freq = ChangeType<double>(vec.at(2), 838e6);
    }
    if (vec.size() > 3) {
        sample_rate = ChangeType<double>(vec.at(3), 270e3);
    }
    if (vec.size() > 4) {
        gain = ChangeType<double>(vec.at(4), 50);
    }
    if (vec.size() > 5) {
        mtu = ChangeType<int>(vec.at(5), 127);
    }

    flatbuffers::FlatBufferBuilder builder;
    auto parameter = EasyConnect::CreateParameterFrame(builder, mtu, tx_freq, gain, rx_freq, gain, sample_rate);
    builder.Finish(parameter);

    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType::CommandType_SetParam,
                                               false, EasyConnect::CommandData_ParameterFrame, parameter.Union());

    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(owner));
    builder.Finish(msg);

    link_layer->SendMsg(other.tx, builder.GetBufferPointer(), builder.GetSize());
    link_layer->SendMsg(other.rx, builder.GetBufferPointer(), builder.GetSize());
    LOG(INFO) << "tx_freq: " << tx_freq << "Hz, rx_freq: " << rx_freq << "Hz, sample_rate: " << sample_rate
              << "Hz, MTU: " << mtu;
    LOG(INFO) << "send params to " << other.tx << " and " << other.rx;
}

void SendChat(LinkLayer::sptr &link_layer, const std::vector<std::string> &vec) {
    if (vec.size() < 2) {
        ErrorInput();
        return;
    }

    std::string str;
    auto Ptr = vec.begin() + 1;
    while (Ptr < vec.end()) {
        str.append(*Ptr++);
        if (Ptr != vec.end())
            str.append(" ");
    }

    flatbuffers::FlatBufferBuilder builder;
    auto chat = EasyConnect::CreateStringFrame(builder, builder.CreateString(str));
    builder.Finish(chat);

    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Chat, false,
                                               EasyConnect::CommandData_StringFrame, chat.Union());
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(owner));
    builder.Finish(msg);
    link_layer->SendMsg(other.tx, builder.GetBufferPointer(), builder.GetSize());
//    link_layer->SendMsg(other.rx, builder.GetBufferPointer(), builder.GetSize());
    LOG(INFO) << "chat: " << str;
//    LOG(INFO) << "send chat to " << other.tx << " and " << other.rx;
    LOG(INFO) << "send chat to " << other.tx;
}

void SendFileData(LinkLayer::sptr &link_layer, const std::vector<std::string> &vec) {
    if (vec.size() < 2) {
        ErrorInput();
        return;
    }

    flatbuffers::FlatBufferBuilder builder;

    std::vector<uint8_t> data(vec.at(1).size());
    memcpy(data.data(), vec.at(1).data(), vec.at(1).size());
    auto binary_frame = EasyConnect::CreateFileFrameDirect(builder, vec.at(1).data(), 0, 0, data.size(), nullptr);

    builder.Finish(binary_frame);

    auto msgs = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_FileFrame, binary_frame.Union(),
                                           builder.CreateString(owner));
    builder.Finish(msgs);

    link_layer->SendMsg(other.tx, builder.GetCurrentBufferPointer(), builder.GetSize());
    LOG(INFO) << "send file " << vec.at(1) << " to " << other.tx;
}

uint64_t GetSystemTimeMs() {
    timeval tv{};
    gettimeofday(&tv, nullptr);
    return uint64_t((tv.tv_sec + (tv.tv_usec / 1000000.0)) * 1000);
}

void SendHop(LinkLayer::sptr &link_layer, const std::vector<std::string> &vec) {
    double freq = 838e6;
    uint64_t time = 20;
    if (vec.size() > 1) {
        freq = ChangeType<double>(vec.at(1), 838e6);
    }
    if (vec.size() > 2) {
        time = ChangeType<uint64_t>(vec.at(2), 20);
    }
    time += GetSystemTimeMs();

    flatbuffers::FlatBufferBuilder builder;
    auto hop = EasyConnect::CreateHopFrame(builder, time, freq);
    builder.Finish(hop);

    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Hop, false,
                                               EasyConnect::CommandData_HopFrame, hop.Union());
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(owner));
    builder.Finish(msg);
    link_layer->SendMsg(other.tx, builder.GetBufferPointer(), builder.GetSize());
    link_layer->SendMsg(other.rx, builder.GetBufferPointer(), builder.GetSize());
    LOG(INFO) << "freq: " << freq << "Hz, time: " << time << "ms";
    LOG(INFO) << "send hop to " << other.tx << " and " << other.rx;
}

void SendSync(LinkLayer::sptr &link_layer, const std::vector<std::string> &vec) {
    std::string ntp_ip = "120.25.115.20";
    if (vec.size() > 2) {
        ntp_ip = vec.at(1);
    }

    flatbuffers::FlatBufferBuilder builder;
    auto ntp = EasyConnect::CreateStringFrame(builder, builder.CreateString(ntp_ip));
    builder.Finish(ntp);

    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_NtpSync, false,
                                               EasyConnect::CommandData_StringFrame, ntp.Union());
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(owner));
    builder.Finish(msg);
    link_layer->SendMsg(other.tx, builder.GetBufferPointer(), builder.GetSize());
    link_layer->SendMsg(other.rx, builder.GetBufferPointer(), builder.GetSize());
//    LOG(INFO) << "ntp server ip: " << ntp_ip;
    LOG(INFO) << "send ntp sync to " << other.tx << " and " << other.rx;
}

void TestTime(LinkLayer::sptr &link_layer, const std::vector<std::string> &vec) {
    uint64_t time = 80;//ms
    if (vec.size() > 1) {
        time = ChangeType<uint64_t>(vec.at(1), 20);//ms
    }
    time += GetSystemTimeMs();

    flatbuffers::FlatBufferBuilder builder;
    auto testtime = EasyConnect::CreateLongFrame(builder, time);
    builder.Finish(testtime);

    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_TestTime, false,
                                               EasyConnect::CommandData_LongFrame, testtime.Union());
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(owner));
    builder.Finish(msg);
    link_layer->SendMsg(other.tx, builder.GetBufferPointer(), builder.GetSize());
    link_layer->SendMsg(other.rx, builder.GetBufferPointer(), builder.GetSize());
    LOG(INFO) << "test time: " << time << "ms";
    LOG(INFO) << "send test time to " << other.tx << " and " << other.rx;
}

void BroadcastHost(LinkLayer::sptr &link_layer, const std::vector<std::string> &vec) {
    std::string ip = "127.0.0.1";
    if (vec.size() > 1) {
        ip = vec.at(1);
    }

    link_layer->BroadcastBroker(ip);
    LOG(INFO) << "ip: " << ip;
}

void CopeInputCmd(LinkLayer::sptr &link_layer, const std::string &input) {
    std::vector<std::string> vec;
    boost::split(vec, input, boost::is_any_of(" "));
    if (vec.empty()) {
        return ErrorInput();
    }

    Command cmd = StringToMod(vec.at(0));

    switch (cmd) {
        case START: {
            Start(link_layer, true);
            break;
        }
        case STOP: {
            Start(link_layer, false);
            break;
        }
        case SETPARAM: {
            SetParam(link_layer, vec);
            break;
        }
        case EXIT: {
            stop_signal_called = true;
            break;
        }
        case CHATDATA: {
            SendChat(link_layer, vec);
            break;
        }
        case FILEDATA: {
            SendFileData(link_layer, vec);
            break;
        }
        case DEVICE: {
            break;
        }
        case PARAMS: {
            break;
        }
        case HOP: {
            SendHop(link_layer, vec);
            break;
        }
        case SYNC: {
            SendSync(link_layer, vec);
            break;
        }
        case CANCEL: {
            break;
        }
        case TESTTIME: {
            TestTime(link_layer, vec);
            break;
        }
        case IP: {
            BroadcastHost(link_layer, vec);
            break;
        }
        default:{
            //ErrorInput();
            return;
        }
    }
}

int main(int argc, char *argv[]) {
    try {

        google::InitGoogleLogging(argv[0]);
        //FLAGS_logtostderr = true;//设置日志消息是否转到标准输出而不是日志文件
        FLAGS_alsologtostderr = true;//设置日志消息除了日志文件之外是否去标准输出
        FLAGS_colorlogtostderr = true;//设置记录到标准输出的颜色消息（如果终端支持）
        system("mkdir logs > /dev/null");
        FLAGS_logbufsecs = 0;//多少秒打印一次日志 0为实时刷新

        google::SetLogDestination(google::INFO, "./logs/INFO_");
        google::SetLogDestination(google::WARNING, "./logs/WARNING_");
        google::SetLogDestination(google::ERROR, "./logs/ERROR_");
        google::SetLogDestination(google::FATAL, "./logs/FATAL_");

        LOG(INFO) << "Version: " << EasyConnectVersion();
        std::string filename = "DeviceCfg.json";

        Json::Value config;
        bool ret = ReadConfigFile(filename, config);
        if (!ret) {
            return -1;
        }

        std::signal(SIGINT, &sig_int_handler);
        LOG(INFO) << "程序启动成功,时间: " << __DATE__ << " " << __TIME__;

        LinkLayer::sptr link_layer = LinkLayer::make(config);

        std::string input;
        while (not stop_signal_called) {
            std::cout << ">>";
            getline(std::cin, input);
            CopeInputCmd(link_layer, input);
        }

        Start(link_layer, false);

        LOG(INFO) << "exit...";
    } catch (std::exception &e) {
        LOG(ERROR) << e.what();
    }
    exit(0);
}