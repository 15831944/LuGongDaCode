//
// Created by locate on 26/4/2020.
//

#include <easy_connect_generated.h>
#include <easy_connect/EasyConnect.hpp>
#include <glog/logging.h>
#include <csignal>
#include <iostream>
#include <ctime>
#include <sys/time.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

std::string owner = "client";

struct OTHER {
    std::string tx = "transmitter";
    std::string rx = "receiver";
};

OTHER other;

static bool stop_signal_called = false;

void sig_int_handler(int) { stop_signal_called = true; }

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
    NTPSYNC = 9,
    CANCEL = 10,
    TESTTIME = 11,
    IP = 12,
    UNKNOWN
};

using namespace std;

void on_msg(const std::string &, void *, size_t) {

}

void ErrorInput() {
    LOG(WARNING) << "error input";
}

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
    if (cmd == "ntp") return NTPSYNC;
    if (cmd == "cancel") return CANCEL;
    if (cmd == "time") return TESTTIME;
    if (cmd == "ip") return IP;
    return UNKNOWN;
}

void Start(EasyConnectIface::sptr &conn, bool start) {
    flatbuffers::FlatBufferBuilder builder;

    auto cmd = EasyConnect::CreateCommandFrame(builder,
                                               start ? EasyConnect::CommandType_Start : EasyConnect::CommandType_Stop,
                                               false);
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(owner));
    builder.Finish(msg);

    conn->send_to(other.tx, builder.GetBufferPointer(), builder.GetSize());
    conn->send_to(other.rx, builder.GetBufferPointer(), builder.GetSize());
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

void SetParam(EasyConnectIface::sptr &conn, const std::vector<std::string> &vec) {
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

    conn->send_to(other.tx, builder.GetBufferPointer(), builder.GetSize());
    conn->send_to(other.rx, builder.GetBufferPointer(), builder.GetSize());
    LOG(INFO) << "tx_freq: " << tx_freq << "Hz, rx_freq: " << rx_freq << "Hz, sample_rate: " << sample_rate
              << "Hz, MTU: " << mtu;
    LOG(INFO) << "send params to " << other.tx << " and " << other.rx;
}

void SendChat(EasyConnectIface::sptr &conn, const std::vector<std::string> &vec) {
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
    conn->send_to(other.tx, builder.GetBufferPointer(), builder.GetSize());
    conn->send_to(other.rx, builder.GetBufferPointer(), builder.GetSize());
    LOG(INFO) << "chat: " << str;
    LOG(INFO) << "send chat to " << other.tx << " and " << other.rx;
}

uint64_t GetSystemTimeMs() {
    timeval tv{};
    gettimeofday(&tv, nullptr);
    return uint64_t((tv.tv_sec + (tv.tv_usec / 1000000.0)) * 1000);
}

void SendHop(EasyConnectIface::sptr &conn, const std::vector<std::string> &vec) {
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
    conn->send_to(other.tx, builder.GetBufferPointer(), builder.GetSize());
    conn->send_to(other.rx, builder.GetBufferPointer(), builder.GetSize());
    LOG(INFO) << "freq: " << freq << "Hz, time: " << time << "ms";
    LOG(INFO) << "send hop to " << other.tx << " and " << other.rx;
}

void SendNTPSync(EasyConnectIface::sptr &conn, const std::vector<std::string> &vec) {
    std::string ntp_ip = "192.168.20.171";
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
    conn->send_to(other.tx, builder.GetBufferPointer(), builder.GetSize());
    conn->send_to(other.rx, builder.GetBufferPointer(), builder.GetSize());
    LOG(INFO) << "ntp server ip: " << ntp_ip;
    LOG(INFO) << "send ntp sync to " << other.tx << " and " << other.rx;
}

void TestTime(EasyConnectIface::sptr &conn, const std::vector<std::string> &vec) {
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
    conn->send_to(other.tx, builder.GetBufferPointer(), builder.GetSize());
    conn->send_to(other.rx, builder.GetBufferPointer(), builder.GetSize());
    LOG(INFO) << "test time: " << time << "ms";
    LOG(INFO) << "send test time to " << other.tx << " and " << other.rx;
}

void BroadcastHost(EasyConnectIface::sptr &conn, const std::vector<std::string> &vec) {
    std::string ip = "127.0.0.1";
    if (vec.size() > 1) {
        ip = vec.at(1);
    }

    conn->broadcast_host(ip);
    LOG(INFO) << "ip: " << ip;
}

void CopeInputCmd(EasyConnectIface::sptr &conn, const std::string &input) {
    std::vector<std::string> vec;
    boost::split(vec, input, boost::is_any_of(" "));
    if (vec.empty()) {
        return ErrorInput();
    }

    Command cmd = StringToMod(vec.at(0));

    switch (cmd) {
        case START: {
            Start(conn, true);
            break;
        }
        case STOP: {
            Start(conn, false);
            break;
        }
        case SETPARAM: {
            SetParam(conn, vec);
            break;
        }
        case EXIT: {
            break;
        }
        case CHATDATA: {
            SendChat(conn, vec);
            break;
        }
        case FILEDATA: {
            break;
        }
        case DEVICE: {
            break;
        }
        case PARAMS: {
            break;
        }
        case HOP: {
            SendHop(conn, vec);
            break;
        }
        case NTPSYNC: {
            SendNTPSync(conn, vec);
            break;
        }
        case CANCEL: {
            break;
        }
        case TESTTIME: {
            TestTime(conn, vec);
            break;
        }
        case IP: {
            BroadcastHost(conn, vec);
            break;
        }
        default:
            ErrorInput();
    }
}

int main(int argc, char *argv[]) {
    std::string host = "127.0.0.1";
    if (argc > 1) {
        host = argv[1];
    }
    if (argc > 2) {
        other.tx = argv[2];
    }
    if (argc > 3) {
        other.rx = argv[3];
    }

    EasyConnectIface::Config config;
    config.udp_port = 8774; //change a port
    config.id = owner;
    auto conn = EasyConnectIface::make(config, on_msg);

    conn->create_broker();
    conn->broadcast_host(host);

#if 0
    system("mkdir logs > /dev/null");
    FLAGS_logtostderr = false;
    FLAGS_logbufsecs = 10;
#else
    FLAGS_logtostderr = true;
#endif

    google::InitGoogleLogging(argv[0]);
//    google::SetLogDestination(google::INFO, "./logs/INFO_");
//    google::SetLogDestination(google::WARNING, "./logs/WARNING_");
//    google::SetLogDestination(google::ERROR, "./logs/ERROR_");
//    google::SetLogDestination(google::FATAL, "./logs/FATAL_");


    std::signal(SIGINT, &sig_int_handler);
    LOG(INFO) << "客户端启动成功,时间: " << __DATE__ << " " << __TIME__;

    std::cout << "请输入命令" << std::endl;
    std::string input;
    while (!stop_signal_called) {
        std::cout << ">>";
        getline(std::cin, input);
        CopeInputCmd(conn, input);
    }

    if (stop_signal_called)
        LOG(INFO) << "Ctrl + C Press...";

    LOG(INFO) << "exit...";
    return 0;
}