//
// Created by locate on 4/26/20.
//
#include <easy_connect_generated.h>
#include <easy_connect/EasyConnect.hpp>
#include <chrono>
#include <thread>
#include <glog/logging.h>
#include <iostream>

void on_msg(const std::string &, void *, size_t) {

}

void send_snr(EasyConnectIface::sptr &conn, double snr) {
    flatbuffers::FlatBufferBuilder builder;
    auto snr_frame = EasyConnect::CreateSnrFrame(builder, snr);
    builder.Finish(snr_frame);
    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_SNR, false,
                                               EasyConnect::CommandData_SnrFrame, snr_frame.Union());
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString("GUI"));
    builder.Finish(msg);

//    c->on_msg(id, builder.GetBufferPointer(), builder.GetSize());
    conn->send_to("perception", builder.GetBufferPointer(), builder.GetSize());
}

void send_chat(EasyConnectIface::sptr &conn, const std::string &chat) {
    flatbuffers::FlatBufferBuilder builder;
    auto str = EasyConnect::CreateStringFrame(builder, builder.CreateString(chat));
    builder.Finish(str);
    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Chat, false,
                                               EasyConnect::CommandData_StringFrame, str.Union());
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString("GUI"));
    builder.Finish(msg);

    conn->send_to("perception", builder.GetBufferPointer(), builder.GetSize());
    LOG(INFO) << "send ok";

}

int main(int, char *[]) {
    EasyConnectIface::Config config;
    config.udp_port = 8774; //change a port
    config.id = "client";
    auto conn = EasyConnectIface::make(config, on_msg);

    conn->create_broker();


    conn->broadcast_host("127.0.0.1"); //set zmq setlf

    for (int i = 0; i < 100; ++i) {
        send_chat(conn, "hello world");
        std::cin.get();
    }

    //if you want send msg to your client then
    //conn->send_to("perception",.....);


    return 0;
}