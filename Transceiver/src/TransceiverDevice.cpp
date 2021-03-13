//
// Created by locate on 24/4/2020.
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

void sig_int_handler(int) { stop_signal_called = true; }

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

int main(int argc, char *argv[]) {
    try {
        LOG(INFO) << "Version: " << EasyConnectVersion();
        std::string filename = "DeviceCfg.json";

        Json::Value config;
        bool ret = ReadConfigFile(filename, config);
        if (!ret) {
            return -1;
        }

        remove("ack.txt");
        remove("nck.txt");

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
        LOG(INFO) << "设备端启动成功,时间: " << __DATE__ << " " << __TIME__;

        LinkLayer::sptr link_layer = LinkLayer::make(config);

        if (!config["test"].empty() && config["test"].isBool() && !config["host"].empty() &&
            config["host"].isString()) {
            if (config["test"].asBool()) {
                link_layer->BroadcastBroker(config["host"].asString());
                LOG(INFO) << "for test";
            }
        }

        link_layer->CopeStartCommand();

        while (not stop_signal_called) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        link_layer->CopeStopCommand();

        if (stop_signal_called)
            LOG(INFO) << "Ctrl + C Press...";

        LOG(INFO) << "exit...";
    } catch (std::exception &e) {
        LOG(ERROR) << e.what();
    }
    exit(0);
}