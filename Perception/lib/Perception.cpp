//
// Created by locate on 4/22/20.
//

#include <Perception.hpp>
#include <glog/logging.h>
#include <fstream>
#include "DFT.hpp"
//#include "GNUPlotIface.hpp"
#include "NTPclient.h"
#include <easy_connect_generated.h>
#include "USB.hpp"

using terjin::TestFrame;
using terjin::MessageFrame;

#define msleep(N); boost::this_thread::sleep(boost::posix_time::milliseconds(N));


Perception::Perception() {
    _spec_param.rx_rate = 10e6;
    _spec_param.gain = 70;
    _spec_param.rx_freq = 838e6;
    _spec_param.fft_size = 1024;
    _spec_param.rx_len = 1 << 14; //rx_len

    _spec_param.interval_s = 10e-3; // 10ms
    _spec_param.hop_interval_s = 100e-3;// check need hop 100ms

    _spec_param.trans_band = 500e3;
    _spec_param.performance_count = 10; //get performance for 10 times
    _spec_param.current_performance_index = 0;
    _spec_param.current_performance_count = 0;

    _spec_param.guard_time_s = 40e-3;

    _performance = std::vector<performance_t>(_spec_param.performance_count);
    memset(_performance.data(), 0, sizeof(performance_t) * _performance.size()); //set zero


    _spec_param.spectrum_L = 15; // get lowest channel L

    _spec_param.performance_threshold.snr = 13;

}


Perception::sptr Perception::make() {
    return std::make_shared<Perception>();
}

//on msg
void Perception::on_msg(const std::string &id, void *data, size_t size) {
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

    if (msg_recv->id() == nullptr) {
        LOG(ERROR) << "message: " << msg_recv->any_type() << " have no id";
        return;
    }

    auto src = msg_recv->id()->str();

    switch (msg_recv->any_type()) {
        case EasyConnect::DataAny_NONE:
        case EasyConnect::DataAny_SpectrumData:
        case EasyConnect::DataAny_TestData:
            break;
        case EasyConnect::DataAny_CommandFrame: {
            auto cmd = (EasyConnect::CommandFrame *) msg_recv->any();
            this->handle_command(src, cmd);
            break;
        }
        case EasyConnect::DataAny_FileFrame: {
            auto file = msg_recv->any_as_FileFrame();
            if (file == nullptr)break;
            auto filename = file->name()->str();
            auto pos = file->index();
            auto total = file->total();
            auto *data = file->data()->data(); //this is a (uint8_t*)
            auto size = file->size();
            break;
        }
    }

}

void Perception::handle_command(const std::string &id, EasyConnect::CommandFrame *cmd) {
    switch (cmd->cmd()) {
        case EasyConnect::CommandType_Stop: {
            this->stop_sweeper();
            //send ack
            flatbuffers::FlatBufferBuilder builder;
            auto ack = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Stop, true);
            builder.Finish(ack);
            auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, ack.Union(),
                                                  builder.CreateString(_dev.Perception));
            builder.Finish(msg);
            this->_conn->send_to(id, builder.GetBufferPointer(), builder.GetSize());
            break;
        }
        case EasyConnect::CommandType_None:
            break;
        case EasyConnect::CommandType_Start: {
            LOG(INFO) << "start command.";
            this->start_sweeper();
            //send ack
            flatbuffers::FlatBufferBuilder builder;
            auto ack = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Start, true);
            builder.Finish(ack);
            auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, ack.Union(),
                                                  builder.CreateString(_dev.Perception));
            builder.Finish(msg);
            this->_conn->send_to(id, builder.GetBufferPointer(), builder.GetSize());
            break;
        }
        case EasyConnect::CommandType_SetParam: {
            if (cmd->data_type() != EasyConnect::CommandData_ParameterFrame)break;
            auto *parameter = (EasyConnect::ParameterFrame *) cmd->data();
            this->handle_set_param(parameter);
            break;
        }
        case EasyConnect::CommandType_GetParam:
            break;
        case EasyConnect::CommandType_Chat: {
            if (cmd->data_type() != EasyConnect::CommandData_StringFrame)break;
            auto chat_str = cmd->data_as_StringFrame()->content()->str();
            LOG(INFO) << "chat string: " << chat_str;
            break;
        }
        case EasyConnect::CommandType_NtpSync: {
//            if (cmd->data_type() != EasyConnect::CommandData_StringFrame) {
//                LOG(ERROR) << "ntp sync data error";
//                break;
//            }
//            auto *str = cmd->data_as_StringFrame();
            if (cmd->is_ack())break;
            this->sync_time(id, "");
            break;
        }
        case EasyConnect::CommandType_NtpSyncErr:
        case EasyConnect::CommandType_Exception:
        case EasyConnect::CommandType_Hop:
            break;
        case EasyConnect::CommandType_TestTime: {
            if (cmd->data_type() == EasyConnect::CommandData_StringFrame) {
                auto str = cmd->data_as_StringFrame()->content()->str();//get std::string


            }

            if (cmd->data_type() == EasyConnect::CommandData_LongFrame) {
                auto l = cmd->data_as_LongFrame()->content(); // get uint64

            }

            break;
        }
        case EasyConnect::CommandType_CRC_CHECK:
            break;
        case EasyConnect::CommandType_SNR: {
            auto snr = (EasyConnect::SnrFrame *) cmd->data();
            _performance.at(_spec_param.current_performance_index).snr = snr->content();
            _performance.at(_spec_param.current_performance_index).tsp = snr->tsp();
            _performance.at(_spec_param.current_performance_index).syn = snr->syn();
            _spec_param.current_performance_index++;
            if (_spec_param.current_performance_index >= _spec_param.performance_count) {
                _spec_param.current_performance_index = 0;
            }

//            LOG(INFO) << "snr: " << snr->syn() << " " << snr->tsp();

            break;
        }
        case EasyConnect::CommandType_STATE:
            break;
    }
}


void Perception::handle_set_param(EasyConnect::ParameterFrame *parameter) {
    LOG(INFO) << parameter->rx_freq();
    LOG(INFO) << parameter->rx_gain();
}

void Perception::start_conn(short port) {
    EasyConnectIface::Config config;
    config.id = _dev.Perception;
    config.udp_port = port;
    _conn = EasyConnectIface::make(config,
                                   std::bind(&Perception::on_msg, this, std::placeholders::_1, std::placeholders::_2,
                                             std::placeholders::_3));
    _conn->create_broker();

    _state_thread = std::make_shared<std::thread>(std::bind(&Perception::state_working, this));
}

void Perception::start_sweeper() {
    if (!_usrp) {
        this->init();
    }
    try {
        this->stop_sweeper();


        _recv_running = true;
        _recv_thread = std::make_shared<std::thread>(std::bind(&Perception::recv_working, this));
        _tsp_running = true;
        _tsp_thread = std::make_shared<std::thread>(std::bind(&Perception::tsp_working, this));
    } catch (std::exception &e) {
        LOG(FATAL) << e.what();
    }
}

void Perception::tsp_working() { //发送时间进程
    while(_tsp_running){
        uhd::time_spec_t current_tsp = _usrp->get_time_now();

        this->send_tsp(time_spec_to_ms(current_tsp));

        msleep(10);
    }

}


void Perception::recv_working() {
//    _usrp->set_time_now(uhd::time_spec_t{0, 0});
//    auto now = std::chrono::high_resolution_clock::now();
//    auto duration = now.time_since_epoch();
//    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
//    LOG(INFO) << millis;
//    double full_secs;
//    double real_secs = (double) millis / 1000.0;
//    double frac_secs = std::modf(real_secs, &full_secs);

//    _usrp->set_time_now(uhd::time_spec_t((time_t) full_secs, frac_secs));
//
//    _usrp->set_time_next_pps(uhd::time_spec_t((time_t) full_secs + 1, 0));
//
//    LOG(INFO) << "time spec: " << (time_t) full_secs << " " << frac_secs;


    LOG(INFO) << "init fft";
    //init fft
    auto fft = terjin::DFT::make();
    fft->make(_spec_param.fft_size, 0, false);

    //init buffer
    LOG(INFO) << "init buffer";
    std::vector<std::complex<short> > buffer(_spec_param.rx_len);

    uhd::stream_args_t streamArgs{"sc16", "sc16"};
    streamArgs.channels = {0};
    auto rx_stream = _usrp->get_rx_stream(streamArgs);


    while (_recv_running) {
        clock_t begin,end;
        begin=clock();
        uhd::time_spec_t current = _usrp->get_time_now();

        uhd::time_spec_t data_time =
                std::ceil(current.get_real_secs() / _spec_param.interval_s) * _spec_param.interval_s;


        uhd::stream_cmd_t cmd{uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE};
        cmd.stream_now = false;
        cmd.time_spec = data_time;
        cmd.num_samps = _spec_param.rx_len;
        rx_stream->issue_stream_cmd(cmd);
        size_t acc_num = 0;
        double timeout = 1.0;
        uhd::rx_metadata_t md;

        uhd::time_spec_t timeSpec{0, 0};
        while (acc_num < _spec_param.rx_len) {
            size_t n = std::min((size_t) 2040, _spec_param.rx_len - acc_num);
            size_t recv_num;
            try {
                recv_num = rx_stream->recv(&buffer.at(acc_num), n, md, timeout);
            } catch (std::exception &e) {
                _recv_running = false;
                _iface.reset();
                _usrp.reset();
                LOG(INFO) << e.what();
                break;
            }
            if (md.error_code != md.ERROR_CODE_NONE) {
                LOG(ERROR) << md.strerror();
                break;
            }

            if (timeSpec.get_full_secs() == 0) {
                timeSpec = md.time_spec;
            }

            acc_num += recv_num;
        }


        int group = _spec_param.rx_len / _spec_param.fft_size;
        std::vector<double> spec(_spec_param.fft_size, 0);

        for (int j = 0; j < group; ++j) {

            fft->input(&buffer.at(j * _spec_param.fft_size));
            fft->excute();
            fft->shift();
            auto output = fft->output();

            for (int k = 0; k < _spec_param.fft_size; ++k) {
                spec[k] += std::abs(output[k]);
            }

        }

        for (int k = 0; k < _spec_param.fft_size; ++k) {
            spec[k] = 10 * log10(spec[k]);
        }

        //send spectrum to client
//        static auto plot = terjin::GNUPlotIface::make();
//        plot->plot(spec.data(), spec.size());
        this->send_spectrum(spec.data(), spec.size());


//        end = clock();
//        float time = float(end-begin)/float( CLOCKS_PER_SEC);
//        LOG(INFO)<<"time is "<<time;
//        LOG(INFO)<<"Send Spectrum  "<<"Current tsp is "<<time_spec_to_ms(current_tsp);

        msleep(45);

    }
}

uint64_t Perception::time_spec_to_ms(const uhd::time_spec_t &timeSpec) {
    return timeSpec.get_full_secs() * 1000 + timeSpec.get_frac_secs() * 1000.0;
}

void Perception::stop_sweeper() {
    LOG(INFO) << "stop sweeper";
    _recv_running = false;
    if (_recv_thread && _recv_thread->joinable()) {
        _recv_thread->join();
    }
    _tsp_running = false;
    if (_tsp_thread && _tsp_thread->joinable()) {
        _tsp_thread->join();
    }

    LOG(INFO) << "stop ok";
}


Perception::~Perception() {
    this->stop_sweeper();

    _loop = false;

    if (_state_thread && _state_thread->joinable())_state_thread->join();
    LOG(INFO) << "~Perception()";
}


void Perception::send_tsp(uint64_t tsp){

    //LOG(INFO)<<"tsp";
    flatbuffers::FlatBufferBuilder builder;
    auto tsp_frame = EasyConnect::CreateHopFrame(builder,tsp,838e6);
    builder.Finish(tsp_frame);

    auto command_frame = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Hop, false,
                                                         EasyConnect::CommandData_HopFrame, tsp_frame.Union());
    builder.Finish(command_frame);

    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, command_frame.Union(),
                                          builder.CreateString(_dev.Perception));

    builder.Finish(msg);

    _conn->send_to(_dev.Strategy, builder.GetBufferPointer(), builder.GetSize());
    //_conn->send_to(_dev.Receiver, builder.GetBufferPointer(), builder.GetSize());
};

void Perception::send_spectrum(const double *data, size_t size) {
    //LOG(INFO)<<"Send Spectrum  ";
    if (!_conn)return;

    std::vector<short> temp(size);
    for (size_t j = 0; j < size; ++j) {
        temp[j] = (short) std::round(data[j] * 10);
    }


    flatbuffers::FlatBufferBuilder builder;
    auto spectrum = EasyConnect::CreateSpectrumData(builder, _spec_param.rx_freq, _spec_param.rx_rate, 0,
                                                    _spec_param.fft_size,
                                                    builder.CreateVector(temp.data(), temp.size()));
    builder.Finish(spectrum);


    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_SpectrumData, spectrum.Union(),
                                          builder.CreateString(_dev.Perception));
    builder.Finish(msg);

    _conn->send_to(_dev.Client, builder.GetBufferPointer(), builder.GetSize());
    //_conn->send_to(_dev.Disturb, builder.GetBufferPointer(), builder.GetSize());
    _conn->send_to(_dev.Strategy, builder.GetBufferPointer(), builder.GetSize());
    _conn->send_to("GUIJammer", builder.GetBufferPointer(), builder.GetSize());

    builder.Reset();
}

void Perception::send_hop_command(uint64_t tsp, double hop_freq) {
    flatbuffers::FlatBufferBuilder builder;
    auto hop_frame = EasyConnect::CreateHopFrame(builder, tsp, hop_freq);
    builder.Finish(hop_frame);

    auto command_frame = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Hop, false,
                                                         EasyConnect::CommandData_HopFrame, hop_frame.Union());
    builder.Finish(command_frame);

    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, command_frame.Union(),
                                          builder.CreateString(_dev.Perception));

    builder.Finish(msg);

    _conn->send_to(_dev.Transmitter, builder.GetBufferPointer(), builder.GetSize());
    _conn->send_to(_dev.Receiver, builder.GetBufferPointer(), builder.GetSize());

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

bool Perception::sync_time(const std::string &id, const std::string &ip) {
    LOG(INFO) << "sync time to GPS";

    if (not _usrp) {
        this->init();
    }

    int status = 0;
    try {
        auto locate = _iface->get_locate_data();
        status = locate.status;
    } catch (std::exception &e) {

    } catch (...) {
    }

    if (status == 0) {
        LOG(INFO) << "GPS not lock";
    }

    _usrp->set_time_now(uhd::time_spec_t{0, 0});
    auto now = _iface->get_gps_time();

    LOG(INFO) << "gps now: " << now.get_full_secs() << " " << now.get_frac_secs();

    time_t t = now.get_full_secs();
    LOG(INFO) << std::ctime(&t);

    _usrp->set_time_next_pps(now.get_full_secs() + 1);

    uhd::time_spec_t fpga_time;
    for (int l = 0; l < 10; ++l) {

        std::this_thread::sleep_for(std::chrono::milliseconds(180));
        fpga_time = _usrp->get_time_now();
        LOG(INFO) << "" << fpga_time.get_full_secs() << " " << fpga_time.get_frac_secs();

    }


    auto locate = _iface->get_locate_data();
    LOG(INFO) << "num: " << locate.num;
    LOG(INFO) << "gps status: " << locate.status;
    LOG(INFO) << "location: " << locate.longitude << "," << locate.latitude << "," << locate.altitude;

    send_sync_ack(fpga_time.get_full_secs() * 1000, _dev.Client, _conn, _dev.Perception);
//    const std::string err_str = "Set system time failed, please try again via 'sudo'";
//    auto *ntp_client = new NTPclient(ip);
//    int ret;
//    for (int j = 0; j < 10; ++j) {
//        ret = ntp_client->SetSystemTime();
//        if (ret == 0) {
//            LOG(INFO) << "ntp ok";
//            send_ntp_sync_ack(id, _conn, _dev.Perception); //send ack
//            return;
//        }
//
//        if (ret == -999) {
//            send_error_str_as_ntp_sync_err(id, err_str, _conn, _dev.Perception);
//            return;
//        }
//
//        std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    }


    auto now_epoch = _usrp->get_time_now().get_full_secs();
    auto cpu_time = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    LOG(INFO) << "now epoch: " << now_epoch;
    LOG(INFO) << "cpu epoch: " << cpu_time;
    if (std::abs(now_epoch - cpu_time) < 3600) {
        return true;
    }

}

void Perception::send_exception(const std::string &id, const std::string &str) {
    flatbuffers::FlatBufferBuilder builder;
    auto err = EasyConnect::CreateStringFrame(builder, builder.CreateString(str));
    builder.Finish(err);
    auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_Exception, false,
                                               EasyConnect::CommandData_StringFrame, err.Union());
    builder.Finish(cmd);
    auto msg = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                          builder.CreateString(_dev.Perception));
    builder.Finish(msg);

    _conn->send_to(id, builder.GetBufferPointer(), builder.GetSize());
}

void Perception::broadcast_broker(const std::string &host) {
    _conn->broadcast_host(host);
}

void Perception::send_msg(const std::string &id, void *data, size_t size) {
    _conn->send_to(id, data, size);
}

void Perception::reset_usb() {
    auto usb = terjin::USB::make();
    usb->reset_usb(0x2500, 0x0020);

}

void Perception::init() {
    _usrp = uhd::usrp::multi_usrp::make(std::string(""));
    _usrp->set_rx_subdev_spec(std::string("A:A"));

    _usrp->set_time_source("gpsdo");
    _usrp->set_clock_source("gpsdo");

    _iface = terjin_iface::make(_usrp);

    _iface->set_recv_mode(terjin_iface::recv_mode_t::recv_mode_normal);

    _usrp->set_rx_gain(_spec_param.gain);
    _usrp->set_rx_rate(_spec_param.rx_rate);
    _usrp->set_rx_freq(_spec_param.rx_freq);


    ////Check for 10 MHz lock
    bool ref_locked = false;
    for (int i = 0; i < 10 and not ref_locked; i++) {
        LOG(INFO) << "Waiting for reference lock...";
        ref_locked = _usrp->get_mboard_sensor("ref_locked", 0).to_bool();
        if (not ref_locked) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

void Perception::state_working() {
    while (_loop) {
        if (_conn) {
            flatbuffers::FlatBufferBuilder builder;
            auto cmd = EasyConnect::CreateCommandFrame(builder, EasyConnect::CommandType_STATE);
            builder.Finish(cmd);
            auto ms = EasyConnect::CreateMessage(builder, EasyConnect::DataAny_CommandFrame, cmd.Union(),
                                                 builder.CreateString(_dev.Perception));
            builder.Finish(ms);

            _conn->send_to(_dev.Client, builder.GetBufferPointer(), builder.GetSize());
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
