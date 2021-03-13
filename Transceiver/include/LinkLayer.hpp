//
// Created by locate on 2019/11/21.
//

#ifndef LINKLAYER_HPP
#define LINKLAYER_HPP

#include <string>
#include <memory>
#include <thread>
#include <functional>
#include <list>
#include <mutex>
#include <condition_variable>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <ctime>
#include <boost/algorithm/string.hpp>

#include "Types.hpp"
#include "Frame.hpp"
#include "PhysicsLayer.hpp"
#include "NTPclient.h"

#include <easy_connect/EasyConnect.hpp>
#include <easy_connect/MessageFrame.hpp>
#include <easy_connect_generated.h>
#include <jsoncpp/json/value.h>
#include <uhd/terjin_iface.hpp>

class LinkLayer {
public:
    typedef std::function<void(const std::list<LinkFrame> &)> callback_t;//回调函数
private:

    int _MTU = 150;

    struct LinkParams {
        int max_retransmission_times = 50;//每包发送数据的最大重传次数
        int retransmission_gap = 2000;//每包发送数据的重传间隔 单位ms
    };

    struct PhysicsParams {
        std::complex<double> multiple = 8192.0;//发送数据的幅度×_multiple
        std::vector<std::complex<double>> syn;//同步序列
        bool reverse_syn = false;//相关运算时需要将本地的syn是否需要翻转

        double tx_freq = 838e6;
        double rx_freq = 838e6;
        double sampling_rate = 270e3;
        double clk_rate = 3.456e+07;
        double tx_gain = 50;
        double rx_gain = 50;

        int delay = 100;//ms 第一次启动发送后再等待的时间
        int TSC = 2;//The training sequence [0..7]
        int insert_tsc_per_bytes = 8;//插入training sequence的间隔 单位:字节
        int tx_fifo_size = 1024;//

        //配置文件读取
        double tx_lo_off = 0.0;
        double rx_lo_off = 0.0;
        std::string tx_antenna{};
        std::string rx_antenna{};
        std::string tx_subdev_spec{};
        std::string rx_subdev_spec{};
    };

    LinkParams _link_params;

    PhysicsParams _physics_params;

    std::string _device_id = "transmitter";//本地设备id
    int _udp_port = 8773;

    EasyConnectIface::sptr _trans;//与其他设备的通信接口

    struct {
        std::string gui = "GUI";
        std::string preception = "perception";
    } _Receiver;


    std::string _ntp_server_ip = "120.25.115.20";//NTP服务器IP
    std::shared_ptr<NTPclient> _ntp_client;//NTP客户端
    bool _use_pps = false;

    std::string _send_file_dir = "./";//当前目录
    std::string _recv_file_dir = "./";//当前目录

    std::string _dev_addr = "";//设备地址
    UHDDevice::sptr _dev;//设备对象

    std::list<LinkFrame> _str_fifo;//暂存尚未接收到的ack的发送数据
    std::mutex _sf_mtx;

    std::list<LinkFrame> _file_fifo;//暂存尚未接收到的ack的发送数据
    std::mutex _ff_mtx;

    int _send_file_size = 0;
    std::mutex _sfs_mtx;

    int _send_str_ordinal = 1;//发送序号 类型chat
    int _send_file_ordinal = 1;//发送序号 类型file
    int _retrans_str_ordinal = 1;//当前重传序号 类型chat
    int _retrans_file_ordinal = 1;//当前重传序号 类型file

    std::vector<FileFrame> _recv_file;//存储接收到的文件数据
    int _file_end_ordinal = 0; //当前接收到的文件数据的最后一包序号

    PhysicsLayer::sptr _physics;//物理层对象

    bool _loop = false;//control _retrans_thread and _program_state_thread
    std::shared_ptr<std::thread> _retrans_thread;//重传线程
    std::shared_ptr<std::thread> _program_state_thread;//心跳线程
    std::shared_ptr<std::thread> _throughput_thread;//吞吐量线程

    callback_t _cb;//消息回调

    bool _make = false;//LinkLayer初始化完成标志

    bool _start_flag = false;//_retrans_thread开始标志
    bool _work_ready = false;//_retrans_thread will sleep until work is ready
    std::mutex _retrans_thread_mtx;//use with _cv
    std::condition_variable _cv;//_cv will wait until _mtx_cv is unlock

    //获取系统当前时间
    time_t _nowtime{};
    char _deal_time[64] = {};

    terjin_iface::sptr _iface;

    //2020/8/13 新增变量
    uint _ackCount = 0;//记录单位时间内的ack的数量
    uint _nckCount = 0;//记录单位时间内的nck的数量
    uint _ackCount_GUI = 0;//记录单位时间内的ack的数量
    uint _nckCount_GUI = 0;//记录单位时间内的nck的数量
    std::mutex _mtxCk;

public:
    typedef std::shared_ptr<LinkLayer> sptr;

    explicit LinkLayer(const Json::Value &value);

    ~LinkLayer();

    static sptr make(const Json::Value &value);

    /**
     * 退出线程
     */
    void Stop();

    /**
     * for test
     * @param host
     */
    void BroadcastBroker(const std::string &host) { _trans->broadcast_host(host); }

    /**
     * 处理开始帧
     * @return if success returns true
     */
    bool CopeStartCommand();

    /**
     * 处理停止帧
     */
    void CopeStopCommand();

    void SendMsg(const std::string &id, void *data, size_t size);

private:

    /**
     * create device
     * @param value
     * @return if success returns true
     */
    bool CreateDevice(const Json::Value &value);

    /**
     * 重置参数
     */
    void ResetSetParam();

    /**
     * 与NTP服务器同步时间
     * @return if success returns true
     */
    bool NTPTimeSync();

    /**
     * 绑定消息的回调
     * @param cb
     */
    void BindCallback(callback_t cb);

    /**
     * 展示当前参数
     */
    void DisplayParameters();

    /**
     * 开启线程
     */
    void Start();

    /**
     * 回调函数，其他设备发来的二进制数据将在这里处理
     * @param id 发送端的身份信息
     * @param data 二进制数据的地址
     * @param size 二进制数据的大小
     */
    void OnMsg(const std::string &id, void *data, size_t size);

    /**
     * 处理命令函数
     * @param id 发送者
     * @param cmd 命令
     */
    void HandleCommand(const std::string &id, EasyConnect::CommandFrame *cmd);

    /**
     * 通知线程暂停
     */
    void StopWork();

    /**
     * 通知线程执行
     */
    void StartWork();

    /**
     * 重传线程
     */
    void RetransmissionThread();

    void RetransWork();

    /**
     * 心跳线程
     */
    void ProgramStateThread();

    /**
     * 发送吞吐量
     */
    void ThoughThread();

    /**
     * 发送心跳
     */
    void SendHeart();

    /**
     * 发文件的进度条
     */
    void SendFileProgressBar();

    /**
    * 处理参数帧
    * @param params 参数
    */
    void CopeSetParamsCommand(EasyConnect::ParameterFrame *params);

    /**
     * 处理信息帧
     */
    void CopeChatCommand(std::string &msg);

    /**
     * recv messages from upper levels
     * @param msg the need to send messages
     */
    void RecvChatFromUpperLevels(std::string &msg);

    /**
     * 处理NTP时间同步帧
     * @param ip NTP服务器IP
     * @return if success returns true
     */
    bool CopeTimeSync(std::string &ip);

    /**
     * 处理跳频帧
     * @hopFrame 跳频参数
     */
    bool CopeHopCommand(EasyConnect::HopFrame *hopFrame);

    /**
     * 测试时差
     * @param tm ms
     */
    void CopeTestTime(const uint64_t &tm);

    /**
     * recv physics layer messages
     * @param input the messages from physics layer
     * @param crc_check_result the crc check result
     */
    void RecvFromPhysicsLayer(double timestamp, const BitVector &input, unsigned crc_check_result, double snr);

    /**
     * 发送CRC检验结果
     * @param crc_result CRC检验结果
     */
    void SendCRCResult(double crc_result);

    /**
    * 解析序号
    * @param input 序号的二进制比特
    * @return (整型)序号
    */
    static int ParseOrdinal(const BitVector &input);

    /**
     * 解析传输类型 chat or file
     * @param input
     * @param recv_trans_type
     * @return
     */
    static int ParseTransType(const BitVector &input, TransType &recv_trans_type);

    /**
     * 处理ACK帧
     * @param recv_trans_type 信息 or 文件
     * @param recv_ordinal 序号
     */
    void HandleAckFrame(TransType recv_trans_type, int recv_ordinal);

    /**
     * 处理chat类型的ACK帧
     * @param recv_ordinal 序号
     */
    void HandleChatAckFrame(int recv_ordinal);

    /**
     * 处理file类型的ACK帧
     * @param recv_ordinal 序号
     */
    void HandleFileAckFrame(int recv_ordinal);

    /**
     * 处理MSG帧
     * @param recv_trans_type 信息 or 文件
     * @param input bit
     * @param recv_ordinal 序号
     */
    void HandleMsgFrame(TransType recv_trans_type, const BitVector &input, int recv_ordinal);

    /**
     * 解析空格
     * @param input 含空格数的二进制比特
     * @return (整型)空格数
     */
    static int ParseSpace(const BitVector &input);

    /**
     * 解析有效数据
     * @param input 含内容的二进制比特
     * @param output (string)内容
     */
    static void ParseData(const BitVector &input, std::string &output);

    /**
     * 发送ack
     * @param recv_ordinal 当前接收到的序号
     */
    void SendACK(TransType recv_trans_type, int recv_ordinal);

    /**
     * 发送接收到信息回界面
     * @param chat
     */
    void SendChat(const std::string &chat);

    /**
     * 解析文件内容
     * @param input
     * @param recv_data
     * @param recv_ordinal
     */
    void ParseFileContext(const BitVector &input, const std::string &recv_data, int recv_ordinal);

    /**
     * 去除多余 重复的内容
     * @param vec
     * @param threshold
     */
    static void RemoveExtraElements(std::vector<FileFrame> &vec, int threshold);

    /**
     * 保存接收到的文件
     * @param filename_len
     * @param file_name
     * @return
     */
    int SaveRecvFile(int filename_len, std::string *file_name);

    /**
     * 发送异常
     * @param str 异常信息
     */
    void SendException(const std::string &str);

    /**
     * 处理文件
     * @param filename 文件名
     */
    void CopeFileCommand(const std::string &filename);

    /**
     * 读取文件内容
     * @param file_name
     * @param output
     * @return
     */
    int ReadFile(const std::string &file_name, std::string &output);

    /**
     * 将文件名和文件内容合并
     * @param name
     * @param content
     */
    static void MergeNameContent(std::string &name, std::string &content);

    /**
     * 发送文件
     * @param msg 内容
     * @param filename_len 文件名长度
     */
    void RecvFileFromUpperLevels(std::string msg, int filename_len);

    /**
     * 取消发送
     */
    void CopeCancelSendCommand();

    void RecvBinaryFileData(std::string &msg);

    void SendBinaryFile(const std::string &msg);

    void RetransWarning(LinkFrame &obj);

    //2020/8/13 新增函数
    /**
     * 发送吞吐量
     */
    void SendThroughput();
    void SendThroughput_GUI();

};


#endif //LINKLAYER_HPP
