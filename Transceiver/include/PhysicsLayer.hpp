//
// Created by locate on 2019/10/29.
//

#ifndef PHYSICSLAYER_HPP
#define PHYSICSLAYER_HPP

#include "Frame.hpp"
#include "UHDDevice.hpp"
#include "DetectSynSeq.hpp"
#include "ModulatorBase.hpp"
#include "Types.hpp"

#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <functional>

#include <easy_connect/EasyConnect.hpp>
#include <easy_connect/MessageFrame.hpp>
#include <easy_connect_generated.h>

//物理层
class PhysicsLayer {
public:

    //typedef 回调函数
    typedef std::function<void(double, const BitVector &, unsigned, double)> call_back_t;

    //跳频枚举
    enum HopType {
        RX,
        TX,
        ALL
    };

    //跳频参数
    struct HopParam {
        double time = 0;
        double freq = 0;
    };

    //接收包
    struct REC_BUFFER {
        uhd::time_spec_t _timestamp;
        std::vector<std::complex<short>> _buf;
    };

    //发送和接收函数的超时等待时间
    struct Timeout {
        double tx = 3.0;
        double rx = 3.0;

        void Reset() {
            tx = 3.0;
            rx = 3.0;
        }
    };

    //已调波形
    struct ModBuffer {
        std::vector<std::complex<short>> modBuf;
        PhyFrame::ST st;
    };

    struct EmptyFlag {
        size_t count = 0;
        bool empty = false;

        void Reset() {
            count = 0;
            empty = false;
        }

        EmptyFlag &operator++() {
            ++count;
            return *this;
        }
    };

private:
    call_back_t _cb;//callback function

    bool _loop = false; //true start work thread
    std::shared_ptr<std::thread> _work_thread;//work thread
    std::shared_ptr<std::thread> _hop_thread;//hop freq thread
    std::shared_ptr<Generator> _gen;//CRC polynomial Generator

    UHDDevice::sptr _dev;//usrp device
    EasyConnectIface::sptr _trans;//for communication
    DetectSynSeq::sptr _pn_detector;//detect pn
    DetectSynSeq::sptr _syn_detector;//detect syn

    int _syn_num{};//syn sequence size
    std::vector<std::complex<short>> _syn;//syn sequence

    bool _detect_pn_ready = true;
    std::vector<std::complex<short>> _pn;//pn sequence for test sync

    bool _work_ready = false;
    std::mutex _work_mtx;
    std::condition_variable _work_cv;


    bool _hop_ready = false;
    std::mutex _hop_mtx;
    std::condition_variable _hop_cv;

    HopParam _hop_param{};
    HopType _hop_type = HopType::ALL;

    bool _start_flag = false;//start flag
    bool _first_recv = true;//first recv flag

    int _iq_num{};//IQ number
    int _samples_per_package{};//all recv samples number per package

    std::mutex _mtf_mtx;//for _tx_fifo and _mod_data
    int _tx_fifo_size = 256;//tx fifo size
    std::deque<PhyFrame> _tx_fifo;//tx fifo
    std::queue<ModBuffer> _mod_data;//modulated data

    std::vector<std::complex<short>> _recv_data;//recv data
    std::vector<std::complex<short>> _valid_data;//data to be demodulated

    std::vector<std::complex<short>> _tone;//tone sequence
    std::complex<double> _multiple = 8192.0;//float change to short

    TIMESTAMP _timestamp_offset{};
    int _MTU = 150;//上层配的MTU
    int _actual_mtu{};//_MTU+额外的开销
    int _TSC = 2;//训练序列的序号
    int _insert_tsc_per_bytes = 8;//每个指定字节插入一个训练序列
    int _insert_tsc_num{};//插入训练序列的次数

    ModulatorBase::sptr _modulator;//调制器
    DemodulatorBase::sptr _demodulator;//解调器

    double _one_frame_t = 0.f;//(含同步头)一帧的时长

    TIMESTAMP _next_send_timestamp{};//下一次发送第一个样值的时间戳

    REC_BUFFER _recv_buf;

    struct {
        std::string gui = "GUI";
        std::string preception = "perception";
    } _Receiver;

    std::string _device_id = "transmitter";

    SignalType _signal_type = GMSK;

    Timeout _timeout;

    PhyFrame::ST _pst;

    std::mutex _empty_mtx;
    EmptyFlag _emptyFlag;

    //2020/8/13 新增
    std::vector<std::complex<short>> _seq0;

public:

    typedef std::shared_ptr<PhysicsLayer> sptr;

    explicit PhysicsLayer(UHDDevice::sptr &dev, const std::string &id);

    ~PhysicsLayer();

    static sptr make(UHDDevice::sptr &dev, const std::string &id);

    void Start();

    void Stop();

    void StartWork();

    void StopWork();

    /**
     * 绑定回调函数
     * @param cb
     */
    void BindCallback(call_back_t cb);

    /**
     * 设置通信对象和通信对象
     * @param trans 通信接口
     */
    void SetTrans(EasyConnectIface::sptr &trans);

    /**
     * 设置同步序列
     * @param syn 同步序列
     * @param need_reverse_syn 翻转与否
     * @param multiple ×幅度
     */
    void SetSyn(const std::vector<std::complex<double>> &syn, bool need_reverse_syn, std::complex<double> multiple);

    /**
     * 设置参数
     * @param tx_freq
     * @param rx_freq
     * @param tx_lo_off
     * @param rx_lo_off
     * @param sampling_rate
     * @param tx_gain
     * @param rx_gain
     * @param MTU
     * @param delay
     * @param TSC
     * @param insert_tsc_per_bytes
     * @param tx_fifo_size
     */
    void CopeSetParam(double tx_freq, double rx_freq, double tx_lo_off, double rx_lo_off, double sampling_rate,
                      double tx_gain, double rx_gain, int MTU, int delay, int TSC,
                      int insert_tsc_per_bytes, int tx_fifo_size);

    /**
     * 接收待调制数据
     * @param input 待调制数据
     */
    void RecvFromLinkLayer(const std::list<LinkFrame> &input);

    /**
     * 获取系统当前时间 19701月1日至今
     * @return us
     */
    static uhd::time_spec_t GetSystemTimeUsec();

    /**
     * 当频率跳变时发出去
     */
    void SendHeart();

    /**
     * 清空缓冲区数据
     */
    void ClearPhyscisData();

    /**
     * 改变频率
     * @param time 时间
     * @param freq 频率
     */
    void SetHopFreq(uint64_t time, double freq, HopType type);

    /**
     * 指定时间发送PN序列，用于测试NTP时差
     * @param ts 发送时间
     */
    void SendPNSeq(uhd::time_spec_t *ts);

    /**
     * 接收并检测PN序列
     * @param ts 发送PN序列的时间
     */
    void DetectPNSeq(uhd::time_spec_t *ts);

    bool CheckLinkDataSize();

    PhyFrame::ST GetPhyFrameST();

    void ResetEmptyFlag();

    bool GetEmptyFlag();

private:

    /**
     * 设置PN序列
     */
    void SetPN();

    /**
     * 设置设备参数
     * @param tx_freq
     * @param rx_freq
     * @param tx_lo_off
     * @param rx_lo_off
     * @param sampling_rate
     * @param tx_gain
     * @param rx_gain
     */
    void SetDevParam(double tx_freq, double rx_freq, double tx_lo_off, double rx_lo_off, double sampling_rate,
                     double tx_gain, double rx_gain);

    /**
     * 生成单音信号
     * @param fs 单音信号的频率
     */
    void SetTone(double fs);

    /**
     * 设置启动收发时的时延
     * @param delay
     */
    void SetTSOffset(int delay);

    /**
     * 计算
     * @param insert_tsc_per_bytes
     */
    void CalculateIqSamples(int insert_tsc_per_bytes);

    /**
     * 设置缓冲区
     */
    void SetFifo();

    /**
     * 收发线程
     */
    void Work();

    /**
     * 接收数据
     * @return 接收数据大小
     */
    size_t PullBuffer();

    /**
     * 初始化发送缓冲区
     * @param timestamp
     */
    void InitSendFifo(TIMESTAMP timestamp);

    /**
     * 处理接收的数据
     */
    void CopeRecvData();

    /**
     * 发送SNR结果
     * @param snr
     */
    void SendSNR(double snr, uint64_t tsp, bool detect_syn);

    /**
     * 发射数据
     * @return 发数据大小
     */
    size_t PushBuffer();

    /**
     * 重置发送缓冲区
     * @param timestamp
     */
    void ResetSendFifoTimestamp(TIMESTAMP timestamp);

    /**
     * 更新发送缓冲区
     */
    void PopPushFrame();

    /**
     * 计算时间戳
     * @return
     */
    TIMESTAMP CalculateNextAddSendFrameTimestamp();

    void InsertIQDataToSendFifo(const std::vector<ModBuffer> &iq_data);

    /**
     * 跳频线程
     */
    void HopThread();

    /**
     * 跳频函数
     */
    void FreqHopping();

    /**
     * 发送异常
     * @param str 异常信息
     */
    void SendException(const std::string &str);

    void SendCapacity(bool full_load, double capacity);

    //2020/8/13 新增
    void WriteSeq0(std::complex<short> *buf, size_t size);

    void ReadSeq0();
};

#endif //PHYSICSLAYER_HPP