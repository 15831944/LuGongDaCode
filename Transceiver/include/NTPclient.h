#ifndef NTP_NTPCLIENT_H
#define NTP_NTPCLIENT_H


#define JAN_1970        0x83AA7E80           /// 1900 ~ 1970 年之间的时间 秒数

#include <iostream>
#include <vector>


///NTP 时间戳。
struct ntp_timestamp_t {
    uint seconds;    ///< 从 1900年至今所经过的秒数
    uint fraction;   ///< 小数部份，单位是微秒数的4294.967296( = 2^32 / 10^6 )倍
};


///对系统的 timeval 结构体进行重定义结构体。
struct ntp_timeval {
    long tv_sec;    ///< 秒
    long tv_usec;   ///< 微秒
};


///时间描述信息结构体。
struct ntp_time_context {
    uint year   : 16;  ///< 年
    uint month  :  6;  ///< 月
    uint day    :  6;  ///< 日
    uint week   :  4;  ///< 周几
    uint hour   :  6;  ///< 时
    uint minute :  6;  ///< 分
    uint second :  6;  ///< 秒
    uint msec   : 14;  ///< 毫秒
};

/// NTP 报文格式。
struct ntp_packet {
    u_char mode;      ///< 2 bits，飞跃指示器；3 bits，版本号；3 bits，NTP工作模式
    u_char stratum;      ///< 系统时钟的层数，取值范围为1~16，它定义了时钟的准确度。层数为1的时钟准确度最高，准确度从1到16依次递减，层数为16的时钟处于未同步状态，不能作为参考时钟
    u_char poll;      ///< 轮询时间，即两个连续NTP报文之间的时间间隔
    u_char percision;      ///< 系统时钟的精度

    uint root_delay;  ///< 本地到主参考时钟源的往返时间
    uint root_dispersion;  ///< 系统时钟相对于主参考时钟的最大误差
    uint ref_indentifier;  ///< 参考时钟源的标识

    ntp_timestamp_t reference;      ///< 系统时钟最后一次被设定或更新的时间（应答完成后，用于存储 T1）
    ntp_timestamp_t originate;      ///< NTP请求报文离开发送端时发送端的本地时间（应答完成后，用于存储 T4）
    ntp_timestamp_t receive;      ///< NTP请求报文到达接收端时接收端的本地时间（应答完成后，用于存储 T2）
    ntp_timestamp_t transmit;      ///< NTP应答报文离开应答者时应答者的本地时间（应答完成后，用于存储 T3）
};



////定义一个类，对NTP进行封装以及调用

class NTPclient {

public:
    /**用户调用**/

    explicit NTPclient(std::string mip);

    ///从NTP服务器获得校准时间后,校准当前系统时钟 ,修改系统时间需要root权限!!
    int SetSystemTime();

    ///获得系统当前时间(以 us为单位，1970年1月1日到现在的时间)
    static long long GetNowSystemTime_us();

    ///获得系统当前时间的结构体
    static ntp_time_context GetNowSystemTime_context();

    ///获得服务器当前时间(校准后)(以 us为单位，1970年1月1日到现在的时间)
    long long GetNowServerTime_us();

    ///获得服务器当前时间的结构体
    ntp_time_context GetNowServerTime_context();

    ///自行获取时间戳数组T1、T2、T3、T4 (以 us为单位，1970年1月1日到现在的时间)
    int GetServerTimeStampBuf(std::vector<long long> &time_T);

    ///将 ntp_time_context 转换为 以 1us为单位的时间值（1970年1月1日到现在的时间）
    static uint64_t ntp_time_value(ntp_time_context *tm_context);

    ///将 ntp_time_context 转换为 以 1us为单位的时间值（1970年1月1日到现在的时间）
    static double ntp_time_value_double(ntp_time_context *tm_context);

protected:

    std::string _ip;    //NTP服务器


protected:

    ///获取当前系统的 时间值（以 1us为单位，1970年1月1日到现在的时间）。
    static uint64_t ntp_gettimevalue();

    ///获取当前系统的 timeval 值（1970年1月1日到现在的时间）
    static void ntp_gettimeofday(ntp_timeval &value);

    ///转换（以 1us秒 为单位的）时间值（1970年1月1日到现在的时间）为具体的时间描述信息（即 ntp_time_context）。
    static int ntp_tmctxt_bv(uint64_t xut_time, ntp_time_context &tm_context);

    ///从NTP服务端获取时间戳T1、T2、T3、T4
    static int ntp_get_time_values(const char *host, u_int16_t port, uint timeout_ms, long long T_buf[4]);

    ///初始化 NTP 的请求数据包。
    static void ntp_init_request_packet(ntp_packet *npt_dptr);

    ///将 ntp_packet 中的 网络字节序 字段转换为 主机字节序。
    static void ntp_ntoh_packet(ntp_packet *nptr);

    ///将 ntp_packet 中的 主机字节序 字段转换为 网络字节序。
    static void ntp_hton_packet(ntp_packet *nptr);

    ///将 ntp_timestamp 转换为 ntp_timeval
    static void ntp_timestamp_to_timeval(ntp_timeval *tm_timeval, const ntp_timestamp_t *tm_timestamp);

    ///将 ntp_timeval 转换为 ntp_timestamp_t
    static void ntp_timeval_to_timestamp(ntp_timestamp_t *tm_timestamp, const ntp_timeval *tm_timeval);

    /// ntp_timeval 转换成 1us为单位的值。
    static uint64_t ntp_timeval_us(ntp_timeval *tm_timeval);

    /// ntp_timeval 转换成 1us为单位的值。
    static double ntp_timeval_us_double(ntp_timeval *tm_timeval);

    ///将 ntp_timestamp_t 转换成 1us为单位的值。
    static uint64_t ntp_timestamp_us(ntp_timestamp_t *tm_timestamp);

};


#endif //NTP_NTPCLIENT_H
