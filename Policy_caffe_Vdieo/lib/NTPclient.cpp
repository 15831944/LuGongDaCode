
#include <sys/time.h>
#include <netinet/in.h>
#include <cstring>
#include <arpa/inet.h>
#include <zconf.h>
#include <sstream>
#include <utility>

#include <glog/logging.h>

#include "NTPclient.h"


NTPclient::NTPclient(std::string mip) {
    _ip = std::move(mip);
}


/**
 *  从NTP服务器获得校准时间后,校准当前系统时钟, 修改系统时间需要root权限!!
 *  失败 返回-1
 */
int NTPclient::SetSystemTime() {
    //初始化4个用来装放T1234时间戳的容器
    std::vector<long long> time_stamp(4, 0);
    //从NTP服务器获得时间戳
    auto err = GetServerTimeStampBuf(time_stamp);

    if (err != 0) {
        return err;
    }

    //计算得校准好时间
    auto now_time = time_stamp[3] + ((time_stamp[1] - time_stamp[0]) + (time_stamp[2] - time_stamp[3])) / 2;

    struct timeval tv{};
    tv.tv_sec = now_time / 1000000ULL;  //秒
    tv.tv_usec = now_time % 1000000ULL;//微秒

    //修改系统时间，需要root权限
    if (settimeofday(&tv, nullptr) < 0) {
//        LOG(INFO) << "修改系统时钟失败";
        return -999;
    }

    return 0;
}


/**
 *  获得系统当前时间(以 us为单位，1970年1月1日到现在的时间)
 */
long long NTPclient::GetNowSystemTime_us() {
    return ntp_gettimevalue();
}


/**
 *  获得系统当前时间的结构体
 */
ntp_time_context NTPclient::GetNowSystemTime_context() {
    ntp_time_context time_context{};
    //将时间戳转换成结构体返回
    ntp_tmctxt_bv(ntp_gettimevalue(), time_context);

    std::stringstream ss1;
    ss1 << "系统当前时间："
        << time_context.year << "-"
        << time_context.month << "-"
        << time_context.day << "  "
        << time_context.hour << ":"
        << time_context.minute << ":"
        << time_context.second << "."
        << time_context.msec << "'";

    LOG(INFO) << ss1.str();

    return time_context;
}

/**
 *  获得服务器当前时间(校准后)(以 us为单位，1970年1月1日到现在的时间)
 *  失败 返回 -1
 */
long long NTPclient::GetNowServerTime_us() {
    //初始化4个用来装放T1、T2、T3、T4时间戳的容器
    std::vector<long long> time_stamp(4, 0ULL);
    //从NTP服务器获得时间戳
    auto err = GetServerTimeStampBuf(time_stamp);

    if (err == -1) {
        perror("请求失败..");
        return err;
    }

    //计算得校准好时间
    auto server_now_time = time_stamp[3] + ((time_stamp[1] - time_stamp[0]) + (time_stamp[2] - time_stamp[3])) / 2;

    return server_now_time;
}


/**
 *  获得服务器当前时间的结构体
 *  失败 返回 空结构体（全0）
 */
ntp_time_context NTPclient::GetNowServerTime_context() {
    //服务器当前时间的结构体
    ntp_time_context time_context{};
    //初始化4个用来装放T1234时间戳的容器
    std::vector<long long> time_stamp(4, 0ULL);
    //从NTP服务器获得时间戳
    auto err = GetServerTimeStampBuf(time_stamp);

    if (err == -1) {
        perror("请求失败");
        return time_context;
    }

    //计算得校准好时间
    auto server_now_time = time_stamp[3] + ((time_stamp[1] - time_stamp[0]) + (time_stamp[2] - time_stamp[3])) / 2;

    //将时间戳转换成结构体返回
    ntp_tmctxt_bv(server_now_time, time_context);

    return time_context;

}


/**
 *  从服务器获取时间戳数组T1、T2、T3、T4 (以 us为单位，1970年1月1日到现在的时间)
 *  失败 返回 -1  ；成功 返回 0
 */
int NTPclient::GetServerTimeStampBuf(std::vector<long long> &time_T) {
    if (time_T.size() < 4)
        return -1;

    int it_err = -1;
    long long T[4];
    //从服务器获得时间戳T1、T2、T3、T4    ,3秒超时
    it_err = ntp_get_time_values(_ip.c_str(), 123, 3000, T);

    if (it_err != 0) {
//        LOG(INFO) << _ip.c_str() << "请求失败，可能是因为应答超时...error code: " << it_err;
        return it_err;
    }

    time_T[0] = T[0];
    time_T[1] = T[1];
    time_T[2] = T[2];
    time_T[3] = T[3];

    return 0;
}








/************************************私有函数**********************************************/
/************************************私有函数**********************************************/

///将 ntp_timeval 转换成 timestamp
void NTPclient::ntp_timeval_to_timestamp(ntp_timestamp_t *tm_timestamp, const ntp_timeval *const tm_timeval) {
    const double lft_frac_per_ms = 4.294967296E6;  // 2^32 / 1000

    tm_timestamp->seconds = (uint) (tm_timeval->tv_sec + JAN_1970);
    tm_timestamp->fraction = (uint) ((double) tm_timeval->tv_usec / 1000.0 * lft_frac_per_ms);
}


///将 timestamp转换成 ntp_timeval
void NTPclient::ntp_timestamp_to_timeval(ntp_timeval *tm_timeval, const ntp_timestamp_t *const tm_timestamp) {
    const double xlft_frac_per_ms = 4.294967296E6;  // 2^32 / 1000

    if (tm_timestamp->seconds >= JAN_1970) {
        tm_timeval->tv_sec = (long) (tm_timestamp->seconds - JAN_1970);
        tm_timeval->tv_usec = (long) (tm_timestamp->fraction / xlft_frac_per_ms * 1000.0);
    } else {
        tm_timeval->tv_sec = 0;
        tm_timeval->tv_usec = 0;
    }
}


/// ntp_timeval 转换成 1us为单位的值。
uint64_t NTPclient::ntp_timeval_us(ntp_timeval *tm_timeval) {
    return (1000000ULL * tm_timeval->tv_sec + 1ULL * tm_timeval->tv_usec);
}

/// ntp_timeval 转换成 1us为单位的值。
double NTPclient::ntp_timeval_us_double(ntp_timeval *tm_timeval) {
    return ((double) tm_timeval->tv_sec + (double) tm_timeval->tv_usec / 1000000.f);
}


///将 ntp_timestamp_t 转换成 100 us为单位的值。
uint64_t NTPclient::ntp_timestamp_us(ntp_timestamp_t *const tm_timestamp) {
    ntp_timeval mt_timeval{};
    ntp_timestamp_to_timeval(&mt_timeval, tm_timestamp);
    return ntp_timeval_us(&mt_timeval);
}


/**
 * 返回当前系统的 时间值（以 100 us为单位，1970年1月1日到现在的时间）。
 */
uint64_t NTPclient::ntp_gettimevalue() {
    struct timeval tmval{};
    gettimeofday(&tmval, nullptr);

    return (1000000ULL * tmval.tv_sec + 1ULL * tmval.tv_usec);

}


/**
 * 获取当前系统的 timeval 值（1970年1月1日到现在的时间）。
 */
void NTPclient::ntp_gettimeofday(ntp_timeval &tm_value) {
    struct timeval tmval{};
    gettimeofday(&tmval, nullptr);

    tm_value.tv_sec = tmval.tv_sec;
    tm_value.tv_usec = tmval.tv_usec;
}


/**
 *  将 ntp_time_context 转换为 以 1us为单位的时间值（1970年1月1日到现在的时间）。
 */
uint64_t NTPclient::ntp_time_value(ntp_time_context *tm_context) {
    uint64_t ut_time = 0ULL;

    struct tm xtm_system{};
    ntp_timeval tm_value{};

    xtm_system.tm_sec = tm_context->second;
    xtm_system.tm_min = tm_context->minute;
    xtm_system.tm_hour = tm_context->hour;
    xtm_system.tm_mday = tm_context->day;
    xtm_system.tm_mon = tm_context->month - 1;
    xtm_system.tm_year = tm_context->year - 1900;
    xtm_system.tm_wday = 0;
    xtm_system.tm_yday = 0;
    xtm_system.tm_isdst = 0;

    tm_value.tv_sec = mktime(&xtm_system);
    tm_value.tv_usec = tm_context->msec * 1000;
    if (-1 != tm_value.tv_sec) {
        ut_time = ntp_timeval_us(&tm_value);
    }

    return ut_time;
}

/**
 *  将 ntp_time_context 转换为 以 1us为单位的时间值（1970年1月1日到现在的时间）。
 */
double NTPclient::ntp_time_value_double(ntp_time_context *tm_context) {
    double ut_time = 0;

    struct tm xtm_system{};
    ntp_timeval tm_value{};

    xtm_system.tm_sec = tm_context->second;
    xtm_system.tm_min = tm_context->minute;
    xtm_system.tm_hour = tm_context->hour;
    xtm_system.tm_mday = tm_context->day;
    xtm_system.tm_mon = tm_context->month - 1;
    xtm_system.tm_year = tm_context->year - 1900;
    xtm_system.tm_wday = 0;
    xtm_system.tm_yday = 0;
    xtm_system.tm_isdst = 0;

    tm_value.tv_sec = mktime(&xtm_system);
    tm_value.tv_usec = tm_context->msec * 1000;
    if (-1 != tm_value.tv_sec) {
        ut_time = ntp_timeval_us_double(&tm_value);
    }

    return ut_time;
}


/**
 *   转换（以 us为单位的）时间值（1970年1月1日到现在的时间）为具体的时间描述信息（即 ntp_time_context）。
 *
 * @param [in ] xut_time    : 时间值（1970年1月1日到现在的时间）。
 * @param [out] tm_context : 操作成功返回的时间描述信息。
 *
 *   成功，返回 1；
 */
int NTPclient::ntp_tmctxt_bv(uint64_t ut_time, ntp_time_context &tm_context) {

    struct tm xtm_system{};
    auto tm_time = (time_t) (ut_time / 1000000ULL);
    localtime_r(&tm_time, &xtm_system);

    tm_context.year = xtm_system.tm_year + 1900;
    tm_context.month = xtm_system.tm_mon + 1;
    tm_context.day = xtm_system.tm_mday;
    tm_context.week = xtm_system.tm_wday;
    tm_context.hour = xtm_system.tm_hour;
    tm_context.minute = xtm_system.tm_min;
    tm_context.second = xtm_system.tm_sec;
    tm_context.msec = (uint) ((ut_time % 1000000ULL) / 1000L);

    return 1;
}


/**
 *  初始化 NTP 的请求数据包。
 */
void NTPclient::ntp_init_request_packet(ntp_packet *npt_dptr) {
    const u_char xct_leap_indicator = 0;
    const u_char xct_ntp_version = 3;
    const u_char ntp_mode = 3;  //ntp_mode_client

    npt_dptr->mode = (xct_leap_indicator << 6) | (xct_ntp_version << 3) | (ntp_mode << 0);
    npt_dptr->stratum = 0;
    npt_dptr->poll = 4;
    npt_dptr->percision = ((-6) & 0xFF);

    npt_dptr->root_delay = (1 << 16);
    npt_dptr->root_dispersion = (1 << 16);
    npt_dptr->ref_indentifier = 0;

    npt_dptr->reference.seconds = 0;
    npt_dptr->reference.fraction = 0;
    npt_dptr->originate.seconds = 0;
    npt_dptr->originate.fraction = 0;
    npt_dptr->receive.seconds = 0;
    npt_dptr->receive.fraction = 0;
    npt_dptr->transmit.seconds = 0;
    npt_dptr->transmit.fraction = 0;
}


/**
 * 将 ntp_packet 中的 网络字节序 字段转换为 主机字节序。
 */
void NTPclient::ntp_ntoh_packet(ntp_packet *nptr) {
    nptr->root_delay = ntohl(nptr->root_delay);
    nptr->root_dispersion = ntohl(nptr->root_dispersion);
    nptr->ref_indentifier = ntohl(nptr->ref_indentifier);
    nptr->reference.seconds = ntohl(nptr->reference.seconds);
    nptr->reference.fraction = ntohl(nptr->reference.fraction);
    nptr->originate.seconds = ntohl(nptr->originate.seconds);
    nptr->originate.fraction = ntohl(nptr->originate.fraction);
    nptr->receive.seconds = ntohl(nptr->receive.seconds);
    nptr->receive.fraction = ntohl(nptr->receive.fraction);
    nptr->transmit.seconds = ntohl(nptr->transmit.seconds);
    nptr->transmit.fraction = ntohl(nptr->transmit.fraction);
}


/**
 * 将 ntp_packet 中的 主机字节序 字段转换为 网络字节序。
 */
void NTPclient::ntp_hton_packet(ntp_packet *nptr) {
    nptr->root_delay = htonl(nptr->root_delay);
    nptr->root_dispersion = htonl(nptr->root_dispersion);
    nptr->ref_indentifier = htonl(nptr->ref_indentifier);
    nptr->reference.seconds = htonl(nptr->reference.seconds);
    nptr->reference.fraction = htonl(nptr->reference.fraction);
    nptr->originate.seconds = htonl(nptr->originate.seconds);
    nptr->originate.fraction = htonl(nptr->originate.fraction);
    nptr->receive.seconds = htonl(nptr->receive.seconds);
    nptr->receive.fraction = htonl(nptr->receive.fraction);
    nptr->transmit.seconds = htonl(nptr->transmit.seconds);
    nptr->transmit.fraction = htonl(nptr->transmit.fraction);
}


/**
 * 向 NTP 服务器发送 NTP 请求，获取相关计算所需的时间戳（T1、T2、T3、T4如下所诉）。
 *     1. 客户端 发送一个NTP报文给 服务端，该报文带有它离开 客户端 时的时间戳，该时间戳为 T1。
 *     2. 当此NTP报文到达 服务端 时，服务端 加上自己的时间戳，该时间戳为 T2。
 *     3. 当此NTP报文离开 服务端 时，服务端 再加上自己的时间戳，该时间戳为 T3。
 *     4. 当 客户端 接收到该应答报文时，客户端 的本地时间戳，该时间戳为 T4。
 * @param [in ] host : NTP 服务器的 IP（四段式 IP 地址）。
 * @param [in ] port  : NTP 服务器的 端口号（可取默认的端口号 NTP_PORT : 123）。
 * @param [in ] timeout : 超时时间（单位 毫秒）。
 * @param [out] T_buf : 操作成功返回的相关计算所需的时间戳（T1、T2、T3、T4）。
 *
 *     成功，返回 0；失败，返回 错误码。
 */
int NTPclient::ntp_get_time_values(const char *host, u_int16_t port, uint timeout, long long T_buf[4]) {
    int err = -1;

    int fd_socket;
    ntp_packet npt_buffer{};
    ntp_timeval tm_value{};

    int xit_addrlen = sizeof(struct sockaddr_in);
    struct sockaddr_in skaddr_host{};

    //如果地址为空、超时设置成0、输入的buf为空
    if ((nullptr == host) || (timeout <= 0) || (nullptr == T_buf))
        return -1;

    //获得socket句柄
    fd_socket = socket(AF_INET, SOCK_DGRAM, 0);

    if (fd_socket == -1)
        return -1;

    // 设置 发送/接收 超时时间
    tm_value.tv_sec = (long) ((timeout / 1000));
    tm_value.tv_usec = (long) ((timeout % 1000) * 1000);
    setsockopt(fd_socket, SOL_SOCKET, SO_SNDTIMEO, (char *) &tm_value, sizeof(ntp_timeval));
    setsockopt(fd_socket, SOL_SOCKET, SO_RCVTIMEO, (char *) &tm_value, sizeof(ntp_timeval));

    // 服务端主机地址
    memset(&skaddr_host, 0, sizeof(struct sockaddr_in));
    skaddr_host.sin_family = AF_INET;
    skaddr_host.sin_port = htons(port);
    inet_pton(AF_INET, host, &skaddr_host.sin_addr.s_addr);

    // 初始化请求数据包
    ntp_init_request_packet(&npt_buffer);

    // NTP请求报文离开发送端时发送端的本地时间
    ntp_gettimeofday(tm_value);
    ntp_timeval_to_timestamp(&npt_buffer.originate, &tm_value);

    // T1
    T_buf[0] = (long long) ntp_timeval_us(&tm_value);

    // 转成网络字节序
    ntp_hton_packet(&npt_buffer);

    // 投递请求
    err = sendto(fd_socket,
                 (char *) &npt_buffer,
                 sizeof(ntp_packet),
                 0,
                 (sockaddr *) &skaddr_host,
                 sizeof(struct sockaddr_in));
    if (err < 0)
        return errno;

    memset(&npt_buffer, 0, sizeof(ntp_packet));

    // 接收应答
    err = recvfrom(fd_socket,
                   (char *) &npt_buffer,
                   sizeof(ntp_packet),
                   0,
                   (sockaddr *) &skaddr_host,
                   (socklen_t *) &xit_addrlen);
    if (err < 0)
        return errno;

    if (sizeof(ntp_packet) != err)
        return -1;

    // T4
    T_buf[3] = (long long) ntp_gettimevalue();

    // 转成主机字节序
    ntp_ntoh_packet(&npt_buffer);

    T_buf[1] = (long long) ntp_timestamp_us(&npt_buffer.receive); // T2
    T_buf[2] = (long long) ntp_timestamp_us(&npt_buffer.transmit); // T3

    //(T4-T1) -(T3-T2)
//    std::cout<<"链路时延: "<<((T_buf[3]-T_buf[0])-(T_buf[2]-T_buf[1]))/10000<<"ms"<<std::endl;

    if (fd_socket)
        close(fd_socket);

    err = 0;
    return err;
}