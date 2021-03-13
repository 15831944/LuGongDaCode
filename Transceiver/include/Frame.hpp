//
// Created by locate on 2019/10/29.
//

#ifndef FRAME_HPP
#define FRAME_HPP

#include <cstring>
#include <utility>
#include <vector>
#include <complex>
#include <memory>
#include <bitset>
#include <glog/logging.h>

#include "BitVector.hpp"
#include "GmskImpl.hpp"
#include "Types.hpp"

typedef unsigned long long TIMESTAMP;

class PhyFrame {
public:
    typedef std::shared_ptr<PhyFrame> sptr;

    enum Type {
        Str = 0,
        File = 1,
        UNKNOWN
    };

    struct ST {
        Type type = UNKNOWN;
        int ordinal = 0;
        bool isAck = false;

        void Reset() {
            type = UNKNOWN;
            ordinal = 0;
            isAck = false;
        }

        bool operator==(const ST &obj) {
            return (this->type == obj.type && this->ordinal == obj.ordinal && this->isAck == obj.isAck);
        }
    };

    ST _st;

    bool _vacant = true;//查询该帧是否空闲
    TIMESTAMP _timestamp{};//该帧的时间戳
    size_t _samples_num{};//IQ数据大小

//    std::complex<short> *_send_buffer;
    std::vector<std::complex<short>> _buffer;//数据
    size_t _buffer_num{};//数据大小 = _syn_num + _samples_num;

    std::vector<std::complex<short>> _tone;//单音信号;

    size_t _point_index = 0;

public:
    PhyFrame(std::vector<std::complex<short>> tone, std::vector<std::complex<short>> syn, int samples_num);

    ~PhyFrame();

    static sptr
    make(const std::vector<std::complex<short>> &tone, const std::vector<std::complex<short>> &syn, int samples_num);

    /**
     * 添加IQ数据
     * @param samples IQ数据
     */
    void AddSamples(std::vector<std::complex<short>> samples);

    size_t GetBufferSize();

    /**
     * 获取数据
     * @return 数据
     */
    std::vector<std::complex<short>> &GetBuffer();

    /**
     * 设置该帧的时间戳
     * @param timestamp 该帧的时间戳
     * @param timestamp_offset 时间戳偏移
     */
    void SetTimestamp(TIMESTAMP timestamp, TIMESTAMP timestamp_offset = 0);

    /**
     * 重置时间戳为0
     */
    void ResetTimestamp();

    /**
     * 获得该帧的时间戳
     * @return
     */
    TIMESTAMP GetTimestamp();

    void SetVacant(bool vacant);

    bool GetVacant();

    void SetST(ST st);

    ST GetST();

};


class LinkFrame {
private:

    int _count{};//发送次数

    int _ordinal = 0;//序号
    BitVector _ordinal_bit;

    std::string _data;//数据
    BitVector _data_bit;

    int _append_space{};//添加空格数
    BitVector _append_space_bit;

    BitVector _frame_type_bit;//帧类型 msg帧?ack帧?...
    FrameType _frame_type;

    TransType _trans_type;
    BitVector _trans_type_8bit;//chat? jpg?...

public:
    typedef std::shared_ptr<LinkFrame> sptr;

    LinkFrame(int ordinal, int append_space, FrameType frame_type, TransType trans_type, bool end,
              const std::string &data, int filename_len);

    ~LinkFrame() = default;

    static sptr make(int ordinal, int append_space, FrameType frame_type, TransType trans_type, bool end,
                     const std::string &data, int filename_len);

    void SetOrdinal(int ordinal);

    void AddData(const std::string &input);

    void StrToBitVector();

    void UpdataCount();

    int GetCount() { return _count; }

    int GetOrdinal() const { return _ordinal; }

    BitVector GetOrdinalBit() const { return _ordinal_bit; }

    std::string GetData() { return _data; }

    BitVector GetDataBit() const { return _data_bit; }

    BitVector GetOrdinalDataBit() const { return BitVector(_ordinal_bit, _data_bit); }

    BitVector GetAllBit() const;

    TransType GetTransType() const { return _trans_type; }

    static bool GenerateSpaceNumBit(int num, BitVector &output);

    FrameType GetFrameType() const { return _frame_type; };
};

struct FileFrame {
public:
    int _ordinal;
    std::string _data;
public:
    FileFrame(int ordinal, std::string data) : _ordinal(ordinal), _data(std::move(data)) {}

    bool operator<(const FileFrame &obj) {
        return _ordinal < obj._ordinal;
    }
};


#endif //FRAME_HPP
