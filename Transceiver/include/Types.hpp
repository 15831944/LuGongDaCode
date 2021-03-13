//
// Created by locate on 2019/11/20.
//
#include <BitVector.hpp>

#ifndef TYPES_HPP
#define TYPES_HPP

#define EXPENSES 7 //开销:序号(16bit 2bytes) (空格15bit+帧类型1bit 2bytes) crc校验码(16bit 2bytes) 类型(8bit 1byte) 占了7个字节的开销 单位:字节
#define GUARDPERIODLENGTH 4//保护间隔长度 单位:比特

enum FrameType {
    MSG_FRAME = 0,//信息帧
    ACK_FRAME = 1//ack帧
};

enum TransType {
    TRANS_CHAT = 0,
    TRANS_FILE = 1,
    TRANS_BINARYFILE = 2
};

const std::string IDLESTR = "01011";

const BitVector IDLEFrame = IDLESTR.c_str();

const std::string CHATSTR = "01";

const BitVector CHATFrame = CHATSTR.c_str();

const std::string FILESTR = "10";

const BitVector FILEFrame = FILESTR.c_str();

const std::string BINARYFILESTR = "11";

const BitVector BINARYFILEFRAME = BINARYFILESTR.c_str();

const BitVector MSGFrame = "0";

const BitVector ACKFrame = "1";


#endif //TYPES_HPP
