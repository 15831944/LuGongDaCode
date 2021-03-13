//
// Created by locate on 6/4/2020.
//

#include "terjin/tfm_digital_modem/modem/ModulatorBase.hpp"
#include "terjin/tfm_digital_modem/modem/GmskImpl.hpp"
#include "terjin/tfm_digital_modem/modem/BPSKImpl.hpp"

ModulatorBase::~ModulatorBase() {
    LOG(INFO) << "~ModulatorBase()";
}

ModulatorBase::sptr ModulatorBase::make(SignalType type, int samples, int TSC) {
    switch (type) {
        case SignalType::QPSK:
        case SignalType::BPSK: {
            return ModulatorBase::sptr(new BPSKImpl(samples, TSC));
        }
        case SignalType::FSK4:
        case SignalType::GMSK:
        default: {
            return ModulatorBase::sptr(new GmskModulator(samples, TSC));
        }
    }
}

bool ModulatorBase::IntToBitVector32(BitVector &output, int input, bool need_code) {
    if (input > (1 << 16) - 1) {
        LOG(INFO) << "note: " << input << " > " << (1 << 16) - 1 << "!!!";
        return false;
    }

    std::bitset<16> msg = input;
    BitVector msg_BitV = msg.to_string().data();
//    LOG(INFO) << "msg bit = " << msg_BitV;

    if (!need_code) {
        output = msg_BitV;
    } else {
        output.resize(32);
        output.fill(0);
        ViterbiR2O4 Coder;
        msg_BitV.encode(Coder, output);
    }
    return true;
}

void ModulatorBase::CharToBinarychar(unsigned char *in_char, size_t in_len, BitVector &out_bit_char) {
    out_bit_char.resize(in_len * 8);//一个字节8比特
    out_bit_char.fill(0);
    out_bit_char.unpack(in_char);//字节转换为比特
}

void ModulatorBase::ConvolutionCoder(BitVector &input, BitVector &output) {
    output.resize(input.size() * 2);//1bit卷积编码后的长度是2bit
    output.fill(0);

    ViterbiR2O4 Coder;
    input.encode(Coder, output);//卷积编码
}

void ModulatorBase::CoderWork(BitVector input, BitVector &output) {

    //每8bit翻转
    input.LSB8MSB();

    //添加16位crc校验
    BitVector crc_check(16);
    crc_check.fill(0);

    Parity crc16(CRC16Generator, 16, input.size() + 16);
    crc16.writeParityWord(input, crc_check);

    //将crc校验码添加到数据尾部
    BitVector temp = BitVector(input, crc_check);

    //卷积码编码
    output.resize(temp.size() * 2);
    ConvolutionCoder(temp, output);
}


///Demodulator
DemodulatorBase::~DemodulatorBase() {
    LOG(INFO) << "~DemodulatorBase()";
}

DemodulatorBase::sptr DemodulatorBase::make(SignalType type, int samples, int TSC) {
    switch (type) {
        case SignalType::QPSK:
        case SignalType::BPSK: {
            return DemodulatorBase::sptr(new BPSKImpl(samples, TSC));
        }
        case SignalType::FSK4:
        case SignalType::GMSK:
        default: {
            return DemodulatorBase::sptr(new GmskDemodulator(samples, TSC));
        }
    }
}

BitVector DemodulatorBase::ViterbiDecode(const SoftVector &input) {
    BitVector decode(input.size() / 2);
    ViterbiR2O4 Coder;
    input.decode(Coder, decode);
    return decode;
}