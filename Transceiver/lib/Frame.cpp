//
// Created by locate on 2019/10/29.
//

#include "Frame.hpp"

PhyFrame::PhyFrame(std::vector<std::complex<short>> tone, std::vector<std::complex<short>> syn, int samples_num) {

    _samples_num = samples_num;

    _buffer_num = tone.size() + syn.size() + _samples_num;

    _buffer.resize(_buffer_num, std::complex<short>(0, 0));
    memcpy(&_buffer.front(), tone.data(), sizeof(std::complex<short>) * tone.size());
    memcpy(&_buffer.at(tone.size()), syn.data(), sizeof(std::complex<short>) * syn.size());
    _point_index = tone.size() + syn.size();
}

PhyFrame::~PhyFrame() = default;

PhyFrame::sptr PhyFrame::make(const std::vector<std::complex<short>> &tone, const std::vector<std::complex<short>> &syn,
                              int samples_num) {
    return std::make_shared<PhyFrame>(tone, syn, samples_num);
}

void PhyFrame::AddSamples(std::vector<std::complex<short>> samples) {
    if (samples.size() > _samples_num) {
        LOG(INFO) << "发送数据(" << samples.size() << "bit) > (" << _samples_num << "bit), 此次发送数据无效";
        return;
    }
    memcpy(&_buffer.at(_point_index), samples.data(), sizeof(std::complex<short>) * samples.size());
}

std::vector<std::complex<short>> &PhyFrame::GetBuffer() {
    return _buffer;
}

void PhyFrame::SetTimestamp(TIMESTAMP timestamp, TIMESTAMP timestamp_offset) {
    _timestamp = timestamp + timestamp_offset;
}

void PhyFrame::ResetTimestamp() {
    _timestamp = 0;
}

TIMESTAMP PhyFrame::GetTimestamp() {
    return _timestamp;
}

void PhyFrame::SetVacant(bool vacant) {
    _vacant = vacant;
}

bool PhyFrame::GetVacant() {
    return _vacant;
}

size_t PhyFrame::GetBufferSize() {
    return _buffer_num;
}

void PhyFrame::SetST(ST st) {
    _st = st;
}

PhyFrame::ST PhyFrame::GetST() {
    return _st;
}

//----------------------------------------------------
//----------------------------------------------------
//----------------------------------------------------
//LinkFrame
LinkFrame::LinkFrame(int ordinal, int append_space, FrameType frame_type, TransType trans_type, bool end,
                     const std::string &data, int filename_len) {
    _ordinal = ordinal;
    _append_space = append_space;
    _data = data;

    _frame_type = frame_type;
    switch (_frame_type) {
        case MSG_FRAME: {
            _frame_type_bit = MSGFrame;
            break;
        }
        case ACK_FRAME: {
            _frame_type_bit = ACKFrame;
            break;
        }
    }


    //"0" 代表结尾 否则用"1"
    BitVector firstBit("1");
    if (end) {
        firstBit = BitVector("0");
    }

    _trans_type = trans_type;
    switch (_trans_type) {
        case TRANS_CHAT: {
            _trans_type_8bit = BitVector(BitVector(CHATFrame, firstBit), IDLEFrame);
            break;
        }
        case TRANS_FILE: {
            std::bitset<5> msg = filename_len;
            BitVector msg_BitV = msg.to_string().data();
            BitVector name_frame = msg_BitV;
            _trans_type_8bit = BitVector(BitVector(FILEFrame, firstBit), name_frame);
            break;
        }
        case TRANS_BINARYFILE: {
            _trans_type_8bit = BitVector(BitVector(BINARYFILEFRAME, firstBit), IDLEFrame);
        }
    }


    GmskModulator::IntToBitVector32(_ordinal_bit, _ordinal, false);
    GenerateSpaceNumBit(_append_space, _append_space_bit);
    StrToBitVector();
}

LinkFrame::sptr LinkFrame::make(int ordinal, int append_space, FrameType frame_type, TransType trans_type, bool end,
                                const std::string &data, int filename_len) {
    return std::make_shared<LinkFrame>(ordinal, append_space, frame_type, trans_type, end, data, filename_len);
}

void LinkFrame::SetOrdinal(int ordinal) {
    _ordinal = ordinal;
    GmskModulator::IntToBitVector32(_ordinal_bit, _ordinal, false);
}

void LinkFrame::AddData(const std::string &input) {
    _data = input;
    StrToBitVector();
}

void LinkFrame::StrToBitVector() {
    _data_bit.resize(_data.size() * 8);
    _data_bit.unpack((unsigned char *) _data.c_str());
}

void LinkFrame::UpdataCount() {
    _count++;
}

BitVector LinkFrame::GetAllBit() const {
    return BitVector(
            BitVector(BitVector(BitVector(_ordinal_bit, _frame_type_bit), _append_space_bit), _trans_type_8bit),
            _data_bit);
}

bool LinkFrame::GenerateSpaceNumBit(int num, BitVector &output) {
    if (num > (1 << 15) - 1) {
        LOG(INFO) << "note: " << num << " > " << (1 << 15) - 1 << "!!!";
        return false;
    }

    std::bitset<15> msg = num;
    BitVector msg_BitV = msg.to_string().data();
    output = msg_BitV;
    return true;
}
