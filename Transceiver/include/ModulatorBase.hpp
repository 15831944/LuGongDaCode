//
// Created by locate on 6/4/2020.
//

#ifndef TRANSCEIVER_MODULATORBASE_HPP
#define TRANSCEIVER_MODULATORBASE_HPP

#include <memory>
#include <BitVector.hpp>
#include <complex>
#include <vector>
#include <string>
#include <thread>
#include "Types.hpp"
#include "RootRaisedCosine.hpp"

#include <glog/logging.h>

//信号类型
enum SignalType {
    GMSK,
    QPSK,
    BPSK,
    FSK4
};

class ModulatorBase {
public:
    typedef std::shared_ptr<ModulatorBase> sptr;

    virtual ~ModulatorBase();

    static sptr make(SignalType type, int samples, int TSC);

    /**
    * convert char to binary char
    * @param in_char the input char content
    * @param in_len the input char len
    * @param out_bit_char the output
    */
    static void CharToBinarychar(unsigned char *in_char, size_t in_len, BitVector &out_bit_char);

    /**
     * convolution code coding
     * @param input the input
     * @param output the output
     */
    static void ConvolutionCoder(BitVector &input, BitVector &output);

    static bool IntToBitVector32(BitVector &output, int input, bool need_code);

    static void CoderWork(BitVector input, BitVector &output);

    virtual void ModWork(int tsc_num, int insert_tsc_per_bytes, const BitVector &input,
                         std::vector<std::complex<float>> &output) = 0;

};

class DemodulatorBase {
public:
    typedef std::shared_ptr<DemodulatorBase> sptr;

    virtual ~DemodulatorBase();

    static sptr make(SignalType type, int samples, int TSC);

    virtual int DemodWork(int actual_mtu, int tsc_num, int insert_tsc_per_bytes,
                          const std::vector<std::complex<short>> &input, SoftVector &output, double &snr) = 0;

    static BitVector ViterbiDecode(const SoftVector &input);

};


#endif //TRANSCEIVER_MODULATORBASE_HPP
