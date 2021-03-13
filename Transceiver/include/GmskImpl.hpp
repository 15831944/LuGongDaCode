//
// Created by locate on 2019/8/27.
//
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <string>
#include <vector>
#include <bitset>
#include <ctime>
#include <algorithm>
#include <complex>
#include <string>
#include <memory>
#include <thread>
#include <glog/logging.h>
#include <queue>

#include "Complex.hpp"
#include "BitVector.hpp"
#include "Vector.hpp"
#include "SigProcLib.hpp"
#include "strain.hpp"
#include "ModulatorBase.hpp"

#ifndef GMSKMODULATOR_HPP
#define GMSKMODULATOR_HPP

class GmskModulator : public ModulatorBase {
public:
    typedef std::shared_ptr<GmskModulator> sptr;

    int _TSC{};//训练序列序号

    int _samples_per_packages{};

    std::shared_ptr<SigProcLib> _sig_proc;

public:
    GmskModulator(int samples, int TSC);

    ~GmskModulator() override;

    static sptr make(int samples, int TSC);

    void ModWork(int tsc_num, int insert_tsc_per_bytes,
                 const BitVector &input, std::vector<std::complex<float>> &output) override;

};

class GmskDemodulator : public DemodulatorBase {
public:

    typedef std::shared_ptr<GmskDemodulator> sptr;

    int _TSC{};//训练序列序号

    int _samples_per_packages{};

    int _gts_size{};// = gTrainingSequence[_TSC].size();

    float _detect_threshold = 8.0;

    std::shared_ptr<SigProcLib> _sig_proc;

public:

    GmskDemodulator(int samples, int TSC);

    ~GmskDemodulator() override;

    static sptr make(int samples, int TSC);

    int DemodWork(int actual_mtu, int tsc_num, int insert_tsc_per_bytes,
                  const std::vector<std::complex<short>> &input, SoftVector &output, double &snr) override;

    static void Sc16ToSignalVector(const std::vector<std::complex<short>> &input, signalVector &output, double amp);

    double ML_SNR(signalVector signal);
};

#endif //GMSKMODULATOR_HPP
