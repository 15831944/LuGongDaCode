//
// Created by locate on 2019/9/27.
//

#ifndef DETECTSYNSEQ_HPP
#define DETECTSYNSEQ_HPP

#include <iostream>
#include <vector>
#include <deque>
#include <complex>
#include <cmath>
#include <algorithm>
#include <glog/logging.h>
#include <strain.hpp>
#include <memory>
#include <mutex>

class DetectSynSeq {
private:

    int _symbol_count = 0;//_head_symbol  查询数据的位置
    std::deque<std::complex<double>> _head_symbol;//fifo

    std::vector<double> _peak; //存放峰值的容器对象

    int _syn_size = 64;
    std::vector<std::complex<double>> _syn; //同步序列

    bool _need_reverse_syn = false;
    std::vector<std::complex<double>> _conj_syn;

    std::complex<double> _corr = 0.f;//corr 峰值
    double _threshold = 64;
    bool _find_syn = false;//是否同步序列

    std::mutex _mtx;

public:
    typedef std::shared_ptr<DetectSynSeq> sptr;

    DetectSynSeq();

    ~DetectSynSeq();

    static sptr make();

    void SetSyn(std::vector<std::complex<double>> syn, bool need_reverse_syn = false);

    void GetSyn(std::vector<std::complex<short>> &syn, std::complex<double> multiple);

    void DetectSynWork(const std::vector<std::complex<short>> &buffer);

    bool GetFindSyn() { return _find_syn; }

    int GetSymbolCount() { return _symbol_count; }

    static double CalculateSynAutocorrelation(const std::vector<std::complex<double>> &syn);

private:
    void GenerateSyn();

    void ConjSyn(bool need_reverse_syn);

    double CaculateMeanAmp();

    void ReceiveSymbol(const std::complex<short> &sample);

    void Filter();

    void ResetParmaters();

};

#endif //DETECTSYNSEQ_HPP
