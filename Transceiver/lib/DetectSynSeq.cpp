//
// Created by locate on 2019/9/27.
//

#include "DetectSynSeq.hpp"

DetectSynSeq::DetectSynSeq() = default;

DetectSynSeq::~DetectSynSeq() {
    _head_symbol.clear();//存放进入相关操作的IQ数据
    _syn_size = 0;
    _peak.clear(); //存放峰值的容器对象
    _syn.clear(); //同步序列
    _conj_syn.clear();

    _find_syn = false;//是否同步序列
    _symbol_count = 0;//_head_symbol  查询数据的位置
    _corr = 0.0;//corr 峰值
}

DetectSynSeq::sptr DetectSynSeq::make() {
    return std::make_shared<DetectSynSeq>();
};


void DetectSynSeq::GenerateSyn() {
    _syn.resize(_syn_size);
    for (int i = 0; i < _syn_size; ++i) {
        _syn.at(i) = std::complex<double>(sreal[i], simag[i]);
    }
}

void DetectSynSeq::ConjSyn(bool need_reverse_syn) {
    _conj_syn.resize(_syn_size);
    for (int i = 0; i < _syn_size; ++i) {
        _conj_syn.at(i) = std::conj(_syn.at(i));
    }
    if (need_reverse_syn) {
        std::reverse(_conj_syn.begin(), _conj_syn.end());
    }
}

double DetectSynSeq::CaculateMeanAmp() {
    double sumPower = 0;
    for (auto buf : _head_symbol) {
        sumPower += (buf.real() * buf.real() + buf.imag() * buf.imag());
    }

    return std::sqrt(sumPower / (double) _syn_size);
}

void DetectSynSeq::DetectSynWork(const std::vector<std::complex<short>> &buffer) {
    ResetParmaters();
    for (auto &buf : buffer) {
        ReceiveSymbol(buf);//一个个输入数据
        if (_find_syn) break;
    }
}

void DetectSynSeq::ReceiveSymbol(const std::complex<short> &sample) {
    _symbol_count++;//记录输入数据的长度
    _head_symbol.emplace_back(sample.real(), sample.imag());//尾部添加1个数据 //注意里面实际已经有 64 个0了
    _head_symbol.pop_front();//弹出头部数据

    double mean_amp = CaculateMeanAmp();
    if (mean_amp == 0)return;

    Filter();

    double cur_peak = std::sqrt(std::norm((_corr))) / mean_amp;
//    _peak.emplace_back(cur_peak);

    if (cur_peak >= _threshold * 0.7) {
        _find_syn = true;

//        LOG(INFO) << "current peak: " << cur_peak;
//        LOG(INFO) << "current peak index: " << _symbol_count;
    }
}

void DetectSynSeq::ResetParmaters() {
    _symbol_count = 0;
    _find_syn = false;

//    _peak.clear();
    _head_symbol.clear();
    _head_symbol.resize(_syn_size, std::complex<double>(0.0, 0.0));
}

void DetectSynSeq::Filter() {
    _corr = 0;
    for (int i = 0; i < _syn_size; ++i) {
        _corr += _conj_syn.at(i) * _head_symbol.at(i);
    }

}

void DetectSynSeq::SetSyn(std::vector<std::complex<double>> syn, bool need_reverse_syn) {
    _need_reverse_syn = need_reverse_syn;

    if (syn.empty()) {
        _syn_size = 64;

        GenerateSyn();

        if (!_need_reverse_syn)_threshold = CalculateSynAutocorrelation(_syn);//64
        else _threshold = 63.4956;

    } else {
        _syn_size = syn.size();

        _syn.resize(_syn_size);
        memcpy(_syn.data(), syn.data(), sizeof(std::complex<double>) * _syn_size);

        _threshold = CalculateSynAutocorrelation(syn);
    }

    ConjSyn(_need_reverse_syn);

    _head_symbol.clear();
    _head_symbol.resize(_syn_size, std::complex<double>(0.0, 0.0));
}

void DetectSynSeq::GetSyn(std::vector<std::complex<short>> &syn, std::complex<double> multiple) {
    if (multiple.imag() != 0) {
        multiple.imag(0);
    }

    syn.resize(_syn_size, std::complex<short>(0, 0));

    for (int i = 0; i < _syn_size; ++i) {
        syn.at(i) = static_cast<std::complex<short>>(_syn.at(i) * multiple);
    }
}

double DetectSynSeq::CalculateSynAutocorrelation(const std::vector<std::complex<double>> &syn) {
    double threshold = 0.0;
    for (auto j : syn) {
        threshold += std::sqrt(std::norm(j * std::conj(j)));
    }
    LOG(INFO) << "syn threshold: " << threshold;
    return threshold;
}