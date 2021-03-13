//
// Created by locate on 12/5/2020.
//

#include "BPSKImpl.hpp"

BPSKImpl::sptr BPSKImpl::make(int IQNumbers, int TSC) {
    return std::make_shared<BPSKImpl>(TSC, IQNumbers);
}

BPSKImpl::BPSKImpl(int IQNumbers, int TSC) {
    _TSC = TSC;
    _iq_number = IQNumbers;
    _guard.resize(GUARDPERIODLENGTH, std::complex<float>(0, 0));
    _filter = RootRaisedCosine::make(0.4, _iq_number / _factor, _factor);
    _h = _filter->GetImpulseResponse();
    GenerateMidamble();
    LOG(INFO) << "BPSKImpl()";
}

BPSKImpl::~BPSKImpl() {
    LOG(INFO) << "~BPSKImpl()";
}

void BPSKImpl::BPSKMod(const BitVector &input, std::vector<std::complex<float>> &output) {
    output.resize(input.size());
    for (size_t i = 0; i < input.size(); ++i) {
        output.at(i) = std::complex<float>(static_cast<float>(2 * input.bit(i) - 1), 0);
    }
}

void BPSKImpl::BPSKDemod(const std::vector<std::complex<float>> &data, SoftVector &output) {
    output.resize(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        if (data.at(i).real() >= 0)
            output[i] = 1.0;
        else {
            output[i] = 0.0;
        }
    }
}

void BPSKImpl::GenerateMidamble() {
    std::vector<std::complex<float>> middle_midamble;
    std::vector<std::complex<float>> midamble;

    // only use middle 16 bits of each TSC
    BPSKMod(BPSKTrainingSequence[_TSC].segment(5, 16), middle_midamble);
    BPSKMod(BPSKTrainingSequence[_TSC], midamble);

    /*std::cout << "middle_midamble: ";
    for (auto i : middle_midamble) {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "midamble: ";
    for (auto i : midamble) {
        std::cout << i << " ";
    }
    std::cout << std::endl;*/

    //相关、计算TOA、幅度以及训练序列的BPSK调制波形
    unsigned start_index = (middle_midamble.size() % 2) ? middle_midamble.size() / 2 : middle_midamble.size() / 2 - 1;
    std::vector<std::complex<float>> autocorr = Correlate(midamble, middle_midamble, false, start_index,
                                                          midamble.size());

    _midambles.sequence_all = midamble;
    _midambles.sequence = middle_midamble;
    _midambles.sequenceReversedConjugated = ReverseConjugate(middle_midamble);
    _midambles.gain = fabs(PeakDetect(autocorr, &_midambles.TOA, nullptr));

    LOG(INFO) << "TOA: " << _midambles.TOA << ", Gain: " << _midambles.gain;
    /*std::cout << "autocorr: ";
    for (auto i : autocorr) {
        std::cout << i << " ";
    }
    std::cout << std::endl;*/

    //预期TOA的值
    _expected_TOA_peak = (unsigned) round(_midambles.TOA + (_midambles.sequenceReversedConjugated.size() - 1) / 2);
}


std::vector<std::complex<float>> BPSKImpl::ReverseConjugate(const std::vector<std::complex<float>> &input) {
    std::vector<std::complex<float>> output(input.size());

    auto input_it = input.begin();
    auto out_it = output.end() - 1;

    while (input_it < input.end()) {
        *out_it = std::conj(*input_it);
        input_it++;
        out_it--;
    }

    return output;
}

std::vector<std::complex<float>> BPSKImpl::Correlate(const std::vector<std::complex<float>> &a,
                                                     const std::vector<std::complex<float>> &b,
                                                     bool b_reversed_conjugated, unsigned start_index, unsigned len) {
    std::vector<std::complex<float>> tmp;
    if (b_reversed_conjugated) {
        tmp = b;
    } else {
        tmp = ReverseConjugate(b);
    }

    return Convolve(a, tmp, start_index, len);
}

std::vector<std::complex<float>> BPSKImpl::Convolve(const std::vector<std::complex<float>> &a,
                                                    const std::vector<std::complex<float>> &b,
                                                    unsigned start_index, unsigned len) {
    std::vector<std::complex<float>> out(len, 0);
    auto outP = out.begin();

    unsigned start = start_index;
    unsigned stop = start_index + len;

    while (start < stop) {
        auto aP = a.begin() + start;
        auto bP = b.begin();

        std::complex<float> sum(0, 0);
        while (bP < b.end()) {
            if (aP < a.begin())break;
            if (aP < a.end()) {
                sum += (*aP) * (*bP);
            }

            aP--;
            bP++;
        }

        *outP = sum;
        outP++;
        start++;

    }
    return out;
}

std::complex<float> BPSKImpl::PeakDetect(const std::vector<std::complex<float>> &input,
                                         float *peakIndex, float *avgPwr) {
    std::complex<float> maxVal = 0.0;
    float maxIndex = 0.0;
    float sumPower = 0.0;

    for (size_t i = 0; i < input.size(); i++) {
        float samplePower = std::norm(input.at(i));
        if (samplePower > maxVal.real()) {
            maxVal = samplePower;
            maxIndex = i;
        }
        sumPower += samplePower;
    }

    if (peakIndex != nullptr)
        *peakIndex = maxIndex;


    if (avgPwr != nullptr)
        *avgPwr = (sumPower - maxVal.real()) / float(input.size() - 1);

    return input.at(maxIndex);
}

void BPSKImpl::UpSample(int factor, std::vector<std::complex<float>> &input) {
    std::vector<std::complex<float>> output(factor * input.size(), 0);
    for (size_t j = 0; j < input.size(); ++j) {
        output.at(j * factor) = input.at(j);
    }

    input.clear();
    input = output;
}

void BPSKImpl::DownSample(int factor, std::vector<std::complex<float >> &input) {
    std::vector<std::complex<float >> output(input.size() / factor);
    for (size_t j = 0; j < output.size(); ++j) {
        output.at(j) = input.at(j * factor);
    }
    input = output;
}

void BPSKImpl::ModWork(int tsc_num, int insert_tsc_per_bytes, const BitVector &input,
                       std::vector<std::complex<float>> &output) {
    if (tsc_num <= 0) {
        LOG(ERROR) << "tsc_num cannot equal to 0";
        return;
    }
    output.clear();
    std::vector<std::complex<float>> mod;
    BitVector normal;
    if (tsc_num == 1) {
        normal = BitVector(BPSKTrainingSequence[_TSC], input);
        BPSKMod(normal, mod);
        mod.insert(mod.end(), _guard.begin(), _guard.end());
        UpSample(_factor, mod);
        if (mod.size() != (size_t) _iq_number) {
            LOG(ERROR) << "mod size != iq number";
            mod.resize(_iq_number, 0);
        }
        output = Convolve(mod, _h, (_h.size() % 2) ? _h.size() / 2 : (_h.size() / 2 - 1), mod.size());
    } else {
        int seg_size = insert_tsc_per_bytes * 8 * 2;
        int i = 0;
        for (i = 0; i < tsc_num; ++i) {
            normal.clear();
            mod.clear();

            if (i != (tsc_num - 1)) {
                normal = BitVector(BPSKTrainingSequence[_TSC], input.segment(i * seg_size, seg_size));
            } else {
                normal = BitVector(BPSKTrainingSequence[_TSC],
                                   input.segment(i * seg_size, input.size() - i * seg_size));
            }

            BPSKMod(normal, mod);
            mod.insert(mod.end(), _guard.begin(), _guard.end());
            UpSample(_factor, mod);

            auto tmp = Convolve(mod, _h, (_h.size() % 2) ? _h.size() / 2 : (_h.size() / 2 - 1), mod.size());
            output.insert(output.end(), tmp.begin(), tmp.end());
        }
    }
}

int BPSKImpl::DemodWork(int actual_mtu, int tsc_num, int insert_tsc_per_bytes,
                        const std::vector<std::complex<short>> &input, SoftVector &output, double &snr) {
    if (tsc_num <= 0)
        return -1;

    output.clear();
    int BT_size = BPSKTrainingSequence[_TSC].size();
    int start_index = (_h.size() % 2) ? _h.size() / 2 : (_h.size() / 2 - 1);
    int seg_size = (insert_tsc_per_bytes * 8 * 2 + BT_size + GUARDPERIODLENGTH) * _factor;

    snr = 0.f;
    float ampl = 0;
    float TOA = 0;

    std::vector<std::complex<float>> recv_data(input.size());
    for (size_t j = 0; j < input.size(); ++j) {
        recv_data.at(j) = input.at(j);
    }

    std::vector<std::complex<float>> tmp_data;
    for (int k = 0; k < tsc_num; ++k) {
        snr = 0.f;
        ampl = 0;
        TOA = 0;

        if (tsc_num == 1) {
            tmp_data = recv_data;
        } else if (k == (tsc_num - 1)) {
            tmp_data.resize(recv_data.size() - k * seg_size);
            memcpy(tmp_data.data(), &recv_data.at(k * seg_size),
                   sizeof(std::complex<float>) * (recv_data.size() - k * seg_size));
        } else {
            tmp_data.resize(seg_size);
            memcpy(tmp_data.data(), &recv_data.at(k * seg_size), sizeof(std::complex<float>) * seg_size);
        }

        tmp_data = Convolve(tmp_data, _h, start_index, tmp_data.size());

        DownSample(_factor, tmp_data);


        if (!AnalyzeTrafficBurst(tmp_data, &ampl, &TOA)) {
            output.clear();
            return -1;
        }

        if (!DelayVector(tmp_data, -TOA)) {
            output.clear();
            return -2;
        }

        ScaleVector(tmp_data, std::complex<float>(1.f / ampl, 0.f));

        CopeFreqOffset(tmp_data);
        CopePhaseOffset(tmp_data);

        //TODO 估计信噪比

        SoftVector tmp_out;
        BPSKDemod(tmp_data, tmp_out);

        if (k == 0) {
            output = tmp_out.segment(BT_size, tmp_data.size() - BT_size);
        } else {
            output = SoftVector(output, tmp_out.segment(BT_size, tmp_data.size() - BT_size));
        }

    }
    return 0;
}

bool BPSKImpl::AnalyzeTrafficBurst(std::vector<std::complex<float>> &input, float *amplitude, float *TOA) {
    std::vector<std::complex<float>> input_seg(BPSKTrainingSequence[_TSC].size());//接收的训练序列
    memcpy(input_seg.data(), &input.at(0), input_seg.size() * sizeof(std::complex<float>));

    //相关
    int maxTOA = 3;//理想TOA值
    int corr_len = 2 * maxTOA + 1;
    std::vector<std::complex<float>> correlated_burst = Correlate(input_seg, _midambles.sequenceReversedConjugated,
                                                                  true,
                                                                  _expected_TOA_peak - maxTOA, corr_len);

    /*std::cout << "correlated_burst: ";
    for (auto i : correlated_burst) {
        std::cout << i << " ";
    }
    std::cout << std::endl;*/

    //检测峰值 TOA和平均功率
    float meanPower = 0.f;
    std::complex<float> amp = PeakDetect(correlated_burst, TOA, &meanPower);
    if (meanPower <= 0)return false;

    *amplitude = fabs(amp);

    float valleyPower = 0.0;
    auto peakPtr = correlated_burst.begin() + (int) std::rint(*TOA);

    // check for bogus results
    if ((*TOA < 0.0) || (*TOA > correlated_burst.size())) {
        *amplitude = 0.0;
        return false;
    }

    int numRms = 0;
    for (int i = 2; i <= 5; i++) {
        if (peakPtr - i >= correlated_burst.begin()) {
            valleyPower += std::norm(*(peakPtr - i));
            numRms++;
        }
        if (peakPtr + i < correlated_burst.end()) {
            valleyPower += std::norm(*(peakPtr + i));
            numRms++;
        }
    }

    if (numRms < 2) {
        // check for bogus results
        *amplitude = 0.0;
        return false;
    }

    auto RMS = (float) (sqrtf(valleyPower / (float) numRms) + 0.00001);
    float peakToMean = std::abs(*amplitude) / RMS;
    if (peakToMean < _detect_threshold)return false;

    *amplitude /= _midambles.gain;
    *TOA = *TOA - (float) maxTOA;

//    LOG(INFO) << "TOA: " << *TOA << ", peakAmpl: " << std::abs(*amplitude) << ", RMS:" << RMS << ", peakToMean: "
//              << peakToMean;

    return true;
}

bool BPSKImpl::DelayVector(std::vector<std::complex<float>> &input, float TOA) {
    TOA = std::floor(TOA);

    float maxTOA = 3;//最大偏移
    if (fabs((double) TOA) > maxTOA) {
        return false;
    }

    std::vector<std::complex<float>> tmp(input.size(), std::complex<float>(0, 0));
    auto ttr = tmp.begin();
    auto itr = input.begin();

    if (TOA < 0) {
        itr = input.begin() + (-TOA);
        while (itr < input.end()) {
            (*ttr++) = (*itr++);
        }
    } else if (TOA > 0) {
        ttr = tmp.begin() + TOA;
        while (ttr < tmp.end()) {
            (*ttr++) = (*itr++);
        }
    } else {
        tmp = input;
    }

    input.resize(tmp.size() - GUARDPERIODLENGTH);
    memcpy(input.data(), tmp.data(), sizeof(std::complex<float>) * input.size());

    return true;
}

void BPSKImpl::ScaleVector(std::vector<std::complex<float>> &input, std::complex<float> amp) {
    auto ptr = input.begin();
    while (ptr < input.end()) {
        *ptr *= amp;
        ptr++;
    }
}

bool BPSKImpl::CopeFreqOffset(std::vector<std::complex<float>> &input) {
    std::vector<std::complex<float>> input_seg(_midambles.sequence_all.size());//接收到的训练序列
    memcpy(input_seg.data(), &input.at(0), input_seg.size() * sizeof(std::complex<float>));

    std::vector<std::complex<float>> multi_res(_midambles.sequence_all.size());//存储共轭相乘后的值

    // Do the multiplication element wise
    for (size_t ii = 0; ii < _midambles.sequence_all.size(); ii++) {
        multi_res.at(ii) = input_seg.at(ii) * std::conj(_midambles.sequence_all.at(ii));
    }

    //消除频偏
    std::complex<float> freq_offset(0, 0);
    for (size_t j = 1; j < _midambles.sequence_all.size(); ++j) {
        freq_offset += (std::conj(multi_res.at(j - 1)) * multi_res.at(j));
    }

    freq_offset /= std::complex<float>(float(_midambles.sequence_all.size() - 1), 0);//求平均
//    LOG(INFO) << "freq offset: " << freq_offset << "->" << std::arg(freq_offset) / (2 * M_PI * 1) << "Hz";

//    if (std::arg(freq_offset) == 0) {
//        return true;
//    }

    ///fix me
    //把z共轭相乘回去即可
    auto tmp = freq_offset;
    for (size_t i = 1; i < input.size(); ++i) {
        input.at(i) *= std::conj(tmp);
        tmp *= freq_offset;
    }

    return true;
}

bool BPSKImpl::CopePhaseOffset(std::vector<std::complex<float>> &input) {
    std::vector<std::complex<float>> input_seg(_midambles.sequence_all.size());//接收到的训练序列
    memcpy(input_seg.data(), &input.at(0), input_seg.size() * sizeof(std::complex<float>));

    std::complex<float> corr = std::complex<float>(0, 0);//存储共轭相乘累加后的值

    // Do the multiplication element wise
    for (size_t ii = 0; ii < _midambles.sequence_all.size(); ii++) {
        corr += (input_seg.at(ii) * std::conj(_midambles.sequence_all.at(ii)));
    }

    //消除相偏
    corr /= std::complex<float>(_midambles.sequence_all.size(), 0);//求平均
//    LOG(INFO) << "phase offset: " << corr << "->"
//              << std::arg(corr) * (180.f / M_PI) << "°";

    for (auto &d : input) {
        d *= std::conj(corr);
    }

    return true;
}
