//
// Created by locate on 2019/8/27.
//

#include "GmskImpl.hpp"

GmskModulator::GmskModulator(int samples, int TSC) {

    _TSC = TSC;

    _samples_per_packages = samples;

    _sig_proc = SigProcLib::make(_samples_per_packages, _TSC);
    _sig_proc->sigProcLibSetup();
    _sig_proc->generateGSMPulse();
}

GmskModulator::~GmskModulator() = default;

GmskModulator::sptr GmskModulator::make(int samples, int TSC) {
    return std::make_shared<GmskModulator>(samples, TSC);
}

void GmskModulator::ModWork(int tsc_num, int insert_tsc_per_bytes,
                            const BitVector &input, std::vector<std::complex<float>> &output) {
//    LOG(INFO) << "normal(" << input.size() << "bit) = " << input;
    if (tsc_num == 1) {
        BitVector normal_1 = BitVector(gTrainingSequence[_TSC], input);
        signalVector *signal_1 = _sig_proc->modulateBurst(normal_1, _sig_proc->GetGsmPulse(), GUARDPERIODLENGTH);
        signalVector::signalVectorToSc32(*signal_1, output);
        delete signal_1;
        signal_1 = nullptr;
        return;
    }

    std::vector<signalVector *> signal(tsc_num);
    int seg_size = insert_tsc_per_bytes * 8 * 2;
    int i = 0;
    for (i = 0; i < tsc_num - 1; ++i) {//注意最后一部分未处理
        BitVector normal = BitVector(gTrainingSequence[_TSC], input.segment(i * seg_size, seg_size));
//        LOG(INFO) << "normal(" << normal.size() << "bit) = " << normal;
        signal.at(i) = _sig_proc->modulateBurst(normal, _sig_proc->GetGsmPulse(), GUARDPERIODLENGTH);

        std::vector<std::complex<float>> temp;
        signalVector::signalVectorToSc32(*signal.at(i), temp);
        output.insert(output.end(), temp.begin(), temp.end());
    }

    //处理最后一部分
    BitVector normal_end = BitVector(gTrainingSequence[_TSC], input.segment(i * seg_size, input.size() - i * seg_size));
//    LOG(INFO) << "normal_end(" << normal_end.size() << "bit) = " << normal_end;
    signal.at(i) = _sig_proc->modulateBurst(normal_end, _sig_proc->GetGsmPulse(), GUARDPERIODLENGTH);

    std::vector<std::complex<float>> temp2;
    signalVector::signalVectorToSc32(*signal.at(i), temp2);
    output.insert(output.end(), temp2.begin(), temp2.end());

    for (auto &s: signal) {
        if (s) {
            delete s;
            s = nullptr;
        }
    }
}


//----------------------------------------------------
//----------------------------------------------------
//----------------------------------------------------
//GmskDemodulator
GmskDemodulator::GmskDemodulator(int samples, int TSC) {

    _TSC = TSC;

    _samples_per_packages = samples;

    _gts_size = gTrainingSequence[_TSC].size();

    _sig_proc = SigProcLib::make(_samples_per_packages, _TSC);
    _sig_proc->sigProcLibSetup();
    _sig_proc->generateGSMPulse();
    _sig_proc->generateMidamble();
}

GmskDemodulator::~GmskDemodulator() {
    LOG(INFO) << "~GmskDemodulator()";
}

GmskDemodulator::sptr GmskDemodulator::make(int samples, int TSC) {
    return std::make_shared<GmskDemodulator>(samples, TSC);
}

int GmskDemodulator::DemodWork(int actual_mtu, int tsc_num, int insert_tsc_per_bytes,
                               const std::vector<std::complex<short>> &input, SoftVector &output, double &snr) {

    if (tsc_num <= 0)
        return -1;

    snr = 0.0;

    if (tsc_num == 1) {
        signalVector signal_1;
        Sc16ToSignalVector(input, signal_1, 1);

        _sig_proc->ResetDemodParamters();
        bool success = _sig_proc->analyzeTrafficBurst(signal_1, _detect_threshold, 1,
                                                      0, true,
                                                      &_sig_proc->_channelResponse,
                                                      &_sig_proc->_channelResponseOffset);
        if (!success) {
//            LOG(INFO) << "success = " << success;
            return -1;
        }

        snr += ML_SNR(signal_1);

        _sig_proc->scaleVector(*_sig_proc->_channelResponse, fcomplex(1.0, 0.0) / _sig_proc->_amplitude);
        _sig_proc->designDFE(*_sig_proc->_channelResponse, float(1.0 / _sig_proc->_noisePwr), 7,
                             &_sig_proc->_feedForwardFilter, &_sig_proc->_feedbackFilter);
        _sig_proc->scaleVector(signal_1, fcomplex(1.0, 0.0) / _sig_proc->_amplitude);
        SoftVector *demod_1 = _sig_proc->equalizeBurst(signal_1, (_sig_proc->_TOA - _sig_proc->_channelResponseOffset),
                                                       *_sig_proc->_feedForwardFilter,
                                                       *_sig_proc->_feedbackFilter);
//        LOG(INFO) << "demod_1(" << demod_1->size() << "bit) = " << *demod_1;

        //TODO 如果TOA不等于0 需要根据TOA的值取bit
        output = demod_1->segment(_gts_size, actual_mtu * 8 * 2);
        delete demod_1;
        _sig_proc->DestroyChannelRespAndDFE();
        return 0;
    }

    std::vector<SoftVector *> demod(tsc_num);
    int i = 0;
    int seg_size = insert_tsc_per_bytes * 8 * 2 + _gts_size + GUARDPERIODLENGTH;
    for (i = 0; i < tsc_num - 1; ++i) {
        signalVector signal;
        std::vector<std::complex<short>> temp_input(seg_size);
        memcpy(temp_input.data(), &input.at(i * seg_size), sizeof(std::complex<short>) * seg_size);
        Sc16ToSignalVector(temp_input, signal, 1);
//        LOG(INFO) << "signal(" << signal.size() << "bit) = " << signal;

        _sig_proc->ResetDemodParamters();
        bool success = _sig_proc->analyzeTrafficBurst(signal, _detect_threshold, 1,
                                                      0, true,
                                                      &_sig_proc->_channelResponse,
                                                      &_sig_proc->_channelResponseOffset);
        if (!success) {
//            LOG(INFO) << "success = " << success;
            output.clear();
            return -1;
        }

        snr += ML_SNR(signal);

        _sig_proc->scaleVector(*_sig_proc->_channelResponse, fcomplex(1.0, 0.0) / _sig_proc->_amplitude);
        _sig_proc->designDFE(*_sig_proc->_channelResponse, float(1.0 / _sig_proc->_noisePwr), 7,
                             &_sig_proc->_feedForwardFilter, &_sig_proc->_feedbackFilter);
        _sig_proc->scaleVector(signal, fcomplex(1.0, 0.0) / _sig_proc->_amplitude);
        demod.at(i) = _sig_proc->equalizeBurst(signal, (_sig_proc->_TOA - _sig_proc->_channelResponseOffset),
                                               *_sig_proc->_feedForwardFilter,
                                               *_sig_proc->_feedbackFilter);
//        LOG(INFO) << "demod(" << demod.at(i)->size() << "bit) = " << *demod.at(i);

        //TODO 如果TOA不等于0 需要根据TOA的值取bit
        if (i == 0) {
            output = demod.at(i)->segment(_gts_size, insert_tsc_per_bytes * 8 * 2);
        } else {
            output = SoftVector(output, demod.at(i)->segment(_gts_size, insert_tsc_per_bytes * 8 * 2));
        }
        _sig_proc->DestroyChannelRespAndDFE();
    }


    signalVector signal_end;
    std::vector<std::complex<short>> input_end(input.size() - i * seg_size);
    memcpy(input_end.data(), &input.at(i * seg_size), sizeof(std::complex<short>) * (input.size() - i * seg_size));
    Sc16ToSignalVector(input_end, signal_end, 1);

    _sig_proc->ResetDemodParamters();
    bool success = _sig_proc->analyzeTrafficBurst(signal_end, _detect_threshold, 1,
                                                  0, true,
                                                  &_sig_proc->_channelResponse,
                                                  &_sig_proc->_channelResponseOffset);
    if (!success) {
//        LOG(INFO) << "success = " << success;
        output.clear();
        return -1;
    }

    snr += ML_SNR(signal_end);
    snr /= (float) tsc_num;

    _sig_proc->scaleVector(*_sig_proc->_channelResponse, fcomplex(1.0, 0.0) / _sig_proc->_amplitude);
    _sig_proc->designDFE(*_sig_proc->_channelResponse, float(1.0 / _sig_proc->_noisePwr), 7,
                         &_sig_proc->_feedForwardFilter, &_sig_proc->_feedbackFilter);
    _sig_proc->scaleVector(signal_end, fcomplex(1.0, 0.0) / _sig_proc->_amplitude);
    demod.at(i) = _sig_proc->equalizeBurst(signal_end, (_sig_proc->_TOA - _sig_proc->_channelResponseOffset),
                                           *_sig_proc->_feedForwardFilter,
                                           *_sig_proc->_feedbackFilter);
//    LOG(INFO) << "demod(" << demod.at(i)->size() << "bit) = " << *demod.at(i);

    //TODO 如果TOA不等于0 需要根据TOA的值取bit
    output = SoftVector(output, demod.at(i)->segment(
            _gts_size, (input.size() - _gts_size - GUARDPERIODLENGTH - i * seg_size)));
    _sig_proc->DestroyChannelRespAndDFE();

    for (auto &d: demod) {
        if (d) {
            delete d;
            d = nullptr;
        }
    }

    return 0;
}

void GmskDemodulator::Sc16ToSignalVector(const std::vector<std::complex<short>> &input,
                                         signalVector &output, double amp) {
    if (amp == 0) {
        amp = 1;
    }
    output.resize(input.size());
    for (size_t i = 0; i < input.size(); ++i) {
        output[i] = fcomplex(input.at(i).real() / amp, input.at(i).imag() / amp);
    }
}

double GmskDemodulator::ML_SNR(signalVector signal) {

    size_t recv_mid_size = _sig_proc->_gMidambles.at(_TSC).sequence_all->size();//相关长度

    _sig_proc->scaleVector(signal, fcomplex(1.0, 0.0) / _sig_proc->_amplitude);//归一化幅度
    _sig_proc->delayVector(signal, -_sig_proc->_TOA);//处理延迟

    signalVector recv_mid(signal.begin(), 0, recv_mid_size);//接收的训练序列
    signalVector::iterator recv_midPtr = recv_mid.begin();
    signalVector::iterator midPtr = _sig_proc->_gMidambles.at(_TSC).sequence_all->begin();//本地训练序列


    float Nss = 1;//每符号的采样点数
    float h = 0.0;
    float r = 0.0;
    for (; recv_midPtr < recv_mid.end(); recv_midPtr++) {
        h += (recv_midPtr->conj() * (*midPtr)).real();
        r += recv_midPtr->norm2();
        midPtr++;
    }

    h /= (float) recv_mid_size;//取平均
    h *= h;

    r /= (float) recv_mid_size;

    double snr = (Nss * Nss * h) / float(r - Nss * h);

    return (10.0 * log10(snr));
}