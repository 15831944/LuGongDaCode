//
// Created by locate on 12/5/2020.
//

#ifndef TRANSCEIVERDEMO_BPSKIMPL_HPP
#define TRANSCEIVERDEMO_BPSKIMPL_HPP

#include <ModulatorBase.hpp>

const BitVector BPSKTrainingSequence[] = {
        BitVector("00100101110000100010010111"),
        BitVector("00101101110111100010110111"),
        BitVector("01000011101110100100001110"),
        BitVector("01000111101101000100011110"),
        BitVector("00011010111001000001101011"),
        BitVector("01001110101100000100111010"),
        BitVector("10100111110110001010011111"),
        BitVector("11101111000100101110111100"),
};

class BPSKImpl : public ModulatorBase, public DemodulatorBase {
private:
    typedef struct {
        std::vector<std::complex<float>> sequence_all{};//训练序列
        std::vector<std::complex<float>> sequence{};//训练序列的中间16位
        std::vector<std::complex<float>> sequenceReversedConjugated{};//翻转共轭
        float TOA{};//相关值最大的下标
        float gain{};//相关值
    } CorrelationSequence;

public:
    typedef std::shared_ptr<BPSKImpl> sptr;

    CorrelationSequence _midambles;//本地训练序列

    int _TSC = 0;

    int _iq_number = 0;

    std::vector<std::complex<float>> _guard;

    static const int _factor = 4;

    float _detect_threshold = 8.0;

    RootRaisedCosine::sptr _filter;
    std::vector<std::complex<float>> _h;

    unsigned int _expected_TOA_peak = 0;
public:

    static sptr make(int IQNumbers, int TSC);

    explicit BPSKImpl(int IQNumbers, int TSC);

    ~BPSKImpl() override;

    /**
     * Transfer the information bits to a BPSK constellation
     * @param input: an array of 0 and 1
     * @param output: complex array of float.
     * output is filled with -1 and 1 according to
     *
     * 0对应载波相位 2π,已调信号为:cos(Wc*T) = cos(Wc*T)
     * 1对应载波相位  π,已调信号为:cos(Wc*T + π) = -cos(Wc*T)
     */
    void BPSKMod(const BitVector &input, std::vector<std::complex<float>> &output);

    /**
     * Make hard decision based on BPSK constellation
     * @param input: complex with the data
     * @param output: an array to be filled with 0 1
     */
    void BPSKDemod(const std::vector<std::complex<float>> &data, SoftVector &output);

    void GenerateMidamble();

    static std::vector<std::complex<float>> ReverseConjugate(const std::vector<std::complex<float>> &input);

    static std::vector<std::complex<float>> Correlate(const std::vector<std::complex<float>> &a,
                                                      const std::vector<std::complex<float>> &b,
                                                      bool b_reversed_conjugated, unsigned start_index, unsigned len);

    static std::vector<std::complex<float>> Convolve(const std::vector<std::complex<float>> &a,
                                                     const std::vector<std::complex<float>> &b,
                                                     unsigned start_index, unsigned len);

    static std::complex<float>
    PeakDetect(const std::vector<std::complex<float>> &input, float *peakIndex, float *avgPwr);

    void UpSample(int factor, std::vector<std::complex<float>> &input);

    void DownSample(int factor, std::vector<std::complex<float >> &input);

    void ModWork(int tsc_num, int insert_tsc_per_bytes, const BitVector &input,
                 std::vector<std::complex<float>> &output) override;

    int DemodWork(int actual_mtu, int tsc_num, int insert_tsc_per_bytes,
                  const std::vector<std::complex<short>> &input, SoftVector &output, double &snr) override;

    bool AnalyzeTrafficBurst(std::vector<std::complex<float>> &input, float *amplitude, float *TOA);

    bool DelayVector(std::vector<std::complex<float>> &input, float TOA);

    void ScaleVector(std::vector<std::complex<float>> &input, std::complex<float> amp);

    bool CopeFreqOffset(std::vector<std::complex<float>> &input);

    bool CopePhaseOffset(std::vector<std::complex<float>> &input);

};


#endif //TRANSCEIVERDEMO_BPSKIMPL_HPP
