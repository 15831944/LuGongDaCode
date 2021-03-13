//
// Created by locate on 2019/10/21.
//

#ifndef SIGPROCLIB_HPP
#define SIGPROCLIB_HPP

#include "Vector.hpp"
#include "Complex.hpp"
#include "BitVector.hpp"
#include "Types.hpp"


#include <memory>
#include <complex>

#define TABLESIZE 1024

const BitVector gTrainingSequence[] = {
        BitVector("00100101110000100010010111"),
        BitVector("00101101110111100010110111"),
        BitVector("01000011101110100100001110"),
        BitVector("01000111101101000100011110"),
        BitVector("00011010111001000001101011"),
        BitVector("01001110101100000100111010"),
        BitVector("10100111110110001010011111"),
        BitVector("11101111000100101110111100"),
};

const BitVector gDummyBurst(
        "0001111101101110110000010100100111000001001000100000001111100011100010111000101110001010111010010100011001100111001111010011111000100101111101010000");

const BitVector gRACHSynchSequence("01001011011111111001100110101010001111000");

/** Constants */
static const float M_PI_F = (float) M_PI;
static const float M_2PI_F = (float) (2.0 * M_PI);
static const float M_1_2PI_F = 1 / M_2PI_F;


/** Indicated signalVector symmetry */
enum Symmetry {
    NONE = 0,
    ABSSYM = 1
};

/** Convolution type indicator */
enum ConvType {
    FULL_SPAN = 0,
    OVERLAP_ONLY = 1,
    START_ONLY = 2,
    WITH_TAIL = 3,
    NO_DELAY = 4,
    CUSTOM = 5,
    UNDEFINED = 255
};

/** the core data structure of the Transceiver */
class signalVector : public Vector<fcomplex> {

private:

    Symmetry symmetry;   ///< the symmetry of the vector
    bool realOnly;       ///< true if vector is real-valued, not fcomplex-valued

public:

    /** Constructors */
    explicit signalVector(int dSize = 0, Symmetry wSymmetry = NONE) : Vector<fcomplex>(dSize), realOnly(false) {
        symmetry = wSymmetry;
    };

    signalVector(fcomplex *wData, size_t start, size_t span, Symmetry wSymmetry = NONE) :
            Vector<fcomplex>(nullptr, wData + start, wData + start + span), realOnly(false) {
        symmetry = wSymmetry;
    };

    signalVector(const signalVector &vec1, const signalVector &vec2) : Vector<fcomplex>(vec1, vec2), realOnly(false) {
        symmetry = vec1.symmetry;
    };

    signalVector(const signalVector &wVector) : Vector<fcomplex>(wVector.size()), realOnly(false) {
        wVector.copyTo(*this);
        symmetry = wVector.getSymmetry();
    };

    static void signalVectorToSc32(const signalVector &input, std::vector<std::complex<float>> &output) {
        int inSize = input.size();
        output.resize(inSize, std::complex<float>(0.f, 0.f));
        memcpy(output.data(), input.mData, sizeof(std::complex<float>) * inSize);
    }

    /** symmetry operators */
    Symmetry getSymmetry() const { return symmetry; };

    void setSymmetry(Symmetry wSymmetry) { symmetry = wSymmetry; };

    /** real-valued operators */
    bool isRealOnly() const { return realOnly; };

    void isRealOnly(bool wOnly) { realOnly = wOnly; };
};

class SigProcLib {
private:

    typedef std::shared_ptr<SigProcLib> sptr;

    /** Lookup tables for trigonometric approximation*/
    std::vector<float> _cosTable{};
    std::vector<float> _sinTable{};

    /** Static vectors that contain a precomputed +/- f_b/4 sinusoid */
    //储存相位轨迹
    std::shared_ptr<signalVector> _GMSKRotation{};
    std::shared_ptr<signalVector> _GMSKReverseRotation{};

    /** Static ideal RACH and midamble correlation waveforms */
    typedef struct {
        signalVector *sequence_all{};//训练序列
        signalVector *sequence{};//训练序列的中间16位
        signalVector *sequenceReversedConjugated{};//翻转共轭
        float TOA{};//相关值最大的下标
        fcomplex gain{};//相关值
    } CorrelationSequence;

public:

    int _samples_per_symbol = 1;// The number of samples per GSM symbol.
    int _num_per_packet = 4096;
    int _TSC = 2;//The training sequence [0..7]

    signalVector _gsmPulse;//The GSM pulse used for modulation.

    double _noisePwr{};

    fcomplex _amplitude{};//The estimated amplitude of received TSC burst.
    float _TOA{};//The estimate time-of-arrival of received TSC burst.
    float _channelResponseOffset{};//The time offset b/w the first sample of the channel response and the reported TOA.

    bool _requestChannel = true;//Set to true if channel estimation is desired.
    signalVector *_channelResponse{};//The estimated channel.
    signalVector *_feedForwardFilter{};//The designed feed forward filter.
    signalVector *_feedbackFilter{};//The designed feedback filter.

    std::vector<CorrelationSequence> _gMidambles;
    CorrelationSequence _gRACHSequence;

public:

    SigProcLib(int num_per_packet, int TSC, int samplesPerSymbol = 1);

    ~SigProcLib();

    static sptr make(int num_per_packet, int TSC, int samplesPerSymbol = 1);

    /** Convert a linear number to a dB value */
    static float dB(float x);

    /** Convert a dB value into a linear value */
    static float dBinv(float x);

    /** Compute the energy of a vector */
    static float vectorNorm2(const signalVector &x);

    /** Compute the average power of a vector */
    static float vectorPower(const signalVector &x);

    float cosLookup(float x);

    float sinLookup(float x);

    fcomplex expjLookup(float x);

    void initTrigTables();

    void initGMSKRotationTables();

    /** Setup the signal processing library */
    void sigProcLibSetup();

    void GMSKRotate(signalVector &x);

    void GMSKReverseRotate(signalVector &x);

    /**
     * Convolve two vectors.
     * @param a,b The vectors to be convolved.
     * @param c, A preallocated vector to hold the convolution result.
     * @param spanType The type/span of the convolution.
     * @param startIx
     * @param len
     * @return The convolution result.
     */
    static signalVector *
    convolve(const signalVector *a, const signalVector *b, signalVector *c, ConvType spanType, unsigned startIx = 0,
             unsigned len = 0);

    /**
     * Generate the GSM pulse.
     * @param symbolLength The size of the pulse.
     */
    void generateGSMPulse(int symbolLength = 2);

    /**
     * Frequency shift a vector.
     * @param y The frequency shifted vector.
     * @param x The vector to-be-shifted.
     * @param freq The digital frequency shift
     * @param startPhase The starting phase of the oscillator
     * @param finalPhase The final phase of the oscillator
     * @return The frequency shifted vector.
     */
    signalVector *frequencyShift(signalVector *y, signalVector *x, float freq = 0.0, float startPhase = 0.0,
                                 float *finalPhase = nullptr);

    static signalVector *reverseConjugate(signalVector *b);

    /**
     * Correlate two vectors.
     * @param a,b The vectors to be correlated.
     * @param c, A preallocated vector to hold the correlation result.
     * @param spanType The type/span of the correlation.
     * @param bReversedConjugated
     * @param startIx
     * @param len
     * @return The correlation result.
     */
    static signalVector *
    correlate(signalVector *a, signalVector *b, signalVector *c, ConvType spanType, bool bReversedConjugated = false,
              unsigned startIx = 0, unsigned len = 0);

    /** Operate soft slicer on real-valued portion of vector */
    static bool vectorSlicer(signalVector *x);

    /** GMSK modulate a GSM burst of bits */
    signalVector *modulateBurst(const BitVector &wBurst, const signalVector &gsmPulse, int guardPeriodLength = 0);

    /** Sinc function */
    float sinc(float x);

    /** Delay a vector */
    void delayVector(signalVector &wBurst, float delay);

    static void conjugateVector(signalVector &x);

    /** Add two vectors in-place */
    static bool addVector(signalVector &x, signalVector &y);

    /** Multiply two vectors in-place*/
    static bool multVector(signalVector &x, signalVector &y);

    /** Generate a vector of gaussian noise */
    static signalVector *gaussianNoise(int length, float variance = 1.0, fcomplex mean = fcomplex(0.0));

    /**
     * Given a non-integer index, interpolate a sample.
     * @param inSig The signal from which to interpolate.
     * @param ix The index.
     * @return The interpolated signal value.
     */
    fcomplex interpolatePoint(const signalVector &inSig, float ix);

    /**
     * Given a correlator output, locate the correlation peak.
     * @param rxBurst The correlator result.
     * @param peakIndex Pointer to value to receive interpolated peak index.
     * @param avgPower Power to value to receive mean power.
     * @return Peak value.
     */
    fcomplex peakDetect(const signalVector &rxBurst, float *peakIndex, float *avgPwr);

    /**
     * Apply a scalar to a vector.
     * @param x The vector of interest.
     * @param scale The scalar.
     */
    static void scaleVector(signalVector &x, const fcomplex &scale);

    /**
     * Add a constant offset to a vecotr.
     * @param x The vector of interest.
     * @param offset The offset.
     */
    static void offsetVector(signalVector &x, const fcomplex &offset);

    /**
     * Generate a modulated GSM midamble, stored within the library.
     * @return Success.
     */
    bool generateMidamble();

    /**
     * Generate a modulated RACH sequence, stored within the library.
     * @return Success.
     */
    bool generateRACHSequence();

    /**
     * Energy detector, checks to see if received burst energy is above a threshold.
     * @param rxBurst The received GSM burst of interest.
     * @param windowLength The number of burst samples used to compute burst energy
     * @param detectThreshold The detection threshold, a linear value.
     * @param avgPwr The average power of the received burst.
     * @return True if burst energy is above threshold.
     */
    bool energyDetect(signalVector &rxBurst, unsigned windowLength, float detectThreshold, float *avgPwr = nullptr);

    /**
     * RACH correlator/detector.
     * @param rxBurst The received GSM burst of interest.
     * @param detectThreshold The threshold that the received burst's post-correlator SNR is compared against to determine validity.
     * @param amplitude The estimated amplitude of received RACH burst.
     * @param TOA The estimate time-of-arrival of received RACH burst.
     * @return True if burst SNR is larger that the detectThreshold value.
     */
    bool detectRACHBurst(signalVector &rxBurst, float detectThreshold, fcomplex *amplitude,
                         float *TOA);

    /**
     * Normal burst correlator, detector, channel estimator.
     * @param rxBurst The received GSM burst of interest.
     * @param detectThreshold The threshold that the received burst's post-correlator SNR is compared against to determine validity.
     * @param maxTOA The maximum expected time-of-arrival
     * @param requestChannel Set to true if channel estimation is desired.
     * @param channelResponse The estimated channel.
     * @param channelResponseOffset The time offset b/w the first sample of the channel response and the reported TOA.
     * @return True if burst SNR is larger that the detectThreshold value.
     */
    bool analyzeTrafficBurst(signalVector &rxBurst, float detectThreshold, unsigned maxTOA,
                             unsigned tsc_startIx, bool requestChannel = false,
                             signalVector **channelResponse = nullptr,
                             float *channelResponseOffset = nullptr);

    /**
	 * Decimate a vector.
     * @param wVector The vector of interest.
     * @param decimationFactor The amount of decimation, i.e. the decimation factor.
     * @return The decimated signal vector.
     */
    signalVector *decimateVector(signalVector &wVector, int decimationFactor);

    void ResetDemodParamters();

    /**
     * Demodulates a received burst using a soft-slicer.
	 * @param rxBurst The burst to be demodulated.
     * @return The demodulated bit sequence.
     */
    SoftVector *
    demodulateBurst(signalVector &rxBurst, const fcomplex &channel);

    /**
     * Creates a simple Kaiser-windowed low-pass FIR filter.
     * @param cutoffFreq The digital 3dB bandwidth of the filter.
     * @param filterLen The number of taps in the filter.
     * @param gainDC The DC gain of the filter.
     * @return The desired LPF
     */
    signalVector *createLPF(float cutoffFreq, int filterLen, float gainDC = 1.0);

    /**
	 * Change sampling rate of a vector via polyphase resampling.
     * @param wVector The vector to be resampled.
     * @param P The numerator, i.e. the amount of upsampling.
     * @param Q The denominator, i.e. the amount of downsampling.
	 * @param LPF An optional low-pass filter used in the resampling process.
	 * @return A vector resampled at P/Q of the original sampling rate.
     */
    signalVector *polyphaseResampleVector(signalVector &wVector, int P, int Q, signalVector *LPF);

    /**
	 * Change the sampling rate of a vector via linear interpolation.
	 * @param wVector The vector to be resampled.
	 * @param expFactor Ratio of new sampling rate/original sampling rate.
	 * @param endPoint ???
	 * @return A vector resampled a expFactor*original sampling rate.
     */
    signalVector *resampleVector(signalVector &wVector, float expFactor, const fcomplex &endPoint);

    /**
	 * Design the necessary filters for a decision-feedback equalizer.
	 * @param channelResponse The multipath channel that we're mitigating.
	 * @param SNRestimate The signal-to-noise estimate of the channel, a linear value
	 * @param Nf The number of taps in the feedforward filter.
	 * @param feedForwardFilter The designed feed forward filter.
	 * @param feedbackFilter The designed feedback filter.
	 * @return True if DFE can be designed.
     */
    bool designDFE(signalVector &channelResponse, float SNRestimate, int Nf, signalVector **feedForwardFilter,
                   signalVector **feedbackFilter);

    /**
	 * Equalize/demodulate a received burst via a decision-feedback equalizer.
	 * @param rxBurst The received burst to be demodulated.
	 * @param feedForwardFilter The feed forward filter of the DFE.
	 * @param feedbackFilter The feedback filter of the DFE.
	 * @return The demodulated bit sequence.
     */
    SoftVector *
    equalizeBurst(signalVector &rxBurst, float TOA, signalVector &feedForwardFilter, signalVector &feedbackFilter);

    void DestroyChannelRespAndDFE();

    signalVector GetGsmPulse() { return _gsmPulse; }
};

#endif //SIGPROCLIB_HPP