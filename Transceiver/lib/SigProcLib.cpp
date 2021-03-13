//
// Created by locate on 2019/10/21.
//

#include "SigProcLib.hpp"
#include <iostream>
#include <glog/logging.h>
#include <memory>
#include <vector>

using namespace std;

SigProcLib::SigProcLib(int num_per_packet, int TSC, int samplesPerSymbol) {
    assert((TSC >= 0) && (TSC <= 8));

    _samples_per_symbol = samplesPerSymbol;
    _TSC = TSC;

    if (_num_per_packet < num_per_packet) {
        _num_per_packet = num_per_packet + 1;
    }

    _amplitude = 1.0;
    _noisePwr = 0.001 / sqrtf(2);

    // add 1 element for wrap around
    _cosTable.resize(TABLESIZE + 2, 0.0);
    _sinTable.resize(TABLESIZE + 2, 0.0);

    _gMidambles.resize(8);

    _GMSKRotation = std::make_shared<signalVector>(_num_per_packet * _samples_per_symbol);
    _GMSKReverseRotation = std::make_shared<signalVector>(_num_per_packet * _samples_per_symbol);
}

SigProcLib::~SigProcLib() {
    DestroyChannelRespAndDFE();
    for (auto &g : _gMidambles) {
        delete g.sequence_all;
        delete g.sequence;
        delete g.sequenceReversedConjugated;
    }
}

SigProcLib::sptr SigProcLib::make(int num_per_packet, int TSC, int samplesPerSymbol) {
    return std::make_shared<SigProcLib>(num_per_packet, TSC, samplesPerSymbol);
}


// dB relative to 1.0.
// if > 1.0, then return 0 dB
float SigProcLib::dB(float x) {

    float arg = 1.0F;
    float dB = 0.0F;

    if (x >= 1.0F) return 0.0F;
    if (x <= 0.0F) return -200.0F;

    float prevArg = arg;
    float prevdB = dB;
    float stepSize = 16.0F;
    float dBstepSize = 12.0F;
    while (stepSize > 1.0F) {
        do {
            prevArg = arg;
            prevdB = dB;
            arg /= stepSize;
            dB -= dBstepSize;
        } while (arg > x);
        arg = prevArg;
        dB = prevdB;
        stepSize *= 0.5F;
        dBstepSize -= 3.0F;
    }
    return ((arg - x) * (dB - 3.0F) + (x - arg * 0.5F) * dB) / (arg - arg * 0.5F);
}

// 10^(-dB/10), inverse of dB func.
float SigProcLib::dBinv(float x) {

    float arg = 1.0F;
    float dB = 0.0F;

    if (x >= 0.0F) return 1.0F;
    if (x <= -200.0F) return 0.0F;

    float prevArg = arg;
    float prevdB = dB;
    float stepSize = 16.0F;
    float dBstepSize = 12.0F;
    while (stepSize > 1.0F) {
        do {
            prevArg = arg;
            prevdB = dB;
            arg /= stepSize;
            dB -= dBstepSize;
        } while (dB > x);
        arg = prevArg;
        dB = prevdB;
        stepSize *= 0.5F;
        dBstepSize -= 3.0F;
    }

    return ((dB - x) * (arg * 0.5F) + (x - (dB - 3.0F)) * (arg)) / 3.0F;
}

float SigProcLib::vectorNorm2(const signalVector &x) {
    signalVector::const_iterator xPtr = x.begin();
    float Energy = 0.0;
    for (; xPtr != x.end(); xPtr++) {
        Energy += xPtr->norm2();
    }
    return Energy;
}


float SigProcLib::vectorPower(const signalVector &x) {
    return vectorNorm2(x) / x.size();
}

/** compute cosine via lookup table */
float SigProcLib::cosLookup(const float x) {
    float arg = x * M_1_2PI_F;
    while (arg > 1.0F) arg -= 1.0F;
    while (arg < 0.0F) arg += 1.0F;

    const float argT = arg * ((float) TABLESIZE);
    const int argI = (int) argT;
    const float delta = argT - argI;
    const float iDelta = 1.0F - delta;
    return iDelta * _cosTable.at(argI) + delta * _cosTable.at(argI + 1);
}

/** compute sine via lookup table */
float SigProcLib::sinLookup(const float x) {
    float arg = x * M_1_2PI_F;
    while (arg > 1.0F) arg -= 1.0F;
    while (arg < 0.0F) arg += 1.0F;

    const float argT = arg * ((float) TABLESIZE);
    const int argI = (int) argT;
    const float delta = argT - argI;
    const float iDelta = 1.0F - delta;
    return iDelta * _sinTable.at(argI) + delta * _sinTable.at(argI + 1);
}


/** compute e^(-jx) via lookup table. */
fcomplex SigProcLib::expjLookup(float x) {
    float arg = x * M_1_2PI_F;
    while (arg > 1.0F) arg -= 1.0F;
    while (arg < 0.0F) arg += 1.0F;

    const float argT = arg * ((float) TABLESIZE);
    const int argI = (int) argT;
    const float delta = argT - argI;
    const float iDelta = 1.0F - delta;
    return fcomplex(iDelta * _cosTable.at(argI) + delta * _cosTable.at(argI + 1),
                    iDelta * _sinTable.at(argI) + delta * _sinTable.at(argI + 1));
}

/** Library setup functions */
void SigProcLib::initTrigTables() {
    for (int i = 0; i < TABLESIZE + 1; i++) {
        _cosTable.at(i) = cos(2.0 * M_PI * i / TABLESIZE);
        _sinTable.at(i) = sin(2.0 * M_PI * i / TABLESIZE);
    }
}

void SigProcLib::initGMSKRotationTables() {

    signalVector::iterator rotPtr = _GMSKRotation->begin();
    signalVector::iterator revPtr = _GMSKReverseRotation->begin();
    float phase = 0.0;
    while (rotPtr != _GMSKRotation->end()) {
        *rotPtr++ = expjLookup(phase);
        *revPtr++ = expjLookup(-phase);
        phase += M_PI_F / 2.0F / (float) _samples_per_symbol;
    }
}

void SigProcLib::sigProcLibSetup() {
    initTrigTables();//初始化三角函数表
    initGMSKRotationTables();//初始化GMSK相位轨迹表???
}

void SigProcLib::GMSKRotate(signalVector &x) {
    signalVector::iterator xPtr = x.begin();

    signalVector::iterator rotPtr = _GMSKRotation->begin();

    if (x.isRealOnly()) {
        while (xPtr < x.end()) {

            *xPtr = (*rotPtr++) * (xPtr->real());

            xPtr++;
        }
    } else {
        while (xPtr < x.end()) {
            *xPtr = *rotPtr++ * (*xPtr);

            xPtr++;
        }
    }
}

void SigProcLib::GMSKReverseRotate(signalVector &x) {
    signalVector::iterator xPtr = x.begin();
    signalVector::iterator rotPtr = _GMSKReverseRotation->begin();
    if (x.isRealOnly()) {
        while (xPtr < x.end()) {
            *xPtr = *rotPtr++ * (xPtr->real());
            xPtr++;
        }
    } else {
        while (xPtr < x.end()) {
            *xPtr = *rotPtr++ * (*xPtr);
            xPtr++;
        }
    }
}


signalVector *SigProcLib::convolve(const signalVector *a, const signalVector *b, signalVector *c,
                                   ConvType spanType, unsigned startIx, unsigned len) {
    if ((a == nullptr) || (b == nullptr)) return nullptr;
    int La = a->size();
    int Lb = b->size();

    int startIndex;
    unsigned int outSize;
    switch (spanType) {
        case FULL_SPAN:
            startIndex = 0;
            outSize = La + Lb - 1;
            break;
        case OVERLAP_ONLY:
            startIndex = La;
            outSize = abs(La - Lb) + 1;
            break;
        case START_ONLY:
            startIndex = 0;
            outSize = La;
            break;
        case WITH_TAIL:
            startIndex = Lb;
            outSize = La;
            break;
        case NO_DELAY:
            if (Lb % 2)
                startIndex = Lb / 2;
            else
                startIndex = Lb / 2 - 1;
            outSize = La;
            break;
        case CUSTOM:
            startIndex = startIx;
            outSize = len;
            break;
        default:
            return nullptr;
    }

    if (c == nullptr)
        c = new signalVector(outSize);
    else if (c->size() != outSize)
        return nullptr;

    signalVector::const_iterator aStart = a->begin();
    signalVector::const_iterator bStart = b->begin();
    signalVector::const_iterator aEnd = a->end();
    signalVector::const_iterator bEnd = b->end();
    signalVector::iterator cPtr = c->begin();
    int t = startIndex;
    int stopIndex = startIndex + outSize;

    switch (b->getSymmetry()) {
        case NONE: {
            while (t < stopIndex) {
                signalVector::const_iterator aP = aStart + t;
                signalVector::const_iterator bP = bStart;
                if (a->isRealOnly() && b->isRealOnly()) {
                    float sum = 0.0;
                    while (bP < bEnd) {
                        if (aP < aStart) break;
                        if (aP < aEnd) sum += (aP->real()) * (bP->real());
                        aP--;
                        bP++;
                    }
                    *cPtr++ = sum;
                } else if (a->isRealOnly()) {
                    fcomplex sum = 0.0;
                    while (bP < bEnd) {
                        if (aP < aStart) break;
                        if (aP < aEnd) sum += (*bP) * (aP->real());
                        aP--;
                        bP++;
                    }
                    *cPtr++ = sum;
                } else if (b->isRealOnly()) {
                    fcomplex sum = 0.0;
                    while (bP < bEnd) {
                        if (aP < aStart) break;
                        if (aP < aEnd) sum += (*aP) * (bP->real());
                        aP--;
                        bP++;
                    }
                    *cPtr++ = sum;
                } else {
                    fcomplex sum = 0.0;
                    while (bP < bEnd) {
                        if (aP < aStart) break;
                        if (aP < aEnd) sum += (*aP) * (*bP);
                        aP--;
                        bP++;
                    }
                    *cPtr++ = sum;
                }
                t++;
            }
        }
            break;
        case ABSSYM: {
            fcomplex sum = 0.0;

            bool isOdd = (bool) (Lb % 2);
            if (isOdd)
                bEnd = bStart + (Lb + 1) / 2;
            else
                bEnd = bStart + Lb / 2;


            while (t < stopIndex) {
                signalVector::const_iterator aP = aStart + t;
                signalVector::const_iterator aPsym = aP - Lb + 1;
                signalVector::const_iterator bP = bStart;
                sum = 0.0;
                if (!b->isRealOnly()) {
                    while (bP < bEnd) {
                        if (aP < aStart) break;
                        if (aP == aPsym)
                            sum += (*aP) * (*bP);
                        else if ((aP < aEnd) && (aPsym >= aStart))
                            sum += ((*aP) + (*aPsym)) * (*bP);
                        else if (aP < aEnd)
                            sum += (*aP) * (*bP);
                        else if (aPsym >= aStart)
                            sum += (*aPsym) * (*bP);
                        aP--;
                        aPsym++;
                        bP++;
                    }
                } else {
                    while (bP < bEnd) {
                        if (aP < aStart) break;
                        if (aP == aPsym)
                            sum += (*aP) * (bP->real());
                        else if ((aP < aEnd) && (aPsym >= aStart))
                            sum += ((*aP) + (*aPsym)) * (bP->real());
                        else if (aP < aEnd)
                            sum += (*aP) * (bP->real());
                        else if (aPsym >= aStart)
                            sum += (*aPsym) * (bP->real());
                        aP--;
                        aPsym++;
                        bP++;
                    }
                }
                *cPtr++ = sum;
                t++;
            }
        }
            break;
        default:
            return nullptr;
    }
    return c;
}


void SigProcLib::generateGSMPulse(int symbolLength) {

    int numSamples = _samples_per_symbol * symbolLength + 1;
    _gsmPulse.resize(numSamples);
    signalVector::iterator gsmIt = _gsmPulse.begin();
    int centerPoint = (numSamples - 1) / 2;
    for (int i = 0; i < numSamples; i++) {
        float arg = (float) (i - centerPoint) / (float) _samples_per_symbol;
        *gsmIt++ = 0.96 * exp(-1.1380 * arg * arg - 0.527 * arg * arg * arg * arg); // GSM pulse approx.
    }

    float avgAbsval = sqrtf(vectorNorm2(_gsmPulse) / (float) _samples_per_symbol);
    gsmIt = _gsmPulse.begin();
    for (int i = 0; i < numSamples; i++)
        *gsmIt++ /= avgAbsval;
    _gsmPulse.isRealOnly(true);
    _gsmPulse.setSymmetry(ABSSYM);
}

signalVector *
SigProcLib::frequencyShift(signalVector *y, signalVector *x, float freq, float startPhase, float *finalPhase) {

    if (!x) return nullptr;

    if (y == nullptr) {
        y = new signalVector(x->size());
        y->isRealOnly(x->isRealOnly());
        if (y == nullptr) return nullptr;
    }

    if (y->size() < x->size()) return nullptr;

    float phase = startPhase;
    signalVector::iterator yP = y->begin();
    signalVector::iterator xPEnd = x->end();
    signalVector::iterator xP = x->begin();

    if (x->isRealOnly()) {
        while (xP < xPEnd) {
            (*yP++) = expjLookup(phase) * ((xP++)->real());
            phase += freq;
        }
    } else {
        while (xP < xPEnd) {
            (*yP++) = (*xP++) * expjLookup(phase);
            phase += freq;
        }
    }


    if (finalPhase) *finalPhase = phase;

    return y;
}

signalVector *SigProcLib::reverseConjugate(signalVector *b) {
    auto *tmp = new signalVector(b->size());
    tmp->isRealOnly(b->isRealOnly());
    signalVector::iterator bP = b->begin();
    signalVector::iterator bPEnd = b->end();
    signalVector::iterator tmpP = tmp->end() - 1;
    if (!b->isRealOnly()) {
        while (bP < bPEnd) {
            *tmpP-- = bP->conj();
            bP++;
        }
    } else {
        while (bP < bPEnd) {
            *tmpP-- = bP->real();
            bP++;
        }
    }

    return tmp;
}

signalVector *SigProcLib::correlate(signalVector *a, signalVector *b, signalVector *c,
                                    ConvType spanType, bool bReversedConjugated, unsigned startIx, unsigned len) {
    signalVector *tmp = nullptr;

    if (!bReversedConjugated) {
        tmp = reverseConjugate(b);
    } else {
        tmp = b;
    }

    c = convolve(a, tmp, c, spanType, startIx, len);

    if (!bReversedConjugated) delete tmp;

    return c;
}


/* soft output slicer */
bool SigProcLib::vectorSlicer(signalVector *x) {

    signalVector::iterator xP = x->begin();
    signalVector::iterator xPEnd = x->end();
    while (xP < xPEnd) {
        *xP = (fcomplex) (0.5 * (xP->real() + 1.0F));
        if (xP->real() > 1.0) *xP = 1.0;
        if (xP->real() < 0.0) *xP = 0.0;
        xP++;
    }
    return true;
}

signalVector *SigProcLib::modulateBurst(const BitVector &wBurst, const signalVector &gsmPulse, int guardPeriodLength) {

    int burstSize = _samples_per_symbol * (wBurst.size() + guardPeriodLength);
    signalVector modBurst(burstSize);// = new signalVector(burstSize);
    modBurst.isRealOnly(true);
    modBurst.fill(0.0);
    signalVector::iterator modBurstItr = modBurst.begin();

    // if wBurst are the raw bits
    /* Raw bits are not differentially encoded */
    for (unsigned int i = 0; i < wBurst.size(); i++) {
        *modBurstItr = 2.0 * (wBurst[i] & 0x01) - 1.0;
        modBurstItr += _samples_per_symbol;
    }

    // shift up pi/2
    // ignore starting phase, since spec allows for discontinuous phase
    GMSKRotate(modBurst);

    modBurst.isRealOnly(false);

    // filter w/ pulse shape
    /* Single Gaussian pulse approximation shaping */
    signalVector *shapedBurst = convolve(&modBurst, &gsmPulse, nullptr, NO_DELAY);

    return shapedBurst;
}

float SigProcLib::sinc(float x) {
    if ((x >= 0.01F) || (x <= -0.01F)) return (sinLookup(x) / x);
    return 1.0F;
}

void SigProcLib::delayVector(signalVector &wBurst, float delay) {

    int intOffset = (int) floor(delay);//=delay  floor向下取整
    float fracOffset = delay - intOffset;

    // do fractional shift first, only do it for reasonable offsets
    if (fabs(fracOffset) > 1e-2) {
        // create sinc function
        signalVector sincVector(21);
        sincVector.isRealOnly(true);
        signalVector::iterator sincBurstItr = sincVector.begin();
        for (int i = 0; i < 21; i++)
            *sincBurstItr++ = (fcomplex) sinc(M_PI_F * (i - 10 - fracOffset));

        signalVector shiftedBurst(wBurst.size());
        convolve(&wBurst, &sincVector, &shiftedBurst, NO_DELAY);
        wBurst.clone(shiftedBurst);
    }

    if (intOffset < 0) {
        intOffset = -intOffset;
        signalVector::iterator wBurstItr = wBurst.begin();
        signalVector::iterator shiftedItr = wBurst.begin() + intOffset;
        while (shiftedItr < wBurst.end())
            *wBurstItr++ = *shiftedItr++;
        while (wBurstItr < wBurst.end())
            *wBurstItr++ = 0.0;
    } else {
        signalVector::iterator wBurstItr = wBurst.end() - 1;
        signalVector::iterator shiftedItr = wBurst.end() - 1 - intOffset;
        while (shiftedItr >= wBurst.begin())
            *wBurstItr-- = *shiftedItr--;
        while (wBurstItr >= wBurst.begin())
            *wBurstItr-- = 0.0;
    }
}

signalVector *SigProcLib::gaussianNoise(int length, float variance, fcomplex mean) {

    auto *noise = new signalVector(length);
    signalVector::iterator nPtr = noise->begin();
    float stddev = sqrtf(variance);
    while (nPtr < noise->end()) {
        float u1 = (float) rand() / (float) RAND_MAX;
        while (u1 == 0.0)
            u1 = (float) rand() / (float) RAND_MAX;
        float u2 = (float) rand() / (float) RAND_MAX;
        float arg = 2.0 * M_PI * u2;
        *nPtr = mean + stddev * fcomplex(cos(arg), sin(arg)) * sqrtf(-2.0 * log(u1));
        nPtr++;
    }

    return noise;
}

fcomplex SigProcLib::interpolatePoint(const signalVector &inSig, float ix) {
    int start = (int) (floor(ix) - 10);
    if (start < 0) start = 0;
    int end = (int) (floor(ix) + 11);
    if ((unsigned) end > inSig.size() - 1) end = inSig.size() - 1;

    fcomplex pVal = 0.0;
    if (!inSig.isRealOnly()) {
        for (int i = start; i < end; i++)
            pVal += inSig[i] * sinc(M_PI_F * (i - ix));
    } else {
        for (int i = start; i < end; i++)
            pVal += inSig[i].real() * sinc(M_PI_F * (i - ix));
    }

    return pVal;
}


fcomplex SigProcLib::peakDetect(const signalVector &rxBurst, float *peakIndex, float *avgPwr) {

    fcomplex maxVal = 0.0;
    float maxIndex = -1;
    float sumPower = 0.0;

    for (unsigned int i = 0; i < rxBurst.size(); i++) {
        float samplePower = rxBurst[i].norm2();
        if (samplePower > maxVal.real()) {
            maxVal = samplePower;
            maxIndex = i;
        }
        sumPower += samplePower;
    }

    // interpolate around the peak
    // to save computation, we'll use early-late balancing
    float earlyIndex = maxIndex - 1;
    float lateIndex = maxIndex + 1;

    float incr = 0.5;
    while (incr > 1.0 / 1024.0) {
        fcomplex earlyP = interpolatePoint(rxBurst, earlyIndex);
        fcomplex lateP = interpolatePoint(rxBurst, lateIndex);
        if (earlyP < lateP)
            earlyIndex += incr;
        else if (earlyP > lateP)
            earlyIndex -= incr;
        else break;
        incr /= 2.0;
        lateIndex = earlyIndex + 2.0;
    }

    maxIndex = earlyIndex + 1.0;
    maxVal = interpolatePoint(rxBurst, maxIndex);

    if (peakIndex != nullptr)
        *peakIndex = maxIndex;

    if (avgPwr != nullptr)
        *avgPwr = (sumPower - maxVal.norm2()) / (rxBurst.size() - 1);

    return maxVal;
}

//Apply a scalar to a vector.
void SigProcLib::scaleVector(signalVector &x, const fcomplex &scale) {
    signalVector::iterator xP = x.begin();
    signalVector::iterator xPEnd = x.end();
    if (!x.isRealOnly()) {
        while (xP < xPEnd) {
            *xP = *xP * scale;
            xP++;
        }
    } else {
        while (xP < xPEnd) {
            *xP = xP->real() * scale;
            xP++;
        }
    }
}

/** in-place conjugation */
void SigProcLib::conjugateVector(signalVector &x) {
    if (x.isRealOnly()) return;
    signalVector::iterator xP = x.begin();
    signalVector::iterator xPEnd = x.end();
    while (xP < xPEnd) {
        *xP = xP->conj();
        xP++;
    }
}


// in-place addition!!
bool SigProcLib::addVector(signalVector &x, signalVector &y) {
    signalVector::iterator xP = x.begin();
    signalVector::iterator yP = y.begin();
    signalVector::iterator xPEnd = x.end();
    signalVector::iterator yPEnd = y.end();
    while ((xP < xPEnd) && (yP < yPEnd)) {
        *xP = *xP + *yP;
        xP++;
        yP++;
    }
    return true;
}

// in-place multiplication!!
bool SigProcLib::multVector(signalVector &x, signalVector &y) {
    signalVector::iterator xP = x.begin();
    signalVector::iterator yP = y.begin();
    signalVector::iterator xPEnd = x.end();
    signalVector::iterator yPEnd = y.end();
    while ((xP < xPEnd) && (yP < yPEnd)) {
        *xP = (*xP) * (*yP);
        xP++;
        yP++;
    }
    return true;
}


void SigProcLib::offsetVector(signalVector &x, const fcomplex &offset) {
    signalVector::iterator xP = x.begin();
    signalVector::iterator xPEnd = x.end();
    if (!x.isRealOnly()) {
        while (xP < xPEnd) {
            *xP += offset;
            xP++;
        }
    } else {
        while (xP < xPEnd) {
            *xP = xP->real() + offset;
            xP++;
        }
    }
}

bool SigProcLib::generateMidamble() {

    signalVector emptyPulse(1);
    *(emptyPulse.begin()) = 1.0;

    // only use middle 16 bits of each TSC
    signalVector *middleMidamble = modulateBurst(gTrainingSequence[_TSC].segment(5, 16), emptyPulse, 0);
    signalVector *midamble = modulateBurst(gTrainingSequence[_TSC], _gsmPulse, 0);

    if (midamble == nullptr) return false;
    if (middleMidamble == nullptr) return false;

    // NOTE: Because ideal TSC 16-bit midamble is 5 symbols into burst,
    //       the ideal TSC has an + 90 degree phase shift,
    //       due to the pi/2 frequency shift, that
    //       needs to be accounted for.
    //       26-midamble is 0 symbols into burst, has +0 degree phase shift.
    scaleVector(*middleMidamble, fcomplex(0.0, 1.0));//根据TSC位置决定需不需要添加这行代码
//    scaleVector(*midamble, fcomplex(1.0, 0.0));//根据TSC位置决定需不需要添加这行代码

    signalVector *autocorr = correlate(midamble, middleMidamble, nullptr, NO_DELAY);

    if (autocorr == nullptr) return false;

    _gMidambles.at(_TSC).sequence_all = midamble;
    _gMidambles.at(_TSC).sequence = middleMidamble;
    _gMidambles.at(_TSC).sequenceReversedConjugated = reverseConjugate(middleMidamble);
    _gMidambles.at(_TSC).gain = peakDetect(*autocorr, &_gMidambles.at(_TSC).TOA, nullptr);
//    LOG(INFO) << "TOA: " << _gMidambles.at(_TSC).TOA;

    //gMidambles[_TSC]->TOA -= 5*_samples_per_symbol;

    delete autocorr;

    return true;
}

bool SigProcLib::generateRACHSequence() {

    signalVector *RACHSeq = modulateBurst(gRACHSynchSequence, _gsmPulse, 0);
    assert(RACHSeq);

    signalVector *autocorr = correlate(RACHSeq, RACHSeq, nullptr, NO_DELAY);
    assert(autocorr);

    _gRACHSequence.sequence = RACHSeq;
    _gRACHSequence.sequenceReversedConjugated = reverseConjugate(RACHSeq);
    _gRACHSequence.gain = peakDetect(*autocorr, &_gRACHSequence.TOA, nullptr);

    delete autocorr;

    return true;
}


bool SigProcLib::detectRACHBurst(signalVector &rxBurst, float detectThreshold, fcomplex *amplitude, float *TOA) {

    //static fcomplex staticData[500];

    //signalVector correlatedRACH(staticData,0,rxBurst.size());
    signalVector correlatedRACH(rxBurst.size());
    correlate(&rxBurst, _gRACHSequence.sequenceReversedConjugated, &correlatedRACH, NO_DELAY, true);

    float meanPower;
    fcomplex peakAmpl = peakDetect(correlatedRACH, TOA, &meanPower);

    float valleyPower = 0.0;

    // check for bogus results
    if ((*TOA < 0.0) || (*TOA > correlatedRACH.size())) {
        *amplitude = 0.0;
        return false;
    }
    fcomplex *peakPtr = correlatedRACH.begin() + (int) rint(*TOA);

    float numSamples = 0.0;
    for (int i = 57 * _samples_per_symbol; i <= 107 * _samples_per_symbol; i++) {
        if (peakPtr + i >= correlatedRACH.end())
            break;
        valleyPower += (peakPtr + i)->norm2();
        numSamples++;
    }

    if (numSamples < 2) {
        *amplitude = 0.0;
        return false;
    }

    //RMS root mean sqrt 均方根
    float RMS = sqrtf(valleyPower / (float) numSamples) + 0.00001;
    float peakToMean = peakAmpl.abs() / RMS;

    *amplitude = peakAmpl / (_gRACHSequence.gain);

    *TOA = (*TOA) - _gRACHSequence.TOA - 8 * _samples_per_symbol;

    return (peakToMean > detectThreshold);
}

bool SigProcLib::energyDetect(signalVector &rxBurst, unsigned windowLength, float detectThreshold, float *avgPwr) {

    signalVector::const_iterator windowItr = rxBurst.begin();
    float energy = 0.0;
    if (windowLength <= 0) windowLength = 20;
    if (windowLength > rxBurst.size()) windowLength = rxBurst.size();
    for (unsigned i = 0; i < windowLength; i++) {
        energy += windowItr->norm2();
        windowItr += 4;
    }
    if (avgPwr) *avgPwr = energy / windowLength;
    return (energy / windowLength > detectThreshold * detectThreshold);
}


bool SigProcLib::analyzeTrafficBurst(signalVector &rxBurst, float detectThreshold, unsigned maxTOA,
                                     unsigned tsc_startIx,
                                     bool requestChannel, signalVector **channelResponse,
                                     float *channelResponseOffset) {

    if (maxTOA < 3 * (unsigned) _samples_per_symbol) {
        maxTOA = 3 * _samples_per_symbol;
    }
    unsigned spanTOA = maxTOA;
    if (spanTOA < 5 * (unsigned) _samples_per_symbol) {
        spanTOA = 5 * _samples_per_symbol;
    }

    unsigned mid_tscIx = (tsc_startIx + 5) * _samples_per_symbol;
    unsigned mid_tsc_len = _gMidambles.at(_TSC).sequence->size();
    unsigned startIx = (mid_tscIx - spanTOA) * _samples_per_symbol;
    unsigned endIx = (mid_tscIx + mid_tsc_len + spanTOA) * _samples_per_symbol;
    unsigned windowLen = endIx - startIx;
    unsigned corrLen = 2 * maxTOA + 1;

    auto expectedTOAPeak = (unsigned) round(
            _gMidambles.at(_TSC).TOA + (_gMidambles.at(_TSC).sequenceReversedConjugated->size() - 1) / 2);

    signalVector burstSegment(rxBurst.begin(), startIx, windowLen);

    signalVector correlatedBurst(corrLen);
    correlate(&burstSegment, _gMidambles.at(_TSC).sequenceReversedConjugated,
              &correlatedBurst, CUSTOM, true,
              expectedTOAPeak - maxTOA, corrLen);

    float meanPower = 0.0;
    _amplitude = peakDetect(correlatedBurst, &_TOA, &meanPower);
    float valleyPower = 0.0;
    fcomplex *peakPtr = correlatedBurst.begin() + (int) rint(_TOA);

    // check for bogus results
    if ((_TOA < 0.0) || (_TOA > correlatedBurst.size())) {
        _amplitude = 0.0;
        return false;
    }

    int numRms = 0;
    for (int i = 2 * _samples_per_symbol; i <= 5 * _samples_per_symbol; i++) {
        if (peakPtr - i >= correlatedBurst.begin()) {
            valleyPower += (peakPtr - i)->norm2();
            numRms++;
        }
        if (peakPtr + i < correlatedBurst.end()) {
            valleyPower += (peakPtr + i)->norm2();
            numRms++;
        }
    }

    if (numRms < 2) {
        // check for bogus results
        _amplitude = 0.0;
        return false;
    }

    float RMS = sqrtf(valleyPower / (float) numRms) + 0.00001;
    float peakToMean = (_amplitude.abs()) / RMS;

    _amplitude = (_amplitude) / _gMidambles.at(_TSC).gain;
    _TOA = (_TOA) - (maxTOA);
//    LOG(INFO) << "correlatedBurst: " << correlatedBurst;
//    LOG(INFO) << "TOA: " << _TOA << ", amp: " << _amplitude << ", peakToMean: " << peakToMean << ", RMS: " << RMS;

    if (requestChannel && (peakToMean > detectThreshold)) {
        float TOAoffset = maxTOA;
        delayVector(correlatedBurst, -(_TOA));
        // midamble only allows estimation of a 6-tap channel
        signalVector channelVector(6 * _samples_per_symbol);
        float maxEnergy = -1.0;
        int maxI = -1;
        for (int i = 0; i < 7; i++) {
            if (TOAoffset + (i - 5) * _samples_per_symbol + channelVector.size() > correlatedBurst.size()) continue;
            if (TOAoffset + (i - 5) * _samples_per_symbol < 0) continue;
            correlatedBurst.segmentCopyTo(channelVector, (int) floor(TOAoffset + (i - 5) * _samples_per_symbol),
                                          channelVector.size());
            float energy = vectorNorm2(channelVector);
            if (energy > 0.95 * maxEnergy) {
                maxI = i;
                maxEnergy = energy;
            }
        }

        *channelResponse = new signalVector(channelVector.size());
        correlatedBurst.segmentCopyTo(**channelResponse, (int) floor(TOAoffset + (maxI - 5) * _samples_per_symbol),
                                      (*channelResponse)->size());
        scaleVector(**channelResponse, fcomplex(1.0, 0.0) / _gMidambles.at(_TSC).gain);

        if (channelResponseOffset)
            *channelResponseOffset = 5 * _samples_per_symbol - maxI;

    }

    return (peakToMean > detectThreshold);
}

signalVector *SigProcLib::decimateVector(signalVector &wVector, int decimationFactor) {

    if (decimationFactor <= 1) return nullptr;

    auto *decVector = new signalVector(wVector.size() / decimationFactor);
    decVector->isRealOnly(wVector.isRealOnly());

    signalVector::iterator vecItr = decVector->begin();
    for (unsigned int i = 0; i < wVector.size(); i += decimationFactor)
        *vecItr++ = wVector[i];

    return decVector;
}

void SigProcLib::ResetDemodParamters() {
    _amplitude = 1.0;//The estimated amplitude of received TSC burst.
    _TOA = 0.0;//The estimate time-of-arrival of received TSC burst.
    _channelResponseOffset = 0.0;//The time offset b/w the first sample of the channel response and the reported TOA.

    _requestChannel = true;//Set to true if channel estimation is desired.
    DestroyChannelRespAndDFE();
}


SoftVector *SigProcLib::demodulateBurst(signalVector &rxBurst, const fcomplex &channel) {
    //Apply a scalar to a vector.
    scaleVector(rxBurst, ((fcomplex) 1.0) / channel);
    delayVector(rxBurst, -_TOA);

    signalVector *shapedBurst = &rxBurst;

    // shift up by a quarter of a frequency
    // ignore starting phase, since spec allows for discontinuous phase
    GMSKReverseRotate(*shapedBurst);

    // run through slicer
    if (_samples_per_symbol > 1) {
        signalVector *decShapedBurst = decimateVector(*shapedBurst, _samples_per_symbol);
        shapedBurst = decShapedBurst;
    }

    vectorSlicer(shapedBurst);

    auto *burstBits = new SoftVector(shapedBurst->size());

    SoftVector::iterator burstItr = burstBits->begin();
    signalVector::iterator shapedItr = shapedBurst->begin();
    for (; shapedItr < shapedBurst->end(); shapedItr++)
        *burstItr++ = shapedItr->real();

    if (_samples_per_symbol > 1) delete shapedBurst;

    return burstBits;
}


// 1.0 is sampling frequency
// must satisfy cutoffFreq > 1/filterLen
signalVector *SigProcLib::createLPF(float cutoffFreq, int filterLen, float gainDC) {

    auto *LPF = new signalVector(filterLen - 1);
    LPF->isRealOnly(true);
    LPF->setSymmetry(ABSSYM);
    signalVector::iterator itr = LPF->begin();
    double sum = 0.0;
    for (int i = 1; i < filterLen; i++) {
        float ys = sinc(M_2PI_F * cutoffFreq * ((float) i - (float) (filterLen) / 2.0F));
        float yg = 4.0F * cutoffFreq;
        // Blackman -- less brickwall (sloping transition) but larger stopband attenuation
        float yw = 0.42 - 0.5 * cos(((float) i) * M_2PI_F / (float) (filterLen)) +
                   0.08 * cos(((float) i) * 2 * M_2PI_F / (float) (filterLen));
        // Hamming -- more brickwall with smaller stopband attenuation
        //float yw = 0.53836F - 0.46164F * cos(((float)i)*M_2PI_F/(float)(filterLen+1));
        *itr++ = (fcomplex) ys * yg * yw;
        sum += ys * yg * yw;
    }

    float normFactor = gainDC / sum; //sqrtf(gainDC/vectorNorm2(*LPF));
    // normalize power
    itr = LPF->begin();
    for (int i = 1; i < filterLen; i++) {
        *itr = *itr * normFactor;
        itr++;
    }
    return LPF;

}


#define POLYPHASESPAN 10

// assumes filter group delay is 0.5*(length of filter)
signalVector *SigProcLib::polyphaseResampleVector(signalVector &wVector, int P, int Q, signalVector *LPF) {

    bool deleteLPF = false;

    if (LPF == nullptr) {
        float cutoffFreq = (P < Q) ? (1.0 / (float) Q) : (1.0 / (float) P);
        LPF = createLPF(cutoffFreq / 3.0, 100 * POLYPHASESPAN + 1, Q);
        deleteLPF = true;
    }

    auto *resampledVector = new signalVector((int) ceil(wVector.size() * (float) P / (float) Q));
    resampledVector->fill(0);
    resampledVector->isRealOnly(wVector.isRealOnly());
    signalVector::iterator newItr = resampledVector->begin();

    //FIXME: need to update for real-only vectors
    int outputIx = (LPF->size() + 1) / 2 / Q; //((P > Q) ? P : Q);
    while (newItr < resampledVector->end()) {
        int outputBranch = (outputIx * Q) % P;
        int inputOffset = (outputIx * Q - outputBranch) / P;
        signalVector::const_iterator inputItr = wVector.begin() + inputOffset;
        signalVector::const_iterator filtItr = LPF->begin() + outputBranch;
        while (inputItr >= wVector.end()) {
            inputItr--;
            filtItr += P;
        }
        fcomplex sum = 0.0;
        if ((LPF->getSymmetry() != ABSSYM) || (P > 1)) {
            if (!LPF->isRealOnly()) {
                while ((inputItr >= wVector.begin()) && (filtItr < LPF->end())) {
                    sum += (*inputItr) * (*filtItr);
                    inputItr--;
                    filtItr += P;
                }
            } else {
                while ((inputItr >= wVector.begin()) && (filtItr < LPF->end())) {
                    sum += (*inputItr) * (filtItr->real());
                    inputItr--;
                    filtItr += P;
                }
            }
        } else {
            signalVector::const_iterator revInputItr = inputItr - LPF->size() + 1;
            signalVector::const_iterator filtMidpoint = LPF->begin() + (LPF->size() - 1) / 2;
            if (!LPF->isRealOnly()) {
                while (filtItr <= filtMidpoint) {
                    if (inputItr < revInputItr) break;
                    if (inputItr == revInputItr)
                        sum += (*inputItr) * (*filtItr);
                    else if ((inputItr < wVector.end()) && (revInputItr >= wVector.begin()))
                        sum += (*inputItr + *revInputItr) * (*filtItr);
                    else if (inputItr < wVector.end())
                        sum += (*inputItr) * (*filtItr);
                    else if (revInputItr >= wVector.begin())
                        sum += (*revInputItr) * (*filtItr);
                    inputItr--;
                    revInputItr++;
                    filtItr++;
                }
            } else {
                while (filtItr <= filtMidpoint) {
                    if (inputItr < revInputItr) break;
                    if (inputItr == revInputItr)
                        sum += (*inputItr) * (filtItr->real());
                    else if ((inputItr < wVector.end()) && (revInputItr >= wVector.begin()))
                        sum += (*inputItr + *revInputItr) * (filtItr->real());
                    else if (inputItr < wVector.end())
                        sum += (*inputItr) * (filtItr->real());
                    else if (revInputItr >= wVector.begin())
                        sum += (*revInputItr) * (filtItr->real());
                    inputItr--;
                    revInputItr++;
                    filtItr++;
                }
            }
        }
        *newItr = sum;
        newItr++;
        outputIx++;
    }

    if (deleteLPF) delete LPF;

    return resampledVector;
}


signalVector *SigProcLib::resampleVector(signalVector &wVector, float expFactor, const fcomplex &endPoint) {

    if (expFactor < 1.0) return nullptr;

    auto *retVec = new signalVector((int) ceil(wVector.size() * expFactor));

    float t = 0.0;

    signalVector::iterator retItr = retVec->begin();
    while (retItr < retVec->end()) {
        auto tLow = (unsigned int) floor(t);
        unsigned tHigh = tLow + 1;
        if (tLow > wVector.size() - 1) break;
        if (tHigh > wVector.size()) break;
        fcomplex lowPoint = wVector[tLow];
        fcomplex highPoint = (tHigh == wVector.size()) ? endPoint : wVector[tHigh];
        fcomplex a = (tHigh - t);
        fcomplex b = (t - tLow);
        *retItr = (a * lowPoint + b * highPoint);
        t += 1.0 / expFactor;
    }

    return retVec;

}


// Assumes symbol-spaced sampling!!!
// Based upon paper by Al-Dhahir and Cioffi
bool SigProcLib::designDFE(signalVector &channelResponse, float SNRestimate, int Nf, signalVector **feedForwardFilter,
                           signalVector **feedbackFilter) {

    signalVector G0(Nf);
    signalVector G1(Nf);
    signalVector::iterator G0ptr = G0.begin();
    signalVector::iterator G1ptr = G1.begin();
    signalVector::iterator chanPtr = channelResponse.begin();

    int nu = channelResponse.size() - 1;

    *G0ptr = 1.0 / sqrtf(SNRestimate);
    for (int j = 0; j <= nu; j++) {
        *G1ptr = chanPtr->conj();
        G1ptr++;
        chanPtr++;
    }

    signalVector *L[Nf];
    signalVector::iterator Lptr;
    float d = 0.0;
    for (int i = 0; i < Nf; i++) {
        d = G0.begin()->norm2() + G1.begin()->norm2();
        L[i] = new signalVector(Nf + nu);
        Lptr = L[i]->begin() + i;
        G0ptr = G0.begin();
        G1ptr = G1.begin();
        while ((G0ptr < G0.end()) && (Lptr < L[i]->end())) {
            *Lptr = (*G0ptr * (G0.begin()->conj()) + *G1ptr * (G1.begin()->conj())) / d;
            Lptr++;
            G0ptr++;
            G1ptr++;
        }
        fcomplex k = (*G1.begin()) / (*G0.begin());

        if (i != Nf - 1) {
            signalVector G0new = G1;
            scaleVector(G0new, k.conj());
            addVector(G0new, G0);

            signalVector G1new = G0;
            scaleVector(G1new, k * (-1.0));
            addVector(G1new, G1);
            delayVector(G1new, -1.0);

            scaleVector(G0new, 1.0 / sqrtf(1.0 + k.norm2()));
            scaleVector(G1new, 1.0 / sqrtf(1.0 + k.norm2()));
            G0 = G0new;
            G1 = G1new;
        }
    }

    *feedbackFilter = new signalVector(nu);
    L[Nf - 1]->segmentCopyTo(**feedbackFilter, Nf, nu);
    scaleVector(**feedbackFilter, (fcomplex) -1.0);
    conjugateVector(**feedbackFilter);

    signalVector v(Nf);
    signalVector::iterator vStart = v.begin();
    signalVector::iterator vPtr;
    *(vStart + Nf - 1) = (fcomplex) 1.0;
    for (int k = Nf - 2; k >= 0; k--) {
        Lptr = L[k]->begin() + k + 1;
        vPtr = vStart + k + 1;
        fcomplex v_k = 0.0;
        for (int j = k + 1; j < Nf; j++) {
            v_k -= (*vPtr) * (*Lptr);
            vPtr++;
            Lptr++;
        }
        *(vStart + k) = v_k;
    }

    *feedForwardFilter = new signalVector(Nf);
    signalVector::iterator w = (*feedForwardFilter)->begin();
    for (int i = 0; i < Nf; i++) {
        delete L[i];
        fcomplex w_i = 0.0;
        int endPt = (nu < (Nf - 1 - i)) ? nu : (Nf - 1 - i);
        vPtr = vStart + i;
        chanPtr = channelResponse.begin();
        for (int k = 0; k < endPt + 1; k++) {
            w_i += (*vPtr) * (chanPtr->conj());
            vPtr++;
            chanPtr++;
        }
        *w = w_i / d;
        w++;
    }


    return true;

}

// Assumes symbol-rate sampling!!!!
SoftVector *
SigProcLib::equalizeBurst(signalVector &rxBurst, float TOA, signalVector &feedForwardFilter,
                          signalVector &feedbackFilter) {

    delayVector(rxBurst, -TOA);

    signalVector *postForwardFull = convolve(&rxBurst, &feedForwardFilter, nullptr, FULL_SPAN);

    auto *postForward = new signalVector(rxBurst.size());
    postForwardFull->segmentCopyTo(*postForward, feedForwardFilter.size() - 1, rxBurst.size());
    delete postForwardFull;

    signalVector::iterator dPtr = postForward->begin();
    signalVector::iterator dBackPtr;
    signalVector::iterator rotPtr = _GMSKRotation->begin();
    signalVector::iterator revRotPtr = _GMSKReverseRotation->begin();

    auto *DFEoutput = new signalVector(postForward->size());
    signalVector::iterator DFEItr = DFEoutput->begin();

    // NOTE: can insert the midamble and/or use midamble to estimate BER
    for (; dPtr < postForward->end(); dPtr++) {
        dBackPtr = dPtr - 1;
        signalVector::iterator bPtr = feedbackFilter.begin();
        while ((bPtr < feedbackFilter.end()) && (dBackPtr >= postForward->begin())) {
            *dPtr = *dPtr + (*bPtr) * (*dBackPtr);
            bPtr++;
            dBackPtr--;
        }
        *dPtr = *dPtr * (*revRotPtr);
        *DFEItr = *dPtr;
        // make decision on symbol
        *dPtr = (dPtr->real() > 0.0) ? 1.0 : -1.0;
        //*DFEItr = *dPtr;
        *dPtr = *dPtr * (*rotPtr);
        DFEItr++;
        rotPtr++;
        revRotPtr++;
    }

    vectorSlicer(DFEoutput);

    auto *burstBits = new SoftVector(postForward->size());
    SoftVector::iterator burstItr = burstBits->begin();
    DFEItr = DFEoutput->begin();
    for (; DFEItr < DFEoutput->end(); DFEItr++)
        *burstItr++ = DFEItr->real();

    delete postForward;
    delete DFEoutput;

    return burstBits;
}

void SigProcLib::DestroyChannelRespAndDFE() {
    if (_channelResponse) {
        delete _channelResponse;
        _channelResponse = nullptr;
    }

    if (_feedForwardFilter) {
        delete _feedForwardFilter;
        _feedForwardFilter = nullptr;
    }

    if (_feedbackFilter) {
        delete _feedbackFilter;
        _feedbackFilter = nullptr;
    }
}
