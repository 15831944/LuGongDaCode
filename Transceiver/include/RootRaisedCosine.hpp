//
// Created by locate on 2020/3/15.
//

#ifndef PULSESHAPE_HPP
#define PULSESHAPE_HPP

#include <iostream>
#include <glog/logging.h>
#include <vector>
#include <cmath>
#include <memory>
#include <complex>

class RootRaisedCosine {
public:
    typedef std::shared_ptr<RootRaisedCosine> sptr;

    //! Constructor
    RootRaisedCosine() = default;

    //! Constructor
    explicit RootRaisedCosine(double roll_off_factor, int filter_length, int upsampling_factor);

    static sptr make();

    static sptr make(double roll_off_factor, int filter_length, int upsampling_factor);

    //! Destructor
    ~RootRaisedCosine() = default;

    //! Set pulse_shape, roll_off_factor between 0 and 1, filter_length even
    void SetPulseShape(double roll_off_factor, int filter_length, int upsampling_factor);

    //! Get the Roll-off factor
    double GetRollOff() const;

    std::vector<std::complex<float>> GetImpulseResponse();

protected:
    //! The roll off factor (i.e. alpha)
    double _roll_off_factor{};

    //! Samples per input symbol
    int _upsampling_factor{};

    //! Length in symbols
    int _pulse_length{};

    //! The impulse resounse of the pulse shaping filter
    std::vector<std::complex<float>> _impulse_response;
};


#endif //PULSESHAPE_HPP
