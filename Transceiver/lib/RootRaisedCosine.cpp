//
// Created by locate on 2020/3/15.
//

#include "RootRaisedCosine.hpp"

RootRaisedCosine::RootRaisedCosine(double roll_off_factor, int filter_length, int upsampling_factor) {
    SetPulseShape(roll_off_factor, filter_length, upsampling_factor);
}

RootRaisedCosine::sptr RootRaisedCosine::make() {
    return std::make_shared<RootRaisedCosine>();
}

RootRaisedCosine::sptr RootRaisedCosine::make(double roll_off_factor, int filter_length, int upsampling_factor) {
    return std::make_shared<RootRaisedCosine>(roll_off_factor, filter_length, upsampling_factor);
}

void RootRaisedCosine::SetPulseShape(double roll_off_factor, int filter_length, int upsampling_factor) {
    if (roll_off_factor <= 0 || roll_off_factor > 1) {
        LOG(INFO) << "Root_Raised_Cosine: roll-off out of range";
        return;
    }

    _roll_off_factor = roll_off_factor;

    double t, num, den, tmp_arg;
    _upsampling_factor = upsampling_factor;
    _pulse_length = filter_length;
    _impulse_response.resize(filter_length * upsampling_factor + 1);

    for (size_t i = 0; i < _impulse_response.size(); i++) {
        // delayed to be casual
        t = (double) (i - filter_length * upsampling_factor / 2.f) / upsampling_factor;
        den = 1 - (4 * roll_off_factor * t) * (4 * roll_off_factor * t);
        if (t == 0) {
            _impulse_response.at(i) = 1 + (4 * roll_off_factor / M_PI) - roll_off_factor;
        } else if (den == 0) {
            tmp_arg = M_PI / (4 * roll_off_factor);
            _impulse_response.at(i) = roll_off_factor / std::sqrt(2.0)
                                      * ((1 + 2 / M_PI) * std::sin(tmp_arg) + (1 - 2 / M_PI) * std::cos(tmp_arg));
        } else {
            num = std::sin(M_PI * (1 - roll_off_factor) * t)
                  + std::cos(M_PI * (1 + roll_off_factor) * t) * 4 * roll_off_factor * t;
            _impulse_response.at(i) = num / (M_PI * t * den);
        }

        _impulse_response.at(i) /= std::sqrt(double(upsampling_factor));
    }

}

double RootRaisedCosine::GetRollOff() const {
    return _roll_off_factor;
}

std::vector<std::complex<float>> RootRaisedCosine::GetImpulseResponse() {
    return _impulse_response;
}
