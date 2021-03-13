//
// Created by Administrator on 2019/7/31.
//

#ifndef DSP_USB_HPP
#define DSP_USB_HPP

#include <iostream>
#include <boost/shared_ptr.hpp>

namespace terjin{
class USB {
public:
    typedef boost::shared_ptr<USB> sptr;

    static sptr make();

    virtual bool CheckIfExists(uint16_t idVendor,uint16_t idProduct) = 0;

    virtual int reset_usb() = 0;

    virtual int reset_usb(uint16_t idVendor,uint16_t idProduct) = 0;
};
}

#endif //DSP_USB_HPP
