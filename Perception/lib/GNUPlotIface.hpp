//
// Created by root on 19-3-14.
//

#ifndef DSP_GNUPLOTIFACE_HPP
#define DSP_GNUPLOTIFACE_HPP

#include <boost/shared_ptr.hpp>
#include <terjin/config.h>
#include <string>

namespace terjin {
    class  GNUPlotIface {
    public:
        typedef boost::shared_ptr<GNUPlotIface> sptr;

        static sptr make();

        GNUPlotIface() = default;;

        virtual ~GNUPlotIface() = default;;;

        virtual void plot(double *buf, int number) = 0;

        virtual void plot(short *buf, int number) = 0;

        virtual void plot(float *buf, int number) = 0;

        virtual void plotter(const std::string &command) = 0;

        virtual void plot(double *x, double *y, int number) = 0;

        virtual void plot(float *x, float *y, int number) = 0;

    };

}


#endif //DSP_GNUPLOTIFACE_HPP
