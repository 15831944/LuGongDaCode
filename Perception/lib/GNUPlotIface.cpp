//
// Created by root on 19-3-14.
//

#include "GNUPlotIface.hpp"
#include "GNUPlot.hpp"

terjin::GNUPlotIface::sptr terjin::GNUPlotIface::make() {
    return terjin::GNUPlotIface::sptr(new GNUPlot());
}
