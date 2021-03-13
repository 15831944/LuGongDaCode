#ifndef SIGPROCLIB_H
#define SIGPROCLIB_H
#include <fftw3.h>
#include <common/Types.hpp>
#include <common/Complex.hpp>

void initTrigTables();
float sinLookup(const float x);
float cosLookup(const float x);
float fast_atan2f(float y, float x);
fcomplex powtheta(fcomplex z, int n);
#endif
