#ifndef REAL_H
#define REAL_H

#include "chrono_parallel/ChParallelDefines.h"
#include <float.h>
typedef float real;
#define ZERO_EPSILON FLT_EPSILON


static inline real clamp(const real & a, const real & clamp_min, const real & clamp_max) {
	if (a < clamp_min) {
		return clamp_min;
	} else if (a > clamp_max) {
		return clamp_max;
	} else {
		return a;
	}

}

static inline real lerp(const real &a, const real &b, real alpha) {
	return (a + alpha * (b - a));

}

static inline bool IsZero(const real &a) {
	return fabs(a) < ZERO_EPSILON;
}


static inline bool isEqual(const real &_a, const real &_b) {
	real ab;
	ab = fabs(_a - _b);
	if (fabs(ab) < ZERO_EPSILON)
		return 1;
	real a, b;
	a = fabs(_a);
	b = fabs(_b);
	if (b > a) {
		return ab < ZERO_EPSILON * b;
	} else {
		return ab < ZERO_EPSILON * a;
	}
}


static inline real sign(const real &x) {
	if (x < 0) {
		return -1;
	} else if (x > 0) {
		return 1;
	} else {
		return 0;
	}
}


template<class T>
real max3(const T &a) {
	return max(a.x, max(a.y, a.z));
}
template<class T>
real min3(const T &a) {
	return min(a.x, min(a.y, a.z));
}

#endif
