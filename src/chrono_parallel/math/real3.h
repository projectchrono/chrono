// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: SSE and normal implementation of a 3D vector
// =============================================================================

#pragma once

#include <algorithm>

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/real.h"
#include "chrono_parallel/math/real2.h"
#include "chrono_parallel/math/sse.h"

namespace chrono {

class real3 {
public:
	inline real3() {
	}
	inline explicit real3(real a) :
			array { a, a, a, 0 } {
	}
	inline real3(real a, real b, real c) :
			array { a, b, c, 0 } {
	}
	inline real3(const real3& v) :
			array { v.x, v.y, v.z, 0 } {
	}
	inline real operator[](unsigned int i) const {
		return array[i];
	}
	inline real& operator[](unsigned int i) {
		return array[i];
	}
	inline real3& operator=(const real3& rhs) {
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		w = 0;
		return *this;  // Return a reference to myself.
	}

#if defined(USE_AVX)
	inline real3(__m256d m) {
		_mm256_storeu_pd(&array[0], m);
	}
	inline operator __m256d() const {
		return _mm256_loadu_pd(&array[0]);
	}
	inline real3& operator=(const __m256d& rhs) {
		_mm256_storeu_pd(&array[0], rhs);
		return *this;
	}
	static inline __m256d Set(real x) {
		return _mm256_set1_pd(x);
	}
	static inline __m256d Set(real x, real y, real z) {
		return _mm256_setr_pd(x, y, z, 0.0);
	}

#elif defined(USE_SSE)
	inline real3(__m128 m) {_mm_storeu_ps(&array[0], m);}
	inline operator __m128() const {return _mm_loadu_ps(&array[0]);}
	inline real3& operator=(const __m128& rhs) {
		_mm_storeu_ps(&array[0], rhs);
		return *this;
	}
	static inline __m128 Set(real x) {
		return _mm_set1_ps(x);
	}
	static inline __m128 Set(real x, real y, real z) {
		return _mm_setr_ps(x, y, z, 0.0f);
	}
#else
	static inline real3 Set(real x) {
		return real3(x);
	}
	static inline real3 Set(real x, real y, real z) {
		return real3(x, y, z);
	}
#endif
	// ========================================================================================
	union {
		real array[4];
		struct {
			real x, y, z, w;
		};
	};
};

real3 operator+(const real3& a, real b);
real3 operator-(const real3& a, real b);
real3 operator*(const real3& a, real b);
real3 operator/(const real3& a, real b);

real3 operator+(const real3& a, const real3& b);
real3 operator-(const real3& a, const real3& b);
real3 operator*(const real3& a, const real3& b);
real3 operator/(const real3& a, const real3& b);

OPERATOR_EQUALSALT(*, real, real3)
OPERATOR_EQUALSALT(/, real, real3)
OPERATOR_EQUALSALT(+, real, real3)
OPERATOR_EQUALSALT(-, real, real3)

OPERATOR_EQUALSALT(*, real3, real3)
OPERATOR_EQUALSALT(/, real3, real3)
OPERATOR_EQUALSALT(+, real3, real3)
OPERATOR_EQUALSALT(-, real3, real3)

real3 operator-(const real3& a);
real3 operator*(real lhs, const real3& rhs);
real3 operator/(real lhs, const real3& rhs);

bool operator<(const real3& lhs, const real3& rhs);
bool operator>(const real3& lhs, const real3& rhs);
bool operator==(const real3& lhs, const real3& rhs);

real3 Cross(const real3& b, const real3& c);
real Dot(const real3& v1, const real3& v2);
real Dot(const real3& v);
real3 Normalize(const real3& v);
real3 Sqrt(const real3& v);
real3 Round(const real3& v);
real Length(const real3& v);

static inline real Length2(const real3& v1) {
	return Dot(v1);
}

inline real SafeLength(const real3& v) {
	real len_sq = Length2(v);
	if (len_sq) {
		return Sqrt(len_sq);
	} else {
		return 0.0f;
	}
}

inline real3 SafeNormalize(const real3& v, const real3& safe = real3(0)) {
	real len_sq = Length2(v);
	if (len_sq > real(0)) {
		return v * InvSqrt(len_sq);
	} else {
		return safe;
	}
}

real Max(const real3& a);
real Min(const real3& a);
real3 Max(const real3& a, const real3& b);
real3 Min(const real3& a, const real3& b);
real3 Max(const real3& a, const real& b);
real3 Min(const real3& a, const real& b);

bool IsZero(const real3& v);

real3 Abs(const real3& v);
real3 Sign(const real3& v);
static inline real3 Clamp(const real3& v, real max_length) {
	real3 x = v;
	real len_sq = Dot(x);
	real inv_len = InvSqrt(len_sq);

	if (len_sq > Sqr(max_length))
		x *= inv_len * max_length;

	return x;
}
real3 Clamp(const real3& a, const real3& clamp_min, const real3& clamp_max);

//
// static inline real3 Clamp(const real3& a, const real3& clamp_min, const real3& clamp_max) {
//    real3 clampv;
//    clampv.x = Clamp(a.x, clamp_min.x, clamp_max.x);
//    clampv.y = Clamp(a.y, clamp_min.y, clamp_max.y);
//    clampv.z = Clamp(a.z, clamp_min.z, clamp_max.z);
//    return clampv;
//}
//
real3 OrthogonalVector(const real3& v);
real3 UnitOrthogonalVector(const real3& v);
//
static inline void Sort(real& a, real& b, real& c) {
	if (a > b)
		Swap(a, b);
	if (b > c)
		Swap(b, c);
	if (a > b)
		Swap(a, b);
}
static void Print(real3 v, const char* name) {
	printf("%s\n", name);
	printf("%f %f %f\n", v[0], v[1], v[2]);
}
}
