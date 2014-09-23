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

#ifndef REAL3_H
#define REAL3_H

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/real.h"
#include "chrono_parallel/math/real2.h"

#define R3  real3
#define ZERO_VECTOR R3(0)

#ifdef ENABLE_SSE
static const __m128 SIGNMASK = _mm_castsi128_ps(_mm_set1_epi32(0x80000000));
#endif

class CHRONO_ALIGN_16 real3 {
 public:
   union {
      struct {
         real x, y, z;
      };
#ifdef ENABLE_SSE
      __m128 mmvalue;
#endif
   };

#ifdef ENABLE_SSE
   inline real3() : mmvalue(_mm_setzero_ps()) {}
   inline real3(real a) : mmvalue(_mm_set1_ps(a)) {}
   inline real3(real a, real b, real c) : mmvalue(_mm_setr_ps(a,b,c,0)) {}
   inline real3(__m128 m) : mmvalue(m) {}

   inline operator __m128() const {return mmvalue;}

   inline real3 operator+(const real3& b) const {return _mm_add_ps(mmvalue, b.mmvalue);}
   inline real3 operator-(const real3& b) const {return _mm_sub_ps(mmvalue, b.mmvalue);}
   inline real3 operator*(const real3& b) const {return _mm_mul_ps(mmvalue, b.mmvalue);}
   inline real3 operator/(const real3& b) const {return _mm_div_ps(mmvalue, b.mmvalue);}
   inline real3 operator-() const {return _mm_xor_ps(mmvalue, SIGNMASK);}

   inline real3 operator+(real b) const {return _mm_add_ps(mmvalue, _mm_set1_ps(b));}
   inline real3 operator-(real b) const {return _mm_sub_ps(mmvalue, _mm_set1_ps(b));}
   inline real3 operator*(real b) const {return _mm_mul_ps(mmvalue, _mm_set1_ps(b));}
   inline real3 operator/(real b) const {return _mm_div_ps(mmvalue, _mm_set1_ps(b));}

   inline real dot(const real3 &b) const {return _mm_cvtss_f32(_mm_dp_ps(mmvalue, b.mmvalue, 0x71));}
   inline real length() const {return _mm_cvtss_f32(_mm_sqrt_ss(_mm_dp_ps(mmvalue, mmvalue, 0x71)));}
   inline real rlength() const {return _mm_cvtss_f32(_mm_rsqrt_ss(_mm_dp_ps(mmvalue, mmvalue, 0x71)));}
   inline real3 normalize() const {return _mm_div_ps(mmvalue, _mm_sqrt_ps(_mm_dp_ps(mmvalue, mmvalue, 0x7F)));}

   inline real3 cross(const real3 &b) const {
      return _mm_sub_ps(
            _mm_mul_ps(_mm_shuffle_ps(mmvalue, mmvalue, _MM_SHUFFLE(3, 0, 2, 1)), _mm_shuffle_ps(b.mmvalue, b.mmvalue, _MM_SHUFFLE(3, 1, 0, 2))),
            _mm_mul_ps(_mm_shuffle_ps(mmvalue, mmvalue, _MM_SHUFFLE(3, 1, 0, 2)), _mm_shuffle_ps(b.mmvalue, b.mmvalue, _MM_SHUFFLE(3, 0, 2, 1)))
      );
   }
#else
   inline real3()
         : x(0), y(0), z(0) {
   }
   inline real3(real a)
         : x(a), y(a), z(a) {
   }
   inline real3(real a,
                real b,
                real c)
         : x(a), y(b), z(c) {
   }

   inline real3 operator+(const real3& b) const {
      return real3(x + b.x, y + b.y, z + b.z);
   }
   inline real3 operator-(const real3& b) const {
      return real3(x - b.x, y - b.y, z - b.z);
   }
   inline real3 operator*(const real3& b) const {
      return real3(x * b.x, y * b.y, z * b.z);
   }
   inline real3 operator/(const real3& b) const {
      return real3(x / b.x, y / b.y, z / b.z);
   }
   inline real3 operator-() const {
      return real3(-x, -y, -z);
   }

   inline real3 operator+(real b) const {
      return real3(x + b, y + b, z + b);
   }
   inline real3 operator-(real b) const {
      return real3(x - b, y - b, z - b);
   }
   inline real3 operator*(real b) const {
      return real3(x * b, y * b, z * b);
   }
   inline real3 operator/(real b) const {
      return real3(x / b, y / b, z / b);
   }

   inline real dot(const real3 &b) const {
      return x * b.x + y * b.y + z * b.z;
   }
   inline real length() const {
      return sqrt(x * x + y * y + z * z);
   }
   inline real rlength() const {
      return real(1.0) / length();
   }
   inline real3 normalize() const {
      return real3(x, y, z) * rlength();
   }

   inline real3 cross(const real3 &b) const {
      real3 result;
      result.x = (y * b.z) - (z * b.y);
      result.y = (z * b.x) - (x * b.z);
      result.z = (x * b.y) - (y * b.x);
      return result;
   }

#endif
   inline bool operator ==(const real3 &b) {
      return ((x == b.x) && (y == b.y) && (z == b.z));
   }

   inline real3& operator+=(const real3& b) {
      *this = *this + b;
      return *this;
   }
   inline real3& operator-=(const real3& b) {
      *this = *this - b;
      return *this;
   }
   inline real3& operator*=(const real3& b) {
      *this = *this * b;
      return *this;
   }
   inline real3& operator/=(const real3& b) {
      *this = *this / b;
      return *this;
   }

   inline real3& operator+=(real b) {
      *this = *this + b;
      return *this;
   }
   inline real3& operator-=(real b) {
      *this = *this - b;
      return *this;
   }
   inline real3& operator*=(real b) {
      *this = *this * b;
      return *this;
   }
   inline real3& operator/=(real b) {
      *this = *this / b;
      return *this;
   }

};

inline real3 operator+(real a,
                       const real3& b) {
   return b + a;
}
inline real3 operator-(real a,
                       const real3& b) {
   return real3(a) - b;
}
inline real3 operator*(real a,
                       const real3& b) {
   return b * a;
}
inline real3 operator/(real a,
                       const real3& b) {
   return real3(a) / b;
}
inline bool operator ==(const real3 &a,
                        const real3 &b) {
   return a == b;
}

inline real dot(const real3& a,
                const real3& b) {
   return a.dot(b);
}
inline real3 cross(const real3& a,
                   const real3& b) {
   return a.cross(b);
}
inline real length(const real3& a) {
   return a.length();
}
inline real rlength(const real3& a) {
   return a.rlength();
}
inline real3 normalize(const real3& a) {
   return a.normalize();
}

static inline std::ostream &operator<<(std::ostream &out,
                                  const real3 &a) {
   out << "[" << a.x << ", " << a.y << ", " << a.z << "]" << std::endl;
   return out;
}

static inline real3 ceil(const real3 &a) {
   return R3(std::ceil(a.x), std::ceil(a.y), std::ceil(a.z));
}
static inline real3 lerp(const real3 &a,
                         const real3 &b,
                         real alpha) {
   return (a + alpha * (b - a));
}
static inline real3 fabs(const real3 &a) {
   return R3(std::fabs(a.x), std::fabs(a.y), std::fabs(a.z));
}
static inline bool isEqual(const real3 &a,
                           const real3 &b) {
   return isEqual(a.x, b.x) && isEqual(a.y, b.y) && isEqual(a.z, b.z);
}
static inline bool IsZero(const real3 &a) {
   return IsZero(a.x) && IsZero(a.y) && IsZero(a.z);
}

#endif
