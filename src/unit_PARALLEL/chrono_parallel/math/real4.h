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
// Description: SSE and normal implementation of a 4D vector/Quaternion
// =============================================================================

#ifndef REAL4_H
#define REAL4_H

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/real2.h"
#include "chrono_parallel/math/real3.h"

#define R4  real4
#ifdef ENABLE_SSE
//#define _mm_shufd(xmm, mask) _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(xmm), mask))

static inline real horizontal_add(const __m128 & a) {
   __m128 t1 = _mm_hadd_ps(a, a);
   __m128 t2 = _mm_hadd_ps(t1, t1);
   return _mm_cvtss_f32(t2);
}

template<int i0, int i1, int i2, int i3>
static inline __m128i constant4i() {
   static const union {
      int i[4];
      __m128i xmm;
   }u = { {i0, i1, i2, i3}};
   return u.xmm;
}

template<int i0, int i1, int i2, int i3>
static inline __m128 change_sign(__m128 const & a) {
   if ((i0 | i1 | i2 | i3) == 0) return a;
   __m128i mask = constant4i<i0 ? 0x80000000 : 0, i1 ? 0x80000000 : 0, i2 ? 0x80000000 : 0, i3 ? 0x80000000 : 0>();
   return _mm_xor_ps(a, _mm_castsi128_ps(mask));     // flip sign bits
}
#endif

class CHRONO_ALIGN_16 real4 {
 public:
   union {
      struct {
         real w, x, y, z;
      };
#ifdef ENABLE_SSE
      __m128 mmvalue;
#endif
   };

#ifdef ENABLE_SSE
   inline real4() : mmvalue(_mm_setzero_ps()) {}
   inline real4(real a) : mmvalue(_mm_set1_ps(a)) {}
   inline real4(real a, real b, real c) : mmvalue(_mm_setr_ps(0, a,b,c)) {}
   inline real4(real3 a) : mmvalue(_mm_setr_ps(0, a.x,a.y,a.z)) {}
   inline real4(real d, real a, real b, real c) : mmvalue(_mm_setr_ps(d, a,b,c)) {}
   inline real4(__m128 m) : mmvalue(m) {}

   operator __m128() const {return mmvalue;}

   inline real4 operator+(const real4& b) const {return _mm_add_ps(mmvalue, b.mmvalue);}
   inline real4 operator-(const real4& b) const {return _mm_sub_ps(mmvalue, b.mmvalue);}
   inline real4 operator*(const real4& b) const {return _mm_mul_ps(mmvalue, b.mmvalue);}
   inline real4 operator/(const real4& b) const {return _mm_div_ps(mmvalue, b.mmvalue);}
   inline real4 operator-() const {return _mm_xor_ps(mmvalue, SIGNMASK);}

   inline real4 operator+(real b) const {return _mm_add_ps(mmvalue, _mm_set1_ps(b));}
   inline real4 operator-(real b) const {return _mm_sub_ps(mmvalue, _mm_set1_ps(b));}
   inline real4 operator*(real b) const {return _mm_mul_ps(mmvalue, _mm_set1_ps(b));}
   inline real4 operator/(real b) const {return _mm_div_ps(mmvalue, _mm_set1_ps(b));}

   inline real dot(const real4 &b) const {__m128 l = _mm_mul_ps(mmvalue,b.mmvalue); return horizontal_add(l);}

   inline real4 inv() const {
      __m128 l = _mm_mul_ps(mmvalue,mmvalue);
      real t1 = horizontal_add(l);
      return real4 (change_sign<0,1,1,1>(mmvalue))/t1;
   }
#else
   inline real4()
         : w(0), x(0), y(0), z(0) {
   }
   inline real4(real a)
         : w(a), x(a), y(a), z(a) {
   }
   inline real4(real a,
                real b,
                real c)
         : w(0), x(a), y(b), z(c) {
   }
   inline real4(real3 a)
         : w(0), x(a.x), y(a.y), z(a.z) {
   }
   inline real4(real d,
                real a,
                real b,
                real c)
         : w(d), x(a), y(b), z(c) {
   }

   inline real4 operator+(const real4& b) const {
      return real4(w + b.w, x + b.x, y + b.y, z + b.z);
   }
   inline real4 operator-(const real4& b) const {
      return real4(w - b.w, x - b.x, y + b.y, z - b.z);
   }
   inline real4 operator*(const real4& b) const {
      return real4(w * b.w, x * b.x, y * b.y, z * b.z);
   }
   inline real4 operator/(const real4& b) const {
      return real4(w / b.w, x / b.x, y / b.y, z / b.z);
   }
   inline real4 operator-() const {
      return real4(-w, -x, -y, -z);
   }

   inline real4 operator+(real b) const {
      return real4(w + b, x + b, y + b, z + b);
   }
   inline real4 operator-(real b) const {
      return real4(w - b, x - b, y + b, z - b);
   }
   inline real4 operator*(real b) const {
      return real4(w * b, x * b, y * b, z * b);
   }
   inline real4 operator/(real b) const {
      return real4(w / b, x / b, y / b, z / b);
   }

   inline real dot(const real4 &b) const {
      return x * b.x + y * b.y + z * b.z + w * b.w;
   }

   inline real4 inv() const {
      real4 temp;
      real t1 = w * w + x * x + y * y + z * z;
      t1 = 1.0 / t1;
      temp.w = t1 * w;
      temp.x = -t1 * x;
      temp.y = -t1 * y;
      temp.z = -t1 * z;
      return temp;
   }
#endif

   operator real3() const {
      return real3(x, y, z);
   }

   inline real4& operator+=(const real4& b) {
      *this = *this + b;
      return *this;
   }
   inline real4& operator-=(const real4& b) {
      *this = *this - b;
      return *this;
   }
   inline real4& operator*=(const real4& b) {
      *this = *this * b;
      return *this;
   }
   inline real4& operator/=(const real4& b) {
      *this = *this / b;
      return *this;
   }

   inline real4& operator+=(real b) {
      *this = *this + b;
      return *this;
   }
   inline real4& operator-=(real b) {
      *this = *this - b;
      return *this;
   }
   inline real4& operator*=(real b) {
      *this = *this * b;
      return *this;
   }
   inline real4& operator/=(real b) {
      *this = *this / b;
      return *this;
   }

};

inline real4 operator+(real a,
                       const real4& b) {
   return b + a;
}
inline real4 operator-(real a,
                       const real4& b) {
   return real4(a) - b;
}
inline real4 operator*(real a,
                       const real4& b) {
   return b * a;
}
inline real4 operator/(real a,
                       const real4& b) {
   return real4(a) / b;
}

static inline real dot(const real4 &a,
                       const real4 &b) {
   return a.dot(b);
}

typedef real4 quaternion;

static inline quaternion inv(const quaternion &a) {
   return a.inv();
}

static inline real4 operator ~(real4 const & a) {
#ifdef ENABLE_SSE
   return real4(change_sign<0,1,1,1>(a));
#else
   return real4(a.w, -a.x, -a.y, -a.z);
#endif
}

static inline std::ostream &operator<<(std::ostream &out,
                                  const real4 &a) {
   out << "[" << a.w << ", " << a.x << ", " << a.y << ", " << a.z << "]" << std::endl;
   return out;
}

static inline real3 make_real3(const real4 &rhs) {
   return R3(rhs.x, rhs.y, rhs.z);
}

static inline bool operator ==(const real4 &a,
                               const real4 &b) {
   return ((a.w == b.w) && (a.x == b.x) && (a.y == b.y) && (a.z == b.z));
}

static inline quaternion normalize(const quaternion &a) {
   real length = sqrt(dot(a, a));
   if (length < ZERO_EPSILON) {
      return R4(1, 0, 0, 0);
   }
   length = 1.0 / length;
   return R4(a.w * length, a.x * length, a.y * length, a.z * length);
}

static inline quaternion Q_from_AngAxis(const real &angle,
                                        const real3 &axis) {
   quaternion quat;
   real halfang;
   real sinhalf;
   halfang = (angle * 0.5);
   sinhalf = sin(halfang);
   quat.w = cos(halfang);
   quat.x = axis.x * sinhalf;
   quat.y = axis.y * sinhalf;
   quat.z = axis.z * sinhalf;
   return (quat);
}

static inline quaternion mult_classic(const quaternion &qa,
                                      const quaternion &qb) {
   quaternion temp;

   temp.w = qa.w * qb.w - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z;
   temp.x = qa.w * qb.x + qa.x * qb.w - qa.z * qb.y + qa.y * qb.z;
   temp.y = qa.w * qb.y + qa.y * qb.w + qa.z * qb.x - qa.x * qb.z;
   temp.z = qa.w * qb.z + qa.z * qb.w - qa.y * qb.x + qa.x * qb.y;
   return temp;
}
static inline quaternion mult(const quaternion &a,
                              const quaternion &b) {
#ifdef ENABLE_SSE
   __m128 a1123 = _mm_shuffle_ps(a,a,0xE5);
   __m128 a2231 = _mm_shuffle_ps(a,a,0x7A);
   __m128 b1000 = _mm_shuffle_ps(b,b,0x01);
   __m128 b2312 = _mm_shuffle_ps(b,b,0x9E);
   __m128 t1 = _mm_mul_ps(a1123, b1000);
   __m128 t2 = _mm_mul_ps(a2231, b2312);
   __m128 t12 = _mm_add_ps(t1, t2);
   __m128 t12m = change_sign<1,0,0,0>(t12);
   __m128 a3312 = _mm_shuffle_ps(a,a,0x9F);
   __m128 b3231 = _mm_shuffle_ps(b,b,0x7B);
   __m128 a0000 = _mm_shuffle_ps(a,a,0x00);
   __m128 t3 = _mm_mul_ps(a3312, b3231);
   __m128 t0 = _mm_mul_ps(a0000, b);
   __m128 t03 = _mm_sub_ps(t0, t3);
   return _mm_add_ps(t03, t12m);
#else
   quaternion temp;
   temp.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
   temp.x = a.w * b.x + a.x * b.w - a.z * b.y + a.y * b.z;
   temp.y = a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z;
   temp.z = a.w * b.z + a.z * b.w - a.y * b.x + a.x * b.y;
   return temp;
#endif
}

static inline quaternion lerp(const quaternion &a,
                              const quaternion &b,
                              real alpha) {
   return normalize(a + alpha * (b - a));

}

static inline real angle(const quaternion &a,
                         const quaternion &b) {

   real s = sqrtf(dot(a, a) * dot(b, b));

   return acos(dot(a, b) / s);

}
static inline quaternion slerp(const quaternion &a,
                               const quaternion &b,
                               real alpha) {
   quaternion ans = a;
   real theta = angle(a, b);
   if (theta != 0) {
      real d = 1.0 / sin(theta);
      real s0 = sin((1.0 - alpha) * theta);
      real s1 = sin(alpha * theta);
      if (dot(a, b) < 0) {
         ans.x = (a.x * s0 - b.x * s1) * d;
         ans.y = (a.y * s0 - b.y * s1) * d;
         ans.z = (a.z * s0 - b.z * s1) * d;
         ans.w = (a.w * s0 - b.w * s1) * d;

      } else {
         ans.x = (a.x * s0 + b.x * s1) * d;
         ans.y = (a.y * s0 + b.y * s1) * d;
         ans.z = (a.z * s0 + b.z * s1) * d;
         ans.w = (a.w * s0 + b.w * s1) * d;

      }

   }
   return ans;

}

static inline real3 quatRotate(const real3 &v,
                               const quaternion &q) {
#ifdef ENABLE_SSE
   real3 t = 2 * cross(real3(q.x, q.y, q.z), v);
   return v + q.w * t + cross(real3(q.x, q.y, q.z), t);
   //return v+2.0*cross(cross(v,real3(q.x,q.y,q.z))+q.w*v, real3(q.x,q.y,q.z));
   //quaternion r = mult(mult(q, R4(0, v.x, v.y, v.z)), inv(q));
   //return real3(r.x, r.y, r.z);
#else
   real e0e0 = q.w * q.w;
   real e1e1 = q.x * q.x;
   real e2e2 = q.y * q.y;
   real e3e3 = q.z * q.z;
   real e0e1 = q.w * q.x;
   real e0e2 = q.w * q.y;
   real e0e3 = q.w * q.z;
   real e1e2 = q.x * q.y;
   real e1e3 = q.x * q.z;
   real e2e3 = q.y * q.z;
   return R3(((e0e0 + e1e1) * 2 - 1) * v.x + ((e1e2 - e0e3) * 2) * v.y + ((e1e3 + e0e2) * 2) * v.z,
             ((e1e2 + e0e3) * 2) * v.x + ((e0e0 + e2e2) * 2 - 1) * v.y + ((e2e3 - e0e1) * 2) * v.z,
             ((e1e3 - e0e2) * 2) * v.x + ((e2e3 + e0e1) * 2) * v.y + ((e0e0 + e3e3) * 2 - 1) * v.z);
#endif
}
static inline real3 quatRotateMat(const real3 &v,
                                  const quaternion &q) {

   real3 result;
   result.x = (q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z) * v.x + (2 * q.x * q.y - 2 * q.w * q.z) * v.y + (2 * q.x * q.z + 2 * q.w * q.y) * v.z;
   result.y = (2 * q.x * q.y + 2 * q.w * q.z) * v.x + (q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z) * v.y + (2 * q.y * q.z - 2 * q.w * q.x) * v.z;
   result.z = (2 * q.x * q.z - 2 * q.w * q.y) * v.x + (2 * q.y * q.z + 2 * q.w * q.x) * v.y + (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) * v.z;
   return result;  //quatRotate(v,q);
}
static inline real3 quatRotateMatT(const real3 &v,
                                   const quaternion &q) {

//	real3 result;
//	result.x = (q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z) * v.x + (2 * q.x * q.y + 2 * q.w * q.z) * v.y
//			+ (2 * q.x * q.z - 2 * q.w * q.y) * v.z;
//	result.y = (2 * q.x * q.y - 2 * q.w * q.z) * v.x + (q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z) * v.y
//			+ (2 * q.y * q.z + 2 * q.w * q.x) * v.z;
//	result.z = (2 * q.x * q.z + 2 * q.w * q.y) * v.x + (2 * q.y * q.z - 2 * q.w * q.x) * v.y
//			+ (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) * v.z;
   return quatRotate(v, ~q);
}
static inline quaternion operator %(const quaternion rhs,
                                    const quaternion lhs) {
   return mult(rhs, lhs);
}

static inline real3 AMatV(const real4 &q) {
   real3 V;

   real e0e0 = q.w * q.w;
   real e2e2 = q.y * q.y;
   real e0e1 = q.w * q.x;
   real e0e3 = q.w * q.z;
   real e1e2 = q.x * q.y;
   real e2e3 = q.y * q.z;

   V.x = (e1e2 - e0e3) * 2;
   V.y = (e0e0 + e2e2) * 2 - 1;
   V.z = (e2e3 + e0e1) * 2;

   return V;
}

#endif
