#ifndef REAL3_H
#define REAL3_H

#include <nmmintrin.h>

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/real.h"
#include "chrono_parallel/math/real2.h"

#define R3  real3
#define ZERO_VECTOR R3(0)
  static const __m128 SIGNMASK = _mm_castsi128_ps(_mm_set1_epi32(0x80000000));


class CHRONO_ALIGN_16 real3  {
public:
  union{
     struct { real x, y, z, w; };
     __m128 mmvalue;
  };


  inline real3() : mmvalue(_mm_setzero_ps()) {}
  inline real3(float a) : mmvalue(_mm_set1_ps(a)) {}
  inline real3(float a, float b, float c) :  mmvalue(_mm_setr_ps(a,b,c,0)) {}
  inline real3(__m128 m) : mmvalue(m) {}

  inline operator __m128() const { return mmvalue; }

  inline real3 operator+(const real3& b) const { return _mm_add_ps(mmvalue, b.mmvalue);}
  inline real3 operator-(const real3& b) const { return _mm_sub_ps(mmvalue, b.mmvalue);}
  inline real3 operator*(const real3& b) const { return _mm_mul_ps(mmvalue, b.mmvalue);}
  inline real3 operator/(const real3& b) const { return _mm_div_ps(mmvalue, b.mmvalue);}
  inline real3 operator-()               const { return _mm_xor_ps(mmvalue, SIGNMASK); }

  inline real3& operator+=(const real3& b) { *this = *this + b; return *this; }
  inline real3& operator-=(const real3& b) { *this = *this - b; return *this; }
  inline real3& operator*=(const real3& b) { *this = *this * b; return *this; }
  inline real3& operator/=(const real3& b) { *this = *this / b; return *this; }

  inline real3 operator+(real b) const { return _mm_add_ps(mmvalue, _mm_set1_ps(b)); }
  inline real3 operator-(real b) const { return _mm_sub_ps(mmvalue, _mm_set1_ps(b)); }
  inline real3 operator*(real b) const { return _mm_mul_ps(mmvalue, _mm_set1_ps(b)); }
  inline real3 operator/(real b) const { return _mm_div_ps(mmvalue, _mm_set1_ps(b)); }

  inline real3& operator+=(real b) { *this = *this + b; return *this; }
  inline real3& operator-=(real b) { *this = *this - b; return *this; }
  inline real3& operator*=(real b) { *this = *this * b; return *this; }
  inline real3& operator/=(real b) { *this = *this / b; return *this; }
  inline bool operator ==(const real3 &b) {return ((x == b.x) && (y == b.y) && (z == b.z));}

  inline real dot(const real3 &b) const {return _mm_cvtss_f32(_mm_dp_ps(mmvalue, b.mmvalue, 0x71));}
  inline real length() const  {return _mm_cvtss_f32(_mm_sqrt_ss(_mm_dp_ps(mmvalue, mmvalue, 0x71)));}
  inline real rlength() const  {return _mm_cvtss_f32(_mm_rsqrt_ss(_mm_dp_ps(mmvalue, mmvalue, 0x71)));}
  inline real3 normalize() const {return _mm_mul_ps(mmvalue, _mm_rsqrt_ps(_mm_dp_ps(mmvalue, mmvalue, 0x7F)));}

  inline real3 cross(const real3 &b) const {
	  return _mm_sub_ps(
	      _mm_mul_ps(_mm_shuffle_ps(mmvalue, mmvalue, _MM_SHUFFLE(3, 0, 2, 1)), _mm_shuffle_ps(b.mmvalue, b.mmvalue, _MM_SHUFFLE(3, 1, 0, 2))),
	      _mm_mul_ps(_mm_shuffle_ps(mmvalue, mmvalue, _MM_SHUFFLE(3, 1, 0, 2)), _mm_shuffle_ps(b.mmvalue, b.mmvalue, _MM_SHUFFLE(3, 0, 2, 1)))
	     );
  }
};

inline real3 operator+(real a, const real3& b) { return b + a; }
inline real3 operator-(real a, const real3& b) { return real3(a) - b; }
inline real3 operator*(real a, const real3& b) { return b * a; }
inline real3 operator/(real a, const real3& b) { return real3(a) / b; }
inline bool operator ==(const real3 &a, const real3 &b) {return a==b;}

inline float dot(const real3& a, const real3& b) { return a.dot(b); }
inline real3 cross(const real3& a, const real3& b) { return a.cross(b); }
inline float length(const real3& a) { return a.length(); }
inline float rlength(const real3& a) { return a.rlength(); }
inline real3 normalize(const real3& a) { return a.normalize(); }

static inline ostream &operator<<(ostream &out, real3 &a) {
	out << "[" << a.x << ", " << a.y << ", " << a.z << "]" << endl;
	return out;
}

static inline real3 ceil(const real3 &a) {return R3(ceil(a.x), ceil(a.y), ceil(a.z));}
static inline real3 lerp(const real3 &a, const real3 &b, real alpha) {return (a + alpha * (b - a));}
static inline real3 fabs(const real3 &a) {return R3(fabs(a.x), fabs(a.y), fabs(a.z));}
static inline bool isEqual(const real3 &a, const real3 &b) {return isEqual(a.x, b.x) && isEqual(a.y, b.y) && isEqual(a.z, b.z);}
static inline bool IsZero(const real3 &a) {return IsZero(a.x) && IsZero(a.y) && IsZero(a.z);}

#endif
