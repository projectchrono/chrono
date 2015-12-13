
#include "chrono_parallel/math/real3.h"
#include "chrono_parallel/math/sse.h"
#if defined(CHRONO_PARALLEL_USE_SIMD) && defined(CHRONO_PARALLEL_HAS_AVX) && defined(CHRONO_PARALLEL_USE_DOUBLE)
#define set_m128r(lo, hi) _mm256_insertf128_ps(_mm256_castps128_ps256(lo), (hi), 1)

template <int i0, int i1, int i2, int i3>
static inline __m128i constant4i() {
    static const union {
        int i[4];
        __m128i xmm;
    } u = {{i0, i1, i2, i3}};
    return u.xmm;
}

// permute vector Vec2d
template <int i0, int i1>
static inline __m128d permute2d(__m128d const& a) {
    // is shuffling needed
    const bool do_shuffle = (i0 > 0) || (i1 != 1 && i1 >= 0);
    // is zeroing needed
    const bool do_zero = ((i0 | i1) < 0 && (i0 | i1) & 0x80);

    if (do_zero && !do_shuffle) {  // zeroing, not shuffling
        if ((i0 & i1) < 0)
            return _mm_setzero_pd();  // zero everything
        // zero some elements
        __m128i mask1 = constant4i<-int(i0 >= 0), -int(i0 >= 0), -int(i1 >= 0), -int(i1 >= 0)>();
        return _mm_and_pd(a, _mm_castsi128_pd(mask1));  // zero with AND mask
    } else if (do_shuffle && !do_zero) {                // shuffling, not zeroing
        return _mm_shuffle_pd(a, a, (i0 & 1) | (i1 & 1) << 1);
    } else if (do_shuffle && do_zero) {  // shuffling and zeroing
        // both shuffle and zero
        if (i0 < 0 && i1 >= 0) {  // zero low half, shuffle high half
            return _mm_shuffle_pd(_mm_setzero_pd(), a, (i1 & 1) << 1);
        }
        if (i0 >= 0 && i1 < 0) {  // shuffle low half, zero high half
            return _mm_shuffle_pd(a, _mm_setzero_pd(), i0 & 1);
        }
    }
    return a;  // trivial case: do nothing
}

// blend vectors Vec2d
template <int i0, int i1>
static inline __m128d blend2d(__m128d const& a, __m128d const& b) {
    // Combine all the indexes into a single bitfield, with 8 bits for each
    const int m1 = (i0 & 3) | (i1 & 3) << 8;

    // Mask to zero out negative indexes
    const int m2 = (i0 < 0 ? 0 : 0xFF) | (i1 < 0 ? 0 : 0xFF) << 8;

    if ((m1 & 0x0202 & m2) == 0) {
        // no elements from b, only elements from a and possibly zero
        return permute2d<i0, i1>(a);
    }
    if (((m1 ^ 0x0202) & 0x0202 & m2) == 0) {
        // no elements from a, only elements from b and possibly zero
        return permute2d<i0 & ~2, i1 & ~2>(b);
    }
    // selecting from both a and b without zeroing
    if ((i0 & 2) == 0) {  // first element from a, second element from b
        return _mm_shuffle_pd(a, b, (i0 & 1) | (i1 & 1) << 1);
    } else {  // first element from b, second element from a
        return _mm_shuffle_pd(b, a, (i0 & 1) | (i1 & 1) << 1);
    }
}

template <int i0, int i1, int i2, int i3, int i4, int i5, int i6, int i7>
static inline __m256 constant8f() {
    static const union {
        int i[8];
        __m256 ymm;
    } u = {{i0, i1, i2, i3, i4, i5, i6, i7}};
    return u.ymm;
}

template <int i0, int i1, int i2, int i3>
static inline __m256 permute4d(__m256 const& a) {
    const int ior = i0 | i1 | i2 | i3;  // OR indexes

    // is zeroing needed
    const bool do_zero = ior < 0 && (ior & 0x80);  // at least one index is negative, and not -0x100

    // is shuffling needed
    const bool do_shuffle = (i0 > 0) || (i1 != 1 && i1 >= 0) || (i2 != 2 && i2 >= 0) || (i3 != 3 && i3 >= 0);

    if (!do_shuffle) {  // no shuffling needed
        if (do_zero) {  // zeroing
            if ((i0 & i1 & i2 & i3) < 0) {
                return _mm256_setzero_pd();  // zero everything
            }
            // zero some elements
            __m256d const mask =
                _mm256_castps_pd(constant8f<-int(i0 >= 0), -int(i0 >= 0), -int(i1 >= 0), -int(i1 >= 0), -int(i2 >= 0),
                                            -int(i2 >= 0), -int(i3 >= 0), -int(i3 >= 0)>());
            return _mm256_and_pd(a, mask);  // zero with AND mask
        } else {
            return a;  // do nothing
        }
    }

    // Needed contents of low/high part of each source register in VSHUFPD
    // 0: a.low, 1: a.high, 3: zero
    const int s1 = (i0 < 0 ? 3 : (i0 & 2) >> 1) | (i2 < 0 ? 0x30 : (i2 & 2) << 3);
    const int s2 = (i1 < 0 ? 3 : (i1 & 2) >> 1) | (i3 < 0 ? 0x30 : (i3 & 2) << 3);
    // permute mask
    const int sm = (i0 < 0 ? 0 : (i0 & 1)) | (i1 < 0 ? 1 : (i1 & 1)) << 1 | (i2 < 0 ? 0 : (i2 & 1)) << 2 |
                   (i3 < 0 ? 1 : (i3 & 1)) << 3;

    if (s1 == 0x01 || s1 == 0x11 || s2 == 0x01 || s2 == 0x11) {
        // too expensive to use 256 bit permute, split into two 128 bit permutes
        __m128d alo = _mm256_castps256_ps128(a);
        __m128d ahi = _mm256_extractf128_ps(a, 1);
        __m128d rlo = blend2d<i0, i1>(alo, ahi);
        __m128d rhi = blend2d<i2, i3>(alo, ahi);
        return _mm256_castps_pd(set_m128r(_mm_castpd_ps(rlo), _mm_castpd_ps(rhi)));
    }

    // make operands for VSHUFPD
    __m256d r1, r2;

    switch (s1) {
        case 0x00:  // LL
            r1 = _mm256_insertf128_pd(a, _mm256_castpd256_pd128(a), 1);
            break;
        case 0x03:  // LZ
            r1 = _mm256_insertf128_pd(do_zero ? _mm256_setzero_pd() : __m256d(a), _mm256_castpd256_pd128(a), 1);
            break;
        case 0x10:  // LH
            r1 = a;
            break;
        case 0x13:  // ZH
            r1 = do_zero ? _mm256_and_pd(a, _mm256_castps_pd(constant8f<0, 0, 0, 0, -1, -1, -1, -1>())) : __m256d(a);
            break;
        case 0x30:  // LZ
            if (do_zero) {
                __m128d t = _mm256_castpd256_pd128(a);
                t = _mm_and_pd(t, t);
                r1 = _mm256_castpd128_pd256(t);
            } else
                r1 = a;
            break;
        case 0x31:  // HZ
            r1 = _mm256_castpd128_pd256(_mm256_extractf128_pd(a, 1));
            break;
        case 0x33:  // ZZ
            r1 = do_zero ? _mm256_setzero_pd() : __m256d(a);
            break;
    }

    if (s2 == s1) {
        if (sm == 0x0A)
            return r1;
        r2 = r1;
    } else {
        switch (s2) {
            case 0x00:  // LL
                r2 = _mm256_insertf128_pd(a, _mm256_castpd256_pd128(a), 1);
                break;
            case 0x03:  // ZL
                r2 = _mm256_insertf128_pd(do_zero ? _mm256_setzero_pd() : __m256d(a), _mm256_castpd256_pd128(a), 1);
                break;
            case 0x10:  // LH
                r2 = a;
                break;
            case 0x13:  // ZH
                r2 =
                    do_zero ? _mm256_and_pd(a, _mm256_castps_pd(constant8f<0, 0, 0, 0, -1, -1, -1, -1>())) : __m256d(a);
                break;
            case 0x30:  // LZ
                if (do_zero) {
                    __m128d t = _mm256_castpd256_pd128(a);
                    t = _mm_and_pd(t, t);
                    r2 = _mm256_castpd128_pd256(t);
                } else
                    r2 = a;
                break;
            case 0x31:  // HZ
                r2 = _mm256_castpd128_pd256(_mm256_extractf128_pd(a, 1));
                break;
            case 0x33:  // ZZ
                r2 = do_zero ? _mm256_setzero_pd() : __m256d(a);
                break;
        }
    }
    return _mm256_shuffle_pd(r1, r2, sm);
}
#endif

namespace chrono {

namespace simd {
#if defined(CHRONO_PARALLEL_USE_SIMD) && defined(CHRONO_PARALLEL_HAS_AVX) && defined(CHRONO_PARALLEL_USE_DOUBLE)

static const __m256d SIGNMASK = _mm256_castsi256_pd(_mm256_set1_epi64x(0x8000000000000000));

inline __m256d Set(real x) {
    return _mm256_set1_pd(x);
}
inline __m256d Set(real x, real y, real z) {
    return _mm256_setr_pd(x, y, z, 0.0);
}
inline __m256d Add(__m256d a, __m256d b) {
    return _mm256_add_pd(a, b);
}
inline __m256d Sub(__m256d a, __m256d b) {
    return _mm256_sub_pd(a, b);
}
inline __m256d Mul(__m256d a, __m256d b) {
    return _mm256_mul_pd(a, b);
}
inline __m256d Div(__m256d a, __m256d b) {
    return _mm256_div_pd(a, b);
}
inline __m256d Negate(__m256d a) {
    return _mm256_xor_pd(a, SIGNMASK);
}
// http://stackoverflow.com/questions/10454150/intel-avx-256-bits-version-of-dot-product-for-double-precision-floating-point

inline real Dot(__m256d a) {
    __m256d xy = _mm256_mul_pd(a, a);
    __m256d temp = _mm256_hadd_pd(xy, xy);
    __m128d hi128 = _mm256_extractf128_pd(temp, 1);
    __m128d dot_prod = _mm_add_sd(_mm256_castpd256_pd128(temp), hi128);
    return _mm_cvtsd_f64(dot_prod);
}
inline real Dot(__m256d a, __m256d b) {
    __m256d xy = _mm256_mul_pd(a, b);
    __m256d temp = _mm256_hadd_pd(xy, xy);
    __m128d hi128 = _mm256_extractf128_pd(temp, 1);
    __m128d dot_prod = _mm_add_sd(_mm256_castpd256_pd128(temp), hi128);
    return _mm_cvtsd_f64(dot_prod);
}
inline __m256d SquareRoot(__m256d v) {
    return _mm256_sqrt_pd(v);
}

inline __m256d Cross(__m256d a, __m256d b) {
    __m256d a1 = permute4d<1, 2, 0, -256>(a);
    __m256d b1 = permute4d<1, 2, 0, -256>(b);
    __m256d a2 = permute4d<2, 0, 1, -256>(a);
    __m256d b2 = permute4d<2, 0, 1, -256>(b);
    __m256d c = a1 * b2 - a2 * b1;
    return c;
}

#elif defined(CHRONO_PARALLEL_USE_SIMD) && defined(CHRONO_PARALLEL_HAS_SSE) && !defined(CHRONO_PARALLEL_USE_DOUBLE)

// http://fastcpp.blogspot.com/2011/03/changing-sign-of-float-values-using-sse.html
static const __m128 SIGNMASK = _mm_castsi128_ps(_mm_set1_epi32(0x80000000));

inline __m128 Set(real x) {
    return _mm_set1_ps(x);
}
inline __m128 Set(real x, real y, real z) {
    return _mm_setr_ps(x, y, z, 0.0f);
}
inline __m128 Add(__m128 a, __m128 b) {
    return _mm_add_ps(a, b);
}
inline __m128 Sub(__m128 a, __m128 b) {
    return _mm_sub_ps(a, b);
}
inline __m128 Mul(__m128 a, __m128 b) {
    return _mm_mul_ps(a, b);
}
inline __m128 Div(__m128 a, __m128 b) {
    return _mm_div_ps(a, b);
}
inline __m128 Negate(__m128 a) {
    return _mm_xor_ps(a, SIGNMASK);
}
inline real Dot(__m128 a) {
    return _mm_cvtss_f32(_mm_dp_ps(a, a, 0x71));
}
inline real Dot(__m128 a, __m128 b) {
    return _mm_cvtss_f32(_mm_dp_ps(a, b, 0x71));
}
inline __m128 SquareRoot(__m128 v) {
    return _mm_sqrt_ps(v);
}
inline __m128 Cross(__m128 a, __m128 b) {
    return _mm_sub_ps(
        _mm_mul_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1)), _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 1, 0, 2))),
        _mm_mul_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 0, 2)), _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1))));
}

#else

inline real3 Set(real x) {
    return real3(x);
}
inline real3 Set(real x, real y, real z) {
    return real3(x, y, z);
}
inline real3 Add(real3 a, real3 b) {
    return real3(a.x + b.x, a.y + b.y, a.z + b.z);
}
inline real3 Sub(real3 a, real3 b) {
    return real3(a.x - b.x, a.y - b.y, a.z - b.z);
}
inline real3 Mul(real3 a, real3 b) {
    return real3(a.x * b.x, a.y * b.y, a.z * b.z);
}
inline real3 Div(real3 a, real3 b) {
    return real3(a.x / b.x, a.y / b.y, a.z / b.z);
}
inline real3 Negate(real3 a) {
    return real3(-a.x, -a.y, -a.z);
}
inline real Dot(const real3& v1, const real3& v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}
inline real Dot(const real3& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}
inline real3 SquareRoot(real3 v) {
    return real3(Sqrt(v.x), Sqrt(v.y), Sqrt(v.z));
}
inline real3 Cross(const real3& a, const real3& b) {
    real3 result;
    result.x = (a.y * b.z) - (a.z * b.y);
    result.y = (a.z * b.x) - (a.x * b.z);
    result.z = (a.x * b.y) - (a.y * b.x);
    return result;
}
#endif
}
//========================================================
real3 real3::operator+(const real3& b) const {
    return simd::Add(*this, b);
}
real3 real3::operator-(const real3& b) const {
    return simd::Sub(*this, b);
}
real3 real3::operator*(const real3& b) const {
    return simd::Mul(*this, b);
}
real3 real3::operator/(const real3& b) const {
    return simd::Div(*this, b);
}
//========================================================
real3 real3::operator+(real b) const {
    return simd::Add(*this, simd::Set(b));
}
real3 real3::operator-(real b) const {
    return simd::Sub(*this, simd::Set(b));
}
real3 real3::operator*(real b) const {
    return simd::Mul(*this, simd::Set(b));
}
real3 real3::operator/(real b) const {
    return simd::Div(*this, simd::Set(b));
}
//========================================================

real Dot(const real3& v1, const real3& v2) {
    return simd::Dot(v1, v2);
}
real Dot(const real3& v) {
    return simd::Dot(v);
}
real3 Sqrt(const real3& v) {
    return simd::SquareRoot(v);
}
real3 Cross(const real3& b, const real3& c) {
    return simd::Cross(b, c);
}
}
