#include "chrono_parallel/math/sse.h"
#include "chrono_parallel/math/real3.h"
#include "chrono_parallel/math/real4.h"
#include "chrono_parallel/math/mat33.h"

#if defined(USE_AVX)
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
static inline __m256d permute4d(__m256d const& a) {
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
        __m128d alo = _mm256_castpd256_pd128(a);
        __m128d ahi = _mm256_extractf128_pd(a, 1);
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
#if defined(USE_AVX)

static const __m256d SIGNMASK = _mm256_castsi256_pd(_mm256_set1_epi64x(0x8000000000000000));

inline __m256d Set(real x) {
    return _mm256_set1_pd(x);
}
inline __m256d Set(real x, real y, real z) {
    return _mm256_setr_pd(x, y, z, 0.0);
}
inline __m256d Set(real x, real y, real z, real w) {
    return _mm256_setr_pd(x, y, z, w);
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

// inline __m256d Add(__m256d a, real b) {
//    return _mm256_add_pd(a, _mm256_set1_pd(b));
//}
// inline __m256d Sub(__m256d a, real b) {
//    return _mm256_sub_pd(a, _mm256_set1_pd(b));
//}
// inline __m256d Mul(__m256d a, real b) {
//    return _mm256_mul_pd(a, _mm256_set1_pd(b));
//}
// inline __m256d Div(__m256d a, real b) {
//    return _mm256_div_pd(a, _mm256_set1_pd(b));
//}

inline __m256d Negate(__m256d a) {
    return _mm256_xor_pd(a, SIGNMASK);
}
//
// inline real4 Negate(real4 a) {
//    return real4(-a[0], -a[1], -a[2], -a[3]);
//}
// inline real3 Negate(real3 a) {
//    return real3(-a[0], -a[1], -a[2]);
//}

// http://stackoverflow.com/questions/10454150/intel-avx-256-bits-version-of-dot-product-for-double-precision-floating-point

inline real HorizontalAdd(__m256d a) {
    __m256d temp = _mm256_hadd_pd(a, a);
    __m128d lo128 = _mm256_extractf128_pd(temp, 0);
    __m128d hi128 = _mm256_extractf128_pd(temp, 1);

    __m128d dot_prod = _mm_add_sd(lo128, hi128);
    return _mm_cvtsd_f64(dot_prod);
}

inline real Dot(__m256d a) {
    __m256d xy = _mm256_mul_pd(a, a);
    return HorizontalAdd(xy);
}
inline real Dot(__m256d a, __m256d b) {
    __m256d xy = _mm256_mul_pd(a, b);
    return HorizontalAdd(xy);
}

// inline real Dot(const real3& v1, const real3& v2) {
//    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
//}
// inline real Dot(const real3& v) {
//    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
//}

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

// inline real3 Cross(const real3& a, const real3& b) {
//    real3 result;
//    result[0] = (a[1] * b[2]) - (a[2] * b[1]);
//    result[1] = (a[2] * b[0]) - (a[0] * b[2]);
//    result[2] = (a[0] * b[1]) - (a[1] * b[0]);
//    result[3] = 0;
//    return result;
//}

inline real3 Normalize(const real3& v) {
    real t = simd::Dot(v);
    real dp = InvSqrt(t);
    real3 tmp = _mm256_mul_pd(v, Set(dp));
    tmp[3] = 0.0;
    return tmp;
}
inline __m256d Normalize(const __m256d& v) {
    real t = simd::Dot(v);
    real dp = InvSqrt(t);
    return _mm256_mul_pd(v, Set(dp));;
}
static const __m256d abs_mask =
    _mm256_castsi256_pd(_mm256_setr_epi32(-1, 0x7FFFFFFF, -1, 0x7FFFFFFF, -1, 0x7FFFFFFF, -1, 0x7FFFFFFF));
inline __m256d Abs(__m256d v) {
    return _mm256_and_pd(v, abs_mask);
}

inline __m256d Max(const __m256d& v1, const __m256d& v2) {
    return _mm256_max_pd(v1, v2);
}
inline __m256d Min(const __m256d& v1, const __m256d& v2) {
    return _mm256_min_pd(v1, v2);
}
// http://stackoverflow.com/questions/9795529/how-to-find-the-horizontal-maximum-in-a-256-bit-avx-vector
inline real Max(const __m256d& x) {
    __m256d y = _mm256_permute2f128_pd(x, x, 1);  // permute 128-bit values
    __m256d m1 = _mm256_max_pd(x, y);             // m1[0] = max(x[0], x[2]), m1[1] = max(x[1], x[3]), etc.
    __m256d m2 = _mm256_permute_pd(m1, 5);        // set m2[0] = m1[1], m2[1] = m1[0], etc.
    __m256d m = _mm256_max_pd(m1, m2);  // all m[0] ... m[3] contain the horizontal max(x[0], x[1], x[2], x[3])
    __m128d lo128 = _mm256_extractf128_pd(m, 0);  // get low bits
    return _mm_cvtsd_f64(lo128);                  // get a single double from low bits
}
inline real Min(const __m256d& x) {
    __m256d y = _mm256_permute2f128_pd(x, x, 1);  // permute 128-bit values
    __m256d m1 = _mm256_min_pd(x, y);             // m1[0] = max(x[0], x[2]), m1[1] = min(x[1], x[3]), etc.
    __m256d m2 = _mm256_permute_pd(m1, 5);        // set m2[0] = m1[1], m2[1] = m1[0], etc.
    __m256d m = _mm256_min_pd(m1, m2);  // all m[0] ... m[3] contain the horizontal min(x[0], x[1], x[2], x[3])
    __m128d lo128 = _mm256_extractf128_pd(m, 0);  // get low bits
    return _mm_cvtsd_f64(lo128);                  // get a single double from low bits
}
inline bool IsEqual(const __m256d& a, const __m256d& b) {
    //        const __m256d SIGN_MASK = _mm256_set1_pd(-0.0);
    //
    //        __m256d x = _mm256_cmp_pd(a, b, _CMP_EQ_UQ);
    //       // __m256i ii = _mm256_castpd_si256(x);
    //
    //        union {
    //            int64_t f[4];
    //            struct{
    //            	int64_t x, y, z, w;
    //            };
    //            __m256i m;
    //        } temp;
    //        _mm256_storeu_si256(&temp.m, ii);

    //        int vmask = _mm256_movemask_pd(x);
    //        bool result = (vmask == 0xffff);
    //            x = _mm256_andnot_pd(SIGN_MASK, x);
    //            x = _mm256_cmp_pd(a, b, _CMP_EQ_OQ);
    //        real3 t(x);
    //        real3 u(a);
    //        real3 v(b);
    //       // printf("O: %d %d %d %d\n", temp[0], temp[1], temp[2], temp.w);
    //        printf("O: %f %f %f %f\n", u[0], u[1], u[2], u.w);
    //        printf("O: %f %f %f %f\n", v[0], v[1], v[2], v.w);
    //        printf("O: %f %f %f %f\n", t[0], t[1], t[2], t.w);
    //       // printf("mask %d \n", vmask);
    //      //  return !result;  //_mm256_movemask_pd(x) != 0;
    return false;
}

// http://stackoverflow.com/questions/10454150/intel-avx-256-bits-version-of-dot-product-for-double-precision-floating-point
__m256d Dot4(__m256d v, __m256d a, __m256d b, __m256d c) {
    __m256d xy0 = _mm256_mul_pd(v, a);
    __m256d xy1 = _mm256_mul_pd(v, b);
    __m256d xy2 = _mm256_mul_pd(v, c);
    __m256d xy3 = _mm256_set1_pd(0);  // last dot prod is a dud

    // low to high: xy00+xy01 xy10+xy11 xy02+xy03 xy12+xy13
    __m256d temp01 = _mm256_hadd_pd(xy0, xy1);

    // low to high: xy20+xy21 xy30+xy31 xy22+xy23 xy32+xy33
    __m256d temp23 = _mm256_hadd_pd(xy2, xy3);

    // low to high: xy02+xy03 xy12+xy13 xy20+xy21 xy30+xy31
    __m256d swapped = _mm256_permute2f128_pd(temp01, temp23, 0x21);

    // low to high: xy00+xy01 xy10+xy11 xy22+xy23 xy32+xy33
    __m256d blended = _mm256_blend_pd(temp01, temp23, 0b1100);

    __m256d dotproduct = _mm256_add_pd(swapped, blended);
    return dotproduct;
}
__m256d Round(__m256d a) {
    return _mm256_round_pd(a, _MM_FROUND_TO_NEAREST_INT);
}

template <int i0, int i1, int i2, int i3>
static __m256d change_sign(__m256d const& a) {
    // printf("I: %f %f %f %f\n", a[0], a[1], a[2], a[3]);
    if ((i0 | i1 | i2 | i3) == 0) {
        return a;
    }

    __m256d mask = _mm256_castsi256_pd(_mm256_setr_epi32(0, i0 ? (int)0x80000000 : 0, 0, i1 ? (int)0x80000000 : 0, 0,
                                                         i2 ? (int)0x80000000 : 0, 0, i3 ? (int)0x80000000 : 0));

    // printf("M: %f %f %f %f\n", mask[0], mask[1], mask[2], mask[3]);

    __m256d res = _mm256_xor_pd(a, mask);

    // printf("O: %f %f %f %f\n", res[0], res[1], res[2], res[3]);
    return res;
}
inline __m256d QuatMult(__m256d a, __m256d b) {
    __m256d a1123 = permute4d<1, 1, 2, 3>(a);
    __m256d a2231 = permute4d<2, 2, 3, 1>(a);
    __m256d b1000 = permute4d<1, 0, 0, 0>(b);
    __m256d b2312 = permute4d<2, 3, 1, 2>(b);
    __m256d t1 = a1123 * b1000;
    __m256d t2 = a2231 * b2312;
    __m256d t12 = t1 + t2;
    __m256d t12m = change_sign<1, 0, 0, 0>(t12);
    __m256d a3312 = permute4d<3, 3, 1, 2>(a);
    __m256d b3231 = permute4d<3, 2, 3, 1>(b);
    __m256d a0000 = permute4d<0, 0, 0, 0>(a);
    __m256d t3 = a3312 * b3231;
    __m256d t0 = a0000 * b;
    __m256d t03 = t0 - t3;
    return t03 + t12m;
}
//========================================================
//========================================================
//========================================================

#elif defined(USE_SSE)

// http://fastcpp.blogspot.com/2011/03/changing-sign-of-float-values-using-sse.html
static const __m128 SIGNMASK = _mm_castsi128_ps(_mm_set1_epi32(0x80000000));

inline __m128 Set(real x) {
    return _mm_set1_ps(x);
}
inline __m128 Set(real x, real y, real z) {
    return _mm_setr_ps(x, y, z, 0.0f);
}
inline __m128 Set(real x, real y, real z, real w) {
    return _mm_setr_ps(x, y, z, w);
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

inline __m128 Add(__m128 a, real b) {
    return _mm_add_ps(a, _mm_set1_ps(b));
}
inline __m128 Sub(__m128 a, real b) {
    return _mm_sub_ps(a, _mm_set1_ps(b));
}
inline __m128 Mul(__m128 a, real b) {
    return _mm_mul_ps(a, _mm_set1_ps(b));
}
inline __m128 Div(__m128 a, real b) {
    return _mm_div_ps(a, _mm_set1_ps(b));
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

//========================================================
//========================================================
//========================================================

#else

inline real3 Set(real x) {
    return real3(x);
}
inline real3 Set(real x, real y, real z) {
    return real3(x, y, z);
}
inline real3 Add(real3 a, real3 b) {
    return real3(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}
inline real3 Sub(real3 a, real3 b) {
    return real3(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
inline real3 Mul(real3 a, real3 b) {
    return real3(a[0] * b[0], a[1] * b[1], a[2] * b[2]);
}
inline real3 Div(real3 a, real3 b) {
    return real3(a[0] / b[0], a[1] / b[1], a[2] / b[2]);
}

inline real3 Add(real3 a, real b) {
    return real3(a[0] + b, a[1] + b, a[2] + b);
}
inline real3 Sub(real3 a, real b) {
    return real3(a[0] - b, a[1] - b, a[2] - b);
}
inline real3 Mul(real3 a, real b) {
    return real3(a[0] * b, a[1] * b, a[2] * b);
}
inline real3 Div(real3 a, real b) {
    return real3(a[0] / b, a[1] / b, a[2] / b);
}

inline real3 Negate(real3 a) {
    return real3(-a[0], -a[1], -a[2]);
}
inline real Dot(const real3& v1, const real3& v2) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}
inline real Dot(const real3& v) {
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}
inline real3 SquareRoot(real3 v) {
    return real3(Sqrt(v[0]), Sqrt(v[1]), Sqrt(v[2]));
}
inline real3 Cross(const real3& a, const real3& b) {
    real3 result;
    result[0] = (a[1] * b[2]) - (a[2] * b[1]);
    result[1] = (a[2] * b[0]) - (a[0] * b[2]);
    result[2] = (a[0] * b[1]) - (a[1] * b[0]);
    return result;
}

//========================================================
//========================================================
//========================================================

inline real4 Add(const real4& a, const real4& b) {
    return real4(a[0] + b[0], a[1] + b[1], a[2] + b[2], a.w + b.w);
}
inline real4 Sub(const real4& a, const real4& b) {
    return real4(a[0] - b[0], a[1] + b[1], a[2] - b[2], a.w - b.w);
}
inline real4 Mul(const real4& a, const real4& b) {
    return real4(a[0] * b[0], a[1] * b[1], a[2] * b[2], a.w * b.w);
}
inline real4 Div(const real4& a, const real4& b) {
    return real4(a[0] / b[0], a[1] / b[1], a[2] / b[2], a.w / b.w);
}

inline real4 Add(const real4& a, const real3& b) {
    return real4(a[0] + b[0], a[1] + b[1], a[2] + b[2], a.w);
}
inline real4 Sub(const real4& a, const real3& b) {
    return real4(a[0] - b[0], a[1] + b[1], a[2] - b[2], a.w);
}
inline real4 Mul(const real4& a, const real3& b) {
    return real4(a[0] * b[0], a[1] * b[1], a[2] * b[2], a.w);
}
inline real4 Div(const real4& a, const real3& b) {
    return real4(a[0] / b[0], a[1] / b[1], a[2] / b[2], a.w);
}

inline real4 Add(const real4& a, real b) {
    return real4(a[0] + b, a[1] + b, a[2] + b, a.w + b);
}
inline real4 Sub(const real4& a, real b) {
    return real4(a[0] - b, a[1] + b, a[2] - b, a.w - b);
}
inline real4 Mul(const real4& a, real b) {
    return real4(a[0] * b, a[1] * b, a[2] * b, a.w * b);
}
inline real4 Div(const real4& a, real b) {
    return real4(a[0] / b, a[1] / b, a[2] / b, a.w / b);
}
inline real4 Negate(real4 a) {
    return real4(-a[0], -a[1], -a[2], -a.w);
}
#endif
}
//========================================================
real3 operator+(const real3& a, const real3& b) {
    return simd::Add(a, b);
}
real3 operator-(const real3& a, const real3& b) {
    return simd::Sub(a, b);
}
real3 operator*(const real3& a, const real3& b) {
    return simd::Mul(a, b);
}
real3 operator/(const real3& a, const real3& b) {
    return simd::Div(a, b);
}
//========================================================
real3 operator+(const real3& a, real b) {
    return simd::Add(a, simd::Set(b));
}
real3 operator-(const real3& a, real b) {
    return simd::Sub(a, simd::Set(b));
}
real3 operator*(const real3& a, real b) {
    return simd::Mul(a, simd::Set(b));
}
real3 operator/(const real3& a, real b) {
    return simd::Div(a, simd::Set(b));
}

real3 operator*(real lhs, const real3& rhs) {
    return simd::Mul(simd::Set(lhs), rhs);
}

real3 operator/(real lhs, const real3& rhs) {
    return simd::Div(simd::Set(lhs), rhs);
}

real3 operator-(const real3& a) {
    return simd::Negate(a);
}
//========================================================

real Dot(const real3& v1, const real3& v2) {
    return simd::Dot(v1, v2);
}
real Dot(const real3& v) {
    return simd::Dot(v);
}
real3 Normalize(const real3& v) {
    return simd::Normalize(v);
}
real Length(const real3& v) {
    return Sqrt(simd::Dot(v));
}
real3 Sqrt(const real3& v) {
    return simd::SquareRoot(v);
}
real3 Cross(const real3& b, const real3& c) {
    return simd::Cross(b, c);
}
real3 Abs(const real3& v) {
    return simd::Abs(v);
}
real3 Sign(const real3& v) {
    return simd::Max(simd::Min(v, simd::Set(1)), simd::Set(-1));
}
real3 Max(const real3& a, const real3& b) {
    return simd::Max(a, b);
}

real3 Min(const real3& a, const real3& b) {
    return simd::Min(a, b);
}

real3 Max(const real3& a, const real& b) {
    return simd::Max(a, simd::Set(b));
}

real3 Min(const real3& a, const real& b) {
    return simd::Min(a, simd::Set(b));
}
real Max(const real3& a) {
    return simd::Max(a);
}
real Min(const real3& a) {
    return simd::Min(a);
}

real3 Clamp(const real3& a, const real3& clamp_min, const real3& clamp_max) {
    return simd::Max(clamp_min, simd::Min(a, clamp_max));
}

bool operator==(const real3& a, const real3& b) {
    return (a[0] == b[0]) && (a[1] == b[1]) && (a[2] == b[2]);
    // return simd::IsEqual(a, b);
}

real3 Round(const real3& v) {
    return simd::Round(v);
}
// real3 LessThan(real3 a, real3 b){
//
//}
bool IsZero(const real3& v) {
    real3 t = simd::Abs(v);
    return t[0] < C_EPSILON && t[1] < C_EPSILON && t[2] < C_EPSILON;
}
//========================================================
real4 operator+(const real4& a, const real4& b) {
    return simd::Add(a, b);
}
real4 operator-(const real4& a, const real4& b) {
    return simd::Sub(a, b);
}
real4 operator*(const real4& a, const real4& b) {
    return simd::Mul(a, b);
}
real4 operator/(const real4& a, const real4& b) {
    return simd::Div(a, b);
}
//========================================================
real4 operator+(const real4& a, real b) {
    return simd::Add(a, simd::Set(b));
}
real4 operator-(const real4& a, real b) {
    return simd::Sub(a, simd::Set(b));
}
real4 operator*(const real4& a, real b) {
    return simd::Mul(a, simd::Set(b));
}
real4 operator/(const real4& a, real b) {
    return simd::Div(a, simd::Set(b));
}
real4 operator-(const real4& a) {
    return simd::Negate(a);
}

//========================================================
quaternion operator+(const quaternion& a, real b) {
    return simd::Add(a, simd::Set(b));
}
quaternion operator-(const quaternion& a, real b) {
    return simd::Sub(a, simd::Set(b));
}
quaternion operator*(const quaternion& a, real b) {
    return simd::Mul(a, simd::Set(b));
}
quaternion operator/(const quaternion& a, real b) {
    return simd::Div(a, simd::Set(b));
}
quaternion operator-(const quaternion& a) {
    return simd::Negate(a);
}
quaternion operator~(const quaternion& a) {
    return simd::change_sign<0, 1, 1, 1>(a);
}
quaternion Inv(const quaternion& a) {
    real t1 = Dot(a);
    return (~a) / t1;
}
real Dot(const quaternion& v1, const quaternion& v2) {
    return simd::Dot(v1, v2);
}
real Dot(const quaternion& v) {
    return simd::Dot(v);
}
quaternion Mult(const quaternion& a, const quaternion& b) {
    return simd::QuatMult(a, b);
}
quaternion Normalize(const quaternion& v) {
    return simd::Normalize(v);
}
//========================================================

real3 operator*(const Mat33& M, const real3& v) {
    real3 t1 = simd::Mul(M.cols[0], simd::Set(v[0]));
    real3 t2 = simd::Mul(M.cols[1], simd::Set(v[1]));
    real3 t3 = simd::Mul(M.cols[2], simd::Set(v[2]));
    return simd::Add(t1, simd::Add(t2, t3));
}
Mat33 Abs(const Mat33& m) {
    return Mat33(simd::Abs(m.cols[0]), simd::Abs(m.cols[1]), simd::Abs(m.cols[2]));
}
Mat33 SkewSymmetric(const real3& r) {
    return Mat33(real3(0, r[2], -r[1]), real3(-r[2], 0, r[0]), real3(r[1], -r[0], 0));
}

Mat33 MultTranspose(const Mat33& M, const Mat33& N) {
    return Mat33(M * real3(N.cols[0][0], N.cols[1][0], N.cols[2][0]),  //
                 M * real3(N.cols[0][1], N.cols[1][1], N.cols[2][1]),  //
                 M * real3(N.cols[0][2], N.cols[1][2], N.cols[2][2]));
}

Mat33 TransposeMult(const Mat33& M, const Mat33& N) {
    Mat33 result;

    result.cols[0] = simd::Dot4(N.cols[0], M.cols[0], M.cols[1], M.cols[2]);
    result.cols[1] = simd::Dot4(N.cols[1], M.cols[0], M.cols[1], M.cols[2]);
    result.cols[2] = simd::Dot4(N.cols[2], M.cols[0], M.cols[1], M.cols[2]);

    return result;
}

real Trace(const Mat33& m) {
    return m.cols[0][0] + m.cols[1][1] + m.cols[2][2];
}
// Multiply a 3x1 by a 1x3 to get a 3x3
Mat33 OuterProduct(const real3& a, const real3& b) {
    return Mat33(a * b[0], a * b[1], a * b[2]);
}

// real3 UnpackLow(const real3& v1, const real3& v2) {
//    return _mm256_unpacklo_pd(v1, v2);
//}
// real3 UnpackHigh(const real3& v1, const real3& v2) {
//    return _mm256_unpackhi_pd(v1, v2);
//}

// Mat33 Transpose(const Mat33& M) {
//    real3 T0 = simd::UnpackLow(M.cols[0], M.cols[1]);
//    real3 T1 = simd::UnpackLow(M.cols[2], Set(0));
//    real3 T2 = simd::UnpackHigh(M.cols[0], M.cols[1]);
//    real3 T3 = simd::UnpackHigh(M.cols[2], Set(0));
//
//    /* Assigning transposed values back into I[0-3] */
//    I0 = _mm_unpacklo_epi64(T0, T1);
//    I1 = _mm_unpackhi_epi64(T0, T1);
//    I2 = _mm_unpacklo_epi64(T2, T3);
//    I3 = _mm_unpackhi_epi64(T2, T3);
//}

// void M4x4_SSE(const mat33& B, const mat33& A, const mat33& C) {
//    real3 row1 = B.cols[0];
//    real3 row2 = B.cols[1];
//    real3 row3 = B.cols[2];
//
//    {
//        real3 brod1 = simd::Set(A.cols[0][0]);
//        real3 brod2 = simd::Set(A.cols[1][0]);
//        real3 brod3 = simd::Set(A.cols[2][0]);
//        real3 row = simd::Add(simd::Add(simd::Mul(brod1, row1), simd::Mul(brod2, row2)), simd::Mul(brod3, row3));
//        C.cols[0] = row;
//    }
//    {
//        real3 brod1 = simd::Set(A.cols[0][1]);
//        real3 brod2 = simd::Set(A.cols[1][1]);
//        real3 brod3 = simd::Set(A.cols[2][1]);
//        real3 row = simd::Add(simd::Add(simd::Mul(brod1, row1), simd::Mul(brod2, row2)), simd::Mul(brod3, row3));
//        C.cols[1] = row;
//    }
//    {
//        real3 brod1 = simd::Set(A.cols[0][2]);
//        real3 brod2 = simd::Set(A.cols[1][2]);
//        real3 brod3 = simd::Set(A.cols[2][2]);
//        real3 row = simd::Add(simd::Add(simd::Mul(brod1, row1), simd::Mul(brod2, row2)), simd::Mul(brod3, row3));
//        C.cols[2] = row;
//    }
//}
}
