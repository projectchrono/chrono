#include "chrono_parallel/math/sse.h"
#include "chrono_parallel/math/real.h"

using namespace chrono;

namespace simd {

static const __m256d NEGATEMASK = _mm256_castsi256_pd(_mm256_set1_epi64x(0x8000000000000000));
static const __m256d ABSMASK =
    _mm256_castsi256_pd(_mm256_setr_epi32(-1, 0x7FFFFFFF, -1, 0x7FFFFFFF, -1, 0x7FFFFFFF, -1, 0x7FFFFFFF));

// Mask gets the first 3 elements out of 4, sets last element to zero
static const __m256d REAL3MASK = _mm256_castsi256_pd(
    _mm256_setr_epi64x(0xffffffffffffffff, 0xffffffffffffffff, 0xffffffffffffffff, 0x0000000000000000));

// Functions that will work on all 4 wide double types
//========================================================
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
inline __m256d Div3(__m256d a, __m256d b) {
    return _mm256_and_pd(_mm256_div_pd(a, b), REAL3MASK);
}
inline __m256d Negate(__m256d a) {
    return _mm256_xor_pd(a, NEGATEMASK);
}
inline __m256d SquareRoot(__m256d v) {
    return _mm256_sqrt_pd(v);
}
// http://stackoverflow.com/questions/10454150/intel-avx-256-bits-version-of-dot-product-for-double-precision-floating-point
inline real HorizontalAdd(__m256d a) {
    __m256d temp = _mm256_hadd_pd(a, a);
    __m128d lo128 = _mm256_extractf128_pd(temp, 0);
    __m128d hi128 = _mm256_extractf128_pd(temp, 1);
    __m128d dot_prod = _mm_add_pd(lo128, hi128);
    return _mm_cvtsd_f64(dot_prod);
}
inline real Dot3(__m256d a) {
    //__m256d xy = _mm256_and_pd(a, REAL3MASK);
    __m256d xy = _mm256_mul_pd(a, a);
    return HorizontalAdd(xy);
}
inline real Dot3(__m256d a, __m256d b) {
    __m256d xy = _mm256_mul_pd(a, b);
    return HorizontalAdd(xy);
}
inline real Dot4(__m256d a) {
    __m256d xy = _mm256_mul_pd(a, a);
    return HorizontalAdd(xy);
}
inline real Dot4(__m256d a, __m256d b) {
    __m256d xy = _mm256_mul_pd(a, b);
    return HorizontalAdd(xy);
}
// inline __m256d Cross(__m256d a, __m256d b) {
//    __m256d a1 = permute4d<1, 2, 0, -256>(a);
//    __m256d b1 = permute4d<1, 2, 0, -256>(b);
//    __m256d a2 = permute4d<2, 0, 1, -256>(a);
//    __m256d b2 = permute4d<2, 0, 1, -256>(b);
//    __m256d c = _mm256_sub_pd(_mm256_mul_pd(a1 , b2) , _mm256_mul_pd(a2 , b1));
//    return c;
//}
inline __m256d Normalize(const __m256d& v) {
    real t = simd::Dot4(v);
    real dp = InvSqrt(t);
    return _mm256_mul_pd(v, _mm256_set1_pd(dp));
}
inline __m256d Abs(__m256d v) {
    return _mm256_and_pd(v, ABSMASK);
}
inline __m256d Max(__m256d v1, __m256d v2) {
    return _mm256_max_pd(v1, v2);
}
inline __m256d Min(__m256d v1, __m256d v2) {
    return _mm256_min_pd(v1, v2);
}
// http://stackoverflow.com/questions/9795529/how-to-find-the-horizontal-maximum-in-a-256-bit-avx-vector
inline real Max(__m256d x) {
    __m256d y = _mm256_permute2f128_pd(x, x, 1);  // permute 128-bit values
    __m256d m1 = _mm256_max_pd(x, y);             // m1[0] = max(x[0], x[2]), m1[1] = max(x[1], x[3]), etc.
    __m256d m2 = _mm256_permute_pd(m1, 5);        // set m2[0] = m1[1], m2[1] = m1[0], etc.
    __m256d m = _mm256_max_pd(m1, m2);            // all m[0] ... m[3] contain the horiz max(x[0], x[1], x[2], x[3])
    __m128d lo128 = _mm256_extractf128_pd(m, 0);  // get low bits
    return _mm_cvtsd_f64(lo128);                  // get a single double from low bits
}
inline real Min(__m256d x) {
    __m256d y = _mm256_permute2f128_pd(x, x, 1);  // permute 128-bit values
    __m256d m1 = _mm256_min_pd(x, y);             // m1[0] = max(x[0], x[2]), m1[1] = min(x[1], x[3]), etc.
    __m256d m2 = _mm256_permute_pd(m1, 5);        // set m2[0] = m1[1], m2[1] = m1[0], etc.
    __m256d m = _mm256_min_pd(m1, m2);            // all m[0] ... m[3] contain the horiz min(x[0], x[1], x[2], x[3])
    __m128d lo128 = _mm256_extractf128_pd(m, 0);  // get low bits
    return _mm_cvtsd_f64(lo128);                  // get a single double from low bits
}
inline real Max3(__m256d a) {
    __m256d x = _mm256_permute_pd(a, 1);          // copy over 4th value
    __m256d y = _mm256_permute2f128_pd(x, x, 1);  // permute 128-bit values
    __m256d m1 = _mm256_max_pd(x, y);             // m1[0] = max(x[0], x[2]), m1[1] = max(x[1], x[3]), etc.
    __m256d m2 = _mm256_permute_pd(m1, 5);        // set m2[0] = m1[1], m2[1] = m1[0], etc.
    __m256d m = _mm256_max_pd(m1, m2);            // all m[0] ... m[3] contain the horiz max(x[0], x[1], x[2], x[3])
    __m128d lo128 = _mm256_extractf128_pd(m, 0);  // get low bits
    return _mm_cvtsd_f64(lo128);                  // get a single double from low bits
}
inline real Min3(__m256d a) {
    __m256d x = _mm256_permute_pd(a, 1);          // copy over 4th value
    __m256d y = _mm256_permute2f128_pd(x, x, 1);  // permute 128-bit values
    __m256d m1 = _mm256_min_pd(x, y);             // m1[0] = max(x[0], x[2]), m1[1] = min(x[1], x[3]), etc.
    __m256d m2 = _mm256_permute_pd(m1, 5);        // set m2[0] = m1[1], m2[1] = m1[0], etc.
    __m256d m = _mm256_min_pd(m1, m2);            // all m[0] ... m[3] contain the horiz min(x[0], x[1], x[2], x[3])
    __m128d lo128 = _mm256_extractf128_pd(m, 0);  // get low bits
    return _mm_cvtsd_f64(lo128);                  // get a single double from low bits
}

inline __m256d Round(__m256d a) {
    return _mm256_round_pd(a, _MM_FROUND_TO_NEAREST_INT);
}

template <int i0, int i1, int i2, int i3>
static __m256d change_sign(__m256d a) {
    if ((i0 | i1 | i2 | i3) == 0) {
        return a;
    }
    __m256d mask = _mm256_castsi256_pd(_mm256_setr_epi32(0, i0 ? (int)0x80000000 : 0, 0, i1 ? (int)0x80000000 : 0, 0,
                                                         i2 ? (int)0x80000000 : 0, 0, i3 ? (int)0x80000000 : 0));

    __m256d res = _mm256_xor_pd(a, mask);

    return res;
}
//========================================================
// inline __m256d Cross3(__m256d a, __m256d b) {
//    __m256d a1 = permute4d<1, 2, 0, -256>(a);
//    __m256d b1 = permute4d<1, 2, 0, -256>(b);
//    __m256d a2 = permute4d<2, 0, 1, -256>(a);
//    __m256d b2 = permute4d<2, 0, 1, -256>(b);
//    __m256d c = a1 * b2 - a2 * b1;
//    return _mm256_and_pd(c, REAL3MASK);
//}

inline __m256d Cross3(__m256d a012, __m256d b012) {
#if defined(CHRONO_AVX_2_0) && defined(CHRONO_HAS_FMA)
    // https://www.nersc.gov/assets/Uploads/Language-Impact-on-Vectorization-Vector-Programming-in-C++.pdf
    __m256d a201 = _mm256_permute4x64_pd(a012, _MM_SHUFFLE(3, 1, 0, 2));
    __m256d b201 = _mm256_permute4x64_pd(b012, _MM_SHUFFLE(3, 1, 0, 2));
    __m256d tmp = _mm256_fmsub_pd(b012, a201, _mm256_mul_pd(a012, b201));
    tmp = _mm256_permute4x64_pd(tmp, _MM_SHUFFLE(3, 1, 0, 2));
    tmp = _mm256_blend_pd(_mm256_setzero_pd(), tmp, 0x7);  // put zero on 4th position
#else
    __m256d tmp = {0, 0, 0, 0};
#endif
    return tmp;
}

inline __m256d Normalize3(__m256d v) {
    __m256d xy = _mm256_mul_pd(v, v);
    __m256d temp = _mm256_hadd_pd(xy, xy);
    __m128d lo128 = _mm256_extractf128_pd(temp, 0);
    __m128d hi128 = _mm256_extractf128_pd(temp, 1);
    __m128d dot_prod = _mm_add_pd(lo128, hi128);
    __m128d len = _mm_sqrt_pd(dot_prod);
    __m256d tmp = _mm256_div_pd(v, _mm256_set1_pd(_mm_cvtsd_f64(len)));
    return _mm256_and_pd(tmp, REAL3MASK);
}
inline real Length3(__m256d v) {
    __m256d xy = _mm256_mul_pd(v, v);
    __m256d temp = _mm256_hadd_pd(xy, xy);
    __m128d lo128 = _mm256_extractf128_pd(temp, 0);
    __m128d hi128 = _mm256_extractf128_pd(temp, 1);
    __m128d dot_prod = _mm_add_pd(lo128, hi128);
    __m128d len = _mm_sqrt_pd(dot_prod);
    return _mm_cvtsd_f64(len);
}
inline bool IsEqual(__m256d a, __m256d b) {
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
inline bool IsZero(__m256d v, real eps) {
    __m256d a = _mm256_and_pd(v, ABSMASK);
    __m256d c = _mm256_cmp_pd(a, _mm256_set1_pd(eps), _CMP_NLT_US);
    int mask = _mm256_movemask_pd(c);
    return mask == 0;
}
// http://stackoverflow.com/questions/10454150/intel-avx-256-bits-version-of-dot-product-for-double-precision-floating-point
inline __m256d Dot4(__m256d v, __m256d a, __m256d b, __m256d c) {
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
    __m256d blended = _mm256_blend_pd(temp01, temp23, 0xC);

    __m256d dotproduct = _mm256_add_pd(swapped, blended);
    return dotproduct;
}
inline __m256d Dot4(__m256d v, __m256d a, __m256d b, __m256d c, __m256d d) {
    __m256d xy0 = _mm256_mul_pd(v, a);
    __m256d xy1 = _mm256_mul_pd(v, b);
    __m256d xy2 = _mm256_mul_pd(v, c);
    __m256d xy3 = _mm256_mul_pd(v, d);

    // low to high: xy00+xy01 xy10+xy11 xy02+xy03 xy12+xy13
    __m256d temp01 = _mm256_hadd_pd(xy0, xy1);

    // low to high: xy20+xy21 xy30+xy31 xy22+xy23 xy32+xy33
    __m256d temp23 = _mm256_hadd_pd(xy2, xy3);

    // low to high: xy02+xy03 xy12+xy13 xy20+xy21 xy30+xy31
    __m256d swapped = _mm256_permute2f128_pd(temp01, temp23, 0x21);

    // low to high: xy00+xy01 xy10+xy11 xy22+xy23 xy32+xy33
    __m256d blended = _mm256_blend_pd(temp01, temp23, 0xC);

    __m256d dotproduct = _mm256_add_pd(swapped, blended);
    return dotproduct;
}

inline __m256d QuatMult(__m256d a, __m256d b) {
#if defined(CHRONO_AVX_2_0)
    __m256d a1123 = _mm256_permute4x64_pd(a, _MM_SHUFFLE(3, 2, 1, 1));
    __m256d a2231 = _mm256_permute4x64_pd(a, _MM_SHUFFLE(1, 3, 2, 2));
    __m256d b1000 = _mm256_permute4x64_pd(b, _MM_SHUFFLE(0, 0, 0, 1));
    __m256d b2312 = _mm256_permute4x64_pd(b, _MM_SHUFFLE(2, 1, 3, 2));
    __m256d t1 = _mm256_mul_pd(a1123, b1000);
    __m256d t2 = _mm256_mul_pd(a2231, b2312);
    __m256d t12 = _mm256_add_pd(t1, t2);
    __m256d t12m = change_sign<1, 0, 0, 0>(t12);
    __m256d a3312 = _mm256_permute4x64_pd(a, _MM_SHUFFLE(2, 1, 3, 3));
    __m256d b3231 = _mm256_permute4x64_pd(b, _MM_SHUFFLE(1, 3, 2, 3));
    __m256d a0000 = _mm256_permute4x64_pd(a, _MM_SHUFFLE(0, 0, 0, 0));
    __m256d t3 = _mm256_mul_pd(a3312, b3231);
    __m256d t0 = _mm256_mul_pd(a0000, b);
    __m256d t03 = _mm256_sub_pd(t0, t3);
    return _mm256_add_pd(t03, t12m);
#else
    __m256d tmp = {0, 0, 0, 0};
    return tmp;
#endif
}

inline __m128i Set(int x) {
    return _mm_set1_epi32(x);
}
inline __m128i Sub(__m128i a, __m128i b) {
    return _mm_sub_epi32(a, b);
}

inline __m128i Add(__m128i a, __m128i b) {
    return _mm_add_epi32(a, b);
}

inline __m128i Max(__m128i a, __m128i b) {
    return _mm_max_epi32(a, b);
}
inline __m128i Min(__m128i a, __m128i b) {
    return _mm_min_epi32(a, b);
}
}
