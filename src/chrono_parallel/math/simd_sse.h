#include "chrono_parallel/math/sse.h"
#include "chrono_parallel/math/real.h"
#include "chrono_parallel/math/real3.h"
#include "chrono_parallel/math/real4.h"

using namespace chrono;

namespace simd {

// http://fastcpp.blogspot.com/2011/03/changing-sign-of-float-values-using-sse.html
static const __m128 NEGATEMASK = _mm_castsi128_ps(_mm_set1_epi32(0x80000000));
static const __m128 ABSMASK = _mm_castsi128_ps(_mm_set1_epi32(0x7FFFFFFF));
static const __m128 REAL3MASK = _mm_castsi128_ps(_mm_setr_epi32(0xffffffff, 0xffffffff, 0xffffffff, 0x00000000));

template <int i0, int i1, int i2, int i3>
static __m128 change_sign(__m128 a) {
    if ((i0 | i1 | i2 | i3) == 0) {
        return a;
    }
    __m128 mask = _mm_castsi128_ps(_mm_setr_epi32(i0 ? (int)0x80000000 : 0, i1 ? (int)0x80000000 : 0,
                                                  i2 ? (int)0x80000000 : 0, i3 ? (int)0x80000000 : 0));

    __m128 res = _mm_xor_ps(a, mask);

    return res;
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
inline __m128 Div3(__m128 a, __m128 b) {
    return _mm_and_ps(_mm_div_ps(a, b), REAL3MASK);
}
inline __m128 Negate(__m128 a) {
    return _mm_xor_ps(a, NEGATEMASK);
}
inline real Dot3(__m128 a) {
    return _mm_cvtss_f32(_mm_dp_ps(a, a, 0x71));
}
inline real Dot3(__m128 a, __m128 b) {
    return _mm_cvtss_f32(_mm_dp_ps(a, b, 0x71));
}
inline real Dot4(__m128 a) {
    return _mm_cvtss_f32(_mm_dp_ps(a, a, 0xF1));
}
inline real Dot4(__m128 a, __m128 b) {
    return _mm_cvtss_f32(_mm_dp_ps(a, b, 0xF1));
}
inline __m128 SquareRoot(__m128 v) {
    return _mm_sqrt_ps(v);
}
inline __m128 Cross(__m128 a, __m128 b) {
    return _mm_sub_ps(
        _mm_mul_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1)), _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 1, 0, 2))),
        _mm_mul_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 0, 2)), _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1))));
}
inline __m128 Cross3(__m128 a, __m128 b) {
    __m128 tmp = _mm_sub_ps(
        _mm_mul_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1)), _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 1, 0, 2))),
        _mm_mul_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 0, 2)), _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1))));
    return _mm_and_ps(tmp, REAL3MASK);
}
inline __m128 Normalize3(__m128 v) {
    real t = Dot3(v);
    real dp = InvSqrt(t);
    __m128 tmp = _mm_mul_ps(v, _mm_set1_ps(dp));
    return _mm_and_ps(tmp, REAL3MASK);
}
inline __m128 Normalize(__m128 v) {
    real t = Dot4(v);
    real dp = InvSqrt(t);
    return _mm_mul_ps(v, _mm_set1_ps(dp));
}
inline __m128 Max(__m128 v1, __m128 v2) {
    return _mm_max_ps(v1, v2);
}
inline __m128 Min(__m128 v1, __m128 v2) {
    return _mm_min_ps(v1, v2);
}

inline real Min(__m128 x) {
    __m128 low = _mm_movehl_ps(x, x);                                             /* [2, 3, 2, 3] */
    __m128 low_accum = _mm_min_ps(low, x);                                        /* [0|2, 1|3, 2|2, 3|3] */
    __m128 elem1 = _mm_shuffle_ps(low_accum, low_accum, _MM_SHUFFLE(1, 1, 1, 1)); /* [1|3, 1|3, 1|3, 1|3] */
    __m128 accum = _mm_min_ss(low_accum, elem1);
    return _mm_cvtss_f32(accum);
}
inline real Max(__m128 x) {
    __m128 low = _mm_movehl_ps(x, x);                                             /* [2, 3, 2, 3] */
    __m128 low_accum = _mm_max_ps(low, x);                                        /* [0|2, 1|3, 2|2, 3|3] */
    __m128 elem1 = _mm_shuffle_ps(low_accum, low_accum, _MM_SHUFFLE(1, 1, 1, 1)); /* [1|3, 1|3, 1|3, 1|3] */
    __m128 accum = _mm_max_ss(low_accum, elem1);
    return _mm_cvtss_f32(accum);
}
inline real Min3(__m128 a) {
    __m128 x = _mm_permute_ps(a, 6);                                              // copy over 4th value
    __m128 low = _mm_movehl_ps(x, x);                                             /* [2, 3, 2, 3] */
    __m128 low_accum = _mm_min_ps(low, x);                                        /* [0|2, 1|3, 2|2, 3|3] */
    __m128 elem1 = _mm_shuffle_ps(low_accum, low_accum, _MM_SHUFFLE(1, 1, 1, 1)); /* [1|3, 1|3, 1|3, 1|3] */
    __m128 accum = _mm_min_ss(low_accum, elem1);
    return _mm_cvtss_f32(accum);
}
inline real Max3(__m128 a) {
    __m128 x = _mm_permute_ps(a, 6);                                              // copy over 4th value
    __m128 low = _mm_movehl_ps(x, x);                                             /* [2, 3, 2, 3] */
    __m128 low_accum = _mm_max_ps(low, x);                                        /* [0|2, 1|3, 2|2, 3|3] */
    __m128 elem1 = _mm_shuffle_ps(low_accum, low_accum, _MM_SHUFFLE(1, 1, 1, 1)); /* [1|3, 1|3, 1|3, 1|3] */
    __m128 accum = _mm_max_ss(low_accum, elem1);
    return _mm_cvtss_f32(accum);
}
inline __m128 Abs(__m128 v) {
    return _mm_and_ps(v, ABSMASK);
}

inline __m128 Round(__m128 a) {
    return _mm_round_ps(a, _MM_FROUND_TO_NEAREST_INT);
}

inline __m128 QuatMult(__m128 a, __m128 b) {
    __m128 a1123 = _mm_shuffle_ps(a, a, 0xE5);
    __m128 a2231 = _mm_shuffle_ps(a, a, 0x7A);
    __m128 b1000 = _mm_shuffle_ps(b, b, 0x01);
    __m128 b2312 = _mm_shuffle_ps(b, b, 0x9E);
    __m128 t1 = _mm_mul_ps(a1123, b1000);
    __m128 t2 = _mm_mul_ps(a2231, b2312);
    __m128 t12 = _mm_add_ps(t1, t2);
    __m128 t12m = change_sign<1, 0, 0, 0>(t12);
    __m128 a3312 = _mm_shuffle_ps(a, a, 0x9F);
    __m128 b3231 = _mm_shuffle_ps(b, b, 0x7B);
    __m128 a0000 = _mm_shuffle_ps(a, a, 0x00);
    __m128 t3 = _mm_mul_ps(a3312, b3231);
    __m128 t0 = _mm_mul_ps(a0000, b);
    __m128 t03 = _mm_sub_ps(t0, t3);
    return _mm_add_ps(t03, t12m);
}
// inline __m128 Dot4(__m128 v, __m128 a, __m128 b, __m128 c) {
//    __m128 u1 = _mm_shuffle_ps(v, v, _MM_SHUFFLE(0, 0, 0, 0));
//    __m128 u2 = _mm_shuffle_ps(v, v, _MM_SHUFFLE(1, 1, 1, 1));
//    __m128 u3 = _mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 2, 2, 2));
//
//    __m128 prod1 = _mm_mul_ps(u1, a);
//    __m128 prod2 = _mm_mul_ps(u2, b);
//    __m128 prod3 = _mm_mul_ps(u3, c);
//
//    return _mm_add_ps(_mm_add_ps(prod1, prod2), _mm_add_ps(prod3, _mm_set1_ps(0)));
//}

inline __m128 Dot4(__m128 v, __m128 a, __m128 b, __m128 c) {
    __m128 prod1 = _mm_mul_ps(a, v);
    __m128 prod2 = _mm_mul_ps(b, v);
    __m128 prod3 = _mm_mul_ps(c, v);
    return _mm_hadd_ps(_mm_hadd_ps(prod1, prod2), _mm_hadd_ps(prod3, _mm_set1_ps(0)));
}

inline __m128 Dot4(__m128 v, __m128 a, __m128 b, __m128 c, __m128 d) {
    __m128 prod1 = _mm_mul_ps(a, v);
    __m128 prod2 = _mm_mul_ps(b, v);
    __m128 prod3 = _mm_mul_ps(c, v);
    __m128 prod4 = _mm_mul_ps(d, v);
    return _mm_hadd_ps(_mm_hadd_ps(prod1, prod2), _mm_hadd_ps(prod3, prod4));
}
inline real HorizontalAdd(real4 a) {
    return a[0] + a[1] + a[2] + a[3];
}
inline real HorizontalAdd(real3 a) {
    return a[0] + a[1] + a[2];
}
inline bool IsZero(const real3& v, const real& a) {
    return chrono::Abs(v.x) < a && chrono::Abs(v.y) < a && chrono::Abs(v.z) < a;
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

} // end namespace simd
