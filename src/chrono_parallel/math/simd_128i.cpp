#include "chrono_parallel/math/sse.h"
#include "chrono_parallel/math/real3.h"
#include "chrono_parallel/math/real4.h"
#include "chrono_parallel/math/other_types.h"
namespace chrono {

namespace simd {
#if defined(USE_AVX) || defined(USE_SSE)

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
//========================================================
//========================================================
//========================================================

#else

//========================================================
//========================================================
//========================================================
#endif
}
int3 operator-(const int3& a, const int3& b) {
    return simd::Sub(a, b);
}
int3 operator-(const int3& a, const int& b) {
    return simd::Sub(a, simd::Set(b));
}
int3 operator+(const int3& a, const int3& b) {
    return simd::Add(a, b);
}
int3 operator+(const int3& a, const int& b) {
    return simd::Add(a, simd::Set(b));
}
int3 Clamp(const int3& a, const int3& clamp_min, const int3& clamp_max) {
    return simd::Max(clamp_min, simd::Min(a, clamp_max));
}
}
