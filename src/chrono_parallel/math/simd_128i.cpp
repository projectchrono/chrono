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

#endif
}
}
