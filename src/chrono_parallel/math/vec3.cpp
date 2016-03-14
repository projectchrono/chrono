#include "chrono_parallel/math/sse.h"
#include "chrono_parallel/math/other_types.h"

#if defined(USE_SSE)
#include "chrono_parallel/math/simd_sse.h"
#elif defined(USE_AVX)
#include "chrono_parallel/math/simd_avx.h"
#else
#include "chrono_parallel/math/simd_non.h"
#endif

namespace chrono {

CUDA_HOST_DEVICE vec3 operator-(const vec3& a, const vec3& b) {
    return simd::Sub(a, b);
}
CUDA_HOST_DEVICE vec3 operator-(const vec3& a, const int& b) {
    return simd::Sub(a, simd::Set(b));
}
CUDA_HOST_DEVICE vec3 operator+(const vec3& a, const vec3& b) {
    return simd::Add(a, b);
}
CUDA_HOST_DEVICE vec3 operator+(const vec3& a, const int& b) {
    return simd::Add(a, simd::Set(b));
}
CUDA_HOST_DEVICE vec3 Clamp(const vec3& a, const vec3& clamp_min, const vec3& clamp_max) {
    return simd::Max(clamp_min, simd::Min(a, clamp_max));
}
}
