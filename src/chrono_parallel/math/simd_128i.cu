#include "chrono_parallel/math/other_types.h"

#if defined(__CUDA_ARCH__)
#include "chrono_parallel/math/simd_non.h"
namespace chrono {

CUDA_DEVICE vec3 operator-(const vec3& a, const vec3& b) {
    return sisd::Sub(a, b);
}
CUDA_DEVICE vec3 operator-(const vec3& a, const int& b) {
    return sisd::Sub(a, sisd::Set(b));
}
CUDA_DEVICE vec3 operator+(const vec3& a, const vec3& b) {
    return sisd::Add(a, b);
}
CUDA_DEVICE vec3 operator+(const vec3& a, const int& b) {
    return sisd::Add(a, sisd::Set(b));
}
CUDA_DEVICE vec3 Clamp(const vec3& a, const vec3& clamp_min, const vec3& clamp_max) {
    return sisd::Max(clamp_min, sisd::Min(a, clamp_max));
}
}
#endif
