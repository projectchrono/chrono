// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Florian Reinle
// =============================================================================
// Vulkan RT render filter.
// =============================================================================

#include "chrono_sensor/vulkan/ChFilterVulkanRTRender.h"
#include "chrono_sensor/vulkan/ChVulkanRTUtils.h"
#include "chrono_sensor/vulkan/ChVulkanRTBuffer.h"
#include "chrono_sensor/ChConfigSensor.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <sstream>
#include <fstream>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "chrono/physics/ChBody.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChDepthCamera.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChNormalCamera.h"
#if defined(CHRONO_HAS_VULKAN_RT) && !defined(CHRONO_HAS_OPTIX)
#include "chrono_sensor/sensors/ChPhysCameraSensor.h"
#endif
#include "chrono_sensor/sensors/ChRadarSensor.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"
#include "chrono_sensor/sensors/ChVulkanCameraSensor.h"
#include "chrono_thirdparty/stb/stb_image.h"

namespace chrono {
namespace sensor {

struct ChVulkanRTRenderCache {
    struct TextureImage {
        int width = 0;
        int height = 0;
        std::vector<uint8_t> rgba;
        std::vector<float> rgba_float;
        bool hdr = false;
        bool has_translucent_alpha = false;

        bool Valid() const { return width > 0 && height > 0 && (!rgba.empty() || !rgba_float.empty()); }
    };

    struct BvhNode {
        ChVector3d bmin = ChVector3d(0.0, 0.0, 0.0);
        ChVector3d bmax = ChVector3d(0.0, 0.0, 0.0);
        int left = -1;
        int right = -1;
        uint32_t first = 0;
        uint32_t count = 0;

        bool IsLeaf() const { return left < 0; }
    };

    struct CachedPrimitive {
        const ChVulkanRTPrimitive* primitive = nullptr;
        ChVector3d world_min = ChVector3d(0.0, 0.0, 0.0);
        ChVector3d world_max = ChVector3d(0.0, 0.0, 0.0);
        std::vector<uint32_t> triangle_indices;
        std::vector<BvhNode> triangle_nodes;
    };

    const ChVulkanRTScene* scene = nullptr;
    uint64_t scene_revision = 0;
    std::vector<CachedPrimitive> primitives;
    std::vector<uint32_t> primitive_indices;
    std::vector<BvhNode> primitive_nodes;
    std::unordered_map<std::string, TextureImage> textures;
};

namespace {

constexpr double CH_VKRT_PI = 3.141592653589793238462643383279502884;
constexpr double CH_VKRT_EPS = 1e-6;
constexpr double CH_VKRT_SHADOW_EPS = 1e-4;
constexpr uint32_t CH_VKRT_TRIANGLE_LEAF_SIZE = 8;
constexpr uint32_t CH_VKRT_PRIMITIVE_LEAF_SIZE = 4;
constexpr size_t CH_VKRT_TRAVERSAL_STACK_SIZE = 128;

struct RayHit {
    bool hit = false;
    double t = std::numeric_limits<double>::max();
    ChVector3d normal = ChVector3d(0.0, 0.0, 1.0);
    ChVector3d tangent = ChVector3d(1.0, 0.0, 0.0);
    ChVulkanRTTexCoord uv;
    bool has_uv = false;
    ChVulkanRTMaterial material;
    ChVector3d primitive_origin = ChVector3d(0.0, 0.0, 0.0);
    ChVector3d translational_velocity = ChVector3d(0.0, 0.0, 0.0);
    ChVector3d angular_velocity = ChVector3d(0.0, 0.0, 0.0);
    float object_id = 0.f;
};

struct TextureSample4 {
    float r = 1.f;
    float g = 1.f;
    float b = 1.f;
    float a = 1.f;
    bool valid = false;
};

struct EvaluatedMaterial {
    ChVector3f diffuse = ChVector3f(0.75f, 0.75f, 0.75f);
    ChVector3f specular = ChVector3f(0.2f, 0.2f, 0.2f);
    ChVector3f emissive = ChVector3f(0.f, 0.f, 0.f);
    float opacity = 1.f;
    float roughness = 1.f;
    float metallic = 0.f;
    bool use_specular_workflow = false;
};

inline uint8_t ToByte(float v) {
    v = std::max(0.f, std::min(1.f, v));
    return static_cast<uint8_t>(v * 255.f + 0.5f);
}

inline ChVector3d NormalizeSafe(const ChVector3d& v, const ChVector3d& fallback = ChVector3d(0.0, 0.0, 1.0)) {
    const double len = v.Length();
    if (len < CH_VKRT_EPS)
        return fallback;
    return v / len;
}

inline ChVector3f Clamp01(const ChVector3f& c) {
    return ChVector3f(std::max(0.f, std::min(1.f, c.x())),
                      std::max(0.f, std::min(1.f, c.y())),
                      std::max(0.f, std::min(1.f, c.z())));
}

inline ChVector3f Mul(const ChVector3f& a, const ChVector3f& b) {
    return ChVector3f(a.x() * b.x(), a.y() * b.y(), a.z() * b.z());
}

inline ChVector3f PowRGB(const ChVector3f& c, float e) {
    return ChVector3f(std::pow(std::max(0.f, c.x()), e),
                      std::pow(std::max(0.f, c.y()), e),
                      std::pow(std::max(0.f, c.z()), e));
}

inline float ClampFloat(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

inline float MaxComponent(const ChVector3f& c) {
    return std::max(c.x(), std::max(c.y(), c.z()));
}

inline float FresnelSchlickScalar(float cos_theta, float f0) {
    const float ct = ClampFloat(cos_theta, 0.f, 1.f);
    return f0 + (1.f - f0) * std::pow(1.f - ct, 5.f);
}

inline ChVector3f FresnelSchlick(float cos_theta, const ChVector3f& f0) {
    const float ct = ClampFloat(cos_theta, 0.f, 1.f);
    const float k = std::pow(1.f - ct, 5.f);
    return f0 + k * (ChVector3f(1.f, 1.f, 1.f) - f0);
}

inline float LerpFloat(float a, float b, float t) {
    return a + (b - a) * t;
}

inline uint32_t HashU32(uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}

inline float HashUnitFloat(uint32_t x) {
    return static_cast<float>(HashU32(x) & 0x00ffffffu) / static_cast<float>(0x01000000u);
}

inline float OptixJitterComponent(unsigned int x, unsigned int y, unsigned int sample_idx, unsigned int axis) {
    return HashUnitFloat((x + 1u) * 73856093u ^ (y + 1u) * 19349663u ^ (sample_idx + 1u) * 83492791u ^ axis * 2654435761u);
}

inline float NormalDistOptix(float ndh, float roughness) {
    // Keep the same microfacet normalization used by OptiX shader_utils.cuh::NormalDist():
    // the 1/pi factor is intentionally omitted there and must not be reintroduced here.
    const float rough_sqr = std::max(0.02f, roughness) * std::max(0.02f, roughness);
    const float den = ndh * ndh * (rough_sqr - 1.f) + 1.f;
    return rough_sqr / std::max(1e-5f, den * den);
}

inline float HammonSmithOptix(float ndv, float ndl, float roughness) {
    // Matches OptiX shader_utils.cuh::HammonSmith().  A standard Smith-GGX
    // visibility term gives visibly different highlights and penumbra contrast.
    const float denominator = LerpFloat(2.f * std::abs(ndv) * std::abs(ndl),
                                        std::abs(ndl) + std::abs(ndv),
                                        ClampFloat(roughness, 0.f, 1.f));
    return 0.5f / std::max(1e-5f, denominator);
}

inline ChVector3d Reflect(const ChVector3d& d, const ChVector3d& n) {
    return d - 2.0 * d.Dot(n) * n;
}

inline float RadialLensFunction(float rd2, const LensParams& lens_params) {
    const double rd2_d = static_cast<double>(rd2);
    const double rd4 = rd2_d * rd2_d;
    const double rd6 = rd4 * rd2_d;
    const double rd8 = rd4 * rd4;
    const double rd10 = rd6 * rd4;
    const double rd12 = rd6 * rd6;
    const double rd14 = rd8 * rd6;
    const double rd16 = rd8 * rd8;
    const double rd18 = rd10 * rd8;

    return static_cast<float>(1.0 +
                              lens_params.a0 * rd2_d +
                              lens_params.a1 * rd4 +
                              lens_params.a2 * rd6 +
                              lens_params.a3 * rd8 +
                              lens_params.a4 * rd10 +
                              lens_params.a5 * rd12 +
                              lens_params.a6 * rd14 +
                              lens_params.a7 * rd16 +
                              lens_params.a8 * rd18);
}

inline ChVector3f GammaCorrect(const ChVector3f& c, float gamma) {
    if (gamma <= 0.f || std::abs(gamma - 1.f) < 1e-6f)
        return c;
    const float inv_gamma = 1.f / gamma;
    const auto clamped = Clamp01(c);
    return ChVector3f(std::pow(clamped.x(), inv_gamma), std::pow(clamped.y(), inv_gamma), std::pow(clamped.z(), inv_gamma));
}

uint32_t PackRGB9E5(float r, float g, float b) {
    r = std::max(0.f, r);
    g = std::max(0.f, g);
    b = std::max(0.f, b);
    const float m = std::max(r, std::max(g, b));
    if (m <= 0.f)
        return 0u;

    int exp_shared = std::max(-16, static_cast<int>(std::floor(std::log2(m))) + 1);
    float denom = std::ldexp(1.f, exp_shared - 9);
    uint32_t rm = static_cast<uint32_t>(std::round(r / denom));
    uint32_t gm = static_cast<uint32_t>(std::round(g / denom));
    uint32_t bm = static_cast<uint32_t>(std::round(b / denom));
    if (rm > 511u || gm > 511u || bm > 511u) {
        ++exp_shared;
        denom = std::ldexp(1.f, exp_shared - 9);
        rm = static_cast<uint32_t>(std::round(r / denom));
        gm = static_cast<uint32_t>(std::round(g / denom));
        bm = static_cast<uint32_t>(std::round(b / denom));
    }
    const uint32_t be = static_cast<uint32_t>(std::max(0, std::min(31, exp_shared + 15)));
    return (std::min(rm, 511u)) | (std::min(gm, 511u) << 9) | (std::min(bm, 511u) << 18) | (be << 27);
}

inline PixelRGBA8 MakeRGBA(const ChVector3f& c, uint8_t a = 255) {
    const auto clamped = Clamp01(c);
    PixelRGBA8 p;
    p.R = ToByte(clamped.x());
    p.G = ToByte(clamped.y());
    p.B = ToByte(clamped.z());
    p.A = a;
    return p;
}

inline ChVector3d MinVec(const ChVector3d& a, const ChVector3d& b) {
    return ChVector3d(std::min(a.x(), b.x()), std::min(a.y(), b.y()), std::min(a.z(), b.z()));
}

inline ChVector3d MaxVec(const ChVector3d& a, const ChVector3d& b) {
    return ChVector3d(std::max(a.x(), b.x()), std::max(a.y(), b.y()), std::max(a.z(), b.z()));
}

inline void ResetAABB(ChVector3d& bmin, ChVector3d& bmax) {
    const double inf = std::numeric_limits<double>::infinity();
    bmin = ChVector3d(inf, inf, inf);
    bmax = ChVector3d(-inf, -inf, -inf);
}

inline void ExpandAABB(ChVector3d& bmin, ChVector3d& bmax, const ChVector3d& p) {
    bmin = MinVec(bmin, p);
    bmax = MaxVec(bmax, p);
}

inline void ExpandAABB(ChVector3d& bmin, ChVector3d& bmax, const ChVector3d& omin, const ChVector3d& omax) {
    bmin = MinVec(bmin, omin);
    bmax = MaxVec(bmax, omax);
}

inline ChVector3d Centroid(const ChVector3d& bmin, const ChVector3d& bmax) {
    return 0.5 * (bmin + bmax);
}

inline int LongestAxis(const ChVector3d& bmin, const ChVector3d& bmax) {
    const ChVector3d ext = bmax - bmin;
    if (ext.y() > ext.x() && ext.y() >= ext.z())
        return 1;
    if (ext.z() > ext.x() && ext.z() > ext.y())
        return 2;
    return 0;
}

inline ChVector3d TriangleMin(const ChVulkanRTTriangle& tri) {
    return MinVec(tri.v0, MinVec(tri.v1, tri.v2));
}

inline ChVector3d TriangleMax(const ChVulkanRTTriangle& tri) {
    return MaxVec(tri.v0, MaxVec(tri.v1, tri.v2));
}

inline ChVector3d TriangleCentroid(const ChVulkanRTTriangle& tri) {
    return (tri.v0 + tri.v1 + tri.v2) / 3.0;
}

ChVulkanRTRenderCache::TextureImage LoadTextureRGBA(const std::string& filename) {
    ChVulkanRTRenderCache::TextureImage texture;
    if (filename.empty())
        return texture;

    int width = 0;
    int height = 0;
    int channels = 0;

    if (stbi_is_hdr(filename.c_str())) {
        float* rawf = stbi_loadf(filename.c_str(), &width, &height, &channels, 4);
        (void)channels;
        if (!rawf || width <= 0 || height <= 0) {
            if (rawf)
                stbi_image_free(rawf);
            std::cerr << "WARNING: Chrono::Sensor Vulkan RT could not load HDR texture: " << filename << "\n";
            return texture;
        }

        texture.width = width;
        texture.height = height;
        texture.hdr = true;
        texture.rgba_float.resize(static_cast<size_t>(width) * static_cast<size_t>(height) * 4u);
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                const size_t dst = (static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)) * 4u;
                const size_t src = (static_cast<size_t>(height - y - 1) * static_cast<size_t>(width) + static_cast<size_t>(x)) * 4u;
                texture.rgba_float[dst + 0] = std::max(0.f, rawf[src + 0]);
                texture.rgba_float[dst + 1] = std::max(0.f, rawf[src + 1]);
                texture.rgba_float[dst + 2] = std::max(0.f, rawf[src + 2]);
                texture.rgba_float[dst + 3] = rawf[src + 3];
                if (rawf[src + 3] < 0.999f)
                    texture.has_translucent_alpha = true;
            }
        }
        stbi_image_free(rawf);
        return texture;
    }

    unsigned char* raw = stbi_load(filename.c_str(), &width, &height, &channels, 4);
    (void)channels;
    if (!raw || width <= 0 || height <= 0) {
        if (raw)
            stbi_image_free(raw);
        std::cerr << "WARNING: Chrono::Sensor Vulkan RT could not load texture: " << filename << "\n";
        return texture;
    }

    texture.width = width;
    texture.height = height;
    texture.rgba.resize(static_cast<size_t>(width) * static_cast<size_t>(height) * 4u);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const size_t dst = (static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)) * 4u;
            const size_t src = (static_cast<size_t>(height - y - 1) * static_cast<size_t>(width) + static_cast<size_t>(x)) * 4u;
            texture.rgba[dst + 0] = raw[src + 0];
            texture.rgba[dst + 1] = raw[src + 1];
            texture.rgba[dst + 2] = raw[src + 2];
            texture.rgba[dst + 3] = raw[src + 3];
            if (raw[src + 3] < 255)
                texture.has_translucent_alpha = true;
        }
    }
    stbi_image_free(raw);
    return texture;
}

void EnsureTexture(ChVulkanRTRenderCache& cache, const std::string& filename) {
    if (filename.empty() || cache.textures.find(filename) != cache.textures.end())
        return;
    cache.textures.emplace(filename, LoadTextureRGBA(filename));
}

void EnsureMaterialTextures(ChVulkanRTRenderCache& cache, const ChVulkanRTMaterial& mat) {
    EnsureTexture(cache, mat.diffuse_texture);
    EnsureTexture(cache, mat.specular_texture);
    EnsureTexture(cache, mat.emissive_texture);
    EnsureTexture(cache, mat.normal_texture);
    EnsureTexture(cache, mat.roughness_texture);
    EnsureTexture(cache, mat.metallic_texture);
    EnsureTexture(cache, mat.opacity_texture);
    EnsureTexture(cache, mat.weight_texture);
}

inline float Wrap01(float x) {
    x = x - std::floor(x);
    return x < 0.f ? x + 1.f : x;
}

TextureSample4 LerpTextureSample(const TextureSample4& a, const TextureSample4& b, float t) {
    TextureSample4 out;
    out.r = a.r + (b.r - a.r) * t;
    out.g = a.g + (b.g - a.g) * t;
    out.b = a.b + (b.b - a.b) * t;
    out.a = a.a + (b.a - a.a) * t;
    out.valid = a.valid || b.valid;
    return out;
}

TextureSample4 Texel(const ChVulkanRTRenderCache::TextureImage& texture, int x, int y) {
    x = (x % texture.width + texture.width) % texture.width;
    y = (y % texture.height + texture.height) % texture.height;
    const size_t idx = (static_cast<size_t>(y) * static_cast<size_t>(texture.width) + static_cast<size_t>(x)) * 4u;
    if (texture.hdr && idx + 3 < texture.rgba_float.size()) {
        return TextureSample4{texture.rgba_float[idx + 0],
                              texture.rgba_float[idx + 1],
                              texture.rgba_float[idx + 2],
                              texture.rgba_float[idx + 3],
                              true};
    }
    return TextureSample4{texture.rgba[idx + 0] / 255.f,
                          texture.rgba[idx + 1] / 255.f,
                          texture.rgba[idx + 2] / 255.f,
                          texture.rgba[idx + 3] / 255.f,
                          true};
}

TextureSample4 SampleTexture(const ChVulkanRTRenderCache* cache,
                             const std::string& filename,
                             const ChVulkanRTTexCoord& uv,
                             float scale_u,
                             float scale_v) {
    if (!cache || filename.empty())
        return TextureSample4{};
    const auto found = cache->textures.find(filename);
    if (found == cache->textures.end() || !found->second.Valid())
        return TextureSample4{};

    const auto& texture = found->second;
    const float u = Wrap01(uv.u * scale_u);
    const float v = Wrap01(uv.v * scale_v);
    const float x = u * static_cast<float>(texture.width) - 0.5f;
    const float y = v * static_cast<float>(texture.height) - 0.5f;
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const float tx = x - static_cast<float>(x0);
    const float ty = y - static_cast<float>(y0);

    const auto c00 = Texel(texture, x0, y0);
    const auto c10 = Texel(texture, x0 + 1, y0);
    const auto c01 = Texel(texture, x0, y0 + 1);
    const auto c11 = Texel(texture, x0 + 1, y0 + 1);
    return LerpTextureSample(LerpTextureSample(c00, c10, tx), LerpTextureSample(c01, c11, tx), ty);
}

EvaluatedMaterial EvaluateMaterial(const ChVulkanRTRenderCache* cache, const RayHit& hit) {
    EvaluatedMaterial out;
    out.diffuse = hit.material.diffuse;
    out.specular = hit.material.specular;
    out.emissive = hit.material.emissive;
    out.opacity = ClampFloat(hit.material.opacity, 0.f, 1.f);
    out.roughness = ClampFloat(hit.material.roughness, 0.02f, 1.f);
    out.metallic = ClampFloat(hit.material.metallic, 0.f, 1.f);
    out.use_specular_workflow = hit.material.use_specular_workflow;

    if (hit.has_uv) {
        if (!hit.material.diffuse_texture.empty()) {
            const auto tex = SampleTexture(cache, hit.material.diffuse_texture, hit.uv, hit.material.tex_scale_u, hit.material.tex_scale_v);
            if (tex.valid) {
                out.diffuse = PowRGB(ChVector3f(tex.r, tex.g, tex.b), 2.2f);
                // Keep parity with the GPU shader: Kd alpha is a continuous
                // surface-opacity term, not only a zero-alpha cutout mask.
                out.opacity *= ClampFloat(tex.a, 0.f, 1.f);
            }
        }
        if (!hit.material.specular_texture.empty()) {
            const auto tex = SampleTexture(cache, hit.material.specular_texture, hit.uv, hit.material.tex_scale_u, hit.material.tex_scale_v);
            if (tex.valid)
                out.specular = ChVector3f(tex.r, tex.g, tex.b);
        }
        if (!hit.material.emissive_texture.empty()) {
            const auto tex = SampleTexture(cache, hit.material.emissive_texture, hit.uv, hit.material.tex_scale_u, hit.material.tex_scale_v);
            if (tex.valid)
                out.emissive = ChVector3f(tex.r, tex.g, tex.b);
        }
        if (!hit.material.roughness_texture.empty()) {
            const auto tex = SampleTexture(cache, hit.material.roughness_texture, hit.uv, hit.material.tex_scale_u, hit.material.tex_scale_v);
            if (tex.valid)
                out.roughness = ClampFloat(tex.r, 0.02f, 1.f);
        }
        if (!hit.material.metallic_texture.empty()) {
            const auto tex = SampleTexture(cache, hit.material.metallic_texture, hit.uv, hit.material.tex_scale_u, hit.material.tex_scale_v);
            if (tex.valid)
                out.metallic = ClampFloat(tex.r, 0.f, 1.f);
        }
        if (!hit.material.opacity_texture.empty()) {
            const auto tex = SampleTexture(cache, hit.material.opacity_texture, hit.uv, hit.material.tex_scale_u, hit.material.tex_scale_v);
            if (tex.valid)
                out.opacity = ClampFloat(tex.r, 0.f, 1.f);
        }
        if (!hit.material.weight_texture.empty()) {
            const auto tex = SampleTexture(cache, hit.material.weight_texture, hit.uv, hit.material.tex_scale_u, hit.material.tex_scale_v);
            if (tex.valid)
                out.opacity *= ClampFloat(tex.r, 0.f, 1.f);
        }
    }

    return out;
}

bool IntersectAABBRange(const ChVector3d& ro,
                        const ChVector3d& rd,
                        const ChVector3d& bmin,
                        const ChVector3d& bmax,
                        double max_t,
                        double& entry_t) {
    double tmin = CH_VKRT_EPS;
    double tmax = max_t;

    for (int axis = 0; axis < 3; ++axis) {
        const double o = ro[axis];
        const double d = rd[axis];
        const double mn = bmin[axis];
        const double mx = bmax[axis];

        if (std::abs(d) < CH_VKRT_EPS) {
            if (o < mn || o > mx)
                return false;
            continue;
        }

        double t1 = (mn - o) / d;
        double t2 = (mx - o) / d;
        if (t1 > t2)
            std::swap(t1, t2);

        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
        if (tmin > tmax)
            return false;
    }

    entry_t = tmin;
    return true;
}

bool IntersectLocalAABB(const ChVector3d& ro,
                        const ChVector3d& rd,
                        const ChVector3d& bmin,
                        const ChVector3d& bmax,
                        double max_t) {
    double entry_t = 0.0;
    return IntersectAABBRange(ro, rd, bmin, bmax, max_t, entry_t);
}

void ExpandTransformedAABB(const ChVulkanRTPrimitive& primitive,
                           const ChVector3d& local_min,
                           const ChVector3d& local_max,
                           ChVector3d& world_min,
                           ChVector3d& world_max) {
    for (int ix = 0; ix < 2; ++ix) {
        for (int iy = 0; iy < 2; ++iy) {
            for (int iz = 0; iz < 2; ++iz) {
                const ChVector3d p(ix ? local_max.x() : local_min.x(),
                                   iy ? local_max.y() : local_min.y(),
                                   iz ? local_max.z() : local_min.z());
                ExpandAABB(world_min, world_max, primitive.frame.TransformPointLocalToParent(p));
            }
        }
    }
}

bool PrimitiveLocalBounds(const ChVulkanRTPrimitive& primitive, ChVector3d& local_min, ChVector3d& local_max) {
    switch (primitive.type) {
        case ChVulkanRTPrimitiveType::BOX: {
            const ChVector3d half(std::max(CH_VKRT_EPS, primitive.scale.x() * 0.5),
                                  std::max(CH_VKRT_EPS, primitive.scale.y() * 0.5),
                                  std::max(CH_VKRT_EPS, primitive.scale.z() * 0.5));
            local_min = -half;
            local_max = half;
            return true;
        }
        case ChVulkanRTPrimitiveType::SPHERE:
        case ChVulkanRTPrimitiveType::MESH_PROXY: {
            const double radius = std::max(CH_VKRT_EPS, primitive.scale.x());
            local_min = ChVector3d(-radius, -radius, -radius);
            local_max = ChVector3d(radius, radius, radius);
            return true;
        }
        case ChVulkanRTPrimitiveType::CYLINDER: {
            const double radius = std::max(CH_VKRT_EPS, primitive.scale.x());
            const double half_h = std::max(CH_VKRT_EPS, primitive.scale.z() * 0.5);
            local_min = ChVector3d(-radius, -radius, -half_h);
            local_max = ChVector3d(radius, radius, half_h);
            return true;
        }
        case ChVulkanRTPrimitiveType::TRIANGLE_MESH:
            if (!primitive.has_aabb)
                return false;
            local_min = primitive.aabb_min;
            local_max = primitive.aabb_max;
            return true;
    }
    return false;
}

int BuildTriangleBVH(ChVulkanRTRenderCache::CachedPrimitive& cached, uint32_t first, uint32_t count) {
    using Node = ChVulkanRTRenderCache::BvhNode;
    const ChVulkanRTPrimitive& primitive = *cached.primitive;

    const int node_id = static_cast<int>(cached.triangle_nodes.size());
    cached.triangle_nodes.push_back(Node{});
    Node& node = cached.triangle_nodes.back();
    node.first = first;
    node.count = count;
    ResetAABB(node.bmin, node.bmax);

    ChVector3d centroid_min;
    ChVector3d centroid_max;
    ResetAABB(centroid_min, centroid_max);

    for (uint32_t i = first; i < first + count; ++i) {
        const auto& tri = primitive.triangles[cached.triangle_indices[i]];
        ExpandAABB(node.bmin, node.bmax, TriangleMin(tri), TriangleMax(tri));
        ExpandAABB(centroid_min, centroid_max, TriangleCentroid(tri));
    }

    if (count <= CH_VKRT_TRIANGLE_LEAF_SIZE)
        return node_id;

    const int axis = LongestAxis(centroid_min, centroid_max);
    if (centroid_max[axis] - centroid_min[axis] <= CH_VKRT_EPS)
        return node_id;

    const uint32_t mid = first + count / 2;
    auto begin = cached.triangle_indices.begin();
    std::nth_element(begin + first, begin + mid, begin + first + count, [&](uint32_t a, uint32_t b) {
        return TriangleCentroid(primitive.triangles[a])[axis] < TriangleCentroid(primitive.triangles[b])[axis];
    });

    const int left = BuildTriangleBVH(cached, first, mid - first);
    const int right = BuildTriangleBVH(cached, mid, first + count - mid);
    cached.triangle_nodes[static_cast<size_t>(node_id)].left = left;
    cached.triangle_nodes[static_cast<size_t>(node_id)].right = right;
    cached.triangle_nodes[static_cast<size_t>(node_id)].count = 0;
    return node_id;
}

int BuildPrimitiveBVH(ChVulkanRTRenderCache& cache, uint32_t first, uint32_t count) {
    using Node = ChVulkanRTRenderCache::BvhNode;

    const int node_id = static_cast<int>(cache.primitive_nodes.size());
    cache.primitive_nodes.push_back(Node{});
    Node& node = cache.primitive_nodes.back();
    node.first = first;
    node.count = count;
    ResetAABB(node.bmin, node.bmax);

    ChVector3d centroid_min;
    ChVector3d centroid_max;
    ResetAABB(centroid_min, centroid_max);

    for (uint32_t i = first; i < first + count; ++i) {
        const auto& primitive = cache.primitives[cache.primitive_indices[i]];
        ExpandAABB(node.bmin, node.bmax, primitive.world_min, primitive.world_max);
        ExpandAABB(centroid_min, centroid_max, Centroid(primitive.world_min, primitive.world_max));
    }

    if (count <= CH_VKRT_PRIMITIVE_LEAF_SIZE)
        return node_id;

    const int axis = LongestAxis(centroid_min, centroid_max);
    if (centroid_max[axis] - centroid_min[axis] <= CH_VKRT_EPS)
        return node_id;

    const uint32_t mid = first + count / 2;
    auto begin = cache.primitive_indices.begin();
    std::nth_element(begin + first, begin + mid, begin + first + count, [&](uint32_t a, uint32_t b) {
        const auto& pa = cache.primitives[a];
        const auto& pb = cache.primitives[b];
        return Centroid(pa.world_min, pa.world_max)[axis] < Centroid(pb.world_min, pb.world_max)[axis];
    });

    const int left = BuildPrimitiveBVH(cache, first, mid - first);
    const int right = BuildPrimitiveBVH(cache, mid, first + count - mid);
    cache.primitive_nodes[static_cast<size_t>(node_id)].left = left;
    cache.primitive_nodes[static_cast<size_t>(node_id)].right = right;
    cache.primitive_nodes[static_cast<size_t>(node_id)].count = 0;
    return node_id;
}

void BuildRenderCache(ChVulkanRTRenderCache& cache, const std::shared_ptr<ChVulkanRTScene>& scene) {
    cache = ChVulkanRTRenderCache{};
    if (!scene)
        return;

    cache.scene = scene.get();
    cache.scene_revision = scene->GetRevision();

    const auto& background = scene->GetBackground();
    if (background.mode == BackgroundMode::ENVIRONMENT_MAP)
        EnsureTexture(cache, background.env_tex);
    for (const auto& light : scene->GetLights())
        EnsureTexture(cache, light.texture);

    const auto& primitives = scene->GetPrimitives();
    cache.primitives.reserve(primitives.size());
    cache.primitive_indices.reserve(primitives.size());

    for (const auto& primitive : primitives) {
        EnsureMaterialTextures(cache, primitive.material);
        for (const auto& tri : primitive.triangles)
            EnsureMaterialTextures(cache, tri.material);

        ChVector3d local_min;
        ChVector3d local_max;
        if (!PrimitiveLocalBounds(primitive, local_min, local_max))
            continue;

        ChVulkanRTRenderCache::CachedPrimitive cached;
        cached.primitive = &primitive;
        ResetAABB(cached.world_min, cached.world_max);
        ExpandTransformedAABB(primitive, local_min, local_max, cached.world_min, cached.world_max);

        if (primitive.type == ChVulkanRTPrimitiveType::TRIANGLE_MESH && !primitive.triangles.empty()) {
            cached.triangle_indices.resize(primitive.triangles.size());
            std::iota(cached.triangle_indices.begin(), cached.triangle_indices.end(), 0u);
            cached.triangle_nodes.reserve(std::max<size_t>(1, primitive.triangles.size() * 2));
            BuildTriangleBVH(cached, 0, static_cast<uint32_t>(cached.triangle_indices.size()));
        }

        cache.primitive_indices.push_back(static_cast<uint32_t>(cache.primitives.size()));
        cache.primitives.push_back(std::move(cached));
    }

    if (!cache.primitive_indices.empty()) {
        cache.primitive_nodes.reserve(std::max<size_t>(1, cache.primitive_indices.size() * 2));
        BuildPrimitiveBVH(cache, 0, static_cast<uint32_t>(cache.primitive_indices.size()));
    }
}

bool IntersectSphere(const ChVulkanRTPrimitive& primitive, const ChVector3d& origin, const ChVector3d& dir, RayHit& best) {
    const ChVector3d ro = primitive.frame.TransformPointParentToLocal(origin);
    const ChVector3d rd = primitive.frame.TransformDirectionParentToLocal(dir);
    const double radius = std::max(CH_VKRT_EPS, primitive.scale.x());

    const double a = rd.Dot(rd);
    const double b = 2.0 * ro.Dot(rd);
    const double c = ro.Dot(ro) - radius * radius;
    const double disc = b * b - 4.0 * a * c;
    if (disc < 0.0)
        return false;

    const double sqrt_disc = std::sqrt(disc);
    double t = (-b - sqrt_disc) / (2.0 * a);
    if (t <= CH_VKRT_EPS)
        t = (-b + sqrt_disc) / (2.0 * a);
    if (t <= CH_VKRT_EPS || t >= best.t)
        return false;

    const ChVector3d local_hit = ro + rd * t;
    const ChVector3d local_normal = NormalizeSafe(local_hit);
    // Match optix/shaders/sphere.cu exactly: normals are not face-forwarded,
    // sphere V is linear in z, and the tangent/atan2 convention is p.y,-p.x.
    const ChVector3d tangent = ChVector3d(local_hit.y(), -local_hit.x(), 0.0);

    best.hit = true;
    best.t = t;
    best.normal = NormalizeSafe(primitive.frame.TransformDirectionLocalToParent(local_normal));
    best.tangent = NormalizeSafe(primitive.frame.TransformDirectionLocalToParent(tangent), ChVector3d(1.0, 0.0, 0.0));
    best.uv = ChVulkanRTTexCoord{static_cast<float>(std::atan2(local_hit.x(), local_hit.y()) / (2.0 * CH_VKRT_PI) + 0.5),
                                  static_cast<float>(local_hit.z() / (2.0 * radius) + 0.5)};
    best.has_uv = true;
    best.material = primitive.material;
    return true;
}
bool IntersectBox(const ChVulkanRTPrimitive& primitive, const ChVector3d& origin, const ChVector3d& dir, RayHit& best) {
    const ChVector3d ro = primitive.frame.TransformPointParentToLocal(origin);
    const ChVector3d rd = primitive.frame.TransformDirectionParentToLocal(dir);
    const ChVector3d half(std::max(CH_VKRT_EPS, primitive.scale.x() * 0.5),
                          std::max(CH_VKRT_EPS, primitive.scale.y() * 0.5),
                          std::max(CH_VKRT_EPS, primitive.scale.z() * 0.5));

    double tmin = -std::numeric_limits<double>::infinity();
    double tmax = std::numeric_limits<double>::infinity();
    ChVector3d nmin(0.0, 0.0, 0.0);

    for (int axis = 0; axis < 3; ++axis) {
        const double o = ro[axis];
        const double d = rd[axis];
        const double h = half[axis];
        if (std::abs(d) < CH_VKRT_EPS) {
            if (o < -h || o > h)
                return false;
            continue;
        }

        double t1 = (-h - o) / d;
        double t2 = (h - o) / d;
        double nsign = -1.0;
        if (t1 > t2) {
            std::swap(t1, t2);
            nsign = 1.0;
        }

        if (t1 > tmin) {
            tmin = t1;
            nmin = ChVector3d(0.0, 0.0, 0.0);
            nmin[axis] = nsign;
        }
        tmax = std::min(tmax, t2);
        if (tmin > tmax)
            return false;
    }

    double t = tmin;
    if (t <= CH_VKRT_EPS)
        t = tmax;
    if (t <= CH_VKRT_EPS || t >= best.t)
        return false;

    const ChVector3d local_hit = ro + rd * t;
    float u = 0.f;
    float v = 0.f;
    ChVector3d tangent(1.0, 0.0, 0.0);
    const int axis = std::abs(nmin.x()) > 0.5 ? 0 : (std::abs(nmin.y()) > 0.5 ? 1 : 2);
    if (axis == 0) {
        u = static_cast<float>(local_hit.y() / primitive.scale.y() + 0.5);
        v = static_cast<float>((local_hit.z() / primitive.scale.z() + 0.5) * nmin.x());
        tangent = ChVector3d(0.0, 1.0, 0.0);
    } else if (axis == 1) {
        u = static_cast<float>(local_hit.x() / primitive.scale.x() + 0.5);
        v = static_cast<float>(-(local_hit.z() / primitive.scale.z() + 0.5) * nmin.y());
        tangent = ChVector3d(1.0, 0.0, 0.0);
    } else {
        u = static_cast<float>(local_hit.x() / primitive.scale.x() + 0.5);
        v = static_cast<float>((local_hit.y() / primitive.scale.y() + 0.5) * nmin.z());
        tangent = ChVector3d(1.0, 0.0, 0.0);
    }

    best.hit = true;
    best.t = t;
    best.normal = NormalizeSafe(primitive.frame.TransformDirectionLocalToParent(nmin));
    best.tangent = NormalizeSafe(primitive.frame.TransformDirectionLocalToParent(tangent), ChVector3d(1.0, 0.0, 0.0));
    best.uv = ChVulkanRTTexCoord{u, v};
    best.has_uv = true;
    best.material = primitive.material;
    return true;
}
bool IntersectCylinder(const ChVulkanRTPrimitive& primitive, const ChVector3d& origin, const ChVector3d& dir, RayHit& best) {
    const ChVector3d ro = primitive.frame.TransformPointParentToLocal(origin);
    const ChVector3d rd = primitive.frame.TransformDirectionParentToLocal(dir);
    const double radius = std::max(CH_VKRT_EPS, primitive.scale.x());
    const double half_h = std::max(CH_VKRT_EPS, primitive.scale.z() * 0.5);

    bool found = false;
    bool side_hit = false;
    double best_t = best.t;
    ChVector3d best_n(0.0, 0.0, 1.0);
    ChVector3d best_p(0.0, 0.0, 0.0);

    const double a = rd.x() * rd.x() + rd.y() * rd.y();
    const double b = 2.0 * (ro.x() * rd.x() + ro.y() * rd.y());
    const double c = ro.x() * ro.x() + ro.y() * ro.y() - radius * radius;
    const double disc = b * b - 4.0 * a * c;
    if (a > CH_VKRT_EPS && disc >= 0.0) {
        const double sqrt_disc = std::sqrt(disc);
        for (double t : {(-b - sqrt_disc) / (2.0 * a), (-b + sqrt_disc) / (2.0 * a)}) {
            const double z = ro.z() + rd.z() * t;
            if (t > CH_VKRT_EPS && t < best_t && z >= -half_h && z <= half_h) {
                best_t = t;
                best_p = ro + rd * t;
                best_n = NormalizeSafe(ChVector3d(best_p.x(), best_p.y(), 0.0));
                side_hit = true;
                found = true;
            }
        }
    }

    if (std::abs(rd.z()) > CH_VKRT_EPS) {
        for (double zcap : {-half_h, half_h}) {
            const double t = (zcap - ro.z()) / rd.z();
            const ChVector3d p = ro + rd * t;
            if (t > CH_VKRT_EPS && t < best_t && p.x() * p.x() + p.y() * p.y() <= radius * radius) {
                best_t = t;
                best_p = p;
                // Match optix/shaders/cylinder.cu exactly: both cap hit programs
                // report the historical custom-primitive normal (0,1,0), not +/-Z.
                // This matters for normal-camera parity and for cap highlights.
                best_n = ChVector3d(0.0, 1.0, 0.0);
                side_hit = false;
                found = true;
            }
        }
    }

    if (!found)
        return false;

    float u = 0.f;
    float v = 0.f;
    ChVector3d tangent(1.0, 0.0, 0.0);
    if (side_hit) {
        u = static_cast<float>(std::atan2(best_p.y(), best_p.x()) / (2.0 * CH_VKRT_PI));
        v = static_cast<float>(best_p.z() / primitive.scale.z() + 0.5);
        tangent = ChVector3d(best_p.y(), 0.0, -best_p.x());
    } else if (best_p.z() > 0.0) {
        u = static_cast<float>(-best_p.x() / (2.0 * radius) + 0.5);
        v = static_cast<float>(-best_p.y() / (2.0 * radius) + 0.5);
        tangent = ChVector3d(-1.0, 0.0, 0.0);
    } else {
        u = static_cast<float>(best_p.x() / (2.0 * radius) + 0.5);
        v = static_cast<float>(best_p.y() / (2.0 * radius) + 0.5);
        tangent = ChVector3d(1.0, 0.0, 0.0);
    }

    best.hit = true;
    best.t = best_t;
    best.normal = NormalizeSafe(primitive.frame.TransformDirectionLocalToParent(best_n));
    best.tangent = NormalizeSafe(primitive.frame.TransformDirectionLocalToParent(tangent), ChVector3d(1.0, 0.0, 0.0));
    best.uv = ChVulkanRTTexCoord{u, v};
    best.has_uv = true;
    best.material = primitive.material;
    return true;
}
bool IntersectTriangle(const ChVulkanRTPrimitive& primitive,
                       const ChVulkanRTTriangle& tri,
                       const ChVector3d& rd,
                       const ChVector3d& ro,
                       RayHit& best) {
    const ChVector3d e1 = tri.v1 - tri.v0;
    const ChVector3d e2 = tri.v2 - tri.v0;
    const ChVector3d pvec = rd.Cross(e2);
    const double det = e1.Dot(pvec);

    if (primitive.backface_cull) {
        if (det <= CH_VKRT_EPS)
            return false;
    } else if (std::abs(det) <= CH_VKRT_EPS) {
        return false;
    }

    const double inv_det = 1.0 / det;
    const ChVector3d tvec = ro - tri.v0;
    const double u = tvec.Dot(pvec) * inv_det;
    if (u < 0.0 || u > 1.0)
        return false;

    const ChVector3d qvec = tvec.Cross(e1);
    const double v = rd.Dot(qvec) * inv_det;
    if (v < 0.0 || u + v > 1.0)
        return false;

    const double t = e2.Dot(qvec) * inv_det;
    if (t <= CH_VKRT_EPS || t >= best.t)
        return false;

    const double w = 1.0 - u - v;
    ChVector3d local_normal = tri.normal;
    if (tri.has_vertex_normals)
        local_normal = NormalizeSafe(w * tri.n0 + u * tri.n1 + v * tri.n2, tri.normal);
    // OptiX leaves triangle normals in their interpolated geometric direction;
    // do not face-forward them here or normal-camera and back-face lighting differ.

    best.hit = true;
    best.t = t;
    best.normal = NormalizeSafe(primitive.frame.TransformDirectionLocalToParent(local_normal));
    best.tangent = NormalizeSafe(primitive.frame.TransformDirectionLocalToParent(tri.tangent), ChVector3d(1.0, 0.0, 0.0));
    if (tri.has_uvs) {
        best.uv.u = static_cast<float>(w * tri.uv0.u + u * tri.uv1.u + v * tri.uv2.u);
        best.uv.v = static_cast<float>(w * tri.uv0.v + u * tri.uv1.v + v * tri.uv2.v);
        best.has_uv = true;
    } else {
        best.uv = ChVulkanRTTexCoord{};
        best.has_uv = false;
    }
    best.material = tri.material;
    return true;
}
bool IntersectMesh(const ChVulkanRTRenderCache::CachedPrimitive& cached,
                   const ChVector3d& origin,
                   const ChVector3d& dir,
                   RayHit& best) {
    const ChVulkanRTPrimitive& primitive = *cached.primitive;
    if (primitive.triangles.empty() || !primitive.has_aabb)
        return false;

    const ChVector3d ro = primitive.frame.TransformPointParentToLocal(origin);
    const ChVector3d rd = primitive.frame.TransformDirectionParentToLocal(dir);

    if (!IntersectLocalAABB(ro, rd, primitive.aabb_min, primitive.aabb_max, best.t))
        return false;

    bool any = false;
    if (cached.triangle_nodes.empty()) {
        for (const auto& tri : primitive.triangles)
            any = IntersectTriangle(primitive, tri, rd, ro, best) || any;
        return any;
    }

    std::array<int, CH_VKRT_TRAVERSAL_STACK_SIZE> stack;
    size_t stack_size = 0;
    stack[stack_size++] = 0;

    while (stack_size > 0) {
        const int node_id = stack[--stack_size];
        const auto& node = cached.triangle_nodes[static_cast<size_t>(node_id)];
        double entry_t = 0.0;
        if (!IntersectAABBRange(ro, rd, node.bmin, node.bmax, best.t, entry_t) || entry_t >= best.t)
            continue;

        if (node.IsLeaf()) {
            for (uint32_t i = node.first; i < node.first + node.count; ++i) {
                const auto tri_id = cached.triangle_indices[i];
                any = IntersectTriangle(primitive, primitive.triangles[tri_id], rd, ro, best) || any;
            }
        } else {
            if (stack_size + 2 <= stack.size()) {
                stack[stack_size++] = node.left;
                stack[stack_size++] = node.right;
            }
        }
    }

    return any;
}

bool IntersectPrimitive(const ChVulkanRTRenderCache::CachedPrimitive& cached,
                        const ChVector3d& origin,
                        const ChVector3d& dir,
                        RayHit& best) {
    const ChVulkanRTPrimitive& primitive = *cached.primitive;
    const double old_t = best.t;
    bool hit = false;
    switch (primitive.type) {
        case ChVulkanRTPrimitiveType::BOX:
            hit = IntersectBox(primitive, origin, dir, best);
            break;
        case ChVulkanRTPrimitiveType::SPHERE:
        case ChVulkanRTPrimitiveType::MESH_PROXY:
            hit = IntersectSphere(primitive, origin, dir, best);
            break;
        case ChVulkanRTPrimitiveType::CYLINDER:
            hit = IntersectCylinder(primitive, origin, dir, best);
            break;
        case ChVulkanRTPrimitiveType::TRIANGLE_MESH:
            hit = IntersectMesh(cached, origin, dir, best);
            break;
    }
    if (hit && best.hit && best.t <= old_t) {
        best.primitive_origin = primitive.frame.TransformPointLocalToParent(ChVector3d(0.0, 0.0, 0.0));
        best.translational_velocity = primitive.translational_velocity;
        best.angular_velocity = primitive.angular_velocity;
        best.object_id = primitive.object_id;
    }
    return hit;
}

RayHit TraceRay(const ChVulkanRTRenderCache* cache,
                const std::shared_ptr<ChVulkanRTScene>& scene,
                const ChVector3d& origin,
                const ChVector3d& dir,
                double max_t = std::numeric_limits<double>::max()) {
    RayHit best;
    best.t = max_t;

    if (cache && !cache->primitive_nodes.empty()) {
        std::array<int, CH_VKRT_TRAVERSAL_STACK_SIZE> stack;
        size_t stack_size = 0;
        stack[stack_size++] = 0;

        while (stack_size > 0) {
            const int node_id = stack[--stack_size];
            const auto& node = cache->primitive_nodes[static_cast<size_t>(node_id)];
            double entry_t = 0.0;
            if (!IntersectAABBRange(origin, dir, node.bmin, node.bmax, best.t, entry_t) || entry_t >= best.t)
                continue;

            if (node.IsLeaf()) {
                for (uint32_t i = node.first; i < node.first + node.count; ++i) {
                    const uint32_t primitive_id = cache->primitive_indices[i];
                    const auto& primitive = cache->primitives[primitive_id];
                    IntersectPrimitive(primitive, origin, dir, best);
                }
            } else if (stack_size + 2 <= stack.size()) {
                stack[stack_size++] = node.left;
                stack[stack_size++] = node.right;
            }
        }
        return best;
    }

    if (scene) {
        for (const auto& primitive : scene->GetPrimitives()) {
            ChVulkanRTRenderCache::CachedPrimitive cached;
            cached.primitive = &primitive;
            IntersectPrimitive(cached, origin, dir, best);
        }
    }
    return best;
}

ChVector3f BackgroundColor(const ChVulkanRTRenderCache* cache,
                           const std::shared_ptr<ChVulkanRTScene>& scene,
                           const ChVector3d& ray_dir) {
    const Background* background = scene ? &scene->GetBackground() : nullptr;
    if (!background)
        return ChVector3f(0.4f, 0.5f, 0.6f);

    if (background->mode == BackgroundMode::ENVIRONMENT_MAP) {
        // Match optix/shaders/miss.cu: equirectangular environment maps assume z-up,
        // x/y azimuth, and texture RGB is converted from sRGB into linear space before
        // the camera raygen applies the final gamma curve.
        const ChVector3d d = NormalizeSafe(ray_dir);
        const float tex_u = static_cast<float>(std::atan2(d.y(), d.x()) / (2.0 * CH_VKRT_PI) + 0.5);
        const float tex_v = static_cast<float>(std::asin(std::max(-1.0, std::min(1.0, d.z()))) / CH_VKRT_PI + 0.5);
        const auto tex = SampleTexture(cache, background->env_tex, ChVulkanRTTexCoord{tex_u, tex_v}, 1.f, 1.f);
        if (tex.valid)
            return PowRGB(ChVector3f(tex.r, tex.g, tex.b), 2.2f);
        return background->color_zenith;
    }

    if (background->mode == BackgroundMode::GRADIENT) {
        // OptiX uses max(0, ray_dir.z) in the miss shader, not a [-1,1] remap.
        // Downward rays therefore return the horizon color exactly.
        const float mix = std::max(0.f, static_cast<float>(NormalizeSafe(ray_dir).z()));
        return mix * background->color_zenith + (1.f - mix) * background->color_horizon;
    }

    return background->color_zenith;
}

ChVector3d ShadingNormal(const ChVulkanRTRenderCache* cache, const RayHit& hit) {
    ChVector3d n = NormalizeSafe(hit.normal);
    if (!hit.has_uv || hit.material.normal_texture.empty())
        return n;

    const auto tex = SampleTexture(cache, hit.material.normal_texture, hit.uv, hit.material.tex_scale_u, hit.material.tex_scale_v);
    if (!tex.valid)
        return n;

    ChVector3d tangent = NormalizeSafe(hit.tangent - hit.tangent.Dot(n) * n, ChVector3d(1.0, 0.0, 0.0));
    ChVector3d bitangent = NormalizeSafe(n.Cross(tangent), ChVector3d(0.0, 1.0, 0.0));
    const ChVector3d normal_delta(2.0 * tex.r - 1.0, 2.0 * tex.g - 1.0, 2.0 * tex.b - 1.0);
    ChVector3d mapped = normal_delta.x() * tangent + normal_delta.y() * bitangent + normal_delta.z() * n;
    return NormalizeSafe(mapped, n);
}

ChVector3f TraceCameraColor(const ChVulkanRTRenderCache* cache,
                            const std::shared_ptr<ChVulkanRTScene>& scene,
                            const ChVector3d& origin,
                            const ChVector3d& dir,
                            int depth,
                            bool use_gi);

ChVector3f ShadowAttenuation(const ChVulkanRTRenderCache* cache,
                             const std::shared_ptr<ChVulkanRTScene>& scene,
                             const ChVector3d& origin,
                             const ChVector3d& dir,
                             double max_t,
                             int depth = 0) {
    if (depth >= 8)
        return ChVector3f(1.f, 1.f, 1.f);

    const RayHit hit = TraceRay(cache, scene, origin, dir, max_t);
    if (!hit.hit)
        return ChVector3f(1.f, 1.f, 1.f);

    const EvaluatedMaterial mat = EvaluateMaterial(cache, hit);
    if (mat.opacity >= 1.f - 1e-5f)
        return ChVector3f(0.f, 0.f, 0.f);

    float pass = ClampFloat(1.f - mat.opacity, 0.f, 1.f);
    const ChVector3f shadow_tint = 0.75f * ChVector3f(1.f, 1.f, 1.f) + 0.25f * Clamp01(mat.diffuse);
    if (mat.roughness < 0.35f && MaxComponent(mat.specular) > 0.04f)
        pass = std::max(pass, 0.82f * pass + 0.18f);
    const ChVector3d next_origin = origin + dir * (hit.t + CH_VKRT_SHADOW_EPS);
    const double remaining = std::isfinite(max_t) ? std::max(0.0, max_t - hit.t - CH_VKRT_SHADOW_EPS) : max_t;
    return Mul(shadow_tint, pass * ShadowAttenuation(cache, scene, next_origin, dir, remaining, depth + 1));
}

ChVector3f Shade(const ChVulkanRTRenderCache* cache,
                 const std::shared_ptr<ChVulkanRTScene>& scene,
                 const RayHit& hit,
                 const ChVector3d& origin,
                 const ChVector3d& dir,
                 int depth,
                 bool use_gi) {
    const ChVector3d hit_pos = origin + dir * hit.t;
    const ChVector3d normal = ShadingNormal(cache, hit);
    const ChVector3d view_dir = NormalizeSafe(-dir);
    const EvaluatedMaterial mat = EvaluateMaterial(cache, hit);

    if (mat.opacity <= 1e-6f)
        return TraceCameraColor(cache, scene, hit_pos + dir * CH_VKRT_SHADOW_EPS, dir, depth + 1, use_gi);

    const float ndv = static_cast<float>(std::max(0.0, normal.Dot(view_dir)));
    const ChVector3f ambient_light = scene ? scene->GetAmbientLight() : ChVector3f(0.08f, 0.08f, 0.08f);
    const ChVector3f one(1.f, 1.f, 1.f);
    ChVector3f f0 = mat.use_specular_workflow ? 0.08f * mat.specular
                                               : mat.metallic * mat.diffuse + (1.f - mat.metallic) * ChVector3f(0.04f, 0.04f, 0.04f);

    const float ambient_shape = 0.35f + 0.65f * ClampFloat(static_cast<float>(normal.z() * 0.5 + 0.5), 0.f, 1.f);
    ChVector3f color(0.f, 0.f, 0.f);
    if (!use_gi) {
        const ChVector3f ambient_floor(std::max(ambient_light.x(), 0.025f),
                                       std::max(ambient_light.y(), 0.025f),
                                       std::max(ambient_light.z(), 0.025f));
        const ChVector3f sky = BackgroundColor(cache, scene, normal);
        const float sky_weight = (scene && scene->GetBackground().mode == BackgroundMode::SOLID_COLOR) ? 0.12f : 0.28f;
        color += mat.opacity * Mul(mat.diffuse, ambient_floor + sky_weight * sky) * ambient_shape;
    }

    bool used_explicit_light = false;
    if (scene) {
        for (const auto& light : scene->GetLights()) {
            ChVector3d light_dir(0.0, 0.0, 0.0);
            double max_shadow_t = std::numeric_limits<double>::max();
            float attenuation = 1.f;

            if (light.type == LightType::DIRECTIONAL_LIGHT) {
                light_dir = NormalizeSafe(ChVector3d(light.dir));
            } else if (light.type == LightType::POINT_LIGHT || light.type == LightType::SPOT_LIGHT ||
                       light.type == LightType::RECTANGLE_LIGHT || light.type == LightType::DISK_LIGHT) {
                ChVector3d to_light = ChVector3d(light.pos) - hit_pos;
                const double dist = to_light.Length();
                max_shadow_t = std::max(CH_VKRT_EPS, dist - CH_VKRT_SHADOW_EPS);
                light_dir = NormalizeSafe(to_light);
                if (!light.const_color) {
                    attenuation = light.atten_scale / static_cast<float>(std::max(CH_VKRT_EPS, dist * dist));
                }
                if (light.type == LightType::SPOT_LIGHT) {
                    const ChVector3d spot_dir = NormalizeSafe(ChVector3d(light.dir));
                    const double angle = std::acos(std::max(-1.0, std::min(1.0, spot_dir.Dot(-light_dir))));
                    if (2.0 * angle > static_cast<double>(light.angle)) {
                        attenuation = 0.f;
                    } else if (!light.const_color && light.angle_atten_rate >= 0.f) {
                        float angle_attenuation = ClampFloat(light.angle_atten_rate * (light.angle - 2.f * static_cast<float>(angle)), 0.f, 1.f);
                        attenuation *= angle_attenuation * angle_attenuation;
                    }
                } else if (light.type == LightType::RECTANGLE_LIGHT || light.type == LightType::DISK_LIGHT) {
                    // Area lights are currently approximated by their center point in the Vulkan
                    // bring-up renderer. The one-sided term keeps the OptiX API semantics: the
                    // light emits along its stored surface normal/direction.
                    const ChVector3d area_dir = NormalizeSafe(ChVector3d(light.dir));
                    attenuation *= ClampFloat(static_cast<float>(area_dir.Dot(-light_dir)), 0.f, 1.f);
                }
            } else if (light.type == LightType::ENVIRONMENT_LIGHT) {
                const std::array<ChVector3d, 4> env_dirs = {
                    NormalizeSafe(normal + ChVector3d(0.36, 0.10, 0.93), normal),
                    NormalizeSafe(normal + ChVector3d(-0.48, 0.42, 0.77), normal),
                    NormalizeSafe(normal + ChVector3d(0.18, -0.72, 0.67), normal),
                    NormalizeSafe(normal + ChVector3d(-0.08, -0.18, 0.98), normal)};
                ChVector3f env_sum(0.f, 0.f, 0.f);
                float env_w = 0.f;
                for (const auto& env_dir : env_dirs) {
                    const float env_ndl = static_cast<float>(std::max(0.0, normal.Dot(env_dir)));
                    if (env_ndl <= 0.f)
                        continue;
                    const ChVector3f env_shadow = ShadowAttenuation(cache, scene, hit_pos + env_dir * CH_VKRT_SHADOW_EPS, env_dir, std::numeric_limits<double>::max());
                    env_sum += env_ndl * Mul(BackgroundColor(cache, scene, env_dir), env_shadow);
                    env_w += env_ndl;
                }
                if (env_w > 0.f)
                    color += mat.opacity * Mul(Mul(mat.diffuse, light.color), env_sum / env_w);
                used_explicit_light = true;
                continue;
            } else {
                continue;
            }

            if (attenuation <= 0.f)
                continue;

            const float ndl = static_cast<float>(std::max(0.0, normal.Dot(light_dir)));
            if (ndl <= 0.f)
                continue;

            // Match the OptiX shadow trace convention: the ray starts at the hit point and skips
            // the first scene-epsilon interval along the light ray. Offsetting along the shading
            // normal introduces scale-dependent light leaks at contact shadows and on steep normals.
            const ChVector3d shadow_origin = hit_pos + light_dir * CH_VKRT_SHADOW_EPS;
            const double shadow_max_t = std::isfinite(max_shadow_t) ? std::max(0.0, max_shadow_t - CH_VKRT_SHADOW_EPS) : max_shadow_t;
            const ChVector3f shadow = ShadowAttenuation(cache, scene, shadow_origin, light_dir, shadow_max_t);
            if (MaxComponent(shadow) <= 1e-5f) {
                used_explicit_light = true;
                continue;
            }

            const ChVector3d half_vec = NormalizeSafe(light_dir + view_dir, normal);
            const float ndh = static_cast<float>(std::max(0.0, normal.Dot(half_vec)));
            const float vdh = static_cast<float>(std::max(0.0, view_dir.Dot(half_vec)));
            const ChVector3f fresnel = FresnelSchlick(vdh, f0);
            const float d_term = NormalDistOptix(ndh, mat.roughness);
            const float g_term = HammonSmithOptix(ndv, ndl, mat.roughness);
            // Match OptiX direct-light semantics: CheckVisibleAndSample*Light stores one
            // Lambert cosine in LightSample::L and the camera BRDF code applies NdL again.
            // This restores the grazing-angle shadow contrast visible in the OptiX demos.
            const ChVector3f incoming = attenuation * ndl * ndl * Mul(light.color, shadow);
            const ChVector3f diffuse_albedo = mat.use_specular_workflow ? mat.diffuse : (1.f - mat.metallic) * mat.diffuse;
            const ChVector3f diffuse_term = Mul(Mul(one - fresnel, diffuse_albedo), incoming);
            const ChVector3f specular_term = d_term * g_term * Mul(fresnel, incoming);
            color += mat.opacity * (diffuse_term + specular_term);
            used_explicit_light = true;
        }
    }

    (void)used_explicit_light;

    color += hit.material.emissive_power * mat.emissive * std::abs(static_cast<float>(normal.Dot(view_dir)));

    if (depth < 6) {
        if (mat.opacity < 1.f - 1.f / 255.f) {
            const bool likely_glass = mat.opacity > 1e-5f && mat.opacity < 0.98f &&
                                      mat.roughness < 0.55f && MaxComponent(mat.specular) > 0.035f;
            ChVector3d refract_dir = dir;
            ChVector3d face_normal = dir.Dot(normal) < 0.0 ? normal : -normal;
            if (likely_glass) {
                const double eta = dir.Dot(normal) < 0.0 ? (1.0 / 1.5) : 1.5;
                const double cos_i = -std::max(-1.0, std::min(1.0, face_normal.Dot(dir)));
                const double sin2_t = eta * eta * std::max(0.0, 1.0 - cos_i * cos_i);
                if (sin2_t <= 1.0) {
                    refract_dir = NormalizeSafe(eta * dir + (eta * cos_i - std::sqrt(1.0 - sin2_t)) * face_normal, dir);
                }
            }
            const ChVector3f transmit_tint = likely_glass ? (0.78f * ChVector3f(1.f, 1.f, 1.f) + 0.22f * Clamp01(mat.diffuse))
                                                          : ChVector3f(1.f, 1.f, 1.f);
            const ChVector3f refracted = TraceCameraColor(cache, scene, hit_pos + refract_dir * CH_VKRT_SHADOW_EPS, refract_dir, depth + 1, use_gi);
            color += (1.f - mat.opacity) * Mul(transmit_tint, refracted);
        }

        const float f0_luma = ClampFloat(MaxComponent(f0), 0.f, 1.f);
        float reflect_weight = FresnelSchlickScalar(ndv, f0_luma) * (1.f - mat.roughness) * (1.f - mat.roughness);
        reflect_weight = std::max(reflect_weight, mat.metallic * mat.metallic * (1.f - mat.roughness));
        if (mat.opacity < 1.f)
            reflect_weight = std::max(reflect_weight, FresnelSchlickScalar(ndv, 0.04f) * (1.f - 0.65f * mat.roughness));
        reflect_weight = ClampFloat(reflect_weight, 0.f, 0.85f);
        if (reflect_weight > 1e-4f) {
            const ChVector3d reflect_dir = NormalizeSafe(Reflect(dir, normal), dir);
            const ChVector3f reflected = TraceCameraColor(cache, scene, hit_pos + normal * CH_VKRT_SHADOW_EPS, reflect_dir, depth + 1, use_gi);
            color += reflect_weight * reflected;
        }
    }

    if (use_gi && depth == 0) {
        const ChVector3d gi_dir = NormalizeSafe(normal + ChVector3d(0.31, -0.21, 0.92), normal);
        const float gi_ndl = static_cast<float>(std::max(0.0, normal.Dot(gi_dir)));
        if (gi_ndl > 0.f) {
            const ChVector3f gi = TraceCameraColor(cache, scene, hit_pos + gi_dir * CH_VKRT_SHADOW_EPS, gi_dir, depth + 1, false);
            color += 0.35f * mat.opacity * gi_ndl * Mul(mat.diffuse, gi);
        }
    }

    return color;
}

ChVector3f TraceCameraColor(const ChVulkanRTRenderCache* cache,
                            const std::shared_ptr<ChVulkanRTScene>& scene,
                            const ChVector3d& origin,
                            const ChVector3d& dir,
                            int depth,
                            bool use_gi) {
    if (depth > 6)
        return BackgroundColor(cache, scene, dir);

    const RayHit hit = TraceRay(cache, scene, origin, dir);
    if (!hit.hit)
        return BackgroundColor(cache, scene, dir);

    return Shade(cache, scene, hit, origin, dir, depth, use_gi);
}

float CameraHFOV(const std::shared_ptr<ChVulkanSensor>& sensor) {
#if defined(CHRONO_HAS_VULKAN_RT) && !defined(CHRONO_HAS_OPTIX)
    if (auto camera = std::dynamic_pointer_cast<ChPhysCameraSensor>(sensor))
        return camera->GetHFOV();
#endif
    if (auto camera = std::dynamic_pointer_cast<ChVulkanCameraSensor>(sensor))
        return camera->GetHFOV();
    if (auto camera = std::dynamic_pointer_cast<ChCameraSensor>(sensor))
        return camera->GetHFOV();
    if (auto camera = std::dynamic_pointer_cast<ChDepthCamera>(sensor))
        return camera->GetHFOV();
    if (auto camera = std::dynamic_pointer_cast<ChNormalCamera>(sensor))
        return camera->GetHFOV();
    if (auto camera = std::dynamic_pointer_cast<ChSegmentationCamera>(sensor))
        return camera->GetHFOV();
    return static_cast<float>(CH_VKRT_PI / 3.0);
}

float CameraGamma(const std::shared_ptr<ChVulkanSensor>& sensor) {
#if defined(CHRONO_HAS_VULKAN_RT) && !defined(CHRONO_HAS_OPTIX)
    if (auto camera = std::dynamic_pointer_cast<ChPhysCameraSensor>(sensor))
        return camera->GetGamma();
#endif
    if (auto camera = std::dynamic_pointer_cast<ChVulkanCameraSensor>(sensor))
        return camera->GetGamma();
    if (auto camera = std::dynamic_pointer_cast<ChCameraSensor>(sensor))
        return camera->GetGamma();
    return 1.f;
}

bool CameraUseGI(const std::shared_ptr<ChVulkanSensor>& sensor) {
#if defined(CHRONO_HAS_VULKAN_RT) && !defined(CHRONO_HAS_OPTIX)
    if (auto camera = std::dynamic_pointer_cast<ChPhysCameraSensor>(sensor))
        return camera->GetUseGI();
#endif
    if (auto camera = std::dynamic_pointer_cast<ChCameraSensor>(sensor))
        return camera->GetUseGI();
    return false;
}

CameraLensModelType CameraLensModel(const std::shared_ptr<ChVulkanSensor>& sensor) {
#if defined(CHRONO_HAS_VULKAN_RT) && !defined(CHRONO_HAS_OPTIX)
    if (auto camera = std::dynamic_pointer_cast<ChPhysCameraSensor>(sensor))
        return camera->GetLensModelType();
#endif
    if (auto camera = std::dynamic_pointer_cast<ChCameraSensor>(sensor))
        return camera->GetLensModelType();
    if (auto camera = std::dynamic_pointer_cast<ChDepthCamera>(sensor))
        return camera->GetLensModelType();
    if (auto camera = std::dynamic_pointer_cast<ChNormalCamera>(sensor))
        return camera->GetLensModelType();
    if (auto camera = std::dynamic_pointer_cast<ChSegmentationCamera>(sensor))
        return camera->GetLensModelType();
    return CameraLensModelType::PINHOLE;
}

LensParams CameraLensParameters(const std::shared_ptr<ChVulkanSensor>& sensor) {
#if defined(CHRONO_HAS_VULKAN_RT) && !defined(CHRONO_HAS_OPTIX)
    if (auto camera = std::dynamic_pointer_cast<ChPhysCameraSensor>(sensor))
        return camera->GetLensParameters();
#endif
    if (auto camera = std::dynamic_pointer_cast<ChCameraSensor>(sensor))
        return camera->GetLensParameters();
    if (auto camera = std::dynamic_pointer_cast<ChDepthCamera>(sensor))
        return camera->GetLensParameters();
    if (auto camera = std::dynamic_pointer_cast<ChNormalCamera>(sensor))
        return camera->GetLensParameters();
    if (auto camera = std::dynamic_pointer_cast<ChSegmentationCamera>(sensor))
        return camera->GetLensParameters();
    return LensParams{};
}

unsigned int CameraSampleFactor(const std::shared_ptr<ChVulkanSensor>& sensor) {
#if defined(CHRONO_HAS_VULKAN_RT) && !defined(CHRONO_HAS_OPTIX)
    if (auto camera = std::dynamic_pointer_cast<ChPhysCameraSensor>(sensor))
        return std::max(1u, camera->GetSampleFactor());
#endif
    if (auto camera = std::dynamic_pointer_cast<ChVulkanCameraSensor>(sensor))
        return std::max(1u, camera->GetSampleFactor());
    if (auto camera = std::dynamic_pointer_cast<ChCameraSensor>(sensor))
        return std::max(1u, camera->GetSampleFactor());
    return 1u;
}

float CameraMaxDepth(const std::shared_ptr<ChVulkanSensor>& sensor) {
    if (auto camera = std::dynamic_pointer_cast<ChDepthCamera>(sensor))
        return camera->GetMaxDepth();
    return 1000.f;
}

bool IsLidarPipeline(VulkanPipelineType pipeline) {
    return pipeline == VulkanPipelineType::LIDAR_SINGLE || pipeline == VulkanPipelineType::LIDAR_MULTI;
}

uint32_t OutputWordsPerPixel(VulkanPipelineType pipeline) {
    switch (pipeline) {
        case VulkanPipelineType::CAMERA:
        case VulkanPipelineType::SEGMENTATION:
        case VulkanPipelineType::DEPTH_CAMERA:
            return 1u;
        case VulkanPipelineType::LIDAR_SINGLE:
        case VulkanPipelineType::LIDAR_MULTI:
            return 2u;
        case VulkanPipelineType::NORMAL_CAMERA:
            return 3u;
        case VulkanPipelineType::PHYS_CAMERA:
            return 4u;
        case VulkanPipelineType::RADAR:
            return 8u;
    }
    return 8u;
}

ChVector3d LidarRayDirection(const std::shared_ptr<ChLidarSensor>& lidar,
                             unsigned int x,
                             unsigned int y,
                             unsigned int width,
                             unsigned int height,
                             const ChVector3d& forward,
                             const ChVector3d& left,
                             const ChVector3d& up) {
    const unsigned int sample_radius = std::max(1u, lidar->GetSampleRadius());
    const unsigned int sample_dim = sample_radius * 2u - 1u;

    double theta = 0.0;
    double phi = 0.0;

    if (sample_radius > 1u) {
        const unsigned int beam_w = std::max(1u, width / sample_dim);
        const unsigned int beam_h = std::max(1u, height / sample_dim);
        const unsigned int beam_x = x / sample_dim;
        const unsigned int beam_y = y / sample_dim;

        const double beam_phi = (static_cast<double>(beam_y) / static_cast<double>(std::max(1u, beam_h - 1u))) *
                                    (lidar->GetMaxVertAngle() - lidar->GetMinVertAngle()) +
                                lidar->GetMinVertAngle();
        const double beam_theta = (static_cast<double>(beam_x) / static_cast<double>(std::max(1u, beam_w - 1u))) *
                                      lidar->GetHFOV() -
                                  lidar->GetHFOV() / 2.0;

        const unsigned int local_x = x % sample_dim;
        const unsigned int local_y = y % sample_dim;
        const double frac_x = (static_cast<double>(local_x) + 0.5) / static_cast<double>(sample_dim) * 2.0 - 1.0;
        const double frac_y = (static_cast<double>(local_y) + 0.5) / static_cast<double>(sample_dim) * 2.0 - 1.0;

        double local_theta = 0.0;
        double local_phi = 0.0;
        if (lidar->GetBeamShape() == LidarBeamShape::ELLIPTICAL) {
            local_theta = frac_x * lidar->GetHorizDivAngle() / 2.0;
            local_phi = frac_y * lidar->GetVertDivAngle() / 2.0;
        } else {
            const double angle = std::atan2(frac_y, frac_x);
            const double ring = std::max(std::abs(frac_x), std::abs(frac_y));
            const double axis_x = lidar->GetVertDivAngle() / 2.0 * ring;
            const double axis_y = lidar->GetHorizDivAngle() / 2.0 * ring;
            double radius = 0.0;
            if (axis_x != 0.0 || axis_y != 0.0) {
                const double den = std::sqrt(axis_x * axis_x * std::sin(angle) * std::sin(angle) +
                                             axis_y * axis_y * std::cos(angle) * std::cos(angle));
                radius = den > CH_VKRT_EPS ? (axis_x * axis_y) / den : 0.0;
            }
            local_theta = radius * std::sin(angle);
            local_phi = radius * std::cos(angle);
        }

        theta = beam_theta + local_theta;
        phi = beam_phi + local_phi;
    } else {
        phi = (static_cast<double>(y) / static_cast<double>(std::max(1u, height - 1u))) *
                  (lidar->GetMaxVertAngle() - lidar->GetMinVertAngle()) +
              lidar->GetMinVertAngle();
        theta = (static_cast<double>(x) / static_cast<double>(std::max(1u, width - 1u))) * lidar->GetHFOV() -
                lidar->GetHFOV() / 2.0;
    }

    const double xy_proj = std::cos(phi);
    const double z = std::sin(phi);
    const double y_comp = xy_proj * std::sin(theta);
    const double x_comp = xy_proj * std::cos(theta);
    return NormalizeSafe(forward * x_comp + left * y_comp + up * z, forward);
}

ChVector3d RadarRayDirection(const std::shared_ptr<ChRadarSensor>& radar,
                             unsigned int x,
                             unsigned int y,
                             unsigned int width,
                             unsigned int height,
                             const ChVector3d& forward,
                             const ChVector3d& left,
                             const ChVector3d& up) {
    const double dx = (static_cast<double>(x) + 0.5) / static_cast<double>(width) * 2.0 - 1.0;
    const double dy = (static_cast<double>(y) + 0.5) / static_cast<double>(height) * 2.0 - 1.0;
    const double theta = dx * radar->GetHFOV() / 2.0;
    const double phi = -radar->GetVFOV() / 2.0 + (dy * 0.5 + 0.5) * radar->GetVFOV();
    const double xy_proj = std::cos(phi);
    const double z = std::sin(phi);
    const double y_comp = xy_proj * std::sin(theta);
    const double x_comp = xy_proj * std::cos(theta);
    return NormalizeSafe(forward * x_comp + left * y_comp + up * z, forward);
}

void ApplyOptixLensModel(double& uv_x,
                         double& uv_y,
                         double hfov,
                         CameraLensModelType lens_model,
                         const LensParams& lens_params) {
    if (lens_model == CameraLensModelType::FOV_LENS && (std::abs(uv_x) > 1e-5 || std::abs(uv_y) > 1e-5)) {
        const double focal = 1.0 / std::tan(hfov / 2.0);
        const double x_norm = uv_x / focal;
        const double y_norm = uv_y / focal;
        const double rd = std::sqrt(x_norm * x_norm + y_norm * y_norm);
        if (rd > CH_VKRT_EPS) {
            const double ru = std::tan(rd * hfov) / (2.0 * std::tan(hfov / 2.0));
            uv_x = x_norm * (ru / rd) * focal;
            uv_y = y_norm * (ru / rd) * focal;
        }
    } else if (lens_model == CameraLensModelType::RADIAL) {
        const double recip_focal = std::tan(hfov / 2.0);
        const double x_norm = uv_x * recip_focal;
        const double y_norm = uv_y * recip_focal;
        const float rd2 = static_cast<float>(x_norm * x_norm + y_norm * y_norm);
        const double distortion_ratio = RadialLensFunction(rd2, lens_params);
        uv_x = x_norm * distortion_ratio / recip_focal;
        uv_y = y_norm * distortion_ratio / recip_focal;
    }
}


}  // namespace

namespace {

uint32_t AlignUpU32(uint32_t value, uint32_t alignment) {
    if (alignment == 0)
        return value;
    return (value + alignment - 1u) & ~(alignment - 1u);
}

VkDeviceSize AlignUpVk(VkDeviceSize value, VkDeviceSize alignment) {
    if (alignment == 0)
        return value;
    return (value + alignment - 1u) & ~(alignment - 1u);
}

float BitsToFloat(uint32_t bits) {
    float out = 0.f;
    std::memcpy(&out, &bits, sizeof(float));
    return out;
}

constexpr uint32_t CH_VKRT_INVALID_TEXTURE = 0xffffffffu;

struct ChVulkanRTGpuVertex {
    float pos[4];
    float normal[4];
    float uv[4];       // u, v, has_uv, reserved
    float tangent[4];  // tangent xyz, reserved
};

struct ChVulkanRTGpuTriangle {
    uint32_t i0;
    uint32_t i1;
    uint32_t i2;
    uint32_t material;
};

struct ChVulkanRTGpuMaterial {
    float diffuse[4];   // linear diffuse rgb, scalar opacity
    float specular[4];  // specular rgb, shininess
    float emissive[4];  // emissive rgb, emissive power
    float params[4];    // opacity, roughness, metallic, use_specular_workflow
    uint32_t ids[4];    // class_id, instance_id, reserved, reserved
    float sensor[4];    // lidar_intensity, radar_backscatter, object_id, reserved
    uint32_t texture0[4];  // diffuse, specular, emissive, normal
    uint32_t texture1[4];  // roughness, metallic, opacity, weight
    float tex_scale[4];    // u scale, v scale, reserved, reserved
};

struct ChVulkanRTGpuTexture {
    uint32_t info[4];  // texel offset, width, height, flags (bit 0 = RGB9E5 HDR)
};

struct ChVulkanRTGpuLight {
    float pos_range[4];    // pos xyz, range
    float dir_type[4];     // dir xyz, LightType as float
    float color_atten[4];  // color rgb, attenuation scale
    float params[4];       // angle, angle_falloff_start, angle_atten_rate, area/radius
};

struct ChVulkanRTGpuSceneData {
    float ambient[4];      // rgb, reserved
    float background0[4];  // zenith rgb, reserved
    float background1[4];  // horizon rgb, reserved
    uint32_t counts[4];    // light count, background mode, environment texture id, texture count
};

struct ChVulkanRTGpuPushConstants {
    float origin_max_t[4];
    float forward_hfov[4];
    float right_aspect[4];
    float up_gamma[4];
    float aux0[4];
    float aux1[4];
    uint32_t width;
    uint32_t height;
    uint32_t pipeline;
    uint32_t flags;
};

struct ChVulkanRTGpuFrame {
    std::shared_ptr<ChVulkanSensor> sensor;
    VulkanPipelineType pipeline = VulkanPipelineType::CAMERA;
    unsigned int width = 0;
    unsigned int height = 0;
    ChVector3d origin = ChVector3d(0.0, 0.0, 0.0);
    ChVector3d forward = ChVector3d(1.0, 0.0, 0.0);
    ChVector3d right = ChVector3d(0.0, -1.0, 0.0);
    ChVector3d up = ChVector3d(0.0, 0.0, 1.0);
    float hfov = 0.f;
    float vfov = 0.f;
    float min_vert_angle = 0.f;
    float max_vert_angle = 0.f;
    float clip_near = 0.001f;
    float max_distance = 1000.f;
    float max_depth = 1000.f;
    float gamma = 1.f;
    float aspect = 1.f;
    float tan_half_hfov = 1.f;
    float aux_ray_factor = 1.f;
    bool use_gi = false;
    uint32_t ray_recursions = 1;
    SensorHostRGBA8Buffer* rgba8 = nullptr;
    SensorHostRGBDHalf4Buffer* rgbd = nullptr;
    SensorHostDepthBuffer* depth = nullptr;
    SensorHostNormalBuffer* normal = nullptr;
    SensorHostSemanticBuffer* semantic = nullptr;
    SensorHostDIBuffer* di = nullptr;
    SensorHostRadarBuffer* radar = nullptr;
};

std::string VulkanShaderPath(const std::string& name) {
#ifdef CHRONO_SENSOR_VULKAN_SHADER_DIR
    const std::string base = CHRONO_SENSOR_VULKAN_SHADER_DIR;
    if (!base.empty())
        return base + "/" + name;
#endif
    return name;
}

std::vector<uint32_t> LoadSpirvFile(const std::string& filename) {
    std::ifstream in(filename, std::ios::binary | std::ios::ate);
    if (!in) {
        std::ostringstream out;
        out << "Chrono::Sensor Vulkan RT GPU shader not found: " << filename;
        throw std::runtime_error(out.str());
    }
    const std::streamsize size = in.tellg();
    if (size <= 0)
        throw std::runtime_error("Chrono::Sensor Vulkan RT GPU shader is empty: " + filename);
    if ((size % static_cast<std::streamsize>(sizeof(uint32_t))) != 0)
        throw std::runtime_error("Chrono::Sensor Vulkan RT GPU shader has invalid SPIR-V size: " + filename);
    std::vector<uint32_t> words(static_cast<size_t>(size) / sizeof(uint32_t));
    in.seekg(0, std::ios::beg);
    if (!in.read(reinterpret_cast<char*>(words.data()), size))
        throw std::runtime_error("Chrono::Sensor Vulkan RT GPU shader could not be read: " + filename);
    return words;
}

ChVulkanRTGpuMaterial MakeGpuMaterial(const ChVulkanRTMaterial& mat, float object_id) {
    ChVulkanRTGpuMaterial out{};
    out.diffuse[0] = mat.diffuse.x();
    out.diffuse[1] = mat.diffuse.y();
    out.diffuse[2] = mat.diffuse.z();
    out.diffuse[3] = mat.opacity;
    out.specular[0] = mat.specular.x();
    out.specular[1] = mat.specular.y();
    out.specular[2] = mat.specular.z();
    out.specular[3] = mat.shininess;
    out.emissive[0] = mat.emissive.x();
    out.emissive[1] = mat.emissive.y();
    out.emissive[2] = mat.emissive.z();
    out.emissive[3] = mat.emissive_power;
    out.params[0] = mat.opacity;
    out.params[1] = mat.roughness;
    out.params[2] = mat.metallic;
    out.params[3] = mat.use_specular_workflow ? 1.f : 0.f;
    out.ids[0] = static_cast<uint32_t>(mat.class_id);
    out.ids[1] = static_cast<uint32_t>(mat.instance_id);
    out.ids[2] = 0u;
    out.ids[3] = 0u;
    out.sensor[0] = mat.lidar_intensity;
    out.sensor[1] = mat.radar_backscatter;
    out.sensor[2] = object_id;
    out.sensor[3] = 0.f;
    for (int i = 0; i < 4; ++i) {
        out.texture0[i] = CH_VKRT_INVALID_TEXTURE;
        out.texture1[i] = CH_VKRT_INVALID_TEXTURE;
    }
    out.tex_scale[0] = mat.tex_scale_u;
    out.tex_scale[1] = mat.tex_scale_v;
    out.tex_scale[2] = 0.f;
    out.tex_scale[3] = 0.f;
    return out;
}

ChVulkanRTGpuLight MakeGpuLight(const ChVulkanRTLight& light) {
    ChVulkanRTGpuLight out{};
    out.pos_range[0] = light.pos.x();
    out.pos_range[1] = light.pos.y();
    out.pos_range[2] = light.pos.z();
    out.pos_range[3] = light.range;
    ChVector3f dir = light.dir;
    const float dir_len = dir.Length();
    if (dir_len > 1e-12f)
        dir = dir / dir_len;
    out.dir_type[0] = dir.x();
    out.dir_type[1] = dir.y();
    out.dir_type[2] = dir.z();
    out.dir_type[3] = static_cast<float>(static_cast<int>(light.type));
    out.color_atten[0] = light.color.x();
    out.color_atten[1] = light.color.y();
    out.color_atten[2] = light.color.z();
    out.color_atten[3] = light.const_color ? -1.f : light.atten_scale;
    out.params[0] = light.angle;
    out.params[1] = light.angle_falloff_start;
    out.params[2] = light.angle_atten_rate;
    out.params[3] = light.radius > 0.f ? light.radius : light.area;
    return out;
}

class ScopedVulkanShaderModule {
  public:
    ScopedVulkanShaderModule(std::shared_ptr<ChVulkanRTDevice> device, const std::string& path) : m_device(std::move(device)) {
        const auto code = LoadSpirvFile(path);
        VkShaderModuleCreateInfo create_info = {};
        create_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        create_info.codeSize = code.size() * sizeof(uint32_t);
        create_info.pCode = code.data();
        CH_VULKAN_CHECK(vkCreateShaderModule(m_device->GetDevice(), &create_info, nullptr, &m_module));
    }

    ~ScopedVulkanShaderModule() {
        if (m_module && m_device)
            vkDestroyShaderModule(m_device->GetDevice(), m_module, nullptr);
    }

    VkShaderModule Get() const { return m_module; }

  private:
    std::shared_ptr<ChVulkanRTDevice> m_device;
    VkShaderModule m_module = VK_NULL_HANDLE;
};

}  // namespace

struct ChVulkanRTGpuRenderer {
    explicit ChVulkanRTGpuRenderer(std::shared_ptr<ChVulkanRTDevice> device) : m_device(std::move(device)) {
        if (!m_device || !m_device->GetDevice())
            throw std::runtime_error("Chrono::Sensor Vulkan RT GPU renderer cannot be created without a Vulkan device");
        CreateCommandObjects();
        CreatePipeline();
    }

    ~ChVulkanRTGpuRenderer() { Destroy(); }

    bool Render(const std::shared_ptr<ChVulkanRTScene>& scene, const ChVulkanRTGpuFrame& frame) {
        if (!scene || !m_device || !m_device->GetDevice())
            return false;
        if (frame.width == 0 || frame.height == 0)
            return true;

        if (m_scene_revision != scene->GetRevision()) {
            BuildScene(scene);
            m_scene_revision = scene->GetRevision();
            m_descriptors_dirty = true;
        }

        if (m_vertices.empty() || m_triangles.empty())
            return false;

        EnsureOutput(frame.width, frame.height, frame.pipeline);
        if (m_descriptors_dirty)
            WriteDescriptors();

        ChVulkanRTGpuPushConstants pc{};
        pc.origin_max_t[0] = static_cast<float>(frame.origin.x());
        pc.origin_max_t[1] = static_cast<float>(frame.origin.y());
        pc.origin_max_t[2] = static_cast<float>(frame.origin.z());
        pc.origin_max_t[3] = std::max(1.f, frame.max_distance);
        pc.forward_hfov[0] = static_cast<float>(frame.forward.x());
        pc.forward_hfov[1] = static_cast<float>(frame.forward.y());
        pc.forward_hfov[2] = static_cast<float>(frame.forward.z());
        pc.forward_hfov[3] = frame.hfov;
        pc.right_aspect[0] = static_cast<float>(frame.right.x());
        pc.right_aspect[1] = static_cast<float>(frame.right.y());
        pc.right_aspect[2] = static_cast<float>(frame.right.z());
        pc.right_aspect[3] = frame.aspect;
        pc.up_gamma[0] = static_cast<float>(frame.up.x());
        pc.up_gamma[1] = static_cast<float>(frame.up.y());
        pc.up_gamma[2] = static_cast<float>(frame.up.z());
        pc.up_gamma[3] = frame.gamma;
        pc.aux0[0] = frame.max_depth;
        pc.aux0[1] = frame.clip_near;
        pc.aux0[2] = frame.tan_half_hfov;
        pc.aux0[3] = frame.aux_ray_factor;
        pc.aux1[0] = frame.min_vert_angle;
        pc.aux1[1] = frame.max_vert_angle;
        pc.aux1[2] = frame.vfov;
        pc.aux1[3] = frame.max_distance;
        pc.width = frame.width;
        pc.height = frame.height;
        pc.pipeline = static_cast<uint32_t>(frame.pipeline);
        const uint32_t recursion_count = std::min<uint32_t>(255u, std::max<uint32_t>(1u, frame.ray_recursions));
        pc.flags = (frame.use_gi ? 1u : 0u) | (recursion_count << 8);

        RecordAndSubmitRender(pc, frame.width, frame.height, frame.pipeline);
        CopyOutputToHost(frame);
        return true;
    }

  private:
    void Destroy() {
        if (!m_device || !m_device->GetDevice())
            return;
        VkDevice device = m_device->GetDevice();
        vkDeviceWaitIdle(device);
        if (m_tlas)
            m_device->vkDestroyAccelerationStructureKHR(device, m_tlas, nullptr);
        if (m_blas)
            m_device->vkDestroyAccelerationStructureKHR(device, m_blas, nullptr);
        if (m_pipeline)
            vkDestroyPipeline(device, m_pipeline, nullptr);
        if (m_pipeline_layout)
            vkDestroyPipelineLayout(device, m_pipeline_layout, nullptr);
        if (m_descriptor_pool)
            vkDestroyDescriptorPool(device, m_descriptor_pool, nullptr);
        if (m_descriptor_set_layout)
            vkDestroyDescriptorSetLayout(device, m_descriptor_set_layout, nullptr);
        if (m_command_pool)
            vkDestroyCommandPool(device, m_command_pool, nullptr);
        if (m_fence)
            vkDestroyFence(device, m_fence, nullptr);
        m_tlas = VK_NULL_HANDLE;
        m_blas = VK_NULL_HANDLE;
        m_pipeline = VK_NULL_HANDLE;
        m_pipeline_layout = VK_NULL_HANDLE;
        m_descriptor_pool = VK_NULL_HANDLE;
        m_descriptor_set_layout = VK_NULL_HANDLE;
        m_command_pool = VK_NULL_HANDLE;
        m_fence = VK_NULL_HANDLE;
    }

    void CreateCommandObjects() {
        VkCommandPoolCreateInfo pool_info = {};
        pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        pool_info.queueFamilyIndex = m_device->GetQueueFamilyIndex();
        CH_VULKAN_CHECK(vkCreateCommandPool(m_device->GetDevice(), &pool_info, nullptr, &m_command_pool));

        VkCommandBufferAllocateInfo alloc_info = {};
        alloc_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        alloc_info.commandPool = m_command_pool;
        alloc_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        alloc_info.commandBufferCount = 1;
        CH_VULKAN_CHECK(vkAllocateCommandBuffers(m_device->GetDevice(), &alloc_info, &m_command_buffer));

        VkFenceCreateInfo fence_info = {};
        fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        CH_VULKAN_CHECK(vkCreateFence(m_device->GetDevice(), &fence_info, nullptr, &m_fence));
    }

    void BeginCommands() {
        CH_VULKAN_CHECK(vkResetCommandBuffer(m_command_buffer, 0));
        VkCommandBufferBeginInfo begin_info = {};
        begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        CH_VULKAN_CHECK(vkBeginCommandBuffer(m_command_buffer, &begin_info));
    }

    void EndSubmitWait() {
        CH_VULKAN_CHECK(vkEndCommandBuffer(m_command_buffer));
        CH_VULKAN_CHECK(vkResetFences(m_device->GetDevice(), 1, &m_fence));
        VkSubmitInfo submit_info = {};
        submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submit_info.commandBufferCount = 1;
        submit_info.pCommandBuffers = &m_command_buffer;
        CH_VULKAN_CHECK(vkQueueSubmit(m_device->GetQueue(), 1, &submit_info, m_fence));
        CH_VULKAN_CHECK(vkWaitForFences(m_device->GetDevice(), 1, &m_fence, VK_TRUE, std::numeric_limits<uint64_t>::max()));
    }

    void CreatePipeline() {
        std::array<VkDescriptorSetLayoutBinding, 9> bindings = {};
        bindings[0].binding = 0;
        bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
        bindings[0].descriptorCount = 1;
        bindings[0].stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
        bindings[1].binding = 1;
        bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[1].descriptorCount = 1;
        bindings[1].stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR;
        bindings[2].binding = 2;
        bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[2].descriptorCount = 1;
        bindings[2].stageFlags = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR;
        bindings[3].binding = 3;
        bindings[3].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[3].descriptorCount = 1;
        bindings[3].stageFlags = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR;
        bindings[4].binding = 4;
        bindings[4].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[4].descriptorCount = 1;
        bindings[4].stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
        bindings[5].binding = 5;
        bindings[5].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[5].descriptorCount = 1;
        bindings[5].stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR;
        bindings[6].binding = 6;
        bindings[6].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[6].descriptorCount = 1;
        bindings[6].stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR;
        bindings[7].binding = 7;
        bindings[7].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[7].descriptorCount = 1;
        bindings[7].stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
        bindings[8].binding = 8;
        bindings[8].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[8].descriptorCount = 1;
        bindings[8].stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR;

        VkDescriptorSetLayoutCreateInfo layout_info = {};
        layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        layout_info.bindingCount = static_cast<uint32_t>(bindings.size());
        layout_info.pBindings = bindings.data();
        CH_VULKAN_CHECK(vkCreateDescriptorSetLayout(m_device->GetDevice(), &layout_info, nullptr, &m_descriptor_set_layout));

        VkPushConstantRange push_range = {};
        push_range.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
        push_range.offset = 0;
        push_range.size = sizeof(ChVulkanRTGpuPushConstants);

        VkPipelineLayoutCreateInfo pipeline_layout_info = {};
        pipeline_layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        pipeline_layout_info.setLayoutCount = 1;
        pipeline_layout_info.pSetLayouts = &m_descriptor_set_layout;
        pipeline_layout_info.pushConstantRangeCount = 1;
        pipeline_layout_info.pPushConstantRanges = &push_range;
        CH_VULKAN_CHECK(vkCreatePipelineLayout(m_device->GetDevice(), &pipeline_layout_info, nullptr, &m_pipeline_layout));

        ScopedVulkanShaderModule raygen(m_device, VulkanShaderPath("chrono_sensor_vkrt.rgen.spv"));
        ScopedVulkanShaderModule miss(m_device, VulkanShaderPath("chrono_sensor_vkrt.rmiss.spv"));
        ScopedVulkanShaderModule closest(m_device, VulkanShaderPath("chrono_sensor_vkrt.rchit.spv"));
        ScopedVulkanShaderModule shadow_miss(m_device, VulkanShaderPath("chrono_sensor_vkrt_shadow.rmiss.spv"));
        ScopedVulkanShaderModule shadow_anyhit(m_device, VulkanShaderPath("chrono_sensor_vkrt_shadow.rahit.spv"));

        std::array<VkPipelineShaderStageCreateInfo, 5> stages = {};
        stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[0].stage = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
        stages[0].module = raygen.Get();
        stages[0].pName = "main";
        stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[1].stage = VK_SHADER_STAGE_MISS_BIT_KHR;
        stages[1].module = miss.Get();
        stages[1].pName = "main";
        stages[2].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[2].stage = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;
        stages[2].module = closest.Get();
        stages[2].pName = "main";
        stages[3].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[3].stage = VK_SHADER_STAGE_MISS_BIT_KHR;
        stages[3].module = shadow_miss.Get();
        stages[3].pName = "main";
        stages[4].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[4].stage = VK_SHADER_STAGE_ANY_HIT_BIT_KHR;
        stages[4].module = shadow_anyhit.Get();
        stages[4].pName = "main";

        std::array<VkRayTracingShaderGroupCreateInfoKHR, 5> groups = {};
        groups[0].sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
        groups[0].type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
        groups[0].generalShader = 0;
        groups[0].closestHitShader = VK_SHADER_UNUSED_KHR;
        groups[0].anyHitShader = VK_SHADER_UNUSED_KHR;
        groups[0].intersectionShader = VK_SHADER_UNUSED_KHR;
        groups[1].sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
        groups[1].type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
        groups[1].generalShader = 1;
        groups[1].closestHitShader = VK_SHADER_UNUSED_KHR;
        groups[1].anyHitShader = VK_SHADER_UNUSED_KHR;
        groups[1].intersectionShader = VK_SHADER_UNUSED_KHR;
        groups[2].sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
        groups[2].type = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR;
        groups[2].generalShader = VK_SHADER_UNUSED_KHR;
        groups[2].closestHitShader = 2;
        groups[2].anyHitShader = VK_SHADER_UNUSED_KHR;
        groups[2].intersectionShader = VK_SHADER_UNUSED_KHR;
        groups[3].sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
        groups[3].type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
        groups[3].generalShader = 3;
        groups[3].closestHitShader = VK_SHADER_UNUSED_KHR;
        groups[3].anyHitShader = VK_SHADER_UNUSED_KHR;
        groups[3].intersectionShader = VK_SHADER_UNUSED_KHR;
        groups[4].sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
        groups[4].type = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR;
        groups[4].generalShader = VK_SHADER_UNUSED_KHR;
        groups[4].closestHitShader = VK_SHADER_UNUSED_KHR;
        groups[4].anyHitShader = 4;
        groups[4].intersectionShader = VK_SHADER_UNUSED_KHR;

        VkRayTracingPipelineCreateInfoKHR create_info = {};
        create_info.sType = VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_CREATE_INFO_KHR;
        create_info.stageCount = static_cast<uint32_t>(stages.size());
        create_info.pStages = stages.data();
        create_info.groupCount = static_cast<uint32_t>(groups.size());
        create_info.pGroups = groups.data();
        create_info.maxPipelineRayRecursionDepth = 2;
        create_info.layout = m_pipeline_layout;
        CH_VULKAN_CHECK(m_device->vkCreateRayTracingPipelinesKHR(m_device->GetDevice(), VK_NULL_HANDLE, VK_NULL_HANDLE, 1, &create_info, nullptr, &m_pipeline));

        CreateShaderBindingTable(static_cast<uint32_t>(groups.size()));

        std::array<VkDescriptorPoolSize, 2> pool_sizes = {};
        pool_sizes[0].type = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
        pool_sizes[0].descriptorCount = 1;
        pool_sizes[1].type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        pool_sizes[1].descriptorCount = 8;
        VkDescriptorPoolCreateInfo pool_info = {};
        pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        pool_info.maxSets = 1;
        pool_info.poolSizeCount = static_cast<uint32_t>(pool_sizes.size());
        pool_info.pPoolSizes = pool_sizes.data();
        CH_VULKAN_CHECK(vkCreateDescriptorPool(m_device->GetDevice(), &pool_info, nullptr, &m_descriptor_pool));

        VkDescriptorSetAllocateInfo set_alloc = {};
        set_alloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        set_alloc.descriptorPool = m_descriptor_pool;
        set_alloc.descriptorSetCount = 1;
        set_alloc.pSetLayouts = &m_descriptor_set_layout;
        CH_VULKAN_CHECK(vkAllocateDescriptorSets(m_device->GetDevice(), &set_alloc, &m_descriptor_set));
    }

    void CreateShaderBindingTable(uint32_t group_count) {
        const auto& caps = m_device->GetCapabilities();
        const uint32_t handle_size = caps.shader_group_handle_size;
        const uint32_t handle_alignment = std::max(1u, caps.shader_group_handle_alignment);
        const uint32_t base_alignment = std::max(1u, caps.shader_group_base_alignment);
        const uint32_t handle_stride = AlignUpU32(handle_size, handle_alignment);
        if (group_count < 5)
            throw std::runtime_error("Chrono::Sensor Vulkan RT GPU pipeline expects raygen, camera miss/hit, and shadow miss/any-hit shader groups");

        m_sbt_stride = handle_stride;
        m_sbt_raygen_offset = 0;
        m_sbt_miss_offset = AlignUpVk(m_sbt_raygen_offset + m_sbt_stride, base_alignment);
        m_sbt_hit_offset = AlignUpVk(m_sbt_miss_offset + 2u * m_sbt_stride, base_alignment);
        m_sbt_miss_size = 2u * m_sbt_stride;
        m_sbt_hit_size = 2u * m_sbt_stride;
        const VkDeviceSize sbt_size = AlignUpVk(m_sbt_hit_offset + m_sbt_hit_size, base_alignment);

        std::vector<uint8_t> handles(static_cast<size_t>(handle_size) * group_count);
        CH_VULKAN_CHECK(m_device->vkGetRayTracingShaderGroupHandlesKHR(m_device->GetDevice(), m_pipeline, 0, group_count, handles.size(), handles.data()));

        m_sbt_buffer = std::make_unique<ChVulkanRTBuffer>(m_device,
                                                          sbt_size,
                                                          VK_BUFFER_USAGE_SHADER_BINDING_TABLE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        auto* dst = static_cast<uint8_t*>(m_sbt_buffer->Map());
        std::memset(dst, 0, static_cast<size_t>(sbt_size));
        std::memcpy(dst + static_cast<size_t>(m_sbt_raygen_offset), handles.data() + static_cast<size_t>(0) * handle_size, handle_size);
        std::memcpy(dst + static_cast<size_t>(m_sbt_miss_offset), handles.data() + static_cast<size_t>(1) * handle_size, handle_size);
        std::memcpy(dst + static_cast<size_t>(m_sbt_miss_offset + m_sbt_stride), handles.data() + static_cast<size_t>(3) * handle_size, handle_size);
        std::memcpy(dst + static_cast<size_t>(m_sbt_hit_offset), handles.data() + static_cast<size_t>(2) * handle_size, handle_size);
        std::memcpy(dst + static_cast<size_t>(m_sbt_hit_offset + m_sbt_stride), handles.data() + static_cast<size_t>(4) * handle_size, handle_size);
        m_sbt_buffer->Unmap();
    }

    void EnsureBuffer(std::unique_ptr<ChVulkanRTBuffer>& buffer,
                      VkDeviceSize size,
                      VkBufferUsageFlags usage,
                      VkMemoryPropertyFlags memory_flags) {
        if (size == 0)
            size = 1;
        if (!buffer || buffer->GetSize() < size) {
            buffer = std::make_unique<ChVulkanRTBuffer>(m_device, size, usage, memory_flags);
            m_descriptors_dirty = true;
        }
    }

    template <class T>
    void UploadVector(std::unique_ptr<ChVulkanRTBuffer>& buffer, const std::vector<T>& values, VkBufferUsageFlags usage) {
        const VkDeviceSize copy_size = static_cast<VkDeviceSize>(sizeof(T) * values.size());
        const VkDeviceSize size = std::max<VkDeviceSize>(1, copy_size);

        // These buffers are read by ray-tracing shaders for every hit.  Keeping
        // them HOST_VISIBLE makes the GPU fetch material/triangle/texture data
        // through a slow host aperture on many drivers.  Upload once through a
        // staging buffer and keep the render data device-local.
        EnsureBuffer(buffer,
                     size,
                     usage | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        if (copy_size == 0)
            return;

        auto staging = std::make_unique<ChVulkanRTBuffer>(m_device,
                                                          copy_size,
                                                          VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        std::memcpy(staging->Map(), values.data(), static_cast<size_t>(copy_size));
        staging->Unmap();

        BeginCommands();
        VkBufferCopy copy_region = {};
        copy_region.srcOffset = 0;
        copy_region.dstOffset = 0;
        copy_region.size = copy_size;
        vkCmdCopyBuffer(m_command_buffer, staging->GetBuffer(), buffer->GetBuffer(), 1, &copy_region);

        VkMemoryBarrier barrier = {};
        barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_KHR;
        vkCmdPipelineBarrier(m_command_buffer,
                             VK_PIPELINE_STAGE_TRANSFER_BIT,
                             VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_KHR | VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR,
                             0,
                             1,
                             &barrier,
                             0,
                             nullptr,
                             0,
                             nullptr);
        EndSubmitWait();
    }

    uint32_t RegisterTexture(const std::string& filename) {
        if (filename.empty())
            return CH_VKRT_INVALID_TEXTURE;
        const auto found = m_texture_ids.find(filename);
        if (found != m_texture_ids.end())
            return found->second;

        const auto texture = LoadTextureRGBA(filename);
        if (!texture.Valid())
            return CH_VKRT_INVALID_TEXTURE;

        const uint32_t texture_id = static_cast<uint32_t>(m_textures.size());
        if (texture.has_translucent_alpha)
            m_texture_has_alpha.emplace(filename, true);
        const uint32_t offset = static_cast<uint32_t>(m_texture_pixels.size());
        ChVulkanRTGpuTexture gpu_texture{};
        gpu_texture.info[0] = offset;
        gpu_texture.info[1] = static_cast<uint32_t>(texture.width);
        gpu_texture.info[2] = static_cast<uint32_t>(texture.height);
        gpu_texture.info[3] = texture.hdr ? 1u : 0u;
        m_textures.push_back(gpu_texture);
        m_texture_pixels.reserve(m_texture_pixels.size() + static_cast<size_t>(texture.width) * static_cast<size_t>(texture.height));
        if (texture.hdr) {
            for (size_t i = 0; i + 3 < texture.rgba_float.size(); i += 4)
                m_texture_pixels.push_back(PackRGB9E5(texture.rgba_float[i + 0], texture.rgba_float[i + 1], texture.rgba_float[i + 2]));
        } else {
            for (size_t i = 0; i + 3 < texture.rgba.size(); i += 4) {
                const uint32_t packed = static_cast<uint32_t>(texture.rgba[i + 0]) |
                                        (static_cast<uint32_t>(texture.rgba[i + 1]) << 8) |
                                        (static_cast<uint32_t>(texture.rgba[i + 2]) << 16) |
                                        (static_cast<uint32_t>(texture.rgba[i + 3]) << 24);
                m_texture_pixels.push_back(packed);
            }
        }
        m_texture_ids.emplace(filename, texture_id);
        return texture_id;
    }

    bool TextureHasAlpha(const std::string& filename) const {
        const auto found = m_texture_has_alpha.find(filename);
        return found != m_texture_has_alpha.end() && found->second;
    }

    ChVulkanRTGpuMaterial MakeGpuMaterialForScene(const ChVulkanRTMaterial& mat, float object_id) {
        ChVulkanRTGpuMaterial out = MakeGpuMaterial(mat, object_id);
        out.texture0[0] = RegisterTexture(mat.diffuse_texture);
        out.texture0[1] = RegisterTexture(mat.specular_texture);
        out.texture0[2] = RegisterTexture(mat.emissive_texture);
        out.texture0[3] = RegisterTexture(mat.normal_texture);
        out.texture1[0] = RegisterTexture(mat.roughness_texture);
        out.texture1[1] = RegisterTexture(mat.metallic_texture);
        out.texture1[2] = RegisterTexture(mat.opacity_texture);
        out.texture1[3] = RegisterTexture(mat.weight_texture);

        // Shadow rays can use the OptiX-style terminate-on-first-hit fast path
        // only when no material can transmit light.  Keep diffuse alpha cards,
        // explicit opacity/weight maps, and glass on the slower material-aware
        // shadow path so correctness is preserved for those scenes.
        if (mat.opacity < 0.999f ||
            !mat.opacity_texture.empty() ||
            !mat.weight_texture.empty() ||
            TextureHasAlpha(mat.diffuse_texture)) {
            m_scene_has_transparency = true;
        }
        return out;
    }

    void AddGpuTriangle(const ChVector3d& v0,
                        const ChVector3d& v1,
                        const ChVector3d& v2,
                        const ChVector3d& n0,
                        const ChVector3d& n1,
                        const ChVector3d& n2,
                        const ChVulkanRTMaterial& mat,
                        float object_id,
                        ChVulkanRTTexCoord uv0 = ChVulkanRTTexCoord{},
                        ChVulkanRTTexCoord uv1 = ChVulkanRTTexCoord{},
                        ChVulkanRTTexCoord uv2 = ChVulkanRTTexCoord{},
                        const ChVector3d& tangent = ChVector3d(1.0, 0.0, 0.0),
                        bool has_uv = false) {
        const uint32_t base = static_cast<uint32_t>(m_vertices.size());
        const auto world_tangent = NormalizeSafe(tangent, ChVector3d(1.0, 0.0, 0.0));
        auto make_vertex = [&](const ChVector3d& p, const ChVector3d& n, const ChVulkanRTTexCoord& uv) {
            ChVulkanRTGpuVertex out{};
            out.pos[0] = static_cast<float>(p.x());
            out.pos[1] = static_cast<float>(p.y());
            out.pos[2] = static_cast<float>(p.z());
            out.pos[3] = 1.f;
            const auto nn = NormalizeSafe(n);
            out.normal[0] = static_cast<float>(nn.x());
            out.normal[1] = static_cast<float>(nn.y());
            out.normal[2] = static_cast<float>(nn.z());
            out.normal[3] = 0.f;
            out.uv[0] = uv.u;
            out.uv[1] = uv.v;
            out.uv[2] = has_uv ? 1.f : 0.f;
            out.uv[3] = 0.f;
            out.tangent[0] = static_cast<float>(world_tangent.x());
            out.tangent[1] = static_cast<float>(world_tangent.y());
            out.tangent[2] = static_cast<float>(world_tangent.z());
            out.tangent[3] = 0.f;
            return out;
        };
        m_vertices.push_back(make_vertex(v0, n0, uv0));
        m_vertices.push_back(make_vertex(v1, n1, uv1));
        m_vertices.push_back(make_vertex(v2, n2, uv2));
        const uint32_t mat_id = static_cast<uint32_t>(m_materials.size());
        m_materials.push_back(MakeGpuMaterialForScene(mat, object_id));
        m_indices.push_back(base + 0u);
        m_indices.push_back(base + 1u);
        m_indices.push_back(base + 2u);
        m_triangles.push_back(ChVulkanRTGpuTriangle{base + 0u, base + 1u, base + 2u, mat_id});
    }

    void AddLocalTriangle(const ChVulkanRTPrimitive& primitive,
                          const ChVector3d& a,
                          const ChVector3d& b,
                          const ChVector3d& c,
                          const ChVector3d& normal) {
        const auto p0 = primitive.frame.TransformPointLocalToParent(a);
        const auto p1 = primitive.frame.TransformPointLocalToParent(b);
        const auto p2 = primitive.frame.TransformPointLocalToParent(c);
        const auto n = primitive.frame.TransformDirectionLocalToParent(normal);

        const double sx = std::max(1e-9, std::abs(primitive.scale.x()));
        const double sy = std::max(1e-9, std::abs(primitive.scale.y()));
        const double sz = std::max(1e-9, std::abs(primitive.scale.z()));
        const double ax = std::abs(normal.x());
        const double ay = std::abs(normal.y());
        const double az = std::abs(normal.z());
        auto planar_uv = [&](const ChVector3d& p) {
            ChVulkanRTTexCoord uv;
            if (az >= ax && az >= ay) {
                uv.u = static_cast<float>(p.x() / sx + 0.5);
                uv.v = static_cast<float>(p.y() / sy + 0.5);
            } else if (ay >= ax) {
                uv.u = static_cast<float>(p.x() / sx + 0.5);
                uv.v = static_cast<float>(p.z() / sz + 0.5);
            } else {
                uv.u = static_cast<float>(p.y() / sy + 0.5);
                uv.v = static_cast<float>(p.z() / sz + 0.5);
            }
            return uv;
        };
        const auto tangent = primitive.frame.TransformDirectionLocalToParent(NormalizeSafe(b - a, ChVector3d(1.0, 0.0, 0.0)));
        AddGpuTriangle(p0,
                       p1,
                       p2,
                       n,
                       n,
                       n,
                       primitive.material,
                       primitive.object_id,
                       planar_uv(a),
                       planar_uv(b),
                       planar_uv(c),
                       tangent,
                       true);
    }

    void AddBox(const ChVulkanRTPrimitive& primitive) {
        const double hx = primitive.scale.x() * 0.5;
        const double hy = primitive.scale.y() * 0.5;
        const double hz = primitive.scale.z() * 0.5;
        const ChVector3d p[8] = {
            ChVector3d(-hx, -hy, -hz), ChVector3d(hx, -hy, -hz), ChVector3d(hx, hy, -hz), ChVector3d(-hx, hy, -hz),
            ChVector3d(-hx, -hy, hz),  ChVector3d(hx, -hy, hz),  ChVector3d(hx, hy, hz),  ChVector3d(-hx, hy, hz),
        };
        auto face = [&](int a, int b, int c, int d, const ChVector3d& n) {
            AddLocalTriangle(primitive, p[a], p[b], p[c], n);
            AddLocalTriangle(primitive, p[a], p[c], p[d], n);
        };
        face(0, 1, 2, 3, ChVector3d(0, 0, -1));
        face(4, 7, 6, 5, ChVector3d(0, 0, 1));
        face(0, 4, 5, 1, ChVector3d(0, -1, 0));
        face(1, 5, 6, 2, ChVector3d(1, 0, 0));
        face(2, 6, 7, 3, ChVector3d(0, 1, 0));
        face(3, 7, 4, 0, ChVector3d(-1, 0, 0));
    }

    void AddSphere(const ChVulkanRTPrimitive& primitive) {
        const int stacks = 16;
        const int slices = 32;
        const double r = std::max(1e-6, primitive.scale.x());
        auto local = [&](int iy, int ix) {
            const double v = static_cast<double>(iy) / static_cast<double>(stacks);
            const double phi = -0.5 * CH_VKRT_PI + v * CH_VKRT_PI;
            const double u = static_cast<double>(ix) / static_cast<double>(slices);
            const double theta = u * 2.0 * CH_VKRT_PI;
            const double cp = std::cos(phi);
            return ChVector3d(r * cp * std::cos(theta), r * cp * std::sin(theta), r * std::sin(phi));
        };
        auto uv = [&](int iy, int ix) {
            ChVulkanRTTexCoord out;
            out.u = static_cast<float>(static_cast<double>(ix) / static_cast<double>(slices));
            out.v = static_cast<float>(static_cast<double>(iy) / static_cast<double>(stacks));
            return out;
        };
        auto tangent = [&](int ix) {
            const double theta = (static_cast<double>(ix) / static_cast<double>(slices)) * 2.0 * CH_VKRT_PI;
            return primitive.frame.TransformDirectionLocalToParent(ChVector3d(-std::sin(theta), std::cos(theta), 0.0));
        };
        auto add_sphere_triangle = [&](const ChVector3d& a,
                                       const ChVector3d& b,
                                       const ChVector3d& c,
                                       int iya,
                                       int ixa,
                                       int iyb,
                                       int ixb,
                                       int iyc,
                                       int ixc) {
            const auto p0 = primitive.frame.TransformPointLocalToParent(a);
            const auto p1 = primitive.frame.TransformPointLocalToParent(b);
            const auto p2 = primitive.frame.TransformPointLocalToParent(c);
            AddGpuTriangle(p0,
                           p1,
                           p2,
                           primitive.frame.TransformDirectionLocalToParent(NormalizeSafe(a)),
                           primitive.frame.TransformDirectionLocalToParent(NormalizeSafe(b)),
                           primitive.frame.TransformDirectionLocalToParent(NormalizeSafe(c)),
                           primitive.material,
                           primitive.object_id,
                           uv(iya, ixa),
                           uv(iyb, ixb),
                           uv(iyc, ixc),
                           tangent(ixa),
                           true);
        };
        for (int iy = 0; iy < stacks; ++iy) {
            for (int ix = 0; ix < slices; ++ix) {
                const auto p00 = local(iy, ix);
                const auto p10 = local(iy, ix + 1);
                const auto p01 = local(iy + 1, ix);
                const auto p11 = local(iy + 1, ix + 1);
                if (iy > 0)
                    add_sphere_triangle(p00, p10, p11, iy, ix, iy, ix + 1, iy + 1, ix + 1);
                if (iy + 1 < stacks)
                    add_sphere_triangle(p00, p11, p01, iy, ix, iy + 1, ix + 1, iy + 1, ix);
            }
        }
    }

    void AddCylinder(const ChVulkanRTPrimitive& primitive) {
        const int slices = 32;
        const double r = std::max(1e-6, primitive.scale.x());
        const double hz = std::max(1e-6, primitive.scale.z() * 0.5);
        const ChVector3d top_center(0, 0, hz);
        const ChVector3d bot_center(0, 0, -hz);
        for (int i = 0; i < slices; ++i) {
            const double u0 = static_cast<double>(i) / static_cast<double>(slices);
            const double u1 = static_cast<double>(i + 1) / static_cast<double>(slices);
            const double t0 = u0 * 2.0 * CH_VKRT_PI;
            const double t1 = u1 * 2.0 * CH_VKRT_PI;
            const ChVector3d b0(r * std::cos(t0), r * std::sin(t0), -hz);
            const ChVector3d b1(r * std::cos(t1), r * std::sin(t1), -hz);
            const ChVector3d t0p(r * std::cos(t0), r * std::sin(t0), hz);
            const ChVector3d t1p(r * std::cos(t1), r * std::sin(t1), hz);
            const ChVector3d n0(std::cos(t0), std::sin(t0), 0);
            const ChVector3d n1(std::cos(t1), std::sin(t1), 0);
            const auto tangent = primitive.frame.TransformDirectionLocalToParent(ChVector3d(-std::sin(t0), std::cos(t0), 0.0));
            ChVulkanRTTexCoord uv_b0{static_cast<float>(u0), 0.f};
            ChVulkanRTTexCoord uv_b1{static_cast<float>(u1), 0.f};
            ChVulkanRTTexCoord uv_t0{static_cast<float>(u0), 1.f};
            ChVulkanRTTexCoord uv_t1{static_cast<float>(u1), 1.f};
            AddGpuTriangle(primitive.frame.TransformPointLocalToParent(b0),
                           primitive.frame.TransformPointLocalToParent(b1),
                           primitive.frame.TransformPointLocalToParent(t1p),
                           primitive.frame.TransformDirectionLocalToParent(n0),
                           primitive.frame.TransformDirectionLocalToParent(n1),
                           primitive.frame.TransformDirectionLocalToParent(n1),
                           primitive.material,
                           primitive.object_id,
                           uv_b0,
                           uv_b1,
                           uv_t1,
                           tangent,
                           true);
            AddGpuTriangle(primitive.frame.TransformPointLocalToParent(b0),
                           primitive.frame.TransformPointLocalToParent(t1p),
                           primitive.frame.TransformPointLocalToParent(t0p),
                           primitive.frame.TransformDirectionLocalToParent(n0),
                           primitive.frame.TransformDirectionLocalToParent(n1),
                           primitive.frame.TransformDirectionLocalToParent(n0),
                           primitive.material,
                           primitive.object_id,
                           uv_b0,
                           uv_t1,
                           uv_t0,
                           tangent,
                           true);
            AddLocalTriangle(primitive, top_center, t0p, t1p, ChVector3d(0, 0, 1));
            AddLocalTriangle(primitive, bot_center, b1, b0, ChVector3d(0, 0, -1));
        }
    }

    void AddMesh(const ChVulkanRTPrimitive& primitive) {
        for (const auto& tri : primitive.triangles) {
            const auto p0 = primitive.frame.TransformPointLocalToParent(tri.v0);
            const auto p1 = primitive.frame.TransformPointLocalToParent(tri.v1);
            const auto p2 = primitive.frame.TransformPointLocalToParent(tri.v2);
            const auto n0 = primitive.frame.TransformDirectionLocalToParent(tri.has_vertex_normals ? tri.n0 : tri.normal);
            const auto n1 = primitive.frame.TransformDirectionLocalToParent(tri.has_vertex_normals ? tri.n1 : tri.normal);
            const auto n2 = primitive.frame.TransformDirectionLocalToParent(tri.has_vertex_normals ? tri.n2 : tri.normal);
            const auto tangent = primitive.frame.TransformDirectionLocalToParent(tri.tangent);
            AddGpuTriangle(p0,
                           p1,
                           p2,
                           n0,
                           n1,
                           n2,
                           tri.material,
                           primitive.object_id,
                           tri.uv0,
                           tri.uv1,
                           tri.uv2,
                           tangent,
                           tri.has_uvs);
        }
    }

    void BuildScene(const std::shared_ptr<ChVulkanRTScene>& scene) {
        m_vertices.clear();
        m_indices.clear();
        m_triangles.clear();
        m_materials.clear();
        m_textures.clear();
        m_texture_pixels.clear();
        m_texture_ids.clear();
        m_texture_has_alpha.clear();
        m_lights.clear();
        m_scene_has_transparency = false;
        m_scene_data = ChVulkanRTGpuSceneData{};

        const auto& background = scene->GetBackground();
        const uint32_t env_texture_id = RegisterTexture(background.env_tex);
        const auto ambient = scene->GetAmbientLight();
        m_scene_data.ambient[0] = ambient.x();
        m_scene_data.ambient[1] = ambient.y();
        m_scene_data.ambient[2] = ambient.z();
        m_scene_data.background0[0] = background.color_zenith.x();
        m_scene_data.background0[1] = background.color_zenith.y();
        m_scene_data.background0[2] = background.color_zenith.z();
        m_scene_data.background1[0] = background.color_horizon.x();
        m_scene_data.background1[1] = background.color_horizon.y();
        m_scene_data.background1[2] = background.color_horizon.z();
        m_scene_data.counts[1] = static_cast<uint32_t>(background.mode);
        m_scene_data.counts[2] = env_texture_id;

        for (const auto& light : scene->GetLights())
            m_lights.push_back(MakeGpuLight(light));
        m_scene_data.counts[0] = static_cast<uint32_t>(m_lights.size());

        for (const auto& primitive : scene->GetPrimitives()) {
            switch (primitive.type) {
                case ChVulkanRTPrimitiveType::BOX:
                    AddBox(primitive);
                    break;
                case ChVulkanRTPrimitiveType::SPHERE:
                case ChVulkanRTPrimitiveType::MESH_PROXY:
                    AddSphere(primitive);
                    break;
                case ChVulkanRTPrimitiveType::CYLINDER:
                    AddCylinder(primitive);
                    break;
                case ChVulkanRTPrimitiveType::TRIANGLE_MESH:
                    AddMesh(primitive);
                    break;
            }
        }

        if (m_vertices.empty() || m_triangles.empty())
            return;

        UploadVector(m_vertex_buffer,
                     m_vertices,
                     VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                         VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadVector(m_index_buffer,
                     m_indices,
                     VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT);
        if (m_textures.empty())
            m_textures.push_back(ChVulkanRTGpuTexture{});
        if (m_texture_pixels.empty())
            m_texture_pixels.push_back(0xffffffffu);
        m_scene_data.counts[3] = static_cast<uint32_t>(m_textures.size());

        UploadVector(m_triangle_buffer, m_triangles, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT);
        UploadVector(m_material_buffer, m_materials, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT);
        UploadVector(m_texture_buffer, m_textures, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT);
        UploadVector(m_texture_pixel_buffer, m_texture_pixels, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT);
        UploadVector(m_light_buffer, m_lights, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT);
        m_scene_data.ambient[3] = m_scene_has_transparency ? 1.f : 0.f;
        const std::vector<ChVulkanRTGpuSceneData> scene_upload = {m_scene_data};
        UploadVector(m_scene_buffer, scene_upload, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT);
        BuildAccelerationStructures();
    }

    void BuildAccelerationStructures() {
        VkDevice device = m_device->GetDevice();
        if (m_tlas) {
            m_device->vkDestroyAccelerationStructureKHR(device, m_tlas, nullptr);
            m_tlas = VK_NULL_HANDLE;
        }
        if (m_blas) {
            m_device->vkDestroyAccelerationStructureKHR(device, m_blas, nullptr);
            m_blas = VK_NULL_HANDLE;
        }

        VkAccelerationStructureGeometryTrianglesDataKHR triangles = {};
        triangles.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
        triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
        triangles.vertexData.deviceAddress = m_vertex_buffer->GetDeviceAddress();
        triangles.vertexStride = sizeof(ChVulkanRTGpuVertex);
        triangles.maxVertex = static_cast<uint32_t>(m_vertices.size() - 1);
        triangles.indexType = VK_INDEX_TYPE_UINT32;
        triangles.indexData.deviceAddress = m_index_buffer->GetDeviceAddress();

        VkAccelerationStructureGeometryKHR geometry = {};
        geometry.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
        geometry.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
        // Use opaque AS geometry whenever possible.  Fully opaque scenes then
        // get the fast OptiX-style shadow path (terminate on first hit, skip
        // closest-hit shading).  Scenes with opacity still build non-opaque
        // geometry so a future any-hit shadow path can inspect alpha masks.
        geometry.flags = m_scene_has_transparency ? 0 : VK_GEOMETRY_OPAQUE_BIT_KHR;
        geometry.geometry.triangles = triangles;

        VkAccelerationStructureBuildGeometryInfoKHR build_info = {};
        build_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
        build_info.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
        build_info.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
        build_info.geometryCount = 1;
        build_info.pGeometries = &geometry;
        build_info.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;

        const uint32_t primitive_count = static_cast<uint32_t>(m_triangles.size());
        VkAccelerationStructureBuildSizesInfoKHR build_sizes = {};
        build_sizes.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
        m_device->vkGetAccelerationStructureBuildSizesKHR(device,
                                                          VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
                                                          &build_info,
                                                          &primitive_count,
                                                          &build_sizes);

        m_blas_buffer = std::make_unique<ChVulkanRTBuffer>(m_device,
                                                           build_sizes.accelerationStructureSize,
                                                           VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                                           VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        VkAccelerationStructureCreateInfoKHR as_create = {};
        as_create.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
        as_create.buffer = m_blas_buffer->GetBuffer();
        as_create.size = build_sizes.accelerationStructureSize;
        as_create.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
        CH_VULKAN_CHECK(m_device->vkCreateAccelerationStructureKHR(device, &as_create, nullptr, &m_blas));

        auto scratch = std::make_unique<ChVulkanRTBuffer>(m_device,
                                                          build_sizes.buildScratchSize,
                                                          VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                                          VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        build_info.dstAccelerationStructure = m_blas;
        build_info.scratchData.deviceAddress = scratch->GetDeviceAddress();

        VkAccelerationStructureBuildRangeInfoKHR range = {};
        range.primitiveCount = primitive_count;
        const VkAccelerationStructureBuildRangeInfoKHR* range_ptr = &range;

        BeginCommands();
        m_device->vkCmdBuildAccelerationStructuresKHR(m_command_buffer, 1, &build_info, &range_ptr);
        VkMemoryBarrier barrier = {};
        barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        barrier.srcAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
        barrier.dstAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_KHR;
        vkCmdPipelineBarrier(m_command_buffer,
                             VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR,
                             VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR,
                             0,
                             1,
                             &barrier,
                             0,
                             nullptr,
                             0,
                             nullptr);
        EndSubmitWait();

        BuildTopLevelAccelerationStructure();
        m_descriptors_dirty = true;
    }

    void BuildTopLevelAccelerationStructure() {
        VkDevice device = m_device->GetDevice();
        VkAccelerationStructureDeviceAddressInfoKHR addr_info = {};
        addr_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
        addr_info.accelerationStructure = m_blas;
        const VkDeviceAddress blas_addr = m_device->vkGetAccelerationStructureDeviceAddressKHR(device, &addr_info);

        VkAccelerationStructureInstanceKHR instance = {};
        instance.transform.matrix[0][0] = 1.f;
        instance.transform.matrix[1][1] = 1.f;
        instance.transform.matrix[2][2] = 1.f;
        instance.instanceCustomIndex = 0;
        instance.mask = 0xff;
        instance.instanceShaderBindingTableRecordOffset = 0;
        instance.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
        instance.accelerationStructureReference = blas_addr;

        m_instance_buffer = std::make_unique<ChVulkanRTBuffer>(m_device,
                                                               sizeof(instance),
                                                               VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
                                                                   VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                                               VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        std::memcpy(m_instance_buffer->Map(), &instance, sizeof(instance));
        m_instance_buffer->Unmap();

        VkAccelerationStructureGeometryInstancesDataKHR instances = {};
        instances.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
        instances.arrayOfPointers = VK_FALSE;
        instances.data.deviceAddress = m_instance_buffer->GetDeviceAddress();

        VkAccelerationStructureGeometryKHR geometry = {};
        geometry.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
        geometry.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
        geometry.geometry.instances = instances;

        VkAccelerationStructureBuildGeometryInfoKHR build_info = {};
        build_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
        build_info.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
        build_info.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
        build_info.geometryCount = 1;
        build_info.pGeometries = &geometry;
        build_info.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;

        const uint32_t primitive_count = 1;
        VkAccelerationStructureBuildSizesInfoKHR build_sizes = {};
        build_sizes.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
        m_device->vkGetAccelerationStructureBuildSizesKHR(device,
                                                          VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
                                                          &build_info,
                                                          &primitive_count,
                                                          &build_sizes);

        m_tlas_buffer = std::make_unique<ChVulkanRTBuffer>(m_device,
                                                           build_sizes.accelerationStructureSize,
                                                           VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                                           VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        VkAccelerationStructureCreateInfoKHR as_create = {};
        as_create.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
        as_create.buffer = m_tlas_buffer->GetBuffer();
        as_create.size = build_sizes.accelerationStructureSize;
        as_create.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
        CH_VULKAN_CHECK(m_device->vkCreateAccelerationStructureKHR(device, &as_create, nullptr, &m_tlas));

        auto scratch = std::make_unique<ChVulkanRTBuffer>(m_device,
                                                          build_sizes.buildScratchSize,
                                                          VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                                          VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        build_info.dstAccelerationStructure = m_tlas;
        build_info.scratchData.deviceAddress = scratch->GetDeviceAddress();

        VkAccelerationStructureBuildRangeInfoKHR range = {};
        range.primitiveCount = primitive_count;
        const VkAccelerationStructureBuildRangeInfoKHR* range_ptr = &range;

        BeginCommands();
        m_device->vkCmdBuildAccelerationStructuresKHR(m_command_buffer, 1, &build_info, &range_ptr);
        EndSubmitWait();
    }

    void EnsureOutput(unsigned int width, unsigned int height, VulkanPipelineType pipeline) {
        const VkDeviceSize needed = static_cast<VkDeviceSize>(width) * static_cast<VkDeviceSize>(height) *
                                    static_cast<VkDeviceSize>(OutputWordsPerPixel(pipeline)) * sizeof(uint32_t);

        // Keep shader writes in device-local memory.  Writing the raygen output
        // directly to HOST_VISIBLE memory forces many drivers onto a slow
        // uncached path and makes the renderer look almost serial.  The output
        // is copied once into a host-visible staging buffer after tracing.
        EnsureBuffer(m_output_buffer,
                     needed,
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        EnsureBuffer(m_output_staging_buffer,
                     needed,
                     VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                     VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    }

    void WriteDescriptors() {
        VkWriteDescriptorSetAccelerationStructureKHR as_info = {};
        as_info.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR;
        as_info.accelerationStructureCount = 1;
        as_info.pAccelerationStructures = &m_tlas;

        VkDescriptorBufferInfo material_info = {m_material_buffer->GetBuffer(), 0, m_material_buffer->GetSize()};
        VkDescriptorBufferInfo vertex_info = {m_vertex_buffer->GetBuffer(), 0, m_vertex_buffer->GetSize()};
        VkDescriptorBufferInfo triangle_info = {m_triangle_buffer->GetBuffer(), 0, m_triangle_buffer->GetSize()};
        VkDescriptorBufferInfo output_info = {m_output_buffer->GetBuffer(), 0, m_output_buffer->GetSize()};
        VkDescriptorBufferInfo texture_info = {m_texture_buffer->GetBuffer(), 0, m_texture_buffer->GetSize()};
        VkDescriptorBufferInfo texture_pixel_info = {m_texture_pixel_buffer->GetBuffer(), 0, m_texture_pixel_buffer->GetSize()};
        VkDescriptorBufferInfo light_info = {m_light_buffer->GetBuffer(), 0, m_light_buffer->GetSize()};
        VkDescriptorBufferInfo scene_info = {m_scene_buffer->GetBuffer(), 0, m_scene_buffer->GetSize()};

        std::array<VkWriteDescriptorSet, 9> writes = {};
        writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[0].pNext = &as_info;
        writes[0].dstSet = m_descriptor_set;
        writes[0].dstBinding = 0;
        writes[0].descriptorCount = 1;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
        writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[1].dstSet = m_descriptor_set;
        writes[1].dstBinding = 1;
        writes[1].descriptorCount = 1;
        writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[1].pBufferInfo = &material_info;
        writes[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[2].dstSet = m_descriptor_set;
        writes[2].dstBinding = 2;
        writes[2].descriptorCount = 1;
        writes[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[2].pBufferInfo = &vertex_info;
        writes[3].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[3].dstSet = m_descriptor_set;
        writes[3].dstBinding = 3;
        writes[3].descriptorCount = 1;
        writes[3].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[3].pBufferInfo = &triangle_info;
        writes[4].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[4].dstSet = m_descriptor_set;
        writes[4].dstBinding = 4;
        writes[4].descriptorCount = 1;
        writes[4].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[4].pBufferInfo = &output_info;
        writes[5].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[5].dstSet = m_descriptor_set;
        writes[5].dstBinding = 5;
        writes[5].descriptorCount = 1;
        writes[5].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[5].pBufferInfo = &texture_info;
        writes[6].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[6].dstSet = m_descriptor_set;
        writes[6].dstBinding = 6;
        writes[6].descriptorCount = 1;
        writes[6].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[6].pBufferInfo = &texture_pixel_info;
        writes[7].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[7].dstSet = m_descriptor_set;
        writes[7].dstBinding = 7;
        writes[7].descriptorCount = 1;
        writes[7].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[7].pBufferInfo = &light_info;
        writes[8].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[8].dstSet = m_descriptor_set;
        writes[8].dstBinding = 8;
        writes[8].descriptorCount = 1;
        writes[8].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[8].pBufferInfo = &scene_info;
        vkUpdateDescriptorSets(m_device->GetDevice(), static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
        m_descriptors_dirty = false;
    }

    void RecordAndSubmitRender(const ChVulkanRTGpuPushConstants& pc, unsigned int width, unsigned int height, VulkanPipelineType pipeline) {
        BeginCommands();
        vkCmdBindPipeline(m_command_buffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, m_pipeline);
        vkCmdBindDescriptorSets(m_command_buffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, m_pipeline_layout, 0, 1, &m_descriptor_set, 0, nullptr);
        vkCmdPushConstants(m_command_buffer,
                           m_pipeline_layout,
                           VK_SHADER_STAGE_RAYGEN_BIT_KHR,
                           0,
                           sizeof(ChVulkanRTGpuPushConstants),
                           &pc);

        const VkDeviceAddress sbt_addr = m_sbt_buffer->GetDeviceAddress();
        VkStridedDeviceAddressRegionKHR raygen = {sbt_addr + m_sbt_raygen_offset, m_sbt_stride, m_sbt_stride};
        VkStridedDeviceAddressRegionKHR miss = {sbt_addr + m_sbt_miss_offset, m_sbt_stride, m_sbt_miss_size};
        VkStridedDeviceAddressRegionKHR hit = {sbt_addr + m_sbt_hit_offset, m_sbt_stride, m_sbt_hit_size};
        VkStridedDeviceAddressRegionKHR callable = {};
        m_device->vkCmdTraceRaysKHR(m_command_buffer, &raygen, &miss, &hit, &callable, width, height, 1);

        const VkDeviceSize copy_size = static_cast<VkDeviceSize>(width) * static_cast<VkDeviceSize>(height) *
                                        static_cast<VkDeviceSize>(OutputWordsPerPixel(pipeline)) * sizeof(uint32_t);

        VkMemoryBarrier shader_to_copy = {};
        shader_to_copy.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        shader_to_copy.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        shader_to_copy.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        vkCmdPipelineBarrier(m_command_buffer,
                             VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_KHR,
                             VK_PIPELINE_STAGE_TRANSFER_BIT,
                             0,
                             1,
                             &shader_to_copy,
                             0,
                             nullptr,
                             0,
                             nullptr);

        VkBufferCopy copy_region = {};
        copy_region.srcOffset = 0;
        copy_region.dstOffset = 0;
        copy_region.size = copy_size;
        vkCmdCopyBuffer(m_command_buffer, m_output_buffer->GetBuffer(), m_output_staging_buffer->GetBuffer(), 1, &copy_region);

        VkMemoryBarrier copy_to_host = {};
        copy_to_host.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        copy_to_host.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        copy_to_host.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        vkCmdPipelineBarrier(m_command_buffer,
                             VK_PIPELINE_STAGE_TRANSFER_BIT,
                             VK_PIPELINE_STAGE_HOST_BIT,
                             0,
                             1,
                             &copy_to_host,
                             0,
                             nullptr,
                             0,
                             nullptr);
        EndSubmitWait();
    }

    void CopyOutputToHost(const ChVulkanRTGpuFrame& frame) {
        const uint32_t* words = static_cast<const uint32_t*>(m_output_staging_buffer->Map());
        const size_t count = static_cast<size_t>(frame.width) * static_cast<size_t>(frame.height);

        // Keep the readback layout identical to the host-facing Sensor buffers
        // whenever the format is a plain POD block.  This removes millions of
        // scalar CPU unpack/conversion stores per frame and mirrors the OptiX
        // path, where device kernels write the final buffer format instead of a
        // generic debug word stream that the host repacks pixel-by-pixel.
        if ((frame.pipeline == VulkanPipelineType::LIDAR_SINGLE || frame.pipeline == VulkanPipelineType::LIDAR_MULTI) && frame.di && frame.di->Buffer) {
            static_assert(sizeof(PixelDI) == 2 * sizeof(uint32_t), "Vulkan DI readback expects two 32-bit words");
            std::memcpy(frame.di->Buffer.get(), words, count * sizeof(PixelDI));
        } else if (frame.pipeline == VulkanPipelineType::RADAR && frame.radar && frame.radar->Buffer) {
            static_assert(sizeof(RadarReturn) == 8 * sizeof(uint32_t), "Vulkan radar readback expects eight 32-bit words");
            std::memcpy(frame.radar->Buffer.get(), words, count * sizeof(RadarReturn));
        } else if (frame.pipeline == VulkanPipelineType::DEPTH_CAMERA && frame.depth && frame.depth->Buffer) {
            static_assert(sizeof(PixelDepth) == sizeof(uint32_t), "Vulkan depth readback expects one 32-bit word");
            std::memcpy(frame.depth->Buffer.get(), words, count * sizeof(PixelDepth));
        } else if (frame.pipeline == VulkanPipelineType::NORMAL_CAMERA && frame.normal && frame.normal->Buffer) {
            static_assert(sizeof(PixelNormal) == 3 * sizeof(uint32_t), "Vulkan normal readback expects three 32-bit words");
            std::memcpy(frame.normal->Buffer.get(), words, count * sizeof(PixelNormal));
        } else if (frame.pipeline == VulkanPipelineType::SEGMENTATION && frame.semantic && frame.semantic->Buffer) {
            static_assert(sizeof(PixelSemantic) == sizeof(uint32_t), "Vulkan semantic readback expects one 32-bit word");
            std::memcpy(frame.semantic->Buffer.get(), words, count * sizeof(PixelSemantic));
        } else if (frame.pipeline == VulkanPipelineType::PHYS_CAMERA && frame.rgbd && frame.rgbd->Buffer) {
            const size_t stride = static_cast<size_t>(OutputWordsPerPixel(frame.pipeline));
            for (size_t i = 0; i < count; ++i) {
                const uint32_t* px = words + i * stride;
                frame.rgbd->Buffer[i].R = BitsToFloat(px[0]);
                frame.rgbd->Buffer[i].G = BitsToFloat(px[1]);
                frame.rgbd->Buffer[i].B = BitsToFloat(px[2]);
                frame.rgbd->Buffer[i].D = BitsToFloat(px[3]);
            }
        } else if (frame.rgba8 && frame.rgba8->Buffer) {
            static_assert(sizeof(PixelRGBA8) == sizeof(uint32_t), "Vulkan RGBA8 readback expects one 32-bit word");
            std::memcpy(frame.rgba8->Buffer.get(), words, count * sizeof(PixelRGBA8));
        }
        // Keep the staging buffer persistently mapped; ChVulkanRTBuffer will
        // unmap it when it is resized or destroyed.
    }

    std::shared_ptr<ChVulkanRTDevice> m_device;
    uint64_t m_scene_revision = 0;
    bool m_descriptors_dirty = true;

    std::vector<ChVulkanRTGpuVertex> m_vertices;
    std::vector<uint32_t> m_indices;
    std::vector<ChVulkanRTGpuTriangle> m_triangles;
    std::vector<ChVulkanRTGpuMaterial> m_materials;
    std::vector<ChVulkanRTGpuTexture> m_textures;
    std::vector<uint32_t> m_texture_pixels;
    std::vector<ChVulkanRTGpuLight> m_lights;
    std::unordered_map<std::string, uint32_t> m_texture_ids;
    std::unordered_map<std::string, bool> m_texture_has_alpha;
    bool m_scene_has_transparency = false;
    ChVulkanRTGpuSceneData m_scene_data{};

    std::unique_ptr<ChVulkanRTBuffer> m_vertex_buffer;
    std::unique_ptr<ChVulkanRTBuffer> m_index_buffer;
    std::unique_ptr<ChVulkanRTBuffer> m_triangle_buffer;
    std::unique_ptr<ChVulkanRTBuffer> m_material_buffer;
    std::unique_ptr<ChVulkanRTBuffer> m_texture_buffer;
    std::unique_ptr<ChVulkanRTBuffer> m_texture_pixel_buffer;
    std::unique_ptr<ChVulkanRTBuffer> m_light_buffer;
    std::unique_ptr<ChVulkanRTBuffer> m_scene_buffer;
    std::unique_ptr<ChVulkanRTBuffer> m_output_buffer;
    std::unique_ptr<ChVulkanRTBuffer> m_output_staging_buffer;
    std::unique_ptr<ChVulkanRTBuffer> m_blas_buffer;
    std::unique_ptr<ChVulkanRTBuffer> m_tlas_buffer;
    std::unique_ptr<ChVulkanRTBuffer> m_instance_buffer;
    std::unique_ptr<ChVulkanRTBuffer> m_sbt_buffer;

    VkAccelerationStructureKHR m_blas = VK_NULL_HANDLE;
    VkAccelerationStructureKHR m_tlas = VK_NULL_HANDLE;
    VkCommandPool m_command_pool = VK_NULL_HANDLE;
    VkCommandBuffer m_command_buffer = VK_NULL_HANDLE;
    VkFence m_fence = VK_NULL_HANDLE;
    VkDescriptorSetLayout m_descriptor_set_layout = VK_NULL_HANDLE;
    VkDescriptorPool m_descriptor_pool = VK_NULL_HANDLE;
    VkDescriptorSet m_descriptor_set = VK_NULL_HANDLE;
    VkPipelineLayout m_pipeline_layout = VK_NULL_HANDLE;
    VkPipeline m_pipeline = VK_NULL_HANDLE;
    VkDeviceSize m_sbt_stride = 0;
    VkDeviceSize m_sbt_raygen_offset = 0;
    VkDeviceSize m_sbt_miss_offset = 0;
    VkDeviceSize m_sbt_hit_offset = 0;
    VkDeviceSize m_sbt_miss_size = 0;
    VkDeviceSize m_sbt_hit_size = 0;
};

struct ChVulkanRTGpuRendererCacheKey {
    VkDevice device = VK_NULL_HANDLE;
    const ChVulkanRTScene* scene = nullptr;

    bool operator==(const ChVulkanRTGpuRendererCacheKey& other) const {
        return device == other.device && scene == other.scene;
    }
};

struct ChVulkanRTGpuRendererCacheKeyHash {
    size_t operator()(const ChVulkanRTGpuRendererCacheKey& key) const {
        const auto a = std::hash<uintptr_t>{}(reinterpret_cast<uintptr_t>(key.device));
        const auto b = std::hash<uintptr_t>{}(reinterpret_cast<uintptr_t>(key.scene));
        return a ^ (b + 0x9e3779b97f4a7c15ull + (a << 6) + (a >> 2));
    }
};

std::shared_ptr<ChVulkanRTGpuRenderer> GetSharedVulkanRTGpuRenderer(const std::shared_ptr<ChVulkanRTDevice>& device,
                                                                    const std::shared_ptr<ChVulkanRTScene>& scene) {
    if (!device || !device->GetDevice())
        throw std::runtime_error("Chrono::Sensor Vulkan RT GPU renderer cannot be shared without a Vulkan device");

    // OptiX owns one device context/pipeline/AS cache per render group.  Do the
    // same here: filters for camera/depth/normal/segmentation sensors sharing a
    // Chrono scene must not each rebuild BLAS/TLAS, upload the same materials,
    // or create duplicate SBT/pipeline state.  The output buffer remains inside
    // the renderer and is reused sequentially by the filter graph.
    static std::mutex cache_mutex;
    static std::unordered_map<ChVulkanRTGpuRendererCacheKey,
                              std::weak_ptr<ChVulkanRTGpuRenderer>,
                              ChVulkanRTGpuRendererCacheKeyHash> cache;

    const ChVulkanRTGpuRendererCacheKey key{device->GetDevice(), scene.get()};
    std::lock_guard<std::mutex> lock(cache_mutex);
    auto it = cache.find(key);
    if (it != cache.end()) {
        if (auto existing = it->second.lock())
            return existing;
    }

    auto created = std::make_shared<ChVulkanRTGpuRenderer>(device);
    cache[key] = created;
    return created;
}

ChFilterVulkanRTRender::ChFilterVulkanRTRender(std::shared_ptr<ChVulkanRTDevice> device,
                                               std::shared_ptr<ChVulkanRTScene> scene)
    : ChFilter("VulkanRTRenderer"), m_device(std::move(device)), m_scene(std::move(scene)) {}

ChFilterVulkanRTRender::~ChFilterVulkanRTRender() = default;

void ChFilterVulkanRTRender::Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) {
    auto vulkan_sensor = std::dynamic_pointer_cast<ChVulkanSensor>(pSensor);
    if (!vulkan_sensor) {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
        return;
    }

    m_sensor = vulkan_sensor;
    const unsigned int width = vulkan_sensor->GetWidth();
    const unsigned int height = vulkan_sensor->GetHeight();
    const size_t count = static_cast<size_t>(width) * static_cast<size_t>(height);

    switch (vulkan_sensor->GetPipelineType()) {
        case VulkanPipelineType::LIDAR_SINGLE:
        case VulkanPipelineType::LIDAR_MULTI:
            m_buffer_di = chrono_types::make_shared<SensorHostDIBuffer>();
            m_buffer_di->Width = width;
            m_buffer_di->Height = height;
            m_buffer_di->Buffer = std::shared_ptr<PixelDI[]>(new PixelDI[count]);
            m_buffer_di->Dual_return = false;
            m_buffer_di->Beam_return_count = static_cast<unsigned int>(count);
            bufferInOut = m_buffer_di;
            break;
        case VulkanPipelineType::RADAR:
            m_buffer_radar = chrono_types::make_shared<SensorHostRadarBuffer>();
            m_buffer_radar->Width = width;
            m_buffer_radar->Height = height;
            m_buffer_radar->Buffer = std::shared_ptr<RadarReturn[]>(new RadarReturn[count]);
            m_buffer_radar->Beam_return_count = static_cast<int>(count);
            bufferInOut = m_buffer_radar;
            break;
        case VulkanPipelineType::DEPTH_CAMERA:
            m_buffer_depth = chrono_types::make_shared<SensorHostDepthBuffer>();
            m_buffer_depth->Width = width;
            m_buffer_depth->Height = height;
            m_buffer_depth->Buffer = std::shared_ptr<PixelDepth[]>(new PixelDepth[count]);
            bufferInOut = m_buffer_depth;
            break;
        case VulkanPipelineType::NORMAL_CAMERA:
            m_buffer_normal = chrono_types::make_shared<SensorHostNormalBuffer>();
            m_buffer_normal->Width = width;
            m_buffer_normal->Height = height;
            m_buffer_normal->Buffer = std::shared_ptr<PixelNormal[]>(new PixelNormal[count]);
            bufferInOut = m_buffer_normal;
            break;
        case VulkanPipelineType::SEGMENTATION:
            m_buffer_semantic = chrono_types::make_shared<SensorHostSemanticBuffer>();
            m_buffer_semantic->Width = width;
            m_buffer_semantic->Height = height;
            m_buffer_semantic->Buffer = std::shared_ptr<PixelSemantic[]>(new PixelSemantic[count]);
            bufferInOut = m_buffer_semantic;
            break;
        case VulkanPipelineType::PHYS_CAMERA:
            m_buffer_rgbd = chrono_types::make_shared<SensorHostRGBDHalf4Buffer>();
            m_buffer_rgbd->Width = width;
            m_buffer_rgbd->Height = height;
            m_buffer_rgbd->Buffer = std::shared_ptr<PixelRGBDHalf4[]>(new PixelRGBDHalf4[count]);
            bufferInOut = m_buffer_rgbd;
            break;
        case VulkanPipelineType::CAMERA:
        default:
            m_buffer_rgba8 = chrono_types::make_shared<SensorHostRGBA8Buffer>();
            m_buffer_rgba8->Width = width;
            m_buffer_rgba8->Height = height;
            m_buffer_rgba8->Buffer = std::shared_ptr<PixelRGBA8[]>(new PixelRGBA8[count]);
            bufferInOut = m_buffer_rgba8;
            break;
    }
}

void ChFilterVulkanRTRender::Apply() {
    auto sensor = m_sensor.lock();
    if (!sensor)
        return;

    const unsigned int width = sensor->GetWidth();
    const unsigned int height = sensor->GetHeight();
    if (width == 0 || height == 0)
        return;

    const size_t count = static_cast<size_t>(width) * static_cast<size_t>(height);
    const auto pipeline = sensor->GetPipelineType();

    if (IsLidarPipeline(pipeline)) {
        if (!m_buffer_di)
            m_buffer_di = chrono_types::make_shared<SensorHostDIBuffer>();
        if (!m_buffer_di->Buffer || m_buffer_di->Width != width || m_buffer_di->Height != height)
            m_buffer_di->Buffer = std::shared_ptr<PixelDI[]>(new PixelDI[count]);
        m_buffer_di->Width = width;
        m_buffer_di->Height = height;
        m_buffer_di->TimeStamp = m_time_stamp;
        m_buffer_di->LaunchedCount = sensor->GetNumLaunches();
        m_buffer_di->Dual_return = false;
        m_buffer_di->Beam_return_count = static_cast<unsigned int>(count);
    } else if (pipeline == VulkanPipelineType::RADAR) {
        if (!m_buffer_radar)
            m_buffer_radar = chrono_types::make_shared<SensorHostRadarBuffer>();
        if (!m_buffer_radar->Buffer || m_buffer_radar->Width != width || m_buffer_radar->Height != height)
            m_buffer_radar->Buffer = std::shared_ptr<RadarReturn[]>(new RadarReturn[count]);
        m_buffer_radar->Width = width;
        m_buffer_radar->Height = height;
        m_buffer_radar->TimeStamp = m_time_stamp;
        m_buffer_radar->LaunchedCount = sensor->GetNumLaunches();
        m_buffer_radar->Beam_return_count = static_cast<int>(count);
        m_buffer_radar->invalid_returns = 0;
        m_buffer_radar->Num_clusters = 0;
    } else if (pipeline == VulkanPipelineType::DEPTH_CAMERA) {
        if (!m_buffer_depth)
            m_buffer_depth = chrono_types::make_shared<SensorHostDepthBuffer>();
        if (!m_buffer_depth->Buffer || m_buffer_depth->Width != width || m_buffer_depth->Height != height)
            m_buffer_depth->Buffer = std::shared_ptr<PixelDepth[]>(new PixelDepth[count]);
        m_buffer_depth->Width = width;
        m_buffer_depth->Height = height;
        m_buffer_depth->TimeStamp = m_time_stamp;
        m_buffer_depth->LaunchedCount = sensor->GetNumLaunches();
    } else if (pipeline == VulkanPipelineType::NORMAL_CAMERA) {
        if (!m_buffer_normal)
            m_buffer_normal = chrono_types::make_shared<SensorHostNormalBuffer>();
        if (!m_buffer_normal->Buffer || m_buffer_normal->Width != width || m_buffer_normal->Height != height)
            m_buffer_normal->Buffer = std::shared_ptr<PixelNormal[]>(new PixelNormal[count]);
        m_buffer_normal->Width = width;
        m_buffer_normal->Height = height;
        m_buffer_normal->TimeStamp = m_time_stamp;
        m_buffer_normal->LaunchedCount = sensor->GetNumLaunches();
    } else if (pipeline == VulkanPipelineType::SEGMENTATION) {
        if (!m_buffer_semantic)
            m_buffer_semantic = chrono_types::make_shared<SensorHostSemanticBuffer>();
        if (!m_buffer_semantic->Buffer || m_buffer_semantic->Width != width || m_buffer_semantic->Height != height)
            m_buffer_semantic->Buffer = std::shared_ptr<PixelSemantic[]>(new PixelSemantic[count]);
        m_buffer_semantic->Width = width;
        m_buffer_semantic->Height = height;
        m_buffer_semantic->TimeStamp = m_time_stamp;
        m_buffer_semantic->LaunchedCount = sensor->GetNumLaunches();
    } else if (pipeline == VulkanPipelineType::PHYS_CAMERA) {
        if (!m_buffer_rgbd)
            m_buffer_rgbd = chrono_types::make_shared<SensorHostRGBDHalf4Buffer>();
        if (!m_buffer_rgbd->Buffer || m_buffer_rgbd->Width != width || m_buffer_rgbd->Height != height)
            m_buffer_rgbd->Buffer = std::shared_ptr<PixelRGBDHalf4[]>(new PixelRGBDHalf4[count]);
        m_buffer_rgbd->Width = width;
        m_buffer_rgbd->Height = height;
        m_buffer_rgbd->TimeStamp = m_time_stamp;
        m_buffer_rgbd->LaunchedCount = sensor->GetNumLaunches();
    } else {
        if (!m_buffer_rgba8)
            m_buffer_rgba8 = chrono_types::make_shared<SensorHostRGBA8Buffer>();
        if (!m_buffer_rgba8->Buffer || m_buffer_rgba8->Width != width || m_buffer_rgba8->Height != height)
            m_buffer_rgba8->Buffer = std::shared_ptr<PixelRGBA8[]>(new PixelRGBA8[count]);
        m_buffer_rgba8->Width = width;
        m_buffer_rgba8->Height = height;
        m_buffer_rgba8->TimeStamp = m_time_stamp;
        m_buffer_rgba8->LaunchedCount = sensor->GetNumLaunches();
    }

    ChFrame<double> camera_frame = sensor->GetOffsetPose();
    if (auto body = sensor->GetParent()) {
        // Match ChOptixEngine::UpdateSensorTransforms(): OptiX packs the
        // sensor parent frame with GetVisualModelFrame(), then applies the
        // sensor offset. This keeps Vulkan RT camera rays in the same frame
        // convention as the OptiX raygen path.
        camera_frame = body->GetVisualModelFrame() * sensor->GetOffsetPose();
    }

    const ChVector3d origin = camera_frame.GetPos();
    const ChVector3d forward = NormalizeSafe(camera_frame.TransformDirectionLocalToParent(ChVector3d(1.0, 0.0, 0.0)), ChVector3d(1.0, 0.0, 0.0));
    const ChVector3d right = NormalizeSafe(camera_frame.TransformDirectionLocalToParent(ChVector3d(0.0, -1.0, 0.0)), ChVector3d(0.0, -1.0, 0.0));
    const ChVector3d up = NormalizeSafe(camera_frame.TransformDirectionLocalToParent(ChVector3d(0.0, 0.0, 1.0)), ChVector3d(0.0, 0.0, 1.0));

    const ChVector3d left = NormalizeSafe(camera_frame.TransformDirectionLocalToParent(ChVector3d(0.0, 1.0, 0.0)), ChVector3d(0.0, 1.0, 0.0));

#ifdef CHRONO_SENSOR_VULKAN_RT_GPU_ENABLED
    if (m_device && m_device->GetDevice()) {
        if (!m_gpu_renderer)
            m_gpu_renderer = GetSharedVulkanRTGpuRenderer(m_device, m_scene);

        ChVulkanRTGpuFrame gpu_frame;
        gpu_frame.sensor = sensor;
        gpu_frame.pipeline = pipeline;
        gpu_frame.width = width;
        gpu_frame.height = height;
        gpu_frame.origin = origin;
        gpu_frame.forward = forward;
        gpu_frame.right = right;
        gpu_frame.up = up;
        gpu_frame.aspect = static_cast<float>(static_cast<double>(width) / static_cast<double>(height));
        gpu_frame.hfov = CameraHFOV(sensor);
        gpu_frame.gamma = CameraGamma(sensor);
        gpu_frame.use_gi = CameraUseGI(sensor);
        gpu_frame.ray_recursions = static_cast<uint32_t>(std::max(1, m_ray_recursions));
        gpu_frame.max_depth = CameraMaxDepth(sensor);
        gpu_frame.max_distance = gpu_frame.max_depth;
        gpu_frame.clip_near = 0.001f;
        gpu_frame.tan_half_hfov = static_cast<float>(std::tan(0.5 * static_cast<double>(gpu_frame.hfov)));
        gpu_frame.aux_ray_factor = static_cast<float>(static_cast<double>(gpu_frame.hfov) / CH_VKRT_PI * 2.0);
        gpu_frame.rgba8 = m_buffer_rgba8.get();
        gpu_frame.rgbd = m_buffer_rgbd.get();
        gpu_frame.depth = m_buffer_depth.get();
        gpu_frame.normal = m_buffer_normal.get();
        gpu_frame.semantic = m_buffer_semantic.get();
        gpu_frame.di = m_buffer_di.get();
        gpu_frame.radar = m_buffer_radar.get();

        if (auto lidar = std::dynamic_pointer_cast<ChLidarSensor>(sensor)) {
            gpu_frame.hfov = static_cast<float>(lidar->GetHFOV());
            gpu_frame.min_vert_angle = static_cast<float>(lidar->GetMinVertAngle());
            gpu_frame.max_vert_angle = static_cast<float>(lidar->GetMaxVertAngle());
            gpu_frame.max_distance = static_cast<float>(lidar->GetMaxDistance());
            gpu_frame.clip_near = static_cast<float>(lidar->GetClipNear());
        }

        if (auto radar = std::dynamic_pointer_cast<ChRadarSensor>(sensor)) {
            gpu_frame.hfov = static_cast<float>(radar->GetHFOV());
            gpu_frame.vfov = static_cast<float>(radar->GetVFOV());
            gpu_frame.max_distance = static_cast<float>(radar->GetMaxDistance());
            gpu_frame.clip_near = static_cast<float>(radar->GetClipNear());
        }

        if (m_gpu_renderer->Render(m_scene, gpu_frame))
            return;

        throw std::runtime_error("Chrono::Sensor Vulkan RT GPU renderer had a Vulkan device but no renderable GPU scene; CPU fallback is only used when no Vulkan RT GPU is available");
    }
#endif

    // CPU fallback is only reached when no Vulkan RT device exists.  Do not build
    // the host BVH/cache on the GPU path; doing so was a large serial cost before
    // every Vulkan render and masked ray-tracing parallelism.
    if (!m_render_cache)
        m_render_cache = std::make_unique<ChVulkanRTRenderCache>();
    if (!m_scene || m_render_cache->scene != m_scene.get() || m_render_cache->scene_revision != m_scene->GetRevision())
        BuildRenderCache(*m_render_cache, m_scene);

    auto parallel_for_pixels = [&](const std::function<void(unsigned int, unsigned int)>& fn) {
        const unsigned int hw_threads = std::max(1u, std::thread::hardware_concurrency());
        const unsigned int thread_count = std::min<unsigned int>(height, hw_threads);
        if (thread_count <= 1 || height < 32) {
            for (unsigned int yy = 0; yy < height; ++yy) {
                for (unsigned int xx = 0; xx < width; ++xx)
                    fn(xx, yy);
            }
            return;
        }

        std::vector<std::thread> workers;
        workers.reserve(thread_count);
        for (unsigned int t = 0; t < thread_count; ++t) {
            const unsigned int y_begin = (height * t) / thread_count;
            const unsigned int y_end = (height * (t + 1)) / thread_count;
            workers.emplace_back([&, y_begin, y_end]() {
                for (unsigned int yy = y_begin; yy < y_end; ++yy) {
                    for (unsigned int xx = 0; xx < width; ++xx)
                        fn(xx, yy);
                }
            });
        }
        for (auto& worker : workers)
            worker.join();
    };

    if (IsLidarPipeline(pipeline)) {
        auto lidar = std::dynamic_pointer_cast<ChLidarSensor>(sensor);
        if (!lidar || !m_buffer_di)
            return;

        const ChVulkanRTRenderCache* cache = m_render_cache.get();
        parallel_for_pixels([&](unsigned int x, unsigned int y) {
            const size_t idx = static_cast<size_t>(y) * width + x;
            const ChVector3d dir = LidarRayDirection(lidar, x, y, width, height, forward, left, up);
            const RayHit best = TraceRay(cache, m_scene, origin, dir, 1.5 * static_cast<double>(lidar->GetMaxDistance()));
            PixelDI out{0.f, 0.f};
            if (best.hit && best.t >= static_cast<double>(lidar->GetClipNear())) {
                const ChVector3d n = ShadingNormal(cache, best);
                out.range = static_cast<float>(best.t);
                out.intensity = best.material.lidar_intensity * static_cast<float>(std::abs(n.Dot(-dir)));
            }
            m_buffer_di->Buffer[idx] = out;
        });
        return;
    }

    if (pipeline == VulkanPipelineType::RADAR) {
        auto radar = std::dynamic_pointer_cast<ChRadarSensor>(sensor);
        if (!radar || !m_buffer_radar)
            return;

        const ChVulkanRTRenderCache* cache = m_render_cache.get();
        const ChVector3d radar_velocity = radar->GetTranslationalVelocity();
        parallel_for_pixels([&](unsigned int x, unsigned int y) {
            const size_t idx = static_cast<size_t>(y) * width + x;
            const ChVector3d dir = RadarRayDirection(radar, x, y, width, height, forward, left, up);
            const RayHit best = TraceRay(cache, m_scene, origin, dir, 1.5 * static_cast<double>(radar->GetMaxDistance()));

            RadarReturn out{};
            out.azimuth = static_cast<float>((static_cast<double>(x) / static_cast<double>(width)) * radar->GetHFOV() - radar->GetHFOV() / 2.0);
            out.elevation = static_cast<float>((static_cast<double>(y) / static_cast<double>(height)) * radar->GetVFOV() - radar->GetVFOV() / 2.0);
            out.doppler_velocity[0] = 0.f;
            out.doppler_velocity[1] = 0.f;
            out.doppler_velocity[2] = 0.f;
            out.range = 0.f;
            out.amplitude = 0.f;
            out.objectId = 0.f;

            if (best.hit && best.t >= static_cast<double>(radar->GetClipNear())) {
                const ChVector3d n = ShadingNormal(cache, best);
                const ChVector3d hit_point = origin + dir * best.t;
                const ChVector3d r = hit_point - best.primitive_origin;
                ChVector3d vel_global(0.0, 0.0, 0.0);
                if (best.translational_velocity.Length() > CH_VKRT_EPS || best.angular_velocity.Length() > CH_VKRT_EPS)
                    vel_global = best.translational_velocity + best.angular_velocity.Cross(r) - radar_velocity;

                out.range = static_cast<float>(best.t);
                out.doppler_velocity[0] = static_cast<float>(forward.Dot(vel_global));
                out.doppler_velocity[1] = static_cast<float>(left.Dot(vel_global));
                out.doppler_velocity[2] = static_cast<float>(up.Dot(vel_global));
                out.amplitude = best.material.radar_backscatter * static_cast<float>(std::abs(n.Dot(-dir)));
                out.objectId = best.object_id;
            }

            m_buffer_radar->Buffer[idx] = out;
        });
        return;
    }

    const double hfov = static_cast<double>(CameraHFOV(sensor));
    const double aspect = static_cast<double>(width) / static_cast<double>(height);
    const double tan_h = std::tan(0.5 * hfov);
    const float max_depth = CameraMaxDepth(sensor);
    const float gamma = CameraGamma(sensor);
    const bool use_gi = CameraUseGI(sensor);
    const CameraLensModelType lens_model = CameraLensModel(sensor);
    const LensParams lens_params = CameraLensParameters(sensor);
    const bool is_color_pipeline = pipeline == VulkanPipelineType::CAMERA || pipeline == VulkanPipelineType::PHYS_CAMERA;
    const unsigned int sample_factor = is_color_pipeline ? CameraSampleFactor(sensor) : 1u;
    const ChVulkanRTRenderCache* cache = m_render_cache.get();

    auto render_pixel = [&](unsigned int x, unsigned int y) {
        const size_t idx = static_cast<size_t>(y) * width + x;

        if (is_color_pipeline) {
            ChVector3f accum(0.f, 0.f, 0.f);
            ChVector3d center_dir = forward;
            const unsigned int spp = sample_factor * sample_factor;
            for (unsigned int sy = 0; sy < sample_factor; ++sy) {
                for (unsigned int sx = 0; sx < sample_factor; ++sx) {
                    const unsigned int sample_idx = sy * sample_factor + sx;
                    const bool center_sample = (sample_idx + 1u == spp);
                    const double jx = (sample_factor == 1u || center_sample) ? 0.5 : static_cast<double>(OptixJitterComponent(x, y, sample_idx, 0u));
                    const double jy = (sample_factor == 1u || center_sample) ? 0.5 : static_cast<double>(OptixJitterComponent(x, y, sample_idx, 1u));
                    const double fx = static_cast<double>(x) + jx;
                    const double fy = static_cast<double>(y) + jy;
                    double uv_x = 2.0 * fx / static_cast<double>(width) - 1.0;
                    // Match OptiX raygen exactly: launch-index y grows downward, so
                    // row 0 maps to negative camera-up in the raw sensor buffer.
                    double uv_y = 2.0 * fy / static_cast<double>(height) - 1.0;
                    uv_y *= 1.0 / std::max(CH_VKRT_EPS, aspect);
                    ApplyOptixLensModel(uv_x, uv_y, hfov, lens_model, lens_params);
                    const ChVector3d dir = NormalizeSafe(forward + right * (uv_x * tan_h) + up * (uv_y * tan_h), forward);
                    if (center_sample || sample_factor == 1u)
                        center_dir = dir;
                    accum += TraceCameraColor(cache, m_scene, origin, dir, 0, use_gi);
                }
            }
            accum /= static_cast<float>(std::max(1u, spp));
            const ChVector3f color = GammaCorrect(accum, gamma);
            if (pipeline == VulkanPipelineType::PHYS_CAMERA) {
                const RayHit best = TraceRay(cache, m_scene, origin, center_dir);
                PixelRGBDHalf4 out{};
                out.R = color.x();
                out.G = color.y();
                out.B = color.z();
                out.D = best.hit ? static_cast<float>(best.t) : 0.f;
                m_buffer_rgbd->Buffer[idx] = out;
            } else {
                m_buffer_rgba8->Buffer[idx] = MakeRGBA(color);
            }
            return;
        }

        double uv_x = 2.0 * (static_cast<double>(x) + 0.5) / static_cast<double>(width) - 1.0;
        // Keep row/index convention identical to OptiX raygen.
        double uv_y = 2.0 * (static_cast<double>(y) + 0.5) / static_cast<double>(height) - 1.0;
        uv_y *= 1.0 / std::max(CH_VKRT_EPS, aspect);
        ApplyOptixLensModel(uv_x, uv_y, hfov, lens_model, lens_params);

        // Camera raygen uses tan(hFOV/2).  The current OptiX depth/normal/segmentation
        // raygens still use hFOV/pi*2; mirror that convention for pixel-identical
        // auxiliary buffers instead of silently changing their projection.
        const double ray_factor = hfov / CH_VKRT_PI * 2.0;
        const ChVector3d dir = NormalizeSafe(forward + right * (uv_x * ray_factor) + up * (uv_y * ray_factor), forward);
        const RayHit best = TraceRay(cache, m_scene, origin, dir);

        if (pipeline == VulkanPipelineType::DEPTH_CAMERA) {
            m_buffer_depth->Buffer[idx].depth = best.hit ? static_cast<float>(std::min<double>(best.t, max_depth)) : max_depth;
        } else if (pipeline == VulkanPipelineType::NORMAL_CAMERA) {
            if (best.hit) {
                const ChVector3d n = ShadingNormal(cache, best);
                m_buffer_normal->Buffer[idx].normal_x = static_cast<float>(n.x());
                m_buffer_normal->Buffer[idx].normal_y = static_cast<float>(n.y());
                m_buffer_normal->Buffer[idx].normal_z = static_cast<float>(n.z());
            } else {
                m_buffer_normal->Buffer[idx].normal_x = 0.f;
                m_buffer_normal->Buffer[idx].normal_y = 0.f;
                m_buffer_normal->Buffer[idx].normal_z = 0.f;
            }
        } else if (pipeline == VulkanPipelineType::SEGMENTATION) {
            PixelSemantic p{};
            if (best.hit) {
                p.class_id = best.material.class_id;
                p.instance_id = best.material.instance_id;
            } else {
                p.class_id = 0;
                p.instance_id = 0;
            }
            m_buffer_semantic->Buffer[idx] = p;
        } else {
            const ChVector3f color = TraceCameraColor(cache, m_scene, origin, dir, 0, use_gi);
            m_buffer_rgba8->Buffer[idx] = MakeRGBA(GammaCorrect(color, gamma));
        }
    };

    const unsigned int hw_threads = std::max(1u, std::thread::hardware_concurrency());
    const unsigned int thread_count = std::min<unsigned int>(height, hw_threads);
    if (thread_count <= 1 || height < 32) {
        for (unsigned int y = 0; y < height; ++y) {
            for (unsigned int x = 0; x < width; ++x)
                render_pixel(x, y);
        }
        return;
    }

    std::vector<std::thread> workers;
    workers.reserve(thread_count);
    for (unsigned int t = 0; t < thread_count; ++t) {
        const unsigned int y_begin = (height * t) / thread_count;
        const unsigned int y_end = (height * (t + 1)) / thread_count;
        workers.emplace_back([&, y_begin, y_end]() {
            for (unsigned int y = y_begin; y < y_end; ++y) {
                for (unsigned int x = 0; x < width; ++x)
                    render_pixel(x, y);
            }
        });
    }

    for (auto& worker : workers)
        worker.join();
}

}  // namespace sensor
}  // namespace chrono
