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
// Authors: Kyle Sha
// =============================================================================
// Metal compute implementation of the physics-based camera image pipeline.
// See shaders/ChMetalPhysCamOpsMSL.h for the kernels (ported from phys_cam_ops.cu).
// =============================================================================

#import <Metal/Metal.h>

#include <cstring>
#include <mutex>

#include "chrono_sensor/metal/ChMetalPhysCamOps.h"
#include "chrono_sensor/metal/shaders/ChMetalPhysCamOpsMSL.h"  // kPhysCamOpsMSL

namespace chrono {
namespace sensor {
namespace metal_phys_cam {

namespace {

/// Kernel parameter block. Layout must match `struct PhysCamParams` in the MSL source:
/// six float4s first (each 16-byte aligned), then the 4-byte scalars.
struct alignas(16) PhysCamParams {
    float qe[4];
    float darkCurrent[4];
    float noiseGain[4];
    float sigmaRead[4];
    float gains[4];
    float biases[4];
    uint32_t width, height;
    float f, U, N, C;
    float defocusGain, defocusBias;
    float L, GVignet;
    float t, P, GAggregator;
    float ISO, gamma;
    uint32_t shotSeed, fpnSeed, frame;
};

inline void set3(float dst[4], const float* src) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = 0.f;
}

/// Lazily-created Metal device + the six compute pipelines, plus a pair of staging
/// buffers reused across calls. One instance per process (the filters are only ever
/// driven from the sensor manager's update).
class Ops {
  public:
    static Ops& Get() {
        static Ops instance;
        return instance;
    }

    bool valid() const { return m_valid; }

    /// Run a one-input/one-output kernel (defocus blur, expsr2dv).
    bool run2(id<MTLComputePipelineState> pso, const void* in, void* out, size_t count, const PhysCamParams& params) {
        if (!m_valid || !pso || count == 0)
            return false;
        std::lock_guard<std::mutex> lock(m_mutex);
        @autoreleasepool {
            const size_t bytes = count * 16;
            if (!ensureStaging(bytes, true))
                return false;
            std::memcpy(m_bufA.contents, in, bytes);
            id<MTLCommandBuffer> cb = [m_queue commandBuffer];
            id<MTLComputeCommandEncoder> e = [cb computeCommandEncoder];
            [e setComputePipelineState:pso];
            [e setBuffer:m_bufA offset:0 atIndex:0];
            [e setBuffer:m_bufB offset:0 atIndex:1];
            [e setBytes:&params length:sizeof(PhysCamParams) atIndex:2];
            dispatch(e, pso, count);
            [e endEncoding];
            [cb commit];
            [cb waitUntilCompleted];
            std::memcpy(out, m_bufB.contents, bytes);
        }
        return true;
    }

    /// Run an in-place kernel (vignetting, aggregator, noise).
    bool run1(id<MTLComputePipelineState> pso, void* inout, size_t count, const PhysCamParams& params) {
        if (!m_valid || !pso || count == 0)
            return false;
        std::lock_guard<std::mutex> lock(m_mutex);
        @autoreleasepool {
            const size_t bytes = count * 16;
            if (!ensureStaging(bytes, false))
                return false;
            std::memcpy(m_bufA.contents, inout, bytes);
            id<MTLCommandBuffer> cb = [m_queue commandBuffer];
            id<MTLComputeCommandEncoder> e = [cb computeCommandEncoder];
            [e setComputePipelineState:pso];
            [e setBuffer:m_bufA offset:0 atIndex:0];
            [e setBytes:&params length:sizeof(PhysCamParams) atIndex:1];
            dispatch(e, pso, count);
            [e endEncoding];
            [cb commit];
            [cb waitUntilCompleted];
            std::memcpy(inout, m_bufA.contents, bytes);
        }
        return true;
    }

    id<MTLComputePipelineState> psoDefocus, psoVignette, psoAggregate, psoNoise;
    id<MTLComputePipelineState> psoDvGamma, psoDvSigmoid, psoDvLinear;

  private:
    Ops() {
        @autoreleasepool {
            m_dev = MTLCreateSystemDefaultDevice();
            if (!m_dev)
                return;
            m_queue = [m_dev newCommandQueue];
            if (!m_queue)
                return;
            NSError* err = nil;
            MTLCompileOptions* co = [MTLCompileOptions new];
            if (@available(macOS 14.0, *))
                co.languageVersion = MTLLanguageVersion3_1;
            else
                co.languageVersion = MTLLanguageVersion2_4;
            id<MTLLibrary> lib = [m_dev newLibraryWithSource:@(kPhysCamOpsMSL) options:co error:&err];
            if (!lib) {
                NSLog(@"Chrono::Sensor Metal phys-camera shader error: %@", err);
                return;
            }
            auto make = [&](const char* name) -> id<MTLComputePipelineState> {
                id<MTLFunction> fn = [lib newFunctionWithName:@(name)];
                if (!fn)
                    return nil;
                NSError* e2 = nil;
                return [m_dev newComputePipelineStateWithFunction:fn error:&e2];
            };
            psoDefocus = make("physCamDefocusBlur");
            psoVignette = make("physCamVignetting");
            psoAggregate = make("physCamAggregator");
            psoNoise = make("physCamNoise");
            psoDvGamma = make("physCamExpsr2dvGamma");
            psoDvSigmoid = make("physCamExpsr2dvSigmoid");
            psoDvLinear = make("physCamExpsr2dvLinear");
            m_valid = psoDefocus && psoVignette && psoAggregate && psoNoise && psoDvGamma && psoDvSigmoid && psoDvLinear;
        }
    }

    bool ensureStaging(size_t bytes, bool needB) {
        if (!m_bufA || m_bufA.length < bytes)
            m_bufA = [m_dev newBufferWithLength:bytes options:MTLResourceStorageModeShared];
        if (needB && (!m_bufB || m_bufB.length < bytes))
            m_bufB = [m_dev newBufferWithLength:bytes options:MTLResourceStorageModeShared];
        return m_bufA && (!needB || m_bufB);
    }

    static void dispatch(id<MTLComputeCommandEncoder> e, id<MTLComputePipelineState> pso, size_t count) {
        NSUInteger tg = pso.maxTotalThreadsPerThreadgroup;
        if (tg > 256)
            tg = 256;
        if (tg < 1)
            tg = 1;
        NSUInteger groups = (count + tg - 1) / tg;
        [e dispatchThreadgroups:MTLSizeMake(groups, 1, 1) threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
    }

    id<MTLDevice> m_dev = nil;
    id<MTLCommandQueue> m_queue = nil;
    id<MTLBuffer> m_bufA = nil, m_bufB = nil;
    std::mutex m_mutex;
    bool m_valid = false;
};

}  // namespace

CH_SENSOR_API bool Available() {
    return Ops::Get().valid();
}

CH_SENSOR_API bool
DefocusBlur(const void* buf_in, void* buf_out, unsigned int img_w, unsigned int img_h, float f, float U, float N, float C, float defocus_gain, float defocus_bias) {
    if (!buf_in || !buf_out)
        return false;
    PhysCamParams p = {};
    p.width = img_w;
    p.height = img_h;
    p.f = f;
    p.U = U;
    p.N = N;
    p.C = C;
    p.defocusGain = defocus_gain;
    p.defocusBias = defocus_bias;
    Ops& o = Ops::Get();
    return o.run2(o.psoDefocus, buf_in, buf_out, static_cast<size_t>(img_w) * img_h, p);
}

CH_SENSOR_API bool Vignetting(void* buf_in_out, unsigned int img_w, unsigned int img_h, float f, float L, float G_vignet) {
    if (!buf_in_out)
        return false;
    PhysCamParams p = {};
    p.width = img_w;
    p.height = img_h;
    p.f = f;
    p.L = L;
    p.GVignet = G_vignet;
    Ops& o = Ops::Get();
    return o.run1(o.psoVignette, buf_in_out, static_cast<size_t>(img_w) * img_h, p);
}

CH_SENSOR_API bool Aggregator(void* buf_in_out, unsigned int img_w, unsigned int img_h, float N, float t, float C, float P, const float* rgb_QEs, float G_aggregator) {
    if (!buf_in_out || !rgb_QEs)
        return false;
    PhysCamParams p = {};
    p.width = img_w;
    p.height = img_h;
    p.N = N;
    p.t = t;
    p.C = C;
    p.P = P;
    p.GAggregator = G_aggregator;
    set3(p.qe, rgb_QEs);
    Ops& o = Ops::Get();
    return o.run1(o.psoAggregate, buf_in_out, static_cast<size_t>(img_w) * img_h, p);
}

CH_SENSOR_API bool Noise(void* buf_in_out,
                         unsigned int img_w,
                         unsigned int img_h,
                         float t,
                         const float* dark_currents,
                         const float* noise_gains,
                         const float* sigma_reads,
                         unsigned int shot_seed,
                         unsigned int fpn_seed,
                         unsigned int frame) {
    if (!buf_in_out || !dark_currents || !noise_gains || !sigma_reads)
        return false;
    PhysCamParams p = {};
    p.width = img_w;
    p.height = img_h;
    p.t = t;
    set3(p.darkCurrent, dark_currents);
    set3(p.noiseGain, noise_gains);
    set3(p.sigmaRead, sigma_reads);
    p.shotSeed = shot_seed;
    p.fpnSeed = fpn_seed;
    p.frame = frame;
    Ops& o = Ops::Get();
    return o.run1(o.psoNoise, buf_in_out, static_cast<size_t>(img_w) * img_h, p);
}

CH_SENSOR_API bool
ExpsrToDV(const void* buf_in, void* buf_out, unsigned int img_w, unsigned int img_h, float ISO, const float* gains, const float* biases, float gamma, int crf_type) {
    if (!buf_in || !buf_out || !gains || !biases)
        return false;
    PhysCamParams p = {};
    p.width = img_w;
    p.height = img_h;
    p.ISO = ISO;
    p.gamma = gamma;
    set3(p.gains, gains);
    set3(p.biases, biases);
    Ops& o = Ops::Get();
    id<MTLComputePipelineState> pso = nil;
    switch (crf_type) {
        case 0:
            pso = o.psoDvGamma;
            break;
        case 1:
            pso = o.psoDvSigmoid;
            break;
        case 2:
            pso = o.psoDvLinear;
            break;
        default:
            return false;  // caller reports the invalid CRF type
    }
    return o.run2(pso, buf_in, buf_out, static_cast<size_t>(img_w) * img_h, p);
}

}  // namespace metal_phys_cam
}  // namespace sensor
}  // namespace chrono
