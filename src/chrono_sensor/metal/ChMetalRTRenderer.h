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
// Metal hardware ray-tracing renderer: builds BLAS/TLAS from a RenderScene and
// renders a pinhole camera into an RGBA8 host buffer. Header is pure C++ (PIMPL);
// all Metal code lives in the .mm.
// =============================================================================

#ifndef CH_METAL_RT_RENDERER_H
#define CH_METAL_RT_RENDERER_H

#include <cstdint>

#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/metal/ChMetalRenderTypes.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_metal
/// @{

/// Explicit pinhole camera basis (world space), matching Chrono::Sensor convention.
struct MetalCameraParams {
    float origin[3];
    float forward[3];
    float right[3];
    float up[3];
    float ambient[3];       ///< ambient light color
    float tanHalfV;         ///< tan(vertical_half_fov)
    int aa;                 ///< antialiasing samples per axis
    int mode;               ///< 0 color, 1 depth, 2 normal, 3 segmentation, 4 lidar
    float lidarHFov;        ///< lidar horizontal FOV (rad)
    float lidarVMin;        ///< lidar min elevation (rad)
    float lidarVMax;        ///< lidar max elevation (rad)
    float maxDist;          ///< lidar max range (0 = infinite)
    int lidarSampleRadius;  ///< beam sub-sampling radius (1 = single ray)
    float lidarHDiv;        ///< horizontal beam divergence (rad)
    float lidarVDiv;        ///< vertical beam divergence (rad)
    int lidarReturnMode;    ///< 0 strongest,1 mean,2 first,3 last,4 dual
    float clipNear = 0.f;   ///< lidar/radar near clip (ChLidarSensor::GetClipNear); returns closer than this are ignored
    /// Halves of this render's 64-bit RNG stream seed from ChSensorManager::GetDeterministicSeed.
    /// Split because the shader's uniform block is 32-bit typed; the shader folds both halves back
    /// together. Zero is a valid value and simply reproduces the pre-seeding sample pattern.
    unsigned int rngSeedLo = 0;
    unsigned int rngSeedHi = 0;
    int lensModel;                               ///< 0 pinhole, 1 FOV (fisheye), 2 radial
    float dk1, dk2, dk3;                         ///< radial distortion coefficients
    int bgMode = 0;                              ///< background: 0 procedural/env, 1 gradient, 2 solid
    float bgZenith[3] = {0.26f, 0.48f, 0.82f};   ///< gradient zenith / solid color
    float bgHorizon[3] = {0.58f, 0.72f, 0.88f};  ///< gradient horizon
    float fogColor[3] = {0.6f, 0.7f, 0.8f};      ///< fog color
    float fogScatter = 0.f;                      ///< exponential fog scattering (0 = off)
    int useGi = 0;                               ///< 1 = global illumination requested
    int integratorPath = 0;                      ///< 1 = Integrator::PATH (else LEGACY)
    int hitLimit = 1;                            ///< camera-ray surface-hit budget (OptiX max_depth - 2)
    int lidarBeamShape = 0;                      ///< LidarBeamShape: 0 RECTANGULAR, 1 ELLIPTICAL
    float exposure = 1.f;                        ///< linear exposure/gain
    float vignette = 0.f;                        ///< vignette strength (0 = off)
    float apertureR = 0.f;                       ///< lens aperture radius for depth of field (0 = pinhole)
    float focalDist = 10.f;                      ///< depth-of-field focal distance
    float noiseSigma = 0.f;                      ///< gaussian sensor-noise stddev (0 = off)
    float envIntensity = 1.f;                    ///< environment-map radiance scale (OptiX AddEnvironmentLight intensity_scale)
    float gamma = 2.2f;                          ///< output gamma (OptiX camera.gamma; 2.2 = sRGB, 1 = linear)
    bool useDenoiser = false;                    ///< run the spatial despeckle/denoise pass
};

/// One light, GPU layout (matches the shader's Light struct: 16 floats / 64 bytes).
/// KEEP IN SYNC with `struct Light` in metal/shaders/ChMetalRTShaderMSL.h -- the field
/// order and sizes must match exactly, the buffer is uploaded raw.
struct MetalLightGPU {
    float pos[3];      ///< world position (point/spot) or direction (directional)
    float range;       ///< max_range (informational; the attenuation itself comes from attenScale)
    float color[3];    ///< light color * intensity
    float type;        ///< 0 = point, 1 = directional, 2 = spot, 3 = disk, 4 = rectangle
    float dir[3];      ///< spot axis / disk normal / rectangle edge-1
    float cosOuter;    ///< spot: angle_range (full cone, rad)  | rect: edge-2 .x
    float cosInner;    ///< spot: angle_atten_rate              | rect: edge-2 .y
    float p0;          ///< disk: radius                        | rect: edge-2 .z
    float attenScale;  ///< OptiX atten_scale: max_range > 0 ? 0.01*max_range^2 : 1
    float constColor;  ///< 1 = constant colour (no distance attenuation), 0 = inverse-square
};

class CH_SENSOR_API ChMetalRTRenderer {
  public:
    ChMetalRTRenderer(void* mtlDevice, void* mtlQueue);  // opaque id<MTLDevice>/id<MTLCommandQueue>
    ~ChMetalRTRenderer();

    bool Valid() const;

    void Build(const MetalRenderScene& scene);          // (re)create accel structures, buffers, textures, pipeline
    void UpdateDynamic(const MetalRenderScene& scene);  // upload transforms + refit dynamic geom + rebuild TLAS
    // Renders into a caller-provided RGBA32F buffer (w*h*4 floats). Interpretation by mode:
    //  color: rgb in [0,1]; depth: .r = distance; normal: .rgb = world normal; segmentation: .r=class,.g=instance.
    void Render(const MetalCameraParams& cam, const MetalLightGPU* lights, int numLights, int w, int h, float* rgba32_out);

    /// Load an HDR equirectangular environment map (Radiance .hdr) to use as the
    /// sky background and reflection source. Empty path disables it.
    void SetEnvMap(const std::string& path);

  private:
    struct Impl;
    Impl* p = nullptr;
};

/// @} sensor_metal
}  // namespace sensor
}  // namespace chrono

#endif
