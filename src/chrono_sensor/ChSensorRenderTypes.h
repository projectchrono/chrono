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
// Common rendered-sensor types shared by OptiX and Vulkan RT front-ends.
// =============================================================================

#ifndef CH_SENSOR_RENDER_TYPES_H
#define CH_SENSOR_RENDER_TYPES_H

#include <string>

#include "chrono/core/ChVector3.h"
#include "chrono_sensor/ChConfigSensor.h"

#ifdef CHRONO_HAS_OPTIX
// OptiX historically declares these public render types in global headers.
// Keep using those exact types in OptiX/mixed builds, but also make them
// available from chrono::sensor so Vulkan-shared headers have one namespace.
#include "chrono_sensor/optix/ChOptixDefinitions.h"
#include "chrono_sensor/optix/shaders/ChOptixLightStructs.h"

namespace chrono {
namespace sensor {

using ::BackgroundMode;
using ::CameraLensModelType;
using ::Integrator;
using ::LensParams;
using ::LidarBeamShape;
using ::LightType;
using ::PhysCameraGainParams;
using ::PhysCameraNoiseParams;

}  // namespace sensor
}  // namespace chrono

#else

namespace chrono {
namespace sensor {

/// The type of lens model that a camera can use for rendering.
enum class CameraLensModelType {
    PINHOLE,   ///< Traditional ideal pinhole camera model.
    FOV_LENS,  ///< Wide-angle spherical/FOV lens model.
    RADIAL     ///< Wide-angle lens model based on radial polynomial fit.
};

/// Camera integration mode. Vulkan RT currently maps all modes to its camera path.
enum class Integrator { PATH, VOLUMETRIC, TRANSIENT, TIMEGATED, MITRANSIENT, LEGACY };

/// Type of background used when camera rays miss all objects in the scene.
enum class BackgroundMode { SOLID_COLOR, GRADIENT, ENVIRONMENT_MAP };

/// The supported light categories exposed by Chrono::Sensor demos/utilities.
enum class LightType { POINT_LIGHT, SPOT_LIGHT, DIRECTIONAL_LIGHT, RECTANGLE_LIGHT, DISK_LIGHT, ENVIRONMENT_LIGHT, AREA_LIGHT };

/// The shape of a lidar beam.  This is a public sensor API type and must be
/// available in Vulkan-RT-only builds as well as OptiX builds.
enum class LidarBeamShape {
    RECTANGULAR,  ///< rectangular beam (inclusive of square beam)
    ELLIPTICAL    ///< elliptical beam (inclusive of circular beam)
};

/// Inverse lens parameters for polynomial radial lens models.
struct LensParams {
    float a0 = 0.f;
    float a1 = 0.f;
    float a2 = 0.f;
    float a3 = 0.f;
    float a4 = 0.f;
    float a5 = 0.f;
    float a6 = 0.f;
    float a7 = 0.f;
    float a8 = 0.f;
};

/// Small CUDA-free float3 replacement used by the physics-camera public
/// parameter structs in Vulkan-only builds. It intentionally exposes x/y/z
/// fields to match CUDA's float3 used by the OptiX backend.
struct PhysCameraFloat3 {
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
};

/// Gain parameters for the physics-based camera model.
struct PhysCameraGainParams {
    float defocus_gain = 0.f;
    float defocus_bias = 0.f;
    float vignetting_gain = 0.f;
    float aggregator_gain = 0.f;
    PhysCameraFloat3 expsr2dv_gains;
    PhysCameraFloat3 expsr2dv_biases;
    float expsr2dv_gamma = 1.f;
    int expsr2dv_crf_type = 0;
};

/// Noise parameters for the physics-based camera model.
struct PhysCameraNoiseParams {
    PhysCameraFloat3 dark_currents;
    PhysCameraFloat3 noise_gains;
    PhysCameraFloat3 STD_reads;
    int FPN_rng_seed = 0;
};

}  // namespace sensor
}  // namespace chrono

#endif

namespace chrono {
namespace sensor {

/// Information about the background of a rendered Sensor scene.
struct Background {
    BackgroundMode mode = BackgroundMode::SOLID_COLOR;
    ChVector3f color_zenith = ChVector3f(0.f, 0.f, 0.f);
    ChVector3f color_horizon = ChVector3f(0.f, 0.f, 0.f);
    std::string env_tex;
};

}  // namespace sensor
}  // namespace chrono

#endif
