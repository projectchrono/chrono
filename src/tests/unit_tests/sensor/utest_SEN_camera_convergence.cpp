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
// Authors: Dan Negrut
// =============================================================================
//
// Regression test: supersampling must actually reduce Monte Carlo noise.
//
// The camera raygen programs draw stochastic shading samples from a per-pixel cuRAND state.
// They once re-read that state from rng_buffer[pixel_idx] on every iteration of the
// supersampling loop, which handed all num_spp samples an identical state. Every sample then
// chose the same environment direction and the same GI hemisphere direction, so averaging them
// reduced no variance at all: raising supersample_factor bought geometry antialiasing and
// nothing else. This test pins that behavior down so it cannot come back silently.
//
// Both affected raygen programs are covered: camera_raygen.cu (standard camera, LEGACY and
// PATH integrators) and phys_cam_raygen.cu (physical camera). The remaining camera raygen
// programs (depth, normal, segmentation) have no supersampling loop and advance the stored
// state in place, so they were never affected.
//
// Method. A single flat, uniform, fully rough surface is placed to fill the frame and lit ONLY
// by an environment map, which is the stochastic path. Because the geometry and the material
// are uniform and the ambient term is zeroed, essentially all spatial variation across the
// surface is Monte Carlo noise rather than scene content. Averaging N independent samples
// should shrink that noise like 1/sqrt(N), so the spatial standard deviation at 16 samples per
// pixel must fall well below the value at 1 sample per pixel.
//
// The ratio threshold is deliberately loose. For an ideal generator, theory predicts about 1/4
// for 16 independent samples; the defect produces a ratio of essentially 1.0. The 1/sqrt(N)
// figure is a theoretical target rather than an exact prediction: curand_uniform returns values
// in (0,1] and may consume a variable number of underlying draws, and the integrators differ in
// how many draws they take per sample, so measured ratios sit near but not on the ideal value
// (0.25 to 0.37 across the cases here). Requiring only < 0.75 sits far from both populations, so
// the test discriminates the defect without being sensitive to driver version, GPU model, or
// those implementation details.
//
// =============================================================================

#include <algorithm>
#include <set>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/core/ChDataPath.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChPhysCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"

// For the buffer-level resolution test at the end of this file. It drives cuRAND and the module's own
// noise kernel directly, because two rendered frames of DIFFERENT sizes cannot show a prefix
// relationship between their random streams: their scene content differs too.
#include <cuda.h>
#include <curand_kernel.h>
#include <cuda_runtime_api.h>
#include "chrono_sensor/cuda/curand_utils.cuh"
#include "chrono_sensor/cuda/camera_noise.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"

using namespace chrono;
using namespace chrono::sensor;

namespace {

// RAII guard for the process-global fixed seed.
//
// Needed because the seed is process-global and a GoogleTest ASSERT_* aborts the test body: a fatal
// assertion between SetRandomSeed and ClearRandomSeed would leave the fixed seed switched on and
// silently change every later test in the same binary. A reviewer caught exactly that pattern here.
// Setting the seed through this guard makes the cleanup unconditional.
class FixedSeedGuard {
  public:
    /// seed >= 0 pins the base seed; seed < 0 means "leave seeding cleared", matching the
    /// convention the render helpers in this file already use.
    explicit FixedSeedGuard(long long seed) {
        if (seed >= 0)
            ChSensorManager::SetRandomSeed((unsigned int)seed);
        else
            ChSensorManager::ClearRandomSeed();
    }
    ~FixedSeedGuard() { ChSensorManager::ClearRandomSeed(); }
    FixedSeedGuard(const FixedSeedGuard&) = delete;
    FixedSeedGuard& operator=(const FixedSeedGuard&) = delete;
};

const unsigned int kWidth = 128;
const unsigned int kHeight = 96;

// The statistics below are computed over the central half in each axis, so the surface
// silhouette cannot contribute.
const unsigned int kCentralPixels = (kWidth / 2) * (kHeight / 2);

// One large quad in the z = 0 plane, facing +z, with flat normals and a single uniform matte
// material. Two triangles are enough: the point is uniformity, not detail.
void AddGroundQuad(std::shared_ptr<ChTriangleMeshConnected> mesh, double half_extent) {
    auto& v = mesh->GetCoordsVertices();
    auto& n = mesh->GetCoordsNormals();
    auto& iv = mesh->GetIndicesVertices();
    auto& in = mesh->GetIndicesNormals();

    const int base = (int)v.size();
    v.push_back(ChVector3d(-half_extent, -half_extent, 0));
    v.push_back(ChVector3d(half_extent, -half_extent, 0));
    v.push_back(ChVector3d(half_extent, half_extent, 0));
    v.push_back(ChVector3d(-half_extent, half_extent, 0));
    const int n0 = (int)n.size();
    n.push_back(ChVector3d(0, 0, 1));

    iv.push_back(ChVector3i(base + 0, base + 1, base + 2));
    in.push_back(ChVector3i(n0, n0, n0));
    iv.push_back(ChVector3i(base + 0, base + 2, base + 3));
    in.push_back(ChVector3i(n0, n0, n0));
}

// Build the shared scene: one uniform matte quad lit only by an environment map. The body the
// sensor should attach to is returned through an out-parameter, because GoogleTest ASSERT_*
// macros are only usable in a void-returning function.
void BuildScene(ChSystemNSC& sys,
                std::shared_ptr<ChSensorManager> manager,
                std::shared_ptr<ChBody>* out_body) {
    auto mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    AddGroundQuad(mesh, 40.0);
    auto shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    shape->SetMesh(mesh);
    shape->SetMutable(false);
    auto mat = chrono_types::make_shared<ChVisualMaterial>();
    mat->SetDiffuseColor({0.5f, 0.5f, 0.5f});
    mat->SetSpecularColor({0.0f, 0.0f, 0.0f});
    mat->SetRoughness(1.0f);
    mat->SetMetallic(0.0f);
    mat->SetOpacity(1.0f);
    shape->AddMaterial(mat);

    auto body = chrono_types::make_shared<ChBody>();
    body->SetFixed(true);
    body->AddVisualShape(shape);
    sys.Add(body);

    // Environment lighting only. This is the stochastic path the fix concerns; with an analytic
    // point light the shading would be deterministic and this test would be blind. The same
    // asset is already required by utest_SEN_optixengine.cpp, so this adds no new data
    // dependency to the suite. Asserted explicitly so a missing data directory fails loudly
    // rather than producing an unlit frame and a confusing ratio.
    const std::string env = GetChronoDataFile("sensor/textures/quarry_01_4k.hdr");
    ASSERT_TRUE(std::ifstream(env).good())
        << "environment map not found: " << env
        << "\n  the Chrono data directory must be available to run this test";

    Background bg;
    bg.mode = BackgroundMode::ENVIRONMENT_MAP;
    bg.env_tex = env;
    manager->scene->SetBackground(bg);
    // ChScene defaults ambient to 0.2, which is deterministic and would dilute the noise
    // fraction of the signal. Zero it so what is measured below is the stochastic term alone.
    manager->scene->SetAmbientLight({0.f, 0.f, 0.f});
    manager->scene->AddEnvironmentLight(env, 1.0f);

    *out_body = body;
}

// Mean and spatial standard deviation over the central region, from a green-channel sampler.
// Templated over the pixel type so the 8-bit and 16-bit buffers share one implementation.
template <typename PixelT>
void CentralStats(const PixelT* buf,
                  unsigned int width,
                  unsigned int height,
                  double* out_mean,
                  double* out_std) {
    const unsigned int x0 = width / 4, x1 = 3 * width / 4;
    const unsigned int y0 = height / 4, y1 = 3 * height / 4;
    std::vector<double> vals;
    vals.reserve((x1 - x0) * (y1 - y0));
    for (unsigned int y = y0; y < y1; ++y)
        for (unsigned int x = x0; x < x1; ++x)
            vals.push_back((double)buf[y * width + x].G);

    double mean = 0.0;
    for (double d : vals)
        mean += d;
    mean /= (double)vals.size();
    double sse = 0.0;
    for (double d : vals)
        sse += (d - mean) * (d - mean);
    *out_mean = mean;
    *out_std = std::sqrt(sse / (double)vals.size());
}

// Standard camera (camera_raygen.cu).
void RenderNoise(unsigned int supersample,
                 Integrator integrator,
                 bool use_gi,
                 double* out_mean,
                 double* out_std) {
    ChSystemNSC sys;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    std::shared_ptr<ChBody> body;
    ASSERT_NO_FATAL_FAILURE(BuildScene(sys, manager, &body));

    // Camera above the quad looking straight down, close enough that the surface fills the
    // frame, so the sampled region is all one uniform material.
    auto cam = chrono_types::make_shared<ChCameraSensor>(
        body, 30.0f, chrono::ChFrame<double>({0, 0, 3}, QuatFromAngleY(CH_PI_2)), kWidth, kHeight,
        (float)CH_PI / 3, supersample, CameraLensModelType::PINHOLE,
        use_gi,      // use_diffuse_reflect
        false,       // use_denoiser
        integrator,  // integrator
        1.0f,        // gamma: linear, so noise is not reshaped by the transfer curve
        false        // use_fog
    );
    cam->SetLag(0);
    cam->SetCollectionWindow(0);
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam);

    UserRGBA8BufferPtr rgba;
    for (int i = 0; i < 200; ++i) {
        manager->Update();
        sys.DoStepDynamics(0.01);
        rgba = cam->GetMostRecentBuffer<UserRGBA8BufferPtr>();
        if (rgba && rgba->Buffer && rgba->LaunchedCount > 0)
            break;
    }

    ASSERT_TRUE(rgba);
    ASSERT_TRUE(rgba->Buffer);
    ASSERT_EQ(rgba->Width, kWidth);
    ASSERT_EQ(rgba->Height, kHeight);

    CentralStats(rgba->Buffer.get(), kWidth, kHeight, out_mean, out_std);
}

// Physical camera (phys_cam_raygen.cu). Every post-processing stage is disabled: the noise
// stage in particular injects its own randomness downstream of the raygen and would confound a
// measurement of the raygen's own convergence. All camera-model and control parameters are left
// at their constructor defaults, so no physical constants are invented here.
void RenderNoisePhys(unsigned int supersample, double* out_mean, double* out_std) {
    ChSystemNSC sys;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    std::shared_ptr<ChBody> body;
    ASSERT_NO_FATAL_FAILURE(BuildScene(sys, manager, &body));

    auto cam = chrono_types::make_shared<ChPhysCameraSensor>(
        body, 30.0f, chrono::ChFrame<double>({0, 0, 3}, QuatFromAngleY(CH_PI_2)), kWidth, kHeight,
        CameraLensModelType::PINHOLE,
        supersample,         // supersample_factor
        false,               // use_diffuse_reflect
        false,               // use_denoiser
        false,               // use_defocus_blur
        false,               // use_vignetting
        false,               // use_aggregator
        false,               // use_noise: would add randomness after the raygen
        false,               // use_expsr_to_dv
        Integrator::LEGACY,  // integrator
        1.0f,                // gamma: linear
        false,               // use_fog
        false                // use_motion_blur
    );
    cam->SetLag(0);
    cam->SetCollectionWindow(0);
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA16Access>());
    manager->AddSensor(cam);

    UserRGBA16BufferPtr rgba;
    for (int i = 0; i < 200; ++i) {
        manager->Update();
        sys.DoStepDynamics(0.01);
        rgba = cam->GetMostRecentBuffer<UserRGBA16BufferPtr>();
        if (rgba && rgba->Buffer && rgba->LaunchedCount > 0)
            break;
    }

    ASSERT_TRUE(rgba);
    ASSERT_TRUE(rgba->Buffer);
    ASSERT_EQ(rgba->Width, kWidth);
    ASSERT_EQ(rgba->Height, kHeight);

    CentralStats(rgba->Buffer.get(), kWidth, kHeight, out_mean, out_std);
}

// Shared assertions. Any ASSERT_* failure inside the render helpers is fatal and propagates
// through the ASSERT_NO_FATAL_FAILURE wrappers, so a failure to produce a valid image aborts
// the case rather than reaching the assertions below with stale values.
void CheckConvergence(double mean_1, double std_1, double mean_16, double std_16, const char* label) {
    SCOPED_TRACE(label);

    std::cout << "  [" << label << "]  1 spp : mean " << mean_1 << ", spatial std " << std_1 << "\n";
    std::cout << "  [" << label << "] 16 spp : mean " << mean_16 << ", spatial std " << std_16 << "\n";

    // Guard the premise: if the single-sample render is not actually noisy, the ratio below
    // would be meaningless and the test would pass vacuously. The bound scales with the data
    // range so it works for both the 8-bit and the 16-bit buffer.
    ASSERT_GT(std_1, 1e-3 * std::max(1.0, mean_1))
        << "single-sample render is not noisy enough for this test to mean anything; the "
           "environment light may not be active";

    // The bound is DERIVED, and deliberately derived against the DEFECT's prediction rather than
    // against the ideal estimator. That is what makes it hardware-independent.
    //
    // Two competing predictions:
    //   defect present : every sample of a pixel draws the identical shading random numbers, so
    //                    averaging N of them changes nothing and the ratio is 1.0.
    //   defect absent  : the stochastic component falls as 1/sqrt(N), so with N = 16 the ratio
    //                    tends toward 0.25 PLUS whatever deterministic spatial variation the scene
    //                    contributes, since smooth environment-map gradients across the sampled
    //                    region do not average away with more samples per pixel.
    // The floor of the second case is therefore scene-dependent and GPU-dependent, which is exactly
    // why a bound calibrated to it would be a single-GPU measurement. The CEILING of the first case
    // is not: 1.0 is arithmetic, not empirical.
    //
    // So the test asks only "is the ratio clearly below the no-reduction case", and the margin is
    // quantified rather than asserted. Each std is a sample standard deviation over kCentralPixels
    // values, whose own relative standard error is 1/sqrt(2*(n-1)); the ratio of two such estimates
    // carries roughly sqrt(2) times that. At kCentralPixels = 3072 that is about 1.8 percent, so a
    // bound of 0.75 sits on the order of ten sigma below 1.0. Measured values on the reference GPU
    // were 0.25 to 0.37 with the fix and about 1.0 without it, comfortably either side.
    const double ratio = std_16 / std_1;
    const double kNoReduction = 1.0;   // arithmetic consequence of the defect, not a measurement
    const double kRatioBound = 0.75;
    const double ratio_rel_se = std::sqrt(2.0) / std::sqrt(2.0 * (kCentralPixels - 1));
    const double sigma_margin = (kNoReduction - kRatioBound) / (kNoReduction * ratio_rel_se);
    std::cout << "  [" << label << "] ratio " << ratio << " (bound " << kRatioBound << ", which is "
              << sigma_margin << " sigma below the " << kNoReduction
              << " the defect would produce; ideal-generator floor ~0.25 plus scene gradient)\n";
    EXPECT_LT(ratio, kRatioBound)
        << "supersampling did not reduce Monte Carlo noise (ratio " << ratio
        << "). The per-sample RNG state is most likely being reset every iteration of the "
           "supersampling loop in the raygen program, which makes all samples identical.";

    // Averaging more samples must not shift the estimate: it reduces variance, it does not
    // change what is being estimated. A large shift would mean a biased, not merely noisy,
    // estimator.
    //
    // The tolerance is DERIVED, not chosen. Each reported mean is itself an average over
    // kCentralPixels samples, so its standard error is std/sqrt(kCentralPixels), and the
    // standard error of the difference is the root-sum-square of the two. Allowing 4 sigma
    // makes a spurious failure very unlikely while still catching real bias, and it rescales
    // automatically with scene brightness, bit depth, resolution and sample count. A fixed
    // absolute or fixed relative tolerance cannot do that: a bound loose enough for a bright
    // scene is meaningless on a dark one, and one tight enough for a dark scene fails randomly
    // on a noisy one.
    const double se_diff = std::sqrt((std_1 * std_1 + std_16 * std_16) / (double)kCentralPixels);
    const double mean_tol = 4.0 * se_diff;
    std::cout << "  [" << label << "] mean shift " << std::abs(mean_16 - mean_1) << ", allowed "
              << mean_tol << " (4 sigma of the difference)\n";
    EXPECT_NEAR(mean_16, mean_1, mean_tol)
        << "supersampling shifted the mean by " << std::abs(mean_16 - mean_1) << ", beyond the "
        << mean_tol << " expected from sampling error alone (" << mean_1 << " -> " << mean_16
        << "). That suggests a biased rather than merely noisy estimator.";
}

// ---------------------------------------------------------------------------------------------
// Raw-frame capture, for the determinism, cross-frame and motion-blur cases below.
//
// `seed < 0` leaves seeding alone (clock-derived, the production default). `seed >= 0` pins it
// through ChSensorManager::SetRandomSeed, which is what makes byte-exact comparison possible at
// all: without it every stochastic render differs run to run and no hash can be asserted.
//
// `carrier_speed > 0` attaches the camera to a body that is advanced along +x by hand each step,
// so that with a non-zero collection window the shutter interval spans real camera motion and the
// render is genuinely motion-blurred.
// ---------------------------------------------------------------------------------------------
void RenderRawFrames(unsigned int supersample,
                     long long seed,
                     float collection_window,
                     double carrier_speed,
                     int n_frames,
                     std::vector<std::vector<unsigned char>>* out_frames) {
    out_frames->clear();
    // Guarded rather than set-and-clear: a fatal assertion below would otherwise skip the cleanup
    // and leave the process-global fixed seed switched on for every later test in this binary.
    FixedSeedGuard seed_guard(seed);

    ChSystemNSC sys;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    std::shared_ptr<ChBody> body;
    ASSERT_NO_FATAL_FAILURE(BuildScene(sys, manager, &body));

    // A separate carrier for the camera, so the camera can move while the quad does not. Kept
    // fixed and repositioned explicitly rather than integrated, which keeps the motion exact and
    // independent of gravity or solver settings.
    auto carrier = chrono_types::make_shared<ChBody>();
    carrier->SetFixed(true);
    carrier->SetPos({0, 0, 0});
    sys.Add(carrier);

    // When the camera moves, the frame needs something whose appearance depends on WHERE the
    // camera is. The shared scene is a single uniform diffuse quad lit only by an environment map,
    // and for a diffuse surface the outgoing radiance depends on the surface NORMAL alone, not on
    // position. Translating the camera over it therefore yields a byte-identical image and a
    // motion-blur test built on it would compare two identical frames. A small dark box supplies
    // the missing spatial structure, so camera motion actually smears an edge.
    if (carrier_speed != 0.0) {
        auto marker_shape = chrono_types::make_shared<ChVisualShapeBox>(0.4, 0.4, 0.3);
        auto marker_mat = chrono_types::make_shared<ChVisualMaterial>();
        marker_mat->SetDiffuseColor({0.02f, 0.02f, 0.02f});   // near-black against the lit quad
        marker_mat->SetSpecularColor({0.f, 0.f, 0.f});
        marker_mat->SetRoughness(1.0f);
        marker_mat->SetMetallic(0.0f);
        marker_mat->SetOpacity(1.0f);
        marker_shape->AddMaterial(marker_mat);
        auto marker = chrono_types::make_shared<ChBody>();
        marker->SetFixed(true);
        marker->SetPos({0, 0, 0.15});
        marker->AddVisualShape(marker_shape, chrono::ChFrame<double>());
        sys.Add(marker);
    }

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        carrier, 30.0f, chrono::ChFrame<double>({0, 0, 3}, QuatFromAngleY(CH_PI_2)), kWidth, kHeight,
        (float)CH_PI / 3, supersample, CameraLensModelType::PINHOLE,
        false,               // use_diffuse_reflect
        false,               // use_denoiser
        Integrator::LEGACY,  // integrator
        1.0f,                // gamma: linear
        false                // use_fog
    );
    cam->SetLag(0);
    cam->SetCollectionWindow(collection_window);
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam);

    const double dt = 0.01;
    double t = 0.0;
    unsigned int last_launch = 0;
    for (int i = 0; i < 400 && (int)out_frames->size() < n_frames; ++i) {
        if (carrier_speed != 0.0)
            carrier->SetPos({t * carrier_speed, 0, 0});
        manager->Update();
        sys.DoStepDynamics(dt);
        t += dt;
        auto rgba = cam->GetMostRecentBuffer<UserRGBA8BufferPtr>();
        if (rgba && rgba->Buffer && rgba->LaunchedCount > last_launch) {
            last_launch = rgba->LaunchedCount;
            const unsigned char* p = reinterpret_cast<const unsigned char*>(rgba->Buffer.get());
            out_frames->push_back(std::vector<unsigned char>(p, p + (size_t)kWidth * kHeight * 4));
        }
    }

    ASSERT_EQ((int)out_frames->size(), n_frames)
        << "captured " << out_frames->size() << " frame(s), expected " << n_frames;
}

// Physical camera with the FULL post-processing chain enabled, captured as raw bytes.
//
// The convergence tests deliberately disable every stage, because the noise stage injects its own
// randomness downstream of the raygen and would confound a measurement of the raygen's convergence.
// That left the assembled pipeline uncovered. Here the stages are all on and a fixed seed is used
// instead, which is only possible now that noise seeding honours ChSensorManager::SetRandomSeed.
void RenderPhysFullPipeline(long long seed,
                            bool use_motion_blur,
                            float collection_window,
                            float expsr_time,
                            std::vector<unsigned char>* out_frame,
                            double* out_mean) {
    // Guarded rather than set-and-clear: a fatal assertion below would otherwise skip the cleanup
    // and leave the process-global fixed seed switched on for every later test in this binary.
    FixedSeedGuard seed_guard(seed);

    ChSystemNSC sys;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    std::shared_ptr<ChBody> body;
    ASSERT_NO_FATAL_FAILURE(BuildScene(sys, manager, &body));

    auto cam = chrono_types::make_shared<ChPhysCameraSensor>(
        body, 30.0f, chrono::ChFrame<double>({0, 0, 3}, QuatFromAngleY(CH_PI_2)), kWidth, kHeight,
        CameraLensModelType::PINHOLE,
        1,                   // supersample_factor
        false,               // use_diffuse_reflect
        false,               // use_denoiser
        true,                // use_defocus_blur
        true,                // use_vignetting
        true,                // use_aggregator
        true,                // use_noise: the stochastic stage this test exists to cover
        true,                // use_expsr_to_dv
        Integrator::LEGACY,  // integrator
        1.0f,                // gamma: linear
        false,               // use_fog
        use_motion_blur
    );
    // Explicit control parameters. Left at their defaults the exposure-to-digital stage drove this
    // scene to a fully saturated 65535 in every channel, which made the render seed-independent and
    // the reproducibility comparison vacuous. (aperture_num, expsr_time, ISO, focal_length,
    // focus_dist); expsr_time is the lever and is chosen from a measured sweep, not invented.
    cam->SetCtrlParameters(16.0f, expsr_time, 100.0f, 0.012f, 3.0f);
    cam->SetLag(0);
    cam->SetCollectionWindow(collection_window);
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA16Access>());
    manager->AddSensor(cam);

    UserRGBA16BufferPtr rgba;
    for (int i = 0; i < 200; ++i) {
        manager->Update();
        sys.DoStepDynamics(0.01);
        rgba = cam->GetMostRecentBuffer<UserRGBA16BufferPtr>();
        if (rgba && rgba->Buffer && rgba->LaunchedCount > 0)
            break;
    }

    ASSERT_TRUE(rgba);
    ASSERT_TRUE(rgba->Buffer);
    ASSERT_EQ(rgba->Width, kWidth);
    ASSERT_EQ(rgba->Height, kHeight);

    const unsigned char* p = reinterpret_cast<const unsigned char*>(rgba->Buffer.get());
    const size_t nbytes = (size_t)kWidth * kHeight * 4 * sizeof(uint16_t);
    *out_frame = std::vector<unsigned char>(p, p + nbytes);

    double dummy_std = 0.0;
    CentralStats(rgba->Buffer.get(), kWidth, kHeight, out_mean, &dummy_std);
}

// Mean absolute difference per byte, so "how different" is reportable rather than just "differs".
double MeanAbsDiff(const std::vector<unsigned char>& a, const std::vector<unsigned char>& b) {
    double acc = 0.0;
    const size_t n = std::min(a.size(), b.size());
    for (size_t i = 0; i < n; ++i)
        acc += std::abs((int)a[i] - (int)b[i]);
    return n ? acc / (double)n : 0.0;
}

// Exposure time for the full-pipeline phys-camera case, in seconds.
//
// A measurement, not a magic number. The constructor defaults, and the 0.256 s the phys-camera demo
// uses for its own scene, both drive THIS scene to a fully saturated 65535 in every channel, which
// makes the render seed-independent and any reproducibility comparison vacuous. Sweep printed by
// DISABLED_phys_camera_exposure_calibration below, at f/16, ISO 100, 12 mm, central-region mean out
// of 65535:
//
//     1e-6 ->    271      1e-3 ->  12191   <-- chosen, 18.6 percent of full scale
//     1e-5 ->    879      1e-2 ->  45249
//     1e-4 ->   3021      1e-1 ->  65453   saturated
//                        0.256 ->  65535   saturated
//
// Response is monotonic and close to linear in exposure until it clips, which is itself a sanity
// check on the exposure-to-digital stage. 1e-3 leaves roughly an order of magnitude of headroom in
// both directions, so the test does not sit near either degenerate end.
const float kPhysFullPipelineExposure = 1.0e-3f;

// Per-frame wall-clock timings for a render-bound camera, for the performance probe below.
// Reports every completed frame's interval so the caller can take a median rather than a mean,
// which keeps one scheduler hiccup from dominating the answer.
void TimeFrames(unsigned int width,
                unsigned int height,
                unsigned int supersample,
                int warmup_frames,
                int measured_frames,
                std::vector<double>* out_ms) {
    out_ms->clear();
    ChSystemNSC sys;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    std::shared_ptr<ChBody> body;
    ASSERT_NO_FATAL_FAILURE(BuildScene(sys, manager, &body));

    // A high update rate keeps the camera, not the 30 Hz schedule, the limiting factor.
    auto cam = chrono_types::make_shared<ChCameraSensor>(
        body, 1000.0f, chrono::ChFrame<double>({0, 0, 3}, QuatFromAngleY(CH_PI_2)), width, height,
        (float)CH_PI / 3, supersample, CameraLensModelType::PINHOLE,
        false, false, Integrator::LEGACY, 1.0f, false);
    cam->SetLag(0);
    cam->SetCollectionWindow(0);
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam);

    unsigned int seen = 0, last = 0;
    auto prev = std::chrono::steady_clock::now();
    const int budget = (warmup_frames + measured_frames) * 200 + 2000;
    for (int i = 0; i < budget && (int)out_ms->size() < measured_frames; ++i) {
        manager->Update();
        sys.DoStepDynamics(1e-4);
        auto rgba = cam->GetMostRecentBuffer<UserRGBA8BufferPtr>();
        if (rgba && rgba->Buffer && rgba->LaunchedCount > last) {
            last = rgba->LaunchedCount;
            auto now = std::chrono::steady_clock::now();
            ++seen;
            // Warmup excluded on purpose: the first frames carry OptiX pipeline setup, NVRTC
            // output caching and first-touch allocation, which are not per-frame costs.
            if ((int)seen > warmup_frames)
                out_ms->push_back(std::chrono::duration<double, std::milli>(now - prev).count());
            prev = now;
        }
    }
    ASSERT_EQ((int)out_ms->size(), measured_frames)
        << "only " << out_ms->size() << " frames completed; the loop budget was too small";
}

double Median(std::vector<double> v) {
    if (v.empty())
        return 0.0;
    std::sort(v.begin(), v.end());
    const size_t m = v.size() / 2;
    return v.size() % 2 ? v[m] : 0.5 * (v[m - 1] + v[m]);
}

}  // namespace

// A fixed seed must make a stochastic render byte-exactly reproducible, and different seeds must
// not. This is what the RNG audit asked for and could not have before: seeding read the wall clock,
// so no OptiX render that consumes random numbers was reproducible and no byte-identity hash was
// possible. The first assertion is the regression guard; the second stops it passing vacuously
// because the render stopped being stochastic at all.
TEST(ChOptixEngine, fixed_seed_makes_render_reproducible) {
    std::vector<std::vector<unsigned char>> a, b, c;
    ASSERT_NO_FATAL_FAILURE(RenderRawFrames(1, 12345, 0.f, 0.0, 1, &a));
    ASSERT_NO_FATAL_FAILURE(RenderRawFrames(1, 12345, 0.f, 0.0, 1, &b));
    ASSERT_NO_FATAL_FAILURE(RenderRawFrames(1, 999, 0.f, 0.0, 1, &c));

    EXPECT_EQ(a[0], b[0]) << "same fixed seed produced different bytes; the seed is not reaching "
                             "every RNG the render depends on";
    EXPECT_NE(a[0], c[0]) << "two different seeds produced identical bytes, so either the seed is "
                             "ignored or this render consumes no randomness, which would make the "
                             "reproducibility assertion above meaningless";
    std::cout << "  same seed: mean |diff| " << MeanAbsDiff(a[0], b[0]) << " (must be 0)\n"
              << "  diff seed: mean |diff| " << MeanAbsDiff(a[0], c[0]) << " (must be > 0)\n";
}

// Consecutive frames of a STATIC scene must differ. This is the direct regression test for the
// end-of-frame write-back: before the fix the advanced RNG state was never stored, so every frame
// replayed the previous frame's random sequence and a temporal average over frames gained nothing.
// The seed is fixed so the only thing under test is whether the state advances across frames.
TEST(ChOptixEngine, rng_state_advances_across_frames) {
    std::vector<std::vector<unsigned char>> frames;
    ASSERT_NO_FATAL_FAILURE(RenderRawFrames(1, 4242, 0.f, 0.0, 3, &frames));

    const double d12 = MeanAbsDiff(frames[0], frames[1]);
    const double d13 = MeanAbsDiff(frames[0], frames[2]);
    std::cout << "  frame1 vs frame2: mean |diff| " << d12 << "\n"
              << "  frame1 vs frame3: mean |diff| " << d13 << "\n";

    EXPECT_NE(frames[0], frames[1])
        << "two consecutive frames of a static scene are byte-identical, so the per-pixel RNG state "
           "is not being written back at end of frame and every frame replays the same sequence";
    EXPECT_NE(frames[0], frames[2]) << "frame 3 replays frame 1";
    EXPECT_GT(d12, 0.0);
}

// Motion blur, which the fix touched twice: it edited the shutter-time draw itself and it changed
// which point of the RNG stream that draw comes from. Nothing covered it.
//
// Self-validating on purpose: the first assertion establishes that the setup actually produces
// blur, so if the carrier motion or the collection window were ineffective the test fails loudly
// instead of passing on a pair of identical sharp images.
TEST(ChOptixEngine, motion_blur_is_active_and_stochastic) {
    // At 3 m height with a 60 degree FOV the 128 px frame spans about 3.46 m, i.e. 27 mm/px.
    // 10 m/s over a 20 ms exposure moves 200 mm, roughly 7 px of smear across the marker box
    // edge, well clear of the ~1.5 px an earlier 2 m/s attempt would have produced.
    const double speed = 10.0;   // m/s
    std::vector<std::vector<unsigned char>> sharp, blurred, blurred_again, blurred_other_seed;

    ASSERT_NO_FATAL_FAILURE(RenderRawFrames(4, 777, 0.0f, speed, 1, &sharp));
    ASSERT_NO_FATAL_FAILURE(RenderRawFrames(4, 777, 0.02f, speed, 1, &blurred));
    ASSERT_NO_FATAL_FAILURE(RenderRawFrames(4, 777, 0.02f, speed, 1, &blurred_again));
    ASSERT_NO_FATAL_FAILURE(RenderRawFrames(4, 31337, 0.02f, speed, 1, &blurred_other_seed));

    const double d_blur = MeanAbsDiff(sharp[0], blurred[0]);
    std::cout << "  zero vs 20 ms exposure: mean |diff| " << d_blur << "\n";
    ASSERT_GT(d_blur, 0.0) << "a 20 ms exposure with the camera moving at " << speed
                           << " m/s produced the same image as a zero-length exposure, so motion "
                              "blur is not active and the assertions below would be vacuous";

    // Same seed, same exposure: the shutter samples must repeat exactly.
    EXPECT_EQ(blurred[0], blurred_again[0])
        << "motion blur is not reproducible under a fixed seed, so the shutter draw is reading "
           "randomness the seed does not control";

    // Different seed: the shutter samples must differ, which is what shows the blur is stochastic
    // rather than a fixed sample pattern.
    EXPECT_NE(blurred[0], blurred_other_seed[0])
        << "changing the seed did not change the motion-blurred render, so the shutter time is not "
           "actually being drawn from the per-pixel RNG";
    std::cout << "  same seed:  mean |diff| " << MeanAbsDiff(blurred[0], blurred_again[0])
              << " (must be 0)\n"
              << "  diff seed:  mean |diff| " << MeanAbsDiff(blurred[0], blurred_other_seed[0])
              << " (must be > 0)\n";
}

// The physical camera's assembled post-processing chain: defocus blur, vignetting, aggregator,
// noise and exposure-to-digital all enabled, plus motion blur. The convergence tests switch every
// stage off by design, so until now nothing exercised the pipeline the phys camera actually ships
// with. Reproducibility under a fixed seed is the assertion, which also proves the seed reaches the
// noise stage and not merely the raygen.
TEST(ChOptixEngine, phys_camera_full_pipeline_is_reproducible) {
    std::vector<unsigned char> a, b, c;
    double mean_a = 0, mean_b = 0, mean_c = 0;
    const float e = kPhysFullPipelineExposure;
    ASSERT_NO_FATAL_FAILURE(RenderPhysFullPipeline(2024, true, 0.02f, e, &a, &mean_a));
    ASSERT_NO_FATAL_FAILURE(RenderPhysFullPipeline(2024, true, 0.02f, e, &b, &mean_b));
    ASSERT_NO_FATAL_FAILURE(RenderPhysFullPipeline(4048, true, 0.02f, e, &c, &mean_c));

    std::cout << "  full pipeline, seed 2024: central mean " << mean_a << "\n"
              << "  full pipeline, seed 2024 again: central mean " << mean_b << "\n"
              << "  full pipeline, seed 4048: central mean " << mean_c << "\n";

    // Two-sided non-degeneracy guard. An earlier revision only checked for black, and the test
    // duly failed against a fully SATURATED frame: every byte was 0xFF, so both seeds produced
    // identical output and the inequality below could never hold. A clipped image is just as
    // vacuous as a black one, so both ends are excluded.
    const double kMax = 65535.0;
    ASSERT_GT(mean_a, 0.01 * kMax)
        << "the full phys-camera pipeline produced a near-black central region (mean " << mean_a
        << "), so the comparisons below would be meaningless";
    ASSERT_LT(mean_a, 0.95 * kMax)
        << "the full phys-camera pipeline is saturated (mean " << mean_a << " of " << kMax
        << "). A clipped frame is seed-independent, so the inequality below cannot hold. Lower "
           "kPhysFullPipelineExposure.";

    EXPECT_EQ(a, b) << "the full phys-camera pipeline is not reproducible under a fixed seed; some "
                       "stage, most likely the noise filter, is still seeding itself from the clock";
    EXPECT_NE(a, c) << "two different seeds gave a byte-identical result through a pipeline that "
                       "includes the noise stage, so the noise stage is not actually stochastic";
    std::cout << "  same seed: mean |diff| " << MeanAbsDiff(a, b) << " (must be 0)\n"
              << "  diff seed: mean |diff| " << MeanAbsDiff(a, c) << " (must be > 0)\n";
}

// Performance probe for the per-pixel RNG state store the fix added, disabled by default because
// it is a measurement rather than an assertion.
//
// The fix writes one curandState_t (48 bytes for XORWOW) per pixel per frame that the previous code
// did not. At 1280x720 that is 921600 * 48 = about 44 MB of extra stores per frame, which on this
// GPU's memory bandwidth should cost tens of microseconds against a frame measured in milliseconds.
// This probe measures the frame cost so that estimate can be checked against reality rather than
// asserted.
//
// To get the A/B, run this, then comment out the `camera.rng_buffer[pixel_idx] = rng;` line in
// src/chrono_sensor/optix/shaders/camera_raygen.cu, re-run `ninja` so the shader is re-staged (the
// build-time staging rule makes that reliable; editing the build-tree copy directly does not), and
// run this again. Restore the line afterwards.
//
// Warmup frames are excluded, the median is reported rather than the mean, and the interquartile
// range is printed so a noisy machine is visible instead of silently skewing the number.
TEST(ChOptixEngine, DISABLED_perf_rng_state_writeback) {
    const unsigned int w = 1280, h = 720;
    const unsigned int spp_factor = 2;   // 4 samples per pixel
    std::vector<double> ms;
    ASSERT_NO_FATAL_FAILURE(TimeFrames(w, h, spp_factor, 20, 100, &ms));

    std::vector<double> sorted = ms;
    std::sort(sorted.begin(), sorted.end());
    const double med = Median(ms);
    const double q1 = sorted[sorted.size() / 4];
    const double q3 = sorted[(3 * sorted.size()) / 4];

    const double bytes_per_frame = (double)w * h * 48.0;
    std::cout << "  resolution        : " << w << " x " << h << ", " << (spp_factor * spp_factor)
              << " samples per pixel" << std::endl;
    std::cout << "  frames measured   : " << ms.size() << " (after 20 warmup)" << std::endl;
    std::cout << "  median frame time : " << med << " ms" << std::endl;
    std::cout << "  IQR               : " << q1 << " to " << q3 << " ms" << std::endl;
    std::cout << "  min / max         : " << sorted.front() << " / " << sorted.back() << " ms"
              << std::endl;
    std::cout << "  added RNG stores  : " << (bytes_per_frame / (1024.0 * 1024.0))
              << " MiB per frame" << std::endl;
    std::cout << "  ==> compare this median against a build with the write-back removed; the "
                 "difference is the cost of the fix" << std::endl;

    // Not an assertion about performance, only that the probe produced usable numbers.
    EXPECT_GT(med, 0.0);
}

// Calibration probe for kPhysFullPipelineExposure, disabled by default because it is a
// measurement rather than an assertion. Run it with
//   --gtest_filter='*phys_camera_exposure_calibration*' --gtest_also_run_disabled_tests
// and read off an exposure whose central mean sits mid-range instead of clipped at 65535.
TEST(ChOptixEngine, DISABLED_phys_camera_exposure_calibration) {
    const float sweep[] = {1.0e-6f, 1.0e-5f, 1.0e-4f, 1.0e-3f, 1.0e-2f, 1.0e-1f, 0.256f};
    std::cout << "  exposure [s]      central mean (of 65535)" << std::endl;
    for (float e : sweep) {
        std::vector<unsigned char> frame;
        double mean = 0.0;
        ASSERT_NO_FATAL_FAILURE(RenderPhysFullPipeline(1, false, 0.f, e, &frame, &mean));
        std::cout << "  " << e << "        " << mean
                  << (mean >= 0.95 * 65535.0 ? "   <-- saturated" : "") << std::endl;
    }
}

// camera_raygen.cu, LEGACY integrator with environment lighting: the path the defect was found on.
TEST(ChOptixEngine, supersampling_reduces_noise) {
    double m1 = 0, s1 = 0, m16 = 0, s16 = 0;
    ASSERT_NO_FATAL_FAILURE(RenderNoise(1, Integrator::LEGACY, false, &m1, &s1));
    ASSERT_NO_FATAL_FAILURE(RenderNoise(4, Integrator::LEGACY, false, &m16, &s16));
    CheckConvergence(m1, s1, m16, s16, "LEGACY, env light");
}

// camera_raygen.cu, PATH integrator with diffuse-reflection GI. These draw additional
// randomness from the same per-ray state (camera_path_shader.cuh and the GI hemisphere
// sampling), so they were affected by the same defect and must converge too.
TEST(ChOptixEngine, supersampling_reduces_noise_path_gi) {
    double m1 = 0, s1 = 0, m16 = 0, s16 = 0;
    ASSERT_NO_FATAL_FAILURE(RenderNoise(1, Integrator::PATH, true, &m1, &s1));
    ASSERT_NO_FATAL_FAILURE(RenderNoise(4, Integrator::PATH, true, &m16, &s16));
    CheckConvergence(m1, s1, m16, s16, "PATH, GI on");
}

// phys_cam_raygen.cu carried an identical copy of the defect, so it gets identical coverage.
TEST(ChOptixEngine, supersampling_reduces_noise_phys_camera) {
    double m1 = 0, s1 = 0, m16 = 0, s16 = 0;
    ASSERT_NO_FATAL_FAILURE(RenderNoisePhys(1, &m1, &s1));
    ASSERT_NO_FATAL_FAILURE(RenderNoisePhys(4, &m16, &s16));
    CheckConvergence(m1, s1, m16, s16, "phys camera");
}

// =============================================================================
// TWO SENSORS IN ONE SCENE UNDER ONE FIXED SEED.
//
// These are the end-to-end regression tests for the RNG stream-identity fix, and the only ones here
// that FAIL against the code as it stood before it. Everything above tests one sensor at a time, and
// a single-sensor suite cannot see this defect at all: with one fixed seed handed to every
// curand_init call, one camera looks perfectly correct and reproducible. The damage only appears
// when a second sensor exists, because it then draws the identical random sequence.
//
// Measured on the pre-fix code with a standalone cuRAND probe: two equally sized buffers seeded from
// one value produced 4096 identical draws out of 4096, and a 640x360 buffer was a bit-exact prefix
// of a 1280x720 one. Here that shows up as two cameras producing byte-identical noise.
//
// Both cameras sit at the same pose with the same settings on purpose. Any deterministic part of the
// render is then identical between them, so a difference in the output can only come from the random
// streams, which is what is under test.
// =============================================================================

namespace {

// Render two identically configured cameras in ONE scene under one fixed base seed.
// `with_noise_filter` adds ChFilterCameraNoiseConstNormal to both, exercising the filter-owned RNG
// buffers rather than the raygen ones.
void RenderTwoCameras(long long seed,
                      bool with_noise_filter,
                      std::vector<unsigned char>* out_a,
                      std::vector<unsigned char>* out_b) {
    out_a->clear();
    out_b->clear();
    // Guarded rather than set-and-clear: a fatal assertion below would otherwise skip the cleanup
    // and leave the process-global fixed seed switched on for every later test in this binary.
    FixedSeedGuard seed_guard(seed);

    ChSystemNSC sys;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    std::shared_ptr<ChBody> body;
    ASSERT_NO_FATAL_FAILURE(BuildScene(sys, manager, &body));

    // Two engines would be created if the update rates differed, so keep them equal: this test is
    // about stream identity, not about engine grouping.
    std::shared_ptr<ChCameraSensor> cams[2];
    for (int i = 0; i < 2; ++i) {
        cams[i] = chrono_types::make_shared<ChCameraSensor>(
            body, 30.0f, chrono::ChFrame<double>({0, 0, 3}, QuatFromAngleY(CH_PI_2)), kWidth, kHeight,
            (float)CH_PI / 3, 1, CameraLensModelType::PINHOLE, false, false, Integrator::LEGACY, 1.0f, false);
        cams[i]->SetLag(0);
        cams[i]->SetCollectionWindow(0.f);
        if (with_noise_filter)
            cams[i]->PushFilter(chrono_types::make_shared<ChFilterCameraNoiseConstNormal>(0.f, 0.05f));
        cams[i]->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
        manager->AddSensor(cams[i]);
    }

    const double dt = 0.01;
    unsigned int last[2] = {0, 0};
    std::vector<unsigned char>* out[2] = {out_a, out_b};
    for (int step = 0; step < 400 && (out_a->empty() || out_b->empty()); ++step) {
        manager->Update();
        sys.DoStepDynamics(dt);
        for (int i = 0; i < 2; ++i) {
            if (!out[i]->empty())
                continue;
            auto rgba = cams[i]->GetMostRecentBuffer<UserRGBA8BufferPtr>();
            if (rgba && rgba->Buffer && rgba->LaunchedCount > last[i]) {
                last[i] = rgba->LaunchedCount;
                const unsigned char* p = reinterpret_cast<const unsigned char*>(rgba->Buffer.get());
                *out[i] = std::vector<unsigned char>(p, p + (size_t)kWidth * kHeight * 4);
            }
        }
    }

    ASSERT_FALSE(out_a->empty()) << "camera 0 produced no frame";
    ASSERT_FALSE(out_b->empty()) << "camera 1 produced no frame";
}

}  // namespace

// The stochastic raygen path. Two cameras with no noise filter still consume randomness, through
// environment-light sampling in camera_raygen.cu, so their frames must differ.
TEST(ChOptixEngine, two_cameras_under_one_fixed_seed_render_differently) {
    std::vector<unsigned char> a, b;
    ASSERT_NO_FATAL_FAILURE(RenderTwoCameras(24680, false, &a, &b));

    EXPECT_NE(a, b) << "two cameras in one scene rendered byte-identical frames under a fixed seed, "
                       "so they are sharing one RNG stream. This is the defect: cuRAND separates "
                       "generators by subsequence, which the per-pixel index already uses, so two "
                       "buffers given the same seed ARE the same stream.";
    std::cout << "  two cameras, raygen RNG: mean |diff| " << MeanAbsDiff(a, b) << " (must be > 0)\n";
}

// The filter-owned RNG buffers, which are a separate set of nine call sites from the raygen ones.
//
// WHAT THIS TEST DOES NOT EXERCISE (audit-2 finding C4): each camera here carries ONE noise filter,
// so this does not cover two stochastic filters of the same RngUsage on a SINGLE sensor. That
// configuration is legal, since PushFilter applies no type check, and it is separated by the
// per-filter stream index rather than by the usage constant. It is covered at the seed level by
// duplicate_same_purpose_filters_get_different_seeds in utest_SEN_rng_streams.cpp, and not at the
// rendered-output level anywhere.
TEST(ChOptixEngine, two_cameras_with_noise_filters_render_differently) {
    std::vector<unsigned char> a, b;
    ASSERT_NO_FATAL_FAILURE(RenderTwoCameras(13579, true, &a, &b));

    EXPECT_NE(a, b) << "two cameras carrying the same noise filter produced identical noise under a "
                       "fixed seed, so ChFilterCameraNoiseConstNormal is not deriving a per-sensor "
                       "seed";
    std::cout << "  two cameras, noise filter: mean |diff| " << MeanAbsDiff(a, b) << " (must be > 0)\n";
}

// Separating the streams must not cost reproducibility, which is the whole point of the fixed seed.
// Building the same two-camera scene twice in one process must give both cameras the same frames,
// and it is what forces the manager RNG id to be a reusable slot rather than a counter.
TEST(ChOptixEngine, two_camera_scene_is_reproducible_across_rebuilds) {
    std::vector<unsigned char> a1, b1, a2, b2;
    ASSERT_NO_FATAL_FAILURE(RenderTwoCameras(11111, true, &a1, &b1));
    ASSERT_NO_FATAL_FAILURE(RenderTwoCameras(11111, true, &a2, &b2));

    EXPECT_EQ(a1, a2) << "camera 0 changed between two builds of an identical scene under the same "
                         "fixed seed, so the stream key depends on process history rather than on "
                         "the scene";
    EXPECT_EQ(b1, b2) << "camera 1 changed between two builds of an identical scene";
    EXPECT_NE(a1, b1) << "the two cameras still have to differ from each other";
}

// =============================================================================
// BUFFER-LEVEL EVIDENCE FOR THE DIFFERENT-RESOLUTION AXIS.
//
// Added because audit 2 found the resolution axis proved only at the seed level (finding A4). The
// original defect was measured as a PREFIX relationship: a 640x360 buffer was bit-exact with the
// first 230,400 generators of a 1280x720 one. Two rendered frames of different sizes cannot show
// that, because their scene content differs too, so this test drives cuRAND directly through the
// module's own entry points and compares the noise itself.
//
// It carries its own negative control: the equal-seed case reproduces the defect in-test, so the
// test demonstrates "fails before, passes after" without needing the pre-change library on hand.
// =============================================================================

namespace {

// Apply constant-normal noise to a flat mid-grey buffer of w*h pixels, drawing from the FIRST w*h
// generators of `states`. Returns the noised bytes. This is the module's own noise kernel, so what is
// compared is the same arithmetic a real camera performs.
std::vector<unsigned char> NoiseFromStates(curandState_t* states, unsigned int w, unsigned int h) {
    const size_t nbytes = (size_t)w * h * 4;
    unsigned char* dev = cudaMallocHelper<unsigned char>((unsigned int)nbytes);
    cudaMemset(dev, 128, nbytes);
    CUstream stream = 0;  // default stream; this test does no concurrent work
    // stdev is in NORMALIZED units: the kernel divides the pixel by 255, adds the noise, clamps to
    // [0,1] and scales back. 0.05 matches what the render tests above use and is about 13 counts in
    // 8-bit terms. An early draft passed 20.0f here, which is twenty times the whole range, so every
    // pixel saturated to 0 or 255 and two independent streams agreed roughly half the time on sign
    // alone. That made the test look like it had found a prefix relationship when it had only found a
    // bad parameter.
    cuda_camera_noise_const_normal(dev, (int)w, (int)h, 0.f, 0.05f, states, stream);
    cudaStreamSynchronize(0);
    std::vector<unsigned char> host(nbytes);
    cudaMemcpy(host.data(), dev, nbytes, cudaMemcpyDeviceToHost);
    cudaFreeHelper<unsigned char>(dev);
    return host;
}

// Fraction of RGB bytes that agree between two noised buffers. The alpha byte is skipped because the
// noise kernel never writes it, so it agrees by construction and would put a floor of 25% under any
// whole-buffer statistic, making a threshold meaningless.
double RgbAgreementFraction(const std::vector<unsigned char>& a, const std::vector<unsigned char>& b) {
    size_t same = 0, total = 0;
    for (size_t i = 0; i + 3 < a.size() && i + 3 < b.size(); i += 4) {
        for (size_t c = 0; c < 3; ++c) {
            if (a[i + c] == b[i + c])
                ++same;
            ++total;
        }
    }
    return total ? (double)same / (double)total : 1.0;
}

}  // namespace

TEST(ChOptixEngine, smaller_buffer_is_not_a_prefix_of_larger_under_derived_seeds) {
    const unsigned int wBig = 1280, hBig = 720;
    const unsigned int wSmall = 640, hSmall = 360;
    const unsigned int nBig = wBig * hBig, nSmall = wSmall * hSmall;
    ASSERT_LT(nSmall, nBig);

    curandState_t* sBig = cudaMallocHelper<curandState_t>(nBig);
    curandState_t* sSmall = cudaMallocHelper<curandState_t>(nSmall);

    // (1) THE DEFECT, reproduced here: one seed handed to both buffers, which is what the code did
    // before the fix. The small buffer's generators are then bit-identical to the large buffer's
    // first nSmall, so the noise drawn from each is the same.
    const unsigned long long one_seed = 12345ull;
    init_cuda_rng(one_seed, sBig, nBig);
    init_cuda_rng(one_seed, sSmall, nSmall);
    cudaStreamSynchronize(0);
    std::vector<unsigned char> defect_big = NoiseFromStates(sBig, wSmall, hSmall);
    std::vector<unsigned char> defect_small = NoiseFromStates(sSmall, wSmall, hSmall);
    ASSERT_EQ(defect_big, defect_small)
        << "control failed: with one shared seed the smaller buffer was expected to be a bit-exact "
           "prefix of the larger. If this stops holding, cuRAND's behaviour changed and the rest of "
           "this test proves nothing.";
    std::cout << "  shared seed (the old behaviour): mean |diff| "
              << MeanAbsDiff(defect_big, defect_small) << " (must be 0)\n";

    // (2) THE FIX: seeds derived for two different sensor ordinals, exactly as GetDeterministicSeed
    // builds them. The differing resolutions are irrelevant to the key, which is the point: streams
    // are separated by identity, not by size.
    const unsigned long long base = 12345ull;
    const unsigned long long seed_big =
        base + ChSensorManager::MakeRngStreamId(0, 0, 0, RngUsage::CameraNoiseConstNormal);
    const unsigned long long seed_small =
        base + ChSensorManager::MakeRngStreamId(0, 1, 0, RngUsage::CameraNoiseConstNormal);
    ASSERT_NE(seed_big, seed_small);

    init_cuda_rng(seed_big, sBig, nBig);
    init_cuda_rng(seed_small, sSmall, nSmall);
    cudaStreamSynchronize(0);
    std::vector<unsigned char> fixed_big = NoiseFromStates(sBig, wSmall, hSmall);
    std::vector<unsigned char> fixed_small = NoiseFromStates(sSmall, wSmall, hSmall);

    EXPECT_NE(fixed_big, fixed_small)
        << "a 640x360 buffer is still a bit-exact prefix of a 1280x720 one under derived seeds, so "
           "the resolution axis of the defect is not fixed";
    std::cout << "  derived seeds: mean |diff| " << MeanAbsDiff(fixed_big, fixed_small)
              << " (must be > 0)\n";

    // The audit-1 criterion asked for the smaller not to be a prefix. Reporting the fraction of
    // agreeing bytes says how far from a prefix it is, rather than only that it is not one.
    //
    // Where the threshold comes from, so it is a prediction rather than a guess: with noise of stdev
    // s counts added independently to both buffers and rounded to integers, two draws land on the
    // same value with probability about 1/(2*s*sqrt(pi)). At s = 0.05*255 = 12.75 counts that is
    // about 2.2%. A prefix relationship would be 100%. 0.10 sits an order of magnitude above the
    // prediction and an order of magnitude below the defect, so it discriminates without being
    // sensitive to the exact rounding or clamping behaviour.
    const double frac = RgbAgreementFraction(fixed_big, fixed_small);
    std::cout << "  identical RGB bytes over the overlapping region: " << 100.0 * frac
              << "% (predicted about 2.2% for independent streams, 100% for a prefix)\n";
    EXPECT_LT(frac, 0.10) << "far too many overlapping bytes agree for two independent streams";

    // And the control, to show the statistic can see a prefix when there is one.
    const double defect_frac = RgbAgreementFraction(defect_big, defect_small);
    std::cout << "  same statistic on the shared-seed case: " << 100.0 * defect_frac
              << "% (must be 100%)\n";
    EXPECT_DOUBLE_EQ(defect_frac, 1.0) << "the agreement statistic cannot even see a known prefix";

    cudaFreeHelper<curandState_t>(sBig);
    cudaFreeHelper<curandState_t>(sSmall);
}

// =============================================================================
// TWO PHYSICAL CAMERAS. Added for audit-2 finding A6: the phys-camera shot-noise buffer is one of the
// nine rewired sites, and the one implicated in the hang recorded as DEVIATIONS D16, yet nothing
// asserted that two phys cameras draw different shot noise.
// =============================================================================

namespace {

void RenderTwoPhysCameras(long long seed,
                          float expsr_time,
                          std::vector<unsigned char>* out_a,
                          std::vector<unsigned char>* out_b) {
    out_a->clear();
    out_b->clear();
    // Guarded rather than set-and-clear: a fatal assertion below would otherwise skip the cleanup
    // and leave the process-global fixed seed switched on for every later test in this binary.
    FixedSeedGuard seed_guard(seed);

    ChSystemNSC sys;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    std::shared_ptr<ChBody> body;
    ASSERT_NO_FATAL_FAILURE(BuildScene(sys, manager, &body));

    std::shared_ptr<ChPhysCameraSensor> cams[2];
    for (int i = 0; i < 2; ++i) {
        cams[i] = chrono_types::make_shared<ChPhysCameraSensor>(
            body, 30.0f, chrono::ChFrame<double>({0, 0, 3}, QuatFromAngleY(CH_PI_2)), kWidth, kHeight,
            CameraLensModelType::PINHOLE, 1, false, false, true, true, true,
            true,  // use_noise: the stochastic stage under test
            true, Integrator::LEGACY, 1.0f, false, false);
        cams[i]->SetCtrlParameters(16.0f, expsr_time, 100.0f, 0.012f, 3.0f);
        cams[i]->SetLag(0);
        cams[i]->SetCollectionWindow(0.f);
        cams[i]->PushFilter(chrono_types::make_shared<ChFilterRGBA16Access>());
        manager->AddSensor(cams[i]);
    }

    unsigned int last[2] = {0, 0};
    std::vector<unsigned char>* out[2] = {out_a, out_b};
    for (int step = 0; step < 300 && (out_a->empty() || out_b->empty()); ++step) {
        manager->Update();
        sys.DoStepDynamics(0.01);
        for (int i = 0; i < 2; ++i) {
            if (!out[i]->empty())
                continue;
            auto rgba = cams[i]->GetMostRecentBuffer<UserRGBA16BufferPtr>();
            if (rgba && rgba->Buffer && rgba->LaunchedCount > last[i]) {
                last[i] = rgba->LaunchedCount;
                const unsigned char* p = reinterpret_cast<const unsigned char*>(rgba->Buffer.get());
                *out[i] = std::vector<unsigned char>(p, p + (size_t)kWidth * kHeight * 4 * sizeof(uint16_t));
            }
        }
    }
    ASSERT_FALSE(out_a->empty()) << "phys camera 0 produced no frame";
    ASSERT_FALSE(out_b->empty()) << "phys camera 1 produced no frame";
}

}  // namespace

TEST(ChOptixEngine, two_phys_cameras_under_one_fixed_seed_render_differently) {
    std::vector<unsigned char> a, b;
    ASSERT_NO_FATAL_FAILURE(RenderTwoPhysCameras(31415, kPhysFullPipelineExposure, &a, &b));

    EXPECT_NE(a, b) << "two physical cameras with the full pipeline, including shot noise, produced "
                       "byte-identical frames under one fixed base seed, so their shot-noise and "
                       "raygen streams are shared";
    std::cout << "  two phys cameras: mean |diff| " << MeanAbsDiff(a, b) << " (must be > 0)\n";
}

// =============================================================================
// AUDIT-2 FINDING A6, PROPERLY. The test above (two_phys_cameras_..._render_differently) shows two
// physical cameras produce different frames, and that is NOT enough: a physical camera draws from TWO
// separate streams, its OptiX raygen buffer and its shot-noise buffer, so differing frames could be
// caused entirely by the raygen stream while the shot-noise buffers remained shared. The reviewer
// caught that the earlier test conflated them, and it was right.
//
// This isolates the shot-noise stream by asking the production seed helper directly, for that usage
// alone, using the real identities the two registered cameras were given.
// =============================================================================

namespace {

// Construct and register two physical cameras with noise enabled, without rendering. Registration is
// what mints their identities, which is all this test needs.
void RegisterTwoPhysCameras(ChSystemNSC& sys,
                            std::shared_ptr<ChSensorManager> manager,
                            std::shared_ptr<ChBody> body,
                            std::shared_ptr<ChPhysCameraSensor>* out0,
                            std::shared_ptr<ChPhysCameraSensor>* out1) {
    std::shared_ptr<ChPhysCameraSensor> cams[2];
    for (int i = 0; i < 2; ++i) {
        cams[i] = chrono_types::make_shared<ChPhysCameraSensor>(
            body, 30.0f, chrono::ChFrame<double>({0, 0, 3}, QuatFromAngleY(CH_PI_2)), kWidth, kHeight,
            CameraLensModelType::PINHOLE, 1, false, false, true, true, true,
            true,  // use_noise, so the shot-noise stage exists
            true, Integrator::LEGACY, 1.0f, false, false);
        cams[i]->SetCtrlParameters(16.0f, kPhysFullPipelineExposure, 100.0f, 0.012f, 3.0f);
        cams[i]->SetLag(0);
        cams[i]->SetCollectionWindow(0.f);
        cams[i]->PushFilter(chrono_types::make_shared<ChFilterRGBA16Access>());
        manager->AddSensor(cams[i]);
    }
    *out0 = cams[0];
    *out1 = cams[1];
}

}  // namespace

TEST(ChOptixEngine, two_phys_cameras_get_independent_shot_noise_streams) {
    FixedSeedGuard seed_guard(20260731);

    ChSystemNSC sys;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    std::shared_ptr<ChBody> body;
    ASSERT_NO_FATAL_FAILURE(BuildScene(sys, manager, &body));
    std::shared_ptr<ChPhysCameraSensor> cam0, cam1;
    ASSERT_NO_FATAL_FAILURE(RegisterTwoPhysCameras(sys, manager, body, &cam0, &cam1));

    // Both are registered, with different ordinals, in the same manager.
    ASSERT_NE(cam0->GetRngSensorOrdinal(), CH_SENSOR_UNASSIGNED_RNG_ID);
    ASSERT_NE(cam1->GetRngSensorOrdinal(), CH_SENSOR_UNASSIGNED_RNG_ID);
    ASSERT_NE(cam0->GetRngSensorOrdinal(), cam1->GetRngSensorOrdinal());
    ASSERT_EQ(cam0->GetRngManagerId(), cam1->GetRngManagerId());

    // (1) THE A6 CONDITION. For the shot-noise usage alone, and holding the filter index equal so the
    // only thing varying is which camera it is, the two cameras must get different seeds. Swept over
    // several filter indices because the phys pipeline's stage count could change and the shot-noise
    // stage's index with it; the property must hold whichever index it lands on.
    for (unsigned int k = 0; k < 8; ++k) {
        const unsigned long long s0 =
            ChSensorManager::GetDeterministicSeed(cam0, RngUsage::PhysCameraShotNoise, k);
        const unsigned long long s1 =
            ChSensorManager::GetDeterministicSeed(cam1, RngUsage::PhysCameraShotNoise, k);
        EXPECT_NE(s0, s1) << "both physical cameras would seed their shot-noise buffer identically at "
                             "filter index " << k << ", so their shot noise is the same stream";
    }

    // (2) And the shot-noise stream must differ from the raygen stream ON THE SAME CAMERA, which is
    // exactly the confusion that made the frame comparison insufficient: if these two coincided, a
    // camera's shot noise would replay its own raygen sequence.
    for (unsigned int k = 0; k < 8; ++k) {
        EXPECT_NE(ChSensorManager::GetDeterministicSeed(cam0, RngUsage::PhysCameraShotNoise, k),
                  ChSensorManager::GetDeterministicSeed(cam0, RngUsage::OptixPhysCameraRaygen, k))
            << "shot noise and raygen share a stream on one camera at filter index " << k;
    }

    // (3) Every pair drawn from {both cameras} x {both phys usages} x {several indices} must be
    // distinct, which is the whole-set version of the two checks above.
    std::set<unsigned long long> seeds;
    size_t n = 0;
    for (auto cam : {cam0, cam1}) {
        for (RngUsage u : {RngUsage::PhysCameraShotNoise, RngUsage::OptixPhysCameraRaygen}) {
            for (unsigned int k = 0; k < 8; ++k) {
                EXPECT_TRUE(seeds.insert(ChSensorManager::GetDeterministicSeed(cam, u, k)).second)
                    << "collision within the two cameras' physical-camera streams";
                ++n;
            }
        }
    }
    EXPECT_EQ(seeds.size(), n);
    std::cout << "  two phys cameras: " << n << " (camera x usage x filter-index) shot-noise and "
                 "raygen seeds, all distinct\n";

}

// THE OTHER HALF OF A6 IS NOT TESTED, AND HERE IS WHY, MEASURED RATHER THAN ASSERTED.
//
// Three reviewers made it binding that ChFilterPhysCameraNoise keep using its caller-supplied
// m_FPN_seed for the fixed-pattern-noise buffer instead of routing it through the new per-sensor
// derivation. The code does exactly that: only the shot-noise buffer goes through
// GetDeterministicSeed. But it cannot be tested from outside, because the FPN seed is unreachable
// through the public API:
//
//   ChPhysCameraSensor's constructor takes no noise-params argument, so FPN_rng_seed always takes its
//   default, and ChFilterPhysCameraNoise::SetFilterModelParameters(dark_current_vec, noise_gain_vec,
//   STD_read_vec) has no FPN-seed argument. The value is captured in the ChFilterPhysCameraNoise
//   constructor, called from the ChPhysCameraSensor constructor, and nothing afterwards can change it.
//
// So no test can give two cameras different FPN seeds and observe the difference.
//
// An earlier revision of this file DID have a test here. It called init_cuda_rng twice with the same
// literal and asserted the results matched. A reviewer correctly pointed out that this proves only
// that init_cuda_rng is deterministic: it never routes a caller-supplied value through
// ChFilterPhysCameraNoise, so it would still pass if that filter were later changed to derive or
// ignore m_FPN_seed. It was deleted rather than kept, because a test that cannot fail for the reason
// named in its own title is worse than no test: it reports coverage that does not exist.
//
// Making this testable needs a setter for the FPN seed or a constructor that accepts noise params.
// Both change ChPhysCameraSensor's public interface, which a seeding fix has no business doing, so
// this is recorded as a genuine gap instead.
