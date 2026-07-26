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

#include <cmath>
#include <fstream>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/core/ChDataPath.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChPhysCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono;
using namespace chrono::sensor;

namespace {

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

    const double ratio = std_16 / std_1;
    std::cout << "  [" << label << "] ratio " << ratio << " (ideal-generator target ~0.25 for 16 "
              << "independent samples; 1.0 means supersampling reduced no noise)\n";
    EXPECT_LT(ratio, 0.75) << "supersampling did not reduce Monte Carlo noise (ratio " << ratio
                           << "). The per-sample RNG state is most likely being reset every "
                              "iteration of the supersampling loop in the raygen program, which "
                              "makes all samples identical.";

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

}  // namespace

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
