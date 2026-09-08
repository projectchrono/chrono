// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Kyle Sha
// =============================================================================
//
// TIER 2 -- statistical / convergence tests for the STOCHASTIC features of the
// Metal RT backend: global illumination, area-light soft shadows, depth of field
// and sensor noise.
//
// WHY THESE ARE STATISTICAL
// -------------------------
// Every feature here draws random numbers inside the shader. A specific pixel value
// is a function of the RNG stream, and the RNG stream is an implementation detail
// that differs between backends (curand vs the PCG hash in ChMetalRTShaderMSL.h) and
// may legitimately change within one backend. So no per-pixel value is asserted.
// What IS asserted is a property of the estimator that any correct implementation
// must satisfy no matter which stream it draws from:
//
//   * more samples per pixel must reduce Monte Carlo variance, and must NOT move the
//     mean (an estimator may be noisy; it may not be biased),
//   * an area light of finite extent must produce a WIDER shadow transition than a
//     point light in the same place -- that is geometry, not sampling,
//   * a finite aperture must reduce high-frequency edge energy relative to a pinhole,
//     and its residual sampling noise must fall as samples are added,
//   * additive sensor noise must scale with the requested sigma and must leave the
//     mean where it was.
//
// This mirrors utest_SEN_camera_convergence.cpp, which is the existing precedent for
// this style. That test is not reused directly because it asks for a ChFilterRGBA16Access
// buffer and Integrator::PATH, neither of which the Metal path produces today.
//
// One extra assertion here is not statistical at all but underpins the whole
// golden-image tier: with all stochastic features off, the Metal renderer is
// FRAME-TO-FRAME DETERMINISTIC (its per-pixel seed is a hash of the pixel index, not
// of time), so a static scene renders bit-identically on every frame. Without that,
// pixel-diff regression testing would be impossible.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <functional>
#include <cstdio>
#include <iostream>
#include <numeric>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/assets/ChVisualMaterial.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono;
using namespace chrono::sensor;

namespace {

// ---------------------------------------------------------------------------
// A rendered frame, as 8-bit luminance
// ---------------------------------------------------------------------------
struct Frame {
    unsigned w = 0, h = 0;
    std::vector<double> lum;  // 0..255
    double at(unsigned x, unsigned y) const { return lum[(size_t)y * w + x]; }
};

// Knobs a test may turn. Everything defaults to the fully deterministic configuration.
struct RenderOpts {
    unsigned supersample = 1;
    bool use_gi = false;
    float noise_sigma = 0.f;
    float aperture = 0.f;
    float focal_dist = 1.f;
    float gamma = 2.2f;        // output transfer curve; 1.0 = linear radiance in the 8-bit buffer
    int frames_to_render = 1;  // >1 renders extra frames of the same static scene
};

// Applies a uniform matte material with the given albedo to every shape on a body.
void Matte(std::shared_ptr<ChBody> body, ChColor albedo) {
    auto m = chrono_types::make_shared<ChVisualMaterial>();
    m->SetDiffuseColor(albedo);
    m->SetSpecularColor({0.f, 0.f, 0.f});
    m->SetRoughness(1.0f);
    m->SetMetallic(0.0f);
    m->SetOpacity(1.0f);
    if (auto vm = body->GetVisualModel())
        for (auto& si : vm->GetShapeInstances()) {
            si.shape->GetMaterials().clear();
            si.shape->AddMaterial(m);
        }
}

// Renders `frames_to_render` frames of a static scene and returns the last one (and,
// optionally, the first, so a caller can compare two frames of the same scene).
// `build` populates the system and the scene; it returns the body to mount the camera on.
using BuildFn = std::function<std::shared_ptr<ChBody>(ChSystemNSC&, std::shared_ptr<ChSensorManager>)>;

void Render(const BuildFn& build, const ChFrame<double>& cam_pose, unsigned W, unsigned H, float hfov, const RenderOpts& opt, Frame* out_last, Frame* out_first = nullptr) {
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    auto mount = build(sys, manager);
    ASSERT_TRUE(mount != nullptr);

    if (opt.noise_sigma > 0.f)
        manager->scene->SetSensorNoise(opt.noise_sigma);
    if (opt.aperture > 0.f)
        manager->scene->SetDepthOfField(opt.aperture, opt.focal_dist);

    auto cam = chrono_types::make_shared<ChCameraSensor>(mount, 100.f, cam_pose, W, H, hfov, opt.supersample, CameraLensModelType::PINHOLE, opt.use_gi, false, Integrator::LEGACY,
                                                         opt.gamma, false);
    cam->SetLag(0);
    cam->SetCollectionWindow(0);
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam);

    int captured = 0;
    unsigned last_launch = 0;
    for (int i = 0; i < 400 && captured < opt.frames_to_render; ++i) {
        manager->Update();
        sys.DoStepDynamics(0.01);
        auto rgba = cam->GetMostRecentBuffer<UserRGBA8BufferPtr>();
        if (!rgba || !rgba->Buffer || rgba->LaunchedCount == last_launch)
            continue;
        last_launch = rgba->LaunchedCount;
        Frame f;
        f.w = rgba->Width;
        f.h = rgba->Height;
        f.lum.resize((size_t)f.w * f.h);
        const PixelRGBA8* px = rgba->Buffer.get();
        for (size_t k = 0; k < f.lum.size(); ++k)
            f.lum[k] = (px[k].R + px[k].G + px[k].B) / 3.0;
        if (captured == 0 && out_first)
            *out_first = f;
        *out_last = std::move(f);
        captured++;
    }
    ASSERT_EQ(captured, opt.frames_to_render) << "camera never delivered enough frames";
    ASSERT_EQ(out_last->w, W);
    ASSERT_EQ(out_last->h, H);
}

// Mean and spatial standard deviation over the central half of the frame in each axis,
// so the scene silhouette cannot contribute.
void CentralStats(const Frame& f, double* mean, double* sd) {
    const unsigned x0 = f.w / 4, x1 = f.w - f.w / 4, y0 = f.h / 4, y1 = f.h - f.h / 4;
    double s = 0, s2 = 0;
    long n = 0;
    for (unsigned y = y0; y < y1; ++y)
        for (unsigned x = x0; x < x1; ++x) {
            const double v = f.at(x, y);
            s += v;
            s2 += v * v;
            n++;
        }
    *mean = s / n;
    *sd = std::sqrt(std::max(0.0, s2 / n - (*mean) * (*mean)));
    ASSERT_GT(n, 0);
}

// ---------------------------------------------------------------------------
// shared scenes
// ---------------------------------------------------------------------------

// A single large matte quad filling the frame, lit only by a strongly graded sky.
// The sky IS the light source in GI mode, and its large angular variation is what makes
// the single-sample estimate noisy; with a flat sky the test would pass vacuously.
std::shared_ptr<ChBody> BuildSkylitWall(ChSystemNSC& sys, std::shared_ptr<ChSensorManager> manager) {
    auto wall = chrono_types::make_shared<ChBodyEasyBox>(0.4, 40, 40, 1000, true, false);
    wall->SetPos({12, 0, 0});
    wall->SetFixed(true);
    sys.Add(wall);
    Matte(wall, ChColor(0.55f, 0.55f, 0.55f));

    Background bg;
    bg.mode = BackgroundMode::GRADIENT;
    // Kept dim on purpose: the statistics below are read from a LINEAR 8-bit buffer, and a
    // sample brighter than 1.0 would be clipped, which would bias the mean and invalidate the
    // unbiasedness assertion. albedo 0.55 x zenith 1.0 = 0.55 max per bounce, well clear of 1.
    bg.color_zenith = {1.0f, 1.0f, 1.0f};
    bg.color_horizon = {0.01f, 0.01f, 0.01f};
    manager->scene->SetBackground(bg);
    manager->scene->SetAmbientLight({0.f, 0.f, 0.f});
    // One weak analytic light so the engine does not substitute its own default rig.
    manager->scene->AddDirectionalLight(ChColor(0.06f, 0.06f, 0.06f), 0.6f, 0.9f);

    auto mount = chrono_types::make_shared<ChBody>();
    mount->SetFixed(true);
    sys.Add(mount);
    return mount;
}

// A wall with a slender post floating in front of it, plus one light that casts the
// post's shadow onto the wall. `area_extent` == 0 uses a point light (hard shadow);
// otherwise a rectangle light of that full edge length across the shadow direction.
//
//   light  at x = 2,  post at x = 6 (half width 0.2),  wall face at x = 9.8
//   umbra half-width on the wall  = 0.2 * (9.8-2)/(6-2)                    = 0.39 m
//   penumbra half-width added     = (area_extent/2) * (9.8-6)/(6-2)        = 0.475*extent
// so a 2 m light widens each edge of the transition by ~0.95 m -- far more than the
// ~1 pixel a point light gives. The numbers are similar triangles, not sampling.
std::shared_ptr<ChBody> BuildShadowScene(ChSystemNSC& sys, std::shared_ptr<ChSensorManager> manager, double area_extent) {
    auto wall = chrono_types::make_shared<ChBodyEasyBox>(0.4, 40, 40, 1000, true, false);
    wall->SetPos({10, 0, 0});
    wall->SetFixed(true);
    sys.Add(wall);
    Matte(wall, ChColor(0.7f, 0.7f, 0.7f));

    auto post = chrono_types::make_shared<ChBodyEasyBox>(0.4, 0.4, 12.0, 1000, true, false);
    post->SetPos({6, 1.5, 0});
    post->SetFixed(true);
    sys.Add(post);
    Matte(post, ChColor(0.2f, 0.2f, 0.2f));

    Background bg;
    bg.mode = BackgroundMode::SOLID_COLOR;
    bg.color_zenith = {0.f, 0.f, 0.f};
    bg.color_horizon = {0.f, 0.f, 0.f};
    manager->scene->SetBackground(bg);
    manager->scene->SetAmbientLight({0.06f, 0.06f, 0.06f});

    if (area_extent <= 0.0)
        manager->scene->AddPointLight({2.f, 3.f, 0.f}, ChColor(1.f, 1.f, 1.f), 80.f);
    else
        manager->scene->AddRectangleLight({2.f, 3.f, 0.f}, ChColor(14.f, 14.f, 14.f), 80.f, {0.f, (float)area_extent, 0.f}, {0.f, 0.f, (float)area_extent});

    auto mount = chrono_types::make_shared<ChBody>();
    mount->SetFixed(true);
    sys.Add(mount);
    return mount;
}

// A bright far wall with a dark near box in front of it: one very high contrast,
// nearly vertical silhouette for the depth-of-field measurements.
std::shared_ptr<ChBody> BuildEdgeScene(ChSystemNSC& sys, std::shared_ptr<ChSensorManager> manager) {
    auto wall = chrono_types::make_shared<ChBodyEasyBox>(0.4, 200, 200, 1000, true, false);
    wall->SetPos({40, 0, 0});
    wall->SetFixed(true);
    sys.Add(wall);
    Matte(wall, ChColor(0.9f, 0.9f, 0.9f));

    // Half width 0.6 m centred on y = 0.6, so its near vertical silhouette edge sits at y = 0,
    // i.e. the exact middle column of the frame; the other edge images far to the left.
    auto box = chrono_types::make_shared<ChBodyEasyBox>(0.4, 1.2, 30.0, 1000, true, false);
    box->SetPos({3.0, 0.6, 0});
    box->SetFixed(true);
    sys.Add(box);
    Matte(box, ChColor(0.03f, 0.03f, 0.03f));

    Background bg;
    bg.mode = BackgroundMode::SOLID_COLOR;
    bg.color_zenith = {0.f, 0.f, 0.f};
    bg.color_horizon = {0.f, 0.f, 0.f};
    manager->scene->SetBackground(bg);
    manager->scene->SetAmbientLight({0.35f, 0.35f, 0.35f});
    manager->scene->AddDirectionalLight(ChColor(0.9f, 0.9f, 0.9f), 0.4f, 0.9f);

    auto mount = chrono_types::make_shared<ChBody>();
    mount->SetFixed(true);
    sys.Add(mount);
    return mount;
}

// ---------------------------------------------------------------------------
// profile helpers used by the shadow / depth-of-field measurements
// ---------------------------------------------------------------------------

// Column profile: luminance averaged down the rows of a horizontal band.
std::vector<double> ColumnProfile(const Frame& f, unsigned y0, unsigned y1) {
    std::vector<double> p(f.w, 0.0);
    for (unsigned x = 0; x < f.w; ++x) {
        double s = 0;
        for (unsigned y = y0; y < y1; ++y)
            s += f.at(x, y);
        p[x] = s / (y1 - y0);
    }
    return p;
}

// Width, in columns, of the 25%-75% band of a monotone-ish transition inside [x0,x1).
// Normalising by the local min/max makes it independent of absolute brightness, so a
// point light and an area light of different total power stay comparable.
double TransitionWidth(const std::vector<double>& p, unsigned x0, unsigned x1) {
    const double lo = *std::min_element(p.begin() + x0, p.begin() + x1);
    const double hi = *std::max_element(p.begin() + x0, p.begin() + x1);
    if (hi - lo < 1e-9)
        return (double)(x1 - x0);
    const double a = lo + 0.25 * (hi - lo), b = lo + 0.75 * (hi - lo);
    long n = 0;
    for (unsigned x = x0; x < x1; ++x)
        if (p[x] > a && p[x] < b)
            n++;
    return (double)n;
}

// Mean |dI/dx| inside a window: a direct measure of how sharp the edge is.
double EdgeGradient(const Frame& f, unsigned x0, unsigned x1, unsigned y0, unsigned y1) {
    double s = 0;
    long n = 0;
    for (unsigned y = y0; y < y1; ++y)
        for (unsigned x = x0 + 1; x < x1; ++x) {
            s += std::fabs(f.at(x, y) - f.at(x - 1, y));
            n++;
        }
    return n ? s / n : 0.0;
}

// Mean |second difference| per row inside a window. Blur lowers the first derivative but
// leaves the profile smooth; sampling noise shows up as raggedness, which the second
// difference picks up while a smooth ramp does not.
double RowRoughness(const Frame& f, unsigned x0, unsigned x1, unsigned y0, unsigned y1) {
    double s = 0;
    long n = 0;
    for (unsigned y = y0; y < y1; ++y)
        for (unsigned x = x0 + 1; x + 1 < x1; ++x) {
            s += std::fabs(f.at(x - 1, y) - 2.0 * f.at(x, y) + f.at(x + 1, y));
            n++;
        }
    return n ? s / n : 0.0;
}

const unsigned kW = 192, kH = 144;
const float kHFOV = (float)(CH_PI / 3);

}  // namespace

// =============================================================================
// 0. determinism -- the premise of the golden-image tier
// =============================================================================
TEST(MetalStochastic, deterministic_when_all_stochastic_features_are_off) {
    RenderOpts opt;
    opt.supersample = 1;
    opt.frames_to_render = 3;
    Frame first, last;
    ASSERT_NO_FATAL_FAILURE(Render(BuildEdgeScene, ChFrame<double>(), kW, kH, kHFOV, opt, &last, &first));

    long diff = 0;
    double maxdiff = 0;
    for (size_t k = 0; k < last.lum.size(); ++k) {
        const double e = std::fabs(last.lum[k] - first.lum[k]);
        if (e > 0) {
            diff++;
            maxdiff = std::max(maxdiff, e);
        }
    }
    std::cout << "  [determinism] frame 1 vs frame 3: " << diff << " differing px, max delta " << maxdiff << "\n";
    EXPECT_EQ(diff, 0) << "the same static scene rendered differently on two frames; pixel-diff "
                          "regression testing (tier 0) relies on this being exact";
}

// =============================================================================
// 1. global illumination -- more samples, less variance, same mean
// =============================================================================
TEST(MetalStochastic, gi_supersampling_reduces_noise) {
    // aa = clamp(supersample,1,4) and spp = aa*aa, so 1 -> 1 sample, 4 -> 16 samples.
    // The ideal-generator variance ratio for 16 independent samples is 1/4; requiring only
    // < 0.75 keeps the test insensitive to how the shader distributes its samples while
    // still failing loudly if supersampling reduces no noise at all (ratio -> 1).
    // gamma = 1.0 (linear output) is essential, not cosmetic. The unbiasedness assertion below
    // compares two means, and pow(x, 1/2.2) is concave, so by Jensen's inequality a noisy
    // single-sample estimate encodes to a LOWER mean than a converged one even when the
    // underlying radiance estimator is perfectly unbiased. Measured on this scene that
    // artefact alone moved the 8-bit mean by 26 LSB. Reading linear radiance removes it, so
    // the test measures the renderer instead of the transfer curve.
    Frame f1, f16;
    RenderOpts o1;
    o1.use_gi = true;
    o1.supersample = 1;
    o1.gamma = 1.0f;
    RenderOpts o16;
    o16.use_gi = true;
    o16.supersample = 4;
    o16.gamma = 1.0f;
    ASSERT_NO_FATAL_FAILURE(Render(BuildSkylitWall, ChFrame<double>(), kW, kH, kHFOV, o1, &f1));
    ASSERT_NO_FATAL_FAILURE(Render(BuildSkylitWall, ChFrame<double>(), kW, kH, kHFOV, o16, &f16));

    double m1 = 0, s1 = 0, m16 = 0, s16 = 0;
    ASSERT_NO_FATAL_FAILURE(CentralStats(f1, &m1, &s1));
    ASSERT_NO_FATAL_FAILURE(CentralStats(f16, &m16, &s16));
    std::cout << "  [GI]  1 spp : mean " << m1 << ", spatial std " << s1 << "\n";
    std::cout << "  [GI] 16 spp : mean " << m16 << ", spatial std " << s16 << "\n";

    // Guard the premise: a noiseless single-sample render would make the ratio meaningless.
    ASSERT_GT(s1, 2.0) << "the 1-spp GI render is not noisy enough for this test to mean anything";

    const double ratio = s16 / s1;
    std::cout << "  [GI] std ratio " << ratio << " (ideal 0.25 for 16 independent samples)\n";
    EXPECT_LT(ratio, 0.75) << "supersampling did not reduce Monte Carlo noise (ratio " << ratio << "); the per-sample RNG state is most likely not advancing between samples";

    // More samples must reduce variance without moving the estimate. The tolerance is
    // derived, not chosen: each mean is itself an average of N pixels, so the standard
    // error of the difference is sqrt((s1^2+s16^2)/N); 4 sigma makes a spurious failure
    // very unlikely and rescales automatically with brightness and resolution.
    const double N = (kW / 2) * (kH / 2);
    const double tol = 4.0 * std::sqrt((s1 * s1 + s16 * s16) / N);
    std::cout << "  [GI] mean shift " << std::fabs(m16 - m1) << ", allowed " << tol << "\n";
    EXPECT_NEAR(m16, m1, tol) << "supersampling shifted the mean, which points at a biased rather "
                                 "than merely noisy estimator";
}

// =============================================================================
// 2. area lights -- a finite emitter must give a wider penumbra than a point one
// =============================================================================
TEST(MetalStochastic, area_light_penumbra_is_wider_than_point_light) {
    // 16 samples per pixel so the stochastic shadow ray is reasonably converged; the
    // assertion is about the WIDTH of the transition, which is similar triangles and does
    // not depend on the sampling pattern.
    RenderOpts opt;
    opt.supersample = 4;

    Frame hard, soft;
    ASSERT_NO_FATAL_FAILURE(Render([](ChSystemNSC& s, std::shared_ptr<ChSensorManager> m) { return BuildShadowScene(s, m, 0.0); }, ChFrame<double>(), kW, kH, kHFOV, opt, &hard));
    ASSERT_NO_FATAL_FAILURE(Render([](ChSystemNSC& s, std::shared_ptr<ChSensorManager> m) { return BuildShadowScene(s, m, 2.0); }, ChFrame<double>(), kW, kH, kHFOV, opt, &soft));

    // The shadow centre sits at wall y = 0, i.e. the middle column; the post itself images
    // well to the left of it. Scan a window around the centre only.
    const unsigned y0 = kH / 3, y1 = kH - kH / 3;
    const unsigned x0 = kW / 2 - 40, x1 = kW / 2 + 40;
    const auto ph = ColumnProfile(hard, y0, y1);
    const auto ps = ColumnProfile(soft, y0, y1);
    const double wh = TransitionWidth(ph, x0, x1);
    const double ws = TransitionWidth(ps, x0, x1);
    std::cout << "  [shadow] point-light 25-75% transition: " << wh << " px\n";
    std::cout << "  [shadow]  area-light 25-75% transition: " << ws << " px\n";

    EXPECT_LT(wh, 6.0) << "a point light produced a soft shadow edge (" << wh << " px)";
    EXPECT_GT(ws, 3.0 * std::max(wh, 1.0)) << "the area light's penumbra (" << ws << " px) is not meaningfully wider than the point light's " << wh
                                           << " px, so the emitter is being sampled as a point";
}

// =============================================================================
// 3. depth of field -- a finite aperture blurs, and its noise converges
// =============================================================================
TEST(MetalStochastic, depth_of_field_blurs_out_of_focus_edges) {
    // Focus on the far wall (40 m) so the near box at 3 m is well out of focus. The predicted
    // blur is pure thin-lens geometry: rays leave the aperture over a radius R and all aim at
    // the focal plane, so at distance d they are spread over R*(1 - d/focal) =
    // 0.15*(1 - 3/40) = 0.139 m, which at 3 m subtends 0.046 rad, i.e. ~7 px here. A pinhole
    // gives a one-pixel edge.
    //
    // The measurement is the WIDTH of the 25-75% transition of the row-averaged column
    // profile, not a gradient magnitude. A gradient would be dominated by lens-sampling
    // speckle (only 16 lens samples are available, since aa is clamped to 4) and would
    // actually go UP with the aperture open; averaging down the rows of a vertical edge
    // suppresses that speckle without touching the blur being measured.
    RenderOpts pin;
    pin.supersample = 4;
    RenderOpts dof;
    dof.supersample = 4;
    dof.aperture = 0.15f;
    dof.focal_dist = 40.f;

    Frame sharp, blurred;
    ASSERT_NO_FATAL_FAILURE(Render(BuildEdgeScene, ChFrame<double>(), kW, kH, kHFOV, pin, &sharp));
    ASSERT_NO_FATAL_FAILURE(Render(BuildEdgeScene, ChFrame<double>(), kW, kH, kHFOV, dof, &blurred));

    const unsigned y0 = kH / 4, y1 = kH - kH / 4;
    const unsigned x0 = kW / 2 - 25, x1 = kW / 2 + 25;
    const double ws = TransitionWidth(ColumnProfile(sharp, y0, y1), x0, x1);
    const double wb = TransitionWidth(ColumnProfile(blurred, y0, y1), x0, x1);
    std::cout << "  [DoF] 25-75% silhouette width: pinhole " << ws << " px, aperture 0.15 m " << wb << " px\n";

    EXPECT_LT(ws, 4.0) << "the pinhole render already has a soft silhouette (" << ws << " px)";
    EXPECT_GT(wb, 2.5 * std::max(ws, 1.0)) << "a 0.15 m aperture focused 37 m past the near box did not soften its silhouette (" << wb << " px vs " << ws << " px)";
}

TEST(MetalStochastic, depth_of_field_noise_falls_with_supersampling) {
    // The lens jitter is one random offset per sample, so at 1 spp the out-of-focus region
    // is not blurred but speckled. Averaging more samples must smooth it. Raggedness is
    // measured per row (a second difference), so the smooth ramp of a converged blur does
    // not register while sampling noise does.
    RenderOpts o1;
    o1.supersample = 1;
    o1.aperture = 0.10f;
    o1.focal_dist = 40.f;
    RenderOpts o4 = o1;
    o4.supersample = 4;

    Frame f1, f4;
    ASSERT_NO_FATAL_FAILURE(Render(BuildEdgeScene, ChFrame<double>(), kW, kH, kHFOV, o1, &f1));
    ASSERT_NO_FATAL_FAILURE(Render(BuildEdgeScene, ChFrame<double>(), kW, kH, kHFOV, o4, &f4));

    const unsigned y0 = kH / 3, y1 = kH - kH / 3;
    const double r1 = RowRoughness(f1, 0, kW, y0, y1);
    const double r4 = RowRoughness(f4, 0, kW, y0, y1);
    std::cout << "  [DoF]  1 spp row roughness " << r1 << "\n";
    std::cout << "  [DoF] 16 spp row roughness " << r4 << "\n";

    ASSERT_GT(r1, 0.5) << "the 1-spp aperture render is not speckled; is depth of field active?";
    EXPECT_LT(r4, 0.8 * r1) << "adding lens samples did not smooth the out-of-focus region";
}

// =============================================================================
// 4. sensor noise -- scales with sigma, leaves the mean alone
// =============================================================================
TEST(MetalStochastic, sensor_noise_scales_with_sigma_and_preserves_the_mean) {
    // A mid-grey wall keeps the noise away from the 0 and 255 clamps, so the additive model
    // is not distorted by clipping.
    auto build = [](ChSystemNSC& sys, std::shared_ptr<ChSensorManager> manager) {
        auto wall = chrono_types::make_shared<ChBodyEasyBox>(0.4, 60, 60, 1000, true, false);
        wall->SetPos({12, 0, 0});
        wall->SetFixed(true);
        sys.Add(wall);
        Matte(wall, ChColor(0.5f, 0.5f, 0.5f));
        Background bg;
        bg.mode = BackgroundMode::SOLID_COLOR;
        bg.color_zenith = {0.f, 0.f, 0.f};
        bg.color_horizon = {0.f, 0.f, 0.f};
        manager->scene->SetBackground(bg);
        manager->scene->SetAmbientLight({0.30f, 0.30f, 0.30f});
        manager->scene->AddDirectionalLight(ChColor(0.5f, 0.5f, 0.5f), 0.f, 0.9f);
        auto mount = chrono_types::make_shared<ChBody>();
        mount->SetFixed(true);
        sys.Add(mount);
        return mount;
    };

    const float sigmas[3] = {0.f, 0.02f, 0.05f};
    double mean[3] = {0, 0, 0}, sd[3] = {0, 0, 0};
    for (int i = 0; i < 3; ++i) {
        RenderOpts o;
        o.noise_sigma = sigmas[i];
        Frame f;
        ASSERT_NO_FATAL_FAILURE(Render(build, ChFrame<double>(), kW, kH, kHFOV, o, &f));
        ASSERT_NO_FATAL_FAILURE(CentralStats(f, &mean[i], &sd[i]));
        std::cout << "  [noise] sigma " << sigmas[i] << " : mean " << mean[i] << ", std " << sd[i] << "\n";
    }

    // sigma = 0 must give a flat frame: the surface is uniform and nothing else is random.
    EXPECT_LT(sd[0], 1.5) << "the noiseless render of a uniform wall is not uniform (std " << sd[0] << ")";

    // The shader adds an INDEPENDENT N(0, sigma) draw to each of R, G and B, in normalised
    // units, then quantises to 8 bit. The statistic above is the mean of the three channels,
    // whose standard deviation is therefore sigma*255/sqrt(3), not sigma*255. Predicting the
    // sqrt(3) instead of hiding it in a loose bound is what lets the band be tight: anything
    // outside 0.8x-1.25x of the prediction is a real defect in the noise model.
    for (int i = 1; i < 3; ++i) {
        const double expect = sigmas[i] * 255.0 / std::sqrt(3.0);
        std::cout << "  [noise] sigma " << sigmas[i] << " : predicted luminance std " << expect << " (= sigma*255/sqrt(3)"
                  << ", three independent channels averaged)\n";
        EXPECT_GT(sd[i], 0.80 * expect) << "sigma " << sigmas[i] << " produced too little noise";
        EXPECT_LT(sd[i], 1.25 * expect) << "sigma " << sigmas[i] << " produced too much noise";
    }
    EXPECT_GT(sd[2], sd[1]) << "noise did not grow with sigma";

    // Zero-mean noise must not shift the image. Allowed shift is 4 standard errors of the
    // mean of the noisier frame.
    const double N = (kW / 2) * (kH / 2);
    for (int i = 1; i < 3; ++i) {
        const double tol = 4.0 * sd[i] / std::sqrt(N) + 1.0;
        std::cout << "  [noise] sigma " << sigmas[i] << " mean shift " << std::fabs(mean[i] - mean[0]) << ", allowed " << tol << "\n";
        EXPECT_NEAR(mean[i], mean[0], tol) << "sensor noise is not zero-mean";
    }
}
