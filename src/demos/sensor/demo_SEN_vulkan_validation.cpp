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
// 1:1 validation of the experimental Vulkan RT camera against the OptiX camera.
//
// Requires a build with BOTH backends enabled (CH_USE_SENSOR_OPTIX=ON and
// CH_USE_SENSOR_VULKAN_RT=ON), i.e. an NVIDIA GPU that also exposes Vulkan RT.
// One deterministic scene is rendered by an OptiX ChCameraSensor and a Vulkan
// ChVulkanCameraSensor placed at the SAME pose, then compared with scalar error
// metrics (per-channel and pooled RMSE / MAE / max-abs / bias / PSNR, plus the
// fraction of differing pixels). It writes the reference image, the candidate
// image, and amplified difference images so discrepancies can be localized.
//
// SCENE AND SETTINGS follow the baseline agreed with Florian Reinle (OTEC), who
// wrote the Vulkan RT backend. The point is to isolate camera projection,
// geometry, normals, material color, direct BRDF evaluation, cast shadows, and
// scene synchronization, with nothing else in the way:
//
//   - large matte gray floor plus two boxes, as ONE explicit triangle mesh with
//     explicit per-face normals (the primitive path is avoided on purpose, see
//     the note on ChBodyEasyBox below);
//   - opaque material, roughness 1.0, metallic 0.0, no textures, no emissive;
//   - fixed camera, black background, zero ambient light;
//   - a single white point light at a known position;
//   - OptiX LEGACY (direct-lighting) integrator, GI off, denoiser off, pinhole
//     lens, one sample per pixel, gamma 1.0 (metrics in linear color space),
//     no fog / noise / motion blur / post-processing.
//
// Run 1 (default) uses const_color = true, so there is no distance attenuation:
// it validates everything except falloff. Run 2 (--falloff) repeats the exact
// same scene with const_color = false, which isolates the inverse-square
// attenuation. The two boxes sit at deliberately different distances from the
// light so run 2 has a measurable gradient.
//
// NOTE on geometry choice: an explicit mesh keeps this demo self-contained, needing
// no Chrono data directory on the machine running the comparison, and it sidesteps a
// separate defect in the OptiX primitive path. ChOptixPipeline::GetBoxMaterial sets
// num_blended_materials = mat_list.size(), which is 0 for a box carrying no explicit
// ChVisualMaterial, and every shading loop in the OptiX shaders is
// "for (b = 0; b < num_blended_materials; b++)", so such a surface accumulates no
// color and renders pure black. GetSphereMaterial and GetNVDBMaterial set the count
// the same way; GetCylinderMaterial and GetMeshMaterial hardcode 1 and are immune.
// Run with --probe-box-material to see it: two identical boxes, one with a material
// and one without, where OptiX blacks out the second and Vulkan shades it correctly.
//
// =============================================================================

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/core/ChDataPath.h"

#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/utils/ChSensorImageCompare.h"

#ifdef CHRONO_HAS_OPTIX
    #include "chrono_sensor/sensors/ChCameraSensor.h"
#endif
#ifdef CHRONO_HAS_VULKAN_RT
    #include "chrono_sensor/sensors/ChVulkanCameraSensor.h"
#endif

using namespace chrono;
using namespace chrono::sensor;

// Vulkan RT is required. OptiX is OPTIONAL: with both backends this does the 1:1 comparison,
// with Vulkan alone it renders and saves the Vulkan image only, which is what makes the
// cross-GPU check possible on an AMD machine that has no OptiX at all. The saved PPMs are
// then diffed offline against the ones produced on the NVIDIA host.
#ifdef CHRONO_HAS_VULKAN_RT

// -----------------------------------------------------------------------------
// Comparison parameters (identical on both cameras)
// -----------------------------------------------------------------------------
unsigned int image_width = 1280;
unsigned int image_height = 720;
float fov = (float)CH_PI_3;       // horizontal field of view
unsigned int supersample = 1;     // PER-AXIS factor, so total samples per pixel is its square.
                                  // 1 by default, per the agreed baseline. Raise with --spp to let
                                  // the OptiX stochastic environment light converge.
float gamma_c = 1.0f;             // linear color space, so metrics are not gamma-warped
unsigned int diff_threshold = 2;  // per-channel code-unit threshold for "differing" pixels

// -----------------------------------------------------------------------------
// Scene constants (all positions are known and fixed, so the run is repeatable)
// -----------------------------------------------------------------------------
const double floor_half_extent = 20.0;               // floor spans [-20, 20] in x and y
const ChVector3d box_a_center(-2.0, 1.5, 0.75);      // near box, closer to the light
const ChVector3d box_b_center(2.0, -1.5, 0.75);      // far box, ~1.6x the light distance
const double box_half_size = 0.75;                   // 1.5 m cubes resting on the floor
// The light sits high, behind the camera, and off to one side: behind so the box faces the
// camera sees are the lit ones, off to the side so the cast shadows stretch across the floor
// into view rather than hiding behind the boxes.
const ChVector3f light_pos(-7.0f, 4.0f, 5.0f);       // single white point light
const ChColor light_color(1.0f, 1.0f, 1.0f);
// atten_scale is 0.01 * max_range^2 on both backends, so max_range = 80 gives atten_scale 64,
// putting the inverse-square factor near unity at this scene's ~8 m light distances.
const float light_max_range = 80.0f;
const ChVector3d camera_pos(-9.0, 0.0, 3.0);         // fixed camera, looking down +x
// Pitch the camera 15 degrees down so the floor, the boxes, and their cast shadows fill most of
// the frame. Level with the horizon, roughly three quarters of the image would be empty
// background, which would let a large block of trivially-matching black pixels dominate the
// metrics. A strip of background is still in view on purpose, to exercise the background path.
const double camera_pitch_down = 15.0 * CH_PI / 180.0;
const float surface_gray = 0.5f;                     // matte gray albedo, floor and boxes alike

// Write an RGBA8 buffer as a binary PPM (P6). Dependency-free and universally
// convertible to PNG; avoids any stb linkage concerns from a demo translation unit.
// Sensor image buffers are stored bottom row first, which is why ChFilterSave calls
// stbi_flip_vertically_on_write(1). PPM is top row first, so the rows are emitted in
// reverse here; without that the saved images come out upside down.
static bool WritePPM(const std::string& path, const UserRGBA8BufferPtr& img) {
    if (!img || !img->Buffer)
        return false;
    std::ofstream f(path, std::ios::binary);
    if (!f)
        return false;
    const unsigned int w = img->Width;
    const unsigned int h = img->Height;
    f << "P6\n" << w << " " << h << "\n255\n";
    const PixelRGBA8* p = img->Buffer.get();
    std::vector<unsigned char> rgb(3 * (size_t)w * h);
    for (unsigned int row = 0; row < h; ++row) {
        const PixelRGBA8* src = p + (size_t)(h - 1 - row) * w;
        unsigned char* dst = rgb.data() + 3 * (size_t)row * w;
        for (unsigned int col = 0; col < w; ++col) {
            dst[3 * (size_t)col + 0] = src[col].R;
            dst[3 * (size_t)col + 1] = src[col].G;
            dst[3 * (size_t)col + 2] = src[col].B;
        }
    }
    f.write(reinterpret_cast<const char*>(rgb.data()), (std::streamsize)rgb.size());
    return (bool)f;
}

// Append one flat quad (v0,v1,v2,v3 counter-clockwise as seen from outside) to the
// mesh as two triangles. Vertices and normals are duplicated per quad so every face
// carries an exact, unambiguous flat normal, with no smoothing across edges.
static void AddQuad(std::shared_ptr<ChTriangleMeshConnected> mesh,
                    const ChVector3d& v0,
                    const ChVector3d& v1,
                    const ChVector3d& v2,
                    const ChVector3d& v3,
                    const ChVector3d& normal) {
    auto& vertices = mesh->GetCoordsVertices();
    auto& normals = mesh->GetCoordsNormals();
    auto& uvs = mesh->GetCoordsUV();
    auto& v_idx = mesh->GetIndicesVertices();
    auto& n_idx = mesh->GetIndicesNormals();
    auto& uv_idx = mesh->GetIndicesUV();
    auto& mat_idx = mesh->GetIndicesMaterials();

    const int base = (int)vertices.size();

    vertices.push_back(v0);
    vertices.push_back(v1);
    vertices.push_back(v2);
    vertices.push_back(v3);
    for (int i = 0; i < 4; ++i)
        normals.push_back(normal);
    uvs.push_back(ChVector2d(0, 0));
    uvs.push_back(ChVector2d(1, 0));
    uvs.push_back(ChVector2d(1, 1));
    uvs.push_back(ChVector2d(0, 1));

    const ChVector3i t0(base + 0, base + 1, base + 2);
    const ChVector3i t1(base + 0, base + 2, base + 3);
    v_idx.push_back(t0);
    v_idx.push_back(t1);
    n_idx.push_back(t0);
    n_idx.push_back(t1);
    uv_idx.push_back(t0);
    uv_idx.push_back(t1);
    mat_idx.push_back(0);
    mat_idx.push_back(0);
}

// Build a standalone fixed body holding a single cube mesh with one explicit material.
// Used by the material sweep, where each sample needs its own material record rather than
// sharing the scene's single gray one.
static std::shared_ptr<ChBody> MakeCubeBody(ChSystemNSC& sys,
                                            const ChVector3d& center,
                                            double half,
                                            float roughness,
                                            float metallic,
                                            const ChColor& diffuse,
                                            const ChColor& specular,
                                            bool specular_workflow);

// Append an axis-aligned cube, all six faces with outward normals.
static void AddCube(std::shared_ptr<ChTriangleMeshConnected> mesh, const ChVector3d& c, double h) {
    const double xm = c.x() - h, xp = c.x() + h;
    const double ym = c.y() - h, yp = c.y() + h;
    const double zm = c.z() - h, zp = c.z() + h;

    AddQuad(mesh, {xp, ym, zm}, {xp, yp, zm}, {xp, yp, zp}, {xp, ym, zp}, {1, 0, 0});
    AddQuad(mesh, {xm, yp, zm}, {xm, ym, zm}, {xm, ym, zp}, {xm, yp, zp}, {-1, 0, 0});
    AddQuad(mesh, {xp, yp, zm}, {xm, yp, zm}, {xm, yp, zp}, {xp, yp, zp}, {0, 1, 0});
    AddQuad(mesh, {xm, ym, zm}, {xp, ym, zm}, {xp, ym, zp}, {xm, ym, zp}, {0, -1, 0});
    AddQuad(mesh, {xm, ym, zp}, {xp, ym, zp}, {xp, yp, zp}, {xm, yp, zp}, {0, 0, 1});
    AddQuad(mesh, {xm, yp, zm}, {xp, yp, zm}, {xp, ym, zm}, {xm, ym, zm}, {0, 0, -1});
}

static std::shared_ptr<ChBody> MakeCubeBody(ChSystemNSC& sys,
                                            const ChVector3d& center,
                                            double half,
                                            float roughness,
                                            float metallic,
                                            const ChColor& diffuse,
                                            const ChColor& specular,
                                            bool specular_workflow) {
    auto m = chrono_types::make_shared<ChTriangleMeshConnected>();
    AddCube(m, ChVector3d(0, 0, 0), half);  // centered at the body origin; the body carries the pose

    auto shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    shape->SetMesh(m);
    shape->SetMutable(false);

    auto mat = chrono_types::make_shared<ChVisualMaterial>();
    mat->SetDiffuseColor({diffuse.R, diffuse.G, diffuse.B});
    mat->SetSpecularColor({specular.R, specular.G, specular.B});
    mat->SetRoughness(roughness);
    mat->SetMetallic(metallic);
    mat->SetOpacity(1.0f);
    mat->SetUseSpecularWorkflow(specular_workflow);
    shape->AddMaterial(mat);

    auto body = chrono_types::make_shared<ChBody>();
    body->SetPos(center);
    body->SetFixed(true);
    body->AddVisualShape(shape);
    sys.Add(body);
    return body;
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;
    std::cout << "Vulkan RT vs OptiX camera 1:1 validation\n" << std::endl;

    // Run 1 (default): const_color = true, no distance attenuation.
    // Run 2 (--falloff): const_color = false, inverse-square attenuation active.
    // --light selects the light type, following the agreed progression: point first, then
    // directional and spot. Directional has no position and no distance attenuation, so
    // --falloff is meaningless for it and is rejected rather than silently ignored.
    enum class LightKind { POINT, DIRECTIONAL, SPOT };
    LightKind light_kind = LightKind::POINT;
    bool const_color = true;
    bool probe_box_material = false;
    bool material_sweep = false;
    int material_index = -1;  // >= 0 isolates a single material sample (floor + that cube only)
    std::string feature;      // mesh | texture | normalmap | opacity | envmap (Florian steps c-f)
    std::string out_dir = "SENSOR_OUTPUT/vulkan_validation/";
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--falloff") == 0) {
            const_color = false;
        } else if (std::strcmp(argv[i], "--probe-box-material") == 0) {
            probe_box_material = true;
        } else if (std::strcmp(argv[i], "--materials") == 0) {
            material_sweep = true;
        } else if (std::strcmp(argv[i], "--material-index") == 0 && i + 1 < argc) {
            material_sweep = true;
            material_index = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--spp") == 0 && i + 1 < argc) {
            const int f = std::atoi(argv[++i]);
            if (f < 1 || f > 32) {
                std::cerr << "--spp takes a per-axis factor in 1..32 (total samples is its square)\n";
                return 1;
            }
            supersample = (unsigned int)f;
        } else if (std::strcmp(argv[i], "--feature") == 0 && i + 1 < argc) {
            feature = argv[++i];
            if (feature != "mesh" && feature != "texture" && feature != "normalmap" &&
                feature != "opacity" && feature != "envmap") {
                std::cerr << "unknown --feature: " << feature
                          << " (use mesh, texture, normalmap, opacity, or envmap)\n";
                return 1;
            }
        } else if (std::strcmp(argv[i], "--light") == 0 && i + 1 < argc) {
            const std::string k = argv[++i];
            if (k == "point")
                light_kind = LightKind::POINT;
            else if (k == "directional")
                light_kind = LightKind::DIRECTIONAL;
            else if (k == "spot")
                light_kind = LightKind::SPOT;
            else {
                std::cerr << "unknown --light value: " << k << " (use point, directional, or spot)\n";
                return 1;
            }
        } else if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            out_dir = argv[++i];
            if (!out_dir.empty() && out_dir.back() != '/')
                out_dir += '/';
        } else {
            std::cout << "usage: demo_SEN_vulkan_validation [--light point|directional|spot] [--falloff]\n"
                      << "                                  [--probe-box-material] [--out <dir>]\n"
                      << "  --light KIND         light type to compare (default point)\n"
                      << "  --falloff            const_color = false, enabling inverse-square distance\n"
                      << "                       attenuation (point and spot only)\n"
                      << "  --probe-box-material diagnostic: add ChBodyEasyBox primitives with and\n"
                      << "                       without an explicit material\n"
                      << "  --out DIR            output directory (default SENSOR_OUTPUT/vulkan_validation/)\n";
            return 1;
        }
    }
    if (light_kind == LightKind::DIRECTIONAL && !const_color) {
        std::cerr << "--falloff is not meaningful for a directional light: it has no position, so there\n"
                  << "is no distance to attenuate over. Re-run without --falloff.\n";
        return 1;
    }

    const char* kind_name = (light_kind == LightKind::POINT)         ? "point"
                            : (light_kind == LightKind::DIRECTIONAL) ? "directional"
                                                                     : "spot";
    std::string tag = std::string("light_") + kind_name + (const_color ? "_constcolor" : "_falloff");
    if (light_kind == LightKind::POINT)  // keep the original baseline filenames stable
        tag = const_color ? "run1_constcolor" : "run2_falloff";
    if (material_sweep)
        tag = std::string("materials_") + kind_name + (const_color ? "_constcolor" : "_falloff");
    if (material_index >= 0)
        tag = std::string("mat") + std::to_string(material_index) + "_" + kind_name +
              (const_color ? "_constcolor" : "_falloff");
    if (!feature.empty())
        tag = std::string("feat_") + feature + "_" + kind_name + (const_color ? "_constcolor" : "_falloff");
    if (supersample != 1)
        tag += "_spp" + std::to_string(supersample * supersample);
    std::cout << "Samples per pixel: " << (supersample * supersample) << " (per-axis factor "
              << supersample << ")\n";
    std::cout << "Configuration: " << tag << "  (light = " << kind_name
              << ", const_color = " << (const_color ? "true" : "false") << ")\n";

    // ------------------------------------------------------------------
    // Scene: one explicit triangle mesh holding the floor and two boxes.
    // ------------------------------------------------------------------
    ChSystemNSC sys;

    auto mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    const double L = floor_half_extent;
    AddQuad(mesh, {-L, -L, 0}, {L, -L, 0}, {L, L, 0}, {-L, L, 0}, {0, 0, 1});
    // When isolating one material sample, the scene is floor + that one cube, so nothing else
    // contributes to the metrics.
    if (material_index < 0 && feature.empty()) {
        AddCube(mesh, box_a_center, box_half_size);
        AddCube(mesh, box_b_center, box_half_size);
    }

    auto mesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    mesh_shape->SetMesh(mesh);
    mesh_shape->SetName("validation_scene");
    mesh_shape->SetMutable(false);

    // A single matte, fully opaque, non-metallic gray material for every surface.
    auto mat = chrono_types::make_shared<ChVisualMaterial>();
    mat->SetDiffuseColor({surface_gray, surface_gray, surface_gray});
    mat->SetSpecularColor({0.0f, 0.0f, 0.0f});
    mat->SetRoughness(1.0f);
    mat->SetMetallic(0.0f);
    mat->SetOpacity(1.0f);
    mesh_shape->AddMaterial(mat);

    auto scene_body = chrono_types::make_shared<ChBody>();
    scene_body->SetPos({0, 0, 0});
    scene_body->SetFixed(true);
    scene_body->AddVisualShape(mesh_shape);
    sys.Add(scene_body);

    std::cout << "Scene: " << mesh->GetIndicesVertices().size() << " triangles, "
              << mesh->GetCoordsVertices().size() << " vertices (floor + 2 boxes)\n";

    // Diagnostic probe, off by default. Adds two ChBodyEasyBox primitives side by side, the LEFT
    // one carrying an explicit ChVisualMaterial and the RIGHT one carrying none. It exists to test
    // one question: does the OptiX box path shade a primitive that has no explicit material?
    // ChOptixPipeline::GetBoxMaterial sets num_blended_materials = mat_list.size(), which is 0 in
    // the no-material case, and every OptiX shading loop iterates over that count.
    if (probe_box_material) {
        auto probe_with = chrono_types::make_shared<ChBodyEasyBox>(1.5, 1.5, 1.5, 1000, true, false);
        probe_with->SetPos({-4.5, 3.2, 0.75});
        probe_with->SetFixed(true);
        sys.Add(probe_with);
        auto probe_mat = chrono_types::make_shared<ChVisualMaterial>();
        probe_mat->SetDiffuseColor({surface_gray, surface_gray, surface_gray});
        probe_mat->SetSpecularColor({0.0f, 0.0f, 0.0f});
        probe_mat->SetRoughness(1.0f);
        probe_mat->SetMetallic(0.0f);
        probe_with->GetVisualModel()->GetShapeInstances()[0].shape->AddMaterial(probe_mat);

        auto probe_without = chrono_types::make_shared<ChBodyEasyBox>(1.5, 1.5, 1.5, 1000, true, false);
        probe_without->SetPos({-4.5, 0.6, 0.75});
        probe_without->SetFixed(true);
        sys.Add(probe_without);

        std::cout << "PROBE: two ChBodyEasyBox primitives added. Left (y=+3.2) has an explicit\n"
                  << "       material, right (y=+0.6) has none. Material counts on the shapes: "
                  << probe_with->GetVisualModel()->GetShapeInstances()[0].shape->GetNumMaterials() << " and "
                  << probe_without->GetVisualModel()->GetShapeInstances()[0].shape->GetNumMaterials() << ".\n";
    }

    // Material sweep (Florian's step b): a row of cubes spanning roughness and metallic, plus two
    // specular-workflow samples. This is where the two backends are most likely to genuinely
    // diverge, since each implements its own microfacet model rather than sharing one.
    // The cubes sit in front of the two baseline boxes so the floor and shadows stay in frame.
    if (material_sweep) {
        struct Sample {
            const char* label;
            float roughness;
            float metallic;
            bool spec_workflow;
        };
        const Sample samples[] = {
            {"rough 1.0 / metal 0.0", 1.00f, 0.0f, false},  // the baseline material, as a control
            {"rough 0.6 / metal 0.0", 0.60f, 0.0f, false},
            {"rough 0.3 / metal 0.0", 0.30f, 0.0f, false},
            {"rough 0.3 / metal 1.0", 0.30f, 1.0f, false},
            {"rough 0.1 / metal 1.0", 0.10f, 1.0f, false},
            {"specular workflow", 0.40f, 0.0f, true},
        };
        const int n_samples = (int)(sizeof(samples) / sizeof(samples[0]));
        if (material_index >= n_samples) {
            std::cerr << "--material-index must be 0.." << (n_samples - 1) << "\n";
            return 1;
        }
        if (material_index >= 0) {
            // One sample, centered in view, so the reported metrics describe that material alone.
            const Sample& s = samples[material_index];
            MakeCubeBody(sys, ChVector3d(-2.0, 0.0, 0.9), 0.9, s.roughness, s.metallic,
                         ChColor(surface_gray, surface_gray, surface_gray), ChColor(0.5f, 0.5f, 0.5f),
                         s.spec_workflow);
            std::cout << "MATERIAL ISOLATION: index " << material_index << " = " << s.label
                      << "  (roughness " << s.roughness << ", metallic " << s.metallic
                      << ", specular_workflow " << (s.spec_workflow ? "true" : "false") << ")\n";
        } else {
            const double spacing = 1.6;
            const double y0 = -((n_samples - 1) * spacing) / 2.0;
            std::cout << "MATERIAL SWEEP: " << n_samples << " cubes at x=-4.5, y from " << y0 << " to "
                      << (y0 + (n_samples - 1) * spacing) << ":\n";
            for (int i = 0; i < n_samples; ++i) {
                MakeCubeBody(sys, ChVector3d(-4.5, y0 + i * spacing, 0.55), 0.55, samples[i].roughness,
                             samples[i].metallic, ChColor(surface_gray, surface_gray, surface_gray),
                             ChColor(0.5f, 0.5f, 0.5f), samples[i].spec_workflow);
                std::cout << "   y=" << (y0 + i * spacing) << "  " << samples[i].label
                          << (samples[i].spec_workflow ? "  (use_specular_workflow=true)" : "") << "\n";
            }
        }
    }

    // Florian's steps c to f, one feature at a time on an otherwise minimal scene
    // (floor plus one object), so the metrics are attributable to that feature alone.
    if (!feature.empty()) {
        if (feature == "mesh") {
            // Step c: an IMPORTED Wavefront mesh carrying its own materials from the .mtl,
            // as opposed to the programmatic mesh + programmatic material used everywhere else.
            auto imported = ChTriangleMeshConnected::CreateFromWavefrontFile(
                GetChronoDataFile("sensor/geometries/box.obj"), true, true);
            auto shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            shape->SetMesh(imported);
            shape->SetName("imported_box");
            shape->SetMutable(false);
            auto body = chrono_types::make_shared<ChBody>();
            body->SetPos({-1.0, 0.0, 1.2});
            body->SetFixed(true);
            body->AddVisualShape(shape);
            sys.Add(body);
            std::cout << "FEATURE mesh: imported sensor/geometries/box.obj, "
                      << imported->GetIndicesVertices().size() << " triangles, "
                      << shape->GetNumMaterials() << " material(s) from the .mtl\n";
        } else if (feature == "texture" || feature == "normalmap") {
            // Steps d: a diffuse texture, and separately a normal map on top of one.
            auto m = chrono_types::make_shared<ChTriangleMeshConnected>();
            AddCube(m, ChVector3d(0, 0, 0), 0.9);
            auto shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            shape->SetMesh(m);
            shape->SetMutable(false);
            auto mat = chrono_types::make_shared<ChVisualMaterial>();
            mat->SetDiffuseColor({1.0f, 1.0f, 1.0f});
            mat->SetSpecularColor({0.0f, 0.0f, 0.0f});
            mat->SetRoughness(1.0f);
            mat->SetMetallic(0.0f);
            if (feature == "texture") {
                mat->SetKdTexture(GetChronoDataFile("sensor/textures/checkerboard.png"));
                std::cout << "FEATURE texture: diffuse checkerboard.png on a 1.8 m cube\n";
            } else {
                mat->SetKdTexture(GetChronoDataFile("sensor/textures/brick.png"));
                mat->SetNormalMapTexture(GetChronoDataFile("sensor/textures/brick_normal.png"));
                std::cout << "FEATURE normalmap: brick.png diffuse + brick_normal.png normal map\n";
            }
            shape->AddMaterial(mat);
            auto body = chrono_types::make_shared<ChBody>();
            body->SetPos({-2.0, 0.0, 0.9});
            body->SetFixed(true);
            body->AddVisualShape(shape);
            sys.Add(body);
        } else if (feature == "opacity") {
            // Step e: partial opacity, which routes through the transmission path on both sides.
            auto m = chrono_types::make_shared<ChTriangleMeshConnected>();
            AddCube(m, ChVector3d(0, 0, 0), 0.9);
            auto shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            shape->SetMesh(m);
            shape->SetMutable(false);
            auto mat = chrono_types::make_shared<ChVisualMaterial>();
            mat->SetDiffuseColor({surface_gray, surface_gray, surface_gray});
            mat->SetSpecularColor({0.0f, 0.0f, 0.0f});
            mat->SetRoughness(1.0f);
            mat->SetMetallic(0.0f);
            mat->SetOpacity(0.5f);
            shape->AddMaterial(mat);
            auto body = chrono_types::make_shared<ChBody>();
            body->SetPos({-2.0, 0.0, 0.9});
            body->SetFixed(true);
            body->AddVisualShape(shape);
            sys.Add(body);
            // A second, fully opaque cube behind it, so transmission has something to show through.
            MakeCubeBody(sys, ChVector3d(1.5, 0.0, 0.9), 0.9, 1.0f, 0.0f,
                         ChColor(surface_gray, surface_gray, surface_gray), ChColor(0.f, 0.f, 0.f), false);
            std::cout << "FEATURE opacity: front cube opacity 0.5, opaque cube behind it\n";
        } else if (feature == "envmap") {
            MakeCubeBody(sys, ChVector3d(-2.0, 0.0, 0.9), 0.9, 1.0f, 0.0f,
                         ChColor(surface_gray, surface_gray, surface_gray), ChColor(0.f, 0.f, 0.f), false);
            std::cout << "FEATURE envmap: environment lighting replaces the point light\n";
        }
    }

    // ------------------------------------------------------------------
    // Sensor manager: zero ambient, black background, one point light.
    // ------------------------------------------------------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->SetRayRecursions(4);
    manager->scene->SetAmbientLight({0.f, 0.f, 0.f});

    Background background;
    background.mode = BackgroundMode::SOLID_COLOR;
    background.color_zenith = {0, 0, 0};
    background.color_horizon = {0, 0, 0};
    manager->scene->SetBackground(background);

    const auto dist_to = [&](const ChVector3d& p) {
        return (p - ChVector3d(light_pos.x(), light_pos.y(), light_pos.z())).Length();
    };

    if (feature == "envmap") {
        // Step f (first half): image-based lighting. The background becomes the env map and the
        // only light is the environment itself, so this exercises a completely different path
        // from the analytic point/spot/directional lights above.
        const std::string env = GetChronoDataFile("sensor/textures/quarry_01_4k.hdr");
        Background env_bg;
        env_bg.mode = BackgroundMode::ENVIRONMENT_MAP;
        env_bg.env_tex = env;
        manager->scene->SetBackground(env_bg);
        manager->scene->AddEnvironmentLight(env, 1.0f);
        std::cout << "Environment light from " << env << ", intensity_scale 1.0\n";
    } else
    switch (light_kind) {
        case LightKind::POINT: {
            manager->scene->AddPointLight(light_pos, light_color, light_max_range, const_color);
            std::cout << "Point light at (" << light_pos.x() << ", " << light_pos.y() << ", " << light_pos.z()
                      << "), max_range " << light_max_range << ", atten_scale "
                      << 0.01f * light_max_range * light_max_range << "\n";
            std::cout << "Light-to-box distances: box A " << dist_to(box_a_center) << " m, box B "
                      << dist_to(box_b_center) << " m\n";
            break;
        }
        case LightKind::DIRECTIONAL: {
            // AddDirectionalLight takes elevation and azimuth, not a vector, and builds
            // light_dir = (cos(el)cos(az), cos(el)sin(az), sin(el)), the unit vector from the hit
            // point toward the light. These angles are chosen to point back along the same
            // direction the point light sits in, so the shading is comparable to run 1.
            const float elevation = 30.0f * (float)CH_PI / 180.0f;
            const float azimuth = 150.0f * (float)CH_PI / 180.0f;
            manager->scene->AddDirectionalLight(light_color, elevation, azimuth);
            std::cout << "Directional light: elevation 30 deg, azimuth 150 deg, i.e. light_dir ("
                      << std::cos(elevation) * std::cos(azimuth) << ", " << std::cos(elevation) * std::sin(azimuth)
                      << ", " << std::sin(elevation) << ")\n";
            std::cout << "No distance attenuation applies to a directional light.\n";
            break;
        }
        case LightKind::SPOT: {
            // Spot placed where the point light was, aimed at the origin. angle_range is the FULL
            // cone angle (the shaders compare 2*angle_from_axis against it), and angle_falloff_start
            // is where the soft edge begins, so a 40 to 60 degree band gives a visible penumbra that
            // exercises the angular attenuation rather than a hard binary cutoff.
            const ChVector3d aim = ChVector3d(0, 0, 0) - ChVector3d(light_pos.x(), light_pos.y(), light_pos.z());
            const ChVector3f spot_dir((float)aim.x(), (float)aim.y(), (float)aim.z());
            const float angle_falloff_start = 40.0f * (float)CH_PI / 180.0f;
            const float angle_range = 60.0f * (float)CH_PI / 180.0f;
            manager->scene->AddSpotLight(light_pos, light_color, light_max_range, spot_dir, angle_falloff_start,
                                         angle_range, const_color);
            std::cout << "Spot light at (" << light_pos.x() << ", " << light_pos.y() << ", " << light_pos.z()
                      << ") aimed at the origin, full cone 60 deg, falloff starts at 40 deg, max_range "
                      << light_max_range << "\n";
            std::cout << "Light-to-box distances: box A " << dist_to(box_a_center) << " m, box B "
                      << dist_to(box_b_center) << " m\n";
            break;
        }
    }

    // Both cameras share the identical pose, resolution, FOV, gamma, supersample.
    ChFrame<double> cam_pose(camera_pos, QuatFromAngleY(camera_pitch_down));
    const float update_rate = 30.f;

    // ------------------------------------------------------------------
    // OptiX reference camera: LEGACY direct lighting, GI off, denoiser off.
    // Only when this build has OptiX.
    // ------------------------------------------------------------------
#ifdef CHRONO_HAS_OPTIX
    auto cam_optix = chrono_types::make_shared<ChCameraSensor>(scene_body,                    // parent body
                                                               update_rate,                   // update rate [Hz]
                                                               cam_pose,                      // offset pose
                                                               image_width,                   // width
                                                               image_height,                  // height
                                                               fov,                           // horizontal FOV
                                                               supersample,                   // supersample factor
                                                               CameraLensModelType::PINHOLE,  // lens model
                                                               false,   // use_diffuse_reflect (GI) off
                                                               false,   // denoiser off
                                                               Integrator::LEGACY,  // direct lighting
                                                               gamma_c,             // gamma (1.0, linear)
                                                               false);              // fog off
    cam_optix->SetName("OptiX reference camera");
    cam_optix->SetLag(0.f);
    cam_optix->SetCollectionWindow(0.f);
    cam_optix->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam_optix);
#endif

    // ------------------------------------------------------------------
    // Vulkan RT camera at the identical pose / resolution / FOV / gamma.
    // ------------------------------------------------------------------
    auto cam_vulkan = chrono_types::make_shared<ChVulkanCameraSensor>(scene_body,    // parent body
                                                                      update_rate,   // update rate [Hz]
                                                                      cam_pose,      // offset pose
                                                                      image_width,   // width
                                                                      image_height,  // height
                                                                      fov,           // horizontal FOV
                                                                      supersample,   // supersample factor
                                                                      gamma_c);      // gamma
    cam_vulkan->SetName("Vulkan RT camera");
    cam_vulkan->SetLag(0.f);
    cam_vulkan->SetCollectionWindow(0.f);
    cam_vulkan->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam_vulkan);

    // ------------------------------------------------------------------
    // Step until both cameras have delivered a frame. The scene is static,
    // so any completed frame is representative.
    // ------------------------------------------------------------------
    const double step_size = 1e-2;
    const int max_steps = 2000;
    UserRGBA8BufferPtr optix_buf, vulkan_buf;

    for (int i = 0; i < max_steps; ++i) {
        manager->Update();
        sys.DoStepDynamics(step_size);

        vulkan_buf = cam_vulkan->GetMostRecentBuffer<UserRGBA8BufferPtr>();
        const bool vulkan_ready = vulkan_buf && vulkan_buf->Buffer && vulkan_buf->LaunchedCount > 0;
#ifdef CHRONO_HAS_OPTIX
        optix_buf = cam_optix->GetMostRecentBuffer<UserRGBA8BufferPtr>();
        const bool optix_ready = optix_buf && optix_buf->Buffer && optix_buf->LaunchedCount > 0;
#else
        const bool optix_ready = true;  // no OptiX in this build; nothing to wait for
#endif
        if (optix_ready && vulkan_ready) {
            std::cout << "Frame(s) rendered at step " << i << " (t=" << sys.GetChTime() << " s)\n";
            break;
        }
    }

    if (!vulkan_buf || !vulkan_buf->Buffer) {
        std::cerr << "ERROR: Vulkan camera produced no frame.\n";
        return 1;
    }
#ifdef CHRONO_HAS_OPTIX
    if (!optix_buf || !optix_buf->Buffer) {
        std::cerr << "ERROR: OptiX camera produced no frame.\n";
        return 1;
    }
#endif

    // ------------------------------------------------------------------
    // Report the mean level of each image before diffing them. A pair of
    // means that differ wildly (or a mean near zero) means one renderer did
    // not light the scene at all, in which case the error metrics below
    // describe a broken setup rather than a real backend discrepancy.
    // ------------------------------------------------------------------
    const auto mean_rgb = [](const UserRGBA8BufferPtr& img) {
        double s = 0.0;
        const unsigned int n = img->Width * img->Height;
        const PixelRGBA8* p = img->Buffer.get();
        for (unsigned int i = 0; i < n; ++i)
            s += (double)p[i].R + p[i].G + p[i].B;
        return s / (3.0 * n);
    };
#ifdef CHRONO_HAS_OPTIX
    std::cout << "Mean level: OptiX " << mean_rgb(optix_buf) << ", Vulkan " << mean_rgb(vulkan_buf)
              << "  (8-bit code units)\n";
#else
    std::cout << "Mean level: Vulkan " << mean_rgb(vulkan_buf) << "  (8-bit code units)\n";
#endif

#ifdef CHRONO_HAS_OPTIX
    // ------------------------------------------------------------------
    // Compare (candidate = Vulkan, reference = OptiX) and report.
    // ------------------------------------------------------------------
    auto result = CompareRGBA8(vulkan_buf, optix_buf, diff_threshold);
    std::cout << "\n";
    PrintImageCompareResult(std::cout, result, "Vulkan vs OptiX, " + tag);
    std::cout << "\n";

    // ------------------------------------------------------------------
    // Persist images: reference, candidate, and amplified difference maps.
    // ------------------------------------------------------------------
    WritePPM(out_dir + tag + "_optix.ppm", optix_buf);
    WritePPM(out_dir + tag + "_vulkan.ppm", vulkan_buf);
    if (auto diff1 = MakeAbsDiffRGBA8(vulkan_buf, optix_buf, 1.0f))
        WritePPM(out_dir + tag + "_diff_x1.ppm", diff1);
    if (auto diff8 = MakeAbsDiffRGBA8(vulkan_buf, optix_buf, 8.0f))
        WritePPM(out_dir + tag + "_diff_x8.ppm", diff8);
    std::cout << "Wrote " << tag << "_{optix,vulkan,diff_x1,diff_x8}.ppm to " << out_dir << "\n";
    std::cout << "(convert to PNG, e.g.:  magick " << out_dir << tag << "_diff_x8.ppm " << tag << "_diff_x8.png)\n";
#else
    // Vulkan-only build: capture the Vulkan image so it can be diffed offline against the
    // image the same tag produced on the NVIDIA host. Nothing to compare against locally.
    WritePPM(out_dir + tag + "_vulkan.ppm", vulkan_buf);
    std::cout << "Vulkan-only build (no OptiX in this configuration).\n";
    std::cout << "Wrote " << tag << "_vulkan.ppm to " << out_dir << "\n";
    std::cout << "Diff it offline against the NVIDIA-host image of the same tag.\n";
#endif

    return 0;
}

#else  // Vulkan RT backend required

int main(int argc, char* argv[]) {
    std::cerr << "demo_SEN_vulkan_validation requires CH_USE_SENSOR_VULKAN_RT=ON.\n"
              << "Add CH_USE_SENSOR_OPTIX=ON (needs an NVIDIA GPU) for the 1:1 comparison;\n"
              << "with Vulkan alone it captures the Vulkan image for an offline cross-GPU diff.\n";
    return 1;
}

#endif
