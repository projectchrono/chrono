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
// Run-time selection of the SCM backend (CPU or GPU) and SCM node storage.
// Tests that need a GPU are skipped when SCMTerrain::IsGpuAvailable() is false.
// =============================================================================

#include <map>
#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/collision/ChCollisionShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace {

struct ScmScene {
    ScmScene() {
        system.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
        system.SetNumThreads(1);

        auto mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        mesh->GetCoordsVertices() = {{-0.2, -0.2, -0.2}, {0.2, -0.2, -0.2}, {0.2, 0.2, -0.2}, {-0.2, 0.2, -0.2},
                                    {-0.2, -0.2, 0.2}, {0.2, -0.2, 0.2}, {0.2, 0.2, 0.2}, {-0.2, 0.2, 0.2}};
        mesh->GetIndicesVertices() = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
                                     {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
        auto material = chrono_types::make_shared<ChContactMaterialSMC>();
        body = chrono_types::make_shared<ChBody>();
        body->SetFixed(true);
        body->SetPos(ChVector3d(0, 0, 0.17));
        body->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeTriangleMesh>(material, mesh, true, false, 0));
        body->EnableCollision(true);
        system.AddBody(body);

        terrain = std::make_unique<SCMTerrain>(&system, false);
        terrain->AddActiveDomain(body, VNULL, ChVector3d(0.6));
    }

    void Initialize() { terrain->Initialize(2, 2, 0.05); }

    void Step() {
        for (int i = 0; i < 3; ++i)
            system.DoStepDynamics(0.001);
    }

    ChSystemSMC system;
    std::shared_ptr<ChBody> body;
    std::unique_ptr<SCMTerrain> terrain;
};

}  // namespace

TEST(SCMBackend, Selection) {
    ChSystemSMC system;
    system.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    SCMTerrain terrain(&system, false);

    EXPECT_EQ(terrain.GetRayCastBackend(), SCMTerrain::Backend::AUTO);
    EXPECT_EQ(terrain.GetContactForceBackend(), SCMTerrain::Backend::AUTO);

    terrain.SetBackend(SCMTerrain::Backend::CPU);
    EXPECT_EQ(terrain.GetRayCastBackend(), SCMTerrain::Backend::CPU);
    EXPECT_EQ(terrain.GetContactForceBackend(), SCMTerrain::Backend::CPU);

    terrain.SetRayCastBackend(SCMTerrain::Backend::GPU);
    EXPECT_EQ(terrain.GetRayCastBackend(), SCMTerrain::Backend::GPU);
    EXPECT_EQ(terrain.GetContactForceBackend(), SCMTerrain::Backend::CPU);

    terrain.SetContactForceBackend(SCMTerrain::Backend::AUTO);
    EXPECT_EQ(terrain.GetRayCastBackend(), SCMTerrain::Backend::GPU);
    EXPECT_EQ(terrain.GetContactForceBackend(), SCMTerrain::Backend::AUTO);

    SCMTerrain::GpuSettings settings;
    settings.min_hits = 17;
    settings.raycast_fp64 = true;
    terrain.SetGpuSettings(settings);
    EXPECT_EQ(terrain.GetGpuSettings().min_hits, 17u);
    EXPECT_TRUE(terrain.GetGpuSettings().raycast_fp64);

#ifndef CHRONO_HAS_SCM_GPU
    EXPECT_FALSE(SCMTerrain::IsGpuAvailable());
#endif
}

TEST(SCMBackend, CpuBackend) {
    ScmScene scene;
    scene.terrain->SetBackend(SCMTerrain::Backend::CPU);
    scene.Initialize();
    scene.Step();

    EXPECT_EQ(scene.terrain->GetNumRaycastGpuSteps(), 0);
    EXPECT_EQ(scene.terrain->GetNumContactForceGpuSteps(), 0);
    EXPECT_GT(scene.terrain->GetNumRayCasts(), 0);
    EXPECT_GT(scene.terrain->GetNumDeformedNodes(), 0u);
    ChVector3d force, torque;
    ASSERT_TRUE(scene.terrain->GetContactForceBody(scene.body, force, torque));
    EXPECT_GT(force.z(), 0);
    EXPECT_GT(scene.terrain->GetNodeInfo(VNULL).sinkage, 0);
}

TEST(SCMBackend, GpuBackend) {
    ScmScene scene;
    scene.terrain->SetBackend(SCMTerrain::Backend::GPU);
    scene.Initialize();
    scene.Step();

    // Without a usable GPU, the GPU backend falls back to the CPU.
    if (SCMTerrain::IsGpuAvailable()) {
        EXPECT_GT(scene.terrain->GetNumRaycastGpuSteps(), 0);
        EXPECT_GT(scene.terrain->GetNumContactForceGpuSteps(), 0);
    } else {
        EXPECT_EQ(scene.terrain->GetNumRaycastGpuSteps(), 0);
        EXPECT_EQ(scene.terrain->GetNumContactForceGpuSteps(), 0);
    }
    EXPECT_GT(scene.terrain->GetNumRayCasts(), 0);
    ChVector3d force, torque;
    ASSERT_TRUE(scene.terrain->GetContactForceBody(scene.body, force, torque));
    EXPECT_GT(force.z(), 0);
}

TEST(SCMBackend, AutoBackendMinHits) {
    if (!SCMTerrain::IsGpuAvailable())
        GTEST_SKIP() << "No usable GPU";

    // This scene has far fewer ray hits than the default AUTO threshold: contact forces stay on the CPU.
    ScmScene scene;
    scene.Initialize();
    scene.Step();
    EXPECT_GT(scene.terrain->GetNumRaycastGpuSteps(), 0);
    EXPECT_EQ(scene.terrain->GetNumContactForceGpuSteps(), 0);

    // With no threshold, AUTO uses the GPU for contact forces too.
    SCMTerrain::GpuSettings settings;
    settings.min_hits = 0;
    scene.terrain->SetGpuSettings(settings);
    scene.Step();
    EXPECT_GT(scene.terrain->GetNumContactForceGpuSteps(), 0);
}

TEST(SCMBackend, SwitchAtRunTime) {
    if (!SCMTerrain::IsGpuAvailable())
        GTEST_SKIP() << "No usable GPU";

    ScmScene scene;
    scene.terrain->SetBackend(SCMTerrain::Backend::CPU);
    scene.Initialize();
    scene.Step();
    ASSERT_EQ(scene.terrain->GetNumRaycastGpuSteps(), 0);
    ASSERT_EQ(scene.terrain->GetNumContactForceGpuSteps(), 0);

    scene.terrain->SetBackend(SCMTerrain::Backend::GPU);
    scene.Step();
    EXPECT_GT(scene.terrain->GetNumRaycastGpuSteps(), 0);
    EXPECT_GT(scene.terrain->GetNumContactForceGpuSteps(), 0);

    const int raycast_steps = scene.terrain->GetNumRaycastGpuSteps();
    const int force_steps = scene.terrain->GetNumContactForceGpuSteps();
    scene.terrain->SetBackend(SCMTerrain::Backend::CPU);
    scene.Step();
    EXPECT_EQ(scene.terrain->GetNumRaycastGpuSteps(), raycast_steps);
    EXPECT_EQ(scene.terrain->GetNumContactForceGpuSteps(), force_steps);
    EXPECT_GT(scene.terrain->GetNumRayCasts(), 0);
}

TEST(SCMBackend, TerrainsAreIndependent) {
    if (!SCMTerrain::IsGpuAvailable())
        GTEST_SKIP() << "No usable GPU";

    // Terrains with different backends and ray-cast precisions, stepped alternately (shared GPU contexts).
    ScmScene cpu;
    ScmScene gpu32;
    ScmScene gpu64;
    cpu.terrain->SetBackend(SCMTerrain::Backend::CPU);
    gpu32.terrain->SetBackend(SCMTerrain::Backend::GPU);
    gpu64.terrain->SetBackend(SCMTerrain::Backend::GPU);
    SCMTerrain::GpuSettings settings;
    settings.raycast_fp64 = true;
    gpu64.terrain->SetGpuSettings(settings);
    cpu.Initialize();
    gpu32.Initialize();
    gpu64.Initialize();
    for (int i = 0; i < 2; i++) {
        cpu.Step();
        gpu32.Step();
        gpu64.Step();
    }

    EXPECT_EQ(cpu.terrain->GetNumRaycastGpuSteps(), 0);
    EXPECT_EQ(cpu.terrain->GetNumContactForceGpuSteps(), 0);
    for (auto* scene : {&gpu32, &gpu64}) {
        EXPECT_GT(scene->terrain->GetNumRaycastGpuSteps(), 0);
        EXPECT_GT(scene->terrain->GetNumContactForceGpuSteps(), 0);
        ChVector3d force, torque;
        ASSERT_TRUE(scene->terrain->GetContactForceBody(scene->body, force, torque));
        EXPECT_GT(force.z(), 0);
    }
}

TEST(SCMStorage, NodeRoundTrip) {
    auto stored_level = [](double value) {
#ifdef CHRONO_SCM_USE_DOUBLE
        return value;
#else
        return static_cast<double>(static_cast<float>(value));
#endif
    };

    for (bool visualization : {false, true}) {
        SCOPED_TRACE(visualization ? "visualized terrain" : "headless terrain");
        ChSystemSMC system;
        system.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
        SCMTerrain terrain(&system, visualization);
        terrain.SetBackend(SCMTerrain::Backend::CPU);
        terrain.Initialize(4, 4, 0.02);

        std::vector<SCMTerrain::NodeLevel> input;
        std::map<std::pair<int, int>, double> expected;
        // Expected levels, indexed by (y + 33) * 67 + (x + 33). Computed once and reused by the parallel check
        // below: recomputing the level expression there can round differently (e.g. FMA contraction).
        std::vector<double> stored(67 * 67);
        // Include negative coordinates and both sides of the 16-node tile boundaries.
        for (int y = -33; y <= 33; ++y) {
            for (int x = -33; x <= 33; ++x) {
                double z = -0.123456789012345 - (x + 33) * 1e-5 - (y + 33) * 2e-5;
                input.emplace_back(ChVector2i(x, y), z);
                expected[{x, y}] = stored_level(z);
                stored[(y + 33) * 67 + (x + 33)] = stored_level(z);
            }
        }
        terrain.SetModifiedNodes(input);
        terrain.SetModifiedNodes({{ChVector2i(-17, 16), -0.987654321098765}});
        expected[{-17, 16}] = stored_level(-0.987654321098765);
        stored[(16 + 33) * 67 + (-17 + 33)] = stored_level(-0.987654321098765);
        EXPECT_EQ(terrain.GetNumDeformedNodes(), expected.size());

        auto output = terrain.GetModifiedNodes(true);
        ASSERT_EQ(output.size(), expected.size());
        for (const auto& node : output) {
            auto key = std::make_pair(node.first.x(), node.first.y());
            auto it = expected.find(key);
            ASSERT_NE(it, expected.end());
            EXPECT_DOUBLE_EQ(node.second, it->second);
            EXPECT_DOUBLE_EQ(terrain.GetHeight(ChVector3d(key.first * 0.02, key.second * 0.02, 0)), it->second);
            expected.erase(it);
        }
        EXPECT_TRUE(expected.empty());
        EXPECT_DOUBLE_EQ(terrain.GetHeight(ChVector3d(1.8, 1.8, 0)), 0);

        int errors = 0;
        #pragma omp parallel for reduction(+:errors) num_threads(4)
        for (int k = 0; k < 10000; ++k) {
            const int x = k % 67 - 33;
            const int y = (k / 67) % 67 - 33;
            if (terrain.GetHeight(ChVector3d(x * 0.02, y * 0.02, 0)) != stored[(y + 33) * 67 + (x + 33)])
                ++errors;
        }
        EXPECT_EQ(errors, 0);
    }
}
