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
// Runtime selection of the CPU SCM backend, including builds with GPU support.
// =============================================================================

#include <cstdlib>
#include <map>
#include <memory>
#include <string>

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

    void RequestGpu() {
#ifdef CHRONO_HAS_SCM_GPU
        auto config = terrain->GetScmGpuConfig();
        config.enabled = true;
        config.min_hits = 0;
        terrain->SetScmGpuConfig(config);
        terrain->SetScmGpuEnabled(true);
        terrain->EnableRaycastGpuHip(true);
#endif
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

TEST(SCMBackend, EnvironmentDefault) {
    ChSystemSMC system;
    system.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    SCMTerrain terrain(&system, false);
    const char* force_cpu = std::getenv("CHRONO_SCM_FORCE_CPU");
    EXPECT_EQ(terrain.IsForceCPU(), force_cpu && std::string(force_cpu) == "1");
    terrain.SetForceCPU(false);
    EXPECT_FALSE(terrain.IsForceCPU());
    terrain.SetForceCPU(true);
    EXPECT_TRUE(terrain.IsForceCPU());
}

TEST(SCMBackend, ForceCPUOverridesGpuSettings) {
    ScmScene scene;
    scene.terrain->SetForceCPU(true);
    scene.RequestGpu();  // Existing application settings must not defeat the override.
    scene.terrain->EnableRaycastGpuReference(true);
    scene.Initialize();
    scene.Step();

    EXPECT_TRUE(scene.terrain->IsForceCPU());
    EXPECT_EQ(scene.terrain->GetNumRaycastGpuSteps(), 0);
    EXPECT_EQ(scene.terrain->GetNumContactForceGpuSteps(), 0);
    EXPECT_GT(scene.terrain->GetNumRayCasts(), 0);
    EXPECT_GT(scene.terrain->GetNumDeformedNodes(), 0u);
    ChVector3d force, torque;
    ASSERT_TRUE(scene.terrain->GetContactForceBody(scene.body, force, torque));
    EXPECT_GT(force.z(), 0);
    EXPECT_GT(scene.terrain->GetNodeInfo(VNULL).sinkage, 0);
}

TEST(SCMBackend, RestoreBackendSettings) {
#ifdef CHRONO_HAS_SCM_GPU
    const char* test_gpu = std::getenv("CHRONO_TEST_SCM_GPU");
    if (!test_gpu || std::string(test_gpu) != "1")
        GTEST_SKIP() << "Set CHRONO_TEST_SCM_GPU=1 to test resuming GPU execution on a compatible device.";
#endif
    ScmScene scene;
    scene.terrain->SetForceCPU(true);
    scene.RequestGpu();
    scene.Initialize();
    scene.Step();
    ASSERT_EQ(scene.terrain->GetNumRaycastGpuSteps(), 0);
    ASSERT_EQ(scene.terrain->GetNumContactForceGpuSteps(), 0);

    scene.terrain->SetForceCPU(false);
    scene.Step();
#ifdef CHRONO_HAS_SCM_GPU
    EXPECT_GT(scene.terrain->GetNumRaycastGpuSteps(), 0);
    EXPECT_GT(scene.terrain->GetNumContactForceGpuSteps(), 0);
#else
    EXPECT_EQ(scene.terrain->GetNumRaycastGpuSteps(), 0);
    EXPECT_EQ(scene.terrain->GetNumContactForceGpuSteps(), 0);
#endif
    const int raycast_steps = scene.terrain->GetNumRaycastGpuSteps();
    const int force_steps = scene.terrain->GetNumContactForceGpuSteps();
    scene.terrain->SetForceCPU(true);
    scene.Step();
    EXPECT_EQ(scene.terrain->GetNumRaycastGpuSteps(), raycast_steps);
    EXPECT_EQ(scene.terrain->GetNumContactForceGpuSteps(), force_steps);
    EXPECT_GT(scene.terrain->GetNumRayCasts(), 0);
}

TEST(SCMBackend, TerrainOverridesAreIndependent) {
#ifdef CHRONO_HAS_SCM_GPU
    const char* test_gpu = std::getenv("CHRONO_TEST_SCM_GPU");
    if (!test_gpu || std::string(test_gpu) != "1")
        GTEST_SKIP() << "Set CHRONO_TEST_SCM_GPU=1 to test simultaneous CPU and GPU terrains on a compatible device.";
#endif
    ScmScene cpu;
    ScmScene other;
    cpu.terrain->SetForceCPU(true);
    other.terrain->SetForceCPU(false);
    cpu.RequestGpu();
    other.RequestGpu();
    cpu.Initialize();
    other.Initialize();
    cpu.Step();
    other.Step();
    EXPECT_TRUE(cpu.terrain->IsForceCPU());
    EXPECT_FALSE(other.terrain->IsForceCPU());
    EXPECT_EQ(cpu.terrain->GetNumRaycastGpuSteps(), 0);
    EXPECT_EQ(cpu.terrain->GetNumContactForceGpuSteps(), 0);
#ifdef CHRONO_HAS_SCM_GPU
    EXPECT_GT(other.terrain->GetNumRaycastGpuSteps(), 0);
    EXPECT_GT(other.terrain->GetNumContactForceGpuSteps(), 0);
#endif
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
        terrain.SetForceCPU(true);
        terrain.Initialize(4, 4, 0.02);

        std::vector<SCMTerrain::NodeLevel> input;
        std::map<std::pair<int, int>, double> expected;
        // Include negative coordinates and both sides of the 16-node tile boundaries.
        for (int y = -33; y <= 33; ++y) {
            for (int x = -33; x <= 33; ++x) {
                double z = -0.123456789012345 - (x + 33) * 1e-5 - (y + 33) * 2e-5;
                input.emplace_back(ChVector2i(x, y), z);
                expected[{x, y}] = stored_level(z);
            }
        }
        terrain.SetModifiedNodes(input);
        terrain.SetModifiedNodes({{ChVector2i(-17, 16), -0.987654321098765}});
        expected[{-17, 16}] = stored_level(-0.987654321098765);
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
            double z = -0.123456789012345 - (x + 33) * 1e-5 - (y + 33) * 2e-5;
            if (x == -17 && y == 16)
                z = -0.987654321098765;
            if (terrain.GetHeight(ChVector3d(x * 0.02, y * 0.02, 0)) != stored_level(z))
                ++errors;
        }
        EXPECT_EQ(errors, 0);
    }
}
