// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
//
// Test that SCM terrain runs on a machine without a usable GPU device.
//
// A build with the SCM GPU backend (CHRONO_HAS_SCM_GPU) must fall back to the CPU paths at run time
// when no device is present, instead of aborting. CTest runs this test twice: once as is, where a
// GPU build on a machine with a device must use the GPU backends, and once with
// CUDA_VISIBLE_DEVICES=-1 and HIP_VISIBLE_DEVICES=-1 (and SCM_UTEST_EXPECT_NO_GPU=1), where the
// simulation must still succeed with zero GPU steps.
//
// =============================================================================

#include <cstdlib>
#include <cstring>

#include "gtest/gtest.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/collision/ChCollisionShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_vehicle/terrain/SCMTerrain.h"

using namespace chrono;
using namespace chrono::vehicle;

// Closed box mesh centered at the origin, with outward-facing triangles.
static std::shared_ptr<ChTriangleMeshConnected> CreateBoxMesh(const ChVector3d& size) {
    auto mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    const ChVector3d h = size / 2;
    auto& v = mesh->GetCoordsVertices();
    v = {{-h.x(), -h.y(), -h.z()}, {+h.x(), -h.y(), -h.z()}, {+h.x(), +h.y(), -h.z()}, {-h.x(), +h.y(), -h.z()},
         {-h.x(), -h.y(), +h.z()}, {+h.x(), -h.y(), +h.z()}, {+h.x(), +h.y(), +h.z()}, {-h.x(), +h.y(), +h.z()}};
    auto& f = mesh->GetIndicesVertices();
    f = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
    return mesh;
}

TEST(SCMTerrain, gpu_fallback) {
    const char* expect_env = std::getenv("SCM_UTEST_EXPECT_NO_GPU");
    const bool expect_no_gpu = expect_env && std::strcmp(expect_env, "1") == 0;

    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // Box resting on the soil, its bottom face slightly below the undeformed surface (z = 0).
    const ChVector3d box_size(0.4, 0.4, 0.2);
    auto box = chrono_types::make_shared<ChBody>();
    box->SetMass(40);
    box->SetInertiaXX(ChVector3d(1, 1, 1));
    box->SetPos(ChVector3d(0, 0, box_size.z() / 2 - 0.005));
    auto material = chrono_types::make_shared<ChContactMaterialSMC>();
    box->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeTriangleMesh>(material, CreateBoxMesh(box_size), false, false, 0.0));
    box->EnableCollision(true);
    sys.AddBody(box);

    SCMTerrain terrain(&sys, false);
    terrain.SetSoilParameters(0.2e6, 0, 1.1, 0, 30, 0.01, 4e7, 3e4);
#ifdef CHRONO_HAS_SCM_GPU
    // Make every step eligible for the GPU contact-force backend (400 hits under the box).
    auto config = terrain.GetScmGpuConfig();
    config.enabled = true;
    config.min_hits = 1;
    terrain.SetScmGpuConfig(config);
    terrain.EnableRaycastGpuHip(true);
#endif
    // An explicit active domain is required by the GPU ray-cast backend.
    terrain.AddActiveDomain(box, VNULL, box_size + ChVector3d(0.1, 0.1, 0.1));
    terrain.Initialize(2.0, 2.0, 0.02);

    const double step = 1e-3;
    const int num_steps = 200;
    for (int i = 0; i < num_steps; i++) {
        terrain.Synchronize(sys.GetChTime());
        sys.DoStepDynamics(step);
        terrain.Advance(step);
    }

    // The soil carries the box, whichever backend computed the forces.
    ChVector3d force, torque;
    ASSERT_TRUE(terrain.GetContactForceBody(box, force, torque));
    EXPECT_GT(force.z(), 0.0);
    EXPECT_GT(box->GetPos().z(), -box_size.z());

    const int gpu_force_steps = terrain.GetNumContactForceGpuSteps();
    const int gpu_raycast_steps = terrain.GetNumRaycastGpuSteps();
    std::cout << "GPU contact-force steps: " << gpu_force_steps << "  GPU ray-cast steps: " << gpu_raycast_steps << std::endl;

    if (expect_no_gpu) {
#ifdef CHRONO_HAS_SCM_GPU
        EXPECT_FALSE(SCMTerrain::IsScmGpuDeviceAvailable());
#endif
        EXPECT_EQ(gpu_force_steps, 0);
        EXPECT_EQ(gpu_raycast_steps, 0);
        return;
    }

#ifdef CHRONO_HAS_SCM_GPU
    if (SCMTerrain::IsScmGpuDeviceAvailable()) {
        EXPECT_GT(gpu_force_steps, 0);
        EXPECT_GT(gpu_raycast_steps, 0);
    }
#else
    EXPECT_EQ(gpu_force_steps, 0);
    EXPECT_EQ(gpu_raycast_steps, 0);
#endif
}
