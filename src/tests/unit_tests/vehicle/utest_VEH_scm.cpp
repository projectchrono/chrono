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
//
// Unit tests for SCM deformable terrain:
// - a run with a single thread (regression for ChOMP::GetThreadNum returning 1
//   in builds without OpenMP, which indexed past the per-thread hit maps)
// - with the GPU backend, CPU and GPU contact forces must agree with bulldozing
//   enabled (regression for hit nodes with no pressure losing their hit level)
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/collision/ChCollisionShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

using namespace chrono;
using namespace chrono::vehicle;

// Results recorded during a run
struct ScmRunResults {
    std::vector<ChVector3d> forces;  // contact force on the body, every 50 steps
    double level_sum;                // sum of levels over all modified nodes at the end
    size_t num_nodes;                // number of modified nodes at the end
    int num_gpu_force_steps;         // steps with contact forces computed on the GPU
};

// Drop a 40 kg mesh box with horizontal and angular velocity onto SCM terrain, with bulldozing enabled.
static ScmRunResults RunScm(int num_threads, bool gpu_forces, int num_steps) {
    ChSystemSMC system;
    system.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    system.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    system.SetNumThreads(num_threads);

    auto mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    mesh->GetCoordsVertices() = {{-0.2, -0.2, -0.1}, {0.2, -0.2, -0.1}, {0.2, 0.2, -0.1}, {-0.2, 0.2, -0.1},
                                 {-0.2, -0.2, 0.1},  {0.2, -0.2, 0.1},  {0.2, 0.2, 0.1},  {-0.2, 0.2, 0.1}};
    mesh->GetIndicesVertices() = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};

    auto material = chrono_types::make_shared<ChContactMaterialSMC>();
    auto body = chrono_types::make_shared<ChBody>();
    body->SetMass(40);
    body->SetInertiaXX(ChVector3d(0.6, 0.6, 1.0));
    body->SetPos(ChVector3d(0, 0, 0.11));
    body->SetPosDt(ChVector3d(0.6, 0.1, 0));
    body->SetAngVelParent(ChVector3d(0, 0, 0.5));
    body->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeTriangleMesh>(material, mesh, true, false, 0));
    body->EnableCollision(true);
    system.AddBody(body);

    SCMTerrain terrain(&system, false);
    terrain.SetSoilParameters(0.2e6, 0, 1.1, 0, 30, 0.01, 4e7, 3e4);
    terrain.EnableBulldozing(true);
    terrain.SetBulldozingParameters(55, 1, 5, 10);
    terrain.AddActiveDomain(body, VNULL, ChVector3d(0.7, 0.7, 0.5));

#ifdef CHRONO_HAS_SCM_GPU
    // Ray casting always on the CPU; contact forces on the GPU only if requested, for any number of hits
    auto config = terrain.GetScmGpuConfig();
    config.enabled = gpu_forces;
    config.min_hits = 0;
    terrain.SetScmGpuConfig(config);
    terrain.SetScmGpuEnabled(gpu_forces);
    terrain.EnableRaycastGpuHip(false);
#endif

    terrain.Initialize(3, 3, 0.02);

    ScmRunResults res;
    for (int i = 1; i <= num_steps; i++) {
        system.DoStepDynamics(1e-3);
        if (i % 50 == 0) {
            ChVector3d frc, trq;
            terrain.GetContactForceBody(body, frc, trq);
            res.forces.push_back(frc);
        }
    }

    auto nodes = terrain.GetModifiedNodes(true);
    res.num_nodes = nodes.size();
    res.level_sum = 0;
    for (const auto& n : nodes)
        res.level_sum += n.second;
    res.num_gpu_force_steps = terrain.GetNumContactForceGpuSteps();

    return res;
}

// A single-threaded run must not crash and must produce a support force.
TEST(SCMTerrain, single_thread) {
    auto res = RunScm(1, false, 200);
    ASSERT_FALSE(res.forces.empty());
    EXPECT_GT(res.forces.back().z(), 0.0);
    EXPECT_GT(res.num_nodes, 0u);
}

#ifdef CHRONO_HAS_SCM_GPU

// CPU and GPU contact forces must agree with bulldozing enabled. Bulldozing caps the raise of
// displaced material at the hit level of each node, so a hit node with no pressure must still record
// its hit level; otherwise the terrain shapes, and then the forces, diverge.
TEST(SCMTerrain, gpu_forces_match_cpu) {
    const int num_steps = 1000;
    auto cpu = RunScm(1, false, num_steps);
    auto gpu = RunScm(1, true, num_steps);

    if (gpu.num_gpu_force_steps == 0)
        GTEST_SKIP() << "SCM GPU contact-force backend not used (no GPU device?)";

    ASSERT_EQ(cpu.forces.size(), gpu.forces.size());
    for (size_t k = 0; k < cpu.forces.size(); k++) {
        double tol = 1e-6 * std::max(1.0, cpu.forces[k].Length());
        EXPECT_NEAR(cpu.forces[k].x(), gpu.forces[k].x(), tol) << "step " << 50 * (k + 1);
        EXPECT_NEAR(cpu.forces[k].y(), gpu.forces[k].y(), tol) << "step " << 50 * (k + 1);
        EXPECT_NEAR(cpu.forces[k].z(), gpu.forces[k].z(), tol) << "step " << 50 * (k + 1);
    }

    EXPECT_EQ(cpu.num_nodes, gpu.num_nodes);
    EXPECT_NEAR(cpu.level_sum, gpu.level_sum, 1e-6 * std::max(1.0, std::abs(cpu.level_sum)));
}

#endif
