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
// Authors: Chrono contributors
// =============================================================================
//
// Benchmark test for a single wheel on SCM terrain -- the small end of the SCM
// scaling set.
//
// This is the control case. The patch is 10 x 1 m, so the modified-node map can
// never exceed a few tens of thousands of entries and stays inside last-level
// cache for the whole run. Any change to how that map is stored should be
// neutral here; this test exists to show that it is, and to catch a change that
// buys large-scale throughput at the cost of small-scale overhead.
//
// Companion tests: btest_VEH_hmmwvSCM (medium), btest_VEH_largeSCM (large).
//
// =============================================================================

#include <iostream>

#include "chrono/utils/ChBenchmark.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChSolverBB.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChWheelTestRig.h"

#include "tests/benchmark_tests/vehicle/ScmBenchmarkUtils.h"

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================

// Polaris running gear. The tire must be a triangle mesh: SCM ray casts against
// the collision mesh, and the GPU ray-cast backend, when built, declines any
// model whose collision shape is a primitive.
const std::string wheel_json = "Polaris/Polaris_Wheel.json";
const std::string tire_json = "Polaris/Polaris_RigidMeshTire.json";

const double patch_length = 10.0;  // m
const double patch_width = 1.0;    // m

const double normal_load = 3000.0;  // N
const double base_speed = 1.0;      // m/s
const double long_slip = 0.1;       // driven wheel, 10% slip
const double settle_time = 0.5;     // s before rig inputs are applied

const double step_size = 2e-4;  // s

// =============================================================================

// GRID_MM is the SCM grid spacing in millimeters (a template parameter has to be integral).
template <int GRID_MM>
class WheelScmTest : public utils::ChBenchmarkTest {
  public:
    WheelScmTest();
    ~WheelScmTest();

    ChSystem* GetSystem() override { return m_sys; }

    void ExecuteStep() override {
        m_rig->Advance(step_size);
        m_scm.Accumulate(*m_terrain);
    }

    const SCMTerrain& GetTerrain() const { return *m_terrain; }

    scm_bench::ScmStats m_scm;

  private:
    ChSystemNSC* m_sys;
    ChWheelTestRig* m_rig;
    std::shared_ptr<SCMTerrain> m_terrain;
};

template <int GRID_MM>
WheelScmTest<GRID_MM>::WheelScmTest() {
    auto wheel = ReadWheelJSON(GetVehicleDataFile(wheel_json));
    auto tire = ReadTireJSON(GetVehicleDataFile(tire_json));

    m_sys = new ChSystemNSC;
    m_sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    m_sys->SetNumThreads(4, 1, 1);
    m_sys->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    m_sys->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    m_sys->GetSolver()->AsIterative()->SetMaxIterations(150);

    tire->SetStepsize(step_size);

    m_rig = new ChWheelTestRig(wheel, tire, *m_sys);
    m_rig->SetGravitationalAcceleration(9.8);
    m_rig->SetNormalLoad(normal_load);
    m_rig->SetStepsize(step_size);
    m_rig->SetVisualizationType(VisualizationType::NONE);

    // No SCM visualization mesh: see EnableTerrainVisualizationMesh.
    m_rig->EnableTerrainVisualizationMesh(false);

    ChWheelTestRig::TerrainPatchSize size;
    size.length = patch_length;
    size.width = patch_width;

    ChWheelTestRig::TerrainParamsSCM params;
    params.Bekker_Kphi = 2e6;
    params.Bekker_Kc = 0;
    params.Bekker_n = 1.1;
    params.Mohr_cohesion = 0;
    params.Mohr_friction = 30;
    params.Janosi_shear = 0.01;
    params.grid_spacing = GRID_MM / 1000.0;

    m_rig->SetTerrainSCM(size, params);
    m_rig->SetConstantLongitudinalSlip(long_slip, base_speed);
    m_rig->SetTimeDelay(settle_time);
    m_rig->Initialize(ChWheelTestRig::Mode::TEST, 0.05);

    m_terrain = std::dynamic_pointer_cast<SCMTerrain>(m_rig->GetTerrain());

    // No AddActiveDomain here: ChWheelTestRig::CreateTerrainSCM already declares one, on the
    // chassis body and sized to the wheel. Adding a second casts the nodes under the wheel once
    // per domain -- same hits, same deformed nodes, inflated SCM_Rays and SCM_RayCast.
    scm_bench::SelectRaycastBackend(*m_terrain, true);
}

template <int GRID_MM>
WheelScmTest<GRID_MM>::~WheelScmTest() {
    std::cerr << "[SCM] wheel delta=" << GRID_MM / 1000.0                       //
              << " deformed_nodes=" << m_terrain->GetModifiedNodes(true).size()  //
              << " t=" << m_sys->GetChTime() << std::endl;
    delete m_rig;
    delete m_sys;
}

// =============================================================================

// ChWheelTestRig runs its own start-up sequence before the wheel rolls: a 2 s drop phase, then the
// SetTimeDelay wait. Measuring has to start after that, or most of the window is a settling
// transient. 2e-4 * 13000 = 2.6 s, against a rig that releases at 2.0 + 0.5 s.
#define NUM_SKIP_STEPS 13000
// 2e-4 * 15000 = 3 s, i.e. 3 m of travel at 1 m/s.
#define NUM_SIM_STEPS 15000
#define REPEATS 5

typedef WheelScmTest<20> wheel_d20_test_type;
typedef WheelScmTest<10> wheel_d10_test_type;

CH_BM_SCM_SIMULATION_ONCE(WheelSCM_D20, wheel_d20_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SCM_SIMULATION_ONCE(WheelSCM_D10, wheel_d10_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);

// =============================================================================

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);
    ::benchmark::RunSpecifiedBenchmarks();
}
