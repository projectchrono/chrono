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

#include <cstdlib>
#include <iostream>
#include <string>

#include "chrono/utils/ChBenchmark.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChSolverBB.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChWheelTestRig.h"

#include "tests/benchmark_tests/vehicle/ScmBenchmarkUtils.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#endif

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

// True only for the interactive run below. The rig's own visual assets and the SCM visualization
// mesh are both off in a timed run: the SCM mesh gates a per-node vertex update inside the
// modified-node loop, which would be charged to soil physics rather than to a visualization timer.
// A run that renders needs both, or there is nothing to see and the soil does not deform on screen.
static bool scm_render = false;

// =============================================================================

// GRID_MM is the SCM grid spacing in millimeters (a template parameter has to be integral).
template <int GRID_MM>
class WheelScmTest : public utils::ChBenchmarkTest {
  public:
    WheelScmTest();
    ~WheelScmTest();

    ChSystem* GetSystem() override { return m_sys; }

    void SimulateVis();

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
    m_rig->SetVisualizationType(scm_render ? VisualizationType::MESH : VisualizationType::NONE);

    // No SCM visualization mesh in a timed run: see the note on scm_render.
    m_rig->EnableTerrainVisualizationMesh(scm_render);

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

// Interactive run. The wheel is driven at a constant longitudinal slip along a 10 x 1 m patch, so
// the camera tracks it down the patch; what to look for is a single continuous rut behind the
// contact patch and a raised bow wave ahead of it.
template <int GRID_MM>
void WheelScmTest<GRID_MM>::SimulateVis() {
#ifdef CHRONO_IRRLICHT
    auto vis = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
    vis->AttachSystem(m_sys);
    vis->SetWindowTitle("Polaris wheel on SCM -- " + std::to_string(GRID_MM) + " mm grid");
    vis->SetWindowSize(1280, 720);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->Initialize();
    vis->AddLightDirectional();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(0, -2.0, 1.0), ChVector3d(0, 0, 0));

    // Render on a frame budget, not once per step. The rig integrates at 2e-4 s, so a render per
    // step is 5000 frames per simulated second: the window sets the pace and the run crawls, even
    // though the physics alone is faster than real time. SCM_BENCH_VIS_FPS overrides; 0 restores a
    // render per step.
    double render_fps = 50;
    if (const char* e = std::getenv("SCM_BENCH_VIS_FPS"))
        render_fps = std::atof(e);
    int render_frame = 0;

    std::cout << "Rig drops the wheel and settles for the first " << 2.0 + settle_time
              << " s of simulated time before it rolls." << std::endl;

    while (vis->Run()) {
        if (render_fps <= 0 || m_sys->GetChTime() >= render_frame / render_fps) {
            const auto& loc = m_rig->GetPos();
            vis->UpdateCamera(loc + ChVector3d(0, -2.0, 1.0), loc);
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
            render_frame++;
        }
        ExecuteStep();
    }
#endif
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

#ifdef CHRONO_IRRLICHT
    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        scm_render = true;  // must be set before the fixture builds the rig and the terrain
        const char* g = std::getenv("SCM_BENCH_VARIANT");
        const std::string variant = g ? g : "D20";
        if (variant == "D10") {
            WheelScmTest<10> test;
            test.SimulateVis();
        } else {
            WheelScmTest<20> test;
            test.SimulateVis();
        }
        return 0;
    }
#endif

    ::benchmark::RunSpecifiedBenchmarks();
}
