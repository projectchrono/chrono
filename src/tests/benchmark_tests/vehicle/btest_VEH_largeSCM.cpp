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
// Benchmark test for a vehicle on a large, long-worked SCM patch -- the large
// end of the SCM scaling set.
//
// Two things separate this from btest_VEH_hmmwvSCM, and each targets a cost
// that only appears at scale:
//
//  1. The patch is 300 x 300 m at 0.02 m spacing and comes from a height map,
//     so the dense base-height matrix is really allocated: 15001^2 entries.
//     A flat patch allocates nothing (SCMLoader::GetInitHeight returns 0 for
//     PatchType::FLAT), so a flat patch cannot measure that cost no matter how
//     large it is made.
//
//  2. The modified-node map is pre-seeded with previously-worked ruts. That map
//     only grows -- a node enters on first contact and is never evicted -- so
//     in a long run it reflects ground covered, not ground currently under a
//     wheel. Reaching ~900k entries by driving would take hours of wall clock;
//     seeding puts the benchmark at that state in under a second, and it is the
//     honest experiment, because the quantity under test is the size of the map,
//     not how it got large.
//
// The SEED0 and SEED4 variants differ ONLY in that seeding, so the pair is
// self-contained: whatever SEED4 costs over SEED0 is what map size costs.
// Everything else -- vehicle, soil, driver, active domains, step size -- is
// identical to btest_VEH_hmmwvSCM, so the three tests differ in scale alone.
//
// Companion tests: btest_VEH_wheelSCM (small), btest_VEH_hmmwvSCM (medium).
//
// =============================================================================

#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

#if defined(__linux__)
    #include <unistd.h>
#endif

#include <cstdlib>
#include <string>

#include "chrono/utils/ChBenchmark.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "tests/benchmark_tests/vehicle/ScmBenchmarkUtils.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

const std::string heightmap_file = "terrain/height_maps/terrain3.bmp";

double patch_size = 300.0;          // m, square patch (reduced by the interactive run, see main)
const double grid_spacing = 0.02;   // m
const double height_min = -0.25;    // m, height map range about the SCM plane
const double height_max = 0.25;     // m

// Pre-seeded ruts. Each is 15 nodes (0.30 m) wide -- one HMMWV tire track -- and spans the full
// patch in x, giving 15 * 15001 = 225,015 nodes per track.
//
// Row spacing is deliberately not a multiple of any power of two, and neither is the first row. A
// rut falls where a vehicle drove, not where a storage scheme would like it to, and a layout that
// happened to line up with an internal block size would flatter one implementation over another.
const int track_width = 15;
int track_j_first = 2003;
const int track_j_pitch = 114;
const double rut_depth = 0.04;  // m below the undeformed surface

const double step_size = 2e-3;

// True only for the interactive run below, which is NOT the timed configuration: the SCM
// visualization mesh spans the whole patch, and at 300 m by 0.02 m that is 15001^2 = 225M vertices.
// The interactive run therefore shrinks the patch (SCM_BENCH_VIS_PATCH, default 20 m) and re-bases
// the seeded ruts into it. It is for looking at the scene, never for a number.
static bool scm_render = false;

// =============================================================================

class LargeScmDriver : public ChDriver {
  public:
    LargeScmDriver(ChVehicle& vehicle, double delay) : ChDriver(vehicle), m_delay(delay) {}

    virtual void Synchronize(double time) override {
        m_throttle = 0;
        m_steering = 0;
        m_braking = 0;

        double eff_time = time - m_delay;
        if (eff_time < 0)
            return;

        m_throttle = (eff_time > 0.2) ? 0.7 : 3.5 * eff_time;
        m_steering = (eff_time < 2) ? 0.0 : 0.6 * std::sin(CH_2PI * (eff_time - 2) / 6);
    }

  private:
    double m_delay;
};

// =============================================================================

// Resident set size in MiB, or 0 where it cannot be read. Reported because half of what this test
// exercises is memory, not time: the dense base-height matrix and the per-node soil records are the
// whole per-rank cost of an SCM patch, and a wall-clock number does not show them.
//
// Only trustworthy for the FIRST variant constructed in a process. Google Benchmark tears down and
// rebuilds the fixture between repetitions, and glibc does not hand a freed multi-hundred-MB matrix
// back to the OS, so by the second variant RSS reflects allocator retention as much as live data.
// To compare variants, run each in its own process (--benchmark_filter=SEEDn).
static double ResidentMiB() {
#if defined(__linux__)
    long pages = 0, resident = 0;
    if (FILE* f = std::fopen("/proc/self/statm", "r")) {
        if (std::fscanf(f, "%ld %ld", &pages, &resident) != 2)
            resident = 0;
        std::fclose(f);
    }
    return resident * (double)sysconf(_SC_PAGESIZE) / (1024 * 1024);
#else
    return 0;
#endif
}

// =============================================================================

// SEED_TRACKS is the number of pre-worked ruts written into the modified-node map at setup, which
// is how this test reaches a map size that driving would take hours to build. Registered as 0, 1, 4
// and 16; ruts are the only difference between the variants.
template <int SEED_TRACKS>
class LargeScmTest : public utils::ChBenchmarkTest {
  public:
    LargeScmTest();
    ~LargeScmTest();

    ChSystem* GetSystem() override { return m_sys; }
    void ExecuteStep() override;

    void SimulateVis();

    const SCMTerrain& GetTerrain() const { return *m_terrain; }

    scm_bench::ScmStats m_scm;

  private:
    void Preseed();

    ChSystemSMC* m_sys;
    SCMTerrain* m_terrain;
    HMMWV_Full* m_hmmwv;
    LargeScmDriver* m_driver;
    size_t m_seeded;
    double m_rss_after_setup;
};

template <int SEED_TRACKS>
LargeScmTest<SEED_TRACKS>::LargeScmTest() : m_seeded(0) {
    m_sys = new ChSystemSMC;
    m_sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    m_sys->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    m_sys->SetNumThreads(4);

    // Terrain first: the vehicle has to be dropped onto whatever height the map gives at the spawn
    // point, and that is only known once the patch is initialized.
    //
    // No visualization mesh. A benchmark never draws it, and it is not free when it is not drawn:
    // m_trimesh_shape gates a per-node vertex update inside the modified-node loop of
    // ComputeInternalForces, charged to that loop rather than to the visualization timer, so it
    // would be measured here as though it were soil physics. At this patch size it would also be
    // the dominant allocation.
    m_terrain = new SCMTerrain(m_sys, scm_render);
    m_terrain->SetSoilParameters(2e6,   // Bekker Kphi
                                 0,     // Bekker Kc
                                 1.1,   // Bekker n exponent
                                 0,     // Mohr cohesive limit (Pa)
                                 30,    // Mohr friction limit (degrees)
                                 0.01,  // Janosi shear coefficient (m)
                                 2e8,   // Elastic stiffness (Pa/m), before plastic yield
                                 3e4    // Damping (Pa s/m), proportional to negative vertical speed
    );
    m_terrain->Initialize(GetVehicleDataFile(heightmap_file), patch_size, patch_size, height_min, height_max,
                          grid_spacing);

    Preseed();

    // Spawn in a corner, heading across the patch, clear of the seeded ruts.
    double x0 = 5.0 - patch_size / 2;
    double y0 = 5.0 - patch_size / 2;
    double z0 = m_terrain->GetInitHeight(ChVector3d(x0, y0, 0)) + 0.7;

    m_hmmwv = new HMMWV_Full(m_sys);
    m_hmmwv->SetContactMethod(ChContactMethod::SMC);
    m_hmmwv->SetChassisFixed(false);
    m_hmmwv->SetInitPosition(ChCoordsys<>(ChVector3d(x0, y0, z0), QuatFromAngleZ(CH_PI / 4)));
    m_hmmwv->SetEngineType(EngineModelType::SHAFTS);
    m_hmmwv->SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    m_hmmwv->SetDriveType(DrivelineTypeWV::AWD);
    m_hmmwv->SetTireType(TireModelType::RIGID_MESH);
    m_hmmwv->SetTireStepSize(step_size);
    m_hmmwv->SetAerodynamicDrag(0.5, 5.0, 1.2);
    m_hmmwv->Initialize();

    const auto vis_type = scm_render ? VisualizationType::MESH : VisualizationType::NONE;
    m_hmmwv->SetChassisVisualizationType(vis_type);
    m_hmmwv->SetSuspensionVisualizationType(vis_type);
    m_hmmwv->SetSteeringVisualizationType(vis_type);
    m_hmmwv->SetWheelVisualizationType(vis_type);
    m_hmmwv->SetTireVisualizationType(vis_type);

    // Same four wheel domains, at the same dimensions, as btest_VEH_hmmwvSCM.
    for (int axle = 0; axle < 2; axle++) {
        for (auto side : {VehicleSide::LEFT, VehicleSide::RIGHT}) {
            m_terrain->AddActiveDomain(m_hmmwv->GetVehicle().GetAxle(axle)->GetWheel(side)->GetSpindle(),
                                       ChVector3d(0, 0, 0), ChVector3d(1.0, 0.3, 1.0));
        }
    }

    scm_bench::SelectRaycastBackend(*m_terrain, true);

    m_driver = new LargeScmDriver(m_hmmwv->GetVehicle(), 1.0);
    m_driver->Initialize();

    m_rss_after_setup = ResidentMiB();
}

template <int SEED_TRACKS>
void LargeScmTest<SEED_TRACKS>::Preseed() {
    if (SEED_TRACKS == 0)
        return;

    int nx = static_cast<int>(std::ceil((patch_size / 2) / grid_spacing));

    std::vector<SCMTerrain::NodeLevel> nodes;
    nodes.reserve(static_cast<size_t>(SEED_TRACKS) * track_width * (2 * nx + 1));

    for (int t = 0; t < SEED_TRACKS; t++) {
        for (int w = 0; w < track_width; w++) {
            int j = track_j_first + t * track_j_pitch + w;
            if (std::abs(j) > nx)  // outside the patch (only reachable on the reduced viz patch)
                continue;
            double y = j * grid_spacing;
            for (int i = -nx; i <= nx; i++) {
                double x = i * grid_spacing;
                double z = m_terrain->GetInitHeight(ChVector3d(x, y, 0)) - rut_depth;
                nodes.push_back(std::make_pair(ChVector2i(i, j), z));
            }
        }
    }

    m_terrain->SetModifiedNodes(nodes);
    m_seeded = nodes.size();
}

template <int SEED_TRACKS>
LargeScmTest<SEED_TRACKS>::~LargeScmTest() {
    std::cerr << "[SCM] large seed_tracks=" << SEED_TRACKS                              //
              << " seeded=" << m_seeded                                                 //
              << " deformed_nodes=" << m_terrain->GetModifiedNodes(true).size()          //
              << " rss_after_setup=" << m_rss_after_setup << "MiB"                       //
              << " rss_now=" << ResidentMiB() << "MiB"                                   //
              << " t=" << m_sys->GetChTime() << std::endl;

    delete m_hmmwv;
    delete m_driver;
    delete m_terrain;
    delete m_sys;
}

template <int SEED_TRACKS>
void LargeScmTest<SEED_TRACKS>::ExecuteStep() {
    double time = m_sys->GetChTime();

    DriverInputs driver_inputs = m_driver->GetInputs();

    m_driver->Synchronize(time);
    m_terrain->Synchronize(time);
    m_hmmwv->Synchronize(time, driver_inputs, *m_terrain);

    m_driver->Advance(step_size);
    m_terrain->Advance(step_size);
    m_hmmwv->Advance(step_size);

    // The vehicle was handed an existing system, so ChVehicle::Advance does not step it
    // (m_ownsSystem is false); the caller has to.
    m_sys->DoStepDynamics(step_size);

    m_scm.Accumulate(*m_terrain);
}

// Interactive run. NOT the timed configuration -- the patch is shrunk so the SCM visualization
// mesh can exist at all. What to look for: the pre-seeded ruts as straight depressions across the
// patch, and the HMMWV cutting its own diagonal track across them.
template <int SEED_TRACKS>
void LargeScmTest<SEED_TRACKS>::SimulateVis() {
#ifdef CHRONO_IRRLICHT
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->AttachVehicle(&m_hmmwv->GetVehicle());
    vis->SetWindowTitle("Large SCM patch -- " + std::to_string(SEED_TRACKS) + " seeded ruts");
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5);
    vis->Initialize();
    vis->AddLightDirectional();
    vis->AddSkyBox();

    double render_fps = 50;  // see the note in btest_VEH_hmmwvSCM; SCM_BENCH_VIS_FPS overrides
    if (const char* e = std::getenv("SCM_BENCH_VIS_FPS"))
        render_fps = std::atof(e);
    int render_frame = 0;

    while (vis->Run()) {
        DriverInputs driver_inputs = m_driver->GetInputs();

        if (render_fps <= 0 || m_sys->GetChTime() >= render_frame / render_fps) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
            render_frame++;
        }
        ExecuteStep();
        vis->Synchronize(m_sys->GetChTime(), driver_inputs);
        vis->Advance(step_size);
    }
#endif
}

// =============================================================================

#define NUM_SKIP_STEPS 500   // hot start (2e-3 * 500 = 1 s)
#define NUM_SIM_STEPS 2000   // timed window (2e-3 * 2000 = 4 s)
#define REPEATS 5

// A sweep, not a single point. Whether the cost of a large modified-node map is worth doing anything
// about depends on how it grows, and one seeded size cannot show that. 0 / 1 / 4 / 16 tracks is
// 0 / 225k / 900k / 3.6M nodes; a 13.7 h two-rover run at 0.02 m spacing reached ~891k, so the
// middle of this range is a real working point and the top of it is where a longer run heads.
typedef LargeScmTest<0> large_seed0_test_type;
typedef LargeScmTest<1> large_seed1_test_type;
typedef LargeScmTest<4> large_seed4_test_type;
typedef LargeScmTest<16> large_seed16_test_type;

CH_BM_SCM_SIMULATION_ONCE(LargeSCM_SEED0, large_seed0_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SCM_SIMULATION_ONCE(LargeSCM_SEED1, large_seed1_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SCM_SIMULATION_ONCE(LargeSCM_SEED4, large_seed4_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SCM_SIMULATION_ONCE(LargeSCM_SEED16, large_seed16_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);

// =============================================================================

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);

#ifdef CHRONO_IRRLICHT
    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        scm_render = true;  // must be set before the fixture builds the terrain

        // The timed patch cannot be drawn: 300 m at 0.02 m is 15001^2 visualization vertices. Shrink
        // it, and re-base the seeded ruts from row 2003 (y = 40 m) into the smaller patch, keeping
        // their width and 114-row pitch. Everything else -- soil parameters, grid spacing, vehicle,
        // active domains, route -- is as benchmarked.
        if (const char* e = std::getenv("SCM_BENCH_VIS_PATCH"))
            patch_size = std::atof(e);
        else
            patch_size = 20.0;
        track_j_first = 50;
        std::cout << "Interactive run: patch " << patch_size << " m, NOT the benchmarked size"
                  << std::endl;

        const char* v = std::getenv("SCM_BENCH_VARIANT");
        const std::string variant = v ? v : "SEED4";
        if (variant == "SEED0") {
            LargeScmTest<0> test;
            test.SimulateVis();
        } else if (variant == "SEED1") {
            LargeScmTest<1> test;
            test.SimulateVis();
        } else {
            LargeScmTest<4> test;
            test.SimulateVis();
        }
        return 0;
    }
#endif

    ::benchmark::RunSpecifiedBenchmarks();
}
