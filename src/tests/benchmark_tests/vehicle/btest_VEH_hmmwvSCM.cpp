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
// Authors: Radu Serban
// =============================================================================
//
// Benchmark test for HMMWV on SCM terrain.
//
// =============================================================================

#include <cstdlib>
#include <string>

#include "chrono/utils/ChBenchmark.h"
#include "chrono/core/ChRandom.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/collision/ChCollisionShapeTriangleMesh.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/physics/ChMassProperties.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "tests/benchmark_tests/vehicle/ScmBenchmarkUtils.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// True only for the interactive run below. The SCM visualization mesh is not free when it is never
// drawn -- it gates a per-node vertex update inside SCM's modified-node loop, and that cost lands in
// the node loop rather than in a visualization timer -- so a timed run must not build it. A run that
// renders must, or the soil does not deform on screen.
static bool scm_render = false;

// =============================================================================

#define MESH_TIRE 0
#define CYL_TIRE 1

double patch_size = 50.0;
int num_div = 1000;

// =============================================================================

class HmmwvScmDriver : public ChDriver {
  public:
    HmmwvScmDriver(ChVehicle& vehicle, double delay) : ChDriver(vehicle), m_delay(delay) {}
    ~HmmwvScmDriver() {}

    virtual void Synchronize(double time) override {
        m_throttle = 0;
        m_steering = 0;
        m_braking = 0;

        double eff_time = time - m_delay;

        // Do not generate any driver inputs for a duration equal to m_delay.
        if (eff_time < 0)
            return;

        if (eff_time > 0.2)
            m_throttle = 0.7;
        else
            m_throttle = 3.5 * eff_time;

        if (eff_time < 2)
            m_steering = 0;
        else
            m_steering = 0.6 * std::sin(CH_2PI * (eff_time - 2) / 6);
    }

  private:
    double m_delay;
};

// =============================================================================

template <int TIRE_TYPE, bool OBJECTS>
class HmmwvScmTest : public utils::ChBenchmarkTest {
  public:
    HmmwvScmTest();
    ~HmmwvScmTest();

    ChSystem* GetSystem() override { return m_hmmwv->GetSystem(); }
    void ExecuteStep() override;

    const SCMTerrain& GetTerrain() const { return *m_terrain; }

    scm_bench::ScmStats m_scm;

    void SimulateVis();

    double GetTime() const { return m_hmmwv->GetSystem()->GetChTime(); }
    double GetLocation() const { return m_hmmwv->GetVehicle().GetPos().x(); }

  private:
    HMMWV_Full* m_hmmwv;
    HmmwvScmDriver* m_driver;
    SCMTerrain* m_terrain;

    double m_step;
};

template <int TIRE_TYPE, bool OBJECTS>
HmmwvScmTest<TIRE_TYPE, OBJECTS>::HmmwvScmTest() : m_step(2e-3) {
    EngineModelType engine_model = EngineModelType::SHAFTS;
    TransmissionModelType transmission_model = TransmissionModelType::AUTOMATIC_SHAFTS;
    DrivelineTypeWV drive_type = DrivelineTypeWV::AWD;
    TireModelType tire_type = (TIRE_TYPE == MESH_TIRE) ? TireModelType::RIGID_MESH : TireModelType::RIGID;
    VisualizationType tire_vis = (TIRE_TYPE == MESH_TIRE) ? VisualizationType::MESH : VisualizationType::PRIMITIVES;

    // Create the HMMWV vehicle, set parameters, and initialize.
    m_hmmwv = new HMMWV_Full();
    m_hmmwv->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    m_hmmwv->SetContactMethod(ChContactMethod::SMC);
    m_hmmwv->SetChassisFixed(false);
    m_hmmwv->SetInitPosition(
        ChCoordsys<>(ChVector3d(5.0 - patch_size / 2, 5.0 - patch_size / 2, 0.7), QuatFromAngleZ(CH_PI / 4)));
    m_hmmwv->SetEngineType(engine_model);
    m_hmmwv->SetTransmissionType(transmission_model);
    m_hmmwv->SetDriveType(drive_type);
    m_hmmwv->SetTireType(tire_type);
    m_hmmwv->SetTireStepSize(m_step);
    m_hmmwv->SetAerodynamicDrag(0.5, 5.0, 1.2);
    m_hmmwv->Initialize();

    m_hmmwv->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    m_hmmwv->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    m_hmmwv->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    m_hmmwv->SetWheelVisualizationType(VisualizationType::NONE);
    m_hmmwv->SetTireVisualizationType(tire_vis);

    m_hmmwv->GetSystem()->SetNumThreads(4);

    // Create the terrain using 4 moving patches
    // No visualization mesh. A benchmark never draws it, and it is not free when it is not drawn:
    // m_trimesh_shape gates a per-node vertex update inside the modified-node loop of
    // ComputeInternalForces, charged to that loop rather than to the visualization timer, so it
    // would be measured here as though it were soil physics.
    m_terrain = new SCMTerrain(m_hmmwv->GetSystem(), scm_render);
    m_terrain->SetSoilParameters(2e6,   // Bekker Kphi
                                 0,     // Bekker Kc
                                 1.1,   // Bekker n exponent
                                 0,     // Mohr cohesive limit (Pa)
                                 30,    // Mohr friction limit (degrees)
                                 0.01,  // Janosi shear coefficient (m)
                                 2e8,   // Elastic stiffness (Pa/m), before plastic yield
                                 3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );

    m_terrain->AddActiveDomain(m_hmmwv->GetVehicle().GetAxle(0)->GetWheel(VehicleSide::LEFT)->GetSpindle(),
                              ChVector3d(0, 0, 0), ChVector3d(1.0, 0.3, 1.0));
    m_terrain->AddActiveDomain(m_hmmwv->GetVehicle().GetAxle(0)->GetWheel(VehicleSide::RIGHT)->GetSpindle(),
                              ChVector3d(0, 0, 0), ChVector3d(1.0, 0.3, 1.0));
    m_terrain->AddActiveDomain(m_hmmwv->GetVehicle().GetAxle(1)->GetWheel(VehicleSide::LEFT)->GetSpindle(),
                              ChVector3d(0, 0, 0), ChVector3d(1.0, 0.3, 1.0));
    m_terrain->AddActiveDomain(m_hmmwv->GetVehicle().GetAxle(1)->GetWheel(VehicleSide::RIGHT)->GetSpindle(),
                              ChVector3d(0, 0, 0), ChVector3d(1.0, 0.3, 1.0));


    scm_bench::SelectRaycastBackend(*m_terrain, true);

    m_terrain->Initialize(patch_size, patch_size, patch_size / num_div);

    // Custom driver
    m_driver = new HmmwvScmDriver(m_hmmwv->GetVehicle(), 1.0);
    m_driver->Initialize();

    // Obstacles: triangle-mesh rocks, NOT primitives.
    //
    // These were ChBodyEasySphere, which made this variant measure a scene its own obstacles were
    // absent from. The GPU ray-cast backend intersects triangle meshes only: a primitive collision
    // shape contributes no faces, never appears as a hit's contactable, and therefore receives no
    // reaction force from the soil -- the spheres fell straight through the terrain while the
    // mesh-tyred vehicle behaved normally. A scaled rock mesh is a real obstacle on both paths, so
    // OBJECTS now measures what it claims to: the cost of taking the active-domain count from 4 to
    // 24 with geometry that both backends can actually see.
    if (OBJECTS) {
        auto rock_mat = ChContactMaterial::DefaultMaterial(m_hmmwv->GetSystem()->GetContactMethod());
        rock_mat->SetFriction(0.2f);
        const double rock_density = 2500;
        const double rock_scale = 0.3;

        for (int i = 0; i < 20; i++) {
            const char* meshfile = (i % 2) ? "robot/curiosity/rocks/rock1.obj"  //
                                           : "robot/curiosity/rocks/rock3.obj";
            auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(meshfile), false, true);
            mesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(rock_scale));

            double mass;
            ChVector3d cog;
            ChMatrix33<> inertia;
            mesh->ComputeMassProperties(true, mass, cog, inertia);
            ChMatrix33<> principal_rot;
            ChVector3d principal_I;
            ChInertiaUtils::PrincipalInertia(inertia, principal_I, principal_rot);

            // Drop each rock a fixed short distance above the soil, measured from its own scaled
            // geometry, so settling takes the same time for every mesh and fits the hot-start window.
            ChVector3d pos((2 * ChRandom::Get() - 1) * 0.45 * patch_size,
                           (2 * ChRandom::Get() - 1) * 0.45 * patch_size, 0.0);
            pos.z() = 0.05 - mesh->GetBoundingBox().min.z();

            auto rock = chrono_types::make_shared<ChBodyAuxRef>();
            rock->SetFrameRefToAbs(ChFrame<>(pos, QUNIT));
            rock->SetFrameCOMToRef(ChFrame<>(cog, principal_rot));
            rock->SetMass(mass * rock_density);
            rock->SetInertiaXX(rock_density * principal_I);

            auto ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(rock_mat, mesh, false, false, 0.005);
            rock->AddCollisionShape(ct_shape);
            rock->EnableCollision(true);

            auto vis_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            vis_shape->SetMesh(mesh);
            vis_shape->SetBackfaceCull(true);
            rock->AddVisualShape(vis_shape);

            m_hmmwv->GetSystem()->Add(rock);
            m_terrain->AddActiveDomain(rock, ChVector3d(0, 0, 0), ChVector3d(0.6, 0.6, 0.6));
        }
    }
}

template <int TIRE_TYPE, bool OBJECTS>
HmmwvScmTest<TIRE_TYPE, OBJECTS>::~HmmwvScmTest() {
    delete m_hmmwv;
    delete m_terrain;
    delete m_driver;
}

template <int TIRE_TYPE, bool OBJECTS>
void HmmwvScmTest<TIRE_TYPE, OBJECTS>::ExecuteStep() {
    double time = m_hmmwv->GetSystem()->GetChTime();

    // Driver inputs
    DriverInputs driver_inputs = m_driver->GetInputs();

    // Update modules (process inputs from other modules)
    m_driver->Synchronize(time);
    m_terrain->Synchronize(time);
    m_hmmwv->Synchronize(time, driver_inputs, *m_terrain);

    // Advance simulation for one timestep for all modules
    m_driver->Advance(m_step);
    m_terrain->Advance(m_step);
    m_hmmwv->Advance(m_step);

    m_scm.Accumulate(*m_terrain);
}

template <int TIRE_TYPE, bool OBJECTS>
void HmmwvScmTest<TIRE_TYPE, OBJECTS>::SimulateVis() {
#ifdef CHRONO_IRRLICHT
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->AttachVehicle(&m_hmmwv->GetVehicle());
    vis->SetWindowTitle("HMMWV SMC benchmark");
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->Initialize();
    vis->AddLightDirectional();
    vis->AddSkyBox();

    while (vis->Run()) {
        DriverInputs driver_inputs = m_driver->GetInputs();

        vis->BeginScene();
        vis->Render();
        ExecuteStep();
        vis->Synchronize(m_hmmwv->GetSystem()->GetChTime(), driver_inputs);
        vis->Advance(m_step);
        vis->EndScene();
    }
#endif
}

// =============================================================================

#define NUM_SKIP_STEPS 500  // number of steps for hot start (2e-3 * 500 = 1s)
#define NUM_SIM_STEPS 2000  // number of simulation steps for each benchmark (2e-3 * 2000 = 4s)
#define REPEATS 10

// NOTE: trick to prevent erros in expanding macros due to types that contain a comma.
typedef HmmwvScmTest<MESH_TIRE, false> mesh_0_test_type;
typedef HmmwvScmTest<CYL_TIRE, false> cyl_0_test_type;
typedef HmmwvScmTest<MESH_TIRE, true> mesh_1_test_type;
typedef HmmwvScmTest<CYL_TIRE, true> cyl_1_test_type;

CH_BM_SCM_SIMULATION_ONCE(HmmwvSCM_MESH_0, mesh_0_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SCM_SIMULATION_ONCE(HmmwvSCM_CYL_0, cyl_0_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SCM_SIMULATION_ONCE(HmmwvSCM_MESH_1, mesh_1_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SCM_SIMULATION_ONCE(HmmwvSCM_CYL_1, cyl_1_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);

// =============================================================================

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);

#ifdef CHRONO_IRRLICHT
    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        scm_render = true;  // must be set before the fixture builds the terrain
        // SCM_BENCH_VARIANT picks which of the four registered variants to render. CYL_0 and CYL_1
        // are not benchmarked -- a cylinder tyre is a primitive collision shape, which the GPU
        // ray-cast backend cannot intersect -- but they are worth being able to look at, since that
        // is the failure this selector makes visible.
        const char* e = std::getenv("SCM_BENCH_VARIANT");
        const std::string variant = e ? e : "MESH_1";
        std::cout << "SCM variant: " << variant << std::endl;
        if (variant == "MESH_0") {
            HmmwvScmTest<MESH_TIRE, false> test;
            test.SimulateVis();
        } else if (variant == "CYL_0") {
            HmmwvScmTest<CYL_TIRE, false> test;
            test.SimulateVis();
        } else if (variant == "CYL_1") {
            HmmwvScmTest<CYL_TIRE, true> test;
            test.SimulateVis();
        } else {
            HmmwvScmTest<MESH_TIRE, true> test;
            test.SimulateVis();
        }
        return 0;
    }
#endif

    ::benchmark::RunSpecifiedBenchmarks();
}
