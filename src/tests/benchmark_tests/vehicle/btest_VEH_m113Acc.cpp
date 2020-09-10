// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
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
// Benchmark test for M113 acceleration test.
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/utils/ChBenchmark.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_models/vehicle/m113/M113_SimplePowertrain.h"
#include "chrono_models/vehicle/m113/M113_Vehicle.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleIrrApp.h"
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

// =============================================================================

template <typename EnumClass, EnumClass SHOE_TYPE>
class M113AccTest : public utils::ChBenchmarkTest {
public:
    M113AccTest();
    ~M113AccTest();

    ChSystem* GetSystem() override { return m_m113->GetSystem(); }
    void ExecuteStep() override;

    void SimulateVis();

private:
    M113_Vehicle* m_m113;
    RigidTerrain* m_terrain;
    ChPathFollowerDriver* m_driver;

    TerrainForces m_shoeL;
    TerrainForces m_shoeR;

    double m_step;
};

template <typename EnumClass, EnumClass SHOE_TYPE>
M113AccTest<EnumClass, SHOE_TYPE>::M113AccTest() : m_step(1e-3) {
    BrakeType brake_type = BrakeType::SIMPLE;
    ChContactMethod contact_method = ChContactMethod::NSC;
    ChassisCollisionType chassis_collision_type = ChassisCollisionType::NONE;

    // Create the M113 vehicle, set parameters, and initialize.
    m_m113 = new M113_Vehicle(false, SHOE_TYPE, brake_type, contact_method, chassis_collision_type);
    m_m113->Initialize(ChCoordsys<>(ChVector<>(-250 + 5, 0, 1.1), ChQuaternion<>(1, 0, 0, 0)));

    m_m113->SetChassisVisualizationType(VisualizationType::NONE);
    m_m113->SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    m_m113->SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    m_m113->SetRoadWheelAssemblyVisualizationType(VisualizationType::PRIMITIVES);
    m_m113->SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    m_m113->SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    // Create the powertrain
    auto powertrain = chrono_types::make_shared<M113_SimplePowertrain>("Powertrain");
    m_m113->InitializePowertrain(powertrain);

    // Create the terrain
    m_terrain = new RigidTerrain(m_m113->GetSystem());
    auto patch_material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    patch_material->SetFriction(0.9f);
    patch_material->SetRestitution(0.01f);
    patch_material->SetYoungModulus(2e7f);
    auto patch = m_terrain->AddPatch(patch_material, ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 500, 5);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.8f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 500, 5);
    m_terrain->Initialize();

    // Create the straight path and the driver system
    auto path = StraightLinePath(ChVector<>(-250, 0, 0.5), ChVector<>(250, 0, 0.5), 1);
    m_driver = new ChPathFollowerDriver(*m_m113, path, "my_path", 1000.0);
    m_driver->GetSteeringController().SetLookAheadDistance(5.0);
    m_driver->GetSteeringController().SetGains(0.5, 0, 0);
    m_driver->GetSpeedController().SetGains(0.4, 0, 0);
    m_driver->Initialize();

    // Solver settings
    auto solver = chrono_types::make_shared<ChSolverPSOR>();
    solver->SetMaxIterations(50);
    solver->SetOmega(0.8);
    solver->SetSharpnessLambda(1.0);
    m_m113->GetSystem()->SetSolver(solver);

    m_m113->GetSystem()->SetMaxPenetrationRecoverySpeed(1.5);
    m_m113->GetSystem()->SetMinBounceSpeed(2.0);

    m_shoeL.resize(m_m113->GetNumTrackShoes(LEFT));
    m_shoeR.resize(m_m113->GetNumTrackShoes(RIGHT));
}

template <typename EnumClass, EnumClass SHOE_TYPE>
M113AccTest<EnumClass, SHOE_TYPE>::~M113AccTest() {
    delete m_m113;
    delete m_terrain;
    delete m_driver;
}

template <typename EnumClass, EnumClass SHOE_TYPE>
void M113AccTest<EnumClass, SHOE_TYPE>::ExecuteStep() {
    double time = m_m113->GetChTime();

    if (time < 0.5) {
        m_driver->SetDesiredSpeed(0);
    } else {
        m_driver->SetDesiredSpeed(1000);
    }

    // Driver inputs
    ChDriver::Inputs driver_inputs = m_driver->GetInputs();

    // Update modules (process inputs from other modules)
    m_driver->Synchronize(time);
    m_terrain->Synchronize(time);
    m_m113->Synchronize(time, driver_inputs, m_shoeL, m_shoeR);

    // Advance simulation for one timestep for all modules
    m_driver->Advance(m_step);
    m_terrain->Advance(m_step);
    m_m113->Advance(m_step);
}

template <typename EnumClass, EnumClass SHOE_TYPE>
void M113AccTest<EnumClass, SHOE_TYPE>::SimulateVis() {
#ifdef CHRONO_IRRLICHT
    ChTrackedVehicleIrrApp app(m_m113, L"M113 acceleration test");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(-200.f, -30.f, 100.f), irr::core::vector3df(-200.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 0.0), 6.0, 0.5);

    app.AssetBindAll();
    app.AssetUpdateAll();

    while (app.GetDevice()->run()) {
        ChDriver::Inputs driver_inputs = m_driver->GetInputs();

        app.BeginScene();
        app.DrawAll();
        ExecuteStep();
        app.Synchronize("Acceleration test", driver_inputs);
        app.Advance(m_step);
        app.EndScene();
    }
#endif
}

// =============================================================================

#define NUM_SKIP_STEPS 1000  // number of steps for hot start
#define NUM_SIM_STEPS 1000   // number of simulation steps for each benchmark
#define REPEATS 10

// NOTE: trick to prevent erros in expanding macros due to types that contain a comma.
typedef M113AccTest<TrackShoeType, TrackShoeType::SINGLE_PIN> sp_test_type;
typedef M113AccTest<TrackShoeType, TrackShoeType::DOUBLE_PIN> dp_test_type;

CH_BM_SIMULATION_LOOP(M113Acc_SP, sp_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SIMULATION_LOOP(M113Acc_DP, dp_test_type, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);

// =============================================================================

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);

#ifdef CHRONO_IRRLICHT
    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        M113AccTest<TrackShoeType, TrackShoeType::SINGLE_PIN> test;
        ////M113AccTest<TrackShoeType, TrackShoeType::DOUBLE_PIN> test;
        test.SimulateVis();
        return 0;
    }
#endif

    ::benchmark::RunSpecifiedBenchmarks();
}
