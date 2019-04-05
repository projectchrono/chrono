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
    M113_SimplePowertrain* m_powertrain;
    RigidTerrain* m_terrain;
    ChPathFollowerDriver* m_driver;

    TerrainForces m_shoeL;
    TerrainForces m_shoeR;

    double m_step;
};

template <typename EnumClass, EnumClass SHOE_TYPE>
M113AccTest<EnumClass, SHOE_TYPE>::M113AccTest() : m_step(1e-3) {
    ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::NSC;
    ChassisCollisionType chassis_collision_type = ChassisCollisionType::NONE;

    // Create the M113 vehicle, set parameters, and initialize.
    m_m113 = new M113_Vehicle(false, SHOE_TYPE, contact_method, chassis_collision_type);
    m_m113->Initialize(ChCoordsys<>(ChVector<>(-250 + 5, 0, 1.1), ChQuaternion<>(1, 0, 0, 0)));

    m_m113->SetChassisVisualizationType(VisualizationType::NONE);
    m_m113->SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    m_m113->SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    m_m113->SetRoadWheelAssemblyVisualizationType(VisualizationType::PRIMITIVES);
    m_m113->SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    m_m113->SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    // Create the powertrain
    m_powertrain = new M113_SimplePowertrain("Powertrain");
    m_powertrain->Initialize(m_m113->GetChassisBody(), m_m113->GetDriveshaft());

    // Create the terrain
    m_terrain = new RigidTerrain(m_m113->GetSystem());
    auto patch = m_terrain->AddPatch(ChCoordsys<>(ChVector<>(0, 0, -5), QUNIT), ChVector<>(500, 5, 10));
    patch->SetContactFrictionCoefficient(0.9f);
    patch->SetContactRestitutionCoefficient(0.01f);
    patch->SetContactMaterialProperties(2e7f, 0.3f);
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
    m_m113->GetSystem()->SetSolverType(ChSolver::Type::SOR);
    m_m113->GetSystem()->SetMaxItersSolverSpeed(50);
    m_m113->GetSystem()->SetMaxItersSolverStab(50);
    m_m113->GetSystem()->SetTol(0);
    m_m113->GetSystem()->SetMaxPenetrationRecoverySpeed(1.5);
    m_m113->GetSystem()->SetMinBounceSpeed(2.0);
    m_m113->GetSystem()->SetSolverOverrelaxationParam(0.8);
    m_m113->GetSystem()->SetSolverSharpnessParam(1.0);

    m_shoeL.resize(m_m113->GetNumTrackShoes(LEFT));
    m_shoeR.resize(m_m113->GetNumTrackShoes(RIGHT));
}

template <typename EnumClass, EnumClass SHOE_TYPE>
M113AccTest<EnumClass, SHOE_TYPE>::~M113AccTest() {
    delete m_m113;
    delete m_powertrain;
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

    // Collect output data from modules (for inter-module communication)
    double throttle_input = m_driver->GetThrottle();
    double steering_input = m_driver->GetSteering();
    double braking_input = m_driver->GetBraking();
    double powertrain_torque = m_powertrain->GetOutputTorque();
    double driveshaft_speed = m_m113->GetDriveshaftSpeed();

    // Update modules (process inputs from other modules)
    m_driver->Synchronize(time);
    m_terrain->Synchronize(time);
    m_m113->Synchronize(time, steering_input, braking_input, powertrain_torque, m_shoeL, m_shoeR);
    m_powertrain->Synchronize(time, throttle_input, driveshaft_speed);

    // Advance simulation for one timestep for all modules
    m_driver->Advance(m_step);
    m_terrain->Advance(m_step);
    m_m113->Advance(m_step);
    m_powertrain->Advance(m_step);
}

template <typename EnumClass, EnumClass SHOE_TYPE>
void M113AccTest<EnumClass, SHOE_TYPE>::SimulateVis() {
#ifdef CHRONO_IRRLICHT
    ChTrackedVehicleIrrApp app(m_m113, m_powertrain, L"M113 acceleration test");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(-200.f, -30.f, 100.f), irr::core::vector3df(-200.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 0.0), 6.0, 0.5);

    app.AssetBindAll();
    app.AssetUpdateAll();

    while (app.GetDevice()->run()) {
        app.BeginScene();
        app.DrawAll();
        ExecuteStep();
        app.Synchronize("Acceleration test", m_driver->GetSteering(), m_driver->GetThrottle(), m_driver->GetBraking());
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
