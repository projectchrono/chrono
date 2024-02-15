// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Verification of different subsystem combinations using the Generic Vehicle.
//
// The vehicle reference frames have Z up, X towards the front of the vehicle,
// and Y pointing to the left.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_models/vehicle/generic/Generic_Vehicle.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/driver/ChInteractiveDriverVSG.h"
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::generic;

// =============================================================================

////std::vector<SuspensionTypeWV> suspension_types_front = {SuspensionTypeWV::DOUBLE_WISHBONE,
////                                                        SuspensionTypeWV::MACPHERSON_STRUT};
////
std::vector<SuspensionTypeWV> suspension_types_rear = {SuspensionTypeWV::DOUBLE_WISHBONE_REDUCED,
                                                       SuspensionTypeWV::MULTI_LINK};

std::vector<SuspensionTypeWV> suspension_types_front = {SuspensionTypeWV::DOUBLE_WISHBONE};
////std::vector<SuspensionTypeWV> suspension_types_rear = {SuspensionTypeWV::RIGID_SUSPENSION};

SteeringTypeWV steering_type = SteeringTypeWV::PITMAN_ARM;
BrakeType brake_type = BrakeType::SHAFTS;
DrivelineTypeWV driveline_type = DrivelineTypeWV::AWD;

EngineModelType engine_type = EngineModelType::SHAFTS;
TransmissionModelType transmission_type = TransmissionModelType::AUTOMATIC_SHAFTS;

TireModelType tire_type = TireModelType::PAC02;

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Simulation step size
double step_size = 1e-3;

// =============================================================================

class Driver : public ChDriver {
  public:
    Driver(ChVehicle& vehicle, double frequency, double delay)
        : ChDriver(vehicle), m_frequency(frequency), m_delay(delay) {}
    ~Driver() {}

    virtual void Synchronize(double time) override {
        double throttle_max = 0.25;
        double steering_max = 0.40;

        if (time < m_delay / 2)
            m_throttle = 0;
        else if (time < m_delay)
            m_throttle = (2 * throttle_max / m_delay) * (time - m_delay / 2);
        else
            m_throttle = throttle_max;

        if (time < m_delay)
            m_steering = 0;
        else
            m_steering = steering_max * std::sin(CH_C_2PI * m_frequency * (time - m_delay));

        m_steering *= std::exp(-time / 10);

        m_braking = 0.0;
    }

    double m_frequency;
    double m_delay;
};

// =============================================================================

Generic_Vehicle CreateVehicle(ChSystem& sys,
                              ChVector<> location,
                              SuspensionTypeWV suspension_type_front,
                              SuspensionTypeWV suspension_type_rear) {
    // Construct vehicle
    Generic_Vehicle vehicle(&sys,                   // containing system
                            false,                  // fixed chassis
                            suspension_type_front,  // front suspension type
                            suspension_type_rear,   // rear suspension type
                            steering_type,          // sterring mechanism type
                            driveline_type,         // driveline type
                            brake_type,             // brake type
                            false,                  // use bodies to model tierods
                            false                   // include an antiroll bar
    );

    // Initialize vehicle
    vehicle.Initialize(ChCoordsys<>(location, QUNIT));

    vehicle.SetChassisVisualizationType(VisualizationType::NONE);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::NONE);

    // Create and initialize the powertrain system
    vehicle.CreateAndInitializePowertrain(engine_type, transmission_type);

    // Create and initialize the tires
    vehicle.CreateAndInitializeTires(tire_type, VisualizationType::PRIMITIVES);

    return vehicle;
}

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // ------------------------
    // Create the Chrono system
    // ------------------------

    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.SetSolverMaxIterations(150);
    sys.SetMaxPenetrationRecoverySpeed(4.0);

    // -------------------
    // Create the vehicles
    // -------------------

    double x_sep = 6;
    double y_sep = 4;

    auto x_num = suspension_types_rear.size();
    auto y_num = suspension_types_front.size();

    std::vector<Generic_Vehicle> vehicles;
    auto i_ego = y_num / 2;

    for (int ix = 0; ix < x_num; ix++) {
        for (int iy = 0; iy < y_num; iy++) {
            auto loc = ChVector<>(ix * x_sep, iy * y_sep, 0.6);
            vehicles.push_back(CreateVehicle(sys, loc, suspension_types_front[iy], suspension_types_rear[ix]));
        }
    }

    // -----------------------
    // Create a vehicle driver
    // -----------------------

    Driver driver(vehicles[0], 0.25, 1.0);

    // ----------------------
    // Create a rigid terrain
    // ----------------------

    double terrain_size_x = 200;
    double terrain_size_y = x_sep * x_num + 10;

    double terrain_x = terrain_size_x / 2 - 10;
    double terrain_y = (y_num - 1) * y_num;

    RigidTerrain terrain(&sys);
    auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);
    auto patch = terrain.AddPatch(patch_mat,                                                 //
                                  ChCoordsys<>(ChVector<>(terrain_x, terrain_y, 0), QUNIT),  //
                                  terrain_size_x, terrain_size_y);
    patch->SetColor(ChColor(0.5f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // ---------------------------------
    // Create the run-time visualization
    // ---------------------------------

#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVehicleVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            // Create the vehicle Irrlicht interface
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle("Generic Vehicles");
            vis_irr->SetChaseCamera(VNULL, 6.0, 1.5);
            ////vis_irr->SetChaseCameraState(utils::ChChaseCamera::Track);
            ////vis_irr->SetChaseCameraPosition(ChVector<>(-6, terrain_y, 3.0));
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AttachVehicle(&vehicles[i_ego]);

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            // Create the vehicle VSG interface
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle("Generic Vehicles");
            vis_vsg->SetWindowSize(ChVector2<int>(1200, 900));
            vis_vsg->SetWindowPosition(ChVector2<int>(100, 300));
            vis_vsg->AttachVehicle(&vehicles[i_ego]);
            vis_vsg->SetChaseCamera(VNULL, 8.0, 1.5);
            ////vis_vsg->SetChaseCameraState(utils::ChChaseCamera::Track);
            ////vis_vsg->SetChaseCameraPosition(ChVector<>(-6, terrain_y, 3.0));
            vis_vsg->SetUseSkyBox(true);
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_C_PI_2, CH_C_PI_4);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // ---------------
    // Simulation loop
    // ---------------

    for (auto& vehicle : vehicles)
        vehicle.EnableRealtime(true);

    while (vis->Run()) {
        double time = sys.GetChTime();

        // Stop simulation when vehicles reach the end of the terrain patch
        bool done = false;
        for (const auto& vehicle : vehicles) {
            if (vehicle.GetPos().x() > terrain_size_x - 10)
                done = true;
        }
        if (done)
            break;

        // Render scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        for (auto& vehicle : vehicles)
            vehicle.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        for (auto& vehicle : vehicles)
            vehicle.Advance(step_size);
        vis->Advance(step_size);

        // Advance state of entire system (containing all vehicles)
        sys.DoStepDynamics(step_size);
    }

    return 0;
}
