// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Demonstration of a steering path-follower PID controller.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <vector>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/geometry/ChCLineBezier.h"
#include "chrono/assets/ChLineShape.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/vehicle/Vehicle.h"
#include "chrono_vehicle/powertrain/SimplePowertrain.h"
#include "chrono_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/tire/LugreTire.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehicleIrrApp.h"

#include "ModelDefs.h"
#include "generic/Generic_PathFollowerDriver.h"

using namespace chrono;
using namespace geometry;

// =============================================================================

// Type of tire model (RIGID or LUGRE)
TireModelType tire_model = RIGID;

// JSON file names for vehicle model, tire models, and (simple) powertrain
std::string vehicle_file("generic/vehicle/Vehicle_DoubleWishbones.json");
std::string rigidtire_file("generic/tire/RigidTire.json");
std::string lugretire_file("generic/tire/LugreTire.json");
std::string simplepowertrain_file("generic/powertrain/SimplePowertrain.json");
std::string path_file("pathS.txt");

// Initial vehicle position
ChVector<> initLoc(-125, -125, 0.6);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);
// ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
// ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
// ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
// ChQuaternion<> initRot(0, 0, 0, 1);

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 300.0;  // size in X direction
double terrainWidth = 300.0;   // size in Y direction

// Simulation step size
double step_size = 2e-3;

// Point on chassis tracked by the chase camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// =============================================================================

int main(int argc, char* argv[]) {
    // --------------------------
    // Create the various modules
    // --------------------------

    // Create the vehicle system
    Vehicle vehicle(vehicle::GetDataFile(vehicle_file));
    vehicle.Initialize(ChCoordsys<>(initLoc, initRot));
    ////vehicle.GetChassis()->SetBodyFixed(true);

    // Create the ground
    RigidTerrain terrain(vehicle.GetSystem(), terrainHeight, terrainLength, terrainWidth, 0.9);

    // Create and initialize the powertrain system
    SimplePowertrain powertrain(vehicle::GetDataFile(simplepowertrain_file));
    powertrain.Initialize();

    // Create and initialize the tires
    int num_axles = vehicle.GetNumberAxles();
    int num_wheels = 2 * num_axles;

    std::vector<ChSharedPtr<ChTire> > tires(num_wheels);

    switch (tire_model) {
        case RIGID: {
            std::vector<ChSharedPtr<RigidTire> > tires_rigid(num_wheels);
            for (int i = 0; i < num_wheels; i++) {
                tires_rigid[i] = ChSharedPtr<RigidTire>(new RigidTire(vehicle::GetDataFile(rigidtire_file), terrain));
                tires_rigid[i]->Initialize(vehicle.GetWheelBody(i));
                tires[i] = tires_rigid[i];
            }
            break;
        }
        case LUGRE: {
            std::vector<ChSharedPtr<LugreTire> > tires_lugre(num_wheels);
            for (int i = 0; i < num_wheels; i++) {
                tires_lugre[i] = ChSharedPtr<LugreTire>(new LugreTire(vehicle::GetDataFile(lugretire_file), terrain));
                tires_lugre[i]->Initialize(vehicle.GetWheelBody(i));
                tires[i] = tires_lugre[i];
            }
            break;
        }
    }

    // ----------------------
    // Create the Bezier path
    // ----------------------

    ChBezierCurve* path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
    ////path->write("my_path.txt");

    // To visualize the path, create a fixed body and add the path as its asset
    ChSharedPtr<ChBody> road(new ChBody);
    road->SetBodyFixed(true);
    vehicle.GetSystem()->AddBody(road);
    ChSharedPtr<ChLineShape> path_asset(new ChLineShape);
    path_asset->SetLineGeometry(ChSharedPtr<ChLineBezier>(new ChLineBezier(path)));
    path_asset->SetColor(ChColor(0.0f, 0.8f, 0.0f));
    road->AddAsset(path_asset);

    // ------------------------
    // Create the driver system
    // ------------------------

    Generic_PathFollowerDriver driver(vehicle, path);
    driver.GetSteeringController().SetLookAheadDistance(20);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.Reset();

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    ChVehicleIrrApp app(vehicle, powertrain, L"Steering Controller Demo");

    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    //app.EnableGrid(false);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);

    app.SetTimestep(step_size);

    app.AssetBindAll();
    app.AssetUpdateAll();

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = app.GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = app.GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    ChTireForces tire_forces(num_wheels);
    ChWheelStates wheel_states(num_wheels);
    double driveshaft_speed;
    double powertrain_torque;
    double throttle_input;
    double steering_input;
    double braking_input;

    ChRealtimeStepTimer realtime_timer;

    while (app.GetDevice()->run()) {
        // Render sentinel and target locations for the path-follower controller.
        const ChVector<>& pS = driver.GetSteeringController().GetSentinelLocation();
        const ChVector<>& pT = driver.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df(pS.x, pS.y, pS.z));
        ballT->setPosition(irr::core::vector3df(pT.x, pT.y, pT.z));

        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();

        // Collect output data from modules (for inter-module communication)
        throttle_input = driver.GetThrottle();
        steering_input = driver.GetSteering();
        braking_input = driver.GetBraking();
        powertrain_torque = powertrain.GetOutputTorque();
        driveshaft_speed = vehicle.GetDriveshaftSpeed();
        for (int i = 0; i < num_wheels; i++) {
            tire_forces[i] = tires[i]->GetTireForce();
            wheel_states[i] = vehicle.GetWheelState(i);
        }

        // Update modules (process inputs from other modules)
        double time = vehicle.GetSystem()->GetChTime();
        driver.Update(time);
        powertrain.Update(time, throttle_input, driveshaft_speed);
        vehicle.Update(time, steering_input, braking_input, powertrain_torque, tire_forces);
        terrain.Update(time);
        for (int i = 0; i < num_wheels; i++)
            tires[i]->Update(time, wheel_states[i]);
        app.Update("Driver inputs", steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);
        driver.Advance(step);
        powertrain.Advance(step);
        vehicle.Advance(step);
        terrain.Advance(step);
        for (int i = 0; i < num_wheels; i++)
            tires[i]->Advance(step);
        app.Advance(step);
    }

    app.GetDevice()->drop();
    return 0;
}
