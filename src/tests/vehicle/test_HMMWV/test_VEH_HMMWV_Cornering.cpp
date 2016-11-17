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
// HMMWV constant radius turn test.
// This test uses a parameterized circular Bezier path.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <cmath>
#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

void CalcControlPoints(double run,
                       double radius,
                       double offset,
                       int nturns,
                       std::vector<ChVector<>>& points,
                       std::vector<ChVector<>>& inCV,
                       std::vector<ChVector<>>& outCV) {
    // Height of path
    double z = 0.1;

    // Approximate circular path using 4 points
    double factor = radius * (4.0 / 3.0) * std::tan(CH_C_PI / 8);

    ChVector<> P1(radius + offset, -radius, z);
    ChVector<> P1_in = P1 - ChVector<>(factor, 0, 0);
    ChVector<> P1_out = P1 + ChVector<>(factor, 0, 0);

    ChVector<> P2(2 * radius + offset, 0, z);
    ChVector<> P2_in = P2 - ChVector<>(0, factor, 0);
    ChVector<> P2_out = P2 + ChVector<>(0, factor, 0);

    ChVector<> P3(radius + offset, radius, z);
    ChVector<> P3_in = P3 + ChVector<>(factor, 0, 0);
    ChVector<> P3_out = P3 - ChVector<>(factor, 0, 0);

    ChVector<> P4(offset, 0, z);
    ChVector<> P4_in = P4 + ChVector<>(0, factor, 0);
    ChVector<> P4_out = P4 - ChVector<>(0, factor, 0);

    // Start point
    ChVector<> P0(-run, -radius, z);
    ChVector<> P0_in = P0;
    ChVector<> P0_out = P0 + ChVector<>(factor, 0, 0);

    points.push_back(P0);
    inCV.push_back(P0_in);
    outCV.push_back(P0_out);

    for (int i = 0; i < nturns; i++) {
        points.push_back(P1);
        inCV.push_back(P1_in);
        outCV.push_back(P1_out);

        points.push_back(P2);
        inCV.push_back(P2_in);
        outCV.push_back(P2_out);

        points.push_back(P3);
        inCV.push_back(P3_in);
        outCV.push_back(P3_out);

        points.push_back(P4);
        inCV.push_back(P4_in);
        outCV.push_back(P4_out);
    }
}

// =============================================================================

int main(int argc, char* argv[]) {
    // -------------------------------
    // Parameters for the Bezier curve
    // -------------------------------

    double radius = 20;
    double offset = 1;
    double run = 30;
    int nturns = 4;

    // Terrain dimensions
    double terrainHeight = 0;
    double terrainLength = 20 + 2 * std::max(run, 2 * radius + offset);
    double terrainWidth = 20 + 2 * radius;

    // Initial vehicle location
    ChVector<> initLoc(-run, -radius, 0.5);

    // ---------------------------------------------
    // Parameters for steering and speed controllers
    // ---------------------------------------------

    double look_ahead_dist = 5;
    double Kp_steering = 0.5;
    double Ki_steering = 0;
    double Kd_steering = 0;

    double target_speed = 12;
    double Kp_speed = 0.4;
    double Ki_speed = 0;
    double Kd_speed = 0;

    // ---------------------
    // Simulation parameters
    // ---------------------

    // Simulation step size
    double step_size = 1e-3;

    // Simulation end time
    double t_end = 100;

    // Render FPS
    double fps = 60;

    // ------------------------------
    // Create the vehicle and terrain
    // ------------------------------

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(ChMaterialSurfaceBase::DEM);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc));
    my_hmmwv.SetInitFwdVel(1.5 * target_speed);
    my_hmmwv.SetPowertrainType(PowertrainModelType::SHAFTS);
    my_hmmwv.SetDriveType(DrivelineType::RWD);
    my_hmmwv.SetTireType(TireModelType::RIGID);
    my_hmmwv.SetTireStepSize(step_size);
    my_hmmwv.Initialize();

    my_hmmwv.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetWheelVisualizationType(VisualizationType::NONE);
    my_hmmwv.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // Create the terrain
    RigidTerrain terrain(my_hmmwv.GetSystem());
    terrain.SetContactFrictionCoefficient(0.9f);
    terrain.SetContactRestitutionCoefficient(0.01f);
    terrain.SetContactMaterialProperties(2e7f, 0.3f);
    terrain.SetColor(ChColor(1, 1, 1));
    terrain.SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 100, 50);
    terrain.Initialize(terrainHeight, terrainLength, terrainWidth);

    // ----------------------------------------------------------
    // Create the Bezier path and the path-follower driver system
    // ----------------------------------------------------------

    std::vector<ChVector<>> points;
    std::vector<ChVector<>> inCV;
    std::vector<ChVector<>> outCV;
    CalcControlPoints(run, radius, offset, nturns, points, inCV, outCV);
    ChBezierCurve path(points, inCV, outCV);

    ChPathFollowerDriver driver(my_hmmwv.GetVehicle(), &path, "my_path", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(look_ahead_dist);
    driver.GetSteeringController().SetGains(Kp_steering, Ki_steering, Kd_steering);
    driver.GetSpeedController().SetGains(Kp_speed, Ki_speed, Kd_speed);

    driver.Initialize();

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    ChVehicleIrrApp app(&my_hmmwv.GetVehicle(), &my_hmmwv.GetPowertrain(), L"Constant radius turn test",
                        irr::core::dimension2d<irr::u32>(800, 640));

    app.SetHUDLocation(500, 20);
    app.SetSkyBox();
    app.AddTypicalLogo();
    app.AddTypicalLights(irr::core::vector3df(-250.f, -250.f, 200.f), irr::core::vector3df(-250.f, 250.f, 200.f));
    app.AddTypicalLights(irr::core::vector3df(250.f, -250.f, 200.f), irr::core::vector3df(250.0f, 250.f, 200.f));
    app.EnableGrid(false);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);

    app.SetTimestep(step_size);

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = app.GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = app.GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);

    // Finalize construction of visualization assets
    app.AssetBindAll();
    app.AssetUpdateAll();

    // ---------------
    // Simulation loop
    // ---------------

    // Driver location in vehicle local frame
    ChVector<> driver_pos = my_hmmwv.GetChassis()->GetLocalDriverCoordsys().pos;

    // Number of simulation steps between miscellaneous events
    double render_step_size = 1 / fps;
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    ChRealtimeStepTimer realtime_timer;
    int sim_frame = 0;
    int render_frame = 0;

    while (app.GetDevice()->run()) {
        // Extract system state
        double time = my_hmmwv.GetSystem()->GetChTime();
        ChVector<> acc_CG = my_hmmwv.GetVehicle().GetChassisBody()->GetPos_dtdt();
        ChVector<> acc_driver = my_hmmwv.GetVehicle().GetVehicleAcceleration(driver_pos);

        // End simulation
        if (time >= t_end)
            break;

        // Collect output data from modules (for inter-module communication)
        double throttle_input = driver.GetThrottle();
        double steering_input = driver.GetSteering();
        double braking_input = driver.GetBraking();

        // Update sentinel and target location markers for the path-follower controller.
        // Note that we do this whether or not we are currently using the path-follower driver.
        const ChVector<>& pS = driver.GetSteeringController().GetSentinelLocation();
        const ChVector<>& pT = driver.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x, (irr::f32)pS.y, (irr::f32)pS.z));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x, (irr::f32)pT.y, (irr::f32)pT.z));

        // Render scene and output POV-Ray data
        if (sim_frame % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();

            render_frame++;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, steering_input, braking_input, throttle_input, terrain);
        app.Synchronize("Follower driver", steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        my_hmmwv.Advance(step_size);
        app.Advance(step_size);

        // Increment simulation frame number
        sim_frame++;
    }

    return 0;
}
