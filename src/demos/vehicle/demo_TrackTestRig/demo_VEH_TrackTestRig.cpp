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
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "chrono_vehicle/powertrain/SimplePowertrain.h"
#include "chrono_vehicle/utils/ChVehicleIrrApp.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/utils/ChTrackTestRig.h"
#include "chrono_vehicle/tracked_vehicle/utils/ChIrrGuiDriverTTR.h"

#include "models/vehicle/m113/M113_TrackAssembly.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace m113;

// =============================================================================
// USER SETTINGS
// =============================================================================
double post_limit = 0.2;

// Simulation step size
double step_size = 1e-2;
double render_step_size = 1.0 / 50;  // Time interval between two render frames

// =============================================================================
// JSON file for track test rig
std::string suspensionTest_file("hmmwv/suspensionTest/HMMWV_ST_front.json");

// JSON file for vehicle and vehicle side
std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle.json");
int side = 0;

// =============================================================================
int main(int argc, char* argv[]) {
    // Create an M113 track assembly.
    auto track_assembly = std::make_shared<M113_TrackAssembly>(LEFT);

    // Create and initialize the testing mechanism.
    ChVector<> sprocket_loc(0, 1, 0);
    ChVector<> idler_loc(-3.92, 1, -0.12);  //// Original x value: -3.97
    std::vector<ChVector<> > susp_locs(5);
    susp_locs[0] = ChVector<>(-0.65, 1, -0.215);
    susp_locs[1] = ChVector<>(-1.3175, 1, -0.215);
    susp_locs[2] = ChVector<>(-1.985, 1, -0.215);
    susp_locs[3] = ChVector<>(-2.6525, 1, -0.215);
    susp_locs[4] = ChVector<>(-3.32, 1, -0.215);

    ChTrackTestRig rig(track_assembly, sprocket_loc, idler_loc, susp_locs, ChMaterialSurfaceBase::DVI);
    //rig.GetSystem()->Set_G_acc(ChVector<>(0, 0, 0));
    rig.GetSystem()->SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
    rig.GetSystem()->SetIterLCPmaxItersSpeed(50);
    rig.GetSystem()->SetIterLCPmaxItersStab(50);
    rig.GetSystem()->SetTol(0);
    rig.GetSystem()->SetMaxPenetrationRecoverySpeed(1.5);
    rig.GetSystem()->SetMinBounceSpeed(2.0);
    rig.GetSystem()->SetIterLCPomega(0.8);
    rig.GetSystem()->SetIterLCPsharpnessLambda(1.0);

    rig.Initialize(ChCoordsys<>());

    ////rig.SetCollide(TrackCollide::SPROCKET_LEFT | TrackCollide::SHOES_LEFT);

    // Create the vehicle Irrlicht application.
    ChVector<> target_point = rig.GetPostPosition();
    ////ChVector<> target_point = idler_loc;
    ////ChVector<> target_point = sprocket_loc;

    ChVehicleIrrApp app(&rig, NULL, L"Suspension Test Rig");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(target_point, 3.0, 1.0);
    app.SetChaseCameraPosition(target_point + ChVector<>(0, 3, 0));
    app.SetChaseCameraMultipliers(1e-4, 10);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // Create the driver system and set the time response for keyboard inputs.
    ChIrrGuiDriverTTR driver(app, post_limit);
    double steering_time = 1.0;      // time to go from 0 to max
    double displacement_time = 2.0;  // time to go from 0 to max applied post motion
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetDisplacementDelta(render_step_size / displacement_time * post_limit);
    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    TireForces tire_forces(2);
    WheelStates wheel_states(2);

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    TrackShoeForces shoe_forces(1);

    // Initialize simulation frame counter
    int step_number = 0;
    ChRealtimeStepTimer realtime_timer;

    while (app.GetDevice()->run()) {
        // Render scene
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();

            if (step_number == 2)
                app.WriteImageToFile("assembled_track.jpg");
        }

        // Collect output data from modules
        double steering_input = driver.GetSteering();
        double post_input = driver.GetDisplacement();

        // Update modules (process inputs from other modules)
        double time = rig.GetChTime();
        driver.Synchronize(time);
        rig.Synchronize(time, post_input, shoe_forces);
        app.Synchronize("", steering_input, 0, 0);

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);
        driver.Advance(step);
        rig.Advance(step);
        app.Advance(step);

        // Increment frame number
        step_number++;
    }

    return 0;
}
