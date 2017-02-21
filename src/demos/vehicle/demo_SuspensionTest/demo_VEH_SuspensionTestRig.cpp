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
// Demonstration program for a suspension test rig.
//
// A test rig can be instantiated either from a vehicle JSON specification file
// (by indicating the axle to be used in the rig), or from a test rig JSON
// specification file (with or without a steering mechanism).
//
// Driver inputs for a suspension test rig include left/right post displacements
// and steering input (the latter being ignored if the tested suspension is not
// attached to a steering mechanism).  These driver inputs can be obtained from
// an interactive driver system (of type ChIrrGuiDriverSTR) or from a data file
// (using a driver system of type ChDataDriverSTR).
//
// If data collection is enabled, an output file named 'output.dat' will be
// generated in the directory specified by the variable out_dir. This ASCII file
// contains one line per output time, each with the following information:
//  [col  1]     time
//  [col  2]     left post input, a value in [-1,1]
//  [col  3]     right post input, a value in [-1,1]
//  [col  4]     steering input, a value in [-1,1]
//  [col  5]     actual left post dispalcement
//  [col  6]     actual right post displacement
//  [col  7- 9]  application point for left tire force
//  [col 10-12]  left tire force
//  [col 13-15]  left tire moment
//  [col 16-18]  application point for right tire force
//  [col 19-21]  right tire force
//  [col 22-24]  right tire moment
//
// Tire forces are expressed in the global frame, as applied to the center of
// the associated wheel.
//
// =============================================================================

#include "chrono/core/ChFileutils.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChIrrGuiDriverSTR.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChDataDriverSTR.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================
// USER SETTINGS
// =============================================================================

// Simulation step size
double step_size = 1e-3;

// Time interval between two render frames (1/FPS)
double render_step_size = 1.0 / 50;

// Specification of tested suspension:
//   'true':  use suspension from a vehicle JSON file?
//   'false': use JSON suspension test rig file
bool use_vehicle_file = true;

// Specification of test rig inputs:
//   'true':  use driver inputs from file
//   'false': use interactive Irrlicht driver
bool use_data_driver = false;

// JSON file for suspension test rig
////std::string str_file("hmmwv/suspensionTest/HMMWV_ST_front.json");
std::string str_file("hmmwv/suspensionTest/HMMWV_ST_rear.json");

// JSON file for vehicle and axle index
std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle.json");
int axle_index = 0;
double post_limit = 0.15;

// File with driver inputs
std::string driver_file("hmmwv/suspensionTest/ST_inputs.dat");

// JSON files for tire models (rigid)
std::string tire_file("hmmwv/tire/HMMWV_RigidTire.json");

// Output collection
bool collect_output = false;
std::string out_dir = "../SUSPENSION_TEST_RIG";
double out_step_size = 1.0 / 100;

// =============================================================================
int main(int argc, char* argv[]) {
    // Use rigid wheels to actuate suspension.
    auto tire_L = std::make_shared<RigidTire>(vehicle::GetDataFile(tire_file));
    auto tire_R = std::make_shared<RigidTire>(vehicle::GetDataFile(tire_file));

    // Create the suspension test rig.
    std::unique_ptr<ChSuspensionTestRig> rig;
    if (use_vehicle_file) {
        // From a vehicle JSON specification file (selecting a particular axle)
        rig = std::unique_ptr<ChSuspensionTestRig>(
            new ChSuspensionTestRig(vehicle::GetDataFile(vehicle_file), axle_index, post_limit, tire_L, tire_R));
    } else {
        // From a suspension test rig JSON specification file
        rig = std::unique_ptr<ChSuspensionTestRig>(
            new ChSuspensionTestRig(vehicle::GetDataFile(str_file), tire_L, tire_R));
    }

    // Initialize suspension test rig (this automatically initializes tires).
    rig->Initialize(ChCoordsys<>());

    rig->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetWheelVisualizationType(VisualizationType::PRIMITIVES);
    if (rig->HasSteering()) {
        rig->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    }
    rig->SetTireVisualizationType(VisualizationType::MESH);

    // Create the vehicle Irrlicht application.
    ChVehicleIrrApp app(rig.get(), NULL, L"Suspension Test Rig");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(0.5 * (rig->GetWheelPos(LEFT) + rig->GetWheelPos(RIGHT)), 2.0, 1.0);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // Create and initialize the driver system.
    std::unique_ptr<ChDriverSTR> driver;
    if (use_data_driver) {
        // Driver with inputs from file
        auto data_driver = new ChDataDriverSTR(*rig, vehicle::GetDataFile(driver_file));
        driver = std::unique_ptr<ChDriverSTR>(data_driver);
    } else {
        // Interactive driver
        auto irr_driver = new ChIrrGuiDriverSTR(app);
        double steering_time = 1.0;      // time to go from 0 to max
        double displacement_time = 2.0;  // time to go from 0 to max applied post motion
        irr_driver->SetSteeringDelta(render_step_size / steering_time);
        irr_driver->SetDisplacementDelta(render_step_size / displacement_time);
        driver = std::unique_ptr<ChDriverSTR>(irr_driver);
    }
    driver->Initialize();

    // Initialize output
    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    std::string out_file = out_dir + "/output.dat";
    utils::CSV_writer out_csv(" ");

    // ---------------
    // Simulation loop
    // ---------------

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Number of simulation steps between two data collection frames
    int out_steps = (int)std::ceil(out_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;
    ChRealtimeStepTimer realtime_timer;

    while (app.GetDevice()->run()) {
        // Render scene
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();
        }

        // Collect output data from modules
        double steering_input = driver->GetSteering();
        double left_input = driver->GetDisplacementLeft();
        double right_input = driver->GetDisplacementRight();

        // Update modules (process inputs from other modules)
        double time = rig->GetChTime();
        driver->Synchronize(time);
        rig->Synchronize(time, steering_input, left_input, right_input);
        app.Synchronize("", steering_input, 0, 0);

        // Write output data
        if (collect_output && step_number % out_steps == 0) {
            // Current tire forces
            auto tire_force_L = tire_L->GetTireForce(true);
            auto tire_force_R = tire_R->GetTireForce(true);
            out_csv << time << left_input << right_input << steering_input;
            out_csv << rig->GetActuatorDisp(VehicleSide::LEFT) << rig->GetActuatorDisp(VehicleSide::RIGHT);
            out_csv << tire_force_L.point << tire_force_L.force << tire_force_L.moment;
            out_csv << tire_force_R.point << tire_force_R.force << tire_force_R.moment;
            out_csv << std::endl;
        }

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);
        driver->Advance(step);
        rig->Advance(step);
        app.Advance(step);

        // Increment frame number
        step_number++;
    }

    // Write output file
    if (collect_output) {
        out_csv.write_to_file(out_file);
    }

    return 0;
}
