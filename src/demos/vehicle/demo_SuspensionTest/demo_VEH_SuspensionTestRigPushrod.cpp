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
// Demonstration program for a suspension test rig with direct actuation on the
// spindle bodies.
//
// A test rig can be instantiated either from a vehicle JSON specification file
// (by indicating the axle to be used in the rig), or from a test rig JSON
// specification file (with or without a steering mechanism).
//
// Driver inputs for a suspension test rig include left/right rod displacements
// and steering input (the latter being ignored if the tested suspension is not
// attached to a steering mechanism).  These driver inputs can be obtained from
// an interactive driver system (of type ChIrrGuiDriverSTR) or from a data file
// (using a driver system of type ChDataDriverSTR).
//
// If data collection is enabled, an output file named 'output.dat' will be
// generated in the directory specified by the variable out_dir. This ASCII file
// contains one line per output time, each with the following information:
//  [col 1]      time
//  [col 2-4]    left input, left spindle z, left wheel travel
//  [col 5-6]    left spring force, left shock force
//  [col 7-9]    right input, right spindle z, right wheel travel
//  [col 10-11]  right spring force, right shock force
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/utils/ChVehicleIrrApp.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChIrrGuiDriverSTR.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChDataDriverSTR.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================
// USER SETTINGS
// =============================================================================

// Simulation step size
double step_size = 1e-3;

// Specification of tested suspension:
//   'true':  use suspension from a vehicle JSON file (extract specified axle)
//   'false': use JSON suspension test rig file
bool use_vehicle_file = true;

// JSON file for vehicle and axle index
std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle.json");
int axle_index = 0;
double post_limit = 0.15;

// JSON file for suspension test rig
////std::string str_file("hmmwv/suspensionTest/HMMWV_ST_front.json");
std::string str_file("hmmwv/suspensionTest/HMMWV_ST_rear.json");

// Specification of test rig inputs:
//   'true':  use driver inputs from file
//   'false': use interactive Irrlicht driver
bool use_data_driver = true;
std::string driver_file("hmmwv/suspensionTest/ST_inputs.dat");

// JSON files for tire models (rigid)
////std::string tire_file("hmmwv/tire/HMMWV_RigidTire.json");
////std::string tire_file("hmmwv/tire/HMMWV_RigidMeshTire_Coarse.json");
////std::string tire_file("hmmwv/tire/HMMWV_Fiala_converted.json");
std::string tire_file("hmmwv/tire/HMMWV_TMeasyTire.json");
////std::string tire_file("hmmwv/tire/HMMWV_Pac02Tire.json");
////std::string tire_file("hmmwv/tire/HMMWV_Pac89Tire.json");

// Output collection
bool collect_output = true;
std::string out_dir = GetChronoOutputPath() + "SUSPENSION_TEST_RIG_PUSHROD";
double out_step_size = 1.0 / 100;

// =============================================================================
int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create tires.
    auto tire_L = ReadTireJSON(vehicle::GetDataFile(tire_file));
    auto tire_R = ReadTireJSON(vehicle::GetDataFile(tire_file));

    // Create the suspension test rig.
    std::unique_ptr<ChSuspensionTestRigPushrod> rig;
    if (use_vehicle_file) {
        // From a vehicle JSON specification file (selecting a particular axle)
        rig = std::unique_ptr<ChSuspensionTestRigPushrod>(
            new ChSuspensionTestRigPushrod(vehicle::GetDataFile(vehicle_file), axle_index, post_limit, tire_L, tire_R));
    } else {
        // From a suspension test rig JSON specification file
        rig = std::unique_ptr<ChSuspensionTestRigPushrod>(
            new ChSuspensionTestRigPushrod(vehicle::GetDataFile(str_file), tire_L, tire_R));
    }

    rig->SetInitialRideHeight(0.5);

    rig->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetWheelVisualizationType(VisualizationType::NONE);
    if (rig->HasSteering()) {
        rig->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    }
    rig->SetTireVisualizationType(VisualizationType::NONE);

    // Create the vehicle Irrlicht application.
    ChVehicleIrrApp app(rig.get(), L"Suspension Test Rig");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(0.5 * (rig->GetSpindlePos(LEFT) + rig->GetSpindlePos(RIGHT)), 2.0, 0.5);
    app.SetTimestep(step_size);

    // Create and attach the driver system.
    std::shared_ptr<ChDriverSTR> driver;
    if (use_data_driver) {
        // Driver with inputs from file
        auto data_driver = chrono_types::make_shared<ChDataDriverSTR>(vehicle::GetDataFile(driver_file));
        driver = data_driver;
    } else {
        // Interactive driver
        auto irr_driver = chrono_types::make_shared<ChIrrGuiDriverSTR>(app);
        irr_driver->SetSteeringDelta(1.0 / 50);
        irr_driver->SetDisplacementDelta(1.0 / 250);
        driver = irr_driver;
    }
    rig->SetDriver(driver);

    // Initialize suspension test rig.
    rig->Initialize();

    app.AssetBindAll();
    app.AssetUpdateAll();

    // Initialize output
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    std::string out_file = out_dir + "/" + rig->GetSuspension()->GetTemplateName() + ".dat";
    utils::CSV_writer out_csv(" ");

    std::cout << "Rig mass: " << rig->GetMass() << std::endl;

    // ---------------
    // Simulation loop
    // ---------------

    // Number of simulation steps between two data collection frames
    int out_steps = (int)std::ceil(out_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;

    while (app.GetDevice()->run()) {
        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();

        // Write output data
        if (collect_output && driver->Started() && step_number % out_steps == 0) {
            double time = rig->GetChTime();
            auto frc_left = rig->ReportSuspensionForce(LEFT);
            auto frc_right = rig->ReportSuspensionForce(RIGHT);
            out_csv << time;
            out_csv << rig->GetLeftInput() << rig->GetSpindlePos(LEFT).z() << rig->GetWheelTravel(LEFT);
            out_csv << frc_left.spring_force << frc_left.shock_force;
            out_csv << rig->GetRightInput() << rig->GetSpindlePos(RIGHT).z() << rig->GetWheelTravel(RIGHT);
            out_csv << frc_right.spring_force << frc_right.shock_force;
            out_csv << std::endl;
        }

        // Advance simulation of the rig
        rig->Advance(step_size);

        // Update visualization app
        app.Synchronize(tire_L->GetTemplateName(), {rig->GetSteeringInput(), 0, 0});
        app.Advance(step_size);

        if (driver->Ended())
            break;

        // Increment frame number
        step_number++;
    }

    // Write output file
    if (collect_output) {
        out_csv.write_to_file(out_file);
    }

    return 0;
}
