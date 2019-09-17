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
//  [col 1]      time
//  [col 2-4]    left post input, right post input, steering input (in [-1,1])
//  [col 5-6]    actual left post displacement, actual right post displacement
//  [col 7]      current ride height (relative to chassis reference frame)
//  [col 8-10]   application point for left tire force
//  [col 11-13]  left tire force
//  [col 14-16]  left tire moment
//  [col 17-19]  application point for right tire force
//  [col 20-22]  right tire force
//  [col 23-25]  right tire moment
//  [col 26-29]  left angular speed (omega), long. slip (kappa), slip angle (alpha), camber angle (gamma) 
//  [col 30-33]  right angular speed (omega), long. slip (kappa), slip angle (alpha), camber angle (gamma)
//
// Tire forces are expressed in the global frame, as applied to the center of
// the associated wheel.
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChIrrGuiDriverSTR.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChDataDriverSTR.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================
// USER SETTINGS
// =============================================================================

// Simulation step size
double step_size = 1e-3;

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
////std::string tire_file("hmmwv/tire/HMMWV_RigidTire.json");
////std::string tire_file("hmmwv/tire/HMMWV_RigidMeshTire_Coarse.json");
////std::string tire_file("hmmwv/tire/HMMWV_Fiala_converted.json");
std::string tire_file("hmmwv/tire/HMMWV_TMeasyTire.json");
////std::string tire_file("hmmwv/tire/HMMWV_PacejkaTire.json");

// Output collection
bool collect_output = true;
std::string out_dir = GetChronoOutputPath() + "SUSPENSION_TEST_RIG";
double out_step_size = 1.0 / 100;

// =============================================================================
int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create tires.
    auto tire_L = ReadTireJSON(vehicle::GetDataFile(tire_file));
    auto tire_R = ReadTireJSON(vehicle::GetDataFile(tire_file));

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

    rig->SetInitialRideHeight(0.4);

    rig->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetWheelVisualizationType(VisualizationType::NONE);
    if (rig->HasSteering()) {
        rig->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    }
    rig->SetTireVisualizationType(VisualizationType::MESH);

    // Create the vehicle Irrlicht application.
    ChVehicleIrrApp app(rig.get(), NULL, L"Suspension Test Rig");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(0.5 * (rig->GetWheelPos(LEFT) + rig->GetWheelPos(RIGHT)), 2.0, 0.5);
    app.SetTimestep(step_size);

    // Create and attach the driver system.
    std::unique_ptr<ChDriverSTR> driver;
    if (use_data_driver) {
        // Driver with inputs from file
        auto data_driver = new ChDataDriverSTR(vehicle::GetDataFile(driver_file));
        driver = std::unique_ptr<ChDriverSTR>(data_driver);
    } else {
        // Interactive driver
        auto irr_driver = new ChIrrGuiDriverSTR(app);
        irr_driver->SetSteeringDelta(1.0 / 50);
        irr_driver->SetDisplacementDelta(1.0 / 250);
        driver = std::unique_ptr<ChDriverSTR>(irr_driver);
    }
    rig->SetDriver(std::move(driver));

    // Initialize suspension test rig.
    rig->Initialize();

    app.AssetBindAll();
    app.AssetUpdateAll();

    // Initialize output
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    std::string out_file = out_dir + "/" + tire_L->GetTemplateName() + ".dat";
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
        double time = rig->GetChTime();
        
        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();

        // Current tire forces
        auto tire_force_L = rig->GetTireForce(VehicleSide::LEFT);
        auto tire_force_R = rig->GetTireForce(VehicleSide::RIGHT);

        // Tire kinematics
        auto omega_L = rig->GetWheelOmega(VehicleSide::LEFT);
        auto omega_R = rig->GetWheelOmega(VehicleSide::RIGHT);

        double kappa_L = tire_L->GetLongitudinalSlip();
        double alpha_L = tire_L->GetSlipAngle();
        double gamma_L = tire_L->GetCamberAngle();

        double kappa_R = tire_R->GetLongitudinalSlip();
        double alpha_R = tire_R->GetSlipAngle();
        double gamma_R = tire_R->GetCamberAngle();

        // Write output data
        if (collect_output && step_number % out_steps == 0) {
            out_csv << time;
            out_csv << rig->GetDisplacementLeftInput() << rig->GetDisplacementRightInput() << rig->GetSteeringInput();
            out_csv << rig->GetActuatorDisp(VehicleSide::LEFT) << rig->GetActuatorDisp(VehicleSide::RIGHT);
            out_csv << rig->GetRideHeight();
            out_csv << tire_force_L.point << tire_force_L.force << tire_force_L.moment;
            out_csv << tire_force_R.point << tire_force_R.force << tire_force_R.moment;
            out_csv << omega_L << kappa_L << alpha_L << gamma_L;
            out_csv << omega_R << kappa_R << alpha_R << gamma_R;
            out_csv << std::endl;
        }

        // Advance simulation of the rig
        rig->Advance(step_size);

        // Update visualization app
        app.Synchronize(tire_L->GetTemplateName(), rig->GetSteeringInput(), 0, 0);
        app.Advance(step_size);

        // Increment frame number
        step_number++;
    }

    // Write output file
    if (collect_output) {
        out_csv.write_to_file(out_file);
    }

    return 0;
}
