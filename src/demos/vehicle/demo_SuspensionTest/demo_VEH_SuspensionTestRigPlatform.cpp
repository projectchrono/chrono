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
// Demonstration program for a suspension test rig with platforms actuating the
// tires.
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
// See the description of ChSuspensionTestRigPlatform::PlotOutput for details
// on data collected (if output is enabled).
//
// =============================================================================

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

// JSON file for vehicle and axle index (axle_index=0: front axle, axle_index=1: rear axle)
std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle.json");
int axle_index = 0;
double post_limit = 0.15;

// JSON file for suspension test rig
std::string str_file("hmmwv/suspensionTest/HMMWV_ST_front.json");
////std::string str_file("hmmwv/suspensionTest/HMMWV_ST_rear.json");

// Specification of test rig inputs:
//   'true':  use driver inputs from file
//   'false': use interactive Irrlicht driver
bool use_data_driver = true;
std::string driver_file("hmmwv/suspensionTest/ST_inputs.dat");

// JSON files for tire models
////std::string tire_file("hmmwv/tire/HMMWV_RigidTire.json");
////std::string tire_file("hmmwv/tire/HMMWV_RigidMeshTire_Coarse.json");
////std::string tire_file("hmmwv/tire/HMMWV_Fiala_converted.json");
std::string tire_file("hmmwv/tire/HMMWV_TMeasyTire.json");
////std::string tire_file("hmmwv/tire/HMMWV_Pac02Tire.json");
////std::string tire_file("hmmwv/tire/HMMWV_Pac89Tire.json");

// Output collection
bool output = true;
std::string out_dir = GetChronoOutputPath() + "SUSPENSION_TEST_RIG_PLATFORM";
double out_step_size = 1e-2;

// =============================================================================
int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create tires.
    auto tire_L = ReadTireJSON(vehicle::GetDataFile(tire_file));
    auto tire_R = ReadTireJSON(vehicle::GetDataFile(tire_file));

    // Create the suspension test rig.
    std::unique_ptr<ChSuspensionTestRigPlatform> rig;
    if (use_vehicle_file) {
        // From a vehicle JSON specification file (selecting a particular axle)
        rig = std::unique_ptr<ChSuspensionTestRigPlatform>(new ChSuspensionTestRigPlatform(
            vehicle::GetDataFile(vehicle_file), axle_index, post_limit, tire_L, tire_R));
    } else {
        // From a suspension test rig JSON specification file
        rig = std::unique_ptr<ChSuspensionTestRigPlatform>(
            new ChSuspensionTestRigPlatform(vehicle::GetDataFile(str_file), tire_L, tire_R));
    }

    rig->SetInitialRideHeight(0.5);

    rig->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetWheelVisualizationType(VisualizationType::NONE);
    if (rig->HasSteering()) {
        rig->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    }
    rig->SetTireVisualizationType(VisualizationType::MESH);

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

    // Set up rig output
    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
        rig->SetSuspensionOutput(true);
        rig->SetSteeringOutput(true);
        rig->SetOutput(ChVehicleOutput::ASCII, out_dir, "output", out_step_size);
        rig->SetPlotOutput(out_step_size);
    }

    // ---------------
    // Simulation loop
    // ---------------
    std::cout << "Rig mass: " << rig->GetMass() << std::endl;

    while (app.GetDevice()->run()) {
        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();

        // Advance simulation of the rig
        rig->Advance(step_size);

        // Update visualization app
        app.Synchronize(tire_L->GetTemplateName(), {rig->GetSteeringInput(), 0, 0});
        app.Advance(step_size);

        if (driver->Ended())
            break;
    }

    // Write output file and plot (no-op if SetPlotOutput was not called)
    rig->PlotOutput(out_dir, "output_plot");

    return 0;
}
