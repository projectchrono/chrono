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
// Driver inputs for a suspension test rig include left/right post displacements
// and steering input (the latter being ignored if the tested suspension is not
// attached to a steering mechanism).  These driver inputs can be obtained from
// an interactive driver system (of type ChIrrGuiDriverSTR) or from a data file
// (using a driver system of type ChDataDriverSTR).
//
// See the description of ChSuspensionTestRig::PlotOutput for details on data
// collected (if output is enabled).
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/utils/ChVehicleIrrApp.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChIrrGuiDriverSTR.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChDataDriverSTR.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================
// Specification of a vehicle suspension test rig
// Available models:
//    HMMWV
//    MTV

class STR_Setup {
  public:
    virtual std::string SuspensionRigJSON() const = 0;
    virtual std::string VehicleJSON() const = 0;
    virtual std::string TireJSON() const = 0;
    virtual std::string DataDriverFile() const = 0;
    virtual std::vector<int> TestAxles() const = 0;
    virtual std::vector<int> TestSteerings() const = 0;
    virtual double InitRideHeight() const = 0;
    virtual double PostLimit() const = 0;
    virtual double CameraDistance() const = 0;
};

class HMMWV_STR_Setup : public STR_Setup {
  public:
    virtual std::string SuspensionRigJSON() const override { return "hmmwv/suspensionTest/HMMWV_ST_front.json"; }
    virtual std::string VehicleJSON() const override { return "hmmwv/vehicle/HMMWV_Vehicle.json"; }
    virtual std::string TireJSON() const override { return "hmmwv/tire/HMMWV_TMeasyTire.json"; }
    virtual std::string DataDriverFile() const override { return "hmmwv/suspensionTest/ST_inputs.dat"; }
    virtual std::vector<int> TestAxles() const override { return {0}; }
    virtual std::vector<int> TestSteerings() const { return {0}; }
    virtual double InitRideHeight() const override { return 0.5; }
    virtual double PostLimit() const { return 0.15; }
    virtual double CameraDistance() const override { return 2.0; }
};

class MTV_STR_Setup : public STR_Setup {
  public:
    virtual std::string SuspensionRigJSON() const override { return "mtv/suspensionTest/MTV_ST_rear.json"; }
    virtual std::string VehicleJSON() const override { return "mtv/vehicle/MTV_Vehicle_WalkingBeam.json"; }
    virtual std::string TireJSON() const override { return "mtv/tire/FMTV_TMeasyTire.json"; }
    virtual std::string DataDriverFile() const override { return "mtv/suspensionTest/ST_inputs.dat"; }
    virtual std::vector<int> TestAxles() const override { return {1, 2}; }
    virtual std::vector<int> TestSteerings() const { return {}; }
    virtual double InitRideHeight() const override { return 0.55; }
    virtual double PostLimit() const { return 0.15; }
    virtual double CameraDistance() const override { return 4.0; }
};

// =============================================================================
// USER SETTINGS

HMMWV_STR_Setup setup;
////MTV_STR_Setup setup;

// STR rig type
enum class RigMode {PLATFORM, PUSHROD};
RigMode rig_mode = RigMode::PUSHROD;

// Specification of test rig inputs
enum class DriverMode {DATA_FILE, INTERACTIVE};
DriverMode driver_mode = DriverMode::DATA_FILE;

// Output collection
bool output = false;
bool plot = true;
std::string out_dir = GetChronoOutputPath() + "SUSPENSION_TEST_RIG";
double out_step_size = 1e-2;

// Simulation step size
double step_size = 1e-3;

// =============================================================================

std::shared_ptr<ChSuspensionTestRig> CreateFromVehicleModel() {
    // Create the vehicle
    auto vehicle =
        chrono_types::make_shared<WheeledVehicle>(vehicle::GetDataFile(setup.VehicleJSON()), ChContactMethod::SMC);

    // Create the suspension test rig
    std::shared_ptr<ChSuspensionTestRig> rig;
    switch (rig_mode) {
        default:
        case RigMode::PLATFORM: {
            rig = chrono_types::make_shared<ChSuspensionTestRigPlatform>(vehicle, setup.TestAxles(),
                                                                         setup.TestSteerings(), setup.PostLimit());
            break;
        }
        case RigMode::PUSHROD: {
            rig = chrono_types::make_shared<ChSuspensionTestRigPushrod>(vehicle, setup.TestAxles(),
                                                                        setup.TestSteerings(), setup.PostLimit());
            break;
        }
    }

    rig->SetInitialRideHeight(setup.InitRideHeight());

    return rig;
}

std::shared_ptr<ChSuspensionTestRig> CreateFromSpecFile() {
    switch (rig_mode) {
        default:
        case RigMode::PLATFORM:
            return chrono_types::make_shared<ChSuspensionTestRigPlatform>(
                vehicle::GetDataFile(setup.SuspensionRigJSON()));
        case RigMode::PUSHROD:
            return chrono_types::make_shared<ChSuspensionTestRigPushrod>(
                vehicle::GetDataFile(setup.SuspensionRigJSON()));
    }
}

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Option 1: Create the suspension rig from an existing vehicle model
    auto rig = CreateFromVehicleModel(); 

    // Option 2: Create the suspension rig from a JSON rig specification file
    ////auto rig = CreateFromSpecFile();

    // Create and attach the vehicle tires.
    // (not needed if tires are specified in the vehicle's suspension JSON file)
    for (auto& axle : rig->GetVehicle().GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(vehicle::GetDataFile(setup.TireJSON()));
            rig->GetVehicle().InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

    // Optional rig settings
    rig->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetWheelVisualizationType(VisualizationType::NONE);
    rig->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetTireVisualizationType(VisualizationType::MESH);

    // Create the vehicle Irrlicht application.
    ChVehicleIrrApp app(&rig->GetVehicle(), L"Suspension Test Rig");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(0.5 * (rig->GetSpindlePos(0, LEFT) + rig->GetSpindlePos(0, RIGHT)), setup.CameraDistance(), 0.5);
    app.SetTimestep(step_size);

    // Create and attach the driver system.
    switch (driver_mode) {
        case DriverMode::DATA_FILE: {
            auto driver = chrono_types::make_shared<ChDataDriverSTR>(vehicle::GetDataFile(setup.DataDriverFile()));
            rig->SetDriver(driver);
            break;
        }
        case DriverMode::INTERACTIVE: {
            auto driver = chrono_types::make_shared<ChIrrGuiDriverSTR>(app);
            driver->SetSteeringDelta(1.0 / 50);
            driver->SetDisplacementDelta(1.0 / 250);
            rig->SetDriver(driver);
            break;
        }
    }

    // Initialize suspension test rig.
    rig->Initialize();

    app.AssetBindAll();
    app.AssetUpdateAll();

    // Set up rig output
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (output) {
        rig->SetSuspensionOutput(true);
        rig->SetSteeringOutput(true);
        rig->SetOutput(ChVehicleOutput::ASCII, out_dir, "output", out_step_size);
    }
    if (plot) {
        rig->SetPlotOutput(out_step_size);
    }

    // Simulation loop
    while (app.GetDevice()->run()) {
        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();

        // Advance simulation of the rig
        rig->Advance(step_size);

        // Update visualization app
        app.Synchronize(rig->GetDriverMessage(), {rig->GetSteeringInput(), 0, 0});
        app.Advance(step_size);

        if (rig->DriverEnded())
            break;
    }

    // Write output file and plot (no-op if SetPlotOutput was not called)
    rig->PlotOutput(out_dir, "output_plot");

    return 0;
}
