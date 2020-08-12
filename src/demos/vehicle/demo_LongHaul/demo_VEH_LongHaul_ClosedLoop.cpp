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
// Tractor-trailer acceleration test.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/core/ChTimer.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline4WD.h"

#include "subsystems/SemiTractor_tire.h"
#include "subsystems/SemiTractor_powertrain.h"
#include "subsystems/SemiTractor_vehicle.h"
#include "subsystems/SemiTrailer.h"
#include "subsystems/SemiTrailer_tire.h"

#ifdef CHRONO_POSTPROCESS
#include "chrono_postprocess/ChGnuPlot.h"
#endif

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;

// =============================================================================

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Terrain length (X direction)
double terrainLength = 300.0;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = 1e-3;

// Initial vehicle position
ChVector<> initLoc(0, 0, 0.6);

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create systems
    // --------------

    SemiTractor_vehicle vehicle(false, ChContactMethod::NSC);
    vehicle.Initialize(ChCoordsys<>(initLoc, QUNIT));
    vehicle.SetChassisVisualizationType(chassis_vis_type);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::MESH);
    auto drvLine = std::static_pointer_cast<ChShaftsDriveline4WD>(vehicle.GetDriveline());
    drvLine->LockCentralDifferential(0, false);

    SemiTrailer trailer(vehicle.GetSystem(), false);
    trailer.Initialize(vehicle.GetChassis(), ChVector<>(-4.64, 0, 0.0));
    trailer.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    trailer.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    trailer.SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    auto powertrain = chrono_types::make_shared<SemiTractor_powertrain>("Powertrain");
    vehicle.InitializePowertrain(powertrain);

    // Create the tractor tires
    auto tire_FL = chrono_types::make_shared<SemiTractor_tire>("TractorTire_FL");
    auto tire_FR = chrono_types::make_shared<SemiTractor_tire>("TractorTire_FR");

    auto tire_RL1i = chrono_types::make_shared<SemiTractor_tire>("TractorTire_RL1i");
    auto tire_RR1i = chrono_types::make_shared<SemiTractor_tire>("TractorTire_RR1i");
    auto tire_RL1o = chrono_types::make_shared<SemiTractor_tire>("TractorTire_RL1o");
    auto tire_RR1o = chrono_types::make_shared<SemiTractor_tire>("TractorTire_RR1o");

    auto tire_RL2i = chrono_types::make_shared<SemiTractor_tire>("TractorTire_RL2i");
    auto tire_RR2i = chrono_types::make_shared<SemiTractor_tire>("TractorTire_RR2i");
    auto tire_RL2o = chrono_types::make_shared<SemiTractor_tire>("TractorTire_RL2o");
    auto tire_RR2o = chrono_types::make_shared<SemiTractor_tire>("TractorTire_RR2o");

    vehicle.InitializeTire(tire_FL, vehicle.GetAxle(0)->m_wheels[0], tire_vis_type);
    vehicle.InitializeTire(tire_FR, vehicle.GetAxle(0)->m_wheels[1], tire_vis_type);

    vehicle.InitializeTire(tire_RL1i, vehicle.GetAxle(1)->m_wheels[0], tire_vis_type);
    vehicle.InitializeTire(tire_RR1i, vehicle.GetAxle(1)->m_wheels[1], tire_vis_type);
    vehicle.InitializeTire(tire_RL1o, vehicle.GetAxle(1)->m_wheels[2], tire_vis_type);
    vehicle.InitializeTire(tire_RR1o, vehicle.GetAxle(1)->m_wheels[3], tire_vis_type);

    vehicle.InitializeTire(tire_RL2i, vehicle.GetAxle(2)->m_wheels[0], tire_vis_type);
    vehicle.InitializeTire(tire_RR2i, vehicle.GetAxle(2)->m_wheels[1], tire_vis_type);
    vehicle.InitializeTire(tire_RL2o, vehicle.GetAxle(2)->m_wheels[2], tire_vis_type);
    vehicle.InitializeTire(tire_RR2o, vehicle.GetAxle(2)->m_wheels[3], tire_vis_type);

    auto tr_tire_FL = chrono_types::make_shared<SemiTrailer_tire>("FL");
    auto tr_tire_FR = chrono_types::make_shared<SemiTrailer_tire>("FR");
    auto tr_tire_ML = chrono_types::make_shared<SemiTrailer_tire>("ML");
    auto tr_tire_MR = chrono_types::make_shared<SemiTrailer_tire>("MR");
    auto tr_tire_RL = chrono_types::make_shared<SemiTrailer_tire>("RL");
    auto tr_tire_RR = chrono_types::make_shared<SemiTrailer_tire>("RR");

    // Create the tractor tires
    trailer.InitializeTire(tr_tire_FL, trailer.GetAxle(0)->m_wheels[0], tire_vis_type);
    trailer.InitializeTire(tr_tire_FR, trailer.GetAxle(0)->m_wheels[1], tire_vis_type);
    trailer.InitializeTire(tr_tire_ML, trailer.GetAxle(1)->m_wheels[0], tire_vis_type);
    trailer.InitializeTire(tr_tire_MR, trailer.GetAxle(1)->m_wheels[1], tire_vis_type);
    trailer.InitializeTire(tr_tire_RL, trailer.GetAxle(2)->m_wheels[0], tire_vis_type);
    trailer.InitializeTire(tr_tire_RR, trailer.GetAxle(2)->m_wheels[1], tire_vis_type);

    // Create the terrain
    RigidTerrain terrain(vehicle.GetSystem());
    auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);
    patch_mat->SetYoungModulus(2e7f);
    patch_mat->SetPoissonRatio(0.3f);
    for (size_t i = 0; i < 3; i++) {
        auto patch =
            terrain.AddPatch(patch_mat, ChVector<>(terrainLength * i, 0, 0), ChVector<>(0, 0, 1), terrainLength, 5);
        patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
        patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 5);
    }
    terrain.Initialize();

    // Create the vehicle Irrlicht interface
    ChWheeledVehicleIrrApp app(&vehicle, L"Open Loop Road Train Follows Straight Line");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);
    app.SetTimestep(step_size);

    // ----------------------------------------------
    // Create the straight path and the driver system
    // ----------------------------------------------

    auto path = StraightLinePath(ChVector<>(-terrainLength / 2, 0, 0.5), ChVector<>(10 * terrainLength / 2, 0, 0.5), 1);
    ChPathFollowerDriver driver(vehicle, path, "my_path", 1000.0);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // ---------------------------------------------
    // Finalize construction of visualization assets
    // ---------------------------------------------

    app.AssetBindAll();
    app.AssetUpdateAll();

    // ---------------
    // Simulation loop
    // ---------------

    // Running average of vehicle speed
    utils::ChRunningAverage speed_filter(500);
    double last_speed = -1;

    // Record vehicle speed
    ChFunction_Recorder speed_recorder;

    // Initialize simulation frame counter and simulation time
    int step_number = 0;
    double time = 0;
    bool done = false;

    ChTimer<> timer;
    timer.start();
    while (app.GetDevice()->run()) {
        time = vehicle.GetSystem()->GetChTime();

        double speed = speed_filter.Add(vehicle.GetVehicleSpeed());
        if (!done) {
            speed_recorder.AddPoint(time, speed);
            if (time > 6 && std::abs((speed - last_speed) / step_size) < 2e-4) {
                done = true;
                timer.stop();
                std::cout << "Simulation time: " << timer() << std::endl;
                std::cout << "Maximum speed: " << speed << std::endl;
#ifdef CHRONO_POSTPROCESS
                postprocess::ChGnuPlot gplot;
                gplot.SetGrid();
                gplot.SetLabelX("time (s)");
                gplot.SetLabelY("speed (m/s)");
                gplot.Plot(speed_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");
#endif
            }
        }
        last_speed = speed;

        // End simulation
        if (time >= 100)
            break;

        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();

        // Driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        if (done) {
            driver_inputs.m_throttle = 0.1;
            driver_inputs.m_braking = 0.8;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        trailer.Synchronize(time, driver_inputs.m_braking, terrain);
        vehicle.Synchronize(time, driver_inputs, terrain);
        app.Synchronize("Acceleration test", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        trailer.Advance(step_size);
        vehicle.Advance(step_size);
        app.Advance(step_size);

        // Increment frame number
        step_number++;

        app.EndScene();
    }

    return 0;
}
