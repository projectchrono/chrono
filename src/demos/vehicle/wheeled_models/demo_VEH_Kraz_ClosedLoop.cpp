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
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/kraz/Kraz.h"

#ifdef CHRONO_POSTPROCESS
#include "chrono_postprocess/ChGnuPlot.h"
#endif

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::kraz;

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

    // Create the semi-trailer truck
    Kraz truck;
    truck.SetContactMethod(ChContactMethod::NSC);
    truck.SetChassisFixed(false);
    truck.SetInitPosition(ChCoordsys<>(initLoc, QUNIT));
    truck.SetTireStepSize(tire_step_size);
    truck.SetInitFwdVel(0.0);

    truck.Initialize();

    truck.SetChassisVisualizationType(VisualizationType::MESH, VisualizationType::PRIMITIVES);
    truck.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    truck.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES, VisualizationType::PRIMITIVES);
    truck.SetWheelVisualizationType(VisualizationType::MESH, VisualizationType::MESH);
    truck.SetTireVisualizationType(VisualizationType::MESH, VisualizationType::MESH);

    // Create the terrain
    RigidTerrain terrain(truck.GetSystem());
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

    // Create the straight path and the driver system
    auto path = StraightLinePath(ChVector<>(-terrainLength / 2, 0, 0.5), ChVector<>(10 * terrainLength / 2, 0, 0.5), 1);
    ChPathFollowerDriver driver(truck.GetTractor(), path, "my_path", 1000.0);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // Create the vehicle Irrlicht interface
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Semi-trailer truck :: Follows Straight Line");
    vis->SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    truck.GetTractor().SetVisualSystem(vis);

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
    while (vis->Run()) {
        time = truck.GetSystem()->GetChTime();

        double speed = speed_filter.Add(truck.GetTractor().GetSpeed());
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

        vis->BeginScene();
        vis->DrawAll();

        // Driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        if (done) {
            driver_inputs.m_throttle = 0.1;
            driver_inputs.m_braking = 0.8;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        truck.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize("Acceleration test", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        truck.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;

        vis->EndScene();
    }

    return 0;
}
