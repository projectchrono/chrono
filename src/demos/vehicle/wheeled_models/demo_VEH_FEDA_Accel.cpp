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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// FED alpha acceleration test.
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

#include "chrono_models/vehicle/feda/FEDA.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_POSTPROCESS
#include "chrono_postprocess/ChGnuPlot.h"
#endif

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::feda;

// =============================================================================

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;

// Type of powertrain model (SHAFTS, SIMPLE_MAP)
PowertrainModelType powertrain_model = PowertrainModelType::SIMPLE_MAP;

// Type of tire model (PAC02, RIGID)
TireModelType tire_model = TireModelType::PAC02;

// Type of brake model (SIMPLE, SHAFTS)
BrakeType brake_type = BrakeType::SIMPLE;

// Terrain length (X direction)
double terrainLength = 800.0;

// Simulation step sizes
double step_size = 5e-4;
double tire_step_size = 5e-4;

// output directory
const std::string out_dir = GetChronoOutputPath() + "FEDA_ACCELERATION";
bool data_output = false;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create systems
    // --------------

    // Create the FED alpha vehicle, set parameters, and initialize.
    // Typical aerodynamic drag for FED alpha: Cd = 0.6 and area 3.8 m2
    FEDA my_feda;
    my_feda.SetContactMethod(ChContactMethod::SMC);
    my_feda.SetChassisFixed(false);
    my_feda.SetInitPosition(ChCoordsys<>(ChVector<>(-terrainLength / 2 + 5, 0, 0.5), ChQuaternion<>(1, 0, 0, 0)));
    my_feda.SetPowertrainType(powertrain_model);
    my_feda.SetTireType(tire_model);
    my_feda.SetBrakeType(brake_type);
    my_feda.SetTireStepSize(tire_step_size);
    my_feda.SetAerodynamicDrag(0.6, 3.8, 1.2041);
    my_feda.Initialize();

    // Set subsystem visualization mode
    VisualizationType tire_vis_type =
        (tire_model == TireModelType::RIGID_MESH) ? VisualizationType::MESH : VisualizationType::MESH;
    my_feda.SetChassisVisualizationType(chassis_vis_type);
    my_feda.SetSuspensionVisualizationType(suspension_vis_type);
    my_feda.SetSteeringVisualizationType(steering_vis_type);
    my_feda.SetWheelVisualizationType(wheel_vis_type);
    my_feda.SetTireVisualizationType(tire_vis_type);

    // Create the terrain
    auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);
    patch_mat->SetYoungModulus(2e7f);
    patch_mat->SetPoissonRatio(0.3f);
    RigidTerrain terrain(my_feda.GetSystem());
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, 5);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 5);
    terrain.Initialize();

    // Create the straight path and the driver system
    auto path = StraightLinePath(ChVector<>(-terrainLength / 2, 0, 0.5), ChVector<>(terrainLength / 2, 0, 0.5), 1);
    ChPathFollowerDriver driver(my_feda.GetVehicle(), path, "my_path", 1000.0);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // Create the vehicle Irrlicht interface
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("FEDA acceleration test");
    vis->SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 8.0, 0.5);
    vis->Initialize();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AddLight(ChVector<>(0, -30, 100), 250,    ChColor(0.7f, 0.7f, 0.7f));
    vis->AddLight(ChVector<>(0, 50, 100), 130,     ChColor(0.7f, 0.7f, 0.7f));
    vis->AddLight(ChVector<>(-300, -30, 100), 250, ChColor(0.7f, 0.7f, 0.7f));
    vis->AddLight(ChVector<>(-300, 50, 100), 130,  ChColor(0.7f, 0.7f, 0.7f));
    vis->AddLight(ChVector<>(+300, -30, 100), 250, ChColor(0.7f, 0.7f, 0.7f));
    vis->AddLight(ChVector<>(+300, 50, 100), 130,  ChColor(0.7f, 0.7f, 0.7f));
    my_feda.GetVehicle().SetVisualSystem(vis);

    // Prepare output
    if (data_output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

    csv << "time";
    csv << "throttle";
    csv << "VehicleSpeed";
    csv << "CurrentTransmissionGear";
    csv << "Distance";
    csv << std::endl;

    // ---------------
    // Simulation loop
    // ---------------

    // Running average of vehicle speed
    utils::ChRunningAverage speed_filter(500);
    double last_speed = -1;

    // Record vehicle speed
    ChFunction_Recorder speed_recorder, dist_recorder;

    // Initialize simulation frame counter and simulation time
    int step_number = 0;
    double time = 0;
    bool done = false;

    ChTimer<> timer;
    timer.start();
    while (vis->Run()) {
        time = my_feda.GetSystem()->GetChTime();

        double speed = speed_filter.Add(my_feda.GetVehicle().GetSpeed());
        double dist = terrainLength / 2.0 + my_feda.GetVehicle().GetPos().x();
        int gear_pos = my_feda.GetPowertrain()->GetCurrentTransmissionGear();
        if (!done) {
            speed_recorder.AddPoint(time, speed);
            dist_recorder.AddPoint(time, dist);
            if (time > 6 && std::abs((speed - last_speed) / step_size) < 2e-4) {
                done = true;
                timer.stop();
                std::cout << "Simulation time: " << timer() << std::endl;
                std::cout << "Maximum speed: " << speed << std::endl;
#ifdef CHRONO_POSTPROCESS
                postprocess::ChGnuPlot gplot_speed;
                gplot_speed.SetGrid();
                gplot_speed.SetLabelX("time (s)");
                gplot_speed.SetLabelY("speed (m/s)");
                gplot_speed.Plot(speed_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");
                postprocess::ChGnuPlot gplot_dist;
                gplot_dist.SetGrid();
                gplot_dist.SetLabelX("time (s)");
                gplot_dist.SetLabelY("dist (m)");
                gplot_dist.Plot(dist_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");
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
        DriverInputs driver_inputs = driver.GetInputs();
        csv << time;
        csv << driver_inputs.m_throttle;
        csv << 3.6 * speed;
        csv << gear_pos;
        csv << dist;
        csv << std::endl;

        if (done) {
            driver_inputs.m_throttle = 0.1;
            driver_inputs.m_braking = 0.8;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_feda.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize("Acceleration test", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        my_feda.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;

        vis->EndScene();
    }

    if (data_output) {
        csv.write_to_file(out_dir + "/feda_accel_chrono.dat");
    }

    return 0;
}
