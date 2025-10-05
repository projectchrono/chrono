// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
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
// HMMWV acceleration test.
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
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono/assets/ChVisualSystem.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
    #include "chrono_postprocess/ChBlender.h"
using namespace chrono::postprocess;
#endif

#include "../WheeledVehicleModels.h"

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

enum class TerrainType { FLAT, RIGID };
TerrainType terrain_type = TerrainType::RIGID;

// Terrain length (X direction)
double terrainLength = 800.0;

// Include aerodynamic drag
bool include_aero_drag = false;

// Simulation step sizes
double step_size = 1e-3;

// End simulation time
double t_end = 100;

// Output
bool output = false;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // --------------
    // Create vehicle
    // --------------

    ChVector3d init_loc(-terrainLength / 2 + 5, 0, 0.7);
    ChVector3d path_start(-terrainLength / 2, 0, 0.5);
    ChVector3d path_end(+terrainLength / 2, 0, 0.5);

    // Select vehicle model (see VehicleModel.h)
    auto models = WheeledVehicleModel::List();

    int num_models = (int)models.size();
    int which = 0;
    std::cout << "Options:\n";
    for (int i = 0; i < num_models; i++)
        std::cout << std::setw(2) << i + 1 << "  " << models[i].second << std::endl;
    std::cout << "\nSelect vehicle: ";
    std::cin >> which;
    std::cout << std::endl;
    ChClampValue(which, 1, num_models);

    auto vehicle_model = models[which - 1].first;

    // Create the vehicle model
    vehicle_model->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    vehicle_model->Create(ChContactMethod::SMC, ChCoordsys<>(init_loc, QUNIT), false);
    auto& vehicle = vehicle_model->GetVehicle();

    // -----------------------
    // Create output directory
    // -----------------------

    std::string out_dir = GetChronoOutputPath() + "ACCELERATION_TEST";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    out_dir = out_dir + "/" + vehicle_model->ModelName();
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // ------------------
    // Create the terrain
    // ------------------

    std::shared_ptr<ChTerrain> terrain;
    switch (terrain_type) {
        case TerrainType::RIGID:
        default: {
            auto rigid_terrain = chrono_types::make_shared<RigidTerrain>(vehicle.GetSystem());
            auto patch_mat = chrono_types::make_shared<ChContactMaterialSMC>();
            patch_mat->SetFriction(0.9f);
            patch_mat->SetRestitution(0.01f);
            patch_mat->SetYoungModulus(2e7f);
            patch_mat->SetPoissonRatio(0.3f);
            auto patch = rigid_terrain->AddPatch(patch_mat, ChCoordsys<>(), terrainLength, 5);
            patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
            patch->SetTexture(GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 5);
            rigid_terrain->Initialize();
            terrain = rigid_terrain;
            break;
        }
        case TerrainType::FLAT: {
            auto flat_terrain = chrono_types::make_shared<FlatTerrain>(0, 0.9f);
            terrain = flat_terrain;
            break;
        }
    }

    // -----------------------------
    // Create path and driver system
    // -----------------------------

    auto path = StraightLinePath(path_start, path_end, 1);
    ChPathFollowerDriver driver(vehicle, path, "my_path", 1000.0);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // -----------------------------------------
    // Create the vehicle run-time visualization
    // -----------------------------------------

#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::string title = "Vehicle Acceleration Test";
    std::shared_ptr<ChVehicleVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            // Create the vehicle Irrlicht interface
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle(title);
            vis_irr->SetChaseCamera(vehicle_model->TrackPoint(), vehicle_model->CameraDistance(),
                                    vehicle_model->CameraHeight());
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AttachVehicle(&vehicle);

            vis = vis_irr;
#endif
            break;
        }
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            // Create the vehicle VSG interface
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle(title);
            vis_vsg->AttachVehicle(&vehicle);
            vis_vsg->SetChaseCamera(vehicle_model->TrackPoint(), vehicle_model->CameraDistance(),
                                    vehicle_model->CameraHeight());
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowPosition(100, 100);
            vis_vsg->EnableSkyBox();
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
        default:
            break;
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Output file
    utils::ChWriterCSV csv("\t");
    csv.Stream().setf(std::ios::scientific | std::ios::showpos);
    csv.Stream().precision(6);

    csv << "time";
    csv << "throttle";
    csv << "VehicleSpeed";
    csv << "CurrentTransmissionGear";
    csv << "Distance";
    csv << std::endl;

    // Running average of vehicle speed
    utils::ChRunningAverage speed_filter(500);
    double last_speed = -1;

    // Record vehicle speed
    ChFunctionInterp speed_recorder;
    ChFunctionInterp dist_recorder;

    // Initialize simulation frame counter and simulation time
    double time = 0;
    bool done = false;

    ChTimer timer;
    timer.start();
    while (true) {
        time = vehicle.GetSystem()->GetChTime();

        double speed = speed_filter.Add(vehicle.GetSpeed());
        double dist = terrainLength / 2.0 + vehicle.GetPos().x();
        int gear_pos = vehicle.GetTransmission()->GetCurrentGear();

        if (!done) {
            speed_recorder.AddPoint(time, speed);
            dist_recorder.AddPoint(time, dist);

            if (time > 6 && std::abs((speed - last_speed) / step_size) < 2e-4) {
                done = true;
                timer.stop();
                std::cout << "Simulation time: " << timer() << std::endl;
                std::cout << "Maximum speed: " << speed << std::endl;
#ifdef CHRONO_POSTPROCESS
                {
                    postprocess::ChGnuPlot gplot_speed(out_dir + "/speed.gpl");
                    gplot_speed.SetGrid();
                    gplot_speed.SetLabelX("time (s)");
                    gplot_speed.SetLabelY("speed (m/s)");
                    gplot_speed.Plot(speed_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");
                }
                {
                    postprocess::ChGnuPlot gplot_dist(out_dir + "/dist.gpl");
                    gplot_dist.SetGrid();
                    gplot_dist.SetLabelX("time (s)");
                    gplot_dist.SetLabelY("dist (m)");
                    gplot_dist.Plot(dist_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");
                }
#endif
            }
        }
        last_speed = speed;

        // End simulation
        if (time >= t_end)
            break;

        // Render scene
        if (vis) {
            if (!vis->Run())
                break;

            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();
        if (done) {
            driver_inputs.m_throttle = 0.1;
            driver_inputs.m_braking = 0.8;
        }

        // Collect output
        if (output) {
            csv << time;
            csv << driver_inputs.m_throttle;
            csv << 3.6 * speed;
            csv << gear_pos;
            csv << dist;
            csv << std::endl;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain->Synchronize(time);
        vehicle_model->Synchronize(time, driver_inputs, *terrain);
        if (vis)
            vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain->Advance(step_size);
        vehicle_model->Advance(step_size);
        if (vis)
            vis->Advance(step_size);
    }

    if (output)
        csv.WriteToFile(out_dir + "/veh_acceleration.out");

    return 0;
}
