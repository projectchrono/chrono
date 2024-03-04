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
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Type of engine model (SHAFTS, SIMPLE, SIMPLE_MAP)
EngineModelType engine_model = EngineModelType::SHAFTS;

// Type of transmission model (SHAFTS, SIMPLE_MAP)
TransmissionModelType transmission_model = TransmissionModelType::AUTOMATIC_SHAFTS;

// Drive type (FWD, RWD, or AWD)
DrivelineTypeWV drive_type = DrivelineTypeWV::AWD;

// Type of tire model (RIGID, RIGID_MESH, FIALA, PAC89, PAC02, TMEASY, TMSIMPLE)
TireModelType tire_model = TireModelType::TMEASY;

enum class TerrainType { FLAT, RIGID };
TerrainType terrain_type = TerrainType::RIGID;

// Terrain length (X direction)
double terrainLength = 400.0;

// Lane direction
double yaw_angle = 0 * CH_C_DEG_TO_RAD;

// Include aerodynamic drag
bool include_aero_drag = false;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = 1e-3;

// Output
bool output = true;
std::string out_dir = GetChronoOutputPath() + "HMMWV_ACCELERATION";

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    if (terrain_type == TerrainType::FLAT &&
        (tire_model == TireModelType::RIGID || tire_model == TireModelType::RIGID_MESH)) {
        std::cout << "Flat terrain incompatible with a rigid tire model!" << std::endl;
        return 1;
    }

    // Create output directory
    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    // --------------
    // Create systems
    // --------------

    ChQuaternion<> yaw_rot = Q_from_AngZ(yaw_angle);
    ChCoordsys<> patch_sys(VNULL, yaw_rot);
    ChVector<> init_loc = patch_sys.TransformPointLocalToParent(ChVector<>(-terrainLength / 2 + 5, 0, 0.7));
    ChVector<> path_start = patch_sys.TransformPointLocalToParent(ChVector<>(-terrainLength / 2, 0, 0.5));
    ChVector<> path_end = patch_sys.TransformPointLocalToParent(ChVector<>(+terrainLength / 2, 0, 0.5));

    // Create the HMMWV vehicle, set parameters, and initialize.
    // Typical aerodynamic drag for HMMWV: Cd = 0.5 and area ~5 m2
    HMMWV_Full hmmwv;
    hmmwv.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    hmmwv.SetContactMethod(ChContactMethod::SMC);
    hmmwv.SetChassisFixed(false);
    hmmwv.SetInitPosition(ChCoordsys<>(init_loc, yaw_rot));
    hmmwv.SetEngineType(engine_model);
    hmmwv.SetTransmissionType(transmission_model);
    hmmwv.SetDriveType(drive_type);
    hmmwv.SetTireType(tire_model);
    hmmwv.SetTireStepSize(tire_step_size);
    if (include_aero_drag)
        hmmwv.SetAerodynamicDrag(0.5, 5.0, 1.2);
    hmmwv.Initialize();

    // Set subsystem visualization mode
    hmmwv.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetWheelVisualizationType(VisualizationType::NONE);
    hmmwv.SetTireVisualizationType(tire_model == TireModelType::RIGID_MESH ? VisualizationType::MESH
                                                                           : VisualizationType::PRIMITIVES);

    // Create the terrain
    std::shared_ptr<ChTerrain> terrain;
    switch (terrain_type) {
        case TerrainType::RIGID:
        default: {
            auto rigid_terrain = chrono_types::make_shared<RigidTerrain>(hmmwv.GetSystem());
            auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            patch_mat->SetFriction(0.9f);
            patch_mat->SetRestitution(0.01f);
            patch_mat->SetYoungModulus(2e7f);
            patch_mat->SetPoissonRatio(0.3f);
            auto patch = rigid_terrain->AddPatch(patch_mat, ChCoordsys<>(ChVector<>(0), yaw_rot), terrainLength, 5);
            patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 5);
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

    // Create the straight path and the driver system
    auto path = StraightLinePath(path_start, path_end, 1);
    ChPathFollowerDriver driver(hmmwv.GetVehicle(), path, "my_path", 1000.0);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // Create the vehicle run-time visualization interface

#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVehicleVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle("HMMWV acceleration test");
            vis_irr->SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AttachVehicle(&hmmwv.GetVehicle());

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle("HMMWV acceleration test");
            vis_vsg->SetWindowSize(ChVector2<int>(800, 600));
            vis_vsg->SetWindowPosition(ChVector2<int>(100, 300));
            vis_vsg->SetUseSkyBox(true);
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_C_PI_2, CH_C_PI_4);
            vis_vsg->SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 10.0, 0.5);
            vis_vsg->AttachVehicle(&hmmwv.GetVehicle());
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Output file
    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

    // Running average of vehicle speed
    utils::ChRunningAverage speed_filter(500);
    double last_speed = -1;

    // Record vehicle speed
    ChFunction_Recorder speed_recorder;

    // Initialize simulation frame counter and simulation time
    int step_number = 0;
    double time = 0;
    bool done = false;

    ChTimer timer;
    timer.start();
    while (vis->Run()) {
        time = hmmwv.GetSystem()->GetChTime();

        double speed = speed_filter.Add(hmmwv.GetVehicle().GetSpeed());
        if (!done) {
            speed_recorder.AddPoint(time, speed);

            if (output) {
                auto wheel_state = hmmwv.GetVehicle().GetWheel(0, LEFT)->GetState();
                auto frc = hmmwv.GetVehicle().GetTire(0, LEFT)->ReportTireForce(terrain.get());

                csv << time << wheel_state.omega;
                csv << hmmwv.GetChassisBody()->TransformDirectionParentToLocal(frc.force);
                csv << hmmwv.GetChassisBody()->TransformDirectionParentToLocal(frc.moment);
                csv << std::endl;
            }

            if (time > 6 && std::abs((speed - last_speed) / step_size) < 2e-4) {
                done = true;
                timer.stop();
                std::cout << "Maximum speed (m/s): " << speed << std::endl;
                std::cout << "Time (s):            " << time << std::endl;
                std::cout << "Simulation time (s): " << timer() << std::endl;
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
        vis->Render();

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        if (done) {
            driver_inputs.m_throttle = 0.1;
            driver_inputs.m_braking = 0.8;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain->Synchronize(time);
        hmmwv.Synchronize(time, driver_inputs, *terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain->Advance(step_size);
        hmmwv.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;

        // Draw a coordinate system aligned with the world frame
        ////tools::drawCoordsys(vis.get(), ChCoordsys<>(hmmwv.GetVehicle().GetPos(), QUNIT), 2);

        vis->EndScene();
    }

    if (output)
        csv.write_to_file(out_dir + "/tire_force.out");

    return 0;
}
