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
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#ifdef CHRONO_POSTPROCESS
#include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

// Type of powertrain model (SHAFTS, SIMPLE, SIMPLE_CVT)
PowertrainModelType powertrain_model = PowertrainModelType::SHAFTS;

// Drive type (FWD, RWD, or AWD)
DrivelineTypeWV drive_type = DrivelineTypeWV::AWD;

// Type of tire model (RIGID, RIGID_MESH, PACEJKA, LUGRE, FIALA, PAC89, PAC02, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

enum class TerrainType {FLAT, RIGID};
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
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(ChContactMethod::SMC);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(init_loc, yaw_rot));
    my_hmmwv.SetPowertrainType(powertrain_model);
    my_hmmwv.SetDriveType(drive_type);
    my_hmmwv.SetTireType(tire_model);
    my_hmmwv.SetTireStepSize(tire_step_size);
    if (include_aero_drag)
        my_hmmwv.SetAerodynamicDrag(0.5, 5.0, 1.2);
    my_hmmwv.Initialize();

    // Set subsystem visualization mode
    my_hmmwv.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetWheelVisualizationType(VisualizationType::NONE);
    my_hmmwv.SetTireVisualizationType(tire_model == TireModelType::RIGID_MESH ? VisualizationType::MESH
                                                                              : VisualizationType::PRIMITIVES);

    // Create the terrain
    std::shared_ptr<ChTerrain> terrain;
    switch (terrain_type) {
        case TerrainType::RIGID:
        default: {
            auto rigid_terrain = chrono_types::make_shared<RigidTerrain>(my_hmmwv.GetSystem());
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
    ChPathFollowerDriver driver(my_hmmwv.GetVehicle(), path, "my_path", 1000.0);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // Create the vehicle Irrlicht interface
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("HMMWV acceleration test");
    vis->SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    my_hmmwv.GetVehicle().SetVisualSystem(vis);

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

    ChTimer<> timer;
    timer.start();
    while (vis->Run()) {
        time = my_hmmwv.GetSystem()->GetChTime();

        double speed = speed_filter.Add(my_hmmwv.GetVehicle().GetSpeed());
        if (!done) {
            speed_recorder.AddPoint(time, speed);

            if (output) {
                auto wheel_state = my_hmmwv.GetVehicle().GetWheel(0, LEFT)->GetState();
                auto frc = my_hmmwv.GetVehicle().GetTire(0, LEFT)->ReportTireForce(terrain.get());
                   
                csv << time << wheel_state.omega;
                csv << my_hmmwv.GetChassisBody()->TransformDirectionParentToLocal(frc.force);
                csv << my_hmmwv.GetChassisBody()->TransformDirectionParentToLocal(frc.moment);
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
        vis->DrawAll();

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        if (done) {
            driver_inputs.m_throttle = 0.1;
            driver_inputs.m_braking = 0.8;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain->Synchronize(time);
        my_hmmwv.Synchronize(time, driver_inputs, *terrain);
        vis->Synchronize("Acceleration test", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain->Advance(step_size);
        my_hmmwv.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;

        // Draw a coordinate system aligned with the world frame
        tools::drawCoordsys(vis.get(), ChCoordsys<>(my_hmmwv.GetVehicle().GetPos(), QUNIT), 2);

        vis->EndScene();
    }

    if (output)
        csv.write_to_file(out_dir + "/tire_force.out");

    return 0;
}
