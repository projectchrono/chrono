// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Simulation of a HMMWV full model using ANCF tires on rigid terrain.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChStream.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#ifdef CHRONO_MKL
#include "chrono_mkl/ChSolverMKL.h"
#endif

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "models/vehicle/hmmwv/HMMWV.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

// Number of OpenMP threads
int num_threads = 4;

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 2.5);
ChQuaternion<> initRot(1, 0, 0, 0);
////ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
////ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
////ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
////ChQuaternion<> initRot(0, 0, 0, 1);

// Type of driver inputs
enum DriverMode {
    DEFAULT,   // interactive driver (keyboard inputs)
    RECORD,    // interactive driver with input recording
    PLAYBACK,  // playback recorded inputs
    USER_FILE  // read inputs from specified data file
};
DriverMode driver_mode = USER_FILE;
std::string driver_file("generic/driver/Sample_Maneuver.txt");

// Visualization type for chassis & wheels (PRIMITIVES, MESH, or NONE)
VisualizationType vis_type = PRIMITIVES;

// Type of powertrain model (SHAFTS, SIMPLE)
PowertrainModelType powertrain_model = SHAFTS;

// Drive type (FWD, RWD, or AWD)
DrivelineType drive_type = AWD;

// Rigid terrain (RigidTerrain::FLAT, RigidTerrain::HEIGHT_MAP, RigidTerrain::MESH)
RigidTerrain::Type terrain_model = RigidTerrain::FLAT;

double terrainHeight = 0;      // terrain height (FLAT terrain only)
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Point on chassis tracked by the camera (for Irrlicht visualization)
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation step sizes
double step_size = 1e-4;
// Simulation end time
double t_end = 1000;
// Verbose solver output
bool verbose = false;

// Time interval between two render frames (1/FPS)
double render_step_size = 1.0 / 50;

// Output directories
const std::string out_dir = "../HMMWV_ANCF";
const std::string pov_dir = out_dir + "/POVRAY";

// Debug logging
bool debug_output = false;
// Debug output frequency (1/FPS)
double debug_step_size = 1.0 / 2;

// POV-Ray output
bool povray_output = false;

// =============================================================================

int main(int argc, char* argv[]) {

    // ----------------------------------
    // Create the (sequential) DEM system
    // ----------------------------------

    ChSystemDEM* system = new ChSystemDEM;
    system->Set_G_acc(ChVector<>(0, 0, -9.81));

    // Set number threads
    system->SetParallelThreadNumber(num_threads);
    CHOMPfunctions::SetNumThreads(num_threads);

#ifdef CHRONO_MKL
    // MKL solver settings
    ChSolverMKL* mkl_solver_stab = new ChSolverMKL;
    ChSolverMKL* mkl_solver_speed = new ChSolverMKL;
    system->ChangeSolverStab(mkl_solver_stab);
    system->ChangeSolverSpeed(mkl_solver_speed);
    mkl_solver_speed->SetSparsityPatternLock(true);
    mkl_solver_stab->SetSparsityPatternLock(true);
    mkl_solver_speed->SetVerbose(verbose);
#else
    // Default solver settings
    m_system->SetMaxItersSolverSpeed(100);
    m_system->SetMaxItersSolverStab(100);
    m_system->SetSolverType(ChSystem::SOLVER_SOR);
    m_system->SetTol(1e-10);
    m_system->SetTolForce(1e-8);
#endif

    // Integrator settings
    system->SetIntegrationType(ChSystem::INT_HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system->GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(50);
    integrator->SetAbsTolerances(5e-05, 1.8);
    integrator->SetMode(ChTimestepperHHT::POSITION);
    integrator->SetScaling(true);
    integrator->SetVerbose(verbose);
    integrator->SetMaxItersSuccess(5);

    // --------------
    // Create systems
    // --------------

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Full my_hmmwv(system);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetChassisVis(vis_type);
    my_hmmwv.SetWheelVis(vis_type);
    my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    my_hmmwv.SetPowertrainType(powertrain_model);
    my_hmmwv.SetDriveType(drive_type);
    my_hmmwv.SetTireType(ANCF);
    my_hmmwv.Initialize();

    // Access one of the ANCF tires of the vehicle.
    ////HMMWV_ANCFTire* tire_FL = static_cast<HMMWV_ANCFTire*>(my_hmmwv.GetTire(FRONT_LEFT));
    ////tire_FL->EnablePressure(true);

    // Create the terrain
    RigidTerrain terrain(my_hmmwv.GetSystem());
    terrain.SetContactMaterial(0.9f, 0.01f, 2e7f, 0.3f);
    terrain.SetColor(ChColor(0.8f, 0.8f, 0.5f));
    switch (terrain_model) {
        case RigidTerrain::FLAT:
            terrain.SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
            terrain.Initialize(terrainHeight, terrainLength, terrainWidth);
            break;
        case RigidTerrain::HEIGHT_MAP:
            terrain.SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16, 16);
            terrain.Initialize(vehicle::GetDataFile("terrain/height_maps/test64.bmp"), "test64", 128, 128, 0, 4);
            break;
        case RigidTerrain::MESH:
            terrain.SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 100, 100);
            terrain.Initialize(vehicle::GetDataFile("terrain/meshes/test.obj"), "test_mesh");
            break;
    }

    // Create the vehicle Irrlicht interface
    ChWheeledVehicleIrrApp app(&my_hmmwv.GetVehicle(), &my_hmmwv.GetPowertrain(), L"HMMWV ANCF tires Test");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // -----------------
    // Initialize output
    // -----------------

    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (povray_output) {
        if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    }

    std::string driver_rec_file = out_dir + "/driver_inputs.txt";
    utils::CSV_writer driver_csv(" ");

    // ------------------------
    // Create the driver system
    // ------------------------

    // Create the interactive driver system
    ChIrrGuiDriver driver(app);

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    // If needed, set input file for the driver and change the driver's mode.
    if (driver_mode == PLAYBACK) {
        // ATTENTION: It is assumed that the code was previously run in RECORD
        // mode and the driver_rec_file exists!
        driver.SetInputDataFile(driver_rec_file);
        driver.SetInputMode(ChIrrGuiDriver::DATAFILE);
    } else if (driver_mode == USER_FILE) {
        // Use an existing file in the Chrono::Vehicle data directory.
        driver.SetInputDataFile(vehicle::GetDataFile(driver_file));
        driver.SetInputMode(ChIrrGuiDriver::DATAFILE);
    }

    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------

    if (debug_output) {
        GetLog() << "\n\n============ System Configuration ============\n";
        my_hmmwv.LogHardpointLocations();
    }

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);
    int debug_steps = (int)std::ceil(debug_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    ChRealtimeStepTimer realtime_timer;
    int step_number = 0;
    int render_frame = 0;
    double time = 0;

    while (app.GetDevice()->run()) {
        time = my_hmmwv.GetSystem()->GetChTime();

        // End simulation
        if (time >= t_end)
            break;

        // Render scene and output POV-Ray data
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteShapesPovray(my_hmmwv.GetSystem(), filename);
            }

            render_frame++;
        }

        // Debug logging
        if (debug_output && step_number % debug_steps == 0) {
            GetLog() << "\n\n============ System Information ============\n";
            GetLog() << "Time = " << time << "\n\n";
            my_hmmwv.DebugLog(OUT_SPRINGS | OUT_SHOCKS | OUT_CONSTRAINTS);
        }

        // Collect output data from modules (for inter-module communication)
        double throttle_input = driver.GetThrottle();
        double steering_input = driver.GetSteering();
        double braking_input = driver.GetBraking();

        // Driver output
        if (driver_mode == RECORD) {
            driver_csv << time << steering_input << throttle_input << braking_input << std::endl;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, steering_input, braking_input, throttle_input, terrain);
        app.Synchronize(driver.GetInputModeAsString(), steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);
        driver.Advance(step);
        terrain.Advance(step);
        my_hmmwv.Advance(step);
        app.Advance(step);

        // Increment frame number
        step_number++;
    }

    if (driver_mode == RECORD) {
        driver_csv.write_to_file(driver_rec_file);
    }

    return 0;
}
