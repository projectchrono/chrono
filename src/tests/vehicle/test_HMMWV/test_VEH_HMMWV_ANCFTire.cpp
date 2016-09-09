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
// HMMWV full model using ANCF, RIGID, or RIGID_MESH tires on rigid terrain.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

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

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

// Number of OpenMP threads
int num_threads = 4;

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 1.2);
ChQuaternion<> initRot(1, 0, 0, 0);
////ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
////ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
////ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
////ChQuaternion<> initRot(0, 0, 0, 1);

// Visualization type for chassis (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;

// Type of tire type (ANCF, RIGID, RIGID_MESH)
TireModelType tire_model = TireModelType::ANCF;

// Enable/disable tire visualization
bool tire_vis = true;

// Type of powertrain model (SHAFTS, SIMPLE)
PowertrainModelType powertrain_model = PowertrainModelType::SHAFTS;

// Drive type (FWD, RWD, or AWD)
DrivelineType drive_type = DrivelineType::AWD;

// Rigid terrain (RigidTerrain::FLAT, RigidTerrain::HEIGHT_MAP, RigidTerrain::MESH)
RigidTerrain::Type terrain_model = RigidTerrain::FLAT;

// Use material properties for DEM-P contact method?
bool use_mat_properties = true;

// Terrain dimensions (for FLAT terrain)
double terrainHeight = 0;      // terrain height (FLAT terrain only)
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Point on chassis tracked by the camera (for Irrlicht visualization)
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation step sizes
double step_size = 1e-4;
// Simulation end time
double t_end = 5;
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

class MyDriver : public ChDriver {
public:
    MyDriver(ChVehicle& vehicle, double delay) : ChDriver(vehicle), m_delay(delay) {}
    ~MyDriver() {}

    virtual void Synchronize(double time) override {
        m_throttle = 0;
        m_steering = 0;
        m_braking = 0;

        double eff_time = time - m_delay;

        // Do not generate any driver inputs for a duration equal to m_delay.
        if (eff_time < 0)
            return;

        if (eff_time > 0.2)
            m_throttle = 0.8;
        else
            m_throttle = 4 * eff_time;
    }

private:
    double m_delay;
};

// =============================================================================

int main(int argc, char* argv[]) {
    // ----------------------------------
    // Create the (sequential) DEM system
    // ----------------------------------

    ChSystemDEM* system = new ChSystemDEM(use_mat_properties);
    system->Set_G_acc(ChVector<>(0, 0, -9.81));

    // Set number threads
    system->SetParallelThreadNumber(num_threads);
    CHOMPfunctions::SetNumThreads(num_threads);

#ifdef CHRONO_MKL
    // MKL solver settings
    ChSolverMKL<>* mkl_solver_stab = new ChSolverMKL<>;
    ChSolverMKL<>* mkl_solver_speed = new ChSolverMKL<>;
    system->ChangeSolverStab(mkl_solver_stab);
    system->ChangeSolverSpeed(mkl_solver_speed);
    mkl_solver_speed->SetSparsityPatternLock(true);
    mkl_solver_stab->SetSparsityPatternLock(true);
    mkl_solver_speed->SetVerbose(verbose);
#else
    // Default solver settings
    system->SetMaxItersSolverSpeed(100);
    system->SetMaxItersSolverStab(100);
    system->SetSolverType(ChSystem::SOLVER_SOR);
    system->SetTol(1e-10);
    system->SetTolForce(1e-8);
#endif

    // Integrator settings
    system->SetIntegrationType(ChSystem::INT_HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system->GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(50);
    integrator->SetAbsTolerances(5e-05, 1.8);
    integrator->SetMode(ChTimestepperHHT::POSITION);
    integrator->SetStepControl(true);
    integrator->SetModifiedNewton(false);
    integrator->SetScaling(true);
    integrator->SetVerbose(verbose);
    integrator->SetMaxItersSuccess(5);

    // --------------
    // Create systems
    // --------------

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Full my_hmmwv(system);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetChassisVis(chassis_vis_type);
    my_hmmwv.SetWheelVis(VisualizationType::NONE);
    my_hmmwv.EnableTireVis(tire_vis);
    my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    my_hmmwv.SetPowertrainType(powertrain_model);
    my_hmmwv.SetDriveType(drive_type);
    my_hmmwv.SetTireType(tire_model);
    my_hmmwv.Initialize();

    // Downcast tires (if needed)
    switch (tire_model) {
        case TireModelType::ANCF: {
            HMMWV_ANCFTire* tire_FL = static_cast<HMMWV_ANCFTire*>(my_hmmwv.GetTire(FRONT_LEFT));
            HMMWV_ANCFTire* tire_FR = static_cast<HMMWV_ANCFTire*>(my_hmmwv.GetTire(FRONT_RIGHT));
            HMMWV_ANCFTire* tire_RL = static_cast<HMMWV_ANCFTire*>(my_hmmwv.GetTire(REAR_LEFT));
            HMMWV_ANCFTire* tire_RR = static_cast<HMMWV_ANCFTire*>(my_hmmwv.GetTire(REAR_RIGHT));
            ////tire_FL->EnablePressure(false);
            ////tire_FR->EnablePressure(false);
            ////tire_RL->EnablePressure(false);
            ////tire_RR->EnablePressure(false);

            break;
        }
        case TireModelType::RIGID:
        case TireModelType::RIGID_MESH: {
            HMMWV_RigidTire* tire_FL = static_cast<HMMWV_RigidTire*>(my_hmmwv.GetTire(FRONT_LEFT));
            HMMWV_RigidTire* tire_FR = static_cast<HMMWV_RigidTire*>(my_hmmwv.GetTire(FRONT_RIGHT));
            HMMWV_RigidTire* tire_RL = static_cast<HMMWV_RigidTire*>(my_hmmwv.GetTire(REAR_LEFT));
            HMMWV_RigidTire* tire_RR = static_cast<HMMWV_RigidTire*>(my_hmmwv.GetTire(REAR_RIGHT));

            break;
        }
    }

    // Create the terrain
    RigidTerrain terrain(my_hmmwv.GetSystem());
    terrain.SetContactMaterial(0.9f, 0.01f, 2e6f, 0.3f, 2e5f, 40.0f, 2e5f, 20.0f);
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

    // ------------------------
    // Create the driver system
    // ------------------------

    MyDriver driver(my_hmmwv.GetVehicle(), 0.5);
    driver.Initialize();

    // ----------------------------
    // Complete system construction
    // ----------------------------

    system->SetupInitial();

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
    int step_number = 0;
    int render_frame = 0;
    double time = 0;

    while (app.GetDevice()->run()) {
        time = my_hmmwv.GetSystem()->GetChTime();

        // End simulation
        if (time >= t_end)
            break;

        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();

        // Render scene and output POV-Ray data
        if (povray_output && step_number % render_steps == 0) {
            char filename[100];
            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
            utils::WriteShapesPovray(my_hmmwv.GetSystem(), filename);
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

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, steering_input, braking_input, throttle_input, terrain);
        app.Synchronize("", steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        my_hmmwv.Advance(step_size);
        app.Advance(step_size);

        // Increment frame number
        step_number++;
    }

    return 0;
}
