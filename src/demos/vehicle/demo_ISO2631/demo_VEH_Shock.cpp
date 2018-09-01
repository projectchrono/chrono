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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Main driver function for a vehicle specified through JSON files + example for
// obtaining ride quality results presented by ISO 2631-1
//
// If using the Irrlicht interface, driver inputs are obtained from the keyboard.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================
#include <vector>

#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChStream.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/powertrain/SimplePowertrain.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"

#include "chrono_vehicle/ChConfigVehicle.h"

// If Irrlicht support is available...
#ifdef CHRONO_IRRLICHT
// ...include additional headers
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

// ...and specify whether the demo should actually use Irrlicht
#define USE_IRRLICHT
#endif

// =============================================================================

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::vehicle;

// JSON file for vehicle model
std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle.json");

// JSON file for powertrain (simple)
std::string simplepowertrain_file("generic/powertrain/SimplePowertrain.json");

// JSON files tire models (rigid)
std::string rigidtire_file("hmmwv/tire/HMMWV_RigidTire.json");
std::string tmeasytire_file("hmmwv/tire/HMMWV_TMeasyTire.json");

// Driver input file (if not using Irrlicht)
std::string driver_file("generic/driver/Sample_Maneuver.txt");

std::string path_file("paths/straightOrigin.txt");
std::string steering_controller_file("hmmwv/SteeringController.json");
std::string speed_controller_file("hmmwv/SpeedController.json");

// Initial vehicle position
ChVector<> initLoc(90, 0, 0.6);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);

// Simulation step size (should not be too high!)
double step_size = 1e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Point on chassis tracked by the camera (Irrlicht only)
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation length (Povray only)
double tend = 20.0;

// Output directories (Povray only)
const std::string out_dir = GetChronoOutputPath() + "WHEELED_JSON";
const std::string pov_dir = out_dir + "/POVRAY";

// =============================================================================

int main(int argc, char *argv[])
{
    GetLog() << "Copyright (c) 2018 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    
    const int heightVals[6] = { 0, 50, 100, 150, 200, 250 };
    int iObstacle = 1;
    double target_speed = 5.0;
    // JSON files for terrain
    std::string rigidterrain_file("terrain/RigidObstacle1.json");
   
    switch(argc) {
        default:
        case 1:
            GetLog() << "usage: demo_VEH_Shock [ObstacleNumber [Speed]]\n\n";
            GetLog() << "Using standard values for simulation:\n"
                << "Terrain No. = " << iObstacle << " (" << heightVals[iObstacle] << " mm Obstacle Height)\n"
                << "Speed       = " << target_speed << " m/s\n";
                break;
        case 2:
            if(atoi(argv[1]) >= 1 && atoi(argv[1]) <= 5) {
                iObstacle = atoi(argv[1]);
                rigidterrain_file = "terrain/RigidObstacle" + std::to_string(iObstacle) + ".json";
            }
            GetLog() << "Using values for simulation:\n"
                << "Terrain No. = " << iObstacle << " (" << heightVals[iObstacle] << " mm Obstacle Height)\n"
                << "Speed       = " << target_speed << " m/s\n";
                break;
        case 3:
            if(atoi(argv[1]) >= 1 && atoi(argv[1]) <= 5) {
                iObstacle = atoi(argv[1]);
                rigidterrain_file = "terrain/RigidObstacle" + std::to_string(iObstacle) + ".json";
            }
            target_speed = atof(argv[2]);
            GetLog() << "Using values for simulation:\n"
                << "Terrain No. = " << iObstacle << " (" << heightVals[iObstacle] << " mm Obstacle Height)\n"
                << "Speed       = " << target_speed << " m/s\n";
                break;
    }
    // --------------------------
    // Create the various modules
    // --------------------------

    // Create the vehicle system
    WheeledVehicle vehicle(vehicle::GetDataFile(vehicle_file), ChMaterialSurface::NSC);
    vehicle.Initialize(ChCoordsys<>(initLoc, initRot));
    ////vehicle.GetChassis()->SetFixed(true);
    vehicle.SetStepsize(step_size);
    vehicle.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::NONE);

    GetLog() << "\nBe patient - startup may take some time... \n";

    // Create the ground
    RigidTerrain terrain(vehicle.GetSystem(), vehicle::GetDataFile(rigidterrain_file));

    // Create and initialize the powertrain system
    SimplePowertrain powertrain(vehicle::GetDataFile(simplepowertrain_file));
    powertrain.Initialize(vehicle.GetChassisBody(), vehicle.GetDriveshaft());

    // Create and initialize the tires
    int num_axles = vehicle.GetNumberAxles();
    int num_wheels = 2 * num_axles;
#ifdef USE_RIGID_TIRE
    // leads to poor results - much too high!
    std::vector<std::shared_ptr<RigidTire> > tires(num_wheels);
#else
    // handling tire works, but sill too high sesults
    // a validated flexible tire model would be the best choice
    std::vector<std::shared_ptr<TMeasyTire> > tires(num_wheels);
#endif
    for (int i = 0; i < num_wheels; i++) {
#ifdef USE_RIGID_TIRE
        tires[i] = std::make_shared<RigidTire>(vehicle::GetDataFile(rigidtire_file));
#else
        tires[i] = std::make_shared<TMeasyTire>(vehicle::GetDataFile(tmeasytire_file));
#endif
        tires[i]->Initialize(vehicle.GetWheelBody(i), VehicleSide(i % 2));
        tires[i]->SetVisualizationType(VisualizationType::MESH);
    }
    
    ChISO2631_Shock_SeatCushionLogger seat_logger(step_size);
 
#ifdef USE_IRRLICHT

#ifdef USE_RIGID_TIRE
    std::wstring windowTitle = L"Vehicle Shock Test Demo (Rigid Tire) - " + std::to_wstring(heightVals[iObstacle]) + L" mm Obstacle Height";
    ChVehicleIrrApp app(&vehicle, &powertrain, windowTitle.c_str());
#else
    std::wstring windowTitle =  L"Vehicle Shock Test Demo (TMeasy Tire) - " + std::to_wstring(heightVals[iObstacle]) + L" mm Obstacle Height";
    ChVehicleIrrApp app(&vehicle, &powertrain, windowTitle.c_str());
#endif

    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);

    app.SetTimestep(step_size);

    app.AssetBindAll();
    app.AssetUpdateAll();

    //ChIrrGuiDriver driver(app);
    //create the driver
    auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
    ChPathFollowerDriver driver(vehicle, vehicle::GetDataFile(steering_controller_file),
                                vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed, false);

#else

    auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
    ChPathFollowerDriver driver(vehicle, vehicle::GetDataFile(steering_controller_file),
                                vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed, false);

#endif

    driver.Initialize();

    // -----------------
    // Initialize output
    // -----------------

    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << pov_dir << std::endl;
        return 1;
    }

    // Generate JSON information with available output channels
    std::string out_json = vehicle.ExportComponentList();
    std::cout << out_json << std::endl;
    vehicle.ExportComponentList(out_dir + "/component_list.json");

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    TerrainForces tire_forces(num_wheels);
    WheelStates wheel_states(num_wheels);
    double driveshaft_speed;
    double powertrain_torque;
    double throttle_input;
    double steering_input;
    double braking_input;

    // Initialize simulation frame counter and simulation time
    int step_number = 0;
    double time = 0;

    // Logging of seat acceleration data on flat road surface is useless
    double xstart = 100.0; // start logging when the vehicle crosses this x position
    double xend   = 160.0; // end logging here, this also the end of our world

#ifdef USE_IRRLICHT

    ChRealtimeStepTimer realtime_timer;
    

    while (app.GetDevice()->run()) {
 
        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();

        // Collect output data from modules (for inter-module communication)
        throttle_input = driver.GetThrottle();
        steering_input = driver.GetSteering();
        braking_input = driver.GetBraking();
        powertrain_torque = powertrain.GetOutputTorque();
        driveshaft_speed = vehicle.GetDriveshaftSpeed();
        for (int i = 0; i < num_wheels; i++) {
            tire_forces[i] = tires[i]->GetTireForce();
            wheel_states[i] = vehicle.GetWheelState(i);
        }

        // Update modules (process inputs from other modules)
        time = vehicle.GetSystem()->GetChTime();
        driver.Synchronize(time);
        powertrain.Synchronize(time, throttle_input, driveshaft_speed);
        vehicle.Synchronize(time, steering_input, braking_input, powertrain_torque, tire_forces);
        terrain.Synchronize(time);
        for (int i = 0; i < num_wheels; i++)
            tires[i]->Synchronize(time, wheel_states[i], terrain);
        //app.Synchronize(driver.GetInputModeAsString(), steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);
        driver.Advance(step);
        powertrain.Advance(step);
        vehicle.Advance(step);
        terrain.Advance(step);
        for (int i = 0; i < num_wheels; i++)
            tires[i]->Advance(step);
        app.Advance(step);

        double xpos = vehicle.GetWheelPos(0).x();
        if(xpos >= xend) {
            break;
        }
        if(xpos >= xstart) {
            double     speed    = vehicle.GetVehicleSpeed();
            ChVector<> seat_acc = vehicle.GetVehicleAcceleration(vehicle.GetChassis()->GetLocalDriverCoordsys().pos);
            seat_logger.AddData(speed,seat_acc);
        }
        
        // Increment frame number
        step_number++;

        app.EndScene();
    }

#else

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    int render_frame = 0;
    char filename[100];

    double v_pos; 
    while ((v_pos = vehicle.GetWheelPos(0).x()) < xend) {
        if (step_number % render_steps == 0) {
            // Output render data
            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
            utils::WriteShapesPovray(vehicle.GetSystem(), filename);
            std::cout << "Output frame:   " << render_frame << std::endl;
            std::cout << "Sim frame:      " << step_number << std::endl;
            std::cout << "Time:           " << time << std::endl;
            std::cout << "   throttle: " << driver.GetThrottle() << "   steering: " << driver.GetSteering()
                      << "   braking:  " << driver.GetBraking() << std::endl;
            std::cout << std::endl;
            render_frame++;
        }

        // Collect output data from modules (for inter-module communication)
        throttle_input = driver.GetThrottle();
        steering_input = driver.GetSteering();
        braking_input = driver.GetBraking();
        powertrain_torque = powertrain.GetOutputTorque();
        driveshaft_speed = vehicle.GetDriveshaftSpeed();
        for (int i = 0; i < num_wheels; i++) {
            tire_forces[i] = tires[i]->GetTireForce();
            wheel_states[i] = vehicle.GetWheelState(i);
        }

        // Update modules (process inputs from other modules)
        time = vehicle.GetSystem()->GetChTime();
        driver.Synchronize(time);
        powertrain.Synchronize(time, throttle_input, driveshaft_speed);
        vehicle.Synchronize(time, steering_input, braking_input, powertrain_torque, tire_forces);
        terrain.Synchronize(time);
        for (int i = 0; i < num_wheels; i++)
            tires[i]->Synchronize(time, wheel_states[i], terrain);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        powertrain.Advance(step_size);
        vehicle.Advance(step_size);
        terrain.Advance(step_size);
        for (int i = 0; i < num_wheels; i++)
            tires[i]->Advance(step_size);

        if(v_pos >= xstart) {
           double     speed    = vehicle.GetVehicleSpeed();
           ChVector<> seat_acc = vehicle.GetVehicleAcceleration(vehicle.GetChassis()->GetLocalDriverCoordsys().pos);
            seat_logger.AddData(speed,seat_acc);
        }
        // Increment frame number
        step_number++;
    }

#endif

    double se_low  = 0.5;
    double se_high = 0.8;
    double sig_speed = seat_logger.GetSpeed();
    double se = seat_logger.GetSe();
    
    GetLog() << "Ride Quality Results:\n";
    GetLog() << "  Significant Speed                        Vsig = " << sig_speed << " m/s\n";
    GetLog() << "  Equivalent Static Spine Compressive Stress Se = " << se << " MPa\n";
    if(se <= se_low) {
        GetLog() << "Se <= " << se_low << " MPa (ok) - low risc of health effect, below limit for average occupants\n";
    } else if(se >= se_high) {
        GetLog() << "Se >= " << se_high << " MPa - severe risc of health effect!\n";
    } else {
        GetLog() << "Se is between [" << se_low << ";" << se_high << "] - risc of health effects, above limit!\n";
    }
    return 0;
}