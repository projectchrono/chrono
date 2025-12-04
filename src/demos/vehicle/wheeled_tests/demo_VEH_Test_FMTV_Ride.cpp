// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
//
// Test program for LMTV 2.5 ton and the MTV 5 ton trucks, demonstrating chassis
// torsion due to random wheel excitation.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
// All units SI.
//
// =============================================================================

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/RandomSurfaceTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono/input_output/ChWriterCSV.h"

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

#include "../WheeledVehicleModels.h"

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Initial vehicle location
ChVector3d initLoc(-2, 0, 1.0);
ChVector3d vehCOM(-1.933, 0.014, 0.495);

// Simulation step sizes
double step_size = 1e-3;

// Simulation end time
double tend = 300;

// Simulation end x coordinate
double xend = 300;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50
double output_step_size = 1e-2;

// Desired vehicle speed (m/s)
double mph_to_ms = 0.44704;
double target_speed = 5 * mph_to_ms;

// output
bool data_output = true;

std::string path_file("paths/straightOrigin.txt");
std::string steering_controller_file("mtv/SteeringController.json");
std::string speed_controller_file("mtv/SpeedController.json");

std::string output_file_name("quality");

// =============================================================================

int main(int argc, char* argv[]) {
    int terrainCode = 1;
    RandomSurfaceTerrain::SurfaceType surface = RandomSurfaceTerrain::SurfaceType::FLAT;
    // read in argument as simulation duration
    switch (argc) {
        default:
        case 1:
            target_speed = 5;
            break;
        case 2:
            target_speed = atof(argv[1]);
            break;
        case 3:
            target_speed = atof(argv[1]);
            terrainCode = atoi(argv[2]);
            break;
    }
    switch (terrainCode) {
        case 1:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_A_NOCORR;
            std::cout << "ISO8608 track A without correlation.\n";
            break;
        case 2:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_B_NOCORR;
            std::cout << "ISO8608 track B without correlation.\n";
            break;
        case 3:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_C_NOCORR;
            std::cout << "ISO8608 track C without correlation.\n";
            break;
        case 4:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_D_NOCORR;
            std::cout << "ISO8608 track D without correlation.\n";
            break;
        case 5:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_E_NOCORR;
            std::cout << "ISO8608 track E without correlation.\n";
            break;
        case 6:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_F_NOCORR;
            std::cout << "ISO8608 track F without correlation.\n";
            break;
        case 7:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_G_NOCORR;
            std::cout << "ISO8608 track G without correlation.\n";
            break;
        case 8:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_H_NOCORR;
            std::cout << "ISO8608 track H without correlation.\n";
            break;
        case 11:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_A_CORR;
            std::cout << "ISO8608 track A with correlation.\n";
            break;
        case 12:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_B_CORR;
            std::cout << "ISO8608 track B with correlation.\n";
            break;
        case 13:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_C_CORR;
            std::cout << "ISO8608 track C with correlation.\n";
            break;
        case 14:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_D_CORR;
            std::cout << "ISO8608 track D with correlation.\n";
            break;
        case 15:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_E_CORR;
            std::cout << "ISO8608 track E with correlation.\n";
            break;
        case 21:
            surface = RandomSurfaceTerrain::SurfaceType::MAJOR_ROAD_CONCRETE;
            std::cout << "Concrete Major Road with correlation.\n";
            break;
        case 22:
            surface = RandomSurfaceTerrain::SurfaceType::MAJOR_ROAD_ASPHALTIC_CONCRETE;
            std::cout << "Asphaltic Concrete Major Road with correlation.\n";
            break;
        case 23:
            surface = RandomSurfaceTerrain::SurfaceType::MAIN_ROAD_ASPHALTIC_CONCRETE_ON_PAVEMENT;
            std::cout << "Asphaltic Concrete Covered Pavement Main Road with correlation.\n";
            break;
        case 24:
            surface = RandomSurfaceTerrain::SurfaceType::MAIN_ROAD_ASPHALTIC_CONCRETE;
            std::cout << "Asphaltic Concrete  Main Road with correlation.\n";
            break;
        case 25:
            surface = RandomSurfaceTerrain::SurfaceType::TRACK_TILED_CONCRETE_PAVEMENT;
            std::cout << "Tiled Concrete Pavement Track with correlation.\n";
            break;
        default:
            std::cout
                << "Invalid Terrain Code - (1-8 uncorrelated) or (11-15 correlated) or (21-25 Literature Examples)\n";
            return -1;
    }

    output_file_name += "_" + std::to_string((int)target_speed);
    output_file_name += "_" + std::to_string((int)terrainCode);
    target_speed = target_speed * mph_to_ms;

    // ----------------
    // Create the truck
    // ----------------

    std::vector<std::shared_ptr<WheeledVehicleModel>> models = {chrono_types::make_shared<LMTV_Model>(),
                                                                chrono_types::make_shared<MTV_Model>()};
    int which = 0;
    std::cout << "1  LMTV" << std::endl;
    std::cout << "2  MTV" << std::endl;
    std::cout << "\nSelect vehicle: ";
    std::cin >> which;
    std::cout << std::endl;
    ChClampValue(which, 1, 2);
    auto vehicle_model = models[which - 1];

    // Create the vehicle model
    vehicle_model->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    vehicle_model->Create(ChContactMethod::NSC, ChCoordsys<>(initLoc, QUNIT), false);
    auto& vehicle = vehicle_model->GetVehicle();

    std::string model_name = (which == 1) ? "LMTV" : "MTV";
    int num_wheels = 2 * vehicle.GetNumberAxles();

    std::cout << "Vehicle mass: " << vehicle.GetMass() << std::endl;

    // ------------------
    // Create the terrain
    // ------------------

    RandomSurfaceTerrain terrain(vehicle.GetSystem(), xend);
    terrain.Initialize(surface, 2, RandomSurfaceTerrain::VisualisationType::MESH);
    std::cout << "RMS = " << (1000.0 * terrain.GetRMS()) << " mm\n";
    std::cout << "IRI = " << terrain.GetIRI() << " mm/m\n";

    // -----------------
    // Create the driver
    // -----------------

    auto path = ChBezierCurve::Read(GetVehicleDataFile(path_file));
    ChPathFollowerDriver driver(vehicle, GetVehicleDataFile(steering_controller_file),
                                GetVehicleDataFile(speed_controller_file), path, "my_path", target_speed);
    driver.Initialize();

    // ---------------------------------
    // Create the run-time visualization
    // ---------------------------------

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

    // -------------
    // Prepare output
    // -------------

    std::string out_dir;
    if (data_output) {
        out_dir = GetChronoOutputPath() + "FMTV_RIDE";
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
        out_dir = out_dir + "/" + vehicle_model->ModelName();
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    ChWriterCSV csv("\t");
    csv.Stream().setf(std::ios::scientific | std::ios::showpos);
    csv.Stream().precision(6);

    // Number of simulation steps between two render and output frames
    int render_steps = (int)std::ceil(render_step_size / step_size);
    int output_steps = (int)std::ceil(output_step_size / step_size);

    csv << "time";
    csv << "throttle";
    csv << "MotorSpeed";
    csv << "CurrentTransmissionGear";
    for (int i = 0; i < num_wheels; i++) {
        csv << "WheelTorque";
    }
    for (int i = 0; i < num_wheels; i++) {
        csv << "WheelAngVelX";
        csv << "WheelAngVelY";
        csv << "WheelAngVelZ";
    }
    csv << "VehicleSpeed";
    csv << "VehicleDriverAccelerationX";
    csv << "VehicleDriverAccelerationY";
    csv << "VehicleDriverAccelerationZ";

    csv << "VehicleCOMAccelerationX";
    csv << "VehicleCOMAccelerationY";
    csv << "VehicleCOMAccelerationZ";

    for (int i = 0; i < num_wheels; i++) {
        csv << "TireForce";
    }
    csv << "EngineTorque";

    csv << "W0 Pos x"
        << "W0 Pos Y"
        << "W0 Pos Z"
        << "W1 Pos x"
        << "W1 Pos Y"
        << "W1 Pos Z";
    csv << "W2 Pos x"
        << "W2 Pos Y"
        << "W2 Pos Z"
        << "W3 Pos x"
        << "W3 Pos Y"
        << "W3 Pos Z";
    if (num_wheels == 6) {
        csv << "W4 Pos x"
            << "W4 Pos Y"
            << "W4 Pos Z"
            << "W5 Pos x"
            << "W5 Pos Y"
            << "W5 Pos Z";
    }

    for (auto& axle : vehicle.GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            csv << wheel->GetPos();
        }
    }

    csv << std::endl;

    // ---------------
    // Simulation loop
    // ---------------

    std::cout << "data at: " << GetVehicleDataFile(steering_controller_file) << std::endl;
    std::cout << "data at: " << GetVehicleDataFile(speed_controller_file) << std::endl;
    std::cout << "data at: " << GetVehicleDataFile(path_file) << std::endl;

    int step_number = 0;
    double time = 0;

    while (time < tend && vehicle.GetPos().x() < xend) {
        time = vehicle.GetSystem()->GetChTime();

        if (vis && step_number % render_steps == 0) {
            if (!vis->Run())
                break;

            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        vehicle_model->Synchronize(time, driver_inputs, terrain);
        if (vis)
            vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        vehicle_model->Advance(step_size);
        if (vis)
            vis->Advance(step_size);

        if (data_output && step_number % output_steps == 0) {
            csv << time;
            csv << driver_inputs.m_throttle;
            csv << vehicle.GetEngine()->GetMotorSpeed();
            csv << vehicle.GetTransmission()->GetCurrentGear();
            for (unsigned int axle = 0; axle < vehicle.GetNumberAxles(); axle++) {
                csv << vehicle.GetDriveline()->GetSpindleTorque(axle, LEFT);
                csv << vehicle.GetDriveline()->GetSpindleTorque(axle, RIGHT);
            }
            for (unsigned int axle = 0; axle < vehicle.GetNumberAxles(); axle++) {
                csv << vehicle.GetSpindleAngVel(axle, LEFT);
                csv << vehicle.GetSpindleAngVel(axle, RIGHT);
            }
            csv << vehicle.GetSpeed();
            csv << vehicle.GetPointAcceleration(vehicle.GetChassis()->GetLocalDriverCoordsys().pos);

            csv << vehicle.GetPointAcceleration(vehCOM);

            for (auto& axle : vehicle.GetAxles()) {
                for (auto& wheel : axle->GetWheels()) {
                    csv << wheel->GetTire()->ReportTireForce(&terrain).force;
                }
            }

            csv << vehicle.GetEngine()->GetOutputMotorshaftTorque();

            csv << std::endl;
        }

        // Increment frame number
        step_number++;
    }

    if (data_output) {
        csv.WriteToFile(out_dir + "/" + output_file_name + ".dat");
    }

    return 0;
}
