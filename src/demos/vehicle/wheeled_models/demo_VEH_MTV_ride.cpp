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
// Authors: Rainer Gericke
// =============================================================================
//
// Sample test program for MTV 5 ton simulation, demonstrating chassis torsion
// due to random wheel excitation.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
// All units SI.
//
// =============================================================================

#define USE_IRRLICHT

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RandomSurfaceTerrain.h"
#ifdef USE_IRRLICHT
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"
#endif
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_models/vehicle/mtv/MTV.h"

#include "chrono_thirdparty/filesystem/path.h"

#include <chrono>
#include <thread>
#include <math.h>

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::fmtv;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(-2, 0, 1.0);
ChQuaternion<> initRot(1, 0, 0, 0);

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType chassis_rear_vis_type = VisualizationType::NONE;
VisualizationType subchassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

RandomSurfaceTerrain::VisualisationType visType = RandomSurfaceTerrain::VisualisationType::MESH;

// Type of tire model (RIGID, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

ChVector<> vehCOM(-1.933, 0.014, 0.495);

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = step_size;

// Simulation end time
double tend = 300;

// Simulation maximum x coordinate
double xend = 300;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50
double output_step_size = 1e-2;

// vehicle driver inputs
// Desired vehicle speed (m/s)
double mph_to_ms = 0.44704;
double target_speed = 5 * mph_to_ms;

// output directory
const std::string out_dir = GetChronoOutputPath() + "MTV_RND_QUALITY";
const std::string pov_dir = out_dir + "/POVRAY";
bool povray_output = false;
bool data_output = true;

std::string path_file("paths/straightOrigin.txt");
std::string steering_controller_file("mtv/SteeringController.json");
std::string speed_controller_file("mtv/SpeedController.json");

std::string output_file_name("quality");

// =============================================================================

int main(int argc, char* argv[]) {
    int terrainCode = 5;
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
            GetLog() << "ISO8608 track A without correlation.\n";
            break;
        case 2:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_B_NOCORR;
            GetLog() << "ISO8608 track B without correlation.\n";
            break;
        case 3:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_C_NOCORR;
            GetLog() << "ISO8608 track C without correlation.\n";
            break;
        case 4:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_D_NOCORR;
            GetLog() << "ISO8608 track D without correlation.\n";
            break;
        case 5:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_E_NOCORR;
            GetLog() << "ISO8608 track E without correlation.\n";
            break;
        case 6:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_F_NOCORR;
            GetLog() << "ISO8608 track F without correlation.\n";
            break;
        case 7:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_G_NOCORR;
            GetLog() << "ISO8608 track G without correlation.\n";
            break;
        case 8:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_H_NOCORR;
            GetLog() << "ISO8608 track H without correlation.\n";
            break;
        case 11:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_A_CORR;
            GetLog() << "ISO8608 track A with correlation.\n";
            break;
        case 12:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_B_CORR;
            GetLog() << "ISO8608 track B with correlation.\n";
            break;
        case 13:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_C_CORR;
            GetLog() << "ISO8608 track C with correlation.\n";
            break;
        case 14:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_D_CORR;
            GetLog() << "ISO8608 track D with correlation.\n";
            break;
        case 15:
            surface = RandomSurfaceTerrain::SurfaceType::ISO8608_E_CORR;
            GetLog() << "ISO8608 track E with correlation.\n";
            break;
        case 21:
            surface = RandomSurfaceTerrain::SurfaceType::MAJOR_ROAD_CONCRETE;
            GetLog() << "Concrete Major Road with correlation.\n";
            break;
        case 22:
            surface = RandomSurfaceTerrain::SurfaceType::MAJOR_ROAD_ASPHALTIC_CONCRETE;
            GetLog() << "Asphaltic Concrete Major Road with correlation.\n";
            break;
        case 23:
            surface = RandomSurfaceTerrain::SurfaceType::MAIN_ROAD_ASPHALTIC_CONCRETE_ON_PAVEMENT;
            GetLog() << "Asphaltic Concrete Covered Pavement Main Road with correlation.\n";
            break;
        case 24:
            surface = RandomSurfaceTerrain::SurfaceType::MAIN_ROAD_ASPHALTIC_CONCRETE;
            GetLog() << "Asphaltic Concrete  Main Road with correlation.\n";
            break;
        case 25:
            surface = RandomSurfaceTerrain::SurfaceType::TRACK_TILED_CONCRETE_PAVEMENT;
            GetLog() << "Tiled Concrete Pavement Track with correlation.\n";
            break;
        default:
            GetLog()
                << "Invalid Terrain Code - (1-8 uncorrelated) or (11-15 correlated) or (21-25 Literature Examples)\n";
            return -1;
    }

    output_file_name += "_" + std::to_string((int)target_speed);
    output_file_name += "_" + std::to_string((int)terrainCode);
    target_speed = target_speed * mph_to_ms;

    // --------------
    // Create systems
    // --------------

    // Create the vehicle, set parameters, and initialize
    MTV mtv;
    mtv.SetContactMethod(ChContactMethod::NSC);
    mtv.SetChassisFixed(false);
    mtv.UseWalkingBeamRearSuspension(false);
    mtv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    mtv.SetTireType(tire_model);
    mtv.SetTireStepSize(tire_step_size);
    mtv.SetInitFwdVel(target_speed);
    mtv.Initialize();

    mtv.SetChassisVisualizationType(chassis_vis_type);
    mtv.SetChassisRearVisualizationType(chassis_rear_vis_type);
    mtv.SetSubchassisVisualizationType(subchassis_vis_type);
    mtv.SetSuspensionVisualizationType(suspension_vis_type);
    mtv.SetSteeringVisualizationType(steering_vis_type);
    mtv.SetWheelVisualizationType(wheel_vis_type);
    mtv.SetTireVisualizationType(tire_vis_type);

    std::cout << "Vehicle mass: " << mtv.GetVehicle().GetMass() << std::endl;

    // ------------------
    // Create the terrain
    // ------------------
    RandomSurfaceTerrain terrain(mtv.GetSystem(), xend);
    terrain.Initialize(surface, 2, visType);
    GetLog() << "RMS = " << (1000.0 * terrain.GetRMS()) << " mm\n";
    GetLog() << "IRI = " << terrain.GetIRI() << " mm/m\n";

    // create the driver
    auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
    ChPathFollowerDriver driver(mtv.GetVehicle(), vehicle::GetDataFile(steering_controller_file),
                                vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed, false);
    driver.Initialize();

    // -------------------------------------
    // Create the vehicle Irrlicht interface
    // Create the driver system
    // -------------------------------------

#ifdef USE_IRRLICHT
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("MTV ride & twist test");
    vis->SetChaseCamera(trackPoint, 10.0, 0.5);
    vis->Initialize();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->GetSceneManager()->setAmbientLight(irr::video::SColorf(0.1f, 0.1f, 0.1f, 1.0f));
    vis->AddLight(ChVector<>(-50, -30, 40), 200, ChColor(0.7f, 0.7f, 0.7f));
    vis->AddLight(ChVector<>(+10, +30, 40), 200, ChColor(0.7f, 0.7f, 0.7f));
    vis->AttachVehicle(&mtv.GetVehicle());

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = vis->GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = vis->GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);
#endif

    // -------------
    // Prepare output
    // -------------

    if (data_output || povray_output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }
    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        driver.ExportPathPovray(out_dir);
    }

    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Number of simulation steps between two output frames
    int output_steps = (int)std::ceil(output_step_size / step_size);

    csv << "time";
    csv << "throttle";
    csv << "MotorSpeed";
    csv << "CurrentTransmissionGear";
    for (int i = 0; i < 6; i++) {
        csv << "WheelTorque";
    }
    for (int i = 0; i < 6; i++) {
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

    for (int i = 0; i < 6; i++) {
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
    csv << "W4 Pos x"
        << "W4 Pos Y"
        << "W4 Pos Z"
        << "W5 Pos x"
        << "W5 Pos Y"
        << "W5 Pos Z";

    for (auto& axle : mtv.GetVehicle().GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            csv << wheel->GetPos();
        }
    }

    csv << std::endl;

    // ---------------
    // Simulation loop
    // ---------------

    mtv.GetVehicle().LogSubsystemTypes();

    std::cout << "data at: " << vehicle::GetDataFile(steering_controller_file) << std::endl;
    std::cout << "data at: " << vehicle::GetDataFile(speed_controller_file) << std::endl;
    std::cout << "data at: " << vehicle::GetDataFile(path_file) << std::endl;

    int step_number = 0;
    int render_frame = 0;

    double time = 0;

#ifdef USE_IRRLICHT
    while (vis->Run() && (time < tend) && (mtv.GetVehicle().GetPos().x() < xend)) {
#else
    while ((time < tend) && (mtv.GetVehicle().GetPos().x() < xend)) {
#endif
        time = mtv.GetSystem()->GetChTime();

#ifdef USE_IRRLICHT
        // path visualization
        const ChVector<>& pS = driver.GetSteeringController().GetSentinelLocation();
        const ChVector<>& pT = driver.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

        // std::cout<<"Target:\t"<<(irr::f32)pT.x()<<",\t "<<(irr::f32)pT.y()<<",\t "<<(irr::f32)pT.z()<<std::endl;
        // std::cout<<"Vehicle:\t"<<mtv.GetVehicle().GetChassisBody()->GetPos().x()
        //   <<",\t "<<mtv.GetVehicle().GetChassisBody()->GetPos().y()<<",\t "
        //   <<mtv.GetVehicle().GetChassisBody()->GetPos().z()<<std::endl;

        // Render scene
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }
#endif

        if (povray_output && step_number % render_steps == 0) {
            char filename[100];
            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
            utils::WriteVisualizationAssets(mtv.GetSystem(), filename);
            render_frame++;
        }

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        mtv.Synchronize(time, driver_inputs, terrain);

#ifdef USE_IRRLICHT
        vis->Synchronize("Follower driver", driver_inputs);
#endif

        // Advance simulation for one timestep for all modules

        driver.Advance(step_size);
        terrain.Advance(step_size);
        mtv.Advance(step_size);

#ifdef USE_IRRLICHT
        vis->Advance(step_size);
#endif

        if (data_output && step_number % output_steps == 0) {
            // std::cout << time << std::endl;
            csv << time;
            csv << driver_inputs.m_throttle;
            csv << mtv.GetVehicle().GetPowertrain()->GetMotorSpeed();
            csv << mtv.GetVehicle().GetPowertrain()->GetCurrentTransmissionGear();
            for (int axle = 0; axle < 3; axle++) {
                csv << mtv.GetVehicle().GetDriveline()->GetSpindleTorque(axle, LEFT);
                csv << mtv.GetVehicle().GetDriveline()->GetSpindleTorque(axle, RIGHT);
            }
            for (int axle = 0; axle < 3; axle++) {
                csv << mtv.GetVehicle().GetSpindleAngVel(axle, LEFT);
                csv << mtv.GetVehicle().GetSpindleAngVel(axle, RIGHT);
            }
            csv << mtv.GetVehicle().GetSpeed();
            csv << mtv.GetVehicle().GetPointAcceleration(mtv.GetVehicle().GetChassis()->GetLocalDriverCoordsys().pos);

            csv << mtv.GetVehicle().GetPointAcceleration(vehCOM);

            for (auto& axle : mtv.GetVehicle().GetAxles()) {
                for (auto& wheel : axle->GetWheels()) {
                    csv << wheel->GetTire()->ReportTireForce(&terrain).force;
                }
            }

            csv << mtv.GetVehicle().GetPowertrain()->GetMotorTorque();

            csv << std::endl;
        }

        // Increment frame number
        step_number++;
    }

    if (data_output) {
        csv.write_to_file(out_dir + "/" + output_file_name + ".dat");
    }

    return 0;
}
