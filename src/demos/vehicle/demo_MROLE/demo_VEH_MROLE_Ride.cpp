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
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
// All units SI.
//
// =============================================================================

#define USE_IRRLICHT

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RandomSurfaceTerrain.h"
#ifdef USE_IRRLICHT
    #include "chrono_vehicle/driver/ChIrrGuiDriver.h"
    #include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
#endif
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_models/vehicle/mtv/LMTV.h"
#include "chrono_models/vehicle/mrole/mrole.h"

#include "chrono_thirdparty/filesystem/path.h"

#include <chrono>
#include <thread>
#include <math.h>

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::mrole;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(-100, 0, 0.8);
ChQuaternion<> initRot(1, 0, 0, 0);

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType chassis_rear_vis_type = VisualizationType::MESH;
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

// Simulation end x coordinate
double xmax = 300.0;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50
double output_step_size = 1e-2;

// vehicle driver inputs
// Desired vehicle speed (m/s)
const double ms_to_mph = 2.2369362921;
const double mph_to_ms = 0.44704;
double target_speed = 5 * mph_to_ms;
const double m_to_in = 39.37007874;

// output directory
const std::string out_dir = GetChronoOutputPath() + "MROLE_RND_QUALITY";
const std::string pov_dir = out_dir + "/POVRAY";
bool povray_output = false;
bool data_output = true;

std::string path_file("paths/straightOrigin.txt");
std::string steering_controller_file("mtv/SteeringController.json");
std::string speed_controller_file("mtv/SpeedController.json");

std::string output_file_name("quality");

// =============================================================================

int main(int argc, char* argv[]) {
    int terrainCode = 1;
    int tireSelect = 1;
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
        case 4:
            target_speed = atof(argv[1]);
            terrainCode = atoi(argv[2]);
            tireSelect = ChClamp(atoi(argv[3]), 1, 3);
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
    mrole_Full mrole;
    mrole.SetContactMethod(ChContactMethod::NSC);
    mrole.SetChassisFixed(false);
    mrole.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    mrole.SetTireType(tire_model);
    mrole.SetTireStepSize(tire_step_size);
    mrole.SetTireCollisionType(ChTire::CollisionType::ENVELOPE);
    switch (tireSelect) {
        case 1:
            mrole.SelectRoadOperation();
            std::cout << "On-Road Tire Inflation selected." << std::endl;
            break;
        case 2:
            mrole.SelectOffroadSoilOperation();
            std::cout << "Off-Road Tire Inflation for soil selected." << std::endl;
            break;
        case 3:
            mrole.SelectOffroadSandOperation();
            std::cout << "Off-Road Tire Inflation for sand selected." << std::endl;
            break;
    }
    mrole.SetInitFwdVel(target_speed);
    mrole.Initialize();

    mrole.SetChassisVisualizationType(chassis_vis_type);
    mrole.SetSuspensionVisualizationType(suspension_vis_type);
    mrole.SetSteeringVisualizationType(steering_vis_type);
    mrole.SetWheelVisualizationType(wheel_vis_type);
    mrole.SetTireVisualizationType(tire_vis_type);

    std::cout << "Vehicle mass:               " << mrole.GetVehicle().GetVehicleMass() << std::endl;
    std::cout << "Vehicle mass (with tires):  " << mrole.GetTotalMass() << std::endl;

    // ------------------
    // Create the terrain
    // ------------------
    RandomSurfaceTerrain terrain(mrole.GetSystem(), xmax);
    terrain.Initialize(surface, 2, visType);
    GetLog() << "RMS = " << (1000.0 * terrain.GetRMS()) << " mm\n";
    GetLog() << "IRI = " << terrain.GetIRI() << " mm/m\n";

    // create the driver
    auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
    ChPathFollowerDriver driver(mrole.GetVehicle(), vehicle::GetDataFile(steering_controller_file),
                                vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed, false);
    driver.Initialize();

    // ------------------------------
    // measuring device
    chrono::utils::ChISO2631_Vibration_SeatCushionLogger seat_logger(step_size);

    // -------------------------------------
    // Create the vehicle Irrlicht interface
    // Create the driver system
    // -------------------------------------

#ifdef USE_IRRLICHT
    ChWheeledVehicleIrrApp app(&mrole.GetVehicle(), L"MROLE ride & twist test");
    app.SetSkyBox();
    // app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250,
    // 130);
    app.GetSceneManager()->setAmbientLight(irr::video::SColorf(0.1f, 0.1f, 0.1f, 1.0f));
    app.AddTypicalLights(irr::core::vector3df(-50.f, -30.f, 40.f), irr::core::vector3df(10.f, 30.f, 40.f), 50, 50,
                         irr::video::SColorf(0.7f, 0.7f, 0.7f, 1.0f), irr::video::SColorf(0.7f, 0.7f, 0.7f, 1.0f));
    app.SetChaseCamera(trackPoint, 9.0, 0.5);
    /*app.SetTimestep(step_size);*/
    app.AssetBindAll();
    app.AssetUpdateAll();

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = app.GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = app.GetSceneManager()->addSphereSceneNode(0.1f);
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
    for (int i = 0; i < 4; i++) {
        csv << "WheelTorque";
    }
    for (int i = 0; i < 4; i++) {
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

    for (int i = 0; i < 4; i++) {
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

    for (auto& axle : mrole.GetVehicle().GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            csv << wheel->GetPos();
        }
    }

    csv << std::endl;

    // ---------------
    // Simulation loop
    // ---------------

    mrole.GetVehicle().LogSubsystemTypes();

    std::cout << "data at: " << vehicle::GetDataFile(steering_controller_file) << std::endl;
    std::cout << "data at: " << vehicle::GetDataFile(speed_controller_file) << std::endl;
    std::cout << "data at: " << vehicle::GetDataFile(path_file) << std::endl;

    int step_number = 0;
    int render_frame = 0;

    double time = 0;
    double sensor_start_x = 10.0;

#ifdef USE_IRRLICHT
    while (app.GetDevice()->run() && (time < tend) && (mrole.GetVehicle().GetVehiclePos().x() < xmax)) {
#else
    while ((time < tend) && (mrole.GetVehicle().GetVehiclePos().x() < xmax)) {
#endif
        time = mrole.GetSystem()->GetChTime();
        double xpos = mrole.GetVehicle().GetVehiclePos().x();
        if (xpos >= sensor_start_x) {
            double speed = mrole.GetVehicle().GetVehicleSpeed();
            ChVector<> seat_acc = mrole.GetVehicle().GetVehiclePointAcceleration(
                mrole.GetVehicle().GetChassis()->GetLocalDriverCoordsys().pos);
            seat_logger.AddData(speed, seat_acc);
        }

#ifdef USE_IRRLICHT
        // path visualization
        const ChVector<>& pS = driver.GetSteeringController().GetSentinelLocation();
        const ChVector<>& pT = driver.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

        // std::cout<<"Target:\t"<<(irr::f32)pT.x()<<",\t "<<(irr::f32)pT.y()<<",\t "<<(irr::f32)pT.z()<<std::endl;
        // std::cout<<"Vehicle:\t"<<mrole.GetVehicle().GetChassisBody()->GetPos().x()
        //   <<",\t "<<mrole.GetVehicle().GetChassisBody()->GetPos().y()<<",\t "
        //   <<mrole.GetVehicle().GetChassisBody()->GetPos().z()<<std::endl;

        // Render scene
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();
        }

#endif

        if (povray_output && step_number % render_steps == 0) {
            char filename[100];
            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
            utils::WriteShapesPovray(mrole.GetSystem(), filename);
            render_frame++;
        }

        // Driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        mrole.Synchronize(time, driver_inputs, terrain);

#ifdef USE_IRRLICHT
        app.Synchronize("Follower driver", driver_inputs);
#endif

        // Advance simulation for one timestep for all modules

        driver.Advance(step_size);
        terrain.Advance(step_size);
        mrole.Advance(step_size);

#ifdef USE_IRRLICHT
        app.Advance(step_size);
#endif

        if (data_output && step_number % output_steps == 0) {
            // std::cout << time << std::endl;
            csv << time;
            csv << driver_inputs.m_throttle;
            csv << mrole.GetVehicle().GetPowertrain()->GetMotorSpeed();
            csv << mrole.GetVehicle().GetPowertrain()->GetCurrentTransmissionGear();
            for (int axle = 0; axle < 2; axle++) {
                csv << mrole.GetVehicle().GetDriveline()->GetSpindleTorque(axle, LEFT);
                csv << mrole.GetVehicle().GetDriveline()->GetSpindleTorque(axle, RIGHT);
            }
            for (int axle = 0; axle < 2; axle++) {
                csv << mrole.GetVehicle().GetSpindleAngVel(axle, LEFT);
                csv << mrole.GetVehicle().GetSpindleAngVel(axle, RIGHT);
            }
            csv << mrole.GetVehicle().GetVehicleSpeed();
            csv << mrole.GetVehicle().GetVehiclePointAcceleration(
                mrole.GetVehicle().GetChassis()->GetLocalDriverCoordsys().pos);

            csv << mrole.GetVehicle().GetVehiclePointAcceleration(vehCOM);

            for (auto& axle : mrole.GetVehicle().GetAxles()) {
                for (auto& wheel : axle->GetWheels()) {
                    csv << wheel->GetTire()->ReportTireForce(&terrain).force;
                }
            }

            csv << mrole.GetVehicle().GetPowertrain()->GetMotorTorque();

            csv << std::endl;
        }

        // Increment frame number
        step_number++;
    }

    if (data_output) {
        csv.write_to_file(out_dir + "/" + output_file_name + ".dat");
    }

    double ride_limit = 2.0;
    double avg_speed = seat_logger.GetAVGSpeed();
    double cf = seat_logger.GetCrestFactor();
    double awv = seat_logger.GetAW_V();
    double vdv = seat_logger.GetVDV();
    double svdv = seat_logger.GetSeverityVDV();
    double ap = seat_logger.GetAbsorbedPowerVertical();
    double rmsVal = terrain.GetRMS();
    double rmsValIn = rmsVal * m_to_in;
    GetLog() << "Ride Quality Results #1 (ISO 2631-1):\n";
    GetLog() << "  Root Mean Square of the Road = " << rmsVal << " m\n";
    GetLog() << "  Average Speed                = " << avg_speed << " m/s\n";
    GetLog() << "  Weighted Acceleration        = " << awv << " m/s^2\n";
    GetLog() << "  Vibration Dose Value         = " << vdv << " m/s^1.75\n";
    GetLog() << "  Vibration Exposure Time      = " << seat_logger.GetExposureTime() << " s\n";
    GetLog() << "  Crest Factor                 = " << cf << "\n";
    GetLog() << "  VDV based Severity Criterion = " << svdv << "\n";
    if (svdv < 1.75) {
        GetLog() << "\nVDV Severitiy < 1.75: the Weighted Acceleration AWV ist the prefered result\n";
        if (awv <= ride_limit) {
            GetLog() << "  AWV <= " << ride_limit << "m/s^2 (ok)\n";
        } else {
            GetLog() << "  AWV > " << ride_limit << "m/s^2 (above limit)\n";
        }
    } else {
        GetLog() << "\nVDV Severitiy >= 1.75: the Vibration Dose Value VDV ist the prefered result\n";
        if (vdv <= ride_limit) {
            GetLog() << "  VDV <= " << ride_limit << "m/s^1.75 (ok)\n";
        } else {
            GetLog() << "  VDV > " << ride_limit << "m/s^1.75 (above limit)\n";
        }
    }
    GetLog() << "\nRide Quality Results #2 (Absorbed Power Method):\n";
    GetLog() << "  Speed                         = " << avg_speed * ms_to_mph << " mph = " << avg_speed * 3.6
             << " km/h\n";
    GetLog() << "  RMS                           = " << rmsValIn << " in\n";
    GetLog() << "  Absorbed Power                = " << ap << " W\n";

    return 0;
}
