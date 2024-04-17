// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
//
// FEDA vehicle steady state cornering test.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/output/ChVehicleOutputASCII.h"

#include "chrono_models/vehicle/feda/FEDA.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/driver/ChInteractiveDriverVSG.h"
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::feda;

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Initial vehicle location and orientation
ChVector3d initLoc(0, 0, 1.6);
ChQuaternion<> initRot(1, 0, 0, 0);
// ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
// ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
// ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
// ChQuaternion<> initRot(0, 0, 0, 1);

enum class DriverMode { DEFAULT, RECORD, PLAYBACK };
DriverMode driver_mode = DriverMode::DEFAULT;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Collision type for chassis (PRIMITIVES, MESH, or NONE)
CollisionType chassis_collision_type = CollisionType::NONE;

// Type of engine model (SHAFTS, SIMPLE, SIMPLE_MAP)
EngineModelType engine_model = EngineModelType::SIMPLE_MAP;

// Type of transmission model (SHAFTS, SIMPLE_MAP)
TransmissionModelType transmission_model = TransmissionModelType::AUTOMATIC_SIMPLE_MAP;

// Drive type (FWD, RWD, or AWD)
DrivelineTypeWV drive_type = DrivelineTypeWV::AWD;

// Steering type (PITMAN_ARM or PITMAN_ARM_SHAFTS)
SteeringTypeWV steering_type = SteeringTypeWV::PITMAN_ARM;

// Brake type (SIMPLE or SHAFTS)
BrakeType brake_type = BrakeType::SHAFTS;

// Model tierods as bodies (true) or as distance constraints (false)
bool use_tierod_bodies = true;

// Type of tire model (RIGID, RIGID_MESH, TMEASY, FIALA, PAC89, PAC02, TMSIMPLE)
TireModelType tire_model = TireModelType::TMSIMPLE;

// Rigid terrain
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;
double terrainHeight = 0;      // terrain height (FLAT terrain only)
double terrainLength = 200.0;  // size in X direction
double terrainWidth = 200.0;   // size in Y direction

// Point on chassis tracked by the camera
ChVector3d trackPoint(0.0, 0.0, 1.75);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;
bool contact_vis = false;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = 1e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Debug logging
bool debug_output = false;
double debug_step_size = 1.0 / 1;  // FPS = 1

// POV-Ray output
bool povray_output = false;

double turn_radius_ref = 30.0;
double turn_radius = turn_radius_ref + 2.5;
double run_in_length = 10.0;  // straight line before entering circle
double manoever_length = 3000.0;
int circle_repeats = manoever_length / (CH_2PI * turn_radius) + 1;

double initial_speed = 2.0;  // start with low speed
double t_hold = 60.0;
double t_acc = 300;
double desired_speed = initial_speed;
double desired_accy = 6.0;  // m/s^2
double max_speed = sqrt(desired_accy * turn_radius);
double delta_vel = step_size * (max_speed - initial_speed) / t_acc;

bool turn_direction_left = true;

// Simulation end time
double t_end = t_hold + t_acc + 60.0;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;
    ChFunctionInterp steeringGear;
    steeringGear.AddPoint(-1.0, -648.0);
    steeringGear.AddPoint(0.0, 0.0);
    steeringGear.AddPoint(1.0, 648.0);

    if (argc == 2) {
        switch (argv[1][0]) {
            case 'l':
            case 'L':
            default:
                turn_direction_left = true;
                std::cout << "Turn left selected\n";
                break;
            case 'r':
            case 'R':
                turn_direction_left = false;
                std::cout << "Turn right selected\n";
                break;
        }
    } else if (argc == 3) {
        switch (argv[1][0]) {
            case 'l':
            case 'L':
            default:
                turn_direction_left = true;
                std::cout << "Turn left selected\n";
                break;
            case 'r':
            case 'R':
                turn_direction_left = false;
                std::cout << "Turn right selected\n";
                break;
        }
        switch (argv[2][0]) {
            case '1':
                tire_model = TireModelType::PAC02;
                break;
            case '2':
                tire_model = TireModelType::TMSIMPLE;
                break;
            case '3':
                tire_model = TireModelType::TMEASY;
                break;
            default:
                std::cout << "Unsupported tire model selected\n";
                std::cout << " 1 : Pac02Tire\n";
                std::cout << " 2 : TMsimple\n";
                std::cout << " 3 : TMeasy\n";
                return 99;
        }
    }

    // --------------
    // Create systems
    // --------------

    // Create the HMMWV vehicle, set parameters, and initialize
    FEDA feda;
    feda.SetContactMethod(contact_method);
    feda.SetChassisCollisionType(chassis_collision_type);
    feda.SetChassisFixed(false);
    feda.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    feda.SetEngineType(engine_model);
    feda.SetTransmissionType(transmission_model);
    feda.SetBrakeType(brake_type);
    feda.SetTireType(tire_model);
    feda.SetTireStepSize(tire_step_size);
    feda.Initialize();

    if (tire_model == TireModelType::RIGID_MESH)
        tire_vis_type = VisualizationType::MESH;

    feda.SetChassisVisualizationType(chassis_vis_type);
    feda.SetSuspensionVisualizationType(suspension_vis_type);
    feda.SetSteeringVisualizationType(steering_vis_type);
    feda.SetWheelVisualizationType(wheel_vis_type);
    feda.SetTireVisualizationType(tire_vis_type);

    // Associate a collision system
    feda.GetSystem()->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the terrain
    RigidTerrain terrain(feda.GetSystem());

    ChContactMaterialData minfo;
    minfo.mu = 0.8f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);

    std::shared_ptr<RigidTerrain::Patch> patch;
    switch (terrain_model) {
        case RigidTerrain::PatchType::BOX:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), 200, 200);
            break;
        case RigidTerrain::PatchType::HEIGHT_MAP:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("terrain/height_maps/test64.bmp"), 128,
                                     128, 0, 4);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16, 16);
            break;
        case RigidTerrain::PatchType::MESH:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("terrain/meshes/test.obj"));
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 100, 100);
            break;
    }
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

    terrain.Initialize();

    // -----------------
    // Initialize output
    // -----------------

    const std::string out_dir = GetChronoOutputPath() + "FEDA";
    const std::string pov_dir = out_dir + "/POVRAY";

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    }

    // Initialize output file for driver inputs
    std::string driver_file = out_dir + "/driver_inputs.txt";
    utils::ChWriterCSV driver_csv(" ");

    std::string ssc_file = out_dir + "/ssc_";
    if (turn_direction_left) {
        ssc_file.append("left_");
    } else {
        ssc_file.append("right_");
    }
    if (tire_model == TireModelType::PAC02) {
        ssc_file.append("pac02");
    }
    if (tire_model == TireModelType::TMSIMPLE) {
        ssc_file.append("tmsimple");
    }
    if (tire_model == TireModelType::TMEASY) {
        ssc_file.append("tmeasy");
    }
    ssc_file.append(".txt");
    utils::ChWriterCSV ssc_csv(" ");

    // Set up vehicle output
    feda.GetVehicle().SetChassisOutput(true);
    feda.GetVehicle().SetSuspensionOutput(0, true);
    feda.GetVehicle().SetSteeringOutput(0, true);
    feda.GetVehicle().SetOutput(ChVehicleOutput::ASCII, out_dir, "output", 0.1);

    // Generate JSON information with available output channels
    feda.GetVehicle().ExportComponentList(out_dir + "/component_list.json");

    // ------------------------------------------------------------------------------
    // Create the vehicle run-time visualization interface and the interactive driver
    // ------------------------------------------------------------------------------

#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    // Create the straight path and the driver system
    auto path = CirclePath(initLoc, turn_radius, run_in_length, turn_direction_left, circle_repeats);
    // auto path = StraightLinePath(ChVector3d(-terrainLength / 2, 0, 0.5), ChVector3d(terrainLength / 2, 0, 0.5), 1);
    ChPathFollowerDriver driver(feda.GetVehicle(), path, "my_path", initial_speed);
    driver.GetSteeringController().SetLookAheadDistance(6.0);
    driver.GetSteeringController().SetGains(0.05, 0.005, 0.0);
    driver.GetSpeedController().SetGains(0.5, 0, 0);
    driver.Initialize();

    std::shared_ptr<ChVehicleVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            // Create the vehicle Irrlicht interface
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle("FEDA Steady State Cornering Demo");
            vis_irr->SetChaseCamera(trackPoint, 7.0, 0.5);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AttachVehicle(&feda.GetVehicle());
            vis_irr->Initialize();
            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            // Create the vehicle VSG interface
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle("FEDA Steady State Cornering Demo");
            vis_vsg->AttachVehicle(&feda.GetVehicle());
            vis_vsg->SetChaseCamera(trackPoint, 7.0, 0.5);
            vis_vsg->SetWindowSize(ChVector2i(800, 600));
            vis_vsg->SetWindowPosition(ChVector2i(100, 300));
            vis_vsg->SetUseSkyBox(true);
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->SetShadows(true);
            vis_vsg->Initialize();
            vis = vis_vsg;
#endif
            break;
        }
    }

    // ---------------
    // Simulation loop
    // ---------------

    feda.GetVehicle().LogSubsystemTypes();

    if (debug_output) {
        std::cout << "\n\n============ System Configuration ============\n";
        feda.LogHardpointLocations();
    }

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);
    int debug_steps = (int)std::ceil(debug_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;
    int render_frame = 0;

    feda.GetVehicle().EnableRealtime(true);

    double real_speed = feda.GetVehicle().GetChassis()->GetSpeed();
    double real_accy1 = feda.GetVehicle().GetPointAcceleration(ChVector3d(0, 0, 0)).y();
    double real_accy2 = pow(real_speed, 2) / turn_radius;

    driver.SetDesiredSpeed(initial_speed);  // hold speed until steady state reached on the turn circle

    while (true) {
        double time = feda.GetSystem()->GetChTime();
        real_speed = feda.GetVehicle().GetSpeed();
        real_accy1 = feda.GetVehicle().GetPointAcceleration(ChVector3d(0, 0, 0)).y();
        real_accy2 = pow(real_speed, 2) / turn_radius_ref;
        double real_throttle = driver.GetThrottle();

        if (time >= t_hold)
            ssc_csv << time << steeringGear.GetVal(driver.GetSteering()) << real_speed << real_accy2 << std::endl;

        if (time > t_hold) {
            // Increase speed
            desired_speed += delta_vel;
            driver.SetDesiredSpeed(desired_speed);
        }

        // Engine Power Limit reached
        if (time > t_hold && real_throttle == 1.0) {
            std::cout << "Manoever ended because engine power limit is reached.\n";
            break;
        }

        // End simulation
        if (time >= t_end) {
            std::cout << "Manoever ended because max. time " << t_end << " s is reached.\n";
            break;
        }

        if (desired_speed > max_speed) {
            std::cout << "Manoever ended because max. speed " << max_speed << " m/s is reached.\n";
            break;
        }

        // Render scene and output POV-Ray data
        if (vis) {
            if (!vis->Run())
                break;

            if (step_number % render_steps == 0) {
                vis->BeginScene();
                vis->Render();
                vis->EndScene();

                if (povray_output) {
                    // Zero-pad frame numbers in file names for postprocessing
                    std::ostringstream filename;
                    filename << pov_dir << "/data_" << std::setw(4) << std::setfill('0') << render_frame + 1 << ".dat";
                    utils::WriteVisualizationAssets(feda.GetSystem(), filename.str());
                }

                render_frame++;
            }
        }

        // Debug logging
        if (debug_output && step_number % debug_steps == 0) {
            std::cout << "\n\n============ System Information ============\n";
            std::cout << "Time = " << time << "\n\n";
            feda.DebugLog(OUT_SPRINGS | OUT_SHOCKS | OUT_CONSTRAINTS);

            auto marker_driver = feda.GetChassis()->GetMarkers()[0]->GetAbsCoordsys().pos;
            auto marker_com = feda.GetChassis()->GetMarkers()[1]->GetAbsCoordsys().pos;
            std::cout << "Markers\n";
            std::cout << "  Driver loc:      " << marker_driver.x() << " " << marker_driver.y() << " "
                      << marker_driver.z() << std::endl;
            std::cout << "  Chassis COM loc: " << marker_com.x() << " " << marker_com.y() << " " << marker_com.z()
                      << std::endl;
        }

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        feda.Synchronize(time, driver_inputs, terrain);
        if (vis)
            vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        feda.Advance(step_size);
        if (vis)
            vis->Advance(step_size);

        // Increment frame number
        step_number++;
    }

    ssc_csv.WriteToFile(ssc_file);

    // data at manoever end
    double vel_kmh = real_speed * 3.6;
    double vel_mph = vel_kmh / 1.602;
    std::cout << "Reached vehicle speed = " << vel_kmh << " km/h (" << vel_mph << " mph)\n";
    std::cout << "Reached lateral acceleration = " << real_accy2 / 9.81 << " g\n";
    return 0;
}
