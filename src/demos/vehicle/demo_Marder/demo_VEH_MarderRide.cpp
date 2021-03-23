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
// Authors: Radu Serban / Rainer Gericke
// =============================================================================
//
// Demonstration program for Marder vehicle on rigid terrain.
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/core/ChTimer.h"
#include "chrono/solver/ChSolverBB.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#include "chrono_vehicle/terrain/RandomSurfaceTerrain.h"
#include "chrono_vehicle/output/ChVehicleOutputASCII.h"

#include "chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleIrrApp.h"

#include "chrono_models/vehicle/marder/Marder.h"

#ifdef CHRONO_PARDISO_MKL
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::vehicle;
using namespace chrono::vehicle::marder;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================
// Initial vehicle position
ChVector<> initLoc(0, 0, 0.9);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);
// ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
// ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
// ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
// ChQuaternion<> initRot(0, 0, 0, 1);

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Simulation step size
double step_size = 5e-4;

// Use HHT + MKL
bool use_mkl = false;

// Time interval between two render frames
double render_step_size = 1.0 / 120;  // FPS = 120

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 2.0, 0.0);

// Driver input files
std::string path_file("paths/straightOrigin.txt");
std::string steering_controller_file("marder/driver/SteeringController.json");
std::string speed_controller_file("marder/driver/SpeedController.json");

// Output directories
const std::string out_dir = GetChronoOutputPath() + "Marder";
const std::string pov_dir = out_dir + "/POVRAY";
const std::string img_dir = out_dir + "/IMG";

// Output
bool povray_output = false;
bool img_output = false;
bool dbg_output = false;

// =============================================================================

// =============================================================================
int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2021 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    const double MetersToInch = 1.0 / 0.0254;
    const double MetersPerSecToMph = 2.2369362921;

    ChFunction_Recorder zeroLoad;
    zeroLoad.AddPoint(0, 0);
    zeroLoad.AddPoint(5.0, 2.80372);
    zeroLoad.AddPoint(10.0, 3.8346);
    zeroLoad.AddPoint(15.0, 4.78549);

    ChFunction_Recorder accTravel;
    accTravel.AddPoint(1.0, 10.0);
    accTravel.AddPoint(5.0, 10.0);
    accTravel.AddPoint(10.0, 25.0);
    accTravel.AddPoint(15.0, 80.0);
    accTravel.AddPoint(20.0, 300.0);

    int iTerrain = 2;
    if (argc != 3) {
        GetLog() << "usage: prog TerrainNumber Speed\n";
        return 1;
    }
    iTerrain = atoi(argv[1]);

    double target_speed = atof(argv[2]);
    double xpos_max = 100.0;
    initLoc.x() = -accTravel.Get_y(target_speed);
    // --------------------------
    // Construct the M113 vehicle
    // --------------------------

    ChContactMethod contact_method = ChContactMethod::SMC;
    CollisionType chassis_collision_type = CollisionType::NONE;
    TrackShoeType shoe_type = TrackShoeType::SINGLE_PIN;
    DrivelineTypeTV driveline_type = DrivelineTypeTV::SIMPLE;
    BrakeType brake_type = BrakeType::SIMPLE;
    PowertrainModelType powertrain_type = PowertrainModelType::SIMPLE_CVT;

    //// TODO
    //// When using SMC, a double-pin shoe type requires MKL or MUMPS.
    //// However, there appear to still be redundant constraints in the double-pin assembly
    //// resulting in solver failures with MKL and MUMPS (rank-deficient matrix).
    if (shoe_type == TrackShoeType::DOUBLE_PIN)
        contact_method = ChContactMethod::NSC;

    Marder marder;
    marder.SetContactMethod(contact_method);
    marder.SetTrackShoeType(shoe_type);
    marder.SetDrivelineType(driveline_type);
    marder.SetBrakeType(brake_type);
    marder.SetPowertrainType(powertrain_type);
    marder.SetChassisCollisionType(chassis_collision_type);

    ////marder.SetChassisFixed(true);
    ////marder.CreateTrack(false);

    // Disable gravity in this simulation
    ////marder.GetSystem()->Set_G_acc(ChVector<>(0, 0, 0));

    // Control steering type (enable crossdrive capability)
    ////marder.GetDriveline()->SetGyrationMode(true);

    // ------------------------------------------------
    // Initialize the vehicle at the specified position
    // ------------------------------------------------
    marder.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    marder.Initialize();

    // Set visualization type for vehicle components.
    VisualizationType track_vis =
        (shoe_type == TrackShoeType::SINGLE_PIN) ? VisualizationType::MESH : VisualizationType::PRIMITIVES;
    marder.SetChassisVisualizationType(VisualizationType::MESH);
    marder.SetSprocketVisualizationType(track_vis);
    marder.SetIdlerVisualizationType(track_vis);
    marder.SetRollerVisualizationType(track_vis);
    marder.SetRoadWheelAssemblyVisualizationType(track_vis);
    marder.SetRoadWheelVisualizationType(track_vis);
    marder.SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    // --------------------------------------------------
    // Control internal collisions and contact monitoring
    // --------------------------------------------------

    // Enable contact on all tracked vehicle parts, except the left sprocket
    ////marder.GetVehicle().SetCollide(TrackedCollisionFlag::ALL & (~TrackedCollisionFlag::SPROCKET_LEFT));

    // Disable contact for all tracked vehicle parts
    ////marder.GetVehicle().SetCollide(TrackedCollisionFlag::NONE);

    // Disable all contacts for vehicle chassis (if chassis collision was defined)
    ////marder.GetVehicle().SetChassisCollide(false);

    // Disable only contact between chassis and track shoes (if chassis collision was defined)
    ////marder.GetVehicle().SetChassisVehicleCollide(false);

    // Monitor internal contacts for the chassis, left sprocket, left idler, and first shoe on the left track.
    ////marder.GetVehicle().MonitorContacts(TrackedCollisionFlag::CHASSIS | TrackedCollisionFlag::SPROCKET_LEFT |
    ////                        TrackedCollisionFlag::SHOES_LEFT | TrackedCollisionFlag::IDLER_LEFT);

    // Monitor only contacts involving the chassis.
    marder.GetVehicle().MonitorContacts(TrackedCollisionFlag::CHASSIS);

    // Collect contact information.
    // If enabled, number of contacts and local contact point locations are collected for all
    // monitored parts.  Data can be written to a file by invoking ChTrackedVehicle::WriteContacts().
    ////marder.GetVehicle().SetContactCollection(true);

    // ------------------
    // Create the terrain
    // ------------------

    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.7f;
    minfo.Y = 2e7f;

    // Create the ground
    RandomSurfaceTerrain terrain(marder.GetSystem(), xpos_max);
    double track_width = 2.72;
    auto terrain_mat = minfo.CreateMaterial(contact_method);
    terrain.EnableCollisionMesh(terrain_mat, std::abs(initLoc.x()) + 5, 0.06);
    switch (iTerrain) {
        default:
        case 1:
            terrain.Initialize(RandomSurfaceTerrain::SurfaceType::ISO8608_A_CORR, track_width);
            break;
        case 2:
            terrain.Initialize(RandomSurfaceTerrain::SurfaceType::ISO8608_B_CORR, track_width);
            break;
        case 3:
            terrain.Initialize(RandomSurfaceTerrain::SurfaceType::ISO8608_C_CORR, track_width);
            break;
        case 4:
            terrain.Initialize(RandomSurfaceTerrain::SurfaceType::ISO8608_D_CORR, track_width);
            break;
        case 5:
            terrain.Initialize(RandomSurfaceTerrain::SurfaceType::ISO8608_E_CORR, track_width);
            break;
        case 6:
            terrain.Initialize(RandomSurfaceTerrain::SurfaceType::ISO8608_F_NOCORR, track_width);
            break;
        case 7:
            terrain.Initialize(RandomSurfaceTerrain::SurfaceType::ISO8608_G_NOCORR, track_width);
            break;
        case 8:
            terrain.Initialize(RandomSurfaceTerrain::SurfaceType::ISO8608_H_NOCORR, track_width);
            break;
    }

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    ChTrackedVehicleIrrApp app(&marder.GetVehicle(), L"Marder Vehicle Ride");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 10.0, 0.5);
    // app.SetChaseCameraPosition(marder.GetVehicle().GetVehiclePos() + ChVector<>(-10, 0, 0));
    app.SetChaseCameraMultipliers(1e-4, 10);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // Create the driver
    auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
    ChPathFollowerDriver driver(marder.GetVehicle(), vehicle::GetDataFile(steering_controller_file),
                                vehicle::GetDataFile(speed_controller_file), path, "my_path", 0.0, false);
    driver.Initialize();

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    if (img_output) {
        if (!filesystem::create_directory(filesystem::path(img_dir))) {
            std::cout << "Error creating directory " << img_dir << std::endl;
            return 1;
        }
    }

    // Set up vehicle output
    marder.GetVehicle().SetChassisOutput(true);
    marder.GetVehicle().SetTrackAssemblyOutput(VehicleSide::LEFT, true);
    marder.GetVehicle().SetOutput(ChVehicleOutput::ASCII, out_dir, "output", 0.1);

    // Generate JSON information with available output channels
    marder.GetVehicle().ExportComponentList(out_dir + "/component_list.json");

    // ------------------------------
    // Solver and integrator settings
    // ------------------------------

    // Cannot use HHT + MKL with NSC contact
    if (contact_method == ChContactMethod::NSC) {
        use_mkl = false;
    }

#ifndef CHRONO_PARDISO_MKL
    // Cannot use HHT + PardisoMKL if Chrono::PardisoMKL not available
    use_mkl = false;
#endif

    if (use_mkl) {
#ifdef CHRONO_PARDISO_MKL
        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        marder.GetSystem()->SetSolver(mkl_solver);

        marder.GetSystem()->SetTimestepperType(ChTimestepper::Type::HHT);
        auto integrator = std::static_pointer_cast<ChTimestepperHHT>(marder.GetSystem()->GetTimestepper());
        integrator->SetAlpha(-0.2);
        integrator->SetMaxiters(50);
        integrator->SetAbsTolerances(1e-4, 1e2);
        integrator->SetMode(ChTimestepperHHT::ACCELERATION);
        integrator->SetStepControl(false);
        integrator->SetModifiedNewton(false);
        integrator->SetScaling(true);
        integrator->SetVerbose(true);
#endif
    } else {
        auto solver = chrono_types::make_shared<ChSolverBB>();
        solver->SetMaxIterations(120);
        solver->SetOmega(0.8);
        solver->SetSharpnessLambda(1.0);
        marder.GetSystem()->SetSolver(solver);

        marder.GetSystem()->SetMaxPenetrationRecoverySpeed(1.5);
        marder.GetSystem()->SetMinBounceSpeed(2.0);
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    BodyStates shoe_states_left(marder.GetVehicle().GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(marder.GetVehicle().GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left(marder.GetVehicle().GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(marder.GetVehicle().GetNumTrackShoes(RIGHT));

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;
    int render_frame = 0;

    std::ofstream kurs("kurs.txt");

    ChFunction_Recorder accLogger;
    utils::ChButterworth_Lowpass lp(4, step_size, 30.0);
    utils::ChRunningAverage avg(50);

    ChISO2631_Vibration_SeatCushionLogger seat_logger(step_size);
    ChTimer<> timer;
    timer.reset();
    timer.start();
    while (app.GetDevice()->run()) {
        // Debugging output
        if (dbg_output) {
            auto track_L = marder.GetVehicle().GetTrackAssembly(LEFT);
            auto track_R = marder.GetVehicle().GetTrackAssembly(RIGHT);
            cout << "Time: " << marder.GetSystem()->GetChTime() << endl;
            cout << "      Num. contacts: " << marder.GetSystem()->GetNcontacts() << endl;
            const ChFrameMoving<>& c_ref = marder.GetChassisBody()->GetFrame_REF_to_abs();
            const ChVector<>& c_pos = marder.GetVehicle().GetVehiclePos();
            cout << "      chassis:    " << c_pos.x() << "  " << c_pos.y() << "  " << c_pos.z() << endl;
            {
                const ChVector<>& i_pos_abs = track_L->GetIdler()->GetWheelBody()->GetPos();
                const ChVector<>& s_pos_abs = track_L->GetSprocket()->GetGearBody()->GetPos();
                ChVector<> i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
                ChVector<> s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
                cout << "      L idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
                cout << "      L sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
            }
            {
                const ChVector<>& i_pos_abs = track_R->GetIdler()->GetWheelBody()->GetPos();
                const ChVector<>& s_pos_abs = track_R->GetSprocket()->GetGearBody()->GetPos();
                ChVector<> i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
                ChVector<> s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
                cout << "      R idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
                cout << "      R sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
            }
            cout << "      L suspensions (arm angles):";
            for (size_t i = 0; i < track_L->GetNumRoadWheelAssemblies(); i++) {
                cout << " " << track_L->GetRoadWheelAssembly(i)->GetCarrierAngle();
            }
            cout << endl;
            cout << "      R suspensions (arm angles):";
            for (size_t i = 0; i < track_R->GetNumRoadWheelAssemblies(); i++) {
                cout << " " << track_R->GetRoadWheelAssembly(i)->GetCarrierAngle();
            }
            cout << endl;
        }

        if (step_number % render_steps == 0) {
            // Render scene
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteShapesPovray(marder.GetSystem(), filename);
            }
            if (img_output && step_number > 200) {
                char filename[100];
                sprintf(filename, "%s/img_%03d.jpg", img_dir.c_str(), render_frame + 1);
                app.WriteImageToFile(filename);
            }
            render_frame++;
        }

        double time = marder.GetVehicle().GetChTime();
        double speed = avg.Add(marder.GetVehicle().GetVehicleSpeed());
        double xpos = marder.GetVehicle().GetVehiclePos().x();
        double yerr = marder.GetVehicle().GetVehiclePos().y();
        kurs << time << "\t" << xpos << "\t" << yerr << "\t" << speed << "\t" << std::endl;
        if (xpos >= 0.0) {
            ChVector<double> acc =
                marder.GetVehicle().GetVehiclePointAcceleration(marder.GetChassis()->GetLocalDriverCoordsys().pos);
            seat_logger.AddData(speed, acc);
        }
        if (xpos > xpos_max)
            break;
        driver.SetDesiredSpeed(ChSineStep(time, 1.0, 0.0, 2.0, target_speed));
        // Collect output data from modules
        ChDriver::Inputs driver_inputs = driver.GetInputs();
        marder.GetVehicle().GetTrackShoeStates(LEFT, shoe_states_left);
        marder.GetVehicle().GetTrackShoeStates(RIGHT, shoe_states_right);

        // Update modules (process inputs from other modules)

        driver.Synchronize(time);
        terrain.Synchronize(time);
        marder.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
        app.Synchronize("", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        marder.Advance(step_size);
        app.Advance(step_size);

        // Report if the chassis experienced a collision
        if (marder.GetVehicle().IsPartInContact(TrackedCollisionFlag::CHASSIS)) {
            std::cout << time << "  chassis contact" << std::endl;
        }

        // Increment frame number
        step_number++;

        // Spin in place for real time to catch up
        // realtime_timer.Spin(step_size);
    }

    timer.stop();
    kurs.close();
    marder.GetVehicle().WriteContacts("Marder_contacts.out");

    double awv = seat_logger.GetAW_V();
    double ap = seat_logger.GetAbsorbedPowerVertical();
    double vel_avg = seat_logger.GetAVGSpeed();
    double tex = seat_logger.GetExposureTime();
    double rms = terrain.GetRMS();
    double wallclock_time = timer.GetTimeSeconds();
    GetLog() << "Roughness       = " << rms << " m\n";
    GetLog() << "Avg speed       = " << vel_avg << " m/s\n";
    GetLog() << "Awv             = " << awv << " m/s\n";
    GetLog() << "Absorbed Power  = " << ap << " W\n";
    GetLog() << "Exposure Time   = " << tex << " s\n";
    GetLog() << "Wall clock time = " << wallclock_time << " s\n";
    GetLog() << "\nNRMM Formatted Results:\n";
    GetLog() << "Roughness       = " << rms * MetersToInch << " in\n";
    GetLog() << "Avg speed       = " << vel_avg * MetersPerSecToMph << " mph\n";
    GetLog() << "Absorbed Power  = " << (ap - zeroLoad.Get_y(vel_avg)) << " W\n";

    return 0;
}
