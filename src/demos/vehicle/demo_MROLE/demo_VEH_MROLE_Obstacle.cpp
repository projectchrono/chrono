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
// Demonstration program for Marder vehicle on rigid NRMM trapezoidal obstacle
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/core/ChTimer.h"
#include "chrono/solver/ChSolverBB.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#include "chrono_vehicle/powertrain/ChSimpleCVTPowertrain.h"
#include "chrono_vehicle/terrain/ObsModTerrain.h"
#include "chrono_vehicle/output/ChVehicleOutputASCII.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_models/vehicle/mrole/mrole.h"

#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::vehicle;
using namespace chrono::vehicle::mrole;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================
// Initial vehicle position
ChVector<> initLoc(-1, 0, 0.9);

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

// Type of tire model (RIGID, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// Driver input files
std::string path_file("paths/straightOrigin.txt");
std::string steering_controller_file("hmmwv/SteeringController.json");
std::string speed_controller_file("hmmwv/SpeedController.json");

// Output directories
const std::string out_top_dir = GetChronoOutputPath() + "MROLE";
const std::string out_dir = out_top_dir + "/OBSTACLE";
const std::string pov_dir = out_dir + "/POVRAY";
const std::string img_dir = out_dir + "/IMG";

// Output
bool povray_output = false;
bool img_output = false;
bool dbg_output = false;

// =============================================================================
const double mph_to_ms = 0.44704;

// =============================================================================
int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2021 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChFunction_Recorder accTravel;
    accTravel.AddPoint(1.0, 1.0);
    accTravel.AddPoint(5.0, 10.0);
    accTravel.AddPoint(10.0, 25.0);
    accTravel.AddPoint(15.0, 80.0);
    accTravel.AddPoint(20.0, 300.0);

    double target_speed = 2.0;
    double xpos_max = 100.0;
    initLoc.x() = -accTravel.Get_y(target_speed);

    // --------------------------
    // Construct the Marder vehicle
    // --------------------------

    ChContactMethod contact_method = ChContactMethod::SMC;
    CollisionType chassis_collision_type = CollisionType::NONE;
    DrivelineTypeWV driveline_type = DrivelineTypeWV::AWD;
    BrakeType brake_type = BrakeType::SIMPLE;
    PowertrainModelType powertrain_type = PowertrainModelType::SIMPLE_CVT;

    mrole_Full mrole;
    mrole.SetContactMethod(contact_method);
    mrole.SetBrakeType(brake_type);
    mrole.SetPowertrainType(powertrain_type);
    mrole.SetDriveType(driveline_type);
    mrole.SetChassisCollisionType(chassis_collision_type);
    mrole.SetTireType(tire_model);
    mrole.SetTireCollisionType(ChTire::CollisionType::ENVELOPE);
    mrole.SetInitFwdVel(target_speed * mph_to_ms);

    ////mrole.SetChassisFixed(true);
    ////mrole.CreateTrack(false);

    // Disable gravity in this simulation
    ////mrole.GetSystem()->Set_G_acc(ChVector<>(0, 0, 0));

    // Control steering type (enable crossdrive capability)
    ////mrole.GetDriveline()->SetGyrationMode(true);

    // ------------------------------------------------
    // Initialize the vehicle at the specified position
    // ------------------------------------------------
    mrole.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    mrole.Initialize();

    mrole.SetChassisVisualizationType(VisualizationType::NONE);
    mrole.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    mrole.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    mrole.SetWheelVisualizationType(VisualizationType::NONE);
    mrole.SetTireVisualizationType(VisualizationType::MESH);

    // --------------------------------------------------
    // Control internal collisions and contact monitoring
    // --------------------------------------------------

    // Enable contact on all tracked vehicle parts, except the left sprocket
    ////mrole.GetVehicle().SetCollide(TrackedCollisionFlag::ALL & (~TrackedCollisionFlag::SPROCKET_LEFT));

    // Disable contact for all tracked vehicle parts
    ////mrole.GetVehicle().SetCollide(TrackedCollisionFlag::NONE);

    // Disable all contacts for vehicle chassis (if chassis collision was defined)
    ////mrole.GetVehicle().SetChassisCollide(false);

    // Disable only contact between chassis and track shoes (if chassis collision was defined)
    ////mrole.GetVehicle().SetChassisVehicleCollide(false);

    // Monitor internal contacts for the chassis, left sprocket, left idler, and first shoe on the left track.
    ////mrole.GetVehicle().MonitorContacts(TrackedCollisionFlag::CHASSIS | TrackedCollisionFlag::SPROCKET_LEFT |
    ////                        TrackedCollisionFlag::SHOES_LEFT | TrackedCollisionFlag::IDLER_LEFT);

    // Monitor only contacts involving the chassis.
    // mrole.GetVehicle().MonitorContacts(TrackedCollisionFlag::CHASSIS);

    // Collect contact information.
    // If enabled, number of contacts and local contact point locations are collected for all
    // monitored parts.  Data can be written to a file by invoking ChTrackedVehicle::WriteContacts().
    ////mrole.GetVehicle().SetContactCollection(true);

    // under belly points to estimate vehicle/ground interference
    std::vector<ChVector<double> > bellyPts;
    bellyPts.push_back(ChVector<>(1.6, 0, 1.3 - 0.1));
    bellyPts.push_back(ChVector<>(0.7, 0, -0.1));
    bellyPts.push_back(ChVector<>(0, 0, -0.1));
    bellyPts.push_back(ChVector<>(-1.55, 0, -0.1));
    bellyPts.push_back(ChVector<>((-1.55 - 3.45) / 2, 0, -0.1));
    bellyPts.push_back(ChVector<>(-3.45, 0, -0.1));
    bellyPts.push_back(ChVector<>((-3.45 - 5) / 2, 0, -0.1));
    bellyPts.push_back(ChVector<>(-5, 0, -0.1));
    bellyPts.push_back(ChVector<>(-5.7, 0, -0.1));
    bellyPts.push_back(ChVector<>(-6.0, 0, 0.3 - 0.1));
    std::vector<ChFunction_Recorder> clearance;
    clearance.resize(bellyPts.size());

    // ------------------
    // Create the terrain
    // ------------------

    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.7f;
    minfo.Y = 2e7f;

    // Create the ground
    double base_height = 0.0;
    float friction_coef = 0.8f;
    double aa = 170.0;
    double obl = 5.0;
    double obh = 1.0;
    ObsModTerrain terrain(mrole.GetSystem(), base_height, friction_coef, aa, obl, obh);
    auto terrain_mat = minfo.CreateMaterial(contact_method);
    terrain.EnableCollisionMesh(terrain_mat, std::abs(initLoc.x()) + 5, 0.03);
    terrain.Initialize(ObsModTerrain::VisualisationType::MESH);
    xpos_max = terrain.GetXObstacleEnd() + 7.0;

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    ChWheeledVehicleIrrApp app(&mrole.GetVehicle(), L"Marder Vehicle Ride");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 10.0, 0.5);
    // app.SetChaseCameraPosition(mrole.GetVehicle().GetVehiclePos() + ChVector<>(-10, 0, 0));
    app.SetChaseCameraMultipliers(1e-4, 10);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // Create the driver
    auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
    ChPathFollowerDriver driver(mrole.GetVehicle(), vehicle::GetDataFile(steering_controller_file),
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

    // Generate JSON information with available output channels
    // mrole.GetVehicle().ExportComponentList(out_dir + "/component_list.json");

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
        mrole.GetSystem()->SetSolver(mkl_solver);

        mrole.GetSystem()->SetTimestepperType(ChTimestepper::Type::HHT);
        auto integrator = std::static_pointer_cast<ChTimestepperHHT>(mrole.GetSystem()->GetTimestepper());
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
        mrole.GetSystem()->SetSolver(solver);

        mrole.GetSystem()->SetMaxPenetrationRecoverySpeed(1.5);
        mrole.GetSystem()->SetMinBounceSpeed(2.0);
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;
    int render_frame = 0;

    std::ofstream kurs(out_dir + "/path.txt");

    ChTimer<> timer;
    timer.reset();
    timer.start();
    double sim_time = 0;
    double bail_out_time = 30.0;

    ChRunningAverage avg(100);                    // filter angine torque
    std::vector<double> engineForce;              // store obstacle related tractive force
    double effRadius = 0.328414781 + 0.06 / 2.0;  // sprocket pitch radius + track shoe thickness / 2
    double gear_ratio = 0.05;

    while (app.GetDevice()->run()) {
        if (step_number % render_steps == 0) {
            // Render scene
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteShapesPovray(mrole.GetSystem(), filename);
            }
            if (img_output && step_number > 200) {
                char filename[100];
                sprintf(filename, "%s/img_%03d.jpg", img_dir.c_str(), render_frame + 1);
                app.WriteImageToFile(filename);
            }
            render_frame++;
        }

        double time = mrole.GetVehicle().GetChTime();
        sim_time = time;
        double speed = mrole.GetVehicle().GetVehicleSpeed();
        double xpos = mrole.GetVehicle().GetVehiclePos().x();
        double yerr = mrole.GetVehicle().GetVehiclePos().y();
        kurs << time << "\t" << xpos << "\t" << yerr << "\t" << speed << "\t" << std::endl;
        if (xpos >= -1.0 && xpos <= xpos_max) {
            double eTorque =
                avg.Add(std::static_pointer_cast<ChSimpleCVTPowertrain>(mrole.GetPowertrain())->GetMotorTorque());
            engineForce.push_back(eTorque * effRadius / gear_ratio);
            for (size_t i = 0; i < bellyPts.size(); i++) {
                ChVector<> p = mrole.GetVehicle().GetVehiclePointLocation(bellyPts[i]);
                // GetLog() << "BellyZ(" << i << ")" << p.z() << "\n";
                double t = terrain.GetHeight(ChVector<>(p.x(), p.y(), 0));
                clearance[i].AddPoint(xpos, p.z() - t);
            }
        }
        if (xpos > xpos_max) {
            break;
        }
        if (time > bail_out_time) {
            break;
        }
        driver.SetDesiredSpeed(ChSineStep(time, 1.0, 0.0, 2.0, target_speed));

        // Collect output data from modules
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        mrole.Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);
        app.Synchronize("", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        mrole.Advance(step_size);
        terrain.Advance(step_size);
        app.Advance(step_size);

        xpos = mrole.GetVehicle().GetVehiclePos().x();
        if (xpos >= xpos_max) {
            break;
        }

        // Increment frame number
        step_number++;

        // Spin in place for real time to catch up
        // realtime_timer.Spin(step_size);
    }

    timer.stop();
    kurs.close();

    double clearMin = 99.0;
    for (size_t i = 0; i < clearance.size(); i++) {
        double x1, x2;
        double vmin, vmax;
        clearance[i].Estimate_x_range(x1, x2);
        clearance[i].Estimate_y_range(x1, x2, vmin, vmax, 0);
        GetLog() << "Clearance#" << i << " = " << vmin << "\n";
        if (vmin < clearMin) {
            clearMin = vmin;
        }
    }

    double wallclock_time = timer.GetTimeSeconds();
    GetLog() << "Model time      = " << sim_time << " s\n";
    GetLog() << "Wall clock time = " << wallclock_time << " s\n";

    double fMax = 0.0;
    double fMean = 0.0;
    for (size_t i = 0; i < engineForce.size(); i++) {
        if (engineForce[i] > fMax)
            fMax = engineForce[i];
        fMean += engineForce[i];
    }
    fMean /= double(engineForce.size());
    GetLog() << "Average Tractive Force = " << fMean << " N\n";
    GetLog() << "Max Tractive Force     = " << fMax << " N\n";
    GetLog() << "Min. Clearance         = " << clearMin << " m\n";

    return 0;
}
