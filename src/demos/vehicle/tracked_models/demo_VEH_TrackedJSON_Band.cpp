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
// Authors: Radu Serban
// =============================================================================
//
// Main driver function for a tracked vehicle with continuous-band track,
// specified through JSON files.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBand.h"

#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"

#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_MUMPS
#include "chrono_mumps/ChSolverMumps.h"
#endif

#ifdef CHRONO_PARDISO_MKL
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#define USE_IRRLICHT

using namespace chrono;
using namespace chrono::vehicle;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================
// JSON file for vehicle model
std::string vehicle_file("M113/vehicle/M113_Vehicle_BandBushing.json");
////std::string vehicle_file("M113/vehicle/M113_Vehicle_BandANCF.json");

// JSON files for powertrain
std::string engine_file("M113/powertrain/M113_EngineSimple.json");
std::string transmission_file("M113/powertrain/M113_EngineSimpleMap.json");

// Initial vehicle position
ChVector<> initLoc(0, 0, 1.1);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);

// JSON files for terrain (rigid plane)
std::string rigidterrain_file("terrain/RigidPlane.json");

// Simulation length
double t_end = 1.0;

// Simulation step size
double step_size = 1e-4;

// Linear solver (MUMPS or PARDISO_MKL)
ChSolver::Type solver_type = ChSolver::Type::MUMPS;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Output directories
const std::string out_dir = GetChronoOutputPath() + "M113_BAND_JSON";
const std::string pov_dir = out_dir + "/POVRAY";
const std::string img_dir = out_dir + "/IMG";

// Verbose level
bool verbose_solver = false;
bool verbose_integrator = false;

// Output
bool output = true;
bool dbg_output = false;
bool povray_output = false;
bool img_output = false;

// =============================================================================

// Dummy driver class (always returns 1 for throttle, 0 for all other inputs)
class MyDriver {
  public:
    MyDriver() {}
    double GetThrottle() const { return 1; }
    double GetSteering() const { return 0; }
    double GetBraking() const { return 0; }
    void Synchronize(double time) {}
    void Advance(double step) {}
};

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------------------
    // Construct the Tracked vehicle
    // -----------------------------

    // Create the vehicle system
    TrackedVehicle vehicle(vehicle::GetDataFile(vehicle_file), ChContactMethod::SMC);

    // Disable gravity in this simulation
    ////vehicle.GetSystem()->Set_G_acc(ChVector<>(0, 0, 0));

    // Control steering type (enable crossdrive capability).
    ////vehicle.GetDriveline()->SetGyrationMode(true);

    // ------------------------------------------------
    // Initialize the vehicle at the specified position
    // ------------------------------------------------

    vehicle.Initialize(ChCoordsys<>(initLoc, initRot));

    // Set visualization type for vehicle components.
    vehicle.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetIdlerWheelVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    // --------------------------------------------------
    // Control internal collisions and contact monitoring
    // --------------------------------------------------

    // Enable contact on all tracked vehicle parts, except the left sprocket
    ////vehicle.SetCollide(TrackedCollisionFlag::ALL & (~TrackedCollisionFlag::SPROCKET_LEFT));

    // Disable contact for all tracked vehicle parts
    ////vehicle.SetCollide(TrackedCollisionFlag::NONE);

    // Disable all contacts for vehicle chassis (if chassis collision was defined)
    ////vehicle.SetChassisCollide(false);

    // Disable only contact between chassis and track shoes (if chassis collision was defined)
    ////vehicle.SetChassisVehicleCollide(false);

    // Monitor internal contacts for the chassis, left sprocket, left idler, and first shoe on the left track.
    ////vehicle.MonitorContacts(TrackedCollisionFlag::CHASSIS | TrackedCollisionFlag::SPROCKET_LEFT |
    ////                        TrackedCollisionFlag::SHOES_LEFT | TrackedCollisionFlag::IDLER_LEFT);

    // Monitor only contacts involving the chassis.
    vehicle.MonitorContacts(TrackedCollisionFlag::CHASSIS);

    // Collect contact information.
    // If enabled, number of contacts and local contact point locations are collected for all
    // monitored parts.  Data can be written to a file by invoking ChTrackedVehicle::WriteContacts().
    ////vehicle.SetContactCollection(true);

    // ------------------
    // Create the terrain
    // ------------------

    RigidTerrain terrain(vehicle.GetSystem(), vehicle::GetDataFile(rigidterrain_file));
    terrain.Initialize();

    // ----------------------------
    // Create the powertrain system
    // ----------------------------

    auto engine = ReadEngineJSON(vehicle::GetDataFile(engine_file));
    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(transmission_file));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);

#ifdef USE_IRRLICHT
    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    auto vis = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("JSON Band-Tracked Vehicle Demo");
    vis->SetChaseCamera(ChVector<>(0, 0, 0), 6.0, 0.5);
    ////vis->SetChaseCameraPosition(vehicle.GetPos() + ChVector<>(0, 2, 0));
    vis->SetChaseCameraMultipliers(1e-4, 10);
    vis->Initialize();
    vis->AddLightDirectional();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&vehicle);

    // ------------------------
    // Create the driver system
    // ------------------------

    ChInteractiveDriverIRR driver(*vis);

    // Set the time response for keyboard inputs.
    double steering_time = 0.5;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    driver.Initialize();
#else

    // Create a default driver (always returns 1 for throttle, 0 for all other inputs)
    MyDriver driver;

#endif

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }

    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            cout << "Error creating directory " << pov_dir << endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    }

    if (img_output) {
        if (!filesystem::create_directory(filesystem::path(img_dir))) {
            cout << "Error creating directory " << img_dir << endl;
            return 1;
        }
    }

    //Setup chassis position output with column headers
    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);
    csv << "Time (s)" << "Chassis X Pos (m)" << "Chassis Y Pos (m)" << "Chassis Z Pos (m)" << endl;

    // Set up vehicle output
    ////vehicle.SetChassisOutput(true);
    ////vehicle.SetTrackAssemblyOutput(VehicleSide::LEFT, true);
    vehicle.SetOutput(ChVehicleOutput::ASCII, out_dir, "output", 0.1);

    // Generate JSON information with available output channels
    ////vehicle.ExportComponentList(out_dir + "/component_list.json");

    // Export visualization mesh for shoe tread body
    auto shoe0 = std::static_pointer_cast<ChTrackShoeBand>(vehicle.GetTrackShoe(LEFT, 0));
    shoe0->WriteTreadVisualizationMesh(out_dir);
    shoe0->ExportTreadVisualizationMeshPovray(out_dir);

    // ------------------------------
    // Solver and integrator settings
    // ------------------------------

#ifndef CHRONO_PARDISO_MKL
    if (solver_type == ChSolver::Type::PARDISO_MKL)
        solver_type = ChSolver::Type::MUMPS;
#endif
#ifndef CHRONO_MUMPS
    if (solver_type == ChSolver::Type::MUMPS)
        solver_type = ChSolver::Type::PARDISO_MKL;
#endif

    switch (solver_type) {
        default:
            break;
#ifdef CHRONO_MUMPS
        case ChSolver::Type::MUMPS: {
            auto mumps_solver = chrono_types::make_shared<ChSolverMumps>();
            mumps_solver->LockSparsityPattern(true);
            mumps_solver->SetVerbose(verbose_solver);
            vehicle.GetSystem()->SetSolver(mumps_solver);
            break;
        }
#endif
#ifdef CHRONO_PARDISO_MKL
        case ChSolver::Type::PARDISO_MKL: {
            auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
            mkl_solver->LockSparsityPattern(true);
            mkl_solver->SetVerbose(verbose_solver);
            vehicle.GetSystem()->SetSolver(mkl_solver);
            break;
        }
#endif
    }

    vehicle.GetSystem()->SetTimestepperType(ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(vehicle.GetSystem()->GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(50);
    integrator->SetAbsTolerances(1e-2, 1e2);
    integrator->SetMode(ChTimestepperHHT::ACCELERATION);
    integrator->SetStepControl(false);
    integrator->SetModifiedNewton(true);
    integrator->SetScaling(true);
    integrator->SetVerbose(verbose_integrator);

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    BodyStates shoe_states_left(vehicle.GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(vehicle.GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left(vehicle.GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(vehicle.GetNumTrackShoes(RIGHT));

    // Number of steps to run for the simulation
    int sim_steps = (int)std::ceil(t_end / step_size);

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Total execution time (for integration)
    double total_timing = 0;

    // Initialize simulation frame counter
    int step_number = 0;
    int render_frame = 0;

    while (step_number < sim_steps) {
        const ChVector<>& c_pos = vehicle.GetPos();

        // File output
        if (output) {
            csv << vehicle.GetSystem()->GetChTime() << c_pos.x() << c_pos.y() << c_pos.z() << endl;
        }

        // Debugging output
        if (dbg_output) {
            cout << "Time: " << vehicle.GetSystem()->GetChTime() << endl;
            const ChFrameMoving<>& c_ref = vehicle.GetChassisBody()->GetFrame_REF_to_abs();
            cout << "      chassis:    " << c_pos.x() << "  " << c_pos.y() << "  " << c_pos.z() << endl;
            {
                const ChVector<>& i_pos_abs = vehicle.GetTrackAssembly(LEFT)->GetIdler()->GetWheelBody()->GetPos();
                const ChVector<>& s_pos_abs = vehicle.GetTrackAssembly(LEFT)->GetSprocket()->GetGearBody()->GetPos();
                ChVector<> i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
                ChVector<> s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
                cout << "      L idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
                cout << "      L sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
            }
            {
                const ChVector<>& i_pos_abs = vehicle.GetTrackAssembly(RIGHT)->GetIdler()->GetWheelBody()->GetPos();
                const ChVector<>& s_pos_abs = vehicle.GetTrackAssembly(RIGHT)->GetSprocket()->GetGearBody()->GetPos();
                ChVector<> i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
                ChVector<> s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
                cout << "      R idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
                cout << "      R sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
            }
            cout << "      L suspensions (arm angles):" << endl;
            for (size_t i = 0; i < vehicle.GetTrackAssembly(LEFT)->GetNumTrackSuspensions(); i++) {
                cout << " " << vehicle.GetTrackAssembly(VehicleSide::LEFT)->GetTrackSuspension(i)->GetCarrierAngle();
            }
            cout << endl;
            cout << "      R suspensions (arm angles):" << endl;
            for (size_t i = 0; i < vehicle.GetTrackAssembly(RIGHT)->GetNumTrackSuspensions(); i++) {
                cout << " " << vehicle.GetTrackAssembly(VehicleSide::RIGHT)->GetTrackSuspension(i)->GetCarrierAngle();
            }
            cout << endl;
        }

#ifdef USE_IRRLICHT
        if (!vis->Run())
            break;

        // Render scene
        vis->BeginScene();
        vis->Render();
#endif

        if (step_number % render_steps == 0) {
            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(vehicle.GetSystem(), filename);
            }

#ifdef USE_IRRLICHT
            if (img_output && step_number > 200) {
                char filename[100];
                sprintf(filename, "%s/img_%03d.jpg", img_dir.c_str(), render_frame + 1);
                vis->WriteImageToFile(filename);
            }
#endif
            render_frame++;
        }

        // Collect data from modules
        DriverInputs driver_inputs = driver.GetInputs();
        vehicle.GetTrackShoeStates(LEFT, shoe_states_left);
        vehicle.GetTrackShoeStates(RIGHT, shoe_states_right);

        // Update modules (process data from other modules)
        double time = vehicle.GetChTime();
        driver.Synchronize(time);
        terrain.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
#ifdef USE_IRRLICHT
        vis->Synchronize(time, driver_inputs);
#endif

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);
#ifdef USE_IRRLICHT
        vis->Advance(step_size);
#endif

        // Report if the chassis experienced a collision
        if (vehicle.IsPartInContact(TrackedCollisionFlag::CHASSIS)) {
            cout << time << "  chassis contact" << endl;
        }

        // Increment frame number
        step_number++;

        double step_timing = vehicle.GetSystem()->GetTimerStep();
        total_timing += step_timing;

        cout << "Step: " << step_number;
        cout << "   Time: " << time;
        cout << "   Number of Iterations: " << integrator->GetNumIterations();
        cout << "   Step Time: " << step_timing;
        cout << "   Total Time: " << total_timing;
        cout << endl;

#ifdef USE_IRRLICHT
        vis->EndScene();
#endif
    }

    if (output) {
        csv.write_to_file(out_dir + "/chassis_position.txt");
    }

    vehicle.WriteContacts(out_dir + "/contacts.txt");

    return 0;
}
