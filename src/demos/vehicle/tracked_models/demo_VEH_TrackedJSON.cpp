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
// Main driver function for a tracked vehicle specified through JSON files.
//
// If using the Irrlicht interface, driver inputs are obtained from the keyboard.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <vector>

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono/solver/ChDirectSolverLS.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeDoublePin.h"
#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"

#include "chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleVisualSystemIrrlicht.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#ifdef CHRONO_MUMPS
    #include "chrono_mumps/ChSolverMumps.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

TrackShoeType shoe_type = TrackShoeType::SINGLE_PIN;
DrivelineTypeTV driveline_type = DrivelineTypeTV::SIMPLE;
PowertrainModelType powertrain_type = PowertrainModelType::SIMPLE_CVT;

// Initial vehicle position
ChVector<> initLoc(0, 0, 0.8);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);
////ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
////ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
////ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);

// JSON files for terrain (rigid plane)
std::string rigidterrain_file("terrain/RigidPlane.json");

// Specification of vehicle inputs
enum class DriverMode {
    KEYBOARD,  // interactive (Irrlicht) driver
    DATAFILE,  // inputs from data file
    PATH       // drives in a straight line
};
std::string driver_file("M113/driver/Acceleration2.txt");  // used for mode=DATAFILE
double target_speed = 2;                                   // used for mode=PATH

DriverMode driver_mode = DriverMode::DATAFILE;

// Contact formulation (NSC or SMC)
ChContactMethod contact_method = ChContactMethod::SMC;

// Simulation step size
double step_size_NSC = 1e-3;
double step_size_SMC = 5e-4;

// Solver and integrator types
////ChSolver::Type slvr_type = ChSolver::Type::BARZILAIBORWEIN;
////ChSolver::Type slvr_type = ChSolver::Type::PSOR;
////ChSolver::Type slvr_type = ChSolver::Type::MINRES;
////ChSolver::Type slvr_type = ChSolver::Type::GMRES;
////ChSolver::Type slvr_type = ChSolver::Type::SPARSE_LU;
////ChSolver::Type slvr_type = ChSolver::Type::SPARSE_QR;
ChSolver::Type slvr_type = ChSolver::Type::PARDISO_MKL;
////ChSolver::Type slvr_type = ChSolver::Type::MUMPS;

////ChTimestepper::Type intgr_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
ChTimestepper::Type intgr_type = ChTimestepper::Type::EULER_IMPLICIT_PROJECTED;
////ChTimestepper::Type intgr_type = ChTimestepper::Type::EULER_IMPLICIT;
////ChTimestepper::Type intgr_type = ChTimestepper::Type::HHT;

// Verbose output level (solver and integrator)
bool verbose_solver = false;
bool verbose_integrator = false;

// Time interval between two render frames
double render_step_size = 1.0 / 120;  // FPS = 120

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 0.0);

// =============================================================================

void SelectSolver(ChSystem& sys, ChSolver::Type& solver_type, ChTimestepper::Type& integrator_type) {
    // For NSC systems, use implicit linearized Euler and an iterative VI solver
    if (sys.GetContactMethod() == ChContactMethod::NSC) {
        integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
        if (solver_type != ChSolver::Type::BARZILAIBORWEIN && solver_type != ChSolver::Type::APGD &&
            solver_type != ChSolver::Type::PSOR && solver_type != ChSolver::Type::PSSOR) {
            solver_type = ChSolver::Type::BARZILAIBORWEIN;
        }
    }

    // If none of the direct sparse solver modules is enabled, default to SPARSE_QR
    if (solver_type == ChSolver::Type::PARDISO_MKL) {
#ifndef CHRONO_PARDISO_MKL
        solver_type = ChSolver::Type::SPARSE_QR;
#endif
    } else if (solver_type == ChSolver::Type::PARDISO_PROJECT) {
#ifndef CHRONO_PARDISOPROJECT
        solver_type = ChSolver::Type::SPARSE_QR;
#endif
    } else if (solver_type == ChSolver::Type::MUMPS) {
#ifndef CHRONO_MUMPS
        solver_type = ChSolver::Type::SPARSE_QR;
#endif
    }

    if (solver_type == ChSolver::Type::PARDISO_MKL) {
#ifdef CHRONO_PARDISO_MKL
        auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        solver->LockSparsityPattern(true);
        sys.SetSolver(solver);
#endif
    } else if (solver_type == ChSolver::Type::PARDISO_PROJECT) {
#ifdef CHRONO_PARDISOPROJECT
        auto solver = chrono_types::make_shared<ChSolverPardisoProject>();
        solver->LockSparsityPattern(true);
        sys->SetSolver(solver);
#endif
    } else if (solver_type == ChSolver::Type::MUMPS) {
#ifdef CHRONO_MUMPS
        auto solver = chrono_types::make_shared<ChSolverMumps>();
        solver->LockSparsityPattern(true);
        solver->EnableNullPivotDetection(true);
        solver->GetMumpsEngine().SetICNTL(14, 50);
        sys.SetSolver(solver);
#endif
    } else {
        sys.SetSolverType(solver_type);
        switch (solver_type) {
            case ChSolver::Type::SPARSE_LU:
            case ChSolver::Type::SPARSE_QR: {
                auto solver = std::static_pointer_cast<ChDirectSolverLS>(sys.GetSolver());
                solver->LockSparsityPattern(false);
                solver->UseSparsityPatternLearner(false);
                break;
            }
            case ChSolver::Type::BARZILAIBORWEIN:
            case ChSolver::Type::APGD:
            case ChSolver::Type::PSOR: {
                auto solver = std::static_pointer_cast<ChIterativeSolverVI>(sys.GetSolver());
                solver->SetMaxIterations(100);
                solver->SetOmega(0.8);
                solver->SetSharpnessLambda(1.0);

                ////sys.SetMaxPenetrationRecoverySpeed(1.5);
                ////sys.SetMinBounceSpeed(2.0);
                break;
            }
            case ChSolver::Type::BICGSTAB:
            case ChSolver::Type::MINRES:
            case ChSolver::Type::GMRES: {
                auto solver = std::static_pointer_cast<ChIterativeSolverLS>(sys.GetSolver());
                solver->SetMaxIterations(200);
                solver->SetTolerance(1e-10);
                solver->EnableDiagonalPreconditioner(true);
                break;
            }
        }
    }

    sys.SetTimestepperType(integrator_type);
    switch (integrator_type) {
        case ChTimestepper::Type::HHT: {
            auto integrator = std::static_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
            integrator->SetAlpha(-0.2);
            integrator->SetMaxiters(50);
            integrator->SetAbsTolerances(1e-4, 1e2);
            integrator->SetMode(ChTimestepperHHT::ACCELERATION);
            integrator->SetStepControl(false);
            integrator->SetModifiedNewton(false);
            integrator->SetScaling(false);
            break;
        }
        case ChTimestepper::Type::EULER_IMPLICIT: {
            auto integrator = std::static_pointer_cast<ChTimestepperEulerImplicit>(sys.GetTimestepper());
            integrator->SetMaxiters(50);
            integrator->SetAbsTolerances(1e-4, 1e2);
            break;
        }
        case ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED:
        case ChTimestepper::Type::EULER_IMPLICIT_PROJECTED:
            break;
    }
}

void ReportTiming(ChSystem& sys) {
    std::stringstream ss;
    ss.precision(4);
    ss << std::fixed << sys.GetChTime() << " | ";
    ss << sys.GetTimerStep() << " " << sys.GetTimerAdvance() << " " << sys.GetTimerUpdate() << " | ";
    ss << sys.GetTimerJacobian() << " " << sys.GetTimerLSsetup() << " " << sys.GetTimerLSsolve() << " | ";
    ss << sys.GetTimerCollision() << " " << sys.GetTimerCollisionBroad() << " " << sys.GetTimerCollisionNarrow();

    auto LS = std::dynamic_pointer_cast<ChDirectSolverLS>(sys.GetSolver());
    if (LS) {
        ss << " | ";
        ss << LS->GetTimeSetup_Assembly() << " " << LS->GetTimeSetup_SolverCall() << " ";
        ss << LS->GetTimeSolve_Assembly() << " " << LS->GetTimeSolve_SolverCall();
        LS->ResetTimers();
    }
    std::cout << ss.str() << std::endl;
}

void ReportConstraintViolation(ChSystem& sys, double threshold = 1e-3) {
    Eigen::Index imax = 0;
    double vmax = 0;
    std::string nmax = "";
    for (auto joint : sys.Get_linklist()) {
        if (joint->GetConstraintViolation().size() == 0)
            continue;
        Eigen::Index cimax;
        auto cmax = joint->GetConstraintViolation().maxCoeff(&cimax);
        if (cmax > vmax) {
            vmax = cmax;
            imax = cimax;
            nmax = joint->GetNameString();
        }
    }
    if (vmax > threshold)
        std::cout << vmax << "  in  " << nmax << " [" << imax << "]" << std::endl;
}

bool ReportTrackFailure(ChTrackedVehicle& veh, double threshold = 1e-2) {
    for (int i = 0; i < 2; i++) {
        auto track = veh.GetTrackAssembly(VehicleSide(i));
        auto nshoes = track->GetNumTrackShoes();
        auto shoe1 = track->GetTrackShoe(0).get();
        for (int j = 1; j < nshoes; j++) {
            auto shoe2 = track->GetTrackShoe(j % (nshoes - 1)).get();
            auto dir = shoe2->GetShoeBody()->TransformDirectionParentToLocal(shoe2->GetTransform().GetPos() -
                                                                             shoe1->GetTransform().GetPos());
            if (std::abs(dir.y()) > threshold) {
                std::cout << "...Track " << i << " broken between shoes " << j - 1 << " and " << j << std::endl;
                std::cout << "time " << veh.GetChTime() << std::endl;
                std::cout << "shoe " << j - 1 << " position: " << shoe1->GetTransform().GetPos() << std::endl;
                std::cout << "shoe " << j << " position: " << shoe2->GetTransform().GetPos() << std::endl;
                std::cout << "Lateral offset: " << dir.y() << std::endl;
                return true;
            }
            shoe1 = shoe2;
        }
    }
    return false;
}

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------------------
    // Create the various modules
    // --------------------------
    std::string vehicle_file;
    std::string powertrain_file;

    switch (shoe_type) {
        case TrackShoeType::SINGLE_PIN:
            vehicle_file = "M113/vehicle/M113_Vehicle_SinglePin";
            vehicle_file = vehicle_file + (driveline_type == DrivelineTypeTV::SIMPLE ? ".json" : "_BDS.json");
            break;
        case TrackShoeType::DOUBLE_PIN:
            vehicle_file = "M113/vehicle/M113_Vehicle_DoublePin";
            vehicle_file = vehicle_file + (driveline_type == DrivelineTypeTV::SIMPLE ? ".json" : "_BDS.json");
            break;
    }
    switch (powertrain_type) {
        case PowertrainModelType::SIMPLE_CVT:
            powertrain_file = "M113/powertrain/M113_SimpleCVTPowertrain.json";
            break;
        case PowertrainModelType::SIMPLE_MAP:
            powertrain_file = "M113/powertrain/M113_SimpleMapPowertrain.json";
            break;
        case PowertrainModelType::SHAFTS:
            powertrain_file = "M113/powertrain/M113_ShaftsPowertrain.json";
            break;
    }

    // Create the vehicle system
    TrackedVehicle vehicle(vehicle::GetDataFile(vehicle_file), contact_method);

    // Change collision shape for road wheels and idlers (true: cylinder; false: cylshell)
    ////vehicle.GetTrackAssembly(LEFT)->SetWheelCollisionType(false, false, false);
    ////vehicle.GetTrackAssembly(RIGHT)->SetWheelCollisionType(false, false, false);

    // Control steering type (enable crossdrive capability).
    ////vehicle.GetDriveline()->SetGyrationMode(true);

    // Initialize the vehicle at the specified position.
    vehicle.Initialize(ChCoordsys<>(initLoc, initRot));

    // Set visualization type for vehicle components
    vehicle.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetRoadWheelAssemblyVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    // Disable all contacts for vehicle chassis (if chassis collision was defined)
    ////vehicle.SetChassisCollide(false);

    // Disable only contact between chassis and track shoes (if chassis collision was defined)
    ////vehicle.SetChassisVehicleCollide(false);

    // Monitor contacts involving one of the sprockets.
    vehicle.MonitorContacts(TrackedCollisionFlag::SPROCKET_LEFT | TrackedCollisionFlag::SPROCKET_RIGHT);

    // Render contact normals and/or contact forces.
    vehicle.SetRenderContactNormals(true);
    ////vehicle.SetRenderContactForces(true, 1e-4);

    // Create and initialize the powertrain system
    auto powertrain = ReadPowertrainJSON(vehicle::GetDataFile(powertrain_file));
    vehicle.InitializePowertrain(powertrain);

    // Create the terrain
    RigidTerrain terrain(vehicle.GetSystem(), vehicle::GetDataFile(rigidterrain_file));
    terrain.Initialize();

    // Compatibility checks
    if (vehicle.HasBushings()) {
        if (contact_method == ChContactMethod::NSC) {
            cout << "The NSC iterative solvers cannot be used if bushings are present." << endl;
            return 1;
        }
    }

    auto vis = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("JSON Tracked Vehicle Demo");
    vis->SetChaseCamera(ChVector<>(0.0, 0.0, 0.0), 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();

    // ------------------------
    // Create the driver system
    // ------------------------

    std::shared_ptr<ChDriver> driver;
    switch (driver_mode) {
        case DriverMode::KEYBOARD: {
            auto irr_driver = chrono_types::make_shared<ChIrrGuiDriver>(*vis);
            double steering_time = 0.5;  // time to go from 0 to +1 (or from 0 to -1)
            double throttle_time = 1.0;  // time to go from 0 to +1
            double braking_time = 0.3;   // time to go from 0 to +1
            irr_driver->SetSteeringDelta(render_step_size / steering_time);
            irr_driver->SetThrottleDelta(render_step_size / throttle_time);
            irr_driver->SetBrakingDelta(render_step_size / braking_time);
            irr_driver->SetGains(2, 5, 5);
            driver = irr_driver;
            break;
        }
        case DriverMode::DATAFILE: {
            auto data_driver = chrono_types::make_shared<ChDataDriver>(vehicle, vehicle::GetDataFile(driver_file));
            driver = data_driver;
            break;
        }
        case DriverMode::PATH: {
            auto path = chrono::vehicle::StraightLinePath(chrono::ChVector<>(0, 0, 0.02), chrono::ChVector<>(500, 0, 0.02), 50);
            auto path_driver = std::make_shared<ChPathFollowerDriver>(vehicle, path, "my_path", target_speed);
            path_driver->GetSteeringController().SetLookAheadDistance(5.0);
            path_driver->GetSteeringController().SetGains(0.5, 0, 0);
            path_driver->GetSpeedController().SetGains(0.4, 0, 0);
            driver = path_driver;
        }
    }

    driver->Initialize();

    std::cout << "Track shoe type: " << vehicle.GetTrackShoe(LEFT, 0)->GetTemplateName() << std::endl;
    std::cout << "Driveline type:  " << vehicle.GetDriveline()->GetTemplateName() << std::endl;
    std::cout << "Powertrain type: " << powertrain->GetTemplateName() << std::endl;
    std::cout << "Vehicle mass: " << vehicle.GetMass() << std::endl;

    vehicle.SetVisualSystem(vis);

    // ------------------------------
    // Solver and integrator settings
    // ------------------------------

    double step_size = 1e-3;
    switch (contact_method) {
        case ChContactMethod::NSC:
            std::cout << "Use NSC" << std::endl;
            step_size = step_size_NSC;
            break;
        case ChContactMethod::SMC:
            std::cout << "Use SMC" << std::endl;
            step_size = step_size_SMC;
            break;
    }

    SelectSolver(*vehicle.GetSystem(), slvr_type, intgr_type);
    vehicle.GetSystem()->GetSolver()->SetVerbose(verbose_solver);
    vehicle.GetSystem()->GetTimestepper()->SetVerbose(verbose_integrator);

    std::cout << "SOLVER TYPE:     " << (int)slvr_type << std::endl;
    std::cout << "INTEGRATOR TYPE: " << (int)intgr_type << std::endl;

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    BodyStates shoe_states_left(vehicle.GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(vehicle.GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left(vehicle.GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(vehicle.GetNumTrackShoes(RIGHT));

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    int step_number = 0;

    while (vis->Run()) {
        if (step_number % render_steps == 0) {
            // Render scene
            vis->BeginScene();
            vis->DrawAll();
            vis->EndScene();
        }

        // Collect output data from modules (for inter-module communication)
        ChDriver::Inputs driver_inputs = driver->GetInputs();
        vehicle.GetTrackShoeStates(LEFT, shoe_states_left);
        vehicle.GetTrackShoeStates(RIGHT, shoe_states_right);

        // Update modules (process inputs from other modules)
        double time = vehicle.GetChTime();
        driver->Synchronize(time);
        terrain.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
        vis->Synchronize("", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);
        vis->Advance(step_size);

        ////ReportTiming(*vehicle.GetSystem());

        if (ReportTrackFailure(vehicle, 0.1)) {
            ReportConstraintViolation(*vehicle.GetSystem());
            break;
        }

        // Increment frame number
        step_number++;
    }

    return 0;
}
