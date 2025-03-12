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

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeDoublePin.h"
#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
    #include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/driver/ChInteractiveDriverVSG.h"
    #include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "demos/SetChronoSolver.h"

using namespace chrono;
using namespace chrono::vehicle;

using std::cout;
using std::endl;

// =============================================================================
// Specification of a vehicle model from JSON files
// Available models:
//    M113_SinglePin
//    M113_DoublePin
//    M113_RS_SinglePin
//    Marder

class Vehicle_Model {
  public:
    virtual std::string ModelName() const = 0;
    virtual std::string VehicleJSON() const = 0;
    virtual std::string EngineJSON() const = 0;
    virtual std::string TransmissionJSON() const = 0;
    virtual ChVector3d CameraPoint() const = 0;
    virtual double CameraDistance() const = 0;
};

class M113_SinglePin : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "M113_SinglePin"; }
    virtual std::string VehicleJSON() const override {
        return "M113/vehicle/M113_Vehicle_SinglePin.json";
        ////return "M113/vehicle/M113_Vehicle_SinglePin_BDS.json";
    }
    virtual std::string EngineJSON() const override {
        return "M113/powertrain/M113_EngineSimple.json";
        ////return "M113/powertrain/M113_EngineSimpleMap.json";
        ////return "M113/powertrain/M113_EngineShafts.json";
    }
    virtual std::string TransmissionJSON() const override {
        return "M113/powertrain/M113_AutomaticTransmissionSimpleMap.json";
        ////return "M113/powertrain/M113_AutomaticTransmissionShafts.json";
    }
    virtual ChVector3d CameraPoint() const override { return ChVector3d(0, 0, 0); }
    virtual double CameraDistance() const override { return 6.0; }
};

class M113_DoublePin : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "M113_DoublePin"; }
    virtual std::string VehicleJSON() const override {
        return "M113/vehicle/M113_Vehicle_DoublePin.json";
        ////return "M113/vehicle/M113_Vehicle_DoublePin_BDS.json";
    }
    virtual std::string EngineJSON() const override {
        return "M113/powertrain/M113_EngineSimple.json";
        ////return "M113/powertrain/M113_EngineSimpleMap.json";
        ////return "M113/powertrain/M113_EngineShafts.json";
    }
    virtual std::string TransmissionJSON() const override {
        return "M113/powertrain/M113_AutomaticTransmissionSimpleMap.json";
        ////return "M113/powertrain/M113_AutomaticTransmissionShafts.json";
    }
    virtual ChVector3d CameraPoint() const override { return ChVector3d(0, 0, 0); }
    virtual double CameraDistance() const override { return 6.0; }
};

class M113_RS_SinglePin : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "M113_RS_SinglePin"; }
    virtual std::string VehicleJSON() const override {
        ////return "M113_RS/vehicle/M113_Vehicle_SinglePin_Translational_BDS.json";
        return "M113_RS/vehicle/M113_Vehicle_SinglePin_Distance_BDS.json";
    }
    virtual std::string EngineJSON() const override {
        return "M113_RS/powertrain/M113_EngineSimple.json";
        ////return "M113_RS/powertrain/M113_EngineSimpleMap.json";
        ////return "M113_RS/powertrain/M113_EngineShafts.json";
    }
    virtual std::string TransmissionJSON() const override {
        return "M113_RS/powertrain/M113_AutomaticTransmissionSimpleMap.json";
        ////return "M113_RS/powertrain/M113_AutomaticTransmissionShafts.json";
    }
    virtual ChVector3d CameraPoint() const override { return ChVector3d(4, 0, 0); }
    virtual double CameraDistance() const override { return 6.0; }
};

class Marder_SinglePin : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "Marder_SinglePin"; }
    virtual std::string VehicleJSON() const override {
        return "Marder/vehicle/marder_sp_joints_shafts.json";
        ////return "Marder/vehicle/marder_sp_bushings_shafts.json";
        ////return "Marder/vehicle/marder_sp_bushings_simple.json";
    }
    virtual std::string EngineJSON() const override { return "Marder/powertrain/Marder_EngineSimple.json"; }
    virtual std::string TransmissionJSON() const override {
        return "Marder/powertrain/Marder_AutomaticTransmissionSimpleMap.json";
    }
    virtual ChVector3d CameraPoint() const override { return ChVector3d(0, 0, 0); }
    virtual double CameraDistance() const override { return 8.0; }
};

// =============================================================================
// USER SETTINGS
// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::IRRLICHT;

// Current vehicle model selection
////auto vehicle_model = M113_SinglePin();
////auto vehicle_model = M113_DoublePin();
////auto vehicle_model = M113_RS_SinglePin();
auto vehicle_model = Marder_SinglePin();

// JSON files for terrain (rigid plane)
std::string rigidterrain_file("terrain/RigidPlane.json");

// Initial vehicle position
ChVector3d initLoc(0, 0, 0.9);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);
////ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
////ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
////ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);

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

// Verbose output level (solver and integrator)
bool verbose_solver = false;
bool verbose_integrator = false;

// Time interval between two render frames
double render_step_size = 1.0 / 120;  // FPS = 120

// Point on chassis tracked by the camera
ChVector3d trackPoint(0.0, 0.0, 0.0);

// =============================================================================

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
    cout << ss.str() << endl;
}

void ReportConstraintViolation(ChSystem& sys, double threshold = 1e-3) {
    Eigen::Index imax = 0;
    double vmax = 0;
    std::string nmax = "";
    for (auto joint : sys.GetLinks()) {
        if (joint->GetConstraintViolation().size() == 0)
            continue;
        Eigen::Index cimax;
        auto cmax = joint->GetConstraintViolation().maxCoeff(&cimax);
        if (cmax > vmax) {
            vmax = cmax;
            imax = cimax;
            nmax = joint->GetName();
        }
    }
    if (vmax > threshold)
        cout << vmax << "  in  " << nmax << " [" << imax << "]" << endl;
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
                cout << "...Track " << i << " broken between shoes " << j - 1 << " and " << j << endl;
                cout << "time " << veh.GetChTime() << endl;
                cout << "shoe " << j - 1 << " position: " << shoe1->GetTransform().GetPos() << endl;
                cout << "shoe " << j << " position: " << shoe2->GetTransform().GetPos() << endl;
                cout << "Lateral offset: " << dir.y() << endl;
                return true;
            }
            shoe1 = shoe2;
        }
    }
    return false;
}

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create the vehicle system
    cout << "VEHICLE: " << vehicle_model.ModelName() << endl;
    TrackedVehicle vehicle(vehicle::GetDataFile(vehicle_model.VehicleJSON()), contact_method);

    // Change collision shape for road wheels and idlers (true: cylinder; false: cylshell)
    ////vehicle.GetTrackAssembly(LEFT)->SetWheelCollisionType(false, false, false);
    ////vehicle.GetTrackAssembly(RIGHT)->SetWheelCollisionType(false, false, false);

    // Control steering type (enable crossdrive capability).
    ////vehicle.GetDriveline()->SetGyrationMode(true);

    // Initialize the vehicle at the specified position.
    vehicle.Initialize(ChCoordsys<>(initLoc, initRot));
    ////vehicle.GetChassis()->SetFixed(true);

    // Set visualization type for vehicle components
    vehicle.SetChassisVisualizationType(VisualizationType::NONE);
    vehicle.SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetIdlerWheelVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetRollerVisualizationType(VisualizationType::PRIMITIVES);
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
    auto engine = ReadEngineJSON(vehicle::GetDataFile(vehicle_model.EngineJSON()));
    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(vehicle_model.TransmissionJSON()));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);

    cout << "  Track assembly templates" << endl;
    cout << "     Sprocket:   " << vehicle.GetTrackAssembly(LEFT)->GetSprocket()->GetTemplateName() << endl;
    cout << "     Brake:      " << vehicle.GetTrackAssembly(LEFT)->GetBrake()->GetTemplateName() << endl;
    cout << "     Idler:      " << vehicle.GetTrackAssembly(LEFT)->GetIdler()->GetTemplateName() << endl;
    cout << "     Suspension: " << vehicle.GetTrackAssembly(LEFT)->GetTrackSuspension(0)->GetTemplateName() << endl;
    cout << "     Track shoe: " << vehicle.GetTrackShoe(LEFT, 0)->GetTemplateName() << endl;
    cout << "  Driveline type:    " << vehicle.GetDriveline()->GetTemplateName() << endl;
    cout << "  Engine type:       " << engine->GetTemplateName() << endl;
    cout << "  Transmission type: " << transmission->GetTemplateName() << endl;
    cout << "  Vehicle mass:      " << vehicle.GetMass() << endl;

    // Associate a collision system
    vehicle.GetSystem()->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

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

    // -----------------------------------------
    // Create the vehicle run-time visualization
    // and driver system
    // -----------------------------------------

    std::shared_ptr<ChVehicleVisualSystem> vis;
    std::shared_ptr<ChDriver> driver;

    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            // Create the vehicle Irrlicht interface
            auto vis_irr = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle("JSON Tracked Vehicle Demo");
            vis_irr->SetChaseCamera(vehicle_model.CameraPoint(), vehicle_model.CameraDistance(), 0.5);
            vis_irr->SetChaseCameraMultipliers(1e-4, 10);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();

            vis = vis_irr;

            if (driver_mode == DriverMode::KEYBOARD) {
                auto irr_driver = chrono_types::make_shared<ChInteractiveDriverIRR>(*vis_irr);
                double steering_time = 0.5;  // time to go from 0 to +1 (or from 0 to -1)
                double throttle_time = 1.0;  // time to go from 0 to +1
                double braking_time = 0.3;   // time to go from 0 to +1
                irr_driver->SetSteeringDelta(render_step_size / steering_time);
                irr_driver->SetThrottleDelta(render_step_size / throttle_time);
                irr_driver->SetBrakingDelta(render_step_size / braking_time);
                irr_driver->SetGains(2, 5, 5);
                driver = irr_driver;
            }

#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            // Create the vehicle VSG interface
            auto vis_vsg = chrono_types::make_shared<ChTrackedVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle("JSON Tracked Vehicle Demo");
            vis_vsg->SetChaseCamera(vehicle_model.CameraPoint(), vehicle_model.CameraDistance(), 0.5);
            vis_vsg->Initialize();

            vis = vis_vsg;

            if (driver_mode == DriverMode::KEYBOARD) {
                auto vsg_driver = chrono_types::make_shared<ChInteractiveDriverVSG>(*vis_vsg);
                double steering_time = 0.5;  // time to go from 0 to +1 (or from 0 to -1)
                double throttle_time = 1.0;  // time to go from 0 to +1
                double braking_time = 0.3;   // time to go from 0 to +1
                vsg_driver->SetSteeringDelta(render_step_size / steering_time);
                vsg_driver->SetThrottleDelta(render_step_size / throttle_time);
                vsg_driver->SetBrakingDelta(render_step_size / braking_time);
                vsg_driver->SetGains(2, 5, 5);
                driver = vsg_driver;
            }

#endif
            break;
        }
    }

    switch (driver_mode) {
        case DriverMode::DATAFILE: {
            auto data_driver = chrono_types::make_shared<ChDataDriver>(vehicle, vehicle::GetDataFile(driver_file));
            driver = data_driver;
            break;
        }
        case DriverMode::PATH: {
            auto path =
                chrono::vehicle::StraightLinePath(chrono::ChVector3d(0, 0, 0.02), chrono::ChVector3d(500, 0, 0.02), 50);
            auto path_driver = std::make_shared<ChPathFollowerDriver>(vehicle, path, "my_path", target_speed);
            path_driver->GetSteeringController().SetLookAheadDistance(5.0);
            path_driver->GetSteeringController().SetGains(0.5, 0, 0);
            path_driver->GetSpeedController().SetGains(0.4, 0, 0);
            driver = path_driver;
            break;
        }
        default:
            break;
    }
    driver->Initialize();

    vis->AttachVehicle(&vehicle);

    // Solver and integrator settings
    double step_size = 1e-3;
    switch (contact_method) {
        case ChContactMethod::NSC:
            cout << "Use NSC" << endl;
            step_size = step_size_NSC;
            break;
        case ChContactMethod::SMC:
            cout << "Use SMC" << endl;
            step_size = step_size_SMC;
            break;
    }

    SetChronoSolver(*vehicle.GetSystem(), ChSolver::Type::BARZILAIBORWEIN,
                    ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    vehicle.GetSystem()->GetSolver()->SetVerbose(verbose_solver);
    vehicle.GetSystem()->GetTimestepper()->SetVerbose(verbose_integrator);

    cout << "SOLVER TYPE:     " << (int)vehicle.GetSystem()->GetSolver()->GetType() << endl;
    cout << "INTEGRATOR TYPE: " << (int)vehicle.GetSystem()->GetTimestepper()->GetType() << endl;

    // ---------------
    // Simulation loop
    // ---------------

    std::cout << "\n============ Vehicle subsystems ============" << std::endl;
    vehicle.LogSubsystemTypes();

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    int step_number = 0;

    while (vis->Run()) {
        // Render scene
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }

        // Current driver inputs
        DriverInputs driver_inputs = driver->GetInputs();

        // Release chassis
        ////if (vehicle.GetChTime() < 1) {
        ////    driver_inputs.m_throttle = 0;
        ////    driver_inputs.m_braking = 0;
        ////    driver_inputs.m_steering = 0;
        ////} else {
        ////    vehicle.GetChassis()->SetFixed(false);
        ////}

        // Update modules (process inputs from other modules)
        double time = vehicle.GetChTime();
        driver->Synchronize(time);
        terrain.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs);
        vis->Synchronize(time, driver_inputs);

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
