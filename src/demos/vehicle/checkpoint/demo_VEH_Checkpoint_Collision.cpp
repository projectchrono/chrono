// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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
// Demonstration of colliding two vehicles initialized from the same checkpoint
// file and relocated on a collision course.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <cmath>

#include "chrono/physics/ChSystemSMC.h"

#include "chrono/functions/ChFunction.h"
#include "chrono/utils/ChUtils.h"
#include "chrono/input_output/ChWriterCSV.h"
#include "chrono/solver/ChSolverBB.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

using std::cout;
using std::cerr;
using std::endl;

// =============================================================================

enum class ChassisCollisionGeometry { SPHERE_PRIM, SPHERE_MESH, TRIMESH };

ChassisCollisionGeometry geometry_type = ChassisCollisionGeometry::SPHERE_PRIM;

// =============================================================================

// Create a Chrono physical system with default settings for wheeled vehicle simulations.
std::unique_ptr<ChSystemSMC> CreateSystem() {
    auto sys = chrono_types::make_unique<ChSystemSMC>();

    sys->SetGravitationalAcceleration(-9.81 * ChWorldFrame::Vertical());
    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    auto solver = chrono_types::make_shared<ChSolverBB>();
    solver->SetMaxIterations(100);
    solver->SetOmega(0.8);
    solver->SetSharpnessLambda(1.0);
    sys->SetSolver(solver);

    sys->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    return std::move(sys);
}

// =============================================================================

// Create a rigid terrain patch at the specified height.
RigidTerrain CreateTerrain(ChSystem* sys, double height) {
    // Terrain patch dimensions
    double x_size = 200;
    double y_size = 200;

    // Create the terrain
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(sys->GetContactMethod());

    RigidTerrain terrain(sys);
    auto patch = terrain.AddPatch(patch_mat, ChCoordsysd(ChVector3d(0, 0, height), QUNIT), x_size, y_size);
    patch->SetTexture(GetChronoDataFile("textures/checker2.png"), 20, 20);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

    terrain.Initialize();

    return terrain;
}

// =============================================================================

class MyVehicle {
  public:
    MyVehicle(ChSystem* system) : system(system) { hmmwv_full = chrono_types::make_unique<HMMWV_Full>(system); }

    void SetChassisVisualizationType(VisualizationType type) { chassis_visualization_type = type; }
    void SetChassisCollisionType(CollisionType type) { chassis_collision_type = type; }
    void SetChassisCollisionGeometry(const utils::ChBodyGeometry& geometry) { hmmwv_full->SetChassisCollisionGeometry(geometry); }

    void Initialize(const ChCoordsys<>& init_pos) {
        hmmwv_full->SetChassisCollisionType(chassis_collision_type);
        hmmwv_full->SetChassisFixed(false);
        hmmwv_full->SetInitPosition(init_pos);
        hmmwv_full->SetEngineType(EngineModelType::SIMPLE_MAP);
        hmmwv_full->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_CVT);
        hmmwv_full->SetDriveType(DrivelineTypeWV::AWD);
        hmmwv_full->UseTierodBodies(true);
        hmmwv_full->SetSteeringType(SteeringTypeWV::PITMAN_ARM);
        hmmwv_full->SetBrakeType(BrakeType::SHAFTS);
        hmmwv_full->SetTireType(TireModelType::TMEASY);
        hmmwv_full->SetTireCollisionType(ChTire::CollisionType::SINGLE_POINT);
        hmmwv_full->Initialize();
        hmmwv_full->SetChassisVisualizationType(chassis_visualization_type);
        hmmwv_full->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
        hmmwv_full->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
        hmmwv_full->SetWheelVisualizationType(VisualizationType::MESH);
        hmmwv_full->SetTireVisualizationType(VisualizationType::MESH);
    }

    void Initialize(const std::string& out_dir, const ChVector2d& xy_pos, double yaw_angle) {
        // Initialize vehicle
        Initialize(ChCoordsysd());
        auto& vehicle = hmmwv_full->GetVehicle();

        // Overwrite vehicle states with information from checkpoint files
        vehicle.ImportCheckpoint(ChCheckpoint::Format::ASCII, out_dir + "/vehicle_checkpoint.txt");
        int tire_id = 0;
        for (const auto& a : vehicle.GetAxles()) {
            for (const auto& w : a->GetWheels()) {
                if (w->GetTire()) {
                    w->GetTire()->ImportCheckpoint(ChCheckpoint::Format::ASCII, out_dir + "/tire_" + std::to_string(tire_id++) + "_checkpoint.txt");
                }
            }
        }

        // Relocate and reorient vehicle
        vehicle.Relocate(xy_pos, yaw_angle);
    }

    ChWheeledVehicle& GetVehicle() { return hmmwv_full->GetVehicle(); }
    void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) { hmmwv_full->Synchronize(time, driver_inputs, terrain); }
    void Advance(double step) { hmmwv_full->Advance(step); }
    ChVector3d TrackPoint() const { return ChVector3d(0, 0, 1.75); }
    double CameraDistance() const { return 10.0; }
    double CameraHeight() const { return 0.5; }

  private:
    ChSystem* system;
    std::unique_ptr<HMMWV_Full> hmmwv_full;
    VisualizationType chassis_visualization_type = VisualizationType::NONE;
    CollisionType chassis_collision_type = CollisionType::NONE;
};

// =============================================================================

// Steering function that starts at zero and then follows a cosine wave after a specified delay.
class FunctionCosineSteering : public ChFunction {
  public:
    FunctionCosineSteering(double delay, double frequency) : delay(delay), frequency(frequency) {}
    FunctionCosineSteering(const FunctionCosineSteering& other) {
        delay = other.delay;
        frequency = other.frequency;
    }
    virtual FunctionCosineSteering* Clone() const override { return new FunctionCosineSteering(*this); }
    virtual double GetVal(double time) const override {
        if (time < delay)
            return 0;
        return 0.5 + 0.5 * std::cos(CH_2PI * frequency * (time - delay) + CH_PI);
    }

  private:
    double delay;
    double frequency;
};

struct DriverFunctions {
    DriverFunctions(std::shared_ptr<ChFunction> throttle, std::shared_ptr<ChFunction> braking, std::shared_ptr<ChFunction> steering)
        : throttle(throttle), braking(braking), steering(steering) {}

    bool HasFunctions() const { return throttle && braking && steering; }

    std::shared_ptr<ChFunction> throttle;
    std::shared_ptr<ChFunction> braking;
    std::shared_ptr<ChFunction> steering;
};

// Driver system using specified functions
class MyDriver : public ChDriver {
  public:
    MyDriver(ChVehicle& vehicle, const DriverFunctions& functions)
        : ChDriver(vehicle), m_throttle_fun(functions.throttle), m_braking_fun(functions.braking), m_steering_fun(functions.steering) {}

    MyDriver(ChVehicle& vehicle, const std::string& out_dir) : ChDriver(vehicle) {
        ImportCheckpoint(ChCheckpoint::Format::ASCII, out_dir + "/driver_checkpoint.txt");
        m_throttle_fun = chrono_types::make_shared<ChFunctionConst>(GetThrottle());
        m_braking_fun = chrono_types::make_shared<ChFunctionConst>(GetBraking());
        m_steering_fun = chrono_types::make_shared<ChFunctionConst>(GetSteering());
    }

    virtual void Synchronize(double time) override {
        DriverInputs driver_inputs = GetInputs();
        m_throttle = m_throttle_fun->GetVal(time);
        m_braking = m_braking_fun->GetVal(time);
        m_steering = m_steering_fun->GetVal(time);
    }

  private:
    std::shared_ptr<ChFunction> m_throttle_fun;
    std::shared_ptr<ChFunction> m_braking_fun;
    std::shared_ptr<ChFunction> m_steering_fun;
};

// =============================================================================

#ifdef CHRONO_VSG

// Create a VSG visualization system for the specified vehicle and driver.
std::shared_ptr<ChWheeledVehicleVisualSystemVSG> CreateVisualization(MyVehicle& my_vehicle, MyDriver& my_driver) {
    auto& vehicle = my_vehicle.GetVehicle();
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
    vis->AttachVehicle(&vehicle);
    vis->AttachDriver(&my_driver);
    vis->SetChaseCamera(my_vehicle.TrackPoint(), my_vehicle.CameraDistance(), my_vehicle.CameraHeight());
    vis->SetChaseCameraAngle(0.75 * CH_PI);
    vis->SetWindowSize(1280, 800);
    vis->EnableSkyTexture(SkyMode::DOME);
    vis->SetCameraAngleDeg(40);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.8 * CH_PI_2, CH_PI_4);
    vis->EnableShadows();
    vis->Initialize();
    vis->ToggleAbsFrameVisibility();

    return vis;
}

// Render the visualization system at the specified time and render frame rate.
// Return false if the visualization system is closed.
bool RenderVisualization(std::shared_ptr<ChWheeledVehicleVisualSystemVSG> vis, double time, double render_fps) {
    static int render_frame = 0;

    if (vis) {
        if (!vis->Run())
            return false;
        if (time >= render_frame / render_fps) {
            vis->Render();
            render_frame++;
        }
    }

    return true;
}

#endif

// =============================================================================

// Simulate a single vehicle
// - use the specified driver functions
// - stop simulation when reaching the end time or the target speed
// - save final checkpoint in the specified directory
void SimulateSingle(double time_end, double target_speed, const DriverFunctions& functions, const std::string& out_dir) {
    cout << "Simulate single vehicle" << endl;

    // Create the containing Chrono system
    auto sys = CreateSystem();
    double step_size = 2e-3;

    // Create the vehicle model, terrain, and driver system
    // The vehicle is created with no chassis visualization or collision
    MyVehicle my_vehicle(sys.get());
    my_vehicle.Initialize(ChCoordsysd(ChVector3d(0, 0, 0.5), QUNIT));
    auto& vehicle = my_vehicle.GetVehicle();
    vehicle.EnableRealtime(true);

    RigidTerrain terrain = CreateTerrain(sys.get(), 0.0);
    MyDriver my_driver(vehicle, functions);

    // Create the vehicle run-time visualization
#ifdef CHRONO_VSG
    auto vis = CreateVisualization(my_vehicle, my_driver);
#endif

    // Simulation loop
    double render_fps = 50;

    while (true) {
        double time = sys->GetChTime();

#ifdef CHRONO_VSG
        if (!RenderVisualization(vis, time, render_fps))
            break;
#endif
        if (time > time_end)
            break;
        if (vehicle.GetSpeed() >= target_speed)
            break;

        // Synchronize subsystems
        my_driver.Synchronize(time);
        terrain.Synchronize(time);
        my_vehicle.Synchronize(time, my_driver.GetInputs(), terrain);
#ifdef CHRONO_VSG
        vis->Synchronize(time, my_driver.GetInputs());
#endif

        // Advance simulation
        my_driver.Advance(step_size);
        terrain.Advance(step_size);
        my_vehicle.Advance(step_size);
#ifdef CHRONO_VSG
        vis->Advance(step_size);
#endif

        // Advance state of containing system
        sys->DoStepDynamics(step_size);
    }

    // Checkpoint final vehicle and driver state
    cout << "Output vehicle checkpoint file: " << out_dir + "/vehicle_checkpoint.txt" << endl;
    cout << "Output tire checkpoint file:  " << out_dir + "/tire_X_checkpoint.txt" << endl;
    cout << "Output driver checkpoint file:  " << out_dir + "/driver_checkpoint.txt" << endl;
    cout << endl;

    vehicle.ExportCheckpoint(ChCheckpoint::Format::ASCII, out_dir + "/vehicle_checkpoint.txt");
    my_driver.ExportCheckpoint(ChCheckpoint::Format::ASCII, out_dir + "/driver_checkpoint.txt");
    int tire_id = 0;
    for (const auto& a : vehicle.GetAxles()) {
        for (const auto& w : a->GetWheels()) {
            if (w->GetTire()) {
                w->GetTire()->ExportCheckpoint(ChCheckpoint::Format::ASCII, out_dir + "/tire_" + std::to_string(tire_id++) + "_checkpoint.txt");
            }
        }
    }
}

// =============================================================================

// Simulate multiple vehicles
// - initialize from the checkpoint in the specified directory
// - place vehicles at the given x-y locations and orient with the given yaw angles
// - create a driver with specified functions
// - if no driver functions, maintain constant driver inputs (from checkpoint)
void SimulateMultiple(const std::vector<std::pair<ChVector2d, double>>& positions, const DriverFunctions& functions, const std::string& out_dir) {
    cout << "Simulate multiple vehicles" << endl;

    // Create the containing Chrono system
    auto sys = CreateSystem();

    // Create new chassis collision geometry
    ChContactMaterialData minfo;
    minfo.mu = 1.0f;
    minfo.cr = 0.05f;
    minfo.Y = 5e7f;

    utils::ChBodyGeometry geometry;
    geometry.materials.push_back(minfo);

    switch (geometry_type) {
        case ChassisCollisionGeometry::SPHERE_PRIM: {
            utils::ChBodyGeometry::SphereShape sphere(ChVector3d(1.6, 0, 0), 0.5, 0);
            geometry.coll_spheres.push_back(sphere);
            break;
        }
        case ChassisCollisionGeometry::SPHERE_MESH: {
            utils::ChBodyGeometry::TrimeshShape sphere(ChVector3d(1.6, 0, 0), QUNIT, GetChronoDataFile("models/sphere.obj"), VNULL, 0.5, 0.05, 0);
            geometry.coll_meshes.push_back(sphere);
            break;
        }
    }

    // Create the vehicle models and associated driver systems
    // The vehicles are created with the specified collision geometry
    std::vector<std::shared_ptr<MyVehicle>> my_vehicles;
    std::vector<std::shared_ptr<MyDriver>> my_drivers;
    std::vector<ChWriterCSV> csv_writers;
    for (const auto& pos : positions) {
        auto my_vehicle = chrono_types::make_shared<MyVehicle>(sys.get());
        my_vehicle->SetChassisCollisionType(CollisionType::PRIMITIVES);
        my_vehicle->SetChassisVisualizationType(VisualizationType::COLLISION);
        my_vehicle->SetChassisCollisionGeometry(geometry);
        my_vehicle->Initialize(out_dir, pos.first, pos.second);
        auto& vehicle = my_vehicle->GetVehicle();
        vehicle.EnableRealtime(true);

        // Create the driver system
        if (functions.HasFunctions())
            my_drivers.push_back(chrono_types::make_shared<MyDriver>(vehicle, functions));
        else
            my_drivers.push_back(chrono_types::make_shared<MyDriver>(vehicle, out_dir));

        my_vehicles.push_back(my_vehicle);
        csv_writers.push_back(ChWriterCSV(" "));
    }

    // Create the terrain
    RigidTerrain terrain = CreateTerrain(sys.get(), 0.0);

    // Create the vehicle run-time visualization
#ifdef CHRONO_VSG
    auto vis = CreateVisualization(*my_vehicles[0], *my_drivers[0]);
#else
    double time_end = 50;
#endif

    // Simulation loop

    double step_size = 1e-3;
    double render_fps = 50;
    bool collision_start = false;
    
    while (true) {
        double time = sys->GetChTime();

        for (int i = 0; i < my_vehicles.size(); i++) {
            auto& vehicle = my_vehicles[i]->GetVehicle();
            csv_writers[i] << time << vehicle.GetPos() << vehicle.GetSpeed() << std::endl;
        }

#ifdef CHRONO_VSG
        if (!RenderVisualization(vis, time, render_fps))
            break;
#else
        if (time > time_end)
            break;
#endif

        // Synchronize subsystems
        for (int i = 0; i < my_vehicles.size(); i++) {
            my_drivers[i]->Synchronize(time);
            my_vehicles[i]->Synchronize(time, my_drivers[i]->GetInputs(), terrain);
        }
        terrain.Synchronize(time);
#ifdef CHRONO_VSG
        vis->Synchronize(time, my_drivers[0]->GetInputs());
#endif

        // Advance simulation
        for (int i = 0; i < my_vehicles.size(); i++) {
            my_drivers[i]->Advance(step_size);
            my_vehicles[i]->Advance(step_size);
        }
        terrain.Advance(step_size);
#ifdef CHRONO_VSG
        vis->Advance(step_size);
#endif

        // Advance state of entire system (containing all vehicles)
        sys->DoStepDynamics(step_size);

        // Intercept start and end of chassis collision
        // - reduce step size while contact occurs
        // - print vehicle speed after collision
        if (!collision_start && sys->GetNumContacts() > 0) {
            collision_start = true;
            step_size /= 10;
            cout << "Start contact at t:    " << time << endl;
            cout << "  Step size reduced:   " << step_size << endl;
        }

        if (collision_start && sys->GetNumContacts() == 0) {
            step_size *= 10;
            cout << "End contact at t:      " << time << endl;
            cout << "  Step size increased: " << step_size << endl;
            cout << "  Vehicle 1 speed:     " << my_vehicles[0]->GetVehicle().GetSpeed() << endl;
            cout << "  Vehicle 2 speed:     " << my_vehicles[1]->GetVehicle().GetSpeed() << endl;
            collision_start = false;
        }
    }

#ifdef CHRONO_POSTPROCESS
    for (int i = 0; i < my_vehicles.size(); i++) {
        csv_writers[i].WriteToFile(out_dir + "/vehicle_" + std::to_string(i) + ".csv");
    }

    /*
    // Plot trajectories of all vehicles
    postprocess::ChGnuPlot gplot_traj(out_dir + "/vehicle_traj.gpl");
    for (int i = 0; i < my_vehicles.size(); i++) {
        gplot_traj.Plot(out_dir + "/vehicle_" + std::to_string(i) + ".csv", 3, 2, "vehicle " + std::to_string(i), "with lines lw 2");
    }
    gplot_traj.SetTitle("Vehicle Trajectories");
    gplot_traj.SetLabelX("Y [m]");
    gplot_traj.SetLabelY("X [m]");
    gplot_traj.SetAspectRatio(-1);
    */

    // Plot speeds of all vehicles
    postprocess::ChGnuPlot gplot_speed(out_dir + "/vehicle_speed.gpl");
    for (int i = 0; i < my_vehicles.size(); i++) {
        gplot_speed.Plot(out_dir + "/vehicle_" + std::to_string(i) + ".csv", 1, 5, "vehicle " + std::to_string(i), "with lines lw 2");
    }
    gplot_speed.SetTitle("Vehicle Speeds");
    gplot_speed.SetLabelX("t [s]");
    gplot_speed.SetLabelY("v [m/s]");
#endif
}

// =============================================================================

int main(int argc, char* argv[]) {
    // Create output directories
    std::string out_dir = GetChronoOutputPath() + "VEHICLE_CHECKPOINT_1";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }

    // Various driver input functions
    DriverFunctions functions_wave(chrono_types::make_shared<ChFunctionConst>(0.5),             //
                                   chrono_types::make_shared<ChFunctionConst>(0.0),             //
                                   chrono_types::make_shared<FunctionCosineSteering>(1.0, 0.5)  //
    );
    DriverFunctions functions_acc(chrono_types::make_shared<ChFunctionConst>(0.5),  //
                                  chrono_types::make_shared<ChFunctionConst>(0.0),  //
                                  chrono_types::make_shared<ChFunctionConst>(0.0)   //
    );
    DriverFunctions functions_null(nullptr, nullptr, nullptr);

    // - Simulate one vehicle accelerating in straight line until reaching target speed
    // - Simulate two vehicles on a head-on collision
    double time_end = 1000;
    double target_speed = 10;
    SimulateSingle(time_end, target_speed, functions_acc, out_dir);
    SimulateMultiple({{ChVector2d(0, 0), 0.0}, {ChVector2d(20, 0), CH_PI}}, functions_acc, out_dir);

    return 0;
}
