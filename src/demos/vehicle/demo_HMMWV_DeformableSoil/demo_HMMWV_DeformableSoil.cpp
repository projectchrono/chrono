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
// Author: Radu Serban
// =============================================================================
//
// Chrono::Vehicle + ChronoParallel demo program for simulating a HMMWV vehicle
// over rigid or granular material.
//
// The vehicle model uses the utility class ChWheeledVehicleAssembly and is
// based on JSON specification files from the Chrono data directory.
//
// Contact uses the DEM-P (penalty) formulation.
//
// The global reference frame has Z up.
// All units SI.
// =============================================================================

#include <stdio.h>
#include <vector>
#include <cmath>

#include "chrono/core/ChFileutils.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/terrain/DeformableTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleAssembly.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::vehicle;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

// -----------------------------------------------------------------------------
// Terrain parameters
// -----------------------------------------------------------------------------

// Dimensions
double terrainHeight = 0;
double terrainLength = 16.0;  // size in X direction
double terrainWidth = 8.0;    // size in Y direction

// Divisions (X and Y)
int divLength = 1024;
int divWidth = 512;

// -----------------------------------------------------------------------------
// Vehicle parameters
// -----------------------------------------------------------------------------

// Type of wheel/tire (controls both contact and visualization)
enum WheelType { CYLINDRICAL, LUGGED };
WheelType wheel_type = LUGGED;

// JSON files for vehicle model (using different wheel visualization meshes)
std::string vehicle_file_cyl("hmmwv/vehicle/HMMWV_Vehicle_simple.json");
std::string vehicle_file_lug("hmmwv/vehicle/HMMWV_Vehicle_simple_lugged.json");

// JSON files for powertrain (simple)
std::string simplepowertrain_file("hmmwv/powertrain/HMMWV_SimplePowertrain.json");

// Initial vehicle position and orientation
ChVector<> initLoc(-5, -2, 0.6);
ChQuaternion<> initRot(1, 0, 0, 0);

// Contact material properties
float Y_t = 1e6;
float cr_t = 0.1;
float mu_t = 0.8;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Simulation step size
double step_size = 1e-2;

// Time interval between two render frames
double render_step_size = 1.0 / 200;  // FPS = 50

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Output directories
const std::string out_dir = "../HMMWV_DEF_SOIL";
const std::string img_dir = out_dir + "/IMG";

// Visualization output
bool img_output = false;

// =============================================================================

// Callback class for providing driver inputs.
class MyDriverInputs : public ChDriverInputsCallback {
  public:
    MyDriverInputs(double delay) : m_delay(delay), m_throttle(0), m_steering(0), m_braking(0) {}

    virtual void onCallback(double time, double& throttle, double& steering, double& braking) {
        throttle = 0;
        steering = 0;
        braking = 0;

        double eff_time = time - m_delay;

        // Do not generate any driver inputs for a duration equal to m_delay.
        if (eff_time < 0)
            return;

        if (eff_time > 0.2)
            throttle = 0.75;
        else if (eff_time > 0.1)
            throttle = 7.5 * (eff_time - 0.1);

        if (eff_time < 0.5)
            steering = 0;
        else
            steering = 0.7 * std::sin(CH_C_2PI * (eff_time - 0.5) / 6);

        m_throttle = throttle;
        m_steering = steering;
        m_braking = braking;
    }

    double GetThrottle() const { return m_throttle; }
    double GetSteering() const { return m_steering; }
    double GetBraking() const { return m_braking; }

  private:
    double m_delay;
    double m_throttle;
    double m_steering;
    double m_braking;
};

// Callback class for specifying rigid tire contact model.
// This version uses cylindrical contact shapes.
class MyCylindricalTire : public ChTireContactCallback {
  public:
    virtual void onCallback(std::shared_ptr<ChBody> wheelBody, double radius, double width) {
        wheelBody->GetCollisionModel()->ClearModel();
        wheelBody->GetCollisionModel()->AddCylinder(0.46, 0.46, width / 2);
        wheelBody->GetCollisionModel()->BuildModel();

        wheelBody->GetMaterialSurfaceDEM()->SetFriction(mu_t);
        wheelBody->GetMaterialSurfaceDEM()->SetYoungModulus(Y_t);
        wheelBody->GetMaterialSurfaceDEM()->SetRestitution(cr_t);
    }
};

// Callback class for specifying rigid tire contact model.
// This version uses a collection of convex contact shapes (meshes).
class MyLuggedTire : public ChTireContactCallback {
  public:
    MyLuggedTire() {
        std::string lugged_file("hmmwv/lugged_wheel_section.obj");
        geometry::ChTriangleMeshConnected lugged_mesh;
        utils::LoadConvexMesh(vehicle::GetDataFile(lugged_file), lugged_mesh, lugged_convex);
        num_hulls = lugged_convex.GetHullCount();
    }

    virtual void onCallback(std::shared_ptr<ChBody> wheelBody, double radius, double width) {
        ChCollisionModel* coll_model = wheelBody->GetCollisionModel();
        coll_model->ClearModel();

        // Assemble the tire contact from 15 segments, properly offset.
        // Each segment is further decomposed in convex hulls.
        for (int iseg = 0; iseg < 15; iseg++) {
            ChQuaternion<> rot = Q_from_AngAxis(iseg * 24 * CH_C_DEG_TO_RAD, VECT_Y);
            for (int ihull = 0; ihull < num_hulls; ihull++) {
                std::vector<ChVector<> > convexhull;
                lugged_convex.GetConvexHullResult(ihull, convexhull);
                coll_model->AddConvexHull(convexhull, VNULL, rot);
            }
        }

        // Add a cylinder to represent the wheel hub.
        coll_model->AddCylinder(0.223, 0.223, 0.126);

        coll_model->BuildModel();

        wheelBody->GetMaterialSurfaceDEM()->SetFriction(mu_t);
        wheelBody->GetMaterialSurfaceDEM()->SetYoungModulus(Y_t);
        wheelBody->GetMaterialSurfaceDEM()->SetRestitution(cr_t);
    }

  private:
    ChConvexDecompositionHACDv2 lugged_convex;
    int num_hulls;
};

// =============================================================================

int main(int argc, char* argv[]) {
    // --------------
    // Create system.
    // --------------

    ChSystemDEM* system = new ChSystemDEM();

    system->Set_G_acc(ChVector<>(0, 0, -9.81));

    // Solver settings.
    ////system->SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES);
    system->SetIterLCPmaxItersSpeed(50);
    system->SetIterLCPmaxItersStab(50);
    ////system->SetTol(0);
    ////system->SetMaxPenetrationRecoverySpeed(1.5);
    ////system->SetMinBounceSpeed(2.0);
    ////system->SetIterLCPomega(0.8);
    ////system->SetIterLCPsharpnessLambda(1.0);

    // ------------------
    // Create the terrain
    // ------------------

    DeformableTerrain terrain(system);
    terrain.SetPlane(ChCoordsys<>(VNULL, Q_from_AngX(CH_C_PI_2)));
    terrain.SetSoilParametersSCM(2e6,   // Bekker Kphi
                                 0,     // Bekker Kc
                                 1.1,   // Bekker n exponent
                                 0,     // Mohr cohesive limit (Pa)
                                 20,    // Mohr friction limit (degrees)
                                 0.01,  // Janosi shear coefficient (m)
                                 2e8    // Elastic stiffness (Pa/m), before plastic yeld
                                 );
    ////terrain.SetBulldozingFlow(true);     // inflate soil at the border of the rut
    ////terrain.SetBulldozingParameters(40,  // angle of frictionfor erosion of displaced material at the border of the rut
    ////                                1.6);  // displaced material vs downward pressed material.

    ////terrain.SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 80, 16);
    terrain.SetPlotType(vehicle::DeformableTerrain::PLOT_PRESSURE_YELD, 0, 30000.2);
    ////terrain.SetPlotType(vehicle::DeformableTerrain::PLOT_SINKAGE, 0, 0.15);

    terrain.Initialize(terrainHeight, terrainLength, terrainWidth, divLength, divWidth);

    ////AddFixedObstacles(vehicle.GetSystem());

    // -----------------------------------------
    // Create and initialize the vehicle system.
    // -----------------------------------------

    ChWheeledVehicleAssembly* vehicle;
    ChTireContactCallback* tire_cb;

    // Create the vehicle assembly and the callback object for tire contact
    // according to the specified type of tire/wheel.
    switch (wheel_type) {
        case CYLINDRICAL: {
            vehicle = new ChWheeledVehicleAssembly(system, vehicle_file_cyl, simplepowertrain_file);
            tire_cb = new MyCylindricalTire();
        } break;
        case LUGGED: {
            vehicle = new ChWheeledVehicleAssembly(system, vehicle_file_lug, simplepowertrain_file);
            tire_cb = new MyLuggedTire();
        } break;
    }

    vehicle->SetTireContactCallback(tire_cb);

    // Set the callback object for driver inputs. Pass the hold time as a delay in
    // generating driver inputs.
    MyDriverInputs driver_cb(0);
    vehicle->SetDriverInputsCallback(&driver_cb);

    // Initialize the vehicle at a height above the terrain.
    vehicle->Initialize(initLoc, initRot);

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------
    ChWheeledVehicleIrrApp app(vehicle->GetVehicle().get(), vehicle->GetPowertrain().get(), L"HMMWV Deformable Soil Demo");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // -----------------
    // Initialize output
    // -----------------

    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    if (img_output) {
        if (ChFileutils::MakeDirectory(img_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << img_dir << std::endl;
            return 1;
        }
    }

    // ---------------
    // Simulation loop
    // ---------------

    system->SetupInitial();

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;
    int render_frame = 0;
    ChRealtimeStepTimer realtime_timer;

    while (app.GetDevice()->run()) {
        // Render scene
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();

            if (img_output && step_number >= 0) {
                char filename[100];
                sprintf(filename, "%s/img_%03d.jpg", img_dir.c_str(), render_frame + 1);
                app.WriteImageToFile(filename);
            }

            render_frame++;
        }

        // Update vehicle
        double time = system->GetChTime();
        vehicle->Synchronize(time);
        app.Synchronize("", driver_cb.GetSteering(), driver_cb.GetThrottle(), driver_cb.GetBraking());

        // Advance dynamics
        double step = realtime_timer.SuggestSimulationStep(step_size);
        system->DoStepDynamics(step);
        app.Advance(step);


        // Increment frame number
        step_number++;
    }

    // Cleanup
    delete vehicle;
    delete tire_cb;

    return 0;
}
