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
// Gator acceleration test.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/gator/Gator.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::gator;

// =============================================================================

// Initial vehicle location and orientation (m)
ChVector<> initLoc(-40, 0, 0.5);
ChQuaternion<> initRot(1, 0, 0, 0);

// Brake type (SIMPLE or SHAFTS)
BrakeType brake_type = BrakeType::SHAFTS;

// Terrain slope (radians)
double slope = 20 * CH_C_DEG_TO_RAD;

// Set speed (m/s)
double target_speed = 4;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Simulation step sizes
double step_size = 1e-3;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create systems
    // --------------

    // Create the Gator vehicle, set parameters, and initialize
    Gator gator;
    gator.SetContactMethod(ChContactMethod::NSC);
    gator.SetChassisCollisionType(CollisionType::NONE);
    gator.SetChassisFixed(false);
    gator.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    gator.SetBrakeType(brake_type);
    gator.SetTireType(TireModelType::TMEASY);
    gator.SetTireStepSize(step_size);
    gator.SetAerodynamicDrag(0.5, 5.0, 1.2);
    gator.EnableBrakeLocking(true);
    gator.Initialize();

    gator.SetChassisVisualizationType(chassis_vis_type);
    gator.SetSuspensionVisualizationType(suspension_vis_type);
    gator.SetSteeringVisualizationType(steering_vis_type);
    gator.SetWheelVisualizationType(wheel_vis_type);
    gator.SetTireVisualizationType(tire_vis_type);

    // Create the terrain
    RigidTerrain terrain(gator.GetSystem());

    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(ChContactMethod::NSC);

    auto patch1 = terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector<>(-25, 0, 0), QUNIT), 50.0, 20.0);
    patch1->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 40);
    patch1->SetColor(ChColor(0.8f, 0.8f, 0.5f));

    double s = std::sin(slope);
    double c = std::cos(slope);
    auto patch2 =
        terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector<>(100 * c, 0, 100 * s), Q_from_AngY(-slope)), 200.0, 20.0);
    patch2->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 40);
    patch2->SetColor(ChColor(0.8f, 0.5f, 0.8f));

    auto patch3 = terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector<>(200 * c + 25, 0, 200 * s), QUNIT), 50.0, 20.0);
    patch3->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 40);
    patch3->SetColor(ChColor(0.8f, 0.8f, 0.5f));

    terrain.Initialize();

    // Create the straight path and the driver system
    auto path = StraightLinePath(ChVector<>(-50, 0, 0.5), ChVector<>(300, 0, 0.5), 1);
    ChPathFollowerDriver driver(gator.GetVehicle(), path, "my_path", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.6, 0.4, 0.4);
    driver.Initialize();

    // Create the vehicle Irrlicht interface
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Gator Acceleration");
    vis->SetChaseCamera(ChVector<>(0.0, 0.0, 2.0), 5.0, 0.05);
    vis->Initialize();
    vis->AddLightDirectional();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&gator.GetVehicle());

    // ---------------
    // Simulation loop
    // ---------------

    gator.GetVehicle().LogSubsystemTypes();
    std::cout << "\nVehicle mass: " << gator.GetVehicle().GetMass() << std::endl;

    // Initialize simulation frame counters
    int step_number = 0;

    gator.GetVehicle().EnableRealtime(true);
    while (vis->Run()) {
        double time = gator.GetSystem()->GetChTime();

        // Render scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Get driver inputs
        DriverInputs driver_inputs = driver.GetInputs();
        if (time > 13) {
            driver_inputs.m_braking = 1;
            driver_inputs.m_throttle = 0;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        gator.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        gator.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;
    }

    return 0;
}
