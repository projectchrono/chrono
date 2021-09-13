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

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/gator/Gator.h"
#include "chrono_models/vehicle/gator/Gator_SimplePowertrain.h"

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

// Terrain slope (degrees)
double slope = 20;

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

    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(ChContactMethod::NSC);

    auto patch1 = terrain.AddPatch(patch_mat, ChVector<>(-25, 0, 0), ChVector<>(0, 0, 1), 50.0, 20.0);
    patch1->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 40);
    patch1->SetColor(ChColor(0.8f, 0.8f, 0.5f));

    double s = std::sin(slope * CH_C_DEG_TO_RAD);
    double c = std::cos(slope * CH_C_DEG_TO_RAD);
    auto patch2 = terrain.AddPatch(patch_mat, ChVector<>(100 * c, 0, 100 * s), ChVector<>(-s, 0, c), 200.0, 20.0);
    patch2->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 40);
    patch2->SetColor(ChColor(0.8f, 0.5f, 0.8f));

    auto patch3 = terrain.AddPatch(patch_mat, ChVector<>(200 * c + 25, 0, 200 * s), ChVector<>(0, 0, 1), 50.0, 20.0);
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
    ChWheeledVehicleIrrApp app(&gator.GetVehicle(), L"Gator Acceleration");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 2.0), 5.0, 0.05);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // ---------------
    // Simulation loop
    // ---------------

    gator.GetVehicle().LogSubsystemTypes();
    std::cout << "\nVehicle mass: " << gator.GetTotalMass() << std::endl;

    // Initialize simulation frame counters
    int step_number = 0;

    ChRealtimeStepTimer realtime_timer;
    while (app.GetDevice()->run()) {
        double time = gator.GetSystem()->GetChTime();

        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();

        // Get driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();
        if (gator.GetVehicle().GetVehiclePos().x() > 4) {
            driver_inputs.m_braking = 1;
            driver_inputs.m_throttle = 0;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        gator.Synchronize(time, driver_inputs, terrain);
        app.Synchronize("", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        gator.Advance(step_size);
        app.Advance(step_size);

        // Increment frame number
        step_number++;

        // Spin in place for real time to catch up
        realtime_timer.Spin(step_size);
    }

    return 0;
}
