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

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_models/vehicle/gator/Gator.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::gator;

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Initial vehicle location and orientation (m)
ChVector3d initLoc(-40, 0, 0.5);
ChQuaternion<> initRot(1, 0, 0, 0);

// Brake type (SIMPLE or SHAFTS)
BrakeType brake_type = BrakeType::SHAFTS;

// Terrain slope (radians)
double slope = 20 * CH_DEG_TO_RAD;

// Set speed (m/s)
double target_speed = 4;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Simulation step sizes
double step_size = 2e-3;

// End time (used only if no run-time visualization)
double t_end = 20;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // --------------
    // Create systems
    // --------------

    // Create the Gator vehicle, set parameters, and initialize
    Gator gator;
    gator.SetContactMethod(ChContactMethod::SMC);
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

    // Associate a collision system
    gator.GetSystem()->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the terrain
    RigidTerrain terrain(gator.GetSystem());

    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(ChContactMethod::NSC);

    auto patch1 = terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector3d(-25, 0, 0), QUNIT), 200.0, 100.0);
    patch1->SetTexture(GetVehicleDataFile("terrain/textures/tile4.jpg"), 100, 50);

    double s = std::sin(slope);
    double c = std::cos(slope);
    auto patch2 =
        terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector3d(100 * c, 0, 100 * s), QuatFromAngleY(-slope)), 200.0, 20.0);
    patch2->SetTexture(GetVehicleDataFile("terrain/textures/tile4.jpg"), 100, 10);

    terrain.Initialize();

    // Create the straight path and the driver system
    auto path = StraightLinePath(ChVector3d(-50, 0, 0.5), ChVector3d(300, 0, 0.5), 1);
    ChPathFollowerDriver driver(gator.GetVehicle(), path, "my_path", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.6, 0.4, 0.4);
    driver.Initialize();

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

    std::shared_ptr<ChVehicleVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            // Create the vehicle Irrlicht interface
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle("Gator Incline Stop");
            vis_irr->SetChaseCamera(ChVector3d(0.0, 0.0, 2.0), 5.0, 0.05);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AttachVehicle(&gator.GetVehicle());

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            // Create the vehicle VSG interface
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle("Gator Incline Stop");
            vis_vsg->AttachVehicle(&gator.GetVehicle());
            vis_vsg->AttachTerrain(&terrain);
            vis_vsg->SetChaseCamera(ChVector3d(0.0, 0.0, 2.0), 9.0, 0.05);
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowPosition(100, 100);
            vis_vsg->EnableSkyBox();
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // ---------------
    // Simulation loop
    // ---------------

    gator.GetVehicle().LogSubsystemTypes();
    std::cout << "\nVehicle mass: " << gator.GetVehicle().GetMass() << std::endl;

    // Initialize simulation frame counters
    int step_number = 0;
    gator.GetVehicle().EnableRealtime(true);

    while (true) {
        double time = gator.GetSystem()->GetChTime();

        if (vis) {
            // Render scene
            if (!vis->Run())
                break;
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        } else if (time > t_end) {
            break;
        }

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
        if (vis)
            vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        gator.Advance(step_size);
        if (vis)
            vis->Advance(step_size);

        // Increment frame number
        step_number++;
    }

    return 0;
}
