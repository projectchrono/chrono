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
// Demonstration of using a RigidTerrain constructed from different patches.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

//#define USE_JSON

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create systems
    // --------------

    // Simulation step sizes
    double step_size = 3e-3;
    double tire_step_size = 1e-3;

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Reduced my_hmmwv;
    my_hmmwv.SetContactMethod(ChContactMethod::NSC);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetChassisCollisionType(CollisionType::NONE);
    my_hmmwv.SetInitPosition(ChCoordsys<>(ChVector<>(-10, 0, 1), ChQuaternion<>(1, 0, 0, 0)));
    my_hmmwv.SetPowertrainType(PowertrainModelType::SIMPLE);
    my_hmmwv.SetDriveType(DrivelineTypeWV::RWD);
    my_hmmwv.SetTireType(TireModelType::TMEASY);
    my_hmmwv.SetTireStepSize(tire_step_size);
    my_hmmwv.Initialize();

    my_hmmwv.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetWheelVisualizationType(VisualizationType::NONE);
    my_hmmwv.SetTireVisualizationType(VisualizationType::PRIMITIVES);

#ifdef USE_JSON
    // Create the terrain from JSON specification file 
    RigidTerrain terrain(my_hmmwv.GetSystem(), vehicle::GetDataFile("terrain/RigidPatches.json"), true);
#else
    // Create the terrain patches programatically
    RigidTerrain terrain(my_hmmwv.GetSystem());

    auto patch1_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    patch1_mat->SetFriction(0.9f);
    patch1_mat->SetRestitution(0.01f);
    auto patch1 = terrain.AddPatch(patch1_mat, ChCoordsys<>(ChVector<>(-16, 0, 0), QUNIT), 32, 20);
    patch1->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch1->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 20, 20);

    auto patch2_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    patch2_mat->SetFriction(0.9f);
    patch2_mat->SetRestitution(0.01f);
    auto patch2 = terrain.AddPatch(patch1_mat, ChCoordsys<>(ChVector<>(16, 0, 0.15), QUNIT), 32, 30);
    patch2->SetColor(ChColor(1.0f, 0.5f, 0.5f));
    patch2->SetTexture(vehicle::GetDataFile("terrain/textures/concrete.jpg"), 20, 20);

    auto patch3_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    patch3_mat->SetFriction(0.9f);
    patch3_mat->SetRestitution(0.01f);
    auto patch3 = terrain.AddPatch(patch3_mat, ChCoordsys<>(ChVector<>(0, -42, 0), QUNIT),
                                   vehicle::GetDataFile("terrain/meshes/bump.obj"));
    patch3->SetColor(ChColor(0.5f, 0.5f, 0.8f));
    patch3->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), 6.0f, 6.0f);

    auto patch4_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    patch4_mat->SetFriction(0.9f);
    patch4_mat->SetRestitution(0.01f);
    auto patch4 =
        terrain.AddPatch(patch4_mat, ChCoordsys<>(ChVector<>(0, 42, 0), QUNIT),
                         vehicle::GetDataFile("terrain/height_maps/bump64.bmp"), 64.0, 64.0, 0.0, 3.0);
    patch4->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 6.0f, 6.0f);

    terrain.Initialize();
#endif

    // Create the vehicle Irrlicht interface
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Rigid Terrain Demo");
    vis->SetChaseCamera(ChVector<>(0.0, 0.0, .75), 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    my_hmmwv.GetVehicle().SetVisualSystem(vis);

    // Create the interactive driver system
    ChIrrGuiDriver driver(*vis);
    driver.Initialize();

    // Set the time response for steering and throttle keyboard inputs.
    double render_step_size = 1.0 / 50;  // FPS = 50

    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    ////utils::CSV_writer out(" ");
    ////for (int ix = 0; ix < 20; ix++) {
    ////    double x = ix * 1.0;
    ////    for (int iy = 0; iy < 100; iy++) {
    ////        double y = iy * 1.0;
    ////        double z = terrain.CalcHeight(x, y);
    ////        out << x << y << z << std::endl;
    ////    }
    ////}
    ////out.write_to_file("terrain.out");

    // ---------------
    // Simulation loop
    // ---------------

    my_hmmwv.GetVehicle().EnableRealtime(true);
    while (vis->Run()) {
        double time = my_hmmwv.GetSystem()->GetChTime();

        // Render scene
        vis->BeginScene();
        vis->DrawAll();
        vis->EndScene();

        // Get driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(driver.GetInputModeAsString(), driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        my_hmmwv.Advance(step_size);
        vis->Advance(step_size);
    }

    return 0;
}
