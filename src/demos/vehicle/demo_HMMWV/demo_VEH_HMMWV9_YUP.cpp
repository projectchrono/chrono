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
// Demonstration of a Chrono::Vehicle simulation in a non-ISO frame.
// The world frame has Y up, X forward, and Z pointing to the right.
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"

#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0, 1, 10);
double initYaw = 0;
////double initYaw = CH_C_PI / 4;  // 45 deg towards vehicle's left

// Type of tire model (RIGID, RIGID_MESH, TMEASY, PACEJKA, PAC89, FIALA)
TireModelType tire_model = TireModelType::TMEASY;

// Simulation step sizes
double step_size = 3e-3;
double tire_step_size = 1e-3;

// =============================================================================

class MyDriver : public ChDriver {
  public:
    MyDriver(ChVehicle& vehicle) : ChDriver(vehicle) {} 
    virtual void Synchronize(double time) override {
        m_throttle = 0;
        m_steering = 0;
        m_braking = 0;
        if (time > 1) {
            m_throttle = 0.25;
        }
        if (time > 2) {
            m_steering = 0.3;
        }
    }
};

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -------------------------
    // Set World Frame with Y up
    // -------------------------
    std::cout << "World Frame [default]\n" << ChWorldFrame::Rotation() << std::endl;
    ChWorldFrame::SetYUP();
    std::cout << "World Frame [new]\n" << ChWorldFrame::Rotation() << std::endl;
    std::cout << "Vertical direction: " << ChWorldFrame::Vertical() << std::endl;
    std::cout << "Forward direction:  " << ChWorldFrame::Forward() << std::endl;

    // --------------
    // Create vehicle
    // --------------

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Reduced my_hmmwv;
    my_hmmwv.SetContactMethod(ChContactMethod::NSC);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetChassisCollisionType(ChassisCollisionType::NONE);
    my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc, Q_from_AngY(initYaw)));
    my_hmmwv.SetPowertrainType(PowertrainModelType::SIMPLE);
    my_hmmwv.SetDriveType(DrivelineType::RWD);
    my_hmmwv.SetTireType(tire_model);
    my_hmmwv.SetTireCollisionType(ChTire::CollisionType::ENVELOPE);
    my_hmmwv.SetTireStepSize(tire_step_size);
    my_hmmwv.Initialize();

    my_hmmwv.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetWheelVisualizationType(VisualizationType::MESH);
    my_hmmwv.SetTireVisualizationType(VisualizationType::MESH);

    // ------------------
    // Create the terrain
    // ------------------

    RigidTerrain terrain(my_hmmwv.GetSystem());

    auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);

    // "Box" patch
    double terrainHeight = 0;
    double terrainLength = 200.0;  // size in "forward" direction
    double terrainWidth = 100.0;   // size in "lateral" direction
    auto patch =
        terrain.AddPatch(patch_mat, ChVector<>(0, terrainHeight, 0), ChVector<>(0, 1, 0), terrainLength, terrainWidth);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);

    ////// "Meash" patch (mesh assumed to be defined in a Y up frame)
    ////auto patch = terrain.AddPatch(patch_mat, ChCoordsys<>(), vehicle::GetDataFile("terrain/meshes/bump_YUP.obj"),
    ////                              "ground", 0.005);
    ////patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);

    ////// "Height-field map" patch (extents are in the forward and lateral directions of the world frame)
    ////auto patch = terrain.AddPatch(patch_mat, ChCoordsys<>(), vehicle::GetDataFile("terrain/height_maps/bump64.bmp"),
    ////                              "field_mesh", 64.0, 64.0, 0.0, 3.0);
    ////patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 6.0f, 6.0f);

    terrain.Initialize();

    // ----------------------------------------
    // Create the run-time visualization system
    // ----------------------------------------

    ////// "Standard" Irrlicht run-time visualization
    ////ChIrrApp app(my_hmmwv.GetSystem(), L"HMMWV-9 YUP", irr::core::dimension2d<irr::u32>(1000, 800), false, true);
    ////app.AddTypicalLogo();
    ////app.AddTypicalSky();
    ////app.AddTypicalLights();
    ////app.AddTypicalCamera(irr::core::vector3df(-1, 3, -5), irr::core::vector3dfCH(&initLoc));

    // Vehicle Irrlicht run-time visualization
    ChWheeledVehicleIrrApp app(&my_hmmwv.GetVehicle(), L"HMMWV-9 YUP", irr::core::dimension2d<irr::u32>(1000, 800),
                               irr::ELL_NONE);
    app.SetSkyBox();
    app.AddTypicalLights();
    app.SetChaseCamera(ChVector<>(0.0, 0.0, .75), 6.0, 0.5);
    app.SetTimestep(step_size);

    app.AssetBindAll();
    app.AssetUpdateAll();

    // ------------------------
    // Create the driver system
    // ------------------------

    ////// Scripted driver inputs (custom driver system) 
    ////MyDriver driver(my_hmmwv.GetVehicle());
    ////driver.Initialize();

    // Interactive driver
    ChIrrGuiDriver driver(app);
    driver.SetSteeringDelta(0.06);
    driver.SetThrottleDelta(0.02);
    driver.SetBrakingDelta(0.06);
    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------

    ChRealtimeStepTimer realtime_timer;
    while (app.GetDevice()->run()) {
        double time = my_hmmwv.GetSystem()->GetChTime();

        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.RenderFrame(ChVector<>(0, 0, 0), 10);
        app.RenderGrid(ChVector<>(0, 0.01, 0), 20, 1.0);
        app.EndScene();

        // Driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, driver_inputs, terrain);
        app.Synchronize("", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        my_hmmwv.Advance(step_size);
        app.Advance(step_size);

        // Spin in place for real time to catch up
        realtime_timer.Spin(step_size);
    }

    return 0;
}
