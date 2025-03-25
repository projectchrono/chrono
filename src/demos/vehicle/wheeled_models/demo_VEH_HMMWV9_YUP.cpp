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

#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

#define USE_PATH_FOLLOWER

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

// Initial vehicle location and orientation
ChVector3d initLoc(0, 1, 10);
double initYaw = 0;

// Type of tire model (RIGID, RIGID_MESH, TMEASY, PAC89, PAC02, FIALA)
TireModelType tire_model = TireModelType::TMEASY;

// Simulation step sizes
double step_size = 3e-3;
double tire_step_size = 1e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

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
    HMMWV_Reduced hmmwv;
    hmmwv.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    hmmwv.SetContactMethod(ChContactMethod::NSC);
    hmmwv.SetChassisFixed(false);
    hmmwv.SetChassisCollisionType(CollisionType::NONE);
    hmmwv.SetInitPosition(ChCoordsys<>(initLoc, QuatFromAngleY(initYaw)));
    hmmwv.SetEngineType(EngineModelType::SIMPLE);
    hmmwv.SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    hmmwv.SetDriveType(DrivelineTypeWV::RWD);
    hmmwv.SetTireType(tire_model);
    ////hmmwv.SetTireCollisionType(ChTire::CollisionType::ENVELOPE);
    hmmwv.SetTireStepSize(tire_step_size);
    hmmwv.Initialize();

    hmmwv.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetWheelVisualizationType(VisualizationType::MESH);
    hmmwv.SetTireVisualizationType(VisualizationType::MESH);

    // ------------------
    // Create the terrain
    // ------------------

    RigidTerrain terrain(hmmwv.GetSystem());

    auto patch_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);

    // "Box" patch
    double terrainHeight = 0;
    double terrainLength = 300.0;  // size in "forward" direction
    double terrainWidth = 300.0;   // size in "lateral" direction
    auto patch = terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector3d(0, terrainHeight, 0), QuatFromAngleX(-CH_PI_2)),
                                  terrainLength, terrainWidth);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);

    ////// "Mesh" patch (mesh assumed to be defined in a Y up frame)
    ////auto patch = terrain.AddPatch(patch_mat, ChCoordsys<>(), vehicle::GetDataFile("terrain/meshes/bump_YUP.obj"),
    ////                              "ground", 0.005);
    ////patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);

    ////// "Height-field map" patch (extents are in the forward and lateral directions of the world frame)
    ////auto patch = terrain.AddPatch(patch_mat, ChCoordsys<>(), vehicle::GetDataFile("terrain/height_maps/bump64.bmp"),
    ////                              "field_mesh", 64.0, 64.0, 0.0, 3.0);
    ////patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 6.0f, 6.0f);

    terrain.Initialize();

#ifdef USE_PATH_FOLLOWER
    // Path follower driver
    ////auto path = DoubleLaneChangePath(initLoc, 13.5, 4.0, 11.0, 20.0, true);
    ////auto path = CirclePath(initLoc, 20, 50, true, 5);

    std::vector<ChVector3d> points = {
        initLoc + ChVector3d(0, 0.2, 0),    //
        initLoc + ChVector3d(20, 0.2, 0),   //
        initLoc + ChVector3d(20, 0.2, 20),  //
        initLoc + ChVector3d(0, 0.2, 20)    //
    };
    auto path = chrono_types::make_shared<ChBezierCurve>(points, true);

    ChPathFollowerDriver driver(hmmwv.GetVehicle(), path, "my_path", 10);
    driver.GetSteeringController().SetLookAheadDistance(5);
    driver.GetSteeringController().SetGains(0.8, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();
#elif
    // Interactive driver
    ChInteractiveDriver driver(hmmwv.GetVehicle());
    driver.SetSteeringDelta(0.06);
    driver.SetThrottleDelta(0.02);
    driver.SetBrakingDelta(0.06);
    driver.Initialize();
#endif

    // Vehicle Irrlicht run-time visualization
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("HMMWV-9 YUP Demo");
    vis->SetCameraVertical(CameraVerticalDir::Y);
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->Initialize();
    vis->AddLightDirectional(-60, 300);
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AddGrid(1.0, 1.0, 20, 20, ChCoordsys<>(ChVector3d(0, 0.01, 0), ChWorldFrame::Quaternion()), ChColor(1, 0, 0));
    vis->AttachVehicle(&hmmwv.GetVehicle());
    vis->AttachDriver(&driver);

#ifdef USE_PATH_FOLLOWER
    // Visualization of controller points (sentinel & target)
    auto ballS = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
    auto ballT = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
    ballS->SetColor(ChColor(1, 0, 0));
    ballT->SetColor(ChColor(0, 1, 0));
    int iballS = vis->AddVisualModel(ballS, ChFrame<>());
    int iballT = vis->AddVisualModel(ballT, ChFrame<>());
#endif

    // ---------------
    // Simulation loop
    // ---------------

    int render_steps = (int)std::ceil(render_step_size / step_size);
    int step_number = 0;

    hmmwv.GetVehicle().EnableRealtime(true);
    utils::ChRunningAverage RTF_filter(50);

    while (vis->Run()) {
        double time = hmmwv.GetSystem()->GetChTime();

#ifdef USE_PATH_FOLLOWER
        vis->UpdateVisualModel(iballS, ChFrame<>(driver.GetSteeringController().GetSentinelLocation()));
        vis->UpdateVisualModel(iballT, ChFrame<>(driver.GetSteeringController().GetTargetLocation()));
#endif

        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->RenderFrame(ChFrame<>(), 10);
            vis->EndScene();
        }

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        hmmwv.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        hmmwv.Advance(step_size);
        vis->Advance(step_size);
    }

    return 0;
}
