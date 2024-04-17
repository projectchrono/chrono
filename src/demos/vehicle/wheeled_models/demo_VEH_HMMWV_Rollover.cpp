// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Illustration of using collision geometry for the vehicle chassis:
// - rollover if no obstacle present
// - frontal contact with a cylindrical obstacle otherwise
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/ChWorldFrame.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/driver/ChInteractiveDriverVSG.h"
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

// Chassis collision model
auto chassis_coll_type = CollisionType::HULLS;

// Obstacle collision model (nbo obstacle if CollisionType::NONE)
auto obstacle_coll_type = CollisionType::NONE;

// Contact formulation
auto contact_method = ChContactMethod::NSC;

// Collision system
auto collision_system_type = ChCollisionSystem::Type::BULLET;

// Run-time visualization system (IRRLICHT or VSG)
auto vis_type = ChVisualSystem::Type::VSG;

// Simulation step size
double step_size = 2e-3;

// End time (used only if no run-time visualization)
double t_end = 20;

// =============================================================================

void AddObstacle(ChSystem* sys);

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create vehicle
    HMMWV_Full hmmwv;
    hmmwv.SetCollisionSystemType(collision_system_type);
    hmmwv.SetContactMethod(contact_method);
    hmmwv.SetChassisCollisionType(chassis_coll_type);
    hmmwv.SetInitPosition(ChCoordsys<>(ChVector3d(0, 0, 0.5), QUNIT));
    hmmwv.SetEngineType(EngineModelType::SHAFTS);
    hmmwv.SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    hmmwv.SetDriveType(DrivelineTypeWV::AWD);
    hmmwv.UseTierodBodies(false);
    hmmwv.SetSteeringType(SteeringTypeWV::PITMAN_ARM);
    hmmwv.SetBrakeType(BrakeType::SHAFTS);
    hmmwv.SetTireType(TireModelType::TMEASY);
    hmmwv.SetTireStepSize(step_size);
    hmmwv.Initialize();

    hmmwv.SetChassisVisualizationType(VisualizationType::MESH);
    hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetWheelVisualizationType(VisualizationType::MESH);
    hmmwv.SetTireVisualizationType(VisualizationType::MESH);

    // Optionally, enable collision for the vehicle wheels.
    // In this case, you must also disable collision between the chassis and wheels (unless the chassis collision model
    // is accurate enough to account for the wheel wells).
    ////hmmwv.GetVehicle().SetWheelCollide(true);
    ////hmmwv.GetVehicle().SetChassisVehicleCollide(false);

    // Create the terrain
    RigidTerrain terrain(hmmwv.GetSystem());

    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.1f;
    minfo.Y = 2e7f;
    auto terrain_mat = minfo.CreateMaterial(hmmwv.GetSystem()->GetContactMethod());

    auto patch = terrain.AddPatch(terrain_mat, CSYSNORM, 100.0, 100.0);
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), 20, 20);

    auto slope = QuatFromAngleY(-15 * CH_DEG_TO_RAD);
    auto ramp = terrain.AddPatch(terrain_mat, ChCoordsys<>(ChVector3d(20, 3, 0), slope), 20, 6);
    ramp->SetTexture(vehicle::GetDataFile("terrain/textures/concrete.jpg"), 2, 2);

    terrain.Initialize();

    // Add a mesh obstacle
    AddObstacle(hmmwv.GetSystem());

    // Create the vehicle run-time visualization

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
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle("Rollover Demo");
            vis_irr->SetChaseCamera(ChVector3d(0.0, 0.0, 2.0), 5.0, 0.05);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional(70, 20);
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AttachVehicle(&hmmwv.GetVehicle());
            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle("Rollover Demo");
            vis_vsg->SetWindowSize(ChVector2i(1200, 800));
            vis_vsg->SetWindowPosition(ChVector2i(100, 300));
            vis_vsg->AttachVehicle(&hmmwv.GetVehicle());
            vis_vsg->SetChaseCamera(ChVector3d(0.0, 0.0, 2.0), 8.0, 0.3);
            vis_vsg->SetUseSkyBox(true);
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->SetShadows(true);
            vis_vsg->Initialize();
            vis = vis_vsg;
#endif
            break;
        }
    }

    // Simulation loop
    hmmwv.GetVehicle().EnableRealtime(true);

    while (true) {
        double time = hmmwv.GetSystem()->GetChTime();

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

        // Set driver inputs
        DriverInputs driver_inputs = {0, 0.5, 0};

        // Check rollover -- detach chase camera
        if (Vdot(hmmwv.GetChassisBody()->GetRotMat().GetAxisZ(), ChWorldFrame::Vertical()) < 0) {
            auto camera = vis->GetChaseCamera();
            auto camera_pos = vis->GetCameraPosition();
            camera_pos.z() = 1;
            camera.SetCameraPos(camera_pos);
            vis->SetChaseCameraState(utils::ChChaseCamera::Free);
        }

        // Update modules (process inputs from other modules)
        terrain.Synchronize(time);
        hmmwv.Synchronize(time, driver_inputs, terrain);
        if (vis)
            vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        terrain.Advance(step_size);
        hmmwv.Advance(step_size);
        if (vis)
            vis->Advance(step_size);
    }

    return 0;
}

void AddObstacle(ChSystem* sys) {
    if (obstacle_coll_type == CollisionType::NONE)
        return;

    auto pos = ChVector3d(8, 0, 1);
    double radius = 1;
    double length = 2;

    std::string text_filename = "textures/rock.jpg";

    auto body = chrono_types::make_shared<ChBody>();
    body->SetPos(pos);
    body->SetFixed(true);
    body->EnableCollision(true);
    sys->Add(body);

    std::shared_ptr<ChContactMaterial> mat = ChContactMaterial::DefaultMaterial(sys->GetContactMethod());

    if (obstacle_coll_type == CollisionType::PRIMITIVES) {
        auto ct_shape = chrono_types::make_shared<ChCollisionShapeCylinder>(mat, radius, length);
        body->AddCollisionShape(ct_shape);

        auto vis_shape = chrono_types::make_shared<ChVisualShapeCylinder>(radius, length);
        vis_shape->SetTexture(GetChronoDataFile(text_filename));
        body->AddVisualShape(vis_shape);

        return;
    }

    auto mesh =
        ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("models/cylinderZ.obj"), false, true);
    mesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(ChVector3d(radius, radius, length)));

    if (obstacle_coll_type == CollisionType::MESH) {
        auto ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(mat, mesh, false, false, 0.005);
        body->AddCollisionShape(ct_shape);
    } else {
        auto ct_shape = chrono_types::make_shared<ChCollisionShapeConvexHull>(mat, mesh->GetCoordsVertices());
        body->AddCollisionShape(ct_shape);
    }

    auto vis_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    vis_shape->SetMesh(mesh);
    vis_shape->SetBackfaceCull(true);
    vis_shape->SetTexture(GetChronoDataFile(text_filename));
    body->AddVisualShape(vis_shape);
}
