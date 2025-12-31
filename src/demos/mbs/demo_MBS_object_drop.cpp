// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// Simple demo for contact between a primitive object and a plate (NSC or SMC)
//
// =============================================================================

#include "chrono/physics/ChSystem.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChBodyGeometry.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;

// -----------------------------------------------------------------------------

ChContactMethod contact_method = ChContactMethod::SMC;
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;
ChCollisionSystem::Type coll_type = ChCollisionSystem::Type::BULLET;

enum class ObjectType {SPHERE, CYLINDER, CONE};
ObjectType object_type = ObjectType::SPHERE;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2025 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Simulation parameters
    double gravity = -9.81;
    double time_step = contact_method == ChContactMethod::NSC ? 1e-3 : 1e-5;
    double render_fps = 100;

    // Parameters for the falling object
    double radius = 1;
    double height = 1;
    double mass = 100;

    ChVector3d pos(0, 2, 0);
    ChQuaterniond rot = QuatFromAngleZ(CH_PI / 6);
    ChVector3d init_vel(0, 0, 0);

    // Parameters for the plate
    double side = 4;
    double thickness = 0.2;

    // Create the system
    auto sys = ChSystem::Create(contact_method);

    sys->SetGravitationalAcceleration(ChVector3d(0, gravity, 0));
    sys->SetCollisionSystemType(coll_type);

    // Change the default collision effective radius of curvature (SMC only)
    ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(1);

    // Create a material (will be used by both collision shapes)
    ChContactMaterialData mat_data;
    mat_data.mu = 0.4f;
    mat_data.cr = 0.1f;

    // Create the falling body
    {
        ChMatrix33d inertia;
        utils::ChBodyGeometry geometry;
        geometry.materials.push_back(mat_data);

        switch (object_type) {
            case ObjectType::SPHERE: {
                auto sphere = utils::ChBodyGeometry::SphereShape(VNULL, radius, 0);
                sphere.color = ChColor(0.1f, 0.5f, 0.9f);
                geometry.coll_spheres.push_back(sphere);
                inertia = mass * ChSphere::CalcGyration(radius);
                break;
            }
            case ObjectType::CYLINDER: {
                auto cylinder = utils::ChBodyGeometry::CylinderShape(VNULL, VECT_Y, radius, height, 0);
                cylinder.color = ChColor(0.1f, 0.5f, 0.9f);
                geometry.coll_cylinders.push_back(cylinder);
                inertia = mass * ChCylinder::CalcGyration(radius, height);
                break;
            }
            case ObjectType::CONE: {
                auto cone = utils::ChBodyGeometry::ConeShape(ChVector3d(0, 0, 0), VECT_Y, radius, height, 0);
                cone.color = ChColor(0.1f, 0.5f, 0.9f);
                geometry.coll_cones.push_back(cone);
                inertia = mass * ChCone::CalcGyration(radius, height);
                break;
            }
        }

        auto body = chrono_types::make_shared<ChBody>();
        body->SetName("Object");
        body->SetMass(mass);
        body->SetInertia(inertia);
        body->SetPos(pos);
        body->SetRot(rot);
        body->SetPosDt(init_vel);
        body->SetFixed(false);

        geometry.CreateCollisionShapes(body, 0, contact_method);
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        sys->AddBody(body);
    }

    // Create the plate
    {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetName("Plate");
        body->SetMass(1);
        body->SetPos(ChVector3d(0, 0, 0));
        body->SetRot(ChQuaternion<>(1, 0, 0, 0));
        body->SetFixed(true);

        utils::ChBodyGeometry geometry;
        geometry.materials.push_back(mat_data);
        geometry.coll_boxes.push_back(
            utils::ChBodyGeometry::BoxShape(VNULL, QUNIT, ChVector3d(side, thickness, side), 0));

        geometry.CreateCollisionShapes(body, 0, contact_method);
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        sys->AddBody(body);
    }

    // Create the run-time visualization system
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("Object drop");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddTypicalLights();
            vis_irr->AddCamera(ChVector3d(0, 3, -6));
            vis_irr->AttachSystem(sys.get());
            vis_irr->AddGrid(0.2, 0.2, 20, 20, ChCoordsys<>(ChVector3d(0, 0.105, 0), QuatFromAngleX(CH_PI_2)),
                             ChColor(0.1f, 0.1f, 0.1f));

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(sys.get());
            vis_vsg->SetWindowTitle("Object drop");
            vis_vsg->AddCamera(ChVector3d(0, 3, -6));
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowPosition(100, 100);
            vis_vsg->SetCameraVertical(CameraVerticalDir::Y);
            vis_vsg->SetCameraAngleDeg(40.0);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->AddGrid(0.2, 0.2, 20, 20, ChCoordsys<>(ChVector3d(0, 0.105, 0), QuatFromAngleX(CH_PI_2)),
                             ChColor(0.1f, 0.1f, 0.1f));
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Simulation loop
    ChRealtimeStepTimer rt_timer;
    double time = 0.0;
    int render_frame = 0;

    while (vis->Run()) {
        if (time > render_frame / render_fps) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
            render_frame++;
        }

        sys->DoStepDynamics(time_step);
        rt_timer.Spin(time_step);
        time += time_step;
    }

    return 0;
}
