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

#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;

// -----------------------------------------------------------------------------

ChContactMethod contact_method = ChContactMethod::SMC;
ChCollisionSystem::Type coll_type = ChCollisionSystem::Type::BULLET;

// Supported primitives: box, cone, cylinder, rounded box, rounded cylinder, sphere
ChVisualShape::Type object_type = ChVisualShape::Type::SPHERE;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2025 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Simulation parameters
    double gravity = -9.81;
    double time_step = contact_method == ChContactMethod::NSC ? 1e-3 : 1e-5;
    double render_fps = 100;

    // Parameters for the falling object
    double radius = 0.75;
    double height = 1;
    double sradius = 0.2;

    double mass = 2000;

    ChVector3d pos(0, 2, 0);
    ChQuaterniond rot = QuatFromAngleZ(CH_PI / 3);
    ChVector3d init_lin_vel(0, 0, 0);
    ChVector3d init_ang_vel(0, 0, 0);

    // Parameters for the plate
    double side = 8;
    double thickness = 0.2;

    // Create the system
    auto sys = ChSystem::Create(contact_method);

    sys->SetGravitationalAcceleration(ChVector3d(0, gravity, 0));
    sys->SetCollisionSystemType(coll_type);

    // Change the default collision effective radius of curvature (SMC only)
    ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(0.1 * radius);

    // Create a shared contact material
    ChContactMaterialData ct_mat_data;
    ct_mat_data.Y = 2e7f;
    ct_mat_data.mu = 1.0f;
    ct_mat_data.cr = 0.01f;
    auto ct_mat = ct_mat_data.CreateMaterial(contact_method);

    // Create the falling body
    {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetName("Object");
        body->SetFixed(false);
        body->EnableCollision(true);
        body->SetMass(mass);
        body->SetPos(pos);
        body->SetRot(rot);
        body->SetPosDt(init_lin_vel);
        body->SetAngVelLocal(init_ang_vel);

        ChMatrix33d inertia;

        // Create a visualization material
        auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
        vis_mat->SetDiffuseColor(ChColor(0.68f, 0.92f, 0.70f));
        ////vis_mat->SetOpacity(0.75f);

        switch (object_type) {
            case ChVisualShape::Type::BOX: {
                ChVector3d dims(radius, height, radius);

                auto box_vis = chrono_types::make_shared<ChVisualShapeBox>(dims);
                box_vis->AddMaterial(vis_mat);
                body->AddVisualShape(box_vis);

                auto box_ct = chrono_types::make_shared<ChCollisionShapeBox>(ct_mat, dims);
                body->AddCollisionShape(box_ct);

                inertia = mass * ChBox::CalcGyration(dims);

                break;
            }
            case ChVisualShape::Type::SPHERE: {
                auto sphere_vis = chrono_types::make_shared<ChVisualShapeSphere>(radius);
                sphere_vis->AddMaterial(vis_mat);
                body->AddVisualShape(sphere_vis);

                auto sphere_ct = chrono_types::make_shared<ChCollisionShapeSphere>(ct_mat, radius);
                body->AddCollisionShape(sphere_ct);

                inertia = mass * ChSphere::CalcGyration(radius);

                break;
            }
            case ChVisualShape::Type::CYLINDER: {
                auto cyl_vis = chrono_types::make_shared<ChVisualShapeCylinder>(radius, height);
                cyl_vis->AddMaterial(vis_mat);
                body->AddVisualShape(cyl_vis, ChFramed(VNULL, Q_ROTATE_Z_TO_X));

                auto cyl_ct = chrono_types::make_shared<ChCollisionShapeCylinder>(ct_mat, radius, height);
                body->AddCollisionShape(cyl_ct, ChFramed(VNULL, Q_ROTATE_Z_TO_X));

                inertia = mass * ChCylinder::CalcGyration(radius, height);

                break;
            }
            case ChVisualShape::Type::CONE: {
                auto cone_vis = chrono_types::make_shared<ChVisualShapeCone>(radius, height);
                cone_vis->AddMaterial(vis_mat);
                body->AddVisualShape(cone_vis, ChFramed(VNULL, Q_ROTATE_Z_TO_X));

                auto cone_ct = chrono_types::make_shared<ChCollisionShapeCone>(ct_mat, radius, height);
                body->AddCollisionShape(cone_ct, ChFramed(VNULL, Q_ROTATE_Z_TO_X));

                inertia = mass * ChCone::CalcGyration(radius, height);

                break;
            }

            case ChVisualShape::Type::ROUNDEDBOX: {
                ChVector3d dims(radius, height, radius);

                auto rbox_vis = chrono_types::make_shared<ChVisualShapeRoundedBox>(dims, sradius);
                rbox_vis->SetMaterial(0, vis_mat);
                body->AddVisualShape(rbox_vis);

                auto rbox_ct = chrono_types::make_shared<ChCollisionShapeRoundedBox>(ct_mat, dims, sradius);
                body->AddCollisionShape(rbox_ct);

                inertia = mass * ChBox::CalcGyration(dims);

                break;
            }

            case ChVisualShape::Type::ROUNDEDCYL: {
                auto rcyl_vis = chrono_types::make_shared<ChVisualShapeRoundedCylinder>(radius, height, sradius);
                rcyl_vis->SetMaterial(0, vis_mat);
                body->AddVisualShape(rcyl_vis, ChFramed(VNULL, Q_ROTATE_Z_TO_X));

                auto rcyl_ct = chrono_types::make_shared<ChCollisionShapeRoundedCylinder>(ct_mat, radius, height, sradius);
                body->AddCollisionShape(rcyl_ct, ChFramed(VNULL, Q_ROTATE_Z_TO_X));

                inertia = mass * ChCylinder::CalcGyration(radius, height);

                break;
            }
        }

        body->SetInertia(inertia);

        sys->AddBody(body);
    }

    // Create the plate
    {
        auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
        vis_mat->SetDiffuseColor(ChColor(0.37f, 0.62f, 0.62f));

        auto body = chrono_types::make_shared<ChBody>();
        body->SetName("Plate");
        body->SetFixed(true);
        body->EnableCollision(true);
        body->SetMass(1);
        body->SetPos(ChVector3d(0, 0, 0));
        body->SetRot(ChQuaternion<>(1, 0, 0, 0));

        auto box_vis = chrono_types::make_shared<ChVisualShapeBox>(side, thickness, side);
        box_vis->AddMaterial(vis_mat);
        body->AddVisualShape(box_vis);

        auto box_ct = chrono_types::make_shared<ChCollisionShapeBox>(ct_mat, side, thickness, side);
        body->AddCollisionShape(box_ct);

        sys->AddBody(body);
    }

    // Create the run-time visualization system
    auto vis = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();

    vis->AttachSystem(sys.get());

    vis->SetWindowTitle("Object drop");
    vis->SetWindowSize(1280, 800);
    vis->SetWindowPosition(100, 100);

    vis->AddCamera(ChVector3d(-6, 1.5, 5));
    vis->SetCameraVertical(CameraVerticalDir::Y);
    vis->SetCameraAngleDeg(40.0);

    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->EnableShadows();

    vis->SetRefFrameVisibility(true);
    vis->SetCollisionVisibility(true);
    vis->SetContactNormalsVisibility(true);

    vis->AddGrid(0.25, 0.25, 28, 28, ChCoordsys<>(ChVector3d(0, 0.105, 0), QuatFromAngleX(CH_PI_2)), ChColor(0.1f, 0.1f, 0.1f));

    vis->Initialize();

    // Simulation loop
    ChRealtimeStepTimer rt_timer;
    double time = 0.0;
    int render_frame = 0;

    while (vis->Run()) {
        if (time > render_frame / render_fps) {
            vis->Render();
            render_frame++;
        }

        sys->DoStepDynamics(time_step);
        rt_timer.Spin(time_step);
        time += time_step;
    }

    return 0;
}
