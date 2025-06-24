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
// Simple demo for contact between a primitive object and a plate (SMC)
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChContactContainerSMC.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// -----------------------------------------------------------------------------

collision::ChCollisionSystemType collision_type = collision::ChCollisionSystemType::BULLET;

enum class ObjectType { SPHERE, CYLINDER, CONE };
ObjectType object_type = ObjectType::SPHERE;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Simulation parameters
    double gravity = -9.81;
    double time_step = 1e-5;
    double out_step = 2000 * time_step;

    // Parameters for the falling ball
    double radius = 1;
    double height = 1;
    double mass = 100;

    ChVector<> pos(0, 2, 0);
    ChQuaternion<> rot(1, 0, 0, 0);
    ChVector<> init_vel(0, 0, 0);

    // Parameters for the containing bin
    double side = 4;
    double thickness = 0.1;

    // Create the system
    ChSystemSMC sys;

    sys.Set_G_acc(ChVector<>(0, gravity, 0));
    sys.SetCollisionSystemType(collision_type);

    sys.SetContactForceModel(ChSystemSMC::ContactForceModel::Hertz);
    sys.SetAdhesionForceModel(ChSystemSMC::AdhesionForceModel::Constant);

    // Change the default collision effective radius of curvature
    collision::ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(1);

    // Create a material (will be used by both objects)
    auto material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    material->SetRestitution(0.1f);
    material->SetFriction(0.4f);
    material->SetAdhesion(0);  // Magnitude of the adhesion in Constant adhesion model

    // Create the falling object
    ChMatrix33<> inertia;

    auto body = chrono_types::make_shared<ChBody>(collision_type);
    body->SetNameString("Object");
    body->SetMass(mass);
    body->SetPos(pos);
    body->SetRot(rot);
    body->SetPos_dt(init_vel);
    body->SetBodyFixed(false);

    body->SetCollide(true);

    body->GetCollisionModel()->ClearModel();
    switch (object_type) {
        case ObjectType::SPHERE: {
            inertia = mass * utils::CalcSphereGyration(radius);

            body->GetCollisionModel()->AddSphere(material, radius);

            auto sphere = chrono_types::make_shared<ChSphereShape>();
            sphere->GetSphereGeometry().rad = radius;
            sphere->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
            sphere->SetOpacity(1.0f);
            auto ball_vis = chrono_types::make_shared<ChVisualModel>();
            ball_vis->AddShape(sphere);
            body->AddVisualModel(ball_vis);

            break;
        }
        case ObjectType::CYLINDER: {
            inertia = mass * utils::CalcCylinderGyration(radius, height / 2);

            body->GetCollisionModel()->AddCylinder(material, radius, radius, height / 2);

            auto cyl = chrono_types::make_shared<ChCylinderShape>();
            cyl->GetCylinderGeometry().p1 = ChVector<>(0, +height / 2, 0);
            cyl->GetCylinderGeometry().p2 = ChVector<>(0, -height / 2, 0);
            cyl->GetCylinderGeometry().rad = radius;
            body->AddVisualShape(cyl);

            break;
        }
        case ObjectType::CONE: {
            inertia = mass * utils::CalcConeGyration(radius, height / 2);

            body->GetCollisionModel()->AddCone(material, radius, height);

            auto cone = chrono_types::make_shared<ChConeShape>();
            cone->GetConeGeometry().r = radius;
            cone->GetConeGeometry().h = height;
            body->AddVisualShape(cone);

            break;
        }
    }
    body->GetCollisionModel()->BuildModel();

    body->SetInertia(inertia);

    sys.AddBody(body);

    // Create container
    auto bin = chrono_types::make_shared<ChBody>(collision_type);

    bin->SetMass(1);
    bin->SetPos(ChVector<>(0, 0, 0));
    bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
    bin->SetCollide(true);
    bin->SetBodyFixed(true);

    bin->GetCollisionModel()->ClearModel();
    bin->GetCollisionModel()->AddBox(material, side, thickness, side);
    bin->GetCollisionModel()->BuildModel();

    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = ChVector<>(side, thickness, side);
    box->SetColor(ChColor(0.8f, 0.2f, 0.2f));
    box->SetOpacity(0.8f);

    auto bin_vis = chrono_types::make_shared<ChVisualModel>();
    bin_vis->AddShape(box, ChFrame<>());
    bin->AddVisualModel(bin_vis);

    sys.AddBody(bin);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Object drop");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector<>(0, 3, -6));
    vis->AttachSystem(&sys);

    // The soft-real-time cycle
    double time = 0.0;
    double out_time = 0.0;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        tools::drawGrid(vis.get(), 0.4, 0.4, 20, 20, ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
                        ChColor(0.31f, 0.39f, 0.39f), true);
        vis->EndScene();

        while (time < out_time) {
            sys.DoStepDynamics(time_step);
            time += time_step;
        }
        out_time += out_step;
    }

    return 0;
}
