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
// Demo code about collisions and contacts using the penalty method (SMC)
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/physics/ChContactContainerSMC.h"

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

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Helper class to define a cylindrical shape
class MyObstacle {
  public:
    MyObstacle() : radius(2), center(2.9, 0, 2.9) {}

    void AddVisualization(std::shared_ptr<ChBody> body) {
        auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(radius, 1.1);
        cyl->SetColor(ChColor(0.6f, 0.3f, 0.0f));
        body->AddVisualShape(cyl, ChFrame<>(center + ChVector3d(0, 0.55, 0), QuatFromAngleX(CH_PI_2)));
    }

    double radius;
    ChVector3d center;
};

// Custom collision detection callback class
class MyCustomCollisionDetection : public ChSystem::CustomCollisionCallback {
  public:
    MyCustomCollisionDetection(std::shared_ptr<ChBody> ball,
                               std::shared_ptr<ChBody> ground,
                               std::shared_ptr<ChContactMaterial> ball_mat,
                               std::shared_ptr<ChContactMaterial> obst_mat,
                               double ball_radius,
                               const MyObstacle& obstacle)
        : m_ball(ball),
          m_ground(ground),
          m_ball_mat(ball_mat),
          m_obst_mat(obst_mat),
          m_ball_radius(ball_radius),
          m_obst_radius(obstacle.radius),
          m_obst_center(obstacle.center) {}

    virtual void OnCustomCollision(ChSystem* msys) override {
        auto r_sum = m_ball_radius + m_obst_radius;

        // Get current ball position and project on horizontal plane.
        auto b_pos = m_ball->GetPos();
        auto b_center = ChVector2d(b_pos.x(), b_pos.z());

        // Check collision with obstacle (working in the horizontal plane).
        auto o_center = ChVector2d(m_obst_center.x(), m_obst_center.z());
        auto delta = o_center - b_center;
        auto dist2 = delta.Length2();
        if (dist2 >= r_sum * r_sum)
            return;

        // Find collision points on the ball and obstacle and the contact normal.
        auto dist = std::sqrt(dist2);
        ChVector2d normal = delta / dist;
        ChVector2d pt_ball = b_center + m_ball_radius * normal;
        ChVector2d pt_obst = o_center - m_obst_radius * normal;

        // Populate the collision info object (express all vectors in 3D).
        // We pass null pointers to collision shapes.
        ChCollisionInfo contact;
        contact.modelA = m_ball->GetCollisionModel().get();
        contact.modelB = m_ground->GetCollisionModel().get();
        contact.shapeA = nullptr;
        contact.shapeB = nullptr;
        contact.vN = ChVector3d(normal.x(), 0.0, normal.y());
        contact.vpA = ChVector3d(pt_ball.x(), b_pos.y(), pt_ball.y());
        contact.vpB = ChVector3d(pt_obst.x(), b_pos.y(), pt_obst.y());
        contact.distance = dist - r_sum;
        msys->GetContactContainer()->AddContact(contact, m_ball_mat, m_obst_mat);
    }

    std::shared_ptr<ChBody> m_ball;
    std::shared_ptr<ChBody> m_ground;
    std::shared_ptr<ChContactMaterial> m_ball_mat;
    std::shared_ptr<ChContactMaterial> m_obst_mat;
    double m_ball_radius;
    double m_obst_radius;
    ChVector3d m_obst_center;
};

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    ChContactMethod contact_method = ChContactMethod::SMC;

    double ball_radius = 0.5;
    MyObstacle obstacle;
    ChVector3d obst_center(2.9, 0, 2.9);

    // Create the system and the various contact materials
    ChSystem* sys = nullptr;
    std::shared_ptr<ChContactMaterial> ground_mat;
    std::shared_ptr<ChContactMaterial> ball_mat;
    std::shared_ptr<ChContactMaterial> obst_mat;

    switch (contact_method) {
        case ChContactMethod::NSC: {
            sys = new ChSystemNSC;

            auto g_mat = chrono_types::make_shared<ChContactMaterialNSC>();
            g_mat->SetRestitution(0.9f);
            g_mat->SetFriction(0.4f);
            auto b_mat = chrono_types::make_shared<ChContactMaterialNSC>();
            b_mat->SetRestitution(0.9f);
            b_mat->SetFriction(0.5f);
            auto o_mat = chrono_types::make_shared<ChContactMaterialNSC>();
            o_mat->SetRestitution(0.9f);
            o_mat->SetFriction(0.4f);

            ground_mat = g_mat;
            ball_mat = b_mat;
            obst_mat = o_mat;

            break;
        }
        case ChContactMethod::SMC: {
            sys = new ChSystemSMC;

            auto g_mat = chrono_types::make_shared<ChContactMaterialSMC>();
            g_mat->SetRestitution(0.9f);
            g_mat->SetFriction(0.4f);
            auto b_mat = chrono_types::make_shared<ChContactMaterialSMC>();
            b_mat->SetRestitution(0.9f);
            b_mat->SetFriction(0.5f);
            auto o_mat = chrono_types::make_shared<ChContactMaterialSMC>();
            o_mat->SetRestitution(0.9f);
            o_mat->SetFriction(0.4f);

            ground_mat = g_mat;
            ball_mat = b_mat;
            obst_mat = o_mat;

            break;
        }
    }

    sys->SetGravitationalAcceleration(ChVector3d(0, -9.8, 0));
    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the ground body with a plate and side walls (both collision and visualization).
    // Add obstacle visualization (in a separate level with a different color).
    auto ground = chrono_types::make_shared<ChBody>();
    sys->AddBody(ground);
    ground->EnableCollision(true);
    ground->SetFixed(true);

    auto ground_vmat = chrono_types::make_shared<ChVisualMaterial>();
    ground_vmat->SetKdTexture(GetChronoDataFile("textures/blue.png"));

    utils::AddBoxGeometry(ground.get(), ground_mat, ChVector3d(10, 2, 10), ChVector3d(0, -1, 0), QUNIT, true,
                          ground_vmat);
    utils::AddBoxGeometry(ground.get(), ground_mat, ChVector3d(0.2, 2, 10.2), ChVector3d(-5, 0, 0), QUNIT, true,
                          ground_vmat);
    utils::AddBoxGeometry(ground.get(), ground_mat, ChVector3d(0.2, 2, 10.2), ChVector3d(+5, 0, 0), QUNIT, true,
                          ground_vmat);
    utils::AddBoxGeometry(ground.get(), ground_mat, ChVector3d(10.2, 2, 0.2), ChVector3d(0, 0, -5), QUNIT, true,
                          ground_vmat);
    utils::AddBoxGeometry(ground.get(), ground_mat, ChVector3d(10.2, 2, 0.2), ChVector3d(0, 0, +5), QUNIT, true,
                          ground_vmat);

    obstacle.AddVisualization(ground);

    // Create the falling ball
    auto ball = chrono_types::make_shared<ChBody>();
    sys->AddBody(ball);
    ball->SetMass(10);
    ball->SetInertiaXX(4 * ball_radius * ball_radius * ChVector3d(1, 1, 1));
    ball->SetPos(ChVector3d(-3, 1.2 * ball_radius, -3));
    ball->SetPosDt(ChVector3d(5, 0, 5));
    ball->EnableCollision(true);

    auto ball_vmat = chrono_types::make_shared<ChVisualMaterial>();
    ball_vmat->SetKdTexture(GetChronoDataFile("textures/bluewhite.png"));

    utils::AddSphereGeometry(ball.get(), ball_mat, ball_radius, VNULL, QUNIT, true, ball_vmat);

    // Create a custom collision detection callback object and register it with the system
    auto collision =
        std::make_shared<MyCustomCollisionDetection>(ball, ground, ball_mat, obst_mat, ball_radius, obstacle);
    sys->RegisterCustomCollisionCallback(collision);

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
            vis_irr->AttachSystem(sys);
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("Custom contact demo");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector3d(8, 8, -6));
            vis_irr->AddTypicalLights();
            vis_irr->AddLightWithShadow(ChVector3d(0.0, 24.0, 2.0), ChVector3d(0, 0, 0), 35, 2.2, 25.0, 40, 1024,
                                    ChColor(0.8f, 0.8f, 1.0f));
            vis_irr->EnableShadows();

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(sys);
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowPosition(100, 100);
            vis_vsg->SetWindowTitle("Custom contact demo");
            vis_vsg->SetBackgroundColor(ChColor(0.8f, 0.85f, 0.9f));
            vis_vsg->EnableSkyBox();
            vis_vsg->SetCameraVertical(CameraVerticalDir::Y);
            vis_vsg->AddCamera(ChVector3d(8, 8, -6), ChVector3d(0, 0, 0));
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    int frame = 0;
    while (vis->Run()) {
        if (frame % 100 == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }

        sys->DoStepDynamics(1e-4);
        frame++;
    }

    delete sys;
    return 0;
}
