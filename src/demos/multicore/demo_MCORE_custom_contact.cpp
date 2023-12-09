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

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

// Helper class to define a cylindrical shape
class MyObstacle {
  public:
    MyObstacle() : radius(2), center(2.9, 0, 2.9) {}

    void AddVisualization(std::shared_ptr<ChBody> body) {
        auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(radius, 1.1);
        cyl->SetColor(ChColor(0.6f, 0.3f, 0.0f));
        body->AddVisualShape(cyl, ChFrame<>(center + ChVector<>(0, 0.55, 0), Q_from_AngX(CH_C_PI_2)));
    }

    double radius;
    ChVector<> center;
};

// Custom collision detection callback class
class MyCustomCollisionDetection : public ChSystem::CustomCollisionCallback {
  public:
    MyCustomCollisionDetection(std::shared_ptr<ChBody> ball,
                               std::shared_ptr<ChBody> ground,
                               std::shared_ptr<ChMaterialSurface> ball_mat,
                               std::shared_ptr<ChMaterialSurface> obst_mat,
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
        auto b_center = ChVector2<>(b_pos.x(), b_pos.z());

        // Check collision with obstacle (working in the horizontal plane).
        auto o_center = ChVector2<>(m_obst_center.x(), m_obst_center.z());
        auto delta = o_center - b_center;
        auto dist2 = delta.Length2();
        if (dist2 >= r_sum * r_sum)
            return;

        // Find collision points on the ball and obstacle and the contact normal.
        auto dist = std::sqrt(dist2);
        ChVector2<> normal = delta / dist;
        ChVector2<> pt_ball = b_center + m_ball_radius * normal;
        ChVector2<> pt_obst = o_center - m_obst_radius * normal;

        // Populate the collision info object (express all vectors in 3D).
        // We pass null pointers to collision shapes.
        ChCollisionInfo contact;
        contact.modelA = m_ball->GetCollisionModel().get();
        contact.modelB = m_ground->GetCollisionModel().get();
        contact.shapeA = nullptr;
        contact.shapeB = nullptr;
        contact.vN = ChVector<>(normal.x(), 0.0, normal.y());
        contact.vpA = ChVector<>(pt_ball.x(), b_pos.y(), pt_ball.y());
        contact.vpB = ChVector<>(pt_obst.x(), b_pos.y(), pt_obst.y());
        contact.distance = dist - r_sum;
        msys->GetContactContainer()->AddContact(contact, m_ball_mat, m_obst_mat);
    }

    std::shared_ptr<ChBody> m_ball;
    std::shared_ptr<ChBody> m_ground;
    std::shared_ptr<ChMaterialSurface> m_ball_mat;
    std::shared_ptr<ChMaterialSurface> m_obst_mat;
    double m_ball_radius;
    double m_obst_radius;
    ChVector<> m_obst_center;
};

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChContactMethod contact_method = ChContactMethod::SMC;

    double ball_radius = 0.5;
    MyObstacle obstacle;
    ChVector<> obst_center(2.9, 0, 2.9);

    // Create the system and the various contact materials
    ChSystemMulticore* sys = nullptr;
    std::shared_ptr<ChMaterialSurface> ground_mat;
    std::shared_ptr<ChMaterialSurface> ball_mat;
    std::shared_ptr<ChMaterialSurface> obst_mat;
    double time_step = 0;
    int frame_skip = 0;

    switch (contact_method) {
        case ChContactMethod::NSC: {
            sys = new ChSystemMulticoreNSC;

            auto g_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            g_mat->SetRestitution(0.9f);
            g_mat->SetFriction(0.4f);
            auto b_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            b_mat->SetRestitution(0.9f);
            b_mat->SetFriction(0.5f);
            auto o_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            o_mat->SetRestitution(0.9f);
            o_mat->SetFriction(0.4f);

            ground_mat = g_mat;
            ball_mat = b_mat;
            obst_mat = o_mat;

            time_step = 1e-3;
            frame_skip = 10;

            break;
        }
        case ChContactMethod::SMC: {
            sys = new ChSystemMulticoreSMC;

            auto g_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            g_mat->SetRestitution(0.9f);
            g_mat->SetFriction(0.4f);
            auto b_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            b_mat->SetRestitution(0.9f);
            b_mat->SetFriction(0.5f);
            auto o_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            o_mat->SetRestitution(0.9f);
            o_mat->SetFriction(0.4f);

            ground_mat = g_mat;
            ball_mat = b_mat;
            obst_mat = o_mat;

            time_step = 1e-4;
            frame_skip = 100;

            break;
        }
    }

    auto ground_mat_vis = chrono_types::make_shared<ChVisualMaterial>();
    ground_mat_vis->SetKdTexture(GetChronoDataFile("textures/blue.png"));

    sys->Set_G_acc(ChVector<>(0, -9.8, 0));
    sys->SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);

    // Create the ground body with a plate and side walls (both collision and visualization).
    // Add obstacle visualization (in a separate level with a different color).
    auto ground = chrono_types::make_shared<ChBody>();
    sys->AddBody(ground);
    ground->SetCollide(true);
    ground->SetBodyFixed(true);

    utils::AddBoxContainer(ground, ground_mat,                     //
                           ChFrame<>(ChVector<>(0, 1, 0), QUNIT),  //
                           ChVector<>(10, 2, 10), 0.2,             //
                           ChVector<int>(2, -1, 2),                //
                           true, ground_mat_vis);

    obstacle.AddVisualization(ground);

    // Create the falling ball
    auto ball = chrono_types::make_shared<ChBody>();
    sys->AddBody(ball);
    ball->SetMass(10);
    ball->SetInertiaXX(4 * ball_radius * ball_radius * ChVector<>(1, 1, 1));
    ball->SetPos(ChVector<>(-3, 1.2 * ball_radius, -3));
    ball->SetPos_dt(ChVector<>(5, 0, 5));
    ball->SetCollide(true);

    utils::AddSphereGeometry(ball.get(), ball_mat, ball_radius);

    ball->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/bluewhite.png"));

    // Create a custom collision detection callback object and register it with the system
    auto my_collision =
        chrono_types::make_shared<MyCustomCollisionDetection>(ball, ground, ball_mat, obst_mat, ball_radius, obstacle);
    sys->RegisterCustomCollisionCallback(my_collision);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Custom contact demo (Chrono::Multicore)");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(8, 8, -6));
    vis->AddTypicalLights();

    // Simulation loop
    int frame = 0;
    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        if (frame % frame_skip == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }
        sys->DoStepDynamics(time_step);
        realtime_timer.Spin(time_step);
    }

    delete sys;
    return 0;
}
