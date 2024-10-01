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
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code about
// - collisions and contacts
// - using the 'barrel' shape to create rollers for building omnidirectional
//   wheels in a mobile robot.
//
// =============================================================================

#include <cmath>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/assets/ChVisualShapeBarrel.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

double STATIC_rot_speed = 0;
double STATIC_x_speed = 0;
double STATIC_z_speed = 0;
float STATIC_wheelfriction = 0.6f;
#define MAX_ROT_SPEED 0.8
#define MAX_XZ_SPEED 10

/// Following class will be used to manage events from the user interface

class MyEventReceiver : public IEventReceiver {
  public:
    MyEventReceiver() {}

    bool OnEvent(const SEvent& event) {
        // check if user presses keys
        if (event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) {
            switch (event.KeyInput.Key) {
                case irr::KEY_KEY_Q:
                    STATIC_x_speed += 1.5;
                    if (STATIC_x_speed > MAX_XZ_SPEED)
                        STATIC_x_speed = MAX_XZ_SPEED;
                    return true;
                case irr::KEY_KEY_W:
                    STATIC_x_speed -= 1.5;
                    if (STATIC_x_speed < -MAX_XZ_SPEED)
                        STATIC_x_speed = -MAX_XZ_SPEED;
                    return true;
                case irr::KEY_KEY_A:
                    STATIC_z_speed += 1.5;
                    if (STATIC_z_speed > MAX_XZ_SPEED)
                        STATIC_z_speed = MAX_XZ_SPEED;
                    return true;
                case irr::KEY_KEY_Z:
                    STATIC_z_speed -= 1.5;
                    if (STATIC_z_speed < -MAX_XZ_SPEED)
                        STATIC_z_speed = -MAX_XZ_SPEED;
                    return true;
                case irr::KEY_KEY_E:
                    STATIC_rot_speed += 0.05;
                    if (STATIC_rot_speed > MAX_ROT_SPEED)
                        STATIC_rot_speed = MAX_ROT_SPEED;
                    return true;
                case irr::KEY_KEY_R:
                    STATIC_rot_speed -= 0.05;
                    if (STATIC_rot_speed < -MAX_ROT_SPEED)
                        STATIC_rot_speed = -MAX_ROT_SPEED;
                    return true;
                default:
                    break;
            }
        }

        return false;
    }
};

// This small function creates a Mecanum wheel, made with many ChBodySceneNode rigid bodies (a central
// wheel and the many radial rollers, already lined to the wheel with revolute joints.)
// The function returns the pointer to the central wheel.
std::shared_ptr<ChBody> create_mecanum_wheel(ChSystemNSC& sys,
                                             ChVector3d shaft_position,
                                             ChQuaternion<> shaft_alignment,
                                             double wheel_radius,
                                             double wheel_width,
                                             int n_rollers,
                                             double roller_angle,
                                             double roller_midradius,
                                             double roller_density,
                                             double spindle_density) {
    ChFrameMoving<> ftot(shaft_position, shaft_alignment);  // will be used to transform pos & rot of all objects

    auto centralWheel = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y,  //
                                                                      wheel_radius / 2,
                                                                      wheel_width,      // radius, height
                                                                      spindle_density,  // density
                                                                      true,             // visualize
                                                                      false);           // no collision
    centralWheel->SetPos(shaft_position);
    centralWheel->SetRot(shaft_alignment);
    centralWheel->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/pinkwhite.png"));
    sys.Add(centralWheel);

    auto wheel_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    wheel_mat->SetFriction(STATIC_wheelfriction);

    double half_length_roller = 0.5 * wheel_width * 1.0 / (std::cos(roller_angle));
    double roller_elliptical_rad_Hor = wheel_radius;
    double roller_elliptical_rad_Vert = wheel_radius * 1.0 / (std::cos(roller_angle));

    for (int iroller = 0; iroller < n_rollers; iroller++) {
        double pitch = CH_2PI * ((double)iroller / (double)n_rollers);

        double Roffset = -(wheel_radius - roller_midradius);

        // Create the roller
        auto roller = chrono_types::make_shared<ChBody>();
        sys.Add(roller);

        // move it to slanted aligment
        ChFrameMoving<> f1(ChVector3d(0, 0, -(wheel_radius - roller_midradius)), QuatFromAngleZ(roller_angle));
        ChFrameMoving<> f2(ChVector3d(0, 0, 0), QuatFromAngleY(pitch));
        ChFrameMoving<> f3 = f1 >> f2 >> ftot;
        roller->ConcatenatePreTransformation(f3);

        // approximate mass & inertia to a cylinder:
        roller->SetMass(  //
            ChCylinder::GetVolume(roller_elliptical_rad_Hor + Roffset, 2 * half_length_roller) * roller_density);
        roller->SetInertia(  //
            ChCylinder::GetGyration(roller_elliptical_rad_Hor + Roffset, 2 * half_length_roller) * roller_density);

        // add collision shape
        auto shape = chrono_types::make_shared<ChCollisionShapeBarrel>(wheel_mat,                                 //
                                                                       -half_length_roller, +half_length_roller,  //
                                                                       2 * roller_elliptical_rad_Vert,            //
                                                                       2 * roller_elliptical_rad_Hor,             //
                                                                       Roffset);
        roller->AddCollisionShape(shape);
        roller->EnableCollision(true);

        // add visualization shape
        auto rollershape = chrono_types::make_shared<ChVisualShapeBarrel>(-half_length_roller, +half_length_roller,  //
                                                                          2 * roller_elliptical_rad_Vert,
                                                                          2 * roller_elliptical_rad_Hor,  //
                                                                          Roffset);
        roller->AddVisualShape(rollershape);

        // Make the revolute joint between the roller and the central wheel
        // (preconcatenate rotation 90 degrees on X, to set axis of revolute joint)
        ChFrameMoving<> fr(ChVector3d(0, 0, 0), QuatFromAngleX(CH_PI_2));
        ChFrameMoving<> frabs = fr >> f3;
        auto link_roller = chrono_types::make_shared<ChLinkLockRevolute>();
        link_roller->Initialize(roller, centralWheel, frabs);
        sys.AddLink(link_roller);
    }

    return centralWheel;
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create a ChronoENGINE physical system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    double platform_radius = 8;
    double wheel_radius = 3;
    double roller_angle = CH_PI / 4;

    // Create the robot truss, as a circular platform
    auto platform = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y,                 //
                                                                  platform_radius * 0.7, 2,  // radius, height
                                                                  1000,                      // density
                                                                  true,                      // visualize
                                                                  false);                    // no collision
    sys.Add(platform);

    // ChCollisionModel::SetDefaultSuggestedEnvelope(0.01);
    // ChCollisionModel::SetDefaultSuggestedMargin(0.005);

    // create the wheels and link them to the platform

    ChFrame<> f0(ChVector3d(0, 0, 0), QuatFromAngleX(CH_PI_2));
    ChFrame<> f1(ChVector3d(0, 0, platform_radius), QUNIT);
    ChFrame<> f2_wA(VNULL, QuatFromAngleY(0 * (CH_2PI / 3.0)));
    ChFrame<> f2_wB(VNULL, QuatFromAngleY(1 * (CH_2PI / 3.0)));
    ChFrame<> f2_wC(VNULL, QuatFromAngleY(2 * (CH_2PI / 3.0)));
    ChFrame<> ftot_wA = f0 >> f1 >> f2_wA;
    ChFrame<> ftot_wB = f0 >> f1 >> f2_wB;
    ChFrame<> ftot_wC = f0 >> f1 >> f2_wC;

    auto spindle_A = create_mecanum_wheel(sys,
                                          ftot_wA.GetCoordsys().pos,  // wheel position
                                          ftot_wA.GetCoordsys().rot,  // wheel alignment
                                          wheel_radius,               // wheel radius
                                          2.2,                        // wheel width
                                          8,                          // n. of rollers
                                          roller_angle,               // angle of rollers
                                          0.65,                       // max rad. of roller
                                          1000,                       // density of roller
                                          1000);                      // density of the spindle

    auto link_shaftA = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    link_shaftA->Initialize(spindle_A, platform, (f1 >> f2_wA));
    link_shaftA->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0));
    sys.AddLink(link_shaftA);

    auto spindle_B = create_mecanum_wheel(sys,
                                          ftot_wB.GetCoordsys().pos,  // wheel position
                                          ftot_wB.GetCoordsys().rot,  // wheel alignment
                                          wheel_radius,               // wheel radius
                                          2.2,                        // wheel width
                                          8,                          // n. of rollers
                                          roller_angle,               // angle of rollers
                                          0.65,                       // max rad. of roller
                                          1000,                       // density of roller
                                          1000);                      // density of the spindle

    auto link_shaftB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    link_shaftB->Initialize(spindle_B, platform, (f1 >> f2_wB));
    link_shaftB->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0));
    sys.AddLink(link_shaftB);

    auto spindle_C = create_mecanum_wheel(sys,
                                          ftot_wC.GetCoordsys().pos,  // wheel position
                                          ftot_wC.GetCoordsys().rot,  // wheel alignment
                                          wheel_radius,               // wheel radius
                                          2.2,                        // wheel width
                                          8,                          // n. of rollers
                                          roller_angle,               // angle of rollers
                                          0.65,                       // max rad. of roller
                                          1000,                       // density of roller
                                          1000);                      // density of the spindle

    auto link_shaftC = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    link_shaftC->Initialize(spindle_C, platform, (f1 >> f2_wC));
    link_shaftC->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0));
    sys.AddLink(link_shaftC);

    // Create the ground for the collision
    auto ground_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    ground_mat->SetFriction(STATIC_wheelfriction);

    auto ground = chrono_types::make_shared<ChBodyEasyBox>(200, 1, 200,  // size
                                                           1000,         // density
                                                           true,         // visualize
                                                           true,         // collide
                                                           ground_mat);  // contact material
    ground->SetPos(ChVector3d(0, -5, 0));
    ground->SetFixed(true);
    ground->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"), 100, 100);
    sys.Add(ground);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Mecanum robot simulator");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(0, 14, -20));
    vis->AddTypicalLights();

    vis->GetGUIEnvironment()->addStaticText(L"Use keys Q,W, A,Z, E,R to move the robot", rect<s32>(150, 10, 430, 40),
                                            true);

    MyEventReceiver receiver;
    vis->AddUserEventReceiver(&receiver);

    // Prepare the physical system for the simulation

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);

    sys.SetSolverType(ChSolver::Type::PSOR);
    sys.GetSolver()->AsIterative()->SetMaxIterations(30);

    // Simulation loop

    double timestep = 0.01;
    ChRealtimeStepTimer realtime_timer;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // ADVANCE THE SIMULATION FOR ONE TIMESTEP
        sys.DoStepDynamics(timestep);

        // change motor speeds depending on user setpoints from GUI

        ChVector3d imposed_speed(STATIC_x_speed, 0, STATIC_z_speed);
        ChFrame<> roll_twist(ChVector3d(0, -wheel_radius, 0), QuatFromAngleY(-roller_angle));

        ChFrame<> abs_roll_wA = roll_twist >> f2_wA >> ChFrame<>(platform->GetCoordsys());
        double wheel_A_rotspeed =
            (STATIC_rot_speed * platform_radius) +
            ((abs_roll_wA.GetRotMat().transpose() * imposed_speed).x() / std::sin(roller_angle)) / wheel_radius;
        ChFrame<> abs_roll_wB = roll_twist >> f2_wB >> ChFrame<>(platform->GetCoordsys());
        double wheel_B_rotspeed =
            (STATIC_rot_speed * platform_radius) +
            ((abs_roll_wB.GetRotMat().transpose() * imposed_speed).x() / std::sin(roller_angle)) / wheel_radius;
        ChFrame<> abs_roll_wC = roll_twist >> f2_wC >> ChFrame<>(platform->GetCoordsys());
        double wheel_C_rotspeed =
            (STATIC_rot_speed * platform_radius) +
            ((abs_roll_wC.GetRotMat().transpose() * imposed_speed).x() / std::sin(roller_angle)) / wheel_radius;

        if (auto fun = std::dynamic_pointer_cast<ChFunctionConst>(link_shaftA->GetSpeedFunction()))
            fun->SetConstant(wheel_A_rotspeed);
        if (auto fun = std::dynamic_pointer_cast<ChFunctionConst>(link_shaftB->GetSpeedFunction()))
            fun->SetConstant(wheel_B_rotspeed);
        if (auto fun = std::dynamic_pointer_cast<ChFunctionConst>(link_shaftC->GetSpeedFunction()))
            fun->SetConstant(wheel_C_rotspeed);

        realtime_timer.Spin(timestep);
    }

    return 0;
}
