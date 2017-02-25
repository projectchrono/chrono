// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_irrlicht/ChBodySceneNode.h"
#include "chrono_irrlicht/ChBodySceneNodeTools.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include <irrlicht.h>

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
    MyEventReceiver(ChIrrAppInterface* myapp) {
        // store pointer to physical system & other stuff so we can tweak them by user keyboard
        app = myapp;
    }

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

  private:
    ChIrrAppInterface* app;
};

// This small function creates a Mecanum wheel, made with many ChBodySceneNode rigid bodies (a central
// wheel and the many radial rollers, already lined to the wheel with revolute joints.)
// The function returns the pointer to the central wheel.
ChBodySceneNode* create_mecanum_wheel(ChSystem& mphysicalSystem,
                                      ISceneManager* msceneManager,
                                      IVideoDriver* driver,
                                      ChVector<> shaft_position,
                                      ChQuaternion<> shaft_alignment,
                                      double wheel_radius,
                                      double wheel_width,
                                      int n_rollers,
                                      double roller_angle,
                                      double roller_midradius,
                                      double roller_mass,
                                      double spindle_mass) {
    ChFrameMoving<> ftot(shaft_position, shaft_alignment);  // will be used to transform pos & rot of all objects

    ChBodySceneNode* mCentralWheel = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
        &mphysicalSystem, msceneManager, spindle_mass, shaft_position, shaft_alignment,
        ChVector<>(wheel_radius, wheel_width, wheel_radius));

    mCentralWheel->GetBody()->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));  //***TO DO*** provide correct inertia xx yy zz
    mCentralWheel->GetBody()->SetCollide(false);
    mCentralWheel->addShadowVolumeSceneNode();

    video::ITexture* cylinderMap = driver->getTexture(GetChronoDataFile("pinkwhite.png").c_str());
    mCentralWheel->setMaterialTexture(0, cylinderMap);

    double half_length_roller = 0.5 * wheel_width * 1.0 / (cos(roller_angle));
    double roller_elliptical_rad_Hor = wheel_radius;
    double roller_elliptical_rad_Vert = wheel_radius * 1.0 / (cos(roller_angle));

    for (int iroller = 0; iroller < n_rollers; iroller++) {
        double pitch = CH_C_2PI * ((double)iroller / (double)n_rollers);

        // Create the roller
        ChBodySceneNode* mRoller = (ChBodySceneNode*)addChBodySceneNode_easyBarrel(
            &mphysicalSystem, msceneManager, roller_mass, ChVector<>(0, 0, 0), roller_elliptical_rad_Hor,
            roller_elliptical_rad_Vert, -half_length_roller, +half_length_roller, -(wheel_radius - roller_midradius));
        mRoller->setMaterialTexture(0, cylinderMap);
        mRoller->addShadowVolumeSceneNode();
        mRoller->GetBody()->SetInertiaXX(ChVector<>(0.05, 0.005, 0.05));  //***TO DO *** proper inertia
        mRoller->GetBody()->SetCollide(true);
        mRoller->GetBody()->GetMaterialSurface()->SetFriction(STATIC_wheelfriction);
        ChFrameMoving<> f1(ChVector<>(0, 0, -(wheel_radius - roller_midradius)),
                           Q_from_AngAxis(roller_angle, ChVector<>(0, 0, 1)));
        ChFrameMoving<> f2(ChVector<>(0, 0, 0), Q_from_AngAxis(pitch, ChVector<>(0, 1, 0)));
        ChFrameMoving<> f3 = f1 >> f2 >> ftot;
        mRoller->GetBody()->ConcatenatePreTransformation(f3);

        // Make the revolute joint between the roller and the central wheel
        // (preconcatenate rotation 90 degrees on X, to set axis of revolute joint)
        ChFrameMoving<> fr(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI / 2.0, ChVector<>(1, 0, 0)));
        ChFrameMoving<> frabs = fr >> f3;
        auto my_link_roller = std::make_shared<ChLinkLockRevolute>();
        my_link_roller->Initialize(mRoller->GetBody(), mCentralWheel->GetBody(), frabs.GetCoord());
        mphysicalSystem.AddLink(my_link_roller);
    }

    return mCentralWheel;
}

int main(int argc, char* argv[]) {
    // Create a ChronoENGINE physical system
    ChSystem mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Mecanum robot simulator", core::dimension2d<u32>(800, 600), false);

    // create text with info
    IGUIStaticText* textFPS = application.GetIGUIEnvironment()->addStaticText(
        L"Use keys Q,W, A,Z, E,R to move the robot", rect<s32>(150, 10, 430, 40), true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 14, -20));

    // This is for GUI tweaking of system parameters..
    MyEventReceiver receiver(&application);
    // note how to add a custom event receiver to the default interface:
    application.SetUserEventReceiver(&receiver);

    double platform_radius = 8;
    double wheel_radius = 3;
    double roller_angle = CH_C_PI / 4;

    // Create the robot truss, as a circular platform
    ChBodySceneNode* mTrussPlatform = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
        &mphysicalSystem, application.GetSceneManager(), 1, ChVector<>(0, 0, 0), QUNIT,
        ChVector<>(platform_radius * 1.5, 2, platform_radius * 1.5));

    mTrussPlatform->GetBody()->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));  //***TO DO*** provide correct inertia xx yy zz
    mTrussPlatform->GetBody()->SetCollide(true);
    mTrussPlatform->addShadowVolumeSceneNode();

    // ChCollisionModel::SetDefaultSuggestedEnvelope(0.01);
    // ChCollisionModel::SetDefaultSuggestedMargin(0.005);

    // create the wheels and link them to the platform

    ChFrame<> f0(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI / 2.0, ChVector<>(1, 0, 0)));
    ChFrame<> f1(ChVector<>(0, 0, platform_radius), QUNIT);
    ChFrame<> f2_wA(VNULL, Q_from_AngAxis(0 * (CH_C_2PI / 3.0), ChVector<>(0, 1, 0)));
    ChFrame<> f2_wB(VNULL, Q_from_AngAxis(1 * (CH_C_2PI / 3.0), ChVector<>(0, 1, 0)));
    ChFrame<> f2_wC(VNULL, Q_from_AngAxis(2 * (CH_C_2PI / 3.0), ChVector<>(0, 1, 0)));
    ChFrame<> ftot_wA = f0 >> f1 >> f2_wA;
    ChFrame<> ftot_wB = f0 >> f1 >> f2_wB;
    ChFrame<> ftot_wC = f0 >> f1 >> f2_wC;

    ChBodySceneNode* spindle_A =
        create_mecanum_wheel(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver(),
                             ftot_wA.GetCoord().pos,  // wheel position
                             ftot_wA.GetCoord().rot,  // wheel alignment
                             wheel_radius,            // wheel radius
                             2.2,                     // wheel width
                             8,                       // n. of rollers
                             roller_angle,            // angle of rollers
                             0.65,                    // max rad. of roller
                             0.1,                     // mass of single roller
                             0.2);                    // mass of the spindle

    auto my_link_shaftA = std::make_shared<ChLinkEngine>();
    my_link_shaftA->Initialize(spindle_A->GetBody(), mTrussPlatform->GetBody(), (f1 >> f2_wA).GetCoord());
    my_link_shaftA->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(my_link_shaftA->Get_spe_funct()))
        mfun->Set_yconst(0.0);
    mphysicalSystem.AddLink(my_link_shaftA);

    ChBodySceneNode* spindle_B =
        create_mecanum_wheel(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver(),
                             ftot_wB.GetCoord().pos,  // wheel position
                             ftot_wB.GetCoord().rot,  // wheel alignment
                             wheel_radius,            // wheel radius
                             2.2,                     // wheel width
                             8,                       // n. of rollers
                             roller_angle,            // angle of rollers
                             0.65,                    // max rad. of roller
                             0.1,                     // mass of single roller
                             0.2);                    // mass of the spindle
    auto my_link_shaftB = std::make_shared<ChLinkEngine>();
    my_link_shaftB->Initialize(spindle_B->GetBody(), mTrussPlatform->GetBody(), (f1 >> f2_wB).GetCoord());
    my_link_shaftB->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(my_link_shaftB->Get_spe_funct()))
        mfun->Set_yconst(0.0);
    mphysicalSystem.AddLink(my_link_shaftB);

    ChBodySceneNode* spindle_C =
        create_mecanum_wheel(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver(),
                             ftot_wC.GetCoord().pos,  // wheel position
                             ftot_wC.GetCoord().rot,  // wheel alignment
                             wheel_radius,            // wheel radius
                             2.2,                     // wheel width
                             8,                       // n. of rollers
                             roller_angle,            // angle of rollers
                             0.65,                    // max rad. of roller
                             0.1,                     // mass of single roller
                             0.2);                    // mass of the spindle
    auto my_link_shaftC = std::make_shared<ChLinkEngine>();
    my_link_shaftC->Initialize(spindle_C->GetBody(), mTrussPlatform->GetBody(), (f1 >> f2_wC).GetCoord());
    my_link_shaftC->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(my_link_shaftC->Get_spe_funct()))
        mfun->Set_yconst(0.0);
    mphysicalSystem.AddLink(my_link_shaftC);

    // Create the ground for the collision
    ChBodySceneNode* ground = (ChBodySceneNode*)addChBodySceneNode_easyBox(
        &mphysicalSystem, application.GetSceneManager(), 100.0, ChVector<>(0, -5, 0), ChQuaternion<>(1, 0, 0, 0),
        ChVector<>(200, 1, 200));
    ground->GetBody()->SetBodyFixed(true);
    ground->GetBody()->GetMaterialSurface()->SetFriction(STATIC_wheelfriction);

    video::ITexture* cubeMap = application.GetVideoDriver()->getTexture(GetChronoDataFile("cubetexture.png").c_str());
    ground->setMaterialTexture(0, cubeMap);

    // Prepare the physical system for the simulation

    mphysicalSystem.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);

    mphysicalSystem.SetSolverType(ChSolver::Type::SOR);

    mphysicalSystem.SetMaxItersSolverSpeed(30);
    mphysicalSystem.SetMaxItersSolverStab(30);

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    application.SetStepManage(true);
    application.SetTimestep(0.01);

    while (application.GetDevice()->run()) {
        application.GetVideoDriver()->beginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        // ADVANCE THE SIMULATION FOR ONE TIMESTEP
        application.DoStep();

        // change motor speeds depending on user setpoints from GUI

        ChVector<> imposed_speed(STATIC_x_speed, 0, STATIC_z_speed);
        ChFrame<> roll_twist(ChVector<>(0, -wheel_radius, 0), Q_from_AngAxis(-roller_angle, ChVector<>(0, 1, 0)));

        ChFrame<> abs_roll_wA = roll_twist >> f2_wA >> ChFrame<>(mTrussPlatform->GetBody()->GetCoord());
        double wheel_A_rotspeed =
            (STATIC_rot_speed * platform_radius) +
            ((abs_roll_wA.GetA().MatrT_x_Vect(imposed_speed)).x() / sin(roller_angle)) / wheel_radius;
        ChFrame<> abs_roll_wB = roll_twist >> f2_wB >> ChFrame<>(mTrussPlatform->GetBody()->GetCoord());
        double wheel_B_rotspeed =
            (STATIC_rot_speed * platform_radius) +
            ((abs_roll_wB.GetA().MatrT_x_Vect(imposed_speed)).x() / sin(roller_angle)) / wheel_radius;
        ChFrame<> abs_roll_wC = roll_twist >> f2_wC >> ChFrame<>(mTrussPlatform->GetBody()->GetCoord());
        double wheel_C_rotspeed =
            (STATIC_rot_speed * platform_radius) +
            ((abs_roll_wC.GetA().MatrT_x_Vect(imposed_speed)).x() / sin(roller_angle)) / wheel_radius;

        if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(my_link_shaftA->Get_spe_funct()))
            mfun->Set_yconst(wheel_A_rotspeed);
        if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(my_link_shaftB->Get_spe_funct()))
            mfun->Set_yconst(wheel_B_rotspeed);
        if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(my_link_shaftC->Get_spe_funct()))
            mfun->Set_yconst(wheel_C_rotspeed);

        application.GetVideoDriver()->endScene();
    }

    return 0;
}
