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

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/assets/ChBarrelShape.h"
#include "chrono_irrlicht/ChIrrApp.h"


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
std::shared_ptr<ChBody> create_mecanum_wheel(ChSystemNSC& mphysicalSystem,
                                             ChVector<> shaft_position,
                                             ChQuaternion<> shaft_alignment,
                                             double wheel_radius,
                                             double wheel_width,
                                             int n_rollers,
                                             double roller_angle,
                                             double roller_midradius,
                                             double roller_density,
                                             double spindle_density) {
    ChFrameMoving<> ftot(shaft_position, shaft_alignment);  // will be used to transform pos & rot of all objects

    auto mCentralWheel = chrono_types::make_shared<ChBodyEasyCylinder>(wheel_radius / 2, wheel_width,  // radius, height
                                                                       spindle_density,                // density
                                                                       true,                           // visualize
                                                                       false);                         // no collision
    mCentralWheel->SetPos(shaft_position);
    mCentralWheel->SetRot(shaft_alignment);
	mphysicalSystem.Add(mCentralWheel);

	auto mtexturepw = chrono_types::make_shared<ChTexture>();
    mtexturepw->SetTextureFilename(GetChronoDataFile("textures/pinkwhite.png"));
    mCentralWheel->AddAsset(mtexturepw);

    auto wheel_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    wheel_mat->SetFriction(STATIC_wheelfriction);

    double half_length_roller = 0.5 * wheel_width * 1.0 / (cos(roller_angle));
    double roller_elliptical_rad_Hor = wheel_radius;
    double roller_elliptical_rad_Vert = wheel_radius * 1.0 / (cos(roller_angle));

    for (int iroller = 0; iroller < n_rollers; iroller++) {
        double pitch = CH_C_2PI * ((double)iroller / (double)n_rollers);

		double Roffset = -(wheel_radius - roller_midradius);

        // Create the roller
		auto mRoller = chrono_types::make_shared<ChBody>();
		mphysicalSystem.Add(mRoller);

		// move it to slanted aligment
		ChFrameMoving<> f1( ChVector<>(0, 0, -(wheel_radius - roller_midradius)),
							Q_from_AngAxis(roller_angle, ChVector<>(0, 0, 1)));
        ChFrameMoving<> f2( ChVector<>(0, 0, 0), 
							Q_from_AngAxis(pitch, ChVector<>(0, 1, 0)));
        ChFrameMoving<> f3 = f1 >> f2 >> ftot;
        mRoller->ConcatenatePreTransformation(f3);

		// approximate mass & inertia to a cylinder:
		mRoller->SetMass(utils::CalcCylinderVolume(roller_elliptical_rad_Hor + Roffset, 2 * half_length_roller) * roller_density);
		mRoller->SetInertia(utils::CalcCylinderGyration(roller_elliptical_rad_Hor + Roffset, 2 * half_length_roller) * roller_density);

        // add collision shape
        mRoller->GetCollisionModel()->ClearModel();
        mRoller->GetCollisionModel()->AddBarrel(wheel_mat,                                              //
                                                -half_length_roller, +half_length_roller,               //
                                                roller_elliptical_rad_Vert, roller_elliptical_rad_Hor,  //
                                                Roffset);
        mRoller->GetCollisionModel()->BuildModel();
        mRoller->SetCollide(true);

        // add visualization shape
        auto mrollershape =
            chrono_types::make_shared<ChBarrelShape>(-half_length_roller, +half_length_roller,               //
                                                     roller_elliptical_rad_Vert, roller_elliptical_rad_Hor,  //
                                                     Roffset);
        mRoller->AddAsset(mrollershape);

        // Make the revolute joint between the roller and the central wheel
        // (preconcatenate rotation 90 degrees on X, to set axis of revolute joint)
        ChFrameMoving<> fr(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI / 2.0, ChVector<>(1, 0, 0)));
        ChFrameMoving<> frabs = fr >> f3;
        auto my_link_roller = chrono_types::make_shared<ChLinkLockRevolute>();
        my_link_roller->Initialize(mRoller, mCentralWheel, frabs.GetCoord());
        mphysicalSystem.AddLink(my_link_roller);

    }

    return mCentralWheel;
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Mecanum robot simulator", core::dimension2d<u32>(800, 600));
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 14, -20));

    // create text with info
    IGUIStaticText* textFPS = application.GetIGUIEnvironment()->addStaticText(
        L"Use keys Q,W, A,Z, E,R to move the robot", rect<s32>(150, 10, 430, 40), true);

    // This is for GUI tweaking of system parameters..
    MyEventReceiver receiver(&application);
    // note how to add a custom event receiver to the default interface:
    application.SetUserEventReceiver(&receiver);

    double platform_radius = 8;
    double wheel_radius = 3;
    double roller_angle = CH_C_PI / 4;

    // Create the robot truss, as a circular platform
    auto mTrussPlatform = chrono_types::make_shared<ChBodyEasyCylinder>(platform_radius * 0.7, 2,  // radius, height
                                                                        1000,                      // density
                                                                        true,                      // visualize
                                                                        false);                    // no collision
    mphysicalSystem.Add(mTrussPlatform);

	


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

    auto spindle_A =
        create_mecanum_wheel(mphysicalSystem, 
                             ftot_wA.GetCoord().pos,  // wheel position
                             ftot_wA.GetCoord().rot,  // wheel alignment
                             wheel_radius,            // wheel radius
                             2.2,                     // wheel width
                             8,                       // n. of rollers
                             roller_angle,            // angle of rollers
                             0.65,                    // max rad. of roller
                             1000,                    // density of roller
                             1000);                   // density of the spindle

    auto my_link_shaftA = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    my_link_shaftA->Initialize(spindle_A, mTrussPlatform, (f1 >> f2_wA));
    my_link_shaftA->SetSpeedFunction(chrono_types::make_shared<ChFunction_Const>(0));
    mphysicalSystem.AddLink(my_link_shaftA);

    auto spindle_B =
        create_mecanum_wheel(mphysicalSystem, 
                             ftot_wB.GetCoord().pos,  // wheel position
                             ftot_wB.GetCoord().rot,  // wheel alignment
                             wheel_radius,            // wheel radius
                             2.2,                     // wheel width
                             8,                       // n. of rollers
                             roller_angle,            // angle of rollers
                             0.65,                    // max rad. of roller
                             1000,                    // density of roller
                             1000);                   // density of the spindle

    auto my_link_shaftB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    my_link_shaftB->Initialize(spindle_B, mTrussPlatform, (f1 >> f2_wB));
    my_link_shaftB->SetSpeedFunction(chrono_types::make_shared<ChFunction_Const>(0));
    mphysicalSystem.AddLink(my_link_shaftB);

    auto spindle_C =
        create_mecanum_wheel(mphysicalSystem, 
                             ftot_wC.GetCoord().pos,  // wheel position
                             ftot_wC.GetCoord().rot,  // wheel alignment
                             wheel_radius,            // wheel radius
                             2.2,                     // wheel width
                             8,                       // n. of rollers
                             roller_angle,            // angle of rollers
                             0.65,                    // max rad. of roller
                             1000,                    // density of roller
                             1000);                   // density of the spindle

    auto my_link_shaftC = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    my_link_shaftC->Initialize(spindle_C, mTrussPlatform, (f1 >> f2_wC));
    my_link_shaftC->SetSpeedFunction(chrono_types::make_shared<ChFunction_Const>(0));
    mphysicalSystem.AddLink(my_link_shaftC);

    // Create the ground for the collision
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ground_mat->SetFriction(STATIC_wheelfriction);

    auto ground = chrono_types::make_shared<ChBodyEasyBox>(200, 1, 200,  // size
                                                           1000,         // density
                                                           true,         // visualize
                                                           true,         // collide
                                                           ground_mat);  // contact material
    ground->SetPos(ChVector<>(0, -5, 0));
    ground->SetBodyFixed(true);
	mphysicalSystem.Add(ground);

	auto mtexture = chrono_types::make_shared<ChTexture>();
    mtexture->SetTextureFilename(GetChronoDataFile("textures/concrete.jpg"));
    mtexture->SetTextureScale(100, 100);

    ground->AddAsset(mtexture);



	// Use this function for adding a ChIrrNodeAsset to all already created items.
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();
    application.AssetUpdateAll();

    // Prepare the physical system for the simulation

    mphysicalSystem.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);

    mphysicalSystem.SetSolverType(ChSolver::Type::PSOR);
    mphysicalSystem.SetSolverMaxIterations(30);

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    application.SetTimestep(0.01);
    application.SetTryRealtime(true);

    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        // ADVANCE THE SIMULATION FOR ONE TIMESTEP
        application.DoStep();

        // change motor speeds depending on user setpoints from GUI

        ChVector<> imposed_speed(STATIC_x_speed, 0, STATIC_z_speed);
        ChFrame<> roll_twist(ChVector<>(0, -wheel_radius, 0), Q_from_AngAxis(-roller_angle, ChVector<>(0, 1, 0)));

        ChFrame<> abs_roll_wA = roll_twist >> f2_wA >> ChFrame<>(mTrussPlatform->GetCoord());
        double wheel_A_rotspeed =
            (STATIC_rot_speed * platform_radius) +
            ((abs_roll_wA.GetA().transpose() * imposed_speed).x() / sin(roller_angle)) / wheel_radius;
        ChFrame<> abs_roll_wB = roll_twist >> f2_wB >> ChFrame<>(mTrussPlatform->GetCoord());
        double wheel_B_rotspeed =
            (STATIC_rot_speed * platform_radius) +
            ((abs_roll_wB.GetA().transpose() * imposed_speed).x() / sin(roller_angle)) / wheel_radius;
        ChFrame<> abs_roll_wC = roll_twist >> f2_wC >> ChFrame<>(mTrussPlatform->GetCoord());
        double wheel_C_rotspeed =
            (STATIC_rot_speed * platform_radius) +
            ((abs_roll_wC.GetA().transpose() * imposed_speed).x() / sin(roller_angle)) / wheel_radius;

        if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(my_link_shaftA->GetSpeedFunction()))
            mfun->Set_yconst(wheel_A_rotspeed);
        if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(my_link_shaftB->GetSpeedFunction()))
            mfun->Set_yconst(wheel_B_rotspeed);
        if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(my_link_shaftC->GetSpeedFunction()))
            mfun->Set_yconst(wheel_C_rotspeed);

        application.EndScene();
    }

    return 0;
}
