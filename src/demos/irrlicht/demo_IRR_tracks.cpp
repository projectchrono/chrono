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
// - modeling tracks with articulated shoes (as an example of complex model with
//   collisions and constraints)
// - using different meshes for collision and visualization
// - using clones of collision shapes
// - using SetFamilyMaskNoCollisionWithFamily, SetFamily etc. to avoid
//   collisions between different families of bodies.
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/geometry/ChTriangleMeshSoup.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_irrlicht/ChBodySceneNode.h"
#include "chrono_irrlicht/ChBodySceneNodeTools.h"
#include "chrono_irrlicht/ChIrrApp.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

// First of all, define a class for the 'tank' (that is, a set of
// bodies and links which are grouped within this class; so it is
// easier to manage data structures in this example).

class MySimpleTank {
  public:
    // THE DATA

    double throttleL;        // actual value 0...1 of gas throttle (left).
    double throttleR;        // actual value 0...1 of gas throttle (right).
    double max_motor_speed;  // the max rotation speed of the motor [rads/s]

    // The parts making the tank, as 3d Irrlicht scene nodes, each containing
    // the ChBody object
    // .. truss:
    std::shared_ptr<ChBody> truss;
    // .. right front suspension:
    std::shared_ptr<ChBody> wheelRF;
    std::shared_ptr<ChLinkLockRevolute> link_revoluteRF;
    // .. left front suspension:
    std::shared_ptr<ChBody> wheelLF;
    std::shared_ptr<ChLinkLockRevolute> link_revoluteLF;
    // .. right back suspension:
    std::shared_ptr<ChBody> wheelRB;
    std::shared_ptr<ChLinkMotorRotationSpeed> link_motorRB;
    // .. left back suspension:
    std::shared_ptr<ChBody> wheelLB;
    std::shared_ptr<ChLinkMotorRotationSpeed> link_motorLB;

    // THE FUNCTIONS

    // Build and initialize the tank, creating all bodies corresponding to
    // the various parts and adding them to the physical system - also creating
    // and adding constraints to the system.
    MySimpleTank(ChSystemNSC& my_system,        ///< the Chrono physical system
                 ISceneManager* msceneManager,  ///< the Irrlicht scene manager for 3d shapes
                 IVideoDriver* mdriver          ///< the Irrlicht video driver
    ) {
        throttleL = throttleR = 0;  // initially, gas throttle is 0.
        max_motor_speed = 10;

        double my = 0.5;  // left back hub pos
        double mx = 0;

        double shoelength = 0.2;
        double shoethickness = 0.06;
        double shoewidth = 0.3;
        double shoemass = 2;
        double radiustrack = 0.31;
        double wheeldiameter = 0.280 * 2;
        int nwrap = 6;
        int ntiles = 7;
        double rlwidth = 1.20;
        double passo = (ntiles + 1) * shoelength;

        ChVector<> cyl_displA(0, 0.075 + 0.02, 0);
        ChVector<> cyl_displB(0, -0.075 - 0.02, 0);
        double cyl_hthickness = 0.045;

        // --- The tank body ---

        truss = std::make_shared<ChBodyEasyMesh>(GetChronoDataFile("bulldozerB10.obj").c_str(), 1000, false, false, 0,
                                                 true);
        my_system.Add(truss);
        truss->SetPos(ChVector<>(mx + passo / 2, my + radiustrack, rlwidth / 2));
        truss->SetMass(350);
        truss->SetInertiaXX(ChVector<>(13.8, 13.5, 10));

        // --- Right Front suspension ---

        // Load a triangle mesh for wheel visualization
        IAnimatedMesh* irmesh_wheel_view = msceneManager->getMesh(GetChronoDataFile("wheel_view.obj").c_str());

        // ..the tank right-front wheel
        wheelRF =
            std::make_shared<ChBodyEasyMesh>(GetChronoDataFile("wheel_view.obj").c_str(), 1000, false, false, 0, true);
        my_system.Add(wheelRF);
        wheelRF->SetPos(ChVector<>(mx + passo, my + radiustrack, 0));
        wheelRF->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        wheelRF->SetMass(9.0);
        wheelRF->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
        wheelRF->GetMaterialSurfaceNSC()->SetFriction(1.0);

        wheelRF->GetCollisionModel()->ClearModel();
        wheelRF->GetCollisionModel()->AddCylinder(wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness, cyl_displA);
        wheelRF->GetCollisionModel()->AddCylinder(wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness, cyl_displB);
        wheelRF->GetCollisionModel()->BuildModel();
        wheelRF->SetCollide(true);

        auto color_wheel = std::make_shared<ChColorAsset>();
        color_wheel->SetColor(ChColor(0.2f, 0.2f, 0.2f));
        wheelRF->AddAsset(color_wheel);

        // .. create the revolute joint between the wheel and the truss
        link_revoluteRF = std::make_shared<ChLinkLockRevolute>();  // right, front, upper, 1
        link_revoluteRF->Initialize(wheelRF, truss, ChCoordsys<>(ChVector<>(mx + passo, my + radiustrack, 0), QUNIT));
        my_system.AddLink(link_revoluteRF);

        // --- Left Front suspension ---

        // ..the tank left-front wheel

        wheelLF =
            std::make_shared<ChBodyEasyMesh>(GetChronoDataFile("wheel_view.obj").c_str(), 1000, false, false, 0, true);
        my_system.Add(wheelLF);
        wheelLF->SetPos(ChVector<>(mx + passo, my + radiustrack, rlwidth));
        wheelLF->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        wheelLF->SetMass(9.0);
        wheelLF->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
        wheelLF->GetMaterialSurfaceNSC()->SetFriction(1.0);
        wheelLF->AddAsset(color_wheel);

        wheelLF->GetCollisionModel()->ClearModel();
        wheelLF->GetCollisionModel()->AddCylinder(wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness, cyl_displA);
        wheelLF->GetCollisionModel()->AddCylinder(wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness, cyl_displB);
        wheelLF->GetCollisionModel()->BuildModel();
        wheelLF->SetCollide(true);

        // .. create the revolute joint between the wheel and the truss
        link_revoluteLF = std::make_shared<ChLinkLockRevolute>();  // left, front, upper, 1
        link_revoluteLF->Initialize(wheelLF, truss,
                                    ChCoordsys<>(ChVector<>(mx + passo, my + radiustrack, rlwidth), QUNIT));
        my_system.AddLink(link_revoluteLF);

        // --- Right Back suspension ---

        // ..the tank right-back wheel

        wheelRB =
            std::make_shared<ChBodyEasyMesh>(GetChronoDataFile("wheel_view.obj").c_str(), 1000, false, false, 0, true);
        my_system.Add(wheelRB);
        wheelRB->SetPos(ChVector<>(mx, my + radiustrack, 0));
        wheelRB->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        wheelRB->SetMass(9.0);
        wheelRB->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
        wheelRB->GetMaterialSurfaceNSC()->SetFriction(1.0);
        wheelRB->AddAsset(color_wheel);

        wheelRB->GetCollisionModel()->ClearModel();
        wheelRB->GetCollisionModel()->AddCylinder(wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness, cyl_displA);
        wheelRB->GetCollisionModel()->AddCylinder(wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness, cyl_displB);
        wheelRB->GetCollisionModel()->BuildModel();
        wheelRB->SetCollide(true);

        // .. create the motor joint between the wheel and the truss (simplified motor model: just impose speed..)
        link_motorRB = std::make_shared<ChLinkMotorRotationSpeed>();
        link_motorRB->SetSpeedFunction(std::make_shared<ChFunction_Const>());  // actually, default function type
        link_motorRB->Initialize(wheelRB, truss, ChFrame<>(ChVector<>(mx, my + radiustrack, 0), QUNIT));
        my_system.AddLink(link_motorRB);

        // --- Left Back suspension ---

        // ..the tank left-back wheel

        wheelLB =
            std::make_shared<ChBodyEasyMesh>(GetChronoDataFile("wheel_view.obj").c_str(), 1000, false, false, 0, true);
        my_system.Add(wheelLB);
        wheelLB->SetPos(ChVector<>(mx, my + radiustrack, rlwidth));
        wheelLB->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        wheelLB->SetMass(9.0);
        wheelLB->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
        wheelLB->GetMaterialSurfaceNSC()->SetFriction(1.0);
        wheelLB->AddAsset(color_wheel);

        wheelLB->GetCollisionModel()->ClearModel();
        wheelLB->GetCollisionModel()->AddCylinder(wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness, cyl_displA);
        wheelLB->GetCollisionModel()->AddCylinder(wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness, cyl_displB);
        wheelLB->GetCollisionModel()->BuildModel();
        wheelLB->SetCollide(true);

        // .. create the motor joint between the wheel and the truss (simplified motor model: just impose speed..)
        link_motorLB = std::make_shared<ChLinkMotorRotationSpeed>();
        link_motorLB->SetSpeedFunction(std::make_shared<ChFunction_Const>());  // actually, default function type
        link_motorLB->Initialize(wheelLB, truss, ChFrame<>(ChVector<>(mx, my + radiustrack, rlwidth), QUNIT));
        my_system.AddLink(link_motorLB);

        //--- TRACKS ---

        // Load a triangle mesh for collision
        IAnimatedMesh* irmesh_shoe_collision = msceneManager->getMesh(GetChronoDataFile("shoe_collision.obj").c_str());

        auto trimesh = std::make_shared<ChTriangleMeshSoup>();
        fillChTrimeshFromIrlichtMesh(trimesh.get(), irmesh_shoe_collision->getMesh(0));

        ChVector<> mesh_displacement(shoelength * 0.5, 0, 0);    // as mesh origin is not in body center of mass
        ChVector<> joint_displacement(-shoelength * 0.5, 0, 0);  // pos. of shoe-shoe constraint, relative to COG.
        ChVector<> pin_displacement = mesh_displacement + ChVector<>(0, 0.05, 0);

        chrono::ChVector<> position;
        chrono::ChQuaternion<> rotation;

        for (int side = 0; side < 2; side++) {
            mx = 0;
            mx += shoelength;

            double mz = 0;

            if (side == 0)
                mz = 0;
            else
                mz = rlwidth;

            position.Set(mx, my, mz);
            rotation = QUNIT;

            // Create sample body (with empty collision shape; later create the collision model by adding the
            // coll.shapes)
            auto firstBodyShoe = std::make_shared<ChBody>();
            my_system.Add(firstBodyShoe);
            firstBodyShoe->SetMass(shoemass);
            firstBodyShoe->SetPos(position);
            firstBodyShoe->SetRot(rotation);
            firstBodyShoe->SetInertiaXX(ChVector<>(0.1, 0.1, 0.1));

            // Visualization:
            auto shoe_mesh = std::make_shared<ChTriangleMeshShape>();
            firstBodyShoe->AddAsset(shoe_mesh);
            shoe_mesh->GetMesh()->LoadWavefrontMesh(GetChronoDataFile("shoe_view.obj").c_str());
            shoe_mesh->GetMesh()->Transform(-mesh_displacement, ChMatrix33<>(1));
            shoe_mesh->SetVisible(true);

            // Visualize collision mesh
            auto shoe_coll_mesh = std::make_shared<ChTriangleMeshShape>();
            firstBodyShoe->AddAsset(shoe_coll_mesh);
            shoe_coll_mesh->GetMesh()->LoadWavefrontMesh(GetChronoDataFile("shoe_collision.obj").c_str());
            shoe_coll_mesh->GetMesh()->Transform(-mesh_displacement, ChMatrix33<>(1));
            shoe_coll_mesh->SetVisible(false);

            // Collision:
            firstBodyShoe->GetCollisionModel()->SetSafeMargin(0.004);  // inward safe margin
            firstBodyShoe->GetCollisionModel()->SetEnvelope(0.010);    // distance of the outward "collision envelope"
            firstBodyShoe->GetCollisionModel()->ClearModel();
            firstBodyShoe->GetCollisionModel()->AddTriangleMesh(trimesh, false, false, mesh_displacement,
                                                                ChMatrix33<>(1), 0.005);
            firstBodyShoe->GetCollisionModel()->BuildModel();  // Creates the collision model
            firstBodyShoe->SetCollide(true);

            // Avoid creation of contacts with neighbouring shoes, using
            // a collision family (=3) that does not collide with itself
            firstBodyShoe->GetCollisionModel()->SetFamily(3);
            firstBodyShoe->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);

            std::shared_ptr<ChBody> previous_rigidBodyShoe;
            previous_rigidBodyShoe = firstBodyShoe;

            for (int nshoe = 1; nshoe < ntiles; nshoe++) {
                mx += shoelength;
                position.Set(mx, my, mz);

                auto rigidBodyShoe =
                    MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                previous_rigidBodyShoe = rigidBodyShoe;
            }
            for (int nshoe = 0; nshoe < nwrap; nshoe++) {
                double alpha = (CH_C_PI / ((double)(nwrap - 1.0))) * ((double)nshoe);

                double lx = mx + shoelength + radiustrack * sin(alpha);
                double ly = my + radiustrack - radiustrack * cos(alpha);
                position.Set(lx, ly, mz);
                rotation = chrono::Q_from_AngAxis(alpha, ChVector<>(0, 0, 1));
                auto rigidBodyShoe =
                    MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                previous_rigidBodyShoe = rigidBodyShoe;
            }
            for (int nshoe = (ntiles - 1); nshoe >= 0; nshoe--) {
                position.Set(mx, my + 2 * radiustrack, mz);

                auto rigidBodyShoe =
                    MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                previous_rigidBodyShoe = rigidBodyShoe;

                mx -= shoelength;
            }
            for (int nshoe = 0; nshoe < nwrap; nshoe++) {
                double alpha = CH_C_PI + (CH_C_PI / ((double)(nwrap - 1.0))) * ((double)nshoe);

                double lx = mx + 0 + radiustrack * sin(alpha);
                double ly = my + radiustrack - radiustrack * cos(alpha);
                position.Set(lx, ly, mz);
                rotation = chrono::Q_from_AngAxis(alpha, ChVector<>(0, 0, 1));
                auto rigidBodyShoe =
                    MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                previous_rigidBodyShoe = rigidBodyShoe;
            }
            // close track
            ChVector<> linkpos = firstBodyShoe->Point_Body2World(joint_displacement);
            auto link_revolute_shoeshoe = std::make_shared<ChLinkLockRevolute>();
            link_revolute_shoeshoe->Initialize(firstBodyShoe, previous_rigidBodyShoe, ChCoordsys<>(linkpos, QUNIT));
            my_system.AddLink(link_revolute_shoeshoe);
        }
    }

    // Delete the tank object, deleting also all bodies corresponding to
    // the various parts and removing them from the physical system.  Also
    // removes constraints from the system.
    ~MySimpleTank() {
        ChSystem* mysystem = truss->GetSystem();  // trick to get the system here

        mysystem->Remove(link_revoluteRF);
        mysystem->Remove(link_revoluteLF);
        mysystem->Remove(link_motorRB);
        mysystem->Remove(link_motorLB);
        mysystem->Remove(truss);
        mysystem->Remove(wheelRF);
        mysystem->Remove(wheelLF);
        mysystem->Remove(wheelRB);
        mysystem->Remove(wheelLB);
    }

    // Utility function to create quickly a track shoe connected to the previous one
    std::shared_ptr<ChBody> MakeShoe(
        std::shared_ptr<ChBody> previous_shoe,  // will be linked with this one with revolute joint
        std::shared_ptr<ChBody>
            template_shoe,        // collision geometry will be shared with this body, to save memory&cpu time.
        ChVector<> position,      // position
        ChQuaternion<> rotation,  // orientation
        ChSystemNSC& my_system,   // the physical system
        chrono::ChVector<> joint_displacement  // position of shoe-shoe constraint, relative to COG.
    ) {
        auto rigidBodyShoe = std::shared_ptr<ChBody>(template_shoe.get()->Clone());
        ////rigidBodyShoe->SetSystem(template_shoe->GetSystem()); //// RADU: this seems redundant as the system will be
        ///set when adding the body
        rigidBodyShoe->SetPos(position);
        rigidBodyShoe->SetRot(rotation);
        my_system.Add(rigidBodyShoe);

        rigidBodyShoe->GetCollisionModel()->ClearModel();
        rigidBodyShoe->GetCollisionModel()->AddCopyOfAnotherModel(template_shoe->GetCollisionModel().get());
        rigidBodyShoe->GetCollisionModel()->BuildModel();
        rigidBodyShoe->SetCollide(true);

        // Other settings are already copied from template_shoe, except for family and mask.
        // Avoid creation of contacts with neighbouring shoes:
        rigidBodyShoe->GetCollisionModel()->SetFamily(3);
        rigidBodyShoe->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);

        // Create revolute constraint
        if (previous_shoe) {
            ChVector<> linkpos = rigidBodyShoe->Point_Body2World(joint_displacement);
            auto link_revolute_shoeshoe = std::make_shared<ChLinkLockRevolute>();
            link_revolute_shoeshoe->Initialize(rigidBodyShoe, previous_shoe, ChCoordsys<>(linkpos, QUNIT));
            my_system.AddLink(link_revolute_shoeshoe);
        }

        return rigidBodyShoe;
    }
};

// Define a MyEventReceiver class which will be used to manage input
// from the GUI graphical user interface (the interface will
// be created with the basic -yet flexible- platform
// independent toolset of Irrlicht).

class MyEventReceiver : public IEventReceiver {
  public:
    MyEventReceiver(ChIrrAppInterface* myapp, MySimpleTank* atank) {
        // store pointer application
        application = myapp;
        // store pointer to other stuff
        mtank = atank;

        // ..add a GUI slider to control throttle left via mouse
        scrollbar_throttleL =
            application->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(510, 20, 650, 35), 0, 101);
        scrollbar_throttleL->setMax(100);
        scrollbar_throttleL->setPos(50);
        text_throttleL =
            application->GetIGUIEnvironment()->addStaticText(L"Left throttle ", rect<s32>(650, 20, 750, 35), false);

        // ..add a GUI slider to control gas throttle right via mouse
        scrollbar_throttleR =
            application->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(510, 45, 650, 60), 0, 102);
        scrollbar_throttleR->setMax(100);
        scrollbar_throttleR->setPos(50);
        text_throttleR =
            application->GetIGUIEnvironment()->addStaticText(L"Right throttle", rect<s32>(650, 45, 750, 60), false);
    }

    bool OnEvent(const SEvent& event) {
        // check if user moved the sliders with mouse..
        if (event.EventType == EET_GUI_EVENT) {
            s32 id = event.GUIEvent.Caller->getID();
            IGUIEnvironment* env = application->GetIGUIEnvironment();

            switch (event.GUIEvent.EventType) {
                case EGET_SCROLL_BAR_CHANGED:
                    if (id == 101) {  // id of 'throttleL' slider..
                        s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        double newthrottle = ((double)(pos)-50) / 50.0;
                        this->mtank->throttleL = newthrottle;
                        auto mfun = std::static_pointer_cast<ChFunction_Const>(mtank->link_motorLB->GetSpeedFunction());
                        mfun->Set_yconst(newthrottle * 6);
                        return true;
                    }
                    if (id == 102) {  // id of 'throttleR' slider..
                        s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        double newthrottle = ((double)(pos)-50) / 50.0;
                        this->mtank->throttleR = newthrottle;
                        auto mfun = std::static_pointer_cast<ChFunction_Const>(mtank->link_motorRB->GetSpeedFunction());
                        mfun->Set_yconst(newthrottle * 6);
                        return true;
                    }
                    break;
                default:
                    break;
            }
        }

        return false;
    }

  private:
    ChIrrAppInterface* application;
    MySimpleTank* mtank;

    IGUIStaticText* text_throttleL;
    IGUIScrollBar* scrollbar_throttleL;
    IGUIStaticText* text_throttleR;
    IGUIScrollBar* scrollbar_throttleR;
};

//
// This is the program which is executed
//

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // 1- Create a ChronoENGINE physical system: all bodies and constraints
    //    will be handled by this ChSystemNSC object.
    ChSystemNSC my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Modeling a simplified   tank", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add logo, camera, lights and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 0, -6), core::vector3df(-2, 2, 0));

    // 2- Create the rigid bodies of the simpified tank suspension mechanical system
    //   maybe setting position/mass/inertias of
    //   their center of mass (COG) etc.

    // ..the world
    ChBodySceneNode* my_ground = (ChBodySceneNode*)addChBodySceneNode_easyBox(
        &my_system, application.GetSceneManager(), 1.0, ChVector<>(0, -1, 0), QUNIT, ChVector<>(60, 2, 60));
    my_ground->GetBody()->SetBodyFixed(true);
    my_ground->GetBody()->SetCollide(true);
    my_ground->GetBody()->GetMaterialSurfaceNSC()->SetFriction(1.0);
    video::ITexture* groundMap = application.GetVideoDriver()->getTexture(GetChronoDataFile("blu.png").c_str());
    my_ground->setMaterialTexture(0, groundMap);

    // ..some obstacles on the ground:
    for (int i = 0; i < 50; i++) {
        ChBodySceneNode* my_obstacle = (ChBodySceneNode*)addChBodySceneNode_easyBox(
            &my_system, application.GetSceneManager(), 3.0,
            ChVector<>(-6 + 6 * ChRandom(), 2 + 1 * ChRandom(), 6 * ChRandom()),
            Q_from_AngAxis(ChRandom() * CH_C_PI, VECT_Y),
            ChVector<>(0.6 * (1 - 0.4 * ChRandom()), 0.08, 0.3 * (1 - 0.4 * ChRandom())));
        my_obstacle->addShadowVolumeSceneNode();
    }

    // ..the tank (this class - see above - is a 'set' of bodies and links, automatically added at creation)
    MySimpleTank* mytank = new MySimpleTank(my_system, application.GetSceneManager(), application.GetVideoDriver());

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    //
    // USER INTERFACE
    //

    // Create some graphical-user-interface (GUI) items to show on the screen.
    // This requires an event receiver object.
    MyEventReceiver receiver(&application, mytank);
    // note how to add the custom event receiver to the default interface:
    application.SetUserEventReceiver(&receiver);

    //
    // SETTINGS
    //

    my_system.SetMaxItersSolverSpeed(100);  // the higher, the easier to keep the constraints 'mounted'.
    my_system.SetSolverType(ChSolver::Type::SOR);

    //
    // THE SOFT-REAL-TIME CYCLE, SHOWING THE SIMULATION
    //

    application.SetStepManage(true);
    application.SetTimestep(0.03);
    application.SetTryRealtime(true);

    while (application.GetDevice()->run()) {
        // Irrlicht must prepare frame to draw
        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        // .. draw solid 3D items (boxes, cylinders, shapes) belonging to Irrlicht scene, if any
        application.DrawAll();

        // .. draw also a grid (rotated so that it's horizontal)
        ChIrrTools::drawGrid(application.GetVideoDriver(), 2, 2, 30, 30,
                             ChCoordsys<>(ChVector<>(0, 0.01, 0), Q_from_AngX(CH_C_PI_2)),
                             video::SColor(255, 60, 60, 60), true);

        // HERE CHRONO INTEGRATION IS PERFORMED:

        application.DoStep();

        application.EndScene();
    }

    if (mytank)
        delete mytank;

    return 0;
}
