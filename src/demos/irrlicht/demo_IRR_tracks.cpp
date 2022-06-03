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
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_irrlicht/ChIrrMeshTools.h"

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
    MySimpleTank(ChSystemNSC& sys) {
        throttleL = throttleR = 0;  // initially, gas throttle is 0.
        max_motor_speed = 10;

        double my = 0.5;  // left back hub pos
        double mx = 0;

        double shoelength = 0.2;
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

        truss = chrono_types::make_shared<ChBodyEasyMesh>(                   //
            GetChronoDataFile("models/bulldozer/bulldozerB10.obj").c_str(),  // file name
            1000,                                                            // density
            false,                                                           // do not evaluate mass automatically
            true,                                                            // create visualization asset
            false,                                                           // do not collide
            nullptr,                                                         // no need for contact material
            0                                                                // swept sphere radius
        );
        sys.Add(truss);
        truss->SetPos(ChVector<>(mx + passo / 2, my + radiustrack, rlwidth / 2));
        truss->SetMass(350);
        truss->SetInertiaXX(ChVector<>(13.8, 13.5, 10));

        // --- Contact and visualization materials for wheels ---

        auto wheel_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
        wheel_mat->SetFriction(1.0);

        auto wheel_mat_vis = chrono_types::make_shared<ChVisualMaterial>();
        wheel_mat_vis->SetDiffuseColor(ChColor(0.2f, 0.2f, 0.2f));

        // --- Right Front suspension ---

        // ..the tank right-front wheel
        wheelRF = chrono_types::make_shared<ChBodyEasyMesh>(               //
            GetChronoDataFile("models/bulldozer/wheel_view.obj").c_str(),  // data file
            1000,                                                          // density
            false,                                                         // do not compute mass and inertia
            true,                                                          // visualization?
            false,                                                         // collision?
            nullptr,                                                       // no need for contact material
            0);                                                            // mesh sweep sphere radius

        sys.Add(wheelRF);
        wheelRF->SetPos(ChVector<>(mx + passo, my + radiustrack, 0));
        wheelRF->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        wheelRF->SetMass(9.0);
        wheelRF->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));

        wheelRF->GetCollisionModel()->ClearModel();
        wheelRF->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
                                                  cyl_displA);
        wheelRF->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
                                                  cyl_displB);
        wheelRF->GetCollisionModel()->BuildModel();
        wheelRF->SetCollide(true);

        wheelRF->GetVisualShape(0)->SetMaterial(0, wheel_mat_vis);

        // .. create the revolute joint between the wheel and the truss
        link_revoluteRF = chrono_types::make_shared<ChLinkLockRevolute>();  // right, front, upper, 1
        link_revoluteRF->Initialize(wheelRF, truss, ChCoordsys<>(ChVector<>(mx + passo, my + radiustrack, 0), QUNIT));
        sys.AddLink(link_revoluteRF);

        // --- Left Front suspension ---

        // ..the tank left-front wheel

        wheelLF = chrono_types::make_shared<ChBodyEasyMesh>(               //
            GetChronoDataFile("models/bulldozer/wheel_view.obj").c_str(),  // data file
            1000,                                                          // density
            false,                                                         // do not compute mass and inertia
            true,                                                          // visualization?
            false,                                                         // collision?
            nullptr,                                                       // no need for contact material
            0);                                                            // mesh sweep sphere radius

        sys.Add(wheelLF);
        wheelLF->SetPos(ChVector<>(mx + passo, my + radiustrack, rlwidth));
        wheelLF->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        wheelLF->SetMass(9.0);
        wheelLF->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));

        wheelLF->GetCollisionModel()->ClearModel();
        wheelLF->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
                                                  cyl_displA);
        wheelLF->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
                                                  cyl_displB);
        wheelLF->GetCollisionModel()->BuildModel();
        wheelLF->SetCollide(true);

        wheelLF->GetVisualShape(0)->SetMaterial(0, wheel_mat_vis);

        // .. create the revolute joint between the wheel and the truss
        link_revoluteLF = chrono_types::make_shared<ChLinkLockRevolute>();  // left, front, upper, 1
        link_revoluteLF->Initialize(wheelLF, truss,
                                    ChCoordsys<>(ChVector<>(mx + passo, my + radiustrack, rlwidth), QUNIT));
        sys.AddLink(link_revoluteLF);

        // --- Right Back suspension ---

        // ..the tank right-back wheel

        wheelRB = chrono_types::make_shared<ChBodyEasyMesh>(       //
            GetChronoDataFile("models/bulldozer/wheel_view.obj"),  // data file
            1000,                                                  // density
            false,                                                 // do not compute mass and inertia
            true,                                                  // visualization?
            false,                                                 // collision?
            nullptr,                                               // no need for contact material
            0);                                                    // mesh sweep sphere radius

        sys.Add(wheelRB);
        wheelRB->SetPos(ChVector<>(mx, my + radiustrack, 0));
        wheelRB->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        wheelRB->SetMass(9.0);
        wheelRB->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));

        wheelRB->GetCollisionModel()->ClearModel();
        wheelRB->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
                                                  cyl_displA);
        wheelRB->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
                                                  cyl_displB);
        wheelRB->GetCollisionModel()->BuildModel();
        wheelRB->SetCollide(true);

        wheelRB->GetVisualShape(0)->SetMaterial(0, wheel_mat_vis);

        // .. create the motor joint between the wheel and the truss (simplified motor model: just impose speed..)
        link_motorRB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        link_motorRB->SetSpeedFunction(
            chrono_types::make_shared<ChFunction_Const>());  // actually, default function type
        link_motorRB->Initialize(wheelRB, truss, ChFrame<>(ChVector<>(mx, my + radiustrack, 0), QUNIT));
        sys.AddLink(link_motorRB);

        // --- Left Back suspension ---

        // ..the tank left-back wheel

        wheelLB = chrono_types::make_shared<ChBodyEasyMesh>(       //
            GetChronoDataFile("models/bulldozer/wheel_view.obj"),  // data file
            1000,                                                  // density
            false,                                                 // do not compute mass and inertia
            true,                                                  // visualization?
            false,                                                 // collision?
            nullptr,                                               // no need for contact material
            0);                                                    // mesh sweep sphere radius

        sys.Add(wheelLB);
        wheelLB->SetPos(ChVector<>(mx, my + radiustrack, rlwidth));
        wheelLB->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        wheelLB->SetMass(9.0);
        wheelLB->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));

        wheelLB->GetCollisionModel()->ClearModel();
        wheelLB->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
                                                  cyl_displA);
        wheelLB->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
                                                  cyl_displB);
        wheelLB->GetCollisionModel()->BuildModel();
        wheelLB->SetCollide(true);

        wheelLB->GetVisualShape(0)->SetMaterial(0, wheel_mat_vis);

        // .. create the motor joint between the wheel and the truss (simplified motor model: just impose speed..)
        link_motorLB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        link_motorLB->SetSpeedFunction(
            chrono_types::make_shared<ChFunction_Const>());  // actually, default function type
        link_motorLB->Initialize(wheelLB, truss, ChFrame<>(ChVector<>(mx, my + radiustrack, rlwidth), QUNIT));
        sys.AddLink(link_motorLB);

        //--- TRACKS ---

        // Shared visualization models
        auto shoe_trimesh =
            ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("models/bulldozer/shoe_view.obj"));
        ////shoe_trimesh->Transform(-mesh_displacement, ChMatrix33<>(1));
        auto shoe_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
        shoe_mesh->SetMesh(shoe_trimesh);
        shoe_mesh->SetVisible(true);

        auto shoe_coll_trimesh =
            ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("models/bulldozer/shoe_collision.obj"));
        ////shoe_coll_trimesh->Transform(-mesh_displacement, ChMatrix33<>(1));
        auto shoe_coll_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
        shoe_coll_mesh->SetMesh(shoe_coll_trimesh);
        shoe_coll_mesh->SetVisible(false);

        // Load a triangle mesh for collision
        auto trimesh =
            ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("models/bulldozer/shoe_collision.obj"));

        ChVector<> mesh_displacement(shoelength * 0.5, 0, 0);    // as mesh origin is not in body center of mass
        ChVector<> joint_displacement(-shoelength * 0.5, 0, 0);  // pos. of shoe-shoe constraint, relative to COG.

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
            auto firstBodyShoe = chrono_types::make_shared<ChBody>();
            sys.Add(firstBodyShoe);
            firstBodyShoe->SetMass(shoemass);
            firstBodyShoe->SetPos(position);
            firstBodyShoe->SetRot(rotation);
            firstBodyShoe->SetInertiaXX(ChVector<>(0.1, 0.1, 0.1));

            // Visualization:
            firstBodyShoe->AddVisualShape(shoe_mesh, ChFrame<>(-mesh_displacement, ChMatrix33<>(1)));
            firstBodyShoe->AddVisualShape(shoe_coll_mesh, ChFrame<>(-mesh_displacement, ChMatrix33<>(1)));

            // Collision:
            firstBodyShoe->GetCollisionModel()->SetSafeMargin(0.004);  // inward safe margin
            firstBodyShoe->GetCollisionModel()->SetEnvelope(0.010);    // distance of the outward "collision envelope"
            firstBodyShoe->GetCollisionModel()->ClearModel();
            firstBodyShoe->GetCollisionModel()->AddTriangleMesh(chrono_types::make_shared<ChMaterialSurfaceNSC>(),
                                                                trimesh, false, false, mesh_displacement,
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
                    MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, sys, joint_displacement);

                rigidBodyShoe->AddVisualShape(shoe_mesh, ChFrame<>(-mesh_displacement, ChMatrix33<>(1)));
                rigidBodyShoe->AddVisualShape(shoe_coll_mesh, ChFrame<>(-mesh_displacement, ChMatrix33<>(1)));

                previous_rigidBodyShoe = rigidBodyShoe;
            }
            for (int nshoe = 0; nshoe < nwrap; nshoe++) {
                double alpha = (CH_C_PI / ((double)(nwrap - 1.0))) * ((double)nshoe);

                double lx = mx + shoelength + radiustrack * sin(alpha);
                double ly = my + radiustrack - radiustrack * cos(alpha);
                position.Set(lx, ly, mz);
                rotation = chrono::Q_from_AngAxis(alpha, ChVector<>(0, 0, 1));
                auto rigidBodyShoe =
                    MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, sys, joint_displacement);

                rigidBodyShoe->AddVisualShape(shoe_mesh, ChFrame<>(-mesh_displacement, ChMatrix33<>(1)));
                rigidBodyShoe->AddVisualShape(shoe_coll_mesh, ChFrame<>(-mesh_displacement, ChMatrix33<>(1)));

                previous_rigidBodyShoe = rigidBodyShoe;
            }
            for (int nshoe = (ntiles - 1); nshoe >= 0; nshoe--) {
                position.Set(mx, my + 2 * radiustrack, mz);

                auto rigidBodyShoe =
                    MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, sys, joint_displacement);

                rigidBodyShoe->AddVisualShape(shoe_mesh, ChFrame<>(-mesh_displacement, ChMatrix33<>(1)));
                rigidBodyShoe->AddVisualShape(shoe_coll_mesh, ChFrame<>(-mesh_displacement, ChMatrix33<>(1)));

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
                    MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, sys, joint_displacement);

                rigidBodyShoe->AddVisualShape(shoe_mesh, ChFrame<>(-mesh_displacement, ChMatrix33<>(1)));
                rigidBodyShoe->AddVisualShape(shoe_coll_mesh, ChFrame<>(-mesh_displacement, ChMatrix33<>(1)));

                previous_rigidBodyShoe = rigidBodyShoe;
            }

            // close track
            ChVector<> linkpos = firstBodyShoe->Point_Body2World(joint_displacement);
            auto link_revolute_shoeshoe = chrono_types::make_shared<ChLinkLockRevolute>();
            link_revolute_shoeshoe->Initialize(firstBodyShoe, previous_rigidBodyShoe, ChCoordsys<>(linkpos, QUNIT));
            sys.AddLink(link_revolute_shoeshoe);
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
        std::shared_ptr<ChBody> template_shoe,  // collision geometry will be shared with this body
        ChVector<> position,                    // position
        ChQuaternion<> rotation,                // orientation
        ChSystemNSC& sys,                       // the physical system
        chrono::ChVector<> joint_displacement   // position of shoe-shoe constraint, relative to COG.
    ) {
        auto rigidBodyShoe = std::shared_ptr<ChBody>(template_shoe.get()->Clone());
        rigidBodyShoe->SetPos(position);
        rigidBodyShoe->SetRot(rotation);
        sys.Add(rigidBodyShoe);

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
            auto link_revolute_shoeshoe = chrono_types::make_shared<ChLinkLockRevolute>();
            link_revolute_shoeshoe->Initialize(rigidBodyShoe, previous_shoe, ChCoordsys<>(linkpos, QUNIT));
            sys.AddLink(link_revolute_shoeshoe);
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
    MyEventReceiver(ChVisualSystemIrrlicht* vsys, MySimpleTank* atank) {
        // store pointer application
        vis = vsys;
        // store pointer to other stuff
        mtank = atank;

        // ..add a GUI slider to control throttle right via mouse
        scrollbar_throttleR = vis->GetGUIEnvironment()->addScrollBar(true, rect<s32>(510, 20, 650, 35), 0, 101);
        scrollbar_throttleR->setMax(100);
        scrollbar_throttleR->setPos(50);
        text_throttleR =
            vis->GetGUIEnvironment()->addStaticText(L"Right throttle ", rect<s32>(650, 20, 750, 35), false);

        // ..add a GUI slider to control gas throttle left via mouse
        scrollbar_throttleL = vis->GetGUIEnvironment()->addScrollBar(true, rect<s32>(510, 45, 650, 60), 0, 102);
        scrollbar_throttleL->setMax(100);
        scrollbar_throttleL->setPos(50);
        text_throttleL = vis->GetGUIEnvironment()->addStaticText(L"Left throttle", rect<s32>(650, 45, 750, 60), false);
    }

    bool OnEvent(const SEvent& event) {
        // check if user moved the sliders with mouse..
        if (event.EventType == EET_GUI_EVENT) {
            s32 id = event.GUIEvent.Caller->getID();

            switch (event.GUIEvent.EventType) {
                case EGET_SCROLL_BAR_CHANGED:
                    if (id == 101) {  // id of 'throttleR' slider..
                        s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        double newthrottle = ((double)(pos)-50) / 50.0;
                        this->mtank->throttleR = newthrottle;
                        auto mfun = std::static_pointer_cast<ChFunction_Const>(mtank->link_motorRB->GetSpeedFunction());
                        mfun->Set_yconst(newthrottle * 6);
                        return true;
                    }
                    if (id == 102) {  // id of 'throttleL' slider..
                        s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        double newthrottle = ((double)(pos)-50) / 50.0;
                        this->mtank->throttleL = newthrottle;
                        auto mfun = std::static_pointer_cast<ChFunction_Const>(mtank->link_motorLB->GetSpeedFunction());
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
    ChVisualSystemIrrlicht* vis;
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
    ChSystemNSC sys;

    // 2- Create the rigid bodies of the simpified tank suspension mechanical system
    //   maybe setting position/mass/inertias of
    //   their center of mass (COG) etc.

    // ..the world
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ground_mat->SetFriction(1.0);

    auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(60, 2, 60, 1000, true, true, ground_mat);
    my_ground->SetPos(ChVector<>(0, -1, 0));
    my_ground->SetBodyFixed(true);
    my_ground->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
    sys.AddBody(my_ground);

    // ..some obstacles on the ground:
    auto obst_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    for (int i = 0; i < 50; i++) {
        auto my_obstacle = chrono_types::make_shared<ChBodyEasyBox>(
            0.6 * (1 - 0.4 * ChRandom()), 0.08, 0.3 * (1 - 0.4 * ChRandom()), 1000, true, true, obst_mat);
        my_obstacle->SetMass(3);
        my_obstacle->SetPos(ChVector<>(-6 + 6 * ChRandom(), 2 + 1 * ChRandom(), 6 * ChRandom()));
        my_obstacle->SetRot(Q_from_AngAxis(ChRandom() * CH_C_PI, VECT_Y));
        sys.AddBody(my_obstacle);
    }

    // ..the tank (this class - see above - is a 'set' of bodies and links, automatically added at creation)
    MySimpleTank* mytank = new MySimpleTank(sys);

    // Create the Irrlicht visualization sys
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    sys.SetVisualSystem(vis);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Modeling a simplified trackjed vehicle");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0, 0, -6), ChVector<>(-2, 2, 0));
    vis->AddTypicalLights();

    // Create some graphical-user-interface (GUI) items to show on the screen.
    // This requires an event receiver object.
    MyEventReceiver receiver(vis.get(), mytank);
    // note how to add the custom event receiver to the default interface:
    vis->AddUserEventReceiver(&receiver);

    // Solver settings
    sys.SetSolverType(ChSolver::Type::PSOR);
    sys.SetSolverMaxIterations(100);  // the higher, the easier to keep the constraints satisfied.

    // Simulation loop
    double timestep = 0.03;
    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        vis->BeginScene();
        vis->DrawAll();
        tools::drawGrid(vis.get(), 2, 2, 30, 30, ChCoordsys<>(ChVector<>(0, 0.01, 0), Q_from_AngX(CH_C_PI_2)),
                        ChColor(0.3f, 0.3f, 0.3f), true);

        vis->EndScene();
        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);
    }

    if (mytank)
        delete mytank;

    return 0;
}
