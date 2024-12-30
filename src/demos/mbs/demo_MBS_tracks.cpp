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
// - using DisallowCollisionsWith, SetFamily etc. to avoid
//   collisions between different families of bodies.
//
// =============================================================================

#include <cmath>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/geometry/ChTriangleMeshSoup.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChRandom.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_irrlicht/ChIrrMeshTools.h"

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

        ChVector3d cyl_displA(0, 0.075 + 0.02, 0);
        ChVector3d cyl_displB(0, -0.075 - 0.02, 0);
        double cyl_thickness = 0.09;

        // --- The tank body ---

        truss = chrono_types::make_shared<ChBodyEasyMesh>(           //
            GetChronoDataFile("models/bulldozer/bulldozerB10.obj"),  // file name
            1000,                                                    // density
            false,                                                   // do not evaluate mass automatically
            true,                                                    // create visualization asset
            false,                                                   // do not collide
            nullptr,                                                 // no need for contact material
            0                                                        // swept sphere radius
        );
        sys.Add(truss);
        truss->SetPos(ChVector3d(mx + passo / 2, my + radiustrack, rlwidth / 2));
        truss->SetMass(350);
        truss->SetInertiaXX(ChVector3d(13.8, 13.5, 10));

        // --- Contact and visualization materials for wheels ---

        auto wheel_mat = chrono_types::make_shared<ChContactMaterialNSC>();
        wheel_mat->SetFriction(1.0);

        auto wheel_mat_vis = chrono_types::make_shared<ChVisualMaterial>();
        wheel_mat_vis->SetDiffuseColor(ChColor(0.2f, 0.2f, 0.2f));

        // --- Wheel collision shape

        auto wheel_shape =
            chrono_types::make_shared<ChCollisionShapeCylinder>(wheel_mat, wheeldiameter / 2, cyl_thickness);

        // --- Right Front suspension ---

        // ..the tank right-front wheel
        wheelRF = chrono_types::make_shared<ChBodyEasyMesh>(       //
            GetChronoDataFile("models/bulldozer/wheel_view.obj"),  // data file
            1000,                                                  // density
            false,                                                 // do not compute mass and inertia
            true,                                                  // visualization?
            false,                                                 // collision?
            nullptr,                                               // no need for contact material
            0);                                                    // mesh sweep sphere radius

        sys.Add(wheelRF);
        wheelRF->SetPos(ChVector3d(mx + passo, my + radiustrack, 0));
        wheelRF->SetRot(QuatFromAngleX(CH_PI / 2));
        wheelRF->SetMass(9.0);
        wheelRF->SetInertiaXX(ChVector3d(1.2, 1.2, 1.2));

        wheelRF->AddCollisionShape(wheel_shape, ChFrame<>(cyl_displA, QuatFromAngleX(CH_PI_2)));
        wheelRF->AddCollisionShape(wheel_shape, ChFrame<>(cyl_displB, QuatFromAngleX(CH_PI_2)));
        wheelRF->EnableCollision(true);

        wheelRF->GetVisualShape(0)->SetMaterial(0, wheel_mat_vis);

        // .. create the revolute joint between the wheel and the truss
        link_revoluteRF = chrono_types::make_shared<ChLinkLockRevolute>();  // right, front, upper, 1
        link_revoluteRF->Initialize(wheelRF, truss, ChFrame<>(ChVector3d(mx + passo, my + radiustrack, 0), QUNIT));
        sys.AddLink(link_revoluteRF);

        // --- Left Front suspension ---

        // ..the tank left-front wheel

        wheelLF = chrono_types::make_shared<ChBodyEasyMesh>(       //
            GetChronoDataFile("models/bulldozer/wheel_view.obj"),  // data file
            1000,                                                  // density
            false,                                                 // do not compute mass and inertia
            true,                                                  // visualization?
            false,                                                 // collision?
            nullptr,                                               // no need for contact material
            0);                                                    // mesh sweep sphere radius

        sys.Add(wheelLF);
        wheelLF->SetPos(ChVector3d(mx + passo, my + radiustrack, rlwidth));
        wheelLF->SetRot(QuatFromAngleX(CH_PI / 2));
        wheelLF->SetMass(9.0);
        wheelLF->SetInertiaXX(ChVector3d(1.2, 1.2, 1.2));

        wheelLF->AddCollisionShape(wheel_shape, ChFrame<>(cyl_displA, QuatFromAngleX(CH_PI_2)));
        wheelLF->AddCollisionShape(wheel_shape, ChFrame<>(cyl_displB, QuatFromAngleX(CH_PI_2)));
        wheelLF->EnableCollision(true);

        wheelLF->GetVisualShape(0)->SetMaterial(0, wheel_mat_vis);

        // .. create the revolute joint between the wheel and the truss
        link_revoluteLF = chrono_types::make_shared<ChLinkLockRevolute>();  // left, front, upper, 1
        link_revoluteLF->Initialize(wheelLF, truss,
                                    ChFrame<>(ChVector3d(mx + passo, my + radiustrack, rlwidth), QUNIT));
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
        wheelRB->SetPos(ChVector3d(mx, my + radiustrack, 0));
        wheelRB->SetRot(QuatFromAngleX(CH_PI / 2));
        wheelRB->SetMass(9.0);
        wheelRB->SetInertiaXX(ChVector3d(1.2, 1.2, 1.2));

        wheelRB->AddCollisionShape(wheel_shape, ChFrame<>(cyl_displA, QuatFromAngleX(CH_PI_2)));
        wheelRB->AddCollisionShape(wheel_shape, ChFrame<>(cyl_displB, QuatFromAngleX(CH_PI_2)));
        wheelRB->EnableCollision(true);

        wheelRB->GetVisualShape(0)->SetMaterial(0, wheel_mat_vis);

        // .. create the motor joint between the wheel and the truss (simplified motor model: just impose speed..)
        link_motorRB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        link_motorRB->SetSpeedFunction(
            chrono_types::make_shared<ChFunctionConst>());  // actually, default function type
        link_motorRB->Initialize(wheelRB, truss, ChFrame<>(ChVector3d(mx, my + radiustrack, 0), QUNIT));
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
        wheelLB->SetPos(ChVector3d(mx, my + radiustrack, rlwidth));
        wheelLB->SetRot(QuatFromAngleX(CH_PI / 2));
        wheelLB->SetMass(9.0);
        wheelLB->SetInertiaXX(ChVector3d(1.2, 1.2, 1.2));

        wheelLB->AddCollisionShape(wheel_shape, ChFrame<>(cyl_displA, QuatFromAngleX(CH_PI_2)));
        wheelLB->AddCollisionShape(wheel_shape, ChFrame<>(cyl_displB, QuatFromAngleX(CH_PI_2)));
        wheelLB->EnableCollision(true);

        wheelLB->GetVisualShape(0)->SetMaterial(0, wheel_mat_vis);

        // .. create the motor joint between the wheel and the truss (simplified motor model: just impose speed..)
        link_motorLB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        link_motorLB->SetSpeedFunction(
            chrono_types::make_shared<ChFunctionConst>());  // actually, default function type
        link_motorLB->Initialize(wheelLB, truss, ChFrame<>(ChVector3d(mx, my + radiustrack, rlwidth), QUNIT));
        sys.AddLink(link_motorLB);

        //--- TRACKS ---

        // Shared visualization models
        auto shoe_trimesh =
            ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("models/bulldozer/shoe_view.obj"));
        ////shoe_trimesh->Transform(-mesh_displacement, ChMatrix33<>(1));
        auto shoe_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        shoe_mesh->SetMesh(shoe_trimesh);
        shoe_mesh->SetVisible(true);

        auto shoe_coll_trimesh =
            ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("models/bulldozer/shoe_collision.obj"));
        ////shoe_coll_trimesh->Transform(-mesh_displacement, ChMatrix33<>(1));
        auto shoe_coll_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        shoe_coll_mesh->SetMesh(shoe_coll_trimesh);
        shoe_coll_mesh->SetVisible(false);

        // Load a triangle mesh for collision
        auto trimesh =
            ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("models/bulldozer/shoe_collision.obj"));

        ChVector3d mesh_displacement(shoelength * 0.5, 0, 0);    // as mesh origin is not in body center of mass
        ChVector3d joint_displacement(-shoelength * 0.5, 0, 0);  // pos. of shoe-shoe constraint, relative to COG.

        chrono::ChVector3d position;
        chrono::ChQuaternion<> rotation;

        for (int side = 0; side < 2; side++) {
            mx = 0;
            mx += shoelength;

            double mz = (side == 0) ? 0 : rlwidth;
            std::string prefix = (side == 0) ? "L_" : "R_";

            position.Set(mx, my, mz);
            rotation = QUNIT;

            // Create sample body (with empty collision shape; later create the collision model by adding the
            // coll.shapes)
            auto firstBodyShoe = chrono_types::make_shared<ChBody>();
            sys.Add(firstBodyShoe);
            firstBodyShoe->SetName(prefix + "shoe_0");
            firstBodyShoe->SetMass(shoemass);
            firstBodyShoe->SetPos(position);
            firstBodyShoe->SetRot(rotation);
            firstBodyShoe->SetInertiaXX(ChVector3d(0.1, 0.1, 0.1));

            // Visualization:
            firstBodyShoe->AddVisualShape(shoe_mesh, ChFrame<>(-mesh_displacement, ChMatrix33<>(1)));
            firstBodyShoe->AddVisualShape(shoe_coll_mesh, ChFrame<>(-mesh_displacement, ChMatrix33<>(1)));

            // Collision:
            auto coll_model = chrono_types::make_shared<ChCollisionModel>();
            coll_model->SetSafeMargin(0.004f);  // inward safe margin
            coll_model->SetEnvelope(0.010f);    // distance of the outward "collision envelope"
            auto coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(
                chrono_types::make_shared<ChContactMaterialNSC>(), trimesh, false, false, 0.005);
            coll_model->AddShape(coll_shape, ChFrame<>(mesh_displacement, QUNIT));
            firstBodyShoe->AddCollisionModel(coll_model);
            firstBodyShoe->EnableCollision(true);

            // Avoid creation of contacts with neighbouring shoes, using
            // a collision family (=3) that does not collide with itself
            firstBodyShoe->GetCollisionModel()->SetFamily(3);
            firstBodyShoe->GetCollisionModel()->DisallowCollisionsWith(3);

            std::shared_ptr<ChBody> previous_rigidBodyShoe;
            previous_rigidBodyShoe = firstBodyShoe;

            int shoe_index = 1;

            for (int nshoe = 1; nshoe < ntiles; nshoe++) {
                mx += shoelength;
                position.Set(mx, my, mz);

                auto rigidBodyShoe =
                    MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, sys, joint_displacement);

                rigidBodyShoe->SetName(prefix + "shoe_" + std::to_string(shoe_index++));

                rigidBodyShoe->AddVisualShape(shoe_mesh, ChFrame<>(-mesh_displacement, ChMatrix33<>(1)));
                rigidBodyShoe->AddVisualShape(shoe_coll_mesh, ChFrame<>(-mesh_displacement, ChMatrix33<>(1)));

                previous_rigidBodyShoe = rigidBodyShoe;
            }
            for (int nshoe = 0; nshoe < nwrap; nshoe++) {
                double alpha = (CH_PI / ((double)(nwrap - 1.0))) * ((double)nshoe);

                double lx = mx + shoelength + radiustrack * std::sin(alpha);
                double ly = my + radiustrack - radiustrack * std::cos(alpha);
                position.Set(lx, ly, mz);
                rotation = chrono::QuatFromAngleZ(alpha);
                auto rigidBodyShoe =
                    MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, sys, joint_displacement);

                rigidBodyShoe->SetName(prefix + "shoe_" + std::to_string(shoe_index++));

                rigidBodyShoe->AddVisualShape(shoe_mesh, ChFrame<>(-mesh_displacement, ChMatrix33<>(1)));
                rigidBodyShoe->AddVisualShape(shoe_coll_mesh, ChFrame<>(-mesh_displacement, ChMatrix33<>(1)));

                previous_rigidBodyShoe = rigidBodyShoe;
            }
            for (int nshoe = (ntiles - 1); nshoe >= 0; nshoe--) {
                position.Set(mx, my + 2 * radiustrack, mz);

                auto rigidBodyShoe =
                    MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, sys, joint_displacement);

                rigidBodyShoe->SetName(prefix + "shoe_" + std::to_string(shoe_index++));

                rigidBodyShoe->AddVisualShape(shoe_mesh, ChFrame<>(-mesh_displacement, ChMatrix33<>(1)));
                rigidBodyShoe->AddVisualShape(shoe_coll_mesh, ChFrame<>(-mesh_displacement, ChMatrix33<>(1)));

                previous_rigidBodyShoe = rigidBodyShoe;

                mx -= shoelength;
            }
            for (int nshoe = 0; nshoe < nwrap; nshoe++) {
                double alpha = CH_PI + (CH_PI / ((double)(nwrap - 1.0))) * ((double)nshoe);

                double lx = mx + 0 + radiustrack * std::sin(alpha);
                double ly = my + radiustrack - radiustrack * std::cos(alpha);
                position.Set(lx, ly, mz);
                rotation = chrono::QuatFromAngleZ(alpha);
                auto rigidBodyShoe =
                    MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, sys, joint_displacement);

                rigidBodyShoe->SetName(prefix + "shoe_" + std::to_string(shoe_index++));

                rigidBodyShoe->AddVisualShape(shoe_mesh, ChFrame<>(-mesh_displacement, ChMatrix33<>(1)));
                rigidBodyShoe->AddVisualShape(shoe_coll_mesh, ChFrame<>(-mesh_displacement, ChMatrix33<>(1)));

                previous_rigidBodyShoe = rigidBodyShoe;
            }

            // close track
            ChVector3d linkpos = firstBodyShoe->TransformPointLocalToParent(joint_displacement);
            auto link_revolute_shoeshoe = chrono_types::make_shared<ChLinkLockRevolute>();
            link_revolute_shoeshoe->Initialize(firstBodyShoe, previous_rigidBodyShoe, ChFrame<>(linkpos, QUNIT));
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
        ChVector3d position,                    // position
        ChQuaternion<> rotation,                // orientation
        ChSystemNSC& sys,                       // the physical system
        chrono::ChVector3d joint_displacement   // position of shoe-shoe constraint, relative to COG.
    ) {
        auto rigidBodyShoe = std::shared_ptr<ChBody>(template_shoe.get()->Clone());
        rigidBodyShoe->SetPos(position);
        rigidBodyShoe->SetRot(rotation);
        sys.Add(rigidBodyShoe);

        auto coll_model = chrono_types::make_shared<ChCollisionModel>(*template_shoe->GetCollisionModel());
        rigidBodyShoe->AddCollisionModel(coll_model);
        rigidBodyShoe->EnableCollision(true);

        // Other settings are already copied from template_shoe, except for family and mask.
        // Avoid creation of contacts with neighbouring shoes:
        rigidBodyShoe->GetCollisionModel()->SetFamily(3);
        rigidBodyShoe->GetCollisionModel()->DisallowCollisionsWith(3);

        // Create revolute constraint
        if (previous_shoe) {
            ChVector3d linkpos = rigidBodyShoe->TransformPointLocalToParent(joint_displacement);
            auto link_revolute_shoeshoe = chrono_types::make_shared<ChLinkLockRevolute>();
            link_revolute_shoeshoe->Initialize(rigidBodyShoe, previous_shoe, ChFrame<>(linkpos, QUNIT));
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
                        auto mfun = std::static_pointer_cast<ChFunctionConst>(mtank->link_motorRB->GetSpeedFunction());
                        mfun->SetConstant(newthrottle * 6);
                        return true;
                    }
                    if (id == 102) {  // id of 'throttleL' slider..
                        s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        double newthrottle = ((double)(pos)-50) / 50.0;
                        this->mtank->throttleL = newthrottle;
                        auto mfun = std::static_pointer_cast<ChFunctionConst>(mtank->link_motorLB->GetSpeedFunction());
                        mfun->SetConstant(newthrottle * 6);
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
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // 1- Create a Chrono physical system: all bodies and constraints will be handled by this ChSystemNSC object.
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // 2- Create the rigid bodies of the simpified tracked vehicle mechanical system, setting position, mass, inertias
    // of their center of mass (COG) etc.

    // ..the world
    auto ground_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    ground_mat->SetFriction(1.0);

    auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(60, 2, 60, 1000, true, true, ground_mat);
    my_ground->SetPos(ChVector3d(0, -1, 0));
    my_ground->SetFixed(true);
    my_ground->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
    sys.AddBody(my_ground);

    // ..some obstacles on the ground:
    auto obst_mat = chrono_types::make_shared<ChContactMaterialNSC>();

    for (int i = 0; i < 50; i++) {
        auto my_obstacle = chrono_types::make_shared<ChBodyEasyBox>(
            0.6 * (1 - 0.4 * ChRandom::Get()), 0.08, 0.3 * (1 - 0.4 * ChRandom::Get()), 1000, true, true, obst_mat);
        my_obstacle->SetMass(3);
        my_obstacle->SetPos(ChVector3d(-6 + 6 * ChRandom::Get(), 2 + 1 * ChRandom::Get(), 6 * ChRandom::Get()));
        my_obstacle->SetRot(QuatFromAngleY(ChRandom::Get() * CH_PI));
        sys.AddBody(my_obstacle);
    }

    // ..the tank (this class - see above - is a 'set' of bodies and links, automatically added at creation)
    MySimpleTank* mytank = new MySimpleTank(sys);

    // Create the Irrlicht visualization sys
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Modeling a simplified trackjed vehicle");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(0, 0, -6), ChVector3d(-2, 2, 0));
    vis->AddTypicalLights();

    // Create some graphical-user-interface (GUI) items to show on the screen.
    // This requires an event receiver object.
    MyEventReceiver receiver(vis.get(), mytank);
    // note how to add the custom event receiver to the default interface:
    vis->AddUserEventReceiver(&receiver);

    // Solver settings
    sys.SetSolverType(ChSolver::Type::PSOR);
    sys.GetSolver()->AsIterative()->SetMaxIterations(100);

    // Simulation loop
    double timestep = 0.03;
    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        tools::drawGrid(vis.get(), 2, 2, 30, 30, ChCoordsys<>(ChVector3d(0, 0.01, 0), QuatFromAngleX(CH_PI_2)),
                        ChColor(0.3f, 0.3f, 0.3f), true);

        vis->EndScene();
        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);
    }

    if (mytank)
        delete mytank;

    return 0;
}
