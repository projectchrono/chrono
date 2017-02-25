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
// Authors: Radu Serban
// =============================================================================
//
// Demo code about
//     - modeling a complex mechanism:  a forklift
//     - loading .obj 3D meshes for 3d viewing
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkLinActuator.h"
#include "chrono/assets/ChTexture.h"
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

// First of all, define a class for the 'forklift' (that is, a set of
// bodies and links which are grouped within this class; so it is
// easier to manage data structures in this example).

class MySimpleForklift {
  public:
    // THE DATA

    double throttle;  // actual value 0...1 of gas throttle.
    double steer;     // actual value of steering
    double lift;      // actual value of fork lifting

    // The parts making the forklift, as 3d Irrlicht scene nodes, each containing
    // the ChBody object
    // .. truss:
    std::shared_ptr<ChBody> truss;
    // .. right front wheel:
    std::shared_ptr<ChBody> wheelRF;
    std::shared_ptr<ChLinkLockRevolute> link_revoluteRF;
    // .. left front wheel:
    std::shared_ptr<ChBody> wheelLF;
    std::shared_ptr<ChLinkLockRevolute> link_revoluteLF;
    // .. back wheel:
    std::shared_ptr<ChBody> spindleB;
    std::shared_ptr<ChBody> wheelB;
    std::shared_ptr<ChLinkEngine> link_steer_engineB;
    std::shared_ptr<ChLinkEngine> link_engineB;
    // ..the vertical arm
    std::shared_ptr<ChBody> arm;
    std::shared_ptr<ChLinkEngine> link_engineArm;
    // ..the fork
    std::shared_ptr<ChBody> fork;
    std::shared_ptr<ChLinkLinActuator> link_actuatorFork;
    std::shared_ptr<ChLinkLockPrismatic> link_prismaticFork;

    video::ITexture* forkliftTiremap;

    // THE FUNCTIONS

    // Build and initialize the forklift, creating all bodies corresponding to
    // the various parts and adding them to the physical system - also creating
    // and adding constraints to the system.
    MySimpleForklift(ChIrrAppInterface* app, ChVector<> offset = ChVector<>(0, 0, 0)) {
        throttle = 0;  // initially, gas throttle is 0.
        steer = 0;
        lift = 0;

        ChVector<> COG_truss(0, 0.4, 0.5);
        ChVector<> COG_wheelRF(-0.566, 0.282, 1.608);
        ChVector<> COG_wheelLF(0.566, 0.282, 1.608);
        ChVector<> COG_arm(0, 1.300, 1.855);
        ChVector<> COG_fork(0, 0.362, 2.100);
        ChVector<> COG_wheelB(0, 0.282, 0.003);
        ChVector<> POS_pivotarm(0, 0.150, 1.855);
        ChVector<> POS_prismatic(0, 0.150, 1.855);
        double RAD_back_wheel = 0.28;
        double RAD_front_wheel = 0.28;

        forkliftTiremap = app->GetVideoDriver()->getTexture(GetChronoDataFile("tire_truck.png").c_str());

        // --- The car body ---

        truss = std::make_shared<ChBody>();
        app->GetSystem()->Add(truss);
        truss->SetPos(COG_truss);
        truss->SetMass(200);
        truss->SetInertiaXX(ChVector<>(100, 100, 100));
         // collision properties:
        truss->GetCollisionModel()->ClearModel();
        truss->GetCollisionModel()->AddBox(1.227 / 2., 1.621 / 2., 1.864 / 2., ChVector<>(-0.003, 1.019, 0.192));
        truss->GetCollisionModel()->AddBox(0.187 / 2., 0.773 / 2., 1.201 / 2., ChVector<>(0.486, 0.153, -0.047));
        truss->GetCollisionModel()->AddBox(0.187 / 2., 0.773 / 2., 1.201 / 2., ChVector<>(-0.486, 0.153, -0.047));
        truss->GetCollisionModel()->BuildModel();
        truss->SetCollide(true);
         // visualization properties:
        auto truss_asset_assembly = std::make_shared<ChAssetLevel>();
        truss_asset_assembly->GetFrame().SetPos(-COG_truss);
        truss->AddAsset(truss_asset_assembly);
        auto truss_mesh = std::make_shared<ChObjShapeFile>();
        truss_mesh->SetFilename(GetChronoDataFile("forklift_body.obj"));
        truss_asset_assembly->AddAsset(truss_mesh);
        auto truss_texture = std::make_shared<ChTexture>(GetChronoDataFile("tire_truck.png"));
        truss_asset_assembly->AddAsset(truss_texture);



        // ..the right-front wheel

        wheelRF = std::make_shared<ChBody>();
        app->GetSystem()->Add(wheelRF);
        wheelRF->SetPos(COG_wheelRF);
        wheelRF->SetMass(20);
        wheelRF->SetInertiaXX(ChVector<>(2, 2, 2));
         // collision properties:
        ChMatrix33<> Arot(chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
        wheelRF->GetCollisionModel()->ClearModel();
        wheelRF->GetCollisionModel()->AddCylinder(RAD_front_wheel, RAD_front_wheel, 0.1, ChVector<>(0, 0, 0), Arot);
        wheelRF->GetCollisionModel()->BuildModel();
        wheelRF->SetCollide(true);
         // visualization properties:
        auto wheelRF_asset_assembly = std::make_shared<ChAssetLevel>();
        wheelRF_asset_assembly->GetFrame().SetPos(-COG_wheelRF);
        wheelRF->AddAsset(wheelRF_asset_assembly);
        auto wheelRF_mesh = std::make_shared<ChObjShapeFile>();
        wheelRF_mesh->SetFilename(GetChronoDataFile("wheel.obj"));
        wheelRF_asset_assembly->AddAsset(wheelRF_mesh);
        auto wheelRF_texture = std::make_shared<ChTexture>(GetChronoDataFile("tire_truck.png"));
        wheelRF_asset_assembly->AddAsset(wheelRF_texture);
      

        // .. create the revolute joint between the wheel and the truss
        
        link_revoluteRF = std::make_shared<ChLinkLockRevolute>();  // right, front, upper, 1
        link_revoluteRF->Initialize(wheelRF, truss,
                                    ChCoordsys<>(COG_wheelRF, chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
        app->GetSystem()->AddLink(link_revoluteRF);

        // ..the left-front wheel

        wheelLF = std::make_shared<ChBody>();
        app->GetSystem()->Add(wheelLF);
        wheelLF->SetPos(COG_wheelLF);
        wheelLF->SetRot(chrono::Q_from_AngAxis(CH_C_PI, VECT_Y));  // reuse RF wheel shape, flipped
        wheelLF->SetMass(20);
        wheelLF->SetInertiaXX(ChVector<>(2, 2, 2));
         // collision properties:
        wheelLF->GetCollisionModel()->ClearModel();
        wheelLF->GetCollisionModel()->AddCylinder(RAD_front_wheel, RAD_front_wheel, 0.1, ChVector<>(0, 0, 0), Arot);
        wheelLF->GetCollisionModel()->BuildModel();
        wheelLF->SetCollide(true);
         // visualization properties:
        auto wheelLF_asset_assembly = std::make_shared<ChAssetLevel>();
        wheelLF_asset_assembly->GetFrame().SetPos(-COG_wheelRF);
        wheelLF->AddAsset(wheelLF_asset_assembly);
        auto wheelLF_mesh = std::make_shared<ChObjShapeFile>();
        wheelLF_mesh->SetFilename(GetChronoDataFile("wheel.obj"));
        wheelLF_asset_assembly->AddAsset(wheelLF_mesh);
        auto wheelLF_texture = std::make_shared<ChTexture>(GetChronoDataFile("tire_truck.png"));
        wheelLF_asset_assembly->AddAsset(wheelLF_texture);


        // .. create the revolute joint between the wheel and the truss
        link_revoluteLF = std::make_shared<ChLinkLockRevolute>();  // right, front, upper, 1
        link_revoluteLF->Initialize(wheelLF, truss,
                                    ChCoordsys<>(COG_wheelLF, chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
        app->GetSystem()->AddLink(link_revoluteLF);

        // ..the back steering spindle (invisible, no mesh)
        spindleB = std::make_shared<ChBody>();
        app->GetSystem()->Add(spindleB);
        spindleB->SetPos(COG_wheelB);
        spindleB->SetMass(10);
        spindleB->SetInertiaXX(ChVector<>(1, 1, 1));


        // .. create the vertical steering link between the spindle structure and the truss
        link_steer_engineB = std::make_shared<ChLinkEngine>();
        link_steer_engineB->Initialize(
            spindleB, truss,
            ChCoordsys<>(COG_wheelB, chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_X)));  // vertical axis
        link_steer_engineB->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK);
        link_steer_engineB->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION);
        app->GetSystem()->AddLink(link_steer_engineB);

        // ..the back wheel

        wheelB = std::make_shared<ChBody>();
        app->GetSystem()->Add(wheelB);
        wheelB->SetPos(COG_wheelB);
        wheelB->SetRot(chrono::Q_from_AngAxis(CH_C_PI, VECT_Y));  // reuse RF wheel shape, flipped
        wheelB->SetMass(20);
        wheelB->SetInertiaXX(ChVector<>(2, 2, 2));
         // collision properties:
        wheelB->GetCollisionModel()->ClearModel();
        wheelB->GetCollisionModel()->AddCylinder(RAD_back_wheel, RAD_back_wheel, 0.1, ChVector<>(0, 0, 0), Arot);
        wheelB->GetCollisionModel()->BuildModel();
        wheelB->SetCollide(true);

         // visualization properties:
        auto wheelB_asset_assembly = std::make_shared<ChAssetLevel>();
        wheelB_asset_assembly->GetFrame().SetPos(-COG_wheelRF);
        wheelB->AddAsset(wheelB_asset_assembly);
        auto wheelB_mesh = std::make_shared<ChObjShapeFile>();
        wheelB_mesh->SetFilename(GetChronoDataFile("wheel.obj"));
        wheelB_asset_assembly->AddAsset(wheelB_mesh);
        auto wheelB_texture = std::make_shared<ChTexture>(GetChronoDataFile("tire_truck.png"));
        wheelB_asset_assembly->AddAsset(wheelB_texture);


        // .. create the motor between the back wheel and the steering spindle structure
        link_engineB = std::make_shared<ChLinkEngine>();
        link_engineB->Initialize(wheelB, spindleB,
                                 ChCoordsys<>(COG_wheelB, chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
        link_engineB->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK);
        link_engineB->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
        app->GetSystem()->AddLink(link_engineB);

        // ..the arm
        arm = std::make_shared<ChBody>();
        app->GetSystem()->Add(arm);
        arm->SetPos(COG_arm);
        arm->SetMass(100);
        arm->SetInertiaXX(ChVector<>(30, 30, 30));
         // visualization properties:
        auto arm_asset_assembly = std::make_shared<ChAssetLevel>();
        arm_asset_assembly->GetFrame().SetPos(-COG_arm);
        arm->AddAsset(arm_asset_assembly);
        auto arm_mesh = std::make_shared<ChObjShapeFile>();
        arm_mesh->SetFilename(GetChronoDataFile("forklift_arm.obj"));
        arm_asset_assembly->AddAsset(arm_mesh);


        // .. create the revolute joint between the arm and the truss
        link_engineArm = std::make_shared<ChLinkEngine>();  // right, front, upper, 1
        link_engineArm->Initialize(arm, truss,
                                   ChCoordsys<>(POS_pivotarm, chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
        link_engineArm->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK);
        link_engineArm->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION);
        app->GetSystem()->AddLink(link_engineArm);

        // ..the fork

        fork = std::make_shared<ChBody>();
        app->GetSystem()->Add(fork);
        fork->SetPos(COG_fork);
        fork->SetMass(60);
        fork->SetInertiaXX(ChVector<>(15, 15, 15));
         // collision properties:
        fork->GetCollisionModel()->ClearModel();
        fork->GetCollisionModel()->AddBox(0.1 / 2., 0.032 / 2., 1.033 / 2.,ChVector<>(-0.352, -0.312, 0.613));
        fork->GetCollisionModel()->AddBox(0.1 / 2., 0.032 / 2., 1.033 / 2., ChVector<>(0.352, -0.312, 0.613));
        fork->GetCollisionModel()->AddBox(0.344 / 2., 1.134 / 2., 0.101 / 2., ChVector<>(-0.000, 0.321, -0.009));
        fork->GetCollisionModel()->BuildModel();
        fork->SetCollide(true);
         // visualization properties:
        auto fork_asset_assembly = std::make_shared<ChAssetLevel>();
        fork_asset_assembly->GetFrame().SetPos(-COG_fork);
        fork->AddAsset(fork_asset_assembly);
        auto fork_mesh = std::make_shared<ChObjShapeFile>();
        fork_mesh->SetFilename(GetChronoDataFile("forklift_forks.obj"));
        fork_asset_assembly->AddAsset(fork_mesh);


        // .. create the prismatic joint between the fork and arm
        link_prismaticFork = std::make_shared<ChLinkLockPrismatic>();
        link_prismaticFork->Initialize(
            fork, arm,
            ChCoordsys<>(
                POS_prismatic,
                chrono::Q_from_AngAxis(CH_C_PI / 2,
                                       VECT_X)));  // set prism as vertical (default would be aligned to z, horizontal
        app->GetSystem()->AddLink(link_prismaticFork);

        // .. create the linear actuator that pushes upward the fork
        link_actuatorFork = std::make_shared<ChLinkLinActuator>();
        link_actuatorFork->Initialize(fork, arm, false,
                                      ChCoordsys<>(POS_prismatic + ChVector<>(0, 0.01, 0), QUNIT),
                                      ChCoordsys<>(POS_prismatic, QUNIT));
        app->GetSystem()->AddLink(link_actuatorFork);


        // ..a pallet

        auto pallet = std::make_shared<ChBodyEasyMesh>(
            GetChronoDataFile("pallet.obj"),  // mesh .OBJ file
            300,                              // density
            true,   // compute mass, inertia & COG from the mesh (must be a closed watertight mesh!)
            true,   // enable collision with mesh
            0.001,  // sphere swept inflate of mesh - improves robustness of collision detection
            true);  // enable visualization of mesh
        app->GetSystem()->Add(pallet);
        pallet->SetPos(ChVector<>(0, 0.4, 3));
        
        // apply also a texture to the pallet:
        auto pallet_texture = std::make_shared<ChTexture>();
        pallet_texture->SetTextureFilename(GetChronoDataFile("cubetexture.png"));
        pallet->AddAsset(pallet_texture);


        //
        // Move the forklift to initial offset position
        //

        //				pallet->GetBody()->Move(offset);
        truss->Move(offset);
        wheelRF->Move(offset);
        wheelLF->Move(offset);
        wheelB->Move(offset);
        spindleB->Move(offset);
        arm->Move(offset);
        fork->Move(offset);
    }

    // Delete the car object, deleting also all bodies corresponding to
    // the various parts and removing them from the physical system.  Also
    // removes constraints from the system.
    ~MySimpleForklift() {
        ChSystem* mysystem = truss->GetSystem();  // trick to get the system here

        mysystem->Remove(link_revoluteRF);
        mysystem->Remove(link_revoluteLF);
        mysystem->Remove(link_steer_engineB);
        mysystem->Remove(link_engineB);
        mysystem->Remove(link_engineArm);
        mysystem->Remove(link_prismaticFork);
        mysystem->Remove(link_actuatorFork);
        mysystem->Remove(truss);
        mysystem->Remove(wheelRF);
        mysystem->Remove(wheelLF);
        mysystem->Remove(wheelB);
        mysystem->Remove(spindleB);
        mysystem->Remove(arm);
        mysystem->Remove(fork);
    }
};

// Define a MyEventReceiver class which will be used to manage input
// from the GUI graphical user interface (the interface will
// be created with the basic -yet flexible- platform
// independent toolset of Irrlicht).

class MyEventReceiver : public IEventReceiver {
  public:
    MyEventReceiver(ChIrrAppInterface* myapp, MySimpleForklift* mlift) {
        // store pointer to physical system & other stuff so we can tweak them by user keyboard
        app = myapp;
        forklift = mlift;
    }

    bool OnEvent(const SEvent& event) {
        // check if user presses keys
        if (event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) {
            switch (event.KeyInput.Key) {
                case irr::KEY_KEY_Q:
                    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_steer_engineB->Get_rot_funct()))
                        mfun->Set_yconst(-0.6 + mfun->Get_yconst());
                    return true;
                case irr::KEY_KEY_W:
                    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_steer_engineB->Get_rot_funct()))
                        mfun->Set_yconst(+0.3 + mfun->Get_yconst());
                    return true;
                case irr::KEY_KEY_A:
                    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_engineB->Get_spe_funct()))
                        mfun->Set_yconst(0.5 + mfun->Get_yconst());
                    return true;
                case irr::KEY_KEY_Z:
                    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_engineB->Get_spe_funct()))
                        mfun->Set_yconst(-0.5 + mfun->Get_yconst());
                    return true;
                case irr::KEY_KEY_S:
                    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_actuatorFork->Get_dist_funct()))
                        mfun->Set_yconst(0.05 + mfun->Get_yconst());
                    return true;
                case irr::KEY_KEY_X:
                    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_actuatorFork->Get_dist_funct()))
                        mfun->Set_yconst(-0.05 + mfun->Get_yconst());
                    return true;
                case irr::KEY_KEY_D:
                    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_engineArm->Get_rot_funct()))
                        mfun->Set_yconst(0.005 + mfun->Get_yconst());
                    return true;
                case irr::KEY_KEY_C:
                    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_engineArm->Get_rot_funct()))
                        mfun->Set_yconst(-0.005 + mfun->Get_yconst());
                    return true;
                default:
                    break;
            }
        }

        return false;
    }

  private:
    ChIrrAppInterface* app;
    MySimpleForklift* forklift;
};

//
// This is the program which is executed
//

int main(int argc, char* argv[]) {
    // Create a ChronoENGINE physical system
    ChSystem my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Drive a forklift", core::dimension2d<u32>(800, 600), false);

    // add text with info
    IGUIStaticText* textFPS = application.GetIGUIEnvironment()->addStaticText(
        L"Keys: steer=Q,W; throttle=A,Z; lift=S,X; bank=D,C", rect<s32>(150, 10, 430, 40), true);

    // Easy shortcuts to add logo, camera, lights and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(-6, 3, -6));

    // ..the world
    auto my_ground = std::make_shared<ChBodyEasyBox>(40, 2, 40, 1000, true, true);
    my_system.Add(my_ground);
    my_ground->SetBodyFixed(true);
    my_ground->SetPos(ChVector<>(0, -1, 0));
    my_ground->GetMaterialSurface()->SetSfriction(1.0);
    my_ground->GetMaterialSurface()->SetKfriction(1.0);
    auto mtexture = std::make_shared<ChTexture>(GetChronoDataFile("concrete.jpg"));
    my_ground->AddAsset(mtexture);


    // ..some obstacles on the ground:
    for (int i = 0; i < 6; i++) {
        auto my_obstacle = std::make_shared<ChBodyEasyBox>(1, 0.5, 1, 200, true, true);
        my_system.Add(my_obstacle);
        my_obstacle->SetPos(ChVector<>(20 * ChRandom(), 2, 20 * ChRandom()));
        my_obstacle->GetMaterialSurface()->SetFriction(1.0);
        auto mtexture = std::make_shared<ChTexture>(GetChronoDataFile("cubetexture_wood.png"));
        my_obstacle->AddAsset(mtexture);
    }

    // ..the forklift (this class - see above - is a 'set' of bodies and links, automatically added at creation)
    MySimpleForklift* myforklift = new MySimpleForklift(&application);


    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();


    //
    // USER INTERFACE
    //

    // Create some graphical-user-interface (GUI) items to show on the screen.
    // This requires an event receiver object -see above.
    // This is for GUI tweaking of system parameters..
    MyEventReceiver receiver(&application, myforklift);
    // note how to add a custom event receiver to the default interface:
    application.SetUserEventReceiver(&receiver);

    //
    // SETTINGS
    //

    my_system.SetMaxItersSolverSpeed(20);  // the higher, the easier to keep the constraints 'mounted'.

    my_system.SetSolverType(ChSolver::Type::SOR);

    //
    // THE SOFT-REAL-TIME CYCLE, SHOWING THE SIMULATION
    //

    // This will help choosing an integration step which matches the
    // real-time step of the simulation..
    application.SetStepManage(true);
    application.SetTimestep(0.005);

    while (application.GetDevice()->run()) {
        // Irrlicht must prepare frame to draw
        application.GetVideoDriver()->beginScene(true, true, SColor(255, 140, 161, 192));

        // Irrlicht application draws all 3D objects and all GUI items
        application.DrawAll();

        // Advance the simulation time step
        application.DoStep();

        // Irrlicht must finish drawing the frame
        application.GetVideoDriver()->endScene();
    }

    if (myforklift)
        delete myforklift;

    return 0;
}
