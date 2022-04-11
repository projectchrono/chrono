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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// Demo code about
//     - modeling a complex mechanism:  a forklift
//     - loading .obj 3D meshes for 3d viewing
//
// =============================================================================

#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkLinActuator.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"

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

// Definition of the forklift MBS
class MySimpleForklift {
  public:
    // THE DATA

    double throttle;  // actual value 0...1 of gas throttle.
    double steer;     // actual value of steering
    double lift;      // actual value of fork lifting

    // The parts making the forklift, as 3d Irrlicht scene nodes, each containing
    // the ChBody object
    // .. truss:
    std::shared_ptr<ChBody> chassis;
    // .. right front wheel:
    std::shared_ptr<ChBody> wheelRF;
    std::shared_ptr<ChLinkLockRevolute> link_revoluteRF;
    // .. left front wheel:
    std::shared_ptr<ChBody> wheelLF;
    std::shared_ptr<ChLinkLockRevolute> link_revoluteLF;
    // .. back wheel:
    std::shared_ptr<ChBody> spindleB;
    std::shared_ptr<ChBody> wheelB;
    std::shared_ptr<ChLinkMotorRotationAngle> link_steer_engineB;
    std::shared_ptr<ChLinkMotorRotationSpeed> link_engineB;
    // ..the vertical arm
    std::shared_ptr<ChBody> arm;
    std::shared_ptr<ChLinkMotorRotationAngle> link_engineArm;
    // ..the fork
    std::shared_ptr<ChBody> fork;
    std::shared_ptr<ChLinkLinActuator> link_actuatorFork;
    std::shared_ptr<ChLinkLockPrismatic> link_prismaticFork;


    // Build and initialize the forklift, creating all bodies corresponding to
    // the various parts and adding them to the physical system - also creating
    // and adding constraints to the system.
    MySimpleForklift(ChSystem* sys, ChVector<> offset = ChVector<>(0, 0, 0)) {
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

        // --- The car body ---

        chassis = chrono_types::make_shared<ChBody>();
        sys->Add(chassis);
        chassis->SetPos(COG_truss);
        chassis->SetMass(200);
        chassis->SetInertiaXX(ChVector<>(100, 100, 100));

        // collision properties:
        auto chassis_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

        chassis->GetCollisionModel()->ClearModel();
        chassis->GetCollisionModel()->AddBox(chassis_mat, 1.227 / 2., 1.621 / 2., 1.864 / 2., ChVector<>(-0.003, 1.019, 0.192));
        chassis->GetCollisionModel()->AddBox(chassis_mat, 0.187 / 2., 0.773 / 2., 1.201 / 2., ChVector<>(0.486, 0.153, -0.047));
        chassis->GetCollisionModel()->AddBox(chassis_mat, 0.187 / 2., 0.773 / 2., 1.201 / 2., ChVector<>(-0.486, 0.153, -0.047));
        chassis->GetCollisionModel()->BuildModel();
        chassis->SetCollide(true);

        // visualization properties:
        auto chassis_mesh = chrono_types::make_shared<ChObjShapeFile>();
        chassis_mesh->SetFilename(GetChronoDataFile("models/forklift/body.obj"));
        chassis_mesh->SetTexture(GetChronoDataFile("textures/tire_truck.png"));
        chassis->AddVisualShape(chassis_mesh, ChFrame<>(-COG_truss, QUNIT));

        // contact material shared among all wheels
        auto wheel_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

        // visualization shape, shared among all wheels
        auto wheel_mesh = chrono_types::make_shared<ChObjShapeFile>();
        wheel_mesh->SetFilename(GetChronoDataFile("models/forklift/wheel.obj"));
        wheel_mesh->SetTexture(GetChronoDataFile("textures/tire_truck.png"));


        // ..the right-front wheel
        wheelRF = chrono_types::make_shared<ChBody>();
        sys->Add(wheelRF);
        wheelRF->SetPos(COG_wheelRF);
        wheelRF->SetMass(20);
        wheelRF->SetInertiaXX(ChVector<>(2, 2, 2));        
        // collision properties:
        ChMatrix33<> Arot(chrono::Q_from_AngZ(CH_C_PI / 2));
        wheelRF->GetCollisionModel()->ClearModel();
        wheelRF->GetCollisionModel()->AddCylinder(wheel_mat, RAD_front_wheel, RAD_front_wheel, 0.1, ChVector<>(0, 0, 0), Arot);
        wheelRF->GetCollisionModel()->BuildModel();
        wheelRF->SetCollide(true);
        // visualization properties:
        wheelRF->AddVisualShape(wheel_mesh, ChFrame<>(-COG_wheelRF, QUNIT));

        // .. create the revolute joint between the wheel and the truss
        link_revoluteRF = chrono_types::make_shared<ChLinkLockRevolute>();  // right, front, upper, 1
        link_revoluteRF->Initialize(wheelRF, chassis, ChCoordsys<>(COG_wheelRF, chrono::Q_from_AngY(CH_C_PI / 2)));
        sys->AddLink(link_revoluteRF);

        // ..the left-front wheel
        wheelLF = chrono_types::make_shared<ChBody>();
        sys->Add(wheelLF);
        wheelLF->SetPos(COG_wheelLF);
        wheelLF->SetRot(chrono::Q_from_AngY(CH_C_PI));  // reuse RF wheel shape, flipped
        wheelLF->SetMass(20);
        wheelLF->SetInertiaXX(ChVector<>(2, 2, 2));
        // collision properties:
        wheelLF->GetCollisionModel()->ClearModel();
        wheelLF->GetCollisionModel()->AddCylinder(wheel_mat, RAD_front_wheel, RAD_front_wheel, 0.1, ChVector<>(0, 0, 0), Arot);
        wheelLF->GetCollisionModel()->BuildModel();
        wheelLF->SetCollide(true);
        // visualization properties:
        wheelLF->AddVisualShape(wheel_mesh, ChFrame<>(-COG_wheelRF, QUNIT));

        // .. create the revolute joint between the wheel and the truss
        link_revoluteLF = chrono_types::make_shared<ChLinkLockRevolute>();  // right, front, upper, 1
        link_revoluteLF->Initialize(wheelLF, chassis, ChCoordsys<>(COG_wheelLF, chrono::Q_from_AngY(CH_C_PI / 2)));
        sys->AddLink(link_revoluteLF);

        // ..the back steering spindle (invisible)
        spindleB = chrono_types::make_shared<ChBody>();
        sys->Add(spindleB);
        spindleB->SetPos(COG_wheelB);
        spindleB->SetMass(10);
        spindleB->SetInertiaXX(ChVector<>(1, 1, 1));

        // .. create the vertical steering link between the spindle structure and the truss
        link_steer_engineB = chrono_types::make_shared<ChLinkMotorRotationAngle>();
        link_steer_engineB->SetAngleFunction(chrono_types::make_shared<ChFunction_Const>(0));
        link_steer_engineB->Initialize(spindleB, chassis, ChFrame<>(COG_wheelB, chrono::Q_from_AngX(CH_C_PI / 2)));
        sys->AddLink(link_steer_engineB);

        // ..the back wheel
        wheelB = chrono_types::make_shared<ChBody>();
        sys->Add(wheelB);
        wheelB->SetPos(COG_wheelB);
        wheelB->SetRot(chrono::Q_from_AngAxis(CH_C_PI, VECT_Y));
        wheelB->SetMass(20);
        wheelB->SetInertiaXX(ChVector<>(2, 2, 2));
        // collision properties:
        wheelB->GetCollisionModel()->ClearModel();
        wheelB->GetCollisionModel()->AddCylinder(wheel_mat, RAD_back_wheel, RAD_back_wheel, 0.1, ChVector<>(0, 0, 0), Arot);
        wheelB->GetCollisionModel()->BuildModel();
        wheelB->SetCollide(true);
        // visualization properties:
        wheelB->AddVisualShape(wheel_mesh, ChFrame<>(-COG_wheelRF, QUNIT));

        // .. create the motor between the back wheel and the steering spindle structure
        link_engineB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        link_engineB->SetSpeedFunction(chrono_types::make_shared<ChFunction_Const>(0));
        link_engineB->Initialize(wheelB, spindleB, ChFrame<>(COG_wheelB, chrono::Q_from_AngY(CH_C_PI / 2)));
        sys->AddLink(link_engineB);

        // ..the arm
        arm = chrono_types::make_shared<ChBody>();
        sys->Add(arm);
        arm->SetPos(COG_arm);
        arm->SetMass(100);
        arm->SetInertiaXX(ChVector<>(30, 30, 30));
        // visualization properties:
        auto arm_mesh = chrono_types::make_shared<ChObjShapeFile>();
        arm_mesh->SetFilename(GetChronoDataFile("models/forklift/arm.obj"));
        arm->AddVisualShape(arm_mesh, ChFrame<>(-COG_arm, QUNIT));

        // .. create the revolute joint between the arm and the truss
        link_engineArm = chrono_types::make_shared<ChLinkMotorRotationAngle>();
        link_engineArm->SetAngleFunction(chrono_types::make_shared<ChFunction_Const>(0));
        link_engineArm->Initialize(arm, chassis, ChFrame<>(POS_pivotarm, chrono::Q_from_AngY(CH_C_PI / 2)));
        sys->AddLink(link_engineArm);

        // ..the fork
        auto fork_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

        fork = chrono_types::make_shared<ChBody>();
        sys->Add(fork);
        fork->SetPos(COG_fork);
        fork->SetMass(60);
        fork->SetInertiaXX(ChVector<>(15, 15, 15));
        // collision properties:
        fork->GetCollisionModel()->ClearModel();
        fork->GetCollisionModel()->AddBox(fork_mat, 0.1 / 2., 0.032 / 2., 1.033 / 2., ChVector<>(-0.352, -0.312, 0.613));
        fork->GetCollisionModel()->AddBox(fork_mat, 0.1 / 2., 0.032 / 2., 1.033 / 2., ChVector<>(0.352, -0.312, 0.613));
        fork->GetCollisionModel()->AddBox(fork_mat, 0.344 / 2., 1.134 / 2., 0.101 / 2., ChVector<>(-0.000, 0.321, -0.009));
        fork->GetCollisionModel()->BuildModel();
        fork->SetCollide(true);
        // visualization properties:
        auto fork_mesh = chrono_types::make_shared<ChObjShapeFile>();
        fork_mesh->SetFilename(GetChronoDataFile("models/forklift/forks.obj"));
        fork->AddVisualShape(fork_mesh, ChFrame<>(-COG_fork, QUNIT));

        // .. create the prismatic joint between the fork and arm
        // (set joint as vertical; default would be aligned to z, horizontal)
        link_prismaticFork = chrono_types::make_shared<ChLinkLockPrismatic>();
        link_prismaticFork->Initialize(fork, arm, ChCoordsys<>(POS_prismatic, chrono::Q_from_AngX(CH_C_PI / 2)));
        sys->AddLink(link_prismaticFork);

        // .. create the linear actuator that pushes upward the fork
        link_actuatorFork = chrono_types::make_shared<ChLinkLinActuator>();
        link_actuatorFork->Initialize(fork, arm, false, ChCoordsys<>(POS_prismatic + ChVector<>(0, 0.01, 0), QUNIT),
                                      ChCoordsys<>(POS_prismatic, QUNIT));
        sys->AddLink(link_actuatorFork);

        // ..a pallet
        auto pallet_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

        // Create a body with a mesh visualization and collision shape.
        // In order to automatically infer mass and inertia properties, the mesh must be closed and watertight!
        // Specify an inflation radius which can improve robustness of the collision detection.
        auto pallet = chrono_types::make_shared<ChBodyEasyMesh>(  //
            GetChronoDataFile("models/pallet.obj"),               // mesh .OBJ file
            300,                                                  // density
            true,                                                 // automatic evaluation of inertia propserties
            true,                                                 // enable visualization
            true,                                                 // enable collision with mesh
            pallet_mat,                                           // contact material
            0.001);                                               // radius of mesh sweeping sphere
        sys->Add(pallet);
        pallet->SetPos(ChVector<>(0, 0.4, 3));

        // apply a texture to the pallet:
        pallet->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/cubetexture.png"));

        // Move the forklift to initial offset position

        ////pallet->Move(offset);
        chassis->Move(offset);
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
        ChSystem* mysystem = chassis->GetSystem();

        mysystem->Remove(link_revoluteRF);
        mysystem->Remove(link_revoluteLF);
        mysystem->Remove(link_steer_engineB);
        mysystem->Remove(link_engineB);
        mysystem->Remove(link_engineArm);
        mysystem->Remove(link_prismaticFork);
        mysystem->Remove(link_actuatorFork);
        mysystem->Remove(chassis);
        mysystem->Remove(wheelRF);
        mysystem->Remove(wheelLF);
        mysystem->Remove(wheelB);
        mysystem->Remove(spindleB);
        mysystem->Remove(arm);
        mysystem->Remove(fork);
    }
};

// Define a MyEventReceiver class which will be used to manage input from the GUI graphical user interface.
class MyEventReceiver : public IEventReceiver {
  public:
    MyEventReceiver(MySimpleForklift* mlift) {
        // store pointer to physical system & other stuff so we can tweak them by user keyboard
        forklift = mlift;
    }

    bool OnEvent(const SEvent& event) {
        // check if user presses keys
        if (event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) {
            switch (event.KeyInput.Key) {
                case irr::KEY_KEY_Q:
                    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(
                            forklift->link_steer_engineB->GetAngleFunction()))
                        mfun->Set_yconst(+0.3 + mfun->Get_yconst());
                    return true;
                case irr::KEY_KEY_W:
                    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(
                            forklift->link_steer_engineB->GetAngleFunction()))
                        mfun->Set_yconst(-0.3 + mfun->Get_yconst());
                    return true;
                case irr::KEY_KEY_A:
                    if (auto mfun =
                            std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_engineB->GetSpeedFunction()))
                        mfun->Set_yconst(0.5 + mfun->Get_yconst());
                    return true;
                case irr::KEY_KEY_Z:
                    if (auto mfun =
                            std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_engineB->GetSpeedFunction()))
                        mfun->Set_yconst(-0.5 + mfun->Get_yconst());
                    return true;
                case irr::KEY_KEY_S:
                    if (auto mfun =
                            std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_actuatorFork->Get_dist_funct()))
                        mfun->Set_yconst(0.05 + mfun->Get_yconst());
                    return true;
                case irr::KEY_KEY_X:
                    if (auto mfun =
                            std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_actuatorFork->Get_dist_funct()))
                        mfun->Set_yconst(-0.05 + mfun->Get_yconst());
                    return true;
                case irr::KEY_KEY_D:
                    if (auto mfun =
                            std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_engineArm->GetAngleFunction()))
                        mfun->Set_yconst(0.005 + mfun->Get_yconst());
                    return true;
                case irr::KEY_KEY_C:
                    if (auto mfun =
                            std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_engineArm->GetAngleFunction()))
                        mfun->Set_yconst(-0.005 + mfun->Get_yconst());
                    return true;
                default:
                    break;
            }
        }

        return false;
    }

  private:
    MySimpleForklift* forklift;
};

//
// This is the program which is executed
//

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC sys;




    // Contact material for ground
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ground_mat->SetFriction(1.0f);

    // ..the world
    auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(40, 2, 40, 1000, true, true, ground_mat);
    sys.Add(my_ground);
    my_ground->SetBodyFixed(true);
    my_ground->SetPos(ChVector<>(0, -1, 0));
    my_ground->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"));

    // ..some obstacles on the ground:
    for (int i = 0; i < 6; i++) {
        auto my_obstacle = chrono_types::make_shared<ChBodyEasyBox>(1, 0.5, 1, 200, true, true, ground_mat);
        sys.Add(my_obstacle);
        my_obstacle->SetPos(ChVector<>(20 * ChRandom(), 2, 20 * ChRandom()));
        my_obstacle->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/cubetexture_wood.png"));
    }

    // ..the forklift (this class - see above - is a 'set' of bodies and links, automatically added at creation)
    MySimpleForklift* myforklift = new MySimpleForklift(&sys);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    sys.SetVisualSystem(vis);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Forklift demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector<>(-6, 3, -6));

    // Create some graphical-user-interface (GUI) items to show on the screen.
    // This requires an event receiver object -see above.
    // This is for GUI tweaking of system parameters..
    MyEventReceiver receiver(myforklift);
    // note how to add a custom event receiver to the default interface:
    vis->AddUserEventReceiver(&receiver);

    // add text with info
    vis->GetGUIEnvironment()->addStaticText(L"Keys: steer=Q,W; throttle=A,Z; lift=S,X; bank=D,C",
                                            rect<s32>(150, 10, 430, 40), true);

    // Solver settings
    sys.SetSolverType(ChSolver::Type::PSOR);
    sys.SetSolverMaxIterations(20);  // the higher, the easier to keep the constraints satisfied.

    // Simulation loop
    double timestep = 0.005;
    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        vis->BeginScene();
        vis->DrawAll();

        // Advance the simulation time step
        sys.DoStepDynamics(timestep);

        vis->EndScene();

        realtime_timer.Spin(timestep);
    }

    if (myforklift)
        delete myforklift;

    return 0;
}
