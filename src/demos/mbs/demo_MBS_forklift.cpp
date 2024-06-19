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
#include "chrono/physics/ChLinkLockLinActuator.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/core/ChRandom.h"

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
    std::shared_ptr<ChLinkLockLinActuator> link_actuatorFork;
    std::shared_ptr<ChLinkLockPrismatic> link_prismaticFork;

    // Build and initialize the forklift, creating all bodies corresponding to
    // the various parts and adding them to the physical system - also creating
    // and adding constraints to the system.
    MySimpleForklift(ChSystem* sys, ChVector3d offset = ChVector3d(0, 0, 0)) {
        throttle = 0;  // initially, gas throttle is 0.
        steer = 0;
        lift = 0;

        ChVector3d COG_truss(0, 0.4, 0.5);
        ChVector3d COG_wheelRF(-0.566, 0.282, 1.608);
        ChVector3d COG_wheelLF(0.566, 0.282, 1.608);
        ChVector3d COG_arm(0, 1.300, 1.855);
        ChVector3d COG_fork(0, 0.362, 2.100);
        ChVector3d COG_wheelB(0, 0.282, 0.003);
        ChVector3d POS_pivotarm(0, 0.150, 1.855);
        ChVector3d POS_prismatic(0, 0.150, 1.855);
        double RAD_back_wheel = 0.28;
        double RAD_front_wheel = 0.28;

        // --- The car body ---

        chassis = chrono_types::make_shared<ChBody>();
        sys->Add(chassis);
        chassis->SetPos(COG_truss);
        chassis->SetMass(200);
        chassis->SetInertiaXX(ChVector3d(100, 100, 100));

        // collision properties:
        auto chassis_mat = chrono_types::make_shared<ChContactMaterialNSC>();

        auto cshape1 = chrono_types::make_shared<ChCollisionShapeBox>(chassis_mat, 1.227, 1.621, 1.864);
        auto cshape2 = chrono_types::make_shared<ChCollisionShapeBox>(chassis_mat, 0.187, 0.773, 1.201);
        auto cshape3 = chrono_types::make_shared<ChCollisionShapeBox>(chassis_mat, 0.187, 0.773, 1.201);
        chassis->AddCollisionShape(cshape1, ChFrame<>(ChVector3d(-0.003, 1.019, 0.192), QUNIT));
        chassis->AddCollisionShape(cshape2, ChFrame<>(ChVector3d(0.486, 0.153, -0.047), QUNIT));
        chassis->AddCollisionShape(cshape3, ChFrame<>(ChVector3d(-0.486, 0.153, -0.047), QUNIT));
        chassis->EnableCollision(true);

        // visualization properties:
        auto chassis_mesh = chrono_types::make_shared<ChVisualShapeModelFile>();
        chassis_mesh->SetFilename(GetChronoDataFile("models/forklift/body.obj"));
        chassis->AddVisualShape(chassis_mesh, ChFrame<>(-COG_truss, QUNIT));

        // contact material shared among all wheels
        auto wheel_mat = chrono_types::make_shared<ChContactMaterialNSC>();

        // visualization shape, shared among all wheels
        auto wheel_mesh = chrono_types::make_shared<ChVisualShapeModelFile>();
        wheel_mesh->SetFilename(GetChronoDataFile("models/forklift/wheel.obj"));

        // Front and back wheel collision shapes
        auto wshapeF = chrono_types::make_shared<ChCollisionShapeCylinder>(wheel_mat, RAD_front_wheel, 0.2);
        auto wshapeB = chrono_types::make_shared<ChCollisionShapeCylinder>(wheel_mat, RAD_back_wheel, 0.2);

        // ..the right-front wheel
        wheelRF = chrono_types::make_shared<ChBody>();
        sys->Add(wheelRF);
        wheelRF->SetPos(COG_wheelRF);
        wheelRF->SetMass(20);
        wheelRF->SetInertiaXX(ChVector3d(2, 2, 2));
        // collision properties:
        wheelRF->AddCollisionShape(wshapeF, ChFrame<>(VNULL, QuatFromAngleY(CH_PI / 2)));
        wheelRF->EnableCollision(true);
        // visualization properties:
        wheelRF->AddVisualShape(wheel_mesh, ChFrame<>(-COG_wheelRF, QUNIT));

        // .. create the revolute joint between the wheel and the truss
        link_revoluteRF = chrono_types::make_shared<ChLinkLockRevolute>();  // right, front, upper, 1
        link_revoluteRF->Initialize(wheelRF, chassis, ChFrame<>(COG_wheelRF, chrono::QuatFromAngleY(CH_PI / 2)));
        sys->AddLink(link_revoluteRF);

        // ..the left-front wheel
        wheelLF = chrono_types::make_shared<ChBody>();
        sys->Add(wheelLF);
        wheelLF->SetPos(COG_wheelLF);
        wheelLF->SetRot(chrono::QuatFromAngleY(CH_PI));  // reuse RF wheel shape, flipped
        wheelLF->SetMass(20);
        wheelLF->SetInertiaXX(ChVector3d(2, 2, 2));
        // collision properties:
        auto shapeLF = chrono_types::make_shared<ChCollisionShapeCylinder>(wheel_mat, RAD_front_wheel, 0.2);
        wheelLF->AddCollisionShape(wshapeF, ChFrame<>(VNULL, QuatFromAngleY(CH_PI / 2)));
        wheelLF->EnableCollision(true);
        // visualization properties:
        wheelLF->AddVisualShape(wheel_mesh, ChFrame<>(-COG_wheelRF, QUNIT));

        // .. create the revolute joint between the wheel and the truss
        link_revoluteLF = chrono_types::make_shared<ChLinkLockRevolute>();  // right, front, upper, 1
        link_revoluteLF->Initialize(wheelLF, chassis, ChFrame<>(COG_wheelLF, chrono::QuatFromAngleY(CH_PI / 2)));
        sys->AddLink(link_revoluteLF);

        // ..the back steering spindle (invisible)
        spindleB = chrono_types::make_shared<ChBody>();
        sys->Add(spindleB);
        spindleB->SetPos(COG_wheelB);
        spindleB->SetMass(10);
        spindleB->SetInertiaXX(ChVector3d(1, 1, 1));

        // .. create the vertical steering link between the spindle structure and the truss
        link_steer_engineB = chrono_types::make_shared<ChLinkMotorRotationAngle>();
        link_steer_engineB->SetAngleFunction(chrono_types::make_shared<ChFunctionConst>(0));
        link_steer_engineB->Initialize(spindleB, chassis, ChFrame<>(COG_wheelB, QuatFromAngleX(CH_PI_2)));
        sys->AddLink(link_steer_engineB);

        // ..the back wheel
        wheelB = chrono_types::make_shared<ChBody>();
        sys->Add(wheelB);
        wheelB->SetPos(COG_wheelB);
        wheelB->SetRot(chrono::QuatFromAngleY(CH_PI));
        wheelB->SetMass(20);
        wheelB->SetInertiaXX(ChVector3d(2, 2, 2));
        // collision properties:
        wheelB->AddCollisionShape(wshapeB, ChFrame<>(VNULL, QuatFromAngleY(CH_PI_2)));
        wheelB->EnableCollision(true);
        // visualization properties:
        wheelB->AddVisualShape(wheel_mesh, ChFrame<>(-COG_wheelRF, QUNIT));

        // .. create the motor between the back wheel and the steering spindle structure
        link_engineB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        link_engineB->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0));
        link_engineB->Initialize(wheelB, spindleB, ChFrame<>(COG_wheelB, chrono::QuatFromAngleY(CH_PI / 2)));
        sys->AddLink(link_engineB);

        // ..the arm
        arm = chrono_types::make_shared<ChBody>();
        sys->Add(arm);
        arm->SetPos(COG_arm);
        arm->SetMass(100);
        arm->SetInertiaXX(ChVector3d(30, 30, 30));
        // visualization properties:
        auto arm_mesh = chrono_types::make_shared<ChVisualShapeModelFile>();
        arm_mesh->SetFilename(GetChronoDataFile("models/forklift/arm.obj"));
        arm->AddVisualShape(arm_mesh, ChFrame<>(-COG_arm, QUNIT));

        // .. create the revolute joint between the arm and the truss
        link_engineArm = chrono_types::make_shared<ChLinkMotorRotationAngle>();
        link_engineArm->SetAngleFunction(chrono_types::make_shared<ChFunctionConst>(0));
        link_engineArm->Initialize(arm, chassis, ChFrame<>(POS_pivotarm, chrono::QuatFromAngleY(CH_PI / 2)));
        sys->AddLink(link_engineArm);

        // ..the fork
        auto fork_mat = chrono_types::make_shared<ChContactMaterialNSC>();

        fork = chrono_types::make_shared<ChBody>();
        sys->Add(fork);
        fork->SetPos(COG_fork);
        fork->SetMass(60);
        fork->SetInertiaXX(ChVector3d(15, 15, 15));
        // collision properties:
        auto fshape1 = chrono_types::make_shared<ChCollisionShapeBox>(fork_mat, 0.100, 0.032, 1.033);
        auto fshape2 = chrono_types::make_shared<ChCollisionShapeBox>(fork_mat, 0.100, 0.032, 1.033);
        auto fshape3 = chrono_types::make_shared<ChCollisionShapeBox>(fork_mat, 0.344, 1.134, 0.101);
        fork->AddCollisionShape(fshape1, ChFrame<>(ChVector3d(-0.352, -0.312, 0.613), QUNIT));
        fork->AddCollisionShape(fshape2, ChFrame<>(ChVector3d(0.352, -0.312, 0.613), QUNIT));
        fork->AddCollisionShape(fshape3, ChFrame<>(ChVector3d(0.000, 0.321, -0.009), QUNIT));
        fork->EnableCollision(true);
        // visualization properties:
        auto fork_mesh = chrono_types::make_shared<ChVisualShapeModelFile>();
        fork_mesh->SetFilename(GetChronoDataFile("models/forklift/forks.obj"));
        fork->AddVisualShape(fork_mesh, ChFrame<>(-COG_fork, QUNIT));

        // .. create the prismatic joint between the fork and arm
        // (set joint as vertical; default would be aligned to z, horizontal)
        link_prismaticFork = chrono_types::make_shared<ChLinkLockPrismatic>();
        link_prismaticFork->Initialize(fork, arm, ChFrame<>(POS_prismatic, QuatFromAngleX(CH_PI / 2)));
        sys->AddLink(link_prismaticFork);

        // .. create the linear actuator that pushes upward the fork
        link_actuatorFork = chrono_types::make_shared<ChLinkLockLinActuator>();
        link_actuatorFork->Initialize(fork, arm, false, ChFrame<>(POS_prismatic + ChVector3d(0, 0.01, 0), QUNIT),
                                      ChFrame<>(POS_prismatic, QUNIT));
        sys->AddLink(link_actuatorFork);

        // ..a pallet
        auto pallet_mat = chrono_types::make_shared<ChContactMaterialNSC>();

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
        pallet->SetPos(ChVector3d(0, 0.4, 3));

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
                    if (auto mfun = std::dynamic_pointer_cast<ChFunctionConst>(
                            forklift->link_steer_engineB->GetAngleFunction()))
                        mfun->SetConstant(+0.3 + mfun->GetConstant());
                    return true;
                case irr::KEY_KEY_W:
                    if (auto mfun = std::dynamic_pointer_cast<ChFunctionConst>(
                            forklift->link_steer_engineB->GetAngleFunction()))
                        mfun->SetConstant(-0.3 + mfun->GetConstant());
                    return true;
                case irr::KEY_KEY_A:
                    if (auto mfun =
                            std::dynamic_pointer_cast<ChFunctionConst>(forklift->link_engineB->GetSpeedFunction()))
                        mfun->SetConstant(0.5 + mfun->GetConstant());
                    return true;
                case irr::KEY_KEY_Z:
                    if (auto mfun =
                            std::dynamic_pointer_cast<ChFunctionConst>(forklift->link_engineB->GetSpeedFunction()))
                        mfun->SetConstant(-0.5 + mfun->GetConstant());
                    return true;
                case irr::KEY_KEY_S:
                    if (auto mfun = std::dynamic_pointer_cast<ChFunctionConst>(
                            forklift->link_actuatorFork->GetActuatorFunction()))
                        mfun->SetConstant(0.05 + mfun->GetConstant());
                    return true;
                case irr::KEY_KEY_X:
                    if (auto mfun = std::dynamic_pointer_cast<ChFunctionConst>(
                            forklift->link_actuatorFork->GetActuatorFunction()))
                        mfun->SetConstant(-0.05 + mfun->GetConstant());
                    return true;
                case irr::KEY_KEY_D:
                    if (auto mfun =
                            std::dynamic_pointer_cast<ChFunctionConst>(forklift->link_engineArm->GetAngleFunction()))
                        mfun->SetConstant(0.005 + mfun->GetConstant());
                    return true;
                case irr::KEY_KEY_C:
                    if (auto mfun =
                            std::dynamic_pointer_cast<ChFunctionConst>(forklift->link_engineArm->GetAngleFunction()))
                        mfun->SetConstant(-0.005 + mfun->GetConstant());
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

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create a Chrono physical system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Contact material for ground
    auto ground_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    ground_mat->SetFriction(1.0f);

    // ..the world
    auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(40, 2, 40, 1000, true, true, ground_mat);
    sys.Add(my_ground);
    my_ground->SetFixed(true);
    my_ground->SetPos(ChVector3d(0, -1, 0));
    my_ground->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"));

    // ..some obstacles on the ground:
    for (int i = 0; i < 6; i++) {
        auto my_obstacle = chrono_types::make_shared<ChBodyEasyBox>(1, 0.5, 1, 200, true, true, ground_mat);
        sys.Add(my_obstacle);
        my_obstacle->SetPos(ChVector3d(20 * ChRandom::Get(), 2, 20 * ChRandom::Get()));
        my_obstacle->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/cubetexture_wood.png"));
    }

    // ..the forklift (this class - see above - is a 'set' of bodies and links, automatically added at creation)
    MySimpleForklift* myforklift = new MySimpleForklift(&sys);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Forklift demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(-6, 3, -6));

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
    sys.GetSolver()->AsIterative()->SetMaxIterations(20);

    // Simulation loop
    double timestep = 0.005;
    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();

        // Advance the simulation time step
        sys.DoStepDynamics(timestep);

        vis->EndScene();

        realtime_timer.Spin(timestep);
    }

    if (myforklift)
        delete myforklift;

    return 0;
}
