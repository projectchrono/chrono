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
// Authors: Simone Benatti
// =============================================================================
//
// Demo of the hexicopter UAV
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_models/robot/copters/Little_Hexy.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::copter;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::video;
using namespace irr::gui;

/// Following class will be used to manage events from the user interface

class MyEventReceiver : public IEventReceiver {
  public:
    MyEventReceiver(Little_Hexy* hexy) : copter(hexy) {}

    bool OnEvent(const SEvent& event) {
        // check if user presses keys
        if (event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) {
            switch (event.KeyInput.Key) {
                case irr::KEY_KEY_W:
                    copter->Pitch_Down(0.001);
                    std::cout << "Pressing W\n";

                    return true;
                case irr::KEY_KEY_S:
                    copter->Pitch_Up(0.001);
                    std::cout << "Pressing S\n";
                    return true;

                case irr::KEY_KEY_A:
                    copter->Roll_Left(0.001);
                    std::cout << "Pressing A\n";
                    return true;

                case irr::KEY_KEY_D:
                    copter->Roll_Right(0.001);
                    std::cout << "Pressing D\n";
                    return true;

                case irr::KEY_NUMPAD4:
                    copter->Yaw_Left(0.01);
                    std::cout << "Pressing 4\n";
                    return true;

                case irr::KEY_NUMPAD6:
                    copter->Yaw_Right(0.01);
                    std::cout << "Pressing 6\n";
                    return true;

                case irr::KEY_NUMPAD8:
                    copter->Throttle(0.01);
                    std::cout << "Pressing 8\n";
                    return true;

                case irr::KEY_NUMPAD2:
                    copter->Throttle(-0.01);
                    std::cout << "Pressing 2\n";
                    return true;

                default:
                    break;
            }
        }

        return false;
    }

  private:
    Little_Hexy* copter;
};

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));

    Little_Hexy myhexy(sys, VNULL);
    myhexy.AddVisualizationAssets();
    auto mymat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    myhexy.AddCollisionShapes(mymat);

    // Create the ground for the collision
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ground_mat->SetFriction(0.5);

    auto ground = chrono_types::make_shared<ChBodyEasyBox>(200, 200, 1,  // size
                                                           1000,         // density
                                                           true,         // visualize
                                                           true,         // collide
                                                           ground_mat);  // contact material
    ground->SetPos(ChVector<>(0, 0, -3));
    ground->SetBodyFixed(true);
    ground->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"), 100, 100);
    sys.Add(ground);

    // Create the Irrlicht visualization sys
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("HexaCopter Test");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(5, 5, 2));
    vis->AddTypicalLights();

    // create text with info
    vis->GetGUIEnvironment()->addStaticText(
        L"Keys: NUMPAD 8 up; NUMPAD 2 down; A Roll Left; D Roll Right; A Roll Left; W Pitch Down; S Pitch Up; NUMPAD 4 "
        L"Yaw_Left; NUMPAD 6 Yaw_Right",
        rect<s32>(150, 10, 430, 40), true);

    // This is for GUI tweaking of system parameters
    MyEventReceiver receiver(&myhexy);
    vis->AddUserEventReceiver(&receiver);

    // Prepare the physical system for the simulation

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);

    sys.SetSolverType(ChSolver::Type::PSOR);
    sys.SetSolverMaxIterations(30);

    // Simulation loop

    double time_step = 0.005;
    ChRealtimeStepTimer realtime_timer;

    double control[] = {.6, .6, .6, .6, .6, .6};
    myhexy.ControlAbsolute(control);

    while (vis->Run()) {
        ChVector<float> pos = myhexy.GetChassis()->GetPos();
        vis->UpdateCamera(pos + ChVector<>(1,-1,1), pos);

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // ADVANCE THE SIMULATION FOR ONE TIMESTEP
        myhexy.Update(0.01);
        sys.DoStepDynamics(time_step);
        realtime_timer.Spin(time_step);
    }

    return 0;
}
