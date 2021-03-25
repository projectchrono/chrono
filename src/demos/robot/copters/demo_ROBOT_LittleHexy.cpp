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
#include "chrono_irrlicht/ChIrrApp.h"
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
    MyEventReceiver(ChIrrAppInterface* myapp, Little_Hexy* mycopter) {
        // store pointer to physical system & other stuff so we can tweak them by user keyboard
        app = myapp;
        copter = mycopter;
    }

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
    ChIrrAppInterface* app;
    Little_Hexy* copter;
};

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;
    mphysicalSystem.Set_G_acc(ChVector<>(0, 0, -9.81));

    Little_Hexy myhexy(mphysicalSystem, VNULL);
    myhexy.AddVisualizationAssets();
    auto mymat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    myhexy.AddCollisionShapes(mymat);

    // Create the Irrlicht visualization.
    ChIrrApp application(&mphysicalSystem, L"HexaCopter Test", core::dimension2d<u32>(800, 600), VerticalDir::Z);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(5, 5, 2));

    // create text with info
    application.GetIGUIEnvironment()->addStaticText(
        L"Keys: NUMPAD 8 up; NUMPAD 2 down; A Roll Left; D Roll Right; A Roll Left; W Pitch Down; S Pitch Up; NUMPAD 4 "
        L"Yaw_Left; NUMPAD 6 Yaw_Right",
        rect<s32>(150, 10, 430, 40), true);

    // This is for GUI tweaking of system parameters..
    MyEventReceiver receiver(&application, &myhexy);
    // note how to add a custom event receiver to the default interface:
    application.SetUserEventReceiver(&receiver);

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
    mphysicalSystem.Add(ground);

    auto mtexture = chrono_types::make_shared<ChTexture>();
    mtexture->SetTextureFilename(GetChronoDataFile("textures/concrete.jpg"));
    mtexture->SetTextureScale(100, 100);

    ground->AddAsset(mtexture);

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Prepare the physical system for the simulation

    mphysicalSystem.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);

    mphysicalSystem.SetSolverType(ChSolver::Type::PSOR);
    mphysicalSystem.SetSolverMaxIterations(30);

    // Simulation loop

    application.SetTimestep(0.005);
    application.SetTryRealtime(true);
    
    double control[] = {.6, .6, .6, .6, .6, .6};
    myhexy.ControlAbsolute(control);

    while (application.GetDevice()->run()) {
        ChVector<float> pos = myhexy.GetChassis()->GetPos();
        core::vector3df ipos(pos.x(), pos.y(), pos.z());
        core::vector3df offset(1, -1, 1);
        application.GetActiveCamera()->setPosition(ipos + offset);
        application.GetActiveCamera()->setTarget(ipos);

        application.BeginScene(true, true, SColor(255, 140, 161, 192));
        application.DrawAll();
        application.EndScene();

        // ADVANCE THE SIMULATION FOR ONE TIMESTEP
        myhexy.Update(0.01);
        application.DoStep();

        // change motor speeds depending on user setpoints from GUI

    }

    return 0;
}
