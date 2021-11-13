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
// Basic demonstration of multiple wheeled vehicles in a single simulation using
// the SynChrono wrapper
//
// =============================================================================

#include <chrono>

#include "chrono_models/robot/copters/Little_Hexy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/agent/SynCopterAgent.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_synchrono/communication/mpi/SynMPICommunicator.h"
#include "chrono_synchrono/utils/SynLog.h"
#include "chrono_synchrono/utils/SynDataLoader.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::synchrono;
using namespace chrono::copter;
using namespace irr;
using namespace irr::core;
using namespace irr::gui;

// =============================================================================

class IrrAppWrapper {
  public:
    IrrAppWrapper(std::shared_ptr<ChIrrApp> app = nullptr) : app(app) {}

    /*void Advance() {
        if (app)
            app->DoStep();
    }*/

    void Render() {
        if (app) {
            app->BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app->DrawAll();
            app->EndScene();
        }
    }

    void Set(std::shared_ptr<ChIrrApp> app) { this->app = app; }
    bool IsOk() { return app ? app->GetDevice()->run() : true; }

    std::shared_ptr<ChIrrApp> app;
};

//
class MyEventReceiver : public IEventReceiver {
  public:
    //MyEventReceiver();

	void Initialize(ChIrrAppInterface* myapp, Little_Hexy* mycopter) {
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

// Initial copter location and orientation
ChVector<> initLoc(0, 0, 1.0);
ChQuaternion<> initRot(1, 0, 0, 0);

// Point on chassis tracked by the camera
// ChVector<> trackPoint(0.0, 0.0, 1.75);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;

// Simulation step sizes
double step_size = 1e-2;

// Simulation end time
double end_time = 1000;

// How often SynChrono state messages are interchanged
double heartbeat = 1e-2;  // 100[Hz]

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// =============================================================================

// Forward declares for straight forward helper functions
void LogCopyright(bool show);
void AddCommandLineOptions(ChCLI& cli);

// =============================================================================

int main(int argc, char* argv[]) {
    // -----------------------
    // Create SynChronoManager
    // -----------------------
    auto communicator = chrono_types::make_shared<SynMPICommunicator>(argc, argv);
    int node_id = communicator->GetRank();
    int num_nodes = communicator->GetNumRanks();
    SynChronoManager syn_manager(node_id, num_nodes, communicator);

    // Copyright
    LogCopyright(node_id == 0);

    // -----------------------------------------------------
    // CLI SETUP - Get most parameters from the command line
    // -----------------------------------------------------

    ChCLI cli(argv[0]);

    AddCommandLineOptions(cli);
    if (!cli.Parse(argc, argv, node_id == 0))
        return 0;

    // Normal simulation options
    step_size = cli.GetAsType<double>("step_size");
    end_time = cli.GetAsType<double>("end_time");
    heartbeat = cli.GetAsType<double>("heartbeat");

    // Change SynChronoManager settings
    syn_manager.SetHeartbeat(heartbeat);

    // --------------
    // Create systems
    // --------------
    double distance = node_id * 2.5;
    // Create the vehicle, set parameters, and initialize
    ChSystemNSC mphysicalSystem;
    mphysicalSystem.Set_G_acc(ChVector<>(0, 0, -9.81));
    Little_Hexy myhexy(mphysicalSystem, ChVector<>(0, distance, 0));
    myhexy.AddVisualizationAssets();
    auto mymat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    myhexy.AddCollisionShapes(mymat);

    // Add vehicle as an agent and initialize SynChronoManager
    auto agent = chrono_types::make_shared<SynCopterAgent>(&myhexy);
    syn_manager.AddAgent(agent);
    syn_manager.Initialize(&mphysicalSystem);

    IrrAppWrapper appwrap;
    MyEventReceiver receiver;
    // Create the vehicle Irrlicht interface
    if (cli.HasValueInVector<int>("irr", node_id)) {
        auto application = chrono_types::make_shared<ChIrrApp>(&mphysicalSystem, L"HexaCopter Test",
                                                               core::dimension2d<u32>(800, 600));
        appwrap.Set(application);

        // create text with info
        application->GetIGUIEnvironment()->addStaticText(
            L"Keys: NUMPAD 8 up; NUMPAD 2 down; A Roll Left; D Roll Right; A Roll Left; W Pitch Down; S Pitch Up; "
            L"NUMPAD 4 "
            L"Yaw_Left; NUMPAD 6 Yaw_Right",
            rect<s32>(150, 10, 430, 55), true);

        RTSCamera* camera =
            new RTSCamera(application->GetDevice(), application->GetDevice()->getSceneManager()->getRootSceneNode(),
                          application->GetDevice()->getSceneManager(), -1, -160.0f, 1.0f, 0.003f);

        camera->setPosition(core::vector3df(5, 5, 2));
        camera->setTarget(core::vector3df(0, 0, 0));

        // This is for GUI tweaking of system parameters..
        receiver.Initialize(application.get(), &myhexy);
        // note how to add a custom event receiver to the default interface:
        application->SetUserEventReceiver(&receiver);
        application->AssetBindAll();
        application->AssetUpdateAll();
    }

    // ---------------
    // Simulation loop
    // ---------------
    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;

    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

    double control[] = {.6, .6, .6, .6, .6, .6};
    myhexy.ControlAbsolute(control);

    while (appwrap.IsOk() && syn_manager.IsOk()) {
        double time = mphysicalSystem.GetChTime();

        // End simulation
        if (time >= end_time)
            break;

        // Render scene
        if (step_number % render_steps == 0)
            appwrap.Render();
        syn_manager.Synchronize(mphysicalSystem.GetChTime());
        myhexy.Update(step_size);
        mphysicalSystem.DoStepDynamics(step_size);

        // appwrap.Advance();

        // Increment frame number
        step_number++;

        // Log clock time
        if (step_number % 100 == 0 && node_id == 0) {
            std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
            auto time_span = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            SynLog() << (time_span.count() / 1e3) / time << "\n";
        }
    }
    // Properly shuts down other ranks when one rank ends early
    syn_manager.QuitSimulation();
    return 0;
}

void LogCopyright(bool show) {
    if (!show)
        return;

    SynLog() << "Copyright (c) 2020 projectchrono.org\n";
    SynLog() << "Chrono version: " << CHRONO_VERSION << "\n\n";
}

void AddCommandLineOptions(ChCLI& cli) {
    // Standard demo options
    cli.AddOption<double>("Simulation", "s,step_size", "Step size", std::to_string(step_size));
    cli.AddOption<double>("Simulation", "e,end_time", "End time", std::to_string(end_time));
    cli.AddOption<double>("Simulation", "b,heartbeat", "Heartbeat", std::to_string(heartbeat));

    // Irrlicht options
    cli.AddOption<std::vector<int>>("Irrlicht", "i,irr", "Nodes for irrlicht usage", "-1");
}
