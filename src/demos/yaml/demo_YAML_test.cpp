// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Simple demo for populating a Chrono system from a YAML model file and
// simulating it with parameters from a YAML simulation file.
//
// =============================================================================

////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include "chrono_parsers/ChParserYAML.h"

#include "chrono/assets/ChVisualSystem.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;

// -----------------------------------------------------------------------------

bool second_instance = false;  // create a second instance of the model
ChFramed frame1 = second_instance ? ChFramed(ChVector3d(0, -1, 0), QUNIT) : ChFramed(ChVector3d(0, 0, 0), QUNIT);
ChFramed frame2 = ChFramed(ChVector3d(0, +1, 0), QUNIT);
std::string prefix1 = second_instance ? "m1_" : "";
std::string prefix2 = "m2_";
int instance1 = -1;
int instance2 = -1;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2025 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Extract filenames from command-line arguments
    std::string model_yaml_filename = GetChronoDataFile("yaml/models/slider_crank.yaml");
    std::string sim_yaml_filename = GetChronoDataFile("yaml/simulations/basic_mbs.yaml");

    ChCLI cli(argv[0], "");
    cli.AddOption<std::string>("", "m,model_file", "model specification YAML file", model_yaml_filename);
    cli.AddOption<std::string>("", "s,sim_file", "simulation specification YAML file", sim_yaml_filename);

    if (!cli.Parse(argc, argv, true))
        return 1;

    if (argc == 1) {
        cli.Help();
        std::cout << "Using default YAML model and simulation specification" << std::endl;
    }

    std::cout << std::endl;
    std::cout << "Model YAML file:        " << model_yaml_filename << std::endl;
    std::cout << "Simulation YAML file:   " << sim_yaml_filename << std::endl;

    // Create YAML parser object
    parsers::ChParserYAML parser;
    parser.SetVerbose(true);

    // Load the YAML simulation file and create a Chrono system based on its content
    parser.LoadSimulationFile(sim_yaml_filename);
    auto sys = parser.CreateSystem();

    // Load the YAML model and populate the Chrono system
    parser.LoadModelFile(model_yaml_filename);
    instance1 = parser.Populate(*sys, frame1, prefix1);
    if (second_instance)
        instance2 = parser.Populate(*sys, frame2, prefix2);

    // Extract information from parsed YAML files
    const std::string& model_name = parser.GetName();
    double time_end = parser.GetEndtime();
    double time_step = parser.GetTimestep();
    bool real_time = parser.EnforceRealtime();
    bool render = parser.Render();
    double render_fps = parser.GetRenderFPS();
    CameraVerticalDir camera_vertical = parser.GetCameraVerticalDir();
    const ChVector3d& camera_location = parser.GetCameraLocation();
    const ChVector3d& camera_target = parser.GetCameraTarget();
    bool enable_shadows = parser.EnableShadows();

    // Print system hierarchy
    ////sys->ShowHierarchy(std::cout);

    // Create the run-time visualization system
    std::shared_ptr<ChVisualSystem> vis;
    if (render) {
        ChVisualSystem::Type vis_type;

#if defined(CHRONO_VSG)
        vis_type = ChVisualSystem::Type::VSG;
#elif defined(CHRONO_IRRLICHT)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#else
        std::cout << "No Chrono run-time visualization module enabled. Disabling visualization." << std::endl;
        render = false;
#endif

        switch (vis_type) {
            case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
                auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
                vis_irr->SetWindowSize(800, 600);
                vis_irr->SetWindowTitle("YAML model - " + model_name);
                vis_irr->SetCameraVertical(camera_vertical);
                vis_irr->Initialize();
                vis_irr->AddLogo();
                vis_irr->AddTypicalLights();
                vis_irr->AddCamera(camera_location, camera_target);
                vis_irr->AttachSystem(sys.get());

                vis = vis_irr;
#endif
                break;
            }
            default:
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
                vis_vsg->AttachSystem(sys.get());
                vis_vsg->SetWindowTitle("YAML model - " + model_name);
                vis_vsg->AddCamera(camera_location, camera_target);
                vis_vsg->SetWindowSize(1280, 800);
                vis_vsg->SetWindowPosition(100, 100);
                vis_vsg->SetBackgroundColor(ChColor(0.4f, 0.45f, 0.55f));
                vis_vsg->SetCameraVertical(camera_vertical);
                vis_vsg->SetCameraAngleDeg(40.0);
                vis_vsg->SetLightIntensity(1.0f);
                vis_vsg->SetLightDirection(-CH_PI_4, CH_PI_4);
                vis_vsg->EnableShadows(enable_shadows);
                vis_vsg->ToggleAbsFrameVisibility();
                vis_vsg->SetAbsFrameScale(2.0);
                vis_vsg->Initialize();

                vis = vis_vsg;
#endif
                break;
            }
        }
    }

    // Simulation loop
    ChRealtimeStepTimer rt_timer;
    double time = 0;
    int render_frame = 0;

    while (true) {
        if (render) {
            if (!vis->Run())
                break;
            if (time >= render_frame / render_fps) {
                vis->BeginScene();
                vis->Render();
                vis->EndScene();
                render_frame++;
            }
        } else if (time_end > 0 || time >= time_end) {
            break;
        }

        sys->DoStepDynamics(time_step);
        if (real_time)
            rt_timer.Spin(time_step);
        time += time_step;
    }

    return 0;
}
