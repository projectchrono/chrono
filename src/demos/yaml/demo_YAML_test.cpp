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
// Simple demo for populating a Chrono system from a YAML model file.
//
// =============================================================================

////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include "chrono/physics/ChSystem.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChYamlParser.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::utils;

// -----------------------------------------------------------------------------

std::string model_yaml_filename = "yaml/slider_crank.yaml";
////std::string model_yaml_filename = "yaml/slider_crank_reduced.yaml";

ChContactMethod contact_method = ChContactMethod::SMC;
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;
ChCollisionSystem::Type coll_type = ChCollisionSystem::Type::BULLET;

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

    // Simulation parameters
    double gravity = 9.81;
    double time_step = contact_method == ChContactMethod::NSC ? 1e-3 : 1e-4;
    double render_fps = 100;

    // Create the system
    auto sys = ChSystem::Create(contact_method);

    sys->SetGravitationalAcceleration(ChVector3d(0, 0, -gravity));
    sys->SetCollisionSystemType(coll_type);

    // Change the default collision effective radius of curvature (SMC only)
    ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(1);

    // Create YAML parser object and load model file
    ChYamlParser parser;
    parser.SetVerbose(true);
    parser.Load(GetChronoDataFile(model_yaml_filename));
    instance1 = parser.Populate(*sys, frame1, prefix1);
    if (second_instance)
        instance2 = parser.Populate(*sys, frame2, prefix2);

    // Create the run-time visualization system
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("Ball drop demonstration");
            vis_irr->SetCameraVertical(CameraVerticalDir::Z);
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddTypicalLights();
            vis_irr->AddCamera(ChVector3d(2, -8, 0), ChVector3d(2, 0, 0));
            vis_irr->AttachSystem(sys.get());
            vis_irr->AddGrid(0.2, 0.2, 20, 20, ChCoordsys<>(ChVector3d(0, 0.11, 0), QuatFromAngleX(CH_PI_2)),
                             ChColor(0.1f, 0.1f, 0.1f));

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(sys.get());
            vis_vsg->SetWindowTitle("Ball drop demonstration");
            vis_vsg->AddCamera(ChVector3d(2, -8, 0), ChVector3d(2, 0, 0));
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowPosition(100, 100);
            vis_vsg->SetBackgroundColor(ChColor(0.8f, 0.85f, 0.9f));
            vis_vsg->EnableSkyBox();
            vis_vsg->SetCameraVertical(CameraVerticalDir::Z);
            vis_vsg->SetCameraAngleDeg(40.0);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(-CH_PI_4, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->AddGrid(0.2, 0.2, 20, 20, ChCoordsys<>(ChVector3d(0, 0.11, 0), QuatFromAngleX(CH_PI_2)),
                             ChColor(0.1f, 0.1f, 0.1f));
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Simulation loop
    ChRealtimeStepTimer rt_timer;
    double time = 0.0;
    int render_frame = 0;

    while (vis->Run()) {
        if (time >= render_frame / render_fps) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
            render_frame++;
        }

        sys->DoStepDynamics(time_step);
        rt_timer.Spin(time_step);
        time += time_step;
    }

    return 0;
}
