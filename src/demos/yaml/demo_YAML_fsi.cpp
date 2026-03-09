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
// Demo for FSI problems specified through YAML files.
//
// =============================================================================

////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include "chrono_parsers/yaml/ChParserFsiYAML.h"

#include "chrono/assets/ChVisualSystem.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtils.h"

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2025 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Select model
    std::string input;
    int model = 1;
    std::cout << "Options:\n";
    std::cout << "  1. Object drop (SPH) [DEFAULT]" << std::endl;
    std::cout << "  2. Baffle flow (SPH)" << std::endl;
    std::cout << "  3. Wave tank (SPH)" << std::endl;
    std::cout << "  4. Sphere decay (TDPF)" << std::endl;
    std::cout << "  5. Other (user-provided YAML file)" << std::endl;
    std::cout << "\nSelect model: ";
    std::getline(std::cin, input);
    if (!input.empty()) {
        std::istringstream stream(input);
        stream >> model;
        ChClampValue(model, 1, 5);
    }

    // Set input file name
    std::string yaml_filename;
    switch (model) {
        case 1:
            yaml_filename = GetChronoDataFile("yaml/fsi/objectdrop/fsi_objectdrop.yaml");
            break;
        case 2:
            yaml_filename = GetChronoDataFile("yaml/fsi/baffleflow/fsi_baffleflow.yaml");
            break;
        case 3:
            yaml_filename = GetChronoDataFile("yaml/fsi/wavetank/fsi_wavetank.yaml");
            break;
        case 4:
            yaml_filename = GetChronoDataFile("yaml/fsi/sphere_decay/fsi_sphere_decay.yaml");
            break;
        case 5:
            std::cout << "FSI YAML specification file name: ";
            std::getline(std::cin, yaml_filename);
            break;
    }

    std::cout << std::endl;
    std::cout << "YAML specification file: " << yaml_filename << std::endl;

    // Create the FSI YAML parser object
    parsers::ChParserFsiYAML parser(yaml_filename, true);

    // Create the FSI system and the underlying multibody and fluid systems
    parser.CreateFsiSystem();
    auto sysFSI = parser.GetFsiSystem();
    auto sysCFD = parser.GetFluidSystem();
    auto sysMBS = parser.GetMultibodySystem();

    // Extract information from parsed YAML files
    const std::string& model_name = parser.GetName();
    double time_end = parser.GetEndtime();
    double time_step = parser.GetTimestep();
    bool render = parser.Render();
    double render_fps = parser.GetRenderFPS();
    
    auto& parserMBS = parser.GetMbsParser();
    CameraVerticalDir camera_vertical = parser.GetCameraVerticalDir();
    const ChVector3d& camera_location = parser.GetCameraLocation();
    const ChVector3d& camera_target = parser.GetCameraTarget();
    bool output_MBS = parserMBS.Output();
    double output_fps_MBS = parser.GetOutputFPS();

    auto& parserCFD = parser.GetCfdParser();
    bool output_CFD = parserCFD.Output();
    double output_fps_CFD = parser.GetOutputFPS();

    // Create the run-time visualization system
    std::shared_ptr<ChVisualSystem> vis;
#if defined(CHRONO_VSG)
    if (render) {
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachSystem(sysMBS.get());
        visVSG->SetWindowTitle("YAML FSI model - " + model_name);
        visVSG->AddCamera(camera_location, camera_target);
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->SetBackgroundColor(ChColor(0.04f, 0.11f, 0.18f));
        visVSG->SetCameraVertical(camera_vertical);
        visVSG->SetCameraAngleDeg(40.0);
        visVSG->SetLightIntensity(1.0f);
        visVSG->SetLightDirection(-CH_PI_4, CH_PI_4);
        visVSG->EnableShadows(false);
        visVSG->SetAbsFrameScale(2.0);

        auto plugin = parserCFD.GetVisualizationPlugin();
        if (plugin)
            visVSG->AttachPlugin(plugin);

        visVSG->Initialize();
        vis = visVSG;
    }
#else
    render = false;
#endif

    // Create output directory
    if (output_MBS || output_CFD) {
        std::string out_dir = GetChronoOutputPath() + "YAML_FSI";
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
        out_dir = out_dir + "/" + model_name;
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
        parser.SetOutputDir(out_dir);
    }

    // Simulation loop
    double time = 0;
    int render_frame = 0;
    int output_frame_MBS = 0;
    int output_frame_CFD = 0;

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
        } else {
            std::cout << "\rt = " << time;
            if (time_end > 0 && time >= time_end)
                break;
        }

        if (output_MBS && time >= output_frame_MBS / output_fps_MBS) {
            parserMBS.SaveOutput(*sysMBS, output_frame_MBS);
            output_frame_MBS++;
        }

        if (output_CFD && time >= output_frame_CFD / output_fps_CFD) {
            parserCFD.SaveOutput(output_frame_CFD);
            output_frame_CFD++;
        }

        sysFSI->DoStepDynamics(time_step);
        time += time_step;
    }

    return 0;
}
