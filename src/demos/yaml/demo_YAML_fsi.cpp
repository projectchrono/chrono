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
// Demo for FSI SPH problems specified through YAML files.
//
// =============================================================================

////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include "chrono_parsers/yaml/ChParserFsiYAML.h"

#include "chrono/assets/ChVisualSystem.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2025 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Extract filenames from command-line arguments
    std::string fsi_yaml_filename = GetChronoDataFile("yaml/fsi/objectdrop/objectdrop.yaml");
    ////std::string fsi_yaml_filename = GetChronoDataFile("yaml/fsi/baffleflow/baffleflow.yaml");
    ////std::string fsi_yaml_filename = GetChronoDataFile("yaml/fsi/wavetank/wavetank.yaml");

    ChCLI cli(argv[0], "");
    cli.AddOption<std::string>("", "f,fsi_file", "FSI problem specification YAML file", fsi_yaml_filename);

    if (!cli.Parse(argc, argv, true))
        return 1;

    if (argc == 1) {
        cli.Help();
        std::cout << "Using default YAML specification file" << std::endl;
    }

    std::cout << std::endl;
    std::cout << "YAML specification file: " << fsi_yaml_filename << std::endl;

    // Create the FSI YAML parser object
    parsers::ChParserFsiYAML parser(fsi_yaml_filename, true);

    // Create the FSI system and the underlying multibody and fluid systems
    parser.CreateFsiSystem();
    auto sysFSI = parser.GetFsiSystem();
    auto sysCFD = parser.GetFluidSystem();
    auto sysMBS = parser.GetMultibodySystem();
    ////auto sysCFD_type = parser.GetFluidSystemType();

    // Extract information from parsed YAML files
    const std::string& model_name = parser.GetName();
    double time_end = parser.GetEndtime();
    double time_step = parser.GetTimestep();
    bool render = parser.Render();
    double render_fps = parser.GetRenderFPS();
    bool output = parser.Output();
    double output_fps = parser.GetOutputFPS();

    auto& parserMBS = parser.GetMbsParser();
    CameraVerticalDir camera_vertical = parserMBS.GetCameraVerticalDir();
    const ChVector3d& camera_location = parserMBS.GetCameraLocation();
    const ChVector3d& camera_target = parserMBS.GetCameraTarget();
    bool enable_shadows = parserMBS.EnableShadows();
    bool outputMBS = parserMBS.Output();

    auto& parserCFD = parser.GetCfdParser();
    bool outputCFD = parserCFD.Output();

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
        visVSG->SetBackgroundColor(ChColor(0.4f, 0.45f, 0.55f));
        visVSG->SetCameraVertical(camera_vertical);
        visVSG->SetCameraAngleDeg(40.0);
        visVSG->SetLightIntensity(1.0f);
        visVSG->SetLightDirection(-CH_PI_4, CH_PI_4);
        visVSG->EnableShadows(enable_shadows);
        ////visVSG->ToggleAbsFrameVisibility();
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
    if (output) {
        std::string out_dir = GetChronoOutputPath() + "YAML_FSI";
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
        std::string out_dirMBS = out_dir + "/MBS"; 
        if (!filesystem::create_directory(filesystem::path(out_dirMBS))) {
            std::cout << "Error creating directory " << out_dirMBS << std::endl;
            return 1;
        }
        std::string out_dirCFD = out_dir + "/CFD";
        if (!filesystem::create_directory(filesystem::path(out_dirCFD))) {
            std::cout << "Error creating directory " << out_dirCFD << std::endl;
            return 1;
        }
        
        parserMBS.SetOutputDir(out_dirMBS);
        parserCFD.SetOutputDir(out_dirCFD);
    }

    // Simulation loop
    double time = 0;
    int render_frame = 0;
    int output_frame = 0;

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

        if (output && time >= output_frame / output_fps) {
            if (outputMBS)
                parserMBS.SaveOutput(*sysMBS, output_frame);
            if (outputCFD)
                parserCFD.SaveOutput(output_frame);
            output_frame++;
        }

        sysFSI->DoStepDynamics(time_step);
        time += time_step;
    }

    return 0;
}
