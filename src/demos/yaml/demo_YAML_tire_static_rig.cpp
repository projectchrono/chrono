// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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
// Demo for static tire test rig simulations specified through YAML files.
// The tire model to test is selected at run time.
// This demo reproduces demo_VEH_TireStaticTestRig.
//
// =============================================================================

#include <filesystem>
#include <iomanip>

#include "chrono/ChConfig.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_parsers/yaml/ChParserTireStaticRigYAML.h"

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif

using namespace chrono;

using std::cout;
using std::cin;
using std::endl;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << endl;

    // Available tests (YAML simulation specification files and descriptions)
    std::vector<std::pair<std::string, std::string>> tests = {
        {"tire_static_rig_hmmwv_rigid.yaml", "HMMWV rigid tire"},           //
        {"tire_static_rig_hmmwv_rigidmesh.yaml", "HMMWV rigid mesh tire"},  //
#ifdef CHRONO_FEA
    #ifdef CHRONO_PARDISO_MKL
        {"tire_static_rig_hmmwv_ancf4.yaml", "HMMWV ANCF 4-node shell tire"},  //
        {"tire_static_rig_hmmwv_ancf8.yaml", "HMMWV ANCF 8-node shell tire"},  //
        {"tire_static_rig_hmmwv_reissner.yaml", "HMMWV Reissner shell tire"},  //
    #endif
        {"tire_static_rig_hmmwv_mbtire.yaml", "HMMWV MB tire"},  //
#endif
        {"tire_static_rig_polaris_rigid.yaml", "Polaris rigid tire"},           //
        {"tire_static_rig_polaris_rigidmesh.yaml", "Polaris rigid mesh tire"},  //
#if defined(CHRONO_FEA) && defined(CHRONO_PARDISO_MKL)
        {"tire_static_rig_polaris_ancf4.yaml", "Polaris ANCF 4-node shell tire"},  //
#endif
    };

    // Select test
    int num_tests = (int)tests.size();
    int which = 0;
    cout << "Options:\n";
    for (int i = 0; i < num_tests; i++)
        cout << std::setw(2) << i + 1 << "  " << tests[i].second << endl;
    cout << "\nSelect tire: ";
    cin >> which;
    cout << endl;
    ChClampValue(which, 1, num_tests);

    std::string yaml_filename = GetChronoDataFile("yaml/vehicle/tire_static_rig/" + tests[which - 1].first);
    cout << "Simulation YAML file: " << yaml_filename << endl;

    // Create the YAML parser object
    parsers::ChParserTireStaticRigYAML parser(yaml_filename, true);

    // Create Chrono system and the static tire test rig
    auto sys = parser.CreateSystem();
    auto rig = parser.CreateRig(*sys);

    double time_end = parser.GetEndtime();
    double step = parser.GetTimestep();
    bool real_time = parser.EnforceRealtime();
    bool render = parser.VisualizationEnabled();
    bool output = parser.OutputEnabled();

    // Create output directory (named after the simulation specification file)
    if (output) {
        std::string out_dir = GetChronoOutputPath() + "YAML_TIRE_STATIC_RIG";
        if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
            cout << "Error creating directory " << out_dir << endl;
            return 1;
        }
        out_dir += "/" + std::filesystem::path(yaml_filename).stem().string();
        if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
            cout << "Error creating directory " << out_dir << endl;
            return 1;
        }
        parser.SetOutputDir(out_dir);
        cout << "Output directory: " << out_dir << endl;
    }

    const ChVector3d& camera_offset = parser.GetCameraOffset();
    const ChVector3d& camera_target_offset = parser.GetCameraTargetOffset();

    // Create the run-time visualization (camera follows the wheel)
#ifdef CHRONO_VSG
    std::shared_ptr<vsg3d::ChVisualSystemVSG> vis;
    if (render) {
        vis = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        vis->AttachSystem(sys.get());
        vis->SetWindowTitle("Static tire test rig YAML demo - " + tests[which - 1].second);
        vis->SetWindowSize(1200, 600);
        vis->SetWindowPosition(100, 100);
        vis->SetCameraVertical(CameraVerticalDir::Z);
        auto loc = rig->GetWheelPos();
        vis->AddCamera(loc + camera_offset, loc + camera_target_offset);
        vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
        vis->EnableShadows(parser.EnableShadows());
        vis->Initialize();
    }
#else
    if (render)
        cout << "Chrono::VSG run-time visualization module not enabled. Disabling visualization." << endl;
    render = false;
#endif

    // Simulation loop (until the end time or the end of the test)
    ChRealtimeStepTimer rt_timer;
    double time = 0;

    while (time_end < 0 || time < time_end) {
#ifdef CHRONO_VSG
        if (render) {
            auto loc = rig->GetWheelPos();
            vis->UpdateCamera(loc + camera_offset, loc + camera_target_offset);
            if (!parser.Render(*vis, time))
                break;
        }
#endif

        if (!rig->Advance(step))
            break;
        time = sys->GetChTime();

        if (output)
            parser.Output(time);

        // Report the load for the current test (once measurements are enabled)
        if (rig->IsMeasuring())
            cout << std::fixed << std::setprecision(3) << "\rt = " << time << "  load = " << std::setw(10) << rig->GetLoad() << std::flush;

        if (real_time)
            rt_timer.Spin(step);
    }
    cout << "\nSimulated time: " << time << endl;

    // Generate plots (if requested)
    if (output)
        parser.GeneratePlots();

    return 0;
}
