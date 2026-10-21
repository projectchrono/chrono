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
// Demo for single-wheel test rig simulations specified through YAML files.
// The test to run is selected at run time.
// The Viper rover wheel test illustrates the use of a custom wheel assembly (a wheel assembly not built from
// Chrono::Vehicle subsystems), which the application provides to the YAML parser through a registered factory.
//
// =============================================================================

#include <filesystem>
#include <iomanip>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_parsers/yaml/ChParserWheelRigYAML.h"

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif

#include "demos/robot/viper/viper_wheel.h"

using namespace chrono;

using std::cout;
using std::cin;
using std::endl;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << endl;

    // Available tests (YAML simulation specification files and descriptions)
    std::vector<std::pair<std::string, std::string>> tests = {
        {"wheel_rig_rigid.yaml", "Wheel on rigid terrain"},
        {"wheel_suspension_rig_rigid.yaml", "Wheel with steerable suspension on rigid terrain"},
        {"wheel_rig_scm.yaml", "Wheel on SCM terrain"},
#ifdef CHRONO_CRM
        {"wheel_rig_crm.yaml", "Wheel on CRM terrain"},
        {"wheel_suspension_rig_crm.yaml", "Wheel with steerable suspension on CRM terrain"},
        {"viper_wheel_rig_crm.yaml", "Viper rover wheel (custom wheel assembly) on CRM terrain"},
#endif
    };

    // Select test
    int num_tests = (int)tests.size();
    int which = 0;
    cout << "Options:\n";
    for (int i = 0; i < num_tests; i++)
        cout << std::setw(2) << i + 1 << "  " << tests[i].second << endl;
    cout << "\nSelect test: ";
    cin >> which;
    cout << endl;
    ChClampValue(which, 1, num_tests);

    std::string yaml_filename = GetChronoDataFile("yaml/vehicle/wheel_rig/" + tests[which - 1].first);
    cout << "Simulation YAML file: " << yaml_filename << endl;

    // Create the YAML parser object
    parsers::ChParserWheelRigYAML parser(yaml_filename, true);

    // Register the factory for the custom Viper wheel assembly (used by a rig model of CUSTOM type with assembly
    // name "Viper_wheel"); the factory receives the "parameters" node of the rig model specification
    parser.RegisterWheelAssembly("Viper_wheel", [](ChSystem& sys, const YAML::Node& params) {
        auto wheel = chrono_types::make_shared<ViperRigWheel>(sys);
        if (params["radius"])
            wheel->SetRadius(params["radius"].as<double>());
        if (params["width"])
            wheel->SetWidth(params["width"].as<double>());
        if (params["grouser_height"])
            wheel->SetGrouserHeight(params["grouser_height"].as<double>());
        if (params["grouser_width"])
            wheel->SetGrouserWidth(params["grouser_width"].as<double>());
        if (params["num_grousers"])
            wheel->SetNumGrousers(params["num_grousers"].as<int>());
        return wheel;
    });

    // Create Chrono system and the wheel test rig
    auto sys = parser.CreateSystem();
    auto rig = parser.CreateRig(*sys);

    double time_end = parser.GetEndtime();
    double step = parser.GetRigStep();
    bool real_time = parser.EnforceRealtime();
    bool render = parser.VisualizationEnabled();
    bool output = parser.OutputEnabled();

    // Create output directory (named after the simulation specification file)
    if (output) {
        std::string out_dir = GetChronoOutputPath() + "YAML_WHEEL_RIG";
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

    // Create the run-time visualization (camera follows the wheel hub)
#ifdef CHRONO_VSG
    std::shared_ptr<vsg3d::ChVisualSystemVSG> vis;
    if (render) {
        vis = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        if (auto plugin = parser.GetVisualizationPlugin())
            vis->AttachPlugin(plugin);
        vis->AttachSystem(sys.get());
        vis->SetWindowTitle("Wheel test rig YAML demo - " + tests[which - 1].second);
        vis->SetWindowSize(1280, 800);
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

    // Simulation loop
    ChRealtimeStepTimer rt_timer;
    double time = 0;
    int status_frame = 0;
    double status_fps = 10;

    while (time_end < 0 || time < time_end) {
#ifdef CHRONO_VSG
        if (render) {
            auto loc = rig->GetWheelPos();
            vis->UpdateCamera(loc + camera_offset, loc + camera_target_offset);
            if (!parser.Render(*vis, time))
                break;
        }
#endif

        rig->Advance(step);
        time = sys->GetChTime();

        if (output)
            parser.Output(time);

        // Report measured quantities (once measurements are enabled)
        if (rig->OutputEnabled() && time >= status_frame / status_fps) {
            cout << std::fixed << std::setprecision(3) << "\rt = " << time                    //
                 << "  long. slip = " << std::setw(7) << rig->GetLongitudinalSlip()           //
                 << "  slip angle = " << std::setw(7) << rig->GetSlipAngle() * CH_RAD_TO_DEG  //
                 << "  camber = " << std::setw(7) << rig->GetCamberAngle() * CH_RAD_TO_DEG    //
                 << "  steering = " << std::setw(7) << rig->GetSteeringAngle() * CH_RAD_TO_DEG << "  Fz = " << std::setw(9) << rig->ReportWheelForce().force.z() << std::flush;
            status_frame = (int)(time * status_fps) + 1;
        }

        if (real_time)
            rt_timer.Spin(step);
    }
    cout << endl;

    return 0;
}
