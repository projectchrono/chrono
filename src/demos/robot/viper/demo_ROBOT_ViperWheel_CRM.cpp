// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Huzaifa Unjhawala
// =============================================================================
//
// Single-wheel tire test rig with a Viper wheel on CRM terrain.
//
// =============================================================================

////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include <algorithm>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"

#include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "demos/SetChronoSolver.h"
#include "viper_wheel.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

double render_fps = 100;
bool gnuplot_output = true;

bool render = true;

// -----------------------------------------------------------------------------

int main() {
    // --------------------------------
    // Create wheel and tire subsystems
    // --------------------------------

    auto wheel = chrono_types::make_shared<DummyViperWheel>();
    auto tire = chrono_types::make_shared<ViperTire>();
    tire->SetGrouserHeight(0.02);
    tire->SetGrouserWidth(0.01);

    // -------------------------------------------------
    // Create system and set solver and integrator types
    // -------------------------------------------------

    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    double step_size = 2e-4;
    ChSolver::Type solver_type = ChSolver::Type::BARZILAIBORWEIN;
    ChTimestepper::Type integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    SetChronoSolver(sys, solver_type, integrator_type);

    // -----------------------------
    // Create and configure test rig
    // -----------------------------

    ChTireTestRig rig(wheel, tire, &sys);

    rig.SetGravitationalAcceleration(9.8);
    rig.SetNormalLoad(1000);

    rig.SetTireStepsize(step_size);
    rig.SetTireVisualizationType(VisualizationType::COLLISION);

    // Set CRM terrain
    ChTireTestRig::TerrainPatchSize size;
    size.length = 10;
    size.width = 1;
    size.depth = 0.2;

    ChTireTestRig::TerrainParamsCRM params;
    params.sph_params.initial_spacing = 0.01;
    params.mat_props.density = 1700;
    params.mat_props.Young_modulus = 1e6;
    params.mat_props.cohesion_coeff = 0;
    params.mat_props.mu_I0 = 0.04;
    params.mat_props.mu_fric_2 = 0.8;
    params.mat_props.mu_fric_s = 0.8;
    params.mat_props.average_diam = 0.005;

    rig.SetTerrainCRM(size, params);

    // Register custom callback for wheel BCE marker generation
    auto bce_callback = chrono_types::make_shared<ViperTireBCE>(tire, params.sph_params.initial_spacing);
    rig.RegisterWheelBCECreationCallback(bce_callback);

    // -----------------
    // Set test scenario
    // -----------------

    // Scenario: driven wheel
    ////rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionConst>(10.0));
    ////rig.Initialize();

    // Scenario: pulled wheel
    ////rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionConst>(1.0));
    ////rig.Initialize();

    // Scenario: prescribe all motion functions
    //   longitudinal speed: 0.2 m/s
    //   angular speed: 10 RPM
    rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0.2));
    rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionConst>(10 * CH_RPM_TO_RAD_S));

    // Set delay before applying inputs (settling time)
    double input_time_delay = 1.0;
    rig.SetTimeDelay(input_time_delay);

    // Initialize the tire test rig
    ////rig.Initialize(ChTireTestRig::Mode::SUSPEND);
    ////rig.Initialize(ChTireTestRig::Mode::DROP);
    rig.Initialize(ChTireTestRig::Mode::TEST);

    cout << "Rig normal load: " << rig.GetNormalLoad() << endl;
    cout << "Rig total mass:  " << rig.GetMass() << endl;

    // -----------------
    // Initialize output
    // -----------------

    const std::string out_dir = GetChronoOutputPath() + "VIPER_WHEEL_CRM";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    // ---------------------------------
    // Create the run-time visualization
    // ---------------------------------

    std::shared_ptr<ChVisualSystem> vis;
    if (render) {
        // FSI plugin
        auto sysFSI = std::dynamic_pointer_cast<CRMTerrain>(rig.GetTerrain())->GetFsiSystemSPH();
        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(sysFSI.get());
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableRigidBodyMarkers(true);

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sys);
        visVSG->SetWindowTitle("Viper wheel on CRM deformable terrain");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(ChVector3d(1.0, 2.5, 1.0), ChVector3d(0, 1, 0));
        visVSG->SetLightIntensity(0.9f);
        visVSG->SetLightDirection(CH_PI_2, CH_PI / 6);

        visVSG->Initialize();
        vis = visVSG;
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Timers and counters
    ChTimer timer;         // timer for measuring total run time
    double time = 0;       // simulated time
    double sim_time = 0;   // simulation time
    int render_frame = 0;  // render frame counter
    double sim_time_max = 10;

    // Data collection
    ChFunctionInterp long_slip_fct;
    ChFunctionInterp slip_angle_fct;
    ChFunctionInterp camber_angle_fct;

    timer.start();
    while (time < sim_time_max) {
        time = sys.GetChTime();

        if (time >= render_frame / render_fps) {
            auto& loc = rig.GetPos();
            vis->UpdateCamera(loc + ChVector3d(1.0, 2.5, 0.5), loc + ChVector3d(0, 0.25, -0.25));

            if (!vis->Run())
                break;
            vis->Render();
            render_frame++;
        }

        rig.Advance(step_size);
        sim_time += sys.GetTimerStep();

        auto long_slip = rig.GetLongitudinalSlip();
        auto slip_angle = rig.GetSlipAngle() * CH_RAD_TO_DEG;
        auto camber_angle = rig.GetCamberAngle() * CH_RAD_TO_DEG;

        if (gnuplot_output && time > input_time_delay) {
            long_slip_fct.AddPoint(time, long_slip);
            slip_angle_fct.AddPoint(time, slip_angle);
            camber_angle_fct.AddPoint(time, camber_angle);
        }

        cout << "\rRTF: " << sys.GetRTF();
    }
    timer.stop();

    double step_time = timer();
    cout << "\rSimulated time: " << time << endl;
    cout << "Run time (simulation): " << sim_time << "  |  RTF: " << sim_time / time << endl;
    cout << "Run time (total):      " << step_time << "  |  RTF: " << step_time / time << endl;

#ifdef CHRONO_POSTPROCESS
    // ------------
    // Plot results
    // ------------

    if (gnuplot_output && sys.GetChTime() > input_time_delay) {
        postprocess::ChGnuPlot gplot_long_slip(out_dir + "/tmp1.gpl");
        gplot_long_slip.SetGrid();
        gplot_long_slip.SetLabelX("time (s)");
        gplot_long_slip.SetLabelY("Long. slip");
        gplot_long_slip.SetRangeY(-2, +2);
        gplot_long_slip.Plot(long_slip_fct, "", " with lines lt -1 lc rgb'#00AAEE' ");

        postprocess::ChGnuPlot gplot_slip_angle(out_dir + "/tmp2.gpl");
        gplot_slip_angle.SetGrid();
        gplot_slip_angle.SetLabelX("time (s)");
        gplot_slip_angle.SetLabelY("Slip angle (deg)");
        gplot_slip_angle.SetRangeY(-25, +25);
        gplot_slip_angle.Plot(slip_angle_fct, "", " with lines lt -1 lc rgb'#00AAEE' ");

        postprocess::ChGnuPlot gplot_camber_angle(out_dir + "/tmp3.gpl");
        gplot_camber_angle.SetGrid();
        gplot_camber_angle.SetLabelX("time (s)");
        gplot_camber_angle.SetLabelY("Camber angle (deg)");
        gplot_camber_angle.SetRangeY(-5, +5);
        gplot_camber_angle.Plot(camber_angle_fct, "", " with lines lt -1 lc rgb'#00AAEE' ");
    }
#endif

    return 0;
}
