// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Single-wheel test rig using Chrono::Vehicle suspension, wheel and tire.
// This demo only works with handling tire models and rigid terrain.
// See demo_VEH_WheelTestRig for additional options.
//
// =============================================================================

////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include <algorithm>

#include "chrono/physics/ChSystemNSC.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChWheelTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "demos/SetChronoSolver.h"

using namespace chrono;
using namespace chrono::vehicle;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Tire specification file
std::string tire_json = "hmmwv/tire/HMMWV_TMeasyTire.json";
////std::string tire_json = "hmmwv/tire/HMMWV_FialaTire.json";
////std::string tire_json = "hmmwv/tire/HMMWV_Pac89Tire.json";
////std::string tire_json = "hmmwv/tire/HMMWV_Pac02Tire.json";
////std::string tire_json = "Polaris/Polaris_TMeasyTire.json";

// Wheel specification file
std::string wheel_json = "hmmwv/wheel/HMMWV_Wheel.json";
////std::string wheel_json = "Polaris/Polaris_Wheel.json";

// Suspension specification file
std::string suspension_json = "hmmwv/suspension/HMMWV_DoubleWishboneRear.json";
////std::string suspension_json = "Polaris/Polaris_Rear_TrailingArm.json";

double render_fps = 60;
bool debug_output = false;
bool gnuplot_output = true;

// -----------------------------------------------------------------------------

int main() {
    // Create vehicle subsystems
    auto wheel = ReadWheelJSON(GetVehicleDataFile(wheel_json));
    auto tire = ReadTireJSON(GetVehicleDataFile(tire_json));
    auto suspension = ReadSuspensionJSON(GetVehicleDataFile(suspension_json));

    if (std::dynamic_pointer_cast<ChForceElementTire>(tire) == nullptr) {
        cerr << "ERROR: This demo only supports handling tire models." << endl;
        return 1;
    }

    tire->SetCollisionType(ChTire::CollisionType::FOUR_POINTS);

    // Create system and set default solver and integrator types
    ChSystemNSC sys;
    double step_size = 2e-4;
    ChSolver::Type solver_type = ChSolver::Type::BARZILAIBORWEIN;
    ChTimestepper::Type integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;

    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    int num_threads_chrono = std::min(8, ChOMP::GetNumProcs());
    int num_threads_collision = 1;
    int num_threads_eigen = 1;
    int num_threads_pardiso = std::min(8, ChOMP::GetNumProcs());
    sys.SetNumThreads(num_threads_chrono, num_threads_collision, num_threads_eigen);

    SetChronoSolver(sys, solver_type, integrator_type, num_threads_pardiso);

    // Create and configure test rig
    ChWheelSuspensionTestRig rig(sys, wheel, tire, suspension);

    rig.SetGravitationalAcceleration(9.8);
    rig.SetNormalLoad(3000);

    rig.SetStepsize(step_size);
    rig.SetVisualizationType(VisualizationType::MESH);

    rig.SetTerrainRigid(ChWheelTestRig::TerrainPatchSize(10, 1), 0.8f, 0.0f, 2e7f);

    // Set test scenario: prescribe all motion functions
    //   longitudinal speed: 0.2 m/s
    //   angular speed: 10 RPM
    rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0.2));
    rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionConst>(10 * CH_RPM_TO_RAD_S));

    // Set delay before applying inputs (settling time)
    double input_time_delay = 1.0;
    rig.SetTimeDelay(input_time_delay);

    // Initialize the wheel test rig; in TEST mode set a drop speed of 0.05
    ////rig.Initialize(ChWheelTestRig::Mode::SUSPEND);
    ////rig.Initialize(ChWheelTestRig::Mode::DROP);
    rig.Initialize(ChWheelTestRig::Mode::TEST, 0.05);

    // Initialize output
    const std::string out_dir = GetChronoOutputPath() + "TIRE_TEST_RIG";
    if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    // Create the run-time visualization
    auto vis = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowSize(1280, 800);
    vis->SetWindowTitle("Tire Test Rig");
    vis->AddCamera(ChVector3d(1.0, 2.5, 1.0));
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->EnableShadows();
    vis->Initialize();

    // ---------------
    // Simulation loop
    // ---------------

    // Timers and counters
    ChTimer timer;         // timer for measuring total run time
    double time = 0;       // simulated time
    double sim_time = 0;   // simulation time
    int render_frame = 0;  // render frame counter

    // Data collection
    ChFunctionInterp long_slip_fct;
    ChFunctionInterp slip_angle_fct;
    ChFunctionInterp camber_angle_fct;

    timer.start();
    while (vis->Run()) {
        time = sys.GetChTime();

        if (time >= render_frame / render_fps) {
            auto loc = rig.GetWheelPos();
            vis->UpdateCamera(loc + ChVector3d(2.0, 2.5, 1.0), loc + ChVector3d(0, 0.25, -0.25));
            vis->Render();
        }

        rig.Advance(step_size);
        sim_time += sys.GetTimerStep();

        auto long_slip = rig.GetLongitudinalSlip();
        auto slip_angle = rig.GetSlipAngle() * CH_RAD_TO_DEG;
        auto camber_angle = rig.GetCamberAngle() * CH_RAD_TO_DEG;

        if (gnuplot_output && rig.OutputEnabled()) {
            long_slip_fct.AddPoint(time, long_slip);
            slip_angle_fct.AddPoint(time, slip_angle);
            camber_angle_fct.AddPoint(time, camber_angle);
        }

        if (debug_output && rig.OutputEnabled()) {
            cout << time << endl;
            cout << "   " << long_slip << " " << slip_angle << " " << camber_angle << endl;
            auto tforce = rig.ReportWheelForce();
            auto frc = tforce.force;
            auto pnt = tforce.point;
            auto trq = tforce.moment;
            cout << "   " << frc.x() << " " << frc.y() << " " << frc.z() << endl;
            cout << "   " << pnt.x() << " " << pnt.y() << " " << pnt.z() << endl;
            cout << "   " << trq.x() << " " << trq.y() << " " << trq.z() << endl;
        } else {
            cout << "\rRTF: " << sys.GetRTF();
        }
    }
    timer.stop();

    double total_time = timer();
    cout << "\rSimulated time: " << time << endl;
    cout << "Run time (simulation): " << sim_time << "  |  RTF: " << sim_time / time << endl;
    cout << "Run time (total):      " << total_time << "  |  RTF: " << total_time / time << endl;

#ifdef CHRONO_POSTPROCESS
    if (gnuplot_output && rig.OutputEnabled()) {
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
