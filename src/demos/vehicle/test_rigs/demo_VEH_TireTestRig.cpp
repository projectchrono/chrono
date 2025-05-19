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
// Demonstration of the single-wheel tire test rig.
//
// =============================================================================

////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include <algorithm>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/assets/ChVisualShapeFEA.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif
#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
    #include "chrono_postprocess/ChBlender.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "demos/SetChronoSolver.h"

using namespace chrono;
using namespace chrono::vehicle;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Terrain type (RIGID or SCM)
enum class TerrainType { RIGID, SCM };
TerrainType terrain_type = TerrainType::RIGID;

// Tire specification file
////std::string tire_json = "hmmwv/tire/HMMWV_RigidTire.json";
////std::string tire_json = "hmmwv/tire/HMMWV_TMeasyTire.json";
////std::string tire_json = "hmmwv/tire/HMMWV_FialaTire.json";
////std::string tire_json = "hmmwv/tire/HMMWV_Pac89Tire.json";
////std::string tire_json = "hmmwv/tire/HMMWV_Pac02Tire.json";
////std::string tire_json = "hmmwv/tire/HMMWV_ANCF4Tire_Lumped.json";
////std::string tire_json = "hmmwv/tire/HMMWV_ANCF8Tire_Lumped.json";
////std::string tire_json = "hmmwv/tire/HMMWV_ReissnerTire.json";
std::string tire_json = "Polaris/Polaris_TMeasyTire.json";
////std::string tire_json = "Polaris/Polaris_RigidTire.json";
////std::string tire_json = "Polaris/Polaris_RigidMeshTire.json";
////std::string tire_json = "Polaris/Polaris_ANCF4Tire_Lumped.json";

// Wheel specification file
////std::string wheel_json = "hmmwv/wheel/HMMWV_Wheel.json";
std::string wheel_json = "Polaris/Polaris_Wheel.json";

double render_fps = 120;
bool debug_output = false;
bool gnuplot_output = true;
bool blender_output = false;

// -----------------------------------------------------------------------------

int main() {
    // --------------------------------
    // Create wheel and tire subsystems
    // --------------------------------

    auto wheel = ReadWheelJSON(vehicle::GetDataFile(wheel_json));
    auto tire = ReadTireJSON(vehicle::GetDataFile(tire_json));

    bool handling_tire = std::dynamic_pointer_cast<ChForceElementTire>(tire) != nullptr;
    bool fea_tire = std::dynamic_pointer_cast<ChDeformableTire>(tire) != nullptr;

    if (handling_tire && terrain_type == TerrainType::SCM) {
        cerr << "ERROR: Handling tire models cannot be used with SCM terrain." << endl;
        return 1;
    }

    // Set tire contact surface (relevant for FEA tires only)
    if (fea_tire) {
        int collision_family = 7;
        auto surface_type = ChTire::ContactSurfaceType::NODE_CLOUD;
        double surface_dim = 0.02;
        if (terrain_type == TerrainType::SCM) {
            surface_type = ChTire::ContactSurfaceType::TRIANGLE_MESH;
            surface_dim = 0;
        }
        tire->SetContactSurfaceType(surface_type, surface_dim, collision_family);
    }

    // ---------------------------------------------------------
    // Create system and set default solver and integrator types
    // ---------------------------------------------------------

    ChSystem* sys = nullptr;
    double step_size = 0;
    ChSolver::Type solver_type;
    ChTimestepper::Type integrator_type;

    if (fea_tire) {
        sys = new ChSystemSMC;
        step_size = 5e-5;
        solver_type = ChSolver::Type::PARDISO_MKL;
        integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    } else {
        sys = new ChSystemNSC;
        step_size = 2e-4;
        solver_type = ChSolver::Type::BARZILAIBORWEIN;
        integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    }

    // Set collision system
    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Number of OpenMP threads used in Chrono (SCM ray-casting and FEA)
    int num_threads_chrono = std::min(8, ChOMP::GetNumProcs());

    // Number of threads used in collision detection
    int num_threads_collision = 1;

    // Number of threads used by Eigen
    int num_threads_eigen = 1;

    // Number of threads used by PardisoMKL
    int num_threads_pardiso = std::min(8, ChOMP::GetNumProcs());

    sys->SetNumThreads(num_threads_chrono, num_threads_collision, num_threads_eigen);
    SetChronoSolver(*sys, solver_type, integrator_type, num_threads_pardiso);
    tire->SetStepsize(step_size);

    // -----------------------------
    // Create and configure test rig
    // -----------------------------

    ChTireTestRig rig(wheel, tire, sys);

    rig.SetGravitationalAcceleration(9.8);
    rig.SetNormalLoad(3000);

    ////rig.SetCamberAngle(+15 * CH_DEG_TO_RAD);

    rig.SetTireStepsize(step_size);
    rig.SetTireCollisionType(ChTire::CollisionType::FOUR_POINTS);
    rig.SetTireVisualizationType(VisualizationType::MESH);

    if (terrain_type == TerrainType::RIGID) {
        ChTireTestRig::TerrainParamsRigid params;
        params.friction = 0.8f;
        params.restitution = 0;
        params.Young_modulus = 2e7f;
        params.length = 10;
        params.width = 1;

        rig.SetTerrainRigid(params);
    } else {
        ChTireTestRig::TerrainParamsSCM params;
        params.Bekker_Kphi = 2e6;
        params.Bekker_Kc = 0;
        params.Bekker_n = 1.1;
        params.Mohr_cohesion = 0;
        params.Mohr_friction = 30;
        params.Janosi_shear = 0.01;
        params.length = 10;
        params.width = 1;
        params.grid_spacing = 0.05;

        rig.SetTerrainSCM(params);
    }

    // -----------------
    // Set test scenario
    // -----------------

    // Scenario: driven wheel
    ////rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionConst>(10.0));
    ////rig.Initialize();

    // Scenario: pulled wheel
    ////rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionConst>(1.0));
    ////rig.Initialize();

    // Scenario: imobilized wheel
    ////rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0.0));
    ////rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0.0));
    ////rig.Initialize();

    // Scenario: prescribe all motion functions
    //   longitudinal speed: 0.2 m/s
    //   angular speed: 10 RPM
    //   slip angle: sinusoidal +- 5 deg with 5 s period
    rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0.2));
    rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionConst>(10 * CH_RPM_TO_RAD_S));
    rig.SetSlipAngleFunction(chrono_types::make_shared<ChFunctionSine>(5 * CH_DEG_TO_RAD, 0.2));

    // Scenario: specified longitudinal slip (overrrides other definitons of motion functions)
    ////rig.SetConstantLongitudinalSlip(0.2, 0.1);

    // Initialize the tire test rig
    rig.SetTimeDelay(1.0);
    ////rig.Initialize(ChTireTestRig::Mode::SUSPEND);
    ////rig.Initialize(ChTireTestRig::Mode::DROP);
    rig.Initialize(ChTireTestRig::Mode::TEST);

    // Optionally, modify tire visualization (can be done only after initialization)
    if (auto tire_def = std::dynamic_pointer_cast<ChDeformableTire>(tire)) {
        auto visFEA = chrono_types::make_shared<ChVisualShapeFEA>();
        visFEA->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
        visFEA->SetShellResolution(3);
        visFEA->SetWireframe(false);
        visFEA->SetColormapRange(0.0, 5.0);
        visFEA->SetSmoothFaces(true);
        tire_def->AddVisualShapeFEA(visFEA);
    }

    // -----------------
    // Initialize output
    // -----------------

    const std::string out_dir = GetChronoOutputPath() + "TIRE_TEST_RIG";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    // ---------------------------------
    // Create the run-time visualization
    // ---------------------------------

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
            vis_irr->AttachSystem(sys);
            vis_irr->SetCameraVertical(CameraVerticalDir::Z);
            vis_irr->SetWindowSize(1200, 600);
            vis_irr->SetWindowTitle("Tire Test Rig");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector3d(1.0, 2.5, 1.0));
            vis_irr->AddLightDirectional();

            vis_irr->GetActiveCamera()->setFOV(irr::core::PI / 4.5f);

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(sys);
            vis_vsg->SetCameraVertical(CameraVerticalDir::Z);
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowTitle("Tire Test Rig");
            vis_vsg->AddCamera(ChVector3d(1.0, 2.5, 1.0));
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

#ifdef CHRONO_POSTPROCESS
    // ---------------------------
    // Create the Blender exporter
    // ---------------------------

    postprocess::ChBlender blender_exporter(sys);

    if (blender_output) {
        std::string blender_dir = out_dir + "/blender";
        if (!filesystem::create_directory(filesystem::path(blender_dir))) {
            cerr << "Error creating directory " << blender_dir << endl;
            return 1;
        }

        blender_exporter.SetBlenderUp_is_ChronoZ();
        blender_exporter.SetBasePath(blender_dir);
        blender_exporter.AddAll();
        blender_exporter.SetCamera(ChVector3d(3, 3, 1), ChVector3d(0, 0, 0), 50);
        blender_exporter.ExportScript();
    }
#endif

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

    double time_offset = 0.5;

    timer.start();
    while (vis->Run()) {
        time = sys->GetChTime();

        if ((debug_output || gnuplot_output) && time >= time_offset) {
            long_slip_fct.AddPoint(time, tire->GetLongitudinalSlip());
            slip_angle_fct.AddPoint(time, tire->GetSlipAngle() * CH_RAD_TO_DEG);
            camber_angle_fct.AddPoint(time, tire->GetCamberAngle() * CH_RAD_TO_DEG);
        }

        if (time >= render_frame / render_fps) {
            auto& loc = rig.GetPos();
            vis->UpdateCamera(loc + ChVector3d(1.0, 2.5, 0.5), loc + ChVector3d(0, 0.25, -0.25));

            vis->BeginScene();
            vis->Render();
            vis->EndScene();

#ifdef CHRONO_POSTPROCESS
            if (blender_output)
                blender_exporter.ExportData();
#endif
        }

        rig.Advance(step_size);
        sim_time += sys->GetTimerStep();

        if (debug_output) {
            cout << time << endl;
            auto long_slip = tire->GetLongitudinalSlip();
            auto slip_angle = tire->GetSlipAngle();
            auto camber_angle = tire->GetCamberAngle();
            cout << "   " << long_slip << " " << slip_angle << " " << camber_angle << endl;
            auto tforce = rig.ReportTireForce();
            auto frc = tforce.force;
            auto pnt = tforce.point;
            auto trq = tforce.moment;
            cout << "   " << frc.x() << " " << frc.y() << " " << frc.z() << endl;
            cout << "   " << pnt.x() << " " << pnt.y() << " " << pnt.z() << endl;
            cout << "   " << trq.x() << " " << trq.y() << " " << trq.z() << endl;
        } else {
            cout << "\rRTF: " << sys->GetRTF();
        }
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

    if (gnuplot_output && sys->GetChTime() > time_offset) {
        postprocess::ChGnuPlot gplot_long_slip(out_dir + "/tmp1.gpl");
        gplot_long_slip.SetGrid();
        gplot_long_slip.SetLabelX("time (s)");
        gplot_long_slip.SetLabelY("Long. slip");
        gplot_long_slip.Plot(long_slip_fct, "", " with lines lt -1 lc rgb'#00AAEE' ");

        postprocess::ChGnuPlot gplot_slip_angle(out_dir + "/tmp2.gpl");
        gplot_slip_angle.SetGrid();
        gplot_slip_angle.SetLabelX("time (s)");
        gplot_slip_angle.SetLabelY("Slip angle");
        gplot_slip_angle.Plot(slip_angle_fct, "", " with lines lt -1 lc rgb'#00AAEE' ");

        postprocess::ChGnuPlot gplot_camber_angle(out_dir + "/tmp3.gpl");
        gplot_camber_angle.SetGrid();
        gplot_camber_angle.SetLabelX("time (s)");
        gplot_camber_angle.SetLabelY("Camber angle");
        gplot_camber_angle.Plot(camber_angle_fct, "", " with lines lt -1 lc rgb'#00AAEE' ");
    }
#endif

    return 0;
}
