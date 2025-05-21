// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
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
// Demonstration of the single-wheel static tire test rig.
//
// =============================================================================

////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include <algorithm>

#include "chrono/core/ChTimer.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireStaticTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChMBTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "demos/SetChronoSolver.h"

using std::cout;
using std::cerr;
using std::endl;

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Tire specification file
////std::string tire_json = "hmmwv/tire/HMMWV_RigidTire.json";
////std::string tire_json = "hmmwv/tire/HMMWV_ANCF4Tire_Lumped.json";
////std::string tire_json = "hmmwv/tire/HMMWV_ANCF8Tire_Lumped.json";
////std::string tire_json = "hmmwv/tire/HMMWV_ReissnerTire.json";
std::string tire_json = "hmmwv/tire/HMMWV_MBTire.json";
////std::string tire_json = "Polaris/Polaris_RigidTire.json";
////std::string tire_json = "Polaris/Polaris_RigidMeshTire.json";
////std::string tire_json = "Polaris/Polaris_ANCF4Tire_Lumped.json";

// Wheel specification file
std::string wheel_json = "hmmwv/wheel/HMMWV_Wheel.json";
////std::string wheel_json = "Polaris/Polaris_Wheel.json";

// Tire contact model (nodes or faces)
auto surface_type = ChTire::ContactSurfaceType::NODE_CLOUD;

// Test mode
ChTireStaticTestRig::Mode test_mode = ChTireStaticTestRig::Mode::TEST_R;

bool debug_output = false;
bool gnuplot_output = true;

// -----------------------------------------------------------------------------

int main() {
    // --------------------------------
    // Create wheel and tire subsystems
    // --------------------------------

    auto wheel = ReadWheelJSON(vehicle::GetDataFile(wheel_json));
    auto tire = ReadTireJSON(vehicle::GetDataFile(tire_json));

    bool handling_tire = std::dynamic_pointer_cast<ChForceElementTire>(tire) != nullptr;
    bool fea_tire = std::dynamic_pointer_cast<ChDeformableTire>(tire) != nullptr;
    bool mb_tire = std::dynamic_pointer_cast<ChMBTire>(tire) != nullptr;
    bool rigid_tire = std::dynamic_pointer_cast<ChRigidTire>(tire) != nullptr;

    if (handling_tire) {
        cerr << "ERROR: Handling tire models cannot be used with the static test rig." << endl;
        return 1;
    }

    // Set tire contact surface
    int collision_family = 7;
    double surface_dim = 0.02;
    tire->SetContactSurfaceType(surface_type, surface_dim, collision_family);

    // ----------------------------
    // Create system and set solver
    // ----------------------------

    ChSystem* sys = nullptr;
    ChSolver::Type solver_type;
    ChTimestepper::Type integrator_type;
    double step_size = 0;

    if (mb_tire) {
        cout << "Setting solver and integrator for MB tire" << endl;

        sys = new ChSystemSMC;
        step_size = 5e-5;
        ////solver_type = ChSolver::Type::PARDISO_MKL;
        ////solver_type = ChSolver::Type::SPARSE_QR;
        ////solver_type = ChSolver::Type::BICGSTAB;
        solver_type = ChSolver::Type::MINRES;

        integrator_type = ChTimestepper::Type::EULER_IMPLICIT_PROJECTED;
        ////integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    } else if (fea_tire) {
        cout << "Setting solver and integrator for FEA tire" << endl;

        sys = new ChSystemSMC;
        step_size = 1e-4;
        solver_type = ChSolver::Type::PARDISO_MKL;
        integrator_type = ChTimestepper::Type::HHT;
    } else if (rigid_tire || handling_tire) {
        sys = new ChSystemNSC;
        step_size = 2e-4;
        solver_type = ChSolver::Type::BARZILAIBORWEIN;
        integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    }

    // Set collision system
    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Number of OpenMP threads used in Chrono (SCM ray-casting and FEA)
    int num_threads_chrono = std::min(4, ChOMP::GetNumProcs());

    // Number of threads used in collision detection
    int num_threads_collision = std::min(4, ChOMP::GetNumProcs());

    // Number of threads used by Eigen
    int num_threads_eigen = 1;

    // Number of threads used by PardisoMKL
    int num_threads_pardiso = std::min(8, ChOMP::GetNumProcs());

    sys->SetNumThreads(num_threads_chrono, num_threads_collision, num_threads_eigen);
    SetChronoSolver(*sys, solver_type, integrator_type, num_threads_pardiso);
    tire->SetStepsize(step_size);

    auto hht = std::dynamic_pointer_cast<ChTimestepperHHT>(sys->GetTimestepper());
    if (hht) {
        hht->SetAlpha(-0.2);
        hht->SetMaxIters(5);
        hht->SetAbsTolerances(1e-2);
        hht->SetStepControl(false);
        hht->SetMinStepSize(1e-4);
    }

    // -----------------
    // Initialize output
    // -----------------

    const std::string out_dir = GetChronoOutputPath() + "TIRE_STATIC_TEST_RIG/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    cout << "Tire: " << std::quoted(tire->GetName()) << "  template: " << tire->GetTemplateName() << endl;

    // -----------------------------
    // Create and configure test rig
    // -----------------------------

    double g = 9.8;

    ChTireStaticTestRig rig(wheel, tire, sys);

    rig.SetGravitationalAcceleration(g);
    rig.SetOutput(out_dir, gnuplot_output);

    // Set tire options
    rig.SetTireStepsize(step_size);
    rig.SetTireCollisionType(ChTire::CollisionType::FOUR_POINTS);
    rig.SetTireVisualizationType(VisualizationType::MESH);

    // Set rig options
    rig.SetPlateMaterialProperties(0.8, 0, 2e7);

    // Set test parameters
    rig.SetNominalRadialLoad(3600 * g);
    rig.SetStateTransitionDelay(0.1);
    rig.SetRadialLoadSpeed(0.001);
    rig.SetLongitudinalLoadSpeed(0.1);
    rig.SetLateralLoadSpeed(0.1);
    rig.SetTorsionalLoadSpeed(0.1);

    // Initialize the tire test rig
    rig.Initialize(test_mode);

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
            vis_irr->AddCamera(ChVector3d(1.0, 2.5, 1.0), rig.GetWheelPos());
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
            vis_vsg->SetWindowSize(1200, 600);
            vis_vsg->SetWindowTitle("Tire Test Rig");
            vis_vsg->SetBackgroundColor(ChColor(0.4f, 0.5f, 0.6f));
            vis_vsg->AddCamera(ChVector3d(1.0, 2.5, 1.0), rig.GetWheelPos());
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Timers and counters
    ChTimer timer;         // timer for measuring total run time
    double time = 0;       // simulated time
    double sim_time = 0;   // simulation time
    double fps = 120;      // rendering frequency
    int render_frame = 0;  // render frame counter

    timer.start();
    while (vis->Run()) {
        time = sys->GetChTime();

        if (time >= render_frame / fps) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
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
            switch (rig.GetState()) {
                case ChTireStaticTestRig::State::COMPRESSING:
                    std::cout << "\r" << rig.GetCompressionLoad();
                    break;
                case ChTireStaticTestRig::State::DISPLACING:
                    std::cout << "\r" << rig.GetLoad();
                    break;
            }
        }
    }
    timer.stop();

    double step_time = timer();
    cout << "\rSimulated time: " << time << endl;
    cout << "Run time (simulation): " << sim_time << "  |  RTF: " << sim_time / time << endl;
    cout << "Run time (total):      " << step_time << "  |  RTF: " << step_time / time << endl;

    return 0;
}
