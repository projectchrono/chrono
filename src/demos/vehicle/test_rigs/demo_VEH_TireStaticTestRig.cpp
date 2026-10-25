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
// The tire model to test is selected at run time.
//
// =============================================================================

////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include <algorithm>
#include <iomanip>

#include "chrono/core/ChTimer.h"
#include "chrono/utils/ChUtils.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireStaticTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"
#ifdef CHRONO_FEA
    #include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
    #include "chrono_vehicle/wheeled_vehicle/tire/ChMBTire.h"
#endif

#include "chrono_vsg/ChVisualSystemVSG.h"

#include "demos/SetChronoSolver.h"

using std::cout;
using std::cerr;
using std::cin;
using std::endl;

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------

// Available tire models (tire and wheel specification files, and description)
struct TireModel {
    std::string tire_json;
    std::string wheel_json;
    std::string description;
};

std::vector<TireModel> tire_models = {
    {"hmmwv/tire/HMMWV_RigidTire.json", "hmmwv/wheel/HMMWV_Wheel.json", "HMMWV rigid tire"},
    {"hmmwv/tire/HMMWV_RigidMeshTire.json", "hmmwv/wheel/HMMWV_Wheel.json", "HMMWV rigid mesh tire"},
#ifdef CHRONO_FEA
    {"hmmwv/tire/HMMWV_ANCF4Tire_Lumped.json", "hmmwv/wheel/HMMWV_Wheel.json", "HMMWV ANCF 4-node shell tire"},
    {"hmmwv/tire/HMMWV_ANCF8Tire_Lumped.json", "hmmwv/wheel/HMMWV_Wheel.json", "HMMWV ANCF 8-node shell tire"},
    {"hmmwv/tire/HMMWV_ReissnerTire.json", "hmmwv/wheel/HMMWV_Wheel.json", "HMMWV Reissner shell tire"},
    {"hmmwv/tire/HMMWV_MBTire.json", "hmmwv/wheel/HMMWV_Wheel.json", "HMMWV MB tire"},
#endif
    {"Polaris/Polaris_RigidTire.json", "Polaris/Polaris_Wheel.json", "Polaris rigid tire"},
    {"Polaris/Polaris_RigidMeshTire.json", "Polaris/Polaris_Wheel.json", "Polaris rigid mesh tire"},
#ifdef CHRONO_FEA
    {"Polaris/Polaris_ANCF4Tire_Lumped.json", "Polaris/Polaris_Wheel.json", "Polaris ANCF 4-node shell tire"},
#endif
};

// Tire contact model (nodes or faces)
auto surface_type = ChTire::ContactSurfaceType::NODE_CLOUD;

// Test mode
ChTireStaticTestRig::Mode test_mode = ChTireStaticTestRig::Mode::TEST_R;

bool verbose_solvers = false;
bool matrix_write = false;
bool debug_output = false;
bool gnuplot_output = true;

// -----------------------------------------------------------------------------

int main() {
    // -----------------
    // Select tire model
    // -----------------

    int num_models = (int)tire_models.size();
    int which = 0;
    cout << "Options:\n";
    for (int i = 0; i < num_models; i++)
        cout << std::setw(2) << i + 1 << "  " << tire_models[i].description << endl;
    cout << "\nSelect tire: ";
    cin >> which;
    cout << endl;
    ChClampValue(which, 1, num_models);
    const auto& tire_model = tire_models[which - 1];

    // --------------------------------
    // Create wheel and tire subsystems
    // --------------------------------

    auto wheel = ReadWheelJSON(GetVehicleDataFile(tire_model.wheel_json));
    auto tire = ReadTireJSON(GetVehicleDataFile(tire_model.tire_json));

    bool handling_tire = std::dynamic_pointer_cast<ChForceElementTire>(tire) != nullptr;
    bool rigid_tire = std::dynamic_pointer_cast<ChRigidTire>(tire) != nullptr;
    bool fea_tire = false;
    bool mb_tire = false;
#ifdef CHRONO_FEA
    fea_tire = std::dynamic_pointer_cast<ChDeformableTire>(tire) != nullptr;
    mb_tire = std::dynamic_pointer_cast<ChMBTire>(tire) != nullptr;
#endif

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
        step_size = 3e-5;
        ////solver_type = ChSolver::Type::PARDISO_MKL;
        ////solver_type = ChSolver::Type::MUMPS;
        ////solver_type = ChSolver::Type::SPARSE_QR;
        ////solver_type = ChSolver::Type::BICGSTAB;
        solver_type = ChSolver::Type::MINRES;
        ////solver_type = ChSolver::Type::PSOR;

        integrator_type = ChTimestepper::Type::EULER_IMPLICIT_PROJECTED;
        ////integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
        ////integrator_type = ChTimestepper::Type::HHT;
    } else if (fea_tire) {
        cout << "Setting solver and integrator for FEA tire" << endl;

        sys = new ChSystemSMC;
        step_size = 1e-4;
        solver_type = ChSolver::Type::PARDISO_MKL;
        integrator_type = ChTimestepper::Type::HHT;
    } else if (rigid_tire || handling_tire) {
        cout << "Setting solver and integrator for rigid or handling tire" << endl;

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
    SetChronoSolver(*sys, solver_type, integrator_type, num_threads_pardiso, true);
    tire->SetStepsize(step_size);

    auto hht = std::dynamic_pointer_cast<ChTimestepperHHT>(sys->GetTimestepper());
    if (hht) {
        hht->SetAlpha(-0.2);
        hht->SetMaxIters(10);
        hht->SetAbsTolerances(1e2, 1e2);
        hht->SetRelTolerance(1e-3);
        hht->SetStepControl(false);
        hht->SetMinStepSize(1e-5);
        hht->SetJacobianUpdateMethod(ChTimestepperImplicit::JacobianUpdate::AUTOMATIC);
        hht->AcceptTerminatedStep(false);
        hht->SetVerbose(verbose_solvers);
    }

    sys->GetSolver()->SetVerbose(verbose_solvers);

    // -----------------
    // Initialize output
    // -----------------

    const std::string out_dir = GetChronoOutputPath() + "TIRE_STATIC_TEST_RIG/";
    if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    cout << "Tire: " << std::quoted(tire->GetName()) << "  template: " << tire->GetTemplateName() << endl;

    sys->EnableSolverMatrixWrite(matrix_write, out_dir);

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
    rig.SetStateTransitionDelay(100 * step_size);
    rig.SetDropSpeed(0.1);
    rig.SetRadialLoadSpeed(0.01);
    rig.SetLongitudinalLoadSpeed(0.1);
    rig.SetLateralLoadSpeed(0.1);
    rig.SetTorsionalLoadSpeed(0.1);

    // Initialize the tire test rig
    rig.Initialize(test_mode);

    // ---------------------------------
    // Create the run-time visualization
    // ---------------------------------

    auto vis = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    vis->AttachSystem(sys);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowSize(1200, 600);
    vis->SetWindowTitle("Tire Test Rig");
    vis->AddCamera(ChVector3d(1.0, 2.5, 1.0), rig.GetWheelPos());
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
    double fps = 120;      // rendering frequency
    int render_frame = 0;  // render frame counter

    timer.start();
    while (vis->Run()) {
        time = sys->GetChTime();

        if (time >= render_frame / fps) {
            vis->Render();
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
