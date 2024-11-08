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

#include "chrono_models/vehicle/hmmwv/tire/HMMWV_ANCFTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_FialaTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_ReissnerTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_RigidTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_TMeasyTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_Pac89Tire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_MBTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Wheel.h"

#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFToroidalTire.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireStaticTestRig.h"

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

// Contact formulation type (SMC or NSC)
ChContactMethod contact_method = ChContactMethod::SMC;

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Tire model
enum class TireType { RIGID, TMEASY, FIALA, PAC89, PAC02, ANCF4, ANCF8, ANCF_TOROIDAL, REISSNER, MB };
TireType tire_type = TireType::RIGID;

// Test mode
ChTireStaticTestRig::Mode test_mode = ChTireStaticTestRig::Mode::TEST_R;

// Number of OpenMP threads used in Chrono (here, for parallel spring force evaluation and SCM ray-casting)
int num_threads_chrono = 4;

// Number of threads used in collision detection
int num_threads_collision = 4;

// Read from JSON specification file?
bool use_JSON = false;

bool debug_output = false;

// -----------------------------------------------------------------------------

int main() {
    // --------------------------------
    // Create wheel and tire subsystems
    // --------------------------------

    auto wheel = chrono_types::make_shared<hmmwv::HMMWV_Wheel>("Wheel");

    double tire_contact_surface_dim = 0.02;
    int tire_collision_family = 7;

    std::shared_ptr<ChTire> tire;
    if (use_JSON) {
        std::string tire_file;
        switch (tire_type) {
            case TireType::RIGID:
                tire_file = "hmmwv/tire/HMMWV_RigidTire.json";
                break;
            case TireType::TMEASY:
                tire_file = "hmmwv/tire/HMMWV_TMeasyTire.json";
                break;
            case TireType::FIALA:
                tire_file = "hmmwv/tire/HMMWV_FialaTire.json";
                break;
            case TireType::PAC89:
                tire_file = "hmmwv/tire/HMMWV_Pac89Tire.json";
                break;
            case TireType::PAC02:
                tire_file = "hmmwv/tire/HMMWV_Pac02Tire.json";
                break;
            case TireType::ANCF4:
                tire_file = "hmmwv/tire/HMMWV_ANCF4Tire_Lumped.json";
                break;
            case TireType::ANCF8:
                tire_file = "hmmwv/tire/HMMWV_ANCF8Tire_Lumped.json";
                break;
            case TireType::REISSNER:
                tire_file = "hmmwv/tire/HMMWV_ReissnerTire.json";
                break;
        }
        tire = ReadTireJSON(vehicle::GetDataFile(tire_file));
    } else {
        switch (tire_type) {
            case TireType::RIGID:
                tire = chrono_types::make_shared<hmmwv::HMMWV_RigidTire>("Rigid_tire");
                break;
            case TireType::TMEASY:
                tire = chrono_types::make_shared<hmmwv::HMMWV_TMeasyTire>("TMeasy_tire");
                break;
            case TireType::FIALA:
                tire = chrono_types::make_shared<hmmwv::HMMWV_FialaTire>("Fiala_tire");
                break;
            case TireType::PAC89:
                tire = chrono_types::make_shared<hmmwv::HMMWV_Pac89Tire>("Pac89_tire");
                break;
            case TireType::ANCF4: {
                auto ancf4_tire = chrono_types::make_shared<hmmwv::HMMWV_ANCFTire>(
                    "ANCF4_tire", hmmwv::HMMWV_ANCFTire::ElementType::ANCF_4);
                ancf4_tire->SetContactSurfaceType(ChTire::ContactSurfaceType::NODE_CLOUD, tire_contact_surface_dim,
                                                  tire_collision_family);
                tire = ancf4_tire;
                break;
            }
            case TireType::ANCF8: {
                auto ancf8_tire = chrono_types::make_shared<hmmwv::HMMWV_ANCFTire>(
                    "ANCF8_tire", hmmwv::HMMWV_ANCFTire::ElementType::ANCF_8);
                ancf8_tire->SetContactSurfaceType(ChTire::ContactSurfaceType::NODE_CLOUD, tire_contact_surface_dim,
                                                  tire_collision_family);
                tire = ancf8_tire;
                break;
            }
            case TireType::REISSNER: {
                auto reissner_tire = chrono_types::make_shared<hmmwv::HMMWV_ReissnerTire>("Reissner_tire");
                reissner_tire->SetContactSurfaceType(ChTire::ContactSurfaceType::NODE_CLOUD, tire_contact_surface_dim,
                                                     tire_collision_family);
                tire = reissner_tire;
                break;
            }
            case TireType::MB: {
                auto hmmwv_tire = chrono_types::make_shared<hmmwv::HMMWV_MBTire>("MB_Tire");
                hmmwv_tire->IsStiff(false);
                hmmwv_tire->ForceJacobianCalculation(false);
                hmmwv_tire->SetContactSurfaceType(ChTire::ContactSurfaceType::NODE_CLOUD, tire_contact_surface_dim,
                                                  tire_collision_family);
                tire = hmmwv_tire;
                break;
            }
        }
    }

    // ----------------------------
    // Create system and set solver
    // ----------------------------

    ChSystem* sys = nullptr;
    ChSolver::Type solver_type;
    ChTimestepper::Type integrator_type;
    double step_size = 1e-3;

    if (tire_type == TireType::ANCF4 || tire_type == TireType::ANCF8 || tire_type == TireType::ANCF_TOROIDAL ||
        tire_type == TireType::REISSNER) {
        if (contact_method != ChContactMethod::SMC)
            cout << "\nWarning! Contact formulation changed to SMC.\n" << endl;
        contact_method = ChContactMethod::SMC;
    }

    switch (contact_method) {
        case ChContactMethod::SMC:
            sys = new ChSystemSMC;
            step_size = 1e-3;

            ////solver_type = ChSolver::Type::PARDISO_MKL;
            ////solver_type = ChSolver::Type::SPARSE_QR;
            ////solver_type = ChSolver::Type::BICGSTAB;
            solver_type = ChSolver::Type::MINRES;

            integrator_type = ChTimestepper::Type::EULER_IMPLICIT_PROJECTED;
            ////integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;

            break;
        case ChContactMethod::NSC:
            sys = new ChSystemNSC;
            step_size = 1e-3;

            solver_type = ChSolver::Type::BARZILAIBORWEIN;

            integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;

            break;
    }

    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    sys->SetNumThreads(num_threads_chrono, num_threads_collision, 1);

    SetChronoSolver(*sys, solver_type, integrator_type);

    tire->SetStepsize(step_size);

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
    rig.SetOutput(out_dir, true);

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

    // Optionally, modify tire visualization (can be done only after initialization)
    if (auto tire_def = std::dynamic_pointer_cast<ChDeformableTire>(tire)) {
        if (tire_def->GetMeshVisualization())
            tire_def->GetMeshVisualization()->SetColorscaleMinMax(0.0, 5.0);  // range for nodal speed norm
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
            vis_vsg->AddCamera(ChVector3d(1.0, 2.5, 1.0), rig.GetWheelPos());
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->SetShadows(true);
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
        }
    }
    timer.stop();

    double step_time = timer();
    cout << "\rSimulated time: " << time << endl;
    cout << "Run time (simulation): " << sim_time << "  |  RTF: " << sim_time / time << endl;
    cout << "Run time (total):      " << step_time << "  |  RTF: " << step_time / time << endl;

    return 0;
}
