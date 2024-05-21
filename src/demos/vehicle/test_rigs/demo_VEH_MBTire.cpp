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
// MBTire test
//
// =============================================================================

////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include <algorithm>
#include <iomanip>

#include "chrono/core/ChTimer.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_models/vehicle/hmmwv/tire/HMMWV_MBTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Wheel.h"

#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "demos/SetChronoSolver.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------

bool fix_wheel = false;

// -----------------------------------------------------------------------------

// Contact formulation type (SMC or NSC)
ChContactMethod contact_method = ChContactMethod::SMC;

// Terrain type (RIGID or SCM)
enum class TerrainType { RIGID, SCM };
TerrainType terrain_type = TerrainType::SCM;

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::IRRLICHT;

// Number of OpenMP threads used in Chrono (here, for parallel spring force evaluation and SCM ray-casting)
int num_threads_chrono = 4;

// Number of threads used in collision detection
int num_threads_collision = 4;

// =============================================================================

int main() {
    // Create system and set solver
    ChSystem* sys = nullptr;
    ChSolver::Type solver_type;
    ChTimestepper::Type integrator_type;
    double step_size = 1e-3;

    switch (contact_method) {
        case ChContactMethod::SMC:
            sys = new ChSystemSMC;
            step_size = 5e-5;

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
    sys->SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));

    sys->SetNumThreads(num_threads_chrono, num_threads_collision, 1);

    SetChronoSolver(*sys, solver_type, integrator_type);

    // Set vertical load on tire (added mass, in kg)
    double load = 600;

    // Create spindle body
    auto spindle = chrono_types::make_shared<ChBody>();
    spindle->SetFixed(fix_wheel);
    spindle->SetName("Spindle");
    spindle->SetMass(load);
    spindle->SetInertiaXX(ChVector3d(0.01, 0.02, 0.01));
    spindle->SetPos(ChVector3d(0, 0, 0));
    spindle->SetRot(QUNIT);
    sys->AddBody(spindle);

    // Create wheel and tire subsystems
    auto wheel = chrono_types::make_shared<hmmwv::HMMWV_Wheel>("Wheel");
    auto tire = chrono_types::make_shared<hmmwv::HMMWV_MBTire>("Tire");

    wheel->Initialize(nullptr, spindle, VehicleSide::LEFT);
    wheel->SetVisualizationType(VisualizationType::NONE);
    wheel->SetTire(tire);

    if (terrain_type == TerrainType::SCM)
        tire->SetContactSurfaceType(ChTire::ContactSurfaceType::TRIANGLE_MESH, 0.02);
    tire->IsStiff(false);
    tire->ForceJacobianCalculation(false);
    tire->SetStepsize(step_size);
    tire->SetCollisionFamily(11);
    tire->Initialize(wheel);
    tire->SetVisualizationType(VisualizationType::MESH);

    auto tire_radius = tire->GetRadius();
    auto grid_nodes = tire->GetGridNodes();
    auto rim_nodes = tire->GetRimNodes();

    // Create terrain
    std::shared_ptr<ChTerrain> terrain;
    double sizeX = 3;
    double sizeY = 3;

    switch (terrain_type) {
        case TerrainType::RIGID: {
            auto terrain_rigid = chrono_types::make_shared<vehicle::RigidTerrain>(sys);

            ChContactMaterialData cinfo;
            cinfo.mu = 0.8f;
            cinfo.cr = 0.0f;
            cinfo.Y = 2e7f;
            auto patch_mat = cinfo.CreateMaterial(contact_method);

            auto patch = terrain_rigid->AddPatch(patch_mat, ChCoordsys<>(ChVector3d(0, 0, -1.2 * tire_radius), QUNIT),
                                                 sizeX, sizeY, 0.1);
            patch->SetTexture(GetChronoDataFile("textures/concrete.jpg"), 10, 10);

            terrain_rigid->Initialize();
            terrain = terrain_rigid;

            break;
        }
        case TerrainType::SCM: {
            auto terrain_scm = chrono_types::make_shared<vehicle::SCMTerrain>(sys);

            double delta = 0.125;

            terrain_scm->SetPlane(ChCoordsys<>(ChVector3d(0, 0, -1.2 * tire_radius), QUNIT));
            terrain_scm->SetSoilParameters(2e6,   // Bekker Kphi
                                           0,     // Bekker Kc
                                           1.1,   // Bekker n exponent
                                           0,     // Mohr cohesive limit (Pa)
                                           30,    // Mohr friction limit (degrees)
                                           0.01,  // Janosi shear coefficient (m)
                                           2e8,   // Elastic stiffness (Pa/m), before plastic yield
                                           3e4    // Damping (Pa s/m), proportional to negative vertical speed
            );
            terrain_scm->SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.05);
            terrain_scm->Initialize(sizeX, sizeY, delta);
            terrain_scm->AddMovingPatch(spindle, ChVector3d(0, 0, 0),
                                        ChVector3d(2 * tire->GetRadius(), 1.0, 2 * tire->GetRadius()));

            terrain = terrain_scm;

            break;
        }
    }

    // Create run-time visualization interface
    std::shared_ptr<ChVisualSystem> vis;

#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

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
            vis_vsg->SetWindowSize(1200, 600);
            vis_vsg->SetWindowTitle("Tire Test Rig");
            vis_vsg->AddCamera(ChVector3d(1.0, 2.5, 1.0));
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->SetShadows(true);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    ChTimer timer;         // timer for measuring total run time
    double time = 0;       // simulated time
    double sim_time = 0;   // simulation time
    double fps = 120;      // rendering frequency
    int render_frame = 0;  // render frame counter

    // Perform the simulation
    timer.start();
    while (vis->Run()) {
        time = sys->GetChTime();

        // Render
        if (time >= render_frame / fps) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
            render_frame++;
        }

        // Synchronize subsystems
        terrain->Synchronize(time);
        tire->Synchronize(time, *terrain);
        spindle->EmptyAccumulators();
        wheel->Synchronize();

        // Advance state
        terrain->Advance(step_size);
        tire->Advance(step_size);
        sys->DoStepDynamics(step_size);

        sim_time += sys->GetTimerStep();

        std::cout << "\rRTF: " << sys->GetRTF();

        ////std::cout << std::setw(10) << std::setprecision(5) << time << " | " << grid_nodes[0]->GetPos() << std::endl;
    }
    timer.stop();

    double step_time = timer();
    std::cout << "\rSimulated time: " << time << std::endl;
    std::cout << "Run time (simulation): " << sim_time << "  |  RTF: " << sim_time / time << std::endl;
    std::cout << "Run time (total):      " << step_time << "  |  RTF: " << step_time / time << std::endl;

    return 0;
}
