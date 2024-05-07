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

// Contact formulation type (SMC or NSC)
ChContactMethod contact_method = ChContactMethod::SMC;

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::IRRLICHT;

// -----------------------------------------------------------------------------

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
            solver_type = ChSolver::Type::SPARSE_QR;
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

    SetChronoSolver(*sys, solver_type, integrator_type);

    // Create spindle body
    auto spindle = chrono_types::make_shared<ChBody>();
    spindle->SetFixed(true);
    spindle->SetName("rig_spindle");
    spindle->SetMass(0);
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

    tire->IsStiff(false);
    tire->ForceJacobianCalculation(true);
    tire->SetStepsize(step_size);
    tire->SetContactFaceThickness(0.02);
    tire->Initialize(wheel);
    tire->SetVisualizationType(VisualizationType::MESH);

    auto tire_radius = 2 * tire->GetRadius();
    auto grid_nodes = tire->GetGridNodes();
    auto rim_nodes = tire->GetRimNodes();

    // Create terrain
    ChContactMaterialData cinfo;
    cinfo.mu = 0.8f;
    cinfo.cr = 0.0f;
    cinfo.Y = 2e7f;
    auto patch_mat = cinfo.CreateMaterial(contact_method);

    RigidTerrain terrain(sys);
    auto patch = terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector3d(0, 0, -tire_radius), QUNIT), 10.0, 10.0, 0.1);
    patch->SetTexture(GetChronoDataFile("textures/concrete.jpg"), 10, 10);
    terrain.Initialize();

    // Create the vehicle run-time visualization interface and the interactive driver
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

    // Perform the simulation
    while (vis->Run()) {
        double time = sys->GetChTime();

        // Render
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Synchronize subsystems
        terrain.Synchronize(time);
        tire->Synchronize(time, terrain);
        spindle->EmptyAccumulators();
        wheel->Synchronize();

        // Advance state
        terrain.Advance(step_size);
        tire->Advance(step_size);
        sys->DoStepDynamics(step_size);

        std::cout << std::setw(10) << std::setprecision(5) << time << " | " << grid_nodes[0]->GetPos() << std::endl;
    }

    return 0;
}
