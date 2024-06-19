// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Antonio Recuero, Alessandro Tasora, Radu Serban
// =============================================================================
//
// Demonstration of flexible bushing between two bodies
//
// Recall that Irrlicht uses a left-hand frame, so everything is rendered with
// left and right flipped.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkBushing.h"
#include "chrono/physics/ChLinkForce.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

// Select one of the following examples:
//
// 1 - ChLoadBodyBodyBushingGeneric
//     This type of bushing requires two 6x6 matrices for generic stiffness and damping, for both translation and
//     rotation. Optionally, it also supports initial pre-displacement and pre-stress
// 2 - ChLoadBodyBodyBushingMate
//     This type of bushing is like a simplified version of ChLoadBodyBodyBushingGeneric, it adds compliance to both
//     translation and rotation, using three x y z and three Rx Ry Rz stiffness values.
// 3 - ChLoadBodyBodyBushingPlastic
//     A special type of ChLoadBodyBodyBushingSpherical that also provides plastic deformation with a plastic yeld.
// 4 - ChLinkBushing
//     The ChLinkBushing is inherited from the ChLink classes. Differently from the previous example, it does NOT
//     support stiffness matrices, so it should NOT be used for very stiff problems.
// 5 - ChLoadBodyBodyBushingGeneric
//     No stiffness and damping in one rotational direction

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Select example
    std::cout << "Options:" << std::endl;
    std::cout << "1  : ChLoadBodyBodyBushingGeneric" << std::endl;
    std::cout << "     This type of bushing requires two 6x6 matrices for generic stiffness and damping,\n"
                 "     for both translation and rotation. Optionally, it also supports initial\n"
                 "     pre-displacement and pre-stress"
              << std::endl;
    std::cout << "2  : ChLoadBodyBodyBushingMate" << std::endl;
    std::cout << "     This type of bushing is like a simplified version of ChLoadBodyBodyBushingGeneric,\n"
                 "     it adds compliance to both translation and rotation, using three x y z and three\n"
                 "     Rx Ry Rz stiffness values."
              << std::endl;
    std::cout << "3  : ChLoadBodyBodyBushingPlastic" << std::endl;
    std::cout << "     A special type of ChLoadBodyBodyBushingSpherical that also provides plastic deformation\n"
                 "     with a plastic yeld."
              << std::endl;
    std::cout << "4  : ChLinkBushing" << std::endl;
    std::cout << "     The ChLinkBushing is inherited from the ChLink classes. Differently from the previous example,\n"
                 "     it does NOT support stiffness matrices, so it should NOT be used for very stiff problems."
              << std::endl;
    std::cout << "5  : ChLoadBodyBodyBushingGeneric" << std::endl;
    std::cout << "     No stiffness and damping in one rotational direction" << std::endl;
    std::cout << "\nSelect option (1-5): ";
    int example = 1;
    std::cin >> example;
    std::cout << std::endl;
    ChClampValue(example, 1, 5);

    // Create the sys
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // Create the ground body
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(0.6, 0.6, 0.15, 10000, true, false);
    sys.Add(ground);
    ground->SetFixed(true);

    // Create a moving body that will 'bounce' thanks to a flexible bushing.
    // Give it an initial angular velocity and attach also a small sphere to show the anchoring of the bushing.
    auto body = chrono_types::make_shared<ChBodyEasyBox>(0.9, 0.9, 0.15, 1000, true, false);
    sys.Add(body);
    body->SetFixed(false);
    body->SetPos(ChVector3d(1.0, 0.0, 0.0));
    body->SetAngVelLocal(ChVector3d(1.5, 1.5, -1.5));
    body->SetPosDt(ChVector3d(1.0, -0.4, 0.2));
    body->GetVisualShape(0)->SetColor(ChColor(0.6f, 0, 0));

    auto symbol_bushing = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
    body->AddVisualShape(symbol_bushing, ChFrame<>(ChVector3d(-1, 0, 0), QUNIT));

    // Now create the bushing connecting the "body" to the "ground".
    // A bushing is like an invisible connection between two bodies, but differently from constraints, it has some
    // compliance.

    // Bushings are inherited from ChLoad, so they require a 'load container'

    auto load_container = chrono_types::make_shared<ChLoadContainer>();
    sys.Add(load_container);

    // EXAMPLE 1: use  ChLoadBodyBodyBushingGeneric
    //
    // This type of bushing requires two 6x6 matrices for generic stiffness and damping, for both translation and
    // rotation. Optionally, it also supports initial pre-displacement and pre-stress

    ChMatrix66d K_matrix;
    ChMatrix66d R_matrix;

    K_matrix.setZero();
    R_matrix.setZero();
    for (unsigned int ii = 0; ii < 6; ii++) {
        K_matrix(ii, ii) = 1e5;
        R_matrix(ii, ii) = 1e3;
    }

    auto bushing_generic = chrono_types::make_shared<ChLoadBodyBodyBushingGeneric>(
        body,                                  // body A
        ground,                                // body B
        ChFrame<>(ChVector3d(0.0, 0.0, 0.0)),  // initial frame of bushing in abs space
        K_matrix,                              // the 6x6 (translation+rotation) K matrix in local frame
        R_matrix                               // the 6x6 (translation+rotation) R matrix in local frame
    );
    bushing_generic->SetNeutralForce(ChVector3d(100, 0, 0));
    bushing_generic->NeutralDisplacement().SetPos(ChVector3d(0.02, 0, 0));
    if (example == 1) {
        load_container->Add(bushing_generic);
    }

    // EXAMPLE 2: use  ChLoadBodyBodyBushingMate
    //
    // This type of bushing is like a simplified version of ChLoadBodyBodyBushingGeneric, it adds compliance to both
    // translation and rotation, using three x y z and three Rx Ry Rz stiffness values.

    auto bushing_mate = chrono_types::make_shared<ChLoadBodyBodyBushingMate>(
        body,                                  // body A
        ground,                                // body B
        ChFrame<>(ChVector3d(0.0, 0.0, 0.0)),  // initial frame of bushing in abs space
        ChVector3d(95000.0),                   // K stiffness in local frame  [N/m]
        ChVector3d(100.0),                     // R damping in local frame  [N/m/s]
        ChVector3d(95000.0),                   // K rotational stiffness,in local frame [Nm/rad]
        ChVector3d(100.0)                      // R rotational damping, in local frame [Nm/rad/s]
    );
    if (example == 2) {
        load_container->Add(bushing_mate);
    }

    // EXAMPLE 3: use  ChLoadBodyBodyBushingPlastic
    //
    // A special type of ChLoadBodyBodyBushingSpherical that also provides plastic deformation with a plastic yeld.

    auto bushing_plastic = chrono_types::make_shared<ChLoadBodyBodyBushingPlastic>(
        body,                                  // body A
        ground,                                // body B
        ChFrame<>(ChVector3d(0.0, 0.0, 0.0)),  // initial frame of bushing in abs space
        ChVector3d(95000.0),                   // K stiffness in local frame  [N/m]
        ChVector3d(100.0),                     // R damping in local frame  [N/m/s]
        ChVector3d(18000.0)                    // plastic yield [N/m]
    );
    if (example == 3) {
        load_container->Add(bushing_plastic);
    }

    // EXAMPLE 4: use  ChLinkBushing
    //
    // Note, the ChLinkBushing is inherited from the ChLink classes. Differently from the previous example, it does NOT
    // support stiffness matrices, so it should NOT be used for very stiff problems. This behaves in various ways
    // according to the types in its enums:
    // - ChLinkBushing::Spherical: Rotational dofs are free, translational defined by stiffness/damping matrices
    // - ChLinkBushing::Revolute: One rotational dof is free, rest of dofs defined by stiffness/damping matrices
    // - ChLinkBushing::Mount: All six dofs defined by stiffness/damping matrices

    auto bushing_link = chrono_types::make_shared<ChLinkBushing>(ChLinkBushing::Type::Mount);
    bushing_link->Initialize(
        body,                                                              // body A
        ground,                                                            // body B
        ChFrame<>(ChVector3d(0.0, 0.0, 0.0), ChQuaternion<>(1, 0, 0, 0)),  // initial frame of bushing in abs space
        K_matrix,                                                          // K stiffness in local frame
        R_matrix);                                                         // R damping in local frame
    if (example == 4) {
        sys.Add(bushing_link);
    }

    // EXAMPLE 5: ChLoadBodyBodyBushingGeneric
    //
    // Same as example 1, but set the rotational stiffness and damping about Y axis to 0.

    if (example == 5) {
        K_matrix(4, 4) = 0;
        R_matrix(4, 4) = 0;
        bushing_generic->SetStiffnessMatrix(K_matrix);
        bushing_generic->SetDampingMatrix(R_matrix);
        load_container->Add(bushing_generic);
    }

    // Finally, add a force that tends to bend the body

    auto force = chrono_types::make_shared<ChLoadBodyForce>(body, ChVector3d(0, 0, -8e3), false, ChVector3d(1, 0, 0));
    load_container->Add(force);

    // Create the Irrlicht visualization sys
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("ChLinkBushing");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(3, 3, 0));
    vis->AddTypicalLights();

    vis->EnableBodyFrameDrawing(true);

    // Change some solver settings

    // Barzilai-Borwein does not support stiffness matrics => explicit integration for stiff bushings
    ////sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    // MINRES (or GMRES) support stiffness matrices => implicit integration of the stiff bushings
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(200);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);  // IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
    solver->SetVerbose(false);

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(1e-4);
    }

    return 0;
}
