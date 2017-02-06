// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Antonio Recuero
// =============================================================================
//
// Demonstration of the link bushing joint
//
// Recall that Irrlicht uses a left-hand frame, so everything is rendered with
// left and right flipped.
//
// =============================================================================

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/physics/ChLinkBushing.h"
#include "chrono/physics/ChLinkForce.h"

using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    // Create the system
    ChSystem system;
    system.Set_G_acc(ChVector<>(0, 0, -9.81));

    // Create the ground body
    auto ground = std::make_shared<ChBodyEasyBox>(0.9, 0.9, 0.15, 10000, false, true);
    ground->SetPos(ChVector<>(0.0, 0.0, 0.0));
    system.AddBody(ground);
    ground->SetBodyFixed(true);

    // Create the 'bushing' body
    // Give an initial angular velocity
    auto body = std::make_shared<ChBodyEasyBox>(0.9, 0.9, 0.15, 1000, false, true);
    system.AddBody(body);
    body->SetBodyFixed(false);
    body->SetPos(ChVector<>(1.0, 0.0, 0.0));
    body->SetWvel_loc(ChVector<>(1.5, 1.5, -1.5));
    body->SetPos_dt(ChVector<>(1.0, -0.4, 0.2));

    auto body_col = std::make_shared<ChColorAsset>();
    body_col->SetColor(ChColor(0.6f, 0, 0));
    body->AddAsset(body_col);

    // Create the 'ideal joint' body
    // Give an initial angular velocity
    auto body_ij = std::make_shared<ChBodyEasyBox>(0.9, 0.9, 0.15, 1000, false, true);
    system.AddBody(body_ij);
    body_ij->SetBodyFixed(false);
    body_ij->SetPos(ChVector<>(1.0, 0.0, 0.0));
    body_ij->SetWvel_loc(ChVector<>(1.5, 1.5, -1.5));
    body_ij->SetPos_dt(ChVector<>(1.0, -0.4, 0.2));

    auto body_col_ij = std::make_shared<ChColorAsset>();
    body_col_ij->SetColor(ChColor(0.7f, 0.15f, 0.25f));
    body_ij->AddAsset(body_col_ij);

    ChMatrixNM<double, 6, 6> K_matrix;
    ChMatrixNM<double, 6, 6> R_matrix;

    for (unsigned int ii = 0; ii < 6; ii++) {
        K_matrix(ii, ii) = 95000.0;
        R_matrix(ii, ii) = 100.0;
    }

    // Create bushing element acting on selected degrees of freedoms
    // ChLinkBushing::Spherical: Rotational dofs are free, translational defined by stiffness/damping matrices
    // ChLinkBushing::Revolute: One rotational dof is free, rest of dofs defined by stiffness/damping matrices
    // ChLinkBushing::Mount: All six dofs defined by stiffness/damping matrices

    auto my_linkbushing = std::make_shared<ChLinkBushing>(ChLinkBushing::Mount);
    my_linkbushing->Initialize(body, ground, ChCoordsys<>(ChVector<>(0.5, 0.0, 0.0), ChQuaternion<>(1, 0, 0, 0)),
                               K_matrix, R_matrix);
    system.AddLink(my_linkbushing);

    auto my_linkrevolute = std::make_shared<ChLinkLockLock>();
    my_linkrevolute->Initialize(body_ij, ground, ChCoordsys<>(ChVector<>(0.5, 0.0, 0.0), ChQuaternion<>(1, 0, 0, 0)));
    system.AddLink(my_linkrevolute);

    // Create the Irrlicht application
    ChIrrApp application(&system, L"ChLinkBushing", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(3, 0, 3));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Simulation loop
    application.SetTimestep(0.001);
    system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    system.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    system.SetMaxItersSolverSpeed(20000);
    system.SetTolForce(1e-7);
    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        ChIrrTools::drawAllCOGs(system, application.GetVideoDriver(), 2);
        application.EndScene();
        application.DoStep();
        std::cout << "Time t = " << system.GetChTime() << " s\n";
    }

    return 0;
}
