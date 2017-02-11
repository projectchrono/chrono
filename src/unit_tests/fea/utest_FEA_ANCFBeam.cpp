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
// Unit test for ANCF beam element (continuum-based). This unit test uses published
// data to verify the implementation of the internal forces of the ANCFbeamelement.
// For more information, refer to Nachbagauer, Gruber, and Gerstmayr, "Structural and
// continuum mechanics approaches for a 3D shear deformable ANCF beam finite element:
// Application to static and linearized dynamic examples", Journal of Computational
// and Nonlinear Dynamics, April 2013, Vol. 8/021004. Table 1 therein.
// =============================================================================
#include <cstdio>
#include <cmath>

#include "chrono/physics/ChSystem.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono_fea/ChElementBeamANCF.h"
#include "chrono_fea/ChMesh.h"

#ifdef CHRONO_MKL
#include "chrono_mkl/ChSolverMKL.h"
#endif

bool use_mkl = true;
const double u_y_Ref = 8.091623235e-4;
const double u_x_Ref = 1.944145290e-7;
const double rel_Tol = 1e-7;

using namespace chrono;
using namespace chrono::fea;

int main(int argc, char* argv[]) {
    // Create a Chrono::Engine physical system
    ChSystem my_system;

    // Create a mesh, that is a container for groups of elements and
    // their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    my_system.Set_G_acc(ChVector<>(0, -9.81, 0.0));
    const double beam_h = 0.5;  // Beam height (y)
    const double beam_w = 0.1;  // Beam width (z)
    const double beam_l = 2.0;  // Beam length

    unsigned int NElem = 4;  // Number of finite elements

    double rho = 2000.0;                                       // Beam material density
    const double E_mod = 2.07e11;                              // Beam modulus of elasticity
    const double nu_rat = 0.3;                                 // Beam material Poisson ratio
    const double k1 = 10 * (1 + nu_rat) / (12 + 11 * nu_rat);  // Timoshenko coefficient
    const double k2 = k1;                                      // Timoshenko coefficient

    auto m_beamMaterial = std::make_shared<ChMaterialBeamANCF>(rho, E_mod, nu_rat, k1, k2);

    // Create the end nodes
    auto hnodeancf1 = std::make_shared<ChNodeFEAxyzDD>(ChVector<>(0, 0, 0.0), ChVector<>(0, 1, 0), ChVector<>(0, 0, 1));
    auto hnodeancf2 =
        std::make_shared<ChNodeFEAxyzDD>(ChVector<>(beam_l / 4, 0, 0), ChVector<>(0, 1, 0), ChVector<>(0, 0, 1));
    auto hnodeancf3 =
        std::make_shared<ChNodeFEAxyzDD>(ChVector<>(beam_l / 2, 0, 0), ChVector<>(0, 1, 0), ChVector<>(0, 0, 1));
    auto hnodeancf4 =
        std::make_shared<ChNodeFEAxyzDD>(ChVector<>(3.0 * beam_l / 4, 0, 0), ChVector<>(0, 1, 0), ChVector<>(0, 0, 1));
    auto hnodeancf5 =
        std::make_shared<ChNodeFEAxyzDD>(ChVector<>(beam_l, 0, 0), ChVector<>(0, 1, 0), ChVector<>(0, 0, 1));

    // Create the middle nodes
    auto hnodeancfm1 =
        std::make_shared<ChNodeFEAxyzDD>(ChVector<>(beam_l / 8, 0, 0.0), ChVector<>(0, 1, 0), ChVector<>(0, 0, 1));
    auto hnodeancfm2 =
        std::make_shared<ChNodeFEAxyzDD>(ChVector<>(3 * beam_l / 8, 0, 0), ChVector<>(0, 1, 0), ChVector<>(0, 0, 1));
    auto hnodeancfm3 =
        std::make_shared<ChNodeFEAxyzDD>(ChVector<>(5 * beam_l / 8, 0, 0), ChVector<>(0, 1, 0), ChVector<>(0, 0, 1));
    auto hnodeancfm4 =
        std::make_shared<ChNodeFEAxyzDD>(ChVector<>(7 * beam_l / 8, 0, 0), ChVector<>(0, 1, 0), ChVector<>(0, 0, 1));

    hnodeancf1->SetFixed(true);  // Fix ALL coordinates of first (clamped) node

    my_mesh->AddNode(hnodeancf1);
    my_mesh->AddNode(hnodeancf2);
    my_mesh->AddNode(hnodeancf3);
    my_mesh->AddNode(hnodeancf4);
    my_mesh->AddNode(hnodeancf5);

    my_mesh->AddNode(hnodeancfm1);
    my_mesh->AddNode(hnodeancfm2);
    my_mesh->AddNode(hnodeancfm3);
    my_mesh->AddNode(hnodeancfm4);

    // Create the element 1
    auto belementancf1 = std::make_shared<ChElementBeamANCF>();
    belementancf1->SetNodes(hnodeancf1, hnodeancf2, hnodeancfm1);
    belementancf1->SetDimensions(beam_l / 4, beam_h, beam_w);
    belementancf1->SetMaterial(m_beamMaterial);
    belementancf1->SetAlphaDamp(0.0004);
    belementancf1->SetGravityOn(false);
    belementancf1->SetStrainFormulation(ChElementBeamANCF::StrainFormulation::CMPoisson);  // Neglect Poisson effect
    my_mesh->AddElement(belementancf1);

    // Create the element 2
    auto belementancf2 = std::make_shared<ChElementBeamANCF>();
    belementancf2->SetNodes(hnodeancf2, hnodeancf3, hnodeancfm2);
    belementancf2->SetDimensions(beam_l / 4, beam_h, beam_w);
    belementancf2->SetMaterial(m_beamMaterial);
    belementancf2->SetAlphaDamp(0.0004);
    belementancf2->SetGravityOn(false);
    belementancf2->SetStrainFormulation(ChElementBeamANCF::StrainFormulation::CMPoisson);  // Neglect Poisson effect
    my_mesh->AddElement(belementancf2);

    // Create the element 3
    auto belementancf3 = std::make_shared<ChElementBeamANCF>();
    belementancf3->SetNodes(hnodeancf3, hnodeancf4, hnodeancfm3);
    belementancf3->SetDimensions(beam_l / 4, beam_h, beam_w);
    belementancf3->SetMaterial(m_beamMaterial);
    belementancf3->SetAlphaDamp(0.0004);
    belementancf3->SetGravityOn(false);
    belementancf3->SetStrainFormulation(ChElementBeamANCF::StrainFormulation::CMPoisson);  // Neglect Poisson effect
    my_mesh->AddElement(belementancf3);

    // Create the element 4
    auto belementancf4 = std::make_shared<ChElementBeamANCF>();
    belementancf4->SetNodes(hnodeancf4, hnodeancf5, hnodeancfm4);
    belementancf4->SetDimensions(beam_l / 4, beam_h, beam_w);
    belementancf4->SetMaterial(m_beamMaterial);
    belementancf4->SetAlphaDamp(0.0004);
    belementancf4->SetGravityOn(false);
    belementancf4->SetStrainFormulation(ChElementBeamANCF::StrainFormulation::CMPoisson);  // Neglect Poisson effect
    my_mesh->AddElement(belementancf4);

    // Cancel automatic gravity
    my_mesh->SetAutomaticGravity(false);

    // Remember to add the mesh to the system
    my_system.Add(my_mesh);

#ifndef CHRONO_MKL
    use_mkl = false;
#endif
    // Setup solver
    if (use_mkl) {
#ifdef CHRONO_MKL
        auto mkl_solver = std::make_shared<ChSolverMKL<>>();
        mkl_solver->SetSparsityPatternLock(false);
        mkl_solver->SetVerbose(false);
        my_system.SetSolver(mkl_solver);
#endif
    } else {
        my_system.SetSolverType(ChSolver::Type::MINRES);
        auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
        msolver->SetDiagonalPreconditioning(true);
        my_system.SetSolverWarmStarting(true);
        my_system.SetMaxItersSolverSpeed(100);
        my_system.SetMaxItersSolverStab(100);
        my_system.SetTolForce(1e-14);
    }

    // Setup integrator
    my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(10);
    mystepper->SetAbsTolerances(1e-10);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetScaling(false);
    mystepper->SetVerbose(true);
    mystepper->SetModifiedNewton(false);

    // Mark completion of system construction
    my_system.SetupInitial();
    unsigned int num_steps = 50;

    double time_Step = 0.01;
    std::cout << std::fixed << std::setprecision(12);
    for (unsigned int it = 0; it < num_steps; it++) {
        // std::cout << "Position of the tip: " << hnodeancf5->GetPos().y << " m. \n";
        // std::cout << "Long. Position of the tip: " << hnodeancf5->GetPos().x << " m. \n";
        // std::cout << "Lat. Position of the tip: " << hnodeancf5->GetPos().z << " m. \n";

        hnodeancf5->SetForce(ChVector<>(0, -5e5 * std::pow(0.5, 3), 0));
        my_system.DoStepDynamics(time_Step);
    }
    double error_y = (hnodeancf5->GetPos().y() + u_y_Ref) / u_y_Ref;
    double error_x = (hnodeancf5->GetPos().x() + u_x_Ref - 2.0) / u_x_Ref;
    if (ChMax(error_x, error_y) > rel_Tol) {
        return 1;
    }
    std::cout << "Position of the tip: " << hnodeancf5->GetPos().y() << " m. \n";
    std::cout << "Long. Position of the tip: " << hnodeancf5->GetPos().x() << " m. \n";
    std::cout << "Lat. Position of the tip: " << hnodeancf5->GetPos().z() << " m. \n";
    return 0;
}
