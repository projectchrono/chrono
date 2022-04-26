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
#include <iomanip>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/fea/ChElementBeamANCF_3333.h"
#include "chrono/fea/ChMesh.h"

#ifdef CHRONO_PARDISO_MKL
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

bool use_mkl = true;
const double u_y_Ref = 8.091623235e-4;
const double u_x_Ref = 1.944145290e-7;
const double rel_Tol = 1e-7;

using namespace chrono;
using namespace chrono::fea;

int main(int argc, char* argv[]) {
    // Create a Chrono::Engine physical system
    ChSystemNSC sys;

    // Create a mesh, that is a container for groups of elements and
    // their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    sys.Set_G_acc(ChVector<>(0, -9.81, 0.0));
    const double beam_h = 0.5;  // Beam height (y)
    const double beam_w = 0.1;  // Beam width (z)
    const double beam_l = 2.0;  // Beam length

    double rho = 2000.0;                                       // Beam material density
    const double E_mod = 2.07e11;                              // Beam modulus of elasticity
    const double nu_rat = 0.3;                                 // Beam material Poisson ratio
    const double k1 = 10 * (1 + nu_rat) / (12 + 11 * nu_rat);  // Timoshenko coefficient
    const double k2 = k1;                                      // Timoshenko coefficient

    auto m_beamMaterial = chrono_types::make_shared<ChMaterialBeamANCF>(rho, E_mod, nu_rat, k1, k2);

    // Create the end nodes
    auto hnodeancf1 = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector<>(0, 0, 0.0), ChVector<>(0, 1, 0), ChVector<>(0, 0, 1));
    auto hnodeancf2 =
        chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector<>(beam_l / 4, 0, 0), ChVector<>(0, 1, 0), ChVector<>(0, 0, 1));
    auto hnodeancf3 =
        chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector<>(beam_l / 2, 0, 0), ChVector<>(0, 1, 0), ChVector<>(0, 0, 1));
    auto hnodeancf4 =
        chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector<>(3.0 * beam_l / 4, 0, 0), ChVector<>(0, 1, 0), ChVector<>(0, 0, 1));
    auto hnodeancf5 =
        chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector<>(beam_l, 0, 0), ChVector<>(0, 1, 0), ChVector<>(0, 0, 1));

    // Create the middle nodes
    auto hnodeancfm1 =
        chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector<>(beam_l / 8, 0, 0.0), ChVector<>(0, 1, 0), ChVector<>(0, 0, 1));
    auto hnodeancfm2 =
        chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector<>(3 * beam_l / 8, 0, 0), ChVector<>(0, 1, 0), ChVector<>(0, 0, 1));
    auto hnodeancfm3 =
        chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector<>(5 * beam_l / 8, 0, 0), ChVector<>(0, 1, 0), ChVector<>(0, 0, 1));
    auto hnodeancfm4 =
        chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector<>(7 * beam_l / 8, 0, 0), ChVector<>(0, 1, 0), ChVector<>(0, 0, 1));

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
    auto belementancf1 = chrono_types::make_shared<ChElementBeamANCF_3333>();
    belementancf1->SetNodes(hnodeancf1, hnodeancf2, hnodeancfm1);
    belementancf1->SetDimensions(beam_l / 4, beam_h, beam_w);
    belementancf1->SetMaterial(m_beamMaterial);
    belementancf1->SetAlphaDamp(0.0004);
    my_mesh->AddElement(belementancf1);

    // Create the element 2
    auto belementancf2 = chrono_types::make_shared<ChElementBeamANCF_3333>();
    belementancf2->SetNodes(hnodeancf2, hnodeancf3, hnodeancfm2);
    belementancf2->SetDimensions(beam_l / 4, beam_h, beam_w);
    belementancf2->SetMaterial(m_beamMaterial);
    belementancf2->SetAlphaDamp(0.0004);
    my_mesh->AddElement(belementancf2);

    // Create the element 3
    auto belementancf3 = chrono_types::make_shared<ChElementBeamANCF_3333>();
    belementancf3->SetNodes(hnodeancf3, hnodeancf4, hnodeancfm3);
    belementancf3->SetDimensions(beam_l / 4, beam_h, beam_w);
    belementancf3->SetMaterial(m_beamMaterial);
    belementancf3->SetAlphaDamp(0.0004);
    my_mesh->AddElement(belementancf3);

    // Create the element 4
    auto belementancf4 = chrono_types::make_shared<ChElementBeamANCF_3333>();
    belementancf4->SetNodes(hnodeancf4, hnodeancf5, hnodeancfm4);
    belementancf4->SetDimensions(beam_l / 4, beam_h, beam_w);
    belementancf4->SetMaterial(m_beamMaterial);
    belementancf4->SetAlphaDamp(0.0004);
    my_mesh->AddElement(belementancf4);

    // Cancel automatic gravity
    my_mesh->SetAutomaticGravity(false);

    // Remember to add the mesh to the system
    sys.Add(my_mesh);

#ifndef CHRONO_PARDISO_MKL
    use_mkl = false;
#endif
    // Setup solver
    if (use_mkl) {
#ifdef CHRONO_PARDISO_MKL
        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(false);
        mkl_solver->SetVerbose(false);
        sys.SetSolver(mkl_solver);
#endif
    } else {
        auto solver = chrono_types::make_shared<ChSolverMINRES>();
        sys.SetSolver(solver);
        solver->SetMaxIterations(100);
        solver->SetTolerance(1e-15);
        solver->EnableDiagonalPreconditioner(true);
        solver->SetVerbose(false);

        sys.SetSolverForceTolerance(1e-14);
    }

    // Setup integrator
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(10);
    mystepper->SetAbsTolerances(1e-10);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetScaling(false);
    mystepper->SetVerbose(true);
    mystepper->SetModifiedNewton(false);

	// Simulation loop
    unsigned int num_steps = 50;
    double time_Step = 0.01;
    std::cout << std::fixed << std::setprecision(12);
    for (unsigned int it = 0; it < num_steps; it++) {
         //std::cout << "Position of the tip: " << hnodeancf5->GetPos().y() << " m. \n";
         //std::cout << "Long. Position of the tip: " << hnodeancf5->GetPos().x() << " m. \n";
         //std::cout << "Lat. Position of the tip: " << hnodeancf5->GetPos().z() << " m. \n";

        hnodeancf5->SetForce(ChVector<>(0, -5e5 * std::pow(0.5, 3), 0));
        sys.DoStepDynamics(time_Step);
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
