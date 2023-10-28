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
// Authors: Radu Serban
// =============================================================================
//
// Utility function for selecting the Chrono solver and integrator.
// Use with caution and double check that the solver and integrator parameters
// are appropriate for the calling program.
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystem.h"

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono/solver/ChDirectSolverLS.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#ifdef CHRONO_MUMPS
    #include "chrono_mumps/ChSolverMumps.h"
#endif

bool SetChronoSolver(chrono::ChSystem& sys,
                     const chrono::ChSolver::Type& solver_type,
                     const chrono::ChTimestepper::Type& integrator_type,
                     bool verbose = false) {
    auto slvr_type = solver_type;
    auto intg_type = integrator_type;

    using std::cout;
    using std::endl;
    std::string prefix = "[SetChronoSolver] ";

    // For NSC systems, use implicit linearized Euler and an iterative VI solver
    if (sys.GetContactMethod() == chrono::ChContactMethod::NSC) {
        intg_type = chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
        if (verbose)
            cout << prefix << "NSC system - setting integrator to EULER_IMPLICIT_LINEARIZED" << endl;
        if (slvr_type != chrono::ChSolver::Type::BARZILAIBORWEIN &&  //
            slvr_type != chrono::ChSolver::Type::APGD &&             //
            slvr_type != chrono::ChSolver::Type::PSOR &&             //
            slvr_type != chrono::ChSolver::Type::PSSOR) {
            slvr_type = chrono::ChSolver::Type::BARZILAIBORWEIN;
            cout << prefix << "NSC system - setting solver to BARZILAIBORWEIN" << endl;
        }
    }

    // Barzilai-Borwein cannot be used with stiffness matrices
    if (slvr_type == chrono::ChSolver::Type::BARZILAIBORWEIN &&
        sys.GetSystemDescriptor()->GetKblocksList().size() > 0) {
        cout << prefix << "BARZILAIBORWEIN cannot be used for a system that includes stiffness matrices!" << endl;
        return false;
    }

    // If none of the direct sparse solver modules is enabled, default to SPARSE_QR
    if (slvr_type == chrono::ChSolver::Type::PARDISO_MKL) {
#ifndef CHRONO_PARDISO_MKL
        slvr_type = chrono::ChSolver::Type::SPARSE_QR;
        cout << prefix << "Chrono::PardisoMKL not enabled. Setting solver to SPARSE_QR" << endl;
#endif
    } else if (slvr_type == chrono::ChSolver::Type::PARDISO_PROJECT) {
#ifndef CHRONO_PARDISOPROJECT
        slvr_type = chrono::ChSolver::Type::SPARSE_QR;
        cout << prefix << "Chrono::PardisoProject not enabled. Setting solver to SPARSE_QR" << endl;
#endif
    } else if (slvr_type == chrono::ChSolver::Type::MUMPS) {
#ifndef CHRONO_MUMPS
        slvr_type = chrono::ChSolver::Type::SPARSE_QR;
        cout << prefix << "Chrono::MUMPS not enabled. Setting solver to SPARSE_QR" << endl;
#endif
    }

    if (slvr_type == chrono::ChSolver::Type::PARDISO_MKL) {
#ifdef CHRONO_PARDISO_MKL
        auto solver = chrono_types::make_shared<chrono::ChSolverPardisoMKL>();
        solver->LockSparsityPattern(true);
        sys.SetSolver(solver);
#endif
    } else if (slvr_type == chrono::ChSolver::Type::PARDISO_PROJECT) {
#ifdef CHRONO_PARDISOPROJECT
        auto solver = chrono_types::make_shared<chrono::ChSolverPardisoProject>();
        solver->LockSparsityPattern(true);
        sys->SetSolver(solver);
#endif
    } else if (slvr_type == chrono::ChSolver::Type::MUMPS) {
#ifdef CHRONO_MUMPS
        auto solver = chrono_types::make_shared<chrono::ChSolverMumps>();
        solver->LockSparsityPattern(true);
        solver->EnableNullPivotDetection(true);
        solver->GetMumpsEngine().SetICNTL(14, 50);
        sys.SetSolver(solver);
#endif
    } else {
        sys.SetSolverType(slvr_type);
        switch (slvr_type) {
            case chrono::ChSolver::Type::SPARSE_LU:
            case chrono::ChSolver::Type::SPARSE_QR: {
                auto solver = std::static_pointer_cast<chrono::ChDirectSolverLS>(sys.GetSolver());
                solver->LockSparsityPattern(false);
                solver->UseSparsityPatternLearner(false);
                break;
            }
            case chrono::ChSolver::Type::BARZILAIBORWEIN:
            case chrono::ChSolver::Type::APGD:
            case chrono::ChSolver::Type::PSOR: {
                auto solver = std::static_pointer_cast<chrono::ChIterativeSolverVI>(sys.GetSolver());
                solver->SetMaxIterations(100);
                solver->SetOmega(0.8);
                solver->SetSharpnessLambda(1.0);

                ////sys.SetMaxPenetrationRecoverySpeed(1.5);
                ////sys.SetMinBounceSpeed(2.0);
                break;
            }
            case chrono::ChSolver::Type::BICGSTAB:
            case chrono::ChSolver::Type::MINRES:
            case chrono::ChSolver::Type::GMRES: {
                auto solver = std::static_pointer_cast<chrono::ChIterativeSolverLS>(sys.GetSolver());
                solver->SetMaxIterations(200);
                solver->SetTolerance(1e-10);
                solver->EnableDiagonalPreconditioner(true);
                break;
            }
            default:
                break;
        }
    }

    sys.SetTimestepperType(intg_type);
    switch (intg_type) {
        case chrono::ChTimestepper::Type::HHT: {
            auto integrator = std::static_pointer_cast<chrono::ChTimestepperHHT>(sys.GetTimestepper());
            integrator->SetAlpha(-0.2);
            integrator->SetMaxiters(50);
            integrator->SetAbsTolerances(1e-4, 1e2);
            integrator->SetStepControl(false);
            integrator->SetModifiedNewton(false);
            break;
        }
        case chrono::ChTimestepper::Type::EULER_IMPLICIT: {
            auto integrator = std::static_pointer_cast<chrono::ChTimestepperEulerImplicit>(sys.GetTimestepper());
            integrator->SetMaxiters(50);
            integrator->SetAbsTolerances(1e-4, 1e2);
            break;
        }
        case chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED:
        case chrono::ChTimestepper::Type::EULER_IMPLICIT_PROJECTED:
        default:
            break;
    }

    return true;
}
