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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/solver/ChDirectSolverLScomplex.h"
#include "chrono/core/ChSparsityPatternLearner.h"

#define SPM_DEF_SPARSITY 0.9  ///< default predicted sparsity (in [0,1])

namespace chrono {

ChDirectSolverLScomplex::ChDirectSolverLScomplex()
    : m_dim(0)
       {}


bool ChDirectSolverLScomplex::Setup() {

    // Allow the matrix to be compressed, if not yet compressed
    m_mat.makeCompressed();

    // Let the concrete solver perform the factorization
    bool result = FactorizeMatrix();

    if (verbose) {
        GetLog() << " Solver Setup()  n = " << m_dim
                 << "  nnz = " << (int)m_mat.nonZeros() << "\n";
    }

    if (!result) {
        // If the factorization failed, let the concrete solver display an error message.
        GetLog() << "Solver SetupCurrent() failed\n";
        PrintErrorMessage();
    }

    return result;
}

double ChDirectSolverLScomplex::Solve(const ChVectorDynamic<std::complex<double>>& b) {


    // Let the concrete solver compute the solution
    bool result = SolveSystem(b);

    if (verbose) {
        double res_norm = (b - m_mat * m_sol).norm();
        GetLog() << " Solver  |residual| = " << res_norm << "\n\n";
    }

    if (!result) {
        // If the solution failed, let the concrete solver display an error message.
        GetLog() << "Solver SolveCurrent() failed\n";
        PrintErrorMessage();
    }

    return result;
}



// ---------------------------------------------------------------------------

bool ChSolverSparseComplexLU::FactorizeMatrix() {
    m_engine.compute(m_mat);
    return (m_engine.info() == Eigen::Success);
}

bool ChSolverSparseComplexLU::SolveSystem(const ChVectorDynamic<std::complex<double>>& b) {
    m_sol = m_engine.solve(b);
    return (m_engine.info() == Eigen::Success);
}

void ChSolverSparseComplexLU::PrintErrorMessage() {
    // There are only three possible return codes (see Eigen SparseLU.h)
    switch (m_engine.info()) {
        case Eigen::Success:
            GetLog() << "computation was successful\n";
            break;
        case Eigen::NumericalIssue:
            GetLog() << "LU factorization reported a problem, zero diagonal for instance\n";
            break;
        case Eigen::InvalidInput:
            GetLog() << "inputs are invalid, or the algorithm has been improperly called\n";
            break;
        default:
            break;
    }
}

// ---------------------------------------------------------------------------

bool ChSolverSparseComplexQR::FactorizeMatrix() {
    m_engine.compute(m_mat);
    return (m_engine.info() == Eigen::Success);
}

bool ChSolverSparseComplexQR::SolveSystem(const ChVectorDynamic<std::complex<double>>& b) {
    m_sol = m_engine.solve(b);
    return (m_engine.info() == Eigen::Success);
}

void ChSolverSparseComplexQR::PrintErrorMessage() {
    // There are only three possible return codes (see Eigen SparseLU.h)
    switch (m_engine.info()) {
        case Eigen::Success:
            GetLog() << "computation was successful\n";
            break;
        case Eigen::NumericalIssue:
            GetLog() << "QR factorization reported a problem\n";
            break;
        case Eigen::InvalidInput:
            GetLog() << "inputs are invalid, or the algorithm has been improperly called\n";
            break;
        default:
            break;
    }
}

}  // end namespace chrono
