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
// Authors: Dario Mangoni, Radu Serban
// =============================================================================

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#include "chrono/utils/ChOpenMP.h"

namespace chrono {

ChSolverPardisoMKL::ChSolverPardisoMKL(unsigned int num_threads) {
    mkl_set_num_threads(num_threads);
}

bool ChSolverPardisoMKL::FactorizeMatrix() {
    m_engine.compute(m_mat);
    return (m_engine.info() == Eigen::Success);
}

bool ChSolverPardisoMKL::SolveSystem() {
    m_sol = m_engine.solve(m_rhs);
    return (m_engine.info() == Eigen::Success);
}

void ChSolverPardisoMKL::PrintErrorMessage() {
    // There are only three possible return codes (see manageErrorCode in Eigen's PardisoSupport.h)
    switch (m_engine.info()) {
        case Eigen::Success:
            std::cout << "PardisoMKL: computation was successful" << std::endl;
            break;
        case Eigen::NumericalIssue:
            std::cout << "PardisoMKL: provided data did not satisfy the prerequisites" << std::endl;
            break;
        case Eigen::InvalidInput:
            std::cout << "PardisoMKL: inputs are invalid, or the algorithm has been improperly called" << std::endl;
            break;
        case Eigen::NoConvergence:
            // Not a possible error for Pardiso
            break;
    }
}

//----------------------------------------------------------------------------------

ChSolverComplexPardisoMKL::ChSolverComplexPardisoMKL(unsigned int num_threads) {
    mkl_set_num_threads(num_threads);
}

bool ChSolverComplexPardisoMKL::FactorizeMatrix() {
    m_engine.compute(m_mat);
    return (m_engine.info() == Eigen::Success);
}

bool ChSolverComplexPardisoMKL::SolveSystem(const ChVectorDynamic<std::complex<double>>& b) {
    m_sol = m_engine.solve(b);
    return (m_engine.info() == Eigen::Success);
}

void ChSolverComplexPardisoMKL::PrintErrorMessage() {
    // There are only three possible return codes (see manageErrorCode in Eigen's PardisoSupport.h)
    switch (m_engine.info()) {
        case Eigen::Success:
            std::cout << "PardisoMKL: computation was successful" << std::endl;
            break;
        case Eigen::NumericalIssue:
            std::cout << "PardisoMKL: provided data did not satisfy the prerequisites" << std::endl;
            break;
        case Eigen::InvalidInput:
            std::cout << "PardisoMKL: inputs are invalid, or the algorithm has been improperly called" << std::endl;
            break;
        case Eigen::NoConvergence:
            // Not a possible error for Pardiso
            break;
    }
}

}  // end of namespace chrono
