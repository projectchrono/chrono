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

#include "chrono_mkl/ChSolverMKL.h"

namespace chrono {

bool ChSolverMKL::FactorizeMatrix() {
    m_engine.compute(m_mat);
    return (m_engine.info() == Eigen::Success);
}

bool ChSolverMKL::SolveSystem() {
    m_sol = m_engine.solve(m_rhs);
    return (m_engine.info() == Eigen::Success);
}

void ChSolverMKL::PrintErrorMessage() {
    // There are only three possible return codes (see manageErrorCode in Eigen's PardisoSupport.h)
    switch (m_engine.info()) {
        case Eigen::Success:
            GetLog() << "computation was successful";
            break;
        case Eigen::NumericalIssue:
            GetLog() << "provided data did not satisfy the prerequisites";
            break;
        case Eigen::InvalidInput:
            GetLog() << "inputs are invalid, or the algorithm has been improperly called";
            break;
    }
}

}  // end of namespace chrono
