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

#include "chrono_mumps/ChSolverMumps.h"

namespace chrono {

void ChSolverMumps::EnableNullPivotDetection(bool val, double threshold) {
    m_null_pivot_detection = val;
    m_engine.EnableNullPivotDetection(val, threshold);
}

void ChSolverMumps::SetMatrixSymmetryType(MatrixSymmetryType symmetry) {
    m_symmetry = symmetry;

    switch (m_symmetry) {
        case MatrixSymmetryType::GENERAL:
            m_engine.SetMatrixSymmetry(ChMumpsEngine::UNSYMMETRIC);
            break;
        case MatrixSymmetryType::SYMMETRIC_POSDEF:
            m_engine.SetMatrixSymmetry(ChMumpsEngine::SYMMETRIC_POSDEF);
            break;
        case MatrixSymmetryType::SYMMETRIC_INDEF:
            m_engine.SetMatrixSymmetry(ChMumpsEngine::SYMMETRIC_GENERAL);
            break;
        default:
            m_engine.SetMatrixSymmetry(ChMumpsEngine::UNSYMMETRIC);
            break;
    }
}

bool ChSolverMumps::FactorizeMatrix() {
    m_engine.SetMatrix(m_mat);
    auto mumps_err = m_engine.MumpsCall(ChMumpsEngine::mumps_JOB::ANALYZE_FACTORIZE);
    return (mumps_err == 0);
}

bool ChSolverMumps::SolveSystem() {
    m_sol = m_rhs;
    m_engine.SetRhsVector(m_sol);
    auto mumps_err = m_engine.MumpsCall(ChMumpsEngine::mumps_JOB::SOLVE);
    return (mumps_err == 0);
}

void ChSolverMumps::PrintErrorMessage() {
    m_engine.PrintINFOG();
}

}  // namespace chrono
