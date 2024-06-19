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

#include "chrono/solver/ChIterativeSolverVI.h"

namespace chrono {

CH_UPCASTING(ChIterativeSolverVI, ChIterativeSolver)
CH_UPCASTING(ChIterativeSolverVI, ChSolverVI)
CH_UPCASTING(ChSolverVI, ChSolver)  // placed here since ChSolver is missing the .cpp

ChIterativeSolverVI::ChIterativeSolverVI()
    : ChIterativeSolver(50, 0.0, true, false),
      m_omega(1.0),
      m_shlambda(1.0),
      m_iterations(0),
      record_violation_history(false) {}

void ChIterativeSolverVI::SetOmega(double mval) {
    if (mval > 0.)
        m_omega = mval;
}

void ChIterativeSolverVI::SetSharpnessLambda(double mval) {
    if (mval > 0.)
        m_shlambda = mval;
}

void ChIterativeSolverVI::SetMaxIterations(int max_iterations) {
    m_max_iterations = max_iterations;
    violation_history.resize(m_max_iterations);
    dlambda_history.resize(m_max_iterations);
}

void ChIterativeSolverVI::SetRecordViolation(bool mval) {
    record_violation_history = mval;
    SetMaxIterations(m_max_iterations);
}

void ChIterativeSolverVI::AtIterationEnd(double mmaxviolation, double mdeltalambda, unsigned int iternum) {
    if (!record_violation_history)
        return;

    if (iternum >= violation_history.size()){
        assert(false && "Try to access out-of-bound.");
        return;
    }

    violation_history[iternum] = mmaxviolation;
    dlambda_history[iternum] = mdeltalambda;
}

void ChIterativeSolverVI::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChIterativeSolverVI>();
    // serialize parent class
    ChSolver::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_max_iterations);
    archive_out << CHNVP(m_warm_start);
    archive_out << CHNVP(m_tolerance);
    archive_out << CHNVP(m_omega);
    archive_out << CHNVP(m_shlambda);
}

void ChIterativeSolverVI::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChIterativeSolverVI>();
    // deserialize parent class
    ChSolver::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_max_iterations);
    archive_in >> CHNVP(m_warm_start);
    archive_in >> CHNVP(m_tolerance);
    archive_in >> CHNVP(m_omega);
    archive_in >> CHNVP(m_shlambda);
}

}  // end namespace chrono
