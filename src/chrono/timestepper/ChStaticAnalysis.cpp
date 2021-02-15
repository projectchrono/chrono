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

#include <cstdlib>

#include "chrono/timestepper/ChStaticAnalysis.h"

namespace chrono {

ChStaticAnalysis::ChStaticAnalysis(ChIntegrableIIorder& integrable) {
    m_integrable = &integrable;
    X.setZero(1, m_integrable);
};

// -----------------------------------------------------------------------------

ChStaticLinearAnalysis::ChStaticLinearAnalysis(ChIntegrableIIorder& integrable) : ChStaticAnalysis(integrable) {}

void ChStaticLinearAnalysis::StaticAnalysis() {
    ChIntegrableIIorder* integrable = static_cast<ChIntegrableIIorder*>(m_integrable);

    // Set up main vectors
    double T;
    ChStateDelta V(integrable);
    X.resize(integrable->GetNcoords_x());
    V.resize(integrable->GetNcoords_v());
    integrable->StateGather(X, V, T);  // state <- system

    // Set V speed to zero
    V.setZero(integrable->GetNcoords_v(), integrable);
    integrable->StateScatter(X, V, T, true);  // state -> system

    // Set up auxiliary vectors
    ChStateDelta Dx;
    ChVectorDynamic<> R;
    ChVectorDynamic<> Qc;

    Dx.setZero(integrable->GetNcoords_v(), integrable);
    R.setZero(integrable->GetNcoords_v());
    Qc.setZero(integrable->GetNconstr());
    L.setZero(integrable->GetNconstr());

    // Solve
    //     [-dF/dx     Cq' ] [ dx  ] = [ f]
    //     [ Cq        0   ] [  l  ] = [ C]

    integrable->LoadResidual_F(R, 1.0);
    integrable->LoadConstraint_C(Qc, 1.0);

    integrable->StateSolveCorrection(  //
        Dx, L, R, Qc,                  //
        0,                             // factor for  M
        0,                             // factor for  dF/dv
        -1.0,                          // factor for  dF/dx (the stiffness matrix)
        X, V, T,                       // not needed here
        false,                         // do not scatter Xnew Vnew T+dt before computing correction
        false,                         // full update? (not used, since no scatter)
        true                           // force a call to the solver's Setup() function
    );

    X += Dx;

    integrable->StateScatter(X, V, T, true);     // state -> system
    integrable->StateScatterReactions(L);  // -> system auxiliary data
}

// -----------------------------------------------------------------------------

ChStaticNonLinearAnalysis::ChStaticNonLinearAnalysis(ChIntegrableIIorder& integrable)
    : ChStaticAnalysis(integrable),
      m_maxiters(20),
      m_incremental_steps(6),
      m_use_correction_test(true),
      m_reltol(1e-4),
      m_abstol(1e-8),
      m_verbose(false) {}

void ChStaticNonLinearAnalysis::StaticAnalysis() {
    ChIntegrableIIorder* integrable = static_cast<ChIntegrableIIorder*>(m_integrable);

    if (m_verbose) {
        GetLog() << "\nNonlinear statics\n";
        GetLog() << "   max iterations:     " << m_maxiters << "\n";
        GetLog() << "   incremental steps:  " << m_incremental_steps << "\n";
        if (m_use_correction_test) {
            GetLog() << "   stopping test:      correction\n";
            GetLog() << "      relative tol:    " << m_reltol << "\n";
            GetLog() << "      absolute tol:    " << m_abstol << "\n";
        } else {
            GetLog() << "   stopping test:      residual\n";
            GetLog() << "      tolerance:       " << m_abstol << "\n";
        }
        GetLog() << "\n";
    }

    // Set up main vectors
    double T;
    ChStateDelta V(integrable);
    X.resize(integrable->GetNcoords_x());
    V.resize(integrable->GetNcoords_v());
    integrable->StateGather(X, V, T);  // state <- system

    // Set speed to zero
    V.setZero(integrable->GetNcoords_v(), integrable);
    integrable->StateScatter(X, V, T, true);  // state -> system

    // Set up auxiliary vectors
    ChState Xnew;
    ChStateDelta Dx;
    ChVectorDynamic<> R;
    ChVectorDynamic<> Qc;
    Xnew.setZero(integrable->GetNcoords_x(), integrable);
    Dx.setZero(integrable->GetNcoords_v(), integrable);
    R.setZero(integrable->GetNcoords_v());
    Qc.setZero(integrable->GetNconstr());
    L.setZero(integrable->GetNconstr());

    // Use Newton Raphson iteration, solving for the increments
    //      [ - dF/dx    Cq' ] [ Dx  ] = [ f ]
    //      [ Cq         0   ] [ L   ] = [ C ]

    for (int i = 0; i < m_maxiters; ++i) {
        integrable->StateScatter(X, V, T, true);  // state -> system
        R.setZero();
        Qc.setZero();
        integrable->LoadResidual_F(R, 1.0);
        integrable->LoadConstraint_C(Qc, 1.0);

        double cfactor = ChMin(1.0, (i + 2.0) / (m_incremental_steps + 1.0));
        R *= cfactor;
        Qc *= cfactor;

        if (!m_use_correction_test) {
            // Evaluate residual norms
            double R_norm = R.lpNorm<Eigen::Infinity>();
            double Qc_norm = Qc.lpNorm<Eigen::Infinity>();

            if (m_verbose) {
                GetLog() << "--- Nonlinear statics iteration " << i << "  |R|_inf = " << R_norm
                         << "  |Qc|_inf = " << Qc_norm << "\n";
            }

            // Stopping test
            if ((R_norm < m_abstol) && (Qc_norm < m_abstol)) {
                if (m_verbose) {
                    GetLog() << "+++ Newton procedure converged in " << i + 1 << " iterations.\n\n";
                }
                break;
            }
        }

        // Solve linear system for correction
        integrable->StateSolveCorrection(  //
            Dx, L, R, Qc,                  //
            0,                             // factor for  M
            0,                             // factor for  dF/dv
            -1.0,                          // factor for  dF/dx (the stiffness matrix)
            X, V, T,                       // not needed here
            false,                         // do not scatter Xnew Vnew T+dt before computing correction
            false,                         // full update? (not used, since no scatter)
            true                           // force a call to the solver's Setup() function
        );

        Xnew = X + Dx;

        if (m_use_correction_test) {
            // Calculate actual correction in X
            ChState correction = Xnew - X;

            // Evaluate weights and correction WRMS norm
            ChVectorDynamic<> ewt = (m_reltol * Xnew.cwiseAbs() + m_abstol).cwiseInverse();
            double Dx_norm = correction.wrmsNorm(ewt);

            if (m_verbose) {
                GetLog() << "--- Nonlinear statics iteration " << i << "  |Dx|_wrms = " << Dx_norm << "\n";
            }

            // Stopping test
            if (Dx_norm < 1) {
                if (m_verbose) {
                    double R_norm = R.lpNorm<Eigen::Infinity>();
                    double Qc_norm = Qc.lpNorm<Eigen::Infinity>();
                    GetLog() << "+++ Newton procedure converged in " << i + 1 << " iterations.\n";
                    GetLog() << "    |R|_inf = " << R_norm << "  |Qc|_inf = " << Qc_norm << "\n\n";
                }
                X = Xnew;
                break;
            }
        }

        X = Xnew;
    }

    integrable->StateScatter(X, V, T, true);     // state -> system
    integrable->StateScatterReactions(L);  // -> system auxiliary data
}

void ChStaticNonLinearAnalysis::SetCorrectionTolerance(double reltol, double abstol) {
    m_use_correction_test = true;
    m_reltol = reltol;
    m_abstol = abstol;
}

void ChStaticNonLinearAnalysis::SetResidualTolerance(double tol) {
    m_use_correction_test = false;
    m_abstol = tol;
}

void ChStaticNonLinearAnalysis::SetMaxIterations(int max_iters) {
    m_maxiters = max_iters;
    if (m_incremental_steps > m_maxiters)
        m_incremental_steps = m_maxiters;
}

void ChStaticNonLinearAnalysis::SetIncrementalSteps(int incr_steps) {
    m_incremental_steps = incr_steps;
    if (m_maxiters < m_incremental_steps)
        m_maxiters = m_incremental_steps;
}

}  // end namespace chrono
