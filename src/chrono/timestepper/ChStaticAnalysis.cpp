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

ChStaticAnalysis::ChStaticAnalysis() : m_integrable(nullptr){};

void ChStaticAnalysis::SetIntegrable(ChIntegrableIIorder* integrable) {
    m_integrable = integrable;
    X.setZero(1, m_integrable);
}

// -----------------------------------------------------------------------------

ChStaticLinearAnalysis::ChStaticLinearAnalysis() : ChStaticAnalysis() {}

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
    //     [ Cq        0   ] [  l  ] = [-C]

    integrable->LoadResidual_F(R, 1.0);
    integrable->LoadConstraint_C(Qc, 1.0);  // -C  (-sign already included)

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

ChStaticNonLinearAnalysis::ChStaticNonLinearAnalysis()
    : ChStaticAnalysis(),
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
    //      [ Cq         0   ] [ L   ] = [-C ]

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


// -----------------------------------------------------------------------------

ChStaticNonLinearRheonomicAnalysis::ChStaticNonLinearRheonomicAnalysis()
    : ChStaticAnalysis(),
      m_maxiters(20),
      m_incremental_steps(6),
      m_use_correction_test(true),
      m_reltol(1e-4),
      m_abstol(1e-8),
      m_verbose(false),
      automatic_speed_accel_computation(false) {}

void ChStaticNonLinearRheonomicAnalysis::StaticAnalysis() {
    ChIntegrableIIorder* integrable = static_cast<ChIntegrableIIorder*>(m_integrable);

    if (m_verbose) {
        GetLog() << "\nNonlinear static rheonomic\n";
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
    ChStateDelta Vp(integrable);
    ChStateDelta Vm(integrable);
    ChStateDelta A(integrable);
    X.resize(integrable->GetNcoords_x());
    V.resize(integrable->GetNcoords_v());
    Vp.resize(integrable->GetNcoords_v());
    Vm.resize(integrable->GetNcoords_v());
    A.resize(integrable->GetNcoords_v());


    integrable->StateGather(X, V, T);  // state <- system

    // Set speed to zero
    V.setZero(integrable->GetNcoords_v(), integrable);
    integrable->StateScatter(X, V, T, true);  // state -> system

    // Set up auxiliary vectors
    ChState Xnew;
    ChStateDelta Dx;
    ChVectorDynamic<> R;
    ChVectorDynamic<> Qc;
    ChVectorDynamic<> Dl;
    ChVectorDynamic<> L_v;
    Xnew.setZero(integrable->GetNcoords_x(), integrable);
    Dx.setZero(integrable->GetNcoords_v(), integrable);
    R.setZero(integrable->GetNcoords_v());
    Qc.setZero(integrable->GetNconstr());
    L.setZero(integrable->GetNconstr());
    L_v.setZero(integrable->GetNconstr());
    Dl.setZero(integrable->GetNconstr());
    double dt_perturbation = 1e-5;

    // Use Newton Raphson iteration

    for (int i = 0; i < m_maxiters; ++i) {

        integrable->StateScatter(X, V, T, true);  // state -> system

        // total load scaling factor
        double cfactor = ChMin(1.0, (i + 2.0) / (m_incremental_steps + 1.0));

        // Update nonzero speeds and accelerations, if any, calling 
        // the iteration callback, if any:
        if (this->callback_iteration_begin) {
            this->callback_iteration_begin->OnIterationBegin(cfactor, i, this); // this may modify V and A
            integrable->StateGather(X, V, T);  // state <- system
            integrable->StateGatherAcceleration(A);
        }

        // Otherwise, if enabled, compute them automatically from rheonomic constraints:
        // ***WARNING*** this is ok only for not too much stretched elements at the moment!
        if (i==0 && automatic_speed_accel_computation)  // btw. should happen at eeach iteration not only at first
        {
            // solve
            //      [ - dF/dx    Cq' ] [ V  ] = [ 0  ]
            //      [ Cq         0   ] [ L_v] = [-Ct ]

            R.setZero(integrable->GetNcoords_v());
            Qc.setZero(integrable->GetNconstr());
            integrable->LoadConstraint_Ct(Qc, 1.0);  // -Ct

            // Solve linear system for correction
            integrable->StateSolveCorrection(  //
                V, L_v,                        // unknowns
                R, Qc,                         // RHS
                0,                             // factor for  M
                0,                             // factor for  dF/dv
                -1.0,                          // factor for  dF/dx (the stiffness matrix)
                X, V, T,                       // not needed here
                false,                         // do not scatter Xnew Vnew T+dt before computing correction
                false,                         // full update? (not used, since no scatter)
                true                           // force a call to the solver's Setup() function
            );
            /*
            GetLog() << "V=\n" << V << "\n";
            GetLog() << "Qc=\n" <<  Qc << "\n";
            */
            Xnew = X + (V * dt_perturbation);   // small increment in position to recompute V and get acceleration by backward diff.

            integrable->StateScatter(Xnew, V, T, true);  // state -> system

            // Solve linear system for correction
            integrable->StateSolveCorrection(  //
                Vp, L_v,                       // unknowns
                R, Qc,                         // RHS
                0,                             // factor for  M
                0,                             // factor for  dF/dv
                -1.0,                          // factor for  dF/dx (the stiffness matrix)
                X, V, T,                       // not needed here
                false,                         // do not scatter Xnew Vnew T+dt before computing correction
                false,                         // full update? (not used, since no scatter)
                true                           // force a call to the solver's Setup() function
            );

            Xnew = X - (V * dt_perturbation);   // small increment in position to recompute V and get acceleration by backward diff.

            integrable->StateScatter(Xnew, V, T, true);  // state -> system

            // Solve linear system for correction
            integrable->StateSolveCorrection(  //
                Vm, L_v,                       // unknowns
                R, Qc,                         // RHS
                0,                             // factor for  M
                0,                             // factor for  dF/dv
                -1.0,                          // factor for  dF/dx (the stiffness matrix)
                X, V, T,                       // not needed here
                false,                         // do not scatter Xnew Vnew T+dt before computing correction
                false,                         // full update? (not used, since no scatter)
                true                           // force a call to the solver's Setup() function
            );

            A = (Vp - Vm) / (2*dt_perturbation);

            integrable->StateScatter(X, V, T, true);  // state -> system
            integrable->StateScatterAcceleration(A);
        }




        // B) solve for position increments, where in RHS includes all inertial forces:
        //    f automatically includes -centrifugal/gyroscopic terms at given acceleration/speed, and -M*a is added for completing inertial forces
        //
        //      [ - dF/dx    Cq' ] [ Dx  ] = [ f - M*a + Cq*L]
        //      [ Cq         0   ] [ Dl  ] = [ -C            ]

        R.setZero(integrable->GetNcoords_v());
        Qc.setZero(integrable->GetNconstr());
        integrable->LoadResidual_F(R, 1.0);
        integrable->LoadResidual_CqL(R, L, 1.0);
        integrable->LoadResidual_Mv(R, A, -1.0);
        integrable->LoadConstraint_C(Qc, 1.0);  // -C

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
            Dx, Dl, R, Qc,                 //
            0,                             // factor for  M
            0,                             // factor for  dF/dv
            -1.0,                          // factor for  dF/dx (the stiffness matrix)
            X, V, T,                       // not needed here
            false,                         // do not scatter Xnew Vnew T+dt before computing correction
            false,                         // full update? (not used, since no scatter)
            true                           // force a call to the solver's Setup() function
        );

        Xnew = X + Dx;
        L += Dl;

        /*
        GetLog() << "\n\n\ Iteration " << i << "\n\n";
        GetLog() << "R=" <<  R << "\n";
        GetLog() << "Qc=" <<  Qc << "\n";
        GetLog() << "Dx=" <<  Dx << "\n";
        */

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
            if (Dx_norm < 1 && i>3) {
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

    // Compute speed at the end. ***WARNING*** this is ok only for sligtly stretched elements at the moment!

    if (automatic_speed_accel_computation) {

        for (int i = 0; i < m_maxiters; ++i) {

            integrable->StateScatter(X, V, T, true);  // state -> system

            // B) solve for position increments, where in RHS includes all inertial forces:
            //    f automatically includes -centrifugal/gyroscopic terms at given acceleration/speed, and -M*a is added for completing inertial forces
            //
            //      [ - dF/dx    Cq' ] [ Dx  ] = [ f - M*a + Cq*L]
            //      [ Cq         0   ] [ Dl  ] = [ -C            ]

            R.setZero(integrable->GetNcoords_v());
            Qc.setZero(integrable->GetNconstr());
            integrable->LoadResidual_F(R, 1.0);
            integrable->LoadResidual_CqL(R, L, 1.0);
            integrable->LoadResidual_Mv(R, A, -1.0);
            //integrable->LoadConstraint_C(Qc, 1.0);  // -C
            integrable->LoadConstraint_Ct(Qc, 1.0);  // -Ct

            double cfactor = ChMin(1.0, (i + 2.0) / (m_incremental_steps + 1.0));
            R *= cfactor;
            Qc *= cfactor;

            // Solve linear system for correction
            integrable->StateSolveCorrection(  //
                V, Dl, R, Qc,                 //
                0,                             // factor for  M
                0,                             // factor for  dF/dv
                -1.0,                          // factor for  dF/dx (the stiffness matrix)
                X, V, T,                       // not needed here
                false,                         // do not scatter Xnew Vnew T+dt before computing correction
                false,                         // full update? (not used, since no scatter)
                true                           // force a call to the solver's Setup() function
            );

            //Xnew = X + Dx;
            L += Dl;

            //X = Xnew;
        }






        V.setZero(integrable->GetNcoords_v(), integrable);
        R.setZero(integrable->GetNcoords_v());
        Qc.setZero(integrable->GetNconstr());
        L_v.setZero(integrable->GetNconstr());
        integrable->LoadConstraint_Ct(Qc, 1.0);  // -Ct

        // Solve linear system for correction
        integrable->StateSolveCorrection(  //
            V, L_v,                        // unknowns
            R, Qc,                         // RHS
            0,                             // factor for  M
            0,                             // factor for  dF/dv
            -1.0,                          // factor for  dF/dx (the stiffness matrix)
            X, V, T,                       // not needed here
            false,                         // do not scatter Xnew Vnew T+dt before computing correction
            false,                         // full update? (not used, since no scatter)
            true                           // force a call to the solver's Setup() function
        );

        /*
        GetLog() << "\n\n\ Last step " << "\n\n";
        GetLog() << "R=" <<  R << "\n";
        GetLog() << "Qc=" <<  Qc << "\n";
        GetLog() << "V_last=\n" << V << "\n";
        */
        
        integrable->StateScatter(X, V, T, true);  // state -> system
        integrable->StateScatterAcceleration(A);
    }

    integrable->StateScatter(X, V, T, true);     // state -> system
    integrable->StateScatterReactions(L);  // -> system auxiliary data
}

void ChStaticNonLinearRheonomicAnalysis::SetCorrectionTolerance(double reltol, double abstol) {
    m_use_correction_test = true;
    m_reltol = reltol;
    m_abstol = abstol;
}

void ChStaticNonLinearRheonomicAnalysis::SetResidualTolerance(double tol) {
    m_use_correction_test = false;
    m_abstol = tol;
}

void ChStaticNonLinearRheonomicAnalysis::SetMaxIterations(int max_iters) {
    m_maxiters = max_iters;
    if (m_incremental_steps > m_maxiters)
        m_incremental_steps = m_maxiters;
}

void ChStaticNonLinearRheonomicAnalysis::SetIncrementalSteps(int incr_steps) {
    m_incremental_steps = incr_steps;
    if (m_maxiters < m_incremental_steps)
        m_maxiters = m_incremental_steps;
}




// -----------------------------------------------------------------------------

ChStaticNonLinearIncremental::ChStaticNonLinearIncremental()
    : ChStaticAnalysis(),
      max_newton_iters(5),
      m_incremental_steps(6),
      m_use_correction_test(true),
      m_reltol(1e-4),
      m_abstol(1e-8),
      m_verbose(false),
      m_adaptive_newton(true),
      m_adaptive_newton_tolerance(1.0),
      m_adaptive_newton_delay(1),
      m_newton_damping_factor(1.0)
    {}

void ChStaticNonLinearIncremental::StaticAnalysis() {
    ChIntegrableIIorder* integrable = static_cast<ChIntegrableIIorder*>(m_integrable);

    if (m_verbose) {
        GetLog() << "\nNonlinear statics with incremental external load\n";
        GetLog() << "   max Newton iterations per load step:     " << max_newton_iters << "\n";
        GetLog() << "   external load incremental steps:  " << m_incremental_steps << "\n";
        if (m_adaptive_newton) {
            GetLog() << "   using adaptive step for Newton iteration: \n";
            GetLog() << "      step shrinking tolerance:    " << m_adaptive_newton_tolerance << "\n";
            GetLog() << "      policy delayeyd for first steps:    " << m_adaptive_newton_delay << "\n";
        }
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
    ChVectorDynamic<> Dl;
    Xnew.setZero(integrable->GetNcoords_x(), integrable);
    Dx.setZero(integrable->GetNcoords_v(), integrable);
    R.setZero(integrable->GetNcoords_v());
    Qc.setZero(integrable->GetNconstr());
    L.setZero(integrable->GetNconstr());
    Dl.setZero(integrable->GetNconstr());

    // Outer loop: increment the external load(s)
    // by invoking the callback. 

    for (int j = 0; j < this->m_incremental_steps; ++j) {

        // The scaling factor for the external load (A simple linear scaling... it could be be improved).
        // Note on formula: have it ending with 1.0. If m_incremental_steps =1, do just one iteration with scaling =1.0.
        double cfactor = ((double)j + 1.0) / m_incremental_steps;

        // SCALE THE EXTERNAL LOADS!
        // This MUST be implemented by the user via a callback, becauses it is the only way we have to 
        // scale the external forces F_ext. The user must know the final F_ext, then updating data such as scale*F_ext is set in objects.
        // Then, after the callback is executed, as soon as we call integrable->LoadResidual_F(R, 1.0); 
        // we'll get that    R = F_in + scaled_F_ext.

        if (!this->load_increment_callback && m_verbose) 
            GetLog() << "Warning! Load callback not defined. Create one and use SetLoadIncrementCallback(...).\n";
        
        this->load_increment_callback->OnLoadScaling(cfactor, j, this);

        if (m_verbose) {
            GetLog() << "--- Nonlinear statics, outer iteration " << j << ", load scaling: " << cfactor << "\n";
        }

        // Inner loop: use Newton Raphson iteration, solving for the increments
        //      [ - dF/dx    Cq' ] [ Dx  ] = [ F_in + scaled_F_ext + Cq'*L]
        //      [ Cq         0   ] [ Dl  ] = [-C                          ]

        double step_factor = m_newton_damping_factor; // factor for NR step advancement (line search). When 1.0, original NR.
        double R_norm_old = 0;

        for (int i = 0; i < max_newton_iters; ++i) {

            integrable->StateScatter(X, V, T, true);  // state -> system
            R.setZero();
            Qc.setZero();
            integrable->LoadResidual_F(R, 1.0);      // put the F term in RHS  (where F = F_in + scaled_F_ext )  
            integrable->LoadResidual_CqL(R, L, 1.0); // put the Cq*L term in RHS
            integrable->LoadConstraint_C(Qc, 1.0);   // put the C term in RHS

            // Evaluate residual norms
            double R_norm = R.lpNorm<Eigen::Infinity>();
            double Qc_norm = Qc.lpNorm<Eigen::Infinity>();

            if (m_verbose) {
                GetLog() << "---   inner Newton iteration " << i << ",  |R|_inf = " << R_norm << "  |Qc|_inf = " << Qc_norm << " step_factor=" << step_factor << "\n";
            }

            // Basic line search for mitigating the issue of not converging residual.
            // Policy: just roll back half step in case of not-decreasing residual:
            if (true) {
                if ((i > m_adaptive_newton_delay) && (R_norm > m_adaptive_newton_tolerance * R_norm_old)) {
                    // a) Rewind state to previous one with this trick, reusing last Dx and last factor:
                    Xnew = X + (Dx * -step_factor);
                    L -= (Dl * step_factor);
                    // b) Reduce the step factor:
                    step_factor *= 0.5;
                    // c) Advance by the reduced Dx:
                    X = Xnew + (Dx * step_factor);
                    L += (Dl * step_factor);
                    // some debug message
                    if (m_verbose) {
                        GetLog() << "---     >>> |R| old=" << R_norm_old << ", |R|=" << R_norm << ". Diverges! Repeat w/smaller step factor: " << step_factor << "\n";
                    }
                    // d) repeat the for loop, skipping the rest:
                    continue;
                }

                // if things goes well, increase Dx factor at each iteration until factor is maximum again
                step_factor *= 2.0;
                if (step_factor > m_newton_damping_factor)
                    step_factor = m_newton_damping_factor;
            }

            R_norm_old = R_norm;

            if (!m_use_correction_test) {

                // Stopping test
                if ((R_norm < m_abstol) && (Qc_norm < m_abstol)) {
                    if (m_verbose) {
                        GetLog() << "+++   Newton procedure converged in " << i + 1 << " iterations.\n\n";
                    }
                    break;
                }
            }

            // Solve linear system for correction
            integrable->StateSolveCorrection(  //
                Dx, Dl, R, Qc,                 //
                0,                             // factor for  M
                0,                             // factor for  dF/dv
                -1.0,                          // factor for  dF/dx (the stiffness matrix)
                X, V, T,                       // not needed here
                false,                         // do not scatter Xnew Vnew T+dt before computing correction
                false,                         // full update? (not used, since no scatter)
                true                           // force a call to the solver's Setup() function
            );

            // Increment state (and constraint reactions)
            Xnew = X +  (Dx * step_factor);
            L += (Dl * step_factor);

            if (m_use_correction_test) {
                // Calculate actual correction in X
                ChState correction = Xnew - X;

                // Evaluate weights and correction WRMS norm
                ChVectorDynamic<> ewt = (m_reltol * Xnew.cwiseAbs() + m_abstol).cwiseInverse();
                double Dx_norm = correction.wrmsNorm(ewt);

                /*if (m_verbose) {
                    GetLog() << "---  Nonlinear statics iteration " << i << "  |Dx|_wrms = " << Dx_norm << "\n";
                }*/

                // Stopping test
                if (Dx_norm < 1) {
                    if (m_verbose) {
                        GetLog() << "+++  Newton procedure converged in " << i + 1 << " iterations.\n";
                        GetLog() << "     |R|_inf = " << R_norm << "  |Qc|_inf = " << Qc_norm << " |Dx|_wrms = " << Dx_norm << "\n\n";
                    }
                    X = Xnew;
                    break;
                }
            }

            X = Xnew;

        } // end inner loop for Newton iteration

    } // end outer loop incrementing external loads

    integrable->StateScatter(X, V, T, true);     // state -> system
    integrable->StateScatterReactions(L);  // -> system auxiliary data
}

void ChStaticNonLinearIncremental::SetCorrectionTolerance(double reltol, double abstol) {
    m_use_correction_test = true;
    m_reltol = reltol;
    m_abstol = abstol;
}

void ChStaticNonLinearIncremental::SetResidualTolerance(double tol) {
    m_use_correction_test = false;
    m_abstol = tol;
}

void ChStaticNonLinearIncremental::SetMaxIterationsNewton(int max_iters) {
    max_newton_iters = max_iters;
}

void ChStaticNonLinearIncremental::SetIncrementalSteps(int incr_steps) {
    m_incremental_steps = incr_steps;
}

void ChStaticNonLinearIncremental::SetAdaptiveNewtonON( int initial_delay, double growth_tolerance) {
    m_adaptive_newton = true;
    m_adaptive_newton_delay = initial_delay;
    m_adaptive_newton_tolerance = growth_tolerance;
}
void ChStaticNonLinearIncremental::SetAdaptiveNewtonOFF() {
    m_adaptive_newton = false;
}

void ChStaticNonLinearIncremental::SetNewtonDamping(double damping_factor) {
    m_newton_damping_factor = damping_factor;
}


// -----------------------------------------------------------------------------

ChStaticNonLinearRigidMotion::ChStaticNonLinearRigidMotion()
    : ChStaticAnalysis(),
      m_maxiters(20),
      m_incremental_steps(6),
      m_use_correction_test(true),
      m_reltol(1e-4),
      m_abstol(1e-8),
      m_verbose(false) {}

void ChStaticNonLinearRigidMotion::StaticAnalysis() {
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
    ChVectorDynamic<> Dl;
    ChVectorDynamic<> R;
    ChVectorDynamic<> Qc;
    Xnew.setZero(integrable->GetNcoords_x(), integrable);
    Dx.setZero(integrable->GetNcoords_v(), integrable);
    Dl.setZero(integrable->GetNconstr());
    R.setZero(integrable->GetNcoords_v());
    Qc.setZero(integrable->GetNconstr());
    L.setZero(integrable->GetNconstr());

    // Use Newton Raphson iteration, solving for the increments
    //      [ - dF/dx    Cq' ] [ Dx  ] = [ f + Cq*L ]
    //      [ Cq         0   ] [ Dl  ] = [-C ]

    for (int i = 0; i < m_maxiters; ++i) {
        integrable->StateScatter(X, V, T, true);  // state -> system
        R.setZero();
        Qc.setZero();
        integrable->LoadResidual_F(R, 1.0);
        // integrable->LoadResidual_Mv(R, V, 1.0); // V is zero
        integrable->LoadResidual_CqL(R, L, 1.0);  // Update the reaction forces
        integrable->LoadConstraint_C(Qc, 1.0);
        // Do not consider the rheonomic excitation because it constrains the DOFs in fact.
        // integrable->LoadConstraint_Ct(Qc, 1.0);

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
            Dx, Dl, R, Qc,                 //
            0,                             // factor for  M
            0,                             // factor for  dF/dv
            -1.0,                          // factor for  dF/dx (the stiffness matrix)
            X, V, T,                       // not needed here
            false,                         // do not scatter Xnew Vnew T+dt before computing correction
            false,                         // full update? (not used, since no scatter)
            true                           // force a call to the solver's Setup() function
        );

        Xnew = X + Dx;
        L += Dl;

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

        integrable->StateScatter(X, V, T, true);  // state -> system
        integrable->StateScatterReactions(L);     // -> system auxiliary data
    }

    integrable->StateScatter(X, V, T, true);  // state -> system
    integrable->StateScatterReactions(L);     // -> system auxiliary data
}


void ChStaticNonLinearRigidMotion::SetCorrectionTolerance(double reltol, double abstol) {
    m_use_correction_test = true;
    m_reltol = reltol;
    m_abstol = abstol;
}

void ChStaticNonLinearRigidMotion::SetResidualTolerance(double tol) {
    m_use_correction_test = false;
    m_abstol = tol;
}

void ChStaticNonLinearRigidMotion::SetMaxIterations(int max_iters) {
    m_maxiters = max_iters;
    if (m_incremental_steps > m_maxiters)
        m_incremental_steps = m_maxiters;
}

void ChStaticNonLinearRigidMotion::SetIncrementalSteps(int incr_steps) {
    m_incremental_steps = incr_steps;
    if (m_maxiters < m_incremental_steps)
        m_maxiters = m_incremental_steps;
}



}  // end namespace chrono
