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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/core/ChFrame.h"
#include "chrono/timestepper/ChAssemblyAnalysis.h"

namespace chrono {

ChAssemblyAnalysis::ChAssemblyAnalysis(ChIntegrableIIorder& mintegrable) {
    integrable = &mintegrable;
    X.setZero(1, &mintegrable);
    V.setZero(1, &mintegrable);
    A.setZero(1, &mintegrable);
}

AssemblyAnalysis::ExitFlag ChAssemblyAnalysis::AssemblyAnalysis(int action, double dt) {
    ChVectorDynamic<> R;
    ChVectorDynamic<> Qc;
    double T;

    // Set up main vectors
    integrable->StateSetup(X, V, A);
    AssemblyAnalysis::ExitFlag exit_flag = AssemblyAnalysis::ExitFlag::NOT_CONVERGED;

    if (action & AssemblyAnalysis::Level::POSITION) {
        ChStateDelta Dx;

        for (m_last_num_iters = 0; m_last_num_iters < m_max_assembly_iters; m_last_num_iters++) {
            // Set up auxiliary vectors
            Dx.setZero(integrable->GetNumCoordsVelLevel(), GetIntegrable());
            R.setZero(integrable->GetNumCoordsVelLevel());
            Qc.setZero(integrable->GetNumConstraints());
            L.setZero(integrable->GetNumConstraints());

            integrable->StateGather(X, V, T);  // state <- system

            // Solve:
            //
            // [ M         Cq' ] [ dx  ] = [  0]
            // [ Cq        0   ] [ -l  ] = [ -C]

            integrable->LoadConstraint_C(Qc, 1.0, 0.0);  // sign flipped later in StateSolveCorrection

            if (Qc.lpNorm<Eigen::Infinity>() < m_abs_tol_residual) {
                exit_flag = AssemblyAnalysis::ExitFlag::ABSTOL_RESIDUAL;
                break;
            }

            integrable->StateSolveCorrection(Dx, L, R, Qc,
                                             1.0,      // factor for M
                                             0,        // factor for dF/dv
                                             0,        // factor for dF/dx (the stiffness matrix)
                                             X, V, T,  // not needed
                                             false,    // do not scatter Xnew Vnew T+dt before computing correction
                                             UpdateFlags::UPDATE_ALL_NO_VISUAL,    // no need for full update, since no scatter
                                             true,     // call the solver's Setup function
                                             true      // call the solver's Setup analyze phase
            );

            X += Dx;

            integrable->StateScatter(X, V, T, UpdateFlags::UPDATE_ALL);  // state -> system

            double last_update_norm = Dx.lpNorm<Eigen::Infinity>();

            if (last_update_norm < m_abs_tol_update) {
                exit_flag = AssemblyAnalysis::ExitFlag::ABSTOL_UPDATE;
                break;
            }

            if (Dx.size() == X.size()) {
                bool has_zero = (Dx.array() == 0.0).any(); // check against division by zero
                if (!has_zero) {
                    double rel_update_norm = Dx.cwiseQuotient(X).lpNorm<Eigen::Infinity>();

                    if (rel_update_norm == rel_update_norm && rel_update_norm < m_rel_tol_update) {
                        exit_flag = AssemblyAnalysis::ExitFlag::RELTOL_UPDATE;
                        break;
                    }
                }
            }
        }
    }

    if ((action & AssemblyAnalysis::Level::VELOCITY) || (action & AssemblyAnalysis::Level::ACCELERATION)) {

        if (!(action & AssemblyAnalysis::Level::POSITION)) {
            // no risk of not meeting any termination criteria, only position-level assembly can not converge
            exit_flag = AssemblyAnalysis::ExitFlag::SUCCESS;
        }

        // setup auxiliary vectors
        R.setZero(integrable->GetNumCoordsVelLevel());
        Qc.setZero(integrable->GetNumConstraints());
        L.setZero(integrable->GetNumConstraints());

        integrable->StateGather(X, V, T);  // state <- system

        // Perform a linearized semi-implicit Euler integration step
        //
        // [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ v_new  ] = [ M*(v_old) + dt*f]
        // [ Cq                           0   ] [ -dt*l  ] = [ -C/dt - Ct ]

        integrable->LoadResidual_F(R, dt);
        integrable->LoadResidual_Mv(R, V, 1.0);
        integrable->LoadConstraint_C(Qc, 1.0 / dt, 1.0, false);  // sign later flipped in StateSolveCorrection
        integrable->LoadConstraint_Ct(Qc, 1.0, 1.0 * dt);        // sign later flipped in StateSolveCorrection

        integrable->StateSolveCorrection(V, L, R, Qc,
                                         1.0,           // factor for  M
                                         -dt,           // factor for  dF/dv
                                         -dt * dt,      // factor for  dF/dx
                                         X, V, T + dt,  // not needed
                                         false,         // do not scatter Xnew Vnew T+dt before computing correction
                                         UpdateFlags::UPDATE_ALL_NO_VISUAL,  // no need for full update, since no scatter
                                         true,          // call the solver's Setup function
                                         true           // call the solver's Setup analyze phase
        );

        integrable->StateScatter(X, V, T, UpdateFlags::UPDATE_ALL);  // state -> system

        L *= (1.0 / dt);  // Note it is not -(1.0/dt) because we assume StateSolveCorrection already flips sign of L

        if (action & AssemblyAnalysis::Level::ACCELERATION) {
            // Accelerations and reactions consistent with the constraints at the now consistent (X, V).
            //
            // The velocity increment of the step above (new V minus old V, over dt) is NOT this acceleration: that step
            // enforces Cq(q) v_new = -C/dt - Ct with the Jacobian frozen at q, so its increment satisfies
            // Cq a = 0 and omits the quadratic-velocity term -(dCq/dt) v entirely (for a pendulum, the
            // whole centripetal acceleration), and so do its multipliers. At rest the two coincide, which
            // is why this went unnoticed (see issue #846).
            //
            // Rather than solving the smooth acceleration-level problem [M Cq'; Cq 0][a; -l] = [f; Qc]
            // directly (what ChIntegrableIIorder::StateSolveA does), a second step in the same DVI
            // (velocity-level) form is taken from (X, V), with the quadratic-velocity term added to its
            // constraint right-hand side:
            //
            // [ M    Cq' ] [ v2     ] = [ M*V + dt*f ]
            // [ Cq   0   ] [ -dt*l  ] = [ -C/dt - Ct - dt*Qc ]
            //
            // so that Cq (v2 - V)/dt = -Qc holds and, with the solver's multiplier convention (the unknown
            // is -dt*l and StateSolveCorrection returns it with the sign flipped), M (v2 - V)/dt = f + Cq' L
            // with L the returned vector; a = (v2 - V)/dt and L/dt are the acceleration-level solution,
            // scattered exactly as the velocity step scatters its own multipliers (the reaction-vector unit
            // test checks the resulting sign). Only the mass matrix enters this solve: it evaluates the
            // acceleration at the current state and is not an integration step, so the dF/dv and dF/dx
            // factors of the velocity step above must not be used (with them, dF/dv*V would leak into the
            // reported acceleration, e.g. doubling the deceleration of a damped body). Keeping the DVI
            // form matters for unilateral constraints and frictional contacts: their active set and
            // friction direction are decided from the velocity v2 (a separating contact stays inactive,
            // sliding friction opposes the sliding velocity), as in a dynamics step, which a smooth
            // acceleration-level complementarity cannot do. The quadratic-velocity term Qc = d^2C/dt^2
            // along V is obtained by central differencing of C with step Delta, as in
            // ChIntegrableIIorder::StateSolveA; this costs three additional full (non-visual) state
            // updates and puts a roundoff floor of about eps*scale(C)/Delta^2 on the reported
            // accelerations and reactions, also at rest (about 1e-4 * L for a distance constraint of
            // length L, with Delta = 1e-6). As in the velocity step, a = (v2 - V)/dt amplifies the
            // solver's residual in v2 by 1/dt; an iterative solver that is not converged tightly gives
            // poor accelerations here (a direct solver does not).
            const double Delta = 1e-6;

            ChVectorDynamic<> Qcq;
            Qcq.setZero(integrable->GetNumConstraints());
            integrable->LoadConstraint_C(Qcq, -2.0 / (Delta * Delta), -1.0 / Delta);

            ChStateDelta dx(V);
            dx *= Delta;
            ChState xdx(X.size(), GetIntegrable());

            integrable->StateIncrement(xdx, X, dx);
            integrable->StateScatter(xdx, V, T + Delta, UpdateFlags::UPDATE_ALL_NO_VISUAL);
            integrable->LoadConstraint_C(Qcq, 1.0 / (Delta * Delta), 1.0 / Delta);

            integrable->StateIncrement(xdx, X, -dx);
            integrable->StateScatter(xdx, V, T - Delta, UpdateFlags::UPDATE_ALL_NO_VISUAL);
            integrable->LoadConstraint_C(Qcq, 1.0 / (Delta * Delta), 0.0);

            integrable->StateScatter(X, V, T, UpdateFlags::UPDATE_ALL_NO_VISUAL);  // back to (X, V, T)

            R.setZero(integrable->GetNumCoordsVelLevel());
            Qc.setZero(integrable->GetNumConstraints());
            L.setZero(integrable->GetNumConstraints());

            integrable->LoadResidual_F(R, dt);
            integrable->LoadResidual_Mv(R, V, 1.0);
            integrable->LoadConstraint_C(Qc, 1.0 / dt, 1.0, false);  // sign later flipped in StateSolveCorrection
            integrable->LoadConstraint_Ct(Qc, 1.0, 1.0 * dt);        // sign later flipped in StateSolveCorrection
            Qc += dt * Qcq;                                           // quadratic-velocity term, same sign convention

            ChStateDelta V2(V);

            bool success = integrable->StateSolveCorrection(V2, L, R, Qc,
                                                            1.0,           // factor for  M
                                                            0,             // factor for  dF/dv (none: not an integration step, see above)
                                                            0,             // factor for  dF/dx (none)
                                                            X, V, T + dt,  // not needed
                                                            false,         // do not scatter Xnew Vnew T+dt before computing correction
                                                            UpdateFlags::UPDATE_ALL_NO_VISUAL,  // no need for full update, since no scatter
                                                            true,          // call the solver's Setup function
                                                            true           // call the solver's Setup analyze phase
            );

            if (!success) {
                // Do not publish accelerations or reactions from a failed solve; the system keeps the
                // auxiliary data it had (the velocity-level assembly above was still applied).
                return AssemblyAnalysis::ExitFlag::NOT_CONVERGED;
            }

            A = (V2 - V) * (1.0 / dt);
            L *= (1.0 / dt);  // Note it is not -(1.0/dt) because we assume StateSolveCorrection already flips sign of L

            integrable->StateScatterAcceleration(A);  // -> system auxiliary data
            integrable->StateScatterReactions(L);     // -> system auxiliary data
        }
    }

    return exit_flag;
}

}  // end namespace chrono
