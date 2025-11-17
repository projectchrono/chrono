// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include <cmath>

#include "chrono/timestepper/ChTimestepperImplicit.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {

// -----------------------------------------------------------------------------

CH_UPCASTING(ChTimestepperImplicit, ChTimestepper)

ChTimestepperImplicit::ChTimestepperImplicit()
    : jacobian_update_method(JacobianUpdate::EVERY_STEP),
      call_setup(true),
      call_analyze(true),
      jacobian_is_current(false),
      max_iters(6),
      reltol(1e-4),
      abstolS(1e-10),
      abstolL(1e-10),
      convergence_rate(0),
      accept_terminated(true),
      step_control(true),
      maxiters_success(3),
      req_successful_steps(5),
      step_increase_factor(2),
      step_decrease_factor(0.5),
      h_min(1e-10),
      h(1e6),
      num_successful_steps(0),
      num_step_iters(0),
      num_step_setups(0),
      num_step_solves(0),
      num_iters(0),
      num_setups(0),
      num_solves(0),
      num_terminated(0) {}

void ChTimestepperImplicit::SetJacobianUpdateMethod(JacobianUpdate method) {
    // If switching to JacobianUpdate::NEVER from a different strategy, force a call to Setup(),
    // including the analyze phase
    if (method == JacobianUpdate::NEVER && jacobian_update_method != JacobianUpdate::NEVER) {
        call_setup = true;
        call_analyze = true;
    }

    jacobian_update_method = method;
}

void ChTimestepperImplicit::Advance(double dt) {
    // Initialize step counters
    num_step_iters = 0;
    num_step_setups = 0;
    num_step_solves = 0;

    // Call the integrator-specific advance method
    try {
        OnAdvance(dt);
    } catch (const std::exception&) {
        throw;
    }

    // Update cumulative counters
    num_iters += num_step_iters;
    num_setups += num_step_setups;
    num_solves += num_step_solves;
}

// Time advance for an implicit integrator with support for adaptive, error controlled time step.
// Monitor flags controlling whether or not the Jacobian must be updated.
// If using JacobianUpdate::EVERY_ITERATION, a matrix update occurs:
//   - at every iteration
// If using JacobianUpdate::EVERY_STEP, a matrix update occurs:
//   - at the beginning of a step
//   - on a stepsize decrease
//   - if the Newton iteration does not converge with an out-of-date matrix
// If using JacobianUpdate::NEVER, a matrix update occurs:
//   - only at the beginning of the very first step
// If using JacobianUpdate::AUTOMATIC, a matrix update occurs:
//   - on a convergence failure with out of date Jacobian
//
void ChTimestepperImplicit::OnAdvance(double dt) {
    // On entry, call_setup is true only:
    // - at the first iteration on the first step
    // - after a call to SetJacobianUpdateMethod(JacobianUpdate::NEVER)
  
    // If the integrable object was modified, force a call to setup (including the analyze phase).
    // Otherwise, a potantial call to setup need not include the analyze phase.
    if (GetIntegrable()->StateModified()) {
        call_setup = true;
        call_analyze = true;
        if (verbose)
            cout << "  Force full Setup" << endl;
    } else {
        call_analyze = false;
    }

    // Let the concrete integrator initialze step
    InitializeStep();

    // Set time at step end
    // Solution will be advanced to tfinal, possibly taking multiple steps
    double tfinal = T + dt;

    // If we had a streak of successful steps, consider a stepsize increase.
    // Note that we never attempt a step larger than the specified dt value.
    // If step size control is disabled, always use h = dt.
    if (!step_control) {
        h = dt;
        num_successful_steps = 0;
    } else if (num_successful_steps >= req_successful_steps) {
        double new_h = std::min(h * step_increase_factor, dt);
        if (new_h > h + h_min) {
            h = new_h;
            num_successful_steps = 0;
            if (verbose)
                cout << "  Increase stepsize to " << h << endl;
        }
    } else {
        h = std::min(h, dt);
    }

    // Loop until reaching final time
    while (true) {
        PrepareStep();

        // Perform Newton iterations to solve for state at T+h
        Ds_nrm_hist.fill(0.0);
        Dl_nrm_hist.fill(0.0);
        bool converged = false;
        jacobian_is_current = false;

        unsigned int iteration;
        for (iteration = 0; iteration < max_iters; iteration++) {
            // Check if a Jacobian update is required
            if (jacobian_update_method == JacobianUpdate::EVERY_STEP && iteration == 0)
                call_setup = true;
            if (jacobian_update_method == JacobianUpdate::EVERY_ITERATION)
                call_setup = true;

            if (verbose && call_setup)
                cout << "  Call Setup" << endl;

            // Solve linear system and increment state
            Increment();

            // If the Jacobian was updated at this iteration, mark it as up-to-date
            if (call_setup)
                jacobian_is_current = true;

            // Increment counters
            num_step_iters++;
            num_step_solves++;
            if (call_setup)
                num_step_setups++;

            // Set call_setup to 'false' (for JacobianUpdate::NEVER)
            call_setup = false;

            // Check convergence
            bool pass = CheckConvergence(iteration);
            if (pass) {
                converged = true;
                break;
            }
        }

        if (converged) {
            // ------ Newton converged

            // if the number of iterations was low enough, increase the count of successive successful steps
            // (for possible step increase)
            if (iteration < maxiters_success)
                num_successful_steps++;
            else
                num_successful_steps = 0;

            if (verbose) {
                cout << "  Newton converged in " << iteration + 1 << " iterations.";
                cout << "   Number successful steps: " << num_successful_steps;
                cout << "   T = " << T + h << "  h = " << h << endl;
            }

            // advance time (clamp to tfinal if close enough)
            T += h;
            if (std::abs(T - tfinal) < std::min(h_min, 1e-6)) {
                T = tfinal;
            }

            // accept step
            AcceptStep();

        } else if (!jacobian_is_current && jacobian_update_method != JacobianUpdate::NEVER) {
            // ------ Newton did not converge,
            //        but Jacobian is out-of-date and we can re-evaluate it

            // reset the count of successive successful steps
            num_successful_steps = 0;

            // re-attempt step with updated Jacobian
            if (verbose)
                cout << "  Re-attempt step with updated matrix." << endl;

            // force a Jacobian update (for JacobianUpdate::AUTOMATIC)
            call_setup = true;

        } else if (step_control) {
            // ------ Newton did not converge,
            //        but we can reduce stepsize

            // reset the count of successive successful steps
            num_successful_steps = 0;

            // decrease stepsize
            h *= step_decrease_factor;

            // re-attempt step with smaller step-size
            if (verbose)
                cout << "  Reduce stepsize to " << h << endl;

            // bail out if stepsize reaches minimum allowable
            if (h < h_min) {
                cerr << "  [ERROR] Integrator at minimum stepsize. Exiting." << endl;
                throw std::runtime_error("Reached minimum allowable step size.");
            }

            call_setup = true;

        } else {
            // ------ Newton did not converge,
            //        Jacobian is current or we are not allowed to update it, and we do not control stepsize

            if (!accept_terminated) {
                cerr << "  [ERROR] Newton did not converge with up-to-date or non-modifiable Jacobian. Exiting." << endl;
                throw std::runtime_error("Newton did not converge with up-to-date Jacobian.");
            }

            // reset the count of successive successful steps
            num_successful_steps = 0;

            // accept solution as-is and complete step
            if (verbose)
                cout << "  Newton terminated; advance T = " << T + h << endl;

            num_terminated++;

            // advance time (clamp to tfinal if close enough)
            T += h;
            if (std::abs(T - tfinal) < std::min(h_min, 1e-6)) {
                T = tfinal;
            }

            // accept step
            AcceptStep();
        }

        // If successfully reaching end of step (this can only happen if Newton converged or was terminated),
        // break out of loop
        if (T >= tfinal)
            break;

        // Reset step and go back in the loop
        ResetStep();
    }

    FinalizeStep();

    // Reset call_setup to false, in case it is modified from outside the integrator
    call_setup = false;
}

// Check convergence of Newton process.
bool ChTimestepperImplicit::CheckConvergence(int it) {
    bool pass = false;

    // Declare convergence when either the residual is below the absolute tolerance or
    // the WRMS update norm is less than 1 (relative + absolute tolerance test)
    //    |R|_2 < atol
    // or |D|_WRMS < 1
    // Both states and Lagrange multipliers must converge.
    double R_nrm = R.norm();
    double Qc_nrm = Qc.norm();
    double Ds_nrm = Ds.wrmsNorm(ewtS);
    double Dl_nrm = Dl.wrmsNorm(ewtL);

    // Estimate convergence rate
    Ds_nrm_hist[it % 3] = Ds.norm();
    Dl_nrm_hist[it % 3] = Dl.norm();
    if (it < 2)
        convergence_rate = 1;
    else {
        double r21 = Ds_nrm_hist[it % 3] / Ds_nrm_hist[(it - 1) % 3];
        double r10 = Ds_nrm_hist[(it - 1) % 3] / Ds_nrm_hist[(it - 2) % 3];
        convergence_rate = std::log(r21) / std::log(r10);
    }

    if (verbose) {
        cout << "  Newton iteration: " << it;
        cout << "  |R| = " << R_nrm << "  |Qc| = " << Qc_nrm << "  |Ds| = " << Ds_nrm << "  |Dl| = " << Dl_nrm;
        if (it >= 2)
            cout << "  Conv. rate = " << convergence_rate;
        cout << endl;
    }

    if ((R_nrm < abstolS && Qc_nrm < abstolL) || (Ds_nrm < 1 && Dl_nrm < 1))
        pass = true;

    return pass;
}

// Calculate the error weight vector corresponding to the specified solution vector x,
// using the given relative and absolute tolerances
void ChTimestepperImplicit::CalcErrorWeights(const ChVectorDynamic<>& x,
                                             double rtol,
                                             double atol,
                                             ChVectorDynamic<>& ewt) {
    ewt = (rtol * x.cwiseAbs() + atol).cwiseInverse();
}

std::string ChTimestepperImplicit::GetJacobianUpdateMethodAsString(JacobianUpdate jacobian_update) {
    switch (jacobian_update) {
        case JacobianUpdate::EVERY_ITERATION:
            return "EVERY_ITERATION";
        case JacobianUpdate::EVERY_STEP:
            return "EVERY_STEP";
        case JacobianUpdate::NEVER:
            return "NEVER";
        case JacobianUpdate::AUTOMATIC:
            return "AUTOMATIC";
    }
    return "unknown";
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperEulerImplicit)
CH_UPCASTING(ChTimestepperEulerImplicit, ChTimestepperIIorder)
CH_UPCASTING(ChTimestepperEulerImplicit, ChTimestepperImplicit)

ChTimestepperEulerImplicit::ChTimestepperEulerImplicit(ChIntegrableIIorder* intgr)
    : ChTimestepperIIorder(intgr), ChTimestepperImplicit() {}

void ChTimestepperEulerImplicit::OnAdvance(double dt) {
    // setup main vectors
    integrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Ds.setZero(integrable->GetNumCoordsVelLevel(), integrable);
    Dl.setZero(integrable->GetNumConstraints());
    Xnew.setZero(integrable->GetNumCoordsPosLevel(), integrable);
    Vnew.setZero(integrable->GetNumCoordsVelLevel(), integrable);
    R.setZero(integrable->GetNumCoordsVelLevel());
    Qc.setZero(integrable->GetNumConstraints());
    L.setZero(integrable->GetNumConstraints());

    integrable->StateGather(X, V, T);  // state <- system

    // Extrapolate a prediction as warm start

    Xnew = X + V * dt;
    Vnew = V;  //+ A()*dt;

    // Use Newton iteration to solve for v_new
    //
    // [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ Ds     ] = [ M*(v_old - v_new) + dt*f + dt*Cq'*l ]
    // [ Cq                           0   ] [ -dt*Dl ] = [ -C/dt  ]
    //
    unsigned int iteration;
    for (iteration = 0; iteration < max_iters; iteration++) {
        integrable->StateScatter(Xnew, Vnew, T + dt, false);  // state -> system
        R.setZero();
        Qc.setZero();
        integrable->LoadResidual_F(R, dt);                // R  = dt*f
        integrable->LoadResidual_Mv(R, (V - Vnew), 1.0);  // R += M*(v_old - v_new)
        integrable->LoadResidual_CqL(R, L, dt);           // R += dt*Cq'*l
        integrable->LoadConstraint_C(Qc, 1.0 / dt, Qc_do_clamp,
                                     Qc_clamping);  // Qc= C/dt  (sign flipped later in StateSolveCorrection)

        if (verbose)
            cout << " Euler iteration=" << iteration << "  |R|=" << R.lpNorm<Eigen::Infinity>()
                      << "  |Qc|=" << Qc.lpNorm<Eigen::Infinity>() << endl;

        if ((R.lpNorm<Eigen::Infinity>() < abstolS) && (Qc.lpNorm<Eigen::Infinity>() < abstolL))
            break;

        integrable->StateSolveCorrection(  //
            Ds, Dl, R, Qc,                 //
            1.0,                           // factor for M
            -dt,                           // factor for dF/dv
            -dt * dt,                      // factor for dF/dx
            Xnew, Vnew, T + dt,            // not used here (scatter = false)
            false,                         // do not scatter update to Xnew Vnew T+dt before computing correction
            false,                         // no need for full update, since no scatter
            true,                          // always call the solver's Setup
            true                           // always call the solver's Setup analyze phase
        );

        num_step_iters++;
        num_step_setups++;
        num_step_solves++;

        Dl *= (1.0 / dt);  // Note it is not -(1.0/dt) because we assume StateSolveCorrection already flips sign of Dl
        L += Dl;

        Vnew += Ds;

        Xnew = X + Vnew * dt;
    }

    integrable->StateScatterAcceleration(
        (Vnew - V) * (1 / dt));  // -> system auxiliary data (i.e acceleration as measure, fits DVI/MDI)

    X = Xnew;
    V = Vnew;
    T += dt;

    integrable->StateScatter(X, V, T, true);  // state -> system
    integrable->StateScatterReactions(L);     // -> system auxiliary data
}

void ChTimestepperEulerImplicit::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperEulerImplicit>();

    ChTimestepperImplicit::ArchiveOut(archive);
}

void ChTimestepperEulerImplicit::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChTimestepperEulerImplicit>();

    ChTimestepperImplicit::ArchiveIn(archive);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperEulerImplicitLinearized)
CH_UPCASTING(ChTimestepperEulerImplicitLinearized, ChTimestepperIIorder)
CH_UPCASTING(ChTimestepperEulerImplicitLinearized, ChTimestepperImplicit)

ChTimestepperEulerImplicitLinearized::ChTimestepperEulerImplicitLinearized(ChIntegrableIIorder* intgr)
    : ChTimestepperIIorder(intgr), ChTimestepperImplicit() {}

// Performs a step of Euler implicit for II order systems using the Anitescu/Stewart/Trinkle single-iteration method,
// that is a bit like an implicit Euler where one performs only the first Newton corrector iteration. If the solver in
// StateSolveCorrection is a CCP complementarity solver, this is the typical Anitescu stabilized timestepper for DVIs.
void ChTimestepperEulerImplicitLinearized::OnAdvance(double dt) {
    // setup main vectors
    integrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Dl.setZero(integrable->GetNumConstraints());
    R.setZero(integrable->GetNumCoordsVelLevel());
    Qc.setZero(integrable->GetNumConstraints());
    L.setZero(integrable->GetNumConstraints());

    integrable->StateGather(X, V, T);  // state <- system

    integrable->StateGatherReactions(L);  // state <- system (may be needed for warm starting StateSolveCorrection)
    L *= dt;                              // because reactions = forces, here L = impulses

    Vold = V;

    // solve only 1st Newton step, using v_new = 0, so  Dv = v_new , therefore
    //
    // [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ Dv     ] = [ M*(v_old - v_new) + dt*f]
    // [ Cq                           0   ] [ -dt*Dl ] = [ -C/dt - Ct ]
    //
    // becomes the Anitescu/Trinkle timestepper:
    //
    // [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ v_new  ] = [ M*(v_old) + dt*f]
    // [ Cq                           0   ] [ -dt*l  ] = [ -C/dt - Ct ]

    integrable->LoadResidual_F(R, dt);       // R  = df*f
    integrable->LoadResidual_Mv(R, V, 1.0);  // R += M*(v_old)
    integrable->LoadConstraint_C(Qc, 1.0 / dt, Qc_do_clamp,
                                 Qc_clamping);  // Qc = C/dt  (sign will be flipped later in StateSolveCorrection)
    integrable->LoadConstraint_Ct(Qc, 1.0);     // Qc += Ct  (sign will be flipped later in StateSolveCorrection)

    integrable->StateSolveCorrection(  //
        V, L, R, Qc,                   //
        1.0,                           // factor for  M
        -dt,                           // factor for  dF/dv
        -dt * dt,                      // factor for  dF/dx
        X, V, T + dt,                  // not needed
        false,                         // do not scatter update to Xnew Vnew T+dt before computing correction
        false,                         // no need for full update, since no scatter
        true,                          // always call the solver's Setup
        true                           // always call the solver's Setup analyze phase
    );

    L *= (1.0 / dt);  // Note it is not -(1.0/dt) because we assume StateSolveCorrection already flips sign of Dl

    integrable->StateScatterAcceleration(
        (V - Vold) * (1 / dt));  // -> system auxiliary data (i.e acceleration as measure, fits DVI/MDI)

    X += V * dt;

    T += dt;

    integrable->StateScatter(X, V, T, true);  // state -> system
    integrable->StateScatterReactions(L);     // -> system auxiliary data
}

void ChTimestepperEulerImplicitLinearized::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperEulerImplicitLinearized>();

    ChTimestepperImplicit::ArchiveOut(archive);
}

void ChTimestepperEulerImplicitLinearized::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChTimestepperEulerImplicitLinearized>();

    ChTimestepperImplicit::ArchiveIn(archive);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperEulerImplicitProjected)
CH_UPCASTING(ChTimestepperEulerImplicitProjected, ChTimestepperIIorder)
CH_UPCASTING(ChTimestepperEulerImplicitProjected, ChTimestepperImplicit)

ChTimestepperEulerImplicitProjected::ChTimestepperEulerImplicitProjected(ChIntegrableIIorder* intgr)
    : ChTimestepperIIorder(intgr), ChTimestepperImplicit() {}

// Performs a step of Euler implicit for II order systems using a semi implicit Euler without constr.stabilization,
// followed by a projection. That is: a speed problem followed by a position problem that keeps constraint drifting
// 'closed' by using a projection. If the solver in StateSolveCorrection is a CCP complementarity solver, this is the
// Tasora stabilized timestepper for DVIs.
void ChTimestepperEulerImplicitProjected::OnAdvance(double dt) {
    // setup main vectors
    integrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Dl.setZero(integrable->GetNumConstraints());
    R.setZero(integrable->GetNumCoordsVelLevel());
    Qc.setZero(integrable->GetNumConstraints());
    L.setZero(integrable->GetNumConstraints());

    integrable->StateGather(X, V, T);  // state <- system

    Vold = V;

    // 1
    // Do a  Anitescu/Trinkle timestepper (it could be without the C/dt correction):
    //
    // [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ v_new  ] = [ M*(v_old) + dt*f]
    // [ Cq                           0   ] [ -dt*l  ] = [ -Ct ]

    integrable->LoadResidual_F(R, dt);                           // R  = dt*f
    integrable->LoadResidual_Mv(R, V, 1.0);                      // R += M*(v_old)
    integrable->LoadConstraint_C(Qc, 1.0 / dt, Qc_do_clamp, 0);  // Qc = C/dt  ...may be avoided...
    integrable->LoadConstraint_Ct(Qc, 1.0);  // Qc += Ct    (sign will be flipped later by StateSolveCorrection)

    integrable->StateSolveCorrection(  //
        V, L, R, Qc,                   //
        1.0,                           // factor for M
        -dt,                           // factor for dF/dv
        -dt * dt,                      // factor for dF/dx
        X, V, T + dt,                  // not needed
        false,                         // do not scatter update to Xnew Vnew T+dt before computing correction
        false,                         // no need for full update, since no scatter
        true,                          // always call the solver's Setup
        true                           // always call the solver's Setup analyze phase
    );

    L *= (1.0 / dt);  // Note it is not -(1.0/dt) because we assume StateSolveCorrection already flips sign of Dl

    integrable->StateScatterAcceleration(
        (V - Vold) * (1 / dt));  // -> system auxiliary data (i.e acceleration as measure, fits DVI/MDI)

    X += V * dt;

    T += dt;

    integrable->StateScatter(X, V, T, false);  // state -> system
    integrable->StateScatterReactions(L);      // -> system auxiliary data

    // 2
    // Do the position stabilization (single Newton step on constraints, with mass matrix as metric)

    Dl.setZero(integrable->GetNumConstraints());
    R.setZero(integrable->GetNumCoordsVelLevel());
    Qc.setZero(integrable->GetNumConstraints());
    L.setZero(integrable->GetNumConstraints());
    Vold.setZero(integrable->GetNumCoordsVelLevel(), integrable);

    //
    // [ M       Cq' ] [ dpos ] = [  0 ]
    // [ Cq       0  ] [ -l   ] = [ -C ]

    integrable->LoadConstraint_C(Qc, 1.0, false, 0);

    integrable->StateSolveCorrection(  //
        Vold, L, R, Qc,                //
        1.0,                           // factor for M
        0,                             // factor for dF/dv
        0,                             // factor for dF/dx
        X, V, T,                       // not needed
        false,                         // do not scatter update to Xnew Vnew T+dt before computing correction
        false,                         // no need for full update, since no scatter
        true,                          // always call the solver's Setup
        true                           // always call the solver's Setup analyze phase
    );

    X += Vold;  // here we used 'Vold' as 'dpos' to recycle Vold and avoid allocating a new vector dpos

    integrable->StateScatter(X, V, T, true);  // state -> system
}

void ChTimestepperEulerImplicitProjected::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperEulerImplicitProjected>();

    ChTimestepperImplicit::ArchiveOut(archive);
}

void ChTimestepperEulerImplicitProjected::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChTimestepperEulerImplicitProjected>();

    ChTimestepperImplicit::ArchiveIn(archive);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperTrapezoidal)
CH_UPCASTING(ChTimestepperTrapezoidal, ChTimestepperIIorder)
CH_UPCASTING(ChTimestepperTrapezoidal, ChTimestepperImplicit)

ChTimestepperTrapezoidal::ChTimestepperTrapezoidal(ChIntegrableIIorder* intgr)
    : ChTimestepperIIorder(intgr), ChTimestepperImplicit() {}

// Performs a step of trapezoidal implicit for II order systems.
// NOTE this is a modified version of the trapezoidal for DAE: the original derivation would lead to a scheme that
// produces oscillatory reactions in constraints, so this is a modified version that is first order in constraint
// reactions. Use damped HHT or damped Newmark for more advanced options.
void ChTimestepperTrapezoidal::OnAdvance(double dt) {
    // setup main vectors
    integrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Ds.setZero(integrable->GetNumCoordsVelLevel(), integrable);
    Dl.setZero(integrable->GetNumConstraints());
    Xnew.setZero(integrable->GetNumCoordsPosLevel(), integrable);
    Vnew.setZero(integrable->GetNumCoordsVelLevel(), integrable);
    L.setZero(integrable->GetNumConstraints());
    R.setZero(integrable->GetNumCoordsVelLevel());
    Rold.setZero(integrable->GetNumCoordsVelLevel());
    Qc.setZero(integrable->GetNumConstraints());

    integrable->StateGather(X, V, T);  // state <- system
    // integrable->StateGatherReactions(L); // <- system  assume l_old = 0;  otherwise DAE gives oscillatory reactions

    // extrapolate a prediction as a warm start

    Xnew = X + V * dt;
    Vnew = V;  // +A()*dt;

    // Use Newton iteration to solve for v_new
    //
    // [M-dt/2*dF/dv-dt^2/4*dF/dx  Cq'] [Ds      ] = [M*(v_old - v_new) + dt/2(f_old + f_new  + Cq*l_old + Cq*l_new)]
    // [Cq                          0 ] [-dt/2*Dl] = [-C/dt                                                         ]
    //
    integrable->LoadResidual_F(Rold, dt * 0.5);  // dt/2*f_old
    integrable->LoadResidual_Mv(Rold, V, 1.0);   // M*v_old
    // integrable->LoadResidual_CqL(Rold, L, dt*0.5); // dt/2*l_old   assume L_old = 0

    unsigned int iteration;
    for (iteration = 0; iteration < max_iters; iteration) {
        integrable->StateScatter(Xnew, Vnew, T + dt, false);  // state -> system
        R = Rold;
        Qc.setZero();
        integrable->LoadResidual_F(R, dt * 0.5);       // + dt/2*f_new
        integrable->LoadResidual_Mv(R, Vnew, -1.0);    // - M*v_new
        integrable->LoadResidual_CqL(R, L, dt * 0.5);  // + dt/2*Cq*l_new
        integrable->LoadConstraint_C(Qc, 1.0 / dt, Qc_do_clamp,
                                     Qc_clamping);  // Qc= C/dt  (sign will be flipped later in StateSolveCorrection)

        if (verbose)
            cout << " Trapezoidal iteration=" << iteration << "  |R|=" << R.lpNorm<Eigen::Infinity>()
                      << "  |Qc|=" << Qc.lpNorm<Eigen::Infinity>() << endl;

        if ((R.lpNorm<Eigen::Infinity>() < abstolS) && (Qc.lpNorm<Eigen::Infinity>() < abstolL))
            break;

        integrable->StateSolveCorrection(  //
            Ds, Dl, R, Qc,                 //
            1.0,                           // factor for M
            -dt * 0.5,                     // factor for dF/dv
            -dt * dt * 0.25,               // factor for dF/dx
            Xnew, Vnew, T + dt,            // not used here (scatter = false)
            false,                         // do not scatter update to Xnew Vnew T+dt before computing correction
            false,                         // no need for full update, since no scatter
            true,                          // always call the solver's Setup
            true                           // always call the solver's Setup analyze phase
        );

        num_step_iters++;
        num_step_setups++;
        num_step_solves++;

        Dl *= (2.0 / dt);  // Note it is not -(2.0/dt) because we assume StateSolveCorrection already flips sign of Dl
        L += Dl;

        Vnew += Ds;

        Xnew = X + ((Vnew + V) * (dt * 0.5));  // Xnew = Xold + h/2(Vnew+Vold)
    }

    integrable->StateScatterAcceleration(
        (Vnew - V) * (1 / dt));  // -> system auxiliary data (i.e acceleration as measure, fits DVI/MDI)

    X = Xnew;
    V = Vnew;
    T += dt;

    integrable->StateScatter(X, V, T, true);  // state -> system
    integrable->StateScatterReactions(L *=
                                      0.5);  // -> system auxiliary data   (*=0.5 cause we used the hack of l_old = 0)
}

void ChTimestepperTrapezoidal::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperTrapezoidal>();

    ChTimestepperImplicit::ArchiveOut(archive);
}

void ChTimestepperTrapezoidal::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChTimestepperTrapezoidal>();

    ChTimestepperImplicit::ArchiveIn(archive);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperTrapezoidalLinearized)
CH_UPCASTING(ChTimestepperTrapezoidalLinearized, ChTimestepperIIorder)
CH_UPCASTING(ChTimestepperTrapezoidalLinearized, ChTimestepperImplicit)

ChTimestepperTrapezoidalLinearized::ChTimestepperTrapezoidalLinearized(ChIntegrableIIorder* intgr)
    : ChTimestepperIIorder(intgr), ChTimestepperImplicit() {}

void ChTimestepperTrapezoidalLinearized::OnAdvance(double dt) {
    // setup main vectors
    integrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Ds.setZero(integrable->GetNumCoordsVelLevel(), integrable);
    Dl.setZero(integrable->GetNumConstraints());
    Xnew.setZero(integrable->GetNumCoordsPosLevel(), integrable);
    Vnew.setZero(integrable->GetNumCoordsVelLevel(), integrable);
    L.setZero(integrable->GetNumConstraints());
    R.setZero(integrable->GetNumCoordsVelLevel());
    Rold.setZero(integrable->GetNumCoordsVelLevel());
    Qc.setZero(integrable->GetNumConstraints());

    integrable->StateGather(X, V, T);  // state <- system
    // integrable->StateGatherReactions(L); // <- system  assume l_old = 0;

    // extrapolate a prediction as a warm start

    Xnew = X + V * dt;
    Vnew = V;

    // Use Newton to solve for v_new
    //
    // [M-dt/2*dF/dv-dt^2/4*dF/dx  Cq'] [Ds      ] = [M*(v_old - v_new) + dt/2(f_old + f_new  + Cq*l_old + Cq*l_new)]
    // [Cq                          0 ] [-dt/2*Dl] = [-C/dt                                                         ]

    integrable->LoadResidual_F(Rold, dt * 0.5);  // dt/2*f_old
    integrable->LoadResidual_Mv(Rold, V, 1.0);   // M*v_old
    // integrable->LoadResidual_CqL(Rold, L, dt*0.5); // dt/2*l_old  assume l_old = 0;

    integrable->StateScatter(Xnew, Vnew, T + dt, false);  // state -> system
    R = Rold;
    Qc.setZero();
    integrable->LoadResidual_F(R, dt * 0.5);     // + dt/2*f_new
    integrable->LoadResidual_Mv(R, Vnew, -1.0);  // - M*v_new
    // integrable->LoadResidual_CqL(R, L, dt*0.5); // + dt/2*Cq*l_new  assume l_old = 0;
    integrable->LoadConstraint_C(Qc, 1.0 / dt, Qc_do_clamp,
                                 Qc_clamping);  // Qc= C/dt  (sign will be flipped later in StateSolveCorrection)

    integrable->StateSolveCorrection(  //
        Ds, Dl, R, Qc,                 //
        1.0,                           // factor for M
        -dt * 0.5,                     // factor for dF/dv
        -dt * dt * 0.25,               // factor for dF/dx
        Xnew, Vnew, T + dt,            // not used here (scatter = false)
        false,                         // do not scatter update to Xnew Vnew T+dt before computing correction
        false,                         // no need for full update, since no scatter
        true,                          // always call the solver's Setup
        true                           // always call the solver's Setup analyze phase
    );

    num_step_iters = 1;
    num_step_setups = 1;
    num_step_solves = 1;

    Dl *= (2 / dt);  // not -(2/dt) because StateSolveCorrection already flips sign of Dl

    Vnew += Ds;
    L += Dl;

    Ds *= (1 / dt);
    L *= 0.5;  // because L_old = 0

    Xnew = X + (Vnew + V) * (dt * 0.5);

    X = Xnew;
    V = Vnew;
    T += dt;

    integrable->StateScatter(X, V, T, true);   // state -> system
    integrable->StateScatterAcceleration(Ds);  // -> system auxiliary data (accelerations)
    integrable->StateScatterReactions(L);      // -> system auxiliary data (Lagrange multipliers)
}

void ChTimestepperTrapezoidalLinearized::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperTrapezoidalLinearized>();

    ChTimestepperImplicit::ArchiveOut(archive);
}

void ChTimestepperTrapezoidalLinearized::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChTimestepperTrapezoidalLinearized>();

    ChTimestepperImplicit::ArchiveIn(archive);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperNewmark)
CH_UPCASTING(ChTimestepperNewmark, ChTimestepperIIorder)
CH_UPCASTING(ChTimestepperNewmark, ChTimestepperImplicit)

ChTimestepperNewmark::ChTimestepperNewmark(ChIntegrableIIorder* intgr)
    : ChTimestepperIIorder(intgr), ChTimestepperImplicit() {
    // Default method parameters with some damping that also work with DAE constraints
    SetGammaBeta(0.6, 0.3);
}

void ChTimestepperNewmark::SetGammaBeta(double gamma_val, double beta_val) {
    gamma = gamma_val;
    if (gamma < 0.5)
        gamma = 0.5;
    if (gamma > 1)
        gamma = 1;
    beta = beta_val;
    if (beta < 0)
        beta = 0;
    if (beta > 1)
        beta = 1;
}

void ChTimestepperNewmark::OnAdvance(double dt) {
    // setup main vectors
    integrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Ds.setZero(integrable->GetNumCoordsVelLevel(), GetIntegrable());
    Dl.setZero(integrable->GetNumConstraints());
    Xnew.setZero(integrable->GetNumCoordsPosLevel(), integrable);
    Vnew.setZero(integrable->GetNumCoordsVelLevel(), integrable);
    Anew.setZero(integrable->GetNumCoordsVelLevel(), integrable);
    R.setZero(integrable->GetNumCoordsVelLevel());
    Rold.setZero(integrable->GetNumCoordsVelLevel());
    Qc.setZero(integrable->GetNumConstraints());
    L.setZero(integrable->GetNumConstraints());

    integrable->StateGather(X, V, T);  // state <- system
    integrable->StateGatherAcceleration(A);

    // extrapolate a prediction as a warm start

    Vnew = V;
    Xnew = X + Vnew * dt;

    // Use Newton to solve for a_new
    //
    // [ M - dt*gamma*dF/dv - dt^2*beta*dF/dx    Cq' ] [ Ds   ] = [ -M*(a_new) + f_new + Cq*l_new ]
    // [ Cq                                      0   ] [ -Dl  ] = [ -1/(beta*dt^2)*C              ]
    //
    call_setup = true;
    call_analyze = true;

    unsigned int iteration;
    for (iteration = 0; iteration < max_iters; ++iteration) {
        integrable->StateScatter(Xnew, Vnew, T + dt, false);  // state -> system

        R.setZero(integrable->GetNumCoordsVelLevel());
        Qc.setZero(integrable->GetNumConstraints());
        integrable->LoadResidual_F(R, 1.0);          //  f_new
        integrable->LoadResidual_CqL(R, L, 1.0);     //   Cq'*l_new
        integrable->LoadResidual_Mv(R, Anew, -1.0);  //  - M*a_new
        integrable->LoadConstraint_C(
            Qc, (1.0 / (beta * dt * dt)), Qc_do_clamp,
            Qc_clamping);  //  Qc = 1/(beta*dt^2)*C  (sign will be flipped later in StateSolveCorrection)

        if (verbose)
            cout << " Newmark iteration=" << iteration << "  |R|=" << R.lpNorm<Eigen::Infinity>()
                      << "  |Qc|=" << Qc.lpNorm<Eigen::Infinity>() << endl;

        if ((R.lpNorm<Eigen::Infinity>() < abstolS) && (Qc.lpNorm<Eigen::Infinity>() < abstolL)) {
            if (verbose) {
                cout << " Newmark NR converged (" << iteration << ")."
                          << "  T = " << T + dt << "  h = " << dt << endl;
            }
            break;
        }

        if (verbose && jacobian_update_method != JacobianUpdate::EVERY_ITERATION && call_setup)
            cout << " Newmark call Setup." << endl;

        integrable->StateSolveCorrection(  //
            Ds, Dl, R, Qc,                 //
            1.0,                           // factor for M
            -dt * gamma,                   // factor for dF/dv
            -dt * dt * beta,               // factor for dF/dx
            Xnew, Vnew, T + dt,            // not used here (scatter = false)
            false,                         // do not scatter update to Xnew Vnew T+dt before computing correction
            false,                         // no need for full update, since no scatter
            call_setup,                    // if true, call the solver's Setup function
            call_analyze                   // if true, call the solver's Setup analyze phase
        );

        num_step_iters++;
        num_step_solves++;
        if (call_setup)
            num_step_setups++;

        // If using modified Newton, do not call Setup again
        call_setup = (jacobian_update_method == JacobianUpdate::EVERY_ITERATION);

        L += Dl;  // Note it is not -= Dl because we assume StateSolveCorrection flips sign of Dl
        Anew += Ds;

        Xnew = X + V * dt + A * (dt * dt * (0.5 - beta)) + Anew * (dt * dt * beta);

        Vnew = V + A * (dt * (1.0 - gamma)) + Anew * (dt * gamma);
    }

    X = Xnew;
    V = Vnew;
    A = Anew;
    T += dt;

    integrable->StateScatter(X, V, T, true);  // state -> system
    integrable->StateScatterAcceleration(A);  // -> system auxiliary data
    integrable->StateScatterReactions(L);     // -> system auxiliary data
}

void ChTimestepperNewmark::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperNewmark>();

    ChTimestepperImplicit::ArchiveOut(archive);

    // serialize all member data:
    archive << CHNVP(beta);
    archive << CHNVP(gamma);
}

void ChTimestepperNewmark::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChTimestepperNewmark>();

    ChTimestepperImplicit::ArchiveIn(archive);

    // stream in all member data:
    archive >> CHNVP(beta);
    archive >> CHNVP(gamma);
}

}  // end namespace chrono
