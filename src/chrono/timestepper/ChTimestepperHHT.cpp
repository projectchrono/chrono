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

#include <cmath>

#include "chrono/timestepper/ChTimestepperHHT.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperHHT)
CH_UPCASTING(ChTimestepperHHT, ChTimestepperIIorder)
CH_UPCASTING(ChTimestepperHHT, ChTimestepperImplicit)

ChTimestepperHHT::ChTimestepperHHT(ChIntegrableIIorder* intgr) : ChTimestepperIIorder(intgr), ChTimestepperImplicit() {
    // Initialize method parameters with some numerical dissipation
    SetAlpha(-0.2);
}

void ChTimestepperHHT::SetAlpha(double val) {
    alpha = val;
    if (alpha < -CH_1_3)
        alpha = -CH_1_3;
    if (alpha > 0)
        alpha = 0;
    gamma = (1.0 - 2.0 * alpha) / 2.0;
    beta = std::pow((1.0 - alpha), 2) / 4.0;
}

void ChTimestepperHHT::InitializeStep() {
    // Setup state vectors
    integrable->StateSetup(X, V, A);

    // Setup auxiliary vectors
    Ds.setZero(integrable->GetNumCoordsVelLevel(), integrable);
    Dl.setZero(integrable->GetNumConstraints());
    Xnew.setZero(integrable->GetNumCoordsPosLevel(), integrable);
    Vnew.setZero(integrable->GetNumCoordsVelLevel(), integrable);
    Anew.setZero(integrable->GetNumCoordsVelLevel(), integrable);
    R.setZero(integrable->GetNumCoordsVelLevel());
    Rold.setZero(integrable->GetNumCoordsVelLevel());
    Qc.setZero(integrable->GetNumConstraints());
    L.setZero(integrable->GetNumConstraints());
    Lnew.setZero(integrable->GetNumConstraints());

    // State at current time T
    integrable->StateGather(X, V, T);        // state <- system
    integrable->StateGatherAcceleration(A);  // <- system
    integrable->StateGatherReactions(L);     // <- system
}

/*
// Performs a step of HHT (generalized alpha) implicit for II order systems
void ChTimestepperHHT::OnAdvance(double dt) {
    // Setup state vectors
    integrable->StateSetup(X, V, A);

    // Setup auxiliary vectors
    Ds.setZero(integrable->GetNumCoordsVelLevel(), integrable);
    Dl.setZero(integrable->GetNumConstraints());
    Xnew.setZero(integrable->GetNumCoordsPosLevel(), integrable);
    Vnew.setZero(integrable->GetNumCoordsVelLevel(), integrable);
    Anew.setZero(integrable->GetNumCoordsVelLevel(), integrable);
    R.setZero(integrable->GetNumCoordsVelLevel());
    Rold.setZero(integrable->GetNumCoordsVelLevel());
    Qc.setZero(integrable->GetNumConstraints());
    L.setZero(integrable->GetNumConstraints());
    Lnew.setZero(integrable->GetNumConstraints());

    // State at current time T
    integrable->StateGather(X, V, T);        // state <- system
    integrable->StateGatherAcceleration(A);  // <- system
    integrable->StateGatherReactions(L);     // <- system

    // Advance solution to time T+dt, possibly taking multiple steps
    double tfinal = T + dt;  // target final time

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
                std::cout << " +++HHT increase stepsize to " << h << std::endl;
        }
    } else {
        h = std::min(h, dt);
    }

    // Loop until reaching final time
    while (true) {
        Prepare();

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
            ////call_setup = CheckJacobianUpdateRequired(iteration, converged);

            if (verbose && (jacobian_update_method != JacobianUpdate::EVERY_ITERATION) && call_setup)
                std::cout << " HHT call Setup." << std::endl;

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
                std::cout << " HHT Newton converged in " << iteration << " iterations.";
                std::cout << "   Number successful steps: " << num_successful_steps;
                std::cout << "   T = " << T + h << "  h = " << h << std::endl;
            }

            // advance time (clamp to tfinal if close enough)
            T += h;
            if (std::abs(T - tfinal) < std::min(h_min, 1e-6)) {
                T = tfinal;
            }

            // set the state
            X = Xnew;
            V = Vnew;
            A = Anew;
            L = Lnew;

        } else if (!jacobian_is_current && jacobian_update_method != JacobianUpdate::NEVER) {
            // ------ Newton did not converge,
            //        but Jacobian is out-of-date and we can re-evaluate it

            // reset the count of successive successful steps
            num_successful_steps = 0;

            // re-attempt step with updated Jacobian
            if (verbose)
                std::cout << " HHT re-attempt step with updated matrix." << std::endl;

            // Force a Jacobian update for JacobianUpdate::AUTOMATIC
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
                std::cout << " HHT reduce stepsize to " << h << std::endl;

            // bail out if stepsize reaches minimum allowable
            if (h < h_min) {
                if (verbose)
                    std::cerr << " HHT at minimum stepsize. Exiting..." << std::endl;
                throw std::runtime_error("HHT: Reached minimum allowable step size.");
            }

            call_setup = true;

        } else {
            // ------ Newton did not converge,
            //        Jacobian is current or we are not allowed to update it, and we do not control stepsize

            // reset the count of successive successful steps
            num_successful_steps = 0;

            // accept solution as is and complete step
            if (verbose)
                std::cout << " HHT Newton terminated; advance T = " << T + h << std::endl;

            num_terminated++;

            // set the state
            T += h;
            X = Xnew;
            V = Vnew;
            A = Anew;
            L = Lnew;
        }

        // If successfully reaching end of step (this can only happen if Newton converged), break out of loop
        if (T >= tfinal)
            break;

        // Otherwise, scatter state, reset auxiliary vectors, and go back in the loop
        integrable->StateScatter(X, V, T, false);
        Rold.setZero();
        Anew.setZero(integrable->GetNumCoordsVelLevel(), integrable);
    }

    // Scatter state -> system doing a full update
    integrable->StateScatter(X, V, T, true);

    // Scatter auxiliary data (A and L) -> system
    integrable->StateScatterAcceleration(A);
    integrable->StateScatterReactions(L);
}
*/

void ChTimestepperHHT::ResetStep() {
    // Scatter state and reset auxiliary vectors
    integrable->StateScatter(X, V, T, false);
    Rold.setZero();
    Anew.setZero(integrable->GetNumCoordsVelLevel(), integrable);
}

void ChTimestepperHHT::AcceptStep() {
    X = Xnew;
    V = Vnew;
    A = Anew;
    L = Lnew;
}

void ChTimestepperHHT::FinalizeStep() {
    // Scatter state -> system doing a full update
    integrable->StateScatter(X, V, T, true);

    // Scatter auxiliary data (A and L) -> system
    integrable->StateScatterAcceleration(A);
    integrable->StateScatterReactions(L);
}

// Prepare attempting a step of size h (assuming a converged state at the current time t):
// - Initialize residual vector with terms at current time
// - Obtain a prediction at T+h for Newton using extrapolation from solution at current time.
// - If no step size control, start with zero acceleration guess (previous step not guaranteed to have converged)
// - Set the error weight vectors (using solution at current time)
void ChTimestepperHHT::PrepareStep() {
    if (step_control)
        Anew = A;
    Vnew = V + Anew * h;
    Xnew = X + Vnew * h + Anew * (h * h);
    integrable->LoadResidual_F(Rold, -alpha / (1.0 + alpha));       // -alpha/(1.0+alpha) * f_old
    integrable->LoadResidual_CqL(Rold, L, -alpha / (1.0 + alpha));  // -alpha/(1.0+alpha) * Cq'*l_old
    CalcErrorWeights(A, reltol, abstolS, ewtS);

    Lnew = L;

    CalcErrorWeights(L, reltol, abstolL, ewtL);
}

// Calculate a new iterate of the new state at time T+h:
// - Scatter the current estimate of the new state (the state at time T+h)
// - Set up and solve linear system
// - Calculate solution increment
// - Update the estimate of the new state (the state at time T+h)
//
// This is one Newton iteration to solve for a_new
//
// [ M - h*gamma*dF/dv - h^2*beta*dF/dx    Cq' ] [ Ds ] =
// [ Cq                                    0   ] [-Dl ]
//                [ -1/(1+alpha)*M*(a_new) + (f_new + Cq'*l_new) - (alpha/(1+alpha))(f_old + Cq'*l_old)]
//                [  1/(beta*h^2)*C                                                                    ]
//
void ChTimestepperHHT::Increment() {
    // Scatter the current estimate of state at time T+h
    integrable->StateScatter(Xnew, Vnew, T + h, false);

    // Initialize the two segments of the RHS
    R = Rold;      // terms related to state at time T
    Qc.setZero();  // zero

    // Set up linear system
    integrable->LoadResidual_F(R, 1.0);                      //  f_new
    integrable->LoadResidual_CqL(R, Lnew, 1.0);              //  Cq'*l_new
    integrable->LoadResidual_Mv(R, Anew, -1 / (1 + alpha));  // -1/(1+alpha)*M*a_new
    integrable->LoadConstraint_C(
        Qc, 1 / (beta * h * h), Qc_do_clamp,
        Qc_clamping);  //  Qc= 1/(beta*h^2)*C  (sign will be flipped later in StateSolveCorrection)

    // Solve linear system
    integrable->StateSolveCorrection(Ds, Dl, R, Qc,
                                     1 / (1 + alpha),    // factor for  M
                                     -h * gamma,         // factor for  dF/dv
                                     -h * h * beta,      // factor for  dF/dx
                                     Xnew, Vnew, T + h,  // not used here (force_scatter = false)
                                     false,              // do not scatter states
                                     false,              // full update? (not used, since no scatter)
                                     call_setup          // call the solver's Setup() function?
    );

    // Update estimate of state at T+h
    Lnew += Dl;  // not -= Dl because we assume StateSolveCorrection flips sign of Dl
    Anew += Ds;
    Xnew = X + V * h + A * (h * h * (0.5 - beta)) + Anew * (h * h * beta);
    Vnew = V + A * (h * (1.0 - gamma)) + Anew * (h * gamma);
}

void ChTimestepperHHT::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperHHT>();

    ChTimestepperImplicit::ArchiveOut(archive);

    // serialize all member data:
    archive << CHNVP(alpha);
    archive << CHNVP(beta);
    archive << CHNVP(gamma);
}

void ChTimestepperHHT::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChTimestepperHHT>();

    ChTimestepperImplicit::ArchiveIn(archive);

    // stream in all member data:
    archive >> CHNVP(alpha);
    archive >> CHNVP(beta);
    archive >> CHNVP(gamma);
}

}  // end namespace chrono