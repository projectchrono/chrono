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

ChTimestepperHHT::ChTimestepperHHT(ChIntegrableIIorder* intgr)
    : ChTimestepperIIorder(intgr),
      ChTimestepperImplicit(),
      step_control(true),
      maxiters_success(3),
      req_successful_steps(5),
      step_increase_factor(2),
      step_decrease_factor(0.5),
      h_min(1e-10),
      h(1e6),
      num_successful_steps(0) {
    SetAlpha(-0.2);  // default: some dissipation
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

// Performs a step of HHT (generalized alpha) implicit for II order systems
void ChTimestepperHHT::Advance(double dt) {
    // Setup main vectors
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
    numiters = 0;            // total number of NR iterations for this step
    numsetups = 0;
    numsolves = 0;

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
    bool jacobian_is_current = false;
    bool converged = false;

    while (true) {
        Prepare();

        // Perform Newton iterations to solve for state at T+h
        Ds_nrm_hist.fill(0.0);
        Dl_nrm_hist.fill(0.0);
        unsigned int iteration;
        for (iteration = 0; iteration < maxiters; iteration++) {
            // Check if a Jacobian update is required
            call_setup = CheckJacobianUpdateRequired(iteration, converged);

            if (verbose && (jacobian_update_method != JacobianUpdate::EVERY_ITERATION) && call_setup)
                std::cout << " HHT call Setup." << std::endl;

            // Solve linear system and increment state
            Increment();

            // If the Jacobian was updated at this iteration, mark it as up-to-date
            jacobian_is_current = call_setup;

            // Increment counters
            numiters++;
            numsolves++;
            if (call_setup)
                numsetups++;

            // Check convergence
            converged = CheckConvergence(iteration);
            if (converged)
                break;
        }

        if (converged) {
            // ------ NR converged

            // if the number of iterations was low enough, increase the count of successive
            // successful steps (for possible step increase)
            if (iteration < maxiters_success)
                num_successful_steps++;
            else
                num_successful_steps = 0;

            if (verbose) {
                std::cout << " HHT NR converged (" << num_successful_steps << ").";
                std::cout << "  T = " << T + h << "  h = " << h << std::endl;
            }

            // Advance time (clamp to tfinal if close enough)
            T += h;
            if (std::abs(T - tfinal) < std::min(h_min, 1e-6)) {
                T = tfinal;
            }

            // Set the state
            X = Xnew;
            V = Vnew;
            A = Anew;
            L = Lnew;

            /*
            } else if (!jacobian_is_current) {
                // ------ NR did not converge but the matrix was out-of-date

                // reset the count of successive successful steps
                num_successful_steps = 0;

                // re-attempt step with updated matrix
                if (verbose) {
                    std::cout << " HHT re-attempt step with updated matrix." << std::endl;
                }

                call_setup = true;
            */

        } else if (!step_control) {
            // ------ NR did not converge and we do not control stepsize

            // reset the count of successive successful steps
            num_successful_steps = 0;

            // accept solution as is and complete step
            if (verbose) {
                std::cout << " HHT NR terminated.";
                std::cout << "  T = " << T + h << "  h = " << h << std::endl;
            }

            T += h;
            X = Xnew;
            V = Vnew;
            A = Anew;
            L = Lnew;

        } else {
            // ------ NR did not converge

            // reset the count of successive successful steps
            num_successful_steps = 0;

            // decrease stepsize
            h *= step_decrease_factor;

            if (verbose)
                std::cout << " ---HHT reduce stepsize to " << h << std::endl;

            // bail out if stepsize reaches minimum allowable
            if (h < h_min) {
                if (verbose)
                    std::cerr << " HHT at minimum stepsize. Exiting..." << std::endl;
                throw std::runtime_error("HHT: Reached minimum allowable step size.");
            }
        }

        // If successfully reaching end of step ((this can only happen if Newton converged))
        if (T >= tfinal) {
            break;
        }

        // Go back in the loop: scatter state and reset temporary vector
        // Scatter state -> system
        integrable->StateScatter(X, V, T, false);
        Rold.setZero();
        Anew.setZero(integrable->GetNumCoordsVelLevel(), integrable);
    }

    // Scatter state -> system doing a full update
    integrable->StateScatter(X, V, T, true);

    // Scatter auxiliary data (A and L) -> system
    integrable->StateScatterAcceleration(A);
    integrable->StateScatterReactions(L);

    // Reset call_setup to false, in case it is modified from outside the integrator
    call_setup = false;
}

// Prepare attempting a step of size h (assuming a converged state at the current time t):
// - Initialize residual vector with terms at current time
// - Obtain a prediction at T+h for NR using extrapolation from solution at current time.
// - For ACCELERATION mode, if not using step size control, start with zero acceleration
//   guess (previous step not guaranteed to have converged)
// - Set the error weight vectors (using solution at current time)
void ChTimestepperHHT::Prepare() {
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
//                [ -1/(1+alpha)*M*(a_new) + (f_new +Cq*l_new) - (alpha/(1+alpha))(f_old +Cq*l_old)]
//                [  1/(beta*h^2)*C                                                                ]
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
    integrable->StateSolveCorrection(
        Ds, Dl, R, Qc,
        1 / (1 + alpha),    // factor for  M (note: it is 1 in Negrut paper but it was a typo)
        -h * gamma,         // factor for  dF/dv
        -h * h * beta,      // factor for  dF/dx
        Xnew, Vnew, T + h,  // not used here (force_scatter = false)
        false,              // do not scatter states
        false,              // full update? (not used, since no scatter)
        call_setup          // call Setup?
    );

    // Update estimate of state at t+h
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