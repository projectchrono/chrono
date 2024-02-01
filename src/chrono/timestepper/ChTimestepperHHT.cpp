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
CH_UPCASTING(ChTimestepperHHT, ChImplicitIterativeTimestepper)

ChTimestepperHHT::ChTimestepperHHT(ChIntegrableIIorder* intgr)
    : ChTimestepperIIorder(intgr),
      ChImplicitIterativeTimestepper(),
      step_control(true),
      maxiters_success(3),
      req_successful_steps(5),
      step_increase_factor(2),
      step_decrease_factor(0.5),
      h_min(1e-10),
      h(1e6),
      num_successful_steps(0),
      modified_Newton(true) {
    SetAlpha(-0.2);  // default: some dissipation
}

void ChTimestepperHHT::SetAlpha(double val) {
    alpha = val;
    if (alpha < -1.0 / 3.0)
        alpha = -1.0 / 3.0;
    if (alpha > 0)
        alpha = 0;
    gamma = (1.0 - 2.0 * alpha) / 2.0;
    beta = pow((1.0 - alpha), 2) / 4.0;
}

// Performs a step of HHT (generalized alpha) implicit for II order systems
void ChTimestepperHHT::Advance(const double dt) {
    // Downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // Setup main vectors
    mintegrable->StateSetup(X, V, A);

    // Setup auxiliary vectors
    Da.setZero(mintegrable->GetNcoords_a(), mintegrable);
    Dl.setZero(mintegrable->GetNconstr());
    Xnew.setZero(mintegrable->GetNcoords_x(), mintegrable);
    Vnew.setZero(mintegrable->GetNcoords_v(), mintegrable);
    Anew.setZero(mintegrable->GetNcoords_a(), mintegrable);
    R.setZero(mintegrable->GetNcoords_v());
    Rold.setZero(mintegrable->GetNcoords_v());
    Qc.setZero(mintegrable->GetNconstr());
    L.setZero(mintegrable->GetNconstr());
    Lnew.setZero(mintegrable->GetNconstr());

    // State at current time T
    mintegrable->StateGather(X, V, T);        // state <- system
    mintegrable->StateGatherAcceleration(A);  // <- system
    mintegrable->StateGatherReactions(L);     // <- system

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
        double new_h = ChMin(h * step_increase_factor, dt);
        if (new_h > h + h_min) {
            h = new_h;
            num_successful_steps = 0;
            if (verbose)
                GetLog() << " +++HHT increase stepsize to " << h << "\n";
        }
    } else {
        h = ChMin(h, dt);
    }

    // Monitor flags controlling whther or not the Newton matrix must be updated.
    // If using modified Newton, a matrix update occurs:
    //   - at the beginning of a step
    //   - on a stepsize decrease
    //   - if the Newton iteration does not converge with an out-of-date matrix
    // Otherwise, the matrix is updated at each iteration.
    matrix_is_current = false;
    call_setup = true;

    // Loop until reaching final time
    while (true) {
        Prepare(mintegrable);

        // Newton for state at T+h
        Da_nrm_hist.fill(0.0);
        Dl_nrm_hist.fill(0.0);
        bool converged = false;
        int it;

        for (it = 0; it < maxiters; it++) {
            if (verbose && modified_Newton && call_setup)
                GetLog() << " HHT call Setup.\n";

            // Solve linear system and increment state
            Increment(mintegrable);

            // Increment counters
            numiters++;
            numsolves++;
            if (call_setup) {
                numsetups++;
            }

            // If using modified Newton, do not call Setup again
            call_setup = !modified_Newton;

            // Check convergence
            converged = CheckConvergence(it);
            if (converged)
                break;
        }

        if (converged) {
            // ------ NR converged

            // if the number of iterations was low enough, increase the count of successive
            // successful steps (for possible step increase)
            if (it < maxiters_success)
                num_successful_steps++;
            else
                num_successful_steps = 0;

            if (verbose) {
                GetLog() << " HHT NR converged (" << num_successful_steps << ").";
                GetLog() << "  T = " << T + h << "  h = " << h << "\n";
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
            } else if (!matrix_is_current) {
                // ------ NR did not converge but the matrix was out-of-date

                // reset the count of successive successful steps
                num_successful_steps = 0;

                // re-attempt step with updated matrix
                if (verbose) {
                    GetLog() << " HHT re-attempt step with updated matrix.\n";
                }

                call_setup = true;
            */

        } else if (!step_control) {
            // ------ NR did not converge and we do not control stepsize

            // reset the count of successive successful steps
            num_successful_steps = 0;

            // accept solution as is and complete step
            if (verbose) {
                GetLog() << " HHT NR terminated.";
                GetLog() << "  T = " << T + h << "  h = " << h << "\n";
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
                GetLog() << " ---HHT reduce stepsize to " << h << "\n";

            // bail out if stepsize reaches minimum allowable
            if (h < h_min) {
                if (verbose)
                    GetLog() << " HHT at minimum stepsize. Exiting...\n";
                throw ChException("HHT: Reached minimum allowable step size.");
            }

            // force a matrix re-evaluation (due to change in stepsize)
            call_setup = true;
        }

        if (T >= tfinal) {
            break;
        }

        // Go back in the loop: scatter state and reset temporary vector
        // Scatter state -> system
        mintegrable->StateScatter(X, V, T, false);
        Rold.setZero();
        Anew.setZero(mintegrable->GetNcoords_a(), mintegrable);
    }

    // Scatter state -> system doing a full update
    mintegrable->StateScatter(X, V, T, true);

    // Scatter auxiliary data (A and L) -> system
    mintegrable->StateScatterAcceleration(A);
    mintegrable->StateScatterReactions(L);
}

// Prepare attempting a step of size h (assuming a converged state at the current time t):
// - Initialize residual vector with terms at current time
// - Obtain a prediction at T+h for NR using extrapolation from solution at current time.
// - For ACCELERATION mode, if not using step size control, start with zero acceleration
//   guess (previous step not guaranteed to have converged)
// - Set the error weight vectors (using solution at current time)
void ChTimestepperHHT::Prepare(ChIntegrableIIorder* integrable) {
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
// [ M - h*gamma*dF/dv - h^2*beta*dF/dx    Cq' ] [ Da ] =
// [ Cq                                    0   ] [-Dl ]
//                [ -1/(1+alpha)*M*(a_new) + (f_new +Cq*l_new) - (alpha/(1+alpha))(f_old +Cq*l_old)]
//                [  1/(beta*h^2)*C                                                                ]
//
void ChTimestepperHHT::Increment(ChIntegrableIIorder* integrable) {
    // Scatter the current estimate of state at time T+h
    integrable->StateScatter(Xnew, Vnew, T + h, false);

    // Initialize the two segments of the RHS
    R = Rold;      // terms related to state at time T
    Qc.setZero();  // zero

    // Set up linear system
    integrable->LoadResidual_F(R, 1.0);                                              //  f_new
    integrable->LoadResidual_CqL(R, Lnew, 1.0);                                      //  Cq'*l_new
    integrable->LoadResidual_Mv(R, Anew, -1 / (1 + alpha));                          // -1/(1+alpha)*M*a_new
    integrable->LoadConstraint_C(Qc, 1 / (beta * h * h), Qc_do_clamp, Qc_clamping);  //  Qc= 1/(beta*h^2)*C  (sign will be flipped later in StateSolveCorrection)

    // Solve linear system
    integrable->StateSolveCorrection(Da, Dl, R, Qc,
                                     1 / (1 + alpha),    // factor for  M (was 1 in Negrut paper ?!)
                                     -h * gamma,         // factor for  dF/dv
                                     -h * h * beta,      // factor for  dF/dx
                                     Xnew, Vnew, T + h,  // not used here (force_scatter = false)
                                     false,              // do not scatter states
                                     false,              // full update? (not used, since no scatter)
                                     call_setup          // call Setup?
    );

    // Update estimate of state at t+h
    Lnew += Dl;  // not -= Dl because we assume StateSolveCorrection flips sign of Dl
    Anew += Da;
    Xnew = X + V * h + A * (h * h * (0.5 - beta)) + Anew * (h * h * beta);
    Vnew = V + A * (h * (1.0 - gamma)) + Anew * (h * gamma);

    // If Setup was called at this iteration, mark the Newton matrix as up-to-date
    matrix_is_current = call_setup;
}

// Convergence test
bool ChTimestepperHHT::CheckConvergence(int it) {
    bool converged = false;

    // Declare convergence when either the residual is below the absolute tolerance or
    // the WRMS update norm is less than 1 (relative + absolute tolerance test)
    //    |R|_2 < atol
    // or |D|_WRMS < 1
    // Both states and Lagrange multipliers must converge.
    double R_nrm = R.norm();
    double Qc_nrm = Qc.norm();
    double Da_nrm = Da.wrmsNorm(ewtS);
    double Dl_nrm = Dl.wrmsNorm(ewtL);

    // Estimate convergence rate
    Da_nrm_hist[it % 3] = Da.norm();
    Dl_nrm_hist[it % 3] = Dl.norm();
    if (it < 2)
        convergence_rate = 1;
    else {
        double r21 = Da_nrm_hist[it % 3] / Da_nrm_hist[(it - 1) % 3];
        double r10 = Da_nrm_hist[(it - 1) % 3] / Da_nrm_hist[(it - 2) % 3];
        convergence_rate = std::log(r21) / std::log(r10);
    }

    if (verbose) {
        GetLog() << "   HHT iteration=" << numiters;
        GetLog() << "  |R|=" << R_nrm << "  |Qc|=" << Qc_nrm << "  |Da|=" << Da_nrm << "  |Dl|=" << Dl_nrm;
        if (it >= 2)
            GetLog() << "  Conv. rate = " << convergence_rate;
        GetLog() << "\n";
    }

    if ((R_nrm < abstolS && Qc_nrm < abstolL) || (Da_nrm < 1 && Dl_nrm < 1))
        converged = true;

    return converged;
}

// Calculate the error weight vector corresponding to the specified solution vector x,
// using the given relative and absolute tolerances.
void ChTimestepperHHT::CalcErrorWeights(const ChVectorDynamic<>& x, double rtol, double atol, ChVectorDynamic<>& ewt) {
    ewt = (rtol * x.cwiseAbs() + atol).cwiseInverse();
}

void ChTimestepperHHT::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperHHT>();
    // serialize parent class:
    ChTimestepperIIorder::ArchiveOut(archive);
    ChImplicitIterativeTimestepper::ArchiveOut(archive);
    // serialize all member data:
    archive << CHNVP(alpha);
    archive << CHNVP(beta);
    archive << CHNVP(gamma);
}

void ChTimestepperHHT::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChTimestepperHHT>();
    // deserialize parent class:
    ChTimestepperIIorder::ArchiveIn(archive);
    ChImplicitIterativeTimestepper::ArchiveIn(archive);
    // stream in all member data:
    archive >> CHNVP(alpha);
    archive >> CHNVP(beta);
    archive >> CHNVP(gamma);
}

}  // end namespace chrono