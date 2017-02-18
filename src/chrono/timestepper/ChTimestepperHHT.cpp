// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

ChTimestepperHHT::ChTimestepperHHT(ChIntegrableIIorder* mintegrable)
    : ChTimestepperIIorder(mintegrable),
      ChImplicitIterativeTimestepper(),
      mode(ACCELERATION),
      scaling(false),
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

void ChTimestepperHHT::SetAlpha(double malpha) {
    alpha = malpha;
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
    Da.Reset(mintegrable->GetNcoords_a(), mintegrable);
    if (mode == POSITION) {
        Xprev.Reset(mintegrable->GetNcoords_x(), mintegrable);
        Dx.Reset(mintegrable->GetNcoords_v(), mintegrable);
    }
    Dl.Reset(mintegrable->GetNconstr());
    Xnew.Reset(mintegrable->GetNcoords_x(), mintegrable);
    Vnew.Reset(mintegrable->GetNcoords_v(), mintegrable);
    Anew.Reset(mintegrable->GetNcoords_a(), mintegrable);
    R.Reset(mintegrable->GetNcoords_v());
    Rold.Reset(mintegrable->GetNcoords_v());
    Qc.Reset(mintegrable->GetNconstr());
    L.Reset(mintegrable->GetNconstr());
    Lnew.Reset(mintegrable->GetNconstr());

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
    while (T < tfinal) {
        double scaling_factor = scaling ? beta * h * h : 1;
        Prepare(mintegrable, scaling_factor);

        // Newton-Raphson for state at T+h
        bool converged;
        int it;

        for (it = 0; it < maxiters; it++) {
            if (verbose && modified_Newton && call_setup)
                GetLog() << " HHT call Setup.\n";

            // Solve linear system and increment state
            Increment(mintegrable, scaling_factor);

            // Increment counters
            numiters++;
            numsolves++;
            if (call_setup) {
                numsetups++;
            }

            // If using modified Newton, do not call Setup again
            call_setup = !modified_Newton;

            // Check convergence
            converged = CheckConvergence(scaling_factor);
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

            // if needed, adjust stepsize to reach exactly tfinal
            if (std::abs(T + h - tfinal) < 1e-6)
                h = tfinal - T;

            // advance time and set the state
            T += h;
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

        // Scatter state -> system
        mintegrable->StateScatter(X, V, T);

        // In case we go back in the loop
        //// TODO: this is wasted work if we DO NOT go back
        Rold.Reset();
        Anew.Reset(mintegrable->GetNcoords_a(), mintegrable);
    }

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
void ChTimestepperHHT::Prepare(ChIntegrableIIorder* integrable, double scaling_factor) {
    switch (mode) {
        case ACCELERATION:
            if (step_control)
                Anew = A;
            Vnew = V + Anew * h;
            Xnew = X + Vnew * h + Anew * (h * h);
            integrable->LoadResidual_F(Rold, -alpha / (1.0 + alpha));       // -alpha/(1.0+alpha) * f_old
            integrable->LoadResidual_CqL(Rold, L, -alpha / (1.0 + alpha));  // -alpha/(1.0+alpha) * Cq'*l_old
            CalcErrorWeights(A, reltol, abstolS, ewtS);
            break;
        case POSITION:
            Xnew = X;
            Xprev = X;
            Vnew = V * (-(gamma / beta - 1.0)) - A * (h * (gamma / (2.0 * beta) - 1.0));
            Anew = V * (-1.0 / (beta * h)) - A * (1.0 / (2.0 * beta) - 1.0);
            integrable->LoadResidual_F(Rold, -(alpha / (1.0 + alpha)) * scaling_factor);  // -alpha/(1.0+alpha) * f_old
            integrable->LoadResidual_CqL(Rold, L,
                                         -(alpha / (1.0 + alpha)) * scaling_factor);  // -alpha/(1.0+alpha) * Cq'*l_old
            CalcErrorWeights(X, reltol, abstolS, ewtS);
            break;
    }

    Lnew = L;

    CalcErrorWeights(L, reltol, abstolL, ewtL);
}

// Calculate a new iterate of the new state at time T+h:
// - Scatter the current estimate of the new state (the state at time T+h)
// - Set up and solve linear system
// - Calculate solution increment
// - Update the estimate of the new state (the state at time T+h)
//
// This is one iteration of Newton-Raphson to solve for a_new
//
// [ M - dt*gamma*dF/dv - dt^2*beta*dF/dx    Cq' ] [ Da ] =
// [ Cq                                      0   ] [ Dl ]
//                [ -1/(1+alpha)*M*(a_new) + (f_new +Cq*l_new) - (alpha/(1+alpha))(f_old +Cq*l_old)]
//                [  1/(beta*dt^2)*C                                                               ]
//
void ChTimestepperHHT::Increment(ChIntegrableIIorder* integrable, double scaling_factor) {
    // Scatter the current estimate of state at time T+h
    integrable->StateScatter(Xnew, Vnew, T + h);

    // Initialize the two segments of the RHS
    R = Rold;    // terms related to state at time T
    Qc.Reset();  // zero

    switch (mode) {
        case ACCELERATION:
            // Set up linear system
            integrable->LoadResidual_F(R, 1.0);                                              //  f_new
            integrable->LoadResidual_CqL(R, Lnew, 1.0);                                      //  Cq'*l_new
            integrable->LoadResidual_Mv(R, Anew, -1 / (1 + alpha));                          // -1/(1+alpha)*M*a_new
            integrable->LoadConstraint_C(Qc, 1 / (beta * h * h), Qc_do_clamp, Qc_clamping);  //  1/(beta*dt^2)*C

            // Solve linear system
            integrable->StateSolveCorrection(Da, Dl, R, Qc,
                                             1 / (1 + alpha),    // factor for  M (was 1 in Negrut paper ?!)
                                             -h * gamma,         // factor for  dF/dv
                                             -h * h * beta,      // factor for  dF/dx
                                             Xnew, Vnew, T + h,  // not used here (force_scatter = false)
                                             false,              // do not scatter states
                                             call_setup          // call Setup?
                                             );

            // Update estimate of state at t+h
            Lnew += Dl;  // not -= Dl because we assume StateSolveCorrection flips sign of Dl
            Anew += Da;
            Xnew = X + V * h + A * (h * h * (0.5 - beta)) + Anew * (h * h * beta);
            Vnew = V + A * (h * (1.0 - gamma)) + Anew * (h * gamma);

            break;

        case POSITION:
            // Set up linear system
            integrable->LoadResidual_F(R, scaling_factor);                            //  f_new
            integrable->LoadResidual_CqL(R, Lnew, scaling_factor);                    //  Cq'*l_new
            integrable->LoadResidual_Mv(R, Anew, -1 / (1 + alpha) * scaling_factor);  // -1/(1+alpha)*M*a_new
            integrable->LoadConstraint_C(Qc, 1.0, Qc_do_clamp, Qc_clamping);          //  1/(beta*dt^2)*C

            // Solve linear system
            integrable->StateSolveCorrection(Da, Dl, R, Qc,
                                             scaling_factor / ((1 + alpha) * beta * h * h),  // factor for  M
                                             -scaling_factor * gamma / (beta * h),           // factor for  dF/dv
                                             -scaling_factor,                                // factor for  dF/dx
                                             Xnew, Vnew, T + h,  // not used here(force_scatter = false)
                                             false,              // do not scatter states
                                             call_setup          // call Setup?
                                             );

            // Update estimate of state at t+h
            Lnew += Dl * (1.0 / scaling_factor);  // not -= Dl because we assume StateSolveCorrection flips sign of Dl
            Dx += Da;
            Xnew = (X + Dx);
            Vnew = V * (-(gamma / beta - 1.0)) - A * (h * (gamma / (2.0 * beta) - 1.0));
            Vnew += Dx * (gamma / (beta * h));
            Anew = -V * (1.0 / (beta * h)) - A * (1.0 / (2.0 * beta) - 1.0);
            Anew += Dx * (1.0 / (beta * h * h));

            break;
    }

    // If Setup was called at this iteration, mark the Newton matrix as up-to-date
    matrix_is_current = call_setup;
}

// Convergence test
bool ChTimestepperHHT::CheckConvergence(double scaling_factor) {
    bool converged = false;

    switch (mode) {
        case ACCELERATION: {
            // Declare convergence when either the residual is below the absolute tolerance or
            // the WRMS update norm is less than 1 (relative + absolute tolerance test)
            //    |R|_2 < atol
            // or |D|_WRMS < 1
            // Both states and Lagrange multipliers must converge.
            double R_nrm = R.NormTwo();
            double Qc_nrm = Qc.NormTwo();
            double Da_nrm = Da.NormWRMS(ewtS);
            double Dl_nrm = Dl.NormWRMS(ewtL);

            if (verbose) {
                GetLog() << " HHT iteration=" << numiters << "  |R|=" << R_nrm << "  |Qc|=" << Qc_nrm
                         << "  |Da|=" << Da_nrm << "  |Dl|=" << Dl_nrm << "  N = " << R.GetLength()
                         << "  M = " << Qc.GetLength() << "\n";
            }

            if ((R_nrm < abstolS && Qc_nrm < abstolL) || (Da_nrm < 1 && Dl_nrm < 1))
                converged = true;

            break;
        }
        case POSITION: {
            // Declare convergence when the WRMS norm of the update is less than 1
            // (relative + absolute tolerance test).
            // Note that the scaling factor must be properly included in the update to
            // the Lagrange multipliers.
            double Dx_nrm = (Xnew - Xprev).NormWRMS(ewtS);
            Xprev = Xnew;

            double Dl_nrm = Dl.NormWRMS(ewtL);
            Dl_nrm /= scaling_factor;

            if (verbose) {
                GetLog() << " HHT iteration=" << numiters << "  |Dx|=" << Dx_nrm << "  |Dl|=" << Dl_nrm << "\n";
            }

            if (Dx_nrm < 1 && Dl_nrm < 1)
                converged = true;

            break;
        }
    }

    return converged;
}

// Calculate the error weight vector correspondiong to the specified solution vector x,
// using the given relative and absolute tolerances.
void ChTimestepperHHT::CalcErrorWeights(const ChVectorDynamic<>& x, double rtol, double atol, ChVectorDynamic<>& ewt) {
    ewt.Reset(x.GetLength());
    for (int i = 0; i < x.GetLength(); ++i) {
        ewt.ElementN(i) = 1.0 / (rtol * std::abs(x.ElementN(i)) + atol);
    }
}

// Trick to avoid putting the following mapper macro inside the class definition in .h file:
// enclose macros in local 'my_enum_mappers', just to avoid avoiding cluttering of the parent class.
class my_enum_mappers : public ChTimestepperHHT {
public:
    CH_ENUM_MAPPER_BEGIN(HHT_Mode);
    CH_ENUM_VAL(ACCELERATION);
    CH_ENUM_VAL(POSITION);
    CH_ENUM_MAPPER_END(HHT_Mode);
};

void ChTimestepperHHT::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChTimestepperHHT>();
    // serialize parent class:
    ChTimestepperIIorder::ArchiveOUT(marchive);
    ChImplicitIterativeTimestepper::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(alpha);
    marchive << CHNVP(beta);
    marchive << CHNVP(gamma);
    marchive << CHNVP(scaling);
    my_enum_mappers::HHT_Mode_mapper modemapper;
    marchive << CHNVP(modemapper(mode), "mode");
}

void ChTimestepperHHT::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChTimestepperHHT>();
    // deserialize parent class:
    ChTimestepperIIorder::ArchiveIN(marchive);
    ChImplicitIterativeTimestepper::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(alpha);
    marchive >> CHNVP(beta);
    marchive >> CHNVP(gamma);
    marchive >> CHNVP(scaling);
    my_enum_mappers::HHT_Mode_mapper modemapper;
    marchive >> CHNVP(modemapper(mode), "mode");
}

}  // end namespace chrono