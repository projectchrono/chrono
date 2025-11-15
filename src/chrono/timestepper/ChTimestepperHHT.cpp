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
                                     false,              // no need for full update, since no scatter
                                     call_setup,         // if true, call the solver's Setup function
                                     call_analyze        // if true, call the solver's Setup analyze phase
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