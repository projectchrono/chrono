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

namespace chrono {

// -----------------------------------------------------------------------------

CH_UPCASTING(ChTimestepperImplicit, ChTimestepper)

ChTimestepperImplicit::ChTimestepperImplicit()
    : jacobian_update_method(JacobianUpdate::EVERY_STEP),
      call_setup(true),
      maxiters(6),
      reltol(1e-4),
      abstolS(1e-10),
      abstolL(1e-10),
      numiters(0),
      numsetups(0),
      numsolves(0) {}

void ChTimestepperImplicit::SetJacobianUpdateMethod(JacobianUpdate method) {
    // If switching to JacobianUpdate::NEVER from a different strategy, force a Jacobian update
    if (method == JacobianUpdate::NEVER && jacobian_update_method != JacobianUpdate::NEVER)
        call_setup = true;

    jacobian_update_method = method;
}

bool ChTimestepperImplicit::CheckJacobianUpdateRequired(int iteration, bool previous_converged) {
    bool setup;
    switch (jacobian_update_method) {
        default:
        case JacobianUpdate::EVERY_STEP:
            // Update at first Newton iteration or on a stepsize change after a non-converged Newton.
            setup = (iteration == 0 && !previous_converged);
            break;
        case JacobianUpdate::EVERY_ITERATION:
            setup = true;
            break;
        case JacobianUpdate::NEVER:
            // Note that call_setup is always reset to false after an integration step, so the only times it can be true
            // if at the very first step or after a call to SetJacobianUpdateMethod() with JacobianUpdate::NEVER
            setup = call_setup;
            break;
        case JacobianUpdate::AUTOMATIC:
            //// TODO
            setup = true;
            break;
    }
    return setup;
}

bool ChTimestepperImplicit::CheckConvergence(int it) {
    bool converged = false;

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
        std::cout << "  Newton iteration=" << numiters;
        std::cout << "  |R| = " << R_nrm << "  |Qc| =" << Qc_nrm << "  |Ds| =" << Ds_nrm << "  |Dl| =" << Dl_nrm;
        if (it >= 2)
            std::cout << "  Conv. rate = " << convergence_rate;
        std::cout << std::endl;
    }

    if ((R_nrm < abstolS && Qc_nrm < abstolL) || (Ds_nrm < 1 && Dl_nrm < 1))
        converged = true;

    return converged;
}

// Calculate the error weight vector corresponding to the specified solution vector x,
// using the given relative and absolute tolerances.
void ChTimestepperImplicit::CalcErrorWeights(const ChVectorDynamic<>& x,
                                             double rtol,
                                             double atol,
                                             ChVectorDynamic<>& ewt) {
    ewt = (rtol * x.cwiseAbs() + atol).cwiseInverse();
}

void ChTimestepperImplicit::Advance(double dt) {
    // On entry, call_setup is true only:
    // - at the first iteration on the first step
    // - after a call to SetJacobianUpdateMethod(JacobianUpdate::NEVER)

    // Call the integrator-specific advance method
    OnAdvance(dt);

    // Reset call_setup to false, in case it is modified from outside the integrator
    call_setup = false;
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
    numiters = 0;
    numsetups = 0;
    numsolves = 0;

    unsigned int iteration;
    for (iteration = 0; iteration < maxiters; iteration++) {
        integrable->StateScatter(Xnew, Vnew, T + dt, false);  // state -> system
        R.setZero();
        Qc.setZero();
        integrable->LoadResidual_F(R, dt);                // R  = dt*f
        integrable->LoadResidual_Mv(R, (V - Vnew), 1.0);  // R += M*(v_old - v_new)
        integrable->LoadResidual_CqL(R, L, dt);           // R += dt*Cq'*l
        integrable->LoadConstraint_C(Qc, 1.0 / dt, Qc_do_clamp,
                                     Qc_clamping);  // Qc= C/dt  (sign flipped later in StateSolveCorrection)

        if (verbose)
            std::cout << " Euler iteration=" << iteration << "  |R|=" << R.lpNorm<Eigen::Infinity>()
                      << "  |Qc|=" << Qc.lpNorm<Eigen::Infinity>() << std::endl;

        if ((R.lpNorm<Eigen::Infinity>() < abstolS) && (Qc.lpNorm<Eigen::Infinity>() < abstolL))
            break;

        integrable->StateSolveCorrection(  //
            Ds, Dl, R, Qc,                 //
            1.0,                           // factor for  M
            -dt,                           // factor for  dF/dv
            -dt * dt,                      // factor for  dF/dx
            Xnew, Vnew, T + dt,            // not used here (scatter = false)
            false,                         // do not scatter update to Xnew Vnew T+dt before computing correction
            false,                         // full update? (not used, since no scatter)
            true                           // always call the solver's Setup
        );

        numiters++;
        numsetups++;
        numsolves++;

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
        false,                         // full update? (not used, since no scatter)
        true                           // force a call to the solver's Setup() function
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
        1.0,                           // factor for  M
        -dt,                           // factor for  dF/dv
        -dt * dt,                      // factor for  dF/dx
        X, V, T + dt,                  // not needed
        false,                         // do not scatter update to Xnew Vnew T+dt before computing correction
        false,                         // full update? (not used, since no scatter)
        true                           // force a call to the solver's Setup() function
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
        1.0,                           // factor for  M
        0,                             // factor for  dF/dv
        0,                             // factor for  dF/dx
        X, V, T,                       // not needed
        false,                         // do not scatter update to Xnew Vnew T+dt before computing correction
        false,                         // full update? (not used, since no scatter)
        true                           // force a call to the solver's Setup() function
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

    numiters = 0;
    numsetups = 0;
    numsolves = 0;

    unsigned int iteration;
    for (iteration = 0; iteration < maxiters; iteration) {
        integrable->StateScatter(Xnew, Vnew, T + dt, false);  // state -> system
        R = Rold;
        Qc.setZero();
        integrable->LoadResidual_F(R, dt * 0.5);       // + dt/2*f_new
        integrable->LoadResidual_Mv(R, Vnew, -1.0);    // - M*v_new
        integrable->LoadResidual_CqL(R, L, dt * 0.5);  // + dt/2*Cq*l_new
        integrable->LoadConstraint_C(Qc, 1.0 / dt, Qc_do_clamp,
                                     Qc_clamping);  // Qc= C/dt  (sign will be flipped later in StateSolveCorrection)

        if (verbose)
            std::cout << " Trapezoidal iteration=" << iteration << "  |R|=" << R.lpNorm<Eigen::Infinity>()
                      << "  |Qc|=" << Qc.lpNorm<Eigen::Infinity>() << std::endl;

        if ((R.lpNorm<Eigen::Infinity>() < abstolS) && (Qc.lpNorm<Eigen::Infinity>() < abstolL))
            break;

        integrable->StateSolveCorrection(  //
            Ds, Dl, R, Qc,                 //
            1.0,                           // factor for  M
            -dt * 0.5,                     // factor for  dF/dv
            -dt * dt * 0.25,               // factor for  dF/dx
            Xnew, Vnew, T + dt,            // not used here (scatter = false)
            false,                         // do not scatter update to Xnew Vnew T+dt before computing correction
            false,                         // full update? (not used, since no scatter)
            true                           // always force a call to the solver's Setup() function
        );

        numiters++;
        numsetups++;
        numsolves++;

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
        1.0,                           // factor for  M
        -dt * 0.5,                     // factor for  dF/dv
        -dt * dt * 0.25,               // factor for  dF/dx
        Xnew, Vnew, T + dt,            // not used here (scatter = false)
        false,                         // do not scatter update to Xnew Vnew T+dt before computing correction
        false,                         // full update? (not used, since no scatter)
        true                           // force a call to the solver's Setup() function
    );

    numiters = 1;
    numsetups = 1;
    numsolves = 1;

    Dl *= (2.0 / dt);  // Note it is not -(2.0/dt) because we assume StateSolveCorrection already flips sign of Dl
    L += Dl;

    Vnew += Ds;

    Xnew = X + ((Vnew + V) * (dt * 0.5));  // Xnew = Xold + h/2(Vnew+Vold)

    X = Xnew;
    V = Vnew;
    T += dt;

    integrable->StateScatter(X, V, T, true);                 // state -> system
    integrable->StateScatterAcceleration((Ds *= (1 / dt)));  // -> system auxiliary data (i.e acceleration as measure)
    integrable->StateScatterReactions(L *= 0.5);             // -> system auxiliary data (*=0.5 because use l_old = 0)
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
    numiters = 0;
    numsetups = 0;
    numsolves = 0;
    call_setup = true;

    unsigned int iteration;
    for (iteration = 0; iteration < maxiters; ++iteration) {
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
            std::cout << " Newmark iteration=" << iteration << "  |R|=" << R.lpNorm<Eigen::Infinity>()
                      << "  |Qc|=" << Qc.lpNorm<Eigen::Infinity>() << std::endl;

        if ((R.lpNorm<Eigen::Infinity>() < abstolS) && (Qc.lpNorm<Eigen::Infinity>() < abstolL)) {
            if (verbose) {
                std::cout << " Newmark NR converged (" << iteration << ")."
                          << "  T = " << T + dt << "  h = " << dt << std::endl;
            }
            break;
        }

        if (verbose && jacobian_update_method != JacobianUpdate::EVERY_ITERATION && call_setup)
            std::cout << " Newmark call Setup." << std::endl;

        integrable->StateSolveCorrection(  //
            Ds, Dl, R, Qc,                 //
            1.0,                           // factor for  M
            -dt * gamma,                   // factor for  dF/dv
            -dt * dt * beta,               // factor for  dF/dx
            Xnew, Vnew, T + dt,            // not used here (scatter = false)
            false,                         // do not scatter update to Xnew Vnew T+dt before computing correction
            false,                         // full update? (not used, since no scatter)
            call_setup                     // force a call to the solver's Setup() function
        );

        numiters++;
        numsolves++;
        if (call_setup) {
            numsetups++;
        }

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
