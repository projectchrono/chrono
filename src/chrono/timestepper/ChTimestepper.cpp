//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include "timestepper/ChTimestepper.h"

using namespace chrono;

/// Euler explicit timestepper
/// This performs the typical  y_new = y+ dy/dt * dt
/// integration with Euler formula.

void ChTimestepperEulerExpl::Advance(const double dt  ///< timestep to advance
                                     ) {
    // setup main vectors
    GetIntegrable()->StateSetup(Y, dYdt);

    // setup auxiliary vectors
    L.Reset(this->GetIntegrable()->GetNconstr());

    GetIntegrable()->StateGather(Y, T);  // state <- system

    GetIntegrable()->StateSolve(dYdt, L, Y, T, dt, false);  // dY/dt = f(Y,T)

    // Euler formula!
    //   y_new= y + dy/dt * dt

    Y = Y + dYdt * dt;  //  also: GetIntegrable().StateIncrement(y_new, y, Dy);

    T += dt;

    GetIntegrable()->StateScatter(Y, T);            // state -> system
    GetIntegrable()->StateScatterDerivative(dYdt);  // -> system auxiliary data
    GetIntegrable()->StateScatterReactions(L);      // -> system auxiliary data
}

/// Euler explicit timestepper customized for II order.
/// (It gives the same results of ChTimestepperEulerExpl,
/// but this performes a bit faster because it can exploit
/// the special structure of ChIntegrableIIorder)
/// This performs the typical
///    x_new = x + v * dt
///    v_new = v + a * dt
/// integration with Euler formula.

void ChTimestepperEulerExplIIorder::Advance(const double dt  ///< timestep to advance
                                            ) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Dv.Reset(mintegrable->GetNcoords_v(), GetIntegrable());
    L.Reset(mintegrable->GetNconstr());

    mintegrable->StateGather(X, V, T);  // state <- system

    mintegrable->StateSolveA(A, L, X, V, T, dt, false);  // Dv/dt = f(x,v,T)

    // Euler formula!

    X = X + V * dt;  // x_new= x + v * dt

    V = V + A * dt;  // v_new= v + a * dt

    T += dt;

    mintegrable->StateScatter(X, V, T);        // state -> system
    mintegrable->StateScatterAcceleration(A);  // -> system auxiliary data
    mintegrable->StateScatterReactions(L);     // -> system auxiliary data
}

/// Euler semi-implicit timestepper
/// This performs the typical
///    v_new = v + a * dt
///    x_new = x + v_new * dt
/// integration with Euler semi-implicit formula.

void ChTimestepperEulerSemiImplicit::Advance(const double dt  ///< timestep to advance
                                             ) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    L.Reset(mintegrable->GetNconstr());

    mintegrable->StateGather(X, V, T);  // state <- system

    mintegrable->StateSolveA(A, L, X, V, T, dt, false);  // Dv/dt = f(x,v,T)   Dv = f(x,v,T)*dt

    // Semi-implicit Euler formula!   (note the order of update of x and v, respect to original Euler II order explicit)

    V = V + A * dt;  // v_new= v + a * dt

    X = X + V * dt;  // x_new= x + v_new * dt

    T += dt;

    mintegrable->StateScatter(X, V, T);        // state -> system
    mintegrable->StateScatterAcceleration(A);  // -> system auxiliary data
    mintegrable->StateScatterReactions(L);     // -> system auxiliary data
}

/// Performs a step of a 4th order explicit Runge-Kutta
/// integration scheme.

void ChTimestepperRungeKuttaExpl::Advance(const double dt  ///< timestep to advance
                                          ) {
    // setup main vectors
    GetIntegrable()->StateSetup(Y, dYdt);

    // setup auxiliary vectors
    int n_y = GetIntegrable()->GetNcoords_y();
    int n_dy = GetIntegrable()->GetNcoords_dy();
    int n_c = GetIntegrable()->GetNconstr();
    y_new.Reset(n_y, GetIntegrable());
    Dydt1.Reset(n_dy, GetIntegrable());
    Dydt2.Reset(n_dy, GetIntegrable());
    Dydt3.Reset(n_dy, GetIntegrable());
    Dydt4.Reset(n_dy, GetIntegrable());
    L.Reset(n_c);

    GetIntegrable()->StateGather(Y, T);  // state <- system

    GetIntegrable()->StateSolve(Dydt1, L, Y, T, dt,
                                false);  // note, 'false'=no need to update with StateScatter before computation

    y_new = Y + Dydt1 * 0.5 * dt;  // integrable.StateIncrement(y_new, Y, Dydt1*0.5*dt);
    GetIntegrable()->StateSolve(Dydt2, L, y_new, T + dt * 0.5, dt);

    y_new = Y + Dydt2 * 0.5 * dt;  // integrable.StateIncrement(y_new, Y, Dydt2*0.5*dt);
    GetIntegrable()->StateSolve(Dydt3, L, y_new, T + dt * 0.5, dt);

    y_new = Y + Dydt3 * dt;  // integrable.StateIncrement(y_new, Y, Dydt3*dt);
    GetIntegrable()->StateSolve(Dydt4, L, y_new, T + dt, dt);

    Y = Y + (Dydt1 + Dydt2 * 2.0 + Dydt3 * 2.0 + Dydt4) * (1. / 6.) * dt;  // integrable.StateIncrement(...);
    dYdt = Dydt4;                                                          // to check
    T += dt;

    GetIntegrable()->StateScatter(Y, T);            // state -> system
    GetIntegrable()->StateScatterDerivative(dYdt);  // -> system auxiliary data
    GetIntegrable()->StateScatterReactions(L);      // -> system auxiliary data
}

/// Performs a step of a Heun explicit integrator. It is like
/// a 2nd Runge Kutta.

void ChTimestepperHeun::Advance(const double dt  ///< timestep to advance
                                ) {
    // setup main vectors
    GetIntegrable()->StateSetup(Y, dYdt);

    // setup auxiliary vectors
    int n_y = GetIntegrable()->GetNcoords_y();
    int n_dy = GetIntegrable()->GetNcoords_dy();
    int n_c = GetIntegrable()->GetNconstr();
    y_new.Reset(n_y, GetIntegrable());
    Dydt1.Reset(n_dy, GetIntegrable());
    Dydt2.Reset(n_dy, GetIntegrable());
    L.Reset(n_c);

    GetIntegrable()->StateGather(Y, T);  // state <- system

    GetIntegrable()->StateSolve(Dydt1, L, Y, T, dt,
                                false);  // note, 'false'=no need to update with StateScatter before computation

    y_new = Y + Dydt1 * dt;
    GetIntegrable()->StateSolve(Dydt2, L, y_new, T + dt, dt);

    Y = Y + (Dydt1 + Dydt2) * (dt / 2.);
    dYdt = Dydt2;
    T += dt;

    GetIntegrable()->StateScatter(Y, T);            // state -> system
    GetIntegrable()->StateScatterDerivative(dYdt);  // -> system auxiliary data
    GetIntegrable()->StateScatterReactions(L);      // -> system auxiliary data
}

/// Performs a step of a Leapfrog explicit integrator.
/// It is a symplectic method, with 2nd order accuracy,
/// at least when F depends on positions only.
/// Note: uses last step acceleration: changing or resorting
/// the numbering of DOFs will invalidate it.
/// Suggestion: use the ChTimestepperEulerSemiImplicit, it gives
/// the same accuracy with a bit of faster performance.

void ChTimestepperLeapfrog::Advance(const double dt  ///< timestep to advance
                                    ) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    L.Reset(mintegrable->GetNconstr());
    Aold.Reset(mintegrable->GetNcoords_v(), GetIntegrable());

    mintegrable->StateGather(X, V, T);  // state <- system
    mintegrable->StateGatherAcceleration(Aold);

    // advance X (uses last A)
    X = X + V * dt + Aold * (0.5 * dt * dt);

    // computes new A  (NOTE!!true for imposing a state-> system scatter update,because X changed..)
    mintegrable->StateSolveA(A, L, X, V, T, dt, true);  // Dv/dt = f(x,v,T)   Dv = f(x,v,T)*dt

    // advance V

    V = V + (Aold + A) * (0.5 * dt);

    T += dt;

    mintegrable->StateScatter(X, V, T);        // state -> system
    mintegrable->StateScatterAcceleration(A);  // -> system auxiliary data
    mintegrable->StateScatterReactions(L);     // -> system auxiliary data
}

/// Performs a step of Euler implicit for II order systems

void ChTimestepperEulerImplicit::Advance(const double dt  ///< timestep to advance
                                         ) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Dv.Reset(mintegrable->GetNcoords_v(), GetIntegrable());
    Dl.Reset(mintegrable->GetNconstr());
    Xnew.Reset(mintegrable->GetNcoords_x(), mintegrable);
    Vnew.Reset(mintegrable->GetNcoords_v(), mintegrable);
    R.Reset(mintegrable->GetNcoords_v());
    Qc.Reset(mintegrable->GetNconstr());
    L.Reset(mintegrable->GetNconstr());

    mintegrable->StateGather(X, V, T);  // state <- system

    // Extrapolate a prediction as warm start

    Xnew = X + V * dt;
    Vnew = V;  //+ A()*dt;

    // use Newton Raphson iteration to solve implicit Euler for v_new
    //
    // [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ Dv     ] = [ M*(v_old - v_new) + dt*f + dt*Cq'*l ]
    // [ Cq                           0   ] [ -dt*Dl ] = [ C/dt  ]

    for (int i = 0; i < this->GetMaxiters(); ++i) {
        mintegrable->StateScatter(Xnew, Vnew, T + dt);  // state -> system
        R.Reset();
        Qc.Reset();
        mintegrable->LoadResidual_F(R, dt);
        mintegrable->LoadResidual_Mv(R, (V - Vnew), 1.0);
        mintegrable->LoadResidual_CqL(R, L, dt);
        mintegrable->LoadConstraint_C(Qc, 1.0 / dt, Qc_do_clamp, Qc_clamping);

        if (verbose)
            GetLog() << " Euler iteration=" << i << "  |R|=" << R.NormInf() << "  |Qc|=" << Qc.NormInf() << "\n";

        if ((R.NormInf() < this->GetTolerance()) && (Qc.NormInf() < this->GetTolerance()))
            break;

        mintegrable->StateSolveCorrection(
            Dv, Dl, R, Qc,
            1.0,       // factor for  M
            -dt,       // factor for  dF/dv
            -dt * dt,  // factor for  dF/dx
            Xnew, Vnew, T + dt,
            false  // do not StateScatter update to Xnew Vnew T+dt before computing correction
            );

        Dl *= (1.0 / dt);  // Note it is not -(1.0/dt) because we assume StateSolveCorrection already flips sign of Dl
        L += Dl;

        Vnew += Dv;

        Xnew = X + Vnew * dt;
    }

    mintegrable->StateScatterAcceleration(
        (Vnew - V) * (1 / dt));  // -> system auxiliary data (i.e acceleration as measure, fits DVI/MDI)

    X = Xnew;
    V = Vnew;
    T += dt;

    mintegrable->StateScatter(X, V, T);     // state -> system
    mintegrable->StateScatterReactions(L);  // -> system auxiliary data
}

/// Performs a step of Euler implicit for II order systems
/// using the Anitescu/Stewart/Trinkle single-iteration method,
/// that is a bit like an implicit Euler where one performs only
/// the first NR corrector iteration.
/// If the solver in StateSolveCorrection is a CCP complementarity
/// solver, this is the typical Anitescu stabilized timestepper for DVIs.

void ChTimestepperEulerImplicitLinearized::Advance(const double dt  ///< timestep to advance
                                                   ) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Dl.Reset(mintegrable->GetNconstr());
    R.Reset(mintegrable->GetNcoords_v());
    Qc.Reset(mintegrable->GetNconstr());
    L.Reset(mintegrable->GetNconstr());

    mintegrable->StateGather(X, V, T);  // state <- system

    Vold = V;

    // solve only 1st NR step, using v_new = 0, so  Dv = v_new , therefore
    //
    // [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ Dv     ] = [ M*(v_old - v_new) + dt*f]
    // [ Cq                           0   ] [ -dt*Dl ] = [ C/dt + Ct ]
    //
    // becomes the Anitescu/Trinkle timestepper:
    //
    // [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ v_new  ] = [ M*(v_old) + dt*f]
    // [ Cq                           0   ] [ -dt*l  ] = [ C/dt + Ct ]

    mintegrable->LoadResidual_F(R, dt);
    mintegrable->LoadResidual_Mv(R, V, 1.0);
    mintegrable->LoadConstraint_C(Qc, 1.0 / dt, Qc_do_clamp, Qc_clamping);
    mintegrable->LoadConstraint_Ct(Qc, 1.0);

    mintegrable->StateSolveCorrection(V, L, R, Qc,
                                      1.0,           // factor for  M
                                      -dt,           // factor for  dF/dv
                                      -dt * dt,      // factor for  dF/dx
                                      X, V, T + dt,  // not needed
                                      false  // do not StateScatter update to Xnew Vnew T+dt before computing correction
                                      );

    L *= (1.0 / dt);  // Note it is not -(1.0/dt) because we assume StateSolveCorrection already flips sign of Dl

    mintegrable->StateScatterAcceleration(
        (V - Vold) * (1 / dt));  // -> system auxiliary data (i.e acceleration as measure, fits DVI/MDI)

    X += V * dt;

    T += dt;

    mintegrable->StateScatter(X, V, T);     // state -> system
    mintegrable->StateScatterReactions(L);  // -> system auxiliary data
}


/// Performs a step of Euler implicit for II order systems
/// using a semi implicit Euler without constr.stabilization, followed by a projection,
/// that is: a speed problem followed by a position problem that
/// keeps constraint drifting 'closed' by using a projection.
/// If the solver in StateSolveCorrection is a CCP complementarity
/// solver, this is the Tasora stabilized timestepper for DVIs.

void ChTimestepperEulerImplicitProjected::Advance(const double dt  ///< timestep to advance
                                                   ) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Dl.Reset(mintegrable->GetNconstr());
    R.Reset(mintegrable->GetNcoords_v());
    Qc.Reset(mintegrable->GetNconstr());
    L.Reset(mintegrable->GetNconstr());

    mintegrable->StateGather(X, V, T);  // state <- system

    Vold = V;

    // 1
    // Do a  Anitescu/Trinkle timestepper but without the C/dt correction:
    //
    // [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ v_new  ] = [ M*(v_old) + dt*f]
    // [ Cq                           0   ] [ -dt*l  ] = [ Ct ]

    mintegrable->LoadResidual_F(R, dt);
    mintegrable->LoadResidual_Mv(R, V, 1.0);
    mintegrable->LoadConstraint_C(Qc, 1.0 / dt, Qc_do_clamp, 0);
    mintegrable->LoadConstraint_Ct(Qc, 1.0);

    mintegrable->StateSolveCorrection(V, L, R, Qc,
                                      1.0,           // factor for  M
                                      -dt,           // factor for  dF/dv
                                      -dt * dt,      // factor for  dF/dx
                                      X, V, T + dt,  // not needed
                                      false  // do not StateScatter update to Xnew Vnew T+dt before computing correction
                                      );

    L *= (1.0 / dt);  // Note it is not -(1.0/dt) because we assume StateSolveCorrection already flips sign of Dl

    mintegrable->StateScatterAcceleration(
        (V - Vold) * (1 / dt));  // -> system auxiliary data (i.e acceleration as measure, fits DVI/MDI)

    X += V * dt;

    T += dt;

    mintegrable->StateScatter(X, V, T);     // state -> system
    mintegrable->StateScatterReactions(L);  // -> system auxiliary data

    // 2
    // Do the position stabilization (single NR step on constraints, with mass matrix as metric)

    Dl.Reset(mintegrable->GetNconstr());
    R.Reset(mintegrable->GetNcoords_v());
    Qc.Reset(mintegrable->GetNconstr());
    L.Reset(mintegrable->GetNconstr());
    Vold.Reset(mintegrable->GetNcoords_v(),V.GetIntegrable());

    //
    // [ M       Cq' ] [ dpos ] = [ 0 ]
    // [ Cq       0  ] [ l    ] = [ C ]

    mintegrable->LoadConstraint_C(Qc, 1.0, false, 0);

    mintegrable->StateSolveCorrection(Vold, L, R, Qc,
                                      1.0,           // factor for  M
                                      0,             // factor for  dF/dv
                                      0,             // factor for  dF/dx
                                      X, V, T,       // not needed
                                      false          // do not StateScatter update to Xnew Vnew T+dt before computing correction
                                      );

    X += Vold;  //here we used 'Vold' as 'dpos' to recycle Vold and avoid allocating a new vector dpos

    mintegrable->StateScatter(X, V, T);     // state -> system
}

/// Performs a step of trapezoidal implicit for II order systems
/// NOTE this is a modified version of the trapezoidal for DAE: the
/// original derivation would lead to a scheme that produces oscillatory
/// reactions in constraints, so this is a modified version that is first
/// order in constraint reactions. Use damped HHT or damped Newmark for
/// more advanced options.

void ChTimestepperTrapezoidal::Advance(const double dt  ///< timestep to advance
                                       ) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Dv.Reset(mintegrable->GetNcoords_v(), GetIntegrable());
    Dl.Reset(mintegrable->GetNconstr());
    Xnew.Reset(mintegrable->GetNcoords_x(), mintegrable);
    Vnew.Reset(mintegrable->GetNcoords_v(), mintegrable);
    L.Reset(mintegrable->GetNconstr());
    R.Reset(mintegrable->GetNcoords_v());
    Rold.Reset(mintegrable->GetNcoords_v());
    Qc.Reset(mintegrable->GetNconstr());

    mintegrable->StateGather(X, V, T);  // state <- system
    // mintegrable->StateGatherReactions(L); // <- system  assume l_old = 0;  otherwise DAE gives oscillatory reactions

    // extrapolate a prediction as a warm start

    Xnew = X + V * dt;
    Vnew = V;  // +A()*dt;

    // use Newton Raphson iteration to solve implicit trapezoidal for v_new
    //
    // [ M - dt/2*dF/dv - dt^2/4*dF/dx    Cq' ] [ Dv       ] = [ M*(v_old - v_new) + dt/2(f_old + f_new  + Cq*l_old +
    // Cq*l_new)]
    // [ Cq                               0   ] [ -dt/2*Dl ] = [ C/dt ]

    mintegrable->LoadResidual_F(Rold, dt * 0.5);  // dt/2*f_old
    mintegrable->LoadResidual_Mv(Rold, V, 1.0);   // M*v_old
    // mintegrable->LoadResidual_CqL(Rold, L, dt*0.5); // dt/2*l_old   assume L_old = 0

    for (int i = 0; i < this->GetMaxiters(); ++i) {
        mintegrable->StateScatter(Xnew, Vnew, T + dt);  // state -> system
        R = Rold;
        Qc.Reset();
        mintegrable->LoadResidual_F(R, dt * 0.5);                               // + dt/2*f_new
        mintegrable->LoadResidual_Mv(R, Vnew, -1.0);                            // - M*v_new
        mintegrable->LoadResidual_CqL(R, L, dt * 0.5);                          // + dt/2*Cq*l_new
        mintegrable->LoadConstraint_C(Qc, 1.0 / dt, Qc_do_clamp, Qc_clamping);  // C/dt

        if (verbose)
            GetLog() << " Trapezoidal iteration=" << i << "  |R|=" << R.NormTwo() << "  |Qc|=" << Qc.NormTwo() << "\n";

        if ((R.NormInf() < this->GetTolerance()) && (Qc.NormInf() < this->GetTolerance()))
            break;

        mintegrable->StateSolveCorrection(
            Dv, Dl, R, Qc,
            1.0,              // factor for  M
            -dt * 0.5,        // factor for  dF/dv
            -dt * dt * 0.25,  // factor for  dF/dx
            Xnew, Vnew, T + dt,
            false  // do not StateScatter update to Xnew Vnew T+dt before computing correction
            );

        Dl *= (2.0 / dt);  // Note it is not -(2.0/dt) because we assume StateSolveCorrection already flips sign of Dl
        L += Dl;

        Vnew += Dv;

        Xnew = X + ((Vnew + V) * (dt * 0.5));  // Xnew = Xold + h/2(Vnew+Vold)
    }

    mintegrable->StateScatterAcceleration(
        (Vnew - V) * (1 / dt));  // -> system auxiliary data (i.e acceleration as measure, fits DVI/MDI)

    X = Xnew;
    V = Vnew;
    T += dt;

    mintegrable->StateScatter(X, V, T);  // state -> system
    mintegrable->StateScatterReactions(L *=
                                       0.5);  // -> system auxiliary data   (*=0.5 cause we used the hack of l_old = 0)
}

/// Performs a step of trapezoidal implicit linearized for II order systems

void ChTimestepperTrapezoidalLinearized::Advance(const double dt  ///< timestep to advance
                                                 ) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Dv.Reset(mintegrable->GetNcoords_v(), GetIntegrable());
    Dl.Reset(mintegrable->GetNconstr());
    Xnew.Reset(mintegrable->GetNcoords_x(), mintegrable);
    Vnew.Reset(mintegrable->GetNcoords_v(), mintegrable);
    L.Reset(mintegrable->GetNconstr());
    R.Reset(mintegrable->GetNcoords_v());
    Rold.Reset(mintegrable->GetNcoords_v());
    Qc.Reset(mintegrable->GetNconstr());

    mintegrable->StateGather(X, V, T);  // state <- system
    // mintegrable->StateGatherReactions(L); // <- system  assume l_old = 0;

    // extrapolate a prediction as a warm start

    Xnew = X + V * dt;
    Vnew = V;

    // solve implicit trapezoidal for v_new
    //
    // [ M - dt/2*dF/dv - dt^2/4*dF/dx    Cq' ] [ Dv       ] = [ M*(v_old - v_new) + dt/2(f_old + f_new  + Cq*l_old +
    // Cq*l_new)]
    // [ Cq                               0   ] [ -dt/2*Dl ] = [ C/dt ]

    mintegrable->LoadResidual_F(Rold, dt * 0.5);  // dt/2*f_old
    mintegrable->LoadResidual_Mv(Rold, V, 1.0);   // M*v_old
    // mintegrable->LoadResidual_CqL(Rold, L, dt*0.5); // dt/2*l_old  assume l_old = 0;

    mintegrable->StateScatter(Xnew, Vnew, T + dt);  // state -> system
    R = Rold;
    Qc.Reset();
    mintegrable->LoadResidual_F(R, dt * 0.5);     // + dt/2*f_new
    mintegrable->LoadResidual_Mv(R, Vnew, -1.0);  // - M*v_new
    // mintegrable->LoadResidual_CqL(R, L, dt*0.5); // + dt/2*Cq*l_new  assume l_old = 0;
    mintegrable->LoadConstraint_C(Qc, 1.0 / dt, Qc_do_clamp, Qc_clamping);  // C/dt

    mintegrable->StateSolveCorrection(Dv, Dl, R, Qc,
                                      1.0,              // factor for  M
                                      -dt * 0.5,        // factor for  dF/dv
                                      -dt * dt * 0.25,  // factor for  dF/dx
                                      Xnew, Vnew, T + dt,
                                      false  // do not StateScatter update to Xnew Vnew T+dt before computing correction
                                      );

    Dl *= (2.0 / dt);  // Note it is not -(2.0/dt) because we assume StateSolveCorrection already flips sign of Dl
    L += Dl;

    Vnew += Dv;

    Xnew = X + ((Vnew + V) * (dt * 0.5));  // Xnew = Xold + h/2(Vnew+Vold)

    X = Xnew;
    V = Vnew;
    T += dt;

    mintegrable->StateScatter(X, V, T);  // state -> system
    mintegrable->StateScatterAcceleration(
        (Dv *= (1 / dt)));  // -> system auxiliary data (i.e acceleration as measure, fits DVI/MDI)
    mintegrable->StateScatterReactions(L *= 0.5);  // -> system auxiliary data (*=0.5 cause use l_old = 0)
}

/// Performs a step of trapezoidal implicit linearized for II order systems
///*** SIMPLIFIED VERSION -DOES NOT WORK - PREFER ChTimestepperTrapezoidalLinearized

void ChTimestepperTrapezoidalLinearized2::Advance(const double dt  ///< timestep to advance
                                                  ) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Dv.Reset(mintegrable->GetNcoords_v(), GetIntegrable());
    Xnew.Reset(mintegrable->GetNcoords_x(), mintegrable);
    Vnew.Reset(mintegrable->GetNcoords_v(), mintegrable);
    L.Reset(mintegrable->GetNconstr());
    R.Reset(mintegrable->GetNcoords_v());
    Qc.Reset(mintegrable->GetNconstr());

    mintegrable->StateGather(X, V, T);  // state <- system

    // extrapolate a prediction as a warm start

    Xnew = X + V * dt;
    Vnew = V;

    // use Newton Raphson iteration to solve implicit trapezoidal for v_new
    //
    // [ M - dt/2*dF/dv - dt^2/4*dF/dx    Cq' ] [ v_new    ] = [ M*(v_old) + dt/2(f_old + f_new)]
    // [ Cq                               0   ] [ -dt/2*L ] m= [ C/dt                           ]

    mintegrable->LoadResidual_F(R, dt * 0.5);  // dt/2*f_old
    mintegrable->LoadResidual_Mv(R, V, 1.0);   // M*v_old

    mintegrable->StateScatter(Xnew, Vnew, T + dt);  // state -> system
    Qc.Reset();
    mintegrable->LoadResidual_F(R, dt * 0.5);                               // + dt/2*f_new
    mintegrable->LoadConstraint_C(Qc, 1.0 / dt, Qc_do_clamp, Qc_clamping);  // C/dt

    mintegrable->StateSolveCorrection(Vnew, L, R, Qc,
                                      1.0,              // factor for  M
                                      -dt * 0.5,        // factor for  dF/dv
                                      -dt * dt * 0.25,  // factor for  dF/dx
                                      Xnew, Vnew, T + dt,
                                      false  // do not StateScatter update to Xnew Vnew T+dt before computing correction
                                      );

    L *= (2.0 / dt);  // Note it is not -(2.0/dt) because we assume StateSolveCorrection already flips sign of Dl

    X += ((Vnew + V) * (dt * 0.5));  // Xnew = Xold + h/2(Vnew+Vold)

    mintegrable->StateScatterAcceleration(
        (Vnew - V) * (1 / dt));  // -> system auxiliary data (i.e acceleration as measure, fits DVI/MDI)

    V = Vnew;

    T += dt;

    mintegrable->StateScatter(X, V, T);     // state -> system
    mintegrable->StateScatterReactions(L);  // -> system auxiliary data
}

/// Performs a step of HHT (generalized alpha) implicit for II order systems
/// See Negrut et al. 2007.

void ChTimestepperHHT::Advance(const double dt  ///< timestep to advance
                               ) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Da.Reset(mintegrable->GetNcoords_a(), GetIntegrable());
    Dl.Reset(mintegrable->GetNconstr());
    Xnew.Reset(mintegrable->GetNcoords_x(), mintegrable);
    Vnew.Reset(mintegrable->GetNcoords_v(), mintegrable);
    Anew.Reset(mintegrable->GetNcoords_a(), mintegrable);
    R.Reset(mintegrable->GetNcoords_v());
    Rold.Reset(mintegrable->GetNcoords_v());
    Qc.Reset(mintegrable->GetNconstr());
    L.Reset(mintegrable->GetNconstr());

    mintegrable->StateGather(X, V, T);        // state <- system
    mintegrable->StateGatherAcceleration(A);  // <- system
    mintegrable->StateGatherReactions(L);     // <- system

    // extrapolate a prediction as a warm start

	//==7/13/2015
	if(HHTflag==1){
	//Acceleration is Solution vector
		Vnew = V;// + Anew*dt;
		Xnew = X + Vnew * dt;// + Anew * dt * dt;
	}else if(HHTflag==2||HHTflag==3){
	//Position is solution vector
		Xnew = X;
		Vnew = V * (-(gamma/beta-1.0))  - A * dt*(gamma/(2.0*beta)-1.0);
		Anew = V *(-1.0/(beta*dt))- A * (1.0/(2.0*beta)-1.0);
	}
	//
	//GetLog()<<"Velocity 1:"<<Vnew<<"\n";
	//system("pause");
	if(HHTflag==1){
    mintegrable->LoadResidual_F(Rold, -(alpha / (1.0 + alpha)));       // -alpha/(1.0+alpha) * f_old
    mintegrable->LoadResidual_CqL(Rold, L, -(alpha / (1.0 + alpha)));  // -alpha/(1.0+alpha) * Cq'*l_old
	}else if(HHTflag==2){
    mintegrable->LoadResidual_F(Rold, -(alpha / (1.0 + alpha)));       // -alpha/(1.0+alpha) * f_old
    mintegrable->LoadResidual_CqL(Rold, L, -(alpha / (1.0 + alpha)));  // -alpha/(1.0+alpha) * Cq'*l_old
	}else if(HHTflag==3){
    mintegrable->LoadResidual_F(Rold, -(alpha / (1.0 + alpha))*dt*dt);       // -alpha/(1.0+alpha) * f_old
    mintegrable->LoadResidual_CqL(Rold, L, -(alpha / (1.0 + alpha))*dt*dt);  // -alpha/(1.0+alpha) * Cq'*l_old
	}
    // use Newton Raphson iteration to solve HHT for a_new
    // Note: l and l_new and Dl with opposite sign if compared to Negrut et al. 2007.

    //
    // [ M - dt*gamma*dF/dv - dt^2*beta*dF/dx    Cq' ] [ Da       ] = [-1/(1+alpha)*M*(a_new) + (f_new +Cq*l_new) -
    // (alpha/(1+alpha))(f_old +Cq*l_old)]
    // [ Cq                                      0   ] [ Dl       ] = [ 1/(beta*dt^2)*C ]

    for (int i = 0; i < this->GetMaxiters(); ++i) {
        mintegrable->StateScatter(Xnew, Vnew, T + dt);  // state -> system
        R = Rold;
        Qc.Reset();
		if(HHTflag==1){
        mintegrable->LoadResidual_F(R, 1.0);                                                    //  f_new
        mintegrable->LoadResidual_CqL(R, L, 1.0);                                               //  Cq'*l_new
        mintegrable->LoadResidual_Mv(R, Anew, -(1.0 / (1.0 + alpha)));                          // -1/(1+alpha)*M*a_new
        mintegrable->LoadConstraint_C(Qc, (1.0 / (beta * dt * dt)), Qc_do_clamp, Qc_clamping);  //  1/(beta*dt^2)*C
		}else if(HHTflag==2){
	    //Position is solution vector
        mintegrable->LoadResidual_F(R, 1.0);                                                    //  f_new
        mintegrable->LoadResidual_CqL(R, L, 1.0);                                               //  Cq'*l_new
        mintegrable->LoadResidual_Mv(R, Anew, -(1.0 / (1.0 + alpha)));                          // -1/(1+alpha)*M*a_new
        mintegrable->LoadConstraint_C(Qc, (1.0), Qc_do_clamp, Qc_clamping);  //  1/(beta*dt^2)*C
        }else if(HHTflag==3){
	    //Position is solution vector with Scaling
        mintegrable->LoadResidual_F(R, dt*dt);                                                    //  f_new
        mintegrable->LoadResidual_CqL(R, L, dt*dt);                                               //  Cq'*l_new
        mintegrable->LoadResidual_Mv(R, Anew, -(1.0 / (1.0 + alpha))*dt*dt);                          // -1/(1+alpha)*M*a_new
        mintegrable->LoadConstraint_C(Qc, (1.0), Qc_do_clamp, Qc_clamping);  //  1/(beta*dt^2)*C
        }
		verbose=1;
		Iterations+=1;
 /*       if (verbose)
            GetLog() << " HHT iteration=" << i << "  |R|=" << R.NormTwo() << "  |Qc|=" << Qc.NormTwo() << "\n";

        if ((R.NormInf() < this->GetTolerance()) && (Qc.NormInf() < this->GetTolerance()))
            break;*/

		if(HHTflag==1){
        mintegrable->StateSolveCorrection(
            Da, Dl, R, Qc,
            (1.0 / (1.0 + alpha)),  // factor for  M (was 1 in Negrut paper ?!)
            -dt * gamma,            // factor for  dF/dv
            -dt * dt * beta,        // factor for  dF/dx
            Xnew, Vnew, T + dt,
            false  // do not StateScatter update to Xnew Vnew T+dt before computing correction
            );
		}else if(HHTflag==2){
		mintegrable->StateSolveCorrection(
            Da, Dl, R, Qc,
            (1.0 / (1.0 + alpha) / (beta*dt*dt)),  // factor for  M (was 1 in Negrut paper ?!)
            -1.0 / (dt * gamma),            // factor for  dF/dv
            -1.0,        // factor for  dF/dx
            Xnew, Vnew, T + dt,
            false  // do not StateScatter update to Xnew Vnew T+dt before computing correction
            );
		}else if(HHTflag==3){
		mintegrable->StateSolveCorrection(
            Da, Dl, R, Qc,
            (1.0 / (1.0 + alpha) / (beta)),  // factor for  M (was 1 in Negrut paper ?!)
            -1.0 *dt / ( gamma) ,            // factor for  dF/dv
            -dt*dt,        // factor for  dF/dx
            Xnew, Vnew, T + dt,
            false  // do not StateScatter update to Xnew Vnew T+dt before computing correction
            );
		}

        L += Dl;  // Note it is not -= Dl because we assume StateSolveCorrection flips sign of Dl
        if(HHTflag==1){
		Anew += Da;
        Xnew = X + V * dt + A * (dt * dt * (0.5 - beta)) + Anew * (dt * dt * beta);
        Vnew = V + A * (dt * (1.0 - gamma)) + Anew * (dt * gamma);
		}else if(HHTflag==2||HHTflag==3){
        Xnew = Xnew +Da; // X + V * dt + A * (dt * dt * (0.5 - beta)) + Anew * (dt * dt * beta);

		//Vnew = V * (-(gamma/beta-1.0))  - A * dt*(gamma/(2.0*beta)-1.0);
		//Anew = V *(-1.0/(beta*dt))- A * (1.0/(2.0*beta)-1.0);

        Vnew =  V * (-(gamma/beta-1.0)) - A * dt * (gamma/(2.0*beta)-1.0);
		Vnew += (Xnew-X) * (gamma /(beta*dt));  //V + A * (dt * (1.0 - gamma)) + Anew * (dt * gamma);
		Anew =  -V*(1.0/(beta*dt))-A*(1.0/(2.0*beta)-1.0);
		Anew += (Xnew-X)*(1.0/(beta*dt*dt));
		}

		// Creteria for HHT method 7/15/2015
		if(HHTflag==1){
			GetLog() << " HHT iteration=" << i << "  |R|=" << R.NormTwo() << "  |Qc|=" << Qc.NormTwo() << "  " << this->GetTolerance() << "\n";
		    GetLog() << " HHT iteration=" << i << "  |DA|=" << Da.NormTwo() << "  |Dl|=" << Dl.NormTwo() << "  " << this->GetTolerance() << "\n";
			if (((R.NormTwo() < this->GetTolerance()) && (Qc.NormTwo() < this->GetTolerance())) || ((Da.NormTwo() < this->GetTolerance()) && (Dl.NormTwo() < this->GetTolerance())))
				break;
		}else if(HHTflag==2||HHTflag==3){
		   	double Err1;
		    double Err2;
		    ConvergenceViolationCheck(Xnew,L,Da,Dl,Err1,Err2,HHTflag);
		    GetLog() << " HHT iteration=" << i << "  |Err1|=" << Err1 << "  |Err2|=" << Err2 << "  Tol="<<this->GetTolerance() << "\n";
		    if (Err1 < this->GetTolerance() && Err2 < this->GetTolerance())
                break;
		}

		//GetLog()<<"Velocity 1:"<<V<<"\n";
		//GetLog()<<"Velocity 2:"<<Vnew<<"\n";
		//system("pause");
    }

    X = Xnew;
    V = Vnew;
    A = Anew;
    T += dt;

    mintegrable->StateScatter(X, V, T);        // state -> system
    mintegrable->StateScatterAcceleration(A);  // -> system auxiliary data
    mintegrable->StateScatterReactions(L);     // -> system auxiliary data
}

/// Performs a step of Newmark constrained implicit for II order DAE systems
/// See Negrut et al. 2007.

void ChTimestepperNewmark::Advance(const double dt  ///< timestep to advance
                                   ) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Da.Reset(mintegrable->GetNcoords_a(), GetIntegrable());
    Dl.Reset(mintegrable->GetNconstr());
    Xnew.Reset(mintegrable->GetNcoords_x(), mintegrable);
    Vnew.Reset(mintegrable->GetNcoords_v(), mintegrable);
    Anew.Reset(mintegrable->GetNcoords_a(), mintegrable);
    R.Reset(mintegrable->GetNcoords_v());
    Rold.Reset(mintegrable->GetNcoords_v());
    Qc.Reset(mintegrable->GetNconstr());
    L.Reset(mintegrable->GetNconstr());

    mintegrable->StateGather(X, V, T);  // state <- system
    mintegrable->StateGatherAcceleration(A);

    // extrapolate a prediction as a warm start

    Vnew = V;
    Xnew = X + Vnew * dt;

    // use Newton Raphson iteration to solve implicit Newmark for a_new

    //
    // [ M - dt*gamma*dF/dv - dt^2*beta*dF/dx    Cq' ] [ Da   ] = [ -M*(a_new) + f_new + Cq*l_new ]
    // [ Cq                                      0   ] [ Dl   ] = [ 1/(beta*dt^2)*C               ] ]

    for (int i = 0; i < this->GetMaxiters(); ++i) {
        mintegrable->StateScatter(Xnew, Vnew, T + dt);  // state -> system

        R.Reset(mintegrable->GetNcoords_v());
        Qc.Reset(mintegrable->GetNconstr());
        mintegrable->LoadResidual_F(R, 1.0);                                                    //  f_new
        mintegrable->LoadResidual_CqL(R, L, 1.0);                                               //   Cq'*l_new
        mintegrable->LoadResidual_Mv(R, Anew, -1.0);                                            //  - M*a_new
        mintegrable->LoadConstraint_C(Qc, (1.0 / (beta * dt * dt)), Qc_do_clamp, Qc_clamping);  //  1/(beta*dt^2)*C

        if (verbose)
            GetLog() << " Newmark iteration=" << i << "  |R|=" << R.NormTwo() << "  |Qc|=" << Qc.NormTwo() << "\n";

        if ((R.NormInf() < this->GetTolerance()) && (Qc.NormInf() < this->GetTolerance()))
            break;

        mintegrable->StateSolveCorrection(
            Da, Dl, R, Qc,
            1.0,              // factor for  M
            -dt * gamma,      // factor for  dF/dv
            -dt * dt * beta,  // factor for  dF/dx
            Xnew, Vnew, T + dt,
            false  // do not StateScatter update to Xnew Vnew T+dt before computing correction
            );

        L += Dl;  // Note it is not -= Dl because we assume StateSolveCorrection flips sign of Dl
        Anew += Da;

        Xnew = X + V * dt + A * (dt * dt * (0.5 - beta)) + Anew * (dt * dt * beta);

        Vnew = V + A * (dt * (1.0 - gamma)) + Anew * (dt * gamma);
    }

    X = Xnew;
    V = Vnew;
    A = Anew;
    T += dt;

    mintegrable->StateScatter(X, V, T);        // state -> system
    mintegrable->StateScatterAcceleration(A);  // -> system auxiliary data
    mintegrable->StateScatterReactions(L);     // -> system auxiliary data
}
