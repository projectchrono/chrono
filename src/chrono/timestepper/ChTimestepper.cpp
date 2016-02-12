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

#include <cmath>

#include "timestepper/ChTimestepper.h"

using namespace chrono;



//////////////////////////////////////////////////////////////////////////////////////////////////////////


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChTimestepperEulerExpl> a_registration_ChTimestepperEulerExpl;

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



//////////////////////////////////////////////////////////////////////////////////////////////////////////


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChTimestepperEulerExplIIorder> a_registration_ChTimestepperEulerExplIIorder;

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




//////////////////////////////////////////////////////////////////////////////////////////////////////////


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChTimestepperEulerSemiImplicit> a_registration_ChTimestepperEulerSemiImplicit;

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




//////////////////////////////////////////////////////////////////////////////////////////////////////////


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChTimestepperRungeKuttaExpl> a_registration_ChTimestepperRungeKuttaExpl;

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



//////////////////////////////////////////////////////////////////////////////////////////////////////////


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChTimestepperHeun> a_registration_ChTimestepperHeun;

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



//////////////////////////////////////////////////////////////////////////////////////////////////////////


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChTimestepperLeapfrog> a_registration_ChTimestepperLeapfrog;

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



//////////////////////////////////////////////////////////////////////////////////////////////////////////


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChTimestepperEulerImplicit> a_registration_ChTimestepperEulerImplicit;

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

        if ((R.NormInf() < abstolS) && (Qc.NormInf() < abstolL))
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



//////////////////////////////////////////////////////////////////////////////////////////////////////////


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChTimestepperEulerImplicitLinearized> a_registration_ChTimestepperEulerImplicitLinearized;

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




//////////////////////////////////////////////////////////////////////////////////////////////////////////


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChTimestepperEulerImplicitProjected> a_registration_ChTimestepperEulerImplicitProjected;

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




//////////////////////////////////////////////////////////////////////////////////////////////////////////


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChTimestepperTrapezoidal> a_registration_ChTimestepperTrapezoidal;

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

        if ((R.NormInf() < abstolS) && (Qc.NormInf() < abstolL))
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



//////////////////////////////////////////////////////////////////////////////////////////////////////////


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChTimestepperTrapezoidalLinearized> a_registration_ChTimestepperTrapezoidalLinearized;

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



//////////////////////////////////////////////////////////////////////////////////////////////////////////


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChTimestepperTrapezoidalLinearized2> a_registration_ChTimestepperTrapezoidalLinearized2;

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




//////////////////////////////////////////////////////////////////////////////////////////////////////////


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChTimestepperHHT> a_registration_ChTimestepperHHT;

/// Performs a step of HHT (generalized alpha) implicit for II order systems
/// See Negrut et al. 2007.

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
    num_it = 0;              // total number of NR iterations 

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

    while (T < tfinal) {
        double scaling_factor = scaling ? beta * h * h : 1;
        Prepare(mintegrable, scaling_factor);

        // Newton-Raphson for state at T+h
        bool converged;
        int it;
        for (it = 0; it < maxiters; it++) {
            Increment(mintegrable, scaling_factor);
            num_it++;
            converged = CheckConvergence(scaling_factor);
            if (converged)
                break;
        }

        if (converged || !step_control) {
            // NR converged (or step size control disabled):
            // - if the number of iterations was low enough, increase the count of successive
            //   successful steps (for possible step increase)
            // - if needed, adjust stepsize to reach exactly tfinal
            // - advance time and set the state

            if (it < maxiters_success)
                num_successful_steps++;
            else
                num_successful_steps = 0;

            if (verbose) {
                if (converged)
                    GetLog() << " HHT NR converged (" << num_successful_steps << ").";
                else
                    GetLog() << " HHT NR terminated.";
                GetLog() << "  T = " << T + h << "  h = " << h << "\n";
            }

            if (std::abs(T + h - tfinal) < 1e-6)
                h = tfinal - T;

            T += h;
            X = Xnew;
            V = Vnew;
            A = Anew;
            L = Lnew;

        } else {
            // NR did not converge.
            // - reset the count of successive successful steps
            // - decrease stepsize
            // - bail out if stepsize reaches minimum allowable

            num_successful_steps = 0;
            h *= step_decrease_factor;
            if (verbose)
                GetLog() << " ---HHT reduce stepsize to " << h << "\n";

            if (h < h_min) {
                if (verbose)
                    GetLog() << " HHT at minimum stepsize. Exiting...\n";
                throw ChException("HHT: Reached minimum allowable step size.");
            }
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
            integrable->LoadResidual_CqL(Rold, L, -(alpha / (1.0 + alpha)) * scaling_factor);  // -alpha/(1.0+alpha) * Cq'*l_old
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
            integrable->StateSolveCorrection(
                Da, Dl, R, Qc,
                1 / (1 + alpha),  // factor for  M (was 1 in Negrut paper ?!)
                -h * gamma,       // factor for  dF/dv
                -h * h * beta,    // factor for  dF/dx
                Xnew, Vnew, T + h,
                false  // do not StateScatter update to Xnew Vnew T+h before computing correction
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
            integrable->StateSolveCorrection(
                Da, Dl, R, Qc,
                scaling_factor / ((1 + alpha) * beta * h * h),  // factor for  M
                -scaling_factor * gamma / (beta * h),           // factor for  dF/dv
                -scaling_factor,                                // factor for  dF/dx
                Xnew, Vnew, T + h,
                false  // do not StateScatter update to Xnew Vnew T+h before computing correction
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
                GetLog() << " HHT iteration=" << num_it << "  |R|=" << R_nrm << "  |Qc|=" << Qc_nrm
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
                GetLog() << " HHT iteration=" << num_it << "  |Dx|=" << Dx_nrm << "  |Dl|=" << Dl_nrm << "\n";
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChTimestepperNewmark> a_registration_ChTimestepperNewmark;

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

        if ((R.NormInf() < abstolS) && (Qc.NormInf() < abstolL))
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
