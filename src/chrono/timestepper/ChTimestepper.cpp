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

#include "chrono/timestepper/ChTimestepper.h"

namespace chrono {

CH_UPCASTING(ChTimestepperIorder, ChTimestepper)
CH_UPCASTING(ChTimestepperIIorder, ChTimestepper)
CH_UPCASTING(ChImplicitIterativeTimestepper, ChImplicitTimestepper)


// -----------------------------------------------------------------------------

// Trick to avoid putting the following mapper macro inside the class definition in .h file:
// enclose macros in local 'my_enum_mappers', just to avoid avoiding cluttering of the parent class.
class my_enum_mappers : public ChTimestepper {
  public:
    CH_ENUM_MAPPER_BEGIN(Type);
    CH_ENUM_VAL(Type::EULER_IMPLICIT);
    CH_ENUM_VAL(Type::EULER_IMPLICIT_LINEARIZED);
    CH_ENUM_VAL(Type::EULER_IMPLICIT_PROJECTED);
    CH_ENUM_VAL(Type::TRAPEZOIDAL);
    CH_ENUM_VAL(Type::TRAPEZOIDAL_LINEARIZED);
    CH_ENUM_VAL(Type::HHT);
    CH_ENUM_VAL(Type::HEUN);
    CH_ENUM_VAL(Type::RUNGEKUTTA45);
    CH_ENUM_VAL(Type::EULER_EXPLICIT);
    CH_ENUM_VAL(Type::LEAPFROG);
    CH_ENUM_VAL(Type::NEWMARK);
    CH_ENUM_VAL(Type::CUSTOM);
    CH_ENUM_MAPPER_END(Type);
};

void ChTimestepper::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepper>();
    // method type:
    my_enum_mappers::Type_mapper typemapper;
    Type type = GetType();
    archive << CHNVP(typemapper(type), "timestepper_type");
    // serialize all member data:
    archive << CHNVP(verbose);
    archive << CHNVP(Qc_do_clamp);
    archive << CHNVP(Qc_clamping);
}

void ChTimestepper::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/ archive.VersionRead<ChTimestepper>();
    // method type:
    my_enum_mappers::Type_mapper typemapper;
    Type type = GetType();
    archive >> CHNVP(typemapper(type), "timestepper_type");
    // stream in all member data:
    archive >> CHNVP(verbose);
    archive >> CHNVP(Qc_do_clamp);
    archive >> CHNVP(Qc_clamping);
}
// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperEulerExpl)
CH_UPCASTING(ChTimestepperEulerExpl, ChTimestepperIorder)

// Euler explicit timestepper.
// This performs the typical  y_new = y+ dy/dt * dt integration with Euler formula.
void ChTimestepperEulerExpl::Advance(const double dt) {
    // setup main vectors
    GetIntegrable()->StateSetup(Y, dYdt);

    // setup auxiliary vectors
    L.setZero(GetIntegrable()->GetNconstr());

    GetIntegrable()->StateGather(Y, T);  // state <- system

    GetIntegrable()->StateSolve(dYdt, L, Y, T, dt, false, false);  // dY/dt = f(Y,T)

    // Euler formula!
    //   y_new= y + dy/dt * dt

    Y = Y + dYdt * dt;  //  also: GetIntegrable().StateIncrement(y_new, y, Dy);

    T += dt;

    GetIntegrable()->StateScatter(Y, T, true);            // state -> system
    GetIntegrable()->StateScatterDerivative(dYdt);  // -> system auxiliary data
    GetIntegrable()->StateScatterReactions(L);      // -> system auxiliary data
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperEulerExplIIorder)
CH_UPCASTING(ChTimestepperEulerExplIIorder, ChTimestepperIIorder)

// Euler explicit timestepper customized for II order.
// (It gives the same results of ChTimestepperEulerExpl,
// but this performs a bit faster because it can exploit
// the special structure of ChIntegrableIIorder)
// This performs the typical
//    x_new = x + v * dt
//    v_new = v + a * dt
// integration with Euler formula.
void ChTimestepperEulerExplIIorder::Advance(const double dt) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Dv.setZero(mintegrable->GetNcoords_v(), GetIntegrable());
    L.setZero(mintegrable->GetNconstr());

    mintegrable->StateGather(X, V, T);  // state <- system

    mintegrable->StateSolveA(A, L, X, V, T, dt, false, false);  // Dv/dt = f(x,v,T)

    // Euler formula!

    X = X + V * dt;  // x_new= x + v * dt

    V = V + A * dt;  // v_new= v + a * dt

    T += dt;

    mintegrable->StateScatter(X, V, T, true);  // state -> system
    mintegrable->StateScatterAcceleration(A);  // -> system auxiliary data
    mintegrable->StateScatterReactions(L);     // -> system auxiliary data
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperEulerSemiImplicit)
CH_UPCASTING(ChTimestepperEulerSemiImplicit, ChTimestepperIIorder)


// Euler semi-implicit timestepper
// This performs the typical
//    v_new = v + a * dt
//    x_new = x + v_new * dt
// integration with Euler semi-implicit formula.
void ChTimestepperEulerSemiImplicit::Advance(const double dt) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    L.setZero(mintegrable->GetNconstr());

    mintegrable->StateGather(X, V, T);  // state <- system

    mintegrable->StateSolveA(A, L, X, V, T, dt, false, false);  // Dv/dt = f(x,v,T)   Dv = f(x,v,T)*dt

    // Semi-implicit Euler formula!   (note the order of update of x and v, respect to original Euler II order explicit)

    V = V + A * dt;  // v_new= v + a * dt

    X = X + V * dt;  // x_new= x + v_new * dt

    T += dt;

    mintegrable->StateScatter(X, V, T, true);  // state -> system
    mintegrable->StateScatterAcceleration(A);  // -> system auxiliary data
    mintegrable->StateScatterReactions(L);     // -> system auxiliary data
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperRungeKuttaExpl)
CH_UPCASTING(ChTimestepperRungeKuttaExpl, ChTimestepperIorder)


// Performs a step of a 4th order explicit Runge-Kutta integration scheme.
void ChTimestepperRungeKuttaExpl::Advance(const double dt) {
    // setup main vectors
    GetIntegrable()->StateSetup(Y, dYdt);

    // setup auxiliary vectors
    int n_y = GetIntegrable()->GetNcoords_y();
    int n_dy = GetIntegrable()->GetNcoords_dy();
    int n_c = GetIntegrable()->GetNconstr();
    y_new.setZero(n_y, GetIntegrable());
    Dydt1.setZero(n_dy, GetIntegrable());
    Dydt2.setZero(n_dy, GetIntegrable());
    Dydt3.setZero(n_dy, GetIntegrable());
    Dydt4.setZero(n_dy, GetIntegrable());
    L.setZero(n_c);

    GetIntegrable()->StateGather(Y, T);  // state <- system

    GetIntegrable()->StateSolve(Dydt1, L, Y, T, dt,
                                false,  // no need to scatter state before computation
                                false   // full update? (not used since no scatter)
    );  

    y_new = Y + Dydt1 * 0.5 * dt;  // integrable.StateIncrement(y_new, Y, Dydt1*0.5*dt);
    GetIntegrable()->StateSolve(Dydt2, L, y_new, T + dt * 0.5, dt, true, true);

    y_new = Y + Dydt2 * 0.5 * dt;  // integrable.StateIncrement(y_new, Y, Dydt2*0.5*dt);
    GetIntegrable()->StateSolve(Dydt3, L, y_new, T + dt * 0.5, dt, true, true);

    y_new = Y + Dydt3 * dt;  // integrable.StateIncrement(y_new, Y, Dydt3*dt);
    GetIntegrable()->StateSolve(Dydt4, L, y_new, T + dt, dt, true, true);

    Y = Y + (Dydt1 + Dydt2 * 2.0 + Dydt3 * 2.0 + Dydt4) * (1. / 6.) * dt;  // integrable.StateIncrement(...);
    dYdt = Dydt4;                                                          // to check
    T += dt;

    GetIntegrable()->StateScatter(Y, T, true);            // state -> system
    GetIntegrable()->StateScatterDerivative(dYdt);  // -> system auxiliary data
    GetIntegrable()->StateScatterReactions(L);      // -> system auxiliary data
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperHeun)
CH_UPCASTING(ChTimestepperHeun, ChTimestepperIorder)


// Performs a step of a Heun explicit integrator. It is like a 2nd Runge Kutta.
void ChTimestepperHeun::Advance(const double dt) {
    // setup main vectors
    GetIntegrable()->StateSetup(Y, dYdt);

    // setup auxiliary vectors
    int n_y = GetIntegrable()->GetNcoords_y();
    int n_dy = GetIntegrable()->GetNcoords_dy();
    int n_c = GetIntegrable()->GetNconstr();
    y_new.setZero(n_y, GetIntegrable());
    Dydt1.setZero(n_dy, GetIntegrable());
    Dydt2.setZero(n_dy, GetIntegrable());
    L.setZero(n_c);

    GetIntegrable()->StateGather(Y, T);  // state <- system

    GetIntegrable()->StateSolve(Dydt1, L, Y, T, dt,
                                false,  // no need to scatter state before computation
                                false   // full update? ( not used, since no scatter)
    );
    y_new = Y + Dydt1 * dt;
    GetIntegrable()->StateSolve(Dydt2, L, y_new, T + dt, dt, true, true);

    Y = Y + (Dydt1 + Dydt2) * (dt / 2.);
    dYdt = Dydt2;
    T += dt;

    GetIntegrable()->StateScatter(Y, T, true);            // state -> system
    GetIntegrable()->StateScatterDerivative(dYdt);  // -> system auxiliary data
    GetIntegrable()->StateScatterReactions(L);      // -> system auxiliary data
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperLeapfrog)
CH_UPCASTING(ChTimestepperLeapfrog, ChTimestepperIIorder)


// Performs a step of a Leapfrog explicit integrator.
// It is a symplectic method, with 2nd order accuracy,
// at least when F depends on positions only.
// Note: uses last step acceleration: changing or resorting
// the numbering of DOFs will invalidate it.
// Suggestion: use the ChTimestepperEulerSemiImplicit, it gives
// the same accuracy with a bit of faster performance.
void ChTimestepperLeapfrog::Advance(const double dt) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    L.setZero(mintegrable->GetNconstr());
    Aold.setZero(mintegrable->GetNcoords_v(), GetIntegrable());

    mintegrable->StateGather(X, V, T);  // state <- system
    mintegrable->StateGatherAcceleration(Aold);

    // advance X (uses last A)
    X = X + V * dt + Aold * (0.5 * dt * dt);

    // computes new A  (NOTE!!true for imposing a state-> system scatter update,because X changed..)
    mintegrable->StateSolveA(A, L, X, V, T, dt, true, true);  // Dv/dt = f(x,v,T)   Dv = f(x,v,T)*dt

    // advance V

    V = V + (Aold + A) * (0.5 * dt);

    T += dt;

    mintegrable->StateScatter(X, V, T, true);  // state -> system
    mintegrable->StateScatterAcceleration(A);  // -> system auxiliary data
    mintegrable->StateScatterReactions(L);     // -> system auxiliary data
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperEulerImplicit)
CH_UPCASTING(ChTimestepperEulerImplicit, ChTimestepperIIorder)
CH_UPCASTING(ChTimestepperEulerImplicit, ChImplicitIterativeTimestepper)

// Performs a step of Euler implicit for II order systems
void ChTimestepperEulerImplicit::Advance(const double dt) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Dv.setZero(mintegrable->GetNcoords_v(), GetIntegrable());
    Dl.setZero(mintegrable->GetNconstr());
    Xnew.setZero(mintegrable->GetNcoords_x(), mintegrable);
    Vnew.setZero(mintegrable->GetNcoords_v(), mintegrable);
    R.setZero(mintegrable->GetNcoords_v());
    Qc.setZero(mintegrable->GetNconstr());
    L.setZero(mintegrable->GetNconstr());

    mintegrable->StateGather(X, V, T);  // state <- system

    // Extrapolate a prediction as warm start

    Xnew = X + V * dt;
    Vnew = V;  //+ A()*dt;

    // use Newton Raphson iteration to solve implicit Euler for v_new
    //
    // [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ Dv     ] = [ M*(v_old - v_new) + dt*f + dt*Cq'*l ]
    // [ Cq                           0   ] [ -dt*Dl ] = [ -C/dt  ]

    numiters = 0;
    numsetups = 0;
    numsolves = 0;

    for (int i = 0; i < this->GetMaxiters(); ++i) {
        mintegrable->StateScatter(Xnew, Vnew, T + dt, false);  // state -> system
        R.setZero();
        Qc.setZero();
        mintegrable->LoadResidual_F(R, dt);
        mintegrable->LoadResidual_Mv(R, (V - Vnew), 1.0);
        mintegrable->LoadResidual_CqL(R, L, dt);
        mintegrable->LoadConstraint_C(Qc, 1.0 / dt, Qc_do_clamp, Qc_clamping);

        if (verbose)
            GetLog() << " Euler iteration=" << i << "  |R|=" << R.lpNorm<Eigen::Infinity>()
                     << "  |Qc|=" << Qc.lpNorm<Eigen::Infinity>() << "\n";

        if ((R.lpNorm<Eigen::Infinity>() < abstolS) && (Qc.lpNorm<Eigen::Infinity>() < abstolL))
            break;

        mintegrable->StateSolveCorrection(  //
            Dv, Dl, R, Qc,                  //
            1.0,                            // factor for  M
            -dt,                            // factor for  dF/dv
            -dt * dt,                       // factor for  dF/dx
            Xnew, Vnew, T + dt,             // not used here (scatter = false)
            false,                          // do not scatter update to Xnew Vnew T+dt before computing correction
            false,                          // full update? (not used, since no scatter)
            true                            // always call the solver's Setup
        );

        numiters++;
        numsetups++;
        numsolves++;

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

    mintegrable->StateScatter(X, V, T, true);     // state -> system
    mintegrable->StateScatterReactions(L);  // -> system auxiliary data
}

void ChTimestepperEulerImplicit::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperEulerImplicit>();
    // serialize parent class:
    ChTimestepperIIorder::ArchiveOut(archive);
    ChImplicitIterativeTimestepper::ArchiveOut(archive);
}

void ChTimestepperEulerImplicit::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/ archive.VersionRead<ChTimestepperEulerImplicit>();
    // deserialize parent class:
    ChTimestepperIIorder::ArchiveIn(archive);
    ChImplicitIterativeTimestepper::ArchiveIn(archive);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperEulerImplicitLinearized)
CH_UPCASTING(ChTimestepperEulerImplicitLinearized, ChTimestepperIIorder)
CH_UPCASTING(ChTimestepperEulerImplicitLinearized, ChImplicitTimestepper)

// Performs a step of Euler implicit for II order systems
// using the Anitescu/Stewart/Trinkle single-iteration method,
// that is a bit like an implicit Euler where one performs only
// the first NR corrector iteration.
// If the solver in StateSolveCorrection is a CCP complementarity
// solver, this is the typical Anitescu stabilized timestepper for DVIs.
void ChTimestepperEulerImplicitLinearized::Advance(const double dt) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Dl.setZero(mintegrable->GetNconstr());
    R.setZero(mintegrable->GetNcoords_v());
    Qc.setZero(mintegrable->GetNconstr());
    L.setZero(mintegrable->GetNconstr());

    mintegrable->StateGather(X, V, T);  // state <- system

    mintegrable->StateGatherReactions(L);  // state <- system (may be needed for warm starting StateSolveCorrection)
    L *= dt;                               // because reactions = forces, here L = impulses

    Vold = V;

    // solve only 1st NR step, using v_new = 0, so  Dv = v_new , therefore
    //
    // [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ Dv     ] = [ M*(v_old - v_new) + dt*f]
    // [ Cq                           0   ] [ -dt*Dl ] = [ -C/dt - Ct ]
    //
    // becomes the Anitescu/Trinkle timestepper:
    //
    // [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ v_new  ] = [ M*(v_old) + dt*f]
    // [ Cq                           0   ] [ -dt*l  ] = [ -C/dt - Ct ]

    mintegrable->LoadResidual_F(R, dt);
    mintegrable->LoadResidual_Mv(R, V, 1.0);
    mintegrable->LoadConstraint_C(Qc, 1.0 / dt, Qc_do_clamp, Qc_clamping);
    mintegrable->LoadConstraint_Ct(Qc, 1.0);

    mintegrable->StateSolveCorrection(  //
        V, L, R, Qc,                    //
        1.0,                            // factor for  M
        -dt,                            // factor for  dF/dv
        -dt * dt,                       // factor for  dF/dx
        X, V, T + dt,                   // not needed
        false,                          // do not scatter update to Xnew Vnew T+dt before computing correction
        false,                          // full update? (not used, since no scatter)
        true                            // force a call to the solver's Setup() function
    );

    L *= (1.0 / dt);  // Note it is not -(1.0/dt) because we assume StateSolveCorrection already flips sign of Dl

    mintegrable->StateScatterAcceleration(
        (V - Vold) * (1 / dt));  // -> system auxiliary data (i.e acceleration as measure, fits DVI/MDI)

    X += V * dt;

    T += dt;

    mintegrable->StateScatter(X, V, T, true);  // state -> system
    mintegrable->StateScatterReactions(L);     // -> system auxiliary data
}


// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperEulerImplicitProjected)
CH_UPCASTING(ChTimestepperEulerImplicitProjected, ChTimestepperIIorder)
CH_UPCASTING(ChTimestepperEulerImplicitProjected, ChImplicitTimestepper)

// Performs a step of Euler implicit for II order systems
// using a semi implicit Euler without constr.stabilization, followed by a projection,
// that is: a speed problem followed by a position problem that
// keeps constraint drifting 'closed' by using a projection.
// If the solver in StateSolveCorrection is a CCP complementarity
// solver, this is the Tasora stabilized timestepper for DVIs.
void ChTimestepperEulerImplicitProjected::Advance(const double dt) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Dl.setZero(mintegrable->GetNconstr());
    R.setZero(mintegrable->GetNcoords_v());
    Qc.setZero(mintegrable->GetNconstr());
    L.setZero(mintegrable->GetNconstr());

    mintegrable->StateGather(X, V, T);  // state <- system

    Vold = V;

    // 1
    // Do a  Anitescu/Trinkle timestepper (it could be without the C/dt correction):
    //
    // [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ v_new  ] = [ M*(v_old) + dt*f]
    // [ Cq                           0   ] [ -dt*l  ] = [ -Ct ]

    mintegrable->LoadResidual_F(R, dt);
    mintegrable->LoadResidual_Mv(R, V, 1.0);
    mintegrable->LoadConstraint_C(Qc, 1.0 / dt, Qc_do_clamp, 0);  // may be avoided
    mintegrable->LoadConstraint_Ct(Qc, 1.0);

    mintegrable->StateSolveCorrection(  //
        V, L, R, Qc,                    //
        1.0,                            // factor for  M
        -dt,                            // factor for  dF/dv
        -dt * dt,                       // factor for  dF/dx
        X, V, T + dt,                   // not needed
        false,                          // do not scatter update to Xnew Vnew T+dt before computing correction
        false,                          // full update? (not used, since no scatter)
        true                            // force a call to the solver's Setup() function
    );

    L *= (1.0 / dt);  // Note it is not -(1.0/dt) because we assume StateSolveCorrection already flips sign of Dl

    mintegrable->StateScatterAcceleration(
        (V - Vold) * (1 / dt));  // -> system auxiliary data (i.e acceleration as measure, fits DVI/MDI)

    X += V * dt;

    T += dt;

    mintegrable->StateScatter(X, V, T, false);  // state -> system
    mintegrable->StateScatterReactions(L);      // -> system auxiliary data

    // 2
    // Do the position stabilization (single NR step on constraints, with mass matrix as metric)

    Dl.setZero(mintegrable->GetNconstr());
    R.setZero(mintegrable->GetNcoords_v());
    Qc.setZero(mintegrable->GetNconstr());
    L.setZero(mintegrable->GetNconstr());
    Vold.setZero(mintegrable->GetNcoords_v(), V.GetIntegrable());

    //
    // [ M       Cq' ] [ dpos ] = [  0 ]
    // [ Cq       0  ] [ l    ] = [ -C ]

    mintegrable->LoadConstraint_C(Qc, 1.0, false, 0);

    mintegrable->StateSolveCorrection(  //
        Vold, L, R, Qc,                 //
        1.0,                            // factor for  M
        0,                              // factor for  dF/dv
        0,                              // factor for  dF/dx
        X, V, T,                        // not needed
        false,                          // do not scatter update to Xnew Vnew T+dt before computing correction
        false,                          // full update? (not used, since no scatter)
        true                            // force a call to the solver's Setup() function
    );

    X += Vold;  // here we used 'Vold' as 'dpos' to recycle Vold and avoid allocating a new vector dpos

    mintegrable->StateScatter(X, V, T, true);  // state -> system
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperTrapezoidal)
CH_UPCASTING(ChTimestepperTrapezoidal, ChTimestepperIIorder)
CH_UPCASTING(ChTimestepperTrapezoidal, ChImplicitIterativeTimestepper)

// Performs a step of trapezoidal implicit for II order systems
// NOTE this is a modified version of the trapezoidal for DAE: the
// original derivation would lead to a scheme that produces oscillatory
// reactions in constraints, so this is a modified version that is first
// order in constraint reactions. Use damped HHT or damped Newmark for
// more advanced options.
void ChTimestepperTrapezoidal::Advance(const double dt) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Dv.setZero(mintegrable->GetNcoords_v(), GetIntegrable());
    Dl.setZero(mintegrable->GetNconstr());
    Xnew.setZero(mintegrable->GetNcoords_x(), mintegrable);
    Vnew.setZero(mintegrable->GetNcoords_v(), mintegrable);
    L.setZero(mintegrable->GetNconstr());
    R.setZero(mintegrable->GetNcoords_v());
    Rold.setZero(mintegrable->GetNcoords_v());
    Qc.setZero(mintegrable->GetNconstr());

    mintegrable->StateGather(X, V, T);  // state <- system
    // mintegrable->StateGatherReactions(L); // <- system  assume l_old = 0;  otherwise DAE gives oscillatory reactions

    // extrapolate a prediction as a warm start

    Xnew = X + V * dt;
    Vnew = V;  // +A()*dt;

    // use Newton Raphson iteration to solve implicit trapezoidal for v_new
    //
    // [M-dt/2*dF/dv-dt^2/4*dF/dx  Cq'] [Dv      ] = [M*(v_old - v_new) + dt/2(f_old + f_new  + Cq*l_old + Cq*l_new)]
    // [Cq                          0 ] [-dt/2*Dl] = [-C/dt                                                         ]

    mintegrable->LoadResidual_F(Rold, dt * 0.5);  // dt/2*f_old
    mintegrable->LoadResidual_Mv(Rold, V, 1.0);   // M*v_old
    // mintegrable->LoadResidual_CqL(Rold, L, dt*0.5); // dt/2*l_old   assume L_old = 0

    numiters = 0;
    numsetups = 0;
    numsolves = 0;

    for (int i = 0; i < this->GetMaxiters(); ++i) {
        mintegrable->StateScatter(Xnew, Vnew, T + dt, false);  // state -> system
        R = Rold;
        Qc.setZero();
        mintegrable->LoadResidual_F(R, dt * 0.5);                               // + dt/2*f_new
        mintegrable->LoadResidual_Mv(R, Vnew, -1.0);                            // - M*v_new
        mintegrable->LoadResidual_CqL(R, L, dt * 0.5);                          // + dt/2*Cq*l_new
        mintegrable->LoadConstraint_C(Qc, 1.0 / dt, Qc_do_clamp, Qc_clamping);  // -C/dt

        if (verbose)
            GetLog() << " Trapezoidal iteration=" << i << "  |R|=" << R.lpNorm<Eigen::Infinity>()
                     << "  |Qc|=" << Qc.lpNorm<Eigen::Infinity>() << "\n";

        if ((R.lpNorm<Eigen::Infinity>() < abstolS) && (Qc.lpNorm<Eigen::Infinity>() < abstolL))
            break;

        mintegrable->StateSolveCorrection(  //
            Dv, Dl, R, Qc,                  //
            1.0,                            // factor for  M
            -dt * 0.5,                      // factor for  dF/dv
            -dt * dt * 0.25,                // factor for  dF/dx
            Xnew, Vnew, T + dt,             // not used here (scatter = false)
            false,                          // do not scatter update to Xnew Vnew T+dt before computing correction
            false,                          // full update? (not used, since no scatter)
            true                            // always force a call to the solver's Setup() function
        );

        numiters++;
        numsetups++;
        numsolves++;

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

    mintegrable->StateScatter(X, V, T, true);  // state -> system
    mintegrable->StateScatterReactions(L *=
                                       0.5);  // -> system auxiliary data   (*=0.5 cause we used the hack of l_old = 0)
}

void ChTimestepperTrapezoidal::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperTrapezoidal>();
    // serialize parent class:
    ChTimestepperIIorder::ArchiveOut(archive);
    ChImplicitIterativeTimestepper::ArchiveOut(archive);
}

void ChTimestepperTrapezoidal::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/ archive.VersionRead<ChTimestepperTrapezoidal>();
    // deserialize parent class:
    ChTimestepperIIorder::ArchiveIn(archive);
    ChImplicitIterativeTimestepper::ArchiveIn(archive);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperTrapezoidalLinearized)
CH_UPCASTING(ChTimestepperTrapezoidalLinearized, ChTimestepperIIorder)
CH_UPCASTING(ChTimestepperTrapezoidalLinearized, ChImplicitIterativeTimestepper)

// Performs a step of trapezoidal implicit linearized for II order systems
void ChTimestepperTrapezoidalLinearized::Advance(const double dt) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Dv.setZero(mintegrable->GetNcoords_v(), GetIntegrable());
    Dl.setZero(mintegrable->GetNconstr());
    Xnew.setZero(mintegrable->GetNcoords_x(), mintegrable);
    Vnew.setZero(mintegrable->GetNcoords_v(), mintegrable);
    L.setZero(mintegrable->GetNconstr());
    R.setZero(mintegrable->GetNcoords_v());
    Rold.setZero(mintegrable->GetNcoords_v());
    Qc.setZero(mintegrable->GetNconstr());

    mintegrable->StateGather(X, V, T);  // state <- system
    // mintegrable->StateGatherReactions(L); // <- system  assume l_old = 0;

    // extrapolate a prediction as a warm start

    Xnew = X + V * dt;
    Vnew = V;

    // solve implicit trapezoidal for v_new
    //
    // [M-dt/2*dF/dv-dt^2/4*dF/dx  Cq'] [Dv      ] = [M*(v_old - v_new) + dt/2(f_old + f_new  + Cq*l_old + Cq*l_new)]
    // [Cq                          0 ] [-dt/2*Dl] = [-C/dt                                                         ]

    mintegrable->LoadResidual_F(Rold, dt * 0.5);  // dt/2*f_old
    mintegrable->LoadResidual_Mv(Rold, V, 1.0);   // M*v_old
    // mintegrable->LoadResidual_CqL(Rold, L, dt*0.5); // dt/2*l_old  assume l_old = 0;

    mintegrable->StateScatter(Xnew, Vnew, T + dt, false);  // state -> system
    R = Rold;
    Qc.setZero();
    mintegrable->LoadResidual_F(R, dt * 0.5);     // + dt/2*f_new
    mintegrable->LoadResidual_Mv(R, Vnew, -1.0);  // - M*v_new
    // mintegrable->LoadResidual_CqL(R, L, dt*0.5); // + dt/2*Cq*l_new  assume l_old = 0;
    mintegrable->LoadConstraint_C(Qc, 1.0 / dt, Qc_do_clamp, Qc_clamping);  // -C/dt

    mintegrable->StateSolveCorrection(  //
        Dv, Dl, R, Qc,                  //
        1.0,                            // factor for  M
        -dt * 0.5,                      // factor for  dF/dv
        -dt * dt * 0.25,                // factor for  dF/dx
        Xnew, Vnew, T + dt,             // not used here (scatter = false)
        false,                          // do not scatter update to Xnew Vnew T+dt before computing correction
        false,                          // full update? (not used, since no scatter)
        true                            // force a call to the solver's Setup() function
    );

    numiters = 1;
    numsetups = 1;
    numsolves = 1;

    Dl *= (2.0 / dt);  // Note it is not -(2.0/dt) because we assume StateSolveCorrection already flips sign of Dl
    L += Dl;

    Vnew += Dv;

    Xnew = X + ((Vnew + V) * (dt * 0.5));  // Xnew = Xold + h/2(Vnew+Vold)

    X = Xnew;
    V = Vnew;
    T += dt;

    mintegrable->StateScatter(X, V, T, true);                 // state -> system
    mintegrable->StateScatterAcceleration((Dv *= (1 / dt)));  // -> system auxiliary data (i.e acceleration as measure)
    mintegrable->StateScatterReactions(L *= 0.5);             // -> system auxiliary data (*=0.5 because use l_old = 0)
}

void ChTimestepperTrapezoidalLinearized::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperTrapezoidalLinearized>();
    // serialize parent class:
    ChTimestepperIIorder::ArchiveOut(archive);
    ChImplicitIterativeTimestepper::ArchiveOut(archive);
}

void ChTimestepperTrapezoidalLinearized::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/ archive.VersionRead<ChTimestepperTrapezoidalLinearized>();
    // deserialize parent class:
    ChTimestepperIIorder::ArchiveIn(archive);
    ChImplicitIterativeTimestepper::ArchiveIn(archive);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperTrapezoidalLinearized2)
CH_UPCASTING(ChTimestepperTrapezoidalLinearized2, ChTimestepperIIorder)
CH_UPCASTING(ChTimestepperTrapezoidalLinearized2, ChImplicitIterativeTimestepper)

// Performs a step of trapezoidal implicit linearized for II order systems
//*** SIMPLIFIED VERSION -DOES NOT WORK - PREFER ChTimestepperTrapezoidalLinearized
void ChTimestepperTrapezoidalLinearized2::Advance(const double dt) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Dv.setZero(mintegrable->GetNcoords_v(), GetIntegrable());
    Xnew.setZero(mintegrable->GetNcoords_x(), mintegrable);
    Vnew.setZero(mintegrable->GetNcoords_v(), mintegrable);
    L.setZero(mintegrable->GetNconstr());
    R.setZero(mintegrable->GetNcoords_v());
    Qc.setZero(mintegrable->GetNconstr());

    mintegrable->StateGather(X, V, T);  // state <- system

    // extrapolate a prediction as a warm start

    Xnew = X + V * dt;
    Vnew = V;

    // use Newton Raphson iteration to solve implicit trapezoidal for v_new
    //
    // [ M - dt/2*dF/dv - dt^2/4*dF/dx    Cq' ] [ v_new    ] = [ M*(v_old) + dt/2(f_old + f_new)]
    // [ Cq                               0   ] [ -dt/2*L ] m= [ -C/dt                          ]

    mintegrable->LoadResidual_F(R, dt * 0.5);  // dt/2*f_old
    mintegrable->LoadResidual_Mv(R, V, 1.0);   // M*v_old

    mintegrable->StateScatter(Xnew, Vnew, T + dt, false);  // state -> system
    Qc.setZero();
    mintegrable->LoadResidual_F(R, dt * 0.5);                               // + dt/2*f_new
    mintegrable->LoadConstraint_C(Qc, 1.0 / dt, Qc_do_clamp, Qc_clamping);  // -C/dt

    mintegrable->StateSolveCorrection(  //
        Vnew, L, R, Qc,                 //
        1.0,                            // factor for  M
        -dt * 0.5,                      // factor for  dF/dv
        -dt * dt * 0.25,                // factor for  dF/dx
        Xnew, Vnew, T + dt,             // not used here (scatter = false)
        false,                          // do not scatter update to Xnew Vnew T+dt before computing correction
        false,                          // full update? (not used, since no scatter)
        true                            // force a call to the solver's Setup() function
    );

    numiters = 1;
    numsetups = 1;
    numsolves = 1;

    L *= (2.0 / dt);  // Note it is not -(2.0/dt) because we assume StateSolveCorrection already flips sign of Dl

    X += ((Vnew + V) * (dt * 0.5));  // Xnew = Xold + h/2(Vnew+Vold)

    mintegrable->StateScatterAcceleration(
        (Vnew - V) * (1 / dt));  // -> system auxiliary data (i.e acceleration as measure, fits DVI/MDI)

    V = Vnew;

    T += dt;

    mintegrable->StateScatter(X, V, T, true);  // state -> system
    mintegrable->StateScatterReactions(L);     // -> system auxiliary data
}


void ChTimestepperTrapezoidalLinearized2::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperTrapezoidalLinearized2>();
    // serialize parent class:
    ChTimestepperIIorder::ArchiveOut(archive);
    ChImplicitIterativeTimestepper::ArchiveOut(archive);
}

void ChTimestepperTrapezoidalLinearized2::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/ archive.VersionRead<ChTimestepperTrapezoidalLinearized2>();
    // deserialize parent class:
    ChTimestepperIIorder::ArchiveIn(archive);
    ChImplicitIterativeTimestepper::ArchiveIn(archive);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperNewmark)
CH_UPCASTING(ChTimestepperNewmark, ChTimestepperIIorder)
CH_UPCASTING(ChTimestepperNewmark, ChImplicitIterativeTimestepper)

// Set the numerical damping parameter gamma and the beta parameter.
void ChTimestepperNewmark::SetGammaBeta(double mgamma, double mbeta) {
    gamma = mgamma;
    if (gamma < 0.5)
        gamma = 0.5;
    if (gamma > 1)
        gamma = 1;
    beta = mbeta;
    if (beta < 0)
        beta = 0;
    if (beta > 1)
        beta = 1;
}

// Performs a step of Newmark constrained implicit for II order DAE systems
void ChTimestepperNewmark::Advance(const double dt) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Da.setZero(mintegrable->GetNcoords_a(), GetIntegrable());
    Dl.setZero(mintegrable->GetNconstr());
    Xnew.setZero(mintegrable->GetNcoords_x(), mintegrable);
    Vnew.setZero(mintegrable->GetNcoords_v(), mintegrable);
    Anew.setZero(mintegrable->GetNcoords_a(), mintegrable);
    R.setZero(mintegrable->GetNcoords_v());
    Rold.setZero(mintegrable->GetNcoords_v());
    Qc.setZero(mintegrable->GetNconstr());
    L.setZero(mintegrable->GetNconstr());

    mintegrable->StateGather(X, V, T);  // state <- system
    mintegrable->StateGatherAcceleration(A);

    // extrapolate a prediction as a warm start

    Vnew = V;
    Xnew = X + Vnew * dt;

    // use Newton Raphson iteration to solve implicit Newmark for a_new

    //
    // [ M - dt*gamma*dF/dv - dt^2*beta*dF/dx    Cq' ] [ Da   ] = [ -M*(a_new) + f_new + Cq*l_new ]
    // [ Cq                                      0   ] [ Dl   ] = [ -1/(beta*dt^2)*C              ]

    numiters = 0;
    numsetups = 0;
    numsolves = 0;
    bool call_setup = true;

    for (int i = 0; i < this->GetMaxiters(); ++i) {
        mintegrable->StateScatter(Xnew, Vnew, T + dt, false);  // state -> system

        R.setZero(mintegrable->GetNcoords_v());
        Qc.setZero(mintegrable->GetNconstr());
        mintegrable->LoadResidual_F(R, 1.0);                                                    //  f_new
        mintegrable->LoadResidual_CqL(R, L, 1.0);                                               //   Cq'*l_new
        mintegrable->LoadResidual_Mv(R, Anew, -1.0);                                            //  - M*a_new
        mintegrable->LoadConstraint_C(Qc, (1.0 / (beta * dt * dt)), Qc_do_clamp, Qc_clamping);  //  - 1/(beta*dt^2)*C

        if (verbose)
            GetLog() << " Newmark iteration=" << i << "  |R|=" << R.lpNorm<Eigen::Infinity>()
                     << "  |Qc|=" << Qc.lpNorm<Eigen::Infinity>() << "\n";

        if ((R.lpNorm<Eigen::Infinity>() < abstolS) && (Qc.lpNorm<Eigen::Infinity>() < abstolL)) {
            if (verbose) {
                GetLog() << " Newmark NR converged (" << i << ")." << "  T = " << T + dt << "  h = " << dt << "\n";
            }
            break;
        }

        if (verbose && modified_Newton && call_setup)
                GetLog() << " Newmark call Setup.\n";

        mintegrable->StateSolveCorrection(  //
            Da, Dl, R, Qc,                  //
            1.0,                            // factor for  M
            -dt * gamma,                    // factor for  dF/dv
            -dt * dt * beta,                // factor for  dF/dx
            Xnew, Vnew, T + dt,             // not used here (scatter = false)
            false,                          // do not scatter update to Xnew Vnew T+dt before computing correction
            false,                          // full update? (not used, since no scatter)
            call_setup                      // force a call to the solver's Setup() function
        );

        numiters++;
        numsolves++;
        if (call_setup) {
            numsetups++;
        }

        // If using modified Newton, do not call Setup again
        call_setup = !modified_Newton;

        L += Dl;  // Note it is not -= Dl because we assume StateSolveCorrection flips sign of Dl
        Anew += Da;

        Xnew = X + V * dt + A * (dt * dt * (0.5 - beta)) + Anew * (dt * dt * beta);

        Vnew = V + A * (dt * (1.0 - gamma)) + Anew * (dt * gamma);
    }

    X = Xnew;
    V = Vnew;
    A = Anew;
    T += dt;

    mintegrable->StateScatter(X, V, T, true);  // state -> system
    mintegrable->StateScatterAcceleration(A);  // -> system auxiliary data
    mintegrable->StateScatterReactions(L);     // -> system auxiliary data
}

void ChTimestepperNewmark::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperNewmark>();
    // serialize parent class:
    ChTimestepperIIorder::ArchiveOut(archive);
    ChImplicitIterativeTimestepper::ArchiveOut(archive);
    // serialize all member data:
    archive << CHNVP(beta);
    archive << CHNVP(gamma);
}

void ChTimestepperNewmark::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/ archive.VersionRead<ChTimestepperNewmark>();
    // deserialize parent class:
    ChTimestepperIIorder::ArchiveIn(archive);
    ChImplicitIterativeTimestepper::ArchiveIn(archive);
    // stream in all member data:
    archive >> CHNVP(beta);
    archive >> CHNVP(gamma);
}

}  // end namespace chrono
