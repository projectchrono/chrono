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

#include "chrono/timestepper/ChTimestepperExplicit.h"

namespace chrono {

// -----------------------------------------------------------------------------

ChTimestepperExplicit::ChTimestepperExplicit() : lumping_parameters(nullptr) {}

ChTimestepperExplicit::~ChTimestepperExplicit() {
    if (lumping_parameters)
        delete (lumping_parameters);
}

void ChTimestepperExplicit::SetDiagonalLumpingON(double Ck, double Cr) {
    lumping_parameters = new ChLumpingParms(Ck, Cr);
}

void ChTimestepperExplicit::SetDiagonalLumpingOFF() {
    if (lumping_parameters)
        delete (lumping_parameters);
}

double ChTimestepperExplicit::GetLumpingError() {
    if (this->lumping_parameters)
        return lumping_parameters->error;
    else
        return 0;
}

void ChTimestepperExplicit::ResetLumpingError() {
    if (this->lumping_parameters)
        lumping_parameters->error = 0;
}

void ChTimestepperExplicit::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite(1);
    // serialize all member data:
    archive << CHNVP(lumping_parameters);
}

void ChTimestepperExplicit::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead();
    // stream in all member data:
    archive >> CHNVP(lumping_parameters);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperEulerExpl)
CH_UPCASTING(ChTimestepperEulerExpl, ChTimestepperIorder)
CH_UPCASTING(ChTimestepperEulerExpl, ChTimestepperExplicit)

// Euler explicit timestepper.
// This performs the typical  y_new = y+ dy/dt * dt integration with Euler formula.
void ChTimestepperEulerExpl::Advance(double dt) {
    // setup main vectors
    GetIntegrable()->StateSetup(Y, dYdt);

    // setup auxiliary vectors
    L.setZero(GetIntegrable()->GetNumConstraints());

    GetIntegrable()->StateGather(Y, T);  // state <- system

    GetIntegrable()->StateSolve(dYdt, L, Y, T, dt, false, false, lumping_parameters);  // dY/dt = f(Y,T)

    // Euler formula!
    //   y_new= y + dy/dt * dt

    Y = Y + dYdt * dt;  //  also: GetIntegrable().StateIncrement(y_new, y, Dy);

    T += dt;

    GetIntegrable()->StateScatter(Y, T, true);      // state -> system
    GetIntegrable()->StateScatterDerivative(dYdt);  // -> system auxiliary data
    GetIntegrable()->StateScatterReactions(L);      // -> system auxiliary data
}

void ChTimestepperEulerExpl::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperEulerExpl>();
    // serialize parent class:
    ChTimestepperIorder::ArchiveOut(archive);
    ChTimestepperExplicit::ArchiveOut(archive);
}
void ChTimestepperEulerExpl::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChTimestepperEulerExpl>();
    // deserialize parent class:
    ChTimestepperIorder::ArchiveIn(archive);
    ChTimestepperExplicit::ArchiveIn(archive);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperEulerExplIIorder)
CH_UPCASTING(ChTimestepperEulerExplIIorder, ChTimestepperIIorder)
CH_UPCASTING(ChTimestepperEulerExplIIorder, ChTimestepperExplicit)

// Euler explicit timestepper customized for II order.
// (It gives the same results of ChTimestepperEulerExpl,
// but this performs a bit faster because it can exploit
// the special structure of ChIntegrableIIorder)
// This performs the typical
//    x_new = x + v * dt
//    v_new = v + a * dt
// integration with Euler formula.
void ChTimestepperEulerExplIIorder::Advance(double dt) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Dv.setZero(mintegrable->GetNumCoordsVelLevel(), GetIntegrable());
    L.setZero(mintegrable->GetNumConstraints());

    mintegrable->StateGather(X, V, T);  // state <- system

    mintegrable->StateSolveA(A, L, X, V, T, dt, false, false, lumping_parameters);  // Dv/dt = f(x,v,T)

    // Euler formula!

    X = X + V * dt;  // x_new= x + v * dt

    V = V + A * dt;  // v_new= v + a * dt

    T += dt;

    mintegrable->StateScatter(X, V, T, true);  // state -> system
    mintegrable->StateScatterAcceleration(A);  // -> system auxiliary data
    mintegrable->StateScatterReactions(L);     // -> system auxiliary data
}

void ChTimestepperEulerExplIIorder::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperEulerExplIIorder>();
    // serialize parent class:
    ChTimestepperIIorder::ArchiveOut(archive);
    ChTimestepperExplicit::ArchiveOut(archive);
}
void ChTimestepperEulerExplIIorder::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChTimestepperEulerExplIIorder>();
    // deserialize parent class:
    ChTimestepperIIorder::ArchiveIn(archive);
    ChTimestepperExplicit::ArchiveIn(archive);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperEulerSemiImplicit)
CH_UPCASTING(ChTimestepperEulerSemiImplicit, ChTimestepperIIorder)
CH_UPCASTING(ChTimestepperEulerSemiImplicit, ChTimestepperExplicit)

// Euler semi-implicit timestepper
// This performs the typical
//    v_new = v + a * dt
//    x_new = x + v_new * dt
// integration with Euler semi-implicit formula.
void ChTimestepperEulerSemiImplicit::Advance(double dt) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    L.setZero(mintegrable->GetNumConstraints());

    mintegrable->StateGather(X, V, T);  // state <- system

    mintegrable->StateSolveA(A, L, X, V, T, dt, false, false,
                             lumping_parameters);  // Dv/dt = f(x,v,T)   Dv = f(x,v,T)*dt

    // Semi-implicit Euler formula!   (note the order of update of x and v, respect to original Euler II order explicit)

    V = V + A * dt;  // v_new= v + a * dt

    X = X + V * dt;  // x_new= x + v_new * dt

    T += dt;

    mintegrable->StateScatter(X, V, T, true);  // state -> system
    mintegrable->StateScatterAcceleration(A);  // -> system auxiliary data
    mintegrable->StateScatterReactions(L);     // -> system auxiliary data
}

void ChTimestepperEulerSemiImplicit::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperEulerSemiImplicit>();
    // serialize parent class:
    ChTimestepperIIorder::ArchiveOut(archive);
    ChTimestepperExplicit::ArchiveOut(archive);
}
void ChTimestepperEulerSemiImplicit::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChTimestepperEulerSemiImplicit>();
    // deserialize parent class:
    ChTimestepperIIorder::ArchiveIn(archive);
    ChTimestepperExplicit::ArchiveIn(archive);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperRungeKuttaExpl)
CH_UPCASTING(ChTimestepperRungeKuttaExpl, ChTimestepperIorder)
CH_UPCASTING(ChTimestepperRungeKuttaExpl, ChTimestepperExplicit)

// Performs a step of a 4th order explicit Runge-Kutta integration scheme.
void ChTimestepperRungeKuttaExpl::Advance(double dt) {
    // setup main vectors
    GetIntegrable()->StateSetup(Y, dYdt);

    // setup auxiliary vectors
    int n_y = (GetIntegrable()->GetNumCoordsPosLevel() + GetIntegrable()->GetNumCoordsVelLevel());
    int n_dy = (GetIntegrable()->GetNumCoordsVelLevel() + GetIntegrable()->GetNumCoordsAccLevel());
    int n_c = GetIntegrable()->GetNumConstraints();
    y_new.setZero(n_y, GetIntegrable());
    Dydt1.setZero(n_dy, GetIntegrable());
    Dydt2.setZero(n_dy, GetIntegrable());
    Dydt3.setZero(n_dy, GetIntegrable());
    Dydt4.setZero(n_dy, GetIntegrable());
    L.setZero(n_c);

    GetIntegrable()->StateGather(Y, T);  // state <- system

    GetIntegrable()->StateSolve(Dydt1, L, Y, T, dt,
                                false,              // no need to scatter state before computation
                                false,              // full update? (not used since no scatter)
                                lumping_parameters  // optional lumping?
    );

    y_new = Y + Dydt1 * 0.5 * dt;  // integrable.StateIncrement(y_new, Y, Dydt1*0.5*dt);
    GetIntegrable()->StateSolve(Dydt2, L, y_new, T + dt * 0.5, dt, true, true, lumping_parameters);

    y_new = Y + Dydt2 * 0.5 * dt;  // integrable.StateIncrement(y_new, Y, Dydt2*0.5*dt);
    GetIntegrable()->StateSolve(Dydt3, L, y_new, T + dt * 0.5, dt, true, true, lumping_parameters);

    y_new = Y + Dydt3 * dt;  // integrable.StateIncrement(y_new, Y, Dydt3*dt);
    GetIntegrable()->StateSolve(Dydt4, L, y_new, T + dt, dt, true, true, lumping_parameters);

    Y = Y + (Dydt1 + Dydt2 * 2.0 + Dydt3 * 2.0 + Dydt4) * CH_1_6 * dt;  // integrable.StateIncrement(...);
    dYdt = Dydt4;                                                       // to check
    T += dt;

    GetIntegrable()->StateScatter(Y, T, true);      // state -> system
    GetIntegrable()->StateScatterDerivative(dYdt);  // -> system auxiliary data
    GetIntegrable()->StateScatterReactions(L);      // -> system auxiliary data
}

void ChTimestepperRungeKuttaExpl::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperRungeKuttaExpl>();
    // serialize parent class:
    ChTimestepperIorder::ArchiveOut(archive);
    ChTimestepperExplicit::ArchiveOut(archive);
}
void ChTimestepperRungeKuttaExpl::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChTimestepperRungeKuttaExpl>();
    // deserialize parent class:
    ChTimestepperIorder::ArchiveIn(archive);
    ChTimestepperExplicit::ArchiveIn(archive);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperHeun)
CH_UPCASTING(ChTimestepperHeun, ChTimestepperIorder)
CH_UPCASTING(ChTimestepperHeun, ChTimestepperExplicit)

// Performs a step of a Heun explicit integrator. It is like a 2nd Runge Kutta.
void ChTimestepperHeun::Advance(double dt) {
    // setup main vectors
    GetIntegrable()->StateSetup(Y, dYdt);

    // setup auxiliary vectors
    int n_y = (GetIntegrable()->GetNumCoordsPosLevel() + GetIntegrable()->GetNumCoordsVelLevel());
    int n_dy = (GetIntegrable()->GetNumCoordsVelLevel() + GetIntegrable()->GetNumCoordsAccLevel());
    int n_c = GetIntegrable()->GetNumConstraints();
    y_new.setZero(n_y, GetIntegrable());
    Dydt1.setZero(n_dy, GetIntegrable());
    Dydt2.setZero(n_dy, GetIntegrable());
    L.setZero(n_c);

    GetIntegrable()->StateGather(Y, T);  // state <- system

    GetIntegrable()->StateSolve(Dydt1, L, Y, T, dt,
                                false,              // no need to scatter state before computation
                                false,              // full update? ( not used, since no scatter)
                                lumping_parameters  // optional lumping?
    );
    y_new = Y + Dydt1 * dt;
    GetIntegrable()->StateSolve(Dydt2, L, y_new, T + dt, dt, true, true, lumping_parameters);

    Y = Y + (Dydt1 + Dydt2) * (dt / 2.);
    dYdt = Dydt2;
    T += dt;

    GetIntegrable()->StateScatter(Y, T, true);      // state -> system
    GetIntegrable()->StateScatterDerivative(dYdt);  // -> system auxiliary data
    GetIntegrable()->StateScatterReactions(L);      // -> system auxiliary data
}

void ChTimestepperHeun::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperHeun>();
    // serialize parent class:
    ChTimestepperIorder::ArchiveOut(archive);
    ChTimestepperExplicit::ArchiveOut(archive);
}
void ChTimestepperHeun::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChTimestepperHeun>();
    // deserialize parent class:
    ChTimestepperIorder::ArchiveIn(archive);
    ChTimestepperExplicit::ArchiveIn(archive);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperLeapfrog)
CH_UPCASTING(ChTimestepperLeapfrog, ChTimestepperIIorder)
CH_UPCASTING(ChTimestepperLeapfrog, ChTimestepperExplicit)

// Performs a step of a Leapfrog explicit integrator.
// It is a symplectic method, with 2nd order accuracy,
// at least when F depends on positions only.
// Note: uses last step acceleration: changing or resorting
// the numbering of DOFs will invalidate it.
// Suggestion: use the ChTimestepperEulerSemiImplicit, it gives
// the same accuracy with a bit of faster performance.
void ChTimestepperLeapfrog::Advance(double dt) {
    // downcast
    ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

    // setup main vectors
    mintegrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    L.setZero(mintegrable->GetNumConstraints());
    Aold.setZero(mintegrable->GetNumCoordsVelLevel(), GetIntegrable());

    mintegrable->StateGather(X, V, T);  // state <- system
    mintegrable->StateGatherAcceleration(Aold);

    // advance X (uses last A)
    X = X + V * dt + Aold * (0.5 * dt * dt);

    // computes new A  (NOTE!!true for imposing a state-> system scatter update,because X changed..)
    mintegrable->StateSolveA(A, L, X, V, T, dt, true, true, lumping_parameters);  // Dv/dt = f(x,v,T)   Dv = f(x,v,T)*dt

    // advance V

    V = V + (Aold + A) * (0.5 * dt);

    T += dt;

    mintegrable->StateScatter(X, V, T, true);  // state -> system
    mintegrable->StateScatterAcceleration(A);  // -> system auxiliary data
    mintegrable->StateScatterReactions(L);     // -> system auxiliary data
}

void ChTimestepperLeapfrog::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperLeapfrog>();
    // serialize parent class:
    ChTimestepperIIorder::ArchiveOut(archive);
    ChTimestepperExplicit::ArchiveOut(archive);
}
void ChTimestepperLeapfrog::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChTimestepperLeapfrog>();
    // deserialize parent class:
    ChTimestepperIIorder::ArchiveIn(archive);
    ChTimestepperExplicit::ArchiveIn(archive);
}

}  // end namespace chrono
