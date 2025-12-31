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

CH_UPCASTING(ChTimestepperExplicit, ChTimestepper)

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
CH_FACTORY_REGISTER(ChTimestepperEulerExplicitIorder)
CH_UPCASTING(ChTimestepperEulerExplicitIorder, ChTimestepperIorder)
CH_UPCASTING(ChTimestepperEulerExplicitIorder, ChTimestepperExplicit)

ChTimestepperEulerExplicitIorder::ChTimestepperEulerExplicitIorder(ChIntegrable* intgr) : ChTimestepperIorder(intgr) {}

void ChTimestepperEulerExplicitIorder::Advance(double dt) {
    // setup main vectors
    integrable->StateSetup(Y, dYdt);

    // setup auxiliary vectors
    L.setZero(integrable->GetNumConstraints());

    integrable->StateGather(Y, T);  // state <- system

    integrable->StateSolve(dYdt, L, Y, T, dt, false, false, lumping_parameters);  // dY/dt = f(Y,T)

    // Euler formula
    //   y_new= y + dy/dt * dt

    Y = Y + dYdt * dt;  //  also: integrable.StateIncrement(y_new, y, Dy);

    T += dt;

    integrable->StateScatter(Y, T, true);      // state -> system
    integrable->StateScatterDerivative(dYdt);  // -> system auxiliary data
    integrable->StateScatterReactions(L);      // -> system auxiliary data
}

void ChTimestepperEulerExplicitIorder::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperEulerExplicitIorder>();

    ChTimestepperExplicit::ArchiveOut(archive);
}
void ChTimestepperEulerExplicitIorder::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChTimestepperEulerExplicitIorder>();

    ChTimestepperExplicit::ArchiveIn(archive);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperEulerExplicitIIorder)
CH_UPCASTING(ChTimestepperEulerExplicitIIorder, ChTimestepperIIorder)
CH_UPCASTING(ChTimestepperEulerExplicitIIorder, ChTimestepperExplicit)

ChTimestepperEulerExplicitIIorder::ChTimestepperEulerExplicitIIorder(ChIntegrableIIorder* intgr)
    : ChTimestepperIIorder(intgr) {}

// Euler explicit timestepper customized for II order.
// (It gives the same results of ChTimestepperEulerExplicitIorder,
// but this performs a bit faster because it can exploit
// the special structure of ChIntegrableIIorder)
// This performs the typical
//    x_new = x + v * dt
//    v_new = v + a * dt
// integration with Euler formula.
void ChTimestepperEulerExplicitIIorder::Advance(double dt) {
    // setup main vectors
    integrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    Dv.setZero(integrable->GetNumCoordsVelLevel(), integrable);
    L.setZero(integrable->GetNumConstraints());

    integrable->StateGather(X, V, T);  // state <- system

    integrable->StateSolveA(A, L, X, V, T, dt, false, false, lumping_parameters);  // Dv/dt = f(x,v,T)

    // Euler formula!

    X = X + V * dt;  // x_new= x + v * dt

    V = V + A * dt;  // v_new= v + a * dt

    T += dt;

    integrable->StateScatter(X, V, T, true);  // state -> system
    integrable->StateScatterAcceleration(A);  // -> system auxiliary data
    integrable->StateScatterReactions(L);     // -> system auxiliary data
}

void ChTimestepperEulerExplicitIIorder::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperEulerExplicitIIorder>();

    ChTimestepperExplicit::ArchiveOut(archive);
}
void ChTimestepperEulerExplicitIIorder::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChTimestepperEulerExplicitIIorder>();

    ChTimestepperExplicit::ArchiveIn(archive);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperEulerSemiImplicit)
CH_UPCASTING(ChTimestepperEulerSemiImplicit, ChTimestepperIIorder)
CH_UPCASTING(ChTimestepperEulerSemiImplicit, ChTimestepperExplicit)

ChTimestepperEulerSemiImplicit::ChTimestepperEulerSemiImplicit(ChIntegrableIIorder* intgr)
    : ChTimestepperIIorder(intgr) {}

// Euler semi-implicit timestepper
// This performs the typical
//    v_new = v + a * dt
//    x_new = x + v_new * dt
// integration with Euler semi-implicit formula.
void ChTimestepperEulerSemiImplicit::Advance(double dt) {
    // setup main vectors
    integrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    L.setZero(integrable->GetNumConstraints());

    integrable->StateGather(X, V, T);  // state <- system

    integrable->StateSolveA(A, L, X, V, T, dt, false, false,
                            lumping_parameters);  // Dv/dt = f(x,v,T)   Dv = f(x,v,T)*dt

    // Semi-implicit Euler formula!   (note the order of update of x and v, respect to original Euler II order explicit)

    V = V + A * dt;  // v_new= v + a * dt

    X = X + V * dt;  // x_new= x + v_new * dt

    T += dt;

    integrable->StateScatter(X, V, T, true);  // state -> system
    integrable->StateScatterAcceleration(A);  // -> system auxiliary data
    integrable->StateScatterReactions(L);     // -> system auxiliary data
}

void ChTimestepperEulerSemiImplicit::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperEulerSemiImplicit>();

    ChTimestepperExplicit::ArchiveOut(archive);
}
void ChTimestepperEulerSemiImplicit::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChTimestepperEulerSemiImplicit>();

    ChTimestepperExplicit::ArchiveIn(archive);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperRungeKutta)
CH_UPCASTING(ChTimestepperRungeKutta, ChTimestepperIorder)
CH_UPCASTING(ChTimestepperRungeKutta, ChTimestepperExplicit)

ChTimestepperRungeKutta::ChTimestepperRungeKutta(ChIntegrable* intgr) : ChTimestepperIorder(intgr) {}

void ChTimestepperRungeKutta::Advance(double dt) {
    // setup main vectors
    integrable->StateSetup(Y, dYdt);

    // setup auxiliary vectors
    int n_y = (integrable->GetNumCoordsPosLevel() + integrable->GetNumCoordsVelLevel());
    int n_dy = (integrable->GetNumCoordsVelLevel() + integrable->GetNumCoordsAccLevel());
    int n_c = integrable->GetNumConstraints();
    y_new.setZero(n_y, integrable);
    Dydt1.setZero(n_dy, integrable);
    Dydt2.setZero(n_dy, integrable);
    Dydt3.setZero(n_dy, integrable);
    Dydt4.setZero(n_dy, integrable);
    L.setZero(n_c);

    integrable->StateGather(Y, T);  // state <- system

    integrable->StateSolve(Dydt1, L, Y, T, dt,
                           false,              // no need to scatter state before computation
                           false,              // full update? (not used since no scatter)
                           lumping_parameters  // optional lumping?
    );

    y_new = Y + Dydt1 * 0.5 * dt;  // integrable.StateIncrement(y_new, Y, Dydt1*0.5*dt);
    integrable->StateSolve(Dydt2, L, y_new, T + dt * 0.5, dt, true, true, lumping_parameters);

    y_new = Y + Dydt2 * 0.5 * dt;  // integrable.StateIncrement(y_new, Y, Dydt2*0.5*dt);
    integrable->StateSolve(Dydt3, L, y_new, T + dt * 0.5, dt, true, true, lumping_parameters);

    y_new = Y + Dydt3 * dt;  // integrable.StateIncrement(y_new, Y, Dydt3*dt);
    integrable->StateSolve(Dydt4, L, y_new, T + dt, dt, true, true, lumping_parameters);

    Y = Y + (Dydt1 + Dydt2 * 2.0 + Dydt3 * 2.0 + Dydt4) * CH_1_6 * dt;  // integrable.StateIncrement(...);
    dYdt = Dydt4;                                                       // to check
    T += dt;

    integrable->StateScatter(Y, T, true);      // state -> system
    integrable->StateScatterDerivative(dYdt);  // -> system auxiliary data
    integrable->StateScatterReactions(L);      // -> system auxiliary data
}

void ChTimestepperRungeKutta::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperRungeKutta>();

    ChTimestepperExplicit::ArchiveOut(archive);
}
void ChTimestepperRungeKutta::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChTimestepperRungeKutta>();

    ChTimestepperExplicit::ArchiveIn(archive);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperHeun)
CH_UPCASTING(ChTimestepperHeun, ChTimestepperIorder)
CH_UPCASTING(ChTimestepperHeun, ChTimestepperExplicit)

ChTimestepperHeun::ChTimestepperHeun(ChIntegrable* intgr) : ChTimestepperIorder(intgr) {}

// Performs a step of a Heun explicit integrator. It is like a 2nd Runge Kutta.
void ChTimestepperHeun::Advance(double dt) {
    // setup main vectors
    integrable->StateSetup(Y, dYdt);

    // setup auxiliary vectors
    int n_y = (integrable->GetNumCoordsPosLevel() + integrable->GetNumCoordsVelLevel());
    int n_dy = (integrable->GetNumCoordsVelLevel() + integrable->GetNumCoordsAccLevel());
    int n_c = integrable->GetNumConstraints();
    y_new.setZero(n_y, integrable);
    Dydt1.setZero(n_dy, integrable);
    Dydt2.setZero(n_dy, integrable);
    L.setZero(n_c);

    integrable->StateGather(Y, T);  // state <- system

    integrable->StateSolve(Dydt1, L, Y, T, dt,
                           false,              // no need to scatter state before computation
                           false,              // full update? ( not used, since no scatter)
                           lumping_parameters  // optional lumping?
    );
    y_new = Y + Dydt1 * dt;
    integrable->StateSolve(Dydt2, L, y_new, T + dt, dt, true, true, lumping_parameters);

    Y = Y + (Dydt1 + Dydt2) * (dt / 2.);
    dYdt = Dydt2;
    T += dt;

    integrable->StateScatter(Y, T, true);      // state -> system
    integrable->StateScatterDerivative(dYdt);  // -> system auxiliary data
    integrable->StateScatterReactions(L);      // -> system auxiliary data
}

void ChTimestepperHeun::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperHeun>();

    ChTimestepperExplicit::ArchiveOut(archive);
}
void ChTimestepperHeun::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChTimestepperHeun>();

    ChTimestepperExplicit::ArchiveIn(archive);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTimestepperLeapfrog)
CH_UPCASTING(ChTimestepperLeapfrog, ChTimestepperIIorder)
CH_UPCASTING(ChTimestepperLeapfrog, ChTimestepperExplicit)

ChTimestepperLeapfrog::ChTimestepperLeapfrog(ChIntegrableIIorder* intgr) : ChTimestepperIIorder(intgr) {}

// Performs a step of a Leapfrog explicit integrator.
// It is a symplectic method, with 2nd order accuracy, at least when F depends on positions only.
// Note: uses last step acceleration: changing or resorting the numbering of DOFs will invalidate it.
// Suggestion: use the ChTimestepperEulerSemiImplicit, it gives the same accuracy with a bit of faster performance.
void ChTimestepperLeapfrog::Advance(double dt) {
    // setup main vectors
    integrable->StateSetup(X, V, A);

    // setup auxiliary vectors
    L.setZero(integrable->GetNumConstraints());
    Aold.setZero(integrable->GetNumCoordsVelLevel(), integrable);

    integrable->StateGather(X, V, T);  // state <- system
    integrable->StateGatherAcceleration(Aold);

    // advance X (uses last A)
    X = X + V * dt + Aold * (0.5 * dt * dt);

    // computes new A  (NOTE!!true for imposing a state-> system scatter update,because X changed..)
    integrable->StateSolveA(A, L, X, V, T, dt, true, true, lumping_parameters);  // Dv/dt = f(x,v,T)   Dv = f(x,v,T)*dt

    // advance V

    V = V + (Aold + A) * (0.5 * dt);

    T += dt;

    integrable->StateScatter(X, V, T, true);  // state -> system
    integrable->StateScatterAcceleration(A);  // -> system auxiliary data
    integrable->StateScatterReactions(L);     // -> system auxiliary data
}

void ChTimestepperLeapfrog::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepperLeapfrog>();

    ChTimestepperExplicit::ArchiveOut(archive);
}
void ChTimestepperLeapfrog::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChTimestepperLeapfrog>();

    ChTimestepperExplicit::ArchiveIn(archive);
}

}  // end namespace chrono
