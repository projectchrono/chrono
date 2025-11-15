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

#include "chrono/timestepper/ChIntegrable.h"

namespace chrono {

// -----------------------------------------------------------------------------
// Implementation of ChIntegrable methods
// -----------------------------------------------------------------------------

void ChIntegrable::StateIncrement(ChState& y_new, const ChState& y, const ChStateDelta& Dy) {
    //// RADU
    //// The next two lines don't make sense: we assert y and y_new are of the same size, then we resize y_new!!!

    assert(y_new.size() == y.size() && y.size() == Dy.size());

    //// RADU
    //// Fix this poor implementation!
    //// We cannot do the natural thing here (i.e, y_new = y + Dy) because this would lead to stack overflow!!!
    //// Indeed, see implementation of overloaded operator+()

    ////y_new = y + Dy;
    y_new.resize(y.size());
    for (int i = 0; i < y.size(); ++i) {
        y_new(i) = y(i) + Dy(i);
    }
}

// -----------------------------------------------------------------------------
// Implementation of ChIntegrableII methods
// -----------------------------------------------------------------------------

void ChIntegrableIIorder::StateSetup(ChState& x, ChStateDelta& v, ChStateDelta& a) {
    x.resize(GetNumCoordsPosLevel());
    v.resize(GetNumCoordsVelLevel());
    a.resize(GetNumCoordsVelLevel());
}

bool ChIntegrableIIorder::StateSolveA(ChStateDelta& Dvdt,        // result: computed a for a=dv/dt
                                      ChVectorDynamic<>& L,      // result: computed Lagrange multipliers
                                      const ChState& x,          // current state, x
                                      const ChStateDelta& v,     // current state, v
                                      const double T,            // current time T
                                      const double dt,           // timestep (if needed)
                                      bool force_state_scatter,  // if true, scatter x and v to the system
                                      bool full_update,          // if true, perform a full update during scatter
                                      ChLumpingParms* lumping    // use lumped masses to avoid inverting a mass matrix
) {
    if (force_state_scatter)
        StateScatter(x, v, T, full_update);

    ChVectorDynamic<> R(GetNumCoordsVelLevel());

    if (!lumping) {
        ChVectorDynamic<> Qc(GetNumConstraints());
        const double Delta = 1e-6;

        R.setZero();
        Qc.setZero();

        LoadResidual_F(R, 1.0);
        LoadConstraint_C(Qc, -2.0 / (Delta * Delta));

        // numerical differentiation to get the Qc term in constraints
        ChStateDelta dx(v);
        dx *= Delta;
        ChState xdx(x.size(), this);

        StateIncrement(xdx, x, dx);
        StateScatter(xdx, v, T + Delta, full_update);
        LoadConstraint_C(Qc, 1.0 / (Delta * Delta));

        StateIncrement(xdx, x, -dx);
        StateScatter(xdx, v, T - Delta, full_update);
        LoadConstraint_C(Qc, 1.0 / (Delta * Delta));

        StateScatter(x, v, T, full_update);  // back to original state

        bool success = StateSolveCorrection(Dvdt, L, R, Qc, 1.0, 0, 0, x, v, T, false, false, true, true);

        return success;
    } else {
        ChVectorDynamic<> Md(GetNumCoordsVelLevel());  // diagonal mass
        Md.setZero(GetNumCoordsVelLevel());

        // penalty terms from constraints
        R.setZero(GetNumCoordsVelLevel());
        L.setZero(GetNumConstraints());

        LoadResidual_F(R, 1.0);  // rhs, applied forces

        double err = 0;
        LoadLumpedMass_Md(Md, err, 1.0);

        if (GetNumConstraints()) {
            LoadConstraint_C(L, -lumping->Ck_penalty);  // L  = -k*C     // to do: modulate  k  as constraint-dependent,
                                                        // k=lumping->Ck_penalty*Ck_i
            LoadResidual_CqL(R, L, 1.0);                // Fc =  Cq' * (-k*C)    = Cq' * L
            // to do: add c_penalty for speed proportional damping, as   Fc = - Cq' * (k*C + c*(dC/dt))
        }

        // compute acceleration using lumped diagonal mass. No need to invoke a linear solver.
        Dvdt.array() = R.array() / Md.array();  //  a = Md^-1 * F

        return (err == 0);
    }
}

void ChIntegrableIIorder::StateIncrementX(ChState& x_new, const ChState& x, const ChStateDelta& Dx) {
    //// RADU
    //// Fix this poor implementation!
    //// We cannot do the natural thing here (y_new = y + Dy) because this would lead to stack overflow!!!
    //// Indeed, see implementation of overloaded operator+()

    ////x_new = x + Dx;
    x_new.resize(x.size());
    for (int i = 0; i < x.size(); ++i) {
        x_new(i) = x(i) + Dx(i);
    }
}

void ChIntegrableIIorder::StateGather(ChState& y, double& T) {
    ChState mx(GetNumCoordsPosLevel(), y.GetIntegrable());
    ChStateDelta mv(GetNumCoordsVelLevel(), y.GetIntegrable());
    StateGather(mx, mv, T);
    y.segment(0, mx.size()) = mx;
    y.segment(GetNumCoordsPosLevel(), mv.size()) = mv;
}

void ChIntegrableIIorder::StateScatter(const ChState& y, const double T, bool full_update) {
    ChState mx(GetNumCoordsPosLevel(), y.GetIntegrable());
    ChStateDelta mv(GetNumCoordsVelLevel(), y.GetIntegrable());
    mx = y.segment(0, GetNumCoordsPosLevel());
    mv = y.segment(GetNumCoordsPosLevel(), GetNumCoordsVelLevel());
    StateScatter(mx, mv, T, full_update);
}

void ChIntegrableIIorder::StateGatherDerivative(ChStateDelta& Dydt) {
    ChStateDelta mv(GetNumCoordsVelLevel(), Dydt.GetIntegrable());
    ChStateDelta ma(GetNumCoordsVelLevel(), Dydt.GetIntegrable());
    StateGatherAcceleration(ma);
    Dydt.segment(0, GetNumCoordsVelLevel()) = mv;
    Dydt.segment(GetNumCoordsVelLevel(), GetNumCoordsVelLevel()) = ma;
}

void ChIntegrableIIorder::StateScatterDerivative(const ChStateDelta& Dydt) {
    ChStateDelta ma(GetNumCoordsVelLevel(), Dydt.GetIntegrable());
    ma = Dydt.segment(GetNumCoordsVelLevel(), GetNumCoordsVelLevel());
    StateScatterAcceleration(ma);
}

void ChIntegrableIIorder::StateIncrement(ChState& y_new,         // resulting y_new = y + Dy
                                         const ChState& y,       // initial state y
                                         const ChStateDelta& Dy  // state increment Dy
) {
    if (y.size() == GetNumCoordsPosLevel()) {
        // Incrementing the x part only, user provided only x  in y={x, dx/dt}
        StateIncrementX(y_new, y, Dy);

        return;
    }

    if (y.size() == (GetNumCoordsPosLevel() + GetNumCoordsVelLevel())) {
        // Incrementing y in y={x, dx/dt}.
        // PERFORMANCE WARNING! temporary vectors allocated on heap.
        // This is only to support compatibility with 1st order integrators.
        ChState mx(GetNumCoordsPosLevel(), y.GetIntegrable());
        ChStateDelta mv(GetNumCoordsVelLevel(), y.GetIntegrable());
        mx = y.segment(0, GetNumCoordsPosLevel());
        mv = y.segment(GetNumCoordsPosLevel(), GetNumCoordsVelLevel());

        ChStateDelta mDx(GetNumCoordsVelLevel(), y.GetIntegrable());
        ChStateDelta mDv(GetNumCoordsVelLevel(), y.GetIntegrable());
        mDx = Dy.segment(0, GetNumCoordsVelLevel());
        mDv = Dy.segment(GetNumCoordsVelLevel(), GetNumCoordsVelLevel());

        ChState mx_new(GetNumCoordsPosLevel(), y.GetIntegrable());
        ChStateDelta mv_new(GetNumCoordsVelLevel(), y.GetIntegrable());
        StateIncrementX(mx_new, mx, mDx);  // increment positions
        mv_new = mv + mDv;                 // increment speeds

        y_new.segment(0, GetNumCoordsPosLevel()) = mx_new;
        y_new.segment(GetNumCoordsPosLevel(), GetNumCoordsVelLevel()) = mv_new;

        return;
    }

    throw std::runtime_error("StateIncrement() called with a wrong number of elements");
}

bool ChIntegrableIIorder::StateSolve(ChStateDelta& dydt,        // result: computed dydt
                                     ChVectorDynamic<>& L,      // result: computed Lagrange multipliers
                                     const ChState& y,          // current state y
                                     const double T,            // current time T
                                     const double dt,           // timestep (if needed, e.g. in NSC)
                                     bool force_state_scatter,  // if false, y and T are not scattered to the system
                                     bool full_update,          // if true, perform a full update during scatter
                                     ChLumpingParms* lumping    // if not null, uses lumped masses to avoid inverting a
                                                                // mass matrix, and uses penalty for constraints.
) {
    ChState mx(GetNumCoordsPosLevel(), y.GetIntegrable());
    ChStateDelta mv(GetNumCoordsVelLevel(), y.GetIntegrable());
    mx = y.segment(0, GetNumCoordsPosLevel());
    mv = y.segment(GetNumCoordsPosLevel(), GetNumCoordsVelLevel());

    // Solve with custom II order solver
    ChStateDelta ma(GetNumCoordsVelLevel(), y.GetIntegrable());
    if (!StateSolveA(ma, L, mx, mv, T, dt, force_state_scatter, full_update, lumping)) {
        return false;
    }

    dydt.segment(0, GetNumCoordsVelLevel()) = mv;
    dydt.segment(GetNumCoordsVelLevel(), GetNumCoordsVelLevel()) = ma;

    return true;
}

}  // end namespace chrono
