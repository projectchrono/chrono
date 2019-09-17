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
    x.resize(GetNcoords_x());
    v.resize(GetNcoords_v());
    a.resize(GetNcoords_a());
}

bool ChIntegrableIIorder::StateSolveA(ChStateDelta& Dvdt,       // result: computed a for a=dv/dt
                                      ChVectorDynamic<>& L,     // result: computed lagrangian multipliers, if any
                                      const ChState& x,         // current state, x
                                      const ChStateDelta& v,    // current state, v
                                      const double T,           // current time T
                                      const double dt,          // timestep (if needed)
                                      bool force_state_scatter  // if false, x,v and T are not scattered to the system
                                      ) {
    if (force_state_scatter)
        StateScatter(x, v, T);

    ChVectorDynamic<> R(GetNcoords_v());
    ChVectorDynamic<> Qc(GetNconstr());
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
    StateScatter(xdx, v, T + Delta);
    LoadConstraint_C(Qc, 1.0 / (Delta * Delta));

    StateIncrement(xdx, x, -dx);
    StateScatter(xdx, v, T - Delta);
    LoadConstraint_C(Qc, 1.0 / (Delta * Delta));

    StateScatter(x, v, T);  // back to original state

    bool success = StateSolveCorrection(Dvdt, L, R, Qc, 1.0, 0, 0, x, v, T, false, true);

    return success;
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

void ChIntegrableIIorder::StateGather(ChState& y, double& T)  {
    ChState mx(GetNcoords_x(), y.GetIntegrable());
    ChStateDelta mv(GetNcoords_v(), y.GetIntegrable());
    StateGather(mx, mv, T);
    y.segment(0, mx.size()) = mx;
    y.segment(GetNcoords_x(), mv.size()) = mv;
}

void ChIntegrableIIorder::StateScatter(const ChState& y, const double T) {
    ChState mx(GetNcoords_x(), y.GetIntegrable());
    ChStateDelta mv(GetNcoords_v(), y.GetIntegrable());
    mx = y.segment(0, GetNcoords_x());
    mv = y.segment(GetNcoords_x(), GetNcoords_v());
    StateScatter(mx, mv, T);
}

void ChIntegrableIIorder::StateGatherDerivative(ChStateDelta& Dydt) {
    ChStateDelta mv(GetNcoords_v(), Dydt.GetIntegrable());
    ChStateDelta ma(GetNcoords_v(), Dydt.GetIntegrable());
    StateGatherAcceleration(ma);
    Dydt.segment(0, GetNcoords_v()) = mv;
    Dydt.segment(GetNcoords_v(), GetNcoords_v()) = ma;
}

void ChIntegrableIIorder::StateScatterDerivative(const ChStateDelta& Dydt) {
    ChStateDelta ma(GetNcoords_v(), Dydt.GetIntegrable());
    ma = Dydt.segment(GetNcoords_v(), GetNcoords_v());
    StateScatterAcceleration(ma);
}

void ChIntegrableIIorder::StateIncrement(ChState& y_new,         // resulting y_new = y + Dy
                                         const ChState& y,       // initial state y
                                         const ChStateDelta& Dy  // state increment Dy
                                         ) {
    if (y.size() == GetNcoords_x()) {
        // Incrementing the x part only, user provided only x  in y={x, dx/dt}
        StateIncrementX(y_new, y, Dy);

        return;
    }

    if (y.size() == GetNcoords_y()) {
        // Incrementing y in y={x, dx/dt}.
        // PERFORMANCE WARNING! temporary vectors allocated on heap.
        // This is only to support compatibility with 1st order integrators.
        ChState mx(GetNcoords_x(), y.GetIntegrable());
        ChStateDelta mv(GetNcoords_v(), y.GetIntegrable());
        mx = y.segment(0, GetNcoords_x());
        mv = y.segment(GetNcoords_x(), GetNcoords_v());

        ChStateDelta mDx(GetNcoords_v(), y.GetIntegrable());
        ChStateDelta mDv(GetNcoords_a(), y.GetIntegrable());
        mDx = Dy.segment(0, GetNcoords_v());
        mDv = Dy.segment(GetNcoords_v(), GetNcoords_a());
        
        ChState mx_new(GetNcoords_x(), y.GetIntegrable());
        ChStateDelta mv_new(GetNcoords_v(), y.GetIntegrable());
        StateIncrementX(mx_new, mx, mDx);  // increment positions
        mv_new = mv + mDv;                 // increment speeds

        y_new.segment(0, GetNcoords_x()) = mx_new;
        y_new.segment(GetNcoords_x(), GetNcoords_v()) = mv_new;
    
        return;
    }

    throw ChException("StateIncrement() called with a wrong number of elements");
}

bool ChIntegrableIIorder::StateSolve(ChStateDelta& dydt,       // result: computed dydt
                                     ChVectorDynamic<>& L,     // result: computed lagrangian multipliers, if any
                                     const ChState& y,         // current state y
                                     const double T,           // current time T
                                     const double dt,          // timestep (if needed, ex. in NSC)
                                     bool force_state_scatter  // if false, y and T are not scattered to the system
                                     ) {
    ChState mx(GetNcoords_x(), y.GetIntegrable());
    ChStateDelta mv(GetNcoords_v(), y.GetIntegrable());
    mx = y.segment(0, GetNcoords_x());
    mv = y.segment(GetNcoords_x(), GetNcoords_v());

    // Solve with custom II order solver
    ChStateDelta ma(GetNcoords_v(), y.GetIntegrable());
    if (!StateSolveA(ma, L, mx, mv, T, dt, force_state_scatter)) {
        return false;
    }

    dydt.segment(0, GetNcoords_v()) = mv;
    dydt.segment(GetNcoords_v(), GetNcoords_v()) = ma;

    return true;
}

} // end namespace chrono
