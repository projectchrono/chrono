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
    assert(y_new.GetRows() == y.GetRows() && y.GetRows() == Dy.GetRows());
    y_new.Resize(y.GetRows(), y.GetColumns());
    for (int i = 0; i < y.GetRows(); ++i) {
        y_new(i) = y(i) + Dy(i);
    }
}

// -----------------------------------------------------------------------------
// Implementation of ChIntegrableII methods
// -----------------------------------------------------------------------------

void ChIntegrableIIorder::StateSetup(ChState& x, ChStateDelta& v, ChStateDelta& a) {
    x.Resize(GetNcoords_x());
    v.Resize(GetNcoords_v());
    a.Resize(GetNcoords_a());
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

    R.Reset();
    Qc.Reset();

    LoadResidual_F(R, 1.0);
    LoadConstraint_C(Qc, -2.0 / (Delta * Delta));

    // numerical differentiation to get the Qc term in constraints
    ChStateDelta dx(v);
    dx *= Delta;
    ChState xdx(x.GetRows(), this);

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

void ChIntegrableIIorder::StateIncrementX(ChState& x_new,         // resulting x_new = x + Dx
                                          const ChState& x,       // initial state x
                                          const ChStateDelta& Dx  // state increment Dx
                                          ) {
    x_new.Resize(x.GetRows(), x.GetColumns());
    for (int i = 0; i < x.GetRows(); ++i) {
        x_new(i) = x(i) + Dx(i);
    }
}

void ChIntegrableIIorder::StateGather(ChState& y, double& T)  {
    ChState mx(GetNcoords_x(), y.GetIntegrable());
    ChStateDelta mv(GetNcoords_v(), y.GetIntegrable());
    this->StateGather(mx, mv, T);
    y.PasteMatrix(mx, 0, 0);
    y.PasteMatrix(mv, GetNcoords_x(), 0);
}

void ChIntegrableIIorder::StateScatter(const ChState& y, const double T) {
    ChState mx(GetNcoords_x(), y.GetIntegrable());
    ChStateDelta mv(GetNcoords_v(), y.GetIntegrable());
    mx.PasteClippedMatrix(y, 0, 0, GetNcoords_x(), 1, 0, 0);
    mv.PasteClippedMatrix(y, GetNcoords_x(), 0, GetNcoords_v(), 1, 0, 0);
    StateScatter(mx, mv, T);
}

void ChIntegrableIIorder::StateGatherDerivative(ChStateDelta& Dydt) {
    ChStateDelta mv(GetNcoords_v(), Dydt.GetIntegrable());
    ChStateDelta ma(GetNcoords_v(), Dydt.GetIntegrable());
    StateGatherAcceleration(ma);
    Dydt.PasteMatrix(mv, 0, 0);
    Dydt.PasteMatrix(ma, GetNcoords_v(), 0);
}

void ChIntegrableIIorder::StateScatterDerivative(const ChStateDelta& Dydt) {
    ChStateDelta ma(GetNcoords_v(), Dydt.GetIntegrable());
    ma.PasteClippedMatrix(Dydt, GetNcoords_v(), 0, GetNcoords_v(), 1, 0, 0);
    StateScatterAcceleration(ma);
}

void ChIntegrableIIorder::StateIncrement(ChState& y_new,         // resulting y_new = y + Dy
                                         const ChState& y,       // initial state y
                                         const ChStateDelta& Dy  // state increment Dy
                                         ) {
    if (y.GetRows() == this->GetNcoords_x()) {
        // Incrementing the x part only, user provided only x  in y={x, dx/dt}
        StateIncrementX(y_new, y, Dy);

        return;
    }

    if (y.GetRows() == this->GetNcoords_y()) {
        // Incrementing y in y={x, dx/dt}.
        // PERFORMANCE WARNING! temporary vectors allocated on heap. This is only to support
        // compatibility with 1st order integrators.
        ChState mx(this->GetNcoords_x(), y.GetIntegrable());
        ChStateDelta mv(this->GetNcoords_v(), y.GetIntegrable());
        mx.PasteClippedMatrix(y, 0, 0, this->GetNcoords_x(), 1, 0, 0);
        mv.PasteClippedMatrix(y, this->GetNcoords_x(), 0, this->GetNcoords_v(), 1, 0, 0);
        ChStateDelta mDx(this->GetNcoords_v(), y.GetIntegrable());
        ChStateDelta mDv(this->GetNcoords_a(), y.GetIntegrable());
        mDx.PasteClippedMatrix(Dy, 0, 0, this->GetNcoords_v(), 1, 0, 0);
        mDv.PasteClippedMatrix(Dy, this->GetNcoords_v(), 0, this->GetNcoords_a(), 1, 0, 0);
        ChState mx_new(this->GetNcoords_x(), y.GetIntegrable());
        ChStateDelta mv_new(this->GetNcoords_v(), y.GetIntegrable());

        StateIncrementX(mx_new, mx, mDx);  // increment positions
        mv_new = mv + mDv;                 // increment speeds

        y_new.PasteMatrix(mx_new, 0, 0);
        y_new.PasteMatrix(mv_new, this->GetNcoords_x(), 0);
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
    mx.PasteClippedMatrix(y, 0, 0, GetNcoords_x(), 1, 0, 0);
    mv.PasteClippedMatrix(y, GetNcoords_x(), 0, GetNcoords_v(), 1, 0, 0);
    ChStateDelta ma(GetNcoords_v(), y.GetIntegrable());

    // Solve with custom II order solver
    if (!StateSolveA(ma, L, mx, mv, T, dt, force_state_scatter)) {
        return false;
    }

    dydt.PasteMatrix(mv, 0, 0);
    dydt.PasteMatrix(ma, GetNcoords_v(), 0);

    return true;
}

} // end namespace chrono
