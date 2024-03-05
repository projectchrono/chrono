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
    x.resize(GetNumCoordinatesPos());
    v.resize(GetNumCoordinatesVel());
    a.resize(GetNumCoordinatesVel());
}

bool ChIntegrableIIorder::StateSolveA(ChStateDelta& Dvdt,        // result: computed a for a=dv/dt
                                      ChVectorDynamic<>& L,      // result: computed lagrangian multipliers, if any
                                      const ChState& x,          // current state, x
                                      const ChStateDelta& v,     // current state, v
                                      const double T,            // current time T
                                      const double dt,           // timestep (if needed)
                                      bool force_state_scatter,  // if false, x and v are not scattered to the system
                                      bool full_update,          // if true, perform a full update during scatter
                                      ChLumpingParms* lumping    ///< if not null, uses lumped masses to avoid inverting a mass matrix, and uses penalty for constraints.
) {
    if (force_state_scatter)
        StateScatter(x, v, T, full_update);

    ChVectorDynamic<> R(GetNumCoordinatesVel());

    if (! lumping) {
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

        bool success = StateSolveCorrection(Dvdt, L, R, Qc, 1.0, 0, 0, x, v, T, false, false, true);

        return success;
    } 
    else {
        ChVectorDynamic<> Md(GetNumCoordinatesVel()); // diagonal mass
        Md.setZero(GetNumCoordinatesVel());

        // penalty terms from constraints
        R.setZero(GetNumCoordinatesVel());
        L.setZero(GetNumConstraints());

        LoadResidual_F(R, 1.0); // rhs, applied forces

        double err = 0;
        LoadLumpedMass_Md(Md, err, 1.0);

        if (GetNumConstraints()) {
            LoadConstraint_C(L, -lumping->Ck_penalty);  // L  = -k*C     // to do: modulate  k  as constraint-dependent, k=lumping->Ck_penalty*Ck_i
            LoadResidual_CqL(R, L, 1.0);    // Fc =  Cq' * (-k*C)    = Cq' * L
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

void ChIntegrableIIorder::StateGather(ChState& y, double& T)  {
    ChState mx(GetNumCoordinatesPos(), y.GetIntegrable());
    ChStateDelta mv(GetNumCoordinatesVel(), y.GetIntegrable());
    StateGather(mx, mv, T);
    y.segment(0, mx.size()) = mx;
    y.segment(GetNumCoordinatesPos(), mv.size()) = mv;
}

void ChIntegrableIIorder::StateScatter(const ChState& y, const double T, bool full_update) {
    ChState mx(GetNumCoordinatesPos(), y.GetIntegrable());
    ChStateDelta mv(GetNumCoordinatesVel(), y.GetIntegrable());
    mx = y.segment(0, GetNumCoordinatesPos());
    mv = y.segment(GetNumCoordinatesPos(), GetNumCoordinatesVel());
    StateScatter(mx, mv, T, full_update);
}

void ChIntegrableIIorder::StateGatherDerivative(ChStateDelta& Dydt) {
    ChStateDelta mv(GetNumCoordinatesVel(), Dydt.GetIntegrable());
    ChStateDelta ma(GetNumCoordinatesVel(), Dydt.GetIntegrable());
    StateGatherAcceleration(ma);
    Dydt.segment(0, GetNumCoordinatesVel()) = mv;
    Dydt.segment(GetNumCoordinatesVel(), GetNumCoordinatesVel()) = ma;
}

void ChIntegrableIIorder::StateScatterDerivative(const ChStateDelta& Dydt) {
    ChStateDelta ma(GetNumCoordinatesVel(), Dydt.GetIntegrable());
    ma = Dydt.segment(GetNumCoordinatesVel(), GetNumCoordinatesVel());
    StateScatterAcceleration(ma);
}

void ChIntegrableIIorder::StateIncrement(ChState& y_new,         // resulting y_new = y + Dy
                                         const ChState& y,       // initial state y
                                         const ChStateDelta& Dy  // state increment Dy
                                         ) {
    if (y.size() == GetNumCoordinatesPos()) {
        // Incrementing the x part only, user provided only x  in y={x, dx/dt}
        StateIncrementX(y_new, y, Dy);

        return;
    }

    if (y.size() == (GetNumCoordinatesPos() + GetNumCoordinatesVel())) {
        // Incrementing y in y={x, dx/dt}.
        // PERFORMANCE WARNING! temporary vectors allocated on heap.
        // This is only to support compatibility with 1st order integrators.
        ChState mx(GetNumCoordinatesPos(), y.GetIntegrable());
        ChStateDelta mv(GetNumCoordinatesVel(), y.GetIntegrable());
        mx = y.segment(0, GetNumCoordinatesPos());
        mv = y.segment(GetNumCoordinatesPos(), GetNumCoordinatesVel());

        ChStateDelta mDx(GetNumCoordinatesVel(), y.GetIntegrable());
        ChStateDelta mDv(GetNumCoordinatesVel(), y.GetIntegrable());
        mDx = Dy.segment(0, GetNumCoordinatesVel());
        mDv = Dy.segment(GetNumCoordinatesVel(), GetNumCoordinatesVel());
        
        ChState mx_new(GetNumCoordinatesPos(), y.GetIntegrable());
        ChStateDelta mv_new(GetNumCoordinatesVel(), y.GetIntegrable());
        StateIncrementX(mx_new, mx, mDx);  // increment positions
        mv_new = mv + mDv;                 // increment speeds

        y_new.segment(0, GetNumCoordinatesPos()) = mx_new;
        y_new.segment(GetNumCoordinatesPos(), GetNumCoordinatesVel()) = mv_new;
    
        return;
    }

    throw std::runtime_error("StateIncrement() called with a wrong number of elements");
}

bool ChIntegrableIIorder::StateSolve(ChStateDelta& dydt,        // result: computed dydt
                                     ChVectorDynamic<>& L,      // result: computed lagrangian multipliers, if any
                                     const ChState& y,          // current state y
                                     const double T,            // current time T
                                     const double dt,           // timestep (if needed, ex. in NSC)
                                     bool force_state_scatter,  // if false, y and T are not scattered to the system
                                     bool full_update,          // if true, perform a full update during scatter
                                     ChLumpingParms* lumping    // if not null, uses lumped masses to avoid inverting a mass matrix, and uses penalty for constraints.
) {
    ChState mx(GetNumCoordinatesPos(), y.GetIntegrable());
    ChStateDelta mv(GetNumCoordinatesVel(), y.GetIntegrable());
    mx = y.segment(0, GetNumCoordinatesPos());
    mv = y.segment(GetNumCoordinatesPos(), GetNumCoordinatesVel());

    // Solve with custom II order solver
    ChStateDelta ma(GetNumCoordinatesVel(), y.GetIntegrable());
    if (!StateSolveA(ma, L, mx, mv, T, dt, force_state_scatter, full_update, lumping)) {
        return false;
    }

    dydt.segment(0, GetNumCoordinatesVel()) = mv;
    dydt.segment(GetNumCoordinatesVel(), GetNumCoordinatesVel()) = ma;

    return true;
}

} // end namespace chrono
