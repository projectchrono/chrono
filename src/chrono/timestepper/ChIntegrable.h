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

#ifndef CHINTEGRABLE_H
#define CHINTEGRABLE_H

#include <stdlib.h>
#include "core/ChApiCE.h"
#include "core/ChMath.h"
#include "core/ChVectorDynamic.h"
#include "timestepper/ChState.h"

namespace chrono {

/// Interface class for all objects that support time integration.
/// You can inherit your class from this class, by implementing those
/// four functions. By doing this, you can use time integrators from
/// the ChTimestepper hierarchy to integrate in time.

class ChApi ChIntegrable {
  public:
    /// Tells the number of coordinates in the state Y.
    /// Children classes MUST implement this!
    virtual int GetNcoords_y() = 0;

    /// Tells the number of coordinates in the state increment.
    /// This is a base implementation that works in many cases where dim(Y) = dim(dy),
    /// but it can be overridden in the case that y contains quaternions for rotations
    /// rather than simple y+dy
    virtual int GetNcoords_dy() { return GetNcoords_y(); };

    /// Tells the number of lagrangian multipliers (constraints)
    /// By default returns 0.
    virtual int GetNconstr() { return 0; }

    /// This sets up the system state.
    virtual void StateSetup(ChState& y, ChStateDelta& dy) {
        y.Resize(GetNcoords_y(), 1);
        dy.Resize(GetNcoords_dy(), 1);
    };

    /// From system to state Y
    /// Optionally, they will copy system private state, if any, to Y.
    virtual void StateGather(ChState& y, double& T){};

    /// From state Y to system.
    /// This is important because it is called by time integrators all times
    /// they modify the Y state. In some cases, the ChIntegrable object might
    /// contain dependent data structures that might need an update at each change of Y,
    /// if so, this function must be overridden.
    virtual void StateScatter(const ChState& y, const double T){};

    /// Optional: the integrable object might contain last computed state derivative, some integrators might reuse it.
    virtual void StateGatherDerivative(ChStateDelta& Dydt){};

    /// Optional: the integrable object might need to store last computed state derivative, ex. for plotting etc.
    virtual void StateScatterDerivative(const ChStateDelta& Dydt){};

    /// Optional: the integrable object might contain lagrangian multipliers (reaction in constraints)
    virtual void StateGatherReactions(ChVectorDynamic<>& L){};

    /// Optional: the integrable object might contain lagrangian multipliers (reaction in constraints)
    virtual void StateScatterReactions(const ChVectorDynamic<>& L){};

    /// dy/dt = f(y,t)
    /// Given current state y , computes the state derivative dy/dt and
    /// lagrangian multipliers L (if any).
    /// NOTE: some solvers (ex in DVI) cannot compute a classical derivative
    /// dy/dt when v is a function of bounded variation, and f or L are distributions (ex
    /// when there are impulses and discontinuities), so they compute a finite Dy through a finite dt:
    /// this is the reason why this function has an optional parameter dt. In a DVI setting,
    /// one computes Dy, and returns Dy*(1/dt) here in Dydt parameter; if the original Dy has to be known,
    /// just multiply Dydt*dt. The same for impulses: a DVI would compute impulses I, and return L=I*(1/dt).
    /// NOTE! children classes must take care of calling StateScatter(y,T) before
    /// computing Dy, only if force_state_scatter = true (otherwise it is assumed state is already in sync)
    /// NOTE! children classes must take care of resizing Dy and L if needed.
    virtual void StateSolve(ChStateDelta& Dydt,              ///< result: computed Dydt
                            ChVectorDynamic<>& L,            ///< result: computed lagrangian multipliers, if any
                            const ChState& y,                ///< current state y
                            const double T,                  ///< current time T
                            const double dt,                 ///< timestep (if needed, ex. in DVI)
                            bool force_state_scatter = true  ///< if false, y and T are not scattered to the system,
                            /// assuming that someone has done StateScatter just before
                            ) = 0;

    /// Perform y_new = y + Dy
    /// This is a base implementation that works in many cases, but it can be overridden
    /// in the case that y contains quaternions for rotations, rot. exponential is needed
    /// rather than simple y+Dy
    /// NOTE: the system is not updated automatically after the state increment, so one might
    /// need to call StateScatter() if needed.
    virtual void StateIncrement(ChState& y_new,         ///< resulting y_new = y + Dy
                                const ChState& y,       ///< initial state y
                                const ChStateDelta& Dy  ///< state increment Dy
                                ) {
        assert(y_new.GetRows() == y.GetRows() && y.GetRows() == Dy.GetRows());
        y_new.Resize(y.GetRows(), y.GetColumns());
        for (int i = 0; i < y.GetRows(); ++i) {
            y_new(i) = y(i) + Dy(i);
        }
    }

    //
    // Functions required by implicit integration schemes
    //

    /// Assuming an explicit ODE
    ///    H*dy/dt = F(y,t)
    /// Assuming an explicit DAE
    ///    H*dy/dt = F(y,t) + Cq*L
    ///     C(y,t) = 0
    /// this must compute the solution of the change in state to satisfy
    /// the equation required in a Newton Raphson iteration to satisfy an
    /// implicit integrator equation.
    /// If in ODE case:
    ///  Dy = [ c_a*H + c_b*dF/dy ]^-1 * R
    ///  Dy = [ G ]^-1 * R
    /// If with DAE constraints:
    ///  |Dy| = [ G   Cq' ]^-1 * | R |
    ///  |DL|   [ Cq  0   ]      | Qc|
    /// where R is a given residual, dF/dy is F jacobian.
    /// It is up to the child class how to solve such linear system.
    virtual void StateSolveCorrection(ChStateDelta& Dy,             ///< result: computed Dy
                                      ChVectorDynamic<>& L,         ///< result: computed lagrangian multipliers, if any
                                      const ChVectorDynamic<>& R,   ///< the R residual
                                      const ChVectorDynamic<>& Qc,  ///< the Qc residual
                                      const double a,               ///< the factor in c_a*H
                                      const double b,               ///< the factor in c_b*dF/dy
                                      const ChState& y,             ///< current state y
                                      const double T,               ///< current time T
                                      const double dt,              ///< timestep (if needed)
                                      bool force_state_scatter = true  ///< if false, y and T are not scattered to the
                                      /// system, assuming that someone has done
                                      /// StateScatter just before
                                      ) {
        throw ChException("StateSolveCorrection() not implemented, implicit integrators cannot be used. ");
    };

    /// Increment a vector R (usually the residual in a Newton Raphson iteration
    /// for solving an implicit integration step) with a term that has H multiplied a given vector w:
    ///    R += c*H*w
    virtual void LoadResidual_Hv(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                 const ChVectorDynamic<>& v,  ///< the v vector
                                 const double c               ///< a scaling factor
                                 ) {
        throw ChException("LoadResidual_Hv() not implemented, implicit integrators cannot be used. ");
    };

    /// Increment a vector R (usually the residual in a Newton Raphson iteration
    /// for solving an implicit integration step) with the term c*F:
    ///    R += c*F
    virtual void LoadResidual_F(ChVectorDynamic<>& R,  ///< result: the R residual, R += c*F
                                const double c         ///< a scaling factor
                                ) {
        throw ChException("LoadResidual_F() not implemented, implicit integrators cannot be used. ");
    };

    /// Increment a vector R (usually the residual in a Newton Raphson iteration
    /// for solving an implicit integration step) with the term Cq'*L:
    ///    R += c*Cq'*L
    virtual void LoadResidual_CqL(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                  const ChVectorDynamic<>& L,  ///< the L vector
                                  const double c               ///< a scaling factor
                                  ) {
        throw ChException("LoadResidual_CqL() not implemented, implicit integrators cannot be used. ");
    };

    /// Increment a vector Qc (usually the residual in a Newton Raphson iteration
    /// for solving an implicit integration step, constraint part) with the term C:
    ///    Qc += c*C
    virtual void LoadConstraint_C(ChVectorDynamic<>& Qc,        ///< result: the Qc residual, Qc += c*C
                                  const double c,               ///< a scaling factor
                                  const bool do_clamp = false,  ///< enable optional clamping of Qc
                                  const double mclam = 1e30     ///< clamping value
                                  ) {
        throw ChException("LoadConstraint_C() not implemented, implicit integrators cannot be used. ");
    };

    /// Increment a vector Qc (usually the residual in a Newton Raphson iteration
    /// for solving an implicit integration step, constraint part) with the term Ct = partial derivative dC/dt:
    ///    Qc += c*Ct
    virtual void LoadConstraint_Ct(ChVectorDynamic<>& Qc,  ///< result: the Qc residual, Qc += c*Ct
                                   const double c          ///< a scaling factor
                                   ) {
        throw ChException("LoadConstraint_Ct() not implemented, implicit integrators cannot be used. ");
    };
};

/// Special subcase: II-order differential system.
/// Interface class for all objects that support time integration
/// with state y that is second order, that is
///     y = {x, v} , dy/dt={v, a}
/// with positions x, speeds v=dx/dt, accelerations a=ddx/dtdt.
/// This means that special integrators can be used that can exploit
/// the special nature of those systems.
/// You can inherit your class from this class, by implementing those
/// four functions. By doing this, you can use time integrators from
/// the ChTimestepper hierarchy to integrate in time.

class ChApi ChIntegrableIIorder : public ChIntegrable {
  public:
    /// Tells the number of position coordinates x in y = {x, v}
    /// Children classes MUST implement this!
    virtual int GetNcoords_x() = 0;

    /// Tells the number of speed coordinates of v in y = {x, v} and  dy/dt={v, a}
    /// This is a base implementation that works in many cases where dim(v) = dim(x), but
    /// might be less ex. if x uses quaternions and v uses angular vel.
    virtual int GetNcoords_v() { return GetNcoords_x(); };

    /// Tells the number of acceleration coordinates of a in dy/dt={v, a}
    /// This is a default implementation that works in almost all cases, as dim(a) = dim(v),
    virtual int GetNcoords_a() { return GetNcoords_v(); };

    /// This sets up the system state with separate II order components x, v, a
    /// for y = {x, v} and  dy/dt={v, a}
    virtual void StateSetup(ChState& x, ChStateDelta& v, ChStateDelta& a) {
        x.Resize(GetNcoords_x());
        v.Resize(GetNcoords_v());
        a.Resize(GetNcoords_a());
    };

    /// From system to state y={x,v}
    /// Optionally, they will copy system private state, if any, to y={x,v}
    virtual void StateGather(ChState& x, ChStateDelta& v, double& T){};

    /// From state Y={x,v} to system.
    /// This is important because it is called by time integrators all times
    /// they modify the Y state. In some cases, the ChIntegrable object might
    /// contain dependent data structures that might need an update at each change of Y,
    /// if so, this function must be overridden.
    virtual void StateScatter(const ChState& x, const ChStateDelta& v, const double T){};

    /// Optional: the integrable object might contain last computed state derivative, some integrators might use it.
    virtual void StateGatherAcceleration(ChStateDelta& a){};

    /// Optional: the integrable object might contain last computed state derivative, some integrators might use it.
    virtual void StateScatterAcceleration(const ChStateDelta& a){};

    /// a = f(x,v,t)
    /// Given current state y={x,v} , computes acceleration a in the state derivative dy/dt={v,a} and
    /// lagrangian multipliers L (if any).
    /// NOTE: some solvers (ex in DVI) cannot compute a classical derivative
    /// dy/dt when v is a function of bounded variation, and f or L are distributions (ex
    /// when there are impulses and discontinuities), so they compute a finite Dv through a finite dt:
    /// this is the reason why this function has an optional parameter dt. In a DVI setting,
    /// one computes Dv, and returns Dv*(1/dt) here in Dvdt parameter; if the original Dv has to be known,
    /// just multiply Dvdt*dt later. The same for impulses: a DVI would compute impulses I, and return L=I*(1/dt).
    /// NOTE! children classes must take care of calling StateScatter(y,T) before
    /// computing Dy, only if force_state_scatter = true (otherwise it is assumed state is already in sync)
    /// NOTE! children classes must take care of resizing Dv if needed.
    virtual void StateSolveA(ChStateDelta& Dvdt,              ///< result: computed a for a=dv/dt
                             ChVectorDynamic<>& L,            ///< result: computed lagrangian multipliers, if any
                             const ChState& x,                ///< current state, x
                             const ChStateDelta& v,           ///< current state, v
                             const double T,                  ///< current time T
                             const double dt,                 ///< timestep (if needed)
                             bool force_state_scatter = true  ///< if false, x,v and T are not scattered to the system,
                             /// assuming that someone has done StateScatter just before
                             ) = 0;

    /// Perform x_new = x + dx    for x in    Y = {x, dx/dt}
    /// This is a base implementation that works in many cases, but it can be overridden
    /// in the case that x contains quaternions for rotations
    /// NOTE: the system is not updated automatically after the state increment, so one might
    /// need to call StateScatter() if needed.
    virtual void StateIncrementX(ChState& x_new,         ///< resulting x_new = x + Dx
                                 const ChState& x,       ///< initial state x
                                 const ChStateDelta& Dx  ///< state increment Dx
                                 ) {
        x_new.Resize(x.GetRows(), x.GetColumns());
        for (int i = 0; i < x.GetRows(); ++i) {
            x_new(i) = x(i) + Dx(i);
        }
    }

    //
    // Functions required by implicit integration schemes
    //

    /// Assuming an explicit ODE in the form
    ///        M*a = F(x,v,t)
    /// Assuming an explicit DAE in the form
    ///        M*a = F(x,v,t) + Cq'*L
    ///     C(x,t) = 0
    /// this must compute the solution of the change Du (in a or v or x) to satisfy
    /// the equation required in a Newton Raphson iteration for an
    /// implicit integrator equation.
    /// If in ODE case:
    ///  Du = [ c_a*M + c_v*dF/dv + c_x*dF/dx ]^-1 * R
    ///  Du = [ G ]^-1 * R
    /// If with DAE constraints:
    ///  |Du| = [ G   Cq' ]^-1 * | R |
    ///  |DL|   [ Cq  0   ]      | Qc|
    /// where R is a given residual, dF/dv and dF/dx, dF/dv are jacobians (that are also
    /// -R and -K, damping and stiffness (tangent) matrices in many mechanical problems, note the minus sign!).
    /// It is up to the child class how to solve such linear system.
    virtual void StateSolveCorrection(ChStateDelta& Dv,             ///< result: computed Dv
                                      ChVectorDynamic<>& L,         ///< result: computed lagrangian multipliers, if any
                                      const ChVectorDynamic<>& R,   ///< the R residual
                                      const ChVectorDynamic<>& Qc,  ///< the Qc residual
                                      const double c_a,             ///< the factor in c_a*M
                                      const double c_v,             ///< the factor in c_v*dF/dv
                                      const double c_x,             ///< the factor in c_x*dF/dv
                                      const ChState& x,             ///< current state, x part
                                      const ChStateDelta& v,        ///< current state, v part
                                      const double T,               ///< current time T
                                      bool force_state_scatter = true  ///< if false, x,v and T are not scattered to the
                                      /// system, assuming that someone has done
                                      /// StateScatter just before
                                      ) {
        throw ChException("StateSolveCorrection() not implemented, implicit integrators cannot be used. ");
    };

    /// Assuming   M*a = F(x,v,t) + Cq'*L
    ///         C(x,t) = 0
    /// increment a vector R (usually the residual in a Newton Raphson iteration
    /// for solving an implicit integration step) with the term c*F:
    ///    R += c*F
    virtual void LoadResidual_F(ChVectorDynamic<>& R,  ///< result: the R residual, R += c*F
                                const double c         ///< a scaling factor
                                ) {
        throw ChException("LoadResidual_F() not implemented, implicit integrators cannot be used. ");
    };

    /// Assuming   M*a = F(x,v,t) + Cq'*L
    ///         C(x,t) = 0
    /// increment a vector R (usually the residual in a Newton Raphson iteration
    /// for solving an implicit integration step) with a term that has M multiplied a given vector w:
    ///    R += c*M*w
    virtual void LoadResidual_Mv(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                 const ChVectorDynamic<>& w,  ///< the w vector
                                 const double c               ///< a scaling factor
                                 ) {
        throw ChException("LoadResidual_Mv() not implemented, implicit integrators cannot be used. ");
    };

    /// Assuming   M*a = F(x,v,t) + Cq'*L
    ///         C(x,t) = 0
    /// increment a vectorR (usually the residual in a Newton Raphson iteration
    /// for solving an implicit integration step) with the term Cq'*L:
    ///    R += c*Cq'*L
    virtual void LoadResidual_CqL(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                  const ChVectorDynamic<>& L,  ///< the L vector
                                  const double c               ///< a scaling factor
                                  ) {
        throw ChException("LoadResidual_CqL() not implemented, implicit integrators cannot be used. ");
    };

    /// Assuming   M*a = F(x,v,t) + Cq'*L
    ///         C(x,t) = 0
    /// Increment a vector Qc (usually the residual in a Newton Raphson iteration
    /// for solving an implicit integration step, constraint part) with the term C:
    ///    Qc += c*C
    virtual void LoadConstraint_C(ChVectorDynamic<>& Qc,        ///< result: the Qc residual, Qc += c*C
                                  const double c,               ///< a scaling factor
                                  const bool do_clamp = false,  ///< enable optional clamping of Qc
                                  const double mclam = 1e30     ///< clamping value
                                  ) {
        throw ChException("LoadConstraint_C() not implemented, implicit integrators cannot be used. ");
    };

    /// Assuming   M*a = F(x,v,t) + Cq'*L
    ///         C(x,t) = 0
    /// Increment a vector Qc (usually the residual in a Newton Raphson iteration
    /// for solving an implicit integration step, constraint part) with the term Ct = partial derivative dC/dt:
    ///    Qc += c*Ct
    virtual void LoadConstraint_Ct(ChVectorDynamic<>& Qc,  ///< result: the Qc residual, Qc += c*Ct
                                   const double c          ///< a scaling factor
                                   ) {
        throw ChException("LoadConstraint_Ct() not implemented, implicit integrators cannot be used. ");
    };

    // ---------------------

    //
    // OVERRIDE ChIntegrable BASE MEMBERS TO SUPPORT 1st ORDER INTEGRATORS:
    //

    // The following functions override the members in the base ChIntegrable class
    // using a default bookkeeping that allows you to use timesteppers for 1st
    // order integration with this II order system. The trick is that
    // the x and v state parts are assembled into y={x,y} as this is needed,
    // and viceversa. Same for dy/dt={v,a}.
    // NOTE: PERFORMANCE WARNING: this default bookkeeping requires temporary allocation
    //  and deallocation of temporary vectors and some copies.
    // NOTE: performance penalty is not a big issue since one should try to use
    //   custom II order timesteppers if possible.
    // NOTE: children classes does not need to override those default functions.
    //

    /// Tells the number of coordinates in the state y.
    /// (overrides base - just a fallback to enable using with plain 1st order timesteppers)
    virtual int GetNcoords_y() { return GetNcoords_x() + GetNcoords_v(); };

    /// Tells the number of coordinates in the state increment Dy.
    /// (overrides base - just a fallback to enable using with plain 1st order timesteppers)
    virtual int GetNcoords_dy() { return GetNcoords_v() + GetNcoords_a(); };

    /// From system to state y={x,v}
    /// (overrides base - just a fallback to enable using with plain 1st order timesteppers)
    /// PERFORMANCE WARNING! temporary vectors allocated on heap. This is only to support
    /// compatibility with 1st order integrators.
    virtual void StateGather(ChState& y, double& T) {
        ChState mx(GetNcoords_x(), y.GetIntegrable());
        ChStateDelta mv(GetNcoords_v(), y.GetIntegrable());
        this->StateGather(mx, mv, T);
        y.PasteMatrix(&mx, 0, 0);
        y.PasteMatrix(&mv, GetNcoords_x(), 0);
    };

    /// From state y={x,v} to system.
    /// (overrides base - just a fallback to enable using with plain 1st order timesteppers)
    /// PERFORMANCE WARNING! temporary vectors allocated on heap. This is only to support
    /// compatibility with 1st order integrators.
    virtual void StateScatter(const ChState& y, const double T) {
        ChState mx(GetNcoords_x(), y.GetIntegrable());
        ChStateDelta mv(GetNcoords_v(), y.GetIntegrable());
        mx.PasteClippedMatrix(&y, 0, 0, GetNcoords_x(), 1, 0, 0);
        mv.PasteClippedMatrix(&y, GetNcoords_x(), 0, GetNcoords_v(), 1, 0, 0);
        this->StateScatter(mx, mv, T);
    };

    /// The integrable object might contain last computed state derivative, some integrators might reuse it.
    /// (overrides base - just a fallback to enable using with plain 1st order timesteppers)
    /// PERFORMANCE WARNING! temporary vectors allocated on heap. This is only to support
    /// compatibility with 1st order integrators.
    virtual void StateGatherDerivative(ChStateDelta& Dydt) {
        ChStateDelta mv(GetNcoords_v(), Dydt.GetIntegrable());
        ChStateDelta ma(GetNcoords_v(), Dydt.GetIntegrable());
        this->StateGatherAcceleration(ma);
        Dydt.PasteMatrix(&mv, 0, 0);
        Dydt.PasteMatrix(&ma, GetNcoords_v(), 0);
    };

    /// The integrable object might need to store last computed state derivative, ex. for plotting etc.
    /// NOTE! the velocity in dsdt={v,a} is not scattered to the II order integrable, only acceleration is scattered!
    /// (overrides base - just a fallback to enable using with plain 1st order timesteppers)
    /// PERFORMANCE WARNING! temporary vectors allocated on heap. This is only to support
    /// compatibility with 1st order integrators.
    virtual void StateScatterDerivative(const ChStateDelta& Dydt) {
        ChStateDelta ma(GetNcoords_v(), Dydt.GetIntegrable());
        ma.PasteClippedMatrix(&Dydt, GetNcoords_v(), 0, GetNcoords_v(), 1, 0, 0);
        this->StateScatterAcceleration(ma);
    }

    /// Perform y_new = y + Dy
    /// This is a base implementation that works in many cases.
    /// It calls StateIncrementX() if used on x in y={x, dx/dt}.
    /// It calls StateIncrementX() for x, and a normal sum for dx/dt if used on y in y={x, dx/dt}
    virtual void StateIncrement(ChState& y_new,         ///< resulting y_new = y + Dy
                                const ChState& y,       ///< initial state y
                                const ChStateDelta& Dy  ///< state increment Dy
                                ) {
        if (y.GetRows() == this->GetNcoords_x()) {
            // Incrementing the x part only, user provided only x  in y={x, dx/dt}
            this->StateIncrementX(y_new, y, Dy);

            return;
        }
        if (y.GetRows() == this->GetNcoords_y()) {
            // Incrementing y in y={x, dx/dt}.
            // PERFORMANCE WARNING! temporary vecotrs allocated on heap. This is only to support
            // compatibility with 1st order integrators.
            ChState mx(this->GetNcoords_x(), y.GetIntegrable());
            ChStateDelta mv(this->GetNcoords_v(), y.GetIntegrable());
            mx.PasteClippedMatrix(&y, 0, 0, this->GetNcoords_x(), 1, 0, 0);
            mv.PasteClippedMatrix(&y, this->GetNcoords_x(), 0, this->GetNcoords_v(), 1, 0, 0);
            ChStateDelta mDx(this->GetNcoords_v(), y.GetIntegrable());
            ChStateDelta mDv(this->GetNcoords_a(), y.GetIntegrable());
            mDx.PasteClippedMatrix(&Dy, 0, 0, this->GetNcoords_v(), 1, 0, 0);
            mDv.PasteClippedMatrix(&Dy, this->GetNcoords_v(), 0, this->GetNcoords_a(), 1, 0, 0);
            ChState mx_new(this->GetNcoords_x(), y.GetIntegrable());
            ChStateDelta mv_new(this->GetNcoords_v(), y.GetIntegrable());

            this->StateIncrementX(mx_new, mx, mDx);  // increment positions
            mv_new = mv + mDv;                       // increment speeds

            y_new.PasteMatrix(&mx_new, 0, 0);
            y_new.PasteMatrix(&mv_new, this->GetNcoords_x(), 0);
            return;
        }
        throw ChException("StateIncrement() called with a wrong number of elements");
    }

    /// dy/dt = f(y,t)
    /// (overrides base - just a fallback to enable using with plain 1st order timesteppers)
    /// PERFORMANCE WARNING! temporary vectors allocated on heap. This is only to support
    /// compatibility with 1st order integrators.
    virtual void StateSolve(ChStateDelta& dydt,              ///< result: computed dydt
                            ChVectorDynamic<>& L,            ///< result: computed lagrangian multipliers, if any
                            const ChState& y,                ///< current state y
                            const double T,                  ///< current time T
                            const double dt,                 ///< timestep (if needed, ex. in DVI)
                            bool force_state_scatter = true  ///< if false, y and T are not scattered to the system,
                            /// assuming that someone has done StateScatter just before
                            ) {
        ChState mx(this->GetNcoords_x(), y.GetIntegrable());
        ChStateDelta mv(this->GetNcoords_v(), y.GetIntegrable());
        mx.PasteClippedMatrix(&y, 0, 0, this->GetNcoords_x(), 1, 0, 0);
        mv.PasteClippedMatrix(&y, this->GetNcoords_x(), 0, this->GetNcoords_v(), 1, 0, 0);
        ChStateDelta ma(this->GetNcoords_v(), y.GetIntegrable());

        this->StateSolveA(ma, L, mx, mv, T, dt, force_state_scatter);  // Solve with custom II order solver

        dydt.PasteMatrix(&mv, 0, 0);
        dydt.PasteMatrix(&ma, this->GetNcoords_v(), 0);
    }

    /// This was for Ist order implicit integrators, but here we disable it.
    virtual void StateSolveCorrection(ChStateDelta& Dy,             ///< result: computed Dy
                                      ChVectorDynamic<>& L,         ///< result: computed lagrangian multipliers, if any
                                      const ChVectorDynamic<>& R,   ///< the R residual
                                      const ChVectorDynamic<>& Qc,  ///< the Qc residual
                                      const double a,               ///< the factor in c_a*H
                                      const double b,               ///< the factor in c_b*dF/dy
                                      const ChState& y,             ///< current state y
                                      const double T,               ///< current time T
                                      const double dt,              ///< timestep (if needed)
                                      bool force_state_scatter = true  ///< if false, y and T are not scattered to the
                                      /// system, assuming that someone has done
                                      /// StateScatter just before
                                      ) {
        throw ChException(
            "StateSolveCorrection() not implemented for ChIntegrableIIorder, implicit integrators for Ist order cannot "
            "be used. ");
    };
};

/// This class is like ChIntegrableIIorder but it implements StateSolveA
/// using the functions that are used also for implicit integrators, so you need
/// to implement only the StateSolveCorrection  LoadResidual... and LoadConstraint...
/// functions.

class ChApi ChIntegrableIIorderEasy : public ChIntegrableIIorder {
  public:
    /// a = f(x,v,t)
    /// Given current state y={x,v} , computes acceleration a in the state derivative dy/dt={v,a} and
    /// lagrangian multipliers L (if any).
    /// This is a fallback that provides a default computation using the same functions
    /// that are used for the implicit integrators.
    /// WARNING: it avoids the computation of the analytical expression of Qc, but it
    /// requires three StateScatter updates!
    virtual void StateSolveA(ChStateDelta& Dvdt,              ///< result: computed a for a=dv/dt
                             ChVectorDynamic<>& L,            ///< result: computed lagrangian multipliers, if any
                             const ChState& x,                ///< current state, x
                             const ChStateDelta& v,           ///< current state, v
                             const double T,                  ///< current time T
                             const double dt,                 ///< timestep (if needed)
                             bool force_state_scatter = true  ///< if false, x,v and T are not scattered to the system,
                             /// assuming that someone has done StateScatter just before
                             ) {
        if (force_state_scatter)
            this->StateScatter(x, v, T);

        ChVectorDynamic<> R(this->GetNcoords_v());
        ChVectorDynamic<> Qc(this->GetNconstr());
        const double Delta = 1e-6;

        this->LoadResidual_F(R, 1.0);

        this->LoadConstraint_C(Qc, -2.0 / (Delta * Delta));

        // numerical differentiation to get the Qc term in constraints
        ChStateDelta dx(v);
        dx *= Delta;
        ChState xdx(x.GetRows(), this);

        this->StateIncrement(xdx, x, dx);
        this->StateScatter(xdx, v, T + Delta);
        this->LoadConstraint_C(Qc, 1.0 / (Delta * Delta));

        this->StateIncrement(xdx, x, -dx);
        this->StateScatter(xdx, v, T - Delta);
        this->LoadConstraint_C(Qc, 1.0 / (Delta * Delta));

        this->StateScatter(x, v, T);  // back to original state

        this->StateSolveCorrection(Dvdt, L, R, Qc, 1.0, 0, 0, x, v, T, false);
    }
};

/// This is a custom operator "+" that takes care of incremental update
/// of a state y by an increment Dy, if one types "y_new = y+Dy", by calling
/// the specialized StateIncrement() in the ChIntegrable (if any, otherwise
/// it will be a simple vector sum).

inline ChState operator+(const ChState& y, const ChStateDelta& Dy) {
    ChState result(y.GetRows(), y.GetIntegrable());
    y.GetIntegrable()->StateIncrement(result, y, Dy);
    return result;
}
inline ChState& operator+=(ChState& y, const ChStateDelta& Dy) {
    ChState tmp_y(y);
    y.GetIntegrable()->StateIncrement(y, tmp_y, Dy);
    return y;
}

/// This is a custom operator "+" that takes care of incremental update
/// of a state y by an increment Dy, if one types "y_new = Dy+y", by calling
/// the specialized StateIncrement() in the ChIntegrable (if any, otherwise
/// it will be a simple vector sum).

inline ChState operator+(const ChStateDelta& Dy, const ChState& y) {
    ChState result(y.GetRows(), y.GetIntegrable());
    y.GetIntegrable()->StateIncrement(result, y, Dy);
    return result;
}

}  // END_OF_NAMESPACE____
#endif
