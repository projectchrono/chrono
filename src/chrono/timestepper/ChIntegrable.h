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

#ifndef CHINTEGRABLE_H
#define CHINTEGRABLE_H

#include <cstdlib>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMath.h"
#include "chrono/timestepper/ChState.h"

namespace chrono {

// -----------------------------------------------------------------------------

/// Helper class for lumping parameters
class ChLumpingParms {
public:
    ChLumpingParms(double Ck = 1000, double Cr = 0) : Ck_penalty(Ck), Cr_penalty(Cr), error(0) {};
    double error;   // store here the error done when trying lumping masses
    double Ck_penalty;   // stiffness penalty for constraints if any
    double Cr_penalty;      // damping penalty for constraints if any

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) {
        // version number
        archive.VersionWrite(1);
        // serialize all member data:
        archive << CHNVP(Ck_penalty);
        archive << CHNVP(Cr_penalty);
    }

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) {
        // version number
        /*int version =*/ archive.VersionRead();
        // stream in all member data:
        archive >> CHNVP(Ck_penalty);
        archive >> CHNVP(Cr_penalty);
    }
};


/// Interface class for all objects that support time integration.
/// Derived concrete classes can use time integrators for the ChTimestepper hierarchy.
class ChApi ChIntegrable {
  public:
    /// Return the number of coordinates in the state Y.
    virtual int GetNcoords_y() = 0;

    /// Return the number of coordinates in the state increment.
    /// This is a base implementation that works in many cases where dim(Y) = dim(dy),
    /// but it can be overridden in the case that y contains quaternions for rotations
    /// rather than simple y+dy
    virtual int GetNcoords_dy() { return GetNcoords_y(); }

    /// Return the number of lagrangian multipliers (constraints).
    /// By default returns 0.
    virtual int GetNconstr() { return 0; }

    /// Set up the system state.
    virtual void StateSetup(ChState& y, ChStateDelta& dy) {
        y.resize(GetNcoords_y());
        dy.resize(GetNcoords_dy());
    }

    /// Gather system state in specified array.
    /// Optionally, they will copy system private state, if any, to Y.
    virtual void StateGather(ChState& y, double& T) {}

    /// Scatter the states from the provided array to the system.
    /// This function is called by time integrators every time they modify the Y state.
    /// In some cases, the ChIntegrable object might contain dependent data structures
    /// that might need an update at each change of Y. If so, this function must be overridden.
    virtual void StateScatter(const ChState& y, const double T, bool full_update) {}

    /// Gather from the system the state derivatives in specified array.
    /// Optional: the integrable object might contain last computed state derivative, some integrators might reuse it.
    virtual void StateGatherDerivative(ChStateDelta& Dydt) {}

    /// Scatter the state derivatives from the provided array to the system.
    /// Optional: the integrable object might need to store last computed state derivative, ex. for plotting etc.
    virtual void StateScatterDerivative(const ChStateDelta& Dydt) {}

    /// Gather from the system the Lagrange multipliers in specified array.
    /// Optional: the integrable object might contain Lagrange multipliers (reaction in constraints)
    virtual void StateGatherReactions(ChVectorDynamic<>& L) {}

    /// Scatter the Lagrange multipliers from the provided array to the system.
    /// Optional: the integrable object might contain Lagrange multipliers (reaction in constraints)
    virtual void StateScatterReactions(const ChVectorDynamic<>& L) {}

    /// Solve for state derivatives: dy/dt = f(y,t).
    /// Given current state y , computes the state derivative dy/dt and Lagrange multipliers L (if any).
    /// NOTE: some solvers (ex in DVI) cannot compute a classical derivative dy/dt when v is a function of
    /// bounded variation, and f or L are distributions (e.g., when there are impulses and discontinuities),
    /// so they compute a finite Dy through a finite dt. This is the reason why this function has an optional
    /// parameter dt. In a DVI setting, one computes Dy, and returns Dy*(1/dt) here in Dydt parameter; if the
    /// original Dy has to be known, just multiply Dydt*dt. The same for impulses: a DVI would compute
    /// impulses I, and return L=I*(1/dt).
    /// NOTES:
    ///    - derived classes must take care of calling StateScatter(y,T) before computing Dy, only if
    ///      force_state_scatter = true (otherwise it is assumed state is already in sync)
    ///    - derived classes must take care of resizing Dy and L if needed.
    ///
    /// This function must return true if successful and false otherwise.
    virtual bool StateSolve(ChStateDelta& Dydt,        ///< result: computed Dydt
                            ChVectorDynamic<>& L,      ///< result: computed lagrangian multipliers, if any
                            const ChState& y,          ///< current state y
                            const double T,            ///< current time T
                            const double dt,           ///< timestep (if needed, ex. in DVI)
                            bool force_state_scatter,  ///< if false, y and T are not scattered to the system
                            bool full_update,          ///< if true, perform a full update during scatter
                            ChLumpingParms* lumping = nullptr  ///< if not null, uses lumped masses to avoid inverting a mass matrix, and uses penalty for constraints.
                            ) = 0;

    /// Increment state array: y_new = y + Dy.
    /// This is a base implementation that works in many cases, but it can be overridden
    /// in the case that y contains quaternions for rotations, in which case rot. exponential is needed
    /// instead of simply doing y+Dy.
    /// NOTE: the system is not updated automatically after the state increment, so one might
    /// need to call StateScatter().
    virtual void StateIncrement(ChState& y_new,         ///< resulting y_new = y + Dy
                                const ChState& y,       ///< initial state y
                                const ChStateDelta& Dy  ///< state increment Dy
                                );

    //
    // Functions required by implicit integration schemes
    //

    /// Assuming an explicit ODE
    /// <pre>
    ///    H*dy/dt = F(y,t)
    /// </pre>
    /// or an explicit DAE
    /// <pre>
    ///    H*dy/dt = F(y,t) + Cq*L
    ///     C(y,t) = 0
    /// </pre>
    /// this function must compute the state increment as required for a Newton iteration within an implicit integration
    /// scheme. 
    /// For an ODE:
    /// <pre>
    ///  Dy = [ c_a*H + c_b*dF/dy ]^-1 * R
    ///  Dy = [ G ]^-1 * R
    /// </pre>
    /// For a DAE with constraints:
    /// <pre>
    ///  | Dy| = [ G   Cq' ]^-1 * | R |
    ///  |-DL|   [ Cq  0   ]      |-Qc|
    /// </pre>
    /// where R and Qc are given residuals, and dF/dy is the Jacobian of F. 
    /// Note the sign of DL (the method returns DL, not -DL) and the sign of Qc 
    /// (the method expects Qc, not -Qc), because the linear system uses them with flipped signs.
    ///
    /// This function must return true if successful and false otherwise.
    virtual bool StateSolveCorrection(ChStateDelta& Dy,             ///< result: computed Dy
                                      ChVectorDynamic<>& DL,        ///< result: computed DL lagrangian multipliers. Note the sign.
                                      const ChVectorDynamic<>& R,   ///< the R residual
                                      const ChVectorDynamic<>& Qc,  ///< the Qc residual. Note the sign.
                                      const double a,               ///< the factor in c_a*H
                                      const double b,               ///< the factor in c_b*dF/dy
                                      const ChState& y,             ///< current state y
                                      const double T,               ///< current time T
                                      const double dt,              ///< timestep (if needed)
                                      bool force_state_scatter,     ///< if false, y is not scattered to the system
                                      bool full_update,             ///< if true, perform a full update during scatter
                                      bool force_setup              ///< if true, call the solver's Setup() function
    ) {
        throw ChException("StateSolveCorrection() not implemented, implicit integrators cannot be used. ");
    }

    /// Increment a vector R (usually the residual in a Newton Raphson iteration
    /// for solving an implicit integration step) with a term that has H multiplied a given vector w:
    ///    R += c*H*w
    virtual void LoadResidual_Hv(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                 const ChVectorDynamic<>& v,  ///< the v vector
                                 const double c               ///< a scaling factor
                                 ) {
        throw ChException("LoadResidual_Hv() not implemented, implicit integrators cannot be used. ");
    }

    /// Increment a vector R (usually the residual in a Newton Raphson iteration
    /// for solving an implicit integration step) with the term c*F:
    ///    R += c*F
    virtual void LoadResidual_F(ChVectorDynamic<>& R,  ///< result: the R residual, R += c*F
                                const double c         ///< a scaling factor
                                ) {
        throw ChException("LoadResidual_F() not implemented, implicit integrators cannot be used. ");
    }

    /// Increment a vector R (usually the residual in a Newton Raphson iteration
    /// for solving an implicit integration step) with the term Cq'*L:
    ///    R += c*Cq'*L
    virtual void LoadResidual_CqL(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                  const ChVectorDynamic<>& L,  ///< the L vector
                                  const double c               ///< a scaling factor
                                  ) {
        throw ChException("LoadResidual_CqL() not implemented, implicit integrators cannot be used. ");
    }

    /// Increment a vector Qc (usually the residual in a Newton Raphson iteration
    /// for solving an implicit integration step, constraint part) with the term C:
    ///    Qc += c*C
    virtual void LoadConstraint_C(ChVectorDynamic<>& Qc,        ///< result: the Qc residual, Qc += c*C
                                  const double c,               ///< a scaling factor
                                  const bool do_clamp = false,  ///< enable optional clamping of Qc
                                  const double mclam = 1e30     ///< clamping value
                                  ) {
        throw ChException("LoadConstraint_C() not implemented, implicit integrators cannot be used. ");
    }

    /// Increment a vector Qc (usually the residual in a Newton Raphson iteration
    /// for solving an implicit integration step, constraint part) with the term Ct = partial derivative dC/dt:
    ///    Qc += c*Ct
    virtual void LoadConstraint_Ct(ChVectorDynamic<>& Qc,  ///< result: the Qc residual, Qc += c*Ct
                                   const double c          ///< a scaling factor
                                   ) {
        throw ChException("LoadConstraint_Ct() not implemented, implicit integrators cannot be used. ");
    }
};

// -----------------------------------------------------------------------------

/// Special subcase: II-order differential system.
/// Interface class for all objects that support time integration with state y that is second order:
///     y = {x, v} , dy/dt={v, a}
/// with positions x, speeds v=dx/dt, and accelerations a=ddx/dtdt.
/// Such systems permit the use of special integrators that can exploit the particular system structure.
class ChApi ChIntegrableIIorder : public ChIntegrable {
  public:
    /// Return the number of position coordinates x in y = {x, v}
    virtual int GetNcoords_x() = 0;

    /// Return the number of speed coordinates of v in y = {x, v} and  dy/dt={v, a}
    /// This is a base implementation that works in many cases where dim(v) = dim(x), but
    /// might be less ex. if x uses quaternions and v uses angular vel.
    virtual int GetNcoords_v() { return GetNcoords_x(); }

    /// Return the number of acceleration coordinates of a in dy/dt={v, a}
    /// This is a default implementation that works in almost all cases, as dim(a) = dim(v),
    virtual int GetNcoords_a() { return GetNcoords_v(); }

    /// Set up the system state with separate II order components x, v, a
    /// for y = {x, v} and  dy/dt={v, a}
    virtual void StateSetup(ChState& x, ChStateDelta& v, ChStateDelta& a);

    /// From system to state y={x,v}
    /// Optionally, they will copy system private state, if any, to y={x,v}
    virtual void StateGather(ChState& x, ChStateDelta& v, double& T) {}

    /// Scatter the states from the provided arrays to the system.
    /// This function is called by time integrators all times they modify the Y state.
    /// In some cases, the ChIntegrable object might contain dependent data structures
    /// that might need an update at each change of Y. If so, this function must be overridden.
    virtual void StateScatter(const ChState& x, const ChStateDelta& v, const double T, bool full_update) {}

    /// Gather from the system the acceleration in specified array.
    /// Optional: the integrable object might contain last computed state derivative, some integrators might use it.
    virtual void StateGatherAcceleration(ChStateDelta& a) {}

    /// Scatter the acceleration from the provided array to the system.
    /// Optional: the integrable object might contain last computed state derivative, some integrators might use it.
    virtual void StateScatterAcceleration(const ChStateDelta& a) {}

    /// Solve for accelerations: a = f(x,v,t)
    /// Given current state y={x,v} , computes acceleration a in the state derivative dy/dt={v,a} and
    /// lagrangian multipliers L (if any).
    /// NOTES
    ///  - some solvers (ex in DVI) cannot compute a classical derivative dy/dt when v is a function
    ///    of bounded variation, and f or L are distributions (e.g., when there are impulses and
    ///    discontinuities), so they compute a finite Dv through a finite dt. This is the reason why
    ///    this function has an optional parameter dt. In a DVI setting, one computes Dv, and returns
    ///    Dv*(1/dt) here in Dvdt parameter; if the original Dv has to be known, just multiply Dvdt*dt later.
    ///    The same for impulses: a DVI would compute impulses I, and return L=I*(1/dt).
    ///  - derived classes must take care of calling StateScatter(y,T) before computing Dy, only if
    ///    force_state_scatter = true (otherwise it is assumed state is already in sync)
    ///  - derived classes must take care of resizing Dv if needed.
    ///
    /// This function must return true if successful and false otherwise.
    ///
    /// This default implementation uses the same functions already used for implicit integration.
    /// WARNING: this implementation avoids the computation of the analytical expression for Qc, but
    /// at the cost of three StateScatter updates (always complete updates).
    virtual bool StateSolveA(ChStateDelta& Dvdt,        ///< result: computed a for a=dv/dt
                             ChVectorDynamic<>& L,      ///< result: computed lagrangian multipliers, if any
                             const ChState& x,          ///< current state, x
                             const ChStateDelta& v,     ///< current state, v
                             const double T,            ///< current time T
                             const double dt,           ///< timestep (if needed)
                             bool force_state_scatter,  ///< if false, x,v and T are not scattered to the system
                             bool full_update,          ///< if true, perform a full update during scatter
                             ChLumpingParms* lumping = nullptr  ///< if not null, uses lumped masses to avoid inverting a mass matrix, and uses penalty for constraints.
    );

    /// Increment state array:  x_new = x + dx    for x in    Y = {x, dx/dt}
    /// This is a base implementation that works in many cases, but it can be overridden
    /// in the case that x contains quaternions for rotations
    /// NOTE: the system is not updated automatically after the state increment, so one might
    /// need to call StateScatter() if needed.
    virtual void StateIncrementX(ChState& x_new,         ///< resulting x_new = x + Dx
                                 const ChState& x,       ///< initial state x
                                 const ChStateDelta& Dx  ///< state increment Dx
                                 );

    //
    // Functions required by implicit integration schemes
    //

    /// Assuming an explicit ODE in the form
    /// <pre>
    ///        M*a = F(x,v,t)
    /// </pre>
    /// or an explicit DAE in the form
    /// <pre>
    ///        M*a = F(x,v,t) + Cq'*L
    ///     C(x,t) = 0
    /// </pre>
    /// this function must compute the solution of the change Du (in a or v or x) for a Newton iteration within an
    /// implicit integration scheme. 
    /// If in ODE case: <pre>
    ///  Du = [ c_a*M + c_v*dF/dv + c_x*dF/dx ]^-1 * R
    ///  Du = [ H ]^-1 * R
    /// </pre>
    /// If with DAE constraints:
    /// <pre>
    ///  | Du| = [ H   Cq' ]^-1 * | R |
    ///  |-DL|   [ Cq  0   ]      |-Qc|
    /// </pre>
    /// where R is a given residual, H=c_a*M + c_v*dF/dv + c_x*dF/dx  (here F/dv and dF/dx, dF/dv are jacobians, that are also
    /// -R and -K, damping and stiffness (tangent) matrices in many mechanical problems, note the minus sign!).
    /// Note the sign of DL (the method must return DL, not -DL) and the sign of Qc 
    /// (the method expects Qc, not -Qc), because the linear system uses them with flipped signs.
    /// It is up to the derived class how to solve such linear system.
    ///
    /// This function must return true if successful and false otherwise.
    virtual bool StateSolveCorrection(
        ChStateDelta& Dv,             ///< result: computed Dv
        ChVectorDynamic<>& L,         ///< result: computed lagrangian multipliers, if any
        const ChVectorDynamic<>& R,   ///< the R residual
        const ChVectorDynamic<>& Qc,  ///< the Qc residual
        const double c_a,             ///< the factor in c_a*M
        const double c_v,             ///< the factor in c_v*dF/dv
        const double c_x,             ///< the factor in c_x*dF/dv
        const ChState& x,             ///< current state, x part
        const ChStateDelta& v,        ///< current state, v part
        const double T,               ///< current time T
        bool force_state_scatter,     ///< if false, x and v are not scattered to the system
        bool full_update,             ///< if true, perform a full update during scatter
        bool force_setup              ///< if true, call the solver's Setup() function
    ) {
        throw ChException("StateSolveCorrection() not implemented, implicit integrators cannot be used. ");
    }

    /// Assuming   M*a = F(x,v,t) + Cq'*L
    ///         C(x,t) = 0
    /// increment a vector R (usually the residual in a Newton Raphson iteration
    /// for solving an implicit integration step) with the term c*F:
    ///    R += c*F
    virtual void LoadResidual_F(ChVectorDynamic<>& R,  ///< result: the R residual, R += c*F
                                const double c         ///< a scaling factor
                                ) override {
        throw ChException("LoadResidual_F() not implemented, integrator cannot be used. ");
    }

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
    }

    /// Adds the lumped mass to a Md vector, representing a mass diagonal matrix. Used by lumped explicit integrators.
    /// If mass lumping is impossible or approximate, adds scalar error to "error" parameter.
    ///    Md += c*diag(M)    or   Md += c*HRZ(M)
    virtual void LoadLumpedMass_Md(ChVectorDynamic<>& Md,  ///< result: Md vector, diagonal of the lumped mass matrix
                                   double& err,            ///< result: not touched if lumping does not introduce errors
                                   const double c          ///< a scaling factor
    ) {
        throw ChException(
            "LoadLumpedMass_Md() not implemented, explicit integrators with mass lumping cannot be used. ");
    }

    /// Assuming   M*a = F(x,v,t) + Cq'*L
    ///         C(x,t) = 0
    /// increment a vectorR (usually the residual in a Newton Raphson iteration
    /// for solving an implicit integration step) with the term Cq'*L:
    ///    R += c*Cq'*L
    virtual void LoadResidual_CqL(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                  const ChVectorDynamic<>& L,  ///< the L vector
                                  const double c               ///< a scaling factor
                                  ) override {
        throw ChException("LoadResidual_CqL() not implemented, implicit integrators cannot be used. ");
    }

    /// Assuming   M*a = F(x,v,t) + Cq'*L
    ///         C(x,t) = 0
    /// Increment a vector Qc (usually the residual in a Newton Raphson iteration
    /// for solving an implicit integration step, constraint part) with the term C:
    ///    Qc += c*C
    virtual void LoadConstraint_C(ChVectorDynamic<>& Qc,        ///< result: the Qc residual, Qc += c*C
                                  const double c,               ///< a scaling factor
                                  const bool do_clamp = false,  ///< enable optional clamping of Qc
                                  const double mclam = 1e30     ///< clamping value
                                  ) override {
        throw ChException("LoadConstraint_C() not implemented, implicit integrators cannot be used. ");
    }

    /// Assuming   M*a = F(x,v,t) + Cq'*L
    ///         C(x,t) = 0
    /// Increment a vector Qc (usually the residual in a Newton Raphson iteration
    /// for solving an implicit integration step, constraint part) with the term Ct = partial derivative dC/dt:
    ///    Qc += c*Ct
    virtual void LoadConstraint_Ct(ChVectorDynamic<>& Qc,  ///< result: the Qc residual, Qc += c*Ct
                                   const double c          ///< a scaling factor
                                   ) override {
        throw ChException("LoadConstraint_Ct() not implemented, implicit integrators cannot be used. ");
    }

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

    /// Return the number of coordinates in the state Y.
    /// (overrides base - just a fallback to enable using with plain 1st order timesteppers)
    virtual int GetNcoords_y() override { return GetNcoords_x() + GetNcoords_v(); }

    /// Return the number of coordinates in the state increment.
    /// (overrides base - just a fallback to enable using with plain 1st order timesteppers)
    virtual int GetNcoords_dy() override { return GetNcoords_v() + GetNcoords_a(); }

    /// Gather system state in specified array.
    /// (overrides base - just a fallback to enable using with plain 1st order timesteppers)
    /// PERFORMANCE WARNING! temporary vectors allocated on heap. This is only to support
    /// compatibility with 1st order integrators.
    virtual void StateGather(ChState& y, double& T) override;

    /// Scatter the states from the provided array to the system.
    /// (overrides base - just a fallback to enable using with plain 1st order timesteppers)
    /// PERFORMANCE WARNING! temporary vectors allocated on heap. This is only to support
    /// compatibility with 1st order integrators.
    virtual void StateScatter(const ChState& y, const double T, bool full_update) override;

    /// Gather from the system the state derivatives in specified array.
    /// The integrable object might contain last computed state derivative, some integrators might reuse it.
    /// (overrides base - just a fallback to enable using with plain 1st order timesteppers)
    /// PERFORMANCE WARNING! temporary vectors allocated on heap. This is only to support
    /// compatibility with 1st order integrators.
    virtual void StateGatherDerivative(ChStateDelta& Dydt) override;

    /// Scatter the state derivatives from the provided array to the system.
    /// The integrable object might need to store last computed state derivative, ex. for plotting etc.
    /// NOTE! the velocity in dsdt={v,a} is not scattered to the II order integrable, only acceleration is scattered!
    /// (overrides base - just a fallback to enable using with plain 1st order timesteppers)
    /// PERFORMANCE WARNING! temporary vectors allocated on heap. This is only to support
    /// compatibility with 1st order integrators.
    virtual void StateScatterDerivative(const ChStateDelta& Dydt) override;

    /// Increment state array: y_new = y + Dy.
    /// This is a base implementation that works in many cases.
    /// It calls StateIncrementX() if used on x in y={x, dx/dt}.
    /// It calls StateIncrementX() for x, and a normal sum for dx/dt if used on y in y={x, dx/dt}
    virtual void StateIncrement(ChState& y_new,         ///< resulting y_new = y + Dy
                                const ChState& y,       ///< initial state y
                                const ChStateDelta& Dy  ///< state increment Dy
                                ) override;

    /// Solve for state derivatives: dy/dt = f(y,t).
    /// (overrides base - just a fallback to enable using with plain 1st order timesteppers)
    /// PERFORMANCE WARNING! temporary vectors allocated on heap. This is only to support
    /// compatibility with 1st order integrators.
    virtual bool StateSolve(ChStateDelta& dydt,        ///< result: computed dydt
                            ChVectorDynamic<>& L,      ///< result: computed lagrangian multipliers, if any
                            const ChState& y,          ///< current state y
                            const double T,            ///< current time T
                            const double dt,           ///< timestep (if needed, ex. in DVI)
                            bool force_state_scatter,  ///< if false, y and T are not scattered to the system
                            bool full_update,          ///< if true, perform a full update during scatter
                            ChLumpingParms* lumping = nullptr  ///< if not null, uses lumped masses to avoid inverting a mass matrix, and uses penalty for constraints.
                            ) override;

    /// Override of method for Ist order implicit integrators.
    /// This is disabled here because implicit integrators for Ist order cannot be used for a IInd order system.
    virtual bool StateSolveCorrection(ChStateDelta& Dy,
                                      ChVectorDynamic<>& L,
                                      const ChVectorDynamic<>& R,
                                      const ChVectorDynamic<>& Qc,
                                      const double a,
                                      const double b,
                                      const ChState& y,
                                      const double T,
                                      const double dt,
                                      bool force_state_scatter,
                                      bool full_update,
                                      bool force_setup) override final {
        throw ChException(
            "StateSolveCorrection() not implemented for ChIntegrableIIorder, implicit integrators for Ist order cannot "
            "be used. ");
    }

  private:
    using ChIntegrable::StateSetup;
};

// -----------------------------------------------------------------------------

/// Custom operator "+" that takes care of incremental update of a state y by an increment Dy.
/// "y_new = y + Dy", invokes the specialized StateIncrement() in the ChIntegrable. If none is 
/// provided, this defaults to a simple vector sum
inline ChState operator+(const ChState& y, const ChStateDelta& Dy) {
    ChState result(y.size(), y.GetIntegrable());
    y.GetIntegrable()->StateIncrement(result, y, Dy);
    return result;
}
inline ChState& operator+=(ChState& y, const ChStateDelta& Dy) {
    ChState tmp_y(y);
    y.GetIntegrable()->StateIncrement(y, tmp_y, Dy);
    return y;
}

/// Custom operator "+" that takes care of incremental update of a state y by an increment Dy
/// "y_new = Dy + y", invokes the specialized StateIncrement() in the ChIntegrable. If none is 
/// provided, this defaults to a simple vector sum
inline ChState operator+(const ChStateDelta& Dy, const ChState& y) {
    ChState result(y.size(), y.GetIntegrable());
    y.GetIntegrable()->StateIncrement(result, y, Dy);
    return result;
}

/// Custom operator "-" that takes care of incremental update of a state y by an increment Dy.
/// "y_new = y - Dy", invokes the specialized StateIncrement() in the ChIntegrable. If none is 
/// provided, this defaults to a simple vector sum
inline ChState operator-(const ChState& y, const ChStateDelta& Dy) {
    ChState result(y.size(), y.GetIntegrable());
    y.GetIntegrable()->StateIncrement(result, y, Dy*-1);
    return result;
}
inline ChState& operator-=(ChState& y, const ChStateDelta& Dy) {
    ChState tmp_y(y);
    y.GetIntegrable()->StateIncrement(y, tmp_y, Dy*-1);
    return y;
}

/// Custom operator "-" that takes care of incremental update of a state y by an increment Dy
/// "y_new = Dy - y", invokes the specialized StateIncrement() in the ChIntegrable. If none is 
/// provided, this defaults to a simple vector sum
inline ChState operator-(const ChStateDelta& Dy, const ChState& y) {
    ChState result(y.size(), y.GetIntegrable());
    y.GetIntegrable()->StateIncrement(result, y, Dy*-1);
    return result;
}
}  // end namespace chrono

#endif
