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

#ifndef CH_TIMESTEPPER_IMPLICIT_H
#define CH_TIMESTEPPER_IMPLICIT_H

#include <array>

#include "chrono/timestepper/ChTimestepper.h"

namespace chrono {

/// @addtogroup chrono_timestepper
/// @{

/// Base class for implicit solvers (double inheritance)
class ChApi ChTimestepperImplicit{};

/// Base properties for implicit solvers.
/// Such integrators require solution of a nonlinear problem, solved using an iterative Newton process which requires
/// the solution of a linear system at each iteration. A full Newton method uses a current matrix (based on the system
/// Jacobian), updated and factorized at each iteration. Modified Newton methods use possibly out-of-date Jacobian
/// information; the Jacobian can be evaluated at the beginning of the step only, kept constant for the duration of the
/// entire simulation, or a Jacobian re-evaluation can be triggered automatically and adaptively, as necessary.
class ChApi ChTimestepperImplicitIterative : public ChTimestepperImplicit {
  public:
    /// Newton Jacobian update strategies.
    enum class JacobianUpdate {
        EVERY_ITERATION,  ///< Full Newton: Jacobian updated at every iteration
        EVERY_STEP,       ///< Jacobian updated at every step
        NEVER,            ///< Jacobian never updated
        AUTOMATIC         ///< Automatic Jacobian update
    };

    virtual ~ChTimestepperImplicitIterative() {}

    /// Set the max number of iterations using the Newton Raphson procedure.
    void SetMaxIters(int iters) { maxiters = iters; }

    /// Set the relative tolerance.
    /// This tolerance is optionally used by derived classes in the Newton-Raphson
    /// convergence test.
    void SetRelTolerance(double rel_tol) { reltol = rel_tol; }

    /// Set the absolute tolerances.
    /// These tolerances are optionally used by derived classes in the Newton-Raphson
    /// convergence test.  This version sets separate absolute tolerances for states
    /// and Lagrange multipliers.
    void SetAbsTolerances(double abs_tolS, double abs_tolL) {
        abstolS = abs_tolS;
        abstolL = abs_tolL;
    }

    /// Set the absolute tolerances.
    /// These tolerances are optionally used by derived classes in the Newton-Raphson
    /// convergence test.  This version sets equal absolute tolerances for states and
    /// Lagrange multipliers.
    void SetAbsTolerances(double abs_tol) {
        abstolS = abs_tol;
        abstolL = abs_tol;
    }

    /// Set the strategy for Jacobian update (default: EVERY_STEP).
    void SetJacobianUpdateMethod(JacobianUpdate method);

    /// Get the max number of iterations using the Newton Raphson procedure.
    double GetMaxIters() const { return maxiters; }

    /// Get the current Jacobian update startegy.
    JacobianUpdate GetJacobianUpdateMethod() const { return jacobian_update_method; }

    /// Return the number of iterations.
    unsigned int GetNumIterations() const { return numiters; }

    /// Return the number of calls to the solver's Setup function.
    unsigned int GetNumSetupCalls() const { return numsetups; }

    /// Return the number of calls to the solver's Solve function.
    unsigned int GetNumSolveCalls() const { return numsolves; }

    /// Get the last estimated convergence rate for the internal Newton solver.
    /// Note that an estimate can only be calculated after the 3rd iteration. For the first 2 iterations, the
    /// convergence rate estimate is set to 1.
    double GetEstimatedConvergenceRate() const { return convergence_rate; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) {
        // version number
        archive.VersionWrite(1);
        // serialize all member data:
        archive << CHNVP(maxiters);
        archive << CHNVP(reltol);
        archive << CHNVP(abstolS);
        archive << CHNVP(abstolL);
    }

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) {
        // version number
        /*int version =*/archive.VersionRead();
        // stream in all member data:
        archive >> CHNVP(maxiters);
        archive >> CHNVP(reltol);
        archive >> CHNVP(abstolS);
        archive >> CHNVP(abstolL);
    }

  protected:
    ChTimestepperImplicitIterative();

    /// Monitor flags controlling whether or not the Jacobian must be updated.
    /// If using JacobianUpdate::EVERY_ITERATION, a matrix update occurs:
    ///   - at every iteration
    /// If using JacobianUpdate::EVERY_STEP, a matrix update occurs:
    ///   - at the beginning of a step
    ///   - on a stepsize decrease
    ///   - if the Newton iteration does not converge with an out-of-date matrix
    /// If using JacobianUpdate::NEVER, a matrix update occurs:
    ///   - only at the beginning of the very first step
    /// If using JacobianUpdate::AUTOMATIC, a matrix update occurs:
    ///   - when appropriate. TODO: implement
    bool CheckJacobianUpdateRequired(int iteration, bool previous_substep_converged);

    /// Check convergence of Newton process.
    bool CheckConvergence(int iteration, bool verbose);

    /// Calculate error weights based on the given state and tolerances.
    void CalcErrorWeights(const ChVectorDynamic<>& x, double rtol, double atol, ChVectorDynamic<>& ewt);

    JacobianUpdate jacobian_update_method;  ///< Jacobian update strategy
    bool call_setup;                        ///< should the solver's Setup function be called?

    unsigned int maxiters;  ///< maximum number of iterations
    double reltol;          ///< relative tolerance
    double abstolS;         ///< absolute tolerance (states)
    double abstolL;         ///< absolute tolerance (Lagrange multipliers)

    unsigned int numiters;   ///< number of iterations
    unsigned int numsetups;  ///< number of calls to the solver's Setup function
    unsigned int numsolves;  ///< number of calls to the solver's Solve function

    double convergence_rate;  ///< estimated Newton rate of convergence

    ChStateDelta Ds;       ///< state update
    ChVectorDynamic<> Dl;  ///< Lagrange multiplier update
    ChVectorDynamic<> R;   ///< residual of nonlinear system (dynamics portion)
    ChVectorDynamic<> Qc;  ///< residual of nonlinear system (constranints portion)

    std::array<double, 3> Ds_nrm_hist;  ///< last 3 update norms
    std::array<double, 3> Dl_nrm_hist;  ///< last 3 update norms

    ChVectorDynamic<> ewtS;  ///< vector of error weights (states)
    ChVectorDynamic<> ewtL;  ///< vector of error weights (Lagrange multipliers)
};

/// Performs a step of Euler implicit for II order systems.
class ChApi ChTimestepperEulerImplicit : public ChTimestepperIIorder, public ChTimestepperImplicitIterative {
  public:
    ChTimestepperEulerImplicit(ChIntegrableIIorder* intgr = nullptr)
        : ChTimestepperIIorder(intgr), ChTimestepperImplicitIterative() {}

    virtual Type GetType() const override { return Type::EULER_IMPLICIT; }

    /// Performs an integration timestep.
    virtual void Advance(double dt) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;

  protected:
    ChState Xnew;
    ChStateDelta Vnew;
};

/// Performs a step of Euler implicit for II order systems using the Anitescu/Stewart/Trinkle
/// single-iteration method, that is a bit like an implicit Euler where one performs only the
/// first Newton corrector iteration.
/// If using an underlying CCP complementarity solver, this is the typical Anitescu stabilized
/// timestepper for DVIs.
class ChApi ChTimestepperEulerImplicitLinearized : public ChTimestepperIIorder, public ChTimestepperImplicit {
  public:
    ChTimestepperEulerImplicitLinearized(ChIntegrableIIorder* intgr = nullptr)
        : ChTimestepperIIorder(intgr), ChTimestepperImplicit() {}

    virtual Type GetType() const override { return Type::EULER_IMPLICIT_LINEARIZED; }

    /// Performs an integration timestep.
    virtual void Advance(double dt) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;

  protected:
    ChStateDelta Vold;
    ChVectorDynamic<> Dl;
    ChVectorDynamic<> R;
    ChVectorDynamic<> Qc;
};

/// Performs a step of Euler implicit for II order systems using a semi implicit Euler without
/// constraint stabilization, followed by a projection. That is: a speed problem followed by a
/// position problem that keeps constraint drifting 'closed' by using a projection.
/// If using an underlying CCP complementarity solver, this is the typical Tasora stabilized
/// timestepper for DVIs.
class ChApi ChTimestepperEulerImplicitProjected : public ChTimestepperIIorder, public ChTimestepperImplicit {
  public:
    ChTimestepperEulerImplicitProjected(ChIntegrableIIorder* intgr = nullptr)
        : ChTimestepperIIorder(intgr), ChTimestepperImplicit() {}

    virtual Type GetType() const override { return Type::EULER_IMPLICIT_PROJECTED; }

    /// Performs an integration timestep.
    virtual void Advance(double dt) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;

  protected:
    ChStateDelta Vold;
    ChVectorDynamic<> Dl;
    ChVectorDynamic<> R;
    ChVectorDynamic<> Qc;
};

/// Performs a step of trapezoidal implicit for II order systems.
/// NOTE this is a modified version of the trapezoidal for DAE: the original derivation would lead
/// to a scheme that produces oscillatory reactions in constraints, so this is a modified version
/// that is first order in constraint reactions. Use damped HHT or damped Newmark for more advanced options.
class ChApi ChTimestepperTrapezoidal : public ChTimestepperIIorder, public ChTimestepperImplicitIterative {
  public:
    ChTimestepperTrapezoidal(ChIntegrableIIorder* intgr = nullptr)
        : ChTimestepperIIorder(intgr), ChTimestepperImplicitIterative() {}

    virtual Type GetType() const override { return Type::TRAPEZOIDAL; }

    /// Performs an integration timestep.
    virtual void Advance(double dt) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;

  protected:
    ChState Xnew;
    ChStateDelta Vnew;
    ChVectorDynamic<> Rold;
};

/// Performs a step of trapezoidal implicit linearized for II order systems.
class ChApi ChTimestepperTrapezoidalLinearized : public ChTimestepperIIorder, public ChTimestepperImplicitIterative {
  public:
    ChTimestepperTrapezoidalLinearized(ChIntegrableIIorder* intgr = nullptr)
        : ChTimestepperIIorder(intgr), ChTimestepperImplicitIterative() {}

    virtual Type GetType() const override { return Type::TRAPEZOIDAL_LINEARIZED; }

    /// Performs an integration timestep.
    virtual void Advance(double dt) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;

  protected:
    ChState Xnew;
    ChStateDelta Vnew;
    ChVectorDynamic<> Rold;
};

/// Performs a step of Newmark constrained implicit for II order DAE systems.
class ChApi ChTimestepperNewmark : public ChTimestepperIIorder, public ChTimestepperImplicitIterative {
  public:
    ChTimestepperNewmark(ChIntegrableIIorder* intgr = nullptr)
        : ChTimestepperIIorder(intgr), ChTimestepperImplicitIterative() {
        SetGammaBeta(0.6, 0.3);  // default values with some damping, and that works also with DAE constraints
        modified_Newton = true;  // default use modified Newton with jacobian factorization only at beginning
    }

    virtual Type GetType() const override { return Type::NEWMARK; }

    /// Set the numerical damping parameter gamma and the beta parameter.
    /// Gamma: in the [1/2, 1] interval.
    /// For gamma = 1/2, no numerical damping
    /// For gamma > 1/2, more damping
    /// Beta: in the [0, 1] interval.
    /// For beta = 1/4, gamma = 1/2 -> constant acceleration method
    /// For beta = 1/6, gamma = 1/2 -> linear acceleration method
    /// Method is second order accurate only for gamma = 1/2
    void SetGammaBeta(double mgamma, double mbeta);

    double GetGamma() { return gamma; }

    double GetBeta() { return beta; }

    /// Enable/disable modified Newton.
    /// If enabled, the Newton matrix is evaluated, assembled, and factorized only once per step.
    /// If disabled, the Newton matrix is evaluated at every iteration of the nonlinear solver.
    /// Modified Newton iteration is enabled by default.
    void SetModifiedNewton(bool val) { modified_Newton = val; }

    /// Performs an integration timestep.
    virtual void Advance(double dt) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;

  private:
    double gamma;
    double beta;
    ChState Xnew;
    ChStateDelta Vnew;
    ChStateDelta Anew;
    ChVectorDynamic<> Rold;
    bool modified_Newton;
};

/// @} chrono_timestepper

}  // end namespace chrono

#endif
