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

/// Base class for implicit integrators.
/// Such integrators require solution of a nonlinear problem, solved using an iterative Newton process which requires
/// the solution of a linear system at each iteration. A full Newton method uses a current matrix (based on the system
/// Jacobian), updated and factorized at each iteration. Modified Newton methods use possibly out-of-date Jacobian
/// information; the Jacobian can be evaluated at the beginning of the step only, kept constant for the duration of the
/// entire simulation, or a Jacobian re-evaluation can be triggered automatically and adaptively, as necessary.
class ChApi ChTimestepperImplicit : public ChTimestepper {
  public:
    /// Newton Jacobian update strategies.
    enum class JacobianUpdate {
        EVERY_ITERATION,  ///< Full Newton: Jacobian updated at every iteration
        EVERY_STEP,       ///< Jacobian updated at every step
        NEVER,            ///< Jacobian never updated
        AUTOMATIC         ///< Automatic Jacobian update
    };

    virtual ~ChTimestepperImplicit() {}

    /// Set the max number of Newton iterations.
    void SetMaxIters(int iters) { max_iters = iters; }

    /// Set the relative tolerance.
    /// This tolerance is used in the Newton convergence test (optionally used by derived integrator classes).
    void SetRelTolerance(double rel_tol) { reltol = rel_tol; }

    /// Set the absolute tolerances.
    /// These tolerances are used in the Newton convergence test (optionally used by derived integrator classes).
    /// This version sets separate absolute tolerances for states and Lagrange multipliers.
    void SetAbsTolerances(double abs_tolS, double abs_tolL) {
        abstolS = abs_tolS;
        abstolL = abs_tolL;
    }

    /// Set the absolute tolerances.
    /// These tolerances are used in the Newton convergence test (optionally used by derived integrator classes).
    ///  This version sets equal absolute tolerances for states and Lagrange multipliers.
    void SetAbsTolerances(double abs_tol) {
        abstolS = abs_tol;
        abstolL = abs_tol;
    }

    /// Set the strategy for Jacobian update (default: EVERY_STEP).
    void SetJacobianUpdateMethod(JacobianUpdate method);

    /// Get the max number of iterations using the Newton Raphson procedure.
    double GetMaxIters() const { return max_iters; }

    /// Get the current Jacobian update startegy.
    JacobianUpdate GetJacobianUpdateMethod() const { return jacobian_update_method; }

    /// Return Jacobian update method as a string.
    static std::string GetJacobianUpdateMethodAsString(JacobianUpdate jacobian_update);

    /// Return the number of Newton iterations over the last step.
    unsigned int GetNumStepIterations() const { return num_step_iters; }

    /// Return the number of calls to the solver's Setup function made during the last step.
    unsigned int GetNumStepSetupCalls() const { return num_step_setups; }

    /// Return the number of calls to the solver's Solve function made over the last step.
    unsigned int GetNumStepSolveCalls() const { return num_step_solves; }

    /// Get the last estimated convergence rate for the internal Newton solver.
    /// Note that an estimate can only be calculated after the 3rd iteration. For the first 2 iterations, the
    /// convergence rate estimate is set to 1.
    double GetEstimatedConvergenceRate() const { return convergence_rate; }

    /// Return the cumulative number of Newton iterations.
    unsigned int GetNumIterations() const { return num_iters; }

    /// Return the cummulative number of calls to the solver's Setup function.
    unsigned int GetNumSetupCalls() const { return num_setups; }

    /// Return the cumulative number of calls to the solver's Solve function.
    unsigned int GetNumSolveCalls() const { return num_solves; }

    /// Accept step after a non-converged Newton solve (default: true).
    /// If 'true', the solution at the end of a step is accepted as-is, even if the Newton nonlinear solve did not converge (even with an up-to-date Jacobian).
    /// If 'false', an exception is thrown if the Newton iterations do not converge after the allowed maximum number of iterations.
    void AcceptTerminatedStep(bool accept) { accept_terminated = accept; }

    /// Return the number of terminated Newton solves.
    /// These are Newton solves that were terminated after the maximum number of iterations, without achieving
    /// convergence. The Jacobian is either up-to-date or cannot be re-evaluated (because the Jacobian update strategy
    /// is set to JacobianUpdate::NEVER) and step size cannot be reduced (because step-size control is disabled).
    /// The nonlinear system solution is accepted as-is.
    unsigned int GetNumTerminated() const { return num_terminated; }

  public:
    // Functions for adaptive step size control

    /// Turn on/off the internal step size control.
    /// Default: true.
    void SetStepControl(bool enable) { step_control = enable; }

    /// Set the minimum step size.
    /// An exception is thrown if the internal step size decreases below this limit.
    /// Default: 1e-10.
    void SetMinStepSize(double step) { h_min = step; }

    /// Set the maximum allowable number of iterations for counting a step towards a stepsize increase.
    /// Default: 3.
    void SetMaxItersSuccess(int iters) { maxiters_success = iters; }

    /// Set the minimum number of (internal) steps that use at most maxiters_success
    /// before considering a stepsize increase.
    /// Default: 5.
    void SetRequiredSuccessfulSteps(int num_steps) { req_successful_steps = num_steps; }

    /// Set the multiplicative factor for a stepsize increase (must be larger than 1).
    /// Default: 2.
    void SetStepIncreaseFactor(double factor) { step_increase_factor = factor; }

    /// Set the multiplicative factor for a stepsize decrease (must be smaller than 1).
    /// Default: 0.5.
    void SetStepDecreaseFactor(double factor) { step_decrease_factor = factor; }

  public:
    /// Perform an integration step.
    /// This base class manages the Newton iteration counters and defers the integrator implementation to OnAdvance.
    virtual void Advance(double dt) override final;

  public:
    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override {
        // version number
        archive.VersionWrite(1);
        // serialize all member data:
        archive << CHNVP(max_iters);
        archive << CHNVP(reltol);
        archive << CHNVP(abstolS);
        archive << CHNVP(abstolL);
    }

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override {
        // version number
        /*int version =*/archive.VersionRead();
        // stream in all member data:
        archive >> CHNVP(max_iters);
        archive >> CHNVP(reltol);
        archive >> CHNVP(abstolS);
        archive >> CHNVP(abstolL);
    }

  protected:
    ChTimestepperImplicit();

    /// Implementation of integrator-specific time advance.
    /// This base class implementation provides support for an adaptive time-step, error-controlled integrator.
    /// A derived class using these features, must implement the functions . Otherwise, it must override OnAdvance.
    virtual void OnAdvance(double dt);

    /// Initialize integrator at beginning of a new step.
    /// Used only if no override of OnAdvance is provided.
    virtual void InitializeStep() {}

    /// Prepare integrator for attempting a new step.
    /// Used only if no override of OnAdvance is provided.
    virtual void PrepareStep() {}

    /// Calculate new state increment for a Newton iteration.
    /// Used only if no override of OnAdvance is provided.
    virtual void Increment() {}

    /// Reset step data for re-attempting step (with new Jacobian or reduced step size).
    /// Used only if no override of OnAdvance is provided.
    virtual void ResetStep() {}

    /// Accept attempted step (if Newton converged or was terminated).
    /// Used only if no override of OnAdvance is provided.
    virtual void AcceptStep() {}

    /// Finalize step and update solution at end of step.
    /// Used only if no override of OnAdvance is provided.
    virtual void FinalizeStep() {}

    /// Check convergence of Newton process.
    bool CheckConvergence(int iteration);

    /// Calculate error weights based on the given state and tolerances.
    void CalcErrorWeights(const ChVectorDynamic<>& x, double rtol, double atol, ChVectorDynamic<>& ewt);

    JacobianUpdate jacobian_update_method;  ///< Jacobian update strategy
    bool call_setup;                        ///< should the solver's Setup function be called?
    bool call_analyze;                      ///< should the solver's Setup analyze phase be called?
    bool jacobian_is_current;               ///< was the Jacobian evaluated at current Newton iteration?

    unsigned int max_iters;  ///< maximum number of iterations
    double reltol;           ///< relative tolerance
    double abstolS;          ///< absolute tolerance (states)
    double abstolL;          ///< absolute tolerance (Lagrange multipliers)

    unsigned int num_step_iters;   ///< number of iterations during last step
    unsigned int num_step_setups;  ///< number of calls to the solver Setup() function during last step
    unsigned int num_step_solves;  ///< number of calls to the solver Solve() function during last step

    unsigned int num_iters;   ///< current cumulative number of Newton iterations
    unsigned int num_setups;  ///< current cumulative number of Setup() calls
    unsigned int num_solves;  ///< current cummulative number of Solve() calls

    unsigned int num_terminated;  ///< number of terminated NEwton iterations

    double convergence_rate;  ///< estimated Newton rate of convergence

    ChStateDelta Ds;       ///< state update
    ChVectorDynamic<> Dl;  ///< Lagrange multiplier update
    ChVectorDynamic<> R;   ///< residual of nonlinear system (dynamics portion)
    ChVectorDynamic<> Qc;  ///< residual of nonlinear system (constraints portion)

    std::array<double, 3> Ds_nrm_hist;  ///< last 3 update norms
    std::array<double, 3> Dl_nrm_hist;  ///< last 3 update norms

    ChVectorDynamic<> ewtS;  ///< vector of error weights (states)
    ChVectorDynamic<> ewtL;  ///< vector of error weights (Lagrange multipliers)

    bool accept_terminated;  ///< accept or reject steps after a non-converged Newton solve

    bool step_control;                  ///< step size control enabled?
    unsigned int maxiters_success;      ///< maximum number of NR iterations to declare a step successful
    unsigned int req_successful_steps;  ///< required number of successive successful steps for a stepsize increase
    double step_increase_factor;        ///< factor used in increasing stepsize (>1)
    double step_decrease_factor;        ///< factor used in decreasing stepsize (<1)
    double h_min;                       ///< minimum allowable stepsize
    double h;                           ///< internal stepsize
    unsigned int num_successful_steps;  ///< number of successful steps
};

/// Euler implicit for II order systems.
class ChApi ChTimestepperEulerImplicit : public ChTimestepperIIorder, public ChTimestepperImplicit {
  public:
    ChTimestepperEulerImplicit(ChIntegrableIIorder* intgr = nullptr);

    virtual Type GetType() const override { return Type::EULER_IMPLICIT; }

    /// Return the associated integrable object.
    virtual ChIntegrable* GetIntegrable() const override { return integrable; }

    /// Perform an integration step.
    virtual void OnAdvance(double dt) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;

  protected:
    ChState Xnew;
    ChStateDelta Vnew;
};

/// Euler implicit for II order systems using the Anitescu/Stewart/Trinkle single-iteration method.
/// This is similar to an implicit Euler where one performs only the first Newton corrector iteration.
/// If using an underlying CCP complementarity solver, this is the typical Anitescu stabilized timestepper for DVIs.
class ChApi ChTimestepperEulerImplicitLinearized : public ChTimestepperIIorder, public ChTimestepperImplicit {
  public:
    ChTimestepperEulerImplicitLinearized(ChIntegrableIIorder* intgr = nullptr);

    virtual Type GetType() const override { return Type::EULER_IMPLICIT_LINEARIZED; }

    /// Return the associated integrable object.
    virtual ChIntegrable* GetIntegrable() const override { return integrable; }

    /// Perform an integration step.
    virtual void OnAdvance(double dt) override;

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

/// Euler implicit for II order systems, with projection.
/// This method uses a semi-implicit Euler scheme without constraint stabilization, followed by a projection. In other
/// words, a speed problem followed by a position problem that keeps constraint drifting 'closed' by using a projection.
/// If using an underlying CCP complementarity solver, this is the typical Tasora stabilized timestepper for DVIs.
class ChApi ChTimestepperEulerImplicitProjected : public ChTimestepperIIorder, public ChTimestepperImplicit {
  public:
    ChTimestepperEulerImplicitProjected(ChIntegrableIIorder* intgr = nullptr);

    virtual Type GetType() const override { return Type::EULER_IMPLICIT_PROJECTED; }

    /// Return the associated integrable object.
    virtual ChIntegrable* GetIntegrable() const override { return integrable; }

    /// Perform an integration step.
    virtual void OnAdvance(double dt) override;

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

/// Trapezoidal implicit for II order systems.
/// NOTE: this is a modified version of the trapezoidal for DAE: the original derivation would lead to a scheme that
/// produces oscillatory reactions in constraints, so this is a modified version that is first order in constraint
/// reactions. Use damped HHT or damped Newmark for more advanced options.
class ChApi ChTimestepperTrapezoidal : public ChTimestepperIIorder, public ChTimestepperImplicit {
  public:
    ChTimestepperTrapezoidal(ChIntegrableIIorder* intgr = nullptr);

    virtual Type GetType() const override { return Type::TRAPEZOIDAL; }

    /// Return the associated integrable object.
    virtual ChIntegrable* GetIntegrable() const override { return integrable; }

    /// Perform an integration step.
    virtual void OnAdvance(double dt) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;

  protected:
    ChState Xnew;
    ChStateDelta Vnew;
    ChVectorDynamic<> Rold;
};

/// Trapezoidal implicit linearized for II order systems.
class ChApi ChTimestepperTrapezoidalLinearized : public ChTimestepperIIorder, public ChTimestepperImplicit {
  public:
    ChTimestepperTrapezoidalLinearized(ChIntegrableIIorder* intgr = nullptr);

    virtual Type GetType() const override { return Type::TRAPEZOIDAL_LINEARIZED; }

    /// Return the associated integrable object.
    virtual ChIntegrable* GetIntegrable() const override { return integrable; }

    /// Perform an integration step.
    virtual void OnAdvance(double dt) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;

  protected:
    ChState Xnew;
    ChStateDelta Vnew;
    ChVectorDynamic<> Rold;
};

/// Newmark constrained implicit for II order DAE systems.
class ChApi ChTimestepperNewmark : public ChTimestepperIIorder, public ChTimestepperImplicit {
  public:
    ChTimestepperNewmark(ChIntegrableIIorder* intgr = nullptr);

    virtual Type GetType() const override { return Type::NEWMARK; }

    /// Return the associated integrable object.
    virtual ChIntegrable* GetIntegrable() const override { return integrable; }

    /// Set the numerical damping parameter gamma and the beta parameter.
    /// - gamma: in the [1/2, 1] interval.
    ///     gamma = 1/2, no numerical damping
    ///     gamma > 1/2, more damping
    /// - beta: in the [0, 1] interval.
    ///     beta = 1/4, gamma = 1/2 -> constant acceleration method
    ///     beta = 1/6, gamma = 1/2 -> linear acceleration method
    /// Method is second order accurate only for gamma = 1/2.
    void SetGammaBeta(double gamma_val, double beta_val);

    double GetGamma() const { return gamma; }

    double GetBeta() const { return beta; }

    /// Perform an integration step.
    virtual void OnAdvance(double dt) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;

  private:
    double gamma;  ///< Newmark method parameter  0.5 <= gamma <= 1.0
    double beta;   ///< Newmark method parameter  0.0 <= beta  <= 1.0

    ChState Xnew;
    ChStateDelta Vnew;
    ChStateDelta Anew;
    ChVectorDynamic<> Lnew;
    ChVectorDynamic<> Rold;
};

/// @} chrono_timestepper

}  // end namespace chrono

#endif
