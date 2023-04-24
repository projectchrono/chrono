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

#ifndef CHSTATICANALYSIS_H
#define CHSTATICANALYSIS_H

#include "chrono/core/ChApiCE.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/timestepper/ChIntegrable.h"

namespace chrono {

/// Base class for static analysis
class ChApi ChStaticAnalysis {
  public:
    virtual ~ChStaticAnalysis() {}

    /// Get the integrable object.
    ChIntegrable* GetIntegrable() { return m_integrable; }

    /// Access the state, position part, at current analysis.
    const ChState& GetX() const { return X; }

    /// Access the Lagrange multipliers, if any.
    const ChVectorDynamic<>& GetL() const { return L; }

  protected:
    ChStaticAnalysis();

    /// Set associated integrable object.
    void SetIntegrable(ChIntegrableIIorder* integrable);

    /// Performs the static analysis.
    virtual void StaticAnalysis() = 0;

    ChIntegrableIIorder* m_integrable;
    ChState X;
    ChVectorDynamic<> L;

    friend class ChSystem;
};

/// Linear static analysis
class ChApi ChStaticLinearAnalysis : public ChStaticAnalysis {
  public:
    ChStaticLinearAnalysis();
    ~ChStaticLinearAnalysis() {}

  private:
    /// Performs the static analysis, doing a linear solve.
    virtual void StaticAnalysis() override;

    friend class ChSystem;
};

/// Nonlinear static analysis
class ChApi ChStaticNonLinearAnalysis : public ChStaticAnalysis {
  public:
    ChStaticNonLinearAnalysis();
    ~ChStaticNonLinearAnalysis() {}

    /// Enable/disable verbose output (default: false)
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Set the max number of iterations for the Newton Raphson procedure (default: 10).
    void SetMaxIterations(int max_iters);

    /// Set the number of steps that, for the first iterations, make the residual grow linearly (default: 6).
    /// If =0, no incremental application of residual, so it is a classic Newton Raphson iteration, otherwise acts as a
    /// continuation strategy. For values > 0 , it might help convergence. Must be less than maxiters.
    void SetIncrementalSteps(int incr_steps);

    /// Set stopping criteria based on WRMS norm of correction and the specified relative and absolute tolerances.
    /// This is the default, with reltol = 1e-4, abstol = 1e-8.
    /// The Newton Raphson procedure is stopped if the WRMS norm of the correction vector (based on the current state)
    /// is less than 1.
    void SetCorrectionTolerance(double reltol, double abstol);

    /// Set stopping criteria based on norm of residual and the specified tolerance.
    /// The Newton Raphson is stopped when the infinity norm of the residual is below the tolerance.
    void SetResidualTolerance(double tol);

    /// Get the max number of iterations for the Newton Raphson procedure.
    int GetMaxIterations() const { return m_maxiters; }

    /// Set the number of steps that, for the first iterations, make the residual grow linearly.
    int GetIncrementalSteps() const { return m_incremental_steps; }

  private:
    /// Performs the static analysis, doing a non-linear solve.
    virtual void StaticAnalysis() override;

    bool m_verbose;
    int m_maxiters;
    int m_incremental_steps;
    bool m_use_correction_test;
    double m_reltol;
    double m_abstol;

    friend class ChSystem;
};

/// Nonlinear static analysis for a mechanism that is rotating/moving in steady state.
/// If SetAutomaticSpeedAndAccelerationComputation(true), the rotation/movement, if any, is automaticlally assigned via
/// rheonomic constraints. Consider for example, an elastic turbine blade that rotates via a ChLinkMotorRotationAngle:
/// the motor will impose a steady state rotation that creates centrifugal forces, so the  analysis will generate an
/// elongated blade (the ChStaticNonLinearAnalysis cannot capture centrifugal and gyroscopical effects because it resets
/// all speeds/acceleration to zero during its iteration).
/// *WARNING*: at the moment this is ok only for undeformed finite elements! If automatic computation of speed and
/// acceleration is set to false, it is up to the user to provide an iteration callback that keeps the speeds and
/// accelerations updated as needed.
class ChApi ChStaticNonLinearRheonomicAnalysis : public ChStaticAnalysis {
  public:
    ChStaticNonLinearRheonomicAnalysis();
    ~ChStaticNonLinearRheonomicAnalysis() {}

    /// Enable/disable verbose output (default: false)
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Set the max number of iterations for the Newton Raphson procedure (default: 10).
    void SetMaxIterations(int max_iters);

    /// Set the number of steps that, for the first iterations, make the residual grow linearly (default: 6).
    /// If =0, no incremental application of residual, so it is a classic Newton Raphson iteration, otherwise acts as a
    /// continuation strategy. For values > 0 , it might help convergence. Must be less than maxiters.
    void SetIncrementalSteps(int incr_steps);

    /// Set stopping criteria based on WRMS norm of correction and the specified relative and absolute tolerances.
    /// This is the default, with reltol = 1e-4, abstol = 1e-8.
    /// The Newton Raphson procedure is stopped if the WRMS norm of the correction vector (based on the current state)
    /// is less than 1.
    void SetCorrectionTolerance(double reltol, double abstol);

    /// Set stopping criteria based on norm of residual and the specified tolerance.
    /// The Newton Raphson is stopped when the infinity norm of the residual is below the tolerance.
    void SetResidualTolerance(double tol);

    /// Get the max number of iterations for the Newton Raphson procedure.
    int GetMaxIterations() const { return m_maxiters; }

    /// Set the number of steps that, for the first iterations, make the residual grow linearly.
    int GetIncrementalSteps() const { return m_incremental_steps; }

    /// Class to be used as a callback interface for updating the system at each iteration, for example
    /// for incrementing the load or for updating the speeds and accelerations of the parts.
    class ChApi IterationCallback {
      public:
        virtual ~IterationCallback() {}

        /// Perform updates on the model. This is called before each iteration. Must be implemented by child class.
        virtual void OnIterationBegin(
            const double load_scaling,  ///< ranging from 0 to 1, if Newton loop does continuation, could be optionally
                                        ///< used. Otherwise asssume 1=tot load.
            const int iteration_n,      ///< actual number of iteration
            ChStaticNonLinearRheonomicAnalysis* analysis  ///< back-pointer to this analysis
            ) = 0;
    };

    ///  Set the callback to be called at each iteration.
    void SetCallbackIterationBegin(std::shared_ptr<IterationCallback> my_callback) {
        this->callback_iteration_begin = my_callback;
    }

    /// If enabled, the algorithm automatically computes the speed and accelerations of the system by using
    /// the rheonomic constraints. ***WARNING!!!*** this is working only for non-stretched finite elements at the
    /// moment! If not enabled, as by default, you can use an IterationCallback to update the speeds and accelerations
    /// using your c++ code.
    void SetAutomaticSpeedAndAccelerationComputation(bool mv) { this->automatic_speed_accel_computation = mv; }

  private:
    /// Performs the static analysis, doing a non-linear solve.
    virtual void StaticAnalysis() override;

    bool automatic_speed_accel_computation;
    bool m_verbose;
    int m_maxiters;
    int m_incremental_steps;
    bool m_use_correction_test;
    double m_reltol;
    double m_abstol;
    std::shared_ptr<IterationCallback> callback_iteration_begin;

    friend class ChSystem;
};

/// Nonlinear static analysis where the user can define external load(s) that  will be incremented gradually during the
/// solution process. This improves the convergence respect to ChStaticNonlinear, where all the loads (both internal and
/// external) are automatically scaled with a simplified prcedure. It is based on an outer iteration (incrementing
/// external load) and inner iteration (for Newton iteration). A callback will be invoked when the loads must be scaled.
class ChApi ChStaticNonLinearIncremental : public ChStaticAnalysis {
  public:
    ChStaticNonLinearIncremental();
    ~ChStaticNonLinearIncremental() {}

    /// Enable/disable verbose output (default: false)
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Set the max number of inner iterations for the Newton Raphson procedure (default: 5),
    /// where these iterations are preformed at each external load scaling step.
    void SetMaxIterationsNewton(int max_newton_iters);

    /// Set the number of outer iterations that will increment the external load in stepwise manner. (default: 6)
    /// If =1 it uses immediately the final external load, so it boils down to a classic Newton Raphson iteration.
    void SetIncrementalSteps(int incr_steps);

    /// Set stopping criteria based on WRMS norm of correction and the specified relative and absolute tolerances.
    /// This is the default, with reltol = 1e-4, abstol = 1e-8.
    /// The Newton Raphson procedure is stopped if the WRMS norm of the correction vector (based on the current state)
    /// is less than 1.
    void SetCorrectionTolerance(double reltol, double abstol);

    /// Set stopping criteria based on norm of residual and the specified tolerance.
    /// The Newton Raphson is stopped when the infinity norm of the residual is below the tolerance.
    void SetResidualTolerance(double tol);

    /// Get the max number of iterations for the Newton Raphson procedure.
    int GetMaxIterations() const { return max_newton_iters; }

    /// Set the number of steps for the outer iteration (the one that makes the external load grow).
    int GetIncrementalSteps() const { return m_incremental_steps; }

    /// Enable the adaptive size in the inner Newton loop.
    /// If the residual grows more than "growth_tolerance" during the Newton iteration,
    /// step is cancelled and half step is halved, until condition is met.
    /// It can mitigate problems of divergence, but reducing the steps can lead to slow
    /// performance where in some lucky cases one could have just tolerated zigzag non-monotonic
    /// residuals (yet converging in the long run).
    /// Parameter growth_tolerance is 1.0 by default. Sometimes it could be >1 to tolerate also small oscillations.
    /// If this is not working, try also increasing the steps of the outer loop (incremental steps) and/or use Newton
    /// damping.
    void SetAdaptiveNewtonON(
        int initial_delay,  ///< adaptive step policy applied only after n-th Newton steps (sometimes better 1 than 0)
        double growth_tolerance  ///< shrink step only if new_residual bigger than growth_tolerance * old_residual.
    );
    void SetAdaptiveNewtonOFF();

    /// Set damping of the Newton iteration. Default is 1.0.
    /// Using lower values (ex. 0.7) slows the convergence but can fix issues, for example when you see that the
    /// residual has a highly oscillatory behavior.
    void SetNewtonDamping(double damping_factor  ///< default is 1.0 (regular undamped Newton).
    );

    /// Class to be used as a callback interface for updating the system at each step of load increment.
    /// If the user defined loads via ChLoad objects, for example, or via mynode->SetForce(), then in this
    /// callback all these external loads must be updated as final load multiplied by "load_scaling".
    class ChApi LoadIncrementCallback {
      public:
        virtual ~LoadIncrementCallback() {}

        /// Perform updates on the model. This is called before each load scaling. Must be implemented by child class.
        virtual void OnLoadScaling(const double load_scaling,              ///< ranging from 0 to 1
                                   const int iteration_n,                  ///< actual number of outer iteration
                                   ChStaticNonLinearIncremental* analysis  ///< back-pointer to this analysis
                                   ) = 0;
    };

    ///  Set the callback to be called at each iteration.
    void SetLoadIncrementCallback(std::shared_ptr<LoadIncrementCallback> my_callback) {
        this->load_increment_callback = my_callback;
    }

  private:
    /// Performs the static analysis, doing a non-linear solve.
    virtual void StaticAnalysis() override;

    bool m_verbose;
    int max_newton_iters;
    int m_incremental_steps;
    bool m_use_correction_test;
    bool m_adaptive_newton;
    float m_adaptive_newton_tolerance;
    int m_adaptive_newton_delay;
    double m_newton_damping_factor;
    double m_reltol;
    double m_abstol;
    std::shared_ptr<LoadIncrementCallback> load_increment_callback;

    friend class ChSystem;
};



/// Nonlinear static analysis for the system including rigid motion DOFs,
/// for instance, a single pendulum.
class ChApi ChStaticNonLinearRigidMotion : public ChStaticAnalysis {
  public:
    ChStaticNonLinearRigidMotion();
    ~ChStaticNonLinearRigidMotion() {}

    /// Enable/disable verbose output (default: false)
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Set the max number of iterations for the Newton Raphson procedure (default: 10).
    void SetMaxIterations(int max_iters);

    /// Set the number of steps that, for the first iterations, make the residual grow linearly (default: 6).
    /// If =0, no incremental application of residual, so it is a classic Newton Raphson iteration, otherwise acts as a
    /// continuation strategy. For values > 0 , it might help convergence. Must be less than maxiters.
    void SetIncrementalSteps(int incr_steps);

    /// Set stopping criteria based on WRMS norm of correction and the specified relative and absolute tolerances.
    /// This is the default, with reltol = 1e-4, abstol = 1e-8.
    /// The Newton Raphson procedure is stopped if the WRMS norm of the correction vector (based on the current state)
    /// is less than 1.
    void SetCorrectionTolerance(double reltol, double abstol);

    /// Set stopping criteria based on norm of residual and the specified tolerance.
    /// The Newton Raphson is stopped when the infinity norm of the residual is below the tolerance.
    void SetResidualTolerance(double tol);

    /// Get the max number of iterations for the Newton Raphson procedure.
    int GetMaxIterations() const { return m_maxiters; }

    /// Set the number of steps that, for the first iterations, make the residual grow linearly.
    int GetIncrementalSteps() const { return m_incremental_steps; }

  private:
    /// Performs the static analysis, doing a non-linear solve.
    virtual void StaticAnalysis() override;

    bool m_verbose;
    int m_maxiters;
    int m_incremental_steps;
    bool m_use_correction_test;
    double m_reltol;
    double m_abstol;

    friend class ChSystem;
};



}  // end namespace chrono

#endif
