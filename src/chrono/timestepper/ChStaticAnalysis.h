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
    ChStaticAnalysis(ChIntegrableIIorder& integrable);

    virtual ~ChStaticAnalysis() {}

    /// Performs the static analysis.
    virtual void StaticAnalysis() = 0;

    /// Get the integrable object.
    ChIntegrable* GetIntegrable() { return m_integrable; }

    /// Access the state, position part, at current analysis.
    const ChState& GetX() const { return X; }

    /// Access the Lagrange multipliers, if any.
    const ChVectorDynamic<>& GetL() const { return L; }

  protected:
    ChIntegrableIIorder* m_integrable;
    ChState X;
    ChVectorDynamic<> L;
};

/// Linear static analysis
class ChApi ChStaticLinearAnalysis : public ChStaticAnalysis {
  public:
    ChStaticLinearAnalysis(ChIntegrableIIorder& integrable);
    ~ChStaticLinearAnalysis() {}

    /// Performs the static analysis, doing a linear solve.
    virtual void StaticAnalysis() override;
};

/// Nonlinear static analysis
class ChApi ChStaticNonLinearAnalysis : public ChStaticAnalysis {
  public:
    ChStaticNonLinearAnalysis(ChIntegrableIIorder& integrable);
    ~ChStaticNonLinearAnalysis() {}

    /// Performs the static analysis, doing a non-linear solve.
    virtual void StaticAnalysis() override;

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
    bool m_verbose;
    int m_maxiters;
    int m_incremental_steps;
    bool m_use_correction_test;
    double m_reltol;
    double m_abstol;
};


/// Nonlinear static analysis for a mechanism that is rotating/moving in steady state.
/// If SetAutomaticSpeedAndAccelerationComputation(true),
///  the rotation/movement, if any, is automaticlally assigned via rheonomic constraints. 
///  Example, an elastic turbine blade that rotates via a ChLinkMotorRotationAngle: the 
///  motor will impose a steady state rotation that creates centrifugal forces, so the
///  analysis will generate an elongated blade (the ChStaticNonLinearAnalysis cannot capture
///  centrifugal and gyroscopical effects because it resets all speeds/acceleration to zero 
///  during its iteration). ***WARNING*** at the moment this is ok only for undeformed finite elements!
/// If SetAutomaticSpeedAndAccelerationComputation(false), it is up to you to provide an iteration callback
///  that keeps the speeds and accelerations updated as needed.

class ChApi ChStaticNonLinearRheonomicAnalysis : public ChStaticAnalysis {
  public:
    ChStaticNonLinearRheonomicAnalysis(ChIntegrableIIorder& integrable);
    ~ChStaticNonLinearRheonomicAnalysis() {}

    /// Performs the static analysis, doing a non-linear solve.
    virtual void StaticAnalysis() override;

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
        virtual void OnIterationBegin(  const double load_scaling, ///< ranging from 0 to 1, if Newton loop does continuation, could be optionally used. Otherwise asssume 1=tot load.
                                        const int iteration_n,  ///< actual number of iteration
                                        ChStaticNonLinearRheonomicAnalysis* analysis     ///< back-pointer to this analysis
                                  ) = 0;
    };

    ///  Set the callback to be called at each iteration.
    void SetCallbackIterationBegin(std::shared_ptr<IterationCallback> my_callback) { this->callback_iteration_begin = my_callback; }

    /// If enabled, the algorithm automatically computes the speed and accelerations of the system by using
    /// the rheonomic constraints. ***WARNING!!!*** this is working only for non-stretched finite elements at the moment!
    /// If not enabled, as by default, you can use an IterationCallback to update the speeds and accelerations using your c++ code. 
    void SetAutomaticSpeedAndAccelerationComputation(bool mv) { this->automatic_speed_accel_computation = mv; }

  private:
    bool automatic_speed_accel_computation;
    bool m_verbose;
    int m_maxiters;
    int m_incremental_steps;
    bool m_use_correction_test;
    double m_reltol;
    double m_abstol;
    std::shared_ptr<IterationCallback> callback_iteration_begin;
};





}  // end namespace chrono

#endif
