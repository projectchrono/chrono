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

#ifndef CHTIMESTEPPER_HHT_H
#define CHTIMESTEPPER_HHT_H

#include <array>

#include "chrono/timestepper/ChTimestepper.h"

namespace chrono {

/// @addtogroup chrono_timestepper
/// @{

/// Implementation of the HHT implicit integrator for II order systems.
/// This timestepper allows use of an adaptive time-step, as well as optional use of a modified
/// Newton scheme for the solution of the resulting nonlinear problem.
class ChApi ChTimestepperHHT : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper {
  public:
    ChTimestepperHHT(ChIntegrableIIorder* intgr = nullptr);

    /// Return type of the integration method.
    virtual Type GetType() const override { return Type::HHT; }

    /// Set the numerical damping parameter (in the [-1/3, 0] range).
    /// The closer to -1/3, the more damping.
    /// The closer to 0, the less damping (for 0, it is the trapezoidal method).
    /// The method coefficients gamma and beta are set automatically, based on alpha.
    /// Default: -0.2.
    void SetAlpha(double val);

    /// Return the current value of the method parameter alpha.
    double GetAlpha() { return alpha; }

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

    /// Available styles of modified Newton methods
    enum class JacobianUpdate {
        EVERY_ITERATION, // Classical Newton. Jacobian updated at every iteration
        EVERY_STEP, // Jacobian updated at every step
        NEVER,   // Jacobian never updated
        AUTOMATIC   // Jaobian updated when appropriate. TODO: implement
    };

    /// Enable/disable modified Newton.
    /// If enabled, the Newton matrix is evaluated, assembled, and factorized only once
    /// per step or if the Newton iteration does not converge with an out-of-date matrix.
    /// If disabled, the Newton matrix is evaluated at every iteration of the nonlinear solver.
    /// Default: true.
    void SetModifiedNewton(JacobianUpdate style) {
        modified_Newton = style;
        // Reset setup call for JacobianUpdate::NEVER, e.g. if used after another style during the same simulation
        if (style == JacobianUpdate::NEVER) call_setup_for_NEVER = true;
    }
    void SetModifiedNewton(bool enable) { // Overload for backward compatibility with boolean implementation
        if (enable)
            modified_Newton = JacobianUpdate::EVERY_STEP;
        else
            modified_Newton = JacobianUpdate::EVERY_ITERATION;
    }

    /// Perform an integration timestep, by advancing the state by the specified time step.
    virtual void Advance(const double dt) override;

    /// Get the last estimated convergence rate for the internal Newton solver.
    /// Note that an estimate can only be calculated after the 3rd iteration. For the first 2 iterations, the
    /// convergence rate estimate is set to 1.
    double GetEstimatedConvergenceRate() const { return convergence_rate; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;

  private:
    void Prepare(ChIntegrableIIorder* integrable2);
    void Increment(ChIntegrableIIorder* integrable2);
    bool CheckConvergence(int it);
    void CalcErrorWeights(const ChVectorDynamic<>& x, double rtol, double atol, ChVectorDynamic<>& ewt);
    bool SetupRequiredForIteration(int iteration, bool previous_substep_converged);

  private:
    double alpha;  ///< HHT method parameter:  -1/3 <= alpha <= 0
    double gamma;  ///< HHT method parameter:   gamma = 1/2 - alpha
    double beta;   ///< HHT method parameter:   beta = (1 - alpha)^2 / 4

    ChStateDelta Da;         ///< state update
    ChVectorDynamic<> Dl;    ///< Lagrange multiplier update
    ChState Xnew;            ///< current estimate of new positions
    ChStateDelta Vnew;       ///< current estimate of new velocities
    ChStateDelta Anew;       ///< current estimate of new accelerations
    ChVectorDynamic<> Lnew;  ///< current estimate of Lagrange multipliers
    ChVectorDynamic<> R;     ///< residual of nonlinear system (dynamics portion)
    ChVectorDynamic<> Rold;  ///< residual terms depending on previous state
    ChVectorDynamic<> Qc;    ///< residual of nonlinear system (constraints portion)

    std::array<double, 3> Da_nrm_hist;  ///< last 3 update norms
    std::array<double, 3> Dl_nrm_hist;  ///< last 3 update norms
    double convergence_rate;            ///< estimated Newton rate of convergence

    bool step_control;                  ///< step size control enabled?
    unsigned int maxiters_success;      ///< maximum number of NR iterations to declare a step successful
    unsigned int req_successful_steps;  ///< required number of successive successful steps for a stepsize increase
    double step_increase_factor;        ///< factor used in increasing stepsize (>1)
    double step_decrease_factor;        ///< factor used in decreasing stepsize (<1)
    double h_min;                       ///< minimum allowable stepsize
    double h;                           ///< internal stepsize
    unsigned int num_successful_steps;  ///< number of successful steps

    JacobianUpdate modified_Newton;    ///< style of modified Newton?
    bool matrix_is_current;  ///< is the Newton matrix up-to-date?
    bool call_setup;         ///< should the solver's Setup function be called?
    // TODO: I needed the variable below for the corner case where HHT converges after 1 iteration, and SetupRequiredForIteration()
    //       returned true for JacobianUpdate::NEVER at the following step, which is not the desired behavior!
    //       Determining if style JacobianUpdate::NEVER should call setup, cannot be determined by SetupRequiredForIteration() from the iteration count, previous convergence status, etc, without leaving this corner case.
    //       I could not use existing variables such as numsetups or numiters because those are reset to zero at every call of Advance() and I need something that persists across steps.
    //       I needed a persistent state variable to determine whether JacobianUpdate::NEVER should call setup.
    //       ans that values must be different from call_setup which currently gives the immediate value used by StateSolveCorrection() inside Increment()
    //       One a side note, I think call_setup is not needed as a state variable. It could be made local to Advance() and passed to Increment(), rather than be a private member of HHT.
    //       I did not want to change too many things and refactor this so I simply added a new variable.
    bool call_setup_for_NEVER;

    ChVectorDynamic<> ewtS;  ///< vector of error weights (states)
    ChVectorDynamic<> ewtL;  ///< vector of error weights (Lagrange multipliers)
};

/// @} chrono_timestepper

}  // end namespace chrono

#endif
