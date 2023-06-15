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

#include "chrono/timestepper/ChTimestepper.h"

namespace chrono {

/// @addtogroup chrono_timestepper
/// @{

/// Implementation of the HHT implicit integrator for II order systems.
/// This timestepper allows use of an adaptive time-step, as well as optional use of a modified
/// Newton scheme for the solution of the resulting nonlinear problem.
class ChApi ChTimestepperHHT : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper {

  public:
    enum HHT_Mode {
        ACCELERATION,
        POSITION,
    };

  private:
    double alpha;   ///< HHT method parameter:  -1/3 <= alpha <= 0
    double gamma;   ///< HHT method parameter:   gamma = 1/2 - alpha
    double beta;    ///< HHT method parameter:   beta = (1 - alpha)^2 / 4
    HHT_Mode mode;  ///< HHT formulation (ACCELERATION or POSITION)
    bool scaling;   ///< include scaling by beta * h * h (POSITION only)

    ChStateDelta Da;         ///< state update
    ChStateDelta Dx;         ///< cummulative state updates (POSITION only)
    ChVectorDynamic<> Dl;    ///< Lagrange multiplier update
    ChState Xnew;            ///< current estimate of new positions
    ChState Xprev;           ///< previous estimate of new positions (POSITION only)
    ChStateDelta Vnew;       ///< current estimate of new velocities
    ChStateDelta Anew;       ///< current estimate of new accelerations
    ChVectorDynamic<> Lnew;  ///< current estimate of Lagrange multipliers
    ChVectorDynamic<> R;     ///< residual of nonlinear system (dynamics portion)
    ChVectorDynamic<> Rold;  ///< residual terms depending on previous state
    ChVectorDynamic<> Qc;    ///< residual of nonlinear system (constranints portion)

    bool step_control;            ///< step size control enabled?
    int maxiters_success;         ///< maximum number of NR iterations to declare a step successful
    int req_successful_steps;     ///< required number of successive successful steps for a stepsize increase
    double step_increase_factor;  ///< factor used in increasing stepsize (>1)
    double step_decrease_factor;  ///< factor used in decreasing stepsize (<1)
    double h_min;                 ///< minimum allowable stepsize
    double h;                     ///< internal stepsize
    int num_successful_steps;     ///< number of successful steps

    bool modified_Newton;    ///< use modified Newton?
    bool matrix_is_current;  ///< is the Newton matrix up-to-date?
    bool call_setup;         ///< should the solver's Setup function be called?

    ChVectorDynamic<> ewtS;  ///< vector of error weights (states)
    ChVectorDynamic<> ewtL;  ///< vector of error weights (Lagrange multipliers)

    bool convergence_trend_flag = true; ///< A flag to indicate the trend of convergence
    double threshold_R = 1e12; ///< A threshold for R to judge the trend of convergence

  public:
    ChTimestepperHHT(ChIntegrableIIorder* intgr = nullptr);

    virtual Type GetType() const override { return Type::HHT; }

    /// Set the numerical damping parameter.
    /// It must be in the [-1/3, 0] interval. The closer to -1/3, the more damping.
    /// The closer to 0, the less damping (for 0, it is the trapezoidal method).
    /// The method coefficients gamma and beta are set automatically, based on alpha.
    void SetAlpha(double malpha);

    /// Return the current value of the method parameter alpha.
    double GetAlpha() { return alpha; }

    /// Set the HHT formulation.
    void SetMode(HHT_Mode mmode) { mode = mmode; }

    /// Turn scaling on/off.
    void SetScaling(bool mscaling) { scaling = mscaling; }

    /// Turn step size control on/off.
    /// Step size control is enabled by default.
    void SetStepControl(bool val) { step_control = val; }

    /// Set the minimum step size.
    /// An exception is thrown if the internal step size decreases below this limit.
    void SetMinStepSize(double min_step) { h_min = min_step; }

    /// Set the maximum allowable number of iterations for counting a step towards a stepsize increase.
    void SetMaxItersSuccess(int iters) { maxiters_success = iters; }

    /// Set the minimum number of (internal) steps that require at most maxiters_success
    /// before considering a stepsize increase.
    void SetRequiredSuccessfulSteps(int num_steps) { req_successful_steps = num_steps; }

    /// Set the multiplicative factor for a stepsize increase.
    /// Must be a value larger than 1.
    void SetStepIncreaseFactor(double factor) { step_increase_factor = factor; }

    /// Set the multiplicative factor for a stepsize decrease.
    /// Must be a value smaller than 1.
    void SetStepDecreaseFactor(double factor) { step_decrease_factor = factor; }

    /// Enable/disable modified Newton.
    /// If enabled, the Newton matrix is evaluated, assembled, and factorized only once
    /// per step or if the Newton iteration does not converge with an out-of-date matrix.
    /// If disabled, the Newton matrix is evaluated at every iteration of the nonlinear solver.
    /// Modified Newton iteration is enabled by default.
    void SetModifiedNewton(bool val) { modified_Newton = val; }

    /// Perform an integration timestep.
    virtual void Advance(const double dt  ///< timestep to advance
                         ) override;

    /// Get an indicator to tell whether the iteration in current step tends to convergence or divergence.
    /// This could be helpful if you want to fall back to iterate again via using more rigorous stepper settings, 
    /// such as smaller fixed time stepper, turning off ModifiedNerton,etc, 
    /// WHEN you are not satisfied by current iteration result.
    bool GetConvergenceFlag() const { return convergence_trend_flag; }

    /// Set the threshold of norm of R, which is used to judge the trend of convergency.
    /// For different systems, this threshold may be much different, such as a mini robot and a huge wind turbine.
    /// You could turn on 'verbose' of HHT stepper and look at the norm of R for your system firstly, 
    /// and then set a suitable threshold of norm of R. 
    /// This threshold should always be larger than the normal norm of R for your system.
    void SetThreshold_R(double mv) { threshold_R = mv; }

    /// Get the threshold of norm of R, which is used to judge the trend of convergency.
    double GetThreshold_R() const { return threshold_R; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;

  private:
    void Prepare(ChIntegrableIIorder* integrable, double scaling_factor);
    void Increment(ChIntegrableIIorder* integrable, double scaling_factor);
    bool CheckConvergence(double scaling_factor);
    void CalcErrorWeights(const ChVectorDynamic<>& x, double rtol, double atol, ChVectorDynamic<>& ewt);
};

/// @} chrono_timestepper

}  // end namespace chrono

#endif
