// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChTimestepperHHT)

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

  public:
    ChTimestepperHHT(ChIntegrableIIorder* mintegrable = nullptr);

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

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  private:
    void Prepare(ChIntegrableIIorder* integrable, double scaling_factor);
    void Increment(ChIntegrableIIorder* integrable, double scaling_factor);
    bool CheckConvergence(double scaling_factor);
    void CalcErrorWeights(const ChVectorDynamic<>& x, double rtol, double atol, ChVectorDynamic<>& ewt);
};

/// @} chrono_timestepper

}  // end namespace chrono

#endif
