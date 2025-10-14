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

#ifndef CH_TIMESTEPPER_HHT_H
#define CH_TIMESTEPPER_HHT_H

#include "chrono/timestepper/ChTimestepperImplicit.h"

namespace chrono {

/// @addtogroup chrono_timestepper
/// @{

/// Implementation of the HHT implicit integrator for II order systems.
/// This timestepper allows use of an adaptive time-step, as well as optional use of a modified
/// Newton scheme for the solution of the resulting nonlinear problem.
class ChApi ChTimestepperHHT : public ChTimestepperIIorder, public ChTimestepperImplicit {
  public:
    ChTimestepperHHT(ChIntegrableIIorder* intgr = nullptr);

    /// Return type of the integration method.
    virtual ChTimestepper::Type GetType() const override { return ChTimestepper::Type::HHT; }

    /// Return the associated integrable object.
    virtual ChIntegrable* GetIntegrable() const override { return integrable; }

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

    /// Perform an integration step.
    virtual void OnAdvance(double dt) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;

  private:
    void Prepare();
    void Increment();

    double alpha;  ///< HHT method parameter:  -1/3 <= alpha <= 0
    double gamma;  ///< HHT method parameter:   gamma = 1/2 - alpha
    double beta;   ///< HHT method parameter:   beta = (1 - alpha)^2 / 4

    ChState Xnew;            ///< current estimate of new positions
    ChStateDelta Vnew;       ///< current estimate of new velocities
    ChStateDelta Anew;       ///< current estimate of new accelerations
    ChVectorDynamic<> Lnew;  ///< current estimate of Lagrange multipliers
    ChVectorDynamic<> Rold;  ///< residual terms depending on previous state

    bool step_control;                  ///< step size control enabled?
    unsigned int maxiters_success;      ///< maximum number of NR iterations to declare a step successful
    unsigned int req_successful_steps;  ///< required number of successive successful steps for a stepsize increase
    double step_increase_factor;        ///< factor used in increasing stepsize (>1)
    double step_decrease_factor;        ///< factor used in decreasing stepsize (<1)
    double h_min;                       ///< minimum allowable stepsize
    double h;                           ///< internal stepsize
    unsigned int num_successful_steps;  ///< number of successful steps
};

/// @} chrono_timestepper

}  // end namespace chrono

#endif
