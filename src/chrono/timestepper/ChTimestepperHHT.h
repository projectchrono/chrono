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
///
/// The Newton iteration solves for the accelerations and Lagrange multipliers at the end of the step. At each
/// iteration, it evaluates the following quantities:
/// - R: the dynamics residual (generalized forces, inertial forces, and constraint reactions);
/// - C: the constraint violation, at the position level. For constraints imposed at the velocity level, C is the
///   velocity violation multiplied by (beta/gamma)*h;
/// - Ds: the acceleration update, measured in the WRMS norm with weights 1/(reltol*|a| + abstolS),
///   with a the accelerations at the beginning of the step;
/// - Dl: the Lagrange multiplier update, measured in the WRMS norm with weights 1/(reltol*|l| + abstolL),
///   with l the multipliers at the beginning of the step.
///
/// Newton is declared converged if either of the following tests passes:
/// - residual test: |R| < abstolS and |C| < abstolS (2-norms);
/// - update test: |Ds|_WRMS < 1 and |Dl|_WRMS < 1.
///
/// The state tolerance abstolS therefore applies to all state-related quantities, while the multiplier tolerance
/// abstolL applies only to the multiplier update. Both tests are limited by round-off: |C| cannot drop below about
/// eps*|q| (with eps the machine precision and q the generalized coordinates), |R| cannot drop below about
/// k*eps*|q| for a force with stiffness k, and the acceleration update cannot drop below about eps*|q|/(beta*h^2).
/// The last limit grows quickly as the step size decreases; with very small steps, abstolS (or reltol) must stay
/// above it for the update test to pass.
///
/// When Newton fails to converge, the integrator reports the norms and tolerances of both tests at the last
/// iteration (in the exception message, or in the verbose output if the terminated step is accepted).
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

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;

  private:
    // Implementation of virtual methods for an adaptive, error controlled HHT integrator

    /// Initialize integrator at beginning of a new step.
    /// - Set up state vectors
    /// - Set up auxiliary vectors
    /// - Gather state (position and velocity) and auxiliary data (acceleration and multipliers) at beginning of step
    virtual void InitializeStep() override;

    /// Prepare attempting a step of size h (assuming a converged state at the current time t):
    /// - Initialize residual vector with terms at current time
    /// - Obtain a prediction at T+h for Newton using extrapolation from solution at current time.
    /// - If no step size control, start with zero acceleration guess (previous step not guaranteed to have converged)
    /// - Set the error weight vectors (using solution at current time)
    virtual void PrepareStep() override;

    /// Calculate new state increment for a Newton iteration.
    /// - Scatter the current estimate of the new state (the state at time T+h)
    /// - Set up and solve linear system
    /// - Calculate solution increment
    /// - Update the estimate of the new state (the state at time T+h)
    virtual void Increment() override;

    /// Reset step data for re-attempting step (with new Jacobian or reduced step size).
    /// - Scatter state
    /// - Reset auxiliary data (acceleration and residual)
    virtual void ResetStep() override;

    /// Accept attempted step (if Newton converged or was terminated).
    /// Set states and auxiliary data at the end of an intermediate step.
    virtual void AcceptStep() override;

    /// Finalize step and update solution at end of step.
    /// - Scatter state doing a full update.
    /// - Scatter auxiliary data (accelerations and Lagrange multipliers)
    virtual void FinalizeStep() override;

    double alpha;  ///< HHT method parameter:  -1/3 <= alpha <= 0
    double gamma;  ///< HHT method parameter:   gamma = 1/2 - alpha
    double beta;   ///< HHT method parameter:   beta = (1 - alpha)^2 / 4

    ChState Xnew;            ///< current estimate of new positions
    ChStateDelta Vnew;       ///< current estimate of new velocities
    ChStateDelta Anew;       ///< current estimate of new accelerations
    ChVectorDynamic<> Lnew;  ///< current estimate of Lagrange multipliers
    ChVectorDynamic<> Rold;  ///< residual terms depending on previous state
};

/// @} chrono_timestepper

}  // end namespace chrono

#endif
