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

#ifndef CHASSEMBLYANALYSIS_H
#define CHASSEMBLYANALYSIS_H

#include "chrono/core/ChApiCE.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/timestepper/ChIntegrable.h"

namespace chrono {

/// Enumerations for assembly analysis types.
namespace AssemblyAnalysis {
enum Level {
    POSITION = 1 << 0,      ///< satisfy constraints at position level
    VELOCITY = 1 << 1,      ///< satisfy constraints at velocity level
    ACCELERATION = 1 << 2,  ///< satisfy constraints at acceleration level
    FULL = 0xFFFF           ///< full assembly (position + velocity + acceleration)
};

enum class ExitFlag {
    NOT_CONVERGED,    ///< iterations did not converge
    SUCCESS,          ///< no iterations have been performed, no error during velocity and/or acceleration assembly
    ABSTOL_RESIDUAL,  ///< iterations stopped because residual norm below threshold
    RELTOL_UPDATE,    ///< iterations stopped because relative update (Dx/X) norm below threshold
    ABSTOL_UPDATE     ///< iterations stopped because update norm below threshold
};
}  // namespace AssemblyAnalysis

/// Class for assembly analysis.
/// Assembly is performed by satisfying constraints at a position, velocity, and acceleration levels.
/// Assembly at position level involves solving a non-linear problem. Assembly at velocity level is
/// performed by taking a small integration step. Consistent accelerations are obtained through
/// finite differencing.
class ChApi ChAssemblyAnalysis {
  public:
    ChAssemblyAnalysis(ChIntegrableIIorder& mintegrable);

    ~ChAssemblyAnalysis() {}

    /// Perform the assembly analysis.
    /// Assembly is performed by satisfying constraints at position, velocity, and acceleration levels.
    /// Assembly at position level involves solving a non-linear problem.
    /// Assembly at velocity level is performed by taking a small integration step.
    /// Consistent accelerations are obtained through finite differencing.
    AssemblyAnalysis::ExitFlag AssemblyAnalysis(int action, double dt = 1e-7);

    /// Set the max number of Newton-Raphson iterations for the position assembly procedure.
    void SetMaxAssemblyIters(int mi) { m_max_assembly_iters = mi; }

    /// Get the max number of Newton-Raphson iterations for the position assembly procedure.
    int GetMaxAssemblyIters() { return m_max_assembly_iters; }

    /// Set the termination criterion on the infinity norm of the relative state update.
    void SetRelToleranceUpdate(double tol) { m_rel_tol_update = tol; }

    /// Get the termination criterion on the infinity norm of the relative state update.
    double GetRelToleranceUpdate() const { return m_rel_tol_update; }

    /// Set the termination criterion on the infinity norm of the (absolute) state update.
    void SetAbsToleranceUpdate(double tol) { m_abs_tol_update = tol; }

    /// Get the termination criterion on the infinity norm of the (absolute) state update.
    double GetAbsToleranceUpdate() const { return m_abs_tol_update; }

    /// Set the termination criterion on the infinity norm of the residual.
    void SetAbsToleranceResidual(double tol) { m_abs_tol_residual = tol; }

    /// Get the termination criterion on the infinity norm of the residual.
    double GetAbsToleranceResidual() const { return m_abs_tol_residual; }

    /// Get the infinity norm of the last computed residual.
    double GetLastResidualNorm() const { return m_last_residual; }

    /// Get the infinity norm of the last update.
    double GetLastUpdateNorm() const { return m_last_update_norm; }

    /// Get the number of iterations after last assembly.
    unsigned int GetLastIters() const { return m_last_num_iters; }

    /// Get the integrable object.
    ChIntegrable* GetIntegrable() { return integrable; }

    /// Access the Lagrange multipliers.
    const ChVectorDynamic<>& GetLagrangeMultipliers() const { return L; }

    /// Access the current position state vector.
    const ChState& GetStatePos() const { return X; }

    /// Access the current velocity state vector.
    const ChStateDelta& GetStateVel() const { return V; }

    /// Access the current acceleration state vector.
    const ChStateDelta& GetStateAcc() const { return A; }

  private:
    ChIntegrableIIorder* integrable;

    ChState X;
    ChStateDelta V;
    ChStateDelta A;
    ChVectorDynamic<> L;
    unsigned int m_max_assembly_iters = 10;
    double m_rel_tol_update = 1e-6;     ///< termination criterion about relative update infinity norm
    double m_abs_tol_update = 1e-6;     ///< termination criterion about absolute update infinity norm
    double m_abs_tol_residual = 1e-10;  ///< termination criterion about absolute residual infinity norm

    unsigned int m_last_num_iters = 0;
    double m_last_update_norm = 1e-6;  ///< infinity norm of the last computed update
    double m_last_residual = 1e-10;    ///< infinity norm of the residual
};

}  // end namespace chrono

#endif
