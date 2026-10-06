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
    ABSTOL_UPDATE,    ///< iterations stopped because update norm below threshold
    ACCELERATION_INACCURATE  ///< assembly done, but the acceleration-level solve was not accurate enough (see
                             ///< ChSystem::SetAssemblyAccelerationTolerance)
};
}  // namespace AssemblyAnalysis

/// Class for assembly analysis.
/// Assembly is performed by satisfying constraints at a position, velocity, and acceleration levels.
/// Assembly at position level involves solving a non-linear problem. Assembly at velocity level is
/// performed by taking a small linearized integration step. Consistent accelerations and reactions are
/// obtained from a second such step whose constraint right-hand side carries the quadratic-velocity
/// term, so that the acceleration-level constraint equations, including that term, are satisfied while
/// unilateral constraints and frictional contacts keep their velocity-level (DVI) treatment. The
/// quadratic-velocity term is obtained by central differencing of the constraints (step 1e-6), which puts
/// a roundoff floor on the reported accelerations, also at rest, of about 1e-4 times the length scale of
/// the constraint (SI units: about 1e-4 for a 1 m distance constraint, 1e-2 for 100 m); the corresponding
/// floor on the reactions is that acceleration floor times the mass the constraint acts on.
/// Like the implicit integrators, the analysis scatters perturbed states to the system and expects the
/// update of every item to be a function of (position, velocity, time) only. Accelerations and reactions
/// are formed from a velocity increment over the step dt passed to AssemblyAnalysis() (1e-6 when called
/// through ChSystem::DoAssembly()), so the residual of an iterative solver is amplified by 1/dt in them.
/// When they matter, use a direct solver (SPARSE_LU, SPARSE_QR), MINRES or GMRES, or, if unilateral
/// constraints require a VI solver, BARZILAIBORWEIN. The default PSOR solver can leave acceleration errors of
/// several m/s^2 near singular configurations (e.g., a slider-crank at a dead centre), APGD and PJACOBI even
/// in regular ones. ChSystem::DoAssembly() checks the residual of the acceleration-level solve and returns
/// ExitFlag::ACCELERATION_INACCURATE if it is too large. This dt is distinct from the fixed
/// 1e-6 differencing step of the quadratic-velocity term. With active contacts, the active set and the
/// friction are still decided at velocity level: a resting or separating contact gives the same result as
/// before this formulation, while for a sliding frictional contact the relaxed cone complementarity of the
/// velocity-level step already alters the sliding velocity (artificial separation velocity of order mu
/// times the sliding speed), so accelerations reported for such a contact are not meaningful, before or
/// after.
class ChApi ChAssemblyAnalysis {
  public:
    ChAssemblyAnalysis(ChIntegrableIIorder& mintegrable);

    ~ChAssemblyAnalysis() {}

    /// Perform the assembly analysis.
    /// Assembly is performed by satisfying constraints at position, velocity, and acceleration levels.
    /// Assembly at position level involves solving a non-linear problem.
    /// Assembly at velocity level is performed by taking a small integration step.
    /// Consistent accelerations and reactions are obtained from a second linearized step that includes the
    /// quadratic-velocity term, so that Cq a = Qc holds (see the class description for the accuracy floor).
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
