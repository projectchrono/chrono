// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
//
// Physics element that carries its own dynamics, described as a system of DAEs.
// The internal states are integrated simultaneously with the containing system
// and they can be accessed and coupled with other physics elements.
// =============================================================================

#ifndef CH_EXTERNAL_DYNAMICS_DAE_H
#define CH_EXTERNAL_DYNAMICS_DAE_H

#include "chrono/physics/ChPhysicsItem.h"
#include "chrono/solver/ChVariablesGeneric.h"
#include "chrono/solver/ChConstraintNgeneric.h"
#include "chrono/solver/ChKRMBlock.h"

namespace chrono {

/// Physics element that carries its own dynamics, described as an index-3 DAE IVP of the form:
/// <pre>
///   M * y'' + (Cy)^T * lambda = F
///   C(y) = 0
///   y(t0) = y0
///   y'(t0) = yd0
/// </pre>
/// The internal states are integrated simultaneously with the containing system and they can be accessed and coupled
/// with other physics elements.
class ChApi ChExternalDynamicsDAE : public ChPhysicsItem {
  public:
    virtual ~ChExternalDynamicsDAE();

    /// Initialize the physics item.
    virtual void Initialize();

    /// Declare as explicit form (identity mass matrix).
    /// If not in explicit form, a derived class must implement the function CalculateMassMatrix and optionally
    /// CalculateMassTimesVector.
    virtual bool InExplicitForm() const { return false; }

    /// Declare as stiff (default: false).
    /// If stiff, Jacobian information will be generated.
    virtual bool IsStiff() const { return false; }

    /// Declare the DAE as rheonomous (default: false).
    /// A rheonomous system has constraints that depend explicitly on time.
    virtual bool IsRheonomous() const { return false; }

    /// Get number of states (dimension of y).
    virtual unsigned int GetNumStates() const = 0;

    /// Get number of state derivatives (dimension of yd and ydd).
    /// The default implementation assumes equal number of states and state derivatives.
    /// If this is not the case, a derived class must implement the functions IncrementState, CalculateStateIncrement,
    /// and (if stiff) CalculateForceJacobians.
    virtual unsigned int GetNumStateDerivatives() const { return GetNumStates(); }

    /// Get number of algebraic constraints.
    virtual unsigned int GetNumAlgebraicConstraints() const { return 0; }

    /// Get the initial state values (state at initial time).
    ChVectorDynamic<> GetInitialStates();

    /// Get the initial state derivative values (state derivatives at initial time).
    ChVectorDynamic<> GetInitialStateDerivatives();

    /// Get current states.
    const ChVectorDynamic<>& GetStates() const { return m_y; }

    /// Get current state derivatives.
    const ChVectorDynamic<>& GetStateDerivatives() const { return m_yd; }

    /// Get current applied force.
    const ChVectorDynamic<>& GetForce() const { return m_F; }

  protected:
    ChExternalDynamicsDAE();

    /// Set initial conditions.
    /// Must load y0 = y(0).
    virtual void SetInitialConditions(ChVectorDynamic<>& y0, ChVectorDynamic<>& yd0) = 0;

    /// Calculate and return mass matrix.
    /// This function must be implemented if the DAE is in implicit form.
    virtual void CalculateMassMatrix(ChMatrixDynamic<>& M) {
        if (!InExplicitForm()) {
            throw std::runtime_error("CalculateMassMatrix required for a DAE in implicit form.");
        }
    }

    /// Calculate and return the product of the mass matrix and the given vector.
    /// This function is optional and is used only if the DAE is declared in implicit form. If provided, calculate M*v
    /// in the provided vector 'Mv' and return 'true'. In that case, this overwrites the default matrix-vector product
    /// calculation.
    virtual bool CalculateMassTimesVector(const ChVectorDynamic<>& v, ChVectorDynamic<>& Mv) { return false; }

    /// Calculate and return the generalized force (right-hand side) at the provided time and states.
    /// Must load F = F(t,y, yd).
    virtual void CalculateForce(double time,                  ///< current time
                                const ChVectorDynamic<>& y,   ///< current DAE states
                                const ChVectorDynamic<>& yd,  ///< current DAE state derivatives
                                ChVectorDynamic<>& F          ///< output right-hand side force vector
                                ) = 0;

    /// Calculate a linear combination of the Jacobians of the generalized force with respect to the DAE states.
    /// Must load J = alpha * dF/dy + beta * dF/dyd, for given alpha and beta.
    /// Only used if the physics item is declared as stiff.  If provided, load J into the provided matrix 'J'
    /// (already set to zero before the call) and return 'true'. In that case, the user-provided Jacobian will
    /// overwrite the default finite-difference approximation. Note that a derived class must always implement
    /// CalculateForceJacobians if the number of derivatives and states are different.
    virtual bool CalculateForceJacobians(double time,                  ///< current time
                                         const ChVectorDynamic<>& y,   ///< current DAE states
                                         const ChVectorDynamic<>& yd,  ///< current DAE state derivatives
                                         const ChVectorDynamic<>& F,   ///< current right-hand side force vector
                                         double alpha,                 ///< multiplier for Jacobian wrt y
                                         double beta,                  ///< multiplier for Jacobian wrt yd
                                         ChMatrixDynamic<>& J          ///< output Jacobian matrix
    ) {
        if (IsStiff() && m_ny != m_nyd) {
            throw std::runtime_error("CalculateForceJacobians required for a stiff DAE with Ny != Nyd.");
        }
        return false;
    }

    /// Calculate the constraint violations c.
    virtual void CalculateConstraintViolation(double time,                 ///< current time
                                              const ChVectorDynamic<>& y,  ///< current DAE states
                                              ChVectorDynamic<>& c         ///< output constraint violation vector
    ) {
        if (m_nc > 0)
            throw std::runtime_error("CalculateConstraintViolation required if constraints are present.");
    }

    /// Calculate the constraint violation partial derivative with respect to time.
    /// Must load ct = \partial c / \partial t, for c = c(t,y).
    /// This function is optional and only used is the system is rheonomous. If provided, load the constraint
    /// derivatives in the provided vector 'c' and return 'true'. In that case, this overrides the default finite
    /// difference approximation.
    virtual bool CalculateConstraintDerivative(double time,                 ///< current time
                                               const ChVectorDynamic<>& y,  ///< current DAE states
                                               const ChVectorDynamic<>& c,  ///< current constraint violations
                                               ChVectorDynamic<>& ct        ///< output constraint derivative vector
    ) {
        return false;
    }

    /// Calculate the Jacobian of the constraints with respect to the DAE states.
    virtual void CalculateConstraintJacobian(double time,                 ///< current time
                                             const ChVectorDynamic<>& y,  ///< current DAE states
                                             const ChVectorDynamic<>& c,  ///< current constraint violations
                                             ChMatrixDynamic<>& J         ///< output Jacobian matrix
    ) {
        if (m_nc > 0)
            throw std::runtime_error("CalculateConstraintViolation required if constraints are present.");
    }

    /// Increment the state vector by the provided vector.
    /// Must calculate x_new = x + Dv. If the number of derivatives is equal to the number of states, this is a simple
    /// vector addition and this function need not be implemented. Otherwise (e.g., when using quaternions for
    /// rotations), special treatment is required and a derived class must provide an override. Here, 'x' and 'x_new'
    /// are vectors of length Ny, and 'Dv' has length Nyd.
    virtual void IncrementState(const ChVectorDynamic<>& x, const ChVectorDynamic<>& Dv, ChVectorDynamic<>& x_new) {
        if (m_ny != m_nyd)
            throw std::runtime_error("IncrementState required for a DAE with Ny != Nyd.");
    }

    /// Calculate the increment between the two procided state vectors.
    /// Must calculate Dv = x_new - x. If the number of derivatives is equal to the number of states, this is a simple
    /// vector subtraction and this function need not be implemented. Otherwise (e.g., when using quaternions for
    /// rotations), special treatment is required and a derived class must provide an override. Here, 'x' and 'x_new'
    /// are vectors of length Ny, and 'Dv' has length Nyd.
    virtual void CalculateStateIncrement(const ChVectorDynamic<>& x,
                                         const ChVectorDynamic<>& x_new,
                                         ChVectorDynamic<>& Dv) {
        if (m_ny != m_nyd)
            throw std::runtime_error("IncrementState required for a DAE with Ny != Nyd.");
    }

    /// Optional operations performed before an update.
    /// This function is invoked before calls to CalculateForce, CalculateConstraintViolation, and
    /// CalculateConstraintJacobian.
    virtual void OnUpdate(double time,                 ///< current time
                          const ChVectorDynamic<>& y,  ///< current DAE states
                          const ChVectorDynamic<>& yd  ///< current DAE state derivatives
    ) {}

  protected:
    virtual void Update(double time, bool update_assets) override;

    virtual unsigned int GetNumCoordsPosLevel() override { return m_ny; }
    virtual unsigned int GetNumCoordsVelLevel() override { return m_nyd; }
    virtual unsigned int GetNumConstraints() override { return m_nc; }
    virtual unsigned int GetNumConstraintsBilateral() override { return m_nc; }

    // Interface to solver
    virtual void InjectVariables(ChSystemDescriptor& descriptor) override;
    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override;
    virtual void InjectKRMMatrices(ChSystemDescriptor& descriptor) override;

    virtual void IntStateGather(const unsigned int off_x,
                                ChState& x,
                                const unsigned int off_v,
                                ChStateDelta& v,
                                double& T) override;
    virtual void IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T,
                                 bool full_update) override;
    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override;
    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override;
    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void IntStateIncrement(const unsigned int off_x,
                                   ChState& x_new,
                                   const ChState& x,
                                   const unsigned int off_v,
                                   const ChStateDelta& Dv) override;
    virtual void IntStateGetIncrement(const unsigned int off_x,
                                      const ChState& x_new,
                                      const ChState& x,
                                      const unsigned int off_v,
                                      ChStateDelta& Dv) override;
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void IntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& v,
                                    const double c) override;
    virtual void IntLoadLumpedMass_Md(const unsigned int off,
                                      ChVectorDynamic<>& Md,
                                      double& err,
                                      const double c) override;
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp) override;
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override;
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c) override;
    virtual void IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) override;
    virtual void IntFromDescriptor(const unsigned int off_v,
                                   ChStateDelta& v,
                                   const unsigned int off_L,
                                   ChVectorDynamic<>& L) override;

    virtual void LoadConstraintJacobians() override;
    virtual void LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) override;

    // Interface to solver (old style)
    /*
    virtual void VariablesFbReset() override;
    virtual void VariablesFbLoadForces(double factor = 1) override;
    virtual void VariablesQbLoadSpeed() override;
    virtual void VariablesQbSetSpeed(double step = 0) override;
    virtual void VariablesFbIncrementMq() override;
    virtual void VariablesQbIncrementPosition(double step) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void ConstraintsBiLoad_Qc(double factor = 1) override;
    virtual void ConstraintsFbLoadForces(double factor = 1) override;
    virtual void ConstraintsFetch_react(double factor = 1) override;
    */

  private:
    /// Compute the Jacobian at the current time and state.
    /// Use the provided Jacobian (if CalculateForceJacobians is implemented), otherwise approximate with finite
    /// differences.
    void ComputeForceJacobian(double time, double alpha, double beta);

    /// Compute the constraint time derivatives.
    /// Use the provided vector (if CalculateConstraintDerivative is implemented), otherwise approximate with finite
    /// differences.
    void ComputeConstraintDerivative(double time);

    unsigned int m_ny;                                ///< number of DAE states
    unsigned int m_nyd;                               ///< number of DAE state derivatives
    unsigned int m_nc;                                ///< number of algebraic constraints
    ChVectorDynamic<> m_y;                            ///< vector of DAE states
    ChVectorDynamic<> m_yd;                           ///< vector of DAE state first derivatives
    ChVectorDynamic<> m_ydd;                          ///< vector of DAE state second derivatives
    ChVariablesGeneric* m_variables;                  ///< carrier for internal dynamics states
    std::vector<double> m_multipliers;                ///< vector of DAE Lagrange multipliers
    std::vector<ChConstraintNgeneric> m_constraints;  ///< carrier for algebraic constraints

    ChMatrixDynamic<> m_M;   ///< mass matrix
    ChVectorDynamic<> m_F;   ///< generalized force
    ChMatrixDynamic<> m_J;   ///< Jacobian of generalized force w.r.t. DAE states and derivatives
    ChVectorDynamic<> m_c;   ///< vector of constraint violations
    ChVectorDynamic<> m_ct;  ///< vector of constraint time derivatives
    ChMatrixDynamic<> m_Jc;  ///< Jacobian of constraints w.r.t. DAE states

    ChKRMBlock m_KRM;  ///< linear combination of K, R, M for the variables associated with item

    static const double m_FD_delta;  ///< perturbation for finite-difference Jacobian approximation
};

}  // end namespace chrono

#endif
