// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
//
// Physics element that carries its own dynamics, described as a system of ODEs.
// The internal states are integrated simultaneous with the containing system
// and they can be accessed and used coupled with other physics elements.
// =============================================================================

#ifndef CH_EXTERNAL_SYNAMICS_H
#define CH_EXTERNAL_SYNAMICS_H

#include "chrono/physics/ChPhysicsItem.h"
#include "chrono/solver/ChVariablesGenericDiagonalMass.h"
#include "chrono/solver/ChKblockGeneric.h"

namespace chrono {

/// Physics element that carries its own dynamics, described as a system of ODEs.
/// The internal states are integrated simultaneous with the containing system and they can be accessed and used coupled
/// with other physics elements.
class ChApi ChExternalDynamics : public ChPhysicsItem {
  public:
    virtual ~ChExternalDynamics();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChExternalDynamics* Clone() const override;

    /// Initialize the physics item.
    void Initialize();

    /// Get the initial values (state at initial time).
    ChVectorDynamic<> GetInitialStates();

    /// Get current states.
    const ChVectorDynamic<>& GetStates() const { return m_states; }

  protected:
    ChExternalDynamics();

    /// Declare as stiff (default: false).
    /// If stiff, Jacobian information will be generated.
    virtual bool IsStiff() const { return false; }

    /// Specify number of states (dimension of y).
    virtual int GetNumStates() const { return 0; }

    /// Set initial conditions.
    /// Must load y0 = y(0).
    virtual void SetInitialConditions(ChVectorDynamic<>& y0) {}

    /// Calculate and return the ODE right-hand side at the provided time and states.
    /// Must load rhs = f(t,y).
    virtual void CalculateRHS(double time,                 ///< current time
                              const ChVectorDynamic<>& y,  ///< current ODE states
                              ChVectorDynamic<>& rhs       ///< output ODE right-hand side vector
    ) {}

    /// Calculate the Jacobian of the ODE right-hand side with respect to the ODE states.
    /// Must load J = df/dy.
    /// Only used if the physics item is declared as stiff.  If provided, load df/dy into the provided matrix 'jac'
    /// (already set to zero before the call) and return 'true'. In that case, the user-provided Jacobian will
    /// overwrite the default finite-difference approximation.
    virtual bool CalculateJac(double time,                   ///< current time
                              const ChVectorDynamic<>& y,    ///< current ODE states
                              const ChVectorDynamic<>& rhs,  ///< current ODE right-hand side vector
                              ChMatrixDynamic<>& J           ///< output Jacobian matrix
    ) {
        return false;
    }

    virtual void Update(double time, bool update_assets = true) override;

    virtual int GetDOF() override { return m_nstates; }

    ChVariables& Variables() { return *m_variables; }

    // Interface to solver
    virtual void InjectVariables(ChSystemDescriptor& descriptor) override;
    virtual void InjectKRMmatrices(ChSystemDescriptor& descriptor) override;

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
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void IntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& v,
                                    const double c) override;
    virtual void IntLoadLumpedMass_Md(const unsigned int off,
                                      ChVectorDynamic<>& Md,
                                      double& err,
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

    virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) override;

    // Interface to the solver (old style)
    virtual void VariablesFbReset() override;
    virtual void VariablesFbLoadForces(double factor = 1) override;
    virtual void VariablesQbLoadSpeed() override;
    virtual void VariablesQbSetSpeed(double step = 0) override;
    virtual void VariablesFbIncrementMq() override;
    virtual void VariablesQbIncrementPosition(double step) override;
    virtual void ConstraintsFbLoadForces(double factor = 1) override;

    /// Compute the Jacobian at the current time and state.
    void ComputeJac(double time);

  private:
    int m_nstates;                                ///< number of internal ODE states
    ChVectorDynamic<> m_states;                   ///< vector of internal ODE states
    ChVariablesGenericDiagonalMass* m_variables;  ///< carrier for internal dynamics states

    ChVectorDynamic<> m_rhs;  ///< generalized forcing terms (ODE RHS)
    ChMatrixDynamic<> m_jac;  ///< Jacobian of ODE right-hand side w.r.t. ODE states

    ChKblockGeneric m_KRM;  ///< linear combination of K, R, M for the variables associated with item

    static const double m_FD_delta;  ///< perturbation for finite-difference Jacobian approximation
};

}  // end namespace chrono

#endif
