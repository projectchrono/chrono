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
// =============================================================================
//
// =============================================================================

#ifndef CH_SOA_ASSEMBLY_H
#define CH_SOA_ASSEMBLY_H

#include "chrono/core/ChApiCE.h"
#include "chrono/physics/ChExternalDynamicsDAE.h"
#include "chrono/soa/ChSoaMobilizedBody.h"
#include "chrono/soa/ChSoaForce.h"

namespace chrono {
namespace soa {

/// @addtogroup chrono_soa
/// @{

/// Definition of a Chrono subcomponent representing an assembly modeled using SOA relative coordinate formulation.
/// A ChSoaAssembly is a Chrono modeling component with internal dynamics described by an index-3 DAE.
class ChApi ChSoaAssembly : public ChExternalDynamicsDAE {
  public:
    ChSoaAssembly();
    ~ChSoaAssembly();

    void AddBody(std::shared_ptr<ChSoaMobilizedBody> body);

    void RemoveBody(std::shared_ptr<ChSoaMobilizedBody> body);

    void AddForce(std::shared_ptr<ChSoaForce> force);

    std::shared_ptr<ChGroundBody> getGroundBody() const { return m_ground_body; }
    std::vector<std::shared_ptr<ChSoaMobilizedBody>> getBodies() const { return m_bodies; }

    std::shared_ptr<ChSoaMobilizedBody> findBody(const std::string& name) const;

    virtual void Initialize() override;
    bool IsInitialized() const { return m_initialized; }

    void DoForwardKinematics();

  private:
    // SOA assembly recursive traversal functions

    /// Perform a single recursive base-to-tip traversal to perform position and velocity stage calculations required
    /// for a dynamical simulation. As soon as possible, quantities required to eventually produce a State derivative
    /// are calculated.
    void calcPosAndVel(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd);

    /// Perform the required recursive traversals (tip-to-base and base-to-tip) to perform dynamics and acceleration
    /// stage calculations in order to set the derivative of the given state vector. This method should be called only
    /// after the position and velocity stage calculations were completed and any external forces were applied.
    void calcAcc(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd, ChVectorDynamic<>& ydd);

    // Utility classes for SOA calculations

    /// Form the constraint matrix G * Mi * Gt. It performs 'm' additional recursive tree traversals, one for each
    /// Lagrange multiplier, in order to assemble the matrices Gt and (Mi * Gt), one column at a time.
    /// This function should be called only after position- and velocity-stage calculations have been performed for all
    /// bodies in the system with the current state vector.
    void calcCSMatrix();

    /// Calculate the Lagrange multipliers and applies the resulting constraint forces. The multipliers are obtained as
    /// solution of the linear system:
    /// <pre>
    ///		(G * Mi * Gt) * l = G * ud0 + gamma
    /// </pre>
    /// where 'ud0' are the "tree dynamics" generalized accelerations (i.e. the generalized accelerations in the absence
    /// of constraints) and the right-hand side represents the acceleration-level constraint violations corresponding to
    /// ud0.
    /// It is assumed that the constraint matrix has been assembled and that the open-loop accelerations were calculated
    /// and are available in the specified derivative state vector.
    void calcCSForces();

    /// Calculate the Jacobian with respect to generalized velocities of the velocity-level holonomic constraints (i.e.
    /// the time-derivative of the holonomic constraints). Note that this is not always the same as the Jacobian with
    /// respect to generalized coordinates of the holonomic constraints. This is only true if q' = u and so this doesn't
    /// hold if any body in the system uses quaternions; in such a case, a multiplication by the block-diagonal matrix N
    /// is required (where N, of size numQ x numU, is defined through q' = N * u).
    /// This calculation requires 'numP' recursive tree traversals, one for each row of the Jacobian. This is done by
    /// applying the (body and/or mobility) constraint forces corresponding to a single Lagrange multiplier set to -1,
    /// while all other are 0 (the minus sign on the Lagrange multiplier is required in order to get the correct
    /// Jacobian sign). The resulting generalized joint forces encode the corresponding Jacobian row (the transposed
    /// Jacobian is calculated and stored, one column at a time).
    /// This function should be called only after position-stage calculations have been performed for all bodies in the
    /// system with the current state vector.
    /// Note that the vector of Lagrange multipliers is overwritten (and will be zero on exit).
    void calcCSPosJacobian();

    /// Calculate the Jacobian of all velocity-level constraints (the derivative of the holonomic constraints and the
    /// non-holonomic constraints) with respect to the generalized velocities.
    /// This calculation requires 'numP + numV' recursive tree traversals, one for each row of the Jacobian. This is
    /// done by applying the (body and/or mobility) constraint forces corresponding to a single Lagrange multiplier set
    /// to 1, while all other are 0 (the minus sign on the Lagrange multiplier is required in order to get the correct
    /// Jacobian sign). The resulting generalized joint forces encode the corresponding Jacobian row (the transposed
    /// Jacobian is calculated and stored, one column at a time).
    /// This function should be called only after both position- and velocity-stage calculations have been performed for
    /// all bodies in the system with the current state vector.
    /// Note that the vector of Lagrange multipliers is overwritten (and will be zero on exit).
    void calcCSVelJacobian();

    // Force evaluation functions

    /// Apply all current body and mobility forces to the constituent bodies.
    /// This method should be called only after a forward kinematics traversal (call to calcPosAndVel) so that all body
    /// positions and velocities are available.
    void applyForces(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd);

  private:
    // Virtual methods of ChExternalDynamicsDAE
    //// TODO

    virtual unsigned int GetNumStates() const override { return m_num_q; }
    virtual unsigned int GetNumStateDerivatives() const override { return m_num_u; }
    virtual unsigned int GetNumAlgebraicConstraints() const override { return m_num_c; }

    virtual bool InExplicitForm() const override { return true; }
    virtual bool IsStiff() const override { return false; }

    virtual void SetInitialConditions(ChVectorDynamic<>& y0, ChVectorDynamic<>& yd0) override;

    virtual void CalculateMassMatrix(ChMatrixDynamic<>& M) override {}

    virtual void CalculateForce(double time,
                                const ChVectorDynamic<>& y,
                                const ChVectorDynamic<>& yd,
                                ChVectorDynamic<>& F) override {}

    virtual void CalculateConstraintViolation(double time, const ChVectorDynamic<>& y, ChVectorDynamic<>& c) override {}

    virtual void CalculateConstraintJacobian(double time,
                                             const ChVectorDynamic<>& y,
                                             const ChVectorDynamic<>& c,
                                             ChMatrixDynamic<>& J) override {}

    virtual void IncrementState(const ChVectorDynamic<>& x,
                                const ChVectorDynamic<>& Dv,
                                ChVectorDynamic<>& x_new) override {}

    virtual void CalculateStateIncrement(const ChVectorDynamic<>& x,
                                         const ChVectorDynamic<>& x_new,
                                         ChVectorDynamic<>& Dv) override {}

    // Direct access to state vectors

    double getY(int which) const;
    double getYd(int which) const;
    double getYdd(int which) const;

    void setY(int which, double val);
    void setYd(int which, double val);
    void setYdd(int which, double val);

  private:
    std::shared_ptr<ChGroundBody> m_ground_body;
    std::vector<std::shared_ptr<ChSoaMobilizedBody>> m_bodies;

    std::vector<std::shared_ptr<ChSoaForce>> m_forces;

    bool m_initialized;

    int m_num_q;  // num states
    int m_num_u;  // num state derivatives
    int m_num_c;  // num constraints

    ChVectorDynamic<> m_y0;   ///< vector of initial states
    ChVectorDynamic<> m_yd0;  ///< vector of initial state derivatives

    friend class ChSoaMobilizedBody;
};

/// @} chrono_soa

}  // namespace soa
}  // namespace chrono

#endif
