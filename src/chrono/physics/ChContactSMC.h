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
// Authors: Radu Serban, Alessandro Tasora
// =============================================================================
//
// Smooth (penalty-based) contact between two generic contactable objects.
//
// =============================================================================

#ifndef CH_CONTACT_SMC_H
#define CH_CONTACT_SMC_H

#include <cmath>
#include <algorithm>
#include <limits>

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/solver/ChKRMBlock.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChContact.h"
#include "chrono/physics/ChContactMaterialSMC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/timestepper/ChState.h"

namespace chrono {

/// Default implementation of the SMC normal and tangential force calculation.
class ChApi ChDefaultContactForceTorqueSMC : public ChSystemSMC::ChContactForceTorqueSMC {
  public:
    /// Default SMC force calculation algorithm.
    /// This implementation depends on various settings specified at the ChSystemSMC level (such as normal force model,
    /// tangential force model, use of material physical properties, etc).
    virtual ChWrenchd CalculateForceTorque(
        const ChSystemSMC& sys,                    ///< containing system
        const ChVector3d& normal_dir,              ///< normal contact direction (expressed in global frame)
        const ChVector3d& p1,                      ///< most penetrated point on obj1 (expressed in global frame)
        const ChVector3d& p2,                      ///< most penetrated point on obj2 (expressed in global frame)
        const ChVector3d& vel1,                    ///< velocity of contact point on obj1 (expressed in global frame)
        const ChVector3d& vel2,                    ///< velocity of contact point on obj2 (expressed in global frame)
        const ChContactMaterialCompositeSMC& mat,  ///< composite material for contact pair
        double delta,                              ///< overlap in normal direction
        double eff_radius,                         ///< effective radius of curvature at contact
        double mass1,                              ///< mass of obj1
        double mass2,                              ///< mass of obj2
        ChContactable* objA,                       ///< pointer to contactable obj1
        ChContactable* objB                        ///< pointer to contactable obj2
    ) const override;
};

// -----------------------------------------------------------------------------

/// Class for smooth (penalty-based) contact between two generic contactable objects.
class ChApi ChContactSMC : public ChContact {
  public:
    ChContactSMC();

    ChContactSMC(ChContactContainer* contact_container,    ///< contact container
                 ChContactable* obj_A,                     ///< contactable object A
                 ChContactable* obj_B,                     ///< contactable object B
                 const ChCollisionInfo& cinfo,             ///< data for the collision pair
                 const ChContactMaterialCompositeSMC& mat  ///< composite material
    );

    ~ChContactSMC() { delete m_Jac; }

    /// Get the contact force, if computed, in contact coordinate system
    virtual ChVector3d GetContactForce() const override { return this->contact_plane.transpose() * m_force; }

    /// Get the contact torque, if computed, in contact coordinate system
    virtual ChVector3d GetContactTorque() const override { return this->contact_plane.transpose() * m_torque; }

    /// Get the contact penetration (positive if there is overlap).
    double GetContactPenetration() const { return -this->norm_dist; }

    /// Get the contact force, expressed in the absolute frame
    ChVector3d GetContactForceAbs() const { return m_force; }

    /// Get the contact torque, expressed in the absolute frame
    ChVector3d GetContactTorqueAbs() const { return m_torque; }

    /// Access the proxy to the Jacobian.
    const ChKRMBlock* GetJacobianKRM() const { return m_Jac ? &(m_Jac->m_KRM) : NULL; }
    const ChMatrixDynamic<double>* GetJacobianK() const { return m_Jac ? &(m_Jac->m_K) : NULL; }
    const ChMatrixDynamic<double>* GetJacobianR() const { return m_Jac ? &(m_Jac->m_R) : NULL; }

    /// Reinitialize this contact for reuse.
    void Reset(ChContactable* obj_A,                     ///< contactable object A
               ChContactable* obj_B,                     ///< contactable object B
               const ChCollisionInfo& cinfo,             ///< data for the collision pair
               const ChContactMaterialCompositeSMC& mat  ///< composite material
    );

    /// Calculate contact force, and maybe torque too, expressed in absolute coordinates.
    ChWrenchd CalculateForceTorque(
        double delta,                             ///< overlap in normal direction
        const ChVector3d& normal_dir,             ///< normal contact direction (expressed in global frame)
        const ChVector3d& vel1,                   ///< velocity of contact point on objA (expressed in global frame)
        const ChVector3d& vel2,                   ///< velocity of contact point on objB (expressed in global frame)
        const ChContactMaterialCompositeSMC& mat  ///< composite material for contact pair
    );

    /// Compute all forces in a contiguous array.
    /// Used in finite-difference Jacobian approximation.
    void CalculateQ(const ChState& stateA_x,                   ///< state positions for objA
                    const ChStateDelta& stateA_w,              ///< state velocities for objA
                    const ChState& stateB_x,                   ///< state positions for objB
                    const ChStateDelta& stateB_w,              ///< state velocities for objB
                    const ChContactMaterialCompositeSMC& mat,  ///< composite material for contact pair
                    ChVectorDynamic<>& Q                       ///< output generalized forces
    );

    /// Create the Jacobian matrices.
    /// These matrices are created/resized as needed.
    void CreateJacobians();

    /// Calculate Jacobian of generalized contact forces.
    void CalculateJacobians(const ChContactMaterialCompositeSMC& mat);

    /// Apply contact forces to the two objects.
    /// (new version, for interfacing to ChTimestepper and ChIntegrable)
    virtual void ContIntLoadResidual_F(ChVectorDynamic<>& R, const double c) override;

    /// Inject Jacobian blocks into the system descriptor.
    /// Tell to a system descriptor that there are item(s) of type ChKRMBlock in this object
    /// (for further passing it to a solver)
    virtual void ContInjectKRMmatrices(ChSystemDescriptor& mdescriptor) override;

    /// Compute Jacobian of contact forces.
    virtual void ContKRMmatricesLoad(double Kfactor, double Rfactor) override;

  private:
    struct ChContactJacobian {
        ChKRMBlock m_KRM;             ///< sum of scaled K and R, with pointers to sparse variables
        ChMatrixDynamic<double> m_K;  ///< K = dQ/dx
        ChMatrixDynamic<double> m_R;  ///< R = dQ/dv
    };

    ChVector3d m_force;        ///< contact force on objB
    ChVector3d m_torque;       ///< contact torque on objB
    ChContactJacobian* m_Jac;  ///< contact Jacobian data
};

}  // end namespace chrono

#endif
