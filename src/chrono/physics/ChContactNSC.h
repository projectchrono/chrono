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

#ifndef CH_CONTACT_NSC_H
#define CH_CONTACT_NSC_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChContact.h"
#include "chrono/solver/ChConstraintContactNormal.h"
#include "chrono/solver/ChConstraintContactTangential.h"
#include "chrono/solver/ChSystemDescriptor.h"

namespace chrono {

/// Class for non-smooth contact between two generic ChContactable objects.
class ChApi ChContactNSC : public ChContact {
  public:
    ChContactNSC();

    ChContactNSC(ChContactContainer* contact_container,     ///< contact container
                 ChContactable* obj_A,                      ///< contactable object A
                 ChContactable* obj_B,                      ///< contactable object B
                 const ChCollisionInfo& cinfo,              ///< data for the collision pair
                 const ChContactMaterialCompositeNSC& mat,  ///< composite material
                 double min_speed                           ///< minimum speed for rebounce
    );

    ~ChContactNSC() {}

    /// Reinitialize this contact for reuse.
    virtual void Reset(ChContactable* obj_A,                      ///< contactable object A
                       ChContactable* obj_B,                      ///< contactable object B
                       const ChCollisionInfo& cinfo,              ///< data for the collision pair
                       const ChContactMaterialCompositeNSC& mat,  ///< composite material
                       double min_speed                           ///< minimum speed for rebounce
    );

    /// Get the contact force, if computed, in contact coordinate system
    virtual ChVector3d GetContactForce() const override { return react_force; }

    /// Get the contact friction coefficient
    virtual double GetFriction() { return Nx.GetFrictionCoefficient(); }

    /// Set the contact friction coefficient
    virtual void SetFriction(double mf) { Nx.SetFrictionCoefficient(mf); }

    /// Access the constraints
    ChConstraint* GetConstraintNx() { return &Nx; }
    ChConstraint* GetConstraintTu() { return &Tu; }
    ChConstraint* GetConstraintTv() { return &Tv; }

    // UPDATING FUNCTIONS

    virtual void ContIntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void ContIntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void ContIntLoadResidual_CqL(const unsigned int off_L,
                                         ChVectorDynamic<>& R,
                                         const ChVectorDynamic<>& L,
                                         const double c) override;
    virtual void ContIntLoadConstraint_C(const unsigned int off_L,
                                         ChVectorDynamic<>& Qc,
                                         const double c,
                                         bool do_clamp,
                                         double recovery_clamp) override;
    virtual void ContIntToDescriptor(const unsigned int off_L,
                                     const ChVectorDynamic<>& L,
                                     const ChVectorDynamic<>& Qc) override;
    virtual void ContIntFromDescriptor(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsFetch_react(double factor) override;

  protected:
    // The three scalar constraints, to be fed into the system solver
    ChConstraintContactNormal Nx;      ///< normal constraint
    ChConstraintContactTangential Tu;  ///< first tangential sliding constraint
    ChConstraintContactTangential Tv;  ///< second tangential sliding constraint

    float* reactions_cache;  ///< N,U,V reactions potentially stored in a persistent contact manifold
    ChVector3d react_force;  ///< constraint reaction force

    double compliance;
    double complianceT;
    double restitution;
    double dampingf;

    double min_rebounce_speed;
};

}  // end namespace chrono

#endif
