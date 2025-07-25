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

#ifndef CH_CONTACT_H
#define CH_CONTACT_H

#include "chrono/core/ChFrame.h"
#include "chrono/physics/ChContactable.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/ChCollisionInfo.h"

namespace chrono {

class ChContactContainer;

/// Base class for contact between two generic ChContactable objects.
class ChApi ChContact {
  public:
    ChContact() {}

    virtual ~ChContact() {}

    /// Reinitialize geometric information for this contact for reuse.
    void Reset_cinfo(ChContactable* obj_A,         ///< contactable object A
                     ChContactable* obj_B,         ///< contactable object B
                     const ChCollisionInfo& cinfo  ///< data for the contact pair
    );

    /// Get the colliding object A, with point P1
    ChContactable* GetObjA() { return this->objA; }

    /// Get the colliding object B, with point P2
    ChContactable* GetObjB() { return this->objB; }

    /// Get the contact coordinate system, expressed in absolute frame.
    /// This represents the 'main' reference of the link: reaction forces
    /// are expressed in this coordinate system. Its origin is point P2.
    /// (It is the coordinate system of the contact plane and normal)
    ChCoordsys<> GetContactCoords() const;

    /// Returns the pointer to a contained 3x3 matrix representing the UV and normal
    /// directions of the contact. In detail, the X versor (the 1s column of the
    /// matrix) represents the direction of the contact normal.
    const ChMatrix33<>& GetContactPlane() const { return contact_plane; }

    /// Get the contact point 1, in absolute coordinates
    const ChVector3d& GetContactP1() const { return p1; }

    /// Get the contact point 2, in absolute coordinates
    const ChVector3d& GetContactP2() const { return p2; }

    /// Get the contact normal, in absolute coordinates
    const ChVector3d& GetContactNormal() const { return normal; }

    /// Get the contact distance
    double GetContactDistance() const { return norm_dist; }

    /// Get the effective radius of curvature.
    double GetEffectiveCurvatureRadius() const { return eff_radius; }

    /// Get the contact force, if computed, in contact coordinate system
    virtual ChVector3d GetContactForce() const { return ChVector3d(0); }

    /// Get the contact torque, if computed, in contact coordinate system
    virtual ChVector3d GetContactTorque() const { return ChVector3d(0); }

    // UPDATING FUNCTIONS

    virtual void ContIntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {}

    virtual void ContIntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {}

    virtual void ContIntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                         ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                         const ChVectorDynamic<>& L,  ///< the L vector
                                         const double c               ///< a scaling factor
    ) {}

    virtual void ContIntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
                                         ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*C
                                         const double c,            ///< a scaling factor
                                         bool do_clamp,             ///< apply clamping to c*C?
                                         double recovery_clamp      ///< value for min/max clamping of c*C
    ) {}

    virtual void ContIntLoadResidual_F(ChVectorDynamic<>& R, const double c) {}

    virtual void ContInjectKRMmatrices(ChSystemDescriptor& mdescriptor) {}

    virtual void ContKRMmatricesLoad(double Kfactor, double Rfactor) {}

    virtual void ContIntToDescriptor(const unsigned int off_L,    ///< offset in L, Qc
                                     const ChVectorDynamic<>& L,  ///< the L vector
                                     const ChVectorDynamic<>& Qc  ///< the Qc vector
    ) {}

    virtual void ContIntFromDescriptor(const unsigned int off_L,  ///< offset in L
                                       ChVectorDynamic<>& L       ///< the L vector
    ) {}

    virtual void InjectConstraints(ChSystemDescriptor& descriptor) {}

    virtual void ConstraintsBiReset() {}

    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false) {}

    virtual void ConstraintsFetch_react(double factor) {}

  protected:
    ChContact(ChContactContainer* contact_container, ChContactable* obj_A, ChContactable* obj_B);

    ChContactContainer* container;  ///< associated contact container

    ChContactable* objA;  ///< first ChContactable object in the pair
    ChContactable* objB;  ///< second ChContactable object in the pair

    ChVector3d p1;      ///< max penetration point on geo1, after refining, in abs space
    ChVector3d p2;      ///< max penetration point on geo2, after refining, in abs space
    ChVector3d normal;  ///< normal, on surface of master reference (geo1)

    ChMatrix33<> contact_plane;  ///< the plane of contact (X is normal direction)

    double norm_dist;   ///< penetration distance (negative if going inside) after refining
    double eff_radius;  ///< effective radius of curvature at contact
};

}  // end namespace chrono

#endif
