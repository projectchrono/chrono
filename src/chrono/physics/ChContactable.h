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

#ifndef CHCONTACTABLE_H
#define CHCONTACTABLE_H

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChCoordsys.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/solver/ChConstraintTuple.h"

namespace chrono {

/// Forward definition
class ChPhysicsItem;

/// Interface for objects that generate contacts.
/// A derived class must add all its variable sets (of ChVariable type) to the list managed by this class.
class ChApi ChContactable {
  public:
    /// Contactable type (based on number of variables objects and their DOFs).
    enum class Type {
        UNKNOWN,    ///< unknown contactable type
        ONE_1,      ///< 1 variable with 1 DOF
        ONE_2,      ///< 1 variable with 2 DOFs
        ONE_3,      ///< 1 variable with 3 DOFs (e.g., ChNodeFEAxyz)
        ONE_4,      ///< 1 variable with 4 DOFs
        ONE_5,      ///< 1 variable with 5 DOFs
        ONE_6,      ///< 1 variable with 6 DOFs (e.g., ChBody, ChParticle, ChNodeFEAxyzrot)
        TWO_33,     ///< 2 variables, each with 3 DOFs (e.g., segments between 2 ChNodeFEAxyz nodes)
        TWO_66,     ///< 2 variables, each with 6 DOFs (e.g., triangle between 2 ChNodeFEAxyzrot nodes)
        THREE_333,  ///< 3 variables, each with 3 DOFs (e.g., triangle between 3 ChNodeFEAxyz nodes)
        THREE_666   ///< 3 variables, each with 6 DOFs (e.g., triangle between 3 ChNodeFEAxyzrot nodes)
    };

    virtual ~ChContactable() {}

    /// Return the proper contactable Type.
    /// Used for the collision dispatcher in ChContactContainer classes.
    virtual Type GetContactableType() const = 0;

    /// Add the collision model.
    void AddCollisionModel(std::shared_ptr<ChCollisionModel> model);

    /// Add a collision shape.
    /// If this item does not have a collision model, one is created.
    void AddCollisionShape(std::shared_ptr<ChCollisionShape> shape, const ChFrame<>& frame = ChFrame<>());

    /// Access the collision model.
    std::shared_ptr<ChCollisionModel> GetCollisionModel() const;

    /// Indicate whether or not the object must be considered in collision detection.
    virtual bool IsContactActive() = 0;

    /// Create a constraint tuple with the appropriate number of variables for this contactable object.
    /// Required for NSC contact formulation.
    virtual ChConstraintTuple* CreateConstraintTuple() = 0;

    /// Get the number of DOFs affected by this object (position part).
    virtual int GetContactableNumCoordsPosLevel() = 0;

    /// Get the number of DOFs affected by this object (speed part).
    virtual int GetContactableNumCoordsVelLevel() = 0;

    /// Get all the DOFs packed in a single vector (position part).
    virtual void ContactableGetStateBlockPosLevel(ChState& x) = 0;

    /// Get all the DOFs packed in a single vector (speed part).
    virtual void ContactableGetStateBlockVelLevel(ChStateDelta& w) = 0;

    /// Increment the provided state of this object by the given state-delta increment.
    /// Compute: x_new = x + dw.
    virtual void ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) = 0;

    /// Express the local point in absolute frame, for the given state position.
    virtual ChVector3d GetContactPoint(const ChVector3d& loc_point, const ChState& state_x) = 0;

    /// Get the absolute speed of a local point attached to the contactable.
    /// The given point is assumed to be expressed in the local frame of this object.
    /// This function must use the provided states.
    virtual ChVector3d GetContactPointSpeed(const ChVector3d& loc_point,
                                            const ChState& state_x,
                                            const ChStateDelta& state_w) = 0;

    /// Get the absolute speed of point abs_point if attached to the surface.
    virtual ChVector3d GetContactPointSpeed(const ChVector3d& abs_point) = 0;

    /// Return the frame of the associated collision model relative to the contactable object.
    /// ChCollisionModel might call this to get the position of the contact model (when rigid) and sync it.
    virtual ChFrame<> GetCollisionModelFrame() = 0;

    /// Apply the given force & torque at the given location and load into the global generalized force vector.
    /// The force  F and its application point are specified in the absolute reference frame, while the torque T is
    /// specified in the global frame too. Each object must update the entries in R corresponding to its variables.
    /// Required for SMC contact formulation.
    virtual void ContactForceLoadResidual_F(const ChVector3d& F,          ///< force
                                            const ChVector3d& T,          ///< torque
                                            const ChVector3d& abs_point,  ///< application point
                                            ChVectorDynamic<>& R          ///< global generalized force vector
                                            ) = 0;

    /// Compute a contiguous vector of generalized forces Q from a given force & torque at the given point.
    /// Used for computing stiffness matrix (square force jacobian) by backward differentiation.
    /// The force  F and its application point are specified in the global frame, while the torque T is specified in the
    /// global frame too. Each object must set the entries in Q corresponding to its variables, starting at the
    /// specified offset. If needed, the object states must be extracted from the provided state position.
    /// Required for SMC contact formulation.
    virtual void ContactComputeQ(const ChVector3d& F,      ///< force
                                 const ChVector3d& T,      ///< torque
                                 const ChVector3d& point,  ///< application point
                                 const ChState& state_x,   ///< global state vector
                                 ChVectorDynamic<>& Q,     ///< generalized force vector
                                 int offset                ///< index offset
                                 ) = 0;

    /// Return the mass of the contactable object.
    /// Required for SMC contact formulation.
    virtual double GetContactableMass() = 0;

    /// Return the contactable object as a ChPhysicsItem.
    /// Provided only for backward compatibility. In newer code, the reference to a ChPhysicsItem should not be usedr.
    virtual ChPhysicsItem* GetPhysicsItem() = 0;

    /// Set user-data associated with this contactable.
    void SetUserData(const std::shared_ptr<void>& data) { m_data = data; }

    /// Check if this contactable has associated user-data.
    bool HasData() const { return m_data != nullptr; }

    /// Get the user-data using static cast to a known type.
    template <typename T>
    std::shared_ptr<T> GetUserData() const {
        return std::static_pointer_cast<T>(m_data);
    }

    /// Compute the Jacobian part(s) for this contactable item.
    /// For example, if the contactable is a ChBody, this should update the corresponding 1x6 Jacobian.
    /// Required for NSC contact formulation.
    virtual void ComputeJacobianForContactPart(const ChVector3d& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               ChConstraintTuple* jacobian_tuple_N,
                                               ChConstraintTuple* jacobian_tuple_U,
                                               ChConstraintTuple* jacobian_tuple_V,
                                               bool second) = 0;

    /// Compute the rolling Jacobian part(s) for this contactable item.
    /// Required for NSC contact formulation.
    virtual void ComputeJacobianForRollingContactPart(const ChVector3d& abs_point,
                                                      ChMatrix33<>& contact_plane,
                                                      ChConstraintTuple* jacobian_tuple_N,
                                                      ChConstraintTuple* jacobian_tuple_U,
                                                      ChConstraintTuple* jacobian_tuple_V,
                                                      bool second) {}

    /// Method to allow serialization of transient data to archives.
    void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow deserialization of transient data from archives.
    void ArchiveIn(ChArchiveIn& archive_in);

  protected:
    ChContactable();

    std::shared_ptr<ChCollisionModel> collision_model;  ///< collision model for the contactable object
    std::vector<ChVariables*> m_contactable_variables;  ///< variables associated with the contactable object
    std::shared_ptr<void> m_data;                       ///< arbitrary user-data

    friend class ChContactSMC;
};

}  // end namespace chrono

#endif
