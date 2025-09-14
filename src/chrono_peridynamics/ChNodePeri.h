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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHNODEPERI_H
#define CHNODEPERI_H

#include "chrono_peridynamics/ChApiPeridynamics.h"

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/physics/ChContactable.h"
#include "chrono/solver/ChVariablesNode.h"

namespace chrono {

// Forward references (for parent hierarchy pointer)
class ChSystem;

namespace peridynamics {

class ChPeridynamics;

/// @addtogroup chrono_peridynamics
/// @{

/// Class for a single node in the Peridynamics  cluster
class ChApiPeridynamics ChNodePeri : public fea::ChNodeFEAxyz, public ChContactable {
  public:
    ChNodePeri();
    ChNodePeri(const ChNodePeri& other);
    ~ChNodePeri();

    /// Get the horizon radius (max. radius while checking surrounding particles).
    double GetHorizonRadius() { return h_rad; }

    /// Set the horizon radius (max. radius while checking surrounding particles).
    void SetHorizonRadius(double mr);

    /// Get collision radius (for colliding with bodies, boundaries, etc.).
    double GetCollisionRadius() { return coll_rad; }
    /// Set collision radius (for colliding with bodies, boundaries, etc.)
    void SetCollisionRadius(double mr);

    /// Get size of volume cell, assumed approx. as a cube. Used for volume correction with fading on horizon, at
    /// quadrature
    double GetVolumeSize() { return vol_size; }
    /// Set size of volume cell, assumed approx. as a cube. Used for volume correction with fading on horizon, at
    /// quadrature
    void SetVolumeSize(double mvol_size) { vol_size = mvol_size; }

    /// Access the variables of the node.
    // virtual ChVariablesNode& Variables() override { return variables; }

    // INTERFACE TO ChContactable

    virtual ChContactable::Type GetContactableType() const override { return ChContactable::Type::ONE_3; }

    virtual ChConstraintTuple* CreateConstraintTuple() override { return new ChConstraintTuple_1vars<3>(&Variables()); }

    /// Tell if the object must be considered in collision detection.
    virtual bool IsContactActive() override { return true; }

    /// Get the number of DOFs affected by this object (position part).
    virtual int GetContactableNumCoordsPosLevel() override { return 3; }

    /// Get the number of DOFs affected by this object (speed part).
    virtual int GetContactableNumCoordsVelLevel() override { return 3; }

    /// Get all the DOFs packed in a single vector (position part).
    virtual void ContactableGetStateBlockPosLevel(ChState& x) override;

    /// Get all the DOFs packed in a single vector (speed part).
    virtual void ContactableGetStateBlockVelLevel(ChStateDelta& w) override;

    /// Increment the provided state of this object by the given state-delta increment.
    /// Compute: x_new = x + dw.
    virtual void ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) override;

    /// Express the local point in absolute frame, for the given state position.
    virtual ChVector3d GetContactPoint(const ChVector3d& loc_point, const ChState& state_x) override;

    /// Get the absolute speed of a local point attached to the contactable.
    /// The given point is assumed to be expressed in the local frame of this object.
    /// This function must use the provided states.
    virtual ChVector3d GetContactPointSpeed(const ChVector3d& loc_point,
                                            const ChState& state_x,
                                            const ChStateDelta& state_w) override;

    /// Get the absolute speed of point abs_point if attached to the surface.
    /// Easy in this case because there are no roations..
    virtual ChVector3d GetContactPointSpeed(const ChVector3d& abs_point) override;

    /// Return the coordinate system for the associated collision model.
    /// ChCollisionModel might call this to get the position of the
    /// contact model (when rigid) and sync it.
    virtual ChFrame<> GetCollisionModelFrame() override;

    /// Apply the force, expressed in absolute reference, applied in pos, to the
    /// coordinates of the variables. Force for example could come from a penalty model.
    /// The force F and its application point are specified in the absolute reference frame.
    virtual void ContactForceLoadResidual_F(const ChVector3d& F,  ///< force
                                            const ChVector3d& T,  ///< torque
                                            const ChVector3d& abs_point,
                                            ChVectorDynamic<>& R) override;

    /// Apply the given force at the given point and load the generalized force array.
    /// The force and its application point are specified in the gloabl frame.
    /// Each object must set the entries in Q corresponding to its variables, starting at the specified offset.
    /// If needed, the object states must be extracted from the provided state position.
    virtual void ContactComputeQ(const ChVector3d& F,  ///< force
                                 const ChVector3d& T,  ///< torque
                                 const ChVector3d& point,
                                 const ChState& state_x,
                                 ChVectorDynamic<>& Q,
                                 int offset) override;

    /// Compute the jacobian(s) part(s) for this contactable item.
    virtual void ComputeJacobianForContactPart(const ChVector3d& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               ChConstraintTuple* jacobian_tuple_N,
                                               ChConstraintTuple* jacobian_tuple_U,
                                               ChConstraintTuple* jacobian_tuple_V,
                                               bool second) override;

    /// Used by some SMC code.
    virtual double GetContactableMass() override { return GetMass(); }

    /// This is only for backward compatibility. OBSOLETE
    virtual ChPhysicsItem* GetPhysicsItem() override { return nullptr; }

  public:
    bool is_requiring_bonds = true;  // requires collision detection to initialize bonds even if is_fluid is false
    bool is_boundary = false;        // always requires collsion detection
    bool is_fluid = true;            // if not fluid, bonds do not require updating via collision detection proximities

    bool is_colliding = false;  // has a collision model that is already inserted in ChSystem collision engine

    bool IsRequiringCollision() { return (is_boundary || is_fluid || is_requiring_bonds); }

    ChVector3d F_peridyn;  // placeholder for storing peridynamics force (not force density), automatically computed

    double volume;
    double h_rad;
    double coll_rad;
    double vol_size;
    double vol_accumulator;
};

/// @} chrono_peridynamics

}  // end namespace peridynamics
}  // end namespace chrono

#endif
