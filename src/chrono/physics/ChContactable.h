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

#include "chrono/solver/ChConstraintTuple.h"
#include "chrono/physics/ChMaterialSurface.h"
#include "chrono/core/ChVectorDynamic.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/timestepper/ChState.h"

namespace chrono {

/// Forward definition (not needed in MSVC!?)
class type_constraint_tuple;
class ChPhysicsItem;

/// Interface for objects that generate contacts
/// One should inherit from ChContactable_1vars, ChContactable_2vars  etc. depending
/// on the number of ChVariable objects contained in the object (i.e. the variable chunks
/// to whom the contact point position depends, also the variables affected by contact force).
class ChContactable {
  public:
    /// Tell if the object must be considered in collision detection
    virtual bool IsContactActive() = 0;

    /// Get the number of DOFs affected by this object (position part)
    virtual int ContactableGet_ndof_x() = 0;

    /// Get the number of DOFs affected by this object (speed part)
    virtual int ContactableGet_ndof_w() = 0;

    /// Get all the DOFs packed in a single vector (position part)
    virtual void ContactableGetStateBlock_x(ChState& x) = 0;

    /// Get all the DOFs packed in a single vector (speed part)
    virtual void ContactableGetStateBlock_w(ChStateDelta& w) = 0;

    /// Increment the provided state of this object by the given state-delta increment.
    /// Compute: x_new = x + dw.
    virtual void ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) = 0;

    /// Return the pointer to the surface material.
    /// This function returns a reference to the shared pointer member variable and is therefore THREAD SAFE.
    virtual std::shared_ptr<ChMaterialSurface>& GetMaterialSurface() = 0;

    /// Express the local point in absolute frame, for the given state position.
    virtual ChVector<> GetContactPoint(const ChVector<>& loc_point, const ChState& state_x) = 0;

    /// Get the absolute speed of a local point attached to the contactable.
    /// The given point is assumed to be expressed in the local frame of this object.
    /// This function must use the provided states.  
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& loc_point, const ChState& state_x, const ChStateDelta& state_w) = 0;

    /// Get the absolute speed of point abs_point if attached to the surface.
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& abs_point) = 0;

    /// Return the coordinate system for the associated collision model.
    /// ChCollisionModel might call this to get the position of the
    /// contact model (when rigid) and sync it.
    virtual ChCoordsys<> GetCsysForCollisionModel() = 0;

    /// Apply the given force at the given location and load into the global generalized force array.
    /// The force and its application point are given in the absolute reference frame. Each object must 
    /// update the entries in R corresponding to its variables. Force for example could come from a penalty model.
    virtual void ContactForceLoadResidual_F(const ChVector<>& F, const ChVector<>& abs_point, ChVectorDynamic<>& R) = 0;

    /// Apply the given force at the given point and load the generalized force array.
    /// The force and its application point are specified in the global frame.
    /// Each object must set the entries in Q corresponding to its variables, starting at the specified offset.
    /// If needed, the object states must be extracted from the provided state position.
    virtual void ContactForceLoadQ(const ChVector<>& F,
                                   const ChVector<>& point,
                                   const ChState& state_x,
                                   ChVectorDynamic<>& Q,
                                   int offset) = 0;

    /// This can be useful in some SMC code:
    virtual double GetContactableMass() = 0;

    /// This is only for backward compatibility. Note that in recent code
    /// the reference to the ChPhysicsItem should disappear.
    /// The ChPhysicsItem could be the ChContactable itself (ex. see the ChBody) or
    /// a container (ex. the ChMEsh, for ChContactTriangle)
    virtual ChPhysicsItem* GetPhysicsItem() = 0;

	// enum used for dispatcher optimization instead than rtti
	enum eChContactableType {
		CONTACTABLE_UNKNOWN = 0,
		CONTACTABLE_6,
		CONTACTABLE_3,
		CONTACTABLE_333,
		CONTACTABLE_666
	};
	/// This must return the proper eChContactableType enum, for allowing
	/// a faster collision dispatcher in ChContactContainer classes (this enum
	/// will be used instead of slow dynamic_cast<> to infer the type of ChContactable,
	/// if possible)
	virtual eChContactableType GetContactableType() const = 0;

};

// Note that template T1 is the number of DOFs in the referenced ChVariable, 
// for instance = 6 for rigid bodies, =3 for ChNodeXYZ, etc.

template <int T1>
class ChContactable_1vars : public ChContactable, public ChVariableTupleCarrier_1vars<T1> {
  public:
    typedef ChVariableTupleCarrier_1vars<T1> type_variable_tuple_carrier;
    typedef typename ChVariableTupleCarrier_1vars<T1>::type_constraint_tuple type_constraint_tuple;

    /// Compute the jacobian(s) part(s) for this contactable item. For example,
    /// if the contactable is a ChBody, this should update the three corresponding 1x6 jacobian rows.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               type_constraint_tuple& jacobian_tuple_N,
                                               type_constraint_tuple& jacobian_tuple_U,
                                               type_constraint_tuple& jacobian_tuple_V,
                                               bool second) = 0;

    /// Compute the jacobian(s) part(s) for this contactable item, for rolling about N,u,v
    /// (used only for rolling friction NSC contacts)
    virtual void ComputeJacobianForRollingContactPart(const ChVector<>& abs_point,
                                                      ChMatrix33<>& contact_plane,
                                                      type_constraint_tuple& jacobian_tuple_N,
                                                      type_constraint_tuple& jacobian_tuple_U,
                                                      type_constraint_tuple& jacobian_tuple_V,
                                                      bool second){};
};

// Note that template T1 and T2 are the number of DOFs in the referenced ChVariable s, 
// for instance 3 and 3 for an 'edge' betweeen two xyz nodes.

template <int T1, int T2>
class ChContactable_2vars : public ChContactable, public ChVariableTupleCarrier_2vars<T1, T2> {
  public:
    typedef ChVariableTupleCarrier_2vars<T1, T2> type_variable_tuple_carrier;
    typedef typename ChVariableTupleCarrier_2vars<T1, T2>::type_constraint_tuple type_constraint_tuple;

    /// Compute the jacobian(s) part(s) for this contactable item. For example,
    /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               type_constraint_tuple& jacobian_tuple_N,
                                               type_constraint_tuple& jacobian_tuple_U,
                                               type_constraint_tuple& jacobian_tuple_V,
                                               bool second) = 0;

    /// Compute the jacobian(s) part(s) for this contactable item, for rolling about N,u,v
    /// (used only for rolling friction NSC contacts)
    virtual void ComputeJacobianForRollingContactPart(const ChVector<>& abs_point,
                                                      ChMatrix33<>& contact_plane,
                                                      type_constraint_tuple& jacobian_tuple_N,
                                                      type_constraint_tuple& jacobian_tuple_U,
                                                      type_constraint_tuple& jacobian_tuple_V,
                                                      bool second){};
};

// Note that template T1 and T2 and T3 are the number of DOFs in the referenced ChVariable s, 
// for instance 3 and 3 and 3 for a 'triangle face' betweeen two xyz nodes.

template <int T1, int T2, int T3>
class ChContactable_3vars : public ChContactable, public ChVariableTupleCarrier_3vars<T1, T2, T3> {
  public:
    typedef ChVariableTupleCarrier_3vars<T1, T2, T3> type_variable_tuple_carrier;
    typedef typename ChVariableTupleCarrier_3vars<T1, T2, T3>::type_constraint_tuple type_constraint_tuple;

    /// Compute the jacobian(s) part(s) for this contactable item. For example,
    /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               type_constraint_tuple& jacobian_tuple_N,
                                               type_constraint_tuple& jacobian_tuple_U,
                                               type_constraint_tuple& jacobian_tuple_V,
                                               bool second) = 0;

    /// Compute the jacobian(s) part(s) for this contactable item, for rolling about N,u,v
    /// (used only for rolling friction NSC contacts)
    virtual void ComputeJacobianForRollingContactPart(const ChVector<>& abs_point,
                                                      ChMatrix33<>& contact_plane,
                                                      type_constraint_tuple& jacobian_tuple_N,
                                                      type_constraint_tuple& jacobian_tuple_U,
                                                      type_constraint_tuple& jacobian_tuple_V,
                                                      bool second){};

};

}  // end namespace chrono

#endif
