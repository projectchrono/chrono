//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHCONTACTABLE_H
#define CHCONTACTABLE_H

#include "lcp/ChLcpConstraintTuple.h"
#include "physics/ChMaterialSurfaceBase.h"
#include "core/ChVectorDynamic.h"
#include "core/ChMatrix33.h"

namespace chrono {

/// Forward definition (not needed in MSVC!?)
class type_constraint_tuple;
class ChPhysicsItem;

/// Interface for objects that generate contacts
/// One should inherit from ChContactable_1vars, ChContactable_2vars  etc. depending
/// on the number of ChVariable objects contained in the object (i.e. the variable chuncks
/// to whom the contact point position depends, also the variables affected by contact force).

class ChContactable  {
public:
        /// Tell if the object must be considered in collision detection
    virtual bool IsContactActive() = 0;

        /// Return the pointer to the surface material. 
        /// Use dynamic cast to understand if this is a 
        /// ChMaterialSurfaceDEM, ChMaterialSurfaceDVI or others.
        /// This function returns a reference to the shared pointer member
        /// variable and is therefore THREAD SAFE. ///***TODO*** use thread-safe shared ptrs and merge to GetMaterialSurface
    virtual std::shared_ptr<ChMaterialSurfaceBase>& GetMaterialSurfaceBase() =0;

        /// Get the absolute speed of point abs_point if attached to the 
        /// surface.
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& abs_point) = 0;

        /// ChCollisionModel might call this to get the position of the 
        /// contact model (when rigid) and sync it
     virtual ChCoordsys<> GetCsysForCollisionModel() =0;

        /// Apply the force, expressed in absolute reference, applied in pos, to the 
        /// coordinates of the variables. Force for example could come from a penalty model.
    virtual void ContactForceLoadResidual_F(const ChVector<>& F, const ChVector<>& abs_point, 
                                ChVectorDynamic<>& R) = 0;

        /// This can be useful in some DEM code:
    virtual double GetContactableMass() = 0;

        /// This is only for backward compatibility. Note that in recent code
        /// the reference to the ChPhysicsItem should disappear. 
        /// The ChPhysicsItem could be the ChContactable itself (ex. see the ChBody) or 
        /// a container (ex. the ChMEsh, for ChContactTriangle)
    virtual ChPhysicsItem* GetPhysicsItem() = 0;
};


// Note that template T1 is the number of DOFs in the referenced ChVariable, 
// for instance = 6 for rigid bodies, =3 for ChNodeXYZ, etc. 

template <int T1>
class ChContactable_1vars : public ChContactable,  public ChLcpVariableTupleCarrier_1vars<T1> {
public:
    typedef ChLcpVariableTupleCarrier_1vars<T1> type_variable_tuple_carrier;
    typedef typename ChLcpVariableTupleCarrier_1vars<T1>::type_constraint_tuple type_constraint_tuple;

        /// Compute the jacobian(s) part(s) for this contactable item. For example,
        /// if the contactable is a ChBody, this should update the three corresponding 1x6 jacobian rows.
    virtual void ComputeJacobianForContactPart(
                            const ChVector<>& abs_point, 
                            ChMatrix33<>& contact_plane, 
                            type_constraint_tuple& jacobian_tuple_N,
                            type_constraint_tuple& jacobian_tuple_U,
                            type_constraint_tuple& jacobian_tuple_V,
                            bool second) =0;

        /// Compute the jacobian(s) part(s) for this contactable item, for rolling about N,u,v
        /// (used only for rolling friction DVI contacts)
    virtual void ComputeJacobianForRollingContactPart(
                            const ChVector<>& abs_point, 
                            ChMatrix33<>& contact_plane, 
                            type_constraint_tuple& jacobian_tuple_N,
                            type_constraint_tuple& jacobian_tuple_U,
                            type_constraint_tuple& jacobian_tuple_V,
                            bool second) {};
};


// Note that template T1 and T2 are the number of DOFs in the referenced ChVariable s, 
// for instance 3 and 3 for an 'edge' betweeen two xyz nodes. 

template <int T1, int T2>
class ChContactable_2vars : public ChContactable,  public ChLcpVariableTupleCarrier_2vars<T1,T2> {
public:
    typedef ChLcpVariableTupleCarrier_2vars<T1, T2> type_variable_tuple_carrier;
    typedef typename ChLcpVariableTupleCarrier_2vars<T1, T2>::type_constraint_tuple type_constraint_tuple;

        /// Compute the jacobian(s) part(s) for this contactable item. For example,
        /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(
                            const ChVector<>& abs_point, 
                            ChMatrix33<>& contact_plane, 
                            type_constraint_tuple& jacobian_tuple_N,
                            type_constraint_tuple& jacobian_tuple_U,
                            type_constraint_tuple& jacobian_tuple_V,
                            bool second) =0;

        /// Compute the jacobian(s) part(s) for this contactable item, for rolling about N,u,v
        /// (used only for rolling friction DVI contacts)
    virtual void ComputeJacobianForRollingContactPart(
                            const ChVector<>& abs_point, 
                            ChMatrix33<>& contact_plane, 
                            type_constraint_tuple& jacobian_tuple_N,
                            type_constraint_tuple& jacobian_tuple_U,
                            type_constraint_tuple& jacobian_tuple_V,
                            bool second) {};
};


// Note that template T1 and T2 and T3 are the number of DOFs in the referenced ChVariable s, 
// for instance 3 and 3 and 3 for a 'triangle face' betweeen two xyz nodes. 

template <int T1, int T2, int T3>
class ChContactable_3vars : public ChContactable,  public ChLcpVariableTupleCarrier_3vars<T1,T2,T3> {
public:
    typedef ChLcpVariableTupleCarrier_3vars<T1, T2, T3> type_variable_tuple_carrier;
    typedef typename ChLcpVariableTupleCarrier_3vars<T1, T2, T3>::type_constraint_tuple type_constraint_tuple;

        /// Compute the jacobian(s) part(s) for this contactable item. For example,
        /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(
                            const ChVector<>& abs_point, 
                            ChMatrix33<>& contact_plane, 
                            type_constraint_tuple& jacobian_tuple_N, 
                            type_constraint_tuple& jacobian_tuple_U, 
                            type_constraint_tuple& jacobian_tuple_V, 
                            bool second) =0;

        /// Compute the jacobian(s) part(s) for this contactable item, for rolling about N,u,v
        /// (used only for rolling friction DVI contacts)
    virtual void ComputeJacobianForRollingContactPart(
                            const ChVector<>& abs_point, 
                            ChMatrix33<>& contact_plane, 
                            type_constraint_tuple& jacobian_tuple_N,
                            type_constraint_tuple& jacobian_tuple_U,
                            type_constraint_tuple& jacobian_tuple_V,
                            bool second) {};
};




}  // END_OF_NAMESPACE____

#endif  
