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

#ifndef CHCONTACTSURFACENODECLOUD_H
#define CHCONTACTSURFACENODECLOUD_H

#include "chrono/collision/ChCCollisionModel.h"
#include "chrono/fea/ChContactSurface.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_contact
/// @{

/// Proxy to FEA nodes, to grant them the features needed for collision detection.
class ChApi ChContactNodeXYZ : public ChContactable_1vars<3> {

  public:
    ChContactNodeXYZ(ChNodeFEAxyz* anode = 0, ChContactSurface* acontainer = 0) {
        mnode = anode;
        container = acontainer;
    }

    //
    // FUNCTIONS
    //

    /// Access the FEA node to whom this is is a proxy
    ChNodeFEAxyz* GetNode() { return mnode; }
    /// Set the FEA node to whom this is a proxy
    void SetNode(ChNodeFEAxyz* mn) { mnode = mn; }

    /// Get the contact surface container
    ChContactSurface* GetContactSurface() const { return container; }
    /// Set the contact surface container
    void GetContactSurface(ChContactSurface* mc) { container = mc; }

    //
    // INTERFACE TO ChContactable
    //

	virtual ChContactable::eChContactableType GetContactableType() const override { return CONTACTABLE_3; }

    /// Access variables
    virtual ChVariables* GetVariables1() override { return &mnode->Variables(); }

    /// Tell if the object must be considered in collision detection
    virtual bool IsContactActive() override { return true; }

    /// Get the number of DOFs affected by this object (position part)
    virtual int ContactableGet_ndof_x() override { return 3; }

    /// Get the number of DOFs affected by this object (speed part)
    virtual int ContactableGet_ndof_w() override { return 3; }

    /// Get all the DOFs packed in a single vector (position part)
    virtual void ContactableGetStateBlock_x(ChState& x) override { x.PasteVector(this->mnode->pos, 0, 0); }

    /// Get all the DOFs packed in a single vector (speed part)
    virtual void ContactableGetStateBlock_w(ChStateDelta& w) override { w.PasteVector(this->mnode->pos_dt, 0, 0); }

    /// Increment the provided state of this object by the given state-delta increment.
    /// Compute: x_new = x + dw.
    virtual void ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) override {
        this->mnode->NodeIntStateIncrement(0, x_new, x, 0, dw);
    }

    /// Express the local point in absolute frame, for the given state position.
    virtual ChVector<> GetContactPoint(const ChVector<>& loc_point, const ChState& state_x) override {
        return state_x.ClipVector(0, 0);
    }

    /// Get the absolute speed of a local point attached to the contactable.
    /// The given point is assumed to be expressed in the local frame of this object.
    /// This function must use the provided states.
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& loc_point,
                                            const ChState& state_x,
                                            const ChStateDelta& state_w) override {
        return state_w.ClipVector(0, 0);
    }

    /// Get the absolute speed of point abs_point if attached to the
    /// surface. Easy in this case because there are no rotations..
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& abs_point) override { return this->mnode->pos_dt; }

    /// Return the coordinate system for the associated collision model.
    /// ChCollisionModel might call this to get the position of the
    /// contact model (when rigid) and sync it.
    virtual ChCoordsys<> GetCsysForCollisionModel() override { return ChCoordsys<>(this->mnode->pos, QNULL); }

    /// Apply the force, expressed in absolute reference, applied in pos, to the
    /// coordinates of the variables. Force for example could come from a penalty model.
    virtual void ContactForceLoadResidual_F(const ChVector<>& F,
                                            const ChVector<>& abs_point,
                                            ChVectorDynamic<>& R) override;

    /// Apply the given force at the given point and load the generalized force array.
    /// The force and its application point are specified in the global frame.
    /// Each object must set the entries in Q corresponding to its variables, starting at the specified offset.
    /// If needed, the object states must be extracted from the provided state position.
    virtual void ContactForceLoadQ(const ChVector<>& F,
                                   const ChVector<>& point,
                                   const ChState& state_x,
                                   ChVectorDynamic<>& Q,
                                   int offset) override {
        Q.PasteVector(F, offset, 0);
    }

    /// Compute the jacobian(s) part(s) for this contactable item. For example,
    /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               type_constraint_tuple& jacobian_tuple_N,
                                               type_constraint_tuple& jacobian_tuple_U,
                                               type_constraint_tuple& jacobian_tuple_V,
                                               bool second) override;

    virtual double GetContactableMass() override {
        //***TODO***!!!!!!!!!!!!!!!!!!!!
        return 1;
        // return this->mnode->GetMass(); // no!! could be zero in nodes of non-lumped-masses meshes!
    }

    /// Return the pointer to the surface material.
    virtual std::shared_ptr<ChMaterialSurface>& GetMaterialSurface() override;

    /// This is only for backward compatibility
    virtual ChPhysicsItem* GetPhysicsItem() override;

  private:
    ChNodeFEAxyz* mnode;

    ChContactSurface* container;
};

/// Proxy to FEA nodes for collisions, with spheres associated to nodes, for point-cloud
/// type of collisions.
class ChApi ChContactNodeXYZsphere : public ChContactNodeXYZ {

  public:
    ChContactNodeXYZsphere(ChNodeFEAxyz* anode = 0, ChContactSurface* acontainer = 0);

    virtual ~ChContactNodeXYZsphere() { delete collision_model; }

    collision::ChCollisionModel* GetCollisionModel() { return collision_model; }

  private:
    collision::ChCollisionModel* collision_model;
};

/// Proxy to FEA nodes with 3 xyz + 3 rot coords, to grant them the features
/// needed for collision detection.
/// **Note: the ChContactNodeXYZ would be sufficient if ChNodeFEAxyz were inherited
/// from ChNodeFEAxyzrot, but this does not happen -hopefully it will be, in future API-, so we need
/// to implement also this ChContactNodeXYZROT as a proxy to ChNodeFEAxyzrot, sorry for code redundancy.
class ChApi ChContactNodeXYZROT : public ChContactable_1vars<6> {

  public:
    ChContactNodeXYZROT(ChNodeFEAxyzrot* anode = 0, ChContactSurface* acontainer = 0) {
        mnode = anode;
        container = acontainer;
    }

    //
    // FUNCTIONS
    //

    /// Access the FEA node to whom this is is a proxy
    ChNodeFEAxyzrot* GetNode() { return mnode; }
    /// Set the FEA node to whom this is a proxy
    void SetNode(ChNodeFEAxyzrot* mn) { mnode = mn; }

    /// Get the contact surface container
    ChContactSurface* GetContactSurface() const { return container; }
    /// Set the contact surface container
    void GetContactSurface(ChContactSurface* mc) { container = mc; }

    //
    // INTERFACE TO ChContactable
    //

	virtual ChContactable::eChContactableType GetContactableType() const override { return CONTACTABLE_6; }

    /// Access variables
    virtual ChVariables* GetVariables1() override { return &mnode->Variables(); }

    /// Tell if the object must be considered in collision detection
    virtual bool IsContactActive() override { return true; }

    /// Get the number of DOFs affected by this object (position part)
    virtual int ContactableGet_ndof_x() override { return 7; }

    /// Get the number of DOFs affected by this object (speed part)
    virtual int ContactableGet_ndof_w() override { return 6; }

    /// Get all the DOFs packed in a single vector (position part)
    virtual void ContactableGetStateBlock_x(ChState& x) override { x.PasteVector(this->mnode->GetPos(), 0, 0); }

    /// Get all the DOFs packed in a single vector (speed part)
    virtual void ContactableGetStateBlock_w(ChStateDelta& w) override { w.PasteVector(this->mnode->GetPos_dt(), 0, 0); }

    /// Increment the provided state of this object by the given state-delta increment.
    /// Compute: x_new = x + dw.
    virtual void ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) override {
        this->mnode->NodeIntStateIncrement(0, x_new, x, 0, dw);  // no need for angular, assuming contact is centered
    }

    /// Express the local point in absolute frame, for the given state position.
    virtual ChVector<> GetContactPoint(const ChVector<>& loc_point, const ChState& state_x) override {
        return state_x.ClipVector(0, 0);
    }

    /// Get the absolute speed of a local point attached to the contactable.
    /// The given point is assumed to be expressed in the local frame of this object.
    /// This function must use the provided states.
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& loc_point,
                                            const ChState& state_x,
                                            const ChStateDelta& state_w) override {
        return state_w.ClipVector(0, 0);
    }

    /// Get the absolute speed of point abs_point if attached to the
    /// surface. Easy in this case because there are no rotations..
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& abs_point) override { return this->mnode->GetPos_dt(); }

    /// Return the coordinate system for the associated collision model.
    /// ChCollisionModel might call this to get the position of the
    /// contact model (when rigid) and sync it.
    virtual ChCoordsys<> GetCsysForCollisionModel() override { return this->mnode->GetCoord(); }

    /// Apply the force, expressed in absolute reference, applied in pos, to the
    /// coordinates of the variables. Force for example could come from a penalty model.
    virtual void ContactForceLoadResidual_F(const ChVector<>& F,
                                            const ChVector<>& abs_point,
                                            ChVectorDynamic<>& R) override;

    /// Apply the given force at the given point and load the generalized force array.
    /// The force and its application point are specified in the global frame.
    /// Each object must set the entries in Q corresponding to its variables, starting at the specified offset.
    /// If needed, the object states must be extracted from the provided state position.
    virtual void ContactForceLoadQ(const ChVector<>& F,
                                   const ChVector<>& point,
                                   const ChState& state_x,
                                   ChVectorDynamic<>& Q,
                                   int offset) override {
        Q.PasteVector(F, offset, 0);
    }

    /// Compute the jacobian(s) part(s) for this contactable item. For example,
    /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_N,
                                               ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_U,
                                               ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_V,
                                               bool second) override;

    virtual double GetContactableMass() override {
        //***TODO***!!!!!!!!!!!!!!!!!!!!
        return 1;
        // return this->mnode->GetMass(); // no!! could be zero in nodes of non-lumped-masses meshes!
    }

    /// Return the pointer to the surface material.
    virtual std::shared_ptr<ChMaterialSurface>& GetMaterialSurface() override;

    /// This is only for backward compatibility
    virtual ChPhysicsItem* GetPhysicsItem() override;

  private:
    ChNodeFEAxyzrot* mnode;

    ChContactSurface* container;
};

/// Proxy to FEA nodes for collisions, with spheres associated to nodes, for point-cloud
/// type of collisions.
class ChApi ChContactNodeXYZROTsphere : public ChContactNodeXYZROT {

  public:
    ChContactNodeXYZROTsphere(ChNodeFEAxyzrot* anode = 0, ChContactSurface* acontainer = 0);

    virtual ~ChContactNodeXYZROTsphere() { delete collision_model; }

    collision::ChCollisionModel* GetCollisionModel() { return collision_model; }

  private:
    collision::ChCollisionModel* collision_model;
};

/// Class which defines a contact surface for FEA elements, where only xyz nodes
/// in the FEA model are used as contact items for the collision detection.
/// Might be an efficient option in case of dense tessellations (but misses the FEAnodes-vs-FEAfaces
/// cases, and misses FEAedge-vs-edges)
class ChApi ChContactSurfaceNodeCloud : public ChContactSurface {

  public:
    ChContactSurfaceNodeCloud(ChMesh* parentmesh = 0) : ChContactSurface(parentmesh){};

    virtual ~ChContactSurfaceNodeCloud(){};

    //
    // FUNCTIONS
    //

    /// Add a specific node to this collision cloud
    void AddNode(std::shared_ptr<ChNodeFEAxyz> mnode, const double point_radius = 0.001);
    /// Add a specific node to this collision cloud
    void AddNode(std::shared_ptr<ChNodeFEAxyzrot> mnode, const double point_radius = 0.001);

    /// Add all nodes of the mesh to this collision cloud
    void AddAllNodes(const double point_radius = 0.001);

    /// Add nodes of the mesh, belonging to the node_set, to this collision cloud
    void AddFacesFromNodeSet(std::vector<std::shared_ptr<ChNodeFEAbase> >& node_set, const double point_radius = 0.001);

    /// Get the number of nodes.
    unsigned int GetNnodes() const { return (unsigned int)vnodes.size(); }
    /// Get the number of nodes with rotation too
    unsigned int GetNnodesRot() const { return (unsigned int)vnodes_rot.size(); }

    /// Access the N-th node
    std::shared_ptr<ChContactNodeXYZsphere> GetNode(unsigned int n) { return vnodes[n]; };
    /// Access the N-th node with rotation too
    std::shared_ptr<ChContactNodeXYZROTsphere> GetNodeRot(unsigned int n) { return vnodes_rot[n]; };

    // Functions to interface this with ChPhysicsItem container
    virtual void SurfaceSyncCollisionModels();
    virtual void SurfaceAddCollisionModelsToSystem(ChSystem* msys);
    virtual void SurfaceRemoveCollisionModelsFromSystem(ChSystem* msys);

  private:
    std::vector<std::shared_ptr<ChContactNodeXYZsphere> > vnodes;         //  nodes
    std::vector<std::shared_ptr<ChContactNodeXYZROTsphere> > vnodes_rot;  //  nodes with rotations
};

/// @} fea_contact

}  // end namespace fea
}  // end namespace chrono

#endif
