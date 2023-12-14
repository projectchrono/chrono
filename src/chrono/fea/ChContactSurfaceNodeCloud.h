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

#ifndef CHCONTACTSURFACENODECLOUD_H
#define CHCONTACTSURFACENODECLOUD_H

#include "chrono/collision/ChCollisionModel.h"
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
    ChContactNodeXYZ(ChNodeFEAxyz* node = nullptr, ChContactSurface* contact_surface = nullptr);

    /// Access the FEA node to whom this is is a proxy.
    ChNodeFEAxyz* GetNode() { return m_node; }

    /// Set the FEA node to whom this is a proxy.
    void SetNode(ChNodeFEAxyz* node) { m_node = node; }

    /// Get the contact surface container.
    ChContactSurface* GetContactSurface() const { return m_container; }

    /// Set the contact surface container.
    void SetContactSurface(ChContactSurface* contact_surface) { m_container = contact_surface; }

    // INTERFACE TO ChContactable

    virtual ChContactable::eChContactableType GetContactableType() const override { return CONTACTABLE_3; }

    /// Access variables.
    virtual ChVariables* GetVariables1() override { return &m_node->Variables(); }

    /// Tell if the object must be considered in collision detection.
    virtual bool IsContactActive() override { return true; }

    /// Get the number of DOFs affected by this object (position part).
    virtual int ContactableGet_ndof_x() override { return 3; }

    /// Get the number of DOFs affected by this object (speed part).
    virtual int ContactableGet_ndof_w() override { return 3; }

    /// Get all the DOFs packed in a single vector (position part).
    virtual void ContactableGetStateBlock_x(ChState& x) override { x.segment(0, 3) = m_node->pos.eigen(); }

    /// Get all the DOFs packed in a single vector (speed part).
    virtual void ContactableGetStateBlock_w(ChStateDelta& w) override { w.segment(0, 3) = m_node->pos_dt.eigen(); }

    /// Increment the provided state of this object by the given state-delta increment.
    /// Compute: x_new = x + dw.
    virtual void ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) override {
        m_node->NodeIntStateIncrement(0, x_new, x, 0, dw);
    }

    /// Express the local point in absolute frame, for the given state position.
    virtual ChVector<> GetContactPoint(const ChVector<>& loc_point, const ChState& state_x) override {
        return state_x.segment(0, 3);
    }

    /// Get the absolute speed of a local point attached to the contactable.
    /// The given point is assumed to be expressed in the local frame of this object.
    /// This function must use the provided states.
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& loc_point,
                                            const ChState& state_x,
                                            const ChStateDelta& state_w) override {
        return state_w.segment(0, 3);
    }

    /// Get the absolute speed of point abs_point if attached to the surface.
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& abs_point) override { return m_node->pos_dt; }

    /// Return the coordinate system for the associated collision model.
    /// ChCollisionModel might call this to get the position of the contact model (when rigid) and sync it.
    virtual ChCoordsys<> GetCsysForCollisionModel() override { return ChCoordsys<>(m_node->pos, QNULL); }

    /// Apply the force & torque, expressed in absolute reference, to the coordinates of the variables.
    virtual void ContactForceLoadResidual_F(const ChVector<>& F,
                                            const ChVector<>& T,
                                            const ChVector<>& abs_point,
                                            ChVectorDynamic<>& R) override;

    /// Compute a contiguous vector of generalized forces Q from a given force & torque at the given point.
    /// Used for computing stiffness matrix (square force jacobian) by backward differentiation.
    /// The force and its application point are specified in the global frame.
    /// Each object must set the entries in Q corresponding to its variables, starting at the specified offset.
    /// If needed, the object states must be extracted from the provided state position.
    virtual void ContactComputeQ(const ChVector<>& F,
                                   const ChVector<>& T,
                                   const ChVector<>& point,
                                   const ChState& state_x,
                                   ChVectorDynamic<>& Q,
                                   int offset) override {
        Q.segment(offset, 3) = F.eigen();
    }

    /// Compute the jacobian(s) part(s) for this contactable item.
    /// For example, if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               type_constraint_tuple& jacobian_tuple_N,
                                               type_constraint_tuple& jacobian_tuple_U,
                                               type_constraint_tuple& jacobian_tuple_V,
                                               bool second) override;

    virtual double GetContactableMass() override {
        //***TODO***!!!!!!!!!!!!!!!!!!!!
        return 1;
        // return this->m_node->GetMass(); // no!! could be zero in nodes of non-lumped-masses meshes!
    }

    /// This is only for backward compatibility.
    virtual ChPhysicsItem* GetPhysicsItem() override;

  private:
    ChNodeFEAxyz* m_node;
    ChContactSurface* m_container;
};

/// Proxy to FEA nodes for collisions, with spheres associated to nodes, for point-cloud type of collisions.
class ChApi ChContactNodeXYZsphere : public ChContactNodeXYZ {
  public:
    ChContactNodeXYZsphere(ChNodeFEAxyz* node = nullptr, ChContactSurface* contact_surface = nullptr);
    ~ChContactNodeXYZsphere() {}
};

/// Proxy to FEA nodes with 3 xyz + 3 rot coords, to grant them the features needed for collision detection.
/// **Note: the ChContactNodeXYZ would be sufficient if ChNodeFEAxyz were inherited
/// from ChNodeFEAxyzrot, but this does not happen -hopefully it will be, in future API-, so we need
/// to implement also this ChContactNodeXYZROT as a proxy to ChNodeFEAxyzrot, sorry for code redundancy.
class ChApi ChContactNodeXYZROT : public ChContactable_1vars<6> {
  public:
    ChContactNodeXYZROT(ChNodeFEAxyzrot* node = nullptr, ChContactSurface* contact_surface = nullptr);

    /// Access the FEA node to whom this is is a proxy
    ChNodeFEAxyzrot* GetNode() { return m_node; }

    /// Set the FEA node to whom this is a proxy
    void SetNode(ChNodeFEAxyzrot* node) { m_node = node; }

    /// Get the contact surface container
    ChContactSurface* GetContactSurface() const { return m_container; }

    /// Set the contact surface container
    void SetContactSurface(ChContactSurface* contact_surface) { m_container = contact_surface; }

    // INTERFACE TO ChContactable

    virtual ChContactable::eChContactableType GetContactableType() const override { return CONTACTABLE_6; }

    /// Access variables.
    virtual ChVariables* GetVariables1() override { return &m_node->Variables(); }

    /// Tell if the object must be considered in collision detection.
    virtual bool IsContactActive() override { return true; }

    /// Get the number of DOFs affected by this object (position part).
    virtual int ContactableGet_ndof_x() override { return 7; }

    /// Get the number of DOFs affected by this object (speed part).
    virtual int ContactableGet_ndof_w() override { return 6; }

    /// Get all the DOFs packed in a single vector (position part).
    virtual void ContactableGetStateBlock_x(ChState& x) override { x.segment(0, 3) = m_node->GetPos().eigen(); }

    /// Get all the DOFs packed in a single vector (speed part).
    virtual void ContactableGetStateBlock_w(ChStateDelta& w) override { w.segment(0, 3) = m_node->GetPos_dt().eigen(); }

    /// Increment the provided state of this object by the given state-delta increment.
    /// Compute: x_new = x + dw.
    virtual void ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) override {
        m_node->NodeIntStateIncrement(0, x_new, x, 0, dw);  // no need for angular, assuming contact is centered
    }

    /// Express the local point in absolute frame, for the given state position.
    virtual ChVector<> GetContactPoint(const ChVector<>& loc_point, const ChState& state_x) override {
        return state_x.segment(0, 3);
    }

    /// Get the absolute speed of a local point attached to the contactable.
    /// The given point is assumed to be expressed in the local frame of this object.
    /// This function must use the provided states.
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& loc_point,
                                            const ChState& state_x,
                                            const ChStateDelta& state_w) override {
        return state_w.segment(0, 3);
    }

    /// Get the absolute speed of point abs_point if attached to the surface.
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& abs_point) override { return m_node->GetPos_dt(); }

    /// Return the coordinate system for the associated collision model.
    /// ChCollisionModel might call this to get the position of the contact model (when rigid) and sync it.
    virtual ChCoordsys<> GetCsysForCollisionModel() override { return m_node->GetCoord(); }

    /// Apply the force & torque, expressed in absolute reference, to the coordinates of the variables.
    virtual void ContactForceLoadResidual_F(const ChVector<>& F,
                                            const ChVector<>& T,
                                            const ChVector<>& abs_point,
                                            ChVectorDynamic<>& R) override;

    /// Compute a contiguous vector of generalized forces Q from a given force & torque at the given point.
    /// Used for computing stiffness matrix (square force jacobian) by backward differentiation.
    /// The force and its application point are specified in the global frame.
    /// Each object must set the entries in Q corresponding to its variables, starting at the specified offset.
    /// If needed, the object states must be extracted from the provided state position.
    virtual void ContactComputeQ(const ChVector<>& F,
                                   const ChVector<>& T,
                                   const ChVector<>& point,
                                   const ChState& state_x,
                                   ChVectorDynamic<>& Q,
                                   int offset) override {
        Q.segment(offset, 3) = F.eigen();
    }

    /// Compute the jacobian(s) part(s) for this contactable item.
    /// For example, if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_N,
                                               ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_U,
                                               ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_V,
                                               bool second) override;

    virtual double GetContactableMass() override {
        //***TODO***!!!!!!!!!!!!!!!!!!!!
        return 1;
        // return m_node->GetMass(); // no!! could be zero in nodes of non-lumped-masses meshes!
    }

    /// This is only for backward compatibility.
    virtual ChPhysicsItem* GetPhysicsItem() override;

  private:
    ChNodeFEAxyzrot* m_node;
    ChContactSurface* m_container;
};

/// Proxy to FEA nodes for collisions, with spheres associated to nodes, for point-cloud type of collisions.
class ChApi ChContactNodeXYZROTsphere : public ChContactNodeXYZROT {
  public:
    ChContactNodeXYZROTsphere(ChNodeFEAxyzrot* node = nullptr, ChContactSurface* contact_surface = nullptr);
    ~ChContactNodeXYZROTsphere() {}
};

/// Class which defines a contact surface for FEA elements.
/// Only xyz nodes in the FEA model are used as contact items for the collision detection.
/// Might be an efficient option in case of dense tessellations (but misses the node-vs-face and edge-vs-edge cases)
class ChApi ChContactSurfaceNodeCloud : public ChContactSurface {
  public:
    ChContactSurfaceNodeCloud(std::shared_ptr<ChMaterialSurface> material, ChMesh* mesh = nullptr);

    virtual ~ChContactSurfaceNodeCloud() {}

    /// Add a specific node to this collision cloud.
    void AddNode(std::shared_ptr<ChNodeFEAxyz> node, const double point_radius = 0.001);

    /// Add a specific node to this collision cloud.
    void AddNode(std::shared_ptr<ChNodeFEAxyzrot> node, const double point_radius = 0.001);

    /// Utility function to add all nodes of the associated FEA mesh to this collision cloud.
    /// This function does nothing if the contact surface was not yet associated with an FEA mesh.
    void AddAllNodes(const double point_radius = 0.001);

    /// Utility function to add nodes of the associated mesh belonging to the given node_set, to this collision cloud.
    /// This function does nothing if the contact surface was not yet associated with an FEA mesh.
    void AddNodesFromNodeSet(std::vector<std::shared_ptr<ChNodeFEAbase>>& node_set, const double point_radius = 0.001);

    /// Get the list of nodes.
    std::vector<std::shared_ptr<ChContactNodeXYZsphere>>& GetNodeList() { return m_nodes; }

    /// Get the list of nodes with rotational dofs.
    std::vector<std::shared_ptr<ChContactNodeXYZROTsphere>>& GetNodeListRot() { return m_nodes_rot; }

    /// Get the number of nodes.
    unsigned int GetNnodes() const { return (unsigned int)m_nodes.size(); }

    /// Get the number of nodes with rotational dofs.
    unsigned int GetNnodesRot() const { return (unsigned int)m_nodes_rot.size(); }

    /// Access the n-th node.
    std::shared_ptr<ChContactNodeXYZsphere> GetNode(unsigned int n) { return m_nodes[n]; };

    /// Access the n-th node with rotational dofs.
    std::shared_ptr<ChContactNodeXYZROTsphere> GetNodeRot(unsigned int n) { return m_nodes_rot[n]; };

    // Functions to interface this with ChPhysicsItem container.
    virtual void SyncCollisionModels() const override;
    virtual void AddCollisionModelsToSystem(ChCollisionSystem* coll_sys) const override;
    virtual void RemoveCollisionModelsFromSystem(ChCollisionSystem* coll_sys) const override;

  private:
    std::vector<std::shared_ptr<ChContactNodeXYZsphere>> m_nodes;         //  nodes
    std::vector<std::shared_ptr<ChContactNodeXYZROTsphere>> m_nodes_rot;  //  nodes with rotations
};

/// @} fea_contact

}  // end namespace fea
}  // end namespace chrono

#endif
