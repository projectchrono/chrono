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

#ifndef CH_CONTACT_SURFACE_SEGMENT_SET_H
#define CH_CONTACT_SURFACE_SEGMENT_SET_H

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/fea/ChContactSurface.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/physics/ChLoaderU.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_contact
/// @{

// -----------------------------------------------------------------------------

/// Contact segment for FEA elements that use XYZ nodes.
/// Used to 'tessellate' FEA meshes with 1-D elements for collision purposes.
class ChApi ChContactSegmentXYZ : public ChContactable_2vars<3, 3> {
  public:
    ChContactSegmentXYZ();
    ChContactSegmentXYZ(const std::array<std::shared_ptr<ChNodeFEAxyz>, 2>& nodes,
                        ChContactSurface* container = nullptr);

    /// Set the FEA nodes for which this is a proxy.
    void SetNodes(const std::array<std::shared_ptr<ChNodeFEAxyz>, 2>& nodes) { m_nodes = nodes; }

    /// Set node ownership.
    void SetNodeOwnership(const ChVector2b& owns_node) { m_owns_node = owns_node; }

    /// Acccess the specified FEA node for which this is a proxy.
    std::shared_ptr<ChNodeFEAxyz> GetNode(int i) const { return m_nodes[i]; }

    /// Returns true if the specified node is owned by this segment.
    bool OwnsNode(int i) const { return m_owns_node[i]; }

    /// Get the contact surface container.
    ChContactSurface* GetContactSurface() const { return m_container; }

    /// Set the contact surface container.
    void SetContactSurface(ChContactSurface* contact_surface) { m_container = contact_surface; }

    // Interface to ChContactable

    /// Return the type of contactable (here, a contactable with 2 variables, each with 3 DOFs).
    virtual ChContactable::eChContactableType GetContactableType() const override { return CONTACTABLE_33; }

    /// Access variables for node 1.
    virtual ChVariables* GetVariables1() override { return &m_nodes[0]->Variables(); }

    /// Access variables for node 2.
    virtual ChVariables* GetVariables2() override { return &m_nodes[1]->Variables(); }

    /// Indicate if the object must be considered in collision detection.
    virtual bool IsContactActive() override { return true; }

    /// Get the number of DOFs affected by this object (position part).
    virtual int GetContactableNumCoordsPosLevel() override { return 6; }

    /// Get the number of DOFs affected by this object (speed part).
    virtual int GetContactableNumCoordsVelLevel() override { return 6; }

    /// Get all the DOFs packed in a single vector (position part).
    virtual void ContactableGetStateBlockPosLevel(ChState& x) override;

    /// Get all the DOFs packed in a single vector (speed part).
    virtual void ContactableGetStateBlockVelLevel(ChStateDelta& w) override;

    /// Increment the provided state of this object by the given state-delta increment.
    /// Compute: x_new = x + dw.
    virtual void ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) override;

    /// Express the local point in absolute frame, for the given state position.
    virtual ChVector3d GetContactPoint(const ChVector3d& loc_point, const ChState& state_x) override;

    /// Return the frame of the associated collision model relative to the contactable object.
    virtual ChFrame<> GetCollisionModelFrame() override { return ChFrame<>(VNULL, QUNIT); }

    /// Get the absolute speed of a local point attached to the contactable.
    /// The given point is assumed to be expressed in the local frame of this object.
    /// This function must use the provided states.
    virtual ChVector3d GetContactPointSpeed(const ChVector3d& loc_point,
                                            const ChState& state_x,
                                            const ChStateDelta& state_w) override;

    /// Get the absolute speed of point abs_point if attached to the surface.
    virtual ChVector3d GetContactPointSpeed(const ChVector3d& abs_point) override;

    /// Apply the force & torque, expressed in absolute reference, applied in pos, to the
    /// coordinates of the variables. Force for example could come from a penalty model.
    virtual void ContactForceLoadResidual_F(const ChVector3d& F,
                                            const ChVector3d& T,
                                            const ChVector3d& abs_point,
                                            ChVectorDynamic<>& R) override;

    /// Compute a contiguous vector of generalized forces Q from a given force & torque at the given point.
    /// Used for computing stiffness matrix (square force jacobian) by backward differentiation.
    /// The force and its application point are specified in the global frame.
    /// Each object must set the entries in Q corresponding to its variables, starting at the specified offset.
    /// If needed, the object states must be extracted from the provided state position.
    virtual void ContactComputeQ(const ChVector3d& F,
                                 const ChVector3d& T,
                                 const ChVector3d& point,
                                 const ChState& state_x,
                                 ChVectorDynamic<>& Q,
                                 int offset) override;

    /// Compute the jacobian(s) part(s) for this contactable item. For example,
    /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector3d& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               type_constraint_tuple& jacobian_tuple_N,
                                               type_constraint_tuple& jacobian_tuple_U,
                                               type_constraint_tuple& jacobian_tuple_V,
                                               bool second) override;

    /// Return mass of contactable object.
    virtual double GetContactableMass() override {
        //// TODO
        return 1;
    }

    /// This is only for backward compatibility.
    virtual ChPhysicsItem* GetPhysicsItem() override;

    /// Compute u of given point with respect to the segment.
    /// u is the line parameter of the point projection onto the segment, in the node1->node2 direction, and such that
    /// u=0 indicates that P projects into node1.
    double ComputeUfromP(const ChVector3d& P);

  private:
    std::array<std::shared_ptr<ChNodeFEAxyz>, 2> m_nodes;
    ChVector2b m_owns_node;

    ChContactSurface* m_container;
};

// -----------------------------------------------------------------------------

/// Contact segment for FEA elements that use XYZRot nodes.
/// Used to 'tessellate' FEA meshes with 1-D elements for collision purposes.
class ChApi ChContactSegmentXYZRot : public ChContactable_2vars<3, 3> /*, public ChLoadableU*/ {
  public:
    ChContactSegmentXYZRot();
    ChContactSegmentXYZRot(const std::array<std::shared_ptr<ChNodeFEAxyzrot>, 2>& nodes,
                           ChContactSurface* container = nullptr);

    /// Set the FEA nodes for which this is a proxy.
    void SetNodes(const std::array<std::shared_ptr<ChNodeFEAxyzrot>, 2>& nodes) { m_nodes = nodes; }

    /// Set node ownership.
    void SetNodeOwnership(const ChVector2b& owns_node) { m_owns_node = owns_node; }

    /// Acccess the specified FEA node for which this is a proxy.
    std::shared_ptr<ChNodeFEAxyzrot> GetNode(int i) const { return m_nodes[i]; }

    /// Returns true if the specified node is owned by this segment.
    bool OwnsNode(int i) const { return m_owns_node[i]; }

    /// Get the contact surface container.
    ChContactSurface* GetContactSurface() const { return m_container; }

    /// Set the contact surface container.
    void SetContactSurface(ChContactSurface* contact_surface) { m_container = contact_surface; }

    // Interface to ChContactable

    /// Return the type of contactable (here, a contactable with 2 variables, each with 6 DOFs).
    virtual ChContactable::eChContactableType GetContactableType() const override { return CONTACTABLE_66; }

    /// Access variables for node 1.
    virtual ChVariables* GetVariables1() override { return &m_nodes[0]->Variables(); }

    /// Access variables for node 2.
    virtual ChVariables* GetVariables2() override { return &m_nodes[1]->Variables(); }

    /// Indicate if the object must be considered in collision detection.
    virtual bool IsContactActive() override { return true; }

    /// Get the number of DOFs affected by this object (position part).
    virtual int GetContactableNumCoordsPosLevel() override { return 14; }

    /// Get the number of DOFs affected by this object (speed part).
    virtual int GetContactableNumCoordsVelLevel() override { return 12; }

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
    virtual ChVector3d GetContactPointSpeed(const ChVector3d& abs_point) override;

    /// Return the frame of the associated collision model relative to the contactable object.
    virtual ChFrame<> GetCollisionModelFrame() override { return ChFrame<>(VNULL, QUNIT); }

    /// Apply the force & torque, expressed in absolute reference, applied in pos, to the
    /// coordinates of the variables. Force for example could come from a penalty model.
    virtual void ContactForceLoadResidual_F(const ChVector3d& F,
                                            const ChVector3d& T,
                                            const ChVector3d& abs_point,
                                            ChVectorDynamic<>& R) override;

    /// Compute a contiguous vector of generalized forces Q from a given force & torque at the given point.
    /// Used for computing stiffness matrix (square force jacobian) by backward differentiation.
    /// The force and its application point are specified in the global frame.
    /// Each object must set the entries in Q corresponding to its variables, starting at the specified offset.
    /// If needed, the object states must be extracted from the provided state position.
    virtual void ContactComputeQ(const ChVector3d& F,
                                 const ChVector3d& T,
                                 const ChVector3d& point,
                                 const ChState& state_x,
                                 ChVectorDynamic<>& Q,
                                 int offset) override;

    /// Compute the jacobian(s) part(s) for this contactable item. For example,
    /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector3d& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               type_constraint_tuple& jacobian_tuple_N,
                                               type_constraint_tuple& jacobian_tuple_U,
                                               type_constraint_tuple& jacobian_tuple_V,
                                               bool second) override;

    /// Return mass of contactable object.
    virtual double GetContactableMass() override {
        //// TODO
        return 1;
    }

    /// This is only for backward compatibility.
    virtual ChPhysicsItem* GetPhysicsItem() override;

    /// Compute u of contact point with respect to the segment.
    /// u is the line parameter of the point projection onto the segment, in the node1->node2 direction and such that
    /// u=0 indicates that P projects into node1.
    double ComputeUfromP(const ChVector3d& P);

  private:
    std::array<std::shared_ptr<ChNodeFEAxyzrot>, 2> m_nodes;
    ChVector2b m_owns_node;

    ChContactSurface* m_container;
};

// -----------------------------------------------------------------------------

/// Class which defines a contact surface for FEA elements.
/// Suitable for cable and beam elements, it uses one or more capsules per element (depending on the number of nodes).
class ChApi ChContactSurfaceSegmentSet : public ChContactSurface {
  public:
    ChContactSurfaceSegmentSet(std::shared_ptr<ChContactMaterial> material, ChMesh* mesh = nullptr);

    virtual ~ChContactSurfaceSegmentSet() {}

    /// Add the segment specified by the two given XYZ nodes to this contact surface.
    void AddSegment(std::shared_ptr<ChNodeFEAxyz> node1,  ///< segment node1
                    std::shared_ptr<ChNodeFEAxyz> node2,  ///< segment node2
                    bool owns_node1,                      ///< collision segment owns node1
                    bool owns_node2,                      ///< collision segment owns node2
                    double sphere_swept = 0.0             ///< thickness (radius of sweeping sphere)
    );

    /// Add the segment specified by the two given XYZRot nodes to this contact surface.
    void AddSegment(std::shared_ptr<ChNodeFEAxyzrot> node1,  ///< segment node1
                    std::shared_ptr<ChNodeFEAxyzrot> node2,  ///< segment node2
                    bool owns_node1,                         ///< collision segment owns node1
                    bool owns_node2,                         ///< collision segment owns node2
                    double sphere_swept = 0.0                ///< thickness (radius of sweeping sphere)
    );

    /// Utility function to add segments for all 1D elements of the specified FEA mesh to this collision set.
    void AddAllSegments(const ChMesh& mesh, double sphere_swept = 0.0);

    /// Get the list of segments.
    std::vector<std::shared_ptr<ChContactSegmentXYZ>>& GetSegmentsXYZ() { return m_segments; }

    /// Get the list of segments for nodes with rotational DOFs.
    std::vector<std::shared_ptr<ChContactSegmentXYZRot>>& GetSegmentsXYZRot() { return m_segments_rot; }

    /// Get the total number of segments.
    unsigned int GetNumSegments() const { return (unsigned int)(m_segments.size() + m_segments_rot.size()); }

    // Functions to interface this with ChPhysicsItem container.
    virtual void SyncCollisionModels() const override;
    virtual void AddCollisionModelsToSystem(ChCollisionSystem* coll_sys) const override;
    virtual void RemoveCollisionModelsFromSystem(ChCollisionSystem* coll_sys) const override;

  private:
    std::vector<std::shared_ptr<ChContactSegmentXYZ>> m_segments;         // segments with XYZ nodes
    std::vector<std::shared_ptr<ChContactSegmentXYZRot>> m_segments_rot;  // segments with XYZRot nodes
};

/// @} fea_contact

}  // end namespace fea
}  // end namespace chrono

#endif
