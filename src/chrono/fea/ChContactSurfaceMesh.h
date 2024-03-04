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

#ifndef CHCONTACTSURFACEMESH_H
#define CHCONTACTSURFACEMESH_H

#include "chrono/fea/ChContactSurface.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/collision/ChCollisionModel.h"
#include "chrono/physics/ChLoaderUV.h"
#include "chrono/utils/ChUtilsGeometry.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_contact
/// @{

/// Contact element of triangular type.
/// Used to 'tessellate' the surface of FEA meshes for collision purposes.
class ChApi ChContactTriangleXYZ : public ChContactable_3vars<3, 3, 3>, public ChLoadableUV {
  public:
    ChContactTriangleXYZ();
    ChContactTriangleXYZ(const std::array<std::shared_ptr<ChNodeFEAxyz>, 3>& nodes,
                         ChContactSurface* container = nullptr);

    /// Set the FEA nodes for which this is a proxy.
    void SetNodes(const std::array<std::shared_ptr<ChNodeFEAxyz>, 3>& nodes) { m_nodes = nodes; }

    /// Set the contact surface container.
    void SetContactSurface(ChContactSurface* container) { m_container = container; }

    /// Set node ownership.
    void SetNodeOwnership(const ChVector<bool>& owns_node) { m_owns_node = owns_node; }

    /// Set edge ownership.
    void SetEdgeOwnership(const ChVector<bool>& owns_edge) { m_owns_edge = owns_edge; }

    /// Acccess the specified FEA node for which this is a proxy.
    std::shared_ptr<ChNodeFEAxyz> GetNode(int i) const { return m_nodes[i]; }

    /// Get the contact surface container.
    ChContactSurface* GetContactSurface() const { return m_container; }

    /// Returns true if the specified node is owned by this triangle.
    bool OwnsNode(int i) const { return m_owns_node[i]; }

    /// Returns true if the specified edge is owned by this triangle.
    bool OwnsEdge(int i) const { return m_owns_edge[i]; }

    // Interface to ChContactable

    virtual ChContactable::eChContactableType GetContactableType() const override { return CONTACTABLE_333; }

    /// Access variables for node 1.
    virtual ChVariables* GetVariables1() override { return &m_nodes[0]->Variables(); }
    /// Access variables for node 2.
    virtual ChVariables* GetVariables2() override { return &m_nodes[1]->Variables(); }
    /// Access variables for node 3.
    virtual ChVariables* GetVariables3() override { return &m_nodes[2]->Variables(); }

    /// Tell if the object must be considered in collision detection.
    virtual bool IsContactActive() override { return true; }

    /// Get the number of DOFs affected by this object (position part).
    virtual int ContactableGet_ndof_x() override { return 9; }

    /// Get the number of DOFs affected by this object (speed part).
    virtual int ContactableGet_ndof_w() override { return 9; }

    /// Get all the DOFs packed in a single vector (position part).
    virtual void ContactableGetStateBlock_x(ChState& x) override;

    /// Get all the DOFs packed in a single vector (speed part).
    virtual void ContactableGetStateBlock_w(ChStateDelta& w) override;

    /// Increment the provided state of this object by the given state-delta increment.
    /// Compute: x_new = x + dw.
    virtual void ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) override;

    /// Express the local point in absolute frame, for the given state position.
    virtual ChVector<> GetContactPoint(const ChVector<>& loc_point, const ChState& state_x) override;

    /// Get the absolute speed of a local point attached to the contactable.
    /// The given point is assumed to be expressed in the local frame of this object.
    /// This function must use the provided states.
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& loc_point,
                                            const ChState& state_x,
                                            const ChStateDelta& state_w) override;

    /// Get the absolute speed of point abs_point if attached to the
    /// surface. Easy in this case because there are no rotations..
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& abs_point) override;

    /// Return the coordinate system for the associated collision model.
    /// ChCollisionModel might call this to get the position of the
    /// contact model (when rigid) and sync it.
    virtual ChCoordsys<> GetCsysForCollisionModel() override { return ChCoordsys<>(VNULL, QUNIT); }

    /// Apply the force & torque, expressed in absolute reference, applied in pos, to the
    /// coordinates of the variables. Force for example could come from a penalty model.
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
                                   int offset) override;

    /// Compute the jacobian(s) part(s) for this contactable item. For example,
    /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point,
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

    // INTERFACE TO ChLoadable

    /// Gets the number of DOFs affected by this element (position part).
    virtual int LoadableGet_ndof_x() override { return 3 * 3; }

    /// Gets the number of DOFs affected by this element (velocity part).
    virtual int LoadableGet_ndof_w() override { return 3 * 3; }

    /// Gets all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override;

    /// Gets all the DOFs packed in a single vector (velocity part).
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override;

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) override;

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual int Get_field_ncoords() override { return 3; }

    /// Get the number of DOFs sub-blocks.
    virtual int GetSubBlocks() override { return 3; }

    /// Get the offset of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(int nblock) override;

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) override { return 3; }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override;

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(int nblock) const override;

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V coordinates of the surface, each ranging in 0..+1 (as IsTriangleIntegrationNeeded() is true)
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in surface
                           const double V,              ///< parametric coordinate in surface
                           ChVectorDynamic<>& Qi,       ///< Return result of Q = N'*F  here
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is =n. field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override;

    /// Gets the normal to the surface at the parametric coordinate U,V.
    /// Each coordinate ranging in -1..+1.
    virtual ChVector<> ComputeNormal(const double U, const double V) override;

    /// If true, use quadrature over u,v in [0..1] range as triangle volumetric coords.
    virtual bool IsTriangleIntegrationNeeded() override { return true; }

    /// Compute u,v of contact point respect to triangle.
    /// - u is in the node1->node2 direction.
    /// - v is in the node1->node3 direction.
    void ComputeUVfromP(const ChVector<> P, double& u, double& v);

  private:
    std::array<std::shared_ptr<ChNodeFEAxyz>, 3> m_nodes;
    ChVector<bool> m_owns_node;
    ChVector<bool> m_owns_edge;

    ChContactSurface* m_container;
};

// -----------------------------------------------------------------------------

/// Contact element of triangular type - version for triangles where the nodes are of ChNodeFEAxyzrot type.
/// Used to 'tessellate' a generic surface like the outer of tetrahedral meshes.
class ChApi ChContactTriangleXYZROT : public ChContactable_3vars<6, 6, 6>, public ChLoadableUV {
  public:
    ChContactTriangleXYZROT();
    ChContactTriangleXYZROT(const std::array<std::shared_ptr<ChNodeFEAxyzrot>, 3>& nodes,
                            ChContactSurface* container = nullptr);

    /// Set the FEA nodes for which this is a proxy.
    void SetNodes(const std::array<std::shared_ptr<ChNodeFEAxyzrot>, 3>& nodes) { m_nodes = nodes; }

    /// Set the contact surface container.
    void SetContactSurface(ChContactSurface* container) { m_container = container; }

    /// Set node ownership.
    void SetNodeOwnership(const ChVector<bool>& owns_node) { m_owns_node = owns_node; }

    /// Set edge ownership.
    void SetEdgeOwnership(const ChVector<bool>& owns_edge) { m_owns_edge = owns_edge; }

    /// Acccess the specified FEA node for which this is a proxy.
    std::shared_ptr<ChNodeFEAxyzrot> GetNode(int i) const { return m_nodes[i]; }

    /// Get the contact surface container.
    ChContactSurface* GetContactSurface() const { return m_container; }

    /// Returns true if the specified node is owned by this triangle.
    bool OwnsNode(int i) const { return m_owns_node[i]; }

    /// Returns true if the specified edge is owned by this triangle.
    bool OwnsEdge(int i) const { return m_owns_edge[i]; }

    // Interface to ChContactable

    virtual ChContactable::eChContactableType GetContactableType() const override { return CONTACTABLE_666; }

    /// Access variables for node 1.
    virtual ChVariables* GetVariables1() override { return &m_nodes[0]->Variables(); }
    /// Access variables for node 2.
    virtual ChVariables* GetVariables2() override { return &m_nodes[1]->Variables(); }
    /// Access variables for node 3.
    virtual ChVariables* GetVariables3() override { return &m_nodes[2]->Variables(); }

    /// Tell if the object must be considered in collision detection.
    virtual bool IsContactActive() override { return true; }

    /// Get the number of DOFs affected by this object (position part).
    virtual int ContactableGet_ndof_x() override { return 21; }

    /// Get the number of DOFs affected by this object (speed part).
    virtual int ContactableGet_ndof_w() override { return 18; }

    /// Get all the DOFs packed in a single vector (position part).
    virtual void ContactableGetStateBlock_x(ChState& x) override;

    /// Get all the DOFs packed in a single vector (speed part).
    virtual void ContactableGetStateBlock_w(ChStateDelta& w) override;

    /// Increment the provided state of this object by the given state-delta increment.
    /// Compute: x_new = x + dw.
    virtual void ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) override;

    /// Express the local point in absolute frame, for the given state position.
    virtual ChVector<> GetContactPoint(const ChVector<>& loc_point, const ChState& state_x) override;

    /// Get the absolute speed of a local point attached to the contactable.
    /// The given point is assumed to be expressed in the local frame of this object.
    /// This function must use the provided states.
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& loc_point,
                                            const ChState& state_x,
                                            const ChStateDelta& state_w) override;

    /// Get the absolute speed of point abs_point if attached to the
    /// surface. Easy in this case because there are no rotations..
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& abs_point) override;

    /// Return the coordinate system for the associated collision model.
    /// ChCollisionModel might call this to get the position of the
    /// contact model (when rigid) and sync it.
    virtual ChCoordsys<> GetCsysForCollisionModel() override { return ChCoordsys<>(VNULL, QUNIT); }

    /// Apply the force & torque, expressed in absolute reference, applied in pos, to the
    /// coordinates of the variables. Force for example could come from a penalty model.
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
                                   int offset) override;

    /// Compute the jacobian(s) part(s) for this contactable item. For example,
    /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point,
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

    // INTERFACE TO ChLoadable

    /// Gets the number of DOFs affected by this element (position part).
    virtual int LoadableGet_ndof_x() override { return 3 * 7; }

    /// Gets the number of DOFs affected by this element (velocity part).
    virtual int LoadableGet_ndof_w() override { return 3 * 6; }

    /// Gets all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override;

    /// Gets all the DOFs packed in a single vector (velocity part).
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override;

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) override;

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual int Get_field_ncoords() override { return 6; }

    /// Get the number of DOFs sub-blocks.
    virtual int GetSubBlocks() override { return 3; }

    /// Get the offset of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(int nblock) override;

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) override { return 6; }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(int nblock) const override;

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override;

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V coordinates of the surface, each ranging in 0..+1 (as IsTriangleIntegrationNeeded() is true)
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in surface
                           const double V,              ///< parametric coordinate in surface
                           ChVectorDynamic<>& Qi,       ///< Return result of Q = N'*F  here
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is =n. field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override;

    /// Gets the normal to the surface at the parametric coordinate U,V.
    /// Each coordinate ranging in -1..+1.
    virtual ChVector<> ComputeNormal(const double U, const double V) override;

    /// If true, use quadrature over u,v in [0..1] range as triangle volumetric coords.
    virtual bool IsTriangleIntegrationNeeded() override { return true; }

    /// Compute u,v of contact point respect to triangle.
    /// u is node1->node2 direction,
    /// v is node1->node3 direction
    void ComputeUVfromP(const ChVector<> P, double& u, double& v);

  private:
    std::array<std::shared_ptr<ChNodeFEAxyzrot>, 3> m_nodes;
    ChVector<bool> m_owns_node;
    ChVector<bool> m_owns_edge;

    ChContactSurface* m_container;
};

// -----------------------------------------------------------------------------

/// Contact element of segment type.
/// Used to 'tessellate' FEA meshes with 1-D elements for collision purposes.
class ChApi ChContactSegmentXYZ {
  public:
    ChContactSegmentXYZ() : m_owns_node({true, true}) {}
    ChContactSegmentXYZ(const std::array<std::shared_ptr<ChNodeFEAxyz>, 2>& nodes)
        : m_nodes(nodes), m_owns_node({true, true}) {}

    /// Set the FEA nodes for which this is a proxy.
    void SetNodes(const std::array<std::shared_ptr<ChNodeFEAxyz>, 2>& nodes) { m_nodes = nodes; }

    /// Set node ownership.
    void SetNodeOwnership(const ChVector2<bool>& owns_node) { m_owns_node = owns_node; }

    /// Acccess the specified FEA node for which this is a proxy.
    std::shared_ptr<ChNodeFEAxyz> GetNode(int i) const { return m_nodes[i]; }

    /// Returns true if the specified node is owned by this segment.
    bool OwnsNode(int i) const { return m_owns_node[i]; }

  private:
    std::array<std::shared_ptr<ChNodeFEAxyz>, 2> m_nodes;
    ChVector2<bool> m_owns_node;
};

// -----------------------------------------------------------------------------

/// Class which defines a contact surface for FEA elements, using a mesh of triangles.
/// Differently from ChContactSurfaceNodeCloud, this also captures the node-vs-face and edge-vs-edge cases, but has a
/// higher computational overhead.
class ChApi ChContactSurfaceMesh : public ChContactSurface {
  public:
    ChContactSurfaceMesh(std::shared_ptr<ChMaterialSurface> material, ChMesh* mesh = nullptr);

    virtual ~ChContactSurfaceMesh() {}

    /// Add the face specified by the three specified XYZ nodes to this collision mesh.
    void AddFace(std::shared_ptr<ChNodeFEAxyz> node1,       ///< face node1
                 std::shared_ptr<ChNodeFEAxyz> node2,       ///< face node2
                 std::shared_ptr<ChNodeFEAxyz> node3,       ///< face node3
                 std::shared_ptr<ChNodeFEAxyz> edge_node1,  ///< edge node 1 (nullptr if no wing node)
                 std::shared_ptr<ChNodeFEAxyz> edge_node2,  ///< edge node 2 (nullptr if no wing node)
                 std::shared_ptr<ChNodeFEAxyz> edge_node3,  ///< edge node 3 (nullptr if no wing node)
                 bool owns_node1,                           ///< this collision face owns node1
                 bool owns_node2,                           ///< this collision face owns node2
                 bool owns_node3,                           ///< this collision face owns node3
                 bool owns_edge1,                           ///< this collision face owns edge1
                 bool owns_edge2,                           ///< this collision face owns edge2
                 bool owns_edge3,                           ///< this collision face owns edge3
                 double sphere_swept = 0.0                  ///< thickness (radius of sweeping sphere)
    );

    /// Add the face specified by the three specified XYZROT nodes to this collision mesh.
    void AddFace(std::shared_ptr<ChNodeFEAxyzrot> node1,       ///< face node1
                 std::shared_ptr<ChNodeFEAxyzrot> node2,       ///< face node2
                 std::shared_ptr<ChNodeFEAxyzrot> node3,       ///< face node3
                 std::shared_ptr<ChNodeFEAxyzrot> edge_node1,  ///< edge node 1 (nullptr if no wing node)
                 std::shared_ptr<ChNodeFEAxyzrot> edge_node2,  ///< edge node 2 (nullptr if no wing node)
                 std::shared_ptr<ChNodeFEAxyzrot> edge_node3,  ///< edge node 3 (nullptr if no wing node)
                 bool owns_node1,                              ///< this collision face owns node1
                 bool owns_node2,                              ///< this collision face owns node2
                 bool owns_node3,                              ///< this collision face owns node3
                 bool owns_edge1,                              ///< this collision face owns edge1
                 bool owns_edge2,                              ///< this collision face owns edge2
                 bool owns_edge3,                              ///< this collision face owns edge3
                 double sphere_swept = 0.0                     ///< thickness (radius of sweeping sphere)
    );

    /// Utility function to add all boundary faces of the associated FEA mesh to this collision surface.
    /// This function does nothing if the contact surface was not yet associated with an FEA mesh.
    /// The function scans all the finite elements already added in the parent ChMesh and adds the faces
    /// that are not shared (ie. the faces on the boundary 'skin').
    /// For shells, the argument 'ccw' indicates whether the face vertices are provided in a counter-clockwise (default)
    /// or clockwise order(because shells collisions are oriented and might work only from the "outer" side).
    /// Currently supported elements that generate boundary skin:
    /// - solids:
    ///     - ChElementTetrahedron: all solid tetrahedrons
    ///     - ChElementHexahedron: all solid hexahedrons
    /// - shells:
    ///     - ChElementShellANCF_3423 ANCF: shells (only one side)
    ///     - ChElementShellANCF_3443 ANCF: shells (only one side)
    ///     - ChElementShellANCF_3833 ANCF: shells (only one side)
    ///     - ChElementShellReissner: Reissner 4-nodes shells (only one side)
    /// - beams:
    ///     - ChElementCableANCF: ANCF beams (as sphere-swept lines, i.e. sequence of capsules)
    ///     - ChElementBeamEuler: Euler-Bernoulli beams (as sphere-swept lines, i.e. sequence of capsules)
    void AddFacesFromBoundary(double sphere_swept = 0.0,  ///< radius of swept sphere
                              bool ccw = true             ///< indicate clockwise or counterclockwise vertex ordering
    );

    /// Construct a contact surface from a triangular mesh.
    /// FEA nodes are created at the mesh vertex locations.
    void ConstructFromTrimesh(std::shared_ptr<geometry::ChTriangleMeshConnected> trimesh, double sphere_swept = 0.0);

    /// Get the list of triangles.
    std::vector<std::shared_ptr<ChContactTriangleXYZ>>& GetTriangleList() { return m_faces; }

    /// Get the list of triangles for nodes with rotational dofs.
    std::vector<std::shared_ptr<ChContactTriangleXYZROT>>& GetTriangleListRot() { return m_faces_rot; }

    /// Get the number of triangles.
    unsigned int GetNumTriangles() const { return (unsigned int)(m_faces.size() + m_faces_rot.size()); }

    /// Get the number of vertices.
    unsigned int GetNumVertices() const;

    // Functions to interface this with ChPhysicsItem container
    virtual void SyncCollisionModels() const override;
    virtual void AddCollisionModelsToSystem(ChCollisionSystem* coll_sys) const override;
    virtual void RemoveCollisionModelsFromSystem(ChCollisionSystem* coll_sys) const override;

    /// Utility function for exporting the contact mesh in a pointer-less manner.
    /// The mesh is specified as a set of 3D vertex points (with associated velocities) and a set of faces (indices into
    /// the vertex array). In addition, ownership of nodes and edges among the consitutent triangles is returned in
    /// 'owns_node' and 'owns_edge'.
    void OutputSimpleMesh(std::vector<ChVector<>>& vert_pos,       ///< mesh vertices (absolute xyz positions)
                          std::vector<ChVector<>>& vert_vel,       ///< vertex velocities (absolute xyz velocities)
                          std::vector<ChVector<int>>& triangles,   ///< triangle faces (indices in vertex array)
                          std::vector<ChVector<bool>>& owns_node,  ///< node ownership for each triangular face
                          std::vector<ChVector<bool>>& owns_edge   ///< edge ownership for each triangular face
    ) const;

  private:
    typedef std::array<std::shared_ptr<ChNodeFEAxyz>, 3> NodeTripletXYZ;
    typedef std::array<std::shared_ptr<ChNodeFEAxyzrot>, 3> NodeTripletXYZrot;
    void AddFacesFromTripletsXYZ(const std::vector<NodeTripletXYZ>& triangle_ptrs, double sphere_swept);
    void AddFacesFromTripletsXYZrot(const std::vector<NodeTripletXYZrot>& triangle_ptrs, double sphere_swept);

    std::vector<std::shared_ptr<ChContactTriangleXYZ>> m_faces;         ///< XYZ-node collision faces
    std::vector<std::shared_ptr<ChContactTriangleXYZROT>> m_faces_rot;  ///< XYWROT-node collision faces
};

/// @} fea_contact

}  // end namespace fea
}  // end namespace chrono

#endif
