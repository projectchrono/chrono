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
#include "chrono/collision/ChCollisionUtils.h"
#include "chrono/physics/ChLoaderUV.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_contact
/// @{

/// Contact element of triangular type.
/// This can be used to 'tessellate' the surface of FEA meshes for collision purposes.
class ChApi ChContactTriangleXYZ : public ChContactable_3vars<3, 3, 3>, public ChLoadableUV {
  public:
    ChContactTriangleXYZ();
    ChContactTriangleXYZ(std::shared_ptr<ChNodeFEAxyz> n1,
                         std::shared_ptr<ChNodeFEAxyz> n2,
                         std::shared_ptr<ChNodeFEAxyz> n3,
                         ChContactSurface* acontainer = 0);

    virtual ~ChContactTriangleXYZ() { delete collision_model; }

    collision::ChCollisionModel* GetCollisionModel() { return collision_model; }

    /// Access the FEA node to whom this is a proxy as triangle vertex
    std::shared_ptr<ChNodeFEAxyz> GetNode1() const { return mnode1; }
    /// Access the FEA node to whom this is a proxy as triangle vertex
    std::shared_ptr<ChNodeFEAxyz> GetNode2() const { return mnode2; }
    /// Access the FEA node to whom this is a proxy as triangle vertex
    std::shared_ptr<ChNodeFEAxyz> GetNode3() const { return mnode3; }

    /// Set the FEA node to whom this is a proxy
    void SetNode1(std::shared_ptr<ChNodeFEAxyz> mn) { mnode1 = mn; }
    /// Set the FEA node to whom this is a proxy
    void SetNode2(std::shared_ptr<ChNodeFEAxyz> mn) { mnode2 = mn; }
    /// Set the FEA node to whom this is a proxy
    void SetNode3(std::shared_ptr<ChNodeFEAxyz> mn) { mnode3 = mn; }

    /// Get the contact surface container
    ChContactSurface* GetContactSurface() const { return container; }
    /// Set the contact surface container
    void SetContactSurface(ChContactSurface* mc) { container = mc; }

    // INTERFACE TO ChContactable

    virtual ChContactable::eChContactableType GetContactableType() const override { return CONTACTABLE_333; }

    /// Access variables for node 1
    virtual ChVariables* GetVariables1() override { return &mnode1->Variables(); }
    /// Access variables for node 2
    virtual ChVariables* GetVariables2() override { return &mnode2->Variables(); }
    /// Access variables for node 3
    virtual ChVariables* GetVariables3() override { return &mnode3->Variables(); }

    /// Tell if the object must be considered in collision detection
    virtual bool IsContactActive() override { return true; }

    /// Get the number of DOFs affected by this object (position part)
    virtual int ContactableGet_ndof_x() override { return 9; }

    /// Get the number of DOFs affected by this object (speed part)
    virtual int ContactableGet_ndof_w() override { return 9; }

    /// Get all the DOFs packed in a single vector (position part)
    virtual void ContactableGetStateBlock_x(ChState& x) override;

    /// Get all the DOFs packed in a single vector (speed part)
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
        //***TODO***!!!!!!!!!!!!!!!!!!!!
        return 1;
    }

    /// This is only for backward compatibility
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

    /// If true, use quadrature over u,v in [0..1] range as triangle volumetric coords
    virtual bool IsTriangleIntegrationNeeded() override { return true; }

  private:
    /// Compute u,v of contact point respect to triangle.
    /// u is node1->node2 direction,
    /// v is node1->node3 direction
    void ComputeUVfromP(const ChVector<> P, double& u, double& v);

    collision::ChCollisionModel* collision_model;

    std::shared_ptr<ChNodeFEAxyz> mnode1;
    std::shared_ptr<ChNodeFEAxyz> mnode2;
    std::shared_ptr<ChNodeFEAxyz> mnode3;

    ChContactSurface* container;
};

// -----------------------------------------------------------------------------

/// Contact element of triangular type - version for triangles where the
/// nodes are of ChNodeFEAxyzrot type.
/// NOTE! if in future we could have ChNodeFEAxyzrot inherited from ChNodeFEAxyz,
/// probably this class would be unnecessary! (Now, it is a bit redundant with ChContactTriangleXYZ)
/// This can be used to 'tessellate' a generic surface like the
/// outer of tetrahedral meshes
class ChApi ChContactTriangleXYZROT : public ChContactable_3vars<6, 6, 6>, public ChLoadableUV {
  public:
    ChContactTriangleXYZROT();
    ChContactTriangleXYZROT(std::shared_ptr<ChNodeFEAxyzrot> n1,
                            std::shared_ptr<ChNodeFEAxyzrot> n2,
                            std::shared_ptr<ChNodeFEAxyzrot> n3,
                            ChContactSurface* acontainer = 0);

    virtual ~ChContactTriangleXYZROT() { delete collision_model; }

    collision::ChCollisionModel* GetCollisionModel() { return collision_model; }

    // FUNCTIONS

    /// Access the FEA node to whom this is a proxy as triangle vertex
    std::shared_ptr<ChNodeFEAxyzrot> GetNode1() const { return mnode1; }
    /// Access the FEA node to whom this is a proxy as triangle vertex
    std::shared_ptr<ChNodeFEAxyzrot> GetNode2() const { return mnode2; }
    /// Access the FEA node to whom this is a proxy as triangle vertex
    std::shared_ptr<ChNodeFEAxyzrot> GetNode3() const { return mnode3; }

    /// Set the FEA node to whom this is a proxy
    void SetNode1(std::shared_ptr<ChNodeFEAxyzrot> mn) { mnode1 = mn; }
    /// Set the FEA node to whom this is a proxy
    void SetNode2(std::shared_ptr<ChNodeFEAxyzrot> mn) { mnode2 = mn; }
    /// Set the FEA node to whom this is a proxy
    void SetNode3(std::shared_ptr<ChNodeFEAxyzrot> mn) { mnode3 = mn; }

    /// Get the contact surface container
    ChContactSurface* GetContactSurface() const { return container; }
    /// Set the contact surface container
    void SetContactSurface(ChContactSurface* mc) { container = mc; }

    // INTERFACE TO ChContactable

    virtual ChContactable::eChContactableType GetContactableType() const override { return CONTACTABLE_666; }

    /// Access variables for node 1
    virtual ChVariables* GetVariables1() override { return &mnode1->Variables(); }
    /// Access variables for node 2
    virtual ChVariables* GetVariables2() override { return &mnode2->Variables(); }
    /// Access variables for node 3
    virtual ChVariables* GetVariables3() override { return &mnode3->Variables(); }

    /// Tell if the object must be considered in collision detection
    virtual bool IsContactActive() override { return true; }

    /// Get the number of DOFs affected by this object (position part)
    virtual int ContactableGet_ndof_x() override { return 21; }

    /// Get the number of DOFs affected by this object (speed part)
    virtual int ContactableGet_ndof_w() override { return 18; }

    /// Get all the DOFs packed in a single vector (position part)
    virtual void ContactableGetStateBlock_x(ChState& x) override;

    /// Get all the DOFs packed in a single vector (speed part)
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
        //***TODO***!!!!!!!!!!!!!!!!!!!!
        return 1;
    }

    /// This is only for backward compatibility
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

    /// If true, use quadrature over u,v in [0..1] range as triangle volumetric coords
    virtual bool IsTriangleIntegrationNeeded() override { return true; }

  private:
    /// Compute u,v of contact point respect to triangle.
    /// u is node1->node2 direction,
    /// v is node1->node3 direction
    void ComputeUVfromP(const ChVector<> P, double& u, double& v);

    collision::ChCollisionModel* collision_model;

    std::shared_ptr<ChNodeFEAxyzrot> mnode1;
    std::shared_ptr<ChNodeFEAxyzrot> mnode2;
    std::shared_ptr<ChNodeFEAxyzrot> mnode3;

    ChContactSurface* container;
};

// -----------------------------------------------------------------------------

/// Class which defines a contact surface for FEA elements, using a mesh of triangles.
/// Differently from ChContactSurfaceNodeCloud, this also captures the FEAnodes-vs-FEAfaces
/// and FEAedge-vs-FEAedges cases, but it has a higher computational overhead
class ChApi ChContactSurfaceMesh : public ChContactSurface {
  public:
    ChContactSurfaceMesh(std::shared_ptr<ChMaterialSurface> material, ChMesh* mesh = nullptr);

    virtual ~ChContactSurfaceMesh() {}

    /// Given a FEA mesh (ex a mesh of tetrahedrons) it finds the faces on the outer boundary.
    /// That is, it scans all the finite elements already added in the parent ChMesh and adds the faces
    /// that are not shared (ie. the faces on the boundary 'skin').
    /// For shells, the argument 'ccw' indicates whether the face vertices are provided in a counter-clockwise (default)
    /// or clockwise order, this has a reason: shells collisions are oriented and might work only from the "outer" side.
    /// Supported elements that generate boundary skin:
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
    /// More will follow in future.
    void AddFacesFromBoundary(double sphere_swept = 0.0,  ///< radius of swept sphere
                              bool ccw = true             ///< indicate clockwise or counterclockwise vertex ordering
    );

    /// As AddFacesFromBoundary, but only for faces containing selected nodes in node_set.
    // void AddFacesFromNodeSet(std::vector<std::shared_ptr<ChNodeFEAbase> >& node_set); ***TODO***

    /// Get the list of triangles.
    std::vector<std::shared_ptr<ChContactTriangleXYZ>>& GetTriangleList() { return vfaces; }

    /// Get the list of triangles for nodes with rotational dofs.
    std::vector<std::shared_ptr<ChContactTriangleXYZROT>>& GetTriangleListRot() { return vfaces_rot; }

    /// Get the number of triangles.
    unsigned int GetNumTriangles() const { return (unsigned int)(vfaces.size() + vfaces_rot.size()); }

    /// Get the number of vertices.
    unsigned int GetNumVertices() const;

    // Functions to interface this with ChPhysicsItem container
    virtual void SurfaceSyncCollisionModels() override;
    virtual void SurfaceAddCollisionModelsToSystem(ChSystem* msys) override;
    virtual void SurfaceRemoveCollisionModelsFromSystem(ChSystem* msys) override;

  private:
    std::vector<std::shared_ptr<ChContactTriangleXYZ>> vfaces;         ///< faces that collide
    std::vector<std::shared_ptr<ChContactTriangleXYZROT>> vfaces_rot;  ///<  faces that collide
};

/// @} fea_contact

}  // end namespace fea
}  // end namespace chrono

#endif
