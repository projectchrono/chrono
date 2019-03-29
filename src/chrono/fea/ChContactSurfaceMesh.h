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

#ifndef CHCONTACTSURFACEMESH_H
#define CHCONTACTSURFACEMESH_H

#include "chrono/fea/ChContactSurface.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/collision/ChCCollisionModel.h"
#include "chrono/collision/ChCCollisionUtils.h"
#include "chrono/physics/ChLoaderUV.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_contact
/// @{

/// Contact element of triangular type.
/// This can be used to 'tessellate' a generic surface like the
/// outer of tetrahedral meshes
class ChApi ChContactTriangleXYZ : public ChContactable_3vars<3, 3, 3>, public ChLoadableUV {

  public:
    ChContactTriangleXYZ();
    ChContactTriangleXYZ(std::shared_ptr<ChNodeFEAxyz> n1,
                         std::shared_ptr<ChNodeFEAxyz> n2,
                         std::shared_ptr<ChNodeFEAxyz> n3,
                         ChContactSurface* acontainer = 0);

    virtual ~ChContactTriangleXYZ() { delete collision_model; }

    collision::ChCollisionModel* GetCollisionModel() { return collision_model; }

    //
    // FUNCTIONS
    //

    /// Access the FEA node to whom this is a proxy as triangle vertex
    std::shared_ptr<ChNodeFEAxyz> GetNode1() { return mnode1; }
    /// Access the FEA node to whom this is a proxy as triangle vertex
    std::shared_ptr<ChNodeFEAxyz> GetNode2() { return mnode2; }
    /// Access the FEA node to whom this is a proxy as triangle vertex
    std::shared_ptr<ChNodeFEAxyz> GetNode3() { return mnode3; }

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

    //
    // INTERFACE TO ChContactable
    //

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
    virtual void ContactableGetStateBlock_x(ChState& x) override {
        x.PasteVector(this->mnode1->pos, 0, 0);
        x.PasteVector(this->mnode2->pos, 3, 0);
        x.PasteVector(this->mnode3->pos, 6, 0);
    }

    /// Get all the DOFs packed in a single vector (speed part)
    virtual void ContactableGetStateBlock_w(ChStateDelta& w) override {
        w.PasteVector(this->mnode1->pos_dt, 0, 0);
        w.PasteVector(this->mnode2->pos_dt, 3, 0);
        w.PasteVector(this->mnode3->pos_dt, 6, 0);
    }

    /// Increment the provided state of this object by the given state-delta increment.
    /// Compute: x_new = x + dw.
    virtual void ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) override {
        this->mnode1->NodeIntStateIncrement(0, x_new, x, 0, dw);
        this->mnode2->NodeIntStateIncrement(3, x_new, x, 3, dw);
        this->mnode3->NodeIntStateIncrement(6, x_new, x, 6, dw);
    }

    /// Express the local point in absolute frame, for the given state position.
    virtual ChVector<> GetContactPoint(const ChVector<>& loc_point, const ChState& state_x) override {
        // Note: because the reference coordinate system for a ChcontactTriangleXYZ is the identity,
        // the given point loc_point is actually expressed in the global frame. In this case, we
        // calculate the output point here by assuming that its barycentric coordinates do not change
        // with a change in the states of this object.
        double s2, s3;
        this->ComputeUVfromP(loc_point, s2, s3);
        double s1 = 1 - s2 - s3;

        ChVector<> A1 = state_x.ClipVector(0, 0);
        ChVector<> A2 = state_x.ClipVector(3, 0);
        ChVector<> A3 = state_x.ClipVector(6, 0);

        return s1 * A1 + s2 * A2 + s3 * A3;
    }

    /// Get the absolute speed of a local point attached to the contactable.
    /// The given point is assumed to be expressed in the local frame of this object.
    /// This function must use the provided states.
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& loc_point,
                                            const ChState& state_x,
                                            const ChStateDelta& state_w) override {
        // Note: because the reference coordinate system for a ChcontactTriangleXYZ is the identity,
        // the given point loc_point is actually expressed in the global frame. In this case, we
        // calculate the output point here by assuming that its barycentric coordinates do not change
        // with a change in the states of this object.
        double s2, s3;
        this->ComputeUVfromP(loc_point, s2, s3);
        double s1 = 1 - s2 - s3;

        ChVector<> A1_dt = state_w.ClipVector(0, 0);
        ChVector<> A2_dt = state_w.ClipVector(3, 0);
        ChVector<> A3_dt = state_w.ClipVector(6, 0);

        return s1 * A1_dt + s2 * A2_dt + s3 * A3_dt;
    }

    /// Get the absolute speed of point abs_point if attached to the
    /// surface. Easy in this case because there are no rotations..
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& abs_point) override {
        double s2, s3;
        this->ComputeUVfromP(abs_point, s2, s3);
        double s1 = 1 - s2 - s3;
        return (s1 * this->mnode1->pos_dt + s2 * this->mnode2->pos_dt + s3 * this->mnode3->pos_dt);
    }

    /// Return the coordinate system for the associated collision model.
    /// ChCollisionModel might call this to get the position of the
    /// contact model (when rigid) and sync it.
    virtual ChCoordsys<> GetCsysForCollisionModel() override { return ChCoordsys<>(VNULL, QUNIT); }

    /// Apply the force, expressed in absolute reference, applied in pos, to the
    /// coordinates of the variables. Force for example could come from a penalty model.
    virtual void ContactForceLoadResidual_F(const ChVector<>& F,
                                            const ChVector<>& abs_point,
                                            ChVectorDynamic<>& R) override {
        double s2, s3;
        this->ComputeUVfromP(abs_point, s2, s3);
        double s1 = 1 - s2 - s3;
        R.PasteSumVector(F * s1, this->mnode1->NodeGetOffset_w(), 0);
        R.PasteSumVector(F * s2, this->mnode2->NodeGetOffset_w(), 0);
        R.PasteSumVector(F * s3, this->mnode3->NodeGetOffset_w(), 0);
    }

    /// Apply the given force at the given point and load the generalized force array.
    /// The force and its application point are specified in the global frame.
    /// Each object must set the entries in Q corresponding to its variables, starting at the specified offset.
    /// If needed, the object states must be extracted from the provided state position.
    virtual void ContactForceLoadQ(const ChVector<>& F,
                                   const ChVector<>& point,
                                   const ChState& state_x,
                                   ChVectorDynamic<>& Q,
                                   int offset) override {
        // Calculate barycentric coordinates
        ChVector<> A1 = state_x.ClipVector(0, 0);
        ChVector<> A2 = state_x.ClipVector(3, 0);
        ChVector<> A3 = state_x.ClipVector(6, 0);

        double s2, s3;
        double dist;
        int is_into;
        ChVector<> p_projected;
        dist = collision::ChCollisionUtils::PointTriangleDistance(point, A1, A2, A3, s2, s3, is_into, p_projected);
        double s1 = 1 - s2 - s3;
        Q.PasteVector(F * s1, offset + 0, 0);
        Q.PasteVector(F * s2, offset + 3, 0);
        Q.PasteVector(F * s3, offset + 6, 0);
    }

    /// Compute the jacobian(s) part(s) for this contactable item. For example,
    /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               type_constraint_tuple& jacobian_tuple_N,
                                               type_constraint_tuple& jacobian_tuple_U,
                                               type_constraint_tuple& jacobian_tuple_V,
                                               bool second) override {
        
        // compute the triangular area-parameters s1 s2 s3:
        double s2, s3;
        double dist;
        int is_into;
        ChVector<> p_projected;
        dist = collision::ChCollisionUtils::PointTriangleDistance(abs_point, 
                                    this->GetNode1()->pos, 
                                    this->GetNode2()->pos, 
                                    this->GetNode3()->pos, 
                                    s2, s3, is_into, p_projected);
        double s1 = 1 - s2 - s3;

        ChMatrix33<> Jx1;

        Jx1.CopyFromMatrixT(contact_plane);
        if (!second)
            Jx1.MatrNeg();
        jacobian_tuple_N.Get_Cq_1()->PasteClippedMatrix(Jx1, 0, 0, 1, 3, 0, 0);
        jacobian_tuple_U.Get_Cq_1()->PasteClippedMatrix(Jx1, 1, 0, 1, 3, 0, 0);
        jacobian_tuple_V.Get_Cq_1()->PasteClippedMatrix(Jx1, 2, 0, 1, 3, 0, 0);
        jacobian_tuple_N.Get_Cq_1()->MatrScale(s1);
        jacobian_tuple_U.Get_Cq_1()->MatrScale(s1);
        jacobian_tuple_V.Get_Cq_1()->MatrScale(s1);
        jacobian_tuple_N.Get_Cq_2()->PasteClippedMatrix(Jx1, 0, 0, 1, 3, 0, 0);
        jacobian_tuple_U.Get_Cq_2()->PasteClippedMatrix(Jx1, 1, 0, 1, 3, 0, 0);
        jacobian_tuple_V.Get_Cq_2()->PasteClippedMatrix(Jx1, 2, 0, 1, 3, 0, 0);
        jacobian_tuple_N.Get_Cq_2()->MatrScale(s2);
        jacobian_tuple_U.Get_Cq_2()->MatrScale(s2);
        jacobian_tuple_V.Get_Cq_2()->MatrScale(s2);
        jacobian_tuple_N.Get_Cq_3()->PasteClippedMatrix(Jx1, 0, 0, 1, 3, 0, 0);
        jacobian_tuple_U.Get_Cq_3()->PasteClippedMatrix(Jx1, 1, 0, 1, 3, 0, 0);
        jacobian_tuple_V.Get_Cq_3()->PasteClippedMatrix(Jx1, 2, 0, 1, 3, 0, 0);
        jacobian_tuple_N.Get_Cq_3()->MatrScale(s3);
        jacobian_tuple_U.Get_Cq_3()->MatrScale(s3);
        jacobian_tuple_V.Get_Cq_3()->MatrScale(s3);
    }

    /// Might be needed by some SMC models
    virtual double GetContactableMass() override {
        //***TODO***!!!!!!!!!!!!!!!!!!!!
        return 1;
        // this->mnode1->GetMass()+this->mnode2->GetMass()+this->mnode3->GetMass(); // no!! could be zero in nodes of
        // non-lumped-masses meshes!
    }

    /// Return the pointer to the surface material.
    virtual std::shared_ptr<ChMaterialSurface>& GetMaterialSurface() override;

    /// This is only for backward compatibility
    virtual ChPhysicsItem* GetPhysicsItem() override;

    //
    // INTERFACE TO ChLoadable
    //

    /// Gets the number of DOFs affected by this element (position part).
    virtual int LoadableGet_ndof_x() override { return 3 * 3; }

    /// Gets the number of DOFs affected by this element (velocity part).
    virtual int LoadableGet_ndof_w() override { return 3 * 3; }

    /// Gets all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override;

    /// Gets all the DOFs packed in a single vector (velocity part).
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override;

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv) override;

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual int Get_field_ncoords() override { return 3; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() override { return 3; }

    /// Get the offset of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(int nblock) override {
        if (nblock == 0)
            return this->GetNode1()->NodeGetOffset_w();
        if (nblock == 1)
            return this->GetNode2()->NodeGetOffset_w();
        if (nblock == 2)
            return this->GetNode3()->NodeGetOffset_w();
        return 0;
    }

    /// Get the size of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) override { return 3; }

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
    void ComputeUVfromP(const ChVector<> P, double& u, double& v) {
        double dist;
        int is_into;
        ChVector<> p_projected;
        dist = collision::ChCollisionUtils::PointTriangleDistance(P, mnode1->pos, mnode2->pos, mnode3->pos, u, v,
                                                                  is_into, p_projected);
    }

  private:
    collision::ChCollisionModel* collision_model;

    std::shared_ptr<ChNodeFEAxyz> mnode1;
    std::shared_ptr<ChNodeFEAxyz> mnode2;
    std::shared_ptr<ChNodeFEAxyz> mnode3;

    ChContactSurface* container;
};

///////////////////////////////////////////////////////////////////////////////////

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

    //
    // FUNCTIONS
    //

    /// Access the FEA node to whom this is a proxy as triangle vertex
    std::shared_ptr<ChNodeFEAxyzrot> GetNode1() { return mnode1; }
    /// Access the FEA node to whom this is a proxy as triangle vertex
    std::shared_ptr<ChNodeFEAxyzrot> GetNode2() { return mnode2; }
    /// Access the FEA node to whom this is a proxy as triangle vertex
    std::shared_ptr<ChNodeFEAxyzrot> GetNode3() { return mnode3; }

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

    //
    // INTERFACE TO ChContactable
    //

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
    virtual void ContactableGetStateBlock_x(ChState& x) override {
        x.PasteVector(this->mnode1->GetPos(), 0, 0);
        x.PasteQuaternion(this->mnode1->GetRot(), 3, 0);
        x.PasteVector(this->mnode2->GetPos(), 7, 0);
        x.PasteQuaternion(this->mnode2->GetRot(), 10, 0);
        x.PasteVector(this->mnode3->GetPos(), 14, 0);
        x.PasteQuaternion(this->mnode3->GetRot(), 17, 0);
    }

    /// Get all the DOFs packed in a single vector (speed part)
    virtual void ContactableGetStateBlock_w(ChStateDelta& w) override {
        w.PasteVector(this->mnode1->GetPos_dt(), 0, 0);
        w.PasteVector(this->mnode1->GetWvel_loc(), 3, 0);
        w.PasteVector(this->mnode2->GetPos_dt(), 6, 0);
        w.PasteVector(this->mnode2->GetWvel_loc(), 9, 0);
        w.PasteVector(this->mnode3->GetPos_dt(), 12, 0);
        w.PasteVector(this->mnode3->GetWvel_loc(), 15, 0);
    }

    /// Increment the provided state of this object by the given state-delta increment.
    /// Compute: x_new = x + dw.
    virtual void ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) override {
        this->mnode1->NodeIntStateIncrement(0, x_new, x, 0, dw);
        this->mnode2->NodeIntStateIncrement(7, x_new, x, 6, dw);
        this->mnode3->NodeIntStateIncrement(14, x_new, x, 12, dw);
    }

    /// Express the local point in absolute frame, for the given state position.
    virtual ChVector<> GetContactPoint(const ChVector<>& loc_point, const ChState& state_x) override {
        // Note: because the reference coordinate system for a ChContactTriangleXYZROT is the identity,
        // the given point loc_point is actually expressed in the global frame. In this case, we
        // calculate the output point here by assuming that its barycentric coordinates do not change
        // with a change in the states of this object.
        double s2, s3;
        this->ComputeUVfromP(loc_point, s2, s3);
        double s1 = 1 - s2 - s3;

        ChVector<> A1 = state_x.ClipVector(0, 0);
        ChVector<> A2 = state_x.ClipVector(7, 0);
        ChVector<> A3 = state_x.ClipVector(14, 0);

        return s1 * A1 + s2 * A2 + s3 * A3;
    }

    /// Get the absolute speed of a local point attached to the contactable.
    /// The given point is assumed to be expressed in the local frame of this object.
    /// This function must use the provided states.
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& loc_point,
                                            const ChState& state_x,
                                            const ChStateDelta& state_w) override {
        // Note: because the reference coordinate system for a ChContactTriangleXYZROT is the identity,
        // the given point loc_point is actually expressed in the global frame. In this case, we
        // calculate the output point here by assuming that its barycentric coordinates do not change
        // with a change in the states of this object.
        double s2, s3;
        this->ComputeUVfromP(loc_point, s2, s3);
        double s1 = 1 - s2 - s3;

        ChVector<> A1_dt = state_w.ClipVector(0, 0);
        ChVector<> A2_dt = state_w.ClipVector(6, 0);
        ChVector<> A3_dt = state_w.ClipVector(12, 0);

        return s1 * A1_dt + s2 * A2_dt + s3 * A3_dt;
    }

    /// Get the absolute speed of point abs_point if attached to the
    /// surface. Easy in this case because there are no rotations..
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& abs_point) override {
        double s2, s3;
        this->ComputeUVfromP(abs_point, s2, s3);
        double s1 = 1 - s2 - s3;
        return (s1 * this->mnode1->GetPos_dt() + s2 * this->mnode2->GetPos_dt() + s3 * this->mnode3->GetPos_dt());
    }

    /// Return the coordinate system for the associated collision model.
    /// ChCollisionModel might call this to get the position of the
    /// contact model (when rigid) and sync it.
    virtual ChCoordsys<> GetCsysForCollisionModel() override { return ChCoordsys<>(VNULL, QUNIT); }

    /// Apply the force, expressed in absolute reference, applied in pos, to the
    /// coordinates of the variables. Force for example could come from a penalty model.
    virtual void ContactForceLoadResidual_F(const ChVector<>& F,
                                            const ChVector<>& abs_point,
                                            ChVectorDynamic<>& R) override {
        double s2, s3;
        this->ComputeUVfromP(abs_point, s2, s3);
        double s1 = 1 - s2 - s3;
        R.PasteSumVector(F * s1, this->mnode1->NodeGetOffset_w(), 0);
        R.PasteSumVector(F * s2, this->mnode2->NodeGetOffset_w(), 0);
        R.PasteSumVector(F * s3, this->mnode3->NodeGetOffset_w(), 0);
    }

    /// Apply the given force at the given point and load the generalized force array.
    /// The force and its application point are specified in the global frame.
    /// Each object must set the entries in Q corresponding to its variables, starting at the specified offset.
    /// If needed, the object states must be extracted from the provided state position.
    virtual void ContactForceLoadQ(const ChVector<>& F,
                                   const ChVector<>& point,
                                   const ChState& state_x,
                                   ChVectorDynamic<>& Q,
                                   int offset) override {
        // Calculate barycentric coordinates
        ChVector<> A1 = state_x.ClipVector(0, 0);
        ChVector<> A2 = state_x.ClipVector(7, 0);
        ChVector<> A3 = state_x.ClipVector(14, 0);

        double s2, s3;
        double dist;
        int is_into;
        ChVector<> p_projected;
        dist = collision::ChCollisionUtils::PointTriangleDistance(point, A1, A2, A3, s2, s3, is_into, p_projected);
        double s1 = 1 - s2 - s3;
        Q.PasteVector(F * s1, offset + 0, 0);
        Q.PasteVector(F * s2, offset + 6, 0);
        Q.PasteVector(F * s3, offset + 12, 0);
    }

    /// Compute the jacobian(s) part(s) for this contactable item. For example,
    /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               type_constraint_tuple& jacobian_tuple_N,
                                               type_constraint_tuple& jacobian_tuple_U,
                                               type_constraint_tuple& jacobian_tuple_V,
                                               bool second) override {
        // compute the triangular area-parameters s1 s2 s3:
        double s2, s3;
        double dist;
        int is_into;
        ChVector<> p_projected;
        dist = collision::ChCollisionUtils::PointTriangleDistance(abs_point, 
                                    this->GetNode1()->coord.pos, 
                                    this->GetNode2()->coord.pos, 
                                    this->GetNode3()->coord.pos, 
                                    s2, s3, is_into, p_projected);
        double s1 = 1 - s2 - s3;

        ChMatrix33<> Jx1;

        Jx1.CopyFromMatrixT(contact_plane);
        if (!second)
            Jx1.MatrNeg();
        jacobian_tuple_N.Get_Cq_1()->PasteClippedMatrix(Jx1, 0, 0, 1, 3, 0, 0);
        jacobian_tuple_U.Get_Cq_1()->PasteClippedMatrix(Jx1, 1, 0, 1, 3, 0, 0);
        jacobian_tuple_V.Get_Cq_1()->PasteClippedMatrix(Jx1, 2, 0, 1, 3, 0, 0);
        jacobian_tuple_N.Get_Cq_1()->MatrScale(s1);
        jacobian_tuple_U.Get_Cq_1()->MatrScale(s1);
        jacobian_tuple_V.Get_Cq_1()->MatrScale(s1);
        jacobian_tuple_N.Get_Cq_2()->PasteClippedMatrix(Jx1, 0, 0, 1, 3, 0, 0);
        jacobian_tuple_U.Get_Cq_2()->PasteClippedMatrix(Jx1, 1, 0, 1, 3, 0, 0);
        jacobian_tuple_V.Get_Cq_2()->PasteClippedMatrix(Jx1, 2, 0, 1, 3, 0, 0);
        jacobian_tuple_N.Get_Cq_2()->MatrScale(s2);
        jacobian_tuple_U.Get_Cq_2()->MatrScale(s2);
        jacobian_tuple_V.Get_Cq_2()->MatrScale(s2);
        jacobian_tuple_N.Get_Cq_3()->PasteClippedMatrix(Jx1, 0, 0, 1, 3, 0, 0);
        jacobian_tuple_U.Get_Cq_3()->PasteClippedMatrix(Jx1, 1, 0, 1, 3, 0, 0);
        jacobian_tuple_V.Get_Cq_3()->PasteClippedMatrix(Jx1, 2, 0, 1, 3, 0, 0);
        jacobian_tuple_N.Get_Cq_3()->MatrScale(s3);
        jacobian_tuple_U.Get_Cq_3()->MatrScale(s3);
        jacobian_tuple_V.Get_Cq_3()->MatrScale(s3);
    }

    /// Might be needed by some SMC models
    virtual double GetContactableMass() override {
        //***TODO***!!!!!!!!!!!!!!!!!!!!
        return 1;
        // this->mnode1->GetMass()+this->mnode2->GetMass()+this->mnode3->GetMass(); // no!! could be zero in nodes of
        // non-lumped-masses meshes!
    }

    /// Return the pointer to the surface material.
    virtual std::shared_ptr<ChMaterialSurface>& GetMaterialSurface() override;

    /// This is only for backward compatibility
    virtual ChPhysicsItem* GetPhysicsItem() override;

    //
    // INTERFACE TO ChLoadable
    //

    /// Gets the number of DOFs affected by this element (position part).
    virtual int LoadableGet_ndof_x() override { return 3 * 7; }

    /// Gets the number of DOFs affected by this element (velocity part).
    virtual int LoadableGet_ndof_w() override { return 3 * 6; }

    /// Gets all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override;

    /// Gets all the DOFs packed in a single vector (velocity part).
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override;

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv) override;

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual int Get_field_ncoords() override { return 6; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() override { return 3; }

    /// Get the offset of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(int nblock) override {
        if (nblock == 0)
            return this->GetNode1()->NodeGetOffset_w();
        if (nblock == 1)
            return this->GetNode2()->NodeGetOffset_w();
        if (nblock == 2)
            return this->GetNode3()->NodeGetOffset_w();
        return 0;
    }

    /// Get the size of the i-th sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) override { return 6; }

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
    void ComputeUVfromP(const ChVector<> P, double& u, double& v) {
        double dist;
        int is_into;
        ChVector<> p_projected;
        dist = collision::ChCollisionUtils::PointTriangleDistance(P, mnode1->GetPos(), mnode2->GetPos(),
                                                                  mnode3->GetPos(), u, v, is_into, p_projected);
    }

  private:
    collision::ChCollisionModel* collision_model;

    std::shared_ptr<ChNodeFEAxyzrot> mnode1;
    std::shared_ptr<ChNodeFEAxyzrot> mnode2;
    std::shared_ptr<ChNodeFEAxyzrot> mnode3;

    ChContactSurface* container;
};

////////////////////////////////////////////////////////////////////////////////////

/// Class which defines a contact surface for FEA elements, using a mesh of triangles.
/// Differently from ChContactSurfaceNodeCloud, this also captures the FEAnodes-vs-FEAfaces
/// and FEAedge-vs-FEAedges cases, but it has a higher computational overhead
class ChApi ChContactSurfaceMesh : public ChContactSurface {

  public:
    ChContactSurfaceMesh(ChMesh* parentmesh = 0) : ChContactSurface(parentmesh) {}

    virtual ~ChContactSurfaceMesh() {}

    //
    // FUNCTIONS
    //

    /// Given a FEA mesh (ex a mesh of tetrahedrons) it finds the faces on the outer boundary.
    /// That is, it scans all the finite elements already added in the parent ChMesh and adds the faces
    /// that are not shared (ie. the faces on the boundary 'skin').
    /// For shells, the argument 'ccw' indicates whether the face vertices are provided in a counter-clockwise (default)
    /// or clockwise order, this has a reason: shells collisions are oriented and might work only from the "outer" side.
    /// Supported elements that generate boundary skin:
    /// - solids:
    ///     - ChElementTetra_4: tetrahedrons
    ///     - ChFaceBrick_9: solid hexahedrons
    /// - shells:
    ///     - ChElementShellANCF ANCF: shells (only one side)
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
    std::vector<std::shared_ptr<ChContactTriangleXYZ> >& GetTriangleList() { return vfaces; }
    /// Get the list of triangles, for nodes with rotational dofs too.
    std::vector<std::shared_ptr<ChContactTriangleXYZROT> >& GetTriangleListRot() { return vfaces_rot; }

    /// Get the number of triangles.
    unsigned int GetNumTriangles() const { return (unsigned int)(vfaces.size() + vfaces_rot.size()); }

    /// Get the number of vertices.
    unsigned int GetNumVertices() const;

    // Functions to interface this with ChPhysicsItem container
    virtual void SurfaceSyncCollisionModels();
    virtual void SurfaceAddCollisionModelsToSystem(ChSystem* msys);
    virtual void SurfaceRemoveCollisionModelsFromSystem(ChSystem* msys);

  private:
    std::vector<std::shared_ptr<ChContactTriangleXYZ> > vfaces;  //  faces that collide
    std::vector<std::shared_ptr<ChContactTriangleXYZROT> >
        vfaces_rot;  //  faces that collide (for nodes with rotation too)
};

/// @} fea_contact

}  // end namespace fea
}  // end namespace chrono

#endif
