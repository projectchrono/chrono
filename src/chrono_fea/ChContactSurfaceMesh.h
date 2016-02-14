//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File authors: Alessandro Tasora

#ifndef CHCONTACTSURFACEMESH_H
#define CHCONTACTSURFACEMESH_H

#include "chrono_fea/ChContactSurface.h"
#include "chrono_fea/ChNodeFEAxyz.h"
#include "chrono/collision/ChCCollisionModel.h"
#include "chrono/collision/ChCCollisionUtils.h"

namespace chrono {
namespace fea {

/// Contact element of triangular type.
/// This can be used to 'tesselate' a generic surface like the
/// outer of tetrahedral meshes
class ChApiFea ChContactTriangleXYZ : public ChContactable_3vars<3,3,3> {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI_ROOT(ChContactTriangleXYZ);

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

    /// Access variables for node 1
    virtual ChLcpVariables* GetVariables1() { return &mnode1->Variables(); }
    /// Access variables for node 2
    virtual ChLcpVariables* GetVariables2() { return &mnode2->Variables(); }
    /// Access variables for node 3
    virtual ChLcpVariables* GetVariables3() { return &mnode3->Variables(); }

    /// Tell if the object must be considered in collision detection
    virtual bool IsContactActive() { return true; }

    /// Get the absolute speed of point abs_point if attached to the
    /// surface. Easy in this case because there are no roations..
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& abs_point) {
        double s2, s3;
        this->ComputeUVfromP(abs_point, s2, s3);
        double s1 = 1 - s2 - s3;
        return (s1 * this->mnode1->pos_dt + s2 * this->mnode2->pos_dt + s3 * this->mnode3->pos_dt);
    }

    /// ChCollisionModel might call this to get the position of the
    /// contact model (when rigid) and sync it
    virtual ChCoordsys<> GetCsysForCollisionModel() { return ChCoordsys<>(VNULL, QUNIT); }

    /// Apply the force, expressed in absolute reference, applied in pos, to the
    /// coordinates of the variables. Force for example could come from a penalty model.
    virtual void ContactForceLoadResidual_F(const ChVector<>& F, const ChVector<>& abs_point, ChVectorDynamic<>& R) {
        double s2,s3;
        this->ComputeUVfromP(abs_point,s2,s3);
        double s1=1-s2-s3; 
        R.PasteSumVector(F*s1, this->mnode1->NodeGetOffset_w(), 0);
        R.PasteSumVector(F*s2, this->mnode2->NodeGetOffset_w(), 0);
        R.PasteSumVector(F*s3, this->mnode3->NodeGetOffset_w(), 0);
    }

    /// Compute the jacobian(s) part(s) for this contactable item. For example,
    /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               type_constraint_tuple& jacobian_tuple_N,
                                               type_constraint_tuple& jacobian_tuple_U,
                                               type_constraint_tuple& jacobian_tuple_V,
                                               bool second) {
        //***TODO***!!!!!!!!!!!!!!!!!!!!
    }

    /// Might be needed by some DEM models
    virtual double GetContactableMass() {
        //***TODO***!!!!!!!!!!!!!!!!!!!!
        return 1;
        // this->mnode1->GetMass()+this->mnode2->GetMass()+this->mnode3->GetMass(); // no!! could be zero in nodes of
        // non-lumped-masses meshes!
    }

    /// Return the pointer to the surface material.
    virtual std::shared_ptr<ChMaterialSurfaceBase>& GetMaterialSurfaceBase();

    /// This is only for backward compatibility
    virtual ChPhysicsItem* GetPhysicsItem();

  private:
    /// Compute u,v of contact point respect to triangle.
    /// u is node1->node2 direction,
    /// v is node1->node3 direction
    void ComputeUVfromP(const ChVector<> P, double& u, double& v) {
        double dist;
        int is_into;
        ChVector<> p_projected;
        dist = collision::ChCollisionUtils::PointTriangleDistance(P, mnode1->pos, mnode2->pos, mnode3->pos, u, v, is_into, p_projected);
    }

private:
    collision::ChCollisionModel* collision_model;

    std::shared_ptr<ChNodeFEAxyz> mnode1;
    std::shared_ptr<ChNodeFEAxyz> mnode2;
    std::shared_ptr<ChNodeFEAxyz> mnode3;

    ChContactSurface* container;
};

/// Class which defines a contact surface for FEA elements, using a mesh of triangles.
/// Differently from ChContactSurfaceNodeCloud, this also captures the FEAnodes-vs-FEAfaces
/// and FEAedge-vs-FEAedges cases, but it has a higher computational overhead
class ChApiFea ChContactSurfaceMesh : public ChContactSurface {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChContactSurfaceMesh, ChContactSurface);

  public:
    ChContactSurfaceMesh(ChMesh* parentmesh = 0) : ChContactSurface(parentmesh) {}

    virtual ~ChContactSurfaceMesh() {}

    // 
    // FUNCTIONS
    //

    /// Given a solid mesh (ex a mesh of tetrahetrons) it finds the faces on the outer boundary.
    /// That is, it scans all the finite elements already added in the parent ChMesh and adds the faces
    /// that are not shared (ie. the faces on the boundary 'skin').
    /// Supported solids that generate boundary skin:
    /// - tetrahedrons
    /// - ANCF shells (only one side)
    /// - more will follow in future
    void AddFacesFromBoundary(double sphere_swept = 0.0);

    /// As AddFacesFromBoundary, but only for faces containing selected nodes in node_set.
    //void AddFacesFromNodeSet(std::vector<std::shared_ptr<ChNodeFEAbase> >& node_set); ***TODO***

    std::vector<std::shared_ptr<ChContactTriangleXYZ> >& GetTriangleList() { return vfaces; }

    // Functions to interface this with ChPhysicsItem container
    virtual void SurfaceSyncCollisionModels();
    virtual void SurfaceAddCollisionModelsToSystem(ChSystem* msys);
    virtual void SurfaceRemoveCollisionModelsFromSystem(ChSystem* msys);

  private:
    std::vector<std::shared_ptr<ChContactTriangleXYZ> > vfaces;  //  faces that collide
};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif
