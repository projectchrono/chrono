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

#ifndef CHCONTACTSURFACE_H
#define CHCONTACTSURFACE_H

#include "physics/ChContactable.h"
#include "unit_FEA/ChNodeFEAxyz.h"

#include "physics/ChMaterialSurfaceBase.h"
#include "physics/ChMaterialSurface.h"
#include "unit_FEA/ChElementBase.h"

#include "collision/ChCCollisionModel.h"


namespace chrono {

namespace fea {


// Forward references (for parent hierarchy pointer)
class ChContactSurface;


/// Proxy to FEA nodes, to grant them the features 
/// needed for collision detection.

class ChApiFea ChContactNodeXYZ : 
              public ChContactable_1vars<3>,
              public ChShared {

    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChContactNodeXYZ, ChShared);

public:

    ChContactNodeXYZ(ChNodeFEAxyz* anode = 0, ChContactSurface* acontainer = 0) {
        mnode = anode;
        container = acontainer;
    }

    //
    // FUNCTIONS
    //

        /// Access the FEA node to whom this is is a proxy
    ChNodeFEAxyz* GetNode() {return mnode;} 
        /// Set the FEA node to whom this is a proxy
    void SetNode(ChNodeFEAxyz* mn) {mnode = mn;} 

        /// Get the contact surface container
    ChContactSurface* GetContactSurface() const {return container;}
        /// Set the contact surface container
    void GetContactSurface(ChContactSurface* mc) { container = mc;}


    //
    // INTERFACE TO ChContactable
    //

        /// Access variables
    virtual ChLcpVariables* GetVariables1() {return &mnode->Variables(); }

        /// Tell if the object must be considered in collision detection
    virtual bool IsContactActive() { return true; }

        /// Get the absolute speed of point abs_point if attached to the 
        /// surface. Easy in this case because there are no roations..
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& abs_point) {return this->mnode->pos_dt;};

        /// ChCollisionModel might call this to get the position of the 
        /// contact model (when rigid) and sync it
    virtual ChCoordsys<> GetCsysForCollisionModel() {return ChCoordsys<>(this->mnode->pos, QNULL);}

        /// Apply the force, expressed in absolute reference, applied in pos, to the 
        /// coordinates of the variables. Force for example could come from a penalty model.
    virtual void ContactForceLoadResidual_F(const ChVector<>& F, const ChVector<>& abs_point, 
                                         ChVectorDynamic<>& R);

        /// Compute the jacobian(s) part(s) for this contactable item. For example,
        /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point, ChMatrix33<>& contact_plane, 
                            type_constraint_tuple& jacobian_tuple_N,
                            type_constraint_tuple& jacobian_tuple_U,
                            type_constraint_tuple& jacobian_tuple_V,
                            bool second);

    virtual double GetContactableMass()  {return this->mnode->GetMass();}

    /// Return the pointer to the surface material. 
    virtual ChSharedPtr<ChMaterialSurfaceBase>& GetMaterialSurfaceBase();

    /// This is only for backward compatibility
    virtual ChPhysicsItem* GetPhysicsItem();

private:
    ChNodeFEAxyz* mnode;

    ChContactSurface* container;
};




/// Proxy to FEA triangulated face, to grant them the features 
/// needed for collision detection.

class ChApiFea ChContactTriangleXYZ : public ChContactable_3vars<3,3,3> {

    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI_ROOT(ChContactTriangleXYZ);

public:

    ChContactTriangleXYZ(ChNodeFEAxyz* n1 = 0, ChNodeFEAxyz* n2 = 0, ChNodeFEAxyz* n3 = 0, ChContactSurface* acontainer = 0) {
        mnode1 = n1;
        mnode1 = n2;
        mnode1 = n3;
        container = acontainer;
    }


    //
    // FUNCTIONS
    //

        /// Access the FEA node to whom this is a proxy as triangle vertex
    ChNodeFEAxyz* GetNode1() {return mnode1;} 
        /// Access the FEA node to whom this is a proxy as triangle vertex
    ChNodeFEAxyz* GetNode2() {return mnode2;} 
        /// Access the FEA node to whom this is a proxy as triangle vertex
    ChNodeFEAxyz* GetNode3() {return mnode3;} 

        /// Set the FEA node to whom this is a proxy
    void SetNode1(ChNodeFEAxyz* mn) {mnode1 = mn;} 
        /// Set the FEA node to whom this is a proxy
    void SetNode2(ChNodeFEAxyz* mn) {mnode2 = mn;} 
        /// Set the FEA node to whom this is a proxy
    void SetNode3(ChNodeFEAxyz* mn) {mnode3 = mn;} 

        /// Get the contact surface container
    ChContactSurface* GetContactSurface() const {return container;}
        /// Set the contact surface container
    void GetContactSurface(ChContactSurface* mc) { container = mc;}


    //
    // INTERFACE TO ChContactable
    //

        /// Access variables for node 1
    virtual ChLcpVariables* GetVariables1() {return &mnode1->Variables(); }
        /// Access variables for node 2
    virtual ChLcpVariables* GetVariables2() {return &mnode2->Variables(); }
        /// Access variables for node 3
    virtual ChLcpVariables* GetVariables3() {return &mnode3->Variables(); }

        /// Tell if the object must be considered in collision detection
    virtual bool IsContactActive() { return true; }

        /// Get the absolute speed of point abs_point if attached to the 
        /// surface. Easy in this case because there are no roations..
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& abs_point) {
        return this->mnode1->pos_dt; 
        //***TODO***!!!!!!!!!!!!!!!!!!!!
    }

        /// ChCollisionModel might call this to get the position of the 
        /// contact model (when rigid) and sync it
    virtual ChCoordsys<> GetCsysForCollisionModel() {
        return ChCoordsys<>(this->mnode1->pos, QNULL);
        // (Note that it has little importance since used with ChModelBulletTrimesh that does not need it.)
    }

        /// Apply the force, expressed in absolute reference, applied in pos, to the 
        /// coordinates of the variables. Force for example could come from a penalty model.
    virtual void ContactForceLoadResidual_F(const ChVector<>& F, const ChVector<>& abs_point, 
                                         ChVectorDynamic<>& R) {
        //***TODO***!!!!!!!!!!!!!!!!!!!!
    }

        /// Compute the jacobian(s) part(s) for this contactable item. For example,
        /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point, ChMatrix33<>& contact_plane, 
                            type_constraint_tuple& jacobian_tuple_N,
                            type_constraint_tuple& jacobian_tuple_U,
                            type_constraint_tuple& jacobian_tuple_V,
                            bool second) {
        //***TODO***!!!!!!!!!!!!!!!!!!!!
    }

        /// Might be needed by some DEM models
    virtual double GetContactableMass()  {return this->mnode1->GetMass()+this->mnode2->GetMass()+this->mnode3->GetMass();}

        /// Return the pointer to the surface material. 
    virtual ChSharedPtr<ChMaterialSurfaceBase>& GetMaterialSurfaceBase();

        /// This is only for backward compatibility
    virtual ChPhysicsItem* GetPhysicsItem();

private:
    ChNodeFEAxyz* mnode1;
    ChNodeFEAxyz* mnode2;
    ChNodeFEAxyz* mnode3;

    ChContactSurface* container;
};



/// Class which defines a contact surface for FEA elements.
/// The contact surface references the faces of some elements in a ChMesh, and
/// represents them via proxies (ex. triangles, triange edges, vertexes) that are 
/// of ChContactable type. 
/// The contact surface has a material of ChMaterialSurfaceBase type (DVI material 
/// by default, but it can be also switched to a DEM material, etc, using Set).

class ChApiFea ChContactSurface : public ChShared {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChContactSurface, ChShared);


  public:

    ChContactSurface(ChMesh* parentmesh = 0) {
        // default DVI material
        matsurface = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
        mmesh = parentmesh;
    };
    virtual ~ChContactSurface(){};


    // 
    // FUNCTIONS
    //

        /// Get owner mesh
    ChMesh* GetMesh() {return mmesh;}

        /// Set owner mesh
    void SetMesh(ChMesh* mm) {mmesh = mm;}

        /// Set the material surface for 'boundary contact'
    void SetMaterialSurface(const ChSharedPtr<ChMaterialSurfaceBase>& mnewsurf) { matsurface = mnewsurf; }

        /// Set the material surface for 'boundary contact' 
    virtual ChSharedPtr<ChMaterialSurfaceBase>& GetMaterialSurfaceBase() { return matsurface;}

        // Functions to interface this with ChPhysicsItem container 
	virtual void SurfaceSyncCollisionModels() = 0;
	virtual void SurfaceAddCollisionModelsToSystem(ChSystem* msys) = 0;
	virtual void SurfaceRemoveCollisionModelsFromSystem(ChSystem* msys) = 0;


 private:

    // 
    // DATA
    //


    ChSharedPtr<ChMaterialSurfaceBase> matsurface;  // material for contacts

    ChMesh* mmesh;
};



/// Proxy to FEA nodes for collisions, with spheres associated to nodes, for point-cloud 
/// type of collisions.

class ChApiFea ChContactNodeXYZsphere : public ChContactNodeXYZ {

    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChContactNodeXYZsphere, ChContactNodeXYZ);

public:
    ChContactNodeXYZsphere(ChNodeFEAxyz* anode = 0, ChContactSurface* acontainer = 0);

    virtual ~ChContactNodeXYZsphere(){ delete collision_model;}

    collision::ChCollisionModel* GetCollisionModel() {return  collision_model;}

private:
    collision::ChCollisionModel* collision_model;
};


/// Class which defines a contact surface for FEA elements, where only xyz nodes
/// in the FEA model are used as contact items for the collision detection.
/// Might be an efficient option in case of dense tesselations (but misses the FEAnodes-vs-FEAfaces
/// cases, and misses FEAedge-vs-edges)

class ChApiFea ChContactSurfaceNodeCloud : public ChContactSurface {

    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChContactSurfaceNodeCloud, ChContactSurface);


  public:
    ChContactSurfaceNodeCloud(ChMesh* parentmesh = 0) : 
        ChContactSurface(parentmesh) {};

    virtual ~ChContactSurfaceNodeCloud(){};


    // 
    // FUNCTIONS
    //

    void AddNode(ChSharedPtr< ChNodeFEAxyz > mnode); 


        // Functions to interface this with ChPhysicsItem container 
	virtual void SurfaceSyncCollisionModels(); 
	virtual void SurfaceAddCollisionModelsToSystem(ChSystem* msys); 
	virtual void SurfaceRemoveCollisionModelsFromSystem(ChSystem* msys); 

 private:

    // 
    // DATA
    //

    std::vector<ChSharedPtr<ChContactNodeXYZsphere> > vnodes;     //  nodes

};



}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif
