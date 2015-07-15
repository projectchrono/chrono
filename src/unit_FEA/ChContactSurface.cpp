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
// File authors: Andrea Favali, Alessandro Tasora


#include "core/ChMath.h"
#include "unit_FEA/ChContactSurface.h"
#include "physics/ChSystem.h"
#include "collision/ChCModelBullet.h"


using namespace std;


namespace chrono 
{
namespace fea
{


//////////////////////////////////////////////////////////////////////////////
////  ChContactNodeXYZ

    
void ChContactNodeXYZ::ContactForceLoadResidual_F(const ChVector<>& F, const ChVector<>& abs_point, 
                                    ChVectorDynamic<>& R) {
    R.PasteSumVector(F, this->mnode->NodeGetOffset_w() + 0, 0);
}

void ChContactNodeXYZ::ComputeJacobianForContactPart(const ChVector<>& abs_point, ChMatrix33<>& contact_plane, 
            type_constraint_tuple& jacobian_tuple_N, 
            type_constraint_tuple& jacobian_tuple_U, 
            type_constraint_tuple& jacobian_tuple_V, 
            bool second) {
    ChMatrix33<> Jx1;

    Jx1.CopyFromMatrixT(contact_plane);
    if (!second)
        Jx1.MatrNeg();

    jacobian_tuple_N.Get_Cq()->PasteClippedMatrix(&Jx1, 0, 0, 1, 3, 0, 0);
    jacobian_tuple_U.Get_Cq()->PasteClippedMatrix(&Jx1, 1, 0, 1, 3, 0, 0);
    jacobian_tuple_V.Get_Cq()->PasteClippedMatrix(&Jx1, 2, 0, 1, 3, 0, 0);
}

ChSharedPtr<ChMaterialSurfaceBase>& ChContactNodeXYZ::GetMaterialSurfaceBase()
{
    return container->GetMaterialSurfaceBase();
}

ChPhysicsItem* ChContactNodeXYZ::GetPhysicsItem()
{
    return (ChPhysicsItem*)container->GetMesh();
}


//////////////////////////////////////////////////////////////////////////////
////  ChContactTriangleXYZ

ChSharedPtr<ChMaterialSurfaceBase>& ChContactTriangleXYZ::GetMaterialSurfaceBase()
{
    return container->GetMaterialSurfaceBase();
}

ChPhysicsItem* ChContactTriangleXYZ::GetPhysicsItem()
{
    return (ChPhysicsItem*)container->GetMesh();
}

//////////////////////////////////////////////////////////////////////////////
////  ChContactNodeXYZsphere

ChContactNodeXYZsphere::ChContactNodeXYZsphere(ChNodeFEAxyz* anode, ChContactSurface* acontainer)
            : ChContactNodeXYZ(anode, acontainer)  {
        this->collision_model = new collision::ChModelBullet;
        this->collision_model->SetContactable(this);

        //this->collision_model->AddSphere(0.001); //***TODO*** avoid magic number.
        //this->collision_model->BuildModel();

    }



//////////////////////////////////////////////////////////////////////////////
////  ChContactSurfaceNodeCloud


void ChContactSurfaceNodeCloud::AddNode(ChSharedPtr< ChNodeFEAxyz > mnode) {

    if (!mnode)
        return;

    ChSharedPtr<ChContactNodeXYZsphere> newp(new ChContactNodeXYZsphere(mnode.get_ptr(), this));
    
    newp->GetCollisionModel()->AddSphere(0.01);  //***TODO*** avoid magic number.
    newp->GetCollisionModel()->BuildModel();    // will also add to system, if collision is on.
    
    this->vnodes.push_back(newp);
}

void ChContactSurfaceNodeCloud::SurfaceSyncCollisionModels() {
    for (unsigned int j = 0; j < vnodes.size(); j++) {
        this->vnodes[j]->GetCollisionModel()->SyncPosition();
    }
}

void ChContactSurfaceNodeCloud::SurfaceAddCollisionModelsToSystem(ChSystem* msys) {
    assert(msys);
    SurfaceSyncCollisionModels();
    for (unsigned int j = 0; j < vnodes.size(); j++) {
        msys->GetCollisionSystem()->Add(this->vnodes[j]->GetCollisionModel());
    }
}

void ChContactSurfaceNodeCloud::SurfaceRemoveCollisionModelsFromSystem(ChSystem* msys) {
    assert(msys);
    for (unsigned int j = 0; j < vnodes.size(); j++) {
        msys->GetCollisionSystem()->Remove(this->vnodes[j]->GetCollisionModel());
    }
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

