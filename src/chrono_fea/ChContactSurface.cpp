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
#include "chrono_fea/ChContactSurface.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChElementTetra_4.h"
#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChFaceTetra_4.h"
#include "physics/ChSystem.h"
#include "collision/ChCModelBullet.h"
#include <unordered_map>
#include <map>
#include <set>
#include <array>
#include <algorithm>

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

ChContactTriangleXYZ::ChContactTriangleXYZ(ChNodeFEAxyz* n1, ChNodeFEAxyz* n2, ChNodeFEAxyz* n3, ChContactSurface* acontainer) {
        mnode1 = n1;
        mnode1 = n2;
        mnode1 = n3;
        container = acontainer;

        this->collision_model = new collision::ChModelBullet;
        this->collision_model->SetContactable(this);
}


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

        //this->collision_model->AddPoint(0.001); //***TODO*** avoid magic number.
        //this->collision_model->BuildModel();

    }



//////////////////////////////////////////////////////////////////////////////
////  ChContactSurfaceNodeCloud


void ChContactSurfaceNodeCloud::AddNode(ChSharedPtr< ChNodeFEAxyz > mnode, const double point_radius) {

    if (!mnode)
        return;

    ChSharedPtr<ChContactNodeXYZsphere> newp(new ChContactNodeXYZsphere(mnode.get_ptr(), this));
    
    newp->GetCollisionModel()->AddPoint(point_radius);
    newp->GetCollisionModel()->BuildModel();    // will also add to system, if collision is on.
    
    this->vnodes.push_back(newp);
}

/// Add all nodes of the mesh to this collision cloud
void ChContactSurfaceNodeCloud::AddAllNodes(const double point_radius) {
    if (!this->GetMesh())
        return;
    for (unsigned int i = 0; i< this->GetMesh()->GetNnodes(); ++i)
        this->AddNode( this->GetMesh()->GetNode(i).DynamicCastTo<ChNodeFEAxyz>(), point_radius);
}

        /// Add nodes of the mesh, belonging to the node_set, to this collision cloud
void ChContactSurfaceNodeCloud::AddFacesFromNodeSet( std::vector<ChSharedPtr<ChNodeFEAbase> >& node_set, const double point_radius) {
    if (!this->GetMesh())
        return;
    for (unsigned int i = 0; i< node_set.size(); ++i)
        this->AddNode( node_set[i].DynamicCastTo<ChNodeFEAxyz>(), point_radius);
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


//////////////////////////////////////////////////////////////////////////////
////  ChContactSurfaceGeneric


void ChContactSurfaceGeneric::AddFacesFromBoundary(double sphere_swept) {
    
    std::vector< std::array<ChNodeFEAxyz*,3> > triangles;

    ///
    /// Case1. Outer skin boundary of meshes of TETRAHEDRONS:
    ///

    std::multimap< std::array<ChNodeFEAxyz*, 3> , ChFaceTetra_4> face_map;

    for (unsigned int ie= 0; ie< this->mmesh->GetNelements(); ++ie) {
        if (ChSharedPtr<ChElementTetra_4> mtetra = mmesh->GetElement(ie).DynamicCastTo<ChElementTetra_4>()) {
            for (int nface = 0; nface<4; ++nface) {
                ChFaceTetra_4 mface(mtetra, nface);
                std::array<ChNodeFEAxyz*, 3> mface_key = {mface.GetNodeN(0).get_ptr(), mface.GetNodeN(1).get_ptr(), mface.GetNodeN(2).get_ptr()};
                std::sort(mface_key.begin(), mface_key.end());
                face_map.insert( {mface_key, mface} );
            }
        }
    }
    for (unsigned int ie= 0; ie< this->mmesh->GetNelements(); ++ie) {
        if (ChSharedPtr<ChElementTetra_4> mtetra = mmesh->GetElement(ie).DynamicCastTo<ChElementTetra_4>()) {
            for (int nface = 0; nface<4; ++nface) {
                ChFaceTetra_4 mface(mtetra, nface);
                std::array<ChNodeFEAxyz*, 3> mface_key = {mface.GetNodeN(0).get_ptr(), mface.GetNodeN(1).get_ptr(), mface.GetNodeN(2).get_ptr()};
                std::sort(mface_key.begin(), mface_key.end());
                if (face_map.count(mface_key) == 1) {  
                    // Found a face that is not shared.. so it is a boundary face. 
                    triangles.push_back( {mface.GetNodeN(0).get_ptr(), mface.GetNodeN(1).get_ptr(), mface.GetNodeN(2).get_ptr()} );
                }
            }
        }
    }

    ///
    /// Case2. skin of ANCF SHELLS:
    ///
    for (unsigned int ie= 0; ie< this->mmesh->GetNelements(); ++ie) {
        if (ChSharedPtr<ChElementShellANCF> mshell = mmesh->GetElement(ie).DynamicCastTo<ChElementShellANCF>()) {
            ChNodeFEAxyz* nA = mshell->GetNodeA().get_ptr();
            ChNodeFEAxyz* nB = mshell->GetNodeB().get_ptr();
            ChNodeFEAxyz* nC = mshell->GetNodeC().get_ptr();
            ChNodeFEAxyz* nD = mshell->GetNodeD().get_ptr();
            std::array<ChNodeFEAxyz*,3> tri1 = {nA,nB,nC};
            std::array<ChNodeFEAxyz*,3> tri2 = {nA,nC,nD};
            triangles.push_back( tri1 );
            triangles.push_back( tri2 );
        }
    }


    // Compute triangles connectivity 

    std::multimap< std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>, int> edge_map;
        
    for (int it = 0; it<  triangles.size(); ++it) {
        // edges = pairs of vertexes indexes
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeA( triangles[it][0],  triangles[it][1]);
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeB( triangles[it][1],  triangles[it][2]);
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeC( triangles[it][2],  triangles[it][0]);
        // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
        if (medgeA.first>medgeA.second) 
            medgeA = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeA.second, medgeA.first);
        if (medgeB.first>medgeB.second) 
            medgeB = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeB.second, medgeB.first);
        if (medgeC.first>medgeC.second) 
            medgeC = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeC.second, medgeC.first);
        edge_map.insert( { medgeA , it} );
        edge_map.insert( { medgeB , it} );
        edge_map.insert( { medgeC , it} );
    }

    // Create a map of neighbouring triangles, vector of:
    // [Ti TieA TieB TieC]
    std::vector<std::array<int, 4>> tri_map;
    tri_map.resize(triangles.size());

    for (int it = 0; it< triangles.size(); ++it) {
        tri_map[it][0]=it;
        tri_map[it][1]=-1; // default no neighbour
        tri_map[it][2]=-1; // default no neighbour
        tri_map[it][3]=-1; // default no neighbour
        // edges = pairs of vertexes indexes
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeA( triangles[it][0],  triangles[it][1]);
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeB( triangles[it][1],  triangles[it][2]);
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeC( triangles[it][2],  triangles[it][0]);
        // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
        if (medgeA.first>medgeA.second) 
            medgeA = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeA.second, medgeA.first);
        if (medgeB.first>medgeB.second) 
            medgeB = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeB.second, medgeB.first);
        if (medgeC.first>medgeC.second) 
            medgeC = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeC.second, medgeC.first);
        auto retA = edge_map.equal_range(medgeA);
        for (auto fedge=retA.first; fedge!=retA.second; ++fedge) {
            if (fedge->second != it) {
                tri_map[it][1]=fedge->second;
                break;
            }
        }
        auto retB = edge_map.equal_range(medgeB);
        for (auto fedge=retB.first; fedge!=retB.second; ++fedge) {
            if (fedge->second != it) {
                tri_map[it][2]=fedge->second;
                break;
            }
        }
        auto retC = edge_map.equal_range(medgeC);
        for (auto fedge=retC.first; fedge!=retC.second; ++fedge) {
            if (fedge->second != it) {
                tri_map[it][3]=fedge->second;
                break;
            }
        }  
    }

    std::map<std::pair<ChNodeFEAxyz*,ChNodeFEAxyz*>, std::pair<int,int>> winged_edges;
    bool allow_single_wing = true;

    for ( auto aedge = edge_map.begin(); aedge != edge_map.end(); ++aedge ) {
        auto ret = edge_map.equal_range(aedge->first);
        int nt=0;
        std::pair<ChNodeFEAxyz*,ChNodeFEAxyz*> wingedge;
        std::pair<int,int> wingtri;
        wingtri.first = -1;
        wingtri.second = -1;
        for (auto fedge=ret.first; fedge!=ret.second; ++fedge) {
            if (fedge->second == -1)
                break;
            wingedge.first = fedge->first.first;
            wingedge.second= fedge->first.second;
            if (nt==0) 
                wingtri.first  = fedge->second;
            if (nt==1) 
                wingtri.second = fedge->second;
            ++nt;
            if (nt==2) 
                break;
        }
        if ((nt==2) || ((nt==1) && allow_single_wing) ) {
            winged_edges.insert(std::pair<std::pair<ChNodeFEAxyz*,ChNodeFEAxyz*>, std::pair<int,int>>(wingedge,wingtri)); // ok found winged edge!
            aedge->second = -1; // deactivate this way otherwise found again by sister
        }
    }

    //
    // Now create triangles with collision models:
    //

    std::set<ChNodeFEAxyz*> added_vertexes;
        
    // iterate on triangles
    for (int it = 0; it < triangles.size(); ++it) {
        // edges = pairs of vertexes indexes
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeA( triangles[it][0],  triangles[it][1]);
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeB( triangles[it][1],  triangles[it][2]);
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeC( triangles[it][2],  triangles[it][0]);
        // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
        if (medgeA.first>medgeA.second) 
            medgeA = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeA.second, medgeA.first);
        if (medgeB.first>medgeB.second) 
            medgeB = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeB.second, medgeB.first);
        if (medgeC.first>medgeC.second) 
            medgeC = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeC.second, medgeC.first); 
        auto wingedgeA = winged_edges.find(medgeA);
        auto wingedgeB = winged_edges.find(medgeB);
        auto wingedgeC = winged_edges.find(medgeC);

        ChNodeFEAxyz* i_wingvertex_A = 0;
        ChNodeFEAxyz* i_wingvertex_B = 0;
        ChNodeFEAxyz* i_wingvertex_C = 0;

        if (tri_map[it][1] != -1) {
            i_wingvertex_A = triangles[tri_map[it][1]][0];
            if (triangles[tri_map[it][1]][1] != wingedgeA->first.first && triangles[tri_map[it][1]][1] != wingedgeA->first.second)
                i_wingvertex_A = triangles[tri_map[it][1]][1];
            if (triangles[tri_map[it][1]][2] != wingedgeA->first.first && triangles[tri_map[it][1]][2] != wingedgeA->first.second)
                i_wingvertex_A = triangles[tri_map[it][1]][2];
        }

        if (tri_map[it][2] != -1) {
            i_wingvertex_B =triangles[tri_map[it][2]][0];
            if (triangles[tri_map[it][2]][1] != wingedgeB->first.first && triangles[tri_map[it][2]][1] != wingedgeB->first.second)
                i_wingvertex_B = triangles[tri_map[it][2]][1];
            if (triangles[tri_map[it][2]][2] != wingedgeB->first.first && triangles[tri_map[it][2]][2] != wingedgeB->first.second)
                i_wingvertex_B = triangles[tri_map[it][2]][2];
        }

        if (tri_map[it][3] != -1) {
            i_wingvertex_C = triangles[tri_map[it][3]][0];
            if (triangles[tri_map[it][3]][1] != wingedgeC->first.first && triangles[tri_map[it][3]][1] != wingedgeC->first.second)
                i_wingvertex_C = triangles[tri_map[it][3]][1];
            if (triangles[tri_map[it][3]][2] != wingedgeC->first.first && triangles[tri_map[it][3]][2] != wingedgeC->first.second)
                i_wingvertex_C = triangles[tri_map[it][3]][2];
        }

        ChSharedPtr<ChContactTriangleXYZ> contact_triangle (new ChContactTriangleXYZ);
        contact_triangle->SetNode1(triangles[it][0]);
        contact_triangle->SetNode2(triangles[it][1]);
        contact_triangle->SetNode3(triangles[it][2]);
        this->vfaces.push_back(contact_triangle);
        contact_triangle->SetContactSurface(this);

        contact_triangle->GetCollisionModel()->ClearModel();
        ((collision::ChModelBullet*)contact_triangle->GetCollisionModel())->AddTriangleProxy(
                                &triangles[it][0]->pos, 
                                &triangles[it][1]->pos,
                                &triangles[it][2]->pos,
                                // if no wing vertex (ie. 'free' edge), point to opposite vertex, ie vertex in triangle not belonging to edge
                                wingedgeA->second.second != -1 ? &i_wingvertex_A->pos : &triangles[it][2]->pos, 
                                wingedgeB->second.second != -1 ? &i_wingvertex_B->pos : &triangles[it][0]->pos,
                                wingedgeC->second.second != -1 ? &i_wingvertex_C->pos : &triangles[it][1]->pos,
                                (added_vertexes.find(triangles[it][0]) == added_vertexes.end()),
                                (added_vertexes.find(triangles[it][1]) == added_vertexes.end()),
                                (added_vertexes.find(triangles[it][2]) == added_vertexes.end()),
                                // are edges owned by this triangle? (if not, they belong to a neighbouring triangle)
                                wingedgeA->second.first != -1,
                                wingedgeB->second.first != -1,
                                wingedgeC->second.first != -1,
                                sphere_swept);
        contact_triangle->GetCollisionModel()->BuildModel();

        // Mark added vertexes
        added_vertexes.insert( triangles[it][0] );
        added_vertexes.insert( triangles[it][1] );
        added_vertexes.insert( triangles[it][2] );
        // Mark added edges, setting to -1 the 'ti' id of 1st triangle in winged edge {{vi,vj}{ti,tj}}
        wingedgeA->second.first = -1;
        wingedgeB->second.first = -1;
        wingedgeC->second.first = -1;
    }

}

void ChContactSurfaceGeneric::SurfaceSyncCollisionModels() {
    for (unsigned int j = 0; j < vfaces.size(); j++) {
        this->vfaces[j]->GetCollisionModel()->SyncPosition();
    }
}

void ChContactSurfaceGeneric::SurfaceAddCollisionModelsToSystem(ChSystem* msys) {
    assert(msys);
    SurfaceSyncCollisionModels();
    for (unsigned int j = 0; j < vfaces.size(); j++) {
        msys->GetCollisionSystem()->Add(this->vfaces[j]->GetCollisionModel());
    }
}

void ChContactSurfaceGeneric::SurfaceRemoveCollisionModelsFromSystem(ChSystem* msys) {
    assert(msys);
    for (unsigned int j = 0; j < vfaces.size(); j++) {
        msys->GetCollisionSystem()->Remove(this->vfaces[j]->GetCollisionModel());
    }
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

