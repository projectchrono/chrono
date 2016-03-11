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

#include "chrono/collision/ChCModelBullet.h"
#include "chrono/core/ChMath.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_fea/ChContactSurfaceMesh.h"
#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChElementTetra_4.h"
#include "chrono_fea/ChElementBeamANCF.h"
#include "chrono_fea/ChElementBeamEuler.h"
#include "chrono_fea/ChFaceTetra_4.h"
#include "chrono_fea/ChMesh.h"

#include <unordered_map>
#include <map>
#include <set>
#include <array>
#include <algorithm>

using namespace std;

namespace chrono {
namespace fea {

//////////////////////////////////////////////////////////////////////////////
////  ChContactTriangleXYZ

ChContactTriangleXYZ::ChContactTriangleXYZ() {
        this->collision_model = new collision::ChModelBullet;
        this->collision_model->SetContactable(this);
}

ChContactTriangleXYZ::ChContactTriangleXYZ(std::shared_ptr<ChNodeFEAxyz> n1,
                                           std::shared_ptr<ChNodeFEAxyz> n2,
                                           std::shared_ptr<ChNodeFEAxyz> n3,
                                           ChContactSurface* acontainer) {
    mnode1 = n1;
    mnode1 = n2;
    mnode1 = n3;
    container = acontainer;

    this->collision_model = new collision::ChModelBullet;
    this->collision_model->SetContactable(this);
}

std::shared_ptr<ChMaterialSurfaceBase>& ChContactTriangleXYZ::GetMaterialSurfaceBase() {
    return container->GetMaterialSurfaceBase();
}

ChPhysicsItem* ChContactTriangleXYZ::GetPhysicsItem() {
    return (ChPhysicsItem*)container->GetMesh();
}


//////////////////////////////////////////////////////////////////////////////
////  ChContactSurfaceMesh

void ChContactSurfaceMesh::AddFacesFromBoundary(double sphere_swept) {
    std::vector<std::array<ChNodeFEAxyz*, 3>> triangles;
    std::vector<std::array<std::shared_ptr<ChNodeFEAxyz>, 3>> triangles_ptrs;

    ///
    /// Case1. Outer skin boundary of meshes of TETRAHEDRONS:
    ///

    std::multimap< std::array<ChNodeFEAxyz*, 3> , ChFaceTetra_4> face_map;

    for (unsigned int ie= 0; ie< this->mmesh->GetNelements(); ++ie) {
        if (auto mtetra = std::dynamic_pointer_cast<ChElementTetra_4>(mmesh->GetElement(ie))) {
            for (int nface = 0; nface<4; ++nface) {
                ChFaceTetra_4 mface(mtetra, nface);
                std::array<ChNodeFEAxyz*, 3> mface_key = {mface.GetNodeN(0).get(), mface.GetNodeN(1).get(), mface.GetNodeN(2).get()};
                std::sort(mface_key.begin(), mface_key.end());
                face_map.insert( {mface_key, mface} );
            }
        }
    }
    for (unsigned int ie= 0; ie< this->mmesh->GetNelements(); ++ie) {
        if (auto mtetra = std::dynamic_pointer_cast<ChElementTetra_4>(mmesh->GetElement(ie))) {
            for (int nface = 0; nface<4; ++nface) {
                ChFaceTetra_4 mface(mtetra, nface);
                std::array<ChNodeFEAxyz*, 3> mface_key = {mface.GetNodeN(0).get(), mface.GetNodeN(1).get(), mface.GetNodeN(2).get()};
                std::sort(mface_key.begin(), mface_key.end());
                if (face_map.count(mface_key) == 1) {  
                    // Found a face that is not shared.. so it is a boundary face. 
                    triangles.push_back( {mface.GetNodeN(0).get(), mface.GetNodeN(1).get(), mface.GetNodeN(2).get()} );
                    triangles_ptrs.push_back( {mface.GetNodeN(0), mface.GetNodeN(1), mface.GetNodeN(2)} );
                }
            }
        }
    }

    ///
    /// Case2. skin of ANCF SHELLS:
    ///
    for (unsigned int ie= 0; ie< this->mmesh->GetNelements(); ++ie) {
        if (auto mshell = std::dynamic_pointer_cast<ChElementShellANCF>(mmesh->GetElement(ie))) {
            std::shared_ptr<ChNodeFEAxyz> nA = mshell->GetNodeA();
            std::shared_ptr<ChNodeFEAxyz> nB = mshell->GetNodeB();
            std::shared_ptr<ChNodeFEAxyz> nC = mshell->GetNodeC();
            std::shared_ptr<ChNodeFEAxyz> nD = mshell->GetNodeD();
            std::array<ChNodeFEAxyz*,3> tri1 = {nA.get(),nB.get(),nC.get()};
            std::array<ChNodeFEAxyz*,3> tri2 = {nB.get(),nC.get(),nD.get()};
            std::array<std::shared_ptr<ChNodeFEAxyz>,3> tri1_ptrs = {nA,nB,nC};
            std::array<std::shared_ptr<ChNodeFEAxyz>,3> tri2_ptrs = {nB,nC,nD};
            triangles.push_back( tri1 );
            triangles.push_back( tri2 );
            triangles_ptrs.push_back( tri1_ptrs );
            triangles_ptrs.push_back( tri2_ptrs );
        }
    }


    ///
    /// Case3. EULER BEAMS (handles as a skinny triangle, with sphere swept radii, i.e. a capsule):
    ///
    /*
    for (unsigned int ie= 0; ie< this->mmesh->GetNelements(); ++ie) {
        if (auto mbeam = std::dynamic_pointer_cast<ChElementBeamEuler>(mmesh->GetElement(ie))) {
            std::shared_ptr<ChNodeFEAxyzrot> nA = mbeam->GetNodeA();
            std::shared_ptr<ChNodeFEAxyzrot> nB = mbeam->GetNodeB();
            std::array<ChNodeFEAxyz*,3> tri1 = {nA.get(),nB.get(),nB.get()};
            std::array<std::shared_ptr<ChNodeFEAxyz>,3> tri1_ptrs = {nA,nB,nB};
            triangles.push_back( tri1 );
            triangles_ptrs.push_back( tri1_ptrs );
        }
    }
    */

    ///
    /// Case4. ANCF BEAMS (handles as a skinny triangle, with sphere swept radii, i.e. a capsule):
    ///
    for (unsigned int ie= 0; ie< this->mmesh->GetNelements(); ++ie) {
        if (auto mbeam = std::dynamic_pointer_cast<ChElementBeamANCF>(mmesh->GetElement(ie))) {
            std::shared_ptr<ChNodeFEAxyzD> nA = mbeam->GetNodeA();
            std::shared_ptr<ChNodeFEAxyzD> nB = mbeam->GetNodeB();

            auto contact_triangle = std::make_shared<ChContactTriangleXYZ>();
            contact_triangle->SetNode1(nA);
            contact_triangle->SetNode2(nB);
            contact_triangle->SetNode3(nB);
            this->vfaces.push_back(contact_triangle);
            contact_triangle->SetContactSurface(this);

            contact_triangle->GetCollisionModel()->ClearModel();
            ((collision::ChModelBullet*)contact_triangle->GetCollisionModel())->AddTriangleProxy(
                                    &nA->pos, 
                                    &nB->pos,
                                    &nB->pos, 
                                    0, 0, 0, // no wing vertexes
                                    false, false, false, // are vertexes owned by this triangle?
                                    true, false, true, // are edges owned by this triangle? 
                                    mbeam->GetSection()->GetDrawCircularRadius());
            contact_triangle->GetCollisionModel()->BuildModel();
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

        auto contact_triangle = std::make_shared<ChContactTriangleXYZ>();
        contact_triangle->SetNode1(triangles_ptrs[it][0]);
        contact_triangle->SetNode2(triangles_ptrs[it][1]);
        contact_triangle->SetNode3(triangles_ptrs[it][2]);
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

void ChContactSurfaceMesh::SurfaceSyncCollisionModels() {
    for (unsigned int j = 0; j < vfaces.size(); j++) {
        this->vfaces[j]->GetCollisionModel()->SyncPosition();
    }
}

void ChContactSurfaceMesh::SurfaceAddCollisionModelsToSystem(ChSystem* msys) {
    assert(msys);
    SurfaceSyncCollisionModels();
    for (unsigned int j = 0; j < vfaces.size(); j++) {
        msys->GetCollisionSystem()->Add(this->vfaces[j]->GetCollisionModel());
    }
}

void ChContactSurfaceMesh::SurfaceRemoveCollisionModelsFromSystem(ChSystem* msys) {
    assert(msys);
    for (unsigned int j = 0; j < vfaces.size(); j++) {
        msys->GetCollisionSystem()->Remove(this->vfaces[j]->GetCollisionModel());
    }
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

