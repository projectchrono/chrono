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


#include "chrono_fea/ChMeshSurface.h"
#include "chrono_fea/ChMesh.h"
#include "physics/ChSystem.h"
#include "chrono_fea/ChElementTetra_4.h"
#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChFaceTetra_4.h"
#include <unordered_set>
#include "core/ChHashTable.h"
#include <unordered_map>
#include <map>
#include <array>

using namespace std;


namespace chrono 
{
namespace fea
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChMeshSurface> a_registration_ChMeshSurface;


void ChMeshSurface::AddFacesFromNodeSet( std::vector<ChSharedPtr<ChNodeFEAbase> >& node_set ) {

    std::unordered_set<size_t>  node_set_map;

    for (int i= 0; i< node_set.size() ; ++i)
        node_set_map.insert( (size_t)node_set[i].get_ptr() );

    for (int ie= 0; ie< this->mmesh->GetNelements() ; ++ie) {
        if (ChSharedPtr<ChElementTetra_4> mtetra = this->mmesh->GetElement(ie).DynamicCastTo<ChElementTetra_4>() ) {
            bool n0 = (node_set_map.find((size_t)mtetra->GetNodeN(0).get_ptr()) != node_set_map.end());
            bool n1 = (node_set_map.find((size_t)mtetra->GetNodeN(1).get_ptr()) != node_set_map.end());
            bool n2 = (node_set_map.find((size_t)mtetra->GetNodeN(2).get_ptr()) != node_set_map.end());
            bool n3 = (node_set_map.find((size_t)mtetra->GetNodeN(3).get_ptr()) != node_set_map.end());
            
            if (n0 && n1 && n2) {
                ChSharedPtr<ChFaceTetra_4> mface(new ChFaceTetra_4(mtetra,3));
                this->AddFace( mface );
            }
            if (n1 && n2 && n3) {
                ChSharedPtr<ChFaceTetra_4> mface(new ChFaceTetra_4(mtetra,0));
                this->AddFace( mface );
            }
            if (n0 && n2 && n3) {
                ChSharedPtr<ChFaceTetra_4> mface(new ChFaceTetra_4(mtetra,1));
                this->AddFace( mface );
            }
            if (n0 && n1 && n3) {
                ChSharedPtr<ChFaceTetra_4> mface(new ChFaceTetra_4(mtetra,2));
                this->AddFace( mface );
            }
           
        }
        if (ChSharedPtr<ChElementShellANCF> mshell = this->mmesh->GetElement(ie).DynamicCastTo<ChElementShellANCF>() ) {
            this->AddFace( mshell );
        }
    }
    //GetLog() << "AddFacesFromNodeSet found " << this->faces.size() << " faces \n\n";
}


void ChMeshSurface::AddFacesFromBoundary() {
    
    /// Case1. Outer skin boundary of meshes of TETRAHEDRONS:
    ///
    std::multimap< std::array<ChNodeFEAxyz*, 3> , ChFaceTetra_4> face_map;

    for (int ie= 0; ie< this->mmesh->GetNelements(); ++ie) {
        if (ChSharedPtr<ChElementTetra_4> mtetra = mmesh->GetElement(ie).DynamicCastTo<ChElementTetra_4>()) {
            for (int nface = 0; nface<4; ++nface) {
                ChFaceTetra_4 mface(mtetra, nface);
                std::array<ChNodeFEAxyz*, 3> mface_key = {mface.GetNodeN(0).get_ptr(), mface.GetNodeN(1).get_ptr(), mface.GetNodeN(2).get_ptr()};
                std::sort(mface_key.begin(), mface_key.end());
                face_map.insert( {mface_key, mface} );
            }
        }
    }
    for (int ie= 0; ie< this->mmesh->GetNelements(); ++ie) {
        if (ChSharedPtr<ChElementTetra_4> mtetra = mmesh->GetElement(ie).DynamicCastTo<ChElementTetra_4>()) {
            for (int nface = 0; nface<4; ++nface) {
                ChFaceTetra_4 mface(mtetra, nface);
                std::array<ChNodeFEAxyz*, 3> mface_key = {mface.GetNodeN(0).get_ptr(), mface.GetNodeN(1).get_ptr(), mface.GetNodeN(2).get_ptr()};
                std::sort(mface_key.begin(), mface_key.end());
                if (face_map.count(mface_key) == 1) {
                    // Found a face that is not shared.. so it is a boundary face. 
                    // Instance it to be handled via shared ptr, and add it to list...
                    ChSharedPtr<ChFaceTetra_4> boundary_face(new ChFaceTetra_4(mtetra, nface));
                    this->AddFace(boundary_face);
                }
            }
        }
    }

}


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

