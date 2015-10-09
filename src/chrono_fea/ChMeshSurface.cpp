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

using namespace std;


namespace chrono 
{
namespace fea
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChMeshSurface> a_registration_ChMeshSurface;


void ChMeshSurface::AddFacesFromNodeSet( std::vector<ChSharedPtr<ChNodeFEAbase> >& node_set ) {
    
    ChHashTable<size_t, bool> anode_set_map;
    
    for (int i= 0; i< node_set.size() ; ++i)
        anode_set_map.insert( (size_t)node_set[i].get_ptr(), true );

    for (int ie= 0; ie< this->mmesh->GetNelements() ; ++ie) {
        if (ChSharedPtr<ChElementTetra_4> mtetra = this->mmesh->GetElement(ie).DynamicCastTo<ChElementTetra_4>() ) {
            bool n0= false;
            bool n1= false;
            bool n2= false;
            bool n3= false;
            ChHashTable<size_t, bool>::iterator mcached;
            mcached = anode_set_map.find((size_t)mtetra->GetNodeN(0).get_ptr());
            if (mcached != anode_set_map.end()) 
                n0 =true;
            mcached = anode_set_map.find((size_t)mtetra->GetNodeN(1).get_ptr());
            if (mcached != anode_set_map.end()) 
                n1 =true;
            mcached = anode_set_map.find((size_t)mtetra->GetNodeN(2).get_ptr());
            if (mcached != anode_set_map.end()) 
                n2 =true;
            mcached = anode_set_map.find((size_t)mtetra->GetNodeN(3).get_ptr());
            if (mcached != anode_set_map.end()) 
                n3 =true;
           
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
            /*
           if (n3||n1||n2||n3) {
                ChSharedPtr<ChFaceTetra_4> mface(new ChFaceTetra_4(mtetra,3));
                this->AddFace( mface );
            }
            if (n3||n1||n2||n3) {
                ChSharedPtr<ChFaceTetra_4> mface(new ChFaceTetra_4(mtetra,0));
                this->AddFace( mface );
            }
            if (n3||n1||n2||n3) {
                ChSharedPtr<ChFaceTetra_4> mface(new ChFaceTetra_4(mtetra,1));
                this->AddFace( mface );
            }
            if (n3||n1||n2||n3) {
                ChSharedPtr<ChFaceTetra_4> mface(new ChFaceTetra_4(mtetra,2));
                this->AddFace( mface );
            }
            */
           
        }
        if (ChSharedPtr<ChElementShellANCF> mshell = this->mmesh->GetElement(ie).DynamicCastTo<ChElementShellANCF>() ) {
            this->AddFace( mshell );
        }
    }
    GetLog() << "aFound " << this->faces.size() << " faces \n\n";
    
    return;


    std::unordered_set<size_t>  node_set_map;

    for (int i= 0; i< node_set.size() ; ++i)
        node_set_map.insert( (size_t)node_set[i].get_ptr() );

GetLog() << "Inserted " << node_set.size() << "nodes from nodeset\n";

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
            /*
           if (n3||n1||n2||n3) {
                ChSharedPtr<ChFaceTetra_4> mface(new ChFaceTetra_4(mtetra,3));
                this->AddFace( mface );
            }
            if (n3||n1||n2||n3) {
                ChSharedPtr<ChFaceTetra_4> mface(new ChFaceTetra_4(mtetra,0));
                this->AddFace( mface );
            }
            if (n3||n1||n2||n3) {
                ChSharedPtr<ChFaceTetra_4> mface(new ChFaceTetra_4(mtetra,1));
                this->AddFace( mface );
            }
            if (n3||n1||n2||n3) {
                ChSharedPtr<ChFaceTetra_4> mface(new ChFaceTetra_4(mtetra,2));
                this->AddFace( mface );
            }
            */
           
        }
        if (ChSharedPtr<ChElementShellANCF> mshell = this->mmesh->GetElement(ie).DynamicCastTo<ChElementShellANCF>() ) {
            this->AddFace( mshell );
        }
    }
    GetLog() << "Found " << this->faces.size() << " faces \n\n";
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

