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
// Authors: Andrea Favali, Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono_wood/ChMeshLine.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChElementTetraCorot_4.h"
//#include "chrono_ldpm/ChElementLDPM.h"
#include <unordered_set>
#include <map>
#include <array>
#include <algorithm>
#include <memory>
#include <set>


//using namespace ldpm;

namespace chrono {
namespace wood {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChMeshLine)


void ChMeshLine::AddBeamsFromGivenNodeSet(std::vector<std::shared_ptr<fea::ChNodeFEAxyzrot>>& node_set) {
	SortNodesByCoordinates(node_set);    
    for (int i = 0; i < node_set.size()-1; ++i){
		auto beam = chrono_types::make_shared<ChElementBeamEuler>();
        beam->SetNodes(node_set[i], node_set[i+1]);
		AddBeam(beam);  
	}
}

void ChMeshLine::AddBeamsFromNodeSet(std::vector<std::shared_ptr<fea::ChNodeFEAxyzrot>>& node_set) {
    std::unordered_set<size_t> node_set_map;
    for (int i = 0; i < node_set.size(); ++i)
        node_set_map.insert((size_t)node_set[i].get());
    
    std::set<std::pair<std::shared_ptr<fea::ChNodeFEAxyzrot>, std::shared_ptr<fea::ChNodeFEAxyzrot>>> unique_edges;
    
    for (unsigned int ie = 0; ie < this->mmesh->GetNumElements (); ++ie) {
        auto element = mmesh->GetElement(ie);
		///////////////////////////////////////////////////////////////////////////////
        ///
        /// Get Euler beam elements  
        ///
        ///////////////////////////////////////////////////////////////////////////////
        if (auto beam = std::dynamic_pointer_cast<ChElementBeamEuler>(element)) {
            unsigned char nodes = 0;  // if the i-th bit is 1, the corresponding beam node is in the set
            if (node_set_map.find((size_t)beam->GetNodeA().get()) != node_set_map.end())
                nodes |= 1 << 0;
            if (node_set_map.find((size_t)beam->GetNodeB().get()) != node_set_map.end())
                nodes |= 1 << 1;

            if ((nodes & 0x03) == 0x03)  // both nodes are in the set
                AddBeam(beam);
        }	
		/*
        ///////////////////////////////////////////////////////////////////////////////
        ///
        /// Get edge of LDPM tet  
        ///
        ///////////////////////////////////////////////////////////////////////////////
        if (const auto& tetra = std::dynamic_pointer_cast<ChElementLDPM>(element)) {
            auto node0 = std::dynamic_pointer_cast<fea::ChNodeFEAxyzrot>(tetra->GetNode(0));
            auto node1 = std::dynamic_pointer_cast<fea::ChNodeFEAxyzrot>(tetra->GetNode(1));
            auto node2 = std::dynamic_pointer_cast<fea::ChNodeFEAxyzrot>(tetra->GetNode(2));
            auto node3 = std::dynamic_pointer_cast<fea::ChNodeFEAxyzrot>(tetra->GetNode(3));
            std::vector<std::pair<std::shared_ptr<fea::ChNodeFEAxyzrot>, std::shared_ptr<fea::ChNodeFEAxyzrot>>> edges = {
                {node0, node1}, {node0, node2}, {node0, node3}, 
                {node1, node2}, {node1, node3}, {node2, node3}  
            };
            
            for (auto& edge : edges) {
                if (edge.first->GetIndex() < edge.second->GetIndex()) {
                    unique_edges.insert(edge);
                } else {
                    unique_edges.insert({edge.second, edge.first});
                }
            } 
        }
        */
        ///////////////////////////////////////////////////////////////////////////////
        ///
        /// Get edge of terahedral element   
        ///
        ///////////////////////////////////////////////////////////////////////////////
        if (const auto& tetra = std::dynamic_pointer_cast<ChElementTetraCorot_4>(element)) {
            auto node0 = std::dynamic_pointer_cast<fea::ChNodeFEAxyzrot>(tetra->GetNode(0));
            auto node1 = std::dynamic_pointer_cast<fea::ChNodeFEAxyzrot>(tetra->GetNode(1));
            auto node2 = std::dynamic_pointer_cast<fea::ChNodeFEAxyzrot>(tetra->GetNode(2));
            auto node3 = std::dynamic_pointer_cast<fea::ChNodeFEAxyzrot>(tetra->GetNode(3));
            std::vector<std::pair<std::shared_ptr<fea::ChNodeFEAxyzrot>, std::shared_ptr<fea::ChNodeFEAxyzrot>>> edges = {
                {node0, node1}, {node0, node2}, {node0, node3}, 
                {node1, node2}, {node1, node3}, {node2, node3}  
            };
            
            for (auto& edge : edges) {
                if (edge.first->GetIndex() < edge.second->GetIndex()) {
                    unique_edges.insert(edge);
                } else {
                    unique_edges.insert({edge.second, edge.first});
                }
            } 
        }
        
       
        
        
    }
    
    ///////////////////////////////////////////////////////////////////////////////
    ///
    /// Convert edges whose both nodes are in given nodelist   
    ///
    ///////////////////////////////////////////////////////////////////////////////
     
        for (auto& edge : unique_edges) {
        	//std::cout<<edge.first->GetIndex() <<"\t"<< edge.second->GetIndex()<<std::endl;
            	 unsigned char nodes = 0;  // if the i-th bit is 1, the corresponding beam node is in the set
		    if (node_set_map.find((size_t)edge.first.get()) != node_set_map.end())
		        nodes |= 1 << 0;
		    if (node_set_map.find((size_t)edge.second.get()) != node_set_map.end())
		        nodes |= 1 << 1;

		    if ((nodes & 0x03) == 0x03) { // both nodes are in the set
		    	auto beam = chrono_types::make_shared<ChElementBeamEuler>();
        		beam->SetNodes(edge.first, edge.second);
		        AddBeam(beam);  
		    }             
            }            
    
}

void ChMeshLine::AddBeamsFromBoundary() {
    // Boundary lines of BEAMS
    std::multimap<std::array<ChNodeFEAxyzrot*, 2>, std::shared_ptr<ChElementBeamEuler>> line_map;

    for (unsigned int ie = 0; ie < this->mmesh->GetNumElements (); ++ie) {
        if (auto beam = std::dynamic_pointer_cast<ChElementBeamEuler>(mmesh->GetElement(ie))) {
            std::array<ChNodeFEAxyzrot*, 2> beam_key = {beam->GetNodeA().get(), beam->GetNodeB().get()};
            std::sort(beam_key.begin(), beam_key.end());
            line_map.insert({beam_key, beam});
        }
    }

    for (auto& entry : line_map) {
        if (line_map.count(entry.first) == 1) {
            // Found a beam that is not shared, so it is a boundary beam.
            this->AddBeam(entry.second);
        }
    }
}

// Function to sort nodes by their coordinates
void ChMeshLine::SortNodesByCoordinates(std::vector<std::shared_ptr<ChNodeFEAbase>>& node_set) {
    std::sort(node_set.begin(), node_set.end(), [](const std::shared_ptr<ChNodeFEAbase>& a, const std::shared_ptr<ChNodeFEAbase>& b) {
        auto a_xyz = std::dynamic_pointer_cast<ChNodeFEAxyz>(a);
        auto b_xyz = std::dynamic_pointer_cast<ChNodeFEAxyz>(b);
        if (!a_xyz || !b_xyz) {
            throw std::runtime_error("Node cannot be cast to ChNodeFEAxyz.");
        }

        // Compare x-coordinates
        if (a_xyz->GetPos().x() < b_xyz->GetPos().x()) return true;
        if (a_xyz->GetPos().x() > b_xyz->GetPos().x()) return false;

        // x-coordinates are the same, compare y-coordinates
        if (a_xyz->GetPos().y() < b_xyz->GetPos().y()) return true;
        if (a_xyz->GetPos().y() > b_xyz->GetPos().y()) return false;

        // x and y-coordinates are the same, compare z-coordinates
        return a_xyz->GetPos().z() < b_xyz->GetPos().z();
    });
}


// Function to sort nodes by their coordinates
void ChMeshLine::SortNodesByCoordinates(std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& node_set) {
    std::sort(node_set.begin(), node_set.end(), [](const std::shared_ptr<ChNodeFEAxyzrot>& a, const std::shared_ptr<ChNodeFEAxyzrot>& b) {
       
        if (!a || !b) {
            throw std::runtime_error("Node cannot be cast to ChNodeFEAxyz.");
        }

        // Compare x-coordinates
        if (a->GetPos().x() < b->GetPos().x()) return true;
        if (a->GetPos().x() > b->GetPos().x()) return false;

        // x-coordinates are the same, compare y-coordinates
        if (a->GetPos().y() < b->GetPos().y()) return true;
        if (a->GetPos().y() > b->GetPos().y()) return false;

        // x and y-coordinates are the same, compare z-coordinates
        return a->GetPos().z() < b->GetPos().z();
    });
}


}  // end namespace wood
}  // end namespace chrono

