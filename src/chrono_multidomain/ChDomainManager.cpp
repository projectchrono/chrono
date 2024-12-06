// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#include<array>
#include "chrono_multidomain/ChDomainManager.h"
#include "chrono_multidomain/ChSystemDescriptorMultidomain.h"
#include "chrono_multidomain/ChSolverPSORmultidomain.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChElementBase.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/serialization/ChArchiveXML.h"
#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/serialization/ChArchiveUtils.h"

namespace chrono {
namespace multidomain {

using namespace fea;


///////////////////////


void ChDomainManager::PrintDebugDomainInfo(std::shared_ptr<ChDomain> domain) {
	std::cout << "DOMAIN ---- rank: " << domain->GetRank() << "-----------------------------\n";

	for (auto body : domain->GetSystem()->GetBodies()) {
		std::cout << "  ChBody " << body->GetTag() << std::endl;
	}
	for (auto link : domain->GetSystem()->GetLinks()) {
		std::cout << "  ChLink " << link->GetTag() << std::endl;
		if (auto blink = std::dynamic_pointer_cast<ChLink>(link)) {
			auto mbo1 = dynamic_cast<ChBody*>(blink->GetBody1());
			auto mbo2 = dynamic_cast<ChBody*>(blink->GetBody2());
			if (mbo1 && mbo2) {
				std::cout << "       ChBody*  " << mbo1->GetTag() << std::endl;
				std::cout << "       ChBody*  " << mbo2->GetTag() << std::endl;
			}
			auto mnod1 = dynamic_cast<fea::ChNodeFEAxyzrot*>(blink->GetBody1());
			auto mnod2 = dynamic_cast<fea::ChNodeFEAxyzrot*>(blink->GetBody2());
			if (mnod1 && mnod2) {
				std::cout << "       ChNodeFEAxyzrot*  " << mnod1->GetTag() << std::endl;
				std::cout << "       ChNodeFEAxyzrot*  " << mnod2->GetTag() << std::endl;
			}
		}
	}
	for (auto mesh : domain->GetSystem()->GetMeshes()) {
		std::cout << "  ChMesh " << mesh->GetTag() << std::endl;
		for (auto node : mesh->GetNodes()) {
			bool printed = false;
			if (auto mnod = std::dynamic_pointer_cast<fea::ChNodeFEAxyzrot>(node)) {
				std::cout << "          ChNodeFEAxyzrot " << node->GetTag() << "    " << mnod->GetPos() << std::endl;
				printed = true;
			}
			if (auto mnod = std::dynamic_pointer_cast<fea::ChNodeFEAxyz>(node)) {
				std::cout << "          ChNodeFEAxyz " << node->GetTag() << "    " << mnod->GetPos() << std::endl;
				printed = true;
			}
			if (!printed)
				std::cout << "          ChNodeFEAbase " << node->GetTag() << std::endl;
		}
		for (auto el : mesh->GetElements()) {
			std::cout << "      ChElementBase " << std::endl;
			for (int i = 0; i < el->GetNumNodes(); ++i) {
				std::cout << "          ChNodeFEAbase " << el->GetNode(i)->GetTag() << std::endl;
			}
		}
	}
	for (auto phys : domain->GetSystem()->GetOtherPhysicsItems()) {
		std::cout << "  ChPhysicsItem " << phys->GetTag() << std::endl;
	}

	for (auto& interf : domain->GetInterfaces()) {
		std::cout << " interface to domain rank " << interf.second.side_OUT->GetRank() << " ...... \n";
		for (auto& item : interf.second.shared_items)
			std::cout << "  shared item tag " << item.first << std::endl;
		for (auto& node : interf.second.shared_nodes)
			std::cout << "  shared node tag " << node.first << std::endl;
	}
	std::cout << std::endl;
}




//////////////////////////////////////////////////////////////////////////////////////////////





ChDomainBuilderSlices::ChDomainBuilderSlices(
											int tot_domains,
											double mmin,
											double mmax,
											ChAxis maxis,
											bool build_master) {
	this->axis = maxis;
	domains_bounds.push_back(-1e34);
	for (int islice = 0; islice < (tot_domains - 1); ++islice) {
		domains_bounds.push_back(mmin + (islice + 1.0) * (mmax - mmin) / (double)tot_domains);
	}
	domains_bounds.push_back(1e34);

	m_build_master = build_master;
};

ChDomainBuilderSlices::ChDomainBuilderSlices(
											std::vector<double> axis_cuts,
											ChAxis maxis,
											bool build_master) {
	this->axis = maxis;
	domains_bounds.push_back(-1e34);
	for (double dcut : axis_cuts) {
		domains_bounds.push_back(dcut);
	}
	domains_bounds.push_back(1e34);

	m_build_master = build_master;
}


std::shared_ptr<ChDomain> ChDomainBuilderSlices::BuildDomain(
	ChSystem* msys,
	int this_rank) {
	
	assert(this_rank >= 0);
	assert(this_rank < GetTotSlices());

	auto domain = chrono_types::make_shared<ChDomainSlice>(msys, this_rank, domains_bounds[this_rank], domains_bounds[this_rank + 1], axis);

	if (this_rank > 0) {
		int low_domain_rank = this_rank - 1;
		auto low_domain = chrono_types::make_shared<ChDomainSlice>(nullptr, low_domain_rank, domains_bounds[this_rank-1], domains_bounds[this_rank], axis);
		ChDomainInterface& low_interf = domain->GetInterfaces()[low_domain->GetRank()]; // insert domain interface in unordered map
		low_interf.side_IN = domain.get();
		low_interf.side_OUT = low_domain;
	}
	if (this_rank < domains_bounds.size() - 2) {
		int hi_domain_rank = this_rank + 1;
		auto hi_domain = chrono_types::make_shared<ChDomainSlice>(nullptr, hi_domain_rank, domains_bounds[this_rank+1], domains_bounds[this_rank+2], axis);
		ChDomainInterface& hi_interf = domain->GetInterfaces()[hi_domain->GetRank()];  // insert domain interface in unordered map
		hi_interf.side_IN = domain.get();
		hi_interf.side_OUT = hi_domain;
	}
	
	if (this->m_build_master) {
		int imaster_rank = this->GetTotRanks() - 1;
		auto master_dom = chrono_types::make_shared<ChDomainMaster>(nullptr, imaster_rank);
		ChDomainInterface& master_interf = domain->GetInterfaces()[imaster_rank];  // insert domain interface in unordered map
		master_interf.side_IN = domain.get();
		master_interf.side_OUT = master_dom;
	}

	return domain;
}


std::shared_ptr<ChDomain> ChDomainBuilderSlices::BuildMasterDomain(
	ChSystem* msys) {

	assert(this->m_build_master);
	int this_rank = this->GetTotRanks() - 1;

	auto domain = chrono_types::make_shared<ChDomainMaster>(msys, this_rank);
	
	for (int i = 0; i < this->GetTotSlices(); ++i) {
		int in_domain_rank = i;
		auto in_domain = chrono_types::make_shared<ChDomainSlice>(nullptr, in_domain_rank, domains_bounds[i], domains_bounds[i + 1], axis);
		ChDomainInterface& hi_interf = domain->GetInterfaces()[in_domain_rank];  // insert domain interface in unordered map
		hi_interf.side_IN = domain.get();
		hi_interf.side_OUT = in_domain;
	}

	return domain;
}


//////////////////////////////////////////////////////////////////////////////////////////7

// helper structures for managing incremental serialization
template<class T>
class ChIncrementalObj {
public:
	int container_ID =0;
	std::shared_ptr<T> obj;
	void ArchiveOut(ChArchiveOut& archive_out)  // for Chrono serialization
	{
		archive_out << CHNVP(container_ID);
		archive_out << CHNVP(obj);
	}
	void ArchiveIn(ChArchiveIn& archive_in)  // for Chrono serialization
	{
		archive_in >> CHNVP(container_ID);
		archive_in >> CHNVP(obj);
	}
};

// helper function to add graph node info in interfaces
template <class T_node_serial, class T_node_sh>
void InterfaceManageNodeSharedLeaving(
	std::shared_ptr<T_node_serial> obj,
	bool is_overlapping_IN,
	bool is_overlapping_OUT,
	int mtag,
	int parent_tag,
	std::unordered_set<int>& shared_ids,
	std::unordered_set<int>& sent_ids,
	std::vector<ChIncrementalObj<T_node_serial>>& objs_to_send,	
	std::map<int, std::shared_ptr<T_node_sh>>& interface_sharedmap
	) {

	bool is_sharing = (interface_sharedmap.find(mtag) != interface_sharedmap.end());
	bool is_sent    = (sent_ids.find(mtag) != sent_ids.end());

	// if jumps completely in B domain without passing in shared state:
	if (is_overlapping_OUT && !is_overlapping_IN && !is_sharing && !is_sent) {
		// migrate to neighbour
		objs_to_send.push_back(ChIncrementalObj<T_node_serial>{ parent_tag, obj });
		sent_ids.insert(mtag);
	}
	// if is in shared state between A and B:
	if (is_overlapping_OUT && is_overlapping_IN) {
		shared_ids.insert(mtag);
		if (!is_sharing) {
			// enter shared state
			// migrate to neighbour
			if (!is_sent) {
				objs_to_send.push_back(ChIncrementalObj<T_node_serial>{ parent_tag, obj });
				sent_ids.insert(mtag);
			}
		}
	}
}

void ChDomain::DoUpdateSharedLeaving() {


	// Select objects to sent to surrounding domains 
	for (auto& interf : this->interfaces) {

		std::unordered_set<int> shared_ids;
		std::unordered_set<int> sent_ids;

		// prepare the serializer
		interf.second.buffer_sending.str("");
		interf.second.buffer_sending.clear();
		interf.second.buffer_receiving.str("");
		interf.second.buffer_receiving.clear();

		std::shared_ptr<ChArchiveOut> serializer;
		switch (this->serializer_type) {
		case DomainSerializerFormat::BINARY:
			serializer = chrono_types::make_shared<ChArchiveOutBinary>(interf.second.buffer_sending); break;
		case DomainSerializerFormat::JSON:
			serializer = chrono_types::make_shared<ChArchiveOutJSON>(interf.second.buffer_sending); break;
		case DomainSerializerFormat::XML:
			serializer = chrono_types::make_shared<ChArchiveOutXML>(interf.second.buffer_sending); break;
		default: break;
		}

		serializer->SetEmptyShallowContainers(true);	// we will take care of contained items one by one
		serializer->SetUseGetTagAsID(true);				// GetTag of items, when available, will be used to generate unique IDs in serialization
	
		// Prepare a map of pointers to shared items so that they are not serialized but just referenced
		// by tag. For this reason we traverse all the pointers in the shared items.
		ChArchivePointerMap rebinding_pointers;
		rebinding_pointers.SetEmptyShallowContainers(true);	// we will take care of contained items one by one
		for (const auto& body : system->GetBodies()) {
			if (interf.second.shared_items.find(body->GetTag()) != interf.second.shared_items.end())
				rebinding_pointers << CHNVP(body);
		}
		for (const auto& omesh : system->GetMeshes()) {
			if (interf.second.shared_items.find(omesh->GetTag()) != interf.second.shared_items.end())
				rebinding_pointers << CHNVP(omesh); // no nodes traversed, because of SetEmptyShallowContainers
			for (const auto& onode : omesh->GetNodes()) {
				if (interf.second.shared_nodes.find(onode->GetTag()) != interf.second.shared_nodes.end())
					rebinding_pointers << CHNVP(onode);
			}
		}
		for (const auto& oitem : system->GetOtherPhysicsItems()) {
			if (interf.second.shared_items.find(oitem->GetTag()) != interf.second.shared_items.end())
				rebinding_pointers << CHNVP(oitem);
		}
		serializer->ExternalPointersMap() = rebinding_pointers.pointer_map_ptr_id;
		// Avoid propagation of serialization to parent ChSystem when back pointers to system are found:
		   serializer->CutPointers().insert(this->system);  //NO - better, assuming all domains already contain a ChSystem with same GetTag:
		//serializer->UnbindExternalPointer(this->system, this->system->GetTag());

		/*
		std::cout << "           domain << " << this->rank << " ExternalPointersMap size: " << rebinding_pointers.pointer_map_ptr_id.size() << "\n";
		for (auto& m : rebinding_pointers.pointer_map_ptr_id)
			std::cout << "            tags: " << m.second << "\n";
		*/

		std::vector<ChIncrementalObj<ChBody>>		bodies_migrating;
		std::vector<ChIncrementalObj<ChPhysicsItem>>items_migrating;
		std::vector<ChIncrementalObj<ChMesh>>		meshes_migrating;
		std::vector<ChIncrementalObj<ChLinkBase>>	links_migrating;
		std::vector<ChIncrementalObj<ChNodeBase>>   nodes_migrating;
		std::vector<ChIncrementalObj<ChElementBase>>elements_migrating;

		// 1-BODIES overlapping because of collision shapes, or just their center of mass

		for (auto body : system->GetBodies()) {
			ChAABB mabb = body->GetTotalAABB();
			bool is_overlapping_IN = interf.second.side_IN->IsOverlap(mabb) || interf.second.side_IN->IsInto(body->GetPos()); // if aabb is empty (reversed) use center
			bool is_overlapping_OUT = interf.second.side_OUT->IsOverlap(mabb) || interf.second.side_OUT->IsInto(body->GetPos()); // if aabb is empty (reversed) use center

			InterfaceManageNodeSharedLeaving<ChBody, ChPhysicsItem > (
				body,
				is_overlapping_IN,
				is_overlapping_OUT,
				body->GetTag(),
				system->GetAssembly().GetTag(),
				shared_ids,
				sent_ids,
				bodies_migrating,
				interf.second.shared_items
			);
		}

		// 2-LINKS (and BODIES overlapping because of connected links)

		for (const auto& link : system->GetLinks()) {
			if (auto blink = std::dynamic_pointer_cast<ChLink>(link)) {

				ChVector3d reference;
				bool is_referencein_IN = false;
				bool is_referencein_OUT = false;

				auto mbo1 = dynamic_cast<ChBody*>(blink->GetBody1());
				auto mbo2 = dynamic_cast<ChBody*>(blink->GetBody2());
				if (mbo1 && mbo2) {
					reference = mbo2->GetPos();
					is_referencein_IN = interf.second.side_IN->IsInto(reference);
					is_referencein_OUT = interf.second.side_OUT->IsInto(reference);
					std::array<ChBody*, 2> ref_bodies{ mbo1,mbo2 };
					for (ChBody* aref_body : ref_bodies) {
						auto ref_body = std::shared_ptr<ChBody>(aref_body, [](ChBody*) {});
						bool is_overlapping_IN = interf.second.side_IN->IsInto(ref_body->GetPos());
						bool is_overlapping_OUT = interf.second.side_OUT->IsInto(ref_body->GetPos());
						InterfaceManageNodeSharedLeaving<ChBody, ChPhysicsItem >(
							ref_body,
							is_overlapping_IN  || is_referencein_IN, // ie. extend aabb of body to include link reference
							is_overlapping_OUT || is_referencein_OUT, // ie. extend aabb of body to include link reference
							ref_body->GetTag(),
							system->GetAssembly().GetTag(),
							shared_ids,
							sent_ids,
							bodies_migrating,
							interf.second.shared_items
						);
					}
				}

				auto mnod1 = dynamic_cast<fea::ChNodeFEAxyzrot*>(blink->GetBody1());
				auto mnod2 = dynamic_cast<fea::ChNodeFEAxyzrot*>(blink->GetBody2());
				if (mnod1 && mnod2) { 
					reference = mnod2->GetPos();
					is_referencein_IN = interf.second.side_IN->IsInto(reference);
					is_referencein_OUT = interf.second.side_OUT->IsInto(reference);
					std::array<ChNodeFEAxyzrot*, 2> ref_nodes{ mnod1,mnod2 };
					for (ChNodeFEAxyzrot* aref_node : ref_nodes) {
						auto ref_node = std::shared_ptr<ChNodeBase>(aref_node, [](ChNodeBase*) {});
						bool is_overlapping_IN = interf.second.side_IN->IsInto(aref_node->GetPos());
						bool is_overlapping_OUT = interf.second.side_OUT->IsInto(aref_node->GetPos());
						InterfaceManageNodeSharedLeaving<ChNodeBase, ChNodeBase >(
							ref_node,
							is_overlapping_IN  || is_referencein_IN, // ie. extend aabb of body to include link reference
							is_overlapping_OUT || is_referencein_OUT, // ie. extend aabb of body to include link reference
							ref_node->GetTag(),
							system->GetAssembly().GetTag(),
							shared_ids,
							sent_ids,
							nodes_migrating,
							interf.second.shared_nodes
						);
					}
				}

				// Serialize the link when needed. Links are never shared.
				if (is_referencein_OUT) {
					if (sent_ids.find(link->GetTag()) == sent_ids.end()) {
						links_migrating.push_back(ChIncrementalObj<ChLinkBase>{system->GetAssembly().GetTag(), link});
						sent_ids.insert(link->GetTag());
					}
				}

			}
		}
		

		// 3-MESHES

		for (const auto& mmesh : system->GetMeshes()) {

			ChAABB mabb = mmesh->GetTotalAABB();
			bool is_overlapping_IN = interf.second.side_IN->IsOverlap(mabb) || mabb.IsInverted(); // if aabb is empty (reversed) assume always overlap
			bool is_overlapping_OUT = interf.second.side_OUT->IsOverlap(mabb) || mabb.IsInverted(); // if aabb is empty (reversed) assume always overlap
			
			InterfaceManageNodeSharedLeaving<ChMesh, ChPhysicsItem >(
				mmesh,
				is_overlapping_IN,
				is_overlapping_OUT,
				mmesh->GetTag(),
				system->GetAssembly().GetTag(),
				shared_ids,
				sent_ids,
				meshes_migrating,
				interf.second.shared_items
			);

			int parent_tag = mmesh->GetTag();

			// FEA NODES
			for (const auto& node : mmesh->GetNodes()) {
				ChVector3d node_pos = node->GetCenter();
				is_overlapping_IN  = interf.second.side_IN->IsInto(node_pos);
				is_overlapping_OUT = interf.second.side_OUT->IsInto(node_pos);
				int mtag = node->GetTag();

				InterfaceManageNodeSharedLeaving<ChNodeBase, ChNodeBase>(
					node,
					is_overlapping_IN,
					is_overlapping_OUT,
					mtag,
					parent_tag,
					shared_ids,
					sent_ids,
					nodes_migrating,
					interf.second.shared_nodes
				);
			}

			// FEA ELEMENTS
			for (const auto& element : mmesh->GetElements()) {
				ChVector3d reference = element->GetNode(0)->GetCenter();
				bool is_referencein_IN  = interf.second.side_IN->IsInto(reference);
				bool is_referencein_OUT = interf.second.side_OUT->IsInto(reference);

				if (is_referencein_OUT) {

					// Serialize the element. Elements are never shared.
					elements_migrating.push_back(ChIncrementalObj<ChElementBase>{mmesh->GetTag(), element});

				}

				// Also serialize and manage shared lists of the connected nodes.
				for (int i = 0; i < (int)element->GetNumNodes(); ++i) {
					const auto ref_node = element->GetNode(i);
					int ref_tag = ref_node->GetTag();
					ChVector3d node_pos;
					if (const auto nodexyz = std::dynamic_pointer_cast<ChNodeFEAxyz>(ref_node))
						node_pos = nodexyz->GetPos();
					if (const auto nodexyzrot = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(ref_node))
						node_pos = nodexyzrot->GetPos();
					bool is_ref_overlapping_IN = interf.second.side_IN->IsInto(node_pos);
					bool is_ref_overlapping_OUT = interf.second.side_OUT->IsInto(node_pos);

					InterfaceManageNodeSharedLeaving<ChNodeBase, ChNodeBase>(
						ref_node,
						is_ref_overlapping_IN  || is_referencein_IN, // ie. extend aabb of node to include element reference
						is_ref_overlapping_OUT || is_referencein_OUT, // ie. extend aabb of node to include element reference
						ref_tag,
						parent_tag,
						shared_ids,
						sent_ids,
						nodes_migrating,
						interf.second.shared_nodes
					);
				}
					
			} // end fea elements

		}


		// - Purge the shared object sets from those that are not shared anymore.
		//   This is not very efficient. Can be optimized.

		interf.second.shared_items.clear();
		interf.second.shared_nodes.clear();

		for (const auto& body : system->GetBodies()) {
			if (shared_ids.find(body->GetTag()) != shared_ids.end())
				interf.second.shared_items[body->GetTag()] = body;
		}
		for (const auto& oitem : system->GetOtherPhysicsItems()) {
			if (shared_ids.find(oitem->GetTag()) != shared_ids.end())
				interf.second.shared_items[oitem->GetTag()] = oitem;
		}
		for (const auto& mmesh : system->GetMeshes()) {
			if (shared_ids.find(mmesh->GetTag()) != shared_ids.end())
				interf.second.shared_items[mmesh->GetTag()] = mmesh;
			for (const auto& node : mmesh->GetNodes()) {
				if (shared_ids.find(node->GetTag()) != shared_ids.end())
					interf.second.shared_nodes[node->GetTag()] = node;
			}
		}



		// - SERIALIZE

		

		// Serialize outbound bodies
		*serializer << CHNVP(bodies_migrating);
		// Serialize outbound links
		*serializer << CHNVP(links_migrating);
		// Serialize outbound otherphysics items
		*serializer << CHNVP(items_migrating);
		// Serialize outbound meshes
		*serializer << CHNVP(meshes_migrating);
		// Serialize outbound nodes 
		*serializer << CHNVP(nodes_migrating);
		// Serialize outbound elements 
		*serializer << CHNVP(elements_migrating);


		// N-HANDSHAKE IDS
		
		// Serialize also the list of items that this domain considered to be shared with the
		// interface neighbour. This will be sent to the neighbour, so it will know that some
		// nodes must be kept in 'shared' mode even if their bounding box is not overlapping (this is the 
		// case of nodes or bodies connected by a ChLink or a ChElement, where the link or element is in the 
		// other domain). [To do: avoid storing in shared_ids the items that are aabb-overlapping on the interface, 
		// as this can be inferred also by the neighbouring domain.]
		*serializer << CHNVP(shared_ids, "shared_ids");


		//std::cout << "\nSERIALIZE domain " << this->GetRank() << " to interface " << interf.second.side_OUT->GetRank() << "\n"; //***DEBUG
		//std::cout << interf.second.buffer_sending.str(); //***DEBUG
		
	}


}


void  ChDomain::DoUpdateSharedReceived() {
	
	// This will be populated by all interfaces with all neighbours
	std::unordered_set<int> set_of_domainshared;

	for (auto& interf : this->interfaces) {

		std::vector<ChIncrementalObj<ChBody>>		bodies_migrating;
		std::vector<ChIncrementalObj<ChPhysicsItem>>items_migrating;
		std::vector<ChIncrementalObj<ChMesh>>		meshes_migrating;
		std::vector<ChIncrementalObj<ChLinkBase>>	links_migrating;
		std::vector<ChIncrementalObj<ChNodeBase>>   nodes_migrating;
		std::vector<ChIncrementalObj<ChElementBase>>elements_migrating;

		// - DESERIALIZE
		
		// prepare the deserializer
		std::shared_ptr<ChArchiveIn> deserializer;
		switch (this->serializer_type) {
		case DomainSerializerFormat::BINARY:
			deserializer = chrono_types::make_shared<ChArchiveInBinary>(interf.second.buffer_receiving); break;
		case DomainSerializerFormat::JSON:
			deserializer = chrono_types::make_shared<ChArchiveInJSON>(interf.second.buffer_receiving); break;
		case DomainSerializerFormat::XML:
			deserializer = chrono_types::make_shared<ChArchiveInXML>(interf.second.buffer_receiving); break;
		default: break;
		}
		
		// Prepare a map of pointers to already existing items that, if referenced by serializer as external IDs, just
		// need to be rebind. 
		// In this case for simplicity and safety we collect all the pointers in this->system, but future implemetation
		// could just collect the pointers of the shared items.
		
		ChArchivePointerMap rebinding_pointers;
		rebinding_pointers << CHNVP(this->system);
		deserializer->ExternalPointersMap() = rebinding_pointers.pointer_map_id_ptr; // will rebuild all pointers between objects with GetTag()
		deserializer->SharedPointersMap() = rebinding_pointers.shared_ptr_map;
		//deserializer->RebindExternalPointer(this->system, this->system->GetTag()); // assuming all domains have ChSystem with same tag

		//***TEST
		/*
		std::cout << "           deserializer ExternalPointersMap size: " << rebinding_pointers.pointer_map_ptr_id.size() << "\n";
		for (auto& m : rebinding_pointers.pointer_map_id_ptr)
			std::cout << "             tag: " << m.first << "   ptr: " << (int)m.second << "\n";
		*/

		// Deserialize inbound bodies
		*deserializer >> CHNVP(bodies_migrating);
		// Deserialize outbound links
		*deserializer >> CHNVP(links_migrating);
		// Deserialize outbound otherphysics items
		*deserializer >> CHNVP(items_migrating);
		// Deserialize outbound otherphysics items
		*deserializer >> CHNVP(meshes_migrating);
		// Deserialize outbound nodes 
		*deserializer >> CHNVP(nodes_migrating);
		// Deserialize outbound elements 
		*deserializer >> CHNVP(elements_migrating);

		// 1-BODIES

		for (const auto& ibody : bodies_migrating) {
			std::shared_ptr<ChBody> body = ibody.obj;
			//std::shared_ptr<ChAssembly> assembly = std::dynamic_pointer_cast<ChAssembly>(interf.second.shared_items[ibody.container_ID]);
			system->AddBody(body);
			system->GetCollisionSystem()->BindItem(body);
		}
		
		// 2-LINKS

		for (const auto& ilink : links_migrating) {
			std::shared_ptr<ChLinkBase> link = ilink.obj;
			//std::shared_ptr<ChAssembly> assembly = std::dynamic_pointer_cast<ChAssembly>(interf.second.shared_items[ilink.container_ID]);
			system->AddLink(link);
		}

		// 3-OTHER PHYSICS ITEMS

		for (const auto& iitem : items_migrating) {
			std::shared_ptr<ChPhysicsItem> oitem = iitem.obj;
			//std::shared_ptr<ChAssembly> assembly = std::dynamic_pointer_cast<ChAssembly>(interf.second.shared_items[ilink.container_ID]);
			system->AddOtherPhysicsItem(oitem);
			system->GetCollisionSystem()->BindItem(oitem);
		}

		// 4-MESHES

		for (const auto& imesh : meshes_migrating) {
			std::shared_ptr<ChMesh> omesh = imesh.obj;
			//std::shared_ptr<ChAssembly> assembly = std::dynamic_pointer_cast<ChAssembly>(interf.second.shared_items[ilink.container_ID]);
			system->AddMesh(omesh);
			system->GetCollisionSystem()->BindItem(omesh);
		}

		// 5-NODES
		for (const auto& inode : nodes_migrating) {
			std::shared_ptr<ChNodeBase> onode = inode.obj;

			for (const auto& imesh : system->GetMeshes()) {
				if (imesh->GetTag() == inode.container_ID) {
					if (auto feanode = std::dynamic_pointer_cast<ChNodeFEAbase>(onode))
						imesh->AddNode(feanode);
					//system->GetCollisionSystem()->BindItem(feanode);
					break;
				}
			}
		}

		// 6-ELEMENTS
		for (const auto& ielement : elements_migrating) {
			std::shared_ptr<ChElementBase> oelem = ielement.obj;

			for (auto imesh : system->GetMeshes()) {
				if (imesh->GetTag() == ielement.container_ID) {
					imesh->AddElement(oelem);
					break;
				}
			}
		}

		// HANDSHAKE IDS

		// De-serialize also the list of items that the sending domain considered to be shared with the
		// receiving domain. So here we will know that some nodes must be kept in 'shared' mode even if 
		// their bounding box is not overlapping the interface (this is the case of nodes or bodies connected by a 
		// ChLink or a ChElement, where the link or element is in the other domain).

		std::unordered_set<int> shared_ids_incoming;
		*deserializer >> CHNVP(shared_ids_incoming,"shared_ids");

		for (const auto& body : system->GetBodies()) {
			if (shared_ids_incoming.find(body->GetTag()) != shared_ids_incoming.end())
				interf.second.shared_items[body->GetTag()] = body;
		}
		for (const auto& mmesh : system->GetMeshes()) {
			if (shared_ids_incoming.find(mmesh->GetTag()) != shared_ids_incoming.end())
				interf.second.shared_items[mmesh->GetTag()] = mmesh;
			for (const auto& node : mmesh->GetNodes()) {
				if (shared_ids_incoming.find(node->GetTag()) != shared_ids_incoming.end())
					interf.second.shared_nodes[node->GetTag()] = node;
			}
		}
		for (const auto& oitem : system->GetOtherPhysicsItems()) {
			if (shared_ids_incoming.find(oitem->GetTag()) != shared_ids_incoming.end())
				interf.second.shared_items[oitem->GetTag()] = oitem;
		}


		for (auto& mi : interf.second.shared_items)
			set_of_domainshared.insert(mi.first);
		for (auto& mn : interf.second.shared_nodes)
			set_of_domainshared.insert(mn.first);


		// KEEP TRACK OF SHARED VARS
		//
		// This builds the list of shared variables (ChVariable objects used by system descriptors)
		// by selecting only those that belong to items that are in the shared maps.
		// The shared_items containers are of ordered type, ordered by tag ID, so we can be sure 
		// that the order of the variables in shared_vars is the same also on the other side, in the
		// neighbouring domain. 

		ChSystemDescriptor temp_descr;
		
		for (auto& mshitem : interf.second.shared_items) {

			if (auto mmesh = std::dynamic_pointer_cast<fea::ChMesh>(mshitem.second))
				continue; // if mesh, do not share all variables but later InjectVariables of shared nodes only

			mshitem.second->InjectVariables(temp_descr);
		}
		for (auto& mshnode : interf.second.shared_nodes) {
			mshnode.second->InjectVariables(temp_descr);
		}

		interf.second.shared_vars.clear();
		interf.second.shared_vars = temp_descr.GetVariables();

	}

	//
	// Removal of objects that are not anymore overlapping this domain
	//

	std::vector<std::shared_ptr<chrono::fea::ChMesh>> meshes_to_remove;
	for (const auto& mmesh : system->GetMeshes()) {

		// remove nodes spilling outside
		std::vector<std::shared_ptr<chrono::fea::ChNodeFEAbase>> nodes_to_remove;
		for (const auto& node : mmesh->GetNodes()) {
			ChVector3d node_pos = node->GetCenter();
			bool is_overlapping_IN = this->IsInto(node_pos);
			bool is_sharing = (set_of_domainshared.find(node->GetTag()) != set_of_domainshared.end());
			if (!is_overlapping_IN && !is_sharing){
				nodes_to_remove.push_back(node);
			}
		}
		for (const auto& delnode : nodes_to_remove)
			mmesh->RemoveNode(delnode); // can be made more efficient - this require N searches in vector container

		// remove elements spilling outside
		std::vector<std::shared_ptr<chrono::fea::ChElementBase>> elements_to_remove;
		for (const auto& element : mmesh->GetElements()) {
			ChVector3d reference = element->GetNode(0)->GetCenter();
			bool is_reference_IN = this->IsInto(reference);
			//bool is_sharing = (set_of_domainshared.find(element->GetTag()) != set_of_domainshared.end());
			if (!is_reference_IN) {
				elements_to_remove.push_back(element);
			}
		}
		for (const auto& delelement : elements_to_remove)
			mmesh->RemoveElement(delelement); // can be made more efficient - this require N searches in vector container

		// remove the mesh container fully spilling outside 
		auto mabb = mmesh->GetTotalAABB();
		bool is_overlapping_IN = this->IsOverlap(mabb); 
		bool is_sharing = (set_of_domainshared.find(mmesh->GetTag()) != set_of_domainshared.end());
		if (!is_overlapping_IN && !is_sharing)
		{
			meshes_to_remove.push_back(mmesh);
			// unneeded?
			for (auto& interf : this->interfaces) {
				interf.second.shared_items.erase(mmesh->GetTag());
			}
		}
	}
	for (const auto& delmesh : meshes_to_remove)
		system->RemoveMesh(delmesh); // can be made more efficient - this require N searches in vector container

	std::vector<std::shared_ptr<chrono::ChPhysicsItem>> items_to_remove;
	for (const auto& oitem : system->GetOtherPhysicsItems()) {
		// remove the generic physics item spilling outside (could be a ChMesh container)
		auto mabb = oitem->GetTotalAABB();
		bool is_overlapping_IN = this->IsOverlap(mabb);
		bool is_sharing = (set_of_domainshared.find(oitem->GetTag()) != set_of_domainshared.end());
		if (!is_overlapping_IN && !is_sharing)
		{
			items_to_remove.push_back(oitem);
			// unneeded?
			for (auto& interf : this->interfaces) {
				interf.second.shared_items.erase(oitem->GetTag());
			}
		}
	}
	for (const auto& delitem : items_to_remove)
		system->RemoveOtherPhysicsItem(delitem); // can be made more efficient - this require N searches in vector container

	std::vector<std::shared_ptr<chrono::ChBody>> bodies_to_remove;
	for (const auto& body : system->GetBodies()) {
		auto mabb = body->GetTotalAABB();
		bool is_overlapping_IN = this->IsOverlap(mabb) || this->IsInto(body->GetPos()); // if aabb is empty (reversed) use center
		bool is_sharing = (set_of_domainshared.find(body->GetTag()) != set_of_domainshared.end());
		if (!is_overlapping_IN && !is_sharing)
		{
			bodies_to_remove.push_back(body);
			// unneeded?
			for (auto& interf : this->interfaces) {
				interf.second.shared_items.erase(body->GetTag());
			}
		}
	}
	for (const auto& delbody : bodies_to_remove)
		system->RemoveBody(delbody); // can be made more efficient - this require N searches in vector container

	std::vector<std::shared_ptr<chrono::ChLinkBase>> links_to_remove;
	for (const auto& link : system->GetLinks()) {
		if (auto blink = std::dynamic_pointer_cast<ChLink>(link)) {
			ChVector3d reference = blink->GetBody2()->GetPos();
			bool is_referencein_IN = this->IsInto(reference);
			// Delete the link if outside. Links are never shared.
			if (!is_referencein_IN) {
				links_to_remove.push_back(link);
			}
		}
	}
	for (const auto& dellink : links_to_remove)
		system->RemoveLink(dellink); // can be made more efficient - this require N searches in vector container

}




}  // end namespace multidomain
}  // end namespace chrono
