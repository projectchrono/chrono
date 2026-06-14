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
#include "chrono_multidomain/ChDomain.h"
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

static ChVector3d GetNodeCenter(const std::shared_ptr<ChNodeBase>& node) {
    if (!node)
        return ChVector3d(0, 0, 0);
    if (auto nxyzrot = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(node))
        return nxyzrot->GetPos();
    if (auto nxyz = std::dynamic_pointer_cast<ChNodeFEAxyz>(node))
        return nxyz->GetPos();
    return ChVector3d(0, 0, 0);
}

static ChVector3d GetNodeCenter(const std::shared_ptr<ChNodeFEAbase>& node) {
    if (!node)
        return ChVector3d(0, 0, 0);
    if (auto nxyzrot = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(node))
        return nxyzrot->GetPos();
    if (auto nxyz = std::dynamic_pointer_cast<ChNodeFEAxyz>(node))
        return nxyz->GetPos();
    return ChVector3d(0, 0, 0);
}

static ChVector3d GetElementCenter(const std::shared_ptr<ChElementBase>& element) {
    if (!element)
        return ChVector3d(0, 0, 0);
    const int nnodes = static_cast<int>(element->GetNumNodes());
    if (nnodes <= 0)
        return ChVector3d(0, 0, 0);

    ChVector3d center(0, 0, 0);
    int count = 0;
    for (int i = 0; i < nnodes; ++i) {
        auto node = element->GetNode(i);
        if (!node)
            continue;
        center += GetNodeCenter(node);
        ++count;
    }
    if (count <= 0)
        return ChVector3d(0, 0, 0);
    return center / static_cast<double>(count);
}

static std::uint64_t MakeSharedVarKey(bool is_node, int tag, int local_var_index, int dof) {
    // [63]=node/item, [62:31]=tag, [30:15]=local variable index, [14:0]=dof (capped to 15 bits)
    const std::uint64_t type_bit = is_node ? (1ull << 63) : 0ull;
    const std::uint64_t tag_bits = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(tag)) & 0xFFFFFFFFull) << 31;
    const std::uint64_t local_bits = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(local_var_index)) & 0xFFFFull) << 15;
    const std::uint64_t dof_bits = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(dof)) & 0x7FFFull);
    return type_bit | tag_bits | local_bits | dof_bits;
}

static bool SharedVarKeyIsNode(std::uint64_t key) {
    return (key >> 63) != 0ull;
}

static int SharedVarKeyTag(std::uint64_t key) {
    const std::uint32_t raw = static_cast<std::uint32_t>((key >> 31) & 0xFFFFFFFFull);
    return static_cast<int>(raw);
}

static int SharedVarKeyLocalIndex(std::uint64_t key) {
    return static_cast<int>((key >> 15) & 0xFFFFull);
}



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

	if (this->IsMaster() && !this->domain_manager->master_domain_enabled)
		return;

	static int debug_print_leaving = 0;

	// Select objects to sent to surrounding domains 
	for (auto& interf : this->interfaces) {

		if (interf.second.side_OUT->IsMaster() && !this->domain_manager->master_domain_enabled)
			continue;

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
		serializer->SetUseVersions(false);
		serializer->SetEmptyShallowContainers(true);	// we will take care of contained items one by one
		serializer->SetUseGetTagAsID(true);				// GetTag of items, when available, will be used to generate unique IDs in serialization
	
		// Prepare a map of pointers to shared items so that they are not serialized but just referenced
		// by tag. For this reason we traverse all the pointers in the shared items, and ChArchivePointerMap will capture their tags.
		// Note that these shared items might contain pointers to auxiliary structures like visual models,
		// contact materials etc.; if these structures have a GetTag(), the ChArchivePointerMap will capture these IDs too,
		// which is good because if the shared nodes are also in the neighbouring domain, so will be also those aux structures
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
		// Also assume that there is a shared system across all domains, ie. 
		// assuming *all* domains already contain a ChSystem with same GetTag. 
		// (and avoids propagation of serialization to parent ChSystem when back pointers to system are found).
		serializer->UnbindExternalPointer(this->system, this->system->GetTag());

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
				ChVector3d node_pos = GetNodeCenter(node);
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
						const int nnodes = static_cast<int>(element->GetNumNodes());
						if (nnodes <= 0)
							continue;

						// Use element centroid for ownership/migration so element-node ordering does not
						// influence partitioning (GetNode(0) can cause unstable ownership flips).
						const ChVector3d element_center = GetElementCenter(element);
						const bool is_center_IN = interf.second.side_IN->IsInto(element_center);
						const bool is_center_OUT = interf.second.side_OUT->IsInto(element_center);

						bool element_overlaps_IN = false;
						bool element_overlaps_OUT = false;
						for (int i = 0; i < nnodes; ++i) {
							const auto ref_node = element->GetNode(i);
							if (!ref_node)
								continue;
							const ChVector3d node_pos = GetNodeCenter(ref_node);
							element_overlaps_IN = element_overlaps_IN || interf.second.side_IN->IsInto(node_pos);
							element_overlaps_OUT = element_overlaps_OUT || interf.second.side_OUT->IsInto(node_pos);
						}

						// Serialize element only when ownership center moved to OUT and no longer IN.
						if (is_center_OUT && !is_center_IN) {
							elements_migrating.push_back(ChIncrementalObj<ChElementBase>{mmesh->GetTag(), element});
						}

						// Also serialize and manage shared lists of the connected nodes.
						for (int i = 0; i < nnodes; ++i) {
							const auto ref_node = element->GetNode(i);
							if (!ref_node)
								continue;
							int ref_tag = ref_node->GetTag();
	                        	ChVector3d node_pos = GetNodeCenter(ref_node);
							bool is_ref_overlapping_IN = interf.second.side_IN->IsInto(node_pos);
							bool is_ref_overlapping_OUT = interf.second.side_OUT->IsInto(node_pos);
							InterfaceManageNodeSharedLeaving<ChNodeBase, ChNodeBase>(
								ref_node,
								is_ref_overlapping_IN || element_overlaps_IN || is_center_IN,
								is_ref_overlapping_OUT || element_overlaps_OUT || is_center_OUT,
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
				if (shared_ids.find(node->GetTag()) != shared_ids.end()) {
					const int ntag = node->GetTag();
					if (interf.second.shared_nodes.find(ntag) == interf.second.shared_nodes.end()) {
						interf.second.shared_nodes.emplace(ntag, node);
					}
				}
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
		// NOTE: Chrono archive does not provide std::unordered_set serialization wrappers.
		// Serialize shared IDs as a vector on the wire, then reconstruct a set on receive.
		std::vector<int> shared_ids_wire(shared_ids.begin(), shared_ids.end());
		*serializer << CHNVP(shared_ids_wire, "shared_ids");


		//std::cout << "\nSERIALIZE domain " << this->GetRank() << " to interface " << interf.second.side_OUT->GetRank() << "\n"; //***DEBUG
		//std::cout << interf.second.buffer_sending.str(); //***DEBUG
		
	}


}


void  ChDomain::DoUpdateSharedReceived(bool delete_outsiders) {
	
	if (this->IsMaster() && !this->domain_manager->master_domain_enabled)
		return;

	static int debug_print_received = 0;

	// This will be populated by all interfaces with all neighbours
	std::unordered_set<int> set_of_domainshared;

	for (auto& interf : this->interfaces) {

		if (interf.second.side_OUT->IsMaster() && !this->domain_manager->master_domain_enabled)
			continue;

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
		deserializer->SetUseVersions(false);

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
			if (rebinding_pointers.pointer_map_id_ptr.find(body->GetTag()) != rebinding_pointers.pointer_map_id_ptr.end())
				continue;
			//std::shared_ptr<ChAssembly> assembly = std::dynamic_pointer_cast<ChAssembly>(interf.second.shared_items[ibody.container_ID]);
			system->AddBody(body);
			system->GetCollisionSystem()->BindItem(body);
		}
		
		// 2-LINKS

		for (const auto& ilink : links_migrating) {
			std::shared_ptr<ChLinkBase> link = ilink.obj;
			if (rebinding_pointers.pointer_map_id_ptr.find(link->GetTag()) != rebinding_pointers.pointer_map_id_ptr.end())
				continue;
			//std::shared_ptr<ChAssembly> assembly = std::dynamic_pointer_cast<ChAssembly>(interf.second.shared_items[ilink.container_ID]);
			system->AddLink(link);
		}

		// 3-OTHER PHYSICS ITEMS

		for (const auto& iitem : items_migrating) {
			std::shared_ptr<ChPhysicsItem> oitem = iitem.obj;
			if (rebinding_pointers.pointer_map_id_ptr.find(oitem->GetTag()) != rebinding_pointers.pointer_map_id_ptr.end())
				continue;
			//std::shared_ptr<ChAssembly> assembly = std::dynamic_pointer_cast<ChAssembly>(interf.second.shared_items[ilink.container_ID]);
			system->AddOtherPhysicsItem(oitem);
			system->GetCollisionSystem()->BindItem(oitem);
		}

		// 4-MESHES

		for (const auto& imesh : meshes_migrating) {
			std::shared_ptr<ChMesh> omesh = imesh.obj;
			if (rebinding_pointers.pointer_map_id_ptr.find(omesh->GetTag()) != rebinding_pointers.pointer_map_id_ptr.end())
				continue;
			//std::shared_ptr<ChAssembly> assembly = std::dynamic_pointer_cast<ChAssembly>(interf.second.shared_items[ilink.container_ID]);
			system->AddMesh(omesh);
			system->GetCollisionSystem()->BindItem(omesh);
		}

		// 5-NODES
		for (const auto& inode : nodes_migrating) {
			std::shared_ptr<ChNodeBase> onode = inode.obj;
			if (rebinding_pointers.pointer_map_id_ptr.find(onode->GetTag()) != rebinding_pointers.pointer_map_id_ptr.end())
				continue;
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

		std::vector<int> shared_ids_incoming_wire;
		*deserializer >> CHNVP(shared_ids_incoming_wire, "shared_ids");
		std::unordered_set<int> shared_ids_incoming;
		shared_ids_incoming.reserve(shared_ids_incoming_wire.size() * 2 + 1);
		for (int sid : shared_ids_incoming_wire)
			shared_ids_incoming.insert(sid);

		for (const auto& body : system->GetBodies()) {
			if (shared_ids_incoming.find(body->GetTag()) != shared_ids_incoming.end())
				interf.second.shared_items[body->GetTag()] = body;
		}
		for (const auto& mmesh : system->GetMeshes()) {
			if (shared_ids_incoming.find(mmesh->GetTag()) != shared_ids_incoming.end())
				interf.second.shared_items[mmesh->GetTag()] = mmesh;
			for (const auto& node : mmesh->GetNodes()) {
				if (shared_ids_incoming.find(node->GetTag()) != shared_ids_incoming.end()) {
					const int ntag = node->GetTag();
					if (interf.second.shared_nodes.find(ntag) == interf.second.shared_nodes.end()) {
						interf.second.shared_nodes.emplace(ntag, node);
					}
				}
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
            // Preserve original variable ordering as injected by shared items/nodes.
            // In parallel, assign a stable key per appended variable entry so multidomain
            // exchange can validate/remap consistently across interfaces.
            ChSystemDescriptor temp_descr;
            interf.second.shared_var_keys.clear();

            for (auto& mshitem : interf.second.shared_items) {
                if (std::dynamic_pointer_cast<fea::ChMesh>(mshitem.second))
                    continue;  // for meshes, share node vars explicitly via shared_nodes

                const size_t before = temp_descr.GetVariables().size();
                mshitem.second->InjectVariables(temp_descr);
                auto vars_now = temp_descr.GetVariables();
                for (size_t iv = before; iv < vars_now.size(); ++iv) {
                    const int local_idx = static_cast<int>(iv - before);
                    interf.second.shared_var_keys.push_back(
                        MakeSharedVarKey(false, mshitem.first, local_idx, vars_now[iv]->GetDOF()));
                }
            }
            for (auto& mshnode : interf.second.shared_nodes) {
                const size_t before = temp_descr.GetVariables().size();
                mshnode.second->InjectVariables(temp_descr);
                auto vars_now = temp_descr.GetVariables();
                for (size_t iv = before; iv < vars_now.size(); ++iv) {
                    const int local_idx = static_cast<int>(iv - before);
                    interf.second.shared_var_keys.push_back(
                        MakeSharedVarKey(true, mshnode.first, local_idx, vars_now[iv]->GetDOF()));
                }
            }

            // Deduplicate shared variables (mesh + node injection can add repeated entries).
            auto raw_shared_vars = temp_descr.GetVariables();
            std::vector<ChVariables*> unique_shared_vars;
            std::vector<std::uint64_t> unique_shared_keys;
            unique_shared_vars.reserve(raw_shared_vars.size());
            unique_shared_keys.reserve(raw_shared_vars.size());
            std::unordered_set<ChVariables*> seen_shared_vars;
            seen_shared_vars.reserve(raw_shared_vars.size() * 2 + 1);
            std::unordered_set<std::uint64_t> seen_shared_keys;
            seen_shared_keys.reserve(raw_shared_vars.size() * 2 + 1);

            for (size_t iv = 0; iv < raw_shared_vars.size(); ++iv) {
                auto* v = raw_shared_vars[iv];
                if (!v)
                    continue;

                std::uint64_t key = 0ull;
                if (iv < interf.second.shared_var_keys.size())
                    key = interf.second.shared_var_keys[iv];
                if (key == 0ull) {
                    key = MakeSharedVarKey(false, static_cast<int>(iv), 0, v->GetDOF());
                }

                bool keep = true;
                if (key != 0ull) {
                    keep = seen_shared_keys.insert(key).second;
                } else {
                    keep = seen_shared_vars.insert(v).second;
                }
                if (!keep)
                    continue;

                unique_shared_vars.push_back(v);
                unique_shared_keys.push_back(key);
            }

            interf.second.shared_vars.swap(unique_shared_vars);
            interf.second.shared_var_keys.swap(unique_shared_keys);

			if (debug_print_received < 2) {
				std::cout << "[mdom-received] rank=" << this->rank
					<< " from=" << interf.second.side_OUT->GetRank()
					<< " incoming_shared_ids=" << shared_ids_incoming.size()
					<< " shared_items=" << interf.second.shared_items.size()
					<< " shared_nodes=" << interf.second.shared_nodes.size()
					<< " shared_vars=" << interf.second.shared_vars.size()
					<< std::endl;
			}

        }

	if (debug_print_received < 2)
		++debug_print_received;

	//
	// Removal of objects that are not anymore overlapping this domain
	//

	if (delete_outsiders) {
		std::vector<std::shared_ptr<chrono::fea::ChMesh>> meshes_to_remove;
		for (const auto& mmesh : system->GetMeshes()) {

			// Build a conservative candidate set of nodes that are outside and not shared.
			// Elements connected to any such node must be removed first to avoid dangling element-node connectivity.
			std::unordered_set<int> outside_nonshared_node_tags;
			for (const auto& node : mmesh->GetNodes()) {
				ChVector3d node_pos = GetNodeCenter(node);
				bool is_overlapping_IN = this->IsInto(node_pos);
				bool is_sharing = (set_of_domainshared.find(node->GetTag()) != set_of_domainshared.end());
				if (!is_overlapping_IN && !is_sharing) {
					outside_nonshared_node_tags.insert(node->GetTag());
				}
			}

			// remove elements spilling outside (or connected to outside nonshared nodes)
			std::vector<std::shared_ptr<chrono::fea::ChElementBase>> elements_to_remove;
			std::unordered_set<int> nodes_used_by_kept_elements;
			for (const auto& element : mmesh->GetElements()) {
				bool remove_element = false;
				const int nnodes = static_cast<int>(element->GetNumNodes());
				if (nnodes <= 0) {
					remove_element = true;
				}
				for (int i = 0; i < nnodes && !remove_element; ++i) {
					auto enode = element->GetNode(i);
					if (!enode) {
						remove_element = true;
						break;
					}
					const int ntag = enode->GetTag();
					if (outside_nonshared_node_tags.find(ntag) != outside_nonshared_node_tags.end()) {
						remove_element = true;
						break;
					}
				}

						// Keep legacy ownership culling as an additional conservative filter.
						if (!remove_element) {
							ChVector3d reference = GetElementCenter(element);
							bool is_reference_IN = this->IsInto(reference);
							if (!is_reference_IN) {
								remove_element = true;
						}
					}

				if (remove_element) {
					elements_to_remove.push_back(element);
				} else {
					for (int i = 0; i < nnodes; ++i) {
						auto enode = element->GetNode(i);
						if (enode) {
							nodes_used_by_kept_elements.insert(enode->GetTag());
						}
					}
				}
			}
			for (const auto& delelement : elements_to_remove)
				mmesh->RemoveElement(delelement); // can be made more efficient - this require N searches in vector container

			// remove nodes spilling outside, but keep nodes still used by kept elements
			std::vector<std::shared_ptr<chrono::fea::ChNodeFEAbase>> nodes_to_remove;
			for (const auto& node : mmesh->GetNodes()) {
				ChVector3d node_pos = GetNodeCenter(node);
				bool is_overlapping_IN = this->IsInto(node_pos);
				bool is_sharing = (set_of_domainshared.find(node->GetTag()) != set_of_domainshared.end());
				const bool is_used_by_kept_element =
					(nodes_used_by_kept_elements.find(node->GetTag()) != nodes_used_by_kept_elements.end());
				if (!is_overlapping_IN && !is_sharing && !is_used_by_kept_element) {
					nodes_to_remove.push_back(node);
				}
			}
			for (const auto& delnode : nodes_to_remove)
				mmesh->RemoveNode(delnode); // can be made more efficient - this require N searches in vector container

			// remove the mesh container fully spilling outside 
			auto mabb = mmesh->GetTotalAABB();
			bool is_overlapping_IN = this->IsOverlap(mabb);
			bool is_sharing = (set_of_domainshared.find(mmesh->GetTag()) != set_of_domainshared.end());
			if (!is_overlapping_IN && !is_sharing)
			{
				meshes_to_remove.push_back(mmesh);
				// unneeded?
				//for (auto& interf : this->interfaces) {
				//	interf.second.shared_items.erase(mmesh->GetTag());
				//}
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
				//for (auto& interf : this->interfaces) {
				//	interf.second.shared_items.erase(oitem->GetTag());
				//}
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

	
			// Update the vector with scaling weights for masses of shared variables.
			// This is used because objects that are shared between N boundaries must
			// have their mass split too, if a FETI-like domain decomposition approach is used.
			system->Setup();
			// ChSystem::Setup updates assembly offsets but does not repopulate descriptor variables.
			// Rebuild descriptor bookkeeping here so shared ChVariables have valid offsets.
			auto descriptor = system->GetSystemDescriptor();
			if (descriptor) {
				descriptor->BeginInsertion();
				system->InjectConstraints(*descriptor);
				system->InjectVariables(*descriptor);
				system->InjectKRMMatrices(*descriptor);
				descriptor->EndInsertion();
			}

			// Diagnostic: validate that shared vars still belong to the active descriptor list
			// after migration/setup; stale pointers would keep default offsets (typically 0).
			static int debug_shared_var_membership = 0;
			if (debug_shared_var_membership < 2) {
				auto descriptor = system->GetSystemDescriptor();
				std::unordered_set<ChVariables*> descriptor_vars;
				if (descriptor) {
					const auto& vars = descriptor->GetVariables();
					descriptor_vars.reserve(vars.size() * 2 + 1);
					for (auto* v : vars) {
						if (v)
							descriptor_vars.insert(v);
					}
				}

				int shared_total = 0;
				int shared_active = 0;
				int shared_active_in_descriptor = 0;
				int shared_active_off0 = 0;
				int shared_active_nonzero = 0;
				for (const auto& interf : this->interfaces) {
					for (auto* v : interf.second.shared_vars) {
						if (!v)
							continue;
						++shared_total;
						if (!v->IsActive())
							continue;
						++shared_active;
						if (v->GetOffset() == 0)
							++shared_active_off0;
						else
							++shared_active_nonzero;
						if (descriptor_vars.find(v) != descriptor_vars.end())
							++shared_active_in_descriptor;
					}
				}

				std::cout << "[mdom-shared-var-membership] rank=" << this->rank
					<< " call=" << (debug_shared_var_membership + 1)
					<< " descriptor_vars=" << descriptor_vars.size()
					<< " shared_total=" << shared_total
					<< " shared_active=" << shared_active
					<< " shared_active_in_descriptor=" << shared_active_in_descriptor
					<< " shared_active_off0=" << shared_active_off0
					<< " shared_active_nonzero=" << shared_active_nonzero
					<< std::endl;
				++debug_shared_var_membership;
			}

			this->ComputeSharedCoordsWeights(this->CoordWeightsWv());
			}




void ChDomain::ComputeSharedCoordsCounts(ChVectorDynamic<>& N) {
	N.setOnes(this->system->GetNumCoordsVelLevel());

	for (auto& interf : this->interfaces) {
		if (interf.second.side_OUT->IsMaster() && !this->domain_manager->master_domain_enabled)
			continue;

		// Count each coordinate at most once per interface. This prevents duplicated
		// shared_vars entries (from migration/serialization artifacts) from inflating
		// multiplicity and softening the MDOM response.
		std::unordered_set<int> counted_coords_this_interface;
		counted_coords_this_interface.reserve(interf.second.shared_vars.size() * 4 + 1);
		for (auto avar : interf.second.shared_vars) {
			if (!avar->IsActive())
				continue;
			for (int i = 0; i < avar->GetDOF(); ++i) {
				const int idx = avar->GetOffset() + i;
				if (idx >= 0 && idx < N.size() && counted_coords_this_interface.insert(idx).second)
					N(idx) += 1.0;
			}
		}
	}

	// Diagnostic: local-only counters (no collective reduction here to avoid partition-update deadlocks).
	static int diag_print_count = 0;
	if (diag_print_count < 2) {
		double local_shared = 0.0;
		double local_ge3 = 0.0;
		double local_sum_mult = 0.0;
		double local_max_mult = 1.0;
		for (int i = 0; i < N.size(); ++i) {
			const double c = N(i);
			if (c > local_max_mult)
				local_max_mult = c;
			if (c > 1.0) {
				local_shared += 1.0;
				local_sum_mult += c;
				if (c >= 3.0)
					local_ge3 += 1.0;
			}
		}
		const double frac_shared = (N.size() > 0) ? (local_shared / static_cast<double>(N.size())) : 0.0;
		const double avg_mult_shared = (local_shared > 0.0) ? (local_sum_mult / local_shared) : 1.0;
		std::cout << "[mdom-shared-counts-local] rank=" << this->rank
			<< " call=" << (diag_print_count + 1)
			<< " total_coords=" << N.size()
			<< " shared_coords=" << static_cast<long long>(local_shared)
			<< " frac_shared=" << frac_shared
			<< " avg_multiplicity_shared=" << avg_mult_shared
			<< " coords_mult_ge3=" << static_cast<long long>(local_ge3)
			<< " max_multiplicity=" << local_max_mult << std::endl;
		diag_print_count++;
	}
}

void ChDomain::ComputeSharedCoordsWeights(ChVectorDynamic<>&N) {

	ComputeSharedCoordsCounts(N);
	for (int i = 0; i < N.size(); ++i) {
		if (N(i) > 0)
			N(i) = 1.0 / N(i);
		else
			N(i) = 1.0;
	}
}

}  // end namespace multidomain
}  // end namespace chrono
