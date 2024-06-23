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


#include "chrono_multidomain/ChDomainManager.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/fea/ChElementBase.h"

namespace chrono {
namespace multidomain {

using namespace fea;


bool ChDomainManagerSharedmemory::DoUpdateSharedLeaving() {
	for (auto& mdomain : this->domains) {
		mdomain.second->DoUpdateSharedLeaving();
	}
	return true;
}

bool ChDomainManagerSharedmemory::DoDomainsSendReceive() {
	for (auto& mdomain : this->domains) {
		for (auto& minterface : mdomain.second->GetInterfaces()) {
			// receive from neighbour domain to this domain:
			minterface.second.buffer_receiving << this->domains[minterface.second.side_OUT->GetRank()]->GetInterfaces()[mdomain.second->GetRank()].buffer_sending.rdbuf();
			// send from this domain to neighbour domain: not needed because done when the previous line will be called for the neighbour. 
		}
	}
	return true;
}

bool ChDomainManagerSharedmemory::DoUpdateSharedReceived() {
	for (auto& mdomain : this->domains) {
		mdomain.second->DoUpdateSharedReceived();
	}
	return true;
}

void ChDomainManagerSharedmemory::AddDomain(std::shared_ptr<ChDomain> mdomain) {
	domains[mdomain->GetRank()] = mdomain;
}

//////////////////////////////////////////////////////////////////////////////////////////////




ChDomainBuilderSlices::ChDomainBuilderSlices(
											int tot_ranks,
											double mmin,
											double mmax,
											ChAxis maxis) {
	this->axis = maxis;
	domains_bounds.push_back(-1e34);
	for (int islice = 0; islice < (tot_ranks - 1); ++islice) {
		domains_bounds.push_back(mmin + (islice + 1.0) * (mmax - mmin) / (double)tot_ranks);
	}
	domains_bounds.push_back(1e34);
};

ChDomainBuilderSlices::ChDomainBuilderSlices(
											std::vector<double> axis_cuts,
											ChAxis maxis) {
	this->axis = maxis;
	domains_bounds.push_back(-1e34);
	for (double dcut : axis_cuts) {
		domains_bounds.push_back(dcut);
	}
	domains_bounds.push_back(1e34);

}


std::shared_ptr<ChDomain> ChDomainBuilderSlices::BuildDomain(
	ChSystem* msys,
	int this_rank) {

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

	return domain;
}


//////////////////////////////////////////////////////////////////////////////////////////7



void ChDomain::DoUpdateSharedLeaving() {

	

	// Select objects to sent to surrounding domains 
	for (auto& interf : this->interfaces) {

		std::unordered_set<int> shared_ids;
		std::unordered_set<int> sent_ids;

		// prepare the serializer
		interf.second.buffer_sending.clear();
		interf.second.buffer_receiving.clear();
		ChArchiveOutBinary serializer(interf.second.buffer_sending);
		serializer.SetEmptyShallowContainers(true);		// we will take care of contained items one by one
		serializer.SetUseGetTagAsID(true);				// GetTag of items, when available, will be used to generate unique IDs in serialization
		serializer.CutPointers().insert(this->system);  // avoid propagation of serialization to parent system
		
		std::vector<std::shared_ptr<ChBody>>		bodies_to_send;
		std::vector<std::shared_ptr<ChPhysicsItem>>	items_to_send;
		std::vector<std::shared_ptr<ChLinkBase>>	links_to_send;
		std::vector<std::shared_ptr<ChNodeBase>>    nodes_to_send;
		std::vector<std::shared_ptr<fea::ChElementBase>> elements_to_send;

		// 1-BODIES overlapping because of collision shapes 
		for (const auto& body : system->GetBodies()) {
			ChAABB mabb = body->GetTotalAABB();
			bool is_overlapping_A = interf.second.side_IN->IsOverlap(mabb) || interf.second.side_IN->IsInto(body->GetPos()); // if aabb is empty (reversed) use center
			bool is_overlapping_B = interf.second.side_OUT->IsOverlap(mabb) || interf.second.side_OUT->IsInto(body->GetPos()); // if aabb is empty (reversed) use center
			bool is_sharing_AB = (interf.second.shared_items.find(body->GetTag()) != interf.second.shared_items.end());
			// if jumps completely in B domain without passing in shared state:
			if (is_overlapping_B && !is_overlapping_A && !is_sharing_AB) {
				bodies_to_send.push_back(body);
				sent_ids.insert(body->GetTag());
			}
			// if is in shared state between A and B:
			if (is_overlapping_B && is_overlapping_A) {
				shared_ids.insert(body->GetTag());
				if (!is_sharing_AB) {
					// enter shared state
					interf.second.shared_items[body->GetTag()] = body;
					bodies_to_send.push_back(body);
					sent_ids.insert(body->GetTag());
				}
			}
			else if (is_sharing_AB) {
				// exit shared state
				interf.second.shared_items.erase(body->GetTag());
			}
			
		}

		// 2-LINKS (and BODIES overlapping because of connected links)
		for (const auto& link : system->GetLinks()) {
			if (auto blink = std::dynamic_pointer_cast<ChLink>(link)) {
				int tag1 = 0;
				int tag2 = 0;
				int tag_link = blink->GetTag();
				ChAABB mabb;
				ChVector3d reference;
				bool is_referencein_A = false;
				bool is_referencein_B = false;

				auto mbo1 = dynamic_cast<ChBody*>(blink->GetBody1());
				auto mbo2 = dynamic_cast<ChBody*>(blink->GetBody2());
				if (mbo1 && mbo2) {
					tag1 = mbo1->GetTag();
					tag2 = mbo2->GetTag();
					mabb.Inflate(mbo1->GetPos());
					mabb.Inflate(mbo2->GetPos());
					reference = mbo2->GetPos();

					is_referencein_A = interf.second.side_IN->IsInto(reference);
					is_referencein_B = interf.second.side_OUT->IsInto(reference);
					bool is_1_in_A = interf.second.side_IN->IsInto(mbo1->GetPos());
					bool is_1_in_B = interf.second.side_OUT->IsInto(mbo1->GetPos());
					bool is_2_in_A = interf.second.side_IN->IsInto(mbo2->GetPos());
					bool is_2_in_B = interf.second.side_OUT->IsInto(mbo2->GetPos());
					bool is_1_sharing_AB = (interf.second.shared_items.find(tag1) != interf.second.shared_items.end());
					bool is_2_sharing_AB = (interf.second.shared_items.find(tag2) != interf.second.shared_items.end());
					
					if (is_referencein_A && is_1_in_B) {
						shared_ids.insert(tag1);
						if (!is_1_sharing_AB) {
							interf.second.shared_items[tag1] = std::shared_ptr<ChBody>(mbo1, [](ChBody*) {});
							//bodies_to_send.push_back(std::shared_ptr<ChBody>(mbo1, [](ChBody*) {})); // hack: use null deleter to wrap pointer
							//sent_ids.insert(tag1);
						}
					}

					if (is_referencein_A && is_2_in_B) {
						shared_ids.insert(tag2);
						if (!is_2_sharing_AB) {
							interf.second.shared_items[tag2] = std::shared_ptr<ChBody>(mbo2, [](ChBody*) {});
							//bodies_to_send.push_back(std::shared_ptr<ChBody>(mbo2, [](ChBody*) {})); // hack: use null deleter to wrap pointer
							//sent_ids.insert(tag2);
						}
					}

					if (is_referencein_B && is_1_in_A) {
						shared_ids.insert(tag1);
						if (!is_1_sharing_AB) {
							interf.second.shared_items[tag1] = std::shared_ptr<ChBody>(mbo1, [](ChBody*) {});
							//bodies_to_send.push_back(std::shared_ptr<ChBody>(mbo1, [](ChBody*) {})); // hack: use null deleter to wrap pointer
							//sent_ids.insert(tag1);
						}
					}

					if (is_referencein_B && is_2_in_A) {
						shared_ids.insert(tag2);
						if (!is_2_sharing_AB) {
							interf.second.shared_items[tag2] = std::shared_ptr<ChBody>(mbo2, [](ChBody*) {});
							//bodies_to_send.push_back(std::shared_ptr<ChBody>(mbo2, [](ChBody*) {})); // hack: use null deleter to wrap pointer
							//sent_ids.insert(tag2);
						}
					}

				}

				auto mnod1 = dynamic_cast<fea::ChNodeFEAxyzrot*>(blink->GetBody1());
				auto mnod2 = dynamic_cast<fea::ChNodeFEAxyzrot*>(blink->GetBody2());
				if (mnod1 && mnod2) {
					tag1 = mnod1->GetTag();
					tag2 = mnod2->GetTag();
					mabb.Inflate(mnod1->GetPos());
					mabb.Inflate(mnod2->GetPos());
					reference = mnod1->GetPos();

					is_referencein_A = interf.second.side_IN->IsInto(reference);
					is_referencein_B = interf.second.side_OUT->IsInto(reference);
					bool is_1_in_A = interf.second.side_IN->IsInto(mnod1->GetPos());
					bool is_1_in_B = interf.second.side_OUT->IsInto(mnod2->GetPos());
					bool is_2_in_A = interf.second.side_IN->IsInto(mnod1->GetPos());
					bool is_2_in_B = interf.second.side_OUT->IsInto(mnod2->GetPos());
					bool is_1_sharing_AB = (interf.second.shared_items.find(tag1) != interf.second.shared_items.end());
					bool is_2_sharing_AB = (interf.second.shared_items.find(tag2) != interf.second.shared_items.end());

					if (is_referencein_A && is_1_in_B) {
						shared_ids.insert(tag1);
						if (!is_1_sharing_AB) {
							interf.second.shared_nodes[tag1] = std::shared_ptr<ChNodeBase>(mnod1, [](ChNodeBase*) {});
							//nodes_to_send.push_back(std::shared_ptr<ChNodeBase>(mnod1, [](ChNodeBase*) {})); // hack: use null deleter to wrap pointer
							//sent_ids.insert(tag1);
						}
					}

					if (is_referencein_A && is_2_in_B) {
						shared_ids.insert(tag2);
						if (!is_2_sharing_AB) {
							interf.second.shared_nodes[tag2] = std::shared_ptr<ChNodeBase>(mnod2, [](ChNodeBase*) {});
							//nodes_to_send.push_back(std::shared_ptr<ChNodeBase>(mnod2, [](ChNodeBase*) {})); // hack: use null deleter to wrap pointer
							//sent_ids.insert(tag2);
						}
					}

					if (is_referencein_B && is_1_in_A) {
						shared_ids.insert(tag1);
						if (!is_1_sharing_AB) {
							interf.second.shared_nodes[tag1] = std::shared_ptr<ChNodeBase>(mnod1, [](ChNodeBase*) {});
							//nodes_to_send.push_back(std::shared_ptr<ChNodeBase>(mnod1, [](ChNodeBase*) {})); // hack: use null deleter to wrap pointer
							//sent_ids.insert(tag1);
						}
					}

					if (is_referencein_B && is_2_in_A) {
						shared_ids.insert(tag2);
						if (!is_2_sharing_AB) {
							interf.second.shared_nodes[tag2] = std::shared_ptr<ChNodeBase>(mnod2, [](ChNodeBase*) {});
							//nodes_to_send.push_back(std::shared_ptr<ChNodeBase>(mnod2, [](ChNodeBase*) {})); // hack: use null deleter to wrap pointer
							//sent_ids.insert(tag2);
						}
					}

				}

				// Serialize the link when needed. Links are never shared.
				if (is_referencein_B) {
					if (sent_ids.find(link->GetTag()) == sent_ids.end()) {
						links_to_send.push_back(link);
						sent_ids.insert(link->GetTag());
					}
				}

			}
		}
		

		// 3-OTHER PHYSICS ITEMS
		for (const auto& item : system->GetOtherPhysicsItems()) {
			ChAABB mabb = item->GetTotalAABB();
			bool is_overlapping_A = interf.second.side_IN->IsOverlap(mabb) || mabb.IsInverted(); // if aabb is empty (reversed) assume always overlap
			bool is_overlapping_B = interf.second.side_OUT->IsOverlap(mabb) || mabb.IsInverted(); // if aabb is empty (reversed) assume always overlap
			bool is_sharing_AB = (interf.second.shared_items.find(item->GetTag()) != interf.second.shared_items.end());
			
			if (is_overlapping_B && !is_overlapping_A && !is_sharing_AB && (sent_ids.find(item->GetTag()) == sent_ids.end())) {
				items_to_send.push_back(item);
				sent_ids.insert(item->GetTag());
			}
			if (is_overlapping_B && is_overlapping_A) {
				shared_ids.insert(item->GetTag());
				if (!is_sharing_AB) {
					interf.second.shared_items[item->GetTag()] = item;
					items_to_send.push_back(item);
					sent_ids.insert(item->GetTag());
				}
			} else if (is_sharing_AB) {
				interf.second.shared_items.erase(item->GetTag());
			}
		}

		

		// 4-NODES
		// ...
		
		// Serialize outbound nodes
		// ...


		// - SERIALIZE

		// Serialize outbound bodies
		serializer << CHNVP(bodies_to_send);
		// Serialize outbound links
		serializer << CHNVP(links_to_send);
		// Serialize outbound otherphysics items
		serializer << CHNVP(items_to_send);



		// N-HANDSHAKE IDS
		
		// Serialize also the list of items that this domain considered to be shared with the
		// interface neighbour. This will be sent to the neighbour, so it will know that some
		// nodes must be kept in 'shared' mode even if their bounding box is not overlapping (this is the 
		// case of nodes or bodies connected by a ChLink or a ChElement, where the link or element is in the 
		// other domain). [To do: avoid storing in shared_ids the items that are aabb-overlapping on the interface, 
		// as this can be inferred also by the neighbouring domain.]
		serializer << CHNVP(shared_ids);
		
	}


}


void  ChDomain::DoUpdateSharedReceived() {
	
	// This will be populated by all interfaces with all neighbours
	std::unordered_set<int> set_of_domainshared;

	for (auto& interf : this->interfaces) {

		std::vector<std::shared_ptr<ChBody>>		bodies_to_receive;
		std::vector<std::shared_ptr<ChPhysicsItem>>	items_to_receive;
		std::vector<std::shared_ptr<ChLinkBase>>	links_to_receive;
		std::vector<std::shared_ptr<ChNodeBase>>    nodes_to_receive;
		//std::vector<std::shared_ptr<ChElementBase>> elements_to_receive;

		// prepare the deserializer
		ChArchiveInBinary deserializer(interf.second.buffer_receiving);

		// Deserialize inbound bodies
		deserializer >> CHNVP(bodies_to_receive);
		// Serialize outbound links
		deserializer >> CHNVP(links_to_receive);
		// Serialize outbound otherphysics items
		deserializer >> CHNVP(items_to_receive);

		// 1-BODIES

		for (const auto& body : bodies_to_receive) {
			system->AddBody(body);
			system->GetCollisionSystem()->BindItem(body);

			bool is_overlapping_B = interf.second.side_OUT->IsOverlap(body->GetTotalAABB()) || interf.second.side_OUT->IsInto(body->GetPos()); // if aabb is empty (reversed) use center
			bool is_sharing_B = (interf.second.shared_items.find(body->GetTag()) != interf.second.shared_items.end());
			if (is_overlapping_B && !is_sharing_B) {
				interf.second.shared_items[body->GetTag()] = body;
			}
		}
		
		// 2-LINKS

		for (const auto& link : links_to_receive) {
			system->AddLink(link);
		}

		// 3-OTHER PHYSICS ITEMS

		for (const auto& oitem : items_to_receive) {
			system->AddOtherPhysicsItem(oitem);
			system->GetCollisionSystem()->BindItem(oitem);

			ChAABB mabb = oitem->GetTotalAABB();
			bool is_overlapping_B = interf.second.side_OUT->IsOverlap(mabb) || mabb.IsInverted(); // if aabb is empty (reversed) assume always shared
			bool is_sharing_B = (interf.second.shared_items.find(oitem->GetTag()) != interf.second.shared_items.end());
			if (is_overlapping_B && !is_sharing_B) {
				interf.second.shared_items[oitem->GetTag()] = oitem;
			}
		}

		// 3-NODES
		// ...


		// N-HANDSHAKE IDS

		// De-serialize also the list of items that the sending domain considered to be shared with the
		// receiving domain. So here we will know that some nodes must be kept in 'shared' mode even if 
		// their bounding box is not overlapping the interface (this is the case of nodes or bodies connected by a 
		// ChLink or a ChElement, where the link or element is in the other domain).

		std::unordered_set<int> shared_ids;
		deserializer >> CHNVP(shared_ids);

		for (const auto& body : system->GetBodies()) {
			if (shared_ids.find(body->GetTag()) != shared_ids.end())
				interf.second.shared_items[body->GetTag()] = body;
		}
		for (const auto& oitem : system->GetOtherPhysicsItems()) {
			if (shared_ids.find(oitem->GetTag()) != shared_ids.end())
				interf.second.shared_items[oitem->GetTag()] = oitem;
		}
		/*
		***TODO complete also for nodes
		for (const auto& node : ... ) {
			if (shared_ids.find(node->GetTag()) != shared_ids.end())
				interf.second.shared_nodes[node->GetTag()] = node;
		}
		*/


		for (auto& mb : interf.second.shared_bodies)
			set_of_domainshared.insert(mb.first);
		for (auto& mi : interf.second.shared_items)
			set_of_domainshared.insert(mi.first);
		for (auto& mn : interf.second.shared_nodes)
			set_of_domainshared.insert(mn.first);
	}

	//
	// Removal of objects that are not anymore overlapping this domain
	//

	for (const auto& body : system->GetBodies()) {
		auto mabb = body->GetTotalAABB();
		bool is_overlapping_A = this->IsOverlap(mabb) || this->IsInto(body->GetPos()); // if aabb is empty (reversed) use center
		bool is_sharing_A = (set_of_domainshared.find(body->GetTag()) == set_of_domainshared.end());
		if (!is_overlapping_A && !is_sharing_A)
		{
			system->RemoveBody(body);
			/* unneded?
			for (auto& interf : this->interfaces) {
				interf.second.shared_items.erase(body->GetTag());
			}
			*/
		}
	}

	for (const auto& link : system->GetLinks()) {
		if (auto blink = std::dynamic_pointer_cast<ChLink>(link)) {
			ChVector3d reference;
			bool is_referencein_A;
			auto mbo1 = dynamic_cast<ChBody*>(blink->GetBody1());
			auto mbo2 = dynamic_cast<ChBody*>(blink->GetBody2());
			if (mbo1 && mbo2) {
				reference = mbo2->GetPos();
			}
			auto mnod1 = dynamic_cast<fea::ChNodeFEAxyzrot*>(blink->GetBody1());
			auto mnod2 = dynamic_cast<fea::ChNodeFEAxyzrot*>(blink->GetBody2());
			if (mnod1 && mnod2) {
				reference = mnod1->GetPos();
			}

			// Delete the link if outside. Links are never shared.
			is_referencein_A = this->IsInto(reference);
			if (!is_referencein_A) {
				system->Remove(link);
			}

		}
	}




}




}  // end namespace multidomain
}  // end namespace chrono
