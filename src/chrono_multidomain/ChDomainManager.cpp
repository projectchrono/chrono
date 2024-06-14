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


namespace chrono {
namespace multidomain {


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
		auto low_domain = chrono_types::make_shared<ChDomainSlice>(nullptr, low_domain_rank, domains_bounds[low_domain_rank], domains_bounds[this_rank], axis);
		ChDomainInterface& low_interf = domain->GetInterfaces()[low_domain->GetRank()]; // insert domain interface in unordered map
		low_interf.side_IN = domain.get();
		low_interf.side_OUT = low_domain;
	}
	if (this_rank < domains_bounds.size() - 2) {
		int hi_domain_rank = this_rank + 1;
		auto hi_domain = chrono_types::make_shared<ChDomainSlice>(nullptr, hi_domain_rank, domains_bounds[this_rank], domains_bounds[hi_domain_rank], axis);
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

		// prepare the serializer
		interf.second.buffer_sending.clear();
		interf.second.buffer_receiving.clear();
		ChArchiveOutBinary serializer(interf.second.buffer_sending);
		serializer.CutPointers().insert(this->system); // avoid propagation of serialization to parent system

		std::vector<std::shared_ptr<ChBody>>		bodies_to_send;
		std::vector<std::shared_ptr<ChPhysicsItem>>	items_to_send;
		std::vector<std::shared_ptr<ChNodeBase>>    nodes_to_send;

		// 1-BODIES
		for (const auto& item : system->GetBodies()) {
			bool is_overlapping_B = interf.second.side_OUT->IsOverlap(item->GetTotalAABB());
			bool is_sharing_B = (interf.second.shared_items.find(item->GetTag()) != interf.second.shared_items.end());
			if (is_overlapping_B && !is_sharing_B) {
				interf.second.shared_items[item->GetTag()] = item;
				bodies_to_send.push_back(item);
			}
			if (!is_overlapping_B && is_sharing_B) {
				interf.second.shared_items.erase(item->GetTag());
			}
		}

		// Serialize outbound bodies
		serializer << CHNVP(bodies_to_send);

		// 2-NODES
		// ...
		
		// Serialize outbound nodes
		// ...
	}

	// Removal of objects that are not anymore overlapping this domain
	for (const auto& item : system->GetBodies()) {
		if (!this->IsOverlap(item->GetTotalAABB()))
		{
			system->RemoveBody(item);
			for (auto& interf : this->interfaces) {
				interf.second.shared_items.erase(item->GetTag());
			}
		}
	}

}

/*
void  ChDomain::DoItemsSend() {

	for (auto& interf : this->interfaces) {
		// Send buffer to neighbour domain 
		interf.second.side_OUT->interfaces[interf.first].buffer_receiving << interf.second.buffer_sending.rdbuf();  // *** TODO*** use MPI and rank comm
		// Receive buffer from neighbour domain
		interf.second.buffer_receiving << interf.second.side_OUT->interfaces[interf.first].buffer_sending.rdbuf();  // *** TODO*** use MPI and rank comm
	}

}
*/

void  ChDomain::DoUpdateSharedReceived() {

	// Select objects to sent to surrounding domains 
	for (auto& interf : this->interfaces) {

		std::vector<std::shared_ptr<ChBody>>		bodies_to_receive;
		std::vector<std::shared_ptr<ChPhysicsItem>>	items_to_receive;
		std::vector<std::shared_ptr<ChNodeBase>>    nodes_to_receive;

		// prepare the deserializer
		ChArchiveInBinary deserializer(interf.second.buffer_receiving);

		// 1-BODIES
		deserializer >> CHNVP(bodies_to_receive);

		for (const auto& body : bodies_to_receive) {
			system->AddBody(body);
		}
		

		// 2-NODES
		// ...


	}

}




}  // end namespace multidomain
}  // end namespace chrono
