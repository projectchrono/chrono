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

ChDomainManager::ChDomainManager() {
}

void ChDomain::DoUpdateShared() {

	// Select objects to sent to surrounding domains 
	for (auto& interf : this->interfaces) {
		for (const auto& item : system->GetBodies()) {
			bool is_overlapping_B = interf.B_side->IsOverlap(item->GetTotalAABB());
			bool is_sharing_B = (interf.shared_items.find(item->GetTag()) != interf.shared_items.end());
			if (is_overlapping_B && !is_sharing_B) {
				interf.shared_items[item->GetTag()] = item;
				interf.items_to_send.push_back(item);
			}
			if (!is_overlapping_B && is_sharing_B) {
				interf.shared_items.erase(item->GetTag());
			}
		}
	}

	// Removal of objects that are not anymore overlapping this domain
	for (const auto& item : system->GetBodies()) {
		if (!this->IsOverlap(item->GetTotalAABB()))
		{
			system->RemoveBody(item);
			for (auto& interf : this->interfaces) {
				interf.shared_items.erase(item->GetTag());
			}
		}
	}

}


}  // end namespace multidomain
}  // end namespace chrono
