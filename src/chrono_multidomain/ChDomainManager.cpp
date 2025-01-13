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


///////////////////////


void ChDomainManager::PrintDebugDomainInfo(std::shared_ptr<ChDomain> domain) {

	if (domain->IsMaster() && !this->master_domain_enabled)
		return;

	std::cout << "DOMAIN ---- rank: " << domain->GetRank() << "-----------------------------\n";

	for (auto body : domain->GetSystem()->GetBodies()) {
		std::cout << "  ChBody " << body->GetTag();
		if (body->IsFixed()) 
			std::cout << " (fixed)";
		std::cout << std::endl;
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

		if (interf.second.side_OUT->IsMaster() && !this->master_domain_enabled)
			continue;

		std::cout << " interface to domain rank " << interf.second.side_OUT->GetRank() << " ...... \n";
		for (auto& item : interf.second.shared_items)
			std::cout << "  shared item tag " << item.first << std::endl;
		for (auto& node : interf.second.shared_nodes)
			std::cout << "  shared node tag " << node.first << std::endl;
	}

	//std::cout << "Wv weight vector: \n" << domain->GetSystem()->CoordWeightsWv() << "\n";

	std::cout << std::endl;
}




}  // end namespace multidomain
}  // end namespace chrono
