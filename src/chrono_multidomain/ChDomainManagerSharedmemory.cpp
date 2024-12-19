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

//#include<array>
#include "chrono_multidomain/ChDomainManagerSharedmemory.h"
#include "chrono_multidomain/ChSystemDescriptorMultidomain.h"
#include "chrono_multidomain/ChSolverPSORmultidomain.h"


namespace chrono {
namespace multidomain {

using namespace fea;


bool ChDomainManagerSharedmemory::DoDomainSendReceive(int mrank) {
#pragma omp barrier
	if (this->domains.find(mrank) != this->domains.end()) {
		auto& mdomain = domains[mrank];
		for (auto& minterface : mdomain->GetInterfaces()) {
			// receive from neighbour domain to this domain:
			minterface.second.buffer_receiving << this->domains[minterface.second.side_OUT->GetRank()]->GetInterfaces()[mdomain->GetRank()].buffer_sending.rdbuf();
			// send from this domain to neighbour domain: not needed because done when the previous line will be called for the neighbour. 
		}
	}
#pragma omp barrier

	// for debugging
	if (this->verbose_variable_updates) {
		for (int i = 0; i < this->domains.size(); i++) {
#pragma omp barrier
			if (i == mrank) {
				if (this->domains.find(mrank) != this->domains.end()) {
					auto& mdomain = domains[mrank];
					for (auto& interf : mdomain->GetInterfaces()) {
						if (interf.second.side_OUT->IsMaster() && !this->master_domain_enabled)
							continue;
						std::cout << "\nBUFFERS  in domain " << mrank << "\n -> sent to domain " << interf.second.side_OUT->GetRank() << "\n";
						std::cout << interf.second.buffer_sending.str();
						std::cout << "\n <- received from domain " << interf.second.side_OUT->GetRank() << "\n";
						std::cout << interf.second.buffer_receiving.str();
						std::cout.flush();
					}
				}
			}
			std::cout.flush();
#pragma omp barrier
		}
	}

	return true;
}

bool ChDomainManagerSharedmemory::DoDomainPartitionUpdate(int mrank) {
	if (this->domains.find(mrank) != this->domains.end()) {
		auto& mdomain = domains[mrank];
		// 1 serialize outgoing items, update shared items
		mdomain->DoUpdateSharedLeaving();
		// 2 send/receive buffers 
		this->DoDomainSendReceive(mrank); //***COMM+BARRIER***
		// 3 serialize outgoing items, update shared items
		mdomain->DoUpdateSharedReceived();
	}
	return true;
}


int ChDomainManagerSharedmemory::ReduceAll(int mrank, double send, double& received_result, eCh_domainsReduceOperation operation) {
	// Inefficient, implies 2 barriers and other optimizable things, but here for tests against MPI
	if (mrank == 0) {
		this->domain_sends.resize(this->domains.size());
		
		switch (operation) {
		case eCh_domainsReduceOperation::max:
			this->domains_reduced = -1e37;
			break;
		case eCh_domainsReduceOperation::min:
			this->domains_reduced = 1e37;
			break;
		case eCh_domainsReduceOperation::sum:
			this->domains_reduced = 0;
			break;
		case eCh_domainsReduceOperation::prod:
			this->domains_reduced = 1.0;
			break;
		}
	}
#pragma omp barrier

	this->domain_sends[mrank] = send;

#pragma omp barrier

	if (mrank == 0) {
		for (int i = 0; i < this->domains.size(); ++i) {
			switch (operation) {
			case eCh_domainsReduceOperation::max:
				this->domains_reduced = std::max(this->domain_sends[i], this->domains_reduced);
				break;
			case eCh_domainsReduceOperation::min:
				this->domains_reduced = std::min(this->domain_sends[i], this->domains_reduced);
				break;
			case eCh_domainsReduceOperation::sum:
				this->domains_reduced = this->domain_sends[i] + this->domains_reduced;
				break;
			case eCh_domainsReduceOperation::prod:
				this->domains_reduced = this->domain_sends[i] * this->domains_reduced;
				break;
			}
		}
	}
#pragma omp barrier

	received_result = this->domains_reduced;

	return true;
}




bool ChDomainManagerSharedmemory::DoAllDomainInitialize() {
	std::vector<std::shared_ptr<ChDomain>> vdomains;
	for (auto& ido : this->domains)
		vdomains.push_back(ido.second);

#pragma omp parallel num_threads((int)vdomains.size())
	{
		int i = omp_get_thread_num();

		// update all AABBs (the initialize would be called automatically before DoStepDynamics(),
		// but one needs AABBs before calling DoAllDomainPartitionUpdate() the first time, i.e before DoStepDynamics())
		vdomains[i]->GetSystem()->Setup();
		vdomains[i]->GetSystem()->Update();
	}

	// Run the partitioning setup for the first run
	DoAllDomainPartitionUpdate();							//***COMM+BARRIER***
}



bool ChDomainManagerSharedmemory::DoAllDomainPartitionUpdate() {
	std::vector<std::shared_ptr<ChDomain>> vdomains;
	for (auto& ido : this->domains) 
		vdomains.push_back(ido.second);

	#pragma omp parallel num_threads((int)vdomains.size())
	{
			int i = omp_get_thread_num();
			this->DoDomainPartitionUpdate(vdomains[i]->GetRank()); //***COMM+BARRIER***
	}

	if (this->verbose_partition || this->verbose_serialization)
		for (auto& ido : this->domains) {
			if (this->verbose_partition)
				PrintDebugDomainInfo(ido.second);
			if (this->verbose_serialization) {
				if (!(ido.second->IsMaster() && !this->master_domain_enabled)) {
					std::cout << "\n\n::::::::::::: Serialization to domain " << ido.second->GetRank() << " :::::::::::\n";
					for (auto& interf : ido.second->GetInterfaces()) {
						if (interf.second.side_OUT->IsMaster() && !this->master_domain_enabled)
							continue;
						std::cout << "\n\n::::::::::::: ....from interface " << interf.second.side_OUT->GetRank() << " ........\n";
						std::cout << interf.second.buffer_receiving.str();
						std::cout << "\n";
					}
				}
			}
		}

}

bool ChDomainManagerSharedmemory::DoAllStepDynamics(double timestep) {
	std::vector<std::shared_ptr<ChDomain>> vdomains;
	for (auto& ido : this->domains) 
		vdomains.push_back(ido.second);

	#pragma omp parallel num_threads((int)vdomains.size())
	{
		int i = omp_get_thread_num();
		vdomains[i]->GetSystem()->DoStepDynamics(timestep); //***COMM+BARRIER*** cause maybe DoDomainSendReceive in solver
	}
}



void ChDomainManagerSharedmemory::AddDomain(std::shared_ptr<ChDomain> mdomain) {
	domains[mdomain->GetRank()] = mdomain;
	mdomain->domain_manager = this;

	/// Process collision pairs found by the narrow-phase collision step.
	/// Filter out the contacts that are not inside the domain (it can happen
	/// that two bodies are partially overlapping with this domain, ex. leaning out
	/// to the right side: the collision engine of this domain would create contacts also 
	/// between the geometry features that are outside, and at the same time the domain 
	/// on the right would generate them as well, ending with duplicated contacts on a global level.
	/// This filtering discards contacts whose main reference is not exactly contained in this 
	/// domain. A NarrowphaseCallback is used for such filtering.
	class ContactDomainFilter : public ChCollisionSystem::NarrowphaseCallback {
	public:
		virtual ~ContactDomainFilter() {}

		virtual bool OnNarrowphase(ChCollisionInfo& contactinfo) {
			// Trick: in the neighbour domain, vpA and vpB might be swapped, so
			// here make the reference unique 
			ChVector3d reference;
			if (contactinfo.vpA.x() < contactinfo.vpB.x())
				reference = contactinfo.vpA.x();
			else
				reference = contactinfo.vpB.x();

			// test if reference is inside domain
			if (mdomain->IsInto(contactinfo.vpA))
				return true;
			else
				return false;
		}
		ChDomain* mdomain;
	};
	auto mfilter = chrono_types::make_shared<ContactDomainFilter>();
	mfilter->mdomain = mdomain.get();
	mdomain->GetSystem()->GetCollisionSystem()->RegisterNarrowphaseCallback(mfilter);

	// Always change the system descriptor is of multidomain type.
	auto multidomain_descriptor = chrono_types::make_shared<ChSystemDescriptorMultidomain>(mdomain, this);
	mdomain->GetSystem()->SetSystemDescriptor(multidomain_descriptor);

	// By default, change the default solver to be PSOR for multidomain:
	auto multidomain_solver_PSOR = chrono_types::make_shared<ChSolverPSORmultidomain>();
	mdomain->GetSystem()->SetSolver(multidomain_solver_PSOR);

	// By default, skip adding forces F and M*v for nodes that are not "master", i.e. shared but not inside domain:
	mdomain->GetSystem()->EnableResidualFilteringByDomain(true, mdomain.get());

	mdomain->serializer_type = this->serializer_type;
}



void ChDomainManagerSharedmemory::ConsoleOutSerialized(std::string out_msg) {
	for (int i = 0; i < this->domains.size(); i++) {
#pragma omp barrier
		if (i == omp_get_thread_num()) {
			std::cout << out_msg;
			std::cout.flush();
		}
		std::cout.flush();
#pragma omp barrier
	}
}



}  // end namespace multidomain
}  // end namespace chrono
