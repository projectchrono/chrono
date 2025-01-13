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
#include "chrono_multidomain/ChDomainManagerMPI.h"

#include "chrono_multidomain/ChSystemDescriptorMultidomain.h"
#include "chrono_multidomain/ChSolverPSORmultidomain.h"


namespace chrono {
namespace multidomain {


ChDomainManagerMPI::ChDomainManagerMPI(int* argc, char** argv[]) {
	this->mpi_engine.Init(argc,argv);
}

ChDomainManagerMPI::~ChDomainManagerMPI() {
	this->mpi_engine.Finalize();
}

void ChDomainManagerMPI::SetDomain(std::shared_ptr<ChDomain> mdomain) {
	domain = mdomain;
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
	// ***TODO** remove, EnableCoordWeightsWv fixes the same issue but in cleaner way
	//  mdomain->GetSystem()->EnableResidualFilteringByDomain(true, mdomain.get());
	
	// By default, scale forces F, M*v, lumped Md and masses for shared nodes
	mdomain->GetSystem()->EnableCoordWeightsWv(true);

	mdomain->serializer_type = this->serializer_type;

	// Set the tag ID of all ChSystem objects to 1, i.e assume shared ID of system as shared across all domains
	mdomain->GetSystem()->SetTag(1);
	// Set the tag ID of all ChAssembly objects to 2, i.e assume shared ID of assembly containers as shared across all domains
	const_cast<ChAssembly&>(mdomain->GetSystem()->GetAssembly()).SetTag(2);
}


bool ChDomainManagerMPI::DoDomainSendReceive(int mrank) {
	assert(mrank == domain->GetRank());

	if (!(domain->IsMaster() && !this->master_domain_enabled))
		for (auto& minterface : domain->GetInterfaces()) {
			int other_rank = minterface.second.side_OUT->GetRank();

			if (minterface.second.side_OUT->IsMaster() && !this->master_domain_enabled)
				continue;

			// A)
			// non-blocking MPI send from this domain to neighbour domain:
			// equivalent:   MPI_Isend
			ChMPIrequest mrequest1;
			std::string send_string;
			send_string = minterface.second.buffer_sending.rdbuf()->str();
			mpi_engine.SendString_nonblocking(other_rank, send_string, ChMPI::eCh_mpiCommMode::MPI_STANDARD, &mrequest1);

			// B)
			// blocking MPI receive from neighbour domain to this domain:
			// equivalent:  MPI_Probe+MPI_Recv   WILL BLOCK! at each interface. No need for later MPI_WaitAll
			ChMPIstatus mstatus2;
			std::string receive_string;
			mpi_engine.ReceiveString_blocking(other_rank, receive_string, &mstatus2);
			minterface.second.buffer_receiving << receive_string;
		}

	if (this->verbose_variable_updates)
		for (int i = 0; i < GetMPItotranks(); i++) {
			this->mpi_engine.Barrier();							// WILL BLOCK ALL!
			if (i == GetMPIrank()) {
				if (!(domain->IsMaster() && !this->master_domain_enabled))
					for (auto& interf : domain->GetInterfaces())
					{
						if (interf.second.side_OUT->IsMaster() && !this->master_domain_enabled)
							continue;
						std::cout << "\nBUFFERS  in domain " << this->domain->GetRank() << "\n -> sent to domain " << interf.second.side_OUT->GetRank() << "\n"; 
						std::cout << interf.second.buffer_sending.str(); 
						std::cout << "\n <- received from domain " << interf.second.side_OUT->GetRank() << "\n";
						std::cout << interf.second.buffer_receiving.str();
						std::cout.flush();
					}
			}
			std::cout.flush();
			this->mpi_engine.Barrier();							// WILL BLOCK ALL!
		}


	return true;
}



bool ChDomainManagerMPI::DoDomainInitialize(int mrank) {
	assert(mrank == domain->GetRank());

	// update all AABBs (the initialize would be called automatically before DoStepDynamics(),
	// but one needs AABBs before calling DoDomainPartitionUpdate() the first time, i.e before DoStepDynamics())
	domain->GetSystem()->Setup();
	domain->GetSystem()->Update();

	// Run the partitioning setup for the first run.
	// Note that delete_outsiders = fase, so we do not run the cleanup stage because 
	// the master could have injected finite elements that are spilling over the interfaces
	// but the two domains on the interface still do not know that they have to share the outside nodes, 
	// they will get to know this at the next DoDomainPartitionUpdate(), and if we run the delete outsiders 
	// stage right now, those outside nodes would be incorrectly deleted prematurely.
	DoDomainPartitionUpdate(mrank, false);					//***COMM+BARRIER***
	
	if (domain->IsMaster())
		domain->GetSystem()->Clear();

	// This can be needed for updating visual assets and other things
	domain->GetSystem()->Setup();
	domain->GetSystem()->ForceUpdate();
	domain->GetSystem()->Update();

	return true;
}

bool ChDomainManagerMPI::DoDomainPartitionUpdate(int mrank, bool delete_outsiders) {
	assert(mrank == domain->GetRank());

	// 1 serialize outgoing items, update shared items
	domain->DoUpdateSharedLeaving();

	if (this->verbose_serialization)
		for (int i = 0; i < GetMPItotranks(); i++) {
			this->mpi_engine.Barrier(); // trick to force sequential std::cout if MPI on same shell -  BLOCKS ALL!
			if (i == GetMPIrank()) {
				if (!(domain->IsMaster() && !this->master_domain_enabled)) {
					std::cout << "\n\n::::::::::::: Serialization from domain " << domain->GetRank() << " :::::::::::\n";
					for (auto& interf : domain->GetInterfaces()) {
						if (interf.second.side_OUT->IsMaster() && !this->master_domain_enabled)
							continue;
						std::cout << "\n\n::::::::::::: ....to domain " << interf.second.side_OUT->GetRank() << " ........\n";
						std::cout << interf.second.buffer_sending.str();
						std::cout << "\n";
					}
				}
				std::cout.flush();
			}
			std::cout.flush();
			this->mpi_engine.Barrier();															    // -BLOCKS ALL!
		}

	// 2 send/receive buffers 
	this->DoDomainSendReceive(mrank); //***COMM+BARRIER***

	// 3 deserialize incoming items, update shared items
	domain->DoUpdateSharedReceived(delete_outsiders);

	if (this->verbose_partition)
		for (int i = 0; i < GetMPItotranks(); i++) {
			this->mpi_engine.Barrier(); // trick to force sequential std::cout if MPI on same shell -  BLOCKS ALL!
			if (i == GetMPIrank()) {
				PrintDebugDomainInfo(domain);
				std::cout.flush();
			}
			std::cout.flush();
			this->mpi_engine.Barrier();															    // -BLOCKS ALL!
		}

	return true;
}

int ChDomainManagerMPI::ReduceAll(int mrank, double send, double& received_result, eCh_domainsReduceOperation operation) {
	ChMPI::eCh_mpiReduceOperation mpi_operation = ChMPI::eCh_mpiReduceOperation::MPI_sum;
	switch (operation) {
	case eCh_domainsReduceOperation::max:
		mpi_operation = ChMPI::eCh_mpiReduceOperation::MPI_max; break;
	case eCh_domainsReduceOperation::min:
		mpi_operation = ChMPI::eCh_mpiReduceOperation::MPI_min; break;
	case eCh_domainsReduceOperation::sum:
		mpi_operation = ChMPI::eCh_mpiReduceOperation::MPI_sum; break;
	case eCh_domainsReduceOperation::prod:
		mpi_operation = ChMPI::eCh_mpiReduceOperation::MPI_prod; break;
	}
	return this->mpi_engine.ReduceAll(send, received_result, mpi_operation);
}

int ChDomainManagerMPI::GetMPIrank() {
	return mpi_engine.CommRank();
}
int ChDomainManagerMPI::GetMPItotranks() {
	return mpi_engine.CommSize();
}



void ChDomainManagerMPI::ConsoleOutSerialized(std::string out_msg) {
	for (int i = 0; i < GetMPItotranks(); i++) {
		this->mpi_engine.Barrier(); // trick to force sequential std::cout if MPI on same shell
		if (i == GetMPIrank()) {
			std::cout << out_msg;
			std::cout.flush();
		}
		std::cout.flush();
		this->mpi_engine.Barrier();
	}
}


}  // end namespace multidomain
}  // end namespace chrono
