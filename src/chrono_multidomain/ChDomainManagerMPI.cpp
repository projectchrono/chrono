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
	mdomain->GetSystem()->EnableResidualFilteringByDomain(true, mdomain.get());

	mdomain->serializer_type = this->serializer_type;
}


bool ChDomainManagerMPI::DoDomainSendReceive(int mrank) {
	assert(mrank == domain->GetRank());

	for (auto& minterface : domain->GetInterfaces()) {
		int other_rank = minterface.second.side_OUT->GetRank();

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
			this->mpi_engine.Barrier();
			if (i == GetMPIrank()) {
				for (auto& interf : domain->GetInterfaces())
				{
					std::cout << "\nBUFFERS  in domain " << this->domain->GetRank() << "\n -> sent to domain " << interf.second.side_OUT->GetRank() << "\n"; 
					std::cout << interf.second.buffer_sending.str(); 
					std::cout << "\n <- received from domain " << interf.second.side_OUT->GetRank() << "\n";
					std::cout << interf.second.buffer_receiving.str();
					std::cout.flush();
				}
			}
			std::cout.flush();
			this->mpi_engine.Barrier();
		}


	return true;
}



bool ChDomainManagerMPI::DoDomainInitialize(int mrank) {
	assert(mrank == domain->GetRank());

	// update all AABBs (the initialize would be called automatically before DoStepDynamics(),
	// but one needs AABBs before calling DoDomainPartitionUpdate() the first time, i.e before DoStepDynamics())
	domain->GetSystem()->Setup();
	domain->GetSystem()->Update();

	// Run the partitioning setup for the first run
	DoDomainPartitionUpdate(mrank);					//***COMM+BARRIER***

	return true;
}

bool ChDomainManagerMPI::DoDomainPartitionUpdate(int mrank) {
	assert(mrank == domain->GetRank());

	// 1 serialize outgoing items, update shared items
	domain->DoUpdateSharedLeaving();
	// 2 send/receive buffers 
	this->DoDomainSendReceive(mrank); //***COMM+BARRIER***
	// 3 deserialize incoming items, update shared items
	domain->DoUpdateSharedReceived();

	if (this->verbose_partition || this->verbose_serialization)
		for (int i = 0; i < GetMPItotranks(); i++) {
			this->mpi_engine.Barrier(); // trick to force sequential std::cout if MPI on same shell
			if (i == GetMPIrank()) {
				if (this->verbose_partition)
					PrintDebugDomainInfo(domain);
				if (this->verbose_serialization) {
					std::cout << "\n\n::::::::::::: Serialization to domain " << domain->GetRank() << " :::::::::::\n";
					for (auto& interf : domain->GetInterfaces()) {
						std::cout << "\n\n::::::::::::: ....from interface " << interf.second.side_OUT->GetRank() << " ........\n";
						std::cout << interf.second.buffer_receiving.str();
						std::cout << "\n";
					}
				}
				std::cout.flush();
			}
			std::cout.flush();
			this->mpi_engine.Barrier();
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
