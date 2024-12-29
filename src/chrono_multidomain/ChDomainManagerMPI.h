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

#ifndef CHDOMAINMANAGERMPI_H
#define CHDOMAINMANAGERMPI_H


#include "chrono_multidomain/ChDomainManager.h"
#include "chrono_multidomain/ChMpi.h"

namespace chrono {
namespace multidomain {

// Forward decl
class ChDomain;



/// A domain manager for distributed memory parallelism based on MPI.
/// This must be instantiated per each process. 

class ChApiMultiDomain ChDomainManagerMPI : public ChDomainManager {
public:
    /// Create a domain manager based on MPI. One domain manager must be built
    /// per each domain. When built, it calls MPI_Init(), hence the need for the two
    /// argc argv parameters, i.e. those used in main() of the program (but also optional).
    ChDomainManagerMPI(int* argc, char** argv[]);
    virtual ~ChDomainManagerMPI();

    /// As soon as you create a domain via a ChDomainBuilder, you must add it via this method.
    /// Also, this method also takes care of replacing the system descriptor of the system
    /// so that it is of ChSystemDescriptorMultidomain, which is needed when using multidomain
    /// solvers. 
    void SetDomain(std::shared_ptr<ChDomain> mdomain);

    // PER-PROCESS FUNCTIONS
    //
    // These are designed to be called in parallel by multiple domains, each domain
    // being managed by a corresponding process spawn by MPI. 

    /// For a given domain, call Initialize() for the contained ChSystem(), updating
    /// all the AABBs of the collision models, then it calls the first setup of
    /// partitioning using DoDomainPartitionUpdate().
    /// This function might be called before starting the simulation loop.
    virtual bool DoDomainInitialize(int mrank);

    /// For a given domain, send all data in buffer_sending to the 
    /// buffer_receiving of the neighbour domains, and at the same time
    /// it receives into  buffer_receiving the data in buffer_sending of the neighbours.
    /// NOTE: This function is expected to be called in MPI parallelism by all domains, by code
    /// where each MPI process handles one domain.
    /// NOTE: it contains MPI synchronization barriers.
    virtual bool DoDomainSendReceive(int mrank) override;

    /// For a given domain, 
    /// - prepare outgoing serialization calling DoUpdateSharedLeaving()
    /// - send/receive serialization calling DoDomainSendReceive()
    /// - deserialize incoming items and delete outgoing, via DoUpdateSharedLeaving()
    /// NOTE: This function is expected to be called in parallel by all domains.
    /// NOTE: it contains two MPI synchronization barriers.
    virtual bool DoDomainPartitionUpdate(int mrank, bool delete_outsiders = true) override;


    // FOR MATH 

    /// Reduction (combines values from all processes and distributes the result back 
    /// to all processes)
    /// NOTE: This function is expected to be called in parallel by all domains.
    /// NOTE: it contains a MPI synchronization barrier.
    virtual int ReduceAll(int mrank, double send, double& received_result, eCh_domainsReduceOperation operation = eCh_domainsReduceOperation::sum) override;


    virtual void ConsoleOutSerialized(std::string out_msg) override;

    // OTHER

    int GetMPIrank();
    int GetMPItotranks();

    std::shared_ptr<ChDomain> domain;

private:
    ChMPI mpi_engine;
};



}  // end namespace multidomain
}  // end namespace chrono

#endif
