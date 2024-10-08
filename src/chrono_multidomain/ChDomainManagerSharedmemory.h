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

#ifndef CHDOMAINMANAGERSHAREDMEMORY_H
#define CHDOMAINMANAGERSHAREDMEMORY_H


#include "chrono_multidomain/ChDomainManager.h"


namespace chrono {
namespace multidomain {

// Forward decl
class ChDomain;


/// A simple domain manager for multithreaded shared parallelism based on OpenMP.
/// The simpliest form of multidomain communication: domains are residing in the 
/// same memory, so this single stucture handles pointers to all domains. This also
/// means that this must be instanced once and shared among different threads on a single cpu.

class ChApiMultiDomain ChDomainManagerSharedmemory : public ChDomainManager {
public:
    ChDomainManagerSharedmemory() {};
    virtual ~ChDomainManagerSharedmemory() {};

    /// As soon as you create a domain via a ChDomainBuilder, you must add it via this method.
    /// Also, this method also takes care of replacing the system descriptor of the system
    /// so that it is of ChSystemDescriptorMultidomain, which is needed when using multidomain
    /// solvers. 
    void AddDomain(std::shared_ptr<ChDomain> mdomain);

    // PER-PROCESS FUNCTIONS
    //
    // These are designed to be called in parallel by multiple domains, in OpenMP 

    /// For a given domain, send all data in buffer_sending to the 
    /// buffer_receiving of the neighbour domains, and at the same time
    /// it receives into  buffer_receiving the data in buffer_sending of the neighbours.
    /// NOTE: This function is expected to be called in OpenMP parallelism by all domains, by code
    /// running inside a #pragma omp parallel where each thread handles one domain.
    /// NOTE: it contains two OpenMP synchronization barriers.
    virtual bool DoDomainSendReceive(int mrank);

    /// For a given domain, 
    /// - prepare outgoing serialization calling DoUpdateSharedLeaving()
    /// - send/receive serialization calling DoDomainSendReceive()
    /// - deserialize incoming items and delete outgoing, via DoUpdateSharedLeaving()
    /// NOTE: This function is expected to be called in parallel by all domains.
    /// NOTE: it contains two OpenMP synchronization barriers.
    virtual bool DoDomainPartitionUpdate(int mrank);

    // FOR MATH 

    /// Reduction (combines values from all processes and distributes the result back 
    /// to all processes)
    /// NOTE: This function is expected to be called in parallel by all domains.
    /// NOTE: it contains an OpenMP synchronization barrier.
    virtual int ReduceAll(int mrank, double send, double& received_result, eCh_domainsReduceOperation operation = eCh_domainsReduceOperation::sum);

    // GLOBAL FUNCTIONS
    // 
    // These are utility functions to invoke functions at once on all domains
    // of this domain manager, especially in a while{...} simulation loop, to avoid
    // using the #pragma omp parallel.. from the user side.
    // NOTE: these contain the OpenMP parallelization

    /// For all domains, call Initialize() for the contained ChSystem(), updating
    /// all the AABBs of the collision models, then it calls the first setup of
    /// partitioning using DoAllDomainPartitionUpdate().
    /// This function might be called before starting the simulation loop.
    virtual bool DoAllDomainInitialize();

    /// For all domains, call DoDomainPartitionUpdate() using a OpenMP parallelization.
    /// This function might be called before each simulation time step (or periodically)
    /// to migrate bodies, links, nodes, elements between neighbouring nodes.
    virtual bool DoAllDomainPartitionUpdate();

    /// For all domains, call DoStepDynamics() using a OpenMP parallelization.
    /// This function must be called at each simulation time step.
    virtual bool DoAllStepDynamics(double timestep);


    std::unordered_map<int, std::shared_ptr<ChDomain>> domains;

private:
    // buffers for reduce operations to mimic MPI functions
    std::vector<double> domain_sends;
    double domains_reduced;
};




}  // end namespace multidomain
}  // end namespace chrono

#endif
