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

#ifndef CHDOMAINMANAGER_H
#define CHDOMAINMANAGER_H

#include <unordered_set>
#include <unordered_map>
#include <sstream>

#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/geometry/ChGeometry.h"
#include "chrono/solver/ChVariables.h"
#include "chrono_multidomain/ChApiMultiDomain.h"
#include "chrono_multidomain/ChMpi.h"

namespace chrono {
namespace multidomain {

// Forward decl
class ChDomain;

/// Formats for serialization buffers, when migrating objects 
/// through domain boundaries.
enum class DomainSerializerFormat {
    BINARY = 0,
    XML,
    JSON
};

/// Base class of mechanisms that allow inter-domain communication. 
/// This can be inherited for different schemes of inter-communication by
/// implementing the DoDomainSendReceive and DoDomainPartitionUpdate
/// functions. for example we have the MPI distributed-memory flavour, or the
/// OpenMP shared memory version, etc.

class ChApiMultiDomain ChDomainManager {
public:
    ChDomainManager() {};
    virtual ~ChDomainManager() {};

    // PER-PROCESS FUNCTIONS
    //
    // These are assumed to be called in parallel by multiple domains.

    /// For a given domain, send all data in buffer_sending to the 
    /// buffer_receiving of the neighbour domains, and at the same time
    /// it receives into  buffer_receiving the data in buffer_sending of the neighbours.
    /// This is a low level communication primitive that can be used by solvers,
    /// partition update serialization, etc.
    /// NOTE: This function is expected to be called in parallel by all domains.
    /// NOTE: Depending on the implementation (MPI, OpenMP, etc.) it can contain some barrier.
    virtual bool DoDomainSendReceive(int mrank) = 0;

    /// For a given domain, 
    /// - prepare outgoing serialization calling DoUpdateSharedLeaving
    /// - send/receive serialization calling DoDomainSendReceive
    /// - deserialize incoming items and delete outgoing, via DoUpdateSharedLeaving
    /// NOTE: This function is expected to be called in parallel by all domains.
    /// NOTE: Depending on the implementation (MPI, OpenMP, etc.) it can contain some barrier.
    virtual bool DoDomainPartitionUpdate(int mrank, bool delete_outsiders = true) = 0;


    /// For serializing-deserializing objects that cross domain boundaries, domains
    /// will convert transient objects into buffers to be exchanged. The serialization in BINARY 
    /// format is the fastest, whereas XML or JSON are slow and large but useful for debugging.
    DomainSerializerFormat serializer_type = DomainSerializerFormat::BINARY;


    // FOR MATH 
    
    enum class eCh_domainsReduceOperation {
        max = 0,
        min,
        sum,
        prod
    };
    /// Reduction (combines values from all processes and distributes the result back 
    /// to all processes)
    /// NOTE: This function is expected to be called in parallel by all domains.
    /// NOTE: Depending on the implementation (MPI, OpenMP, etc.) it can contain some barrier.
    virtual int ReduceAll(int mrank, double send, double& received_result, eCh_domainsReduceOperation operation = eCh_domainsReduceOperation::sum) = 0;

    /// If false, the master domain (if any) does not partecipate in sending/receiving buffers 
    /// for serialization and for distributed vector algebra, making it 'deactivated'.
    bool master_domain_enabled = true;

    // FOR DEBUGGING 

    bool verbose_partition = false;
    bool verbose_serialization = false;
    bool verbose_variable_updates = false;
    bool verbose_state_matching = false;

    void PrintDebugDomainInfo(std::shared_ptr<ChDomain> domain);

    virtual void ConsoleOutSerialized(std::string out_msg) {};
};





}  // end namespace multidomain
}  // end namespace chrono

#endif
