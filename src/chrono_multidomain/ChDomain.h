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

#ifndef CHDOMAIN_H
#define CHDOMAIN_H

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
#include "chrono_multidomain/ChDomainManager.h"

namespace chrono {
namespace multidomain {


class ChDomainInterface; // forward decl.

/// Base class for managing a single domain, that is a container for a ChSystem
/// that will communicate with other domains to migrate physics items, nodes, elements, etc.

class ChApiMultiDomain ChDomain : public ChOverlapTest {
public:
    ChDomain(ChSystem* msystem, int mrank) {
        system = msystem;
        rank = mrank;
    }
    virtual ~ChDomain() {}

    /// Prepare serialization of with items to send to neighbours, storing in 
    /// the buffer_sending  of the domain interfaces.
    /// Remove items that are not here anymore. 
    /// This is usually followed by a set of global MPI send/receive operations,
    /// and finally a DoUpdateSharedReceived()
    virtual void DoUpdateSharedLeaving();

    /// Deserialize of items received from neighbours, fetching from  
    /// the buffer_receiving  of the domain interfaces. Creates items in ChSystem if needed.
    /// This is usually preceded by a DoUpdateSharedLeaving() and a 
    /// global MPI send/receive operations.
    virtual void DoUpdateSharedReceived(bool delete_outsiders = true);

    /// Get map of interfaces
    std::unordered_map<int, ChDomainInterface>& GetInterfaces()  { return interfaces; };

    /// Get rank of this domain
    int GetRank() {  return rank;  };

    /// Set rank of this domain
    void SetRank(int mrank) { rank = mrank; };

    /// Tells if this is a "master" domain that talks with all other domains.
    virtual bool IsMaster() const { return false; }

    /// Get system used by this domain
    ChSystem* GetSystem() { return system; }

    /// Compute a vector where each element is 1 if the dof is not 
    /// shared, or n if shared among n domains
    void ComputeSharedCoordsCounts(ChVectorDynamic<>& Nv);
    
    /// Compute the weighting vector (partition-of-unity) to split 
    /// shared vectors representing force loads and mass loads at
    /// graph vertexes (bodies, nodes) as 1/n where n is the n of shared copies
    void ComputeSharedCoordsWeights(ChVectorDynamic<>& Wv);

    
    /// Set the type of serialization. Binary is the fastest, Json or XML are memory consuming
    /// and slow, but better for debugging. NOTE! must be the same for all domains!
    /// Anyway, ChDomainManager will automatically initialize this when doing SetDomain() to
    /// make it equal to  ChDomainManager::serializer_type.
    DomainSerializerFormat serializer_type = DomainSerializerFormat::XML; 

    ChDomainManager* domain_manager = nullptr;

private:
    int rank = 0;
    ChSystem* system = nullptr;

    std::unordered_map<int, ChDomainInterface>  interfaces; // key is rank of neighbour domain

     
};


/// Specialization of ChDomain that can be used for initially populating the scene. It wraps all
/// other domains and contained items will spill into the regular domains at the first time step.
class ChApiMultiDomain ChDomainMaster : public ChDomain {
public:
    ChDomainMaster(ChSystem* msystem, int mrank) : ChDomain(msystem, mrank) {
    }
    virtual ~ChDomainMaster() {}

    /// Test if some item, with axis-aligned bounding box, must be included in this domain
    virtual bool IsOverlap(const ChAABB& abox) const override {
            return false;
    }

    /// Test if some item, represented by a single point, must be included in this domain.
    virtual bool IsInto(const ChVector3d& apoint) const override {
            return false;
    }

    virtual bool IsMaster() const override { return true; }

private:
};


////////////////////////////////////////////////////////////////////////////////////////////////


/// Class for interfacing between domains, for example when two domains share
/// a face or an edge or a corner. A domain handles multiples of this, depending
/// on the topology of the domain partitioning.
/// The side_IN_domain is the "internal" side, the side_OUT_domain is the "outer" side
/// and most often (on distributed memory architectures) is just a placeholder domain
/// that has no ChSystem and it is used only to store the geometry&rank of the domains surrounding
/// the one of the running rank.

class ChApiMultiDomain ChDomainInterface {
public:
    ChDomainInterface() {};
    ~ChDomainInterface() {}

    ChDomainInterface(const ChDomainInterface& other) {
        side_IN = other.side_IN;
        side_OUT = other.side_OUT;
        shared_items = other.shared_items;
        shared_nodes = other.shared_nodes;
        buffer_sending << other.buffer_sending.rdbuf();
        buffer_receiving << other.buffer_receiving.rdbuf();
    };
    ChDomainInterface& operator =(const ChDomainInterface& other) {
        side_IN = other.side_IN;
        side_OUT = other.side_OUT;
        shared_items = other.shared_items;
        shared_nodes = other.shared_nodes;
        buffer_sending << other.buffer_sending.rdbuf();
        buffer_receiving << other.buffer_receiving.rdbuf();
        return *this;
    }

    ChDomain* side_IN = nullptr;
    std::shared_ptr<ChDomain> side_OUT;

    // Maps of shared graph nodes:
    std::map<int, std::shared_ptr<ChPhysicsItem>> shared_items;
    std::map<int, std::shared_ptr<ChNodeBase>>    shared_nodes;

    // Maps of shared graph edges: 
    // -no edge sharing by design-

    // This is used to cast inter-domain messages
    std::stringstream buffer_sending;
    std::stringstream buffer_receiving;

    // for the system descriptor:
    std::vector<ChVariables*> shared_vars;

private:
};





}  // end namespace multidomain
}  // end namespace chrono

#endif
