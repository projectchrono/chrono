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

namespace chrono {
namespace multidomain {

// Forward decl
class ChDomain;

/// Base class of mechanisms that allow inter-domain communication. 

class ChApiMultiDomain ChDomainManager {
public:
    ChDomainManager() {};
    virtual ~ChDomainManager() {};

    virtual bool DoUpdateSharedLeaving() = 0;
    virtual bool DoDomainsSendReceive() = 0;
    virtual bool DoUpdateSharedReceived() = 0;
};

/// The simpliest form of multidomain communication: domains are residing in the 
/// same memory, so this single stucture handles pointers to all domains. This also
/// mean that this must be instanced once and shared among different threads on a single cpu.

class ChApiMultiDomain ChDomainManagerSharedmemory : public ChDomainManager {
public:
    ChDomainManagerSharedmemory() {};
    virtual ~ChDomainManagerSharedmemory() {};

    virtual bool DoUpdateSharedLeaving();
    virtual bool DoDomainsSendReceive();
    virtual bool DoUpdateSharedReceived();

    /// As soon as you create a domain via a ChDomainBuilder, you must add it here:
    void AddDomain(std::shared_ptr<ChDomain> mdomain);

    std::unordered_map<int, std::shared_ptr<ChDomain>> domains;
};



/// Class of helpers to manage domain decomposition and inter-domain data serialization/deserialization.

class ChApiMultiDomain ChDomainBuilder {
  public:
    ChDomainBuilder() {};
    virtual ~ChDomainBuilder() {}

    virtual int GetTotRanks() = 0;

  private:
};

class ChApiMultiDomain ChDomainBuilderSlices {
public:
    ChDomainBuilderSlices(
        int tot_ranks,
        double mmin,
        double mmax,
        ChAxis maxis);

    ChDomainBuilderSlices(
        std::vector<double> axis_cuts,
        ChAxis maxis);
    virtual ~ChDomainBuilderSlices() {}

    virtual int GetTotRanks() { return domains_bounds.size() - 1; };

    std::shared_ptr<ChDomain> BuildDomain(
        ChSystem* msys,
        int this_rank);

private:
    std::vector<double> domains_bounds;
    ChAxis axis = ChAxis::X;
};


////////////////////////////////////////////////////////////////////////////////////////////////

class ChDomainInterface; // forward decl.

/// Base class for managing a single domain, that is a container for a ChSystem
/// that will communicate with other domains to migrate physics items, nodes, elements, etc.

class ChApiMultiDomain ChDomain {
public:
    ChDomain(ChSystem* msystem, int mrank) {
        system = msystem;
        rank = mrank;
    }
    virtual ~ChDomain() {}

    /// Test if some item, with axis-aligned bounding box, must be included in this domain.
    /// Children classes must implement this, depending on the shape of the domain.
    virtual bool IsOverlap(ChAABB abox) = 0;

    /// Test if some item, represented by a single point, must be included in this domain.
    virtual bool IsInto(ChVector3d apoint) = 0;

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
    virtual void DoUpdateSharedReceived();


    /// Get map of interfaces
    std::unordered_map<int, ChDomainInterface>& GetInterfaces()  { return interfaces; };

    /// Get rank of this domain
    int GetRank() {  return rank;  };

    /// Set rank of this domain
    void SetRank(int mrank) { rank = mrank; };

private:
    int rank = 0;
    ChSystem* system = nullptr;

    std::unordered_map<int, ChDomainInterface>  interfaces; // key is rank of neighbour domain
};




/// Specialization of ChDomain: sliced space.
/// Domain with infinite extent on two axes but with finite interval on a third axis. 
/// Useful when slicing horizontally something like a river, or vertically something like
/// a tall stack.
class ChApiMultiDomain ChDomainSlice : public ChDomain {
 public:
    ChDomainSlice(ChSystem* msystem, int mrank, double mmin, double mmax, ChAxis maxis) : ChDomain(msystem, mrank) {
        min = mmin;
        max = mmax;
        axis = maxis;
    }
    virtual ~ChDomainSlice() {}

    /// Test if some item, with axis-aligned bounding box, must be included in this domain
    virtual bool IsOverlap(ChAABB abox) override {
        switch (axis) {
        case ChAxis::X:
            if (min <= abox.max.x() && abox.min.x() < max)
                return true;
            else 
                return false;
        case ChAxis::Y:
            if (min <= abox.max.y() && abox.min.y() < max)
                return true;
            else
                return false;
        case ChAxis::Z:
            if (min <= abox.max.z() && abox.min.z() < max)
                return true;
            else
                return false;
        default: 
            return false;
        }
    }

    /// Test if some item, represented by a single point, must be included in this domain.
    virtual bool IsInto(ChVector3d apoint) override {
        switch (axis) {
        case ChAxis::X:
            if (min <= apoint.x() && apoint.x() < max)
                return true;
            else
                return false;
        case ChAxis::Y:
            if (min <= apoint.y() && apoint.y() < max)
                return true;
            else
                return false;
        case ChAxis::Z:
            if (min <= apoint.z() && apoint.z() < max)
                return true;
            else
                return false;
        default:
            return false;
        }
    }

 private: 
     double min = 0;
     double max = 0;
     ChAxis axis = ChAxis::X; 
};

/// Specialization of ChDomain for axis-aligned grid space decomposition.
/// This is like a box. 
/// 
class ChApiMultiDomain ChDomainBox : public ChDomain {
public:
    ChDomainBox(ChSystem* msystem, int mrank, ChAABB domain_aabb) : ChDomain(msystem, mrank) {
        aabb = domain_aabb;
    }
    virtual ~ChDomainBox() {}

    /// Test if some item, with axis-aligned bounding box, must be included in this domain
    virtual bool IsOverlap(ChAABB abox) override {
        if ((aabb.min.x() <= abox.max.x() && abox.min.x() < aabb.max.x()) &&
            (aabb.min.y() <= abox.max.y() && abox.min.y() < aabb.max.y()) &&
            (aabb.min.z() <= abox.max.z() && abox.min.z() < aabb.max.z()))
            return true;
        else
            return false;
    }

    /// Test if some item, represented by a single point, must be included in this domain.
    virtual bool IsInto(ChVector3d apoint) override {
        if ((aabb.min.x() <= apoint.x() && apoint.x() < apoint.x()) &&
            (aabb.min.y() <= apoint.y() && apoint.y() < apoint.y()) &&
            (aabb.min.z() <= apoint.z() && apoint.z() < apoint.z()))
            return true;
        else
            return false;
    }

private:
    ChAABB aabb;
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
    //std::unordered_map<int, std::shared_ptr<ChBody>>        shared_bodies;
    std::unordered_map<int, std::shared_ptr<ChPhysicsItem>> shared_items;
    std::unordered_map<int, std::shared_ptr<ChNodeBase>>    shared_nodes;
    
    // Maps of shared graph edges: 
    // -no edge sharing by design-

    std::stringstream buffer_sending;
    std::stringstream buffer_receiving;

    // for the system descriptor:
    std::unordered_set<ChVariables*> shared_vars;

private:
};





}  // end namespace multidomain
}  // end namespace chrono

#endif
