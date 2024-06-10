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

#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/geometry/ChGeometry.h"
#include "chrono_multidomain/ChApiMultiDomain.h"

namespace chrono {
namespace multidomain {


/// Class for managing domain decomposition and inter-domain data serialization/deserialization.
/// This takes care of transient-persistent-transient converison of chrono models, so that 
/// bodies, links, etc. can migrate through domain boundaries.

class ChApiMultiDomain ChDomainManager {
  public:
    ChDomainManager();
    ~ChDomainManager() {}

  private:
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
    ~ChDomain() {}

    // Test if some item, with axis-aligned bounding box, must be included in this domain.
    // Children classes must implement this, depending on the shape of the domain.
    virtual bool IsOverlap(ChAABB abox) = 0;

    // Test if some item, represented by a single point, must be included in this domain.
    virtual bool IsInto(ChVector3d apoint) = 0;


    virtual void DoUpdateShared();

private:
    int rank = 0;
    ChSystem* system = nullptr;

    std::vector<ChDomainInterface> interfaces;
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
    ~ChDomainSlice() {}

    // Test if some item, with axis-aligned bounding box, must be included in this domain
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

    // Test if some item, represented by a single point, must be included in this domain.
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
    ~ChDomainBox() {}

    // Test if some item, with axis-aligned bounding box, must be included in this domain
    virtual bool IsOverlap(ChAABB abox) override {
        if ((aabb.min.x() <= abox.max.x() && abox.min.x() < aabb.max.x()) &&
            (aabb.min.y() <= abox.max.y() && abox.min.y() < aabb.max.y()) &&
            (aabb.min.z() <= abox.max.z() && abox.min.z() < aabb.max.z()))
            return true;
        else
            return false;
    }

    // Test if some item, represented by a single point, must be included in this domain.
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


// Class for interfacing between domains, for example when two domains share
// a face or an edge or a corner. A domain handles multiples of this, depending
// on the topology of the domain partitioning.
// The A_side_domain is the "internal" side, the B_side_domain is the "outer" side
// and most often (on distributed memory architectures) is just a placeholder domain
// that has no ChSystem and it is used only to store the geometry&rank of the domains surrounding
// the one of the running rank.

class ChApiMultiDomain ChDomainInterface {
public:
    ChDomainInterface(ChDomain* A_side_domain, ChDomain* B_side_domain);
    ~ChDomainInterface() {}

    ChDomain* A_side = nullptr;
    ChDomain* B_side = nullptr;

    std::unordered_map<int, std::shared_ptr<ChPhysicsItem>> shared_items;
    std::unordered_map<int, std::shared_ptr<ChNodeBase>>    shared_nodes;
    std::vector<std::shared_ptr<ChPhysicsItem>> items_to_send;
    std::vector<std::shared_ptr<ChNodeBase>>    nodes_to_send;
private:
};





}  // end namespace multidomain
}  // end namespace chrono

#endif
