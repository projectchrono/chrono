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

#ifndef CHDOMAINBUILDER_H
#define CHDOMAINBUILDER_H


#include "chrono/physics/ChSystem.h"
#include "chrono_multidomain/ChApiMultiDomain.h"
#include "chrono_multidomain/ChDomainManager.h"
#include "chrono_multidomain/ChDomain.h"

namespace chrono {
namespace multidomain {


/// Base class for all domain builders, that is, helpers that setup N ChDomain objects
/// from some geometric splitting of the 3D space. Children classes must specialize this.
/// Be sure to call the BuildDomain() for all ranks, without leaving some domain rank not 
/// built.

class ChApiMultiDomain ChDomainBuilder {
  public:
    ChDomainBuilder() {};
    virtual ~ChDomainBuilder() {}

    /// Get the number of ranks needed for this domain splitting 
    virtual int GetTotRanks() = 0;
    
    /// If there is a master rank, for global coarse operations or other needs, return its rank 
    /// (if master not supported or not enabled, children classes must break via assert)
    virtual int GetMasterRank() = 0;

  private:
};

/// Helper class that setup domains for a "sliced bread" splitting of the
/// 3D space. The splitting can happen along x or y or z axis, even in non uniform spacing.
/// The first and last domain will get infinite extent in the outside direction.

class ChApiMultiDomain ChDomainBuilderSlices : ChDomainBuilder {
public:
    ChDomainBuilderSlices(
        int tot_slices,
        double mmin,
        double mmax,
        ChAxis maxis, 
        bool build_master = false);

    ChDomainBuilderSlices(
        std::vector<double> axis_cuts,
        ChAxis maxis,
        bool build_master = false);

    virtual ~ChDomainBuilderSlices() {}

    virtual int GetTotRanks() override { return GetTotSlices() + (int)m_build_master; };
    
    virtual int GetMasterRank() override { assert(m_build_master);  return GetTotSlices(); };

    virtual int GetTotSlices() { return (int)(domains_bounds.size() - 1); };

    std::shared_ptr<ChDomain> BuildDomain(
        ChSystem* msys,
        int this_rank);

    /// Build the master domain that encloses everything, and that can be populated 
    /// with all elements, nodes, bodies etc. at the beginning. It will migrate all items into
    /// the sliced domains at the first update. You need to create ChDomainBuilderSlices with build_master as true.
    /// The master domain, if used, corresponds to the last rank, ie. GetTotRanks()-1.
    std::shared_ptr<ChDomain> BuildMasterDomain(
        ChSystem* msys);

private:
    std::vector<double> domains_bounds;
    ChAxis axis = ChAxis::X;
    bool m_build_master = false;
};


/// Specialization of ChDomain: sliced space. Used by ChDomainBuilderSlices.
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
    virtual bool IsOverlap(const ChAABB& abox) const override {
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
    virtual bool IsInto(const ChVector3d& apoint) const override {
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
    virtual bool IsOverlap(const ChAABB& abox) const override {
        if ((aabb.min.x() <= abox.max.x() && abox.min.x() < aabb.max.x()) &&
            (aabb.min.y() <= abox.max.y() && abox.min.y() < aabb.max.y()) &&
            (aabb.min.z() <= abox.max.z() && abox.min.z() < aabb.max.z()))
            return true;
        else
            return false;
    }

    /// Test if some item, represented by a single point, must be included in this domain.
    virtual bool IsInto(const ChVector3d& apoint) const override {
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




}  // end namespace multidomain
}  // end namespace chrono

#endif
