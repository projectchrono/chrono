// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHPROXIMITYCONTAINERMESHLESS_H
#define CHPROXIMITYCONTAINERMESHLESS_H

#include <list>

#include "chrono/physics/ChProximityContainer.h"

namespace chrono {

/// @addtogroup chrono_fea
/// @{

/// Class for a proximity pair information in a meshless deformable continumm,
/// made with a cluster of particles - that is, an 'edge' topological connectivity in
/// in a meshless FEA approach, similar to the Smoothed Particle Hydrodynamics.
class ChApi ChProximityMeshless {
  public:
    ChProximityMeshless(collision::ChCollisionModel* mmodA,  ///< model A
                        collision::ChCollisionModel* mmodB   ///< model B
                        ) {
        Reset(mmodA, mmodB);
    }

    virtual ~ChProximityMeshless() {}

    /// Initialize again this constraint.
    virtual void Reset(collision::ChCollisionModel* mmodA,  ///< model A
                       collision::ChCollisionModel* mmodB   ///< model B
                       ) {
        assert(mmodA);
        assert(mmodB);

        this->modA = mmodA;
        this->modB = mmodB;
    }

    /// Get the collision model A, with point P1
    virtual collision::ChCollisionModel* GetModelA() { return this->modA; }
    /// Get the collision model B, with point P2
    virtual collision::ChCollisionModel* GetModelB() { return this->modB; }

  private:
    collision::ChCollisionModel* modA;  ///< model A
    collision::ChCollisionModel* modB;  ///< model B
};

/// Class for container of many proximity pairs for a meshless
/// deformable continuum (necessary for inter-particle material forces),
/// as CPU typical linked list of ChProximityMeshless objects.
/// Such an item must be addd to the physical system if you added
/// an object of class ChMatterMeshless.

class ChApi ChProximityContainerMeshless : public ChProximityContainer {

  protected:
    std::list<ChProximityMeshless*> proximitylist;
    std::list<ChProximityMeshless*>::iterator lastproximity;
    int n_added;

  public:
    ChProximityContainerMeshless();
    ChProximityContainerMeshless(const ChProximityContainerMeshless& other);
    virtual ~ChProximityContainerMeshless();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChProximityContainerMeshless* Clone() const override { return new ChProximityContainerMeshless(*this); }

    /// Tell the number of added contacts
    virtual int GetNproximities() const override { return n_added; }

    /// Remove (delete) all contained contact data.
    virtual void RemoveAllProximities() override;

    /// The collision system will call BeginAddProximities() before adding
    /// all pairs (for example with AddProximity() or similar). Instead of
    /// simply deleting all list of the previous pairs, this optimized implementation
    /// rewinds the link iterator to begin and tries to reuse previous pairs objects
    /// until possible, to avoid too much allocation/deallocation.
    virtual void BeginAddProximities() override;

    /// Add a proximity SPH data between two collision models, if possible.
    virtual void AddProximity(collision::ChCollisionModel* modA,  ///< get contact model 1
                              collision::ChCollisionModel* modB   ///< get contact model 2
                              ) override;

    /// The collision system will call BeginAddContact() after adding
    /// all contacts (for example with AddContact() or similar). This optimized version
    /// purges the end of the list of contacts that were not reused (if any).
    virtual void EndAddProximities() override;

    /// Scans all the proximity pairs and, for each pair, executes the OnReportProximity()
    /// function of the provided callback object.
    virtual void ReportAllProximities(ReportProximityCallback* mcallback) override;

    // Perform some SPH per-edge initializations and accumulations of values
    // into the connected pairs of particles (summation into particle's  J, Amoment, m_v, UserForce -viscous only- )
    // Will be called by the ChMatterMeshless item.
    void AccumulateStep1();

    // Perform some SPH per-edge transfer of forces, given stress tensors in A B nodes
    // Will be called by the ChMatterMeshless item.
    void AccumulateStep2();

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

/// @} chrono_fea

}  // end namespace chrono

#endif
