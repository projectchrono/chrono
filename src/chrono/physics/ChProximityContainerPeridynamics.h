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
// Authors: Alessandro Tasora 
// =============================================================================

#ifndef CHPROXIMITYCONTAINERPERIDYNAMICS_H
#define CHPROXIMITYCONTAINERPERIDYNAMICS_H

#include <list>

#include "chrono/physics/ChProximityContainer.h"

namespace chrono {

/// @addtogroup chrono_fea
/// @{

// Forward declaration
class fea::ChNodePeridynamics;
class fea::ChMatterPeriBase;


class ChApi ChBoundPeridynamics {
  public:
    ChBoundPeridynamics(fea::ChNodePeridynamics* mmodA,  ///< model A
                        fea::ChNodePeridynamics* mmodB   ///< model B
                        ) {
        Reset(mmodA, mmodB);
    }

    virtual ~ChBoundPeridynamics() {}

    /// Initialize again this bound.
    virtual void Reset(fea::ChNodePeridynamics* mmodA,  ///< model A
                       fea::ChNodePeridynamics* mmodB   ///< model B
                       ) {
        assert(mmodA);
        assert(mmodB);

        this->nodeA = mmodA;
        this->nodeB = mmodB;
    }

    /// Get node A
    virtual fea::ChNodePeridynamics* GetNodeA() { return this->nodeA; }
    /// Get node B
    virtual fea::ChNodePeridynamics* GetNodeB() { return this->nodeB; }

  private:
    fea::ChNodePeridynamics* nodeA;  ///< node A
    fea::ChNodePeridynamics* nodeB;  ///< node B
};



/// Class for container of many proximity pairs for a peridynamics 
/// deformable continuum (necessary for inter-particle material forces),
/// Such an item must be addd to the physical system if you added
/// an object of class ChMatterPeridynamics

class ChApi ChProximityContainerPeridynamics : public ChProximityContainer {

  protected:
    // dynamically updated list of bounds generated at each collison detection run:
    std::list<ChBoundPeridynamics*> proximal_bounds;
    std::list<ChBoundPeridynamics*>::iterator last_proximal_bound;
    int n_added;
    // elastic list of bounds, not changing at each collison detection run 
    std::list<ChBoundPeridynamics*> elastic_bounds;
    std::list<ChBoundPeridynamics*>::iterator last_elastic_bound;
    
  public:
    ChProximityContainerPeridynamics();
    ChProximityContainerPeridynamics(const ChProximityContainerPeridynamics& other);
    virtual ~ChProximityContainerPeridynamics();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChProximityContainerPeridynamics* Clone() const override { return new ChProximityContainerPeridynamics(*this); }

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

    /// Add a proximity data between two collision models, if possible.
    virtual void AddProximity(ChCollisionModel* modA,  ///< get contact model 1
                              ChCollisionModel* modB   ///< get contact model 2
                              ) override;

    /// The collision system will call BeginAddContact() after adding
    /// all contacts (for example with AddContact() or similar). This optimized version
    /// purges the end of the list of contacts that were not reused (if any).
    virtual void EndAddProximities() override;

    /// Scans all the proximity pairs and, for each pair, executes the OnReportProximity()
    /// function of the provided callback object.
    virtual void ReportAllProximities(ReportProximityCallback* mcallback) override;



    // PERIDYNAMICS 
     
    /// Remove (delete) all bounds (both elastic and proximal)
    virtual void RemoveAllBounds();

    /// Class to be used as a callback interface for some user defined action to be taken
    /// for each bound  (already added to the container).
    /// It can be used to report or post-process bounds pairs.
    class ChApi ReportBoundCallback {
      public:
        virtual ~ReportBoundCallback() {}

        /// Callback used to report bounds.
        virtual bool OnReportBound(
            fea::ChNodePeridynamics* nodeA,  ///< node A (could be nullptr, if the container does not support it)
            fea::ChNodePeridynamics* nodeB   ///< node B (could be nullptr, if the container does not support it)
            ) = 0;
    };

    /// Scans all the bounds and, for each pair, executes the OnReportBound()
    /// function of the provided callback object.
    virtual void ReportAllBounds(ReportBoundCallback* mcallback);

    /// If there are proximal bonds that can be moved into the persistent elastic_bonds list, 
    /// this will do it. 
    virtual void UpdateProximalToElastic();


    // Perform some  per-edge initializations and accumulations of values
    // Will be called by the ChMatterPeridynamics item.
    void AccumulateStep1();

    // Perform some  per-edge transfer of forces,
    // Will be called by the ChMatterPeridynamics item.
    void AccumulateStep2();

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};



/// Class for handling proximity pairs for a peridynamics 
/// deformable continuum (necessary for inter-particle material forces),
/// Such an item must be addd to the physical system if you want to use 
/// one or more ChMatterPeri materials.

class ChApi ChProximityContainerPeri : public ChProximityContainer {

protected:
    std::list<std::shared_ptr<fea::ChMatterPeriBase>> materials;
    int n_added;

  public:
    ChProximityContainerPeri();
    ChProximityContainerPeri(const ChProximityContainerPeri& other);
    virtual ~ChProximityContainerPeri();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChProximityContainerPeri* Clone() const override { return new ChProximityContainerPeri(*this); }

    /// Adds a peridynamic material. Materials, each acting on cluster of ChNodePeri items, must be added to this 
    /// proximity manager before the simulation starts, because it is 
    /// in charge of reporting to the materials which are the bounds with nearby nodes. 
    void AddMatter(std::shared_ptr<fea::ChMatterPeriBase> mmatter) {
        materials.push_back(mmatter);
    }

    /// Tell the number of added contacts
    virtual int GetNproximities() const override { return n_added; }

    /// Remove (delete) all contained contact data.
    virtual void RemoveAllProximities() override;

    /// The collision system will call BeginAddProximities() before adding
    /// all pairs (for example with AddProximity() ). 
    virtual void BeginAddProximities() override;

    /// Add a proximity data between two collision models, if possible.
    virtual void AddProximity(ChCollisionModel* modA,  ///< get contact model 1
                              ChCollisionModel* modB   ///< get contact model 2
                              ) override;

    /// The collision system will call BeginAddContact() after adding
    /// all contacts (for example with AddContact() or similar). This optimized version
    /// purges the end of the list of contacts that were not reused (if any).
    virtual void EndAddProximities() override;

    /// Scans all the proximity pairs and, for each pair, executes the OnReportProximity()
    /// function of the provided callback object.  
    /// DO NOTHING - please iterate on the bounds of the materials.
    virtual void ReportAllProximities(ReportProximityCallback* mcallback) override;



    // PERIDYNAMICS 
     
   
    /// Setup initial lattice of bonds, running a single shot of collision detection
    /// and then calling Compute.
    void SetupInitialBonds();  // NOT NEEDED?

    /// This will call   ComputeForcesReset(), ComputeForces(), ComputeStates()  for each material 
    virtual void Update(double mytime, bool update_assets = true) override;


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
