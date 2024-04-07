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
#include "chrono/utils/ChUtilsSamplers.h"

namespace chrono {

/// @addtogroup chrono_fea
/// @{

// Forward declaration
class fea::ChNodePeridynamics;
class fea::ChMatterPeriBase;
class fea::ChNodePeri;

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
    std::vector<std::shared_ptr<fea::ChNodePeri>> vnodes;     

    int n_added;
    bool is_updated;

  public:
    ChProximityContainerPeri();
    ChProximityContainerPeri(const ChProximityContainerPeri& other);
    virtual ~ChProximityContainerPeri();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChProximityContainerPeri* Clone() const override { return new ChProximityContainerPeri(*this); }

    // PERIDYNAMICS

    /// Adds a peridynamic material. Materials, each acting on cluster of ChNodePeri items, must be added to this 
    /// proximity manager before the simulation starts. 
    void AddMatter(std::shared_ptr<fea::ChMatterPeriBase> mmatter);

    /// Get the array of materials.
    const std::list<std::shared_ptr<fea::ChMatterPeriBase>>& GetMaterials() const { return materials; }

    /// Add a node. Only nodes that are added here will be simulated. Nodes can be shared among diffferent ChMatterPeri,
    /// but not among different ChProximityContainerPeri.
    void AddNode(std::shared_ptr<fea::ChNodePeri> m_node);

    /// Get the array of nodes.
    const std::vector<std::shared_ptr<fea::ChNodePeri>>& GetNodes() const { return vnodes; }

    // INTERFACE TO 

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



    /// Setup initial lattice of bonds, running a single shot of collision detection
    /// and then calling Compute.
    //void SetupInitialBonds();  // NOT NEEDED?

    /// This will call   ComputeForcesReset(), ComputeForces(), ComputeStates()  for each material 
    virtual void Update(double mytime, bool update_assets = true) override;


    /// Create a region filled with nodes. Adds the node to this, and to the specified matter. This is a helper function
    /// so that you avoid to create all nodes one by one with many calls
    /// to AddNode().
    /*
    void Fill(
        std::shared_ptr<fea::ChMatterPeriBase> mmatter, ///< matter to be used for this volume. Must be added too to this, via AddMatter(). 
        utils::PointVectorD& points,                    ///< points obtained from a sampler of the volume, ex. use utils::Sampler
        const double spacing,                           ///< average spacing used in sampling
        const double mass,                              ///< total mass of the volume
        const double horizon_sfactor = 1.6,             ///< the radius of horizon of the particle is 'spacing' multiplied this value
        const ChCoordsys<> mcoords = CSYSNORM           ///< position and rotation of the volume
    );
    */

    /// Create a box filled with nodes. Face nodes are automatically marked as interface, i.e. are collidable.
    /// Adds the nodes to this, and to the specified matter. This is a helper function
    /// so that you avoid to create all nodes one by one with many calls to AddNode().
    void FillBox(
        std::shared_ptr<fea::ChMatterPeriBase> mmatter, ///< matter to be used for this volume. Must be added too to this, via AddMatter(). 
        const ChVector<> size,         ///< x,y,z sizes of the box to fill (better if integer multiples of spacing)
        const double spacing,          ///< the spacing between two near nodes
        const double initial_density,  ///< density of the material inside the box, for initialization of node's masses
        const ChCoordsys<> boxcoords = CSYSNORM,  ///< position and rotation of the box
        const bool do_centeredcube = false,  ///< if false, array is simply cubic, if true is centered cubes (highest regularity)
        const double horizon_sfactor = 1.6,  ///< the radius of horizon of the particle is 'spacing' multiplied this value
        const double collision_sfactor = 0.3,///< the radius of collision shape (sphere) of the particle is 'spacing' multiplied this value
        const double randomness = 0.0       ///< randomness of the initial distribution lattice, 0...1
    );


    //
    // STATE
    //

    /// Get the number of scalar coordinates (variables), if any, in this item.
    virtual int GetDOF() override { return 3 * GetNnodes(); }

    /// Get the number of nodes.
    unsigned int GetNnodes() const { return (unsigned int)vnodes.size(); }
   


    virtual void IntStateGather(const unsigned int off_x,
                                ChState& x,
                                const unsigned int off_v,
                                ChStateDelta& v,
                                double& T) {
        unsigned int j = 0;
        for (auto& node : vnodes) {
            x.segment(off_x + 3 * j, 3) = node->pos.eigen();
            v.segment(off_v + 3 * j, 3) = node->pos_dt.eigen();
            ++j;
        }
        T = GetChTime();
    }
    virtual void IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T,
                                 bool full_update) {
        unsigned int j = 0;
        for (auto& node : vnodes) {
            node->pos = x.segment(off_x + 3 * j, 3);
            node->pos_dt = v.segment(off_v + 3 * j, 3);
            ++j;
        }
        SetChTime(T);
        Update(T, full_update);
    }
    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
        unsigned int j = 0;
        for (auto& node : vnodes) {
            a.segment(off_a + 3 * j, 3) = node->pos_dtdt.eigen();
            ++j;
        }
    }
    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
        unsigned int j = 0;
        for (auto& node : vnodes) {
            node->SetPos_dtdt(a.segment(off_a + 3 * j, 3));
            ++j;
        }
    }
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c)  {
        
        // assume nodedata.second.node->F already computed via 
        // ComputeForcesReset() and ComputeForces(), called by ChProximityPeri

        unsigned int j = 0;
        for (auto& node : vnodes) {
            // add gravity
            ChVector<> Gforce = GetSystem()->Get_G_acc() * node->GetMass();
            ChVector<> TotForce = node->F + node->UserForce + Gforce;

            R.segment(off + 3 * j, 3) += c * TotForce.eigen();
            ++j;
        }
    }
    virtual void IntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& w,
                                    const double c) {
        unsigned int j = 0;
        for (auto& node : vnodes) {
            R(off + 3 * j) += c * node->GetMass() * w(off + 3 * j);
            R(off + 3 * j + 1) += c * node->GetMass() * w(off + 3 * j + 1);
            R(off + 3 * j + 2) += c * node->GetMass() * w(off + 3 * j + 2);
            ++j;
        }
    }
    virtual void IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) {
        unsigned int j = 0;
        for (auto& node : vnodes) {
            node->variables.Get_qb() = v.segment(off_v + 3 * j, 3);
            node->variables.Get_fb() = R.segment(off_v + 3 * j, 3);
            ++j;
        }
    }
    virtual void IntFromDescriptor(const unsigned int off_v,
                                   ChStateDelta& v,
                                   const unsigned int off_L,
                                   ChVectorDynamic<>& L) {
        unsigned int j = 0;
        for (auto& node : vnodes) {
            v.segment(off_v + 3 * j, 3) = node->variables.Get_qb();
            ++j;
        }
    }

    //
    // SOLVER INTERFACE
    //

    // Override/implement system functions of ChPhysicsItem (to assemble/manage data for system solver))

    virtual void VariablesFbReset() {
        // OBSOLETE
        for (auto& node : vnodes) {
            node->variables.Get_fb().setZero();
        }
    }
    virtual void VariablesFbLoadForces(double factor = 1) {
        
        // assume nodedata.second.node->F already computed via 
        // ComputeForcesReset() and ComputeForces(), called by ChProximityPeri

        for (auto& node : vnodes) {
            // add gravity
            ChVector<> Gforce = GetSystem()->Get_G_acc() * node->GetMass();
            ChVector<> TotForce = node->F + node->UserForce + Gforce;

            node->variables.Get_fb() += factor * TotForce.eigen();
        }
    }
    virtual void VariablesQbLoadSpeed() {
        // OBSOLETE
        for (auto& node : vnodes) {
            // set current speed in 'qb', it can be used by the solver when working in incremental mode
            node->variables.Get_qb() = node->GetPos_dt().eigen();
        }
    }
    virtual void VariablesFbIncrementMq() {
        // OBSOLETE
        for (auto& node : vnodes) {
            node->variables.Compute_inc_Mb_v(node->variables.Get_fb(), node->variables.Get_qb());
        }
    }
    virtual void VariablesQbSetSpeed(double step = 0) {
        // OBSOLETE
        for (auto& node : vnodes) {
            ChVector<> old_pos_dt = node->GetPos_dt();

            // from 'qb' vector, sets body speed, and updates auxiliary data
            node->SetPos_dt(node->variables.Get_qb());

            // Compute accel. by BDF (approximate by differentiation);
            if (step) {
                node->SetPos_dtdt((node->GetPos_dt() - old_pos_dt) / step);
            }
        }
    }
    virtual void VariablesQbIncrementPosition(double step) {
        // OBSOLETE
        // if (!IsActive())
        //	return;

        // TODO PLASTIC FLOW

        for (auto& node : vnodes) {
            // Updates position with incremental action of speed contained in the
            // 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

            ChVector<> newspeed(node->variables.Get_qb());

            // ADVANCE POSITION: pos' = pos + dt * vel
            node->SetPos(node->GetPos() + newspeed * step);
        }
    }
    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) {
        // variables.SetDisabled(!IsActive());
        for (auto& node : vnodes) {
            mdescriptor.InsertVariables(&(node->variables));
        }
    }

    // Other functions

    /// Set no speed and no accelerations (but does not change the position).
    void SetNoSpeedNoAcceleration()  {
        for (auto& node : vnodes) {
            node->SetPos_dt(VNULL);
            node->SetPos_dtdt(VNULL);
        }
    }

    /// Synchronize coll.models coordinates and bounding boxes to the positions of the particles.
    virtual void SyncCollisionModels() {
        for (auto& node : vnodes) {
            if (node->GetCollisionModel())
                node->GetCollisionModel()->SyncPosition();
        }
    }

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
