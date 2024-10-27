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

#ifndef CHASSEMBLY_H
#define CHASSEMBLY_H

#include <cmath>
#include <vector>

#include "chrono/fea/ChMesh.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChLinksAll.h"

namespace chrono {

/// Class for assemblies of physical items.
/// This class is a container of ChPhysicsItems items i.e. rigid bodies, shafts, links, etc.
/// ChAssembly objects can also contain other ChAssembly objects. Location and rotation of the
/// contained ChPhysicsItems are assumed to be expressed with respect to the absolute frame.
class ChApi ChAssembly : public ChPhysicsItem {
  public:
    ChAssembly();
    ChAssembly(const ChAssembly& other);
    ~ChAssembly();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChAssembly* Clone() const override { return new ChAssembly(*this); }

    /// Assignment operator for ChAssembly.
    ChAssembly& operator=(ChAssembly other);

    // CONTAINER FUNCTIONS

    /// Removes all inserted items: bodies, links, etc.
    void Clear();

    // Do not add the same item multiple times; also, do not remove items which haven't ever been added!
    // This will most often cause an assert() failure in debug mode.
    // Note. adding/removing items to the assembly doesn't call Update() automatically.

    /// Attach a body to this assembly.
    void AddBody(std::shared_ptr<ChBody> body);

    /// Attach a shaft to this assembly.
    void AddShaft(std::shared_ptr<ChShaft> shaft);

    /// Attach a link to this assembly.
    void AddLink(std::shared_ptr<ChLinkBase> link);

    /// Attach a mesh to this assembly.
    void AddMesh(std::shared_ptr<fea::ChMesh> mesh);

    /// Attach a ChPhysicsItem object that is not a body, link, or mesh.
    void AddOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> item);

    /// Attach an arbitrary ChPhysicsItem (e.g. ChBody, ChParticles, ChLink, etc.) to the assembly.
    /// It will take care of adding it to the proper list of bodies, links, meshes, or generic physic item. (i.e. it
    /// calls AddBody(), AddShaft(), AddLink(), AddMesh(), or AddOtherPhysicsItem()). Note, you cannot call Add() during
    /// an Update (i.e. items like particle generators that are already inserted in the assembly cannot call this)
    /// because not thread safe; instead, use AddBatch().
    void Add(std::shared_ptr<ChPhysicsItem> item);

    /// Items added in this way are added like in the Add() method, but not instantly,
    /// they are simply queued in a batch of 'to add' items, that are added automatically
    /// at the first Setup() call. This is thread safe.
    void AddBatch(std::shared_ptr<ChPhysicsItem> item);

    /// If some items are queued for addition in the assembly, using AddBatch(), this will
    /// effectively add them and clean the batch. Called automatically at each Setup().
    void FlushBatch();

    /// Remove a body from this assembly.
    void RemoveBody(std::shared_ptr<ChBody> body);
    /// Remove a shaft from this assembly.
    void RemoveShaft(std::shared_ptr<ChShaft> shaft);
    /// Remove a link from this assembly.
    void RemoveLink(std::shared_ptr<ChLinkBase> link);
    /// Remove a mesh from the assembly.
    void RemoveMesh(std::shared_ptr<fea::ChMesh> mesh);
    /// Remove a ChPhysicsItem object that is not a body or a link
    void RemoveOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> item);
    /// Remove arbitrary ChPhysicsItem that was added to the assembly.
    void Remove(std::shared_ptr<ChPhysicsItem> item);

    /// Remove all bodies from this assembly.
    void RemoveAllBodies();
    /// Remove all shafts from this assembly.
    void RemoveAllShafts();
    /// Remove all links from this assembly.
    void RemoveAllLinks();
    /// Remove all meshes from this assembly.
    void RemoveAllMeshes();
    /// Remove all physics items  not in the body, link, or mesh lists.
    void RemoveAllOtherPhysicsItems();

    /// Get the list of bodies.
    virtual const std::vector<std::shared_ptr<ChBody>>& GetBodies() const { return bodylist; }
    /// Get the list of shafts.
    virtual const std::vector<std::shared_ptr<ChShaft>>& GetShafts() const { return shaftlist; }
    /// Get the list of links.
    virtual const std::vector<std::shared_ptr<ChLinkBase>>& GetLinks() const { return linklist; }
    /// Get the list of meshes.
    virtual const std::vector<std::shared_ptr<fea::ChMesh>>& GetMeshes() const { return meshlist; }
    /// Get the list of physics items that are not in the body or link lists.
    virtual const std::vector<std::shared_ptr<ChPhysicsItem>>& GetOtherPhysicsItems() const { return otherphysicslist; }

    /// Search a body by its name.
    std::shared_ptr<ChBody> SearchBody(const std::string& name) const;
    /// Search a body by its ID
    std::shared_ptr<ChBody> SearchBodyID(int id) const;
    /// Search a shaft by its name.
    std::shared_ptr<ChShaft> SearchShaft(const std::string& name) const;
    /// Search a link by its name.
    std::shared_ptr<ChLinkBase> SearchLink(const std::string& name) const;
    /// Search a mesh by its name.
    std::shared_ptr<fea::ChMesh> SearchMesh(const std::string& name) const;
    /// Search from other ChPhysics items (not bodies, links, or meshes) by name.
    std::shared_ptr<ChPhysicsItem> SearchOtherPhysicsItem(const std::string& name) const;
    /// Search a marker by its name.
    std::shared_ptr<ChMarker> SearchMarker(const std::string& name) const;
    /// Search a marker by its unique ID.
    std::shared_ptr<ChMarker> SearchMarker(int id) const;
    /// Search an item (body, link or other ChPhysics items) by name.
    std::shared_ptr<ChPhysicsItem> Search(const std::string& name) const;

    // STATISTICS

    /// Get the total number of bodies added to the assembly, including fixed and sleeping bodies.
    unsigned int GetNumBodies() const { return m_num_bodies_active + m_num_bodies_fixed + m_num_bodies_sleep; }

    /// Get the number of active bodies, excluding sleeping or fixed.
    unsigned int GetNumBodiesActive() const { return m_num_bodies_active; }

    /// Get the number of sleeping bodies.
    unsigned int GetNumBodiesSleeping() const { return m_num_bodies_sleep; }

    /// Get the number of bodies fixed to ground.
    unsigned int GetNumBodiesFixed() const { return m_num_bodies_fixed; }

    /// Get the number of shafts.
    unsigned int GetNumShafts() const { return m_num_shafts; }

    /// Get the number of shafts that are in sleeping mode (excluding fixed shafts).
    unsigned int GetNumShaftsSleeping() const { return m_num_shafts_sleep; }

    /// Get the number of shafts that are fixed to ground.
    unsigned int GetNumShaftsFixed() const { return m_num_shafts_fixed; }

    /// Get the total number of shafts added to the assembly, including the grounded and sleeping shafts.
    unsigned int GetNumShaftsTotal() const { return m_num_shafts + m_num_shafts_fixed + m_num_shafts_sleep; }

    /// Get the number of links (including non active).
    unsigned int GetNumLinks() const { return (unsigned int)linklist.size(); }

    /// Get the number of active links.
    unsigned int GetNumLinksActive() const { return m_num_links_active; }

    /// Get the number of meshes.
    unsigned int GetNumMeshes() const { return m_num_meshes; }

    /// Get the number of other active physics items (including non active).
    unsigned int GetNumOtherPhysicsItems() const { return (unsigned int)otherphysicslist.size(); }

    /// Get the number of other active physics items.
    unsigned int GetNumOtherPhysicsItemsActive() const { return m_num_otherphysicsitems_active; }

    /// Get the number of scalar coordinates at the position level.
    /// This count includes the 4th dimension of quaternions (if any), thus potentially differing from
    /// GetNumCoordsVelLevel().
    virtual unsigned int GetNumCoordsPosLevel() override { return m_num_coords_pos; }

    /// Get the number of scalar coordinates at the velocity level.
    virtual unsigned int GetNumCoordsVelLevel() override { return m_num_coords_vel; }

    /// Get the number of scalar constraints in the assembly.
    virtual unsigned int GetNumConstraints() override { return m_num_constr; }

    /// Get the number of scalar bilateral constraints in the assembly.
    virtual unsigned int GetNumConstraintsBilateral() override { return m_num_constr_bil; }

    /// Get the number of scalar unilateral constraints in the assembly.
    virtual unsigned int GetNumConstraintsUnilateral() override { return m_num_constr_uni; }

    // PHYSICS ITEM INTERFACE

    /// Set the pointer to the parent ChSystem() and
    /// also add to new collision system / remove from old coll.system
    virtual void SetSystem(ChSystem* m_system) override;

    /// Add collision models (if any) for all items in the assembly to the provided collision system.
    virtual void AddCollisionModelsToSystem(ChCollisionSystem* coll_sys) const override;

    /// Remove the collision models (if any) for all items in the assembly from the provided collision system.
    virtual void RemoveCollisionModelsFromSystem(ChCollisionSystem* coll_sys) const override;

    /// Synchronize collision models for all physics items in this assembly.
    virtual void SyncCollisionModels() override;

    /// Counts the number of bodies, links, and meshes.
    /// Computes the offsets of object states in the global state.
    /// Assumes that this->offset_x this->offset_w this->offset_L are already set
    /// as starting point for offsetting all the contained sub objects.
    virtual void Setup() override;

    /// Updates all the auxiliary data and children of
    /// bodies, forces, links, given their current state.
    virtual void Update(double mytime, bool update_assets = true) override;

    /// Updates all the auxiliary data and children of
    /// bodies, forces, links, given their current state.
    virtual void Update(bool update_assets = true) override;

    /// Set zero speed (and zero accelerations) in state, without changing the position.
    virtual void ForceToRest() override;

    // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
    virtual void IntStateGather(const unsigned int off_x,
                                ChState& x,
                                const unsigned int off_v,
                                ChStateDelta& v,
                                double& T) override;
    virtual void IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T,
                                 bool full_update) override;
    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override;
    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override;
    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void IntStateIncrement(const unsigned int off_x,
                                   ChState& x_new,
                                   const ChState& x,
                                   const unsigned int off_v,
                                   const ChStateDelta& Dv) override;
    virtual void IntStateGetIncrement(const unsigned int off_x,
                                      const ChState& x_new,
                                      const ChState& x,
                                      const unsigned int off_v,
                                      ChStateDelta& Dv) override;
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void IntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& w,
                                    const double c) override;
    virtual void IntLoadLumpedMass_Md(const unsigned int off,
                                      ChVectorDynamic<>& Md,
                                      double& err,
                                      const double c) override;
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c) override;
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp) override;
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override;
    virtual void IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) override;
    virtual void IntFromDescriptor(const unsigned int off_v,
                                   ChStateDelta& v,
                                   const unsigned int off_L,
                                   ChVectorDynamic<>& L) override;

    virtual void InjectVariables(ChSystemDescriptor& descriptor) override;

    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override;
    virtual void LoadConstraintJacobians() override;

    virtual void InjectKRMMatrices(ChSystemDescriptor& descriptor) override;
    virtual void LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) override;

    // Old bookkeeping system - to be removed soon
    virtual void VariablesFbReset() override;
    virtual void VariablesFbLoadForces(double factor = 1) override;
    virtual void VariablesQbLoadSpeed() override;
    virtual void VariablesFbIncrementMq() override;
    virtual void VariablesQbSetSpeed(double step = 0) override;
    virtual void VariablesQbIncrementPosition(double step) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void ConstraintsBiLoad_Qc(double factor = 1) override;
    virtual void ConstraintsFbLoadForces(double factor = 1) override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

    // SERIALIZATION

    /// Writes the hierarchy of contained bodies, markers, etc. in ASCII
    /// readable form, mostly for debugging purposes. Level is the tab spacing at the left.
    void ShowHierarchy(std::ostream& outstream, int level = 0) const;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    // SWAP FUNCTION

    /// Swap the contents of the two provided ChAssembly objects.
    /// Implemented as a friend (as opposed to a member function) so classes with a ChAssembly member can use ADL when
    /// implementing their own swap.
    friend ChApi void swap(ChAssembly& first, ChAssembly& second);

  protected:
    virtual void SetupInitial() override;

    std::vector<std::shared_ptr<ChBody>> bodylist;                 ///< list of rigid bodies
    std::vector<std::shared_ptr<ChShaft>> shaftlist;               ///< list of 1-D shafts
    std::vector<std::shared_ptr<ChLinkBase>> linklist;             ///< list of joints (links)
    std::vector<std::shared_ptr<fea::ChMesh>> meshlist;            ///< list of meshes
    std::vector<std::shared_ptr<ChPhysicsItem>> otherphysicslist;  ///< list of other physics objects
    std::vector<std::shared_ptr<ChPhysicsItem>> batch_to_insert;   ///< list of items to insert at once

    // Statistics:
    unsigned int m_num_bodies_active;             ///< number of active bodies
    unsigned int m_num_bodies_sleep;              ///< number of sleeping bodies
    unsigned int m_num_bodies_fixed;              ///< number of fixed bodies
    unsigned int m_num_shafts;                    ///< number of active shafts
    unsigned int m_num_shafts_sleep;              ///< number of sleeping shafts
    unsigned int m_num_shafts_fixed;              ///< number of fixed shafts
    unsigned int m_num_links_active;              ///< number of active links
    unsigned int m_num_meshes;                    ///< number of meshes
    unsigned int m_num_otherphysicsitems_active;  ///< number of other active physics items

    unsigned int m_num_coords_pos;  ///< number of scalar position-level coordinates for all active bodies
    unsigned int m_num_coords_vel;  ///< number of scalar velocity-level coordinates for all active bodies
    unsigned int m_num_constr;      ///< number of scalar constraints
    unsigned int m_num_constr_bil;  ///< number of scalar bilateral constraints
    unsigned int m_num_constr_uni;  ///< number of scalar unilateral constraints

    friend class ChSystem;
    friend class ChSystemMulticore;
};

CH_CLASS_VERSION(ChAssembly, 0)

}  // end namespace chrono

#endif
