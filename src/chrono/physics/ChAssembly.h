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
#include "chrono/fea/ChMesh.h"
#include "chrono/physics/ChLinksAll.h"
#include "chrono/physics/ChPhysicsItem.h"

namespace chrono {

/// Class for assemblies of items, for example ChBody, ChLink, ChMesh, etc.
/// Note that an assembly can be added to another assembly, to create a tree-like hierarchy.
/// All positions of rigid bodies, FEA nodes, etc. are assumed with respect to the absolute frame.

class ChApi ChAssembly : public ChPhysicsItem {
  public:
    ChAssembly();
    ChAssembly(const ChAssembly& other);
    virtual ~ChAssembly();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChAssembly* Clone() const override { return new ChAssembly(*this); }

    //
    // CONTAINER FUNCTIONS
    //

    /// Removes all inserted items: bodies, links, etc.
    void Clear();

    // Do not add the same item multiple times; also, do not remove items which haven't ever been added!
    // This will most often cause an assert() failure in debug mode.
    // Note. adding/removing items to the assembly doesn't call Update() automatically.

    /// Attach a body to this assembly.
    virtual void AddBody(std::shared_ptr<ChBody> body);

    /// Attach a link to this assembly.
    virtual void AddLink(std::shared_ptr<ChLinkBase> link);

    /// Attach a mesh to this assembly.
    virtual void AddMesh(std::shared_ptr<fea::ChMesh> mesh);

    /// Attach a ChPhysicsItem object that is not a body, link, or mesh.
    virtual void AddOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> item);

    /// Attach an arbitrary ChPhysicsItem (e.g. ChBody, ChParticles, ChLink, etc.) to the assembly.
    /// It will take care of adding it to the proper list of bodies, links, meshes, or generic
    /// physic item. (i.e. it calls AddBody(), AddLink(), AddMesh(), or AddOtherPhysicsItem()).
    /// Note, you cannot call Add() during an Update (i.e. items like particle generators that
    /// are already inserted in the assembly cannot call this) because not thread safe; instead,
    /// use AddBatch().
    void Add(std::shared_ptr<ChPhysicsItem> item);

    /// Items added in this way are added like in the Add() method, but not instantly,
    /// they are simply queued in a batch of 'to add' items, that are added automatically
    /// at the first Setup() call. This is thread safe.
    void AddBatch(std::shared_ptr<ChPhysicsItem> item);

    /// If some items are queued for addition in the assembly, using AddBatch(), this will
    /// effectively add them and clean the batch. Called automatically at each Setup().
    void FlushBatch();

    /// Remove a body from this assembly.
    virtual void RemoveBody(std::shared_ptr<ChBody> body);
    /// Remove a link from this assembly.
    virtual void RemoveLink(std::shared_ptr<ChLinkBase> link);
    /// Remove a mesh from the assembly.
    virtual void RemoveMesh(std::shared_ptr<fea::ChMesh> mesh);
    /// Remove a ChPhysicsItem object that is not a body or a link
    virtual void RemoveOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> item);
    /// Remove arbitrary ChPhysicsItem that was added to the assembly.
    void Remove(std::shared_ptr<ChPhysicsItem> item);

    /// Remove all bodies from this assembly.
    void RemoveAllBodies();
    /// Remove all links from this assembly.
    void RemoveAllLinks();
    /// Remove all meshes from this assembly.
    void RemoveAllMeshes();
    /// Remove all physics items  not in the body, link, or mesh lists.
    void RemoveAllOtherPhysicsItems();

    /// Get the list of bodies.
    const std::vector<std::shared_ptr<ChBody>>& Get_bodylist() const { return bodylist; }
    /// Get the list of links.
    const std::vector<std::shared_ptr<ChLinkBase>>& Get_linklist() const { return linklist; }
    /// Get the list of meshes.
    const std::vector<std::shared_ptr<fea::ChMesh>>& Get_meshlist() const { return meshlist; }
    /// Get the list of physics items that are not in the body or link lists.
    const std::vector<std::shared_ptr<ChPhysicsItem>>& Get_otherphysicslist() const { return otherphysicslist; }

    /// Search a body by its name.
    std::shared_ptr<ChBody> SearchBody(const char* name);
    /// Search a link by its name.
    std::shared_ptr<ChLinkBase> SearchLink(const char* name);
    /// Search a mesh by its name.
    std::shared_ptr<fea::ChMesh> SearchMesh(const char* name);
    /// Search from other ChPhysics items (not bodies, links, or meshes) by name.
    std::shared_ptr<ChPhysicsItem> SearchOtherPhysicsItem(const char* name);
    /// Search an item (body, link or other ChPhysics items) by name.
    std::shared_ptr<ChPhysicsItem> Search(const char* name);

    /// Search a marker by its name.
    std::shared_ptr<ChMarker> SearchMarker(const char* name);
    /// Search a marker by its unique ID.
    std::shared_ptr<ChMarker> SearchMarker(int markID);

    //
    // STATISTICS
    //

    /// Get the number of active bodies (so, excluding those that are sleeping or are fixed to ground).
    int GetNbodies() const { return nbodies; }
    /// Get the number of bodies that are in sleeping mode (excluding fixed bodies).
    int GetNbodiesSleeping() const { return nbodies_sleep; }
    /// Get the number of bodies that are fixed to ground.
    int GetNbodiesFixed() const { return nbodies_fixed; }
    /// Get the total number of bodies added to the assembly, including the grounded and sleeping bodies.
    int GetNbodiesTotal() const { return nbodies + nbodies_fixed + nbodies_sleep; }

    /// Get the number of links.
    int GetNlinks() const { return nlinks; }

    /// Get the number of meshes.
    int GetNmeshes() const { return nmeshes; }

    /// Get the number of other physics items (other than bodies, links, or meshes).
    int GetNphysicsItems() const { return nphysicsitems; }

    /// Get the number of coordinates (considering 7 coords for rigid bodies because of the 4 dof of quaternions).
    int GetNcoords() const { return ncoords; }
    /// Get the number of degrees of freedom of the assembly.
    int GetNdof() const { return ndof; }
    /// Get the number of scalar constraints added to the assembly, including constraints on quaternion norms.
    int GetNdoc() const { return ndoc; }
    /// Get the number of system variables (coordinates plus the constraint multipliers, in case of quaternions).
    int GetNsysvars() const { return nsysvars; }
    /// Get the number of coordinates (considering 6 coords for rigid bodies, 3 transl.+3rot.)
    int GetNcoords_w() const { return ncoords_w; }
    /// Get the number of scalar constraints added to the assembly.
    int GetNdoc_w() const { return ndoc_w; }
    /// Get the number of scalar constraints added to the assembly (only bilaterals).
    int GetNdoc_w_C() const { return ndoc_w_C; }
    /// Get the number of scalar constraints added to the assembly (only unilaterals).
    int GetNdoc_w_D() const { return ndoc_w_D; }
    /// Get the number of system variables (coordinates plus the constraint multipliers).
    int GetNsysvars_w() const { return nsysvars_w; }

    //
    // PHYSICS ITEM INTERFACE
    //

    /// Set the pointer to the parent ChSystem() and
    /// also add to new collision system / remove from old coll.system
    virtual void SetSystem(ChSystem* m_system) override;

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
    virtual void SetNoSpeedNoAcceleration() override;

    /// Get the number of scalar coordinates (ex. dim of position vector)
    virtual int GetDOF() override { return GetNcoords(); }
    /// Get the number of scalar coordinates of variables derivatives (ex. dim of speed vector)
    virtual int GetDOF_w() override { return GetNcoords_w(); }
    /// Get the number of scalar constraints, if any, in this item
    virtual int GetDOC() override { return GetNdoc_w(); }
    /// Get the number of scalar constraints, if any, in this item (only bilateral constr.)
    virtual int GetDOC_c() override { return GetNdoc_w_C(); };
    /// Get the number of scalar constraints, if any, in this item (only unilateral constr.)
    virtual int GetDOC_d() override { return GetNdoc_w_D(); };

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
                                 const double T) override;
    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override;
    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override;
    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void IntStateIncrement(const unsigned int off_x,
                                   ChState& x_new,
                                   const ChState& x,
                                   const unsigned int off_v,
                                   const ChStateDelta& Dv) override;
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void IntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& w,
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

    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) override;

    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor) override;
    virtual void ConstraintsLoadJacobians() override;

    virtual void InjectKRMmatrices(ChSystemDescriptor& mdescriptor) override;
    virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) override;

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

    //
    // SERIALIZATION
    //

    /// Writes the hierarchy of contained bodies, markers, etc. in ASCII
    /// readable form, mostly for debugging purposes. Level is the tab spacing at the left.
    void ShowHierarchy(ChStreamOutAscii& m_file, int level = 0);

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  protected:
    std::vector<std::shared_ptr<ChBody>> bodylist;                 ///< list of rigid bodies
    std::vector<std::shared_ptr<ChLinkBase>> linklist;             ///< list of joints (links)
    std::vector<std::shared_ptr<fea::ChMesh>> meshlist;            ///< list of meshes
    std::vector<std::shared_ptr<ChPhysicsItem>> otherphysicslist;  ///< list of other physics objects
    std::vector<std::shared_ptr<ChPhysicsItem>> batch_to_insert;   ///< list of items to insert at once

    // Statistics:
    int nbodies;        ///< number of bodies (currently active)
    int nlinks;         ///< number of links
    int nmeshes;        ///< number of meshes
    int nphysicsitems;  ///< number of other physics items
    int ncoords;        ///< number of scalar coordinates (including 4th dimension of quaternions) for all active bodies
    int ndoc;           ///< number of scalar constraints (including constr. on quaternions)
    int nsysvars;       ///< number of variables (coords+lagrangian mult.), i.e. = ncoords+ndoc  for all active bodies
    int ncoords_w;      ///< number of scalar coordinates when using 3 rot. dof. per body;  for all active bodies
    int ndoc_w;         ///< number of scalar constraints  when using 3 rot. dof. per body;  for all active bodies
    int nsysvars_w;     ///< number of variables when using 3 rot. dof. per body; i.e. = ncoords_w+ndoc_w
    int ndof;           ///< number of degrees of freedom, = ncoords-ndoc =  ncoords_w-ndoc_w ,
    int ndoc_w_C;       ///< number of scalar constraints C, when using 3 rot. dof. per body (excluding unilaterals)
    int ndoc_w_D;       ///< number of scalar constraints D, when using 3 rot. dof. per body (only unilaterals)
    int nbodies_sleep;  ///< number of bodies that are sleeping
    int nbodies_fixed;  ///< number of bodies that are fixed
};

CH_CLASS_VERSION(ChAssembly, 0)

}  // end namespace chrono

#endif
