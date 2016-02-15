//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHASSEMBLY_H
#define CHASSEMBLY_H


#include <math.h>
#include "physics/ChPhysicsItem.h"
#include "physics/ChLinksAll.h"


namespace chrono {




/// Class for assemblies of items, for example ChBody, ChLink, ChMesh, etc.
/// Note that an assembly can be added to another assembly, to create a tree-like 
/// hierarchy.
/// All the positions of rigid bodies, FEA nodes, etc. are assumed respect to 
/// the absolute position. 

class ChApi ChAssembly : public ChPhysicsItem {
    CH_RTTI(ChAssembly, ChObj);

public:
    ChAssembly();

    /// Destructor
    virtual ~ChAssembly();

    /// Copy from another ChAssembly.
    void Copy(ChAssembly* source);


    //
    // CONTAINER FUNCTIONS
    // 

    /// Removes all inserted items: bodies, links, etc.
    void Clear();


    // To attach/remove items (rigid bodies, links, etc.) you must use
    // shared pointer, so that you don't need to care about item deletion,
    // which will be automatic when needed.
    // Please don't add the same item multiple times; also, don't remove
    // items which haven't ever been added! This will most often cause an assert() failure
    // in debug mode.
    // Note. adding/removing items to the system doesn't call Update() automatically.
   
    /// Attach a body to this system. Must be an object of exactly ChBody class.
    virtual void AddBody(std::shared_ptr<ChBody> newbody);
    
    /// Attach a link to this system. Must be an object of ChLink or derived classes.
    virtual void AddLink(std::shared_ptr<ChLink> newlink);

    /// Attach a ChPhysicsItem object that is not a body or link
    virtual void AddOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> newitem);
    
    /// Attach whatever type of ChPhysicsItem (ex a ChBody, or a
    /// ChParticles, or a ChLink, etc.) to the system. It will take care
    /// of adding it to the proper list: of bodies, of links, or of other generic
    /// physic item. (i.e. it calls AddBody(), AddLink() or AddOtherPhysicsItem() ).
    /// Note, you cannot call Add() during an Update (ie. items like particle generators that
    /// are already inserted in thesystem cannot call this) because not thread safe: rather
    /// use AddBatch().
    void Add(std::shared_ptr<ChPhysicsItem> newitem);

    /// Items added in this way are added like in the Add() method, but not instantly,
    /// they are simply queued in a batch of 'to add' items, that are added automatically 
    /// at the first Setup() call. This is thread safe.
    void AddBatch(std::shared_ptr<ChPhysicsItem> newitem);

    /// If some items are queued for addition in system, using AddBatch(), this will
    /// effectively add them and clean the batch. Called automatically at each Setup().
    void FlushBatch();

    /// Remove a body from this system.
    virtual void RemoveBody(std::shared_ptr<ChBody> mbody);
    /// Remove a link from this system.
    virtual void RemoveLink(std::shared_ptr<ChLink> mlink);
    /// Remove a ChPhysicsItem object that is not a body or a link
    virtual void RemoveOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> mitem);
    /// Remove whatever type of ChPhysicsItem that was added to the system.
    /// (suggestion: use this instead of old RemoveBody(), RemoveLink, etc.)
    void Remove(std::shared_ptr<ChPhysicsItem> newitem);

    /// Remove all bodies from this system.
    void RemoveAllBodies();
    /// Remove all links from this system.
    void RemoveAllLinks();
    /// Remove all physics items that were not added to body or link lists.
    void RemoveAllOtherPhysicsItems();


    /// Iterator to scan through the list of all ChBody items.
    class ChApi IteratorBodies {
      public:
        IteratorBodies(std::vector<std::shared_ptr<ChBody>>::iterator p) : node_(p) {}
        IteratorBodies& operator=(const IteratorBodies& other);
        bool operator==(const IteratorBodies& other);
        bool operator!=(const IteratorBodies& other);
        IteratorBodies& operator++();
        std::shared_ptr<ChBody> operator*();
        IteratorBodies() {}
        ~IteratorBodies() {}

      private:
        std::vector<std::shared_ptr<ChBody>>::iterator node_;
    };

    /// Get a ChBody iterator, initialized at the beginning of body list
    IteratorBodies IterBeginBodies();
    IteratorBodies IterEndBodies();

    /// Iterator to scan through the list of all ChLink items.
    class ChApi IteratorLinks {
      public:
        IteratorLinks(std::vector<std::shared_ptr<ChLink>>::iterator p) : node_(p) {}
        IteratorLinks& operator=(const IteratorLinks& other);
        ~IteratorLinks() {}
        bool operator==(const IteratorLinks& other);
        bool operator!=(const IteratorLinks& other);
        IteratorLinks& operator++();
        std::shared_ptr<ChLink> operator*();
        IteratorLinks(){};

      private:
        std::vector<std::shared_ptr<ChLink>>::iterator node_;
    };

    /// Get a ChLink iterator, initialized at the beginning of link list
    IteratorLinks IterBeginLinks();
    IteratorLinks IterEndLinks();

    /// Iterator to scan through the list of all physics items (other than bodies or links).
    class ChApi IteratorOtherPhysicsItems {
      public:
        IteratorOtherPhysicsItems(std::vector<std::shared_ptr<ChPhysicsItem>>::iterator p) : node_(p) {}
        IteratorOtherPhysicsItems& operator=(const IteratorOtherPhysicsItems& other);
        ~IteratorOtherPhysicsItems() {}
        bool operator==(const IteratorOtherPhysicsItems& other);
        bool operator!=(const IteratorOtherPhysicsItems& other);
        IteratorOtherPhysicsItems& operator++();
        std::shared_ptr<ChPhysicsItem> operator*();
        IteratorOtherPhysicsItems(){};

      private:
        std::vector<std::shared_ptr<ChPhysicsItem>>::iterator node_;
    };

    /// Get a ChPhysics iterator, initialized at the beginning of additional ChPhysicsItems
    IteratorOtherPhysicsItems IterBeginOtherPhysicsItems();
    IteratorOtherPhysicsItems IterEndOtherPhysicsItems();

    /// Iterator to scan through ALL physics items (bodies, links, 'other' physics items, contact container).
    /// Note, for performance reasons, if you know in advance that you are going to scan only ChBody items,
    /// the IteratorBodies is faster (same for IteratorLinks). Use IteratorPhysicsItems for generic cases.
    class ChApi IteratorPhysicsItems {
      public:
        IteratorPhysicsItems(ChAssembly* msys);
        IteratorPhysicsItems();
        ~IteratorPhysicsItems();
        IteratorPhysicsItems& operator=(const IteratorPhysicsItems& other);
        bool operator==(const IteratorPhysicsItems& other);
        bool operator!=(const IteratorPhysicsItems& other);
        IteratorPhysicsItems& operator++();
        std::shared_ptr<ChPhysicsItem> operator*();
        // void RewindToBegin();
        // bool ReachedEnd();
        bool HasItem();

      private:
        std::vector<std::shared_ptr<ChBody>>::iterator node_body;
        std::vector<std::shared_ptr<ChLink>>::iterator node_link;
        std::vector<std::shared_ptr<ChPhysicsItem>>::iterator node_otherphysics;
        int stage;
        std::shared_ptr<ChPhysicsItem> mptr;
        ChAssembly* msystem;
    };

    /// Get a ChPhysics iterator
    IteratorPhysicsItems IterBeginPhysicsItems();
    IteratorPhysicsItems IterEndPhysicsItems();

    /// Gets the list of children bodies -low level function-.
    /// NOTE! use this list only to enumerate etc., but NOT to
    /// remove or add items (use the appropriate Remove.. and Add..
    /// functions instead!)
    std::vector<std::shared_ptr<ChBody>>* Get_bodylist() { return &bodylist; }
    /// Gets the list of children links -low level function-.
    /// NOTE! use this list only to enumerate etc., but NOT to
    /// remove or add items (use the appropriate Remove.. and Add..
    /// functions instead!)
    std::vector<std::shared_ptr<ChLink>>* Get_linklist() { return &linklist; }
    /// Gets the list of children physics items that are not in the body or link lists.
    /// NOTE! use this list only to enumerate etc., but NOT to
    /// remove or add items (use the appropriate Remove.. and Add..
    /// functions instead!)
    std::vector<std::shared_ptr<ChPhysicsItem>>* Get_otherphysicslist() { return &otherphysicslist; }

    /// Searches a body from its ChObject name
    std::shared_ptr<ChBody> SearchBody(const char* m_name);
    /// Searches a link from its ChObject name
    std::shared_ptr<ChLink> SearchLink(const char* m_name);
    /// Searches from other ChPhysics items (not bodies or links) from name
    std::shared_ptr<ChPhysicsItem> SearchOtherPhysicsItem(const char* m_name);
    /// Searches whatever item (body, link or other ChPhysics items)
    std::shared_ptr<ChPhysicsItem> Search(const char* m_name);

    /// Searches a marker from its ChObject name 
    std::shared_ptr<ChMarker> SearchMarker(const char* m_name);
    /// Searches a marker from its unique ID 
    std::shared_ptr<ChMarker> SearchMarker(int markID);


    //
    // STATISTICS
    //

    /// Gets the number of active bodies (so, excluding those that are sleeping or are fixed to ground)
    int GetNbodies() { return nbodies; }
    /// Gets the number of bodies that are in sleeping mode (excluding fixed bodies).
    int GetNbodiesSleeping() { return nbodies_sleep; }
    /// Gets the number of bodies that are fixed to ground.
    int GetNbodiesFixed() { return nbodies_fixed; }
    /// Gets the total number of bodies added to the system, including the grounded and sleeping bodies.
    int GetNbodiesTotal() { return nbodies + nbodies_fixed + nbodies_sleep; }

    /// Gets the number of links .
    int GetNlinks() { return nlinks; }
    /// Gets the number of other physics items (not ChLinks or ChBodies).
    int GetNphysicsItems() { return nphysicsitems; }
    /// Gets the number of coordinates (considering 7 coords for rigid bodies because of the 4 dof of quaternions)
    int GetNcoords() { return ncoords; }
    /// Gets the number of degrees of freedom of the system.
    int GetNdof() { return ndof; }
    /// Gets the number of scalar constraints added to the system, including constraints on quaternion norms
    int GetNdoc() { return ndoc; }
    /// Gets the number of system variables (coordinates plus the constraint multipliers, in case of quaternions)
    int GetNsysvars() { return nsysvars; }
    /// Gets the number of coordinates (considering 6 coords for rigid bodies, 3 transl.+3rot.)
    int GetNcoords_w() { return ncoords_w; }
    /// Gets the number of scalar constraints added to the system
    int GetNdoc_w() { return ndoc_w; }
    /// Gets the number of scalar constraints added to the system (only bilaterals)
    int GetNdoc_w_C() { return ndoc_w_C; }
    /// Gets the number of scalar constraints added to the system (only unilaterals)
    int GetNdoc_w_D() { return ndoc_w_D; }
    /// Gets the number of system variables (coordinates plus the constraint multipliers)
    int GetNsysvars_w() { return nsysvars_w; }

    //
    // UPDATING/SETUP FUNCTIONS
    //

    


    //
    // PHYSICS ITEM INTERFACE
    //

    /// Set the pointer to the parent ChSystem() and 
    /// also add to new collision system / remove from old coll.system
    virtual void SetSystem(ChSystem* m_system);

    virtual void SyncCollisionModels();
    //virtual void AddCollisionModelsToSystem(); // no, already done in SetSystem iterating on all sub objects
    //virtual void RemoveCollisionModelsFromSystem(); // no, already done in SetSystem iterating on all sub objects

    /// Counts the number of bodies and links.
    /// Computes the offsets of object states in the global state.
    /// Assumes that this->offset_x this->offset_w this->offset_L are already set
    /// as starting point for offsetting all the contained sub objects.
    virtual void Setup();

    /// Updates all the auxiliary data and children of
    /// bodies, forces, links, given their current state.
    virtual void Update(bool update_assets = true);

    /// Set zero speed (and zero accelerations) in state, without changing the position.
    virtual void SetNoSpeedNoAcceleration();


    /// Get the number of scalar coordinates (ex. dim of position vector)
    virtual int GetDOF() { return GetNcoords(); }
    /// Get the number of scalar coordinates of variables derivatives (ex. dim of speed vector)
    virtual int GetDOF_w() { return GetNcoords_w(); }
    /// Get the number of scalar constraints, if any, in this item
    virtual int GetDOC() { return GetNdoc_w(); }
    /// Get the number of scalar constraints, if any, in this item (only bilateral constr.)
    virtual int GetDOC_c() { return GetNdoc_w_C(); };
    /// Get the number of scalar constraints, if any, in this item (only unilateral constr.)
    virtual int GetDOC_d() { return GetNdoc_w_D(); };

    // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
    virtual void IntStateGather(const unsigned int off_x,
                                ChState& x,
                                const unsigned int off_v,
                                ChStateDelta& v,
                                double& T);
    virtual void IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T);
    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a);
    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a);
    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L);
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L);
    virtual void IntStateIncrement(const unsigned int off_x,
                                   ChState& x_new,
                                   const ChState& x,
                                   const unsigned int off_v,
                                   const ChStateDelta& Dv);
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c);
    virtual void IntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& w,
                                    const double c);
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c);
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp);
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c);
    virtual void IntToLCP(const unsigned int off_v,
                          const ChStateDelta& v,
                          const ChVectorDynamic<>& R,
                          const unsigned int off_L,
                          const ChVectorDynamic<>& L,
                          const ChVectorDynamic<>& Qc);
    virtual void IntFromLCP(const unsigned int off_v, 
                            ChStateDelta& v, 
                            const unsigned int off_L, 
                            ChVectorDynamic<>& L);
    
    virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor);
    
    virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor);
    virtual void ConstraintsLoadJacobians();

    virtual void InjectKRMmatrices(ChLcpSystemDescriptor& mdescriptor);
    virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor);

    // Old bookkeeping system - to be removed soon
    virtual void VariablesFbReset();
    virtual void VariablesFbLoadForces(double factor = 1.);
    virtual void VariablesQbLoadSpeed();
    virtual void VariablesFbIncrementMq();
    virtual void VariablesQbSetSpeed(double step = 0.);
    virtual void VariablesQbIncrementPosition(double step);
    virtual void ConstraintsBiReset();
    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false);
    virtual void ConstraintsBiLoad_Ct(double factor = 1.);
    virtual void ConstraintsBiLoad_Qc(double factor = 1.);
    virtual void ConstraintsFbLoadForces(double factor = 1.);
    
    virtual void ConstraintsLiLoadSuggestedSpeedSolution();
    virtual void ConstraintsLiLoadSuggestedPositionSolution();
    virtual void ConstraintsLiFetchSuggestedSpeedSolution();
    virtual void ConstraintsLiFetchSuggestedPositionSolution();
    virtual void ConstraintsFetch_react(double factor = 1.);


    //
    // SERIALIZATION
    //

    
    /// Writes the hierarchy of contained bodies, markers, etc. in ASCII
    /// readable form, mostly for debugging purposes. Level is the tab spacing at the left.
    void ShowHierarchy(ChStreamOutAscii& m_file, int level=0);


    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);


protected:

    //
    // DATA
    //

    // list of rigid bodies
    std::vector<std::shared_ptr<ChBody>> bodylist;

    // list of joints (links)
    std::vector<std::shared_ptr<ChLink>> linklist;

    // list of other physic objects that are not bodies or links
    std::vector<std::shared_ptr<ChPhysicsItem>> otherphysicslist;

    // list of items to insert when doing Setup() or Flush.
    std::vector<std::shared_ptr<ChPhysicsItem>> batch_to_insert;

    // Statistics:
    int nbodies;        // number of bodies (currently active)
    int nlinks;         // number of links
    int nphysicsitems;  // number of other physics items
    int ncoords;        // number of scalar coordinates (including 4th dimension of quaternions) for all active bodies
    int ndoc;           // number of scalar costraints (including constr. on quaternions)
    int nsysvars;       // number of variables (coords+lagrangian mult.), i.e. = ncoords+ndoc  for all active bodies
    int ncoords_w;      // number of scalar coordinates when using 3 rot. dof. per body;  for all active bodies
    int ndoc_w;         // number of scalar costraints  when using 3 rot. dof. per body;  for all active bodies
    int nsysvars_w;     // number of variables when using 3 rot. dof. per body; i.e. = ncoords_w+ndoc_w
    int ndof;           // number of degrees of freedom, = ncoords-ndoc =  ncoords_w-ndoc_w ,
    int ndoc_w_C;       // number of scalar costraints C, when using 3 rot. dof. per body (excluding unilaterals)
    int ndoc_w_D;       // number of scalar costraints D, when using 3 rot. dof. per body (only unilaterals)
    int nbodies_sleep;  // number of bodies that are sleeping
    int nbodies_fixed;  // number of bodies that are fixed

};



}  // END_OF_NAMESPACE____

#endif
