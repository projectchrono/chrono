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

#ifndef CHMODALASSEMBLY_H
#define CHMODALASSEMBLY_H

#include "chrono/physics/ChAssembly.h"
#include "chrono/solver/ChVariablesGeneric.h"

namespace chrono {

/// Class for assemblies of items, for example ChBody, ChLink, ChMesh, etc.
/// This supports component mode synthesis (CMS) to do substructuring, hence an assembly becomes a "modal body"
/// where many "internal" DOFs of finite elements will be reduced to few modal modes that are superimposed 
/// to the motion of a floating frame (for small deflections). Some nodes can be selected as "boundary nodes"
/// to allow connecting this modal assembly to external joints and forces.

class ChApi ChModalAssembly : public ChAssembly {
  public:
    ChModalAssembly();
    ChModalAssembly(const ChModalAssembly& other);
    ~ChModalAssembly();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChModalAssembly* Clone() const override { return new ChModalAssembly(*this); }

    /// Assignment operator for ChModalAssembly.
    ChModalAssembly& operator=(ChModalAssembly other);

    /// Set true to ignore the dofs of all the "internal" bodies, meshes, etc. (those added with AddInternal() )
    /// and just use the "boundary" bodies, meshes, etc. (those normally added via Add() ).
    /// Set false to consider all internal bodies, meshes etc. as normal items in a normal assembly. 
    void SetModalMode(bool mmodal) {
        this->is_modal = mmodal; 
        this->Setup();
    }

    bool IsModalMode() {
        return this->is_modal;
    }


    //
    // CONTAINER FUNCTIONS
    //

    /// Removes all inserted items: bodies, links, etc., both boundary and internal.
    void Clear();

    /// Attach an internal body to this assembly.
    void AddInternalBody(std::shared_ptr<ChBody> body);

    /// Attach an internal link to this assembly.
    void AddInternalLink(std::shared_ptr<ChLinkBase> link);

    /// Attach an internal mesh to this assembly.
    void AddInternalMesh(std::shared_ptr<fea::ChMesh> mesh);

    /// Attach an internal ChPhysicsItem object that is not a body, link, or mesh.
    void AddInternalOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> item);

    /// Attach an arbitrary internal ChPhysicsItem (e.g. ChBody, ChParticles, ChLink, etc.) to the assembly.
    /// It will take care of adding it to the proper list of internal bodies, links, meshes, or generic
    /// physic item. 
    void AddInternal(std::shared_ptr<ChPhysicsItem> item);


    /// Remove an internal body from this assembly.
    void RemoveInternalBody(std::shared_ptr<ChBody> body);
    /// Remove an internal link from this assembly.
    void RemoveInternalLink(std::shared_ptr<ChLinkBase> link);
    /// Remove an internal mesh from the assembly.
    void RemoveInternalMesh(std::shared_ptr<fea::ChMesh> mesh);
    /// Remove an internal ChPhysicsItem object that is not a body or a link
    void RemoveInternalOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> item);
    /// Remove an internal arbitrary ChPhysicsItem that was added to the assembly.
    void RemoveInternal(std::shared_ptr<ChPhysicsItem> item);

    /// Remove all internal bodies from this assembly.
    void RemoveAllInternalBodies();
    /// Remove all internal links from this assembly.
    void RemoveAllInternalLinks();
    /// Remove all meshes from this assembly.
    void RemoveAllInternalMeshes();
    /// Remove all physics items  not in the body, link, or mesh lists.
    void RemoveAllInternalOtherPhysicsItems();

    /// Get the list of internal bodies.
    const std::vector<std::shared_ptr<ChBody>>& Get_internal_bodylist() const { return internal_bodylist; }
    /// Get the list of internal links.
    const std::vector<std::shared_ptr<ChLinkBase>>& Get_internal_linklist() const { return internal_linklist; }
    /// Get the list of internal meshes.
    const std::vector<std::shared_ptr<fea::ChMesh>>& Get_internal_meshlist() const { return internal_meshlist; }
    /// Get the list of internal physics items that are not in the body or link lists.
    const std::vector<std::shared_ptr<ChPhysicsItem>>& Get_internal_otherphysicslist() const { return internal_otherphysicslist; }

    //
    // STATISTICS
    //

    /// Get the number of internal bodies 
    int GetN_internal_bodies() const { return n_internal_bodies; }
    /// Get the number of internal links.
    int GetN_internal_links() const { return n_internal_links; }
    /// Get the number of internal meshes.
    int GetN_internal_meshes() const { return n_internal_meshes; }
    /// Get the number of other internal physics items (other than bodies, links, or meshes).
    int GetN_internal_physicsItems() const { return n_internal_physicsitems; }

    /// Get the number of internal coordinates (considering 7 coords for rigid bodies because of the 4 dof of quaternions).
    int GetN_internal_coords() const { return n_internal_coords; }
    /// Get the number of internal degrees of freedom of the assembly.
    int GetN_internal_dof() const { return n_internal_dof; }
    /// Get the number of internal scalar constraints added to the assembly, including constraints on quaternion norms because of the 4 dof of quaternions.
    int GetN_internal_doc() const { return n_internal_doc; }
    /// Get the number of internal system variables (coordinates plus the constraint multipliers, in case of quaternions).
    int GetN_internal_sysvars() const { return n_internal_sysvars; }
    /// Get the number of internal coordinates (considering 6 coords for rigid bodies, 3 transl.+3rot.)
    int GetN_internal_coords_w() const { return n_internal_coords_w; }
    /// Get the number of internal scalar constraints added to the assembly.
    int GetN_internal_doc_w() const { return n_internal_doc_w; }
    /// Get the number of internal scalar constraints added to the assembly (only bilaterals).
    int GetN_internal_doc_w_C() const { return n_internal_doc_w_C; }
    /// Get the number of internal scalar constraints added to the assembly (only unilaterals).
    int GetN_internal_doc_w_D() const { return n_internal_doc_w_D; }
    /// Get the number of internal system variables (coordinates plus the constraint multipliers).
    int GetN_internal_sysvars_w() const { return n_internal_sysvars_w; }


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
    virtual int GetDOC_c() override { return GetNdoc_w_C(); }
    /// Get the number of scalar constraints, if any, in this item (only unilateral constr.)
    virtual int GetDOC_d() override { return GetNdoc_w_D(); }

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

    // Old bookkeeping system 
    /*
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
    */

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

    // SWAP FUNCTION

    /// Swap the contents of the two provided ChAssembly objects.
    /// Implemented as a friend (as opposed to a member function) so classes with a ChAssembly member can use ADL when
    /// implementing their own swap.
    friend void swap(ChAssembly& first, ChAssembly& second);

  private:
    virtual void SetupInitial() override;

    // list of BOUNDARY items: [no data, just use the bodylist. linklist etc. in parent ChAssembly class.]

    // list of INTERNAL items: 
    std::vector<std::shared_ptr<ChBody>> internal_bodylist;                 ///< list of rigid bodies
    std::vector<std::shared_ptr<ChLinkBase>> internal_linklist;             ///< list of joints (links)
    std::vector<std::shared_ptr<fea::ChMesh>> internal_meshlist;            ///< list of meshes
    std::vector<std::shared_ptr<ChPhysicsItem>> internal_otherphysicslist;  ///< list of other physics objects

    // MODAL:
    ChVariablesGenericDiagonalMass* modal_variables;
    ChVectorDynamic<> modal_q;
    ChVectorDynamic<> modal_q_dt;
    ChVectorDynamic<> modal_q_dtdt;
    ChVectorDynamic<> modal_F;

    ChKblockGeneric   modal_Hblock;
    ChMatrixDynamic<> modal_M;
    ChMatrixDynamic<> modal_K;
    ChMatrixDynamic<> modal_R;

    // Statistics:
    
    // INTERNAL bodies, meshes etc. are NOT considered in equations of motion. These are
    // used anyway when computing modal analysis for component mode sysnthesis.
    int n_internal_bodies;        ///< number of internal bodies 
    int n_internal_links;         ///< number of internal links
    int n_internal_meshes;        ///< number of internal meshes
    int n_internal_physicsitems;  ///< number of internal other physics items
    int n_internal_coords;        ///< number of internal scalar coordinates (including 4th dimension of quaternions) for all active bodies
    int n_internal_doc;           ///< number of internal scalar constraints (including constr. on quaternions)
    int n_internal_sysvars;       ///< number of internal variables (coords+lagrangian mult.), i.e. = ncoords+ndoc  for all active bodies
    int n_internal_coords_w;      ///< number of internal scalar coordinates when using 3 rot. dof. per body;  
    int n_internal_doc_w;         ///< number of internal scalar constraints  when using 3 rot. dof. per body;  for all active bodies
    int n_internal_sysvars_w;     ///< number of internal variables when using 3 rot. dof. per body; i.e. = n_internal_coords_w+n_internal_doc_w
    int n_internal_dof;           ///< number of internal degrees of freedom, = ncoords-ndoc =  ncoords_w-ndoc_w ,
    int n_internal_doc_w_C;       ///< number of internal scalar constraints C, when using 3 rot. dof. per body (excluding unilaterals)
    int n_internal_doc_w_D;       ///< number of internal scalar constraints D, when using 3 rot. dof. per body (only unilaterals)
    //int n_internal_bodies_sleep;  ///< number of internal bodies that are sleeping
    //int n_internal_bodies_fixed;  ///< number of internal bodies that are fixed
    
    // BOUNDARY bodies, meshes etc.: those of the parent class ChAssembly.
    int n_boundary_bodies;        ///< number of boundary bodies 
    int n_boundary_links;         ///< number of boundary links
    int n_boundary_meshes;        ///< number of boundary meshes
    int n_boundary_physicsitems;  ///< number of boundary other physics items
    int n_boundary_coords;        ///< number of boundary scalar coordinates (including 4th dimension of quaternions) for all active bodies
    int n_boundary_doc;           ///< number of boundary scalar constraints (including constr. on quaternions)
    int n_boundary_sysvars;       ///< number of boundary variables (coords+lagrangian mult.), i.e. = ncoords+ndoc  for all active bodies
    int n_boundary_coords_w;      ///< number of boundary scalar coordinates when using 3 rot. dof. per body;  
    int n_boundary_doc_w;         ///< number of boundary scalar constraints  when using 3 rot. dof. per body;  for all active bodies
    int n_boundary_sysvars_w;     ///< number of boundary variables when using 3 rot. dof. per body; i.e. = n_internal_coords_w+n_internal_doc_w
    int n_boundary_dof;           ///< number of boundary degrees of freedom, = ncoords-ndoc =  ncoords_w-ndoc_w ,
    int n_boundary_doc_w_C;       ///< number of boundary scalar constraints C, when using 3 rot. dof. per body (excluding unilaterals)
    int n_boundary_doc_w_D;       ///< number of boundary scalar constraints D, when using 3 rot. dof. per body (only unilaterals)

    // MODES: represent the motion of the assembly (internal, boundary nodes) 
    int n_modes_coords_w; // number of modes to be used

    /// TOTAL: use the n_coords, n_coords_w, etc. of the parent class.

    bool is_modal;

    friend class ChSystem;
    friend class ChSystemMulticore;
    friend class ChSystemDistributed;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

CH_CLASS_VERSION(ChModalAssembly, 0)

}  // end namespace chrono

#endif
