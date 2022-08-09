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

#include "chrono_modal/ChApiModal.h"
#include "chrono_modal/ChModalDamping.h"
#include "chrono_modal/ChEigenvalueSolver.h"
#include "chrono/physics/ChAssembly.h"
#include "chrono/solver/ChVariablesGeneric.h"
#include <complex>

namespace chrono {
namespace modal {



/// Class for assemblies of items, for example ChBody, ChLink, ChMesh, etc.
/// This supports component mode synthesis (CMS) to do substructuring, hence an assembly becomes a "modal body"
/// where many "internal" DOFs of finite elements will be reduced to few modal modes that are superimposed 
/// to the motion of a floating frame (for small deflections). Some nodes can be selected as "boundary nodes"
/// to allow connecting this modal assembly to external joints and forces.

class ChApiModal ChModalAssembly : public ChAssembly {
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

    /// Compute the undamped modes for the current assembly. 
    /// Later you can fetch results via Get_modes_V(), Get_modes_frequencies() etc.
    /// Usually done for the assembly in full mode, but can be done also SwitchModalReductionON()
    bool ComputeModes(const ChModalSolveUndamped& n_modes_settings); ///< int as n. of lower modes to keep, or a full ChModalSolveUndamped

    /// Compute the undamped modes from M and K matrices. Later you can fetch results via Get_modes_V() etc.
    bool ComputeModesExternalData(ChSparseMatrix& mM, ChSparseMatrix& mK, ChSparseMatrix& full_Cq, 
        const ChModalSolveUndamped& n_modes_settings); ///< int as the n. of lower modes to keep, or a full ChModalSolveUndamped

    /// Compute the damped modes for the entire assembly. 
    /// Expect complex eigenvalues/eigenvectors if damping is used. 
    /// Later you can fetch results via Get_modes_V(), Get_modes_frequencies(), Get_modes_damping_ratios() etc.
    /// Usually done for the assembly in full mode, but can be done also after SwitchModalReductionON()
    bool ComputeModesDamped(const ChModalSolveDamped& n_modes_settings); ///< int as the n. of lower modes to keep, or a full ChModalSolveDamped

    /// Perform modal reduction on this assembly, from the current "full" ("boundary"+"internal") assembly.
    /// - An undamped modal analysis will be done on the full assembly with  nodes. 
    /// - The "internal" nodes will be replaced by n_modes modal coordinates.
    void SwitchModalReductionON(
        const ChModalSolveUndamped& n_modes_settings, ///< int as the n. of lower modes to keep, or a full ChModalSolveUndamped
        const ChModalDamping& damping_model = ChModalDampingNone());   ///< a damping model to use for the reduced model

    /// Perform modal reduction on this assembly that contains only the "boundary" nodes, whereas
    /// the "internal" nodes have been modeled only in an external FEA software with the 
    /// full ("boundary"+"internal") modes. 
    /// - with an external FEA software, the full assembly is modeled with "boundary"+"internal" nodes.
    /// - with an external FEA software, the M mass matrix and the K stiffness matrix are saved to disk. 
    /// - in Chrono, M and K and Cq constraint jacobians (if any) are load from disk and stored in ChSparseMatrix objects
    /// - in Chrono, only boundary nodes are added to a ChModalAssembly
    /// - in Chrono, run this function passing such M and K matrices: a modal analysis will be done on K and M
    /// Note that the size of M (and K) must be at least > n_boundary_coords_w. 
    void SwitchModalReductionON(ChSparseMatrix& full_M, ChSparseMatrix& full_K, ChSparseMatrix& full_Cq, 
        const ChModalSolveUndamped& n_modes_settings,  ///< int as the n. of lower modes to keep, or a full ChModalSolveUndamped
        const ChModalDamping& damping_model = ChModalDampingNone());    ///< a damping model to use for the reduced model


    /// For displaying modes, you can use the following function. It sets the state of this subassembly
    /// (both boundary and inner items) using the n-th eigenvector multiplied by a "amplitude" factor * sin(phase). 
    /// If you increment the phase during an animation, you will see the n-ht mode 
    /// oscillating on the screen. 
    /// It works also if in IsModalMode(). The mode shape is added to the state snapshot that was taken when doing the
    /// last ComputeModes() or ComputeModesDamped().
    void SetFullStateWithModeOverlay(int n_mode, double phase, double amplitude);

    /// For displaying the deformation using internal nodes, you can use the following function. Works only if IsModalMode().
    /// It sets the state of the internal nodes of this subassembly using the current state of the modal coordinates q
    /// given the computed eigenvalues: x=V*q , then it overlays s to the state snapshot x0 stored last time one called a modal analysis.
    /// This is not necessary, but useful during animations, in fact the internal nodes would be completely neglected if IsModalMode() ; but
    /// calling this function one can update their changing positions for visualization, stress recovery, etc.
    void SetInternalStateWithModes(bool full_update);


    /// Resets the state of this subassembly (both boundary and inner items) to the state snapshot that 
    /// was taken when doing the last ComputeModes() or ComputeModesDamped().
    void SetFullStateReset();


    /// Computes the 'local' increment of the subassembly (increment of configuration respect 
    /// to the x0 snapshot configuration, in local reference),
    /// and also gets the speed in local reference.
    void GetStateLocal(ChStateDelta& Dx_local, ChStateDelta& v_local);


    /// Optimization flag. Default true: when in modal reduced mode, during simulations the internal (discarded) 
    /// nodes are updated anyway by superposition of modal shapes etc., for visualization or postprocessing reasons.
    /// In sake of high CPU performance, if no interest in visualization/postprocessing, one can disable this setting to false.
    void SetInternalNodesUpdate(bool mflag);


protected:
    /// Resize modal matrices and hook up the variables to the  M K R block for the solver. To be used all times
    /// the n. of modes of modal reduction (n_modes_coords_w) is changed.
    void SetupModalData(int nmodes_reduction);

public:
    /// Get the number of modal coordinates. Use SwitchModalReductionOn() to change it.
    int Get_n_modes_coords_w() { return n_modes_coords_w; }

    /// Access the vector of modal coordinates
    ChVectorDynamic<>& Get_modal_q() { return modal_q; }
    /// Access the vector of time derivative of modal coordinates (speeds)
    ChVectorDynamic<>& Get_modal_q_dt()  { return modal_q_dt; }
    /// Access the vector of 2nd time derivative of modal coordinates (accelerations)
    ChVectorDynamic<>& Get_modal_q_dtdt()  { return modal_q_dtdt; }
    
    /// Class to be used as a callback interface for computing a custom 
    /// force F applied to the modal coordinates. Assuming F has size= n_modes_coords_w,
    /// A derived class must implement evaluate().
    class ChApiModal CustomForceModalCallback {
      public:
        virtual ~CustomForceModalCallback() {}

        /// Compute the custom force vector applied on the modal coordinates, at the specified configuration.
        virtual void evaluate(  ChVectorDynamic<>& computed_custom_F_modal, //< compute F here, size= n_modes_coords_w
                                const ChModalAssembly& link  ///< associated modal assembly
                                ) = 0;
    };

    /// Specify the optional callback object for computing a custom modal force.
    void RegisterCallback_CustomForceModal(std::shared_ptr<CustomForceModalCallback> mcallback) { m_custom_F_modal_callback = mcallback; }

    /// Class to be used as a callback interface for computing a custom 
    /// force F applied to the full (not reduced) coordinates; when in reduced mode, this force
    /// will be applied with an automatic transformation to the reduced coordinates. 
    /// Assuming F has size= n_boundary_coords_w + n_internal_coords_w.
    /// A derived class must implement evaluate().
    class ChApiModal CustomForceFullCallback {
      public:
        virtual ~CustomForceFullCallback() {}

        /// Compute the custom force vector applied on the full coordinates, at the specified configuration.
        virtual void evaluate(  ChVectorDynamic<>& computed_custom_F_full, //< compute F here, size= n_boundary_coords_w + n_internal_coords_w
                                const ChModalAssembly& link  ///< associated modal assembly
                                ) = 0;
    };

    /// Specify the optional callback object for computing a custom force acting on the full (not reduced) coordinates.
    void RegisterCallback_CustomForceFull(std::shared_ptr<CustomForceFullCallback> mcallback) { m_custom_F_full_callback = mcallback; }


    /// Access the current value of vector of custom applied forces to modal coordinates. 
    /// Use a CustomForceModalCallback to change it.
    ChVectorDynamic<>& Get_custom_F_modal()  { return custom_F_modal; }
    /// Access the current value of vector of custom applied forces to original coordinates.
    /// Use a CustomForceFullCallback to change it.
    ChVectorDynamic<>& Get_custom_F_full()  { return custom_F_full; }

    /// Access the modal mass matrix - read only
    const ChMatrixDynamic<>& Get_modal_M() const { return modal_M; }
    /// Access the modal stiffness matrix - read only
    const ChMatrixDynamic<>& Get_modal_K() const { return modal_K; }
    /// Access the modal damping matrix - read only
    const ChMatrixDynamic<>& Get_modal_R() const { return modal_R; }
    /// Access the Psi matrix as in v_full = Psi * v_reduced, also {v_boundary; v_internal} = Psi * {v_boundary; v_modes} 
    /// Hence Psi contains the "static modes" and the selected "dynamic modes", as in
    /// Psi = [I, 0; Psi_s, Psi_d]  where Psi_d is the matrix of the selected eigenvectors after SwitchModalReductionON().
    const ChMatrixDynamic<>& Get_modal_Psi() const { return Psi; }
    /// Access the snapshot of initial state of the full assembly just at the beginning of SwitchModalReductionON()
    const ChVectorDynamic<>& Get_assembly_x0() const { return assembly_x0; }


    // Use the following function to access results of ComputeModeDamped() or ComputeModes(): 

    /// Access the modal eigenvectors, if previously computed. 
    /// Read only. Use one of the ComputeModes() functions to set it.
    const ChMatrixDynamic<std::complex<double>>& Get_modes_V() const { return modes_V; }

    /// Access the modal eigenvalues, if previously computed.
    /// Read only. Use one of the ComputeModes() functions to set it.
    const ChVectorDynamic<std::complex<double>>& Get_modes_eig() const { return modes_eig; }

    /// Get a vector of (undamped) modal natural frequencies [Hz], if previously computed.
    /// Read only. Use one of the ComputeModes() functions to set it.
    const ChVectorDynamic<double>& Get_modes_frequencies() const { return this->modes_freq; }

    /// Get a vector of modal damping ratios = damping/critical_damping, if previously computed.
    /// Read only. Use one of the ComputeModes() functions to set it.
    const ChVectorDynamic<double>& Get_modes_damping_ratios() const { return this->modes_damping_ratio; }

    /// Access the snapshot of initial state of the assembly at the moment of the analysis.
    /// Read only. Use one of the ComputeModes() functions to set it.
    const ChVectorDynamic<>& Get_modes_assembly_x0() const { return modes_assembly_x0; }


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

    /// Get the number of boundary bodies 
    int GetN_boundary_bodies() const { return n_boundary_bodies; }
    /// Get the number of boundary links.
    int GetN_boundary_links() const { return n_boundary_links; }
    /// Get the number of boundary meshes.
    int GetN_boundary_meshes() const { return n_boundary_meshes; }
    /// Get the number of other boundary physics items (other than bodies, links, or meshes).
    int GetN_boundary_physicsItems() const { return n_boundary_physicsitems; }

    /// Get the number of boundary coordinates (considering 7 coords for rigid bodies because of the 4 dof of quaternions).
    int GetN_boundary_coords() const { return n_boundary_coords; }
    /// Get the number of boundary degrees of freedom of the assembly.
    int GetN_boundary_dof() const { return n_boundary_dof; }
    /// Get the number of boundary scalar constraints added to the assembly, including constraints on quaternion norms because of the 4 dof of quaternions.
    int GetN_boundary_doc() const { return n_boundary_doc; }
    /// Get the number of boundary system variables (coordinates plus the constraint multipliers, in case of quaternions).
    int GetN_boundary_sysvars() const { return n_boundary_sysvars; }
    /// Get the number of boundary coordinates (considering 6 coords for rigid bodies, 3 transl.+3rot.)
    int GetN_boundary_coords_w() const { return n_boundary_coords_w; }
    /// Get the number of boundary scalar constraints added to the assembly.
    int GetN_boundary_doc_w() const { return n_boundary_doc_w; }
    /// Get the number of boundary scalar constraints added to the assembly (only bilaterals).
    int GetN_boundary_doc_w_C() const { return n_boundary_doc_w_C; }
    /// Get the number of boundary scalar constraints added to the assembly (only unilaterals).
    int GetN_boundary_doc_w_D() const { return n_boundary_doc_w_D; }
    /// Get the number of boundary system variables (coordinates plus the constraint multipliers).
    int GetN_boundary_sysvars_w() const { return n_boundary_sysvars_w; }


    //
    // OTHER FUNCTIONS
    //

    /// Dump the  M mass matrix, K damping matrix, R damping matrix, Cq constraint jacobian
    /// matrix (at the current configuration) for this subassembly,
    /// Assumes the rows/columns of the matrices are ordered as the ChVariable objects used in this assembly, 
    /// first the all the "boundary" variables then all the "inner" variables (or modal variables if switched to modal assembly).
    /// The name of the files will be [path]_M.dat [path]_K.dat [path]_R.dat [path]_Cq.dat 
    /// Might throw ChException if file can't be saved.
    void DumpSubassemblyMatrices(bool save_M, bool save_K, bool save_R, bool save_Cq, const char* path);

    /// Compute the mass matrix of the subassembly. 
    /// Assumes the rows/columns of the matrix are ordered as the ChVariable objects used in this assembly, 
    /// first the all the "boundary" itvariablesems then all the "inner" variables (or modal variables if switched to modal assembly).
    void GetSubassemblyMassMatrix(ChSparseMatrix* M);    ///< fill this system mass matrix

    /// Compute the stiffness matrix of the subassembly, i.e. the jacobian -dF/dq where F are stiff loads.
    /// Assumes the rows/columns of the matrix are ordered as the ChVariable objects used in this assembly, 
    /// first the all the "boundary" variables then all the "inner" variables (or modal variables if switched to modal assembly).
    /// Note that not all loads provide a jacobian, as this is optional in their implementation.
    void GetSubassemblyStiffnessMatrix(ChSparseMatrix* K);    ///< fill this system stiffness matrix

    /// Compute the stiffness matrix of the subassembly, i.e. the jacobian -dF/dv where F are stiff loads.
    /// Assumes the rows/columns of the matrix are ordered as the ChVariable objects used in this assembly, 
    /// first the all the "boundary" variables then all the "inner" variables (or modal variables if switched to modal assembly).
    /// Note that not all loads provide a jacobian, as this is optional in their implementation.
    void GetSubassemblyDampingMatrix(ChSparseMatrix* R);    ///< fill this system damping matrix

    /// Compute the constraint jacobian matrix of the subassembly, i.e. the jacobian
    /// Cq=-dC/dq where C are constraints (the lower left part of the KKT matrix).
    /// Assumes the columns of the matrix are ordered as the ChVariable objects used in this assembly, 
    /// i.e. first the all the "boundary" variables then all the "inner" variables (or modal variables if switched to modal assembly),
    /// and assumes the rows of the matrix are ordered as the constraints used in this assembly, i.e. first the boundary and then the inner.
    void GetSubassemblyConstraintJacobianMatrix(ChSparseMatrix* Cq);  ///< fill this system damping matrix


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
    /// Implemented as a friend (as opposed to a member function) so classes with a ChModalAssembly member can use ADL when
    /// implementing their own swap.
    friend void swap(ChModalAssembly& first, ChModalAssembly& second);

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

    ChVectorDynamic<> custom_F_modal;
    ChVectorDynamic<> custom_F_full;
    std::shared_ptr<CustomForceModalCallback> m_custom_F_modal_callback;
    std::shared_ptr<CustomForceFullCallback> m_custom_F_full_callback;

    ChKblockGeneric   modal_Hblock;
    ChMatrixDynamic<> modal_M;
    ChMatrixDynamic<> modal_K;
    ChMatrixDynamic<> modal_R;
    ChMatrixDynamic<> Psi; //***TODO*** maybe prefer sparse Psi matrix, especially for upper blocks...
    ChState           assembly_x0;      // state snapshot of full not reduced assembly at the time of SwitchModalReductionON()

    // Results of eigenvalue analysis like ComputeModes() or ComputeModesDamped(): 
    ChMatrixDynamic<std::complex<double>> modes_V;             // eigenvectors
    ChVectorDynamic<std::complex<double>> modes_eig;           // eigenvalues
    ChVectorDynamic<double>               modes_freq;          // frequencies
    ChVectorDynamic<double>               modes_damping_ratio; // damping ratio
    ChState                               modes_assembly_x0;   // state snapshot of assembly at the time of eigenvector computation

    

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

    bool internal_nodes_update;

    friend class ChSystem;
    friend class ChSystemMulticore;
    friend class ChSystemDistributed;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // end namespace modal

CH_CLASS_VERSION(modal::ChModalAssembly, 0)

}  // end namespace chrono

#endif
