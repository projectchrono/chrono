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
        is_modal = mmodal;
        Setup();
    }

    bool IsModalMode() { return this->is_modal; }

    /// Modal reduction methods.
    enum class ReductionType {
        HERTING,       ///< free-free modes are used as the modal basis, more suitable for subsystems in free boundary
                       ///< conditions, such as helicopter blades.
        CRAIG_BAMPTON  ///< clamped-clamped modes are used as the modal basis.
    };

    /// Do not print the debug information by default.
    bool verbose = false;

    ReductionType modal_reduction_type = ReductionType::CRAIG_BAMPTON; ///< methods for modal reduction


    /// Compute the undamped modes for the current assembly.
    /// Later you can fetch results via Get_modes_V(), Get_modes_frequencies() etc.
    /// Usually done for the assembly in full mode, but can be done also SwitchModalReductionON()
    bool ComputeModes(const ChModalSolveUndamped& n_modes_settings);

    /// Compute the undamped modes from M and K matrices. Later you can fetch results via Get_modes_V() etc.
    bool ComputeModesExternalData(ChSparseMatrix& mM,
                                  ChSparseMatrix& mK,
                                  ChSparseMatrix& full_Cq,
                                  const ChModalSolveUndamped& n_modes_settings);

    /// Compute the damped modes for the entire assembly.
    /// Expect complex eigenvalues/eigenvectors if damping is used.
    /// Later you can fetch results via Get_modes_V(), Get_modes_frequencies(), Get_modes_damping_ratios() etc.
    /// Usually done for the assembly in full mode, but can be done also after SwitchModalReductionON()
    bool ComputeModesDamped(const ChModalSolveDamped& n_modes_settings);

    /// Perform modal reduction on this assembly, from the current "full" ("boundary"+"internal") assembly.
    /// - An undamped modal analysis will be done on the full assembly with  nodes.
    /// - The "internal" nodes will be replaced by n_modes modal coordinates.
    void SwitchModalReductionON(const ChModalSolveUndamped& n_modes_settings,
                                const ChModalDamping& damping_model = ChModalDampingNone());

    /// Perform modal reduction on this assembly that contains only the "boundary" nodes, whereas
    /// the "internal" nodes have been modeled only in an external FEA software with the
    /// full ("boundary"+"internal") modes.
    /// - with an external FEA software, the full assembly is modeled with "boundary"+"internal" nodes.
    /// - with an external FEA software, the M mass matrix and the K stiffness matrix are saved to disk.
    /// - in Chrono, M and K and Cq constraint jacobians (if any) are load from disk and stored in ChSparseMatrix
    /// objects
    /// - in Chrono, only boundary nodes are added to a ChModalAssembly
    /// - in Chrono, run this function passing such M and K matrices: a modal analysis will be done on K and M
    /// Note that the size of M (and K) must be at least > m_num_coords_vel_boundary.
    void SwitchModalReductionON(
        ChSparseMatrix& full_M,
        ChSparseMatrix& full_K,
        ChSparseMatrix& full_Cq,
        const ChModalSolveUndamped&
            n_modes_settings,  ///< int as the n. of lower modes to keep, or a full ChModalSolveUndamped
        const ChModalDamping& damping_model = ChModalDampingNone());  ///< a damping model to use for the reduced model

    void ComputeMassCenter();
    void ComputeProjectionMatrix();
    void UpdateFloatingFrameOfReference();
    void UpdateTransformationMatrix();
    ChFrameMoving<> GetFloatingFrameOfReference() { return this->floating_frame_F; }

    void ComputeLocalFullKMCqMatrix(ChSparseMatrix& full_M, ChSparseMatrix& full_K, ChSparseMatrix& full_Cq);
    void DoModalReduction_HERTING(const ChModalDamping& damping_model = ChModalDampingNone());
    void DoModalReduction_CraigBamption(const ChModalDamping& damping_model = ChModalDampingNone());

    void ComputeInertialKRMmatrix();
    void ComputeStiffnessMatrix();
    void ComputeDampingMatrix();
    void ComputeModalKRMmatrix();

    /// A rigorous mathematical manuplation can be employed to derive the inertial forces and the consequent inertial
    /// damping matrix, or a linear assumption is applied to obtain quite concise expressions.
    /// True: default option, the linear assumption is used.
    /// False: rigorous deviation is used, only for internal test.
    bool use_linear_inertial_term = true;

    /// For displaying modes, you can use the following function. It sets the state of this subassembly
    /// (both boundary and inner items) using the n-th eigenvector multiplied by a "amplitude" factor * sin(phase).
    /// If you increment the phase during an animation, you will see the n-ht mode
    /// oscillating on the screen.
    /// It works also if in IsModalMode(). The mode shape is added to the state snapshot that was taken when doing the
    /// last ComputeModes() or ComputeModesDamped().
    void SetFullStateWithModeOverlay(int n_mode, double phase, double amplitude);

    /// For displaying the deformation using internal nodes, you can use the following function. Works only if
    /// IsModalMode(). It sets the state of the internal nodes of this subassembly using the current state of the modal
    /// coordinates q given the computed eigenvalues: x=V*q , then it overlays s to the state snapshot x0 stored last
    /// time one called a modal analysis. This is not necessary, but useful during animations, in fact the internal
    /// nodes would be completely neglected if IsModalMode() ; but calling this function one can update their changing
    /// positions for visualization, stress recovery, etc.
    void SetInternalStateWithModes(bool full_update);

    /// Resets the state of this subassembly (both boundary and inner items) to the state snapshot that
    /// was taken when doing the last ComputeModes() or ComputeModesDamped().
    void SetFullStateReset();

    /// Computes the increment of the subassembly (the increment of the current configuration respect
    /// to the initial undeformed configuration), and also gets the current speed.
    /// u_locred = P_W^T*[\delta qB; \delta eta]: corotated local displacement.
    /// e_locred = [qB^bar; eta]: local elastic displacement vector.
    /// edt_locred = [qB^bar_dt; eta_dt]: local elastic velocity vector.
    /// u_locred, e_locred and edt_locred are all expressed in the local frame of reference F.
    /// Two different algorithms are provided: 1. definition, 2. projection
    void GetStateLocal(ChStateDelta& u_locred,
                       ChStateDelta& e_locred,
                       ChStateDelta& edt_locred,
                       const std::string& opt = "definition");

    /// Optimization flag. Default true: when in modal reduced mode, during simulations the internal (discarded)
    /// nodes are updated anyway by superposition of modal shapes etc., for visualization or postprocessing reasons.
    /// In sake of high CPU performance, if no interest in visualization/postprocessing, one can disable this setting to
    /// false.
    void SetInternalNodesUpdate(bool mflag);

  protected:
    /// Resize modal matrices and hook up the variables to the  M K R block for the solver. To be used all times
    /// the n. of modes of modal reduction (m_num_coords_modal) is changed.
    void SetupModalData(int nmodes_reduction);

  public:
    /// Get the number of modal coordinates. Use SwitchModalReductionOn() to change it.
    int GetNumCoordinatesModal() { return m_num_coords_modal; }

    ChVectorDynamic<> res_CF; //TODO: move to private


    /// Access the vector of modal coordinates
    ChVectorDynamic<>& Get_modal_q() { return modal_q; }
    /// Access the vector of time derivative of modal coordinates (speeds)
    ChVectorDynamic<>& Get_modal_q_dt() { return modal_q_dt; }
    /// Access the vector of 2nd time derivative of modal coordinates (accelerations)
    ChVectorDynamic<>& Get_modal_q_dtdt() { return modal_q_dtdt; }

    /// Class to be used as a callback interface for computing a custom
    /// force F applied to the modal coordinates. Assuming F has size= n_modes_coords_w,
    /// A derived class must implement evaluate().
    // class ChApiModal CustomForceModalCallback {
    //   public:
    //     virtual ~CustomForceModalCallback() {}

    //    /// Compute the custom force vector applied on the modal coordinates, at the specified configuration.
    //    virtual void evaluate(ChVectorDynamic<>& computed_custom_F_modal,  //< compute F here, size= n_modes_coords_w
    //                          const ChModalAssembly& link                  ///< associated modal assembly
    //                          ) = 0;
    //};

    /// Specify the optional callback object for computing a custom modal force.
    // void RegisterCallback_CustomForceModal(std::shared_ptr<CustomForceModalCallback> mcallback) {
    //     m_custom_F_modal_callback = mcallback;
    // }

    /// Class to be used as a callback interface for computing a custom
    /// force F applied to the full (not reduced) coordinates; when in reduced mode, this force
    /// will be applied with an automatic transformation to the reduced coordinates.
    /// Assuming F has size= n_boundary_coords_w + n_internal_coords_w.
    /// A derived class must implement evaluate().
    // class ChApiModal CustomForceFullCallback {
    //   public:
    //     virtual ~CustomForceFullCallback() {}

    //    /// Compute the custom force vector applied on the full coordinates, at the specified configuration.
    //    virtual void evaluate(ChVectorDynamic<>& computed_custom_F_full,  //< compute F here, size=
    //    n_boundary_coords_w
    //                                                                      //+ n_internal_coords_w
    //                          const ChModalAssembly& link                 ///< associated modal assembly
    //                          ) = 0;
    //};

    /// Specify the optional callback object for computing a custom force acting on the full (not reduced) coordinates.
    // void RegisterCallback_CustomForceFull(std::shared_ptr<CustomForceFullCallback> mcallback) {
    //     m_custom_F_full_callback = mcallback;
    // }

    /// Access the current value of vector of custom applied forces to modal coordinates.
    /// Use a CustomForceModalCallback to change it.
    // ChVectorDynamic<>& Get_custom_F_modal() { return custom_F_modal; }
    /// Access the current value of vector of custom applied forces to original coordinates.
    /// Use a CustomForceFullCallback to change it.
    // ChVectorDynamic<>& Get_custom_F_full() { return custom_F_full; }

    /// Access the modal mass matrix - read only
    const ChMatrixDynamic<>& Get_modal_M() const { return modal_M; }
    /// Access the modal stiffness matrix - read only
    const ChMatrixDynamic<>& Get_modal_K() const { return modal_K; }
    /// Access the modal damping matrix - read only
    const ChMatrixDynamic<>& Get_modal_R() const { return modal_R; }
    /// Access the Psi matrix as in v_full = Psi * v_reduced, also {v_boundary; v_internal} = Psi * {v_boundary;
    /// v_modes} Hence Psi contains the "static modes" and the selected "dynamic modes", as in Psi = [I, 0; Psi_s,
    /// Psi_d]  where Psi_d is the matrix of the selected eigenvectors after SwitchModalReductionON().
    const ChMatrixDynamic<>& Get_modal_Psi() const { return Psi; }
    /// Access the snapshot of the old state of the full assembly at the previous time step
    const ChVectorDynamic<>& Get_full_assembly_x_old() const { return full_assembly_x; }

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

    // CONTAINER FUNCTIONS

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
    /// Remove an internal arbitrary ChPhysicsItem that was.
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
    const std::vector<std::shared_ptr<ChBody>>& GetBodiesInternal() const { return internal_bodylist; }
    /// Get the list of internal links.
    const std::vector<std::shared_ptr<ChLinkBase>>& GetLinksInternal() const { return internal_linklist; }
    /// Get the list of internal meshes.
    const std::vector<std::shared_ptr<fea::ChMesh>>& GetMeshesInternal() const { return internal_meshlist; }
    /// Get the list of internal physics items that are not in the body or link lists.
    const std::vector<std::shared_ptr<ChPhysicsItem>>& GetOtherPhysicsItemsInternal() const {
        return internal_otherphysicslist;
    }

    // STATISTICS

    /// Get the number of internal bodies
    unsigned int GetNumBodiesInternal() const { return m_num_bodies_internal; }

    /// Get the number of internal links.
    unsigned int GetNumLinksInternal() const { return m_num_links_internal; }

    /// Get the number of internal meshes.
    unsigned int GetNumMeshesInternal() const { return m_num_meshes_internal; }

    /// Get the number of other internal physics items (other than bodies, links, or meshes).
    unsigned int GetNumOtherPhysicsItemsInternal() const { return m_num_otherphysicsitems_internal; }

    /// Get the number of internal coordinates at the position level.
    /// Might differ from GetNumCoordinatesPosInternal in case of quaternions.
    unsigned int GetNumCoordinatesPosInternal() const { return m_num_coords_pos_internal; }

    /// Get the number of internal coordinates at the velocity level.
    /// Might differ from GetNumCoordinatesPosInternal in case of quaternions.
    unsigned int GetNumCoordinatesVelInternal() const { return m_num_coords_vel_internal; }

    /// Get the number of internal scalar constraints.
    unsigned int GetNumConstraintsInternal() const { return m_num_constr_internal; }

    /// Get the number of internal bilateral scalar constraints.
    unsigned int GetNumConstraintsBilateralInternal() const { return m_num_constr_bil_internal; }

    /// Get the number of internal unilateral scalar constraints.
    unsigned int GetNumConstraintsUnilateralInternal() const { return m_num_constr_uni_internal; }

    /// Get the number of boundary bodies
    unsigned int GetNumBodiesBoundary() const { return m_num_bodies_boundary; }

    /// Get the number of boundary links.
    unsigned int GetNumLinksBoundary() const { return m_num_links_boundary; }

    /// Get the number of boundary meshes.
    unsigned int GetNumMeshesBoundary() const { return m_num_meshes_boundary; }

    /// Get the number of other boundary physics items (other than bodies, links, or meshes).
    unsigned int GetNumOtherPhysicsItemsBoundary() const { return m_num_otherphysicsitems_boundary; }

    /// Get the number of boundary coordinates at the position level.
    /// Might differ from GetNumCoordinatesPosInternal in case of quaternions.
    unsigned int GetNumCoordinatesPosBoundary() const { return m_num_coords_pos_boundary; }

    /// Get the number of boundary coordinates at the velocity level.
    /// Might differ from GetNumCoordinatesPosInternal in case of quaternions.
    unsigned int GetNumCoordinatesVelBoundary() const { return m_num_coords_vel_boundary; }

    /// Get the number of boundary scalar constraints.
    unsigned int GetNumConstraintsBoundary() const { return m_num_constr_boundary; }

    /// Get the number of boundary scalar bilateral constraints (only bilaterals).
    unsigned int GetNumConstraintsBilateralBoundary() const { return m_num_constr_bil_boundary; }

    /// Get the number of boundary scalar constraints (only unilaterals).
    unsigned int GetNumConstraintsUnilateralBoundary() const { return m_num_constr_uni_boundary; }

    // OTHER FUNCTIONS

    /// Write the mass (M), damping (K), damping (R), and constraint Jacobian (C) matrices at current configuration.
    /// Assumes the rows/columns of the matrices are ordered as the ChVariable objects used in this assembly, first all
    /// the "boundary" variables then all the "inner" variables (or modal variables if switched to modal assembly).
    /// The sparse matrices are saved in COO format in [path]_M.dat [path]_K.dat [path]_R.dat, and [path]_Cq.dat.
    /// By default, uses 1-based indices (as in Matlab).
    void WriteSubassemblyMatrices(bool save_M,
                                  bool save_K,
                                  bool save_R,
                                  bool save_Cq,
                                  const std::string& path,
                                  bool one_indexed = true);

    /// Compute the mass matrix of the subassembly.
    /// Assumes the rows/columns of the matrix are ordered as the ChVariable objects used in this assembly,
    /// first the all the "boundary" itvariablesems then all the "inner" variables (or modal variables if switched to
    /// modal assembly).
    void GetSubassemblyMassMatrix(ChSparseMatrix* M);  ///< fill this system mass matrix

    /// Compute the stiffness matrix of the subassembly, i.e. the jacobian -dF/dq where F are stiff loads.
    /// Assumes the rows/columns of the matrix are ordered as the ChVariable objects used in this assembly,
    /// first the all the "boundary" variables then all the "inner" variables (or modal variables if switched to modal
    /// assembly). Note that not all loads provide a jacobian, as this is optional in their implementation.
    void GetSubassemblyStiffnessMatrix(ChSparseMatrix* K);  ///< fill this system stiffness matrix

    /// Compute the stiffness matrix of the subassembly, i.e. the jacobian -dF/dv where F are stiff loads.
    /// Assumes the rows/columns of the matrix are ordered as the ChVariable objects used in this assembly,
    /// first the all the "boundary" variables then all the "inner" variables (or modal variables if switched to modal
    /// assembly). Note that not all loads provide a jacobian, as this is optional in their implementation.
    void GetSubassemblyDampingMatrix(ChSparseMatrix* R);  ///< fill this system damping matrix

    /// Compute the constraint jacobian matrix of the subassembly, i.e. the jacobian
    /// Cq=-dC/dq where C are constraints (the lower left part of the KKT matrix).
    /// Assumes the columns of the matrix are ordered as the ChVariable objects used in this assembly,
    /// i.e. first the all the "boundary" variables then all the "inner" variables (or modal variables if switched to
    /// modal assembly), and assumes the rows of the matrix are ordered as the constraints used in this assembly, i.e.
    /// first the boundary and then the inner.
    void GetSubassemblyConstraintJacobianMatrix(ChSparseMatrix* Cq);  ///< fill this system damping matrix

    // PHYSICS ITEM INTERFACE

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
    virtual void ForceToRest() override;

    /// Get the number of scalar coordinates (ex. dim of position vector)
    virtual unsigned int GetNumCoordsPosLevel() override { return m_num_coords_pos; }
    /// Get the number of scalar coordinates of variables derivatives (ex. dim of speed vector)
    virtual unsigned int GetNumCoordsVelLevel() override { return m_num_coords_vel; }
    /// Get the number of scalar constraints, if any, in this item
    virtual unsigned int GetNumConstraints() override { return m_num_constr; }
    /// Get the number of scalar constraints, if any, in this item (only bilateral constr.)
    virtual unsigned int GetNumConstraintsBilateral() override { return m_num_constr_bil; }
    /// Get the number of scalar constraints, if any, in this item (only unilateral constr.)
    virtual unsigned int GetNumConstraintsUnilateral() override { return m_num_constr_uni; }

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

    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) override;

    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor) override;
    virtual void LoadConstraintJacobians() override;

    virtual void InjectKRMMatrices(ChSystemDescriptor& mdescriptor) override;
    virtual void LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) override;

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

    /// Get cumulative time for matrix assembly.
    double GetTimeMatrixAssembly() const { return m_timer_matrix_assembly(); }

    /// Get cumulative time for setup.
    double GetTimeSetup() const { return m_timer_setup(); }

    /// Get cumulative time for modal solver.
    double GetTimeModalSolver() const { return m_timer_modal_solver_call(); }

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    // SWAP FUNCTION

    /// Swap the contents of the two provided ChAssembly objects.
    /// Implemented as a friend (as opposed to a member function) so classes with a ChModalAssembly member can use ADL
    /// when implementing their own swap.
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

    // ChVectorDynamic<> custom_F_modal;
    // ChVectorDynamic<> custom_F_full;
    // std::shared_ptr<CustomForceModalCallback> m_custom_F_modal_callback;
    // std::shared_ptr<CustomForceFullCallback> m_custom_F_full_callback;

    // A vector to collect all external forces imposed on the internal nodes.
    // This force will be eventually transformed to the modal forces and applied on the reduced modal assembly.
    ChVectorDynamic<> full_forces_internal;

    ChKRMBlock modal_Hblock;
    ChMatrixDynamic<> modal_M;   // corresponding to boundary and modal accelerations
    ChMatrixDynamic<> modal_K;   // corresponding to boundary and modal coordinates
    ChMatrixDynamic<> modal_R;   // corresponding to boundary and modal velocites
    ChMatrixDynamic<> modal_Cq;  // corresponding to boundary and modal lagrange multipliers
    ChMatrixDynamic<> Psi;       //***TODO*** maybe prefer sparse Psi matrix, especially for upper blocks...

    // a series of new variables for the new formulas to support rotating subassembly
    ChFrameMoving<> floating_frame_F0;  // floating frame of reference F at the time of SwitchModalReductionON()
    ChFrameMoving<> floating_frame_F;
    // ChFrameMoving<> floating_frame_F_old;
    ChFrameMoving<> cog_frame;
    bool is_initialized_F = false;

    ChState full_assembly_x;  // state snapshot of full not reduced assembly at the previous time step

    // Projection matrices
    ChMatrixDynamic<> U_locred;      // rigid body modes of the reduced modal assembly in the deformed configuration
    ChMatrixDynamic<> U_locred_0;    // rigid body modes of the reduced modal assembly in the initial configuration
    ChMatrixDynamic<> Q_0;           // mapping matrix for the displacement of the floating frame F, is constant
    ChMatrixDynamic<> P_parallel_0;  // parallel projector, is constant
    ChMatrixDynamic<> P_perp_0;      // perpendicular projector, is constant
    bool is_projection_initialized = false;

    ChMatrixDynamic<> Psi_S;  // static mode transformation matrix in the mode acceleration method
    ChMatrixDynamic<> Psi_D;  // dynamic mode transformation matrix in the mode acceleration method

    ChMatrixDynamic<> Uloc_B;
    ChMatrixDynamic<> Uloc_I;
    ChMatrixDynamic<> Uloc_B_0;
    ChMatrixDynamic<> Uloc_I_0;

    ChMatrixDynamic<> L_B;
    ChMatrixDynamic<> L_I;
    ChMatrixDynamic<> P_W;  // extended transformation matrix, = diag[L_B,I]
    ChMatrixDynamic<> P_F;  // = diag[R_F, I]

    ChVectorDynamic<> g_quad;  // the quadratic velocity term of the reduced superelement

    // full system matrices in the local floating frame of reference F
    ChSparseMatrix full_M_loc;
    ChSparseMatrix full_K_loc;
    ChSparseMatrix full_R_loc;
    ChSparseMatrix full_Cq_loc;
    // reduced system matrices in the local floating frame of reference F
    ChMatrixDynamic<> M_red;
    ChMatrixDynamic<> K_red;
    ChMatrixDynamic<> R_red;
    ChMatrixDynamic<> Cq_red;

    ChMatrixDynamic<> Km_sup;  // linear material stiffness matrix of the reduced superelement
    ChMatrixDynamic<> Kg_sup;  // nonlinear geometrical stiffness matrix of the reduced superelement due to the internal
                               // forces at boundary nodes B

    ChMatrixDynamic<> Rm_sup;  // linear material damping matrix of the reduced superelement

    // linearzed inertial system matrices of the reduced superelement
    ChMatrixDynamic<> M_sup;   // tangent mass matrix of the reduced superelement
    ChMatrixDynamic<> Ri_sup;  // inertial damping (including gyroscopic damping) matrix of the reduced superelement
    ChMatrixDynamic<> Ki_sup;  // inertial stiffness matrix of the reduced superelement


    // Results of eigenvalue analysis like ComputeModes() or ComputeModesDamped():
    ChMatrixDynamic<std::complex<double>> modes_V;    // eigenvectors
    ChVectorDynamic<std::complex<double>> modes_eig;  // eigenvalues
    ChVectorDynamic<double> modes_freq;               // frequencies
    ChVectorDynamic<double> modes_damping_ratio;      // damping ratio
    ChState modes_assembly_x0;  // full state snapshot of assembly at the time of eigenvector computation

    // Statistics:

    // INTERNAL bodies, meshes etc. are NOT considered in equations of motion. These are
    // used anyway when computing modal analysis for component mode sysnthesis.
    int m_num_bodies_internal;             ///< number of internal bodies
    int m_num_links_internal;              ///< number of internal links
    int m_num_meshes_internal;             ///< number of internal meshes
    int m_num_otherphysicsitems_internal;  ///< number of internal other physics items

    int m_num_coords_pos_internal;  ///< number of scalar coordinates at position level for all active internal objects
    int m_num_coords_vel_internal;  ///< number of scalar coordinates at velocity level for all active internal objects

    int m_num_constr_internal;      ///< number of scalar constraints (velocity level), for all active internal objects
    int m_num_constr_bil_internal;  ///< number of bilateral scalar constraints (velocity level) of internal objects
    int m_num_constr_uni_internal;  ///< number of unilateral scalar constraints (velocity level) of internal objects

    // BOUNDARY bodies, meshes etc.: those of the parent class ChAssembly.
    int m_num_bodies_boundary;             ///< number of boundary bodies
    int m_num_links_boundary;              ///< number of boundary links
    int m_num_meshes_boundary;             ///< number of boundary meshes
    int m_num_otherphysicsitems_boundary;  ///< number of boundary other physics items

    int m_num_coords_pos_boundary;  ///< number of scalar coordinates at position level for all active boundary objects
    int m_num_coords_vel_boundary;  ///< number of scalar coordinates at velocity level for all active boundary objects

    int m_num_constr_boundary;      ///< number of scalar constraints (velocity level), for all active boundary objects
    int m_num_constr_bil_boundary;  ///< number of bilateral scalar constraints (velocity level) at boundary
    int m_num_constr_uni_boundary;  ///< number of unilateral scalar constraints (velocity level) at boundary

    // MODES: represent the motion of the assembly (internal, boundary nodes)
    int m_num_coords_modal;  // number of scalar coordinates at modal level (position and velocity level are the same)

    bool is_modal;

    bool internal_nodes_update;

    mutable ChTimer m_timer_matrix_assembly;
    mutable ChTimer m_timer_modal_solver_call;
    mutable ChTimer m_timer_setup;

    friend class ChSystem;
    friend class ChSystemMulticore;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // end namespace modal

CH_CLASS_VERSION(modal::ChModalAssembly, 0)

}  // end namespace chrono

#endif
