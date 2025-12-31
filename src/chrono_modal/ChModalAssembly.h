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
#include "chrono_modal/ChModalSolverUndamped.h"
#include "chrono_modal/ChModalSolverUndamped.h"
#include "chrono_modal/ChModalSolverDamped.h"
#include "chrono/physics/ChAssembly.h"
#include "chrono/solver/ChVariablesGeneric.h"
#include <complex>
#include <iomanip>

namespace chrono {
namespace modal {

/// @addtogroup modal
/// @{

/// Class for assemblies of items, for example ChBody, ChLink, ChMesh, etc.
/// This supports component mode synthesis (CMS) to do substructuring, hence an assembly becomes a "modal body"
/// where many "internal" DOFs of finite elements will be reduced to few modal modes that are superimposed
/// to the motion of a floating frame (for small deflections). Some nodes can be selected as "boundary nodes"
/// to allow connecting this modal assembly to external joints and forces.
///
/// =============================================================================
/// The development of this modal reduction procedure has been inspired by the research paper:
/// Sonneville, V., Scapolan, M., Shan, M. et al. Modal reduction procedures for flexible multibody dynamics. Multibody
/// Syst Dyn 51, 377–418 (2021).

class ChApiModal ChModalAssembly : public ChAssembly {
  public:
    ChModalAssembly();
    ChModalAssembly(const ChModalAssembly& other);
    ~ChModalAssembly();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChModalAssembly* Clone() const override { return new ChModalAssembly(*this); }

    /// Assignment operator for ChModalAssembly.
    ChModalAssembly& operator=(ChModalAssembly other);

    /// Modal reduction methods.
    enum class ReductionType {
        HERTING,       ///< free-free modes are used as the modal basis, more suitable for subsystems with free boundary
                       ///< conditions, such as helicopter blades or wind turbine blades.
        CRAIG_BAMPTON  ///< clamped-clamped modes are used as the modal basis.
    };

    /// Set the type of modal reduction to be used.
    void SetReductionType(ReductionType type) { m_modal_reduction_type = type; }

    /// Get the type of modal reduction used.
    ReductionType GetReductionType() const { return m_modal_reduction_type; }

    ///// Compute the damped modes for the entire modal assembly.
    ///// Expect complex eigenvalues/eigenvectors if damping is used.
    ///// Later you can fetch results via GetEigenVectors(), GetUndampedFrequencies(),
    ///// GetDampingRatios() etc. Usually done for the assembly in full state, not available in reduced
    ///// state.
    // bool ComputeModesDamped(const ChModalSolverUndamped& modal_solver);

    /// Perform modal reduction on this modal assembly, from the current "full" ("boundary"+"internal") assembly.
    /// - An undamped modal analysis will be done on the full assembly, followed by a modal reduction transformation.
    /// - The "boundary" nodes will be retained.
    /// - The "internal" nodes will be replaced by n_modes modal coordinates.
    template <typename EigensolverType>
    void DoModalReduction(const ChModalSolverUndamped<EigensolverType>& modal_solver,
                          const ChModalDamping& damping_model = ChModalDampingNone());

    /// Perform modal reduction on this modal assembly that contains only the "boundary" nodes, whereas
    /// the "internal" nodes have been modeled only in an external FEA software with the
    /// full ("boundary"+"internal") modes.
    /// - with an external FEA software, the full assembly is modeled with "boundary"+"internal" nodes.
    /// - with an external FEA software, the M mass matrix and the K stiffness matrix are saved to disk.
    /// - in Chrono, M and K and Cq constraint jacobians (if any) are load from disk and stored in ChSparseMatrix
    /// objects.
    /// - in Chrono, only boundary nodes are added to a ChModalAssembly.
    /// - in Chrono, run this function passing such M and K matrices: a modal analysis will be done on K and M
    /// Note that the size of M (and K) must be at least > m_num_coords_vel_boundary.
    template <typename EigensolverType>
    void DoModalReduction(
        ChSparseMatrix& full_M,   ///< mass matrix of the full assembly (boundary+internal)
        ChSparseMatrix& full_K,   ///< stiffness matrix of the full assembly (boundary+internal)
        ChSparseMatrix& full_Cq,  ///< constraint jacobian matrix of the full assembly (boundary+internal)
        const ChModalSolverUndamped<EigensolverType>&
            modal_solver,  ///< settings for the modal analysis, such as the number of modes to extract
        const ChModalDamping& damping_model = ChModalDampingNone()  ///< damping model
    );

    /// Get the floating frame F of the reduced modal assembly.
    ChFrameMoving<> GetFloatingFrameOfReference() { return floating_frame_F; }

    /// Get the residual of constraint equations on the floating frame F.
    /// The configuration of the floating frame F is determined by the six constraint equations using a Newton-Raphson
    /// iteration. The residual of constraint equations is an indicator to check the convergence of the modal method.
    const ChVectorDynamic<>& GetConstraintsResidualF() { return m_res_CF; }

    /// Set verbose output.
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Set whether the static correction is used. By default, it is false.
    /// When some external forces are imposed on the internal bodies and nodes, the static correction is important to
    /// obtain a reasonable accurary of the elastic deformation and internal forces of finite elements, and
    /// reaction forces of constriants in the modal assembly. In this case, it is strongly recommended to set true.
    /// Note: only available to SetUseStaticCorrection(true) when m_internal_nodes_update = true.
    void SetUseStaticCorrection(bool flag);

    ///  Get the flag whether the static correction is used.
    bool GetUsetStaticCorrection() { return static_cast<bool>(m_num_coords_static_correction); }

    /// A rigorous mathematical manipulation can be employed to derive the inertial forces and the consequent inertial
    /// damping matrix, or a linear assumption is applied to obtain quite concise expressions.
    ///  - True: default option, the linear assumption is used.
    ///  - False: rigorous deviation is used, only for internal test.
    void SetUseLinearInertialTerm(bool flag) { m_use_linear_inertial_term = flag; }

    ///// For displaying modes, you can use the following function. It sets the state of this modal assembly
    ///// (both boundary and internal items) using the n-th eigenvector multiplied by an "amplitude" factor *
    /// sin(phase).
    ///// If you increment the phase during an animation, you will see the n-th mode oscillating on the screen.
    ///// The modal shapes are animated based on the initial full state of the modal assembly.
    ///// It works only in full state.
    // void UpdateFullStateWithModeOverlay(unsigned int n_mode, double phase, double amplitude);

    /// Updates the state of internal nodes.
    /// Works only if FlagModelAsReduced(). It sets the state of the internal nodes of this modal assembly using the
    /// current state of the modal coordinates q given the computed eigenvectors: s = V * q , then it overlays s to the
    /// state snapshot x0 stored last time one called a modal reduction. This is not necessary, but useful during
    /// animations, in fact the internal nodes would be completely neglected if m_internal_nodes_update == false; but
    /// calling this function one can update their changing positions for visualization, stress recovery, etc.
    void UpdateInternalState(bool full_update);

    /// Resets the state of this modal assembly (both boundary and internal items) to the state snapshot in the initial
    /// configuration.
    void SetFullStateReset();

    /// Optimization flag. Default true: when in modal reduced mode, during simulations the internal (discarded)
    /// nodes are updated anyway by superposition of modal shapes etc., for visualization or postprocessing purposes.
    /// In sake of high CPU performance, if no interest in visualization/postprocessing, one can disable this by setting
    /// to false, then automatically set m_num_coords_static_correction = 0 to disable the static correction mode.
    void SetInternalNodesUpdate(bool flag);

    /// If true, as by default, this modal assembly will add automatically a gravity load
    /// to all contained boundary and internal bodies/nodes (that support gravity) in the modal reduced state using the
    /// G value from the ChSystem.
    ///  - In modal reduced state, this flag will overwrite mesh->SetAutomaticGravity() for both boundary and internal
    /// meshes.
    ///  - In full state, this flap does NOT affect, and mesh->SetAutomaticGravity() is used for both boundary and
    /// internal meshes.
    /// It is recommended to use a same boolean value to ensure a consistent setting in both full and reduced state,
    /// for example:
    ///  - modal_assembly->SetModalAutomaticGravity(USE_GRAVITY);
    ///  - mesh_internal->SetAutomaticGravity(USE_GRAVITY);
    ///  - mesh_boundary->SetAutomaticGravity(USE_GRAVITY);
    void SetModalAutomaticGravity(bool gravity) { m_modal_automatic_gravity = gravity; }

    /// Tell if this modal assembly will add automatically a gravity load to all contained boundary and internal
    /// bodies/nodes.
    bool GetModalAutomaticGravity() { return m_modal_automatic_gravity; }

    /// Get the modal mass matrix.
    const ChMatrixDynamic<>& GetModalMassMatrix() const { return this->modal_M; }

    /// Get the modal stiffness matrix.
    const ChMatrixDynamic<>& GetModalStiffnessMatrix() const { return this->modal_K; }

    /// Get the modal damping matrix.
    const ChMatrixDynamic<>& GetModalDampingMatrix() const { return this->modal_R; }

    /// Get the modal reduction transformation matrix 'Psi'.
    /// 'Psi' as in v_full = Psi * v_reduced,
    /// also {v_boundary; v_internal} = Psi * {v_boundary; v_modes}.
    /// Hence Psi contains the "static modes", the selected "dynamic modes" and the "static correction mode", as
    /// Psi  = [ I               0               0               ]
    ///        [ Psi_S           Psi_D           Psi_Cor         ]
    ///        [ Psi_S_LambdaI   Psi_D_LambdaI   Psi_Cor_LambdaI ]
    /// where Psi_D is the matrix of the selected eigenvectors after DoModalReduction(), Psi_Cor is the static
    /// correction mode which depends on the external forces imposed on the internal bodies and nodes.
    const ChMatrixDynamic<>& GetModalReductionMatrix() const { return this->Psi; }

    /// Get the modal eigenvectors, if previously computed.
    /// These are the eigenvectors of the original assembly with applied boundary conditions, depending on reduction
    /// type. Use one of the ComputeModes() functions to set it.
    const ChMatrixDynamic<std::complex<double>>& GetEigenVectors() const { return this->m_modal_eigvect; }

    /// Get the modal eigenvalues, if previously computed.
    /// These are the eigenvalues of the original assembly with applied boundary conditions, depending on reduction
    /// type. Use one of the ComputeModes() functions to set it.
    const ChVectorDynamic<std::complex<double>>& GetEigenValues() const { return this->m_modal_eigvals; }

    /// Get a vector of (undamped) modal natural frequencies [Hz], if previously computed.
    /// These are the frequencies of the original assembly with applied boundary conditions, depending on reduction
    /// type. Use one of the ComputeModes() functions to set it.
    const ChVectorDynamic<double>& GetUndampedFrequencies() const { return this->m_modal_freq; }

    /// Get a vector of modal damping ratios = damping/critical_damping, if previously computed.
    /// Use one of the ComputeModes() functions to set it.
    const ChVectorDynamic<double>& GetDampingRatios() const { return this->m_modal_damping_ratios; }

    /// Get the deformed configuration of the full modal assembly.
    const ChVectorDynamic<>& GetDeformedState() const { return this->m_full_state_x; }

    /// Get the initial full state of the modal assembly before the modal reduction.
    const ChVectorDynamic<>& GetInitialState() const { return this->m_full_state_x0; }

    /// Get the vector of modal coordinates (positions).
    ChVectorDynamic<>& GetModalCoordinatesPosLevel() { return this->modal_q; }

    /// Get the vector of time derivative of modal coordinates (velocities).
    ChVectorDynamic<>& GetModalCoordinatesVelLevel() { return this->modal_q_dt; }

    /// Get the vector of 2nd time derivative of modal coordinates (accelerations).
    ChVectorDynamic<>& GetModalCoordinatesAccLevel() { return this->modal_q_dtdt; }

    // CONTAINER FUNCTIONS

    /// Removes all inserted items: bodies, links, etc., both boundary and internal.
    void Clear();

    /// Attach an internal body to this modal assembly.
    void AddInternalBody(std::shared_ptr<ChBody> body);

    /// Attach an internal link to this modal assembly.
    ///  - AddInternalLink(): to link "boundary nodes" with "internal nodes", or "internal nodes" with "internal nodes",
    ///  which will be reduced;
    ///  - AddLink(): to link "boundary nodes" with "boundary nodes", which will be computed and updated in the full
    ///  state.
    void AddInternalLink(std::shared_ptr<ChLinkBase> link);

    /// Attach an internal mesh to this modal assembly.
    void AddInternalMesh(std::shared_ptr<fea::ChMesh> mesh);

    /// Attach an internal ChPhysicsItem object that is not a body, link, or mesh.
    void AddInternalOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> item);

    /// Attach an arbitrary internal ChPhysicsItem (e.g. ChBody, ChParticles, ChLink, etc.) to the modal assembly.
    /// It will take care of adding it to the proper list of internal bodies, links, meshes, or generic
    /// physic item.
    void AddInternal(std::shared_ptr<ChPhysicsItem> item);

    /// Remove an internal body from this modal assembly.
    void RemoveInternalBody(std::shared_ptr<ChBody> body);
    /// Remove an internal link from this modal assembly.
    void RemoveInternalLink(std::shared_ptr<ChLinkBase> link);
    /// Remove an internal mesh from this modal assembly.
    void RemoveInternalMesh(std::shared_ptr<fea::ChMesh> mesh);
    /// Remove an internal ChPhysicsItem object that is not a body or a link
    void RemoveInternalOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> item);
    /// Remove an internal arbitrary ChPhysicsItem that was.
    void RemoveInternal(std::shared_ptr<ChPhysicsItem> item);

    /// Remove all internal bodies from this modal assembly.
    void RemoveAllInternalBodies();
    /// Remove all internal links from this modal assembly.
    void RemoveAllInternalLinks();
    /// Remove all meshes from this modal assembly.
    void RemoveAllInternalMeshes();
    /// Remove all physics items  not in the body, link, or mesh lists.
    void RemoveAllInternalOtherPhysicsItems();

    /// Get the list of both boundary and internal bodies.
    virtual const std::vector<std::shared_ptr<ChBody>>& GetBodies() const override;
    /// Get the list of both boundary and internal links.
    virtual const std::vector<std::shared_ptr<ChLinkBase>>& GetLinks() const override;
    /// Get the list of both boundary and internal meshes.
    virtual const std::vector<std::shared_ptr<fea::ChMesh>>& GetMeshes() const override;
    /// Get the list of both boundary and internal physics items that are not in the body or link lists.
    virtual const std::vector<std::shared_ptr<ChPhysicsItem>>& GetOtherPhysicsItems() const override;

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

    /// Get the number of modal coordinates. Use DoModalReduction() to change it.
    int GetNumCoordinatesModal() { return m_num_coords_modal; }

    // OTHER FUNCTIONS

    /// Write the mass (M), damping (K), damping (R), and constraint Jacobian (C) matrices at current configuration.
    /// Assumes the rows/columns of the matrices are ordered as the ChVariable objects used in this modal assembly,
    /// first all the "boundary" variables then all the "internal" variables (or modal variables if switched to modal
    /// reduced state). The sparse matrices are saved in COO format in [path]_M.dat [path]_K.dat [path]_R.dat, and
    /// [path]_Cq.dat. By default, uses 1-based indices (as in Matlab).
    void WriteSubassemblyMatrices(bool save_M,
                                  bool save_K,
                                  bool save_R,
                                  bool save_Cq,
                                  const std::string& path,
                                  bool one_indexed = true);

    /// Retrieves the stiffness, damping, mass and constraint Jacobian matrices of the modal assembly.
    /// All the matrices are consistent with the ordering of objects in the modal assembly; first the "boundary"
    /// variables, followed by either the internal variables (if in full state) or the modal variables (if in modal
    /// reduced state). Please not that not all loads provide a "stiffness" jacobian, as this is optional in their
    /// implementation i.e. -dF/dq is not available in the stiffness matrix for such objects
    void GetSubassemblyMatrices(ChSparseMatrix* K, ChSparseMatrix* R, ChSparseMatrix* M, ChSparseMatrix* Cq);

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
    virtual void Update(double time, bool update_assets) override;

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

    virtual void InjectVariables(ChSystemDescriptor& descriptor) override;

    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override;
    virtual void LoadConstraintJacobians() override;

    virtual void InjectKRMMatrices(ChSystemDescriptor& descriptor) override;
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

    /// Reset all timers.
    void ResetTimers() {
        m_timer_matrix_assembly.reset();
        m_timer_setup.reset();
        m_timer_modal_solver_call.reset();
    }

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
    /// Set the model as reduced. Cannot go back (from reduced to full state) for now.
    void FlagModelAsReduced();

    /// Initialize the modal assembly: 1.the initial undeformed configuration; 2.the floating frame F;
    void Initialize();

    /// Compute the undamped modes from M and K matrices. Used by DoModalReduction().
    template <typename EigensolverType>
    bool ComputeModesExternalData(const ChSparseMatrix& full_M,
                                  const ChSparseMatrix& full_K,
                                  const ChSparseMatrix& full_Cq,
                                  const ChModalSolverUndamped<EigensolverType>& modal_solver);

    /// Compute the mass property of the assembly.
    /// Used to initialize the floating frame F.
    void ComputeMassCenterFrame();

    /// Calculate the projection matrix.
    void ComputeProjectionMatrix();

    /// Update the floating frame of reference F
    void UpdateFloatingFrameOfReference();

    /// Update the transformation matrices used in the modal method.
    void UpdateTransformationMatrix();

    /// Recover the local M,K,Cq matrices, which are requried in the modal analysis.
    void ComputeLocalFullKMCqMatrices(ChSparseMatrix& full_M, ChSparseMatrix& full_K, ChSparseMatrix& full_Cq);

    /// Prepare sub-block matrices of M K R Cq in local frame of F
    void PartitionLocalSystemMatrices();

    /// Update the static correction mode if SetUseStaticCorrection(true)
    /// in case of external forces imposed on the internal bodies and nodes
    void UpdateStaticCorrectionMode();

    /// Compute the modal M,R,K,Cq matrices which are the tangent matrices used in the time stepper.
    void ComputeModalKRMmatricesGlobal(double Kfactor = 1.0, double Rfactor = 1.0, double Mfactor = 1.0);

    /// [INTERNAL USE ONLY]
    /// Both Herting and Craig-Bampton reductions are implemented in this function.
    void ApplyModeAccelerationTransformation(const ChModalDamping& damping_model = ChModalDampingNone());

    /// Computes the increment of the modal assembly (the increment of the current configuration respect
    /// to the initial "undeformed" configuration), and also gets the current speed.
    /// u_locred = P_W^T*[\delta qB; \delta eta]: corotated local displacement.
    /// e_locred = [qB^bar; eta]: local elastic displacement vector.
    /// edt_locred = [qB^bar_dt; eta_dt]: local elastic velocity vector.
    /// u_locred, e_locred and edt_locred are all expressed in the local floating frame of reference F.
    void GetLocalDeformations(ChVectorDynamic<>& u_locred, ChVectorDynamic<>& e_locred, ChVectorDynamic<>& edt_locred);

    // Internal use only. Compute P_W * H * P_W^T
    ChMatrixDynamic<> GetCorotationalTransformation(const ChMatrixDynamic<>& H);

    virtual void SetupInitial() override;

    /// Resize modal matrices and hook up the variables to the M K R block for the solver. To be used all times
    /// the n. of modes of modal reduction (m_num_coords_modal) is changed.
    void SetupModalData(unsigned int nmodes_reduction);

    // list of BOUNDARY items: [no data, just use the bodylist. linklist etc. in parent ChAssembly class.]

    // list of INTERNAL items:
    std::vector<std::shared_ptr<ChBody>> internal_bodylist;                 ///< list of rigid bodies
    std::vector<std::shared_ptr<ChLinkBase>> internal_linklist;             ///< list of joints (links)
    std::vector<std::shared_ptr<fea::ChMesh>> internal_meshlist;            ///< list of meshes
    std::vector<std::shared_ptr<ChPhysicsItem>> internal_otherphysicslist;  ///< list of other physics objects

    // dummy vectors to provide seamless behaviour in either full or reduced state
    mutable std::vector<std::shared_ptr<ChBody>> bodylist_total;                 ///< list of rigid bodies
    mutable std::vector<std::shared_ptr<ChLinkBase>> linklist_total;             ///< list of joints (links)
    mutable std::vector<std::shared_ptr<fea::ChMesh>> meshlist_total;            ///< list of meshes
    mutable std::vector<std::shared_ptr<ChPhysicsItem>> otherphysicslist_total;  ///< list of other physics objects

    // MODAL:
    ChVariablesGenericDiagonalMass* modal_variables;
    ChKRMBlock modal_Hblock;
    ChVectorDynamic<> modal_q;       ///< modal coordinates
    ChVectorDynamic<> modal_q_dt;    ///< modal velocites
    ChVectorDynamic<> modal_q_dtdt;  ///< modal accelerations

    ChMatrixDynamic<> modal_M;  // tangent mass matrix in the modal reduced state
    ChMatrixDynamic<> modal_K;  // tangent stiffness matrix in the modal reduced state
    ChMatrixDynamic<> modal_R;  // tangent damping matrix in the modal reduced state
    ChMatrixDynamic<> Psi;      ///< mode transformation matrix.
    ChMatrixDynamic<> Psi_S;    ///< static mode transformation matrix - corresponding to internal DOFs.
    ChMatrixDynamic<> Psi_D;    ///< dynamic mode transformation matrix - corresponding to internal DOFs.
    ChMatrixDynamic<> Psi_Cor;  ///< static correction mode - corresponding to internal DOFs.
    ChMatrixDynamic<>
        Psi_S_LambdaI;  ///< static mode transformation matrix - corresponding to internal Lagrange multipliers.
    ChMatrixDynamic<>
        Psi_D_LambdaI;  ///< dynamic mode transformation matrix - corresponding to internal Lagrange multipliers.
    ChMatrixDynamic<> Psi_Cor_LambdaI;  ///< static correction mode - corresponding to internal Lagrange multipliers.

    Eigen::SparseQR<ChSparseMatrix, Eigen::COLAMDOrdering<int>>
        m_solver_invKIIc;  // linear solver for K_IIc^{-1}

    // Results of eigenvalue analysis like ComputeModes() or ComputeModesDamped():
    ChMatrixDynamic<std::complex<double>> m_modal_eigvect;  // eigenvectors
    ChVectorDynamic<std::complex<double>> m_modal_eigvals;  // eigenvalues
    ChVectorDynamic<double> m_modal_freq;                   // frequencies
    ChVectorDynamic<double> m_modal_damping_ratios;         // damping ratios

    // floating frame of reference F
    bool is_initialized = false;
    ChFrameMoving<> cog_frame;          ///< center of mass frame of reference
    ChFrameMoving<> floating_frame_F0;  ///< floating frame of reference F in the initial undeformed configuration
    ChFrameMoving<> floating_frame_F;   ///< floating frame of reference F in the deformed configuration
    // ChFrameMoving<> floating_frame_F_old;
    ChVectorDynamic<> m_res_CF;  ///< residual of the six constraint equations on the floating frame F
    double m_tol_CF = 0;         // tolerance of the Newton-Raphson iteration in searching for the floating frame F

    ChState m_full_state_x0;  // full state snapshot of assembly in the initial undeformed configuration
    ChState m_full_state_x;   // full state snapshot of assembly in the deformed configuration

    // Projection matrices
    ChSparseMatrix U_locred;         // rigid body modes of the reduced modal assembly in the deformed configuration
    ChSparseMatrix U_locred_0;       // rigid body modes of the reduced modal assembly in the initial configuration
    ChMatrixDynamic<> Q_0;           // mapping matrix for the displacement of the floating frame F, is constant
    ChMatrixDynamic<> P_parallel_0;  // parallel projector, is constant
    ChMatrixDynamic<> P_perp_0;      // perpendicular projector, is constant
    bool is_projection_initialized = false;

    // rigid-body modes in local frame F
    ChSparseMatrix Uloc_B;  // rigid-body modes of boundary part in the deformed configuration
    ChSparseMatrix Uloc_I;  // rigid-body modes of internal part in the deformed configuration

    // Corotational transformation matrices
    ChMatrixDynamic<> P_F;  // rotation matrix for the floating frame F, = diag[R_F, I]

    ChVectorDynamic<> m_full_forces_internal;  ///< collect all external forces imposed on the internal nodes. This
                                               ///< force will be eventually transformed to the modal forces and applied
                                               ///< on the reduced modal assembly.

    // full system matrices in the local floating frame of reference F
    ChSparseMatrix full_M_loc;
    ChSparseMatrix full_K_loc;
    ChSparseMatrix full_R_loc;
    ChSparseMatrix full_Cq_loc;

    // the extended K M R matrices for modal assembly with constraints
    ChSparseMatrix full_K_loc_ext;  //=[Kloc, CqIloc^T; CqIloc, 0]
    ChSparseMatrix full_M_loc_ext;  //=[Mloc, 0; 0, 0]

    // reduced system matrices in the local floating frame of reference F
    ChMatrixDynamic<> M_red;  //=Psi^T * full_M_loc_ext * Psi
    ChMatrixDynamic<> K_red;  //=Psi^T * full_K_loc_ext * Psi
    ChMatrixDynamic<> R_red;  //=Psi^T * full_R_loc_ext * Psi

    // cache the partitioned M K block matrices for the sake of the computation of M_red, K_red
    ChSparseMatrix M_BB_loc;
    ChSparseMatrix M_BI_loc;
    ChSparseMatrix M_IB_loc;
    ChSparseMatrix M_II_loc;
    ChSparseMatrix K_BB_loc;
    ChSparseMatrix K_BI_loc;
    ChSparseMatrix K_IB_loc;
    ChSparseMatrix K_II_loc;
    // ChSparseMatrix R_II_loc;  // for test. todo: remove
    ChSparseMatrix Cq_IB_loc;
    ChSparseMatrix Cq_II_loc;
    ChSparseMatrix Cq_I_loc;

    ChSparseMatrix MBI_PsiST_MII;  // an intermediate matrix frequently used when SetUseStaticCorrection(true)
    ChMatrixDynamic<> PTKredP;  // an intermediate matrix, = P_perp_0^T * K_red * P_perp_0, which needs to be recomputed
                                // if m_num_coords_static_correction = true
    ChMatrixDynamic<> PTRredP;  // an intermediate matrix, = P_perp_0^T * R_red * P_perp_0, which might need to be
                                // recomputed if m_num_coords_static_correction = true

    ReductionType m_modal_reduction_type =
        ReductionType::CRAIG_BAMPTON;  ///< methods for modal reduction, Craig-Bampton as default

    bool m_is_model_reduced;  ///< flag to indicate whether in the modal "reduced" state.

    bool m_verbose = false;  ///< output m_verbose info

    bool m_internal_nodes_update;  ///< flag to indicate whether the internal nodes will update for
                                   ///< visualization/postprocessing

    bool m_modal_automatic_gravity;  ///< switch of the gravity load in modal reduced state

    bool m_use_linear_inertial_term = true;  // for internal test

    double m_scaling_factor_CqI =
        1.0;  // scaling factor on the internal part of Cq, to improve the numerical stability.

    // Statistics:

    // INTERNAL bodies, meshes etc. are NOT considered in equations of motion. These are
    // used anyway when computing modal analysis for component mode sysnthesis.
    unsigned int m_num_bodies_internal;             ///< number of internal bodies
    unsigned int m_num_links_internal;              ///< number of internal links
    unsigned int m_num_meshes_internal;             ///< number of internal meshes
    unsigned int m_num_otherphysicsitems_internal;  ///< number of internal other physics items

    unsigned int
        m_num_coords_pos_internal;  ///< number of scalar coordinates at position level for all active internal objects
    unsigned int
        m_num_coords_vel_internal;  ///< number of scalar coordinates at velocity level for all active internal objects

    unsigned int
        m_num_constr_internal;  ///< number of scalar constraints (velocity level), for all active internal objects
    unsigned int
        m_num_constr_bil_internal;  ///< number of bilateral scalar constraints (velocity level) of internal objects
    unsigned int
        m_num_constr_uni_internal;  ///< number of unilateral scalar constraints (velocity level) of internal objects

    // BOUNDARY bodies, meshes etc.: those of the parent class ChAssembly.
    unsigned int m_num_bodies_boundary;             ///< number of boundary bodies
    unsigned int m_num_links_boundary;              ///< number of boundary links
    unsigned int m_num_meshes_boundary;             ///< number of boundary meshes
    unsigned int m_num_otherphysicsitems_boundary;  ///< number of boundary other physics items

    unsigned int
        m_num_coords_pos_boundary;  ///< number of scalar coordinates at position level for all active boundary objects
    unsigned int
        m_num_coords_vel_boundary;  ///< number of scalar coordinates at velocity level for all active boundary objects

    unsigned int
        m_num_constr_boundary;  ///< number of scalar constraints (velocity level), for all active boundary objects
    unsigned int m_num_constr_bil_boundary;  ///< number of bilateral scalar constraints (velocity level) at boundary
    unsigned int m_num_constr_uni_boundary;  ///< number of unilateral scalar constraints (velocity level) at boundary

    // MODES: represent the motion of the modal assembly (internal, boundary nodes)
    unsigned int
        m_num_coords_modal;  ///< number of scalar coordinates at modal level (position and velocity level are the same)

    unsigned int m_num_coords_static_correction;  ///< number of the static correction mode. If
                                                  ///< SetUseStaticCorrection(true), =1; else =0.

    mutable ChTimer m_timer_matrix_assembly;
    mutable ChTimer m_timer_modal_solver_call;
    mutable ChTimer m_timer_setup;

    friend class ChSystem;
    friend class ChSystemMulticore;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename EigensolverType>
bool ChModalAssembly::ComputeModesExternalData(const ChSparseMatrix& full_M,
                                               const ChSparseMatrix& full_K,
                                               const ChSparseMatrix& full_Cq,
                                               const ChModalSolverUndamped<EigensolverType>& modal_solver) {
    // SOLVE EIGENVALUE
    // for undamped system
    // - Must work with large dimension and sparse matrices only
    // - Must work also in free-free cases, with 6 rigid body modes at 0 frequency.

    m_timer_modal_solver_call.start();
    modal_solver.Solve(full_K, full_M, full_Cq, m_modal_eigvect, m_modal_eigvals, m_modal_freq);
    m_timer_modal_solver_call.stop();

    return true;
}

template <typename EigensolverType>
void ChModalAssembly::DoModalReduction(const ChModalSolverUndamped<EigensolverType>& modal_solver,
                                       const ChModalDamping& damping_model) {
    if (m_is_model_reduced)
        return;

    // 1) fetch the full (not reduced) mass and stiffness
    ChSparseMatrix full_K, full_M, full_Cq;
    GetSubassemblyMatrices(&full_K, nullptr, &full_M, &full_Cq);

    // 2) compute modal reduction from full_M, full_K
    this->DoModalReduction(full_M, full_K, full_Cq, modal_solver, damping_model);
}

template <typename EigensolverType>
void ChModalAssembly::DoModalReduction(ChSparseMatrix& full_M,
                                       ChSparseMatrix& full_K,
                                       ChSparseMatrix& full_Cq,
                                       const ChModalSolverUndamped<EigensolverType>& modal_solver,
                                       const ChModalDamping& damping_model) {
    if (m_is_model_reduced)
        return;

    SetupInitial();
    Setup();
    Update(ChTime, true);

    Initialize();

    // in modal reduced state, m_modal_automatic_gravity overwrites the gravity settings for both boundary and internal
    // meshes.
    if (m_modal_automatic_gravity) {
        // Note: the gravity on boundary bodies (ChBody) cannot turn off via SetAutomaticGravity()!
        for (auto& mesh_boundary : meshlist)
            mesh_boundary->SetAutomaticGravity(false);
        for (auto& mesh_internal : internal_meshlist)
            mesh_internal->SetAutomaticGravity(false);
    }

    // recover the local M,K,Cq (full_M_loc, full_K_loc, full_Cq_loc) matrices
    // through rotating back to the local frame of F
    ComputeLocalFullKMCqMatrices(full_M, full_K, full_Cq);

    // prepare sub-block matrices of M K R
    PartitionLocalSystemMatrices();

    //// start of modal reduction transformation
    // 1) compute eigenvalue and eigenvectors
    if (m_modal_reduction_type == ReductionType::HERTING) {
        if (modal_solver.GetNumRequestedModes() < 6) {
            std::cout << "*** At least six rigid-body modes are required for the HERTING modal reduction. "
                      << "The default settings are used." << std::endl;
            throw std::runtime_error("Herting reduction method requires at least six rigid-body modes.");
        }

        ComputeModesExternalData(full_M_loc, full_K_loc, full_Cq_loc, modal_solver);

    } else if (m_modal_reduction_type == ReductionType::CRAIG_BAMPTON) {
        ComputeModesExternalData(M_II_loc, K_II_loc, Cq_II_loc, modal_solver);

    } else {
        std::cerr << "Wrong modal reduction method requested." << std::endl;
        throw std::invalid_argument("Wrong modal reduction method requested.");
    }

    if (m_verbose) {
        if (m_modal_reduction_type == ReductionType::HERTING)
            std::cout << "*** Herting reduction is used." << std::endl;
        else
            std::cout << "*** Craig-Bamption reduction is used." << std::endl;

        for (unsigned int i = 0; i < m_modal_eigvals.size(); ++i)
            std::cout << " Undamped mode n." << i + 1 << "  Frequency [Hz]: " << m_modal_freq(i) << std::endl;
    }

    // 2) bound ChVariables etc. to the modal coordinates, resize matrices, set as modal mode
    FlagModelAsReduced();
    SetupModalData(m_modal_eigvect.cols());

    // 3) compute the transforamtion matrices, also the local rigid-body modes
    UpdateTransformationMatrix();

    // 4) do the Mode Acceleration reduction as in Sonneville2021
    ApplyModeAccelerationTransformation(damping_model);
    //// end of modal reduction transformation

    // initialize the projection matrices
    ComputeProjectionMatrix();

    // initialize the modal K R M matrices
    ComputeModalKRMmatricesGlobal();

    // Debug dump data. ***TODO*** remove
    if (m_verbose) {
        std::ofstream filePsi("dump_modal_Psi.dat");
        filePsi << std::setprecision(12) << std::scientific;
        StreamOut(Psi, filePsi);
        std::ofstream fileM("dump_modal_M.dat");
        fileM << std::setprecision(12) << std::scientific;
        StreamOut(modal_M, fileM);
        std::ofstream fileK("dump_modal_K.dat");
        fileK << std::setprecision(12) << std::scientific;
        StreamOut(modal_K, fileK);
        std::ofstream fileR("dump_modal_R.dat");
        fileR << std::setprecision(12) << std::scientific;
        StreamOut(modal_R, fileR);

        std::ofstream fileM_red("dump_reduced_M.dat");
        fileM_red << std::setprecision(12) << std::scientific;
        StreamOut(M_red, fileM_red);
        std::ofstream fileK_red("dump_reduced_K.dat");
        fileK_red << std::setprecision(12) << std::scientific;
        StreamOut(K_red, fileK_red);
        std::ofstream fileR_red("dump_reduced_R.dat");
        fileR_red << std::setprecision(12) << std::scientific;
        StreamOut(R_red, fileR_red);
    }
}

/// @} modal

}  // end namespace modal

CH_CLASS_VERSION(modal::ChModalAssembly, 0)

}  // end namespace chrono

#endif
