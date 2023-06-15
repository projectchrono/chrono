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


#include "chrono_modal/ChModalAssembly.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

namespace chrono {

using namespace fea;
using namespace collision;
using namespace geometry;

namespace modal {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChModalAssembly)

ChModalAssembly::ChModalAssembly()
    : modal_variables(nullptr),
    n_modes_coords_w(0),
    is_modal(false),
    internal_nodes_update(true)
{}

ChModalAssembly::ChModalAssembly(const ChModalAssembly& other) : ChAssembly(other) {
    
    is_modal = other.is_modal;
    modal_q = other.modal_q;
    modal_q_dt = other.modal_q_dt;
    modal_q_dtdt = other.modal_q_dtdt;
    custom_F_modal = other.custom_F_modal;
    internal_nodes_update = other.internal_nodes_update;
    m_custom_F_modal_callback = other.m_custom_F_modal_callback;
    m_custom_F_full_callback = other.m_custom_F_full_callback;

    //// TODO:  deep copy of the object lists (internal_bodylist, internal_linklist, internal_meshlist,  internal_otherphysicslist)
}

ChModalAssembly::~ChModalAssembly() {
    RemoveAllInternalBodies();
    RemoveAllInternalLinks();
    RemoveAllInternalMeshes();
    RemoveAllInternalOtherPhysicsItems();
    if (modal_variables) 
        delete modal_variables;
}

ChModalAssembly& ChModalAssembly::operator=(ChModalAssembly other) {
    ChModalAssembly tmp(other);
    swap(*this, other);
    return *this;
}

// Note: implement this as a friend function (instead of a member function swap(ChModalAssembly& other)) so that other
// classes that have a ChModalAssembly member (currently only ChSystem) could use it, the same way we use std::swap here.
void swap(ChModalAssembly& first, ChModalAssembly& second) {
    using std::swap;
    // swap(first.nbodies, second.nbodies);
    // ***TODO***
}

void ChModalAssembly::Clear() {
    ChAssembly::Clear(); // parent 

    RemoveAllInternalBodies();
    RemoveAllInternalLinks();
    RemoveAllInternalMeshes();
    RemoveAllInternalOtherPhysicsItems();

    if (modal_variables) 
        delete modal_variables;
}


// Assembly a sparse matrix by bordering square H with rectangular Cq.
//    HCQ = [ H  Cq' ]
//          [ Cq  0  ]
void util_sparse_assembly_2x2symm(Eigen::SparseMatrix<double, Eigen::ColMajor, int>& HCQ,       ///< resulting square sparse matrix (column major)
	const ChSparseMatrix& H,   ///< square sparse H matrix, n_v x n_v
	const ChSparseMatrix& Cq)  ///< rectangular  sparse Cq  n_c x n_v
{
	int n_v   = H.rows();
	int n_c   = Cq.rows();
	HCQ.resize(n_v + n_c, n_v + n_c);
	HCQ.reserve(H.nonZeros() + 2 * Cq.nonZeros());
    HCQ.setZero();

	for (int k=0; k<H.outerSize(); ++k)
		for (ChSparseMatrix::InnerIterator it(H,k); it; ++it) {
			HCQ.insert(it.row(),it.col()) = it.value();
        }

	for (int k=0; k<Cq.outerSize(); ++k)
        for (ChSparseMatrix::InnerIterator it(Cq, k); it; ++it) {
            HCQ.insert(it.row() + n_v, it.col()) = it.value(); // insert Cq
            HCQ.insert(it.col(), it.row() + n_v) = it.value(); // insert Cq'
        }

    // This seems necessary in Release mode
    HCQ.makeCompressed();

    //***NOTE*** 
    // for some reason the HCQ matrix created via .insert() or .elementRef() or triplet insert, is 
    // corrupt in Release mode, not in Debug mode. However, when doing a loop like the one below,
    // it repairs it. 
    // ***TODO*** avoid this bad hack and find the cause of the release/debug difference.
    /*
    for (int k = 0; k < HCQ.rows(); ++k) {
        for (int j = 0; j < HCQ.cols(); ++j) {
            auto foo = HCQ.coeffRef(k, j);
            //GetLog() << HCQ.coeffRef(k,j) << " ";
        }
    }
    */
}



//---------------------------------------------------------------------------------------

void ChModalAssembly::SwitchModalReductionON(ChSparseMatrix& full_M, ChSparseMatrix& full_K, ChSparseMatrix& full_Cq, 
    const ChModalSolveUndamped& n_modes_settings, 
    const ChModalDamping& damping_model
) {
    if (is_modal)
        return;


    // 1) compute eigenvalue and eigenvectors
    this->ComputeModesExternalData(full_M, full_K, full_Cq, n_modes_settings);


    // 2) fetch initial x0 state of assembly, full not reduced
    int bou_int_coords   = this->n_boundary_coords   + this->n_internal_coords;
    int bou_int_coords_w = this->n_boundary_coords_w   + this->n_internal_coords_w;
    double fooT;
    ChStateDelta assembly_v0;
    assembly_x0.setZero(bou_int_coords, nullptr);
    assembly_v0.setZero(bou_int_coords_w, nullptr);
    this->IntStateGather(0, assembly_x0, 0, assembly_v0, fooT);


    // 3) bound ChVariables etc. to the modal coordinates, resize matrices, set as modal mode
    this->SetModalMode(true);
    this->SetupModalData(this->modes_V.cols());



    // 4) do the Herting reduction as in Sonneville, 2021

    ChSparseMatrix K_II = full_K.block(this->n_boundary_coords_w, this->n_boundary_coords_w, this->n_internal_coords_w, this->n_internal_coords_w);
    ChSparseMatrix K_IB = full_K.block(this->n_boundary_coords_w, 0,                         this->n_internal_coords_w, this->n_boundary_coords_w);

    ChSparseMatrix M_II = full_M.block(this->n_boundary_coords_w, this->n_boundary_coords_w, this->n_internal_coords_w, this->n_internal_coords_w);
    ChSparseMatrix M_IB = full_M.block(this->n_boundary_coords_w, 0,                         this->n_internal_coords_w, this->n_boundary_coords_w);

    ChSparseMatrix Cq_B = full_Cq.block(0,                         0,                full_Cq.rows(), this->n_boundary_coords_w);
    ChSparseMatrix Cq_I = full_Cq.block(0, this->n_boundary_coords_w,                full_Cq.rows(), this->n_internal_coords_w);

    ChMatrixDynamic<> V_B = this->modes_V.block(0                        , 0,                this->n_boundary_coords_w, this->n_modes_coords_w).real();
    ChMatrixDynamic<> V_I = this->modes_V.block(this->n_boundary_coords_w, 0,                this->n_internal_coords_w, this->n_modes_coords_w).real();

    // K_IIc = [ K_II   Cq_I' ]
    //         [ Cq_I     0   ]

    Eigen::SparseMatrix<double> K_IIc;
    util_sparse_assembly_2x2symm(K_IIc, K_II, Cq_I);
    K_IIc.makeCompressed();

    // Matrix of static modes (constrained, so use K_IIc instead of K_II,
    // the original unconstrained Herting reduction is Psi_S = - K_II^{-1} * K_IB )
    //
    // {Psi_S; foo} = - K_IIc^{-1} * {K_IB ; Cq_B}
    
    ChMatrixDynamic<> Psi_S(this->n_internal_coords_w, this->n_boundary_coords_w);

    // avoid computing K_IIc^{-1}, effectively do n times a linear solve:
    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int> >   solver;
    solver.analyzePattern(K_IIc);
    solver.factorize(K_IIc); 
    for (int i = 0; i < K_IB.cols(); ++i) { 
        ChVectorDynamic<> rhs(this->n_internal_coords_w + full_Cq.rows());
        if (Cq_B.rows())
            rhs << K_IB.col(i).toDense(), Cq_B.col(i).toDense();
        else
            rhs << K_IB.col(i).toDense();
        ChVectorDynamic<> x = solver.solve(rhs);
        Psi_S.block(0,i, this->n_internal_coords_w, 1) = -x.head(this->n_internal_coords_w);
    }

    // Matrix of dynamic modes (V_B and V_I already computed as constrained eigenmodes, 
    // but use K_IIc instead of K_II anyway, to reuse K_IIc already factored before)
    //
    // {Psi_D; foo} = - K_IIc^{-1} * {(M_IB * V_B + M_II * V_I) ; 0}

    ChMatrixDynamic<> Psi_D(this->n_internal_coords_w, this->n_modes_coords_w);

    for (int i = 0; i < this->n_modes_coords_w; ++i) { 
        ChVectorDynamic<> rhs(this->n_internal_coords_w + full_Cq.rows());
        rhs << (M_IB * V_B + M_II * V_I).col(i) , Eigen::VectorXd::Zero(full_Cq.rows()) ;
        ChVectorDynamic<> x = solver.solve(rhs);
        Psi_D.block(0,i, this->n_internal_coords_w, 1) = -x.head(this->n_internal_coords_w);
    }



    // Psi = [ I     0    ]
    //       [Psi_S  Psi_D]
    Psi.setZero(this->n_boundary_coords_w + this->n_internal_coords_w, this->n_boundary_coords_w + this->n_modes_coords_w);
    //***TODO*** maybe prefer sparse Psi matrix, especially for upper blocks...

    Psi << Eigen::MatrixXd::Identity(n_boundary_coords_w, n_boundary_coords_w), Eigen::MatrixXd::Zero(n_boundary_coords_w, n_modes_coords_w),
           Psi_S,                                                               Psi_D;

    // Modal reduction of the M K matrices
    this->modal_M = Psi.transpose() * full_M * Psi;
    this->modal_K = Psi.transpose() * full_K * Psi;

    this->modal_R.setZero(modal_M.rows(), modal_M.cols()); // default R=0 , zero damping
    
    // Modal reduction of R damping matrix: compute using user-provided damping model 
    damping_model.ComputeR(*this, this->modal_M, this->modal_K, Psi, this->modal_R);


    // Reset to zero all the atomic masses of the boundary nodes because now their mass is represented by  this->modal_M
    // NOTE! this should be made more generic and future-proof by implementing a virtual method ex. RemoveMass() in all ChPhysicsItem 
    for (auto& body : bodylist) {
            body->SetMass(0);
            body->SetInertia(VNULL);
    }
    for (auto& item : this->meshlist) {
        if (auto mesh = std::dynamic_pointer_cast<ChMesh>(item)) {
            for (auto& node : mesh->GetNodes()) {
                if (auto xyz = std::dynamic_pointer_cast<ChNodeFEAxyz>(node))
                    xyz->SetMass(0);
                if (auto xyzrot = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(node)) {
                    xyzrot->SetMass(0);
                    xyzrot->GetInertia().setZero();
                }
            }
        }
    }
    
    // Invalidate results of the initial eigenvalue analysis because now the DOFs are different after reduction,
    // to avoid that one could be tempted to plot those eigenmodes, which now are not exactly the ones of the reduced assembly.
    this->modes_assembly_x0.resize(0);
    this->modes_damping_ratio.resize(0);
    this->modes_eig.resize(0);
    this->modes_freq.resize(0);
    this->modes_V.resize(0, 0);

    // Debug dump data. ***TODO*** remove
    if (true) {
        ChStreamOutAsciiFile fileP("dump_modal_Psi.dat");
        fileP.SetNumFormat("%.12g");
        StreamOutDenseMatlabFormat(Psi, fileP);
        ChStreamOutAsciiFile fileM("dump_modal_M.dat");
        fileM.SetNumFormat("%.12g");
        StreamOutDenseMatlabFormat(this->modal_M, fileM);
        ChStreamOutAsciiFile fileK("dump_modal_K.dat");
        fileK.SetNumFormat("%.12g");
        StreamOutDenseMatlabFormat(this->modal_K, fileK);
        ChStreamOutAsciiFile fileR("dump_modal_R.dat");
        fileR.SetNumFormat("%.12g");
        StreamOutDenseMatlabFormat(this->modal_R, fileR);
    }
}

void ChModalAssembly::SwitchModalReductionON(
    const ChModalSolveUndamped& n_modes_settings, 
    const ChModalDamping& damping_model
) {
    if (is_modal)
        return;

    // 1) fetch the full (not reduced) mass and stiffness
    ChSparseMatrix full_M;
    ChSparseMatrix full_K;
    ChSparseMatrix full_Cq;

    this->GetSubassemblyMassMatrix(&full_M);
    this->GetSubassemblyStiffnessMatrix(&full_K);
    this->GetSubassemblyConstraintJacobianMatrix(&full_Cq);

    // 2) compute modal reduction from full_M, full_K
    this->SwitchModalReductionON(full_M, full_K, full_Cq, n_modes_settings, damping_model);
}


void ChModalAssembly::SetupModalData(int nmodes_reduction) {

    this->n_modes_coords_w = nmodes_reduction;

    if (!modal_variables || (modal_variables->Get_ndof() != this->n_modes_coords_w)) {

        // Initialize ChVariable object used for modal variables
        if (modal_variables)
            delete modal_variables;
        modal_variables = new ChVariablesGenericDiagonalMass(this->n_modes_coords_w);
        modal_variables->GetMassDiagonal().setZero(); // diag. mass not needed, the mass will be defined via this->modal_Hblock 

        // Initialize the modal_Hblock, which is a ChKblockGeneric referencing all ChVariable items:
        std::vector<ChVariables*> mvars;
        // - for BOUNDARY variables: trick to collect all ChVariable references..
        ChSystemDescriptor temporary_descriptor;
        for (auto& body : bodylist)
            body->InjectVariables(temporary_descriptor);
        for (auto& link : linklist)
            link->InjectVariables(temporary_descriptor);
        for (auto& mesh : meshlist)
            mesh->InjectVariables(temporary_descriptor);
        for (auto& item : otherphysicslist)
            item->InjectVariables(temporary_descriptor);
        mvars = temporary_descriptor.GetVariablesList();
        // - for the MODAL variables:
        mvars.push_back(this->modal_variables);

        // NOTE! Purge the not active variables, so that there is a  1-to-1 mapping
        // between the assembly's matrices this->modal_M, modal_K, modal_R and the modal_Hblock->Get_K() block.
        // In fact the ChKblockGeneric modal_Hblock could also handle the not active vars, but the modal_M, K etc are 
        // computed for the active-only variables for simplicity in the Herting transformation.
        std::vector<ChVariables*> mvars_active;
        for (auto mvar : mvars) {
            if (mvar->IsActive())
                mvars_active.push_back(mvar);
        }

        this->modal_Hblock.SetVariables(mvars_active);

        // Initialize vectors to be used with modal coordinates:
        this->modal_q.setZero(this->n_modes_coords_w);
        this->modal_q_dt.setZero(this->n_modes_coords_w);
        this->modal_q_dtdt.setZero(this->n_modes_coords_w);
        this->custom_F_modal.setZero(this->n_modes_coords_w);
        this->custom_F_full.setZero(this->n_boundary_coords_w + this->n_internal_coords_w );
    }
}

bool ChModalAssembly::ComputeModes(const ChModalSolveUndamped& n_modes_settings) {

    m_timer_matrix_assembly.start();
    ChSparseMatrix full_M;
    ChSparseMatrix full_K;
    ChSparseMatrix full_Cq;

    this->GetSubassemblyMassMatrix(&full_M);
    this->GetSubassemblyStiffnessMatrix(&full_K);
    this->GetSubassemblyConstraintJacobianMatrix(&full_Cq);

    m_timer_matrix_assembly.stop();


    // SOLVE EIGENVALUE
    this->ComputeModesExternalData(full_M, full_K, full_Cq, n_modes_settings);

    return true;
}

bool ChModalAssembly::ComputeModesExternalData(ChSparseMatrix& full_M, ChSparseMatrix& full_K, ChSparseMatrix& full_Cq, const ChModalSolveUndamped& n_modes_settings) {

    m_timer_setup.start();
    this->SetupInitial();
    this->Setup();
    this->Update();

    // fetch the state snapshot for this analysis
    double fooT;
    ChStateDelta modes_assembly_v0;
    modes_assembly_x0.setZero(this->ncoords, nullptr);
    modes_assembly_v0.setZero(this->ncoords_w, nullptr);
    this->IntStateGather(0, modes_assembly_x0, 0, modes_assembly_v0, fooT);

    // cannot use more modes than n. of tot coords, if so, clamp
    //int nmodes_clamped = ChMin(nmodes, this->ncoords_w);

    this->Setup();

    assert(full_M.rows()  == this->ncoords_w); 
    assert(full_K.rows()  == this->ncoords_w); 
    assert(full_Cq.cols() == this->ncoords_w); 

    m_timer_setup.stop();

    // SOLVE EIGENVALUE 
    // for undamped system (use generalized constrained eigen solver)
    // - Must work with large dimension and sparse matrices only
    // - Must work also in free-free cases, with 6 rigid body modes at 0 frequency.
    m_timer_modal_solver_call.start();
    n_modes_settings.Solve(full_M, full_K, full_Cq, this->modes_V, this->modes_eig, this->modes_freq);
    m_timer_modal_solver_call.stop();

    m_timer_setup.start();

    this->modes_damping_ratio.setZero(this->modes_freq.rows());

    this->Setup();

    m_timer_setup.stop();


    return true;
}

bool ChModalAssembly::ComputeModesDamped(const ChModalSolveDamped& n_modes_settings) {

    m_timer_setup.start();

    this->SetupInitial();
    this->Setup();
    this->Update();

    // fetch the state snapshot of this analysis
    double fooT;
    ChStateDelta modes_assembly_v0;
    modes_assembly_x0.setZero(this->ncoords, nullptr);
    modes_assembly_v0.setZero(this->ncoords_w, nullptr);
    this->IntStateGather(0, modes_assembly_x0, 0, modes_assembly_v0, fooT);

    this->Setup();

    m_timer_setup.stop();

    m_timer_matrix_assembly.start();

    ChSparseMatrix full_M;
    ChSparseMatrix full_R;
    ChSparseMatrix full_K;
    ChSparseMatrix full_Cq;

    this->GetSubassemblyMassMatrix(&full_M);
    this->GetSubassemblyDampingMatrix(&full_R);
    this->GetSubassemblyStiffnessMatrix(&full_K);
    this->GetSubassemblyConstraintJacobianMatrix(&full_Cq);

    m_timer_matrix_assembly.stop();


    // SOLVE QUADRATIC EIGENVALUE 
    // for damped system (use quadratic constrained eigen solver)
    // - Must work with large dimension and sparse matrices only
    // - Must work also in free-free cases, with 6 rigid body modes at 0 frequency.
    m_timer_modal_solver_call.start();
    n_modes_settings.Solve(full_M, full_R, full_K, full_Cq, this->modes_V, this->modes_eig, this->modes_freq, this->modes_damping_ratio);
    m_timer_modal_solver_call.stop();

    m_timer_setup.start();
    this->Setup();
    m_timer_setup.stop();

    return true;
}



void ChModalAssembly::SetFullStateWithModeOverlay(int n_mode, double phase, double amplitude) {

    if (n_mode >= this->modes_V.cols()) {
        this->Update();
        throw ChException("Error: mode " + std::to_string(n_mode) + " is beyond the " + std::to_string(this->modes_V.cols()) + " computed eigenvectors.");
    }

    if (this->modes_V.rows() != this->ncoords_w) {
        this->Update();
        return;
    }

    double fooT=0;
    ChState assembly_x_new;
    ChStateDelta assembly_v;
    ChStateDelta assembly_Dx;
    
    assembly_x_new.setZero(this->ncoords, nullptr);
    assembly_v.setZero(this->ncoords_w, nullptr);
    assembly_Dx.setZero(this->ncoords_w, nullptr);
    
    // pick the nth eigenvector
    assembly_Dx = sin(phase) * amplitude * this->modes_V.col(n_mode).real() +cos(phase) * amplitude * this->modes_V.col(n_mode).imag();
    
    this->IntStateIncrement(0, assembly_x_new, this->modes_assembly_x0, 0, assembly_Dx); // x += amplitude * eigenvector

    this->IntStateScatter(0, assembly_x_new, 0, assembly_v, fooT, true);

    this->Update();
}


void ChModalAssembly::SetInternalStateWithModes(bool full_update) {
   
    if (!this->is_modal)
        return;

    int bou_int_coords   = this->n_boundary_coords   + this->n_internal_coords;
    int bou_int_coords_w = this->n_boundary_coords_w + this->n_internal_coords_w;
    int bou_mod_coords_w = this->n_boundary_coords_w + this->n_modes_coords_w;
    
    if (this->Psi.rows() != bou_int_coords_w || 
        this->Psi.cols() != bou_mod_coords_w)
        return;

    // Fetch current dx state (e reduced)
    ChStateDelta assembly_Dx_reduced;
    ChStateDelta assembly_v_reduced;
    assembly_Dx_reduced.setZero(bou_mod_coords_w, nullptr);
    assembly_v_reduced.setZero (bou_mod_coords_w, nullptr);
    this->GetStateLocal(assembly_Dx_reduced,assembly_v_reduced);


    bool needs_temporary_bou_int = this->is_modal;
    if (needs_temporary_bou_int) 
        this->is_modal = false; // to have IntStateIncrement IntStateScatter referencing both boundary AND INTERNAL items
    


    ChState assembly_x_new;
    ChStateDelta assembly_v;
    ChStateDelta assembly_Dx;
    
    assembly_x_new.setZero(bou_int_coords, nullptr);
    assembly_v.setZero(bou_int_coords_w, nullptr);
    assembly_Dx.setZero(bou_int_coords_w, nullptr);
    
    // compute dx = Psi * dx_reduced  
    assembly_Dx = this->Psi * assembly_Dx_reduced;
    
    this->IntStateIncrement(0, assembly_x_new, this->assembly_x0, 0, assembly_Dx); 

    // scatter to internal nodes only and update them 
    unsigned int displ_x = 0 - this->offset_x;
    unsigned int displ_v = 0 - this->offset_w;
    double T = this->GetChTime();
    for (auto& body : internal_bodylist) {
        if (body->IsActive())
            body->IntStateScatter(displ_x + body->GetOffset_x(), assembly_x_new, displ_v + body->GetOffset_w(), assembly_v, T, full_update);
        else
            body->Update(T, full_update);
    }
    for (auto& mesh : internal_meshlist) {
        mesh->IntStateScatter(displ_x + mesh->GetOffset_x(), assembly_x_new, displ_v + mesh->GetOffset_w(), assembly_v, T, full_update);
    }
    for (auto& link : internal_linklist) {
        if (link->IsActive())
            link->IntStateScatter(displ_x + link->GetOffset_x(), assembly_x_new, displ_v + link->GetOffset_w(), assembly_v, T, full_update);
        else
            link->Update(T, full_update);
    }
    for (auto& item : internal_otherphysicslist) {
        item->IntStateScatter(displ_x + item->GetOffset_x(), assembly_x_new, displ_v + item->GetOffset_w(), assembly_v, T, full_update);
    }


    if (needs_temporary_bou_int) 
        this->is_modal = true;
}


void ChModalAssembly::SetFullStateReset() {
   
    
    if (this->modes_assembly_x0.rows() != this->ncoords)
        return;

    double fooT=0;
    ChStateDelta assembly_v;
 
    assembly_v.setZero(this->ncoords_w, nullptr);

    this->IntStateScatter(0, this->modes_assembly_x0, 0, assembly_v, fooT, true);

    this->Update();

}


void ChModalAssembly::SetInternalNodesUpdate(bool mflag) {
        this->internal_nodes_update = mflag;
}


//---------------------------------------------------------------------------------------

// Note: removing items from the assembly incurs linear time cost

void ChModalAssembly::AddInternalBody(std::shared_ptr<ChBody> body) {
    assert(std::find(std::begin(internal_bodylist), std::end(internal_bodylist), body) == internal_bodylist.end());
    assert(body->GetSystem() == nullptr);  // should remove from other system before adding here

    // set system and also add collision models to system
    body->SetSystem(system);
    internal_bodylist.push_back(body);

	////system->is_initialized = false;  // Not needed, unless/until ChBody::SetupInitial does something
	system->is_updated = false;
}

void ChModalAssembly::RemoveInternalBody(std::shared_ptr<ChBody> body) {
    auto itr = std::find(std::begin(internal_bodylist), std::end(internal_bodylist), body);
    assert(itr != internal_bodylist.end());

    internal_bodylist.erase(itr);
    body->SetSystem(nullptr);

    system->is_updated = false;
}

void ChModalAssembly::AddInternalLink(std::shared_ptr<ChLinkBase> link) {
    assert(std::find(std::begin(internal_linklist), std::end(internal_linklist), link) == internal_linklist.end());

    link->SetSystem(system);
    internal_linklist.push_back(link);

	////system->is_initialized = false;  // Not needed, unless/until ChLink::SetupInitial does something
    system->is_updated = false;
}

void ChModalAssembly::RemoveInternalLink(std::shared_ptr<ChLinkBase> link) {
    auto itr = std::find(std::begin(internal_linklist), std::end(internal_linklist), link);
    assert(itr != internal_linklist.end());

    internal_linklist.erase(itr);
    link->SetSystem(nullptr);

    system->is_updated = false;
}

void ChModalAssembly::AddInternalMesh(std::shared_ptr<fea::ChMesh> mesh) {
    assert(std::find(std::begin(internal_meshlist), std::end(internal_meshlist), mesh) == internal_meshlist.end());

    mesh->SetSystem(system);
    internal_meshlist.push_back(mesh);

	system->is_initialized = false;
    system->is_updated = false;
}

void ChModalAssembly::RemoveInternalMesh(std::shared_ptr<fea::ChMesh> mesh) {
    auto itr = std::find(std::begin(internal_meshlist), std::end(internal_meshlist), mesh);
    assert(itr != internal_meshlist.end());

    internal_meshlist.erase(itr);
    mesh->SetSystem(nullptr);

    system->is_updated = false;
}

void ChModalAssembly::AddInternalOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> item) {
    assert(!std::dynamic_pointer_cast<ChBody>(item));
    assert(!std::dynamic_pointer_cast<ChLinkBase>(item));
    assert(!std::dynamic_pointer_cast<ChMesh>(item));
    assert(std::find(std::begin(internal_otherphysicslist), std::end(internal_otherphysicslist), item) == internal_otherphysicslist.end());
    // assert(item->GetSystem()==nullptr); // should remove from other system before adding here

    // set system and also add collision models to system
    item->SetSystem(system);
    internal_otherphysicslist.push_back(item);

	////system->is_initialized = false;  // Not needed, unless/until ChPhysicsItem::SetupInitial does something
    system->is_updated = false;
}

void ChModalAssembly::RemoveInternalOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> item) {
    auto itr = std::find(std::begin(internal_otherphysicslist), std::end(internal_otherphysicslist), item);
    assert(itr != internal_otherphysicslist.end());

    internal_otherphysicslist.erase(itr);
    item->SetSystem(nullptr);

    system->is_updated = false;
}

void ChModalAssembly::AddInternal(std::shared_ptr<ChPhysicsItem> item) {
    if (auto body = std::dynamic_pointer_cast<ChBody>(item)) {
        AddInternalBody(body);
        return;
    }

    if (auto link = std::dynamic_pointer_cast<ChLinkBase>(item)) {
        AddInternalLink(link);
        return;
    }

    if (auto mesh = std::dynamic_pointer_cast<fea::ChMesh>(item)) {
        AddInternalMesh(mesh);
        return;
    }

    AddInternalOtherPhysicsItem(item);
}


void ChModalAssembly::RemoveInternal(std::shared_ptr<ChPhysicsItem> item) {
    if (auto body = std::dynamic_pointer_cast<ChBody>(item)) {
        RemoveInternalBody(body);
        return;
    }

    if (auto link = std::dynamic_pointer_cast<ChLinkBase>(item)) {
        RemoveInternalLink(link);
        return;
    }

    if (auto mesh = std::dynamic_pointer_cast<fea::ChMesh>(item)) {
        RemoveInternalMesh(mesh);
        return;
    }

    RemoveInternalOtherPhysicsItem(item);
}

void ChModalAssembly::RemoveAllInternalBodies() {
    for (auto& body : internal_bodylist) {
        body->SetSystem(nullptr);
    }
    internal_bodylist.clear();

    if (system)
        system->is_updated = false;
}

void ChModalAssembly::RemoveAllInternalLinks() {
    for (auto& link : internal_linklist) {
        link->SetSystem(nullptr);
    }
    internal_linklist.clear();

    if (system)
        system->is_updated = false;
}

void ChModalAssembly::RemoveAllInternalMeshes() {
    for (auto& mesh : internal_meshlist) {
        mesh->SetSystem(nullptr);
    }
    internal_meshlist.clear();

    if (system)
        system->is_updated = false;
}

void ChModalAssembly::RemoveAllInternalOtherPhysicsItems() {
    for (auto& item : internal_otherphysicslist) {
        item->SetSystem(nullptr);
    }
    internal_otherphysicslist.clear();

    if (system)
        system->is_updated = false;
}


// -----------------------------------------------------------------------------


void ChModalAssembly::GetSubassemblyMassMatrix(ChSparseMatrix* M) {

    this->SetupInitial();
    this->Setup();
    this->Update();

    ChSystemDescriptor temp_descriptor;

    this->InjectVariables(temp_descriptor);
    this->InjectKRMmatrices(temp_descriptor);
    this->InjectConstraints(temp_descriptor);

    // Load all KRM matrices with the M part only
    KRMmatricesLoad(0, 0, 1.0);
    // For ChVariable objects without a ChKblock, but still with a mass:
    temp_descriptor.SetMassFactor(1.0);

    // Fill system-level M matrix
    temp_descriptor.ConvertToMatrixForm(nullptr, M, nullptr, nullptr, nullptr, nullptr, false, false);
    //M->makeCompressed();
}

void ChModalAssembly::GetSubassemblyStiffnessMatrix(ChSparseMatrix* K) {
    
    this->SetupInitial();
    this->Setup();
    this->Update();

    ChSystemDescriptor temp_descriptor;

    this->InjectVariables(temp_descriptor);
    this->InjectKRMmatrices(temp_descriptor);
    this->InjectConstraints(temp_descriptor);

    // Load all KRM matrices with the K part only
    this->KRMmatricesLoad(1.0, 0, 0);
    // For ChVariable objects without a ChKblock, but still with a mass:
    temp_descriptor.SetMassFactor(0.0);

    // Fill system-level K matrix
    temp_descriptor.ConvertToMatrixForm(nullptr, K, nullptr, nullptr, nullptr, nullptr, false, false);
    //K->makeCompressed();
}

void ChModalAssembly::GetSubassemblyDampingMatrix(ChSparseMatrix* R) {
    
    this->SetupInitial();
    this->Setup();
    this->Update();

    ChSystemDescriptor temp_descriptor;

    this->InjectVariables(temp_descriptor);
    this->InjectKRMmatrices(temp_descriptor);
    this->InjectConstraints(temp_descriptor);

    // Load all KRM matrices with the R part only
    this->KRMmatricesLoad(0, 1.0, 0);
    // For ChVariable objects without a ChKblock, but still with a mass:
    temp_descriptor.SetMassFactor(0.0);

    // Fill system-level R matrix
    temp_descriptor.ConvertToMatrixForm(nullptr, R, nullptr, nullptr, nullptr, nullptr, false, false);
    //R->makeCompressed();

}

void ChModalAssembly::GetSubassemblyConstraintJacobianMatrix(ChSparseMatrix* Cq) {
    
    this->SetupInitial();
    this->Setup();
    this->Update();

    ChSystemDescriptor temp_descriptor;

    this->InjectVariables(temp_descriptor);
    this->InjectKRMmatrices(temp_descriptor);
    this->InjectConstraints(temp_descriptor);

    // Load all jacobian matrices
    this->ConstraintsLoadJacobians();

    // Fill system-level R matrix
    temp_descriptor.ConvertToMatrixForm(Cq, nullptr, nullptr, nullptr, nullptr, nullptr, false, false);
    //Cq->makeCompressed();
}

void ChModalAssembly::DumpSubassemblyMatrices(bool save_M, bool save_K, bool save_R, bool save_Cq, const char* path) {
    char filename[300];
    const char* numformat = "%.12g";

    if (save_M) {
        ChSparseMatrix mM;
        this->GetSubassemblyMassMatrix(&mM);
        sprintf(filename, "%s%s", path, "_M.dat");
        ChStreamOutAsciiFile file_M(filename);
        file_M.SetNumFormat(numformat);
        StreamOutSparseMatlabFormat(mM, file_M);
    }
    if (save_K) {
        ChSparseMatrix mK;
        this->GetSubassemblyStiffnessMatrix(&mK);
        sprintf(filename, "%s%s", path, "_K.dat");
        ChStreamOutAsciiFile file_K(filename);
        file_K.SetNumFormat(numformat);
        StreamOutSparseMatlabFormat(mK, file_K);
    }
    if (save_R) {
        ChSparseMatrix mR;
        this->GetSubassemblyDampingMatrix(&mR);
        sprintf(filename, "%s%s", path, "_R.dat");
        ChStreamOutAsciiFile file_R(filename);
        file_R.SetNumFormat(numformat);
        StreamOutSparseMatlabFormat(mR, file_R);
    }
    if (save_Cq) {
        ChSparseMatrix mCq;
        this->GetSubassemblyConstraintJacobianMatrix(&mCq);
        sprintf(filename, "%s%s", path, "_Cq.dat");
        ChStreamOutAsciiFile file_Cq(filename);
        file_Cq.SetNumFormat(numformat);
        StreamOutSparseMatlabFormat(mCq, file_Cq);
    }
}

// -----------------------------------------------------------------------------

void ChModalAssembly::SetSystem(ChSystem* m_system) {
    
    ChAssembly::SetSystem(m_system); // parent
    

    for (auto& body : internal_bodylist) {
        body->SetSystem(m_system);
    }
    for (auto& link : internal_linklist) {
        link->SetSystem(m_system);
    }
    for (auto& mesh : internal_meshlist) {
        mesh->SetSystem(m_system);
    }
    for (auto& item : internal_otherphysicslist) {
        item->SetSystem(m_system);
    }
}

void ChModalAssembly::SyncCollisionModels() {

    ChAssembly::SyncCollisionModels(); // parent

    for (auto& body : internal_bodylist) {
        body->SyncCollisionModels();
    }
    for (auto& link : internal_linklist) {
        link->SyncCollisionModels();
    }
    for (auto& mesh : internal_meshlist) {
        mesh->SyncCollisionModels();
    }
    for (auto& item : internal_otherphysicslist) {
        item->SyncCollisionModels();
    }
}

// -----------------------------------------------------------------------------
// UPDATING ROUTINES

void ChModalAssembly::SetupInitial() {

    ChAssembly::SetupInitial(); // parent

    for (int ip = 0; ip < internal_bodylist.size(); ++ip) {
        internal_bodylist[ip]->SetupInitial();
    }
    for (int ip = 0; ip < internal_linklist.size(); ++ip) {
        internal_linklist[ip]->SetupInitial();
    }
    for (int ip = 0; ip < internal_meshlist.size(); ++ip) {
        internal_meshlist[ip]->SetupInitial();
    }
    for (int ip = 0; ip < internal_otherphysicslist.size(); ++ip) {
        internal_otherphysicslist[ip]->SetupInitial();
    }
}

// Count all bodies, links, meshes, and other physics items.
// Set counters (DOF, num constraints, etc) and offsets.
void ChModalAssembly::Setup() {

    ChAssembly::Setup(); // parent

    n_boundary_bodies = nbodies;
    n_boundary_links = nlinks;
    n_boundary_meshes = nmeshes;
    n_boundary_physicsitems = nphysicsitems;
    n_boundary_coords = ncoords;
    n_boundary_coords_w = ncoords_w;
    n_boundary_doc = ndoc;
    n_boundary_doc_w = ndoc_w;
    n_boundary_doc_w_C = ndoc_w_C;
    n_boundary_doc_w_D = ndoc_w_D;
    n_boundary_sysvars   = nsysvars;
    n_boundary_sysvars_w = nsysvars_w;
    n_boundary_dof       = ndof;

    n_internal_bodies = 0;
    n_internal_links = 0;
    n_internal_meshes = 0;
    n_internal_physicsitems = 0;
    n_internal_coords = 0;
    n_internal_coords_w = 0;
    n_internal_doc = 0;
    n_internal_doc_w = 0;
    n_internal_doc_w_C = 0;
    n_internal_doc_w_D = 0;

    // For the "internal" items:
    //

    for (auto& body : internal_bodylist) {
        if (body->GetBodyFixed())
        {
            //throw ChException("Cannot use a fixed body as internal");
        }
        else if (body->GetSleeping())
        {
            //throw ChException("Cannot use a sleeping body as internal");
        }
        else {
            n_internal_bodies++;

            body->SetOffset_x(this->offset_x + n_boundary_coords   + n_internal_coords);
            body->SetOffset_w(this->offset_w + n_boundary_coords_w + n_internal_coords_w);
            body->SetOffset_L(this->offset_L + n_boundary_doc_w    + n_internal_doc_w);

            body->Setup();  // currently, no-op

            n_internal_coords += body->GetDOF();
            n_internal_coords_w += body->GetDOF_w();
            n_internal_doc_w += body->GetDOC();      // not really needed since ChBody introduces no constraints
        }
    }

    for (auto& link : internal_linklist) {
        if (link->IsActive()) {
            n_internal_links++;

            link->SetOffset_x(this->offset_x + n_boundary_coords   + n_internal_coords);
            link->SetOffset_w(this->offset_w + n_boundary_coords_w + n_internal_coords_w);
            link->SetOffset_L(this->offset_L + n_boundary_doc_w    + n_internal_doc_w);

            link->Setup();  // compute DOFs etc. and sets the offsets also in child items, if any

            n_internal_coords += link->GetDOF();
            n_internal_coords_w += link->GetDOF_w();
            n_internal_doc_w += link->GetDOC();
            n_internal_doc_w_C += link->GetDOC_c();
            n_internal_doc_w_D += link->GetDOC_d();
        }
    }

    for (auto& mesh : internal_meshlist) {
        n_internal_meshes++;

        mesh->SetOffset_x(this->offset_x + n_boundary_coords   + n_internal_coords);
        mesh->SetOffset_w(this->offset_w + n_boundary_coords_w + n_internal_coords_w);
        mesh->SetOffset_L(this->offset_L + n_boundary_doc_w    + n_internal_doc_w);

        mesh->Setup();  // compute DOFs and iteratively call Setup for child items

        n_internal_coords += mesh->GetDOF();
        n_internal_coords_w += mesh->GetDOF_w();
        n_internal_doc_w += mesh->GetDOC();
        n_internal_doc_w_C += mesh->GetDOC_c();
        n_internal_doc_w_D += mesh->GetDOC_d();
    }

    for (auto& item : internal_otherphysicslist) {
        n_internal_physicsitems++;

        item->SetOffset_x(this->offset_x + n_boundary_coords   + n_internal_coords);
        item->SetOffset_w(this->offset_w + n_boundary_coords_w + n_internal_coords_w);
        item->SetOffset_L(this->offset_L + n_boundary_doc_w    + n_internal_doc_w);

        item->Setup();

        n_internal_coords += item->GetDOF();
        n_internal_coords_w += item->GetDOF_w();
        n_internal_doc_w += item->GetDOC();
        n_internal_doc_w_C += item->GetDOC_c();
        n_internal_doc_w_D += item->GetDOC_d();
    }

    n_internal_doc = n_internal_doc_w + n_internal_bodies;          // number of constraints including quaternion constraints.
    n_internal_sysvars = n_internal_coords + n_internal_doc;        // total number of variables (coordinates + lagrangian multipliers)
    n_internal_sysvars_w = n_internal_coords_w + n_internal_doc_w;  // total number of variables (with 6 dof per body)
    n_internal_dof = n_internal_coords_w - n_internal_doc_w;

    this->custom_F_full.setZero(this->n_boundary_coords_w + this->n_internal_coords_w);

    // For the modal part:
    //

    // (nothing to count)


    // For the entire assembly:
    //

    if (this->is_modal == false) {
        ncoords    = n_boundary_coords    + n_internal_coords;
        ncoords_w  = n_boundary_coords_w  + n_internal_coords_w;
        ndoc       = n_boundary_doc       + n_internal_doc;
        ndoc_w     = n_boundary_doc_w     + n_internal_doc_w;
        ndoc_w_C   = n_boundary_doc_w_C   + n_internal_doc_w_C;
        ndoc_w_D   = n_boundary_doc_w_D   + n_internal_doc_w_D;
        nsysvars   = n_boundary_sysvars   + n_internal_sysvars;
        nsysvars_w = n_boundary_sysvars_w + n_internal_sysvars_w;
        ndof       = n_boundary_dof       + n_internal_dof;
        nbodies += n_internal_bodies;
        nlinks  += n_internal_links;
        nmeshes += n_internal_meshes;
        nphysicsitems += n_internal_physicsitems;
    }
    else {
        ncoords    = n_boundary_coords    + n_modes_coords_w; // no need for a n_modes_coords, same as n_modes_coords_w
        ncoords_w  = n_boundary_coords_w  + n_modes_coords_w;
        ndoc       = n_boundary_doc;
        ndoc_w     = n_boundary_doc_w;
        ndoc_w_C   = n_boundary_doc_w_C;
        ndoc_w_D   = n_boundary_doc_w_D;
        nsysvars   = n_boundary_sysvars   + n_modes_coords_w; // no need for a n_modes_coords, same as n_modes_coords_w
        nsysvars_w = n_boundary_sysvars_w + n_modes_coords_w;
        ndof       = n_boundary_dof       + n_modes_coords_w;
    }
}



// Update all physical items (bodies, links, meshes, etc), including their auxiliary variables.
// Updates all forces (automatic, as children of bodies)
// Updates all markers (automatic, as children of bodies).
void ChModalAssembly::Update(bool update_assets) {

    ChAssembly::Update(update_assets);  // parent

    if (is_modal == false) {
        //// NOTE: do not switch these to range for loops (may want to use OMP for)
        for (int ip = 0; ip < (int)internal_bodylist.size(); ++ip) {
            internal_bodylist[ip]->Update(ChTime, update_assets);
        }
        for (int ip = 0; ip < (int)internal_otherphysicslist.size(); ++ip) {
            internal_otherphysicslist[ip]->Update(ChTime, update_assets);
        }
        for (int ip = 0; ip < (int)internal_linklist.size(); ++ip) {
            internal_linklist[ip]->Update(ChTime, update_assets);
        }
        for (int ip = 0; ip < (int)internal_meshlist.size(); ++ip) {
            internal_meshlist[ip]->Update(ChTime, update_assets);
        }

        if (m_custom_F_full_callback)
            m_custom_F_full_callback->evaluate(this->custom_F_full, *this);
    }
    else {
        // If in modal reduction mode, the internal parts would not be updated (actually, these could even be removed)
        // However one still might want to see the internal nodes "moving" during animations, 
        if (this->internal_nodes_update)
            this->SetInternalStateWithModes(update_assets);

        if (m_custom_F_modal_callback)
            m_custom_F_modal_callback->evaluate(this->custom_F_modal, *this);

        if (m_custom_F_full_callback)
            m_custom_F_full_callback->evaluate(this->custom_F_full, *this);
    }

}

void ChModalAssembly::SetNoSpeedNoAcceleration() {

    ChAssembly::SetNoSpeedNoAcceleration();  // parent

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            body->SetNoSpeedNoAcceleration();
        }
        for (auto& link : internal_linklist) {
            link->SetNoSpeedNoAcceleration();
        }
        for (auto& mesh : internal_meshlist) {
            mesh->SetNoSpeedNoAcceleration();
        }
        for (auto& item : internal_otherphysicslist) {
            item->SetNoSpeedNoAcceleration();
        }
    }
    else {
        this->modal_q_dt.setZero(this->n_modes_coords_w);
        this->modal_q_dtdt.setZero(this->n_modes_coords_w);
    }
}


void ChModalAssembly::GetStateLocal(ChStateDelta& Dx, ChStateDelta& v_local) {

    if (is_modal == false) {
        // to do? not useful for the moment.
        return;
    }
    else {
        Dx.setZero(this->n_boundary_coords_w + this->n_modes_coords_w, nullptr);

        // fetch the state snapshot (modal reduced)
        int bou_mod_coords   = this->n_boundary_coords     + this->n_modes_coords_w;
        int bou_mod_coords_w = this->n_boundary_coords_w   + this->n_modes_coords_w;
        double fooT;
        ChState       x_mod;
        ChStateDelta  v_mod;
        x_mod.setZero(bou_mod_coords, nullptr);
        v_mod.setZero(bou_mod_coords_w, nullptr);
        this->IntStateGather(0, x_mod, 0, v_mod, fooT);

        // the old state snapshot (modal reduced)
        ChState x0_mod;
        x0_mod.setZero(bou_mod_coords, nullptr);
        x0_mod.segment(0, this->n_boundary_coords) = this->assembly_x0.segment(0, this->n_boundary_coords);
        
        this->IntStateGetIncrement(0, x_mod, x0_mod, 0, Dx);

        v_local = v_mod;

        //***TODO***
        // transform all Dx and all v_local into the local corotated reference. Now works only if no large rotations in modal assembly.
    }
}






void ChModalAssembly::IntStateGather(const unsigned int off_x,
                                ChState& x,
                                const unsigned int off_v,
                                ChStateDelta& v,
                                double& T) {
    ChAssembly::IntStateGather(off_x, x, off_v, v, T);  // parent

    unsigned int displ_x = off_x - this->offset_x;
    unsigned int displ_v = off_v - this->offset_w;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntStateGather(displ_x + body->GetOffset_x(), x, displ_v + body->GetOffset_w(), v, T);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntStateGather(displ_x + link->GetOffset_x(), x, displ_v + link->GetOffset_w(), v, T);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntStateGather(displ_x + mesh->GetOffset_x(), x, displ_v + mesh->GetOffset_w(), v, T);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntStateGather(displ_x + item->GetOffset_x(), x, displ_v + item->GetOffset_w(), v, T);
        }
    }
    else {
        x.segment(off_x + this->n_boundary_coords,   this->n_modes_coords_w) = this->modal_q;
        v.segment(off_v + this->n_boundary_coords_w, this->n_modes_coords_w) = this->modal_q_dt;
    }
}

void ChModalAssembly::IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T,
                                 bool full_update) {
    ChAssembly::IntStateScatter(off_x, x, off_v, v, T, full_update);  // parent

    unsigned int displ_x = off_x - this->offset_x;
    unsigned int displ_v = off_v - this->offset_w;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntStateScatter(displ_x + body->GetOffset_x(), x, displ_v + body->GetOffset_w(), v, T, full_update);
            else
                body->Update(T, full_update);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntStateScatter(displ_x + mesh->GetOffset_x(), x, displ_v + mesh->GetOffset_w(), v, T, full_update);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntStateScatter(displ_x + link->GetOffset_x(), x, displ_v + link->GetOffset_w(), v, T, full_update);
            else
                link->Update(T, full_update);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntStateScatter(displ_x + item->GetOffset_x(), x, displ_v + item->GetOffset_w(), v, T, full_update);
        }

        if (m_custom_F_full_callback)
            m_custom_F_full_callback->evaluate(this->custom_F_full, *this);
    }
    else {
        this->modal_q    = x.segment(off_x + this->n_boundary_coords,   this->n_modes_coords_w);
        this->modal_q_dt = v.segment(off_v + this->n_boundary_coords_w, this->n_modes_coords_w);

        // Update: 
        // If in modal reduction mode, the internal parts would not be updated (actually, these could even be removed)
        // However one still might want to see the internal nodes "moving" during animations, 
        if (this->internal_nodes_update)
            this->SetInternalStateWithModes(full_update);

        if (m_custom_F_modal_callback)
            m_custom_F_modal_callback->evaluate(this->custom_F_modal, *this);

        if (m_custom_F_full_callback)
            m_custom_F_full_callback->evaluate(this->custom_F_full, *this);
    }
}

void ChModalAssembly::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {

    ChAssembly::IntStateGatherAcceleration(off_a, a);  // parent

    unsigned int displ_a = off_a - this->offset_w;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntStateGatherAcceleration(displ_a + body->GetOffset_w(), a);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntStateGatherAcceleration(displ_a + link->GetOffset_w(), a);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntStateGatherAcceleration(displ_a + mesh->GetOffset_w(), a);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntStateGatherAcceleration(displ_a + item->GetOffset_w(), a);
        }
    }
    else {
        a.segment(off_a + this->n_boundary_coords_w, this->n_modes_coords_w) = this->modal_q_dtdt;
    }
}

// From state derivative (acceleration) to system, sometimes might be needed
void ChModalAssembly::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {

    ChAssembly::IntStateScatterAcceleration(off_a, a);  // parent

    unsigned int displ_a = off_a - this->offset_w;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntStateScatterAcceleration(displ_a + body->GetOffset_w(), a);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntStateScatterAcceleration(displ_a + link->GetOffset_w(), a);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntStateScatterAcceleration(displ_a + mesh->GetOffset_w(), a);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntStateScatterAcceleration(displ_a + item->GetOffset_w(), a);
        }
    }
    else {
        this->modal_q_dtdt = a.segment(off_a + this->n_boundary_coords_w, this->n_modes_coords_w);
    }
}

// From system to reaction forces (last computed) - some timestepper might need this
void ChModalAssembly::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    
    ChAssembly::IntStateGatherReactions(off_L, L);  // parent

    unsigned int displ_L = off_L - this->offset_L;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntStateGatherReactions(displ_L + body->GetOffset_L(), L);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntStateGatherReactions(displ_L + link->GetOffset_L(), L);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntStateGatherReactions(displ_L + mesh->GetOffset_L(), L);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntStateGatherReactions(displ_L + item->GetOffset_L(), L);
        }
    }
}

// From reaction forces to system, ex. store last computed reactions in ChLinkBase objects for plotting etc.
void ChModalAssembly::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {

    ChAssembly::IntStateScatterReactions(off_L, L);  // parent

    unsigned int displ_L = off_L - this->offset_L;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntStateScatterReactions(displ_L + body->GetOffset_L(), L);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntStateScatterReactions(displ_L + link->GetOffset_L(), L);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntStateScatterReactions(displ_L + mesh->GetOffset_L(), L);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntStateScatterReactions(displ_L + item->GetOffset_L(), L);
        }
    }
}

void ChModalAssembly::IntStateIncrement(const unsigned int off_x,
                                   ChState& x_new,
                                   const ChState& x,
                                   const unsigned int off_v,
                                   const ChStateDelta& Dv) {

    ChAssembly::IntStateIncrement(off_x, x_new, x, off_v, Dv);  // parent

    unsigned int displ_x = off_x - this->offset_x;
    unsigned int displ_v = off_v - this->offset_w;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntStateIncrement(displ_x + body->GetOffset_x(), x_new, x, displ_v + body->GetOffset_w(), Dv);
        }

        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntStateIncrement(displ_x + link->GetOffset_x(), x_new, x, displ_v + link->GetOffset_w(), Dv);
        }

        for (auto& mesh : internal_meshlist) {
            mesh->IntStateIncrement(displ_x + mesh->GetOffset_x(), x_new, x, displ_v + mesh->GetOffset_w(), Dv);
        }

        for (auto& item : internal_otherphysicslist) {
            item->IntStateIncrement(displ_x + item->GetOffset_x(), x_new, x, displ_v + item->GetOffset_w(), Dv);
        }
    }
    else {
        x_new.segment(off_x + this->n_boundary_coords, this->n_modes_coords_w) = 
            x.segment(off_x + this->n_boundary_coords,   this->n_modes_coords_w) 
         + Dv.segment(off_v + this->n_boundary_coords_w, this->n_modes_coords_w);
    }
}

void ChModalAssembly::IntStateGetIncrement(const unsigned int off_x,
                                   const ChState& x_new,
                                   const ChState& x,
                                   const unsigned int off_v,
                                   ChStateDelta& Dv) {

    ChAssembly::IntStateGetIncrement(off_x, x_new, x, off_v, Dv);  // parent

    unsigned int displ_x = off_x - this->offset_x;
    unsigned int displ_v = off_v - this->offset_w;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntStateGetIncrement(displ_x + body->GetOffset_x(), x_new, x, displ_v + body->GetOffset_w(), Dv);
        }

        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntStateGetIncrement(displ_x + link->GetOffset_x(), x_new, x, displ_v + link->GetOffset_w(), Dv);
        }

        for (auto& mesh : internal_meshlist) {
            mesh->IntStateGetIncrement(displ_x + mesh->GetOffset_x(), x_new, x, displ_v + mesh->GetOffset_w(), Dv);
        }

        for (auto& item : internal_otherphysicslist) {
            item->IntStateGetIncrement(displ_x + item->GetOffset_x(), x_new, x, displ_v + item->GetOffset_w(), Dv);
        }
    }
    else {
        Dv.segment(off_v + this->n_boundary_coords_w,  this->n_modes_coords_w) = 
        x_new.segment(off_x + this->n_boundary_coords, this->n_modes_coords_w) - 
            x.segment(off_x + this->n_boundary_coords, this->n_modes_coords_w);
    }
}


void ChModalAssembly::IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
                                   ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
                                   const double c)          ///< a scaling factor
{
    ChAssembly::IntLoadResidual_F(off, R, c);  // parent

    unsigned int displ_v = off - this->offset_w;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntLoadResidual_F(displ_v + body->GetOffset_w(), R, c);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntLoadResidual_F(displ_v + link->GetOffset_w(), R, c);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntLoadResidual_F(displ_v + mesh->GetOffset_w(), R, c);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntLoadResidual_F(displ_v + item->GetOffset_w(), R, c);
        }

        // Add custom forces (applied to the original non reduced system) 
        if (!this->custom_F_full.isZero()) {
            R.segment(displ_v, this->n_boundary_coords_w + this->n_internal_coords_w) += c * this->custom_F_full;
        }
    }
    else {
        // 1-
        // Add elastic forces from current modal deformations
        ChStateDelta Dx_local(this->n_boundary_coords_w + this->n_modes_coords_w, nullptr);
        ChStateDelta v_local(this->n_boundary_coords_w + this->n_modes_coords_w, nullptr);
        this->GetStateLocal(Dx_local, v_local);

        R.segment(off, this->n_boundary_coords_w + this->n_modes_coords_w) -= c * (this->modal_K * Dx_local + this->modal_R * v_local); //  note -= sign

        // 2-
        // Add custom forces (in modal coordinates)
        if (!this->custom_F_modal.isZero())
            R.segment(off + this->n_boundary_coords_w, this->n_modes_coords_w) += c * this->custom_F_modal;

        // 3-
        // Add custom forces (applied to the original non reduced system, and transformed into reduced) 
        if (!this->custom_F_full.isZero())
            R.segment(off, this->n_boundary_coords_w + this->n_modes_coords_w) += c * this->Psi.transpose() * this->custom_F_full;

    }
}

void ChModalAssembly::IntLoadResidual_Mv(const unsigned int off,      ///< offset in R residual
                                    ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                    const ChVectorDynamic<>& w,  ///< the w vector
                                    const double c               ///< a scaling factor
) {
    unsigned int displ_v = off - this->offset_w;

    if (is_modal == false) {
        ChAssembly::IntLoadResidual_Mv(off, R, w, c);  // parent

        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntLoadResidual_Mv(displ_v + body->GetOffset_w(), R, w, c);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntLoadResidual_Mv(displ_v + link->GetOffset_w(), R, w, c);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntLoadResidual_Mv(displ_v + mesh->GetOffset_w(), R, w, c);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntLoadResidual_Mv(displ_v + item->GetOffset_w(), R, w, c);
        }
    } 
    else {
        ChVectorDynamic<> w_modal = w.segment(off, this->n_boundary_coords_w + this->n_modes_coords_w);
        R.segment(off, this->n_boundary_coords_w + this->n_modes_coords_w) += c * (this->modal_M * w_modal);
    }
}

void ChModalAssembly::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                     ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                     const ChVectorDynamic<>& L,  ///< the L vector
                                     const double c               ///< a scaling factor
) {
    ChAssembly::IntLoadResidual_CqL(off_L, R, L, c);  // parent

    unsigned int displ_L = off_L - this->offset_L;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntLoadResidual_CqL(displ_L + body->GetOffset_L(), R, L, c);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntLoadResidual_CqL(displ_L + link->GetOffset_L(), R, L, c);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntLoadResidual_CqL(displ_L + mesh->GetOffset_L(), R, L, c);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntLoadResidual_CqL(displ_L + item->GetOffset_L(), R, L, c);
        }
    }
}

void ChModalAssembly::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
                                     ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*C
                                     const double c,            ///< a scaling factor
                                     bool do_clamp,             ///< apply clamping to c*C?
                                     double recovery_clamp      ///< value for min/max clamping of c*C
) {
    ChAssembly::IntLoadConstraint_C(off_L, Qc, c, do_clamp, recovery_clamp);  // parent

    unsigned int displ_L = off_L - this->offset_L;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntLoadConstraint_C(displ_L + body->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntLoadConstraint_C(displ_L + link->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntLoadConstraint_C(displ_L + mesh->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntLoadConstraint_C(displ_L + item->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
        }
    }
}

void ChModalAssembly::IntLoadConstraint_Ct(const unsigned int off_L,  ///< offset in Qc residual
                                      ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*Ct
                                      const double c             ///< a scaling factor
) {
    ChAssembly::IntLoadConstraint_Ct(off_L, Qc, c);  // parent

    unsigned int displ_L = off_L - this->offset_L;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntLoadConstraint_Ct(displ_L + body->GetOffset_L(), Qc, c);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntLoadConstraint_Ct(displ_L + link->GetOffset_L(), Qc, c);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntLoadConstraint_Ct(displ_L + mesh->GetOffset_L(), Qc, c);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntLoadConstraint_Ct(displ_L + item->GetOffset_L(), Qc, c);
        }
    }
}

void ChModalAssembly::IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) {

    ChAssembly::IntToDescriptor(off_v, v, R, off_L, L, Qc);  // parent

    unsigned int displ_L = off_L - this->offset_L;
    unsigned int displ_v = off_v - this->offset_w;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntToDescriptor(displ_v + body->GetOffset_w(), v, R, displ_L + body->GetOffset_L(), L, Qc);
        }

        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntToDescriptor(displ_v + link->GetOffset_w(), v, R, displ_L + link->GetOffset_L(), L, Qc);
        }

        for (auto& mesh : internal_meshlist) {
            mesh->IntToDescriptor(displ_v + mesh->GetOffset_w(), v, R, displ_L + mesh->GetOffset_L(), L, Qc);
        }

        for (auto& item : internal_otherphysicslist) {
            item->IntToDescriptor(displ_v + item->GetOffset_w(), v, R, displ_L + item->GetOffset_L(), L, Qc);
        }
    } 
    else {
        this->modal_variables->Get_qb() = v.segment(off_v + this->n_boundary_coords_w,  this->n_modes_coords_w);
        this->modal_variables->Get_fb() = R.segment(off_v + this->n_boundary_coords_w,  this->n_modes_coords_w);
    }
}

void ChModalAssembly::IntFromDescriptor(const unsigned int off_v,
                                   ChStateDelta& v,
                                   const unsigned int off_L,
                                   ChVectorDynamic<>& L) {

    ChAssembly::IntFromDescriptor(off_v, v, off_L, L);  // parent

    unsigned int displ_L = off_L - this->offset_L;
    unsigned int displ_v = off_v - this->offset_w;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntFromDescriptor(displ_v + body->GetOffset_w(), v, displ_L + body->GetOffset_L(), L);
        }

        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntFromDescriptor(displ_v + link->GetOffset_w(), v, displ_L + link->GetOffset_L(), L);
        }

        for (auto& mesh : internal_meshlist) {
            mesh->IntFromDescriptor(displ_v + mesh->GetOffset_w(), v, displ_L + mesh->GetOffset_L(), L);
        }

        for (auto& item : internal_otherphysicslist) {
            item->IntFromDescriptor(displ_v + item->GetOffset_w(), v, displ_L + item->GetOffset_L(), L);
        }
    }
    else {
        v.segment(off_v + this->n_boundary_coords_w,  this->n_modes_coords_w) = this->modal_variables->Get_qb();
    }
}

// -----------------------------------------------------------------------------

void ChModalAssembly::InjectVariables(ChSystemDescriptor& mdescriptor) {

    ChAssembly::InjectVariables(mdescriptor);  // parent

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            body->InjectVariables(mdescriptor);
        }
        for (auto& link : internal_linklist) {
            link->InjectVariables(mdescriptor);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->InjectVariables(mdescriptor);
        }
        for (auto& item : internal_otherphysicslist) {
            item->InjectVariables(mdescriptor);
        }
    }
    else {
        mdescriptor.InsertVariables(this->modal_variables);
    }
}



void ChModalAssembly::InjectConstraints(ChSystemDescriptor& mdescriptor) {

    ChAssembly::InjectConstraints(mdescriptor);  // parent

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            body->InjectConstraints(mdescriptor);
        }
        for (auto& link : internal_linklist) {
            link->InjectConstraints(mdescriptor);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->InjectConstraints(mdescriptor);
        }
        for (auto& item : internal_otherphysicslist) {
            item->InjectConstraints(mdescriptor);
        }
    }
}



void ChModalAssembly::ConstraintsLoadJacobians() {

    ChAssembly::ConstraintsLoadJacobians();  // parent

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            body->ConstraintsLoadJacobians();
        }
        for (auto& link : internal_linklist) {
            link->ConstraintsLoadJacobians();
        }
        for (auto& mesh : internal_meshlist) {
            mesh->ConstraintsLoadJacobians();
        }
        for (auto& item : internal_otherphysicslist) {
            item->ConstraintsLoadJacobians();
        }
    }
}

void ChModalAssembly::InjectKRMmatrices(ChSystemDescriptor& mdescriptor) {

    if (is_modal == false) {

        ChAssembly::InjectKRMmatrices(mdescriptor);  // parent

        for (auto& body : internal_bodylist) {
            body->InjectKRMmatrices(mdescriptor);
        }
        for (auto& link : internal_linklist) {
            link->InjectKRMmatrices(mdescriptor);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->InjectKRMmatrices(mdescriptor);
        }
        for (auto& item : internal_otherphysicslist) {
            item->InjectKRMmatrices(mdescriptor);
        }
    }
    else {
        mdescriptor.InsertKblock(&this->modal_Hblock);
    }
}

void ChModalAssembly::KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {

    if (is_modal == false) {
        {
            ChAssembly::KRMmatricesLoad(Kfactor, Rfactor, Mfactor);  // parent

            for (auto& body : internal_bodylist) {
                body->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
            }
            for (auto& link : internal_linklist) {
                link->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
            }
            for (auto& mesh : internal_meshlist) {
                mesh->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
            }
            for (auto& item : internal_otherphysicslist) {
                item->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
            }
        }
    } 
    else {
        this->modal_Hblock.Get_K() = 
            this->modal_K * Kfactor + 
            this->modal_R * Rfactor + 
            this->modal_M * Mfactor;
    }
}

// -----------------------------------------------------------------------------
//  STREAMING - FILE HANDLING


void ChModalAssembly::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChModalAssembly>();

    // serialize parent class
    ChAssembly::ArchiveOut(marchive);

    // serialize all member data:

    marchive << CHNVP(internal_bodylist, "internal_bodies");
    marchive << CHNVP(internal_linklist, "internal_links");
    marchive << CHNVP(internal_meshlist, "internal_meshes");
    marchive << CHNVP(internal_otherphysicslist, "internal_other_physics_items");
    marchive << CHNVP(is_modal, "is_modal");
    marchive << CHNVP(modal_q, "modal_q");
    marchive << CHNVP(modal_q_dt, "modal_q_dt");
    marchive << CHNVP(modal_q_dtdt, "modal_q_dtdt");
    marchive << CHNVP(custom_F_modal, "custom_F_modal");
    marchive << CHNVP(custom_F_full, "custom_F_full");
    marchive << CHNVP(internal_nodes_update, "internal_nodes_update");
}

void ChModalAssembly::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChModalAssembly>();

    // deserialize parent class
    ChAssembly::ArchiveIn(marchive);

    // stream in all member data:
    
    // trick needed because the "AddIntenal...()" functions are required
    std::vector<std::shared_ptr<ChBody>> tempbodies;
    marchive >> CHNVP(tempbodies, "internal_bodies");
    RemoveAllBodies();
    for (auto& body : tempbodies) 
        AddInternalBody(body);
    std::vector<std::shared_ptr<ChLink>> templinks;
    marchive >> CHNVP(templinks, "internal_links");
    RemoveAllLinks();
    for (auto& link : templinks) 
        AddInternalLink(link);
    std::vector<std::shared_ptr<ChMesh>> tempmeshes;
    marchive >> CHNVP(tempmeshes, "internal_mesh");
    RemoveAllMeshes();
    for (auto& mesh : tempmeshes) 
        AddInternalMesh(mesh);
    std::vector<std::shared_ptr<ChPhysicsItem>> tempotherphysics;
    marchive >> CHNVP(tempotherphysics, "internal_other_physics_items");
    RemoveAllOtherPhysicsItems();
    for (auto& mphys : tempotherphysics) 
        AddInternalOtherPhysicsItem(mphys);

    marchive >> CHNVP(is_modal, "is_modal");
    marchive >> CHNVP(modal_q, "modal_q");
    marchive >> CHNVP(modal_q_dt, "modal_q_dt");
    marchive >> CHNVP(modal_q_dtdt, "modal_q_dtdt");
    marchive >> CHNVP(custom_F_modal, "custom_F_modal");
    marchive >> CHNVP(custom_F_full, "custom_F_full");
    marchive >> CHNVP(internal_nodes_update, "internal_nodes_update");

    // Recompute statistics, offsets, etc.
    Setup();
}



}  // end namespace modal

}  // end namespace chrono
