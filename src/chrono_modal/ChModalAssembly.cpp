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
using namespace geometry;

namespace modal {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChModalAssembly)

ChModalAssembly::ChModalAssembly()
    : modal_variables(nullptr), n_modes_coords_w(0), is_modal(false), internal_nodes_update(true) {}

ChModalAssembly::ChModalAssembly(const ChModalAssembly& other) : ChAssembly(other) {
    is_modal = other.is_modal;
    modal_q = other.modal_q;
    modal_q_dt = other.modal_q_dt;
    modal_q_dtdt = other.modal_q_dtdt;
    custom_F_modal = other.custom_F_modal;
    internal_nodes_update = other.internal_nodes_update;
    m_custom_F_modal_callback = other.m_custom_F_modal_callback;
    m_custom_F_full_callback = other.m_custom_F_full_callback;

    //// TODO:  deep copy of the object lists (internal_bodylist, internal_linklist, internal_meshlist,
    /// internal_otherphysicslist)
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
// classes that have a ChModalAssembly member (currently only ChSystem) could use it, the same way we use std::swap
// here.
void swap(ChModalAssembly& first, ChModalAssembly& second) {
    using std::swap;
    // swap(first.nbodies, second.nbodies);
    // ***TODO***
}

void ChModalAssembly::Clear() {
    ChAssembly::Clear();  // parent

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
void util_sparse_assembly_2x2symm(
    Eigen::SparseMatrix<double, Eigen::ColMajor, int>& HCQ,  ///< resulting square sparse matrix (column major)
    const ChSparseMatrix& H,                                 ///< square sparse H matrix, n_v x n_v
    const ChSparseMatrix& Cq)                                ///< rectangular  sparse Cq  n_c x n_v
{
    int n_v = H.rows();
    int n_c = Cq.rows();
    HCQ.resize(n_v + n_c, n_v + n_c);
    HCQ.reserve(H.nonZeros() + 2 * Cq.nonZeros());
    HCQ.setZero();

    for (int k = 0; k < H.outerSize(); ++k)
        for (ChSparseMatrix::InnerIterator it(H, k); it; ++it) {
            HCQ.insert(it.row(), it.col()) = it.value();
        }

    for (int k = 0; k < Cq.outerSize(); ++k)
        for (ChSparseMatrix::InnerIterator it(Cq, k); it; ++it) {
            HCQ.insert(it.row() + n_v, it.col()) = it.value();  // insert Cq
            HCQ.insert(it.col(), it.row() + n_v) = it.value();  // insert Cq'
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
void ChModalAssembly::SwitchModalReductionON(ChSparseMatrix& full_M,
                                             ChSparseMatrix& full_K,
                                             ChSparseMatrix& full_Cq,
                                             const ChModalSolveUndamped& n_modes_settings,
                                             const ChModalDamping& damping_model) {
    if (this->is_modal)
        return;

    this->SetupInitial();
    this->Setup();
    this->Update();

    // fetch the initial state of assembly, full not reduced, as an initialization
    double fooT;
    this->full_assembly_x.setZero(this->ncoords, nullptr);
    ChStateDelta full_assembly_v;
    full_assembly_v.setZero(this->ncoords_w, nullptr);
    this->IntStateGather(0, this->full_assembly_x, 0, full_assembly_v, fooT);

    // store the initial state of assembly, later will be used to compute the elastic deformation, etc.
    this->modes_assembly_x0.setZero(this->ncoords, nullptr);  //[qB; qI]
    this->modes_assembly_x0 = this->full_assembly_x;

    // initialize the floating frame of reference F to be placed at COG
    this->UpdateFloatingFrameOfReference();

    // recover the local M,K,Cq (full_M_loc, full_K_loc, full_Cq_loc) matrices
    // through rotating back to the local frame of F
    this->ComputeLocalFullKMCqMatrix(full_M, full_K, full_Cq);

    if (this->modal_reduction_type == Herting) {
        // 1) compute eigenvalue and eigenvectors
        int expected_eigs = 0;
        for (auto freq_span : n_modes_settings.freq_spans)
            expected_eigs += freq_span.nmodes;

        if (expected_eigs < 6) {
            GetLog() << "*** At least six rigid-body modes are required for the modal reduction. "
                        "The default settings are used.\n";
            this->ComputeModesExternalData(this->full_M_loc, this->full_K_loc, this->full_Cq_loc,
                                           ChModalSolveUndamped(6));
        } else
            this->ComputeModesExternalData(this->full_M_loc, this->full_K_loc, this->full_Cq_loc, n_modes_settings);

        // 2) bound ChVariables etc. to the modal coordinates, resize matrices, set as modal mode
        this->SetModalMode(true);
        this->SetupModalData(this->modes_V.cols());

        if (this->verbose) {
            GetLog() << "*** Herting reduction is used.\n";
            for (int i = 0; i < this->modes_eig.size(); ++i)
                GetLog() << " Damped mode n." << i << "  frequency [Hz]: " << this->modes_freq(i) << "\n";
        }

        // 3) compute the transforamtion matrices, also the local rigid-body modes
        this->UpdateTransformationMatrix();

        // 4) do the Herting reduction as in Sonneville2021
        this->DoModalReduction_Herting(damping_model);

    } else if (this->modal_reduction_type == Craig_Bampton) {
        // 1) retrieve the local M,K,Cq matrices sliced for internal nodes, and then compute eigenvalue and eigenvectors
        ChSparseMatrix full_M_II_loc =
            this->full_M_loc.block(n_boundary_coords_w, n_boundary_coords_w, n_internal_coords_w, n_internal_coords_w);
        ChSparseMatrix full_K_II_loc =
            this->full_K_loc.block(n_boundary_coords_w, n_boundary_coords_w, n_internal_coords_w, n_internal_coords_w);
        ChSparseMatrix full_Cq_II_loc =
            this->full_Cq_loc.block(n_boundary_doc, n_boundary_coords_w, n_internal_doc, n_internal_coords_w);

        this->ComputeModesExternalData(full_M_II_loc, full_K_II_loc, full_Cq_II_loc, n_modes_settings);

        // 2) bound ChVariables etc. to the modal coordinates, resize matrices, set as modal mode
        this->SetModalMode(true);
        this->SetupModalData(this->modes_V.cols());

        if (this->verbose) {
            GetLog() << "*** Craig Bampton reduction is used.\n";
            for (int i = 0; i < this->modes_eig.size(); ++i)
                GetLog() << " Damped mode n." << i << "  frequency [Hz]: " << this->modes_freq(i) << "\n";
        }

        // 3) compute the transforamtion matrices, also the local rigid-body modes
        this->UpdateTransformationMatrix();

        // 4) do the Craig-Bampton reduction as in Cardona2001
        this->DoModalReduction_CraigBamption(damping_model);

    } else {
        GetLog() << "*** The modal reduction type is specified incorrectly...\n";
        assert(0);
    }

    // initialize the projection matrices
    this->ComputeProjectionMatrix();

    // initialize the modal K R M matrices
    this->ComputeModalKRMmatrix();

    // Debug dump data. ***TODO*** remove
    if (this->verbose) {
        ChStreamOutAsciiFile filePsi("dump_modal_Psi.dat");
        filePsi.SetNumFormat("%.12g");
        StreamOutDenseMatlabFormat(Psi, filePsi);
        ChStreamOutAsciiFile fileM("dump_modal_M.dat");
        fileM.SetNumFormat("%.12g");
        StreamOutDenseMatlabFormat(this->modal_M, fileM);
        ChStreamOutAsciiFile fileK("dump_modal_K.dat");
        fileK.SetNumFormat("%.12g");
        StreamOutDenseMatlabFormat(this->modal_K, fileK);
        ChStreamOutAsciiFile fileR("dump_modal_R.dat");
        fileR.SetNumFormat("%.12g");
        StreamOutDenseMatlabFormat(this->modal_R, fileR);
        ChStreamOutAsciiFile fileCq("dump_modal_Cq.dat");
        fileCq.SetNumFormat("%.12g");
        StreamOutDenseMatlabFormat(this->modal_Cq, fileCq);

        ChStreamOutAsciiFile fileM_red("dump_reduced_M.dat");
        fileM_red.SetNumFormat("%.12g");
        StreamOutDenseMatlabFormat(this->M_red, fileM_red);
        ChStreamOutAsciiFile fileK_red("dump_reduced_K.dat");
        fileK_red.SetNumFormat("%.12g");
        StreamOutDenseMatlabFormat(this->K_red, fileK_red);
        ChStreamOutAsciiFile fileR_red("dump_reduced_R.dat");
        fileR_red.SetNumFormat("%.12g");
        StreamOutDenseMatlabFormat(this->R_red, fileR_red);
        ChStreamOutAsciiFile fileCq_red("dump_reduced_Cq.dat");
        fileCq_red.SetNumFormat("%.12g");
        StreamOutDenseMatlabFormat(this->Cq_red, fileCq_red);
    }
}

void ChModalAssembly::SwitchModalReductionON(const ChModalSolveUndamped& n_modes_settings,
                                             const ChModalDamping& damping_model) {
    if (this->is_modal)
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

void ChModalAssembly::ComputeMassCenter() {
    // Build a temporary mesh to collect all nodes and elements in the modal assembly because it happens
    // that the boundary nodes are added in the boundary 'meshlist' whereas their associated elements might
    // be in the 'internal_meshlist', leading to a mess in the mass computation.
    auto mmesh_bou_int = chrono_types::make_shared<ChMesh>();
    // collect boundary mesh
    for (auto& item : meshlist) {
        if (auto mesh = std::dynamic_pointer_cast<ChMesh>(item)) {
            for (auto& node : mesh->GetNodes())
                mmesh_bou_int->AddNode(node);
            for (auto& ele : mesh->GetElements())
                mmesh_bou_int->AddElement(ele);
        }
    }
    // collect internal mesh
    for (auto& item : internal_meshlist) {
        if (auto mesh = std::dynamic_pointer_cast<ChMesh>(item)) {
            for (auto& node : mesh->GetNodes())
                mmesh_bou_int->AddNode(node);
            for (auto& ele : mesh->GetElements())
                mmesh_bou_int->AddElement(ele);
        }
    }

    double mass_total = 0;
    ChVector<> mass_weighted_radius(0);
    ChMatrix33<> inertial_total(0);

    // for boundary bodies
    for (auto& body : bodylist) {
        if (body->IsActive()) {
            mass_total += body->GetMass();
            mass_weighted_radius += body->GetMass() * body->GetPos();
            inertial_total +=
                body->GetInertia() + body->GetMass() * (body->GetPos().Length2() * ChMatrix33<>(1.0) -
                                                        body->GetPos().eigen() * body->GetPos().eigen().transpose());
        }
    }
    // for internal bodies
    for (auto& body : internal_bodylist) {
        if (body->IsActive()) {
            mass_total += body->GetMass();
            mass_weighted_radius += body->GetMass() * body->GetPos();
            inertial_total +=
                body->GetInertia() + body->GetMass() * (body->GetPos().Length2() * ChMatrix33<>(1.0) -
                                                        body->GetPos().eigen() * body->GetPos().eigen().transpose());
        }
    }

    // compute the mass properties of the mesh
    double mmesh_mass = 0;
    ChVector<> mmesh_cog(0);
    ChMatrix33<> mmesh_inertia(0);
    mmesh_bou_int->ComputeMassProperties(mmesh_mass, mmesh_cog, mmesh_inertia);

    // mass property for the entire assembly
    mass_total += mmesh_mass;
    mass_weighted_radius += mmesh_mass * mmesh_cog;
    inertial_total += mmesh_inertia;

    ChVector<> cog_x;
    if (mass_total) {
        cog_x = mass_weighted_radius / mass_total;

        this->cog_frame.SetPos(cog_x);

        // The inertia tensor about cog, but still aligned with the absolute frame
        ChMatrix33<> inertia_cog = inertial_total - mass_total * (cog_x.Length2() * ChMatrix33<>(1.0) -
                                                                  cog_x.eigen() * cog_x.eigen().transpose());
        Eigen::EigenSolver<Eigen::MatrixXd> es(inertia_cog);
        ChVector<> prin_inertia = es.eigenvalues().real();  // principal moments of inertia
        ChMatrix33<> prin_axis = es.eigenvectors().real();  // principal axes of inertia
        ChQuaternion qrot = prin_axis.Get_A_quaternion();

        this->cog_frame.SetRot(qrot);

    } else {
        // place at the position of the first boundary body/node of subassembly
        cog_x = this->full_assembly_x.segment(0, 3);

        this->cog_frame.SetPos(cog_x);

        ChQuaternion q_axis = this->full_assembly_x.segment(3, 4);
        this->cog_frame.SetRot(q_axis);
    }
}

void ChModalAssembly::UpdateFloatingFrameOfReference() {
    double fooT;
    ChState x_mod;
    ChStateDelta v_mod;
    x_mod.setZero(this->ncoords, nullptr);
    v_mod.setZero(this->ncoords_w, nullptr);
    this->IntStateGather(0, x_mod, 0, v_mod, fooT);

    if (!this->is_initialized_F) {
        // the floating frame F is initialized at COG in the initial configuration
        this->ComputeMassCenter();

        this->floating_frame_F = this->cog_frame;
        // this->floating_frame_F_old = this->floating_frame_F;

        // store the initial floating frame of reference F0 in the initial configuration
        this->floating_frame_F0 = this->floating_frame_F;

        this->is_initialized_F = true;

    } else {
        // update the configuration of the floating frame F using Newton-Raphson iteration
        // to satisfy the constraint equation: C_F = U^T * M * e_loc = 0.

        int bou_mod_coords_w = this->n_boundary_coords_w + this->n_modes_coords_w;

        auto ComputeResidual_ConstrF = [&](ChVectorDynamic<>& mConstr_F) {
            this->UpdateTransformationMatrix();
            this->ComputeProjectionMatrix();

            ChStateDelta u_locred(bou_mod_coords_w, nullptr);
            ChStateDelta e_locred(bou_mod_coords_w, nullptr);
            ChStateDelta edt_locred(bou_mod_coords_w, nullptr);
            this->GetStateLocal(u_locred, e_locred, edt_locred, "definition");

            // the constraint vector C_F to eliminate the redundant DOFs of the floating frame F
            mConstr_F = this->U_locred_0.transpose() * this->M_red * e_locred;  // of size 6*1, expected to be zero
        };

        int ite_count = 0;
        int NR_limit = 10;
        double tol = 1.e-12;
        bool converged_flag_F = false;

        while (!converged_flag_F && ite_count < NR_limit) {
            ChVectorDynamic<> constr_F(6);
            ComputeResidual_ConstrF(constr_F);

            // Jacobian of the constraint vector C_F w.r.t. the floating frame F
            ChMatrixDynamic<> jac_F;
            jac_F.setZero(6, 6);
            jac_F = -this->U_locred_0.transpose() * this->M_red * this->U_locred * this->P_F.transpose();

            ChVectorDynamic<> delta_F(6);
            delta_F = jac_F.colPivHouseholderQr().solve(-constr_F);

            ChVector<> pos_F = this->floating_frame_F.GetPos() + delta_F.head(3);

            ChQuaternion<> incr_rotF(QNULL);
            incr_rotF.Q_from_Rotv(ChVector<>(delta_F.tail(3)));  // rot.in local basis - as in system wide vectors
            ChQuaternion<> rot_F = this->floating_frame_F.GetRot() * incr_rotF;

            this->floating_frame_F.SetPos(pos_F);
            this->floating_frame_F.SetRot(rot_F);

            if (constr_F.norm() < tol)
                converged_flag_F = true;

            ite_count++;

            if (!converged_flag_F && ite_count == NR_limit)
                GetLog() << "--->>> Warning: NR iteration to search for F might be divergent...\n";
        }

        // update the velocity of the floating frame F
        ChVectorDynamic<> vel_F(6);  // qdt_F
        vel_F = this->P_F * this->Q_0 * (this->P_W.transpose() * v_mod);
        this->floating_frame_F.SetPos_dt(vel_F.head(3));
        this->floating_frame_F.SetWvel_loc(vel_F.tail(3));

        // update again for safe
        this->UpdateTransformationMatrix();
        this->ComputeProjectionMatrix();
    }

    if (verbose) {
        ChVector<> pos_F = this->floating_frame_F.GetPos();
        ChVector<> theta_F = this->floating_frame_F.GetRot().Q_to_Rotv() * CH_C_RAD_TO_DEG;
        GetLog() << "this->floating_frame_F: pos: " << pos_F.x() << "  " << pos_F.y() << "  " << pos_F.z()
                 << "  rot: " << theta_F.x() << "  " << theta_F.y() << "  " << theta_F.z() << "\n";
    }

    // store the old configuration of the floating frame F
    // this->floating_frame_F_old = this->floating_frame_F;
}

void ChModalAssembly::UpdateTransformationMatrix() {
    int bou_mod_coords = this->n_boundary_coords + this->n_modes_coords_w;
    int bou_mod_coords_w = this->n_boundary_coords_w + this->n_modes_coords_w;

    // fetch the state snapshot (modal reduced)
    double fooT;
    ChState x_mod;       // =[qB; eta]
    ChStateDelta v_mod;  // =[qB_dt; eta_dt]
    x_mod.setZero(bou_mod_coords, nullptr);
    v_mod.setZero(bou_mod_coords_w, nullptr);
    this->IntStateGather(0, x_mod, 0, v_mod, fooT);

    L_B.setIdentity(n_boundary_coords_w, n_boundary_coords_w);
    for (int i_bou = 0; i_bou < n_boundary_coords_w / 6; i_bou++) {
        L_B.block(6 * i_bou, 6 * i_bou, 3, 3) = floating_frame_F.GetA();
    }
    L_I.setIdentity(n_internal_coords_w, n_internal_coords_w);
    for (int i_int = 0; i_int < n_internal_coords_w / 6; i_int++) {
        L_I.block(6 * i_int, 6 * i_int, 3, 3) = floating_frame_F.GetA();
    }

    //  rigid body modes of boudnary bodies and nodes
    Uloc_B.setZero(n_boundary_coords_w, 6);
    for (int i_bou = 0; i_bou < n_boundary_coords_w / 6; i_bou++) {
        Uloc_B.block(6 * i_bou, 0, 3, 3) = ChMatrix33<>(1.0);
        ChVector<> X_B =
            floating_frame_F.GetRot().RotateBack(ChVector<>(x_mod.segment(7 * i_bou, 3)) - floating_frame_F.GetPos());
        Uloc_B.block(6 * i_bou, 3, 3, 3) = -ChStarMatrix33<>(X_B);
        // todo:boundary nodes must have 4 rotational DOFs from quaternion parametrization
        ChQuaternion<> quat_bou = x_mod.segment(7 * i_bou + 3, 4);
        Uloc_B.block(6 * i_bou + 3, 3, 3, 3) = ChMatrix33<>(quat_bou.GetConjugate() * floating_frame_F.GetRot());
    }
    //  rigid body modes of internal bodies and nodes
    if (internal_nodes_update) {
        Uloc_I.setZero(n_internal_coords_w, 6);
        for (int i_int = 0; i_int < n_internal_coords_w / 6; i_int++) {
            Uloc_I.block(6 * i_int, 0, 3, 3) = ChMatrix33<>(1.0);
            ChVector<> X_I = floating_frame_F.GetRot().RotateBack(
                ChVector<>(full_assembly_x.segment(n_boundary_coords + 7 * i_int, 3)) - floating_frame_F.GetPos());
            Uloc_I.block(6 * i_int, 3, 3, 3) = -ChStarMatrix33<>(X_I);
            // todo:internal nodes must have 4 rotational DOFs from quaternion parametrization
            ChQuaternion<> quat_int = full_assembly_x.segment(n_boundary_coords + 7 * i_int + 3, 4);
            Uloc_I.block(6 * i_int + 3, 3, 3, 3) = ChMatrix33<>(quat_int.GetConjugate() * floating_frame_F.GetRot());
        }
    }

    P_W.setIdentity(bou_mod_coords_w, bou_mod_coords_w);
    for (int i_bou = 0; i_bou < n_boundary_coords_w / 6; i_bou++) {
        P_W.block(6 * i_bou, 6 * i_bou, 3, 3) = floating_frame_F.GetA();
    }

    P_F.setIdentity(6, 6);
    P_F.topLeftCorner(3, 3) = floating_frame_F.GetA();
}

void ChModalAssembly::ComputeProjectionMatrix() {
    int bou_mod_coords_w = this->n_boundary_coords_w + this->n_modes_coords_w;

    if (!is_projection_initialized) {
        U_locred.setZero(bou_mod_coords_w, 6);
        U_locred.topRows(n_boundary_coords_w) = Uloc_B;

        this->U_locred_0 = this->U_locred;
        this->Uloc_B_0 = this->Uloc_B;
        if (this->internal_nodes_update) {
            this->Uloc_I_0 = this->Uloc_I;
        }

        Eigen::ColPivHouseholderQR<ChMatrixDynamic<>> UTMU_solver =
            (U_locred_0.transpose() * M_red * U_locred_0).colPivHouseholderQr();
        Q_0.setZero(6, bou_mod_coords_w);
        Q_0 = UTMU_solver.solve(U_locred_0.transpose() * M_red);

        P_parallel_0.setZero(bou_mod_coords_w, bou_mod_coords_w);
        P_parallel_0 = U_locred_0 * Q_0;

        ChMatrixDynamic<> I_bm;
        I_bm.setIdentity(bou_mod_coords_w, bou_mod_coords_w);
        P_perp_0.setZero(bou_mod_coords_w, bou_mod_coords_w);
        P_perp_0 = I_bm - P_parallel_0;

        this->is_projection_initialized = true;
        if (verbose)
            GetLog() << "Projection matrices are initialized.\n";

    } else {
        // U_locred is used to update the floating frame F
        U_locred.setZero(bou_mod_coords_w, 6);
        U_locred.topRows(n_boundary_coords_w) = Uloc_B;
    }
}

void ChModalAssembly::ComputeLocalFullKMCqMatrix(ChSparseMatrix& full_M,
                                                 ChSparseMatrix& full_K,
                                                 ChSparseMatrix& full_Cq) {
    // todo: to fill the sparse P_BI in a more straightforward and efficient way
    ChMatrixDynamic<> P_BI;
    P_BI.setIdentity(n_boundary_coords_w + n_internal_coords_w, n_boundary_coords_w + n_internal_coords_w);
    for (int i_bou = 0; i_bou < n_boundary_coords_w / 6; i_bou++) {
        P_BI.block(6 * i_bou, 6 * i_bou, 3, 3) = floating_frame_F.GetA();
    }
    for (int i_int = 0; i_int < n_internal_coords_w / 6; i_int++) {
        P_BI.block(n_boundary_coords_w + 6 * i_int, n_boundary_coords_w + 6 * i_int, 3, 3) = floating_frame_F.GetA();
    }
    ChSparseMatrix P_BI_sp = P_BI.sparseView();

    full_M_loc = P_BI_sp.transpose() * full_M * P_BI_sp;
    full_K_loc = P_BI_sp.transpose() * full_K * P_BI_sp;
    full_Cq_loc = full_Cq * P_BI_sp;

    full_M_loc.makeCompressed();
    full_K_loc.makeCompressed();
    full_Cq_loc.makeCompressed();

    // temporarily retrieve the original local damping matrix
    // todo: develop a more reasonable modal damping model
    ChSparseMatrix full_R;
    this->GetSubassemblyDampingMatrix(&full_R);
    full_R_loc = P_BI_sp.transpose() * full_R * P_BI_sp;
    full_R_loc.makeCompressed();
}

void ChModalAssembly::DoModalReduction_Herting(const ChModalDamping& damping_model) {
    // 1) compute eigenvalue and eigenvectors of the full subsystem.
    // It is calculated in the local floating frame of reference F, thus there must be six rigid-body modes.
    // It is expected that the eigenvalues of the six rigid-body modes are zero, but
    // maybe nonzero if the geometrical stiffness matrix Kg is involved, we also have the opportunity
    // to consider the inertial damping and inertial stiffness matrices Ri,Ki respectively.

    assert(this->modes_V.cols() >= 6);  // at least six rigid-body modes are required.

    // K_IIc = [  K_II   Cq_II' ]
    //         [ Cq_II     0    ]
    ChSparseMatrix K_II_loc = full_K_loc.block(this->n_boundary_coords_w, this->n_boundary_coords_w,
                                               this->n_internal_coords_w, this->n_internal_coords_w);

    Eigen::SparseMatrix<double> K_IIc_loc;
    if (this->n_internal_doc_w) {
        ChSparseMatrix Cq_II_loc = full_Cq_loc.block(this->n_boundary_doc_w, this->n_boundary_coords_w,
                                                     this->n_internal_doc_w, this->n_internal_coords_w);
        util_sparse_assembly_2x2symm(K_IIc_loc, K_II_loc, Cq_II_loc);
    } else
        K_IIc_loc = K_II_loc;
    K_IIc_loc.makeCompressed();

    // Matrix of static modes (constrained, so use K_IIc instead of K_II,
    // the original unconstrained Herting reduction is Psi_S = - K_II^{-1} * K_IB
    //
    // Psi_S_C = {Psi_S; Psi_S_LambdaI} = - K_IIc^{-1} * {K_IB ; Cq_IB}
    ChSparseMatrix Cq_IB_loc =
        full_Cq_loc.block(this->n_boundary_doc_w, 0, this->n_internal_doc_w, this->n_boundary_coords_w);
    Psi_S.setZero(this->n_internal_coords_w, this->n_boundary_coords_w);
    ChMatrixDynamic<> Psi_S_C(this->n_internal_coords_w + this->n_internal_doc_w, this->n_boundary_coords_w);
    ChMatrixDynamic<> Psi_S_LambdaI(this->n_internal_doc_w, this->n_boundary_coords_w);

    // avoid computing K_IIc^{-1}, effectively do n times a linear solve:
    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
    solver.analyzePattern(K_IIc_loc);
    solver.factorize(K_IIc_loc);
    ChSparseMatrix K_IB_loc =
        full_K_loc.block(this->n_boundary_coords_w, 0, this->n_internal_coords_w, this->n_boundary_coords_w);
    for (int i = 0; i < this->n_boundary_coords_w; ++i) {
        ChVectorDynamic<> rhs(this->n_internal_coords_w + this->n_internal_doc_w);
        if (this->n_internal_doc_w)
            rhs << K_IB_loc.col(i).toDense(), Cq_IB_loc.col(i).toDense();
        else
            rhs << K_IB_loc.col(i).toDense();

        ChVectorDynamic<> x = solver.solve(rhs.sparseView());

        Psi_S.col(i) = -x.head(this->n_internal_coords_w);
        Psi_S_C.col(i) = -x;
        if (this->n_internal_doc_w)
            Psi_S_LambdaI.col(i) = -x.tail(this->n_internal_doc_w);
    }

    ChVectorDynamic<> c_modes(this->modes_V.cols());
    c_modes.setOnes();

    for (int i_try = 0; i_try < 2; i_try++) {
        // The modal shapes of the first six rigid-body modes solved from the eigensolver might be not accurate,
        // leanding to potential numerical instability. Thus, we construct the rigid-body modal shapes directly.
        this->modes_V.block(0, 0, this->n_boundary_coords_w, 6) = Uloc_B;
        this->modes_V.block(this->n_boundary_coords_w, 0, this->n_internal_coords_w, 6) = Uloc_I;

        for (int i_mode = 0; i_mode < this->modes_V.cols(); ++i_mode) {
            if (c_modes(i_mode))
                // Normalize modes_V to improve the condition of M_red.
                // When i_try==0, c_modes==1, it doesnot change modes_V, but tries to obtain M_red and then find the
                // suitable coefficents c_modes;
                // When i_try==1, c_modes works to improve the condition of M_red for the sake of numerical stability.
                this->modes_V.col(i_mode) *= c_modes(i_mode);
            else
                this->modes_V.col(i_mode).normalize();
        }

        // ChMatrixDynamic<> check_orthogonality = modes_V.real().transpose() * full_M_loc * modes_V.real();
        // GetLog() << "check_orthogonality:\n" << check_orthogonality << "\n";

        ChMatrixDynamic<> V_B = this->modes_V.block(0, 0, this->n_boundary_coords_w, this->n_modes_coords_w).real();
        ChMatrixDynamic<> V_I =
            this->modes_V.block(this->n_boundary_coords_w, 0, this->n_internal_coords_w, this->n_modes_coords_w).real();

        // Matrix of dynamic modes (V_B and V_I already computed as constrained eigenmodes,
        // but use K_IIc instead of K_II anyway, to reuse K_IIc already factored before)
        //
        // Psi_D_C = {Psi_D; Psi_D_LambdaI} = - K_IIc^{-1} * {(M_IB * V_B + M_II * V_I) ; 0}
        Psi_D.setZero(this->n_internal_coords_w, this->n_modes_coords_w);
        ChMatrixDynamic<> Psi_D_C(this->n_internal_coords_w + this->n_internal_doc_w, this->n_modes_coords_w);
        ChMatrixDynamic<> Psi_D_LambdaI(this->n_internal_doc_w, this->n_modes_coords_w);

        ChSparseMatrix M_II_loc = full_M_loc.block(this->n_boundary_coords_w, this->n_boundary_coords_w,
                                                   this->n_internal_coords_w, this->n_internal_coords_w);
        ChSparseMatrix M_IB_loc =
            full_M_loc.block(this->n_boundary_coords_w, 0, this->n_internal_coords_w, this->n_boundary_coords_w);
        ChMatrixDynamic<> rhs_top = M_IB_loc * V_B + M_II_loc * V_I;
        for (int i = 0; i < this->n_modes_coords_w; ++i) {
            ChVectorDynamic<> rhs(this->n_internal_coords_w + this->n_internal_doc_w);
            if (this->n_internal_doc_w)
                rhs << rhs_top.col(i), Eigen::VectorXd::Zero(this->n_internal_doc_w);
            else
                rhs << rhs_top.col(i);

            ChVectorDynamic<> x = solver.solve(rhs.sparseView());

            Psi_D.col(i) = -x.head(this->n_internal_coords_w);
            Psi_D_C.col(i) = -x;
            if (this->n_internal_doc_w)
                Psi_D_LambdaI.col(i) = -x.tail(this->n_internal_doc_w);
        }

        // Psi = [ I     0    ]
        //       [Psi_S  Psi_D]
        Psi.setZero(this->n_boundary_coords_w + this->n_internal_coords_w,
                    this->n_boundary_coords_w + this->n_modes_coords_w);
        //***TODO*** maybe prefer sparse Psi matrix, especially for upper blocks...

        Psi << Eigen::MatrixXd::Identity(n_boundary_coords_w, n_boundary_coords_w),
            Eigen::MatrixXd::Zero(n_boundary_coords_w, n_modes_coords_w), Psi_S, Psi_D;

        // Modal reduction on the local M K matrices.
        // Now we assume there is no prestress in the initial configuration,
        // so only material mass and stiffness matrices are used here.
        this->M_red = Psi.transpose() * full_M_loc * Psi;
        this->K_red = Psi.transpose() * full_K_loc * Psi;
        this->K_red.block(0, n_boundary_coords_w, n_boundary_coords_w, n_modes_coords_w).setZero();
        this->K_red.block(n_boundary_coords_w, 0, n_modes_coords_w, n_boundary_coords_w).setZero();

        // Maybe also have a reduced Cq matrix......
        ChSparseMatrix Cq_B_loc = full_Cq_loc.topRows(this->n_boundary_doc_w);
        this->Cq_red = Cq_B_loc * Psi;

        // temporarily set reduced damping matrix from the original local matrix
        // todo: develop a more reasonable damping model
        {
            // Initialize the reduced damping matrix
            this->R_red.setZero(this->M_red.rows(), this->M_red.cols());  // default R=0 , zero damping
            // Modal reduction of R damping matrix: compute using user-provided damping model.
            // todo: maybe the Cq_red is necessary for specifying the suitable modal damping ratios.
            // ChModalDampingNone damping_model;
            // damping_model.ComputeR(*this, this->M_red, this->K_red, Psi, this->R_red);
            this->R_red = Psi.transpose() * full_R_loc * Psi;
            this->R_red.block(0, n_boundary_coords_w, n_boundary_coords_w, n_modes_coords_w).setZero();
            this->R_red.block(n_boundary_coords_w, 0, n_modes_coords_w, n_boundary_coords_w).setZero();
        }

        // Find the suitable coefficients 'c_modes' to normalize 'modes_V' to improve the condition number of 'M_red'.
        if (i_try < 1) {
            double expected_mass = this->M_red.diagonal().head(n_boundary_coords_w).mean();
            for (int i_mode = 0; i_mode < this->modes_V.cols(); ++i_mode)
                c_modes(i_mode) =
                    pow(expected_mass / this->M_red(n_boundary_coords_w + i_mode, n_boundary_coords_w + i_mode), 0.5);
        }
    }

    // Reset to zero all the atomic masses of the boundary nodes because now their mass is represented by
    // this->modal_M NOTE! this should be made more generic and future-proof by implementing a virtual method ex.
    // RemoveMass() in all ChPhysicsItem
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
    // to avoid that one could be tempted to plot those eigenmodes, which now are not exactly the ones of the
    // reduced assembly.
    this->modes_damping_ratio.resize(0);
    this->modes_eig.resize(0);
    this->modes_freq.resize(0);
    this->modes_V.resize(0, 0);
}

void ChModalAssembly::DoModalReduction_CraigBamption(const ChModalDamping& damping_model) {
    // 1) compute eigenvalue and eigenvectors of the full subsystem.
    // It is calculated in the local floating frame of reference F, thus there must be six rigid-body modes.
    // It is expected that the eigenvalues of the six rigid-body modes are zero, but
    // maybe nonzero if the geometrical stiffness matrix Kg is involved, we also have the opportunity
    // to consider the inertial damping and inertial stiffness matrices Ri,Ki respectively.

    // K_IIc = [  K_II   Cq_II' ]
    //         [ Cq_II     0    ]
    ChSparseMatrix K_II_loc = full_K_loc.block(this->n_boundary_coords_w, this->n_boundary_coords_w,
                                               this->n_internal_coords_w, this->n_internal_coords_w);

    Eigen::SparseMatrix<double> K_IIc_loc;
    if (this->n_internal_doc_w) {
        ChSparseMatrix Cq_II_loc = full_Cq_loc.block(this->n_boundary_doc_w, this->n_boundary_coords_w,
                                                     this->n_internal_doc_w, this->n_internal_coords_w);
        util_sparse_assembly_2x2symm(K_IIc_loc, K_II_loc, Cq_II_loc);
    } else
        K_IIc_loc = K_II_loc;
    K_IIc_loc.makeCompressed();

    // Matrix of static modes (constrained, so use K_IIc instead of K_II,
    // the original unconstrained Herting reduction is Psi_S = - K_II^{-1} * K_IB
    //
    // Psi_S_C = {Psi_S; Psi_S_LambdaI} = - K_IIc^{-1} * {K_IB ; Cq_IB}
    ChSparseMatrix Cq_IB_loc =
        full_Cq_loc.block(this->n_boundary_doc_w, 0, this->n_internal_doc_w, this->n_boundary_coords_w);
    Psi_S.setZero(this->n_internal_coords_w, this->n_boundary_coords_w);
    ChMatrixDynamic<> Psi_S_C(this->n_internal_coords_w + this->n_internal_doc_w, this->n_boundary_coords_w);
    ChMatrixDynamic<> Psi_S_LambdaI(this->n_internal_doc_w, this->n_boundary_coords_w);

    // avoid computing K_IIc^{-1}, effectively do n times a linear solve:
    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
    solver.analyzePattern(K_IIc_loc);
    solver.factorize(K_IIc_loc);
    ChSparseMatrix K_IB_loc =
        full_K_loc.block(this->n_boundary_coords_w, 0, this->n_internal_coords_w, this->n_boundary_coords_w);
    for (int i = 0; i < this->n_boundary_coords_w; ++i) {
        ChVectorDynamic<> rhs(this->n_internal_coords_w + this->n_internal_doc_w);
        if (this->n_internal_doc_w)
            rhs << K_IB_loc.col(i).toDense(), Cq_IB_loc.col(i).toDense();
        else
            rhs << K_IB_loc.col(i).toDense();

        ChVectorDynamic<> x = solver.solve(rhs.sparseView());

        Psi_S.col(i) = -x.head(this->n_internal_coords_w);
        Psi_S_C.col(i) = -x;
        if (this->n_internal_doc_w)
            Psi_S_LambdaI.col(i) = -x.tail(this->n_internal_doc_w);
    }

    ChVectorDynamic<> c_modes(this->modes_V.cols());
    c_modes.setOnes();

    for (int i_try = 0; i_try < 2; i_try++) {
        for (int i_mode = 0; i_mode < this->modes_V.cols(); ++i_mode) {
            if (c_modes(i_mode))
                // Normalize modes_V to improve the condition of M_red.
                // When i_try==0, c_modes==1, it doesnot change modes_V, but tries to obtain M_red and then find the
                // suitable coefficents c_modes;
                // When i_try==1, c_modes works to improve the condition of M_red for the sake of numerical stability.
                this->modes_V.col(i_mode) *= c_modes(i_mode);
            else
                this->modes_V.col(i_mode).normalize();
        }

        // ChMatrixDynamic<> check_orthogonality = modes_V.real().transpose() * full_M_loc * modes_V.real();
        // GetLog() << "check_orthogonality:\n" << check_orthogonality << "\n";

        ChMatrixDynamic<> V_I = this->modes_V.block(0, 0, this->n_internal_coords_w, this->n_modes_coords_w).real();

        // Matrix of dynamic modes (V_I already computed as constrained eigenmodes,
        // but use K_IIc instead of K_II anyway, to reuse K_IIc already factored before)
        //
        // Psi_D_C = {Psi_D; Psi_D_LambdaI} = - K_IIc^{-1} * {(M_II * V_I) ; 0}
        Psi_D.setZero(this->n_internal_coords_w, this->n_modes_coords_w);
        ChMatrixDynamic<> Psi_D_C(this->n_internal_coords_w + this->n_internal_doc_w, this->n_modes_coords_w);
        ChMatrixDynamic<> Psi_D_LambdaI(this->n_internal_doc_w, this->n_modes_coords_w);

        ChSparseMatrix M_II_loc = full_M_loc.block(this->n_boundary_coords_w, this->n_boundary_coords_w,
                                                   this->n_internal_coords_w, this->n_internal_coords_w);
        ChMatrixDynamic<> rhs_top = M_II_loc * V_I;
        for (int i = 0; i < this->n_modes_coords_w; ++i) {
            ChVectorDynamic<> rhs(this->n_internal_coords_w + this->n_internal_doc_w);
            if (this->n_internal_doc_w)
                rhs << rhs_top.col(i), Eigen::VectorXd::Zero(this->n_internal_doc_w);
            else
                rhs << rhs_top.col(i);

            ChVectorDynamic<> x = solver.solve(rhs.sparseView());

            Psi_D.col(i) = -x.head(this->n_internal_coords_w);
            Psi_D_C.col(i) = -x;
            if (this->n_internal_doc_w)
                Psi_D_LambdaI.col(i) = -x.tail(this->n_internal_doc_w);
        }

        // Psi = [ I     0    ]
        //       [Psi_S  Psi_D]
        Psi.setZero(this->n_boundary_coords_w + this->n_internal_coords_w,
                    this->n_boundary_coords_w + this->n_modes_coords_w);
        //***TODO*** maybe prefer sparse Psi matrix, especially for upper blocks...

        Psi << Eigen::MatrixXd::Identity(n_boundary_coords_w, n_boundary_coords_w),
            Eigen::MatrixXd::Zero(n_boundary_coords_w, n_modes_coords_w), Psi_S, Psi_D;

        // Modal reduction of the M K matrices.
        this->M_red = Psi.transpose() * full_M_loc * Psi;
        this->K_red = Psi.transpose() * full_K_loc * Psi;
        this->K_red.block(0, n_boundary_coords_w, n_boundary_coords_w, n_modes_coords_w).setZero();
        this->K_red.block(n_boundary_coords_w, 0, n_modes_coords_w, n_boundary_coords_w).setZero();

        // Maybe also have a reduced Cq matrix......
        ChSparseMatrix Cq_B_loc = full_Cq_loc.topRows(this->n_boundary_doc_w);
        this->Cq_red = Cq_B_loc * Psi;

        // temporarily set reduced damping matrix from the original local matrix
        // todo: develop a more reasonable damping model
        {
            // Initialize the reduced damping matrix
            this->R_red.setZero(this->M_red.rows(), this->M_red.cols());  // default R=0 , zero damping
            // Modal reduction of R damping matrix: compute using user-provided damping model.
            // todo: maybe the Cq_red is necessary for specifying the suitable modal damping ratios.
            // ChModalDampingNone damping_model;
            // damping_model.ComputeR(*this, this->M_red, this->K_red, Psi, this->R_red);
            this->R_red = Psi.transpose() * full_R_loc * Psi;
            this->R_red.block(0, n_boundary_coords_w, n_boundary_coords_w, n_modes_coords_w).setZero();
            this->R_red.block(n_boundary_coords_w, 0, n_modes_coords_w, n_boundary_coords_w).setZero();
        }

        // Find the suitable coefficients 'c_modes' to normalize 'modes_V' to improve the condition number of 'M_red'.
        if (i_try < 1) {
            double expected_mass = this->M_red.diagonal().head(n_boundary_coords_w).mean();
            for (int i_mode = 0; i_mode < this->modes_V.cols(); ++i_mode)
                c_modes(i_mode) =
                    pow(expected_mass / this->M_red(n_boundary_coords_w + i_mode, n_boundary_coords_w + i_mode), 0.5);
        }
    }

    // Reset to zero all the atomic masses of the boundary nodes because now their mass is represented by
    // this->modal_M NOTE! this should be made more generic and future-proof by implementing a virtual method ex.
    // RemoveMass() in all ChPhysicsItem
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
    // to avoid that one could be tempted to plot those eigenmodes, which now are not exactly the ones of the
    // reduced assembly.
    this->modes_damping_ratio.resize(0);
    this->modes_eig.resize(0);
    this->modes_freq.resize(0);
    this->modes_V.resize(0, 0);
}

void ChModalAssembly::ComputeInertialKRMmatrix() {
    if (!is_modal)
        return;

    // Inertial mass matrix
    this->M_sup.setZero();
    M_sup = P_W * M_red * P_W.transpose();

    int bou_mod_coords_w = this->n_boundary_coords_w + this->n_modes_coords_w;
    ChMatrixDynamic<> O_F;
    O_F.setZero(bou_mod_coords_w, bou_mod_coords_w);
    for (int i_bou = 0; i_bou < n_boundary_coords_w / 6; i_bou++)
        O_F.block(6 * i_bou, 6 * i_bou, 3, 3) = ChStarMatrix33<>(floating_frame_F.GetWvel_loc());

    // Inertial damping matrix, also called as gyroscopic damping matrix
    this->Ri_sup.setZero();
    Ri_sup = P_W * (O_F * M_red) * P_W.transpose();

    // Inertial stiffness matrix, is zero
    this->Ki_sup.setZero();

    if (!use_linear_inertial_term) {
        int bou_mod_coords = this->n_boundary_coords + this->n_modes_coords_w;

        // fetch the state snapshot (modal reduced)
        double fooT;
        ChState x_mod;       // =[qB; eta]
        ChStateDelta v_mod;  // =[qB_dt; eta_dt]
        x_mod.setZero(bou_mod_coords, nullptr);
        v_mod.setZero(bou_mod_coords_w, nullptr);
        this->IntStateGather(0, x_mod, 0, v_mod, fooT);

        ChStateDelta a_mod;  // =[qB_dtdt; eta_dtdt]
        a_mod.setZero(bou_mod_coords_w, nullptr);
        this->IntStateGatherAcceleration(0, a_mod);

        ChMatrixDynamic<> V;
        ChMatrixDynamic<> O_B;
        V.setZero(bou_mod_coords_w, 6);
        O_B.setZero(bou_mod_coords_w, bou_mod_coords_w);
        for (int i_bou = 0; i_bou < n_boundary_coords_w / 6; i_bou++) {
            V.block(6 * i_bou, 3, 3, 3) =
                ChStarMatrix33<>(floating_frame_F.GetRot().RotateBack(v_mod.segment(6 * i_bou, 3)));
            O_B.block(6 * i_bou + 3, 6 * i_bou + 3, 3, 3) = ChStarMatrix33<>(v_mod.segment(6 * i_bou + 3, 3));
        }

        ChMatrixDynamic<> V_rmom;
        ChMatrixDynamic<> O_thetamom;
        V_rmom.setZero(bou_mod_coords_w, 6);
        O_thetamom.setZero(bou_mod_coords_w, bou_mod_coords_w);
        ChVectorDynamic<> momen = M_red * (P_W.transpose() * v_mod);
        for (int i_bou = 0; i_bou < n_boundary_coords_w / 6; i_bou++) {
            V_rmom.block(6 * i_bou, 3, 3, 3) = ChStarMatrix33<>(momen.segment(6 * i_bou, 3));
            O_thetamom.block(6 * i_bou + 3, 6 * i_bou + 3, 3, 3) = ChStarMatrix33<>(momen.segment(6 * i_bou + 3, 3));
        }

        ChMatrixDynamic<> MVPFQ = M_red * V * P_F * Q_0;
        ChMatrixDynamic<> VrPFQ = V_rmom * P_F * Q_0;

        Ri_sup += P_W * (-M_red * O_F) * P_W.transpose();
        Ri_sup += P_W * (MVPFQ - MVPFQ.transpose()) * P_W.transpose();
        Ri_sup += P_W * (VrPFQ.transpose() - VrPFQ) * P_W.transpose();
        Ri_sup += -O_thetamom + O_B * M_red * P_W.transpose();

        //// Inertial stiffness matrix. Harmful for numerical integration, hence useless.
        // ChVectorDynamic<> f_loc_C = M_red * (P_W.transpose() * a_mod) +
        //                             ((O_F + O_B) * M_red + MVPFQ - MVPFQ.transpose()) * (P_W.transpose() * v_mod);
        // ChMatrixDynamic<> V_iner;
        // ChMatrixDynamic<> V_acc;
        // V_iner.setZero(bou_mod_coords_w, 6);
        // V_acc.setZero(bou_mod_coords_w, 6);
        // for (int i_bou = 0; i_bou < n_boundary_coords_w / 6; i_bou++) {
        //     V_iner.block(6 * i_bou, 3, 3, 3) = ChStarMatrix33<>(f_loc_C.segment(6 * i_bou, 3));
        //     V_acc.block(6 * i_bou, 3, 3, 3) =
        //         ChStarMatrix33<>(floating_frame_F.GetRot().RotateBack(a_mod.segment(6 * i_bou, 3)));
        // }

        // ChVectorDynamic<> h_loc_alpha(6);
        // h_loc_alpha = Q_0 * (P_W.transpose() * v_mod);
        // ChVector<> VF_alpha = h_loc_alpha.head(3);
        // ChMatrixDynamic<> V_alpha;
        // V_alpha.setZero(6, 6);
        // V_alpha.block(0, 3, 3, 3) = -floating_frame_F.GetA() * ChStarMatrix33<>(VF_alpha);

        // ChVectorDynamic<> h_loc_beta(6);
        // h_loc_beta = V.transpose() * M_red * (P_W.transpose() * v_mod);
        // ChVector<> VF_beta = h_loc_beta.head(3);
        // ChMatrixDynamic<> V_beta;
        // V_beta.setZero(6, 6);
        // V_beta.block(0, 3, 3, 3) = ChStarMatrix33<>(floating_frame_F.GetA().transpose() * VF_beta);

        // ChMatrixDynamic<> PFQPWT = P_F * Q_0 * P_W.transpose();
        // Ki_sup = P_W * (M_red * V_acc - V_iner) * PFQPWT +
        //          P_W * ((O_F + O_B) * M_red + MVPFQ - MVPFQ.transpose()) * V * PFQPWT -
        //          P_W * V_rmom * (V_alpha + P_F * Q_0 * V) * PFQPWT - P_W * M_red * O_F * V * PFQPWT +
        //          P_W * Q_0.transpose() * P_F.transpose() * V_rmom.transpose() * V * PFQPWT +
        //          P_W * M_red * V * V_alpha * PFQPWT - P_W * Q_0.transpose() * V_beta * PFQPWT;
    }
}

void ChModalAssembly::ComputeStiffnessMatrix() {
    if (!is_modal)
        return;

    ChMatrixDynamic<> PTKP = P_perp_0.transpose() * K_red * P_perp_0;
    // material stiffness matrix of reduced modal assembly
    Km_sup = P_W * PTKP * P_W.transpose();

    {  // geometric stiffness matrix of reduced modal assembly
        int bou_mod_coords = this->n_boundary_coords + this->n_modes_coords_w;
        int bou_mod_coords_w = this->n_boundary_coords_w + this->n_modes_coords_w;

        double fooT;
        ChState x_mod;       // =[qB; eta]
        ChStateDelta v_mod;  // =[qB_dt; eta_dt]
        x_mod.setZero(bou_mod_coords, nullptr);
        v_mod.setZero(bou_mod_coords_w, nullptr);
        this->IntStateGather(0, x_mod, 0, v_mod, fooT);

        ChStateDelta u_locred(bou_mod_coords_w, nullptr);
        ChStateDelta e_locred(bou_mod_coords_w, nullptr);
        ChStateDelta edt_locred(bou_mod_coords_w, nullptr);
        this->GetStateLocal(u_locred, e_locred, edt_locred, "definition");

        ChVectorDynamic<> g_loc_alpha(bou_mod_coords_w);
        g_loc_alpha = P_perp_0.transpose() * (K_red * e_locred);

        ChMatrixDynamic<> V_F1;
        V_F1.setZero(bou_mod_coords_w, 6);
        ChMatrixDynamic<> V_F2;
        V_F2.setZero(bou_mod_coords_w, 6);
        for (int i_bou = 0; i_bou < n_boundary_coords_w / 6; i_bou++) {
            V_F1.block(6 * i_bou, 3, 3, 3) = ChStarMatrix33<>(g_loc_alpha.segment(6 * i_bou, 3));
            V_F2.block(6 * i_bou, 3, 3, 3) = ChStarMatrix33<>(u_locred.segment(6 * i_bou, 3));
        }

        Kg_sup.setZero();
        Kg_sup = P_W * (-V_F1 + PTKP * V_F2) * P_F * Q_0 * P_W.transpose();
    }
}

void ChModalAssembly::ComputeDampingMatrix() {
    if (!is_modal)
        return;

    // material damping matrix of the reduced modal assembly
    Rm_sup = P_W * (P_perp_0.transpose() * R_red * P_perp_0) * P_W.transpose();
}

void ChModalAssembly::ComputeModalKRMmatrix() {
    ComputeStiffnessMatrix();
    ComputeDampingMatrix();
    ComputeInertialKRMmatrix();

    // modal mass matrix
    this->modal_M = M_sup;

    // modal stiffness matrix
    this->modal_K = Km_sup + Kg_sup;
    if (!use_linear_inertial_term)
        this->modal_K += Ki_sup;

    // modal damping matrix
    this->modal_R = Rm_sup + Ri_sup;

    // modal constraint Jacobian matrix
    // todo: check the formulas, the below code might be wrong.
    this->modal_Cq = Cq_red * P_W.transpose();
}

void ChModalAssembly::SetupModalData(int nmodes_reduction) {
    this->n_modes_coords_w = nmodes_reduction;
    this->Setup();

    // Initialize matrices
    L_B.setZero(n_boundary_coords_w, n_boundary_coords_w);
    L_I.setZero(n_internal_coords_w, n_internal_coords_w);
    P_W.setZero(n_boundary_coords_w + n_modes_coords_w, n_boundary_coords_w + n_modes_coords_w);
    P_F.setZero(6, 6);
    U_locred.setZero(n_boundary_coords_w + n_modes_coords_w, 6);
    U_locred_0.setZero(n_boundary_coords_w + n_modes_coords_w, 6);
    Q_0.setZero(6, n_boundary_coords_w + n_modes_coords_w);
    P_parallel_0.setZero(n_boundary_coords_w + n_modes_coords_w, n_boundary_coords_w + n_modes_coords_w);
    P_perp_0.setZero(n_boundary_coords_w + n_modes_coords_w, n_boundary_coords_w + n_modes_coords_w);
    Uloc_B.setZero(n_boundary_coords_w, 6);
    Uloc_B_0.setZero(n_boundary_coords_w, 6);
    Uloc_I.setZero(n_internal_coords_w, 6);
    Uloc_I_0.setZero(n_internal_coords_w, 6);

    M_red.setZero(n_boundary_coords_w + n_modes_coords_w, n_boundary_coords_w + n_modes_coords_w);
    K_red.setZero(n_boundary_coords_w + n_modes_coords_w, n_boundary_coords_w + n_modes_coords_w);
    R_red.setZero(n_boundary_coords_w + n_modes_coords_w, n_boundary_coords_w + n_modes_coords_w);
    Cq_red.setZero(n_boundary_doc_w, n_boundary_coords_w + n_modes_coords_w);

    Km_sup.setZero(n_boundary_coords_w + n_modes_coords_w, n_boundary_coords_w + n_modes_coords_w);
    Kg_sup.setZero(n_boundary_coords_w + n_modes_coords_w, n_boundary_coords_w + n_modes_coords_w);
    Rm_sup.setZero(n_boundary_coords_w + n_modes_coords_w, n_boundary_coords_w + n_modes_coords_w);
    M_sup.setZero(n_boundary_coords_w + n_modes_coords_w, n_boundary_coords_w + n_modes_coords_w);
    Ri_sup.setZero(n_boundary_coords_w + n_modes_coords_w, n_boundary_coords_w + n_modes_coords_w);
    Ki_sup.setZero(n_boundary_coords_w + n_modes_coords_w, n_boundary_coords_w + n_modes_coords_w);

    if (!modal_variables || (modal_variables->Get_ndof() != this->n_modes_coords_w)) {
        // Initialize ChVariable object used for modal variables
        if (modal_variables)
            delete modal_variables;
        modal_variables = new ChVariablesGenericDiagonalMass(this->n_modes_coords_w);
        modal_variables->GetMassDiagonal()
            .setZero();  // diag. mass not needed, the mass will be defined via this->modal_Hblock

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
        // In fact the ChKblockGeneric modal_Hblock could also handle the not active vars, but the modal_M, K etc
        // are computed for the active-only variables for simplicity in the Herting transformation.
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
        this->custom_F_full.setZero(this->n_boundary_coords_w + this->n_internal_coords_w);
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

bool ChModalAssembly::ComputeModesExternalData(ChSparseMatrix& full_M,
                                               ChSparseMatrix& full_K,
                                               ChSparseMatrix& full_Cq,
                                               const ChModalSolveUndamped& n_modes_settings) {
    m_timer_setup.start();

    // cannot use more modes than n. of tot coords, if so, clamp
    // int nmodes_clamped = ChMin(nmodes, this->ncoords_w);

    // assert(full_M.rows() == this->ncoords_w);
    // assert(full_K.rows() == this->ncoords_w);
    // assert(full_Cq.cols() == this->ncoords_w);

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

    m_timer_setup.stop();

    return true;
}

bool ChModalAssembly::ComputeModesDamped(const ChModalSolveDamped& n_modes_settings) {
    m_timer_setup.start();

    this->SetupInitial();
    this->Setup();
    this->Update();

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
    n_modes_settings.Solve(full_M, full_R, full_K, full_Cq, this->modes_V, this->modes_eig, this->modes_freq,
                           this->modes_damping_ratio);
    m_timer_modal_solver_call.stop();

    m_timer_setup.start();
    this->Setup();
    m_timer_setup.stop();

    return true;
}

void ChModalAssembly::SetFullStateWithModeOverlay(int n_mode, double phase, double amplitude) {
    if (n_mode >= this->modes_V.cols()) {
        this->Update();
        throw ChException("Error: mode " + std::to_string(n_mode) + " is beyond the " +
                          std::to_string(this->modes_V.cols()) + " computed eigenvectors.");
    }

    if (this->modes_V.rows() != this->ncoords_w) {
        this->Update();
        return;
    }

    double fooT = 0;
    ChState assembly_x_new;
    ChStateDelta assembly_v;
    ChStateDelta assembly_Dx_loc;
    ChStateDelta assembly_Dx;

    assembly_x_new.setZero(this->ncoords, nullptr);
    assembly_v.setZero(this->ncoords_w, nullptr);
    assembly_Dx_loc.setZero(this->ncoords_w, nullptr);
    assembly_Dx.setZero(this->ncoords_w, nullptr);

    // pick the nth eigenvector in local reference F
    assembly_Dx_loc = sin(phase) * amplitude * this->modes_V.col(n_mode).real() +
                      cos(phase) * amplitude * this->modes_V.col(n_mode).imag();

    // transform the above local increment in F to the original mixed basis,
    // then it can be accumulated to modes_assembly_x0 to update the position.
    for (int i = 0; i < ncoords_w / 6; ++i) {
        assembly_Dx.segment(6 * i, 3) = floating_frame_F.GetA() * assembly_Dx_loc.segment(6 * i, 3);  // translation
        assembly_Dx.segment(6 * i + 3, 3) = assembly_Dx_loc.segment(6 * i + 3, 3);                    // rotation
    }

    this->IntStateIncrement(0, assembly_x_new, this->modes_assembly_x0, 0,
                            assembly_Dx);  // x += amplitude * eigenvector

    this->IntStateScatter(0, assembly_x_new, 0, assembly_v, fooT, true);

    this->Update();
}

void ChModalAssembly::SetInternalStateWithModes(bool full_update) {
    if (!this->is_modal)
        return;

    int bou_int_coords = this->n_boundary_coords + this->n_internal_coords;
    int bou_int_coords_w = this->n_boundary_coords_w + this->n_internal_coords_w;
    int bou_mod_coords = this->n_boundary_coords + this->n_modes_coords_w;
    int bou_mod_coords_w = this->n_boundary_coords_w + this->n_modes_coords_w;

    if (this->Psi.rows() != bou_int_coords_w || this->Psi.cols() != bou_mod_coords_w)
        return;

    double fooT;
    ChState x_mod;       // =[qB; eta]
    ChStateDelta v_mod;  // =[qB_dt; eta_dt]
    x_mod.setZero(bou_mod_coords, nullptr);
    v_mod.setZero(bou_mod_coords_w, nullptr);
    this->IntStateGather(0, x_mod, 0, v_mod, fooT);

    // Update w.r.t. the undeformed configuration

    ChStateDelta u_locred(bou_mod_coords_w, nullptr);
    ChStateDelta e_locred(bou_mod_coords_w, nullptr);
    ChStateDelta edt_locred(bou_mod_coords_w, nullptr);
    this->GetStateLocal(u_locred, e_locred, edt_locred, "definition");

    ChStateDelta Dx_internal_loc;  // =[delta_qI^bar]
    Dx_internal_loc.setZero(this->n_internal_coords_w, nullptr);
    Dx_internal_loc.segment(0, this->n_internal_coords_w) =
        Psi_S * e_locred.segment(0, n_boundary_coords_w) +
        Psi_D * e_locred.segment(n_boundary_coords_w, n_modes_coords_w);

    ChState assembly_x_new;  // =[qB_new; qI_new]
    assembly_x_new.setZero(bou_int_coords, nullptr);
    assembly_x_new.head(n_boundary_coords) = x_mod.head(n_boundary_coords);

    for (int i_int = 0; i_int < n_internal_coords_w / 6; i_int++) {
        int offset_x = n_boundary_coords + 7 * i_int;
        ChVector<> r_IF0 = floating_frame_F0.GetA().transpose() *
                           (modes_assembly_x0.segment(offset_x, 3) - floating_frame_F0.GetPos().eigen());
        ChVector<> r_I =
            floating_frame_F.GetPos() + floating_frame_F.GetA() * (r_IF0 + Dx_internal_loc.segment(6 * i_int, 3));
        assembly_x_new.segment(offset_x, 3) = r_I.eigen();

        ChQuaternion<> q_delta;
        q_delta.Q_from_Rotv(Dx_internal_loc.segment(6 * i_int + 3, 3));
        ChQuaternion<> quat_int0 = modes_assembly_x0.segment(offset_x + 3, 4);
        ChQuaternion<> q_refrot = floating_frame_F0.GetRot().GetConjugate() * quat_int0;
        // ChQuaternion<> quat_int = floating_frame_F.GetRot() * q_delta * q_refrot;
        ChQuaternion<> quat_int =
            floating_frame_F.GetRot() * floating_frame_F0.GetRot().GetConjugate() * quat_int0 * q_delta;
        assembly_x_new.segment(offset_x + 3, 4) = quat_int.eigen();
    }

    ChStateDelta assembly_v_new;  // =[qB_dt; qI_dt]
    assembly_v_new.setZero(bou_int_coords_w, nullptr);
    assembly_v_new.segment(0, n_boundary_coords_w) = v_mod.segment(0, n_boundary_coords_w);
    assembly_v_new.segment(n_boundary_coords_w, n_internal_coords_w) =
        L_I * (Psi_S * (L_B.transpose() * v_mod.segment(0, n_boundary_coords_w)) +
               Psi_D * v_mod.segment(n_boundary_coords_w, n_modes_coords_w));

    bool needs_temporary_bou_int = this->is_modal;
    if (needs_temporary_bou_int)
        this->is_modal = false;

    // scatter to internal nodes only and update them
    unsigned int displ_x = 0 - this->offset_x;
    unsigned int displ_v = 0 - this->offset_w;
    double T = this->GetChTime();
    for (auto& body : internal_bodylist) {
        if (body->IsActive())
            body->IntStateScatter(displ_x + body->GetOffset_x(), assembly_x_new, displ_v + body->GetOffset_w(),
                                  assembly_v_new, T, full_update);
        else
            body->Update(T, full_update);
    }
    for (auto& mesh : internal_meshlist) {
        mesh->IntStateScatter(displ_x + mesh->GetOffset_x(), assembly_x_new, displ_v + mesh->GetOffset_w(),
                              assembly_v_new, T, full_update);
    }
    for (auto& link : internal_linklist) {
        if (link->IsActive())
            link->IntStateScatter(displ_x + link->GetOffset_x(), assembly_x_new, displ_v + link->GetOffset_w(),
                                  assembly_v_new, T, full_update);
        else
            link->Update(T, full_update);
    }
    for (auto& item : internal_otherphysicslist) {
        if (item->IsActive())
            item->IntStateScatter(displ_x + item->GetOffset_x(), assembly_x_new, displ_v + item->GetOffset_w(),
                                  assembly_v_new, T, full_update);
    }

    if (needs_temporary_bou_int)
        this->is_modal = true;

    // store the full state for the computation in next time step
    this->full_assembly_x = assembly_x_new;
}

void ChModalAssembly::SetFullStateReset() {
    if (this->modes_assembly_x0.rows() != this->ncoords)
        return;

    double fooT = 0;
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
    assert(!std::dynamic_pointer_cast<ChShaft>(item));
    assert(!std::dynamic_pointer_cast<ChLinkBase>(item));
    assert(!std::dynamic_pointer_cast<ChMesh>(item));
    assert(std::find(std::begin(internal_otherphysicslist), std::end(internal_otherphysicslist), item) ==
           internal_otherphysicslist.end());
    assert(item->GetSystem() == nullptr);  // should remove from other system before adding here

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
    // M->makeCompressed();
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
    // K->makeCompressed();
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
    // R->makeCompressed();
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
    // Cq->makeCompressed();
}

void ChModalAssembly::DumpSubassemblyMatrices(bool save_M,
                                              bool save_K,
                                              bool save_R,
                                              bool save_Cq,
                                              const std::string& path) {
    const char* numformat = "%.12g";

    if (save_M) {
        ChSparseMatrix mM;
        this->GetSubassemblyMassMatrix(&mM);
        ChStreamOutAsciiFile file_M(path + "_M.dat");
        file_M.SetNumFormat(numformat);
        StreamOutSparseMatlabFormat(mM, file_M);
    }
    if (save_K) {
        ChSparseMatrix mK;
        this->GetSubassemblyStiffnessMatrix(&mK);
        ChStreamOutAsciiFile file_K(path + "_K.dat");
        file_K.SetNumFormat(numformat);
        StreamOutSparseMatlabFormat(mK, file_K);
    }
    if (save_R) {
        ChSparseMatrix mR;
        this->GetSubassemblyDampingMatrix(&mR);
        ChStreamOutAsciiFile file_R(path + "_R.dat");
        file_R.SetNumFormat(numformat);
        StreamOutSparseMatlabFormat(mR, file_R);
    }
    if (save_Cq) {
        ChSparseMatrix mCq;
        this->GetSubassemblyConstraintJacobianMatrix(&mCq);
        ChStreamOutAsciiFile file_Cq(path + "_Cq.dat");
        file_Cq.SetNumFormat(numformat);
        StreamOutSparseMatlabFormat(mCq, file_Cq);
    }
}

// -----------------------------------------------------------------------------

void ChModalAssembly::SetSystem(ChSystem* m_system) {
    ChAssembly::SetSystem(m_system);  // parent

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
    ChAssembly::SyncCollisionModels();  // parent

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
    ChAssembly::SetupInitial();  // parent

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
    ChAssembly::Setup();  // parent

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
    n_boundary_sysvars = nsysvars;
    n_boundary_sysvars_w = nsysvars_w;
    n_boundary_dof = ndof;

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
        if (body->GetBodyFixed()) {
            // throw ChException("Cannot use a fixed body as internal");
        } else if (body->GetSleeping()) {
            // throw ChException("Cannot use a sleeping body as internal");
        } else {
            n_internal_bodies++;

            body->SetOffset_x(this->offset_x + n_boundary_coords + n_internal_coords);
            body->SetOffset_w(this->offset_w + n_boundary_coords_w + n_internal_coords_w);
            body->SetOffset_L(this->offset_L + n_boundary_doc_w + n_internal_doc_w);

            body->Setup();  // currently, no-op

            n_internal_coords += body->GetDOF();
            n_internal_coords_w += body->GetDOF_w();
            n_internal_doc_w += body->GetDOC();  // not really needed since ChBody introduces no constraints
        }
    }

    for (auto& link : internal_linklist) {
        if (link->IsActive()) {
            n_internal_links++;

            link->SetOffset_x(this->offset_x + n_boundary_coords + n_internal_coords);
            link->SetOffset_w(this->offset_w + n_boundary_coords_w + n_internal_coords_w);
            link->SetOffset_L(this->offset_L + n_boundary_doc_w + n_internal_doc_w);

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

        mesh->SetOffset_x(this->offset_x + n_boundary_coords + n_internal_coords);
        mesh->SetOffset_w(this->offset_w + n_boundary_coords_w + n_internal_coords_w);
        mesh->SetOffset_L(this->offset_L + n_boundary_doc_w + n_internal_doc_w);

        mesh->Setup();  // compute DOFs and iteratively call Setup for child items

        n_internal_coords += mesh->GetDOF();
        n_internal_coords_w += mesh->GetDOF_w();
        n_internal_doc_w += mesh->GetDOC();
        n_internal_doc_w_C += mesh->GetDOC_c();
        n_internal_doc_w_D += mesh->GetDOC_d();
    }

    for (auto& item : internal_otherphysicslist) {
        n_internal_physicsitems++;

        item->SetOffset_x(this->offset_x + n_boundary_coords + n_internal_coords);
        item->SetOffset_w(this->offset_w + n_boundary_coords_w + n_internal_coords_w);
        item->SetOffset_L(this->offset_L + n_boundary_doc_w + n_internal_doc_w);

        item->Setup();

        n_internal_coords += item->GetDOF();
        n_internal_coords_w += item->GetDOF_w();
        n_internal_doc_w += item->GetDOC();
        n_internal_doc_w_C += item->GetDOC_c();
        n_internal_doc_w_D += item->GetDOC_d();
    }

    n_internal_doc = n_internal_doc_w + n_internal_bodies;  // number of constraints including quaternion constraints.
    n_internal_sysvars =
        n_internal_coords + n_internal_doc;  // total number of variables (coordinates + lagrangian multipliers)
    n_internal_sysvars_w = n_internal_coords_w + n_internal_doc_w;  // total number of variables (with 6 dof per body)
    n_internal_dof = n_internal_coords_w - n_internal_doc_w;

    this->custom_F_full.setZero(this->n_boundary_coords_w + this->n_internal_coords_w);

    // For the modal part:
    //

    // (nothing to count)

    // For the entire assembly:
    //

    if (this->is_modal == false) {
        ncoords = n_boundary_coords + n_internal_coords;
        ncoords_w = n_boundary_coords_w + n_internal_coords_w;
        ndoc = n_boundary_doc + n_internal_doc;
        ndoc_w = n_boundary_doc_w + n_internal_doc_w;
        ndoc_w_C = n_boundary_doc_w_C + n_internal_doc_w_C;
        ndoc_w_D = n_boundary_doc_w_D + n_internal_doc_w_D;
        nsysvars = n_boundary_sysvars + n_internal_sysvars;
        nsysvars_w = n_boundary_sysvars_w + n_internal_sysvars_w;
        ndof = n_boundary_dof + n_internal_dof;
        nbodies += n_internal_bodies;
        nlinks += n_internal_links;
        nmeshes += n_internal_meshes;
        nphysicsitems += n_internal_physicsitems;
    } else {
        ncoords = n_boundary_coords + n_modes_coords_w;  // no need for a n_modes_coords, same as n_modes_coords_w
        ncoords_w = n_boundary_coords_w + n_modes_coords_w;
        ndoc = n_boundary_doc;
        ndoc_w = n_boundary_doc_w;
        ndoc_w_C = n_boundary_doc_w_C;
        ndoc_w_D = n_boundary_doc_w_D;
        nsysvars = n_boundary_sysvars + n_modes_coords_w;  // no need for a n_modes_coords, same as n_modes_coords_w
        nsysvars_w = n_boundary_sysvars_w + n_modes_coords_w;
        ndof = n_boundary_dof + n_modes_coords_w;

        this->custom_F_modal.setZero(this->n_modes_coords_w);
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
        for (int ip = 0; ip < (int)internal_meshlist.size(); ++ip) {
            internal_meshlist[ip]->Update(ChTime, update_assets);
        }
        for (int ip = 0; ip < (int)internal_otherphysicslist.size(); ++ip) {
            internal_otherphysicslist[ip]->Update(ChTime, update_assets);
        }
        for (int ip = 0; ip < (int)internal_linklist.size(); ++ip) {
            internal_linklist[ip]->Update(ChTime, update_assets);
        }

        if (m_custom_F_full_callback)
            m_custom_F_full_callback->evaluate(this->custom_F_full, *this);
    } else {
        // If in modal reduction mode, the internal parts would not be updated (actually, these could even be
        // removed) However one still might want to see the internal nodes "moving" during animations,
        if (this->internal_nodes_update)
            this->SetInternalStateWithModes(update_assets);

        if (m_custom_F_modal_callback)
            m_custom_F_modal_callback->evaluate(this->custom_F_modal, *this);

        if (m_custom_F_full_callback)
            m_custom_F_full_callback->evaluate(this->custom_F_full, *this);

        // always update the floating frame F if possible
        this->UpdateFloatingFrameOfReference();
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
    } else {
        this->modal_q_dt.setZero(this->n_modes_coords_w);
        this->modal_q_dtdt.setZero(this->n_modes_coords_w);
    }
}

void ChModalAssembly::GetStateLocal(ChStateDelta& u_locred,
                                    ChStateDelta& e_locred,
                                    ChStateDelta& edt_locred,
                                    const std::string& opt) {
    if (is_modal == false) {
        // to do? not useful for the moment.
        return;
    } else {
        int bou_mod_coords = this->n_boundary_coords + this->n_modes_coords_w;
        int bou_mod_coords_w = this->n_boundary_coords_w + this->n_modes_coords_w;

        u_locred.setZero(bou_mod_coords_w, nullptr);    // =u_locred =P_W^T*[\delta qB; \delta eta]
        e_locred.setZero(bou_mod_coords_w, nullptr);    // =e_locred =[qB^bar; eta]
        edt_locred.setZero(bou_mod_coords_w, nullptr);  // =edt_locred =[qB^bar_dt; eta_dt]

        // fetch the state snapshot (modal reduced)
        double fooT;
        ChState x_mod;       // =[qB; eta]
        ChStateDelta v_mod;  // =[qB_dt; eta_dt]
        x_mod.setZero(bou_mod_coords, nullptr);
        v_mod.setZero(bou_mod_coords_w, nullptr);
        this->IntStateGather(0, x_mod, 0, v_mod, fooT);

        u_locred.tail(n_modes_coords_w) = modal_q;
        for (int i_bou = 0; i_bou < n_boundary_coords_w / 6; i_bou++) {
            // Method 1. Computed with respect to the initial configuration
            u_locred.segment(6 * i_bou, 3) =
                floating_frame_F.GetRot().RotateBack(x_mod.segment(7 * i_bou, 3)).eigen() -
                floating_frame_F0.GetRot().RotateBack(modes_assembly_x0.segment(7 * i_bou, 3)).eigen();

            ChQuaternion<> q_F0 = floating_frame_F0.GetRot();
            ChQuaternion<> q_F = floating_frame_F.GetRot();
            ChQuaternion<> q_B0 = modes_assembly_x0.segment(7 * i_bou + 3, 4);
            ChQuaternion<> q_B = x_mod.segment(7 * i_bou + 3, 4);
            ChQuaternion<> rel_q = q_B0.GetConjugate() * q_F0 * q_F.GetConjugate() * q_B;

            // Method 2. Computed with respect to the last converged configuration
            /*u_locred.segment(6 * i_bou, 3) =
               floating_frame_F.GetRot().RotateBack(x_mod.segment(7 * i_bou, 3)).eigen() -
               floating_frame_F_old.GetRot().RotateBack(full_assembly_x_old.segment(7 * i_bou, 3)).eigen();

            ChQuaternion<> q_F0 = floating_frame_F_old.GetRot();
            ChQuaternion<> q_F = floating_frame_F.GetRot();
            ChQuaternion<> q_B0 = full_assembly_x_old.segment(7 * i_bou + 3, 4);
            ChQuaternion<> q_B = x_mod.segment(7 * i_bou + 3, 4);
            ChQuaternion<> rel_q = q_B0.GetConjugate() * q_F0 * q_F.GetConjugate() * q_B;*/

            // ChQuaternion<> rel_q = q_F.GetConjugate() * q_B * q_B0.GetConjugate() * q_F0;
            // u_locred.segment(6 * i_bou + 3, 3) = rel_q.Q_to_Rotv().eigen();

            double delta_rot_angle;
            ChVector<> delta_rot_dir;
            rel_q.Q_to_AngAxis(delta_rot_angle, delta_rot_dir);
            u_locred.segment(6 * i_bou + 3, 3) = delta_rot_angle * delta_rot_dir.eigen();
        }

        if (opt == "definition") {
            // method 1: computed according to the definition

            // local elastic displacement
            e_locred.tail(n_modes_coords_w) = x_mod.segment(n_boundary_coords, n_modes_coords_w);
            for (int i_bou = 0; i_bou < n_boundary_coords_w / 6; i_bou++) {
                ChVector<> r_B = x_mod.segment(7 * i_bou, 3);
                ChVector<> r_BF_0 = floating_frame_F0.GetRot().RotateBack(
                    (modes_assembly_x0.segment(7 * i_bou, 3) - floating_frame_F0.GetPos().eigen()));
                e_locred.segment(6 * i_bou, 3) =
                    (floating_frame_F.GetRot().RotateBack((r_B - floating_frame_F.GetPos())) - r_BF_0).eigen();

                ChQuaternion<> quat_bou = x_mod.segment(7 * i_bou + 3, 4);
                ChQuaternion<> quat_bou0 = modes_assembly_x0.segment(7 * i_bou + 3, 4);
                ChQuaternion<> q_delta = quat_bou0.GetConjugate() * floating_frame_F0.GetRot() *
                                         floating_frame_F.GetRot().GetConjugate() * quat_bou;
                // ChQuaternion<> q_delta = floating_frame_F.GetRot().GetConjugate() * quat_bou *
                // quat_bou0.GetConjugate() * floating_frame_F0.GetRot();
                // e_locred.segment(6 * i_bou + 3, 3) = q_delta.Q_to_Rotv().eigen();

                double delta_rot_angle;
                ChVector<> delta_rot_dir;
                q_delta.Q_to_AngAxis(delta_rot_angle, delta_rot_dir);
                e_locred.segment(6 * i_bou + 3, 3) = delta_rot_angle * delta_rot_dir.eigen();
            }

            // local elastic velocity
            edt_locred.tail(n_modes_coords_w) = v_mod.segment(n_boundary_coords_w, n_modes_coords_w);
            for (int i_bou = 0; i_bou < n_boundary_coords_w / 6; i_bou++) {
                ChVector<> r_B = x_mod.segment(7 * i_bou, 3);
                ChVector<> v_B = v_mod.segment(6 * i_bou, 3);
                ChVector<> r_BF_loc = floating_frame_F.GetRot().RotateBack(r_B - floating_frame_F.GetPos());
                edt_locred.segment(6 * i_bou, 3) =
                    (floating_frame_F.GetRot().RotateBack(v_B - floating_frame_F.GetPos_dt()) +
                     ChStarMatrix33(r_BF_loc) * floating_frame_F.GetWvel_loc())
                        .eigen();

                ChVector<> wloc_B = v_mod.segment(6 * i_bou + 3, 3);
                ChQuaternion<> quat_bou = x_mod.segment(7 * i_bou + 3, 4);
                edt_locred.segment(6 * i_bou + 3, 3) =
                    (wloc_B - quat_bou.RotateBack(floating_frame_F.GetWvel_par())).eigen();
            }

        } else if (opt == "projection") {
            // method 2: using the perpendicular projector to extract the pure elastic deformation and velocity
            //***Below code is more stable?? seems more consistent with the update algorithm of floating frame F

            // local elastic displacement
            e_locred = P_perp_0 * u_locred;

            // local elastic velocity
            edt_locred = P_perp_0 * (P_W.transpose() * v_mod);

        } else {
            GetLog() << "The GetStateLocal() type is specified incorrectly...\n";
            assert(0);
        }
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
            if (item->IsActive())
                item->IntStateGather(displ_x + item->GetOffset_x(), x, displ_v + item->GetOffset_w(), v, T);
        }
    } else {
        x.segment(off_x + this->n_boundary_coords, this->n_modes_coords_w) = this->modal_q;
        v.segment(off_v + this->n_boundary_coords_w, this->n_modes_coords_w) = this->modal_q_dt;

        T = GetChTime();
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
                body->IntStateScatter(displ_x + body->GetOffset_x(), x, displ_v + body->GetOffset_w(), v, T,
                                      full_update);
            else
                body->Update(T, full_update);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntStateScatter(displ_x + mesh->GetOffset_x(), x, displ_v + mesh->GetOffset_w(), v, T, full_update);
        }
        for (auto& item : internal_otherphysicslist) {
            if (item->IsActive())
                item->IntStateScatter(displ_x + item->GetOffset_x(), x, displ_v + item->GetOffset_w(), v, T,
                                      full_update);
            else
                item->Update(T, full_update);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntStateScatter(displ_x + link->GetOffset_x(), x, displ_v + link->GetOffset_w(), v, T,
                                      full_update);
            else
                link->Update(T, full_update);
        }

        if (m_custom_F_full_callback)
            m_custom_F_full_callback->evaluate(this->custom_F_full, *this);
    } else {
        this->modal_q = x.segment(off_x + this->n_boundary_coords, this->n_modes_coords_w);
        this->modal_q_dt = v.segment(off_v + this->n_boundary_coords_w, this->n_modes_coords_w);

        // Update:
        this->Update(full_update);
    }

    ChTime = T;
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
            if (item->IsActive())
                item->IntStateGatherAcceleration(displ_a + item->GetOffset_w(), a);
        }
    } else {
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
        for (auto& mesh : internal_meshlist) {
            mesh->IntStateScatterAcceleration(displ_a + mesh->GetOffset_w(), a);
        }
        for (auto& item : internal_otherphysicslist) {
            if (item->IsActive())
                item->IntStateScatterAcceleration(displ_a + item->GetOffset_w(), a);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntStateScatterAcceleration(displ_a + link->GetOffset_w(), a);
        }
    } else {
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
            if (item->IsActive())
                item->IntStateGatherReactions(displ_L + item->GetOffset_L(), L);
        }
    } else {
        // todo:
        //  there might be reactions in the reduced modal assembly due to the existance of this->modal_Cq
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
        for (auto& mesh : internal_meshlist) {
            mesh->IntStateScatterReactions(displ_L + mesh->GetOffset_L(), L);
        }
        for (auto& item : internal_otherphysicslist) {
            if (item->IsActive())
                item->IntStateScatterReactions(displ_L + item->GetOffset_L(), L);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntStateScatterReactions(displ_L + link->GetOffset_L(), L);
        }
    } else {
        // todo:
        //  there might be reactions in the reduced modal assembly due to the existance of this->modal_Cq
    }
}

void ChModalAssembly::IntStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) {
    ChAssembly::IntStateIncrement(off_x, x_new, x, off_v, Dv);  // parent

    if (is_modal == false) {
        unsigned int displ_x = off_x - this->offset_x;
        unsigned int displ_v = off_v - this->offset_w;

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
            if (item->IsActive())
                item->IntStateIncrement(displ_x + item->GetOffset_x(), x_new, x, displ_v + item->GetOffset_w(), Dv);
        }
    } else {
        x_new.segment(off_x + this->n_boundary_coords, this->n_modes_coords_w) =
            x.segment(off_x + this->n_boundary_coords, this->n_modes_coords_w) +
            Dv.segment(off_v + this->n_boundary_coords_w, this->n_modes_coords_w);
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
            if (item->IsActive())
                item->IntStateGetIncrement(displ_x + item->GetOffset_x(), x_new, x, displ_v + item->GetOffset_w(), Dv);
        }
    } else {
        Dv.segment(off_v + this->n_boundary_coords_w, this->n_modes_coords_w) =
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
            if (item->IsActive())
                item->IntLoadResidual_F(displ_v + item->GetOffset_w(), R, c);
        }

        // Add custom forces (applied to the original non reduced system)
        if (!this->custom_F_full.isZero()) {
            R.segment(off, this->n_boundary_coords_w + this->n_internal_coords_w) += c * this->custom_F_full;
        }
    } else {
        int bou_mod_coords = this->n_boundary_coords + this->n_modes_coords_w;
        int bou_mod_coords_w = this->n_boundary_coords_w + this->n_modes_coords_w;

        // 1-
        // Add elastic forces from current modal deformations
        ChStateDelta u_locred(bou_mod_coords_w, nullptr);
        ChStateDelta e_locred(bou_mod_coords_w, nullptr);
        ChStateDelta edt_locred(bou_mod_coords_w, nullptr);
        this->GetStateLocal(u_locred, e_locred, edt_locred, "definition");

        // note: - sign
        R.segment(off, this->n_boundary_coords_w + this->n_modes_coords_w) -=
            c * (P_W * P_perp_0.transpose() * (this->K_red * e_locred + this->R_red * edt_locred));

        {  // quadratic velocity term
            double fooT;
            ChState x_mod;       // =[qB; eta]
            ChStateDelta v_mod;  // =[qB_dt; eta_dt]
            x_mod.setZero(bou_mod_coords, nullptr);
            v_mod.setZero(bou_mod_coords_w, nullptr);
            this->IntStateGather(0, x_mod, 0, v_mod, fooT);

            ChMatrixDynamic<> O_F;
            O_F.setZero(bou_mod_coords_w, bou_mod_coords_w);
            for (int i_bou = 0; i_bou < (int)(n_boundary_coords_w / 6.); i_bou++)
                O_F.block(6 * i_bou, 6 * i_bou, 3, 3) = ChStarMatrix33<>(floating_frame_F.GetWvel_loc());

            ChMatrixDynamic<> mat_OF = P_W * O_F * M_red * P_W.transpose();
            g_quad.setZero(bou_mod_coords_w);
            g_quad += mat_OF * v_mod;

            if (!use_linear_inertial_term) {
                ChMatrixDynamic<> V;
                ChMatrixDynamic<> O_B;
                V.setZero(bou_mod_coords_w, 6);
                O_B.setZero(bou_mod_coords_w, bou_mod_coords_w);
                for (int i_bou = 0; i_bou < (int)(n_boundary_coords_w / 6.); i_bou++) {
                    V.block(6 * i_bou, 3, 3, 3) =
                        ChStarMatrix33<>(floating_frame_F.GetRot().RotateBack(v_mod.segment(6 * i_bou, 3)));
                    O_B.block(6 * i_bou + 3, 6 * i_bou + 3, 3, 3) = ChStarMatrix33<>(v_mod.segment(6 * i_bou + 3, 3));
                }

                ChMatrixDynamic<> mat_OB = P_W * O_B * M_red * P_W.transpose();
                ChMatrixDynamic<> mat_M = P_W * M_red * V * P_F * Q_0 * P_W.transpose();
                g_quad += mat_OB * v_mod;  // leading to divergence
                g_quad += (mat_M - mat_M.transpose()) * v_mod;
            }

            //// note: - sign
            R.segment(off, this->n_boundary_coords_w + this->n_modes_coords_w) -= c * g_quad;
        }

        // 2-
        // Add custom forces (in modal coordinates)
        if (!this->custom_F_modal.isZero())
            R.segment(off + this->n_boundary_coords_w, this->n_modes_coords_w) += c * this->custom_F_modal;

        // 3-
        // Add custom forces (applied to the original non reduced system, and transformed into reduced)
        if (!this->custom_F_full.isZero()) {
            ChVectorDynamic<> F_red;
            F_red.setZero(this->n_boundary_coords_w + this->n_modes_coords_w);
            F_red.head(n_boundary_coords_w) =
                this->custom_F_full.head(this->n_boundary_coords_w) +
                L_B * Psi_S.transpose() * L_I.transpose() * this->custom_F_full.tail(this->n_internal_coords_w);
            F_red.tail(n_modes_coords_w) =
                Psi_D.transpose() * L_I.transpose() * this->custom_F_full.tail(this->n_internal_coords_w);
            R.segment(off, this->n_boundary_coords_w + this->n_modes_coords_w) += c * F_red;
        }
    }
}

void ChModalAssembly::IntLoadResidual_Mv(const unsigned int off,      ///< offset in R residual
                                         ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                         const ChVectorDynamic<>& w,  ///< the w vector
                                         const double c               ///< a scaling factor
) {
    if (is_modal == false) {
        ChAssembly::IntLoadResidual_Mv(off, R, w, c);  // parent
        unsigned int displ_v = off - this->offset_w;

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
            if (item->IsActive())
                item->IntLoadResidual_Mv(displ_v + item->GetOffset_w(), R, w, c);
        }
    } else {
        ChVectorDynamic<> w_modal = w.segment(off, this->n_boundary_coords_w + this->n_modes_coords_w);
        R.segment(off, this->n_boundary_coords_w + this->n_modes_coords_w) += c * (this->modal_M * w_modal);
    }
}

void ChModalAssembly::IntLoadLumpedMass_Md(const unsigned int off, ChVectorDynamic<>& Md, double& err, const double c) {
    unsigned int displ_v = off - this->offset_w;

    if (is_modal == false) {
        ChAssembly::IntLoadLumpedMass_Md(off, Md, err, c);  // parent

        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntLoadLumpedMass_Md(displ_v + body->GetOffset_w(), Md, err, c);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntLoadLumpedMass_Md(displ_v + link->GetOffset_w(), Md, err, c);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntLoadLumpedMass_Md(displ_v + mesh->GetOffset_w(), Md, err, c);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntLoadLumpedMass_Md(displ_v + item->GetOffset_w(), Md, err, c);
        }
    } else {
        Md.segment(off, this->n_boundary_coords_w + this->n_modes_coords_w) += c * this->modal_M.diagonal();

        // lumping should not be used when modal reduced assembly has full, nondiagonal modal_M
        err += (Md.sum() - Md.diagonal().sum());
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
            if (item->IsActive())
                item->IntLoadResidual_CqL(displ_L + item->GetOffset_L(), R, L, c);
        }
    } else {
        // todo:
        //  there might be residual CqL in the reduced modal assembly
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
            if (item->IsActive())
                item->IntLoadConstraint_C(displ_L + item->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
        }
    } else {
        // todo:
        //  there might be constraint C in the reduced modal assembly
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
            if (item->IsActive())
                item->IntLoadConstraint_Ct(displ_L + item->GetOffset_L(), Qc, c);
        }
    } else {
        // todo:
        //  there might be constraint Ct in the reduced modal assembly
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
            if (item->IsActive())
                item->IntToDescriptor(displ_v + item->GetOffset_w(), v, R, displ_L + item->GetOffset_L(), L, Qc);
        }
    } else {
        this->modal_variables->Get_qb() = v.segment(off_v + this->n_boundary_coords_w, this->n_modes_coords_w);
        this->modal_variables->Get_fb() = R.segment(off_v + this->n_boundary_coords_w, this->n_modes_coords_w);
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
            if (item->IsActive())
                item->IntFromDescriptor(displ_v + item->GetOffset_w(), v, displ_L + item->GetOffset_L(), L);
        }
    } else {
        v.segment(off_v + this->n_boundary_coords_w, this->n_modes_coords_w) = this->modal_variables->Get_qb();
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
    } else {
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
    } else {
        // todo:
        //  there might be constraints for the reduced modal assembly: this->modal_Cq
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
    } else {
        // todo:
        //  there might be constraints for the reduced modal assembly: this->modal_Cq
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
    } else {
        mdescriptor.InsertKblock(&this->modal_Hblock);
    }
}

void ChModalAssembly::KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {
    if (is_modal == false) {
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
    } else {
        ComputeModalKRMmatrix();

        this->modal_Hblock.Get_K() = this->modal_K * Kfactor + this->modal_R * Rfactor + this->modal_M * Mfactor;
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
    /*int version =*/marchive.VersionRead<ChModalAssembly>();

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
