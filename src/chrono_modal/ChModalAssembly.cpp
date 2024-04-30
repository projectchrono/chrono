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

#include <iomanip>

#include "chrono_modal/ChModalAssembly.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

namespace chrono {

using namespace fea;

namespace modal {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChModalAssembly)

ChModalAssembly::ChModalAssembly()
    : modal_variables(nullptr), m_num_coords_modal(0), m_is_model_reduced(false), internal_nodes_update(true) {}

ChModalAssembly::ChModalAssembly(const ChModalAssembly& other) : ChAssembly(other) {
    m_is_model_reduced = other.m_is_model_reduced;
    modal_q = other.modal_q;
    modal_q_dt = other.modal_q_dt;
    modal_q_dtdt = other.modal_q_dtdt;
    internal_nodes_update = other.internal_nodes_update;
    full_forces_internal = other.full_forces_internal;

    //// TODO:  deep copy of the object lists (internal_bodylist, internal_linklist, internal_meshlist,
    /// internal_otherphysicslist)
}

ChModalAssembly::~ChModalAssembly() {
    RemoveAllInternalBodies();
    RemoveAllInternalLinks();
    RemoveAllInternalMeshes();
    RemoveAllInternalOtherPhysicsItems();
    if (this->modal_variables)
        delete this->modal_variables;
}

ChModalAssembly& ChModalAssembly::operator=(ChModalAssembly other) {
    ChModalAssembly tmp(other);
    swap(*this, other);
    return *this;
}

void ChModalAssembly::FlagModelAsReduced() {
    m_is_model_reduced = true;
    Setup();
}

// Note: implement this as a friend function (instead of a member function swap(ChModalAssembly& other)) so that other
// classes that have a ChModalAssembly member (currently only ChSystem) could use it, the same way we use std::swap
// here.
void swap(ChModalAssembly& first, ChModalAssembly& second) {
    using std::swap;
    // swap(first.nbodies, second.nbodies);
    // TODO
}

void ChModalAssembly::Clear() {
    ChAssembly::Clear();  // parent

    RemoveAllInternalBodies();
    RemoveAllInternalLinks();
    RemoveAllInternalMeshes();
    RemoveAllInternalOtherPhysicsItems();

    if (this->modal_variables)
        delete this->modal_variables;
}

// Assembly a sparse matrix by bordering square H with rectangular Cq.
//    HCQ = [ H  Cq' ]
//          [ Cq  0  ]
void util_sparse_assembly_2x2symm(
    Eigen::SparseMatrix<double, Eigen::ColMajor, int>& HCQ,  ///< resulting square sparse matrix (column major)
    const ChSparseMatrix& H,                                 ///< square sparse H matrix, n_v x n_v
    const ChSparseMatrix& Cq)                                ///< rectangular  sparse Cq  n_c x n_v
{
    unsigned int n_v = H.rows();
    unsigned int n_c = Cq.rows();
    HCQ.resize(n_v + n_c, n_v + n_c);
    HCQ.reserve(H.nonZeros() + 2 * Cq.nonZeros());
    HCQ.setZero();

    for (unsigned int k = 0; k < H.outerSize(); ++k)
        for (ChSparseMatrix::InnerIterator it(H, k); it; ++it) {
            HCQ.insert(it.row(), it.col()) = it.value();
        }

    for (unsigned int k = 0; k < Cq.outerSize(); ++k)
        for (ChSparseMatrix::InnerIterator it(Cq, k); it; ++it) {
            HCQ.insert(it.row() + n_v, it.col()) = it.value();  // insert Cq
            HCQ.insert(it.col(), it.row() + n_v) = it.value();  // insert Cq'
        }

    // This seems necessary in Release mode
    HCQ.makeCompressed();

    // NOTE
    // for some reason the HCQ matrix created via .insert() or .elementRef() or triplet insert, is
    // corrupt in Release mode, not in Debug mode. However, when doing a loop like the one below,
    // it repairs it.
    // TODO avoid this bad hack and find the cause of the release/debug difference.
    /*
    for (unsigned int k = 0; k < HCQ.rows(); ++k) {
        for (unsigned int j = 0; j < HCQ.cols(); ++j) {
            auto foo = HCQ.coeffRef(k, j);
            //std::cout << HCQ.coeffRef(k,j) << " ";
        }
    }
    */
}

//---------------------------------------------------------------------------------------
void ChModalAssembly::DoModalReduction(ChSparseMatrix& full_M,
                                       ChSparseMatrix& full_K,
                                       ChSparseMatrix& full_Cq,
                                       const ChModalSolveUndamped& n_modes_settings,
                                       const ChModalDamping& damping_model) {
    if (this->m_is_model_reduced)
        return;

    this->SetupInitial();
    this->Setup();
    this->Update();

    // recover the local M,K,Cq (full_M_loc, full_K_loc, full_Cq_loc) matrices
    // through rotating back to the local frame of F
    this->ComputeLocalFullKMCqMatrix(full_M, full_K, full_Cq);

    if (this->m_modal_reduction_type == ReductionType::HERTING) {
        // 1) compute eigenvalue and eigenvectors
        unsigned int expected_eigs = 0;
        for (auto freq_span : n_modes_settings.freq_spans)
            expected_eigs += freq_span.nmodes;

        if (expected_eigs < 6) {
            std::cout << "*** At least six rigid-body modes are required for the HERTING modal reduction. "
                      << "The default settings are used." << std::endl;
            this->ComputeModesExternalData(this->full_M_loc, this->full_K_loc, this->full_Cq_loc,
                                           ChModalSolveUndamped(6));
        } else
            this->ComputeModesExternalData(this->full_M_loc, this->full_K_loc, this->full_Cq_loc, n_modes_settings);

        // 2) bound ChVariables etc. to the modal coordinates, resize matrices, set as modal mode
        this->FlagModelAsReduced();
        this->SetupModalData(this->m_modal_eigvect.cols());

        if (this->m_verbose) {
            std::cout << "*** Herting reduction is used." << std::endl;
            for (unsigned int i = 0; i < this->m_modal_eigvals.size(); ++i)
                std::cout << " Damped mode n." << i << "  frequency [Hz]: " << this->m_modal_freq(i) << std::endl;
        }

        // 3) compute the transforamtion matrices, also the local rigid-body modes
        this->UpdateTransformationMatrix();

        // 4) do the HERTING reduction as in Sonneville2021
        this->ApplyHertingTransformation(damping_model);

    } else if (this->m_modal_reduction_type == ReductionType::CRAIG_BAMPTON) {
        // 1) retrieve the local M,K,Cq matrices sliced for internal nodes, and then compute eigenvalue and eigenvectors
        ChSparseMatrix full_M_II_loc = this->full_M_loc.block(m_num_coords_vel_boundary, m_num_coords_vel_boundary,
                                                              m_num_coords_vel_internal, m_num_coords_vel_internal);
        ChSparseMatrix full_K_II_loc = this->full_K_loc.block(m_num_coords_vel_boundary, m_num_coords_vel_boundary,
                                                              m_num_coords_vel_internal, m_num_coords_vel_internal);
        ChSparseMatrix full_Cq_II_loc = this->full_Cq_loc.block(m_num_constr_boundary, m_num_coords_vel_boundary,
                                                                m_num_constr_internal, m_num_coords_vel_internal);

        this->ComputeModesExternalData(full_M_II_loc, full_K_II_loc, full_Cq_II_loc, n_modes_settings);

        // 2) bound ChVariables etc. to the modal coordinates, resize matrices, set as modal mode
        this->FlagModelAsReduced();
        this->SetupModalData(this->m_modal_eigvect.cols());

        if (this->m_verbose) {
            std::cout << "*** Craig Bampton reduction is used." << std::endl;
            for (unsigned int i = 0; i < this->m_modal_eigvals.size(); ++i)
                std::cout << " Damped mode n." << i << "  frequency [Hz]: " << this->m_modal_freq(i) << std::endl;
        }

        // 3) compute the transforamtion matrices, also the local rigid-body modes
        this->UpdateTransformationMatrix();

        // 4) do the Craig-Bampton reduction
        this->ApplyCraigBamptonTransformation(damping_model);

    } else {
        std::cout << "*** The modal reduction type is specified incorrectly..." << std::endl;
        assert(0);
    }

    // initialize the projection matrices
    this->ComputeProjectionMatrix();

    // initialize the modal K R M matrices
    this->ComputeModalKRMmatrix();

    // Debug dump data. ***TODO*** remove
    if (this->m_verbose) {
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
        std::ofstream fileCq("dump_modal_Cq.dat");
        fileCq << std::setprecision(12) << std::scientific;
        StreamOut(modal_Cq, fileCq);

        std::ofstream fileM_red("dump_reduced_M.dat");
        fileM_red << std::setprecision(12) << std::scientific;
        StreamOut(M_red, fileM_red);
        std::ofstream fileK_red("dump_reduced_K.dat");
        fileK_red << std::setprecision(12) << std::scientific;
        StreamOut(K_red, fileK_red);
        std::ofstream fileR_red("dump_reduced_R.dat");
        fileR_red << std::setprecision(12) << std::scientific;
        StreamOut(R_red, fileR_red);
        std::ofstream fileCq_red("dump_reduced_Cq.dat");
        fileCq_red << std::setprecision(12) << std::scientific;
        StreamOut(Cq_red, fileCq_red);
    }
}

void ChModalAssembly::DoModalReduction(const ChModalSolveUndamped& n_modes_settings,
                                       const ChModalDamping& damping_model) {
    if (this->m_is_model_reduced)
        return;

    // 1) fetch the full (not reduced) mass and stiffness
    ChSparseMatrix full_M;
    ChSparseMatrix full_K;
    ChSparseMatrix full_Cq;

    this->GetSubassemblyMassMatrix(&full_M);
    this->GetSubassemblyStiffnessMatrix(&full_K);
    this->GetSubassemblyConstraintJacobianMatrix(&full_Cq);

    // 2) compute modal reduction from full_M, full_K
    this->DoModalReduction(full_M, full_K, full_Cq, n_modes_settings, damping_model);
}

void ChModalAssembly::ComputeMassCenterFrame() {
    // Build a temporary mesh to collect all nodes and elements in the modal assembly because it happens
    // that the boundary nodes are added in the boundary 'meshlist' whereas their associated elements might
    // be in the 'internal_meshlist', leading to a mess in the mass computation.
    auto mesh_bou_int = chrono_types::make_shared<ChMesh>();
    // collect boundary mesh
    for (auto& item : meshlist) {
        if (auto mesh = std::dynamic_pointer_cast<ChMesh>(item)) {
            for (auto& node : mesh->GetNodes())
                mesh_bou_int->AddNode(node);
            for (auto& ele : mesh->GetElements())
                mesh_bou_int->AddElement(ele);
        }
    }
    // collect internal mesh
    for (auto& item : internal_meshlist) {
        if (auto mesh = std::dynamic_pointer_cast<ChMesh>(item)) {
            for (auto& node : mesh->GetNodes())
                mesh_bou_int->AddNode(node);
            for (auto& ele : mesh->GetElements())
                mesh_bou_int->AddElement(ele);
        }
    }

    double mass_total = 0;
    ChVector3d mass_weighted_radius(0);
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
    double mesh_mass = 0;
    ChVector3d mesh_cog(0);
    ChMatrix33<> mesh_inertia(0);
    mesh_bou_int->ComputeMassProperties(mesh_mass, mesh_cog, mesh_inertia);

    // mass property for the whole modal assembly
    mass_total += mesh_mass;
    mass_weighted_radius += mesh_mass * mesh_cog;
    inertial_total += mesh_inertia;

    ChVector3d cog_x;
    if (mass_total) {
        cog_x = mass_weighted_radius / mass_total;

        this->cog_frame.SetPos(cog_x);

        // The inertia tensor about cog, but still aligned with the absolute frame
        ChMatrix33<> inertia_cog = inertial_total - mass_total * (cog_x.Length2() * ChMatrix33<>(1.0) -
                                                                  cog_x.eigen() * cog_x.eigen().transpose());
        Eigen::EigenSolver<Eigen::MatrixXd> es(inertia_cog);
        ChVector3d prin_inertia = es.eigenvalues().real();  // principal moments of inertia
        ChMatrix33<> prin_axis = es.eigenvectors().real();  // principal axes of inertia
        ChQuaternion qrot = prin_axis.GetQuaternion();

        this->cog_frame.SetRot(qrot);

    } else {
        // place at the position of the first boundary body/node of this modal assembly
        cog_x = this->m_full_state_x0.segment(0, 3);

        this->cog_frame.SetPos(cog_x);

        ChQuaternion q_axis = this->m_full_state_x0.segment(3, 4);
        this->cog_frame.SetRot(q_axis);
    }
}

void ChModalAssembly::UpdateFloatingFrameOfReference() {
    if (!this->is_initialized) {
        // the floating frame F is initialized at COG in the initial configuration
        this->ComputeMassCenterFrame();

        this->floating_frame_F = this->cog_frame;

        // this->floating_frame_F_old = this->floating_frame_F;

        // store the initial floating frame of reference F0 in the initial configuration
        this->floating_frame_F0 = this->floating_frame_F;

        res_CF.setZero(6);
    }

    // If it is in full state, do nothing.
    if (!m_is_model_reduced)
        return;

    // when it is in the modal reduced state,
    // update the configuration of the floating frame F using Newton-Raphson iteration
    // to satisfy the constraint equation: C_F = U_loc^T * M * e_loc = 0.

    unsigned int num_coords_vel_bou_mod = m_num_coords_vel_boundary + this->m_num_coords_modal;

    auto ComputeResidual_ConstrF = [&](ChVectorDynamic<>& mConstr_F) {
        this->UpdateTransformationMatrix();
        this->ComputeProjectionMatrix();

        ChStateDelta u_locred(num_coords_vel_bou_mod, nullptr);
        ChStateDelta e_locred(num_coords_vel_bou_mod, nullptr);
        ChStateDelta edt_locred(num_coords_vel_bou_mod, nullptr);
        this->GetLocalDeformations(u_locred, e_locred, edt_locred);

        // the constraint vector C_F to eliminate the redundant DOFs of the floating frame F
        mConstr_F = this->U_locred_0.transpose() * this->M_red * e_locred;  // of size 6*1, expected to be zero
    };

    unsigned int ite_count = 0;
    unsigned int NR_limit = 10;
    double tol = 1.e-6 * this->M_red.norm();
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

        ChVector3d pos_F = this->floating_frame_F.GetPos() + delta_F.head(3);

        ChQuaternion<> incr_rotF(QNULL);
        incr_rotF.SetFromRotVec(ChVector3d(delta_F.tail(3)));  // rot.in local basis - as in system wise vectors
        ChQuaternion<> rot_F = this->floating_frame_F.GetRot() * incr_rotF;

        this->floating_frame_F.SetPos(pos_F);
        this->floating_frame_F.SetRot(rot_F);

        if (constr_F.norm() < tol)
            converged_flag_F = true;

        ite_count++;

        if (!converged_flag_F && ite_count == NR_limit)
            std::cout << "--->>> Warning: NR iterations to search for F might be divergent..." << std::endl;
    }

    double fooT;
    ChState x_mod;
    ChStateDelta v_mod;
    x_mod.setZero(m_num_coords_pos, nullptr);
    v_mod.setZero(m_num_coords_vel, nullptr);
    this->IntStateGather(0, x_mod, 0, v_mod, fooT);

    // update the velocity of the floating frame F
    ChVectorDynamic<> vel_F(6);  // qdt_F
    vel_F = this->P_F * this->Q_0 * (this->P_W.transpose() * v_mod);
    this->floating_frame_F.SetPosDt(vel_F.head(3));
    this->floating_frame_F.SetAngVelLocal(vel_F.tail(3));

    // update again for safe
    this->UpdateTransformationMatrix();
    this->ComputeProjectionMatrix();

    ComputeResidual_ConstrF(res_CF);

    if (this->m_verbose) {
        ChVector3d pos_F = this->floating_frame_F.GetPos();
        ChVector3d theta_F = this->floating_frame_F.GetRot().GetRotVec() * CH_RAD_TO_DEG;
        std::cout << "this->floating_frame_F: pos: " << pos_F.x() << "  " << pos_F.y() << "  " << pos_F.z()
                  << "  rot[deg]: " << theta_F.x() << "  " << theta_F.y() << "  " << theta_F.z() << std::endl;
    }

    // store the old configuration of the floating frame F
    // this->floating_frame_F_old = this->floating_frame_F;
}

void ChModalAssembly::UpdateTransformationMatrix() {
    unsigned int num_coords_pos_bou_mod = this->m_num_coords_pos_boundary + this->m_num_coords_modal;
    unsigned int num_coords_vel_bou_mod = this->m_num_coords_vel_boundary + this->m_num_coords_modal;

    // fetch the state snapshot (modal reduced)
    double fooT;
    ChState x_mod;       // =[qB; eta]
    ChStateDelta v_mod;  // =[qB_dt; eta_dt]
    x_mod.setZero(num_coords_pos_bou_mod, nullptr);
    v_mod.setZero(num_coords_vel_bou_mod, nullptr);
    this->IntStateGather(0, x_mod, 0, v_mod, fooT);

    L_B.setIdentity(m_num_coords_vel_boundary, m_num_coords_vel_boundary);
    for (unsigned int i_bou = 0; i_bou < m_num_coords_vel_boundary / 6; i_bou++) {
        L_B.block(6 * i_bou, 6 * i_bou, 3, 3) = floating_frame_F.GetRotMat();
    }
    L_I.setIdentity(m_num_coords_vel_internal, m_num_coords_vel_internal);
    for (unsigned int i_int = 0; i_int < m_num_coords_vel_internal / 6; i_int++) {
        L_I.block(6 * i_int, 6 * i_int, 3, 3) = floating_frame_F.GetRotMat();
    }

    //  rigid body modes of boudnary bodies and nodes
    Uloc_B.setZero(m_num_coords_vel_boundary, 6);
    for (unsigned int i_bou = 0; i_bou < m_num_coords_vel_boundary / 6; i_bou++) {
        Uloc_B.block(6 * i_bou, 0, 3, 3) = ChMatrix33<>(1.0);
        ChVector3d X_B =
            floating_frame_F.GetRot().RotateBack(ChVector3d(x_mod.segment(7 * i_bou, 3)) - floating_frame_F.GetPos());
        Uloc_B.block(6 * i_bou, 3, 3, 3) = -ChStarMatrix33<>(X_B);
        // todo:boundary nodes must have 4 rotational DOFs from quaternion parametrization
        ChQuaternion<> quat_bou = x_mod.segment(7 * i_bou + 3, 4);
        Uloc_B.block(6 * i_bou + 3, 3, 3, 3) = ChMatrix33<>(quat_bou.GetConjugate() * floating_frame_F.GetRot());
    }
    //  rigid body modes of internal bodies and nodes
    if (internal_nodes_update) {
        Uloc_I.setZero(m_num_coords_vel_internal, 6);
        for (unsigned int i_int = 0; i_int < m_num_coords_vel_internal / 6; i_int++) {
            Uloc_I.block(6 * i_int, 0, 3, 3) = ChMatrix33<>(1.0);
            ChVector3d X_I = floating_frame_F.GetRot().RotateBack(
                ChVector3d(m_full_state_x.segment(m_num_coords_pos_boundary + 7 * i_int, 3)) -
                floating_frame_F.GetPos());
            Uloc_I.block(6 * i_int, 3, 3, 3) = -ChStarMatrix33<>(X_I);
            // todo:internal nodes must have 4 rotational DOFs from quaternion parametrization
            ChQuaternion<> quat_int = m_full_state_x.segment(m_num_coords_pos_boundary + 7 * i_int + 3, 4);
            Uloc_I.block(6 * i_int + 3, 3, 3, 3) = ChMatrix33<>(quat_int.GetConjugate() * floating_frame_F.GetRot());
        }
    }

    P_W.setIdentity(num_coords_vel_bou_mod, num_coords_vel_bou_mod);
    for (unsigned int i_bou = 0; i_bou < m_num_coords_vel_boundary / 6; i_bou++) {
        P_W.block(6 * i_bou, 6 * i_bou, 3, 3) = floating_frame_F.GetRotMat();
    }

    P_F.setIdentity(6, 6);
    P_F.topLeftCorner(3, 3) = floating_frame_F.GetRotMat();
}

void ChModalAssembly::ComputeProjectionMatrix() {
    unsigned int num_coords_vel_bou_mod = this->m_num_coords_vel_boundary + this->m_num_coords_modal;

    if (!is_projection_initialized) {
        U_locred.setZero(num_coords_vel_bou_mod, 6);
        U_locred.topRows(m_num_coords_vel_boundary) = Uloc_B;

        this->U_locred_0 = this->U_locred;
        this->Uloc_B_0 = this->Uloc_B;
        if (this->internal_nodes_update) {
            this->Uloc_I_0 = this->Uloc_I;
        }

        Eigen::ColPivHouseholderQR<ChMatrixDynamic<>> UTMU_solver =
            (U_locred_0.transpose() * M_red * U_locred_0).colPivHouseholderQr();
        Q_0.setZero(6, num_coords_vel_bou_mod);
        Q_0 = UTMU_solver.solve(U_locred_0.transpose() * M_red);

        P_parallel_0.setZero(num_coords_vel_bou_mod, num_coords_vel_bou_mod);
        P_parallel_0 = U_locred_0 * Q_0;

        ChMatrixDynamic<> I_bm;
        I_bm.setIdentity(num_coords_vel_bou_mod, num_coords_vel_bou_mod);
        P_perp_0.setZero(num_coords_vel_bou_mod, num_coords_vel_bou_mod);
        P_perp_0 = I_bm - P_parallel_0;

        this->is_projection_initialized = true;
        if (m_verbose)
            std::cout << "Projection matrices are initialized.\n";

    } else {
        // Rigid-body modes U_locred in the deformed configuration are used to update the floating frame F
        U_locred.setZero(num_coords_vel_bou_mod, 6);
        U_locred.topRows(m_num_coords_vel_boundary) = Uloc_B;
    }
}

void ChModalAssembly::ComputeLocalFullKMCqMatrix(ChSparseMatrix& full_M,
                                                 ChSparseMatrix& full_K,
                                                 ChSparseMatrix& full_Cq) {
    // todo: to fill the sparse P_BI in a more straightforward and efficient way
    ChMatrixDynamic<> P_BI;
    P_BI.setIdentity(m_num_coords_vel_boundary + m_num_coords_vel_internal,
                     m_num_coords_vel_boundary + m_num_coords_vel_internal);
    for (unsigned int i_bou = 0; i_bou < m_num_coords_vel_boundary / 6; i_bou++) {
        P_BI.block(6 * i_bou, 6 * i_bou, 3, 3) = floating_frame_F.GetRotMat();
    }
    for (unsigned int i_int = 0; i_int < m_num_coords_vel_internal / 6; i_int++) {
        P_BI.block(m_num_coords_vel_boundary + 6 * i_int, m_num_coords_vel_boundary + 6 * i_int, 3, 3) =
            floating_frame_F.GetRotMat();
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

void ChModalAssembly::ApplyHertingTransformation(const ChModalDamping& damping_model) {
    assert(this->m_modal_eigvect.cols() >= 6);  // at least six rigid-body modes are required.

    // K_IIc = [  K_II   Cq_II' ]
    //         [ Cq_II     0    ]
    ChSparseMatrix K_II_loc = full_K_loc.block(m_num_coords_vel_boundary, m_num_coords_vel_boundary,
                                               m_num_coords_vel_internal, m_num_coords_vel_internal);

    Eigen::SparseMatrix<double> K_IIc_loc;
    if (m_num_constr_internal) {
        ChSparseMatrix Cq_II_loc = full_Cq_loc.block(m_num_constr_boundary, m_num_coords_vel_boundary,
                                                     m_num_constr_internal, m_num_coords_vel_internal);
        util_sparse_assembly_2x2symm(K_IIc_loc, K_II_loc, Cq_II_loc);
    } else
        K_IIc_loc = K_II_loc;
    K_IIc_loc.makeCompressed();

    // Matrix of static modes (constrained, so use K_IIc instead of K_II,
    // the original unconstrained HERTING reduction is Psi_S = - K_II^{-1} * K_IB
    //
    // Psi_S_C = {Psi_S; Psi_S_LambdaI} = - K_IIc^{-1} * {K_IB ; Cq_IB}
    ChSparseMatrix Cq_IB_loc =
        full_Cq_loc.block(m_num_constr_boundary, 0, m_num_constr_internal, m_num_coords_vel_boundary);
    Psi_S.setZero(m_num_coords_vel_internal, m_num_coords_vel_boundary);
    ChMatrixDynamic<> Psi_S_C(m_num_coords_vel_internal + m_num_constr_internal, m_num_coords_vel_boundary);
    ChMatrixDynamic<> Psi_S_LambdaI(m_num_constr_internal, m_num_coords_vel_boundary);

    // avoid computing K_IIc^{-1}, effectively do n times a linear solve:
    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
    solver.analyzePattern(K_IIc_loc);
    solver.factorize(K_IIc_loc);
    ChSparseMatrix K_IB_loc =
        full_K_loc.block(m_num_coords_vel_boundary, 0, m_num_coords_vel_internal, m_num_coords_vel_boundary);
    for (unsigned int i = 0; i < m_num_coords_vel_boundary; ++i) {
        ChVectorDynamic<> rhs(m_num_coords_vel_internal + m_num_constr_internal);
        if (m_num_constr_internal)
            rhs << K_IB_loc.col(i).toDense(), Cq_IB_loc.col(i).toDense();
        else
            rhs << K_IB_loc.col(i).toDense();

        ChVectorDynamic<> x = solver.solve(rhs.sparseView());

        Psi_S.col(i) = -x.head(m_num_coords_vel_internal);
        Psi_S_C.col(i) = -x;
        if (m_num_constr_internal)
            Psi_S_LambdaI.col(i) = -x.tail(m_num_constr_internal);
    }

    ChVectorDynamic<> c_modes(this->m_modal_eigvect.cols());
    c_modes.setOnes();

    for (unsigned int i_try = 0; i_try < 2; i_try++) {
        // The modal shapes of the first six rigid-body modes solved from the eigensolver might be not accurate,
        // leading to potential numerical instability. Thus, we construct the rigid-body modal shapes directly.
        this->m_modal_eigvect.block(0, 0, m_num_coords_vel_boundary, 6) = Uloc_B;
        this->m_modal_eigvect.block(m_num_coords_vel_boundary, 0, m_num_coords_vel_internal, 6) = Uloc_I;

        for (unsigned int i_mode = 0; i_mode < this->m_modal_eigvect.cols(); ++i_mode) {
            if (c_modes(i_mode))
                // Normalize m_modal_eigvect to improve the condition of M_red.
                // When i_try==0, c_modes==1, it doesnot change m_modal_eigvect, but tries to obtain M_red and then find
                // the suitable coefficents c_modes; When i_try==1, c_modes works to improve the condition of M_red for
                // the sake of numerical stability.
                this->m_modal_eigvect.col(i_mode) *= c_modes(i_mode);
            else
                this->m_modal_eigvect.col(i_mode).normalize();
        }

        ChMatrixDynamic<> V_B =
            this->m_modal_eigvect.block(0, 0, m_num_coords_vel_boundary, this->m_num_coords_modal).real();
        ChMatrixDynamic<> V_I =
            this->m_modal_eigvect
                .block(m_num_coords_vel_boundary, 0, m_num_coords_vel_internal, this->m_num_coords_modal)
                .real();

        // Matrix of dynamic modes (V_B and V_I already computed as constrained eigenmodes,
        // but use K_IIc instead of K_II anyway, to reuse K_IIc already factored before)
        //
        // Psi_D_C = {Psi_D; Psi_D_LambdaI} = - K_IIc^{-1} * {(M_IB * V_B + M_II * V_I) ; 0}
        Psi_D.setZero(m_num_coords_vel_internal, this->m_num_coords_modal);
        ChMatrixDynamic<> Psi_D_C(m_num_coords_vel_internal + m_num_constr_internal, this->m_num_coords_modal);
        ChMatrixDynamic<> Psi_D_LambdaI(m_num_constr_internal, this->m_num_coords_modal);

        ChSparseMatrix M_II_loc = full_M_loc.block(m_num_coords_vel_boundary, m_num_coords_vel_boundary,
                                                   m_num_coords_vel_internal, m_num_coords_vel_internal);
        ChSparseMatrix M_IB_loc =
            full_M_loc.block(m_num_coords_vel_boundary, 0, m_num_coords_vel_internal, m_num_coords_vel_boundary);
        ChMatrixDynamic<> rhs_top = M_IB_loc * V_B + M_II_loc * V_I;
        for (unsigned int i = 0; i < this->m_num_coords_modal; ++i) {
            ChVectorDynamic<> rhs(m_num_coords_vel_internal + m_num_constr_internal);
            if (m_num_constr_internal)
                rhs << rhs_top.col(i), Eigen::VectorXd::Zero(m_num_constr_internal);
            else
                rhs << rhs_top.col(i);

            ChVectorDynamic<> x = solver.solve(rhs.sparseView());

            Psi_D.col(i) = -x.head(m_num_coords_vel_internal);
            Psi_D_C.col(i) = -x;
            if (m_num_constr_internal)
                Psi_D_LambdaI.col(i) = -x.tail(m_num_constr_internal);
        }

        // Psi = [ I     0    ]
        //       [Psi_S  Psi_D]
        Psi.setZero(m_num_coords_vel_boundary + m_num_coords_vel_internal,
                    m_num_coords_vel_boundary + this->m_num_coords_modal);
        //***TODO*** maybe prefer sparse Psi matrix, especially for upper blocks...

        Psi << Eigen::MatrixXd::Identity(m_num_coords_vel_boundary, m_num_coords_vel_boundary),
            Eigen::MatrixXd::Zero(m_num_coords_vel_boundary, m_num_coords_modal), Psi_S, Psi_D;

        // Modal reduction on the local M K matrices.
        // Now we assume there is no prestress in the initial configuration,
        // so only material mass and stiffness matrices are used here.
        this->M_red = Psi.transpose() * full_M_loc * Psi;
        this->K_red = Psi.transpose() * full_K_loc * Psi;
        // set the off-diagonal blocks to zero to improve the numerical stability.
        this->K_red.block(0, m_num_coords_vel_boundary, m_num_coords_vel_boundary, m_num_coords_modal).setZero();
        this->K_red.block(m_num_coords_vel_boundary, 0, m_num_coords_modal, m_num_coords_vel_boundary).setZero();

        // Maybe also have a reduced Cq matrix......
        ChSparseMatrix Cq_B_loc = full_Cq_loc.topRows(m_num_constr_boundary);
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
            // set the off-diagonal blocks to zero to improve the numerical stability.
            this->R_red.block(0, m_num_coords_vel_boundary, m_num_coords_vel_boundary, m_num_coords_modal).setZero();
            this->R_red.block(m_num_coords_vel_boundary, 0, m_num_coords_modal, m_num_coords_vel_boundary).setZero();
        }

        // Find the suitable coefficients 'c_modes' to normalize 'm_modal_eigvect' to improve the condition number of
        // 'M_red'.
        if (i_try < 1) {
            double expected_mass = this->M_red.diagonal().head(m_num_coords_vel_boundary).mean();
            for (unsigned int i_mode = 0; i_mode < this->m_modal_eigvect.cols(); ++i_mode)
                c_modes(i_mode) = pow(
                    expected_mass / this->M_red(m_num_coords_vel_boundary + i_mode, m_num_coords_vel_boundary + i_mode),
                    0.5);
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
    this->m_modal_damping_ratios.resize(0);
    this->m_modal_eigvals.resize(0);
    this->m_modal_freq.resize(0);
    this->m_modal_eigvect.resize(0, 0);
}

void ChModalAssembly::ApplyCraigBamptonTransformation(const ChModalDamping& damping_model) {
    // K_IIc = [  K_II   Cq_II' ]
    //         [ Cq_II     0    ]
    ChSparseMatrix K_II_loc = full_K_loc.block(m_num_coords_vel_boundary, m_num_coords_vel_boundary,
                                               m_num_coords_vel_internal, m_num_coords_vel_internal);

    Eigen::SparseMatrix<double> K_IIc_loc;
    if (m_num_constr_internal) {
        ChSparseMatrix Cq_II_loc = full_Cq_loc.block(m_num_constr_boundary, m_num_coords_vel_boundary,
                                                     m_num_constr_internal, m_num_coords_vel_internal);
        util_sparse_assembly_2x2symm(K_IIc_loc, K_II_loc, Cq_II_loc);
    } else
        K_IIc_loc = K_II_loc;
    K_IIc_loc.makeCompressed();

    // Matrix of static modes (constrained, so use K_IIc instead of K_II,
    // the original unconstrained Craig-Bamption reduction is Psi_S = - K_II^{-1} * K_IB
    //
    // Psi_S_C = {Psi_S; Psi_S_LambdaI} = - K_IIc^{-1} * {K_IB ; Cq_IB}
    ChSparseMatrix Cq_IB_loc =
        full_Cq_loc.block(m_num_constr_boundary, 0, m_num_constr_internal, m_num_coords_vel_boundary);
    Psi_S.setZero(m_num_coords_vel_internal, m_num_coords_vel_boundary);
    ChMatrixDynamic<> Psi_S_C(m_num_coords_vel_internal + m_num_constr_internal, m_num_coords_vel_boundary);
    ChMatrixDynamic<> Psi_S_LambdaI(m_num_constr_internal, m_num_coords_vel_boundary);

    // avoid computing K_IIc^{-1}, effectively do n times a linear solve:
    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
    solver.analyzePattern(K_IIc_loc);
    solver.factorize(K_IIc_loc);
    ChSparseMatrix K_IB_loc =
        full_K_loc.block(m_num_coords_vel_boundary, 0, m_num_coords_vel_internal, m_num_coords_vel_boundary);
    for (unsigned int i = 0; i < m_num_coords_vel_boundary; ++i) {
        ChVectorDynamic<> rhs(m_num_coords_vel_internal + m_num_constr_internal);
        if (m_num_constr_internal)
            rhs << K_IB_loc.col(i).toDense(), Cq_IB_loc.col(i).toDense();
        else
            rhs << K_IB_loc.col(i).toDense();

        ChVectorDynamic<> x = solver.solve(rhs.sparseView());

        Psi_S.col(i) = -x.head(m_num_coords_vel_internal);
        Psi_S_C.col(i) = -x;
        if (m_num_constr_internal)
            Psi_S_LambdaI.col(i) = -x.tail(m_num_constr_internal);
    }

    ChVectorDynamic<> c_modes(this->m_modal_eigvect.cols());
    c_modes.setOnes();

    for (unsigned int i_try = 0; i_try < 2; i_try++) {
        for (unsigned int i_mode = 0; i_mode < this->m_modal_eigvect.cols(); ++i_mode) {
            if (c_modes(i_mode))
                // Normalize m_modal_eigvect to improve the condition of M_red.
                // When i_try==0, c_modes==1, it doesnot change m_modal_eigvect, but tries to obtain M_red and then find
                // the suitable coefficents c_modes; When i_try==1, c_modes works to improve the condition of M_red for
                // the sake of numerical stability.
                this->m_modal_eigvect.col(i_mode) *= c_modes(i_mode);
            else
                this->m_modal_eigvect.col(i_mode).normalize();
        }

        ChMatrixDynamic<> V_I =
            this->m_modal_eigvect.block(0, 0, m_num_coords_vel_internal, this->m_num_coords_modal).real();

        // Matrix of dynamic modes (V_I already computed as constrained eigenmodes,
        // but use K_IIc instead of K_II anyway, to reuse K_IIc already factored before)
        //
        // Psi_D_C = {Psi_D; Psi_D_LambdaI} = - K_IIc^{-1} * {(M_II * V_I) ; 0}
        Psi_D.setZero(m_num_coords_vel_internal, this->m_num_coords_modal);
        ChMatrixDynamic<> Psi_D_C(m_num_coords_vel_internal + m_num_constr_internal, this->m_num_coords_modal);
        ChMatrixDynamic<> Psi_D_LambdaI(m_num_constr_internal, this->m_num_coords_modal);

        ChSparseMatrix M_II_loc = full_M_loc.block(m_num_coords_vel_boundary, m_num_coords_vel_boundary,
                                                   m_num_coords_vel_internal, m_num_coords_vel_internal);
        ChMatrixDynamic<> rhs_top = M_II_loc * V_I;
        for (unsigned int i = 0; i < this->m_num_coords_modal; ++i) {
            ChVectorDynamic<> rhs(m_num_coords_vel_internal + m_num_constr_internal);
            if (m_num_constr_internal)
                rhs << rhs_top.col(i), Eigen::VectorXd::Zero(m_num_constr_internal);
            else
                rhs << rhs_top.col(i);

            ChVectorDynamic<> x = solver.solve(rhs.sparseView());

            Psi_D.col(i) = -x.head(m_num_coords_vel_internal);
            Psi_D_C.col(i) = -x;
            if (m_num_constr_internal)
                Psi_D_LambdaI.col(i) = -x.tail(m_num_constr_internal);
        }

        // Psi = [ I     0    ]
        //       [Psi_S  Psi_D]
        Psi.setZero(m_num_coords_vel_boundary + m_num_coords_vel_internal,
                    m_num_coords_vel_boundary + this->m_num_coords_modal);
        //***TODO*** maybe prefer sparse Psi matrix, especially for upper blocks...

        Psi << Eigen::MatrixXd::Identity(m_num_coords_vel_boundary, m_num_coords_vel_boundary),
            Eigen::MatrixXd::Zero(m_num_coords_vel_boundary, m_num_coords_modal), Psi_S, Psi_D;

        // Modal reduction of the M K matrices.
        this->M_red = Psi.transpose() * full_M_loc * Psi;
        this->K_red = Psi.transpose() * full_K_loc * Psi;
        // set the off-diagonal blocks to zero to improve the numerical stability.
        this->K_red.block(0, m_num_coords_vel_boundary, m_num_coords_vel_boundary, m_num_coords_modal).setZero();
        this->K_red.block(m_num_coords_vel_boundary, 0, m_num_coords_modal, m_num_coords_vel_boundary).setZero();

        // Maybe also have a reduced Cq matrix......
        ChSparseMatrix Cq_B_loc = full_Cq_loc.topRows(m_num_constr_boundary);
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
            // set the off-diagonal blocks to zero to improve the numerical stability.
            this->R_red.block(0, m_num_coords_vel_boundary, m_num_coords_vel_boundary, m_num_coords_modal).setZero();
            this->R_red.block(m_num_coords_vel_boundary, 0, m_num_coords_modal, m_num_coords_vel_boundary).setZero();
        }

        // Find the suitable coefficients 'c_modes' to normalize 'm_modal_eigvect' to improve the condition number of
        // 'M_red'.
        if (i_try < 1) {
            double expected_mass = this->M_red.diagonal().head(m_num_coords_vel_boundary).mean();
            for (unsigned int i_mode = 0; i_mode < this->m_modal_eigvect.cols(); ++i_mode)
                c_modes(i_mode) = pow(
                    expected_mass / this->M_red(m_num_coords_vel_boundary + i_mode, m_num_coords_vel_boundary + i_mode),
                    0.5);
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
    this->m_modal_damping_ratios.resize(0);
    this->m_modal_eigvals.resize(0);
    this->m_modal_freq.resize(0);
    this->m_modal_eigvect.resize(0, 0);
}

void ChModalAssembly::ComputeInertialKRMmatrix() {
    if (!m_is_model_reduced)
        return;

    // Inertial mass matrix
    this->M_sup.setZero();
    M_sup = P_W * M_red * P_W.transpose();

    unsigned int num_coords_vel_bou_mod = m_num_coords_vel_boundary + this->m_num_coords_modal;
    ChMatrixDynamic<> O_F;
    O_F.setZero(num_coords_vel_bou_mod, num_coords_vel_bou_mod);
    for (unsigned int i_bou = 0; i_bou < m_num_coords_vel_boundary / 6; i_bou++)
        O_F.block(6 * i_bou, 6 * i_bou, 3, 3) = ChStarMatrix33<>(floating_frame_F.GetAngVelLocal());

    // Inertial damping matrix, also called as gyroscopic damping matrix
    this->Ri_sup.setZero();
    Ri_sup = P_W * (O_F * M_red) * P_W.transpose();

    // Inertial stiffness matrix, is zero
    this->Ki_sup.setZero();

    if (!m_use_linear_inertial_term) {  // The below block of code might cause numerical instabilities in current test
        unsigned int num_coords_pos_bou_mod = this->m_num_coords_pos_boundary + this->m_num_coords_modal;

        // fetch the state snapshot (modal reduced)
        double fooT;
        ChState x_mod;       // =[qB; eta]
        ChStateDelta v_mod;  // =[qB_dt; eta_dt]
        x_mod.setZero(num_coords_pos_bou_mod, nullptr);
        v_mod.setZero(num_coords_vel_bou_mod, nullptr);
        this->IntStateGather(0, x_mod, 0, v_mod, fooT);

        ChStateDelta a_mod;  // =[qB_dtdt; eta_dtdt]
        a_mod.setZero(num_coords_vel_bou_mod, nullptr);
        this->IntStateGatherAcceleration(0, a_mod);

        ChMatrixDynamic<> V;
        V.setZero(num_coords_vel_bou_mod, 6);
        for (unsigned int i_bou = 0; i_bou < m_num_coords_vel_boundary / 6; i_bou++) {
            V.block(6 * i_bou, 3, 3, 3) =
                ChStarMatrix33<>(floating_frame_F.GetRot().RotateBack(v_mod.segment(6 * i_bou, 3)));
        }

        ChMatrixDynamic<> V_rmom;
        V_rmom.setZero(num_coords_vel_bou_mod, 6);
        ChVectorDynamic<> momen = M_red * (P_W.transpose() * v_mod);
        for (unsigned int i_bou = 0; i_bou < m_num_coords_vel_boundary / 6; i_bou++) {
            V_rmom.block(6 * i_bou, 3, 3, 3) = ChStarMatrix33<>(momen.segment(6 * i_bou, 3));
        }
        ChMatrixDynamic<> MVPFQ = M_red * V * P_F * Q_0;
        ChMatrixDynamic<> VrPFQ = V_rmom * P_F * Q_0;

        Ri_sup += P_W * (-M_red * O_F) * P_W.transpose();
        Ri_sup += P_W * (MVPFQ - MVPFQ.transpose()) * P_W.transpose();
        Ri_sup += P_W * (VrPFQ.transpose() - VrPFQ) * P_W.transpose();

        //// Leading to divergence. DO NOT use it.
        // ChMatrixDynamic<> O_B;
        // O_B.setZero(num_coords_vel_bou_mod, num_coords_vel_bou_mod);
        // for (unsigned int i_bou = 0; i_bou < m_num_coords_vel_boundary / 6; i_bou++) {
        //     O_B.block(6 * i_bou + 3, 6 * i_bou + 3, 3, 3) = ChStarMatrix33<>(v_mod.segment(6 * i_bou + 3, 3));
        // }
        // ChMatrixDynamic<> O_thetamom;
        // O_thetamom.setZero(num_coords_vel_bou_mod, num_coords_vel_bou_mod);
        // for (unsigned int i_bou = 0; i_bou < m_num_coords_vel_boundary / 6; i_bou++) {
        //     O_thetamom.block(6 * i_bou + 3, 6 * i_bou + 3, 3, 3) = ChStarMatrix33<>(momen.segment(6 * i_bou + 3, 3));
        // }
        // Ri_sup += -O_thetamom + O_B * M_red * P_W.transpose();

        ///*******************************************///
        //// Inertial stiffness matrix. Harmful for numerical integration, hence useless.
        // ChVectorDynamic<> f_loc_C = M_red * (P_W.transpose() * a_mod) +
        //                             ((O_F + O_B) * M_red + MVPFQ - MVPFQ.transpose()) * (P_W.transpose() * v_mod);
        // ChMatrixDynamic<> V_iner;
        // ChMatrixDynamic<> V_acc;
        // V_iner.setZero(num_coords_vel_bou_mod, 6);
        // V_acc.setZero(num_coords_vel_bou_mod, 6);
        // for (unsigned int i_bou = 0; i_bou < m_num_coords_vel_boundary / 6; i_bou++) {
        //     V_iner.block(6 * i_bou, 3, 3, 3) = ChStarMatrix33<>(f_loc_C.segment(6 * i_bou, 3));
        //     V_acc.block(6 * i_bou, 3, 3, 3) =
        //         ChStarMatrix33<>(floating_frame_F.GetRot().RotateBack(a_mod.segment(6 * i_bou, 3)));
        // }

        // ChVectorDynamic<> h_loc_alpha(6);
        // h_loc_alpha = Q_0 * (P_W.transpose() * v_mod);
        // ChVector3d VF_alpha = h_loc_alpha.head(3);
        // ChMatrixDynamic<> V_alpha;
        // V_alpha.setZero(6, 6);
        // V_alpha.block(0, 3, 3, 3) = -floating_frame_F.GetRotMat() * ChStarMatrix33<>(VF_alpha);

        // ChVectorDynamic<> h_loc_beta(6);
        // h_loc_beta = V.transpose() * M_red * (P_W.transpose() * v_mod);
        // ChVector3d VF_beta = h_loc_beta.head(3);
        // ChMatrixDynamic<> V_beta;
        // V_beta.setZero(6, 6);
        // V_beta.block(0, 3, 3, 3) = ChStarMatrix33<>(floating_frame_F.GetRotMat().transpose() * VF_beta);

        // ChMatrixDynamic<> PFQPWT = P_F * Q_0 * P_W.transpose();
        // Ki_sup = P_W * (M_red * V_acc - V_iner) * PFQPWT +
        //          P_W * ((O_F + O_B) * M_red + MVPFQ - MVPFQ.transpose()) * V * PFQPWT -
        //          P_W * V_rmom * (V_alpha + P_F * Q_0 * V) * PFQPWT - P_W * M_red * O_F * V * PFQPWT +
        //          P_W * Q_0.transpose() * P_F.transpose() * V_rmom.transpose() * V * PFQPWT +
        //          P_W * M_red * V * V_alpha * PFQPWT - P_W * Q_0.transpose() * V_beta * PFQPWT;
    }
}

void ChModalAssembly::ComputeStiffnessMatrix() {
    if (!m_is_model_reduced)
        return;

    ChMatrixDynamic<> PTKP = P_perp_0.transpose() * K_red * P_perp_0;
    // material stiffness matrix of reduced modal assembly
    Km_sup = P_W * PTKP * P_W.transpose();

    {  // geometric stiffness matrix of reduced modal assembly
        unsigned int num_coords_pos_bou_mod = this->m_num_coords_pos_boundary + this->m_num_coords_modal;
        unsigned int num_coords_vel_bou_mod = m_num_coords_vel_boundary + this->m_num_coords_modal;

        double fooT;
        ChState x_mod;       // =[qB; eta]
        ChStateDelta v_mod;  // =[qB_dt; eta_dt]
        x_mod.setZero(num_coords_pos_bou_mod, nullptr);
        v_mod.setZero(num_coords_vel_bou_mod, nullptr);
        this->IntStateGather(0, x_mod, 0, v_mod, fooT);

        ChStateDelta u_locred(num_coords_vel_bou_mod, nullptr);
        ChStateDelta e_locred(num_coords_vel_bou_mod, nullptr);
        ChStateDelta edt_locred(num_coords_vel_bou_mod, nullptr);
        this->GetLocalDeformations(u_locred, e_locred, edt_locred);

        ChVectorDynamic<> g_loc_alpha(num_coords_vel_bou_mod);
        g_loc_alpha = P_perp_0.transpose() * (K_red * e_locred);

        ChMatrixDynamic<> V_F1;
        V_F1.setZero(num_coords_vel_bou_mod, 6);
        ChMatrixDynamic<> V_F2;
        V_F2.setZero(num_coords_vel_bou_mod, 6);
        for (unsigned int i_bou = 0; i_bou < m_num_coords_vel_boundary / 6; i_bou++) {
            V_F1.block(6 * i_bou, 3, 3, 3) = ChStarMatrix33<>(g_loc_alpha.segment(6 * i_bou, 3));
            V_F2.block(6 * i_bou, 3, 3, 3) = ChStarMatrix33<>(u_locred.segment(6 * i_bou, 3));
        }

        Kg_sup.setZero();
        Kg_sup = P_W * (-V_F1 + PTKP * V_F2) * P_F * Q_0 * P_W.transpose();
    }
}

void ChModalAssembly::ComputeDampingMatrix() {
    if (!m_is_model_reduced)
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
    if (!m_use_linear_inertial_term)
        this->modal_K += Ki_sup;

    // modal damping matrix
    this->modal_R = Rm_sup + Ri_sup;

    // modal constraint Jacobian matrix
    // todo: check the formulas, the below code might be wrong.
    this->modal_Cq = Cq_red * P_W.transpose();
}

void ChModalAssembly::SetupModalData(unsigned int nmodes_reduction) {
    this->m_num_coords_modal = nmodes_reduction;
    this->Setup();

    // Initialize matrices
    L_B.setZero(m_num_coords_vel_boundary, m_num_coords_vel_boundary);
    L_I.setZero(m_num_coords_vel_internal, m_num_coords_vel_internal);
    P_W.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);
    P_F.setZero(6, 6);
    U_locred.setZero(m_num_coords_vel_boundary + m_num_coords_modal, 6);
    U_locred_0.setZero(m_num_coords_vel_boundary + m_num_coords_modal, 6);
    Q_0.setZero(6, m_num_coords_vel_boundary + m_num_coords_modal);
    P_parallel_0.setZero(m_num_coords_vel_boundary + m_num_coords_modal,
                         m_num_coords_vel_boundary + m_num_coords_modal);
    P_perp_0.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);
    Uloc_B.setZero(m_num_coords_vel_boundary, 6);
    Uloc_B_0.setZero(m_num_coords_vel_boundary, 6);
    Uloc_I.setZero(m_num_coords_vel_internal, 6);
    Uloc_I_0.setZero(m_num_coords_vel_internal, 6);

    M_red.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);
    K_red.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);
    R_red.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);
    Cq_red.setZero(m_num_constr_boundary, m_num_coords_vel_boundary + m_num_coords_modal);

    Km_sup.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);
    Kg_sup.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);
    Rm_sup.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);
    M_sup.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);
    Ri_sup.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);
    Ki_sup.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);

    if (!modal_variables || (modal_variables->GetDOF() != this->m_num_coords_modal)) {
        // Initialize ChVariable object used for modal variables
        if (modal_variables)
            delete modal_variables;
        modal_variables = new ChVariablesGenericDiagonalMass(this->m_num_coords_modal);
        modal_variables->GetMassDiagonal()
            .setZero();  // diag. mass not needed, the mass will be defined via this->modal_Hblock

        // Initialize the modal_Hblock, which is a ChKRMBlock referencing all ChVariable items:
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
        mvars = temporary_descriptor.GetVariables();
        // - for the MODAL variables:
        mvars.push_back(this->modal_variables);

        // NOTE! Purge the not active variables, so that there is a  1-to-1 mapping
        // between the assembly's matrices this->modal_M, modal_K, modal_R and the modal_Hblock->GetMatrix() block.
        // In fact the ChKRMBlock modal_Hblock could also handle the not active vars, but the modal_M, K etc
        // are computed for the active-only variables for simplicity in the HERTING transformation.
        std::vector<ChVariables*> mvars_active;
        for (auto mvar : mvars) {
            if (mvar->IsActive())
                mvars_active.push_back(mvar);
        }

        this->modal_Hblock.SetVariables(mvars_active);

        // Initialize vectors to be used with modal coordinates:
        this->modal_q.setZero(this->m_num_coords_modal);
        this->modal_q_dt.setZero(this->m_num_coords_modal);
        this->modal_q_dtdt.setZero(this->m_num_coords_modal);
        this->full_forces_internal.setZero(m_num_coords_vel_internal);
    }
}

bool ChModalAssembly::ComputeModes(const ChModalSolveUndamped& n_modes_settings) {
    if (this->m_is_model_reduced)
        throw std::runtime_error(
            "Error: it is not supported to compute modes when the modal assembly is already in the reduced state.");

    m_timer_matrix_assembly.start();
    ChSparseMatrix full_M;
    ChSparseMatrix full_K;
    ChSparseMatrix full_Cq;

    this->GetSubassemblyMassMatrix(&full_M);
    this->GetSubassemblyStiffnessMatrix(&full_K);
    this->GetSubassemblyConstraintJacobianMatrix(&full_Cq);

    ComputeLocalFullKMCqMatrix(full_M, full_K, full_Cq);
    m_timer_matrix_assembly.stop();

    // SOLVE EIGENVALUE
    this->ComputeModesExternalData(full_M_loc, full_K_loc, full_Cq_loc, n_modes_settings);

    return true;
}

bool ChModalAssembly::ComputeModesExternalData(ChSparseMatrix& full_M,
                                               ChSparseMatrix& full_K,
                                               ChSparseMatrix& full_Cq,
                                               const ChModalSolveUndamped& n_modes_settings) {
    // SOLVE EIGENVALUE
    // for undamped system (use generalized constrained eigen solver)
    // - Must work with large dimension and sparse matrices only
    // - Must work also in free-free cases, with 6 rigid body modes at 0 frequency.
    m_timer_modal_solver_call.start();
    n_modes_settings.Solve(full_M, full_K, full_Cq, this->m_modal_eigvect, this->m_modal_eigvals, this->m_modal_freq);
    m_timer_modal_solver_call.stop();

    m_timer_setup.start();

    this->m_modal_damping_ratios.setZero(this->m_modal_freq.rows());

    m_timer_setup.stop();

    return true;
}

bool ChModalAssembly::ComputeModesDamped(const ChModalSolveDamped& n_modes_settings) {
    if (m_is_model_reduced)
        throw std::runtime_error(
            "Error: it is not supported to compute modes when the modal assembly is already in the reduced state.");

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

    ComputeLocalFullKMCqMatrix(full_M, full_K, full_Cq);

    m_timer_matrix_assembly.stop();

    // SOLVE QUADRATIC EIGENVALUE
    // for damped system (use quadratic constrained eigen solver)
    // - Must work with large dimension and sparse matrices only
    // - Must work also in free-free cases, with 6 rigid body modes at 0 frequency.
    m_timer_modal_solver_call.start();
    n_modes_settings.Solve(full_M_loc, full_R_loc, full_K_loc, full_Cq_loc, this->m_modal_eigvect,
                           this->m_modal_eigvals, this->m_modal_freq, this->m_modal_damping_ratios);
    m_timer_modal_solver_call.stop();

    m_timer_setup.start();
    this->Setup();
    m_timer_setup.stop();

    return true;
}

void ChModalAssembly::SetFullStateWithModeOverlay(unsigned int n_mode, double phase, double amplitude) {
    if (n_mode >= m_modal_eigvect.cols()) {
        Update();
        throw std::runtime_error("Error: mode " + std::to_string(n_mode) + " is beyond the " +
                                 std::to_string(m_modal_eigvect.cols()) + " computed eigenvectors.");
    }

    if (m_modal_eigvect.rows() != m_num_coords_vel) {
        Update();
        return;  // TODO: why returning? what the Update would do?
    }

    double fooT = 0;
    ChState assembly_x_new;
    ChStateDelta assembly_v;
    ChStateDelta assembly_Dx_loc;
    ChStateDelta assembly_Dx;

    assembly_x_new.setZero(m_num_coords_pos, nullptr);
    assembly_v.setZero(m_num_coords_vel, nullptr);
    assembly_Dx_loc.setZero(m_num_coords_vel, nullptr);
    assembly_Dx.setZero(m_num_coords_vel, nullptr);

    // pick the nth eigenvector in local reference F
    assembly_Dx_loc = sin(phase) * amplitude * this->m_modal_eigvect.col(n_mode).real() +
                      cos(phase) * amplitude * this->m_modal_eigvect.col(n_mode).imag();

    // transform the above local increment in F to the original mixed basis,
    // then it can be accumulated to m_full_state_x0 to update the position.
    for (unsigned int i = 0; i < (unsigned int)(m_num_coords_vel / 6.); ++i) {
        assembly_Dx.segment(6 * i, 3) =
            floating_frame_F.GetRotMat() * assembly_Dx_loc.segment(6 * i, 3);       // translation
        assembly_Dx.segment(6 * i + 3, 3) = assembly_Dx_loc.segment(6 * i + 3, 3);  // rotation
    }

    this->IntStateIncrement(0, assembly_x_new, this->m_full_state_x0, 0,
                            assembly_Dx);  // x += amplitude * eigenvector

    this->IntStateScatter(0, assembly_x_new, 0, assembly_v, fooT, true);

    this->Update();
}

void ChModalAssembly::SetInternalStateWithModes(bool full_update) {
    if (!this->m_is_model_reduced)
        return;

    unsigned int num_coords_pos_bou_int = this->m_num_coords_pos_boundary + this->m_num_coords_pos_internal;
    unsigned int num_coords_vel_bou_int = m_num_coords_vel_boundary + m_num_coords_vel_internal;
    unsigned int num_coords_pos_bou_mod = this->m_num_coords_pos_boundary + this->m_num_coords_modal;
    unsigned int num_coords_vel_bou_mod = m_num_coords_vel_boundary + this->m_num_coords_modal;

    if (this->Psi.rows() != num_coords_vel_bou_int || this->Psi.cols() != num_coords_vel_bou_mod)
        return;

    double fooT;
    ChState x_mod;       // =[qB; eta]
    ChStateDelta v_mod;  // =[qB_dt; eta_dt]
    x_mod.setZero(num_coords_pos_bou_mod, nullptr);
    v_mod.setZero(num_coords_vel_bou_mod, nullptr);
    this->IntStateGather(0, x_mod, 0, v_mod, fooT);

    // Update w.r.t. the undeformed configuration

    ChStateDelta u_locred(num_coords_vel_bou_mod, nullptr);
    ChStateDelta e_locred(num_coords_vel_bou_mod, nullptr);
    ChStateDelta edt_locred(num_coords_vel_bou_mod, nullptr);
    this->GetLocalDeformations(u_locred, e_locred, edt_locred);

    ChStateDelta Dx_internal_loc;  // =[delta_qI^bar]
    Dx_internal_loc.setZero(m_num_coords_vel_internal, nullptr);
    Dx_internal_loc.segment(0, m_num_coords_vel_internal) =
        Psi_S * e_locred.segment(0, m_num_coords_vel_boundary) +
        Psi_D * e_locred.segment(m_num_coords_vel_boundary, m_num_coords_modal);

    ChState assembly_x_new;  // =[qB_new; qI_new]
    assembly_x_new.setZero(num_coords_pos_bou_int, nullptr);
    assembly_x_new.head(m_num_coords_pos_boundary) = x_mod.head(m_num_coords_pos_boundary);

    for (unsigned int i_int = 0; i_int < m_num_coords_vel_internal / 6; i_int++) {
        unsigned int offset_x = m_num_coords_pos_boundary + 7 * i_int;
        ChVector3d r_IF0 = floating_frame_F0.GetRotMat().transpose() *
                           (m_full_state_x0.segment(offset_x, 3) - floating_frame_F0.GetPos().eigen());
        ChVector3d r_I =
            floating_frame_F.GetPos() + floating_frame_F.GetRotMat() * (r_IF0 + Dx_internal_loc.segment(6 * i_int, 3));
        assembly_x_new.segment(offset_x, 3) = r_I.eigen();

        ChQuaternion<> q_delta;
        q_delta.SetFromRotVec(Dx_internal_loc.segment(6 * i_int + 3, 3));
        ChQuaternion<> quat_int0 = m_full_state_x0.segment(offset_x + 3, 4);
        ChQuaternion<> q_refrot = floating_frame_F0.GetRot().GetConjugate() * quat_int0;
        // ChQuaternion<> quat_int = floating_frame_F.GetRot() * q_delta * q_refrot;
        ChQuaternion<> quat_int =
            floating_frame_F.GetRot() * floating_frame_F0.GetRot().GetConjugate() * quat_int0 * q_delta;
        assembly_x_new.segment(offset_x + 3, 4) = quat_int.eigen();
    }

    ChStateDelta assembly_v_new;  // =[qB_dt; qI_dt]
    assembly_v_new.setZero(num_coords_vel_bou_int, nullptr);
    assembly_v_new.segment(0, m_num_coords_vel_boundary) = v_mod.segment(0, m_num_coords_vel_boundary);
    assembly_v_new.segment(m_num_coords_vel_boundary, m_num_coords_vel_internal) =
        L_I * (Psi_S * (L_B.transpose() * v_mod.segment(0, m_num_coords_vel_boundary)) +
               Psi_D * v_mod.segment(m_num_coords_vel_boundary, m_num_coords_modal));

    bool needs_temporary_bou_int = this->m_is_model_reduced;
    if (needs_temporary_bou_int)
        this->m_is_model_reduced = false;

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
        this->m_is_model_reduced = true;

    // store the full state for the computation in next time step
    this->m_full_state_x = assembly_x_new;
}

void ChModalAssembly::SetFullStateReset() {
    if (this->m_full_state_x0.rows() != m_num_coords_pos)
        return;

    double fooT = 0;
    ChStateDelta assembly_v;

    assembly_v.setZero(m_num_coords_vel, nullptr);

    this->IntStateScatter(0, this->m_full_state_x0, 0, assembly_v, fooT, true);

    this->Update();
}

void ChModalAssembly::SetInternalNodesUpdate(bool flag) {
    this->internal_nodes_update = flag;
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
    assert(item->GetSystem() == nullptr || item->GetSystem() == system);

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
    this->InjectKRMMatrices(temp_descriptor);
    this->InjectConstraints(temp_descriptor);

    // Load all KRM matrices with the M part only
    LoadKRMMatrices(0, 0, 1.0);
    // For ChVariable objects without a ChKRMBlock, but still with a mass:
    temp_descriptor.SetMassFactor(1.0);

    // Fill system-level M matrix
    M->resize(temp_descriptor.CountActiveVariables(), temp_descriptor.CountActiveVariables());
    M->setZeroValues();
    temp_descriptor.PasteMassKRMMatrixInto(*M);
    // M->makeCompressed();
}

void ChModalAssembly::GetSubassemblyStiffnessMatrix(ChSparseMatrix* K) {
    this->SetupInitial();
    this->Setup();
    this->Update();

    ChSystemDescriptor temp_descriptor;

    this->InjectVariables(temp_descriptor);
    this->InjectKRMMatrices(temp_descriptor);
    this->InjectConstraints(temp_descriptor);

    // Load all KRM matrices with the K part only
    this->LoadKRMMatrices(1.0, 0, 0);
    // For ChVariable objects without a ChKRMBlock, but still with a mass:
    temp_descriptor.SetMassFactor(0.0);

    // Fill system-level K matrix
    K->resize(temp_descriptor.CountActiveVariables(), temp_descriptor.CountActiveVariables());
    K->setZeroValues();
    temp_descriptor.PasteMassKRMMatrixInto(*K);
    // K->makeCompressed();
}

void ChModalAssembly::GetSubassemblyDampingMatrix(ChSparseMatrix* R) {
    this->SetupInitial();
    this->Setup();
    this->Update();

    ChSystemDescriptor temp_descriptor;

    this->InjectVariables(temp_descriptor);
    this->InjectKRMMatrices(temp_descriptor);
    this->InjectConstraints(temp_descriptor);

    // Load all KRM matrices with the R part only
    this->LoadKRMMatrices(0, 1.0, 0);
    // For ChVariable objects without a ChKRMBlock, but still with a mass:
    temp_descriptor.SetMassFactor(0.0);

    // Fill system-level R matrix
    R->resize(temp_descriptor.CountActiveVariables(), temp_descriptor.CountActiveVariables());
    R->setZeroValues();
    temp_descriptor.PasteMassKRMMatrixInto(*R);
    // R->makeCompressed();
}

void ChModalAssembly::GetSubassemblyConstraintJacobianMatrix(ChSparseMatrix* Cq) {
    this->SetupInitial();
    this->Setup();
    this->Update();

    ChSystemDescriptor temp_descriptor;

    this->InjectVariables(temp_descriptor);
    this->InjectKRMMatrices(temp_descriptor);
    this->InjectConstraints(temp_descriptor);

    // Load all jacobian matrices
    this->LoadConstraintJacobians();

    // Fill system-level R matrix
    Cq->resize(temp_descriptor.CountActiveConstraints(), temp_descriptor.CountActiveVariables());
    Cq->setZeroValues();
    temp_descriptor.PasteConstraintsJacobianMatrixInto(*Cq);
    // Cq->makeCompressed();
}

void ChModalAssembly::WriteSubassemblyMatrices(bool save_M,
                                               bool save_K,
                                               bool save_R,
                                               bool save_Cq,
                                               const std::string& path,
                                               bool one_indexed) {
    if (save_M) {
        ChSparseMatrix mM;
        GetSubassemblyMassMatrix(&mM);
        std::ofstream file_M(path + "_M.dat");
        file_M << std::setprecision(12) << std::scientific;
        StreamOut(mM, file_M, one_indexed);
    }
    if (save_K) {
        ChSparseMatrix mK;
        GetSubassemblyStiffnessMatrix(&mK);
        std::ofstream file_K(path + "_K.dat");
        file_K << std::setprecision(12) << std::scientific;
        StreamOut(mK, file_K, one_indexed);
    }
    if (save_R) {
        ChSparseMatrix mR;
        GetSubassemblyDampingMatrix(&mR);
        std::ofstream file_R(path + "_R.dat");
        file_R << std::setprecision(12) << std::scientific;
        StreamOut(mR, file_R, one_indexed);
    }
    if (save_Cq) {
        ChSparseMatrix mCq;
        GetSubassemblyConstraintJacobianMatrix(&mCq);
        std::ofstream file_Cq(path + "_Cq.dat");
        file_Cq << std::setprecision(12) << std::scientific;
        StreamOut(mCq, file_Cq, one_indexed);
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

    for (unsigned int ip = 0; ip < internal_bodylist.size(); ++ip) {
        internal_bodylist[ip]->SetupInitial();
    }
    for (unsigned int ip = 0; ip < internal_linklist.size(); ++ip) {
        internal_linklist[ip]->SetupInitial();
    }
    for (unsigned int ip = 0; ip < internal_meshlist.size(); ++ip) {
        internal_meshlist[ip]->SetupInitial();
    }
    for (unsigned int ip = 0; ip < internal_otherphysicslist.size(); ++ip) {
        internal_otherphysicslist[ip]->SetupInitial();
    }
}

// Count all bodies, links, meshes, and other physics items.
// Set counters (DOF, num constraints, etc) and offsets.
void ChModalAssembly::Setup() {
    ChAssembly::Setup();  // parent

    m_num_bodies_boundary = m_num_bodies_active;
    m_num_links_boundary = m_num_links_active;
    m_num_meshes_boundary = m_num_meshes;
    m_num_otherphysicsitems_boundary = m_num_otherphysicsitems_active;
    m_num_coords_pos_boundary = m_num_coords_pos;
    m_num_coords_vel_boundary = m_num_coords_vel;
    m_num_constr_boundary = m_num_constr;
    m_num_constr_bil_boundary = m_num_constr_bil;
    m_num_constr_uni_boundary = m_num_constr_uni;

    m_num_bodies_internal = 0;
    m_num_links_internal = 0;
    m_num_meshes_internal = 0;
    m_num_otherphysicsitems_internal = 0;
    m_num_coords_pos_internal = 0;
    m_num_coords_vel_internal = 0;
    m_num_constr_internal = 0;
    m_num_constr_bil_internal = 0;
    m_num_constr_uni_internal = 0;

    // For the "internal" items:
    //

    for (auto& body : internal_bodylist) {
        if (body->IsFixed()) {
            // throw std::runtime_error("Cannot use a fixed body as internal");
        } else if (body->IsSleeping()) {
            // throw std::runtime_error("Cannot use a sleeping body as internal");
        } else {
            m_num_bodies_internal++;

            body->SetOffset_x(this->offset_x + m_num_coords_pos_boundary + m_num_coords_pos_internal);
            body->SetOffset_w(this->offset_w + m_num_coords_vel_boundary + m_num_coords_vel_internal);
            body->SetOffset_L(this->offset_L + m_num_constr_boundary + m_num_constr_internal);

            body->Setup();  // currently, no-op

            m_num_coords_pos_internal += body->GetNumCoordsPosLevel();
            m_num_coords_vel_internal += body->GetNumCoordsVelLevel();
            m_num_constr_internal +=
                body->GetNumConstraints();  // not really needed since ChBody introduces no constraints
        }
    }

    for (auto& link : internal_linklist) {
        if (link->IsActive()) {
            m_num_links_internal++;

            link->SetOffset_x(this->offset_x + m_num_coords_pos_boundary + m_num_coords_pos_internal);
            link->SetOffset_w(this->offset_w + m_num_coords_vel_boundary + m_num_coords_vel_internal);
            link->SetOffset_L(this->offset_L + m_num_constr_boundary + m_num_constr_internal);

            link->Setup();  // compute DOFs etc. and sets the offsets also in child items, if any

            m_num_coords_pos_internal += link->GetNumCoordsPosLevel();
            m_num_coords_vel_internal += link->GetNumCoordsVelLevel();
            m_num_constr_internal += link->GetNumConstraints();
            m_num_constr_bil_internal += link->GetNumConstraintsBilateral();
            m_num_constr_uni_internal += link->GetNumConstraintsUnilateral();
        }
    }

    for (auto& mesh : internal_meshlist) {
        m_num_meshes_internal++;

        mesh->SetOffset_x(this->offset_x + m_num_coords_pos_boundary + m_num_coords_pos_internal);
        mesh->SetOffset_w(this->offset_w + m_num_coords_vel_boundary + m_num_coords_vel_internal);
        mesh->SetOffset_L(this->offset_L + m_num_constr_boundary + m_num_constr_internal);

        mesh->Setup();  // compute DOFs and iteratively call Setup for child items

        m_num_coords_pos_internal += mesh->GetNumCoordsPosLevel();
        m_num_coords_vel_internal += mesh->GetNumCoordsVelLevel();
        m_num_constr_internal += mesh->GetNumConstraints();
        m_num_constr_bil_internal += mesh->GetNumConstraintsBilateral();
        m_num_constr_uni_internal += mesh->GetNumConstraintsUnilateral();
    }

    for (auto& item : internal_otherphysicslist) {
        m_num_otherphysicsitems_internal++;

        item->SetOffset_x(this->offset_x + m_num_coords_pos_boundary + m_num_coords_pos_internal);
        item->SetOffset_w(this->offset_w + m_num_coords_vel_boundary + m_num_coords_vel_internal);
        item->SetOffset_L(this->offset_L + m_num_constr_boundary + m_num_constr_internal);

        item->Setup();

        m_num_coords_pos_internal += item->GetNumCoordsPosLevel();
        m_num_coords_vel_internal += item->GetNumCoordsVelLevel();
        m_num_constr_internal += item->GetNumConstraints();
        m_num_constr_bil_internal += item->GetNumConstraintsBilateral();
        m_num_constr_uni_internal += item->GetNumConstraintsUnilateral();
    }

    // this->custom_F_full.setZero(m_num_coords_vel_boundary + m_num_coords_vel_internal);
    this->full_forces_internal.setZero(m_num_coords_vel_internal);

    // For the modal part:
    //

    // (nothing to count)

    // For the entire assembly:
    //

    if (this->m_is_model_reduced == false) {
        m_num_coords_pos = m_num_coords_pos_boundary + m_num_coords_pos_internal;
        m_num_coords_vel = m_num_coords_vel_boundary + m_num_coords_vel_internal;
        m_num_constr = m_num_constr_boundary + m_num_constr_internal;
        m_num_constr_bil = m_num_constr_bil_boundary + m_num_constr_bil_internal;
        m_num_constr_uni = m_num_constr_uni_boundary + m_num_constr_uni_internal;
        m_num_bodies_active += m_num_bodies_internal;
        m_num_links_active += m_num_links_internal;
        m_num_meshes += m_num_meshes_internal;
        m_num_otherphysicsitems_active += m_num_otherphysicsitems_internal;
    } else {
        m_num_coords_pos =
            m_num_coords_pos_boundary + m_num_coords_modal;  // no need for a n_modes_coords, same as m_num_coords_modal
        m_num_coords_vel = m_num_coords_vel_boundary + m_num_coords_modal;
        m_num_constr = m_num_constr_boundary;
        m_num_constr_bil = m_num_constr_bil_boundary;
        m_num_constr_uni = m_num_constr_uni_boundary;
    }

    if (!this->is_initialized) {
        // fetch the initial state of assembly, full not reduced, as an initialization
        double fooT;
        this->m_full_state_x0.setZero(m_num_coords_pos, nullptr);
        ChStateDelta full_assembly_v;
        full_assembly_v.setZero(m_num_coords_vel, nullptr);
        this->IntStateGather(0, this->m_full_state_x0, 0, full_assembly_v, fooT);

        // also initialize m_full_state_x
        this->m_full_state_x = this->m_full_state_x0;

        // initialize the floating frame of reference F to be placed at COG
        this->UpdateFloatingFrameOfReference();

        this->is_initialized = true;
    }
}

// Update all physical items (bodies, links, meshes, etc), including their auxiliary variables.
// Updates all forces (automatic, as children of bodies)
// Updates all markers (automatic, as children of bodies).
void ChModalAssembly::Update(bool update_assets) {
    ChAssembly::Update(update_assets);  // parent

    if (m_is_model_reduced == false) {
        //// NOTE: do not switch these to range for loops (may want to use OMP for)
        for (unsigned int ip = 0; ip < internal_bodylist.size(); ++ip) {
            internal_bodylist[ip]->Update(ChTime, update_assets);
        }
        for (unsigned int ip = 0; ip < internal_meshlist.size(); ++ip) {
            internal_meshlist[ip]->Update(ChTime, update_assets);
        }
        for (unsigned int ip = 0; ip < internal_otherphysicslist.size(); ++ip) {
            internal_otherphysicslist[ip]->Update(ChTime, update_assets);
        }
        for (unsigned int ip = 0; ip < internal_linklist.size(); ++ip) {
            internal_linklist[ip]->Update(ChTime, update_assets);
        }
    } else {
        // If in modal reduced state, the internal parts would not be updated (actually, these could even be
        // removed) However one still might want to see the internal nodes "moving" during animations,
        if (this->internal_nodes_update)
            this->SetInternalStateWithModes(update_assets);

        // Update the external forces imposed on the internal nodes.
        // Note: the below code requires that the internal bodies and internal nodes are inserted in sequence.
        unsigned int offset_loc = 0;
        for (unsigned int ip = 0; ip < internal_bodylist.size(); ++ip) {
            this->full_forces_internal.segment(offset_loc, 3) = internal_bodylist[ip]->GetAccumulatedForce().eigen();
            this->full_forces_internal.segment(offset_loc + 3, 3) =
                internal_bodylist[ip]->GetAccumulatedTorque().eigen();
            offset_loc += internal_bodylist[ip]->GetNumCoordsVelLevel();
        }
        for (unsigned int ip = 0; ip < internal_meshlist.size(); ++ip) {
            for (auto& node : internal_meshlist[ip]->GetNodes()) {
                if (auto xyz = std::dynamic_pointer_cast<ChNodeFEAxyz>(node)) {
                    this->full_forces_internal.segment(offset_loc, xyz->GetNumCoordsVelLevel()) =
                        xyz->GetForce().eigen();
                    offset_loc += xyz->GetNumCoordsVelLevel();
                }
                if (auto xyzrot = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(node)) {
                    this->full_forces_internal.segment(offset_loc, 3) = xyzrot->GetForce().eigen();
                    this->full_forces_internal.segment(offset_loc + 3, 3) = xyzrot->GetTorque().eigen();
                    offset_loc += xyzrot->GetNumCoordsVelLevel();
                }
            }
        }

        // always update the floating frame F if possible, to improve the numerical accuracy and stability
        this->UpdateFloatingFrameOfReference();
    }
}

void ChModalAssembly::ForceToRest() {
    ChAssembly::ForceToRest();  // parent

    if (m_is_model_reduced == false) {
        for (auto& body : internal_bodylist) {
            body->ForceToRest();
        }
        for (auto& link : internal_linklist) {
            link->ForceToRest();
        }
        for (auto& mesh : internal_meshlist) {
            mesh->ForceToRest();
        }
        for (auto& item : internal_otherphysicslist) {
            item->ForceToRest();
        }
    } else {
        this->modal_q_dt.setZero(this->m_num_coords_modal);
        this->modal_q_dtdt.setZero(this->m_num_coords_modal);
    }
}

void ChModalAssembly::GetLocalDeformations(ChStateDelta& u_locred, ChStateDelta& e_locred, ChStateDelta& edt_locred) {
    if (m_is_model_reduced == false) {
        // to do? not useful for the moment.
        return;
    } else {
        unsigned int num_coords_pos_bou_mod = this->m_num_coords_pos_boundary + this->m_num_coords_modal;
        unsigned int num_coords_vel_bou_mod = m_num_coords_vel_boundary + this->m_num_coords_modal;

        u_locred.setZero(num_coords_vel_bou_mod, nullptr);    // =u_locred =P_W^T*[\delta qB; \delta eta]
        e_locred.setZero(num_coords_vel_bou_mod, nullptr);    // =e_locred =[qB^bar; eta]
        edt_locred.setZero(num_coords_vel_bou_mod, nullptr);  // =edt_locred =[qB^bar_dt; eta_dt]

        // fetch the state snapshot (modal reduced)
        double fooT;
        ChState x_mod;       // =[qB; eta]
        ChStateDelta v_mod;  // =[qB_dt; eta_dt]
        x_mod.setZero(num_coords_pos_bou_mod, nullptr);
        v_mod.setZero(num_coords_vel_bou_mod, nullptr);
        this->IntStateGather(0, x_mod, 0, v_mod, fooT);

        u_locred.tail(m_num_coords_modal) = modal_q;
        for (unsigned int i_bou = 0; i_bou < m_num_coords_vel_boundary / 6; i_bou++) {
            // Computed with respect to the initial configuration
            u_locred.segment(6 * i_bou, 3) =
                floating_frame_F.GetRot().RotateBack(x_mod.segment(7 * i_bou, 3)).eigen() -
                floating_frame_F0.GetRot().RotateBack(m_full_state_x0.segment(7 * i_bou, 3)).eigen();

            ChQuaternion<> q_F0 = floating_frame_F0.GetRot();
            ChQuaternion<> q_F = floating_frame_F.GetRot();
            ChQuaternion<> q_B0 = m_full_state_x0.segment(7 * i_bou + 3, 4);
            ChQuaternion<> q_B = x_mod.segment(7 * i_bou + 3, 4);
            ChQuaternion<> rel_q = q_B0.GetConjugate() * q_F0 * q_F.GetConjugate() * q_B;

            double delta_rot_angle;
            ChVector3d delta_rot_dir;
            rel_q.GetAngleAxis(delta_rot_angle, delta_rot_dir);
            u_locred.segment(6 * i_bou + 3, 3) = delta_rot_angle * delta_rot_dir.eigen();
        }

        // local elastic displacement
        e_locred.tail(m_num_coords_modal) = x_mod.segment(m_num_coords_pos_boundary, m_num_coords_modal);
        for (unsigned int i_bou = 0; i_bou < m_num_coords_vel_boundary / 6; i_bou++) {
            ChVector3d r_B = x_mod.segment(7 * i_bou, 3);
            ChVector3d r_BF_0 = floating_frame_F0.GetRot().RotateBack(
                (m_full_state_x0.segment(7 * i_bou, 3) - floating_frame_F0.GetPos().eigen()));
            e_locred.segment(6 * i_bou, 3) =
                (floating_frame_F.GetRot().RotateBack((r_B - floating_frame_F.GetPos())) - r_BF_0).eigen();

            ChQuaternion<> quat_bou = x_mod.segment(7 * i_bou + 3, 4);
            ChQuaternion<> quat_bou0 = m_full_state_x0.segment(7 * i_bou + 3, 4);
            ChQuaternion<> q_delta = quat_bou0.GetConjugate() * floating_frame_F0.GetRot() *
                                     floating_frame_F.GetRot().GetConjugate() * quat_bou;
            // ChQuaternion<> q_delta = floating_frame_F.GetRot().GetConjugate() * quat_bou *
            // quat_bou0.GetConjugate() * floating_frame_F0.GetRot();
            // e_locred.segment(6 * i_bou + 3, 3) = q_delta.SetFromRotVec().eigen();

            double delta_rot_angle;
            ChVector3d delta_rot_dir;
            q_delta.GetAngleAxis(delta_rot_angle, delta_rot_dir);
            e_locred.segment(6 * i_bou + 3, 3) = delta_rot_angle * delta_rot_dir.eigen();
        }

        // local elastic velocity
        edt_locred.tail(m_num_coords_modal) = v_mod.segment(m_num_coords_vel_boundary, m_num_coords_modal);
        for (unsigned int i_bou = 0; i_bou < m_num_coords_vel_boundary / 6; i_bou++) {
            ChVector3d r_B = x_mod.segment(7 * i_bou, 3);
            ChVector3d v_B = v_mod.segment(6 * i_bou, 3);
            ChVector3d r_BF_loc = floating_frame_F.GetRot().RotateBack(r_B - floating_frame_F.GetPos());
            edt_locred.segment(6 * i_bou, 3) =
                (floating_frame_F.GetRot().RotateBack(v_B - floating_frame_F.GetPosDt()) +
                 ChStarMatrix33(r_BF_loc) * floating_frame_F.GetAngVelLocal())
                    .eigen();

            ChVector3d wloc_B = v_mod.segment(6 * i_bou + 3, 3);
            ChQuaternion<> quat_bou = x_mod.segment(7 * i_bou + 3, 4);
            edt_locred.segment(6 * i_bou + 3, 3) =
                (wloc_B - quat_bou.RotateBack(floating_frame_F.GetAngVelParent())).eigen();
        }

        //// DEVELOPER NOTES
        // Do not try to do the following since it causes instabilities:
        // // local elastic displacement
        // e_locred = P_perp_0 * u_locred;

        // // local elastic velocity
        // edt_locred = P_perp_0 * (P_W.transpose() * v_mod);
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

    if (m_is_model_reduced == false) {
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
        x.segment(off_x + this->m_num_coords_pos_boundary, this->m_num_coords_modal) = this->modal_q;
        v.segment(off_v + m_num_coords_vel_boundary, this->m_num_coords_modal) = this->modal_q_dt;

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

    if (m_is_model_reduced == false) {
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
    } else {
        this->modal_q = x.segment(off_x + this->m_num_coords_pos_boundary, this->m_num_coords_modal);
        this->modal_q_dt = v.segment(off_v + m_num_coords_vel_boundary, this->m_num_coords_modal);

        // Update:
        this->Update(full_update);
    }

    ChTime = T;
}

void ChModalAssembly::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    ChAssembly::IntStateGatherAcceleration(off_a, a);  // parent

    unsigned int displ_a = off_a - this->offset_w;

    if (m_is_model_reduced == false) {
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
        a.segment(off_a + m_num_coords_vel_boundary, this->m_num_coords_modal) = this->modal_q_dtdt;
    }
}

// From state derivative (acceleration) to system, sometimes might be needed
void ChModalAssembly::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    ChAssembly::IntStateScatterAcceleration(off_a, a);  // parent

    unsigned int displ_a = off_a - this->offset_w;

    if (m_is_model_reduced == false) {
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
        this->modal_q_dtdt = a.segment(off_a + m_num_coords_vel_boundary, this->m_num_coords_modal);
    }
}

// From system to reaction forces (last computed) - some timestepper might need this
void ChModalAssembly::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    ChAssembly::IntStateGatherReactions(off_L, L);  // parent

    unsigned int displ_L = off_L - this->offset_L;

    if (m_is_model_reduced == false) {
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

    if (m_is_model_reduced == false) {
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

    if (m_is_model_reduced == false) {
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
        x_new.segment(off_x + this->m_num_coords_pos_boundary, this->m_num_coords_modal) =
            x.segment(off_x + this->m_num_coords_pos_boundary, this->m_num_coords_modal) +
            Dv.segment(off_v + m_num_coords_vel_boundary, this->m_num_coords_modal);
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

    if (m_is_model_reduced == false) {
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
        Dv.segment(off_v + m_num_coords_vel_boundary, this->m_num_coords_modal) =
            x_new.segment(off_x + this->m_num_coords_pos_boundary, this->m_num_coords_modal) -
            x.segment(off_x + this->m_num_coords_pos_boundary, this->m_num_coords_modal);
    }
}

void ChModalAssembly::IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
                                        ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
                                        const double c)          ///< a scaling factor
{
    ChAssembly::IntLoadResidual_F(off, R, c);  // parent

    unsigned int displ_v = off - this->offset_w;

    if (m_is_model_reduced == false) {
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
    } else {
        unsigned int num_coords_pos_bou_mod = this->m_num_coords_pos_boundary + this->m_num_coords_modal;
        unsigned int num_coords_vel_bou_mod = this->m_num_coords_vel_boundary + this->m_num_coords_modal;

        // 1-
        // Add elastic forces from current modal deformations
        ChStateDelta u_locred(num_coords_vel_bou_mod, nullptr);
        ChStateDelta e_locred(num_coords_vel_bou_mod, nullptr);
        ChStateDelta edt_locred(num_coords_vel_bou_mod, nullptr);
        this->GetLocalDeformations(u_locred, e_locred, edt_locred);

        // note: - sign
        R.segment(off, this->m_num_coords_vel_boundary + this->m_num_coords_modal) -=
            c * (P_W * P_perp_0.transpose() * (this->K_red * e_locred + this->R_red * edt_locred));

        // 2-
        // Add quadratic velocity term
        {
            double fooT;
            ChState x_mod;       // =[qB; eta]
            ChStateDelta v_mod;  // =[qB_dt; eta_dt]
            x_mod.setZero(num_coords_pos_bou_mod, nullptr);
            v_mod.setZero(num_coords_vel_bou_mod, nullptr);
            this->IntStateGather(0, x_mod, 0, v_mod, fooT);

            ChMatrixDynamic<> O_F;
            O_F.setZero(num_coords_vel_bou_mod, num_coords_vel_bou_mod);
            for (unsigned int i_bou = 0; i_bou < (m_num_coords_vel_boundary / 6.); i_bou++)
                O_F.block(6 * i_bou, 6 * i_bou, 3, 3) = ChStarMatrix33<>(floating_frame_F.GetAngVelLocal());

            ChMatrixDynamic<> mat_OF = P_W * O_F * M_red * P_W.transpose();
            g_quad.setZero(num_coords_vel_bou_mod);
            g_quad += mat_OF * v_mod;

            if (!m_use_linear_inertial_term) {
                ChMatrixDynamic<> V;
                V.setZero(num_coords_vel_bou_mod, 6);
                for (unsigned int i_bou = 0; i_bou < (m_num_coords_vel_boundary / 6.); i_bou++) {
                    V.block(6 * i_bou, 3, 3, 3) =
                        ChStarMatrix33<>(floating_frame_F.GetRot().RotateBack(v_mod.segment(6 * i_bou, 3)));
                }
                ChMatrixDynamic<> mat_M = P_W * M_red * V * P_F * Q_0 * P_W.transpose();
                g_quad += (mat_M - mat_M.transpose()) * v_mod;

                //// leading to divergence. DO NOT use it.
                // ChMatrixDynamic<> O_B;
                // O_B.setZero(num_coords_vel_bou_mod, num_coords_vel_bou_mod);
                // for (unsigned int i_bou = 0; i_bou < (m_num_coords_vel_boundary / 6.); i_bou++) {
                //     O_B.block(6 * i_bou + 3, 6 * i_bou + 3, 3, 3) = ChStarMatrix33<>(v_mod.segment(6 * i_bou + 3,
                //     3));
                // }
                // ChMatrixDynamic<> mat_OB = P_W * O_B * M_red * P_W.transpose();
                // g_quad += mat_OB * v_mod;
            }

            // note: - sign
            R.segment(off, m_num_coords_vel_boundary + this->m_num_coords_modal) -= c * g_quad;
        }

        // 3-
        // Add external imposed forces on internal items (bodies, nodes)
        if (!this->full_forces_internal.isZero()) {
            ChVectorDynamic<> F_reduced;
            F_reduced.setZero(m_num_coords_vel_boundary + this->m_num_coords_modal);
            F_reduced.head(m_num_coords_vel_boundary) =
                L_B * Psi_S.transpose() * L_I.transpose() * this->full_forces_internal;
            F_reduced.tail(m_num_coords_modal) = Psi_D.transpose() * L_I.transpose() * this->full_forces_internal;
            R.segment(off, m_num_coords_vel_boundary + this->m_num_coords_modal) += c * F_reduced;
        }

        // todo: add gravity forces for internal bodies and internal meshes
        // todo: add gyroscopic torques for internal bodies and internal meshes
    }
}

void ChModalAssembly::IntLoadResidual_Mv(const unsigned int off,      ///< offset in R residual
                                         ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                         const ChVectorDynamic<>& w,  ///< the w vector
                                         const double c               ///< a scaling factor
) {
    if (m_is_model_reduced == false) {
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
        ChVectorDynamic<> w_modal = w.segment(off, m_num_coords_vel_boundary + this->m_num_coords_modal);
        R.segment(off, m_num_coords_vel_boundary + this->m_num_coords_modal) += c * (this->modal_M * w_modal);
    }
}

void ChModalAssembly::IntLoadLumpedMass_Md(const unsigned int off, ChVectorDynamic<>& Md, double& err, const double c) {
    unsigned int displ_v = off - this->offset_w;

    if (m_is_model_reduced == false) {
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
        Md.segment(off, m_num_coords_vel_boundary + this->m_num_coords_modal) += c * this->modal_M.diagonal();

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

    if (m_is_model_reduced == false) {
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
        // there might be residual CqL in the reduced modal assembly
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

    if (m_is_model_reduced == false) {
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
        // there might be constraint C in the reduced modal assembly
    }
}

void ChModalAssembly::IntLoadConstraint_Ct(const unsigned int off_L,  ///< offset in Qc residual
                                           ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*Ct
                                           const double c             ///< a scaling factor
) {
    ChAssembly::IntLoadConstraint_Ct(off_L, Qc, c);  // parent

    unsigned int displ_L = off_L - this->offset_L;

    if (m_is_model_reduced == false) {
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
        // there might be constraint Ct in the reduced modal assembly
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

    if (m_is_model_reduced == false) {
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
        this->modal_variables->State() = v.segment(off_v + m_num_coords_vel_boundary, this->m_num_coords_modal);
        this->modal_variables->Force() = R.segment(off_v + m_num_coords_vel_boundary, this->m_num_coords_modal);
    }
}

void ChModalAssembly::IntFromDescriptor(const unsigned int off_v,
                                        ChStateDelta& v,
                                        const unsigned int off_L,
                                        ChVectorDynamic<>& L) {
    ChAssembly::IntFromDescriptor(off_v, v, off_L, L);  // parent

    unsigned int displ_L = off_L - this->offset_L;
    unsigned int displ_v = off_v - this->offset_w;

    if (m_is_model_reduced == false) {
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
        v.segment(off_v + m_num_coords_vel_boundary, this->m_num_coords_modal) = this->modal_variables->State();
    }
}

// -----------------------------------------------------------------------------

void ChModalAssembly::InjectVariables(ChSystemDescriptor& descriptor) {
    ChAssembly::InjectVariables(descriptor);  // parent

    if (m_is_model_reduced == false) {
        for (auto& body : internal_bodylist) {
            body->InjectVariables(descriptor);
        }
        for (auto& link : internal_linklist) {
            link->InjectVariables(descriptor);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->InjectVariables(descriptor);
        }
        for (auto& item : internal_otherphysicslist) {
            item->InjectVariables(descriptor);
        }
    } else {
        descriptor.InsertVariables(this->modal_variables);
    }
}

void ChModalAssembly::InjectConstraints(ChSystemDescriptor& descriptor) {
    ChAssembly::InjectConstraints(descriptor);  // parent

    if (m_is_model_reduced == false) {
        for (auto& body : internal_bodylist) {
            body->InjectConstraints(descriptor);
        }
        for (auto& link : internal_linklist) {
            link->InjectConstraints(descriptor);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->InjectConstraints(descriptor);
        }
        for (auto& item : internal_otherphysicslist) {
            item->InjectConstraints(descriptor);
        }
    } else {
        // todo:
        // there might be constraints for the reduced modal assembly: this->modal_Cq
    }
}

void ChModalAssembly::LoadConstraintJacobians() {
    ChAssembly::LoadConstraintJacobians();  // parent

    if (!m_is_model_reduced) {
        for (auto& body : internal_bodylist) {
            body->LoadConstraintJacobians();
        }
        for (auto& link : internal_linklist) {
            link->LoadConstraintJacobians();
        }
        for (auto& mesh : internal_meshlist) {
            mesh->LoadConstraintJacobians();
        }
        for (auto& item : internal_otherphysicslist) {
            item->LoadConstraintJacobians();
        }
    } else {
        // todo:
        // there might be constraints for the reduced modal assembly: this->modal_Cq
    }
}

void ChModalAssembly::InjectKRMMatrices(ChSystemDescriptor& descriptor) {
    if (!m_is_model_reduced) {
        ChAssembly::InjectKRMMatrices(descriptor);  // parent

        for (auto& body : internal_bodylist) {
            body->InjectKRMMatrices(descriptor);
        }
        for (auto& link : internal_linklist) {
            link->InjectKRMMatrices(descriptor);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->InjectKRMMatrices(descriptor);
        }
        for (auto& item : internal_otherphysicslist) {
            item->InjectKRMMatrices(descriptor);
        }
    } else {
        descriptor.InsertKRMBlock(&this->modal_Hblock);
    }
}

void ChModalAssembly::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
    if (m_is_model_reduced == false) {
        ChAssembly::LoadKRMMatrices(Kfactor, Rfactor, Mfactor);  // parent

        for (auto& body : internal_bodylist) {
            body->LoadKRMMatrices(Kfactor, Rfactor, Mfactor);
        }
        for (auto& link : internal_linklist) {
            link->LoadKRMMatrices(Kfactor, Rfactor, Mfactor);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->LoadKRMMatrices(Kfactor, Rfactor, Mfactor);
        }
        for (auto& item : internal_otherphysicslist) {
            item->LoadKRMMatrices(Kfactor, Rfactor, Mfactor);
        }
    } else {
        ComputeModalKRMmatrix();

        this->modal_Hblock.GetMatrix() = this->modal_K * Kfactor + this->modal_R * Rfactor + this->modal_M * Mfactor;
    }
}

// -----------------------------------------------------------------------------
//  STREAMING - FILE HANDLING

void ChModalAssembly::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChModalAssembly>();

    // serialize parent class
    ChAssembly::ArchiveOut(archive_out);

    // serialize all member data:

    archive_out << CHNVP(internal_bodylist, "internal_bodies");
    archive_out << CHNVP(internal_linklist, "internal_links");
    archive_out << CHNVP(internal_meshlist, "internal_meshes");
    archive_out << CHNVP(internal_otherphysicslist, "internal_other_physics_items");
    archive_out << CHNVP(m_is_model_reduced, "m_is_model_reduced");
    archive_out << CHNVP(modal_q, "modal_q");
    archive_out << CHNVP(modal_q_dt, "modal_q_dt");
    archive_out << CHNVP(modal_q_dtdt, "modal_q_dtdt");
    archive_out << CHNVP(full_forces_internal, "full_forces_internal");
    archive_out << CHNVP(internal_nodes_update, "internal_nodes_update");
}

void ChModalAssembly::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChModalAssembly>();

    // deserialize parent class
    ChAssembly::ArchiveIn(archive_in);

    // stream in all member data:

    // trick needed because the "AddIntenal...()" functions are required
    std::vector<std::shared_ptr<ChBody>> tempbodies;
    archive_in >> CHNVP(tempbodies, "internal_bodies");
    RemoveAllBodies();
    for (auto& body : tempbodies)
        AddInternalBody(body);
    std::vector<std::shared_ptr<ChLink>> templinks;
    archive_in >> CHNVP(templinks, "internal_links");
    RemoveAllLinks();
    for (auto& link : templinks)
        AddInternalLink(link);
    std::vector<std::shared_ptr<ChMesh>> tempmeshes;
    archive_in >> CHNVP(tempmeshes, "internal_mesh");
    RemoveAllMeshes();
    for (auto& mesh : tempmeshes)
        AddInternalMesh(mesh);
    std::vector<std::shared_ptr<ChPhysicsItem>> tempotherphysics;
    archive_in >> CHNVP(tempotherphysics, "internal_other_physics_items");
    RemoveAllOtherPhysicsItems();
    for (auto& mphys : tempotherphysics)
        AddInternalOtherPhysicsItem(mphys);

    archive_in >> CHNVP(m_is_model_reduced, "m_is_model_reduced");
    archive_in >> CHNVP(modal_q, "modal_q");
    archive_in >> CHNVP(modal_q_dt, "modal_q_dt");
    archive_in >> CHNVP(modal_q_dtdt, "modal_q_dtdt");
    archive_in >> CHNVP(full_forces_internal, "full_forces_internal");
    archive_in >> CHNVP(internal_nodes_update, "internal_nodes_update");

    // Recompute statistics, offsets, etc.
    Setup();
}

}  // end namespace modal

}  // end namespace chrono