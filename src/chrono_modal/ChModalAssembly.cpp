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
#include "chrono_modal/ChGeneralizedEigenvalueSolver.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/core/ChSparsityPatternLearner.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

namespace chrono {

using namespace fea;

namespace modal {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChModalAssembly)

ChModalAssembly::ChModalAssembly()
    : modal_variables(nullptr),
      m_num_coords_modal(0),
      m_num_coords_static_correction(0),
      m_is_model_reduced(false),
      m_internal_nodes_update(true),
      m_modal_automatic_gravity(true) {}

ChModalAssembly::ChModalAssembly(const ChModalAssembly& other) : ChAssembly(other) {
    m_modal_reduction_type = other.m_modal_reduction_type;
    m_num_coords_modal = other.m_num_coords_modal;
    m_num_coords_static_correction = other.m_num_coords_static_correction;
    m_is_model_reduced = other.m_is_model_reduced;
    m_internal_nodes_update = other.m_internal_nodes_update;
    m_modal_automatic_gravity = other.m_modal_automatic_gravity;

    modal_q = other.modal_q;
    modal_q_dt = other.modal_q_dt;
    modal_q_dtdt = other.modal_q_dtdt;

    m_full_forces_internal = other.m_full_forces_internal;

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

void ChModalAssembly::SetUseStaticCorrection(bool flag) {
    if (m_internal_nodes_update)
        m_num_coords_static_correction = flag ? 1 : 0;
    else  // if internal nodes are not required to update
        m_num_coords_static_correction = 0;
}

void ChModalAssembly::SetInternalNodesUpdate(bool flag) {
    m_internal_nodes_update = flag;
    if (!m_internal_nodes_update)
        m_num_coords_static_correction = 0;  // disable the static correction mode
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
void util_sparse_assembly_2x2symm(ChSparseMatrix& HCQ,       ///< resulting square sparse matrix
                                  const ChSparseMatrix& H,   ///< square sparse H matrix, [n_v, n_v]
                                  const ChSparseMatrix& Cq,  ///< rectangular sparse Cq [n_c, n_v]
                                  bool resize_and_reserve = true) {
    unsigned int n_v = H.rows();
    unsigned int n_c = Cq.rows();
    if (resize_and_reserve) {
        HCQ.resize(n_v + n_c, n_v + n_c);
        HCQ.reserve(H.nonZeros() + 2 * Cq.nonZeros());
    }
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

void util_convert_to_colmajor(
    Eigen::SparseMatrix<double, Eigen::ColMajor, int>& H_col,  ///< resulting sparse matrix (column major)
    const ChSparseMatrix& H)                                   ///< input sparse matrix (row major)
{
    H_col.resize(H.rows(), H.cols());
    H_col.reserve(H.nonZeros());
    H_col.setZero();

    for (unsigned int k = 0; k < H.outerSize(); ++k)
        for (ChSparseMatrix::InnerIterator it(H, k); it; ++it) {
            H_col.insert(it.row(), it.col()) = it.value();
        }

    // This seems necessary in Release mode
    H_col.makeCompressed();
}

//---------------------------------------------------------------------------------------

void ChModalAssembly::ComputeMassCenterFrame() {
    // Build a temporary mesh to collect all nodes and elements in the modal assembly because it happens
    // that the boundary nodes are added in the boundary 'meshlist' whereas their associated elements might
    // be in the 'internal_meshlist', leading to a mess in the mass computation.
    auto mesh_bou_int = chrono_types::make_shared<ChMesh>();
    // collect both boundary and internal meshes
    for (const auto& itemvec : {meshlist, internal_meshlist})
        for (const auto& item : itemvec) {
            if (const auto mesh = std::dynamic_pointer_cast<ChMesh>(item)) {
                for (auto& node : mesh->GetNodes())
                    mesh_bou_int->AddNode(node);
                for (auto& ele : mesh->GetElements())
                    mesh_bou_int->AddElement(ele);
            }
        }

    double mass_total = 0;
    ChVector3d mass_weighted_radius(0);
    ChMatrix33<> inertial_total(0);

    // for both boundary and internal bodies
    for (const auto& bodyvec : {bodylist, internal_bodylist})
        for (const auto& body : bodyvec) {
            if (body->IsActive()) {
                mass_total += body->GetMass();
                mass_weighted_radius += body->GetMass() * body->GetPos();
                inertial_total += body->GetInertia() +
                                  body->GetMass() * (body->GetPos().Length2() * ChMatrix33<>(1.0) -
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

    if (mass_total) {
        ChVector3d cog_x = mass_weighted_radius / mass_total;
        this->cog_frame.SetPos(cog_x);

        // The inertia tensor about cog, but still aligned with the absolute frame
        ChMatrix33<> inertia_cog = inertial_total - mass_total * (cog_x.Length2() * ChMatrix33<>(1.0) -
                                                                  cog_x.eigen() * cog_x.eigen().transpose());
        Eigen::EigenSolver<Eigen::MatrixXd> es(inertia_cog);
        ChVector3d prin_inertia = es.eigenvalues().real();  // principal moments of inertia: Jxx, Jyy, Jzz
        ChMatrix33<> prin_axis = es.eigenvectors().real();  // principal axes of inertia

        // the eigenvectors might do not comply with the right-hand rule of coordinate system,
        // thus we do it manually to construct the proper rotation matrix
        ChVector3d axis_X = prin_axis.col(0).normalized();
        ChVector3d axis_Y = prin_axis.col(1).normalized();
        ChVector3d axis_Z = Vcross(axis_X, axis_Y);
        ChMatrix33<> prin_axis_righthand(axis_X, axis_Y, axis_Z);

        ChQuaternion cog_qrot = prin_axis_righthand.GetQuaternion().GetNormalized();
        this->cog_frame.SetRot(cog_qrot);

    } else {
        // place at the position of the first boundary body/node of this modal assembly
        ChVector3d cog_x = m_full_state_x0.segment(0, 3);
        this->cog_frame.SetPos(cog_x);

        ChQuaternion cog_qrot = m_full_state_x0.segment(3, 4);
        this->cog_frame.SetRot(cog_qrot);

        std::cout << "Info: the center of mass is specified at the first boundary body/node of the modal assembly. "
                  << std::endl;
    }
}

void ChModalAssembly::UpdateFloatingFrameOfReference() {
    // If it is in full state, do nothing.
    if (!m_is_model_reduced)
        return;

    // when it is in the modal reduced state,
    // update the configuration of the floating frame F using Newton-Raphson iteration
    // to satisfy the constraint equation: C_F = U_loc^T * M * e_loc = 0.

    unsigned int num_coords_vel_bou_mod = m_num_coords_vel_boundary + m_num_coords_modal;

    auto ComputeConstraintResidualF = [&](ChVectorDynamic<>& mConstr_F) {
        this->UpdateTransformationMatrix();
        this->ComputeProjectionMatrix();

        ChVectorDynamic<> u_locred(num_coords_vel_bou_mod);
        ChVectorDynamic<> e_locred(num_coords_vel_bou_mod);
        ChVectorDynamic<> edt_locred(num_coords_vel_bou_mod);
        this->GetLocalDeformations(u_locred, e_locred, edt_locred);

        // the constraint vector C_F to eliminate the redundant DOFs of the floating frame F
        mConstr_F = this->U_locred_0.transpose() * this->M_red * e_locred;  // of size 6*1, expected to be zero
    };

    if (!m_tol_CF)
        m_tol_CF = 1.e-8 * this->M_red.norm();  // compute only once

    unsigned int ite_count = 0;
    unsigned int NR_limit = 10;
    bool converged_flag_F = false;

    while (!converged_flag_F && ite_count < NR_limit) {
        ChVectorDynamic<> constr_F(6);
        ComputeConstraintResidualF(constr_F);

        // Jacobian of the constraint vector C_F w.r.t. the floating frame F
        ChMatrixDynamic<> jac_F(6, 6);
        jac_F.setZero();
        jac_F = -this->U_locred_0.transpose() * this->M_red * this->U_locred * this->P_F.transpose();

        ChVectorDynamic<> delta_F(6);
        delta_F = jac_F.colPivHouseholderQr().solve(-constr_F);

        ChVector3d pos_F = this->floating_frame_F.GetPos() + delta_F.head(3);

        ChQuaternion<> incr_rotF(QNULL);
        incr_rotF.SetFromRotVec(ChVector3d(delta_F.tail(3)));  // rot.in local basis - as in system wise vectors
        ChQuaternion<> rot_F = this->floating_frame_F.GetRot() * incr_rotF;

        this->floating_frame_F.SetPos(pos_F);
        this->floating_frame_F.SetRot(rot_F);

        if (constr_F.norm() < m_tol_CF)
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

    ChVectorDynamic<> v_mod_loc(m_num_coords_vel);
    v_mod_loc.tail(m_num_coords_modal) = v_mod.tail(m_num_coords_modal);
    for (unsigned int i_node = 0; i_node < m_num_coords_vel_boundary / 6; ++i_node) {
        v_mod_loc.segment(6 * i_node, 3) = floating_frame_F.GetRot().RotateBack(v_mod.segment(6 * i_node, 3)).eigen();
        v_mod_loc.segment(6 * i_node + 3, 3) = v_mod.segment(6 * i_node + 3, 3);
    }

    // update the velocity of the floating frame F
    ChVectorDynamic<> vel_F(6);  // qdt_F
    vel_F = this->P_F * (this->Q_0 * v_mod_loc);
    this->floating_frame_F.SetPosDt(vel_F.head(3));
    this->floating_frame_F.SetAngVelLocal(vel_F.tail(3));

    // update again for safe
    this->UpdateTransformationMatrix();
    this->ComputeProjectionMatrix();

    ComputeConstraintResidualF(m_res_CF);

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
    // transformation matrix for the floating frame F
    P_F.setIdentity(6, 6);
    P_F.topLeftCorner(3, 3) = floating_frame_F.GetRotMat();

    unsigned int num_coords_pos_bou_mod = m_num_coords_pos_boundary + m_num_coords_modal;
    unsigned int num_coords_vel_bou_mod = m_num_coords_vel_boundary + m_num_coords_modal;

    // fetch the state snapshot (modal reduced)
    double fooT;
    ChState x_mod;       // =[qB; eta]
    ChStateDelta v_mod;  // =[qB_dt; eta_dt]
    x_mod.setZero(num_coords_pos_bou_mod, nullptr);
    v_mod.setZero(num_coords_vel_bou_mod, nullptr);
    this->IntStateGather(0, x_mod, 0, v_mod, fooT);

    //  rigid-body modes of boudnary bodies and nodes
    Uloc_B.resize(m_num_coords_vel_boundary, 6);
    Uloc_B.reserve(m_num_coords_vel_boundary * 3);  // n_B/6 nodes, 18 nonzeros for one node
    Uloc_B.setZero();
    for (unsigned int i_node = 0; i_node < m_num_coords_vel_boundary / 6; i_node++) {
        for (int k = 0; k < 3; ++k)
            Uloc_B.insert(6 * i_node + k, k) = 1.0;  // I33

        ChVector3d X_B =
            floating_frame_F.GetRot().RotateBack(ChVector3d(x_mod.segment(7 * i_node, 3)) - floating_frame_F.GetPos());
        // = - tilde(X_B)
        Uloc_B.insert(6 * i_node, 4) = X_B.z();
        Uloc_B.insert(6 * i_node, 5) = -X_B.y();
        Uloc_B.insert(6 * i_node + 1, 3) = -X_B.z();
        Uloc_B.insert(6 * i_node + 1, 5) = X_B.x();
        Uloc_B.insert(6 * i_node + 2, 3) = X_B.y();
        Uloc_B.insert(6 * i_node + 2, 4) = -X_B.x();

        ChQuaternion<> quat_bou = x_mod.segment(7 * i_node + 3, 4);
        ChMatrix33 RB_F = ChMatrix33<>(quat_bou.GetConjugate() * floating_frame_F.GetRot());
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                Uloc_B.insert(6 * i_node + 3 + r, 3 + c) = RB_F(r, c);  // RB^T*RF
    }
    Uloc_B.makeCompressed();

    //  rigid-body modes of internal bodies and nodes, only used in the Herting modal reduction procedure
    if (m_modal_reduction_type == ReductionType::HERTING &&
        (!Uloc_I.nonZeros()))  // Uloc_I is used in Herting reduction transformation once
    {
        Uloc_I.resize(m_num_coords_vel_internal, 6);
        Uloc_I.reserve(m_num_coords_vel_internal * 3);  // n_I/6 nodes, 18 nonzeros for one node
        Uloc_I.setZero();

        for (unsigned int i_node = 0; i_node < m_num_coords_vel_internal / 6; i_node++) {
            for (int k = 0; k < 3; ++k)
                Uloc_I.insert(6 * i_node + k, k) = 1.0;  // I33

            ChVector3d X_I = floating_frame_F.GetRot().RotateBack(
                ChVector3d(m_full_state_x.segment(m_num_coords_pos_boundary + 7 * i_node, 3)) -
                floating_frame_F.GetPos());
            // = - tilde(X_I)
            Uloc_I.insert(6 * i_node, 4) = X_I.z();
            Uloc_I.insert(6 * i_node, 5) = -X_I.y();
            Uloc_I.insert(6 * i_node + 1, 3) = -X_I.z();
            Uloc_I.insert(6 * i_node + 1, 5) = X_I.x();
            Uloc_I.insert(6 * i_node + 2, 3) = X_I.y();
            Uloc_I.insert(6 * i_node + 2, 4) = -X_I.x();

            ChQuaternion<> quat_int = m_full_state_x.segment(m_num_coords_pos_boundary + 7 * i_node + 3, 4);
            ChMatrix33 RI_F = ChMatrix33<>(quat_int.GetConjugate() * floating_frame_F.GetRot());
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    Uloc_I.insert(6 * i_node + 3 + r, 3 + c) = RI_F(r, c);  // RI^T*RF
        }
        Uloc_I.makeCompressed();
    }
}

void ChModalAssembly::ComputeProjectionMatrix() {
    unsigned int num_coords_vel_bou_mod = m_num_coords_vel_boundary + m_num_coords_modal;

    // Rigid-body modes U_locred in the deformed configuration are used to update the floating frame F
    U_locred.resize(num_coords_vel_bou_mod, 6);
    U_locred.reserve(Uloc_B.nonZeros());
    U_locred.setZero();
    for (unsigned int k = 0; k < Uloc_B.outerSize(); ++k)
        for (ChSparseMatrix::InnerIterator it(Uloc_B, k); it; ++it) {
            U_locred.insert(it.row(), it.col()) = it.value();
        }
    U_locred.makeCompressed();

    if (!is_projection_initialized) {
        this->U_locred_0 = this->U_locred;

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
    }
}

void ChModalAssembly::ComputeLocalFullKMCqMatrices(ChSparseMatrix& full_M,
                                                   ChSparseMatrix& full_K,
                                                   ChSparseMatrix& full_Cq) {
    ChSparseMatrix L_BI;

    unsigned int n_v = full_M.rows();
    L_BI.resize(n_v, n_v);
    L_BI.reserve(2 * n_v);  // n_v/6 nodes, 12 nonzeros in R_F and I33 for one node
    L_BI.setZero();

    unsigned int num_coords_vel_bou_int = m_num_coords_vel_boundary + m_num_coords_vel_internal;
    for (unsigned int i_node = 0; i_node < num_coords_vel_bou_int / 6; i_node++) {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                L_BI.insert(6 * i_node + r, 6 * i_node + c) = floating_frame_F.GetRotMat()(r, c);  // R_F

        for (int k = 0; k < 3; ++k)
            L_BI.insert(6 * i_node + 3 + k, 6 * i_node + 3 + k) = 1.0;  // I33
    }
    L_BI.makeCompressed();

    full_M_loc = L_BI.transpose() * full_M * L_BI;
    full_K_loc = L_BI.transpose() * full_K * L_BI;
    full_Cq_loc = full_Cq * L_BI;

    full_M_loc.makeCompressed();
    full_K_loc.makeCompressed();
    full_Cq_loc.makeCompressed();

    // temporarily retrieve the original local damping matrix
    // todo: develop a more reasonable modal damping model, and remove below code
    ChSparseMatrix full_R;
    this->GetSubassemblyMatrices(nullptr, &full_R, nullptr, nullptr);
    full_R_loc = L_BI.transpose() * full_R * L_BI;
    full_R_loc.makeCompressed();
}

void ChModalAssembly::PartitionLocalSystemMatrices() {
    // mass matrix
    M_BB_loc = this->full_M_loc.block(0, 0, m_num_coords_vel_boundary, m_num_coords_vel_boundary);
    M_BI_loc =
        this->full_M_loc.block(0, m_num_coords_vel_boundary, m_num_coords_vel_boundary, m_num_coords_vel_internal);
    M_IB_loc =
        this->full_M_loc.block(m_num_coords_vel_boundary, 0, m_num_coords_vel_internal, m_num_coords_vel_boundary);
    M_II_loc = this->full_M_loc.block(m_num_coords_vel_boundary, m_num_coords_vel_boundary, m_num_coords_vel_internal,
                                      m_num_coords_vel_internal);
    M_BB_loc.makeCompressed();
    M_BI_loc.makeCompressed();
    M_IB_loc.makeCompressed();
    M_II_loc.makeCompressed();

    // stiffness matrix
    K_BB_loc = this->full_K_loc.block(0, 0, m_num_coords_vel_boundary, m_num_coords_vel_boundary);
    K_BI_loc =
        this->full_K_loc.block(0, m_num_coords_vel_boundary, m_num_coords_vel_boundary, m_num_coords_vel_internal);
    K_IB_loc =
        this->full_K_loc.block(m_num_coords_vel_boundary, 0, m_num_coords_vel_internal, m_num_coords_vel_boundary);
    K_II_loc = this->full_K_loc.block(m_num_coords_vel_boundary, m_num_coords_vel_boundary, m_num_coords_vel_internal,
                                      m_num_coords_vel_internal);
    K_BB_loc.makeCompressed();
    K_BI_loc.makeCompressed();
    K_IB_loc.makeCompressed();
    K_II_loc.makeCompressed();

    // constraint matrix
    if (m_num_constr_internal) {
        Cq_I_loc = this->full_Cq_loc.bottomRows(m_num_constr_internal);
        Cq_IB_loc = this->full_Cq_loc.block(m_num_constr_boundary, 0, m_num_constr_internal, m_num_coords_vel_boundary);
        Cq_II_loc = this->full_Cq_loc.block(m_num_constr_boundary, m_num_coords_vel_boundary, m_num_constr_internal,
                                            m_num_coords_vel_internal);
        Cq_I_loc.makeCompressed();
        Cq_IB_loc.makeCompressed();
        Cq_II_loc.makeCompressed();
    }

    // extended K R M matrices with constraints in local frame
    if (m_num_constr_internal) {
        // scale Cq as preconditioning
        m_scaling_factor_CqI = full_K_loc.diagonal().mean();
        {
            // preallocate according to sparsity pattern
            unsigned int dim = full_K_loc.rows() + Cq_I_loc.rows();
            ChSparsityPatternLearner full_K_loc_ext_spl(dim, dim);
            full_K_loc_ext_spl.Apply(full_K_loc_ext);
        }
        util_sparse_assembly_2x2symm(full_K_loc_ext, full_K_loc, Cq_I_loc * m_scaling_factor_CqI, false);

        ChSparseMatrix temp_zero(Cq_I_loc.rows(), Cq_I_loc.cols());
        temp_zero.setZero();
        {
            // preallocate according to sparsity pattern
            unsigned int dim = full_M_loc.rows() + temp_zero.rows();
            ChSparsityPatternLearner full_M_loc_ext_spl(dim, dim);
            full_M_loc_ext_spl.Apply(full_M_loc_ext);
        }
        util_sparse_assembly_2x2symm(full_M_loc_ext, full_M_loc, temp_zero);
    } else {
        full_K_loc_ext = full_K_loc;
        full_M_loc_ext = full_M_loc;
    }
    full_K_loc_ext.makeCompressed();
    full_M_loc_ext.makeCompressed();
}

void ChModalAssembly::ApplyModeAccelerationTransformation(const ChModalDamping& damping_model) {
    if (m_modal_reduction_type == ReductionType::HERTING && m_modal_eigvect.cols() < 6) {
        std::cerr << "ChModalAssembly: at least six rigid-body modes are required for Herting reduction method"
                  << std::endl;
        throw std::invalid_argument("Error: at least six rigid-body modes are required for Herting reduction method.");
    }

    if (m_num_constr_boundary) {
        // It is forbidden to call AddLink() to connect internal bodies/nodes, thus Cq_BI should be zero.
        ChSparseMatrix Cq_BI_loc =
            full_Cq_loc.block(0, m_num_coords_vel_boundary, m_num_constr_boundary, m_num_coords_vel_internal);
        if (Cq_BI_loc.nonZeros())
            throw std::runtime_error(
                "Error: it is forbidden to use AddLink() to connect internal bodies/nodes in ChModalAssembly().");
    }

    // avoid computing K_IIc^{-1}, effectively do n times a linear solve:
    ChSparseMatrix H_II;
    if (m_num_constr_internal) {
        // K_IIc = [  K_II   Cq_II' ]
        //         [ Cq_II     0    ]
        util_sparse_assembly_2x2symm(H_II, K_II_loc, Cq_II_loc * m_scaling_factor_CqI);
        m_solver_invKIIc.analyzePattern(H_II);
        m_solver_invKIIc.factorize(H_II);
    } else {
        m_solver_invKIIc.analyzePattern(K_II_loc);
        m_solver_invKIIc.factorize(K_II_loc);
    }

    // 1) Matrix of static modes (constrained, so use K_IIc instead of K_II,
    // the original unconstrained static reduction is: Psi_S = - K_II^{-1} * K_IB.
    // for constrained subsystem:
    // Psi_S_C = {Psi_S; Psi_S_LambdaI} = - K_IIc^{-1} * {K_IB ; Cq_IB} )
    Psi_S.setZero(m_num_coords_vel_internal, m_num_coords_vel_boundary);
    if (m_num_constr_internal)
        Psi_S_LambdaI.setZero(m_num_constr_internal, m_num_coords_vel_boundary);
    // ChMatrixDynamic<> Psi_S_C(m_num_coords_vel_internal + m_num_constr_internal, m_num_coords_vel_boundary);

    for (unsigned int i = 0; i < m_num_coords_vel_boundary; ++i) {
        ChVectorDynamic<> rhs(m_num_coords_vel_internal + m_num_constr_internal);
        if (m_num_constr_internal)
            rhs << K_IB_loc.col(i).toDense(), Cq_IB_loc.col(i).toDense() * m_scaling_factor_CqI;
        else
            rhs << K_IB_loc.col(i).toDense();

        ChVectorDynamic<> x = m_solver_invKIIc.solve(rhs.sparseView());

        Psi_S.col(i) = -x.head(m_num_coords_vel_internal);
        // Psi_S_C.col(i) = -x;
        if (m_num_constr_internal)
            Psi_S_LambdaI.col(i) = -x.tail(m_num_constr_internal);
    }

    // 2) Matrix of dynamic modes (V_B and V_I already computed, reuse K_IIc already factored before.
    // the original unconstrained dynamic reduction is:
    //  - Herting:       Psi_D = - K_II^{-1} * (M_IB * V_B + M_II * V_I)
    //  - Craig-Bampton: Psi_D = - K_II^{-1} * (             M_II * V_I)
    // for constrained subsystem:
    // Psi_D_C = {Psi_D; Psi_D_LambdaI} = - K_IIc^{-1} * {(M_IB * V_B + M_II * V_I) ; 0} ).
    Psi_D.setZero(m_num_coords_vel_internal, m_num_coords_modal - m_num_coords_static_correction);
    if (m_num_constr_internal)
        Psi_D_LambdaI.setZero(m_num_constr_internal, m_num_coords_modal - m_num_coords_static_correction);
    // ChMatrixDynamic<> Psi_D_C(m_num_coords_vel_internal + m_num_constr_internal, m_num_coords_modal -
    // m_num_coords_static_correction);

    if (m_modal_reduction_type == ReductionType::HERTING) {  // for Herting reduction
        // The modal shapes of the first six rigid-body modes solved from the eigensolver might be not accurate,
        // leading to potential numerical instability. Thus, we construct the rigid-body modal shapes directly.
        m_modal_eigvect.block(0, 0, m_num_coords_vel_boundary, 6) = Uloc_B.toDense();
        m_modal_eigvect.block(m_num_coords_vel_boundary, 0, m_num_coords_vel_internal, 6) = Uloc_I.toDense();
    }

    ChMatrixDynamic<> rhs_dyn(m_num_coords_vel_internal, m_modal_eigvect.cols());
    if (m_modal_reduction_type == ReductionType::HERTING) {  // for Herting reduction
        ChMatrixDynamic<> V_B = m_modal_eigvect.topRows(m_num_coords_vel_boundary).real();
        ChMatrixDynamic<> V_I = m_modal_eigvect.middleRows(m_num_coords_vel_boundary, m_num_coords_vel_internal).real();
        rhs_dyn = M_IB_loc * V_B + M_II_loc * V_I;
    } else if (m_modal_reduction_type == ReductionType::CRAIG_BAMPTON) {  // for Craig-Bampton reduction
        ChMatrixDynamic<> V_I = m_modal_eigvect.real();
        rhs_dyn = M_II_loc * V_I;
    }

    for (unsigned int i = 0; i < m_num_coords_modal - m_num_coords_static_correction; ++i) {
        ChVectorDynamic<> rhs(m_num_coords_vel_internal + m_num_constr_internal);
        if (m_num_constr_internal)
            rhs << rhs_dyn.col(i), Eigen::VectorXd::Zero(m_num_constr_internal);
        else
            rhs << rhs_dyn.col(i);

        ChVectorDynamic<> x = m_solver_invKIIc.solve(rhs.sparseView());

        Psi_D.col(i) = -x.head(m_num_coords_vel_internal);
        // Psi_D_C.col(i) = -x;
        if (m_num_constr_internal)
            Psi_D_LambdaI.col(i) = -x.tail(m_num_constr_internal);
    }

    ChMatrixDynamic<> M_SS =
        M_BB_loc + M_BI_loc * Psi_S + Psi_S.transpose() * M_IB_loc + Psi_S.transpose() * M_II_loc * Psi_S;
    ChMatrixDynamic<> M_DD = Psi_D.transpose() * M_II_loc * Psi_D;

    // Find proper coefficients to normalize 'm_modal_eigvect' to improve the condition number of 'M_red'.
    ChVectorDynamic<> modes_scaling_factor(m_modal_eigvect.cols());
    modes_scaling_factor.setOnes();
    double expected_generalized_mass = M_SS.diagonal().mean();
    for (unsigned int i_mode = 0; i_mode < m_modal_eigvect.cols(); ++i_mode)
        if (M_DD(i_mode, i_mode))
            modes_scaling_factor(i_mode) = pow(expected_generalized_mass / M_DD(i_mode, i_mode), 0.5);

    // Scale eigenvectors of dynamic modes, to improve the numerical stability
    for (unsigned int i_mode = 0; i_mode < m_modal_eigvect.cols(); ++i_mode) {
        m_modal_eigvect.col(i_mode) *= modes_scaling_factor(i_mode);
        Psi_D.col(i_mode) *= modes_scaling_factor(i_mode);
        if (m_num_constr_internal)
            Psi_D_LambdaI.col(i_mode) *= modes_scaling_factor(i_mode);
    }

    // 3) Matrix of static correction mode. The external forces imposed on the internal nodes are required to compute
    // it, here it is initialized as a unit force vector.
    // the original unconstrained dynamic reduction is: Psi_Cor = K_II^{-1} * f_loc.
    // for constrained subsystem:
    // Psi_Cor_C = {Psi_Cor; Psi_Cor_LambdaI} = K_IIc^{-1} * {f_loc ; 0}
    Psi_Cor.setZero(m_num_coords_vel_internal, m_num_coords_static_correction);
    if (m_num_constr_internal)
        Psi_Cor_LambdaI.setZero(m_num_constr_internal, m_num_coords_static_correction);
    // ChMatrixDynamic<> Psi_Cor_C(m_num_coords_vel_internal + m_num_constr_internal, m_num_coords_static_correction);

    if (m_num_coords_static_correction) {
        ChVectorDynamic<> f_loc(m_num_coords_vel_internal);
        f_loc.setOnes();  // initialization
        ChVectorDynamic<> rhs(m_num_coords_vel_internal + m_num_constr_internal);
        if (m_num_constr_internal)
            rhs << f_loc, Eigen::VectorXd::Zero(m_num_constr_internal);
        else
            rhs << f_loc;

        ChVectorDynamic<> x = m_solver_invKIIc.solve(rhs.sparseView());

        Psi_Cor = x.head(m_num_coords_vel_internal);
        // Psi_Cor_C = x;
        if (m_num_constr_internal)
            Psi_Cor_LambdaI = x.tail(m_num_constr_internal);

        // Scale the eigenvector of the static correction mode, to improve the numerical stability
        ChMatrixDynamic<> M_rr = Psi_Cor.transpose() * M_II_loc * Psi_Cor;
        double static_scaling_factor = pow(expected_generalized_mass / M_rr(0, 0), 0.5);
        Psi_Cor *= static_scaling_factor;
        if (m_num_constr_internal)
            Psi_Cor_LambdaI *= static_scaling_factor;
    }

    // cols =   n_B      +      n_eta     +     1                      rows:
    // Psi  = [ I               0               0               ]      n_B
    //        [ Psi_S           Psi_D           Psi_Cor         ]    + n_I
    //        [ Psi_S_LambdaI   Psi_D_LambdaI   Psi_Cor_LambdaI ]    + n_LambdaI
    Psi.setZero(m_num_coords_vel_boundary + m_num_coords_vel_internal + m_num_constr_internal,
                m_num_coords_vel_boundary + m_num_coords_modal);
    //***TODO*** maybe prefer sparse Psi matrix, especially for upper blocks...
    if (m_num_constr_internal)
        Psi << Eigen::MatrixXd::Identity(m_num_coords_vel_boundary, m_num_coords_vel_boundary),
            Eigen::MatrixXd::Zero(m_num_coords_vel_boundary, m_num_coords_modal), Psi_S, Psi_D, Psi_Cor, Psi_S_LambdaI,
            Psi_D_LambdaI, Psi_Cor_LambdaI;
    else
        Psi << Eigen::MatrixXd::Identity(m_num_coords_vel_boundary, m_num_coords_vel_boundary),
            Eigen::MatrixXd::Zero(m_num_coords_vel_boundary, m_num_coords_modal), Psi_S, Psi_D, Psi_Cor;

    // Store this matrix, which will be used in the recomputation of the static correction mode in every time step
    MBI_PsiST_MII = M_BI_loc + Psi_S.transpose() * M_II_loc;
    MBI_PsiST_MII.makeCompressed();

    // Modal reduction transformation on the local M K matrices.
    // Now we assume there is no prestress in the initial configuration,
    // so only material mass and stiffness matrices are used here.
    this->M_red.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);
    this->M_red.topLeftCorner(m_num_coords_vel_boundary, m_num_coords_vel_boundary) = M_SS;

    this->M_red.block(0, m_num_coords_vel_boundary, m_num_coords_vel_boundary,
                      m_num_coords_modal - m_num_coords_static_correction) = MBI_PsiST_MII * Psi_D;
    this->M_red.block(m_num_coords_vel_boundary, 0, m_num_coords_modal - m_num_coords_static_correction,
                      m_num_coords_vel_boundary) = this->M_red
                                                       .block(0, m_num_coords_vel_boundary, m_num_coords_vel_boundary,
                                                              m_num_coords_modal - m_num_coords_static_correction)
                                                       .transpose();  // symmetric block
    this->M_red.block(m_num_coords_vel_boundary, m_num_coords_vel_boundary,
                      m_num_coords_modal - m_num_coords_static_correction,
                      m_num_coords_modal - m_num_coords_static_correction) = Psi_D.transpose() * M_II_loc * Psi_D;
    if (m_num_coords_static_correction) {  // static correction blocks
        this->M_red.block(0, m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction,
                          m_num_coords_vel_boundary, m_num_coords_static_correction) = MBI_PsiST_MII * Psi_Cor;
        this->M_red.block(m_num_coords_vel_boundary,
                          m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction,
                          m_num_coords_modal - m_num_coords_static_correction, m_num_coords_static_correction) =
            Psi_D.transpose() * M_II_loc * Psi_Cor;

        this->M_red.block(m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction, 0,
                          m_num_coords_static_correction, m_num_coords_vel_boundary) =
            this->M_red
                .block(0, m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction,
                       m_num_coords_vel_boundary, m_num_coords_static_correction)
                .transpose();  // symmetric block

        this->M_red.block(m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction,
                          m_num_coords_vel_boundary, m_num_coords_static_correction,
                          m_num_coords_modal - m_num_coords_static_correction) =
            this->M_red
                .block(m_num_coords_vel_boundary,
                       m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction,
                       m_num_coords_modal - m_num_coords_static_correction, m_num_coords_static_correction)
                .transpose();  // symmetric block

        this->M_red.bottomRightCorner(m_num_coords_static_correction, m_num_coords_static_correction) =
            Psi_Cor.transpose() * M_II_loc * Psi_Cor;
    }

    this->K_red.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);
    this->K_red.topLeftCorner(m_num_coords_vel_boundary, m_num_coords_vel_boundary) =
        K_BB_loc + K_BI_loc * Psi_S + Psi_S.transpose() * K_IB_loc + Psi_S.transpose() * K_II_loc * Psi_S;
    this->K_red.block(m_num_coords_vel_boundary, m_num_coords_vel_boundary,
                      m_num_coords_modal - m_num_coords_static_correction,
                      m_num_coords_modal - m_num_coords_static_correction) = Psi_D.transpose() * K_II_loc * Psi_D;
    if (m_num_coords_static_correction) {  // static correction blocks
        this->K_red.block(m_num_coords_vel_boundary,
                          m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction,
                          m_num_coords_modal - m_num_coords_static_correction, m_num_coords_static_correction) =
            Psi_D.transpose() * K_II_loc * Psi_Cor;

        this->K_red.block(m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction,
                          m_num_coords_vel_boundary, m_num_coords_static_correction,
                          m_num_coords_modal - m_num_coords_static_correction) =
            this->K_red
                .block(m_num_coords_vel_boundary,
                       m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction,
                       m_num_coords_modal - m_num_coords_static_correction, m_num_coords_static_correction)
                .transpose();  // symmetric block

        this->K_red.bottomRightCorner(m_num_coords_static_correction, m_num_coords_static_correction) =
            Psi_Cor.transpose() * K_II_loc * Psi_Cor;
    }

    {
        // Initialize the reduced damping matrix
        this->R_red.setZero(this->K_red.rows(), this->K_red.cols());
        // Modal reduction of R damping matrix: compute using user-provided damping model.
        damping_model.ComputeR(*this, this->M_red, this->K_red, Psi, this->R_red);
    }

    // For strict symmetry, copy L=U because the computations above might lead to small errors because of numerical
    // roundoff
    for (int row = 0; row < this->M_red.rows() - 1; ++row)
        for (int col = row + 1; col < this->M_red.cols(); ++col) {
            this->M_red(row, col) = this->M_red(col, row);
            this->K_red(row, col) = this->K_red(col, row);

            // todo: maybe need to remove after completing the development of a proper modal damping model
            this->R_red(row, col) = this->R_red(col, row);
        }

    // Reset to zero all the atomic masses of the boundary nodes because now their mass is represented by
    // this->modal_M.
    // NOTE! this should be made more generic and future-proof by implementing a virtual method ex.
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
    m_modal_damping_ratios.resize(0);
    m_modal_eigvals.resize(0);
    m_modal_freq.resize(0);
    m_modal_eigvect.resize(0, 0);
}

void ChModalAssembly::UpdateStaticCorrectionMode() {
    if (!m_num_coords_static_correction)
        return;

    // Psi_Cor_C = {Psi_Cor; Psi_Cor_LambdaI} = K_IIc^{-1} * {f_loc ; 0}
    Psi_Cor.setZero(m_num_coords_vel_internal, m_num_coords_static_correction);
    if (m_num_constr_internal)
        Psi_Cor_LambdaI.setZero(m_num_constr_internal, m_num_coords_static_correction);
    // ChMatrixDynamic<> Psi_Cor_C(m_num_coords_vel_internal + m_num_constr_internal,
    // m_num_coords_static_correction);

    ChVectorDynamic<> f_loc(m_num_coords_vel_internal);
    // f_loc.setOnes();//initialization
    if (!m_full_forces_internal.isZero()) {
        for (unsigned int i_node = 0; i_node < m_num_coords_vel_internal / 6; ++i_node) {
            f_loc.segment(6 * i_node, 3) =
                floating_frame_F.GetRot().RotateBack(m_full_forces_internal.segment(6 * i_node, 3)).eigen();
            f_loc.segment(6 * i_node + 3, 3) = m_full_forces_internal.segment(6 * i_node + 3, 3);
        }
    }
    // else {
    //    // todo:
    //    // When the external forces imposed on the internal nodes disappear suddenly, or change too fast,
    //    // there is an impusle in the system response due to the change of the static correction modal basis.
    //    // How to optimize further?
    // }
    ChVectorDynamic<> rhs(m_num_coords_vel_internal + m_num_constr_internal);
    if (m_num_constr_internal)
        rhs << f_loc, Eigen::VectorXd::Zero(m_num_constr_internal);
    else
        rhs << f_loc;

    ChVectorDynamic<> x = m_solver_invKIIc.solve(rhs.sparseView());

    Psi_Cor = x.head(m_num_coords_vel_internal);
    // Psi_Cor_C = x;
    if (m_num_constr_internal)
        Psi_Cor_LambdaI = x.tail(m_num_constr_internal);

    // IMPORTANT: scale the static correction mode to improve the numerical stability
    double expected_generalized_mass =
        M_red.topLeftCorner(m_num_coords_vel_boundary, m_num_coords_vel_boundary).diagonal().mean();
    ChMatrixDynamic<> M_rr = Psi_Cor.transpose() * M_II_loc * Psi_Cor;
    double static_scaling_factor = pow(expected_generalized_mass / M_rr(0, 0), 0.5);
    Psi_Cor *= static_scaling_factor;
    if (m_num_constr_internal)
        Psi_Cor_LambdaI *= static_scaling_factor;

    // update Psi
    Psi.block(m_num_coords_vel_boundary,
              m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction,
              m_num_coords_vel_internal, m_num_coords_static_correction) = Psi_Cor;
    if (m_num_constr_internal)
        Psi.bottomRightCorner(m_num_constr_internal, m_num_coords_static_correction) = Psi_Cor_LambdaI;
}

ChMatrixDynamic<> ChModalAssembly::GetCorotationalTransformation(const ChMatrixDynamic<>& H) {
    ChMatrixDynamic<> H_out = H;
    for (unsigned int r = 0; r < m_num_coords_vel_boundary / 6; ++r)
        for (unsigned int c = 0; c < m_num_coords_vel_boundary / 6; ++c) {
            H_out.block<3, 3>(6 * r, 6 * c) =
                floating_frame_F.GetRotMat() * H.block<3, 3>(6 * r, 6 * c) * floating_frame_F.GetRotMat().transpose();
            H_out.block<3, 3>(6 * r + 3, 6 * c) =
                H.block<3, 3>(6 * r + 3, 6 * c) * floating_frame_F.GetRotMat().transpose();
            H_out.block<3, 3>(6 * r, 6 * c + 3) = floating_frame_F.GetRotMat() * H.block<3, 3>(6 * r, 6 * c + 3);
        }

    for (unsigned int i_node = 0; i_node < m_num_coords_vel_boundary / 6; ++i_node) {
        // top-right block
        H_out.block(6 * i_node, m_num_coords_vel_boundary, 3, m_num_coords_modal) =
            floating_frame_F.GetRotMat() * H.block(6 * i_node, m_num_coords_vel_boundary, 3, m_num_coords_modal);

        // bottom-left block
        H_out.block(m_num_coords_vel_boundary, 6 * i_node, m_num_coords_modal, 3) =
            H.block(m_num_coords_vel_boundary, 6 * i_node, m_num_coords_modal, 3) *
            floating_frame_F.GetRotMat().transpose();
    }

    return H_out;
}

void ChModalAssembly::ComputeModalKRMmatricesGlobal(double Kfactor, double Rfactor, double Mfactor) {
    if (!m_is_model_reduced)
        return;

    // Update the reduced M K R matrices, but only for those blocks affected by the static correction mode
    if (m_num_coords_static_correction) {
        // Update the blocks of reduced mass matrix corresponding to the static correction mode
        this->M_red.block(0, m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction,
                          m_num_coords_vel_boundary, m_num_coords_static_correction) = MBI_PsiST_MII * Psi_Cor;
        this->M_red.block(m_num_coords_vel_boundary,
                          m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction,
                          m_num_coords_modal - m_num_coords_static_correction, m_num_coords_static_correction) =
            Psi_D.transpose() * M_II_loc * Psi_Cor;

        this->M_red.block(m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction, 0,
                          m_num_coords_static_correction, m_num_coords_vel_boundary) =
            this->M_red
                .block(0, m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction,
                       m_num_coords_vel_boundary, m_num_coords_static_correction)
                .transpose();  // symmetric block

        this->M_red.block(m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction,
                          m_num_coords_vel_boundary, m_num_coords_static_correction,
                          m_num_coords_modal - m_num_coords_static_correction) =
            this->M_red
                .block(m_num_coords_vel_boundary,
                       m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction,
                       m_num_coords_modal - m_num_coords_static_correction, m_num_coords_static_correction)
                .transpose();  // symmetric block

        this->M_red.bottomRightCorner(m_num_coords_static_correction, m_num_coords_static_correction) =
            Psi_Cor.transpose() * M_II_loc * Psi_Cor;
    }

    if (m_num_coords_static_correction) {
        // Update the blocks of reduced stiffness matrix corresponding to the static correction mode
        this->K_red.block(m_num_coords_vel_boundary,
                          m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction,
                          m_num_coords_modal - m_num_coords_static_correction, m_num_coords_static_correction) =
            Psi_D.transpose() * K_II_loc * Psi_Cor;

        this->K_red.block(m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction,
                          m_num_coords_vel_boundary, m_num_coords_static_correction,
                          m_num_coords_modal - m_num_coords_static_correction) =
            this->K_red
                .block(m_num_coords_vel_boundary,
                       m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction,
                       m_num_coords_modal - m_num_coords_static_correction, m_num_coords_static_correction)
                .transpose();  // symmetric block

        this->K_red.bottomRightCorner(m_num_coords_static_correction, m_num_coords_static_correction) =
            Psi_Cor.transpose() * K_II_loc * Psi_Cor;
    }

    // Since we might do not have the information of R_II_loc, we have to neglect the effect of the static correction
    // mode in terms of damping
    // if (m_num_coords_static_correction) {
    //    // Update the blocks of reduced damping matrix corresponding to the static correction mode
    //    this->R_red.block(m_num_coords_vel_boundary,
    //                      m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction,
    //                      m_num_coords_modal - m_num_coords_static_correction, m_num_coords_static_correction) =
    //        Psi_D.transpose() * R_II_loc * Psi_Cor;

    //    this->R_red.block(m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction,
    //                      m_num_coords_vel_boundary, m_num_coords_static_correction,
    //                      m_num_coords_modal - m_num_coords_static_correction) =
    //        this->R_red
    //            .block(m_num_coords_vel_boundary,
    //                   m_num_coords_vel_boundary + m_num_coords_modal - m_num_coords_static_correction,
    //                   m_num_coords_modal - m_num_coords_static_correction, m_num_coords_static_correction)
    //            .transpose();  // symmetric block

    //    this->R_red.bottomRightCorner(m_num_coords_static_correction, m_num_coords_static_correction) =
    //        Psi_Cor.transpose() * R_II_loc * Psi_Cor;
    //}

    this->modal_M.setZero();
    this->modal_K.setZero();
    this->modal_R.setZero();

    // Inertial mass matrix
    if (Mfactor)
        this->modal_M = GetCorotationalTransformation(M_red);

    if (m_num_coords_static_correction || !(PTKredP.any()))  // avoid duplicate computing if possible
        PTKredP = P_perp_0.transpose() * K_red * P_perp_0;

    // material stiffness matrix of reduced modal assembly
    if (Kfactor)
        this->modal_K = GetCorotationalTransformation(PTKredP);

    if (m_num_coords_static_correction || !(PTRredP.any()))  // avoid duplicate computing if possible
        PTRredP = P_perp_0.transpose() * R_red * P_perp_0;

    // material damping matrix of the reduced modal assembly
    if (Rfactor)
        this->modal_R = GetCorotationalTransformation(PTRredP);

    unsigned int num_coords_pos_bou_mod = m_num_coords_pos_boundary + m_num_coords_modal;
    unsigned int num_coords_vel_bou_mod = m_num_coords_vel_boundary + m_num_coords_modal;

    double fooT;
    ChState x_mod;       // =[qB; eta]
    ChStateDelta v_mod;  // =[qB_dt; eta_dt]
    x_mod.setZero(num_coords_pos_bou_mod, nullptr);
    v_mod.setZero(num_coords_vel_bou_mod, nullptr);
    this->IntStateGather(0, x_mod, 0, v_mod, fooT);

    // geometric stiffness matrix of reduced modal assembly
    if (Kfactor) {
        ChVectorDynamic<> u_locred(num_coords_vel_bou_mod);
        ChVectorDynamic<> e_locred(num_coords_vel_bou_mod);
        ChVectorDynamic<> edt_locred(num_coords_vel_bou_mod);
        this->GetLocalDeformations(u_locred, e_locred, edt_locred);

        ChVectorDynamic<> g_loc_alpha(num_coords_vel_bou_mod);
        g_loc_alpha = P_perp_0.transpose() * (K_red * e_locred);

        ChMatrixDynamic<> V_F1;
        V_F1.setZero(num_coords_vel_bou_mod, 6);
        ChMatrixDynamic<> V_F2;
        V_F2.setZero(num_coords_vel_bou_mod, 6);

        for (unsigned int i_bou = 0; i_bou < m_num_coords_vel_boundary / 6; i_bou++) {
            V_F1.block<3, 3>(6 * i_bou, 3) = ChStarMatrix33<>(g_loc_alpha.segment(6 * i_bou, 3));
            V_F2.block<3, 3>(6 * i_bou, 3) = ChStarMatrix33<>(u_locred.segment(6 * i_bou, 3));
        }
        this->modal_K.noalias() += GetCorotationalTransformation((-V_F1 + PTKredP * V_F2) * P_F * Q_0);
    }

    ChMatrixDynamic<> O_F;
    O_F.setZero(num_coords_vel_bou_mod, num_coords_vel_bou_mod);
    for (unsigned int i_bou = 0; i_bou < m_num_coords_vel_boundary / 6; i_bou++)
        O_F.block<3, 3>(6 * i_bou, 6 * i_bou) = ChStarMatrix33<>(floating_frame_F.GetAngVelLocal());

    // Inertial damping matrix, also known as gyroscopic damping matrix
    if (Rfactor)
        this->modal_R.noalias() += GetCorotationalTransformation(O_F * M_red);

    // Inertial stiffness matrix, is zero

    if (!m_use_linear_inertial_term && (Rfactor || Kfactor)) {
        // For internal test:
        // the below code might cause numerical instabilities

        ChStateDelta a_mod;  // =[qB_dtdt; eta_dtdt]
        a_mod.setZero(num_coords_vel_bou_mod, nullptr);
        this->IntStateGatherAcceleration(0, a_mod);

        ChMatrixDynamic<> V;
        V.setZero(num_coords_vel_bou_mod, 6);
        for (unsigned int i_bou = 0; i_bou < m_num_coords_vel_boundary / 6; i_bou++) {
            V.block<3, 3>(6 * i_bou, 3) =
                ChStarMatrix33<>(floating_frame_F.GetRot().RotateBack(v_mod.segment(6 * i_bou, 3)));
        }

        ChVectorDynamic<> v_mod_loc(m_num_coords_vel);
        v_mod_loc.tail(m_num_coords_modal) = v_mod.tail(m_num_coords_modal);
        for (unsigned int i_node = 0; i_node < m_num_coords_vel_boundary / 6; ++i_node) {
            v_mod_loc.segment(6 * i_node, 3) =
                floating_frame_F.GetRot().RotateBack(v_mod.segment(6 * i_node, 3)).eigen();
            v_mod_loc.segment(6 * i_node + 3, 3) = v_mod.segment(6 * i_node + 3, 3);
        }

        ChMatrixDynamic<> V_rmom;
        V_rmom.setZero(num_coords_vel_bou_mod, 6);
        ChVectorDynamic<> momen = M_red * v_mod_loc;
        for (unsigned int i_bou = 0; i_bou < m_num_coords_vel_boundary / 6; i_bou++) {
            V_rmom.block<3, 3>(6 * i_bou, 3) = ChStarMatrix33<>(momen.segment(6 * i_bou, 3));
        }
        ChMatrixDynamic<> MVPFQ = M_red * V * P_F * Q_0;
        ChMatrixDynamic<> VrPFQ = V_rmom * P_F * Q_0;

        if (Rfactor) {
            // this->modal_R.noalias() += P_W * (-M_red * O_F) * P_W.transpose();
            // this->modal_R.noalias() += P_W * (MVPFQ - MVPFQ.transpose()) * P_W.transpose();
            // this->modal_R.noalias() += P_W * (VrPFQ.transpose() - VrPFQ) * P_W.transpose();
            this->modal_R.noalias() +=
                GetCorotationalTransformation(-M_red * O_F + (MVPFQ - MVPFQ.transpose()) + (VrPFQ.transpose() - VrPFQ));
        }

        //{  // Leading to divergence. DO NOT use it.
        //    ChMatrixDynamic<> O_B;
        //    O_B.setZero(num_coords_vel_bou_mod, num_coords_vel_bou_mod);
        //    for (unsigned int i_bou = 0; i_bou < m_num_coords_vel_boundary / 6; i_bou++) {
        //        O_B.block(6 * i_bou + 3, 6 * i_bou + 3, 3, 3) = ChStarMatrix33<>(v_mod.segment(6 * i_bou + 3, 3));
        //    }
        //    ChMatrixDynamic<> O_thetamom;
        //    O_thetamom.setZero(num_coords_vel_bou_mod, num_coords_vel_bou_mod);
        //    for (unsigned int i_bou = 0; i_bou < m_num_coords_vel_boundary / 6; i_bou++) {
        //        O_thetamom.block(6 * i_bou + 3, 6 * i_bou + 3, 3, 3) =
        //            ChStarMatrix33<>(momen.segment(6 * i_bou + 3, 3));
        //    }
        //    this->modal_R.noalias() += -O_thetamom + O_B * M_red * P_W.transpose();

        //    ///*******************************************///
        //    // Inertial stiffness matrix. Harmful for numerical integration, DO NOT use it.
        //    ChVectorDynamic<> f_loc_C = M_red * (P_W.transpose() * a_mod) +
        //                                ((O_F + O_B) * M_red + MVPFQ - MVPFQ.transpose()) * (P_W.transpose() * v_mod);
        //    ChMatrixDynamic<> V_iner;
        //    ChMatrixDynamic<> V_acc;
        //    V_iner.setZero(num_coords_vel_bou_mod, 6);
        //    V_acc.setZero(num_coords_vel_bou_mod, 6);
        //    for (unsigned int i_bou = 0; i_bou < m_num_coords_vel_boundary / 6; i_bou++) {
        //        V_iner.block(6 * i_bou, 3, 3, 3) = ChStarMatrix33<>(f_loc_C.segment(6 * i_bou, 3));
        //        V_acc.block(6 * i_bou, 3, 3, 3) =
        //            ChStarMatrix33<>(floating_frame_F.GetRot().RotateBack(a_mod.segment(6 * i_bou, 3)));
        //    }

        //    ChVectorDynamic<> h_loc_alpha(6);
        //    h_loc_alpha = Q_0 * (P_W.transpose() * v_mod);
        //    ChVector3d VF_alpha = h_loc_alpha.head(3);
        //    ChMatrixDynamic<> V_alpha;
        //    V_alpha.setZero(6, 6);
        //    V_alpha.block(0, 3, 3, 3) = -floating_frame_F.GetRotMat() * ChStarMatrix33<>(VF_alpha);

        //    ChVectorDynamic<> h_loc_beta(6);
        //    h_loc_beta = V.transpose() * M_red * (P_W.transpose() * v_mod);
        //    ChVector3d VF_beta = h_loc_beta.head(3);
        //    ChMatrixDynamic<> V_beta;
        //    V_beta.setZero(6, 6);
        //    V_beta.block(0, 3, 3, 3) = ChStarMatrix33<>(floating_frame_F.GetRotMat().transpose() * VF_beta);

        //    ChMatrixDynamic<> PFQPWT = P_F * Q_0 * P_W.transpose();
        //    this->modal_K.noalias() += P_W * (M_red * V_acc - V_iner) * PFQPWT +
        //                     P_W * ((O_F + O_B) * M_red + MVPFQ - MVPFQ.transpose()) * V * PFQPWT -
        //                     P_W * V_rmom * (V_alpha + P_F * Q_0 * V) * PFQPWT - P_W * M_red * O_F * V * PFQPWT +
        //                     P_W * Q_0.transpose() * P_F.transpose() * V_rmom.transpose() * V * PFQPWT +
        //                     P_W * M_red * V * V_alpha * PFQPWT - P_W * Q_0.transpose() * V_beta * PFQPWT;
        //}
    }
}

void ChModalAssembly::SetupModalData(unsigned int nmodes_reduction) {
    // modal coordinates are composed of two parts:
    //  - the elastic modal coordinates, \eta
    //  - the static correction mode, \eta_cor
    m_num_coords_modal = nmodes_reduction + m_num_coords_static_correction;

    this->Setup();

    Uloc_I.resize(m_num_coords_vel_internal, 6);
    Uloc_I.reserve(m_num_coords_vel_internal * 3);  // n_I/6 nodes, 18 nonzeros for one node
    Uloc_I.setZero();

    P_F.setZero(6, 6);

    Q_0.setZero(6, m_num_coords_vel_boundary + m_num_coords_modal);
    P_parallel_0.setZero(m_num_coords_vel_boundary + m_num_coords_modal,
                         m_num_coords_vel_boundary + m_num_coords_modal);
    P_perp_0.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);
    PTKredP.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);
    PTRredP.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);

    M_red.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);
    K_red.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);
    R_red.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);

    modal_M.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);
    modal_K.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);
    modal_R.setZero(m_num_coords_vel_boundary + m_num_coords_modal, m_num_coords_vel_boundary + m_num_coords_modal);

    if (!modal_variables || (modal_variables->GetDOF() != m_num_coords_modal)) {
        // Initialize ChVariable object used for modal variables
        if (modal_variables)
            delete modal_variables;
        modal_variables = new ChVariablesGenericDiagonalMass(m_num_coords_modal);
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
        this->modal_q.setZero(m_num_coords_modal);
        this->modal_q_dt.setZero(m_num_coords_modal);
        this->modal_q_dtdt.setZero(m_num_coords_modal);
        m_full_forces_internal.setZero(m_num_coords_vel_internal);
    }
}

void ChModalAssembly::UpdateInternalState(bool update_assets) {
    if (!m_is_model_reduced)
        return;

    unsigned int num_coords_pos_bou_int = m_num_coords_pos_boundary + m_num_coords_pos_internal;
    unsigned int num_coords_vel_bou_int = m_num_coords_vel_boundary + m_num_coords_vel_internal;
    unsigned int num_coords_pos_bou_mod = m_num_coords_pos_boundary + m_num_coords_modal;
    unsigned int num_coords_vel_bou_mod = m_num_coords_vel_boundary + m_num_coords_modal;

    if (this->Psi.rows() != (num_coords_vel_bou_int + m_num_constr_internal) ||
        this->Psi.cols() != num_coords_vel_bou_mod)
        return;

    double fooT;
    ChState x_mod;       // =[qB; eta]
    ChStateDelta v_mod;  // =[qB_dt; eta_dt]
    x_mod.setZero(num_coords_pos_bou_mod, nullptr);
    v_mod.setZero(num_coords_vel_bou_mod, nullptr);
    this->IntStateGather(0, x_mod, 0, v_mod, fooT);

    // Update w.r.t. the initial undeformed configuration

    ChVectorDynamic<> u_locred(num_coords_vel_bou_mod);
    ChVectorDynamic<> e_locred(num_coords_vel_bou_mod);
    ChVectorDynamic<> edt_locred(num_coords_vel_bou_mod);
    this->GetLocalDeformations(u_locred, e_locred, edt_locred);

    // the local deformation of internal bodies and nodes
    ChStateDelta Dx_internal_loc;  // =[delta_qI^bar]
    Dx_internal_loc.setZero(m_num_coords_vel_internal, nullptr);
    Dx_internal_loc.segment(0, m_num_coords_vel_internal) =
        Psi_S * e_locred.segment(0, m_num_coords_vel_boundary) +
        Psi_D * e_locred.segment(m_num_coords_vel_boundary, m_num_coords_modal - m_num_coords_static_correction);
    // add the contribution of the static correction mode
    if (m_num_coords_static_correction)
        Dx_internal_loc.segment(0, m_num_coords_vel_internal) +=
            Psi_Cor * e_locred.tail(m_num_coords_static_correction);

    // the new configuration of both boundary and internal containers
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

    // the new velocity of both boundary and internal containers
    ChStateDelta assembly_v_new;  // =[qB_dt; qI_dt]
    assembly_v_new.setZero(num_coords_vel_bou_int, nullptr);
    assembly_v_new.segment(0, m_num_coords_vel_boundary) = v_mod.segment(0, m_num_coords_vel_boundary);
    // recover the velocity of internal nodes
    ChVectorDynamic<> vloc_bou(m_num_coords_vel_boundary);
    for (unsigned int i_node = 0; i_node < m_num_coords_vel_boundary / 6; ++i_node) {
        vloc_bou.segment(6 * i_node, 3) = floating_frame_F.GetRot().RotateBack(v_mod.segment(6 * i_node, 3)).eigen();
        vloc_bou.segment(6 * i_node + 3, 3) = v_mod.segment(6 * i_node + 3, 3);
    }
    ChVectorDynamic<> vloc_int =
        Psi_S * vloc_bou +
        Psi_D * v_mod.segment(m_num_coords_vel_boundary, m_num_coords_modal - m_num_coords_static_correction);
    ChVectorDynamic<> vpar_int(m_num_coords_vel_internal);
    for (unsigned int i_node = 0; i_node < m_num_coords_vel_internal / 6; ++i_node) {
        vpar_int.segment(6 * i_node, 3) = floating_frame_F.GetRot().Rotate(vloc_int.segment(6 * i_node, 3)).eigen();
        vpar_int.segment(6 * i_node + 3, 3) = vloc_int.segment(6 * i_node + 3, 3);
    }
    assembly_v_new.segment(m_num_coords_vel_boundary, m_num_coords_vel_internal) = vpar_int;
    // add the contribution of the static correction mode
    if (m_num_coords_static_correction) {
        ChVectorDynamic<> vloc_int_static = Psi_Cor * v_mod.tail(m_num_coords_static_correction);
        ChVectorDynamic<> vpar_int_static(m_num_coords_vel_internal);
        for (unsigned int i_node = 0; i_node < m_num_coords_vel_internal / 6; ++i_node) {
            vpar_int_static.segment(6 * i_node, 3) =
                floating_frame_F.GetRot().Rotate(vloc_int_static.segment(6 * i_node, 3)).eigen();
            vpar_int_static.segment(6 * i_node + 3, 3) = vloc_int_static.segment(6 * i_node + 3, 3);
        }
        assembly_v_new.segment(m_num_coords_vel_boundary, m_num_coords_vel_internal) += vpar_int_static;
    }

    bool needs_temporary_bou_int = m_is_model_reduced;
    if (needs_temporary_bou_int)
        m_is_model_reduced = false;

    // scatter to internal nodes only and update them
    double T = this->GetChTime();
    for (auto& body : internal_bodylist) {
        if (body->IsActive())
            body->IntStateScatter(body->GetOffset_x() - this->offset_x, assembly_x_new,
                                  body->GetOffset_w() - this->offset_w, assembly_v_new, T, update_assets);
        else
            body->Update(T, update_assets);
    }
    for (auto& mesh : internal_meshlist) {
        mesh->IntStateScatter(mesh->GetOffset_x() - this->offset_x, assembly_x_new,
                              mesh->GetOffset_w() - this->offset_w, assembly_v_new, T, update_assets);
    }
    for (auto& link : internal_linklist) {
        if (link->IsActive())
            link->IntStateScatter(link->GetOffset_x() - this->offset_x, assembly_x_new,
                                  link->GetOffset_w() - this->offset_w, assembly_v_new, T, update_assets);
        else
            link->Update(T, update_assets);
    }
    for (auto& item : internal_otherphysicslist) {
        if (item->IsActive())
            item->IntStateScatter(item->GetOffset_x() - this->offset_x, assembly_x_new,
                                  item->GetOffset_w() - this->offset_w, assembly_v_new, T, update_assets);
    }

    if (needs_temporary_bou_int)
        m_is_model_reduced = true;

    // store the full state for the computation in next time step
    this->m_full_state_x = assembly_x_new;
}

void ChModalAssembly::SetFullStateReset() {
    if (m_full_state_x0.rows() != m_num_coords_pos)
        return;

    double fooT = 0;
    ChStateDelta assembly_v;

    assembly_v.setZero(m_num_coords_vel, nullptr);

    this->IntStateScatter(0, m_full_state_x0, 0, assembly_v, fooT, true);

    this->Update();
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

const std::vector<std::shared_ptr<ChBody>>& ChModalAssembly::GetBodies() const {
    if (!system->is_updated) {
        bodylist_total.resize(bodylist.size() + internal_bodylist.size());
        for (auto boundary_sel = 0; boundary_sel < bodylist.size(); ++boundary_sel)
            bodylist_total[boundary_sel] = bodylist[boundary_sel];

        for (auto internal_sel = 0; internal_sel < internal_bodylist.size(); ++internal_sel)
            bodylist_total[internal_sel + bodylist.size()] = internal_bodylist[internal_sel];
    }

    return bodylist_total;
}

const std::vector<std::shared_ptr<ChLinkBase>>& ChModalAssembly::GetLinks() const {
    if (!system->is_updated) {
        linklist_total.resize(linklist.size() + internal_linklist.size());
        for (auto boundary_sel = 0; boundary_sel < linklist.size(); ++boundary_sel)
            linklist_total[boundary_sel] = linklist[boundary_sel];

        for (auto internal_sel = 0; internal_sel < internal_linklist.size(); ++internal_sel)
            linklist_total[internal_sel + linklist.size()] = internal_linklist[internal_sel];
    }

    return linklist_total;
}

const std::vector<std::shared_ptr<fea::ChMesh>>& ChModalAssembly::GetMeshes() const {
    if (!system->is_updated) {
        meshlist_total.resize(meshlist.size() + internal_meshlist.size());
        for (auto boundary_sel = 0; boundary_sel < meshlist.size(); ++boundary_sel)
            meshlist_total[boundary_sel] = meshlist[boundary_sel];

        for (auto internal_sel = 0; internal_sel < internal_meshlist.size(); ++internal_sel)
            meshlist_total[internal_sel + meshlist.size()] = internal_meshlist[internal_sel];
    }

    return meshlist_total;
}

const std::vector<std::shared_ptr<ChPhysicsItem>>& ChModalAssembly::GetOtherPhysicsItems() const {
    if (!system->is_updated) {
        otherphysicslist_total.resize(otherphysicslist.size() + internal_otherphysicslist.size());
        for (auto boundary_sel = 0; boundary_sel < otherphysicslist.size(); ++boundary_sel)
            otherphysicslist_total[boundary_sel] = otherphysicslist[boundary_sel];

        for (auto internal_sel = 0; internal_sel < internal_otherphysicslist.size(); ++internal_sel)
            otherphysicslist_total[internal_sel + otherphysicslist.size()] = internal_otherphysicslist[internal_sel];
    }

    return otherphysicslist_total;
}

// -----------------------------------------------------------------------------

void ChModalAssembly::GetSubassemblyMatrices(ChSparseMatrix* K,
                                             ChSparseMatrix* R,
                                             ChSparseMatrix* M,
                                             ChSparseMatrix* Cq) {
    this->SetupInitial();
    this->Setup();
    this->Update();

    ChSystemDescriptor temp_descriptor;

    this->InjectVariables(temp_descriptor);
    this->InjectKRMMatrices(temp_descriptor);
    this->InjectConstraints(temp_descriptor);

    if (K) {
        // Load all KRM matrices with the K part only
        this->LoadKRMMatrices(1.0, 0, 0);
        // For ChVariable objects without a ChKRMBlock, but still with a mass:
        temp_descriptor.SetMassFactor(0.0);

        // Fill system-level K matrix
        ChSparsityPatternLearner spl(temp_descriptor.CountActiveVariables(), temp_descriptor.CountActiveVariables());
        temp_descriptor.PasteMassKRMMatrixInto(spl);
        spl.Apply(*K);
        K->setZeroValues();
        temp_descriptor.PasteMassKRMMatrixInto(*K);
        // K->makeCompressed();
    }

    if (R) {
        // Load all KRM matrices with the R part only
        this->LoadKRMMatrices(0, 1.0, 0);
        // For ChVariable objects without a ChKRMBlock, but still with a mass:
        temp_descriptor.SetMassFactor(0.0);

        // Fill system-level R matrix
        ChSparsityPatternLearner spl(temp_descriptor.CountActiveVariables(), temp_descriptor.CountActiveVariables());
        temp_descriptor.PasteMassKRMMatrixInto(spl);
        spl.Apply(*R);
        R->setZeroValues();
        temp_descriptor.PasteMassKRMMatrixInto(*R);
        // R->makeCompressed();
    }

    if (M) {
        // Load all KRM matrices with the M part only
        LoadKRMMatrices(0, 0, 1.0);
        // For ChVariable objects without a ChKRMBlock, but still with a mass:
        temp_descriptor.SetMassFactor(1.0);

        // Fill system-level M matrix
        ChSparsityPatternLearner spl(temp_descriptor.CountActiveVariables(), temp_descriptor.CountActiveVariables());
        temp_descriptor.PasteMassKRMMatrixInto(spl);
        spl.Apply(*M);
        M->setZeroValues();
        temp_descriptor.PasteMassKRMMatrixInto(*M);
        // M->makeCompressed();
    }

    if (Cq) {
        // Load all jacobian matrices
        this->LoadConstraintJacobians();

        // Fill system-level R matrix
        ChSparsityPatternLearner spl(temp_descriptor.CountActiveConstraints(), temp_descriptor.CountActiveVariables());
        temp_descriptor.PasteConstraintsJacobianMatrixInto(spl);
        spl.Apply(*Cq);
        Cq->setZeroValues();
        temp_descriptor.PasteConstraintsJacobianMatrixInto(*Cq);
        // Cq->makeCompressed();
    }
}

void ChModalAssembly::WriteSubassemblyMatrices(bool save_M,
                                               bool save_K,
                                               bool save_R,
                                               bool save_Cq,
                                               const std::string& path,
                                               bool one_indexed) {
    ChSparseMatrix K, R, M, Cq;
    GetSubassemblyMatrices(save_K ? &K : nullptr, save_R ? &R : nullptr, save_M ? &M : nullptr,
                           save_Cq ? &Cq : nullptr);

    if (save_M) {
        std::ofstream file_M(path + "_M.dat");
        file_M << std::setprecision(12) << std::scientific;
        StreamOut(M, file_M, one_indexed);
    }
    if (save_K) {
        std::ofstream file_K(path + "_K.dat");
        file_K << std::setprecision(12) << std::scientific;
        StreamOut(K, file_K, one_indexed);
    }
    if (save_R) {
        std::ofstream file_R(path + "_R.dat");
        file_R << std::setprecision(12) << std::scientific;
        StreamOut(R, file_R, one_indexed);
    }
    if (save_Cq) {
        std::ofstream file_Cq(path + "_Cq.dat");
        file_Cq << std::setprecision(12) << std::scientific;
        StreamOut(Cq, file_Cq, one_indexed);
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
    m_full_forces_internal.setZero(m_num_coords_vel_internal);

    // For the modal part:
    //

    // (nothing to count)

    // For the entire assembly:
    //

    if (!m_is_model_reduced) {
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
        m_num_coords_pos = m_num_coords_pos_boundary + m_num_coords_modal;
        m_num_coords_vel = m_num_coords_vel_boundary + m_num_coords_modal;
        m_num_constr = m_num_constr_boundary;
        m_num_constr_bil = m_num_constr_bil_boundary;
        m_num_constr_uni = m_num_constr_uni_boundary;
    }
}

void ChModalAssembly::Initialize() {
    if (this->is_initialized)
        return;

    // fetch the initial state of assembly, full not reduced, as an initialization
    // TODO: shall we temporary unset m_is_model_reduced so to retrieve the full state in any case?
    double fooT;
    m_full_state_x0.setZero(m_num_coords_pos, nullptr);
    ChStateDelta full_assembly_v;
    full_assembly_v.setZero(m_num_coords_vel, nullptr);
    this->IntStateGather(0, m_full_state_x0, 0, full_assembly_v, fooT);

    // also initialize m_full_state_x
    this->m_full_state_x = m_full_state_x0;

    // the floating frame F is initialized at COG in the initial configuration
    this->ComputeMassCenterFrame();

    this->floating_frame_F = this->cog_frame;

    // this->floating_frame_F_old = this->floating_frame_F;

    // store the initial floating frame of reference F0 in the initial configuration
    this->floating_frame_F0 = this->floating_frame_F;

    this->m_res_CF.setZero(6);

    this->is_initialized = true;
}

// Update all physical items (bodies, links, meshes, etc), including their auxiliary variables.
// Updates all forces (automatic, as children of bodies)
// Updates all markers (automatic, as children of bodies).
void ChModalAssembly::Update(bool update_assets) {
    ChAssembly::Update(update_assets);

    if (m_is_model_reduced) {
        // If in modal reduced state, the internal parts would not be updated (actually, these could even be
        // removed) However one still might want to see the internal nodes "moving" during animations.
        if (m_internal_nodes_update)
            this->UpdateInternalState(update_assets);

        // always update the floating frame F if possible, to improve the numerical accuracy and stability
        this->UpdateFloatingFrameOfReference();
        //// NOTE: do not switch these to range for loops (may want to use OMP for)

    } else {
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
    }
}

void ChModalAssembly::ForceToRest() {
    ChAssembly::ForceToRest();  // parent

    if (m_is_model_reduced) {
        this->modal_q_dt.setZero(m_num_coords_modal);
        this->modal_q_dtdt.setZero(m_num_coords_modal);
    } else {
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
    }
}

void ChModalAssembly::GetLocalDeformations(ChVectorDynamic<>& u_locred,
                                           ChVectorDynamic<>& e_locred,
                                           ChVectorDynamic<>& edt_locred) {
    if (!m_is_model_reduced)
        return;

    unsigned int num_coords_pos_bou_mod = m_num_coords_pos_boundary + m_num_coords_modal;
    unsigned int num_coords_vel_bou_mod = m_num_coords_vel_boundary + m_num_coords_modal;

    u_locred.setZero(num_coords_vel_bou_mod);    // =u_locred =P_W^T*[\delta qB; \delta eta]
    e_locred.setZero(num_coords_vel_bou_mod);    // =e_locred =[qB^bar; eta]
    edt_locred.setZero(num_coords_vel_bou_mod);  // =edt_locred =[qB^bar_dt; eta_dt]

    // fetch the state snapshot (modal reduced)
    double fooT;
    ChState x_mod;       // =[qB; eta]
    ChStateDelta v_mod;  // =[qB_dt; eta_dt]
    x_mod.setZero(num_coords_pos_bou_mod, nullptr);
    v_mod.setZero(num_coords_vel_bou_mod, nullptr);
    this->IntStateGather(0, x_mod, 0, v_mod, fooT);

    u_locred.tail(m_num_coords_modal) = x_mod.segment(m_num_coords_pos_boundary, m_num_coords_modal);
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
        ChQuaternion<> q_delta =
            quat_bou0.GetConjugate() * floating_frame_F0.GetRot() * floating_frame_F.GetRot().GetConjugate() * quat_bou;
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
        edt_locred.segment(6 * i_bou, 3) = (floating_frame_F.GetRot().RotateBack(v_B - floating_frame_F.GetPosDt()) +
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

void ChModalAssembly::IntStateGather(const unsigned int off_x,
                                     ChState& x,
                                     const unsigned int off_v,
                                     ChStateDelta& v,
                                     double& T) {
    ChAssembly::IntStateGather(off_x, x, off_v, v, T);  // parent

    unsigned int displ_x = off_x - this->offset_x;
    unsigned int displ_v = off_v - this->offset_w;

    if (!m_is_model_reduced) {
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
        x.segment(off_x + m_num_coords_pos_boundary, m_num_coords_modal) = this->modal_q;
        v.segment(off_v + m_num_coords_vel_boundary, m_num_coords_modal) = this->modal_q_dt;

        T = GetChTime();
    }
}

void ChModalAssembly::IntStateScatter(const unsigned int off_x,
                                      const ChState& x,
                                      const unsigned int off_v,
                                      const ChStateDelta& v,
                                      const double T,
                                      bool update_assets) {
    ChAssembly::IntStateScatter(off_x, x, off_v, v, T, update_assets);  // parent

    unsigned int displ_x = off_x - this->offset_x;
    unsigned int displ_v = off_v - this->offset_w;

    if (!m_is_model_reduced) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntStateScatter(displ_x + body->GetOffset_x(), x, displ_v + body->GetOffset_w(), v, T,
                                      update_assets);
            else
                body->Update(T, update_assets);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntStateScatter(displ_x + mesh->GetOffset_x(), x, displ_v + mesh->GetOffset_w(), v, T, update_assets);
        }
        for (auto& item : internal_otherphysicslist) {
            if (item->IsActive())
                item->IntStateScatter(displ_x + item->GetOffset_x(), x, displ_v + item->GetOffset_w(), v, T,
                                      update_assets);
            else
                item->Update(T, update_assets);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntStateScatter(displ_x + link->GetOffset_x(), x, displ_v + link->GetOffset_w(), v, T,
                                      update_assets);
            else
                link->Update(T, update_assets);
        }
    } else {
        this->modal_q = x.segment(off_x + m_num_coords_pos_boundary, m_num_coords_modal);
        this->modal_q_dt = v.segment(off_v + m_num_coords_vel_boundary, m_num_coords_modal);

        // Update:
        this->Update(update_assets);
    }

    SetChTime(T);
}

void ChModalAssembly::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    ChAssembly::IntStateGatherAcceleration(off_a, a);  // parent

    unsigned int displ_a = off_a - this->offset_w;

    if (!m_is_model_reduced) {
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
        a.segment(off_a + m_num_coords_vel_boundary, m_num_coords_modal) = this->modal_q_dtdt;
    }
}

// From state derivative (acceleration) to system, sometimes might be needed
void ChModalAssembly::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    ChAssembly::IntStateScatterAcceleration(off_a, a);  // parent

    unsigned int displ_a = off_a - this->offset_w;

    if (!m_is_model_reduced) {
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
        // Todo: shall we also update the acceleration of internal nodes if m_internal_nodes_update==true ?
        // The algorithm is similar as the recovery of the internal velocity.

        this->modal_q_dtdt = a.segment(off_a + m_num_coords_vel_boundary, m_num_coords_modal);
    }
}

// From system to reaction forces (last computed) - some timestepper might need this
void ChModalAssembly::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    ChAssembly::IntStateGatherReactions(off_L, L);  // parent

    unsigned int displ_L = off_L - this->offset_L;

    if (!m_is_model_reduced) {
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
        // there is no internal Lagrange multiplier in the reduced state
    }
}

// From reaction forces to system, ex. store last computed reactions in ChLinkBase objects for plotting etc.
void ChModalAssembly::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    ChAssembly::IntStateScatterReactions(off_L, L);  // parent

    if (!m_is_model_reduced) {
        unsigned int displ_L = off_L - this->offset_L;

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
    } else if (m_internal_nodes_update && m_num_constr_internal) {
        unsigned int num_coords_vel_bou_int = m_num_coords_vel_boundary + m_num_coords_vel_internal;
        unsigned int num_coords_vel_bou_mod = m_num_coords_vel_boundary + m_num_coords_modal;

        if (this->Psi.rows() != (num_coords_vel_bou_int + m_num_constr_internal) ||
            this->Psi.cols() != num_coords_vel_bou_mod)
            return;

        ChVectorDynamic<> u_locred(num_coords_vel_bou_mod);
        ChVectorDynamic<> e_locred(num_coords_vel_bou_mod);
        ChVectorDynamic<> edt_locred(num_coords_vel_bou_mod);
        this->GetLocalDeformations(u_locred, e_locred, edt_locred);

        // the new Lagrange multipliers of internal constraints
        ChVectorDynamic<> Lambda_internal(m_num_constr_internal);  // =[Lambda_I]
        Lambda_internal = Psi_S_LambdaI * e_locred.segment(0, m_num_coords_vel_boundary) +
                          Psi_D_LambdaI * e_locred.segment(m_num_coords_vel_boundary,
                                                           m_num_coords_modal - m_num_coords_static_correction);
        if (m_num_coords_static_correction)
            Lambda_internal += Psi_Cor_LambdaI * e_locred.tail(m_num_coords_static_correction);

        // flip the sign, and scale back
        Lambda_internal *= -m_scaling_factor_CqI;

        bool needs_temporary_bou_int = m_is_model_reduced;
        if (needs_temporary_bou_int)
            m_is_model_reduced = false;

        // scatter the Lagrange multipliers for the internal links and update them
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntStateScatterReactions(body->GetOffset_L() - this->offset_L - m_num_constr_boundary,
                                               Lambda_internal);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntStateScatterReactions(mesh->GetOffset_L() - this->offset_L - m_num_constr_boundary,
                                           Lambda_internal);
        }
        for (auto& item : internal_otherphysicslist) {
            if (item->IsActive())
                item->IntStateScatterReactions(item->GetOffset_L() - this->offset_L - m_num_constr_boundary,
                                               Lambda_internal);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntStateScatterReactions(link->GetOffset_L() - this->offset_L - m_num_constr_boundary,
                                               Lambda_internal);
        }

        if (needs_temporary_bou_int)
            m_is_model_reduced = true;
    }
}

void ChModalAssembly::IntStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) {
    ChAssembly::IntStateIncrement(off_x, x_new, x, off_v, Dv);  // parent

    if (!m_is_model_reduced) {
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
        x_new.segment(off_x + m_num_coords_pos_boundary, m_num_coords_modal) =
            x.segment(off_x + m_num_coords_pos_boundary, m_num_coords_modal) +
            Dv.segment(off_v + m_num_coords_vel_boundary, m_num_coords_modal);
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

    if (!m_is_model_reduced) {
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
        Dv.segment(off_v + m_num_coords_vel_boundary, m_num_coords_modal) =
            x_new.segment(off_x + m_num_coords_pos_boundary, m_num_coords_modal) -
            x.segment(off_x + m_num_coords_pos_boundary, m_num_coords_modal);
    }
}

void ChModalAssembly::IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
                                        ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
                                        const double c)          ///< a scaling factor
{
    ChAssembly::IntLoadResidual_F(off, R, c);  // parent

    unsigned int displ_v = off - this->offset_w;

    if (!m_is_model_reduced) {
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
        unsigned int num_coords_pos_bou_mod = m_num_coords_pos_boundary + m_num_coords_modal;
        unsigned int num_coords_vel_bou_mod = m_num_coords_vel_boundary + m_num_coords_modal;

        // 1-
        // Add elastic forces from current modal deformations
        ChVectorDynamic<> u_locred(num_coords_vel_bou_mod);
        ChVectorDynamic<> e_locred(num_coords_vel_bou_mod);
        ChVectorDynamic<> edt_locred(num_coords_vel_bou_mod);
        this->GetLocalDeformations(u_locred, e_locred, edt_locred);

        ChVectorDynamic<> f_mod_loc = P_perp_0.transpose() * (this->K_red * e_locred + this->R_red * edt_locred);
        ChVectorDynamic<> f_mod(num_coords_vel_bou_mod);
        f_mod.tail(m_num_coords_modal) = f_mod_loc.tail(m_num_coords_modal);
        for (unsigned int i_node = 0; i_node < m_num_coords_vel_boundary / 6; ++i_node) {
            f_mod.segment(6 * i_node, 3) = floating_frame_F.GetRot().Rotate(f_mod_loc.segment(6 * i_node, 3)).eigen();
            f_mod.segment(6 * i_node + 3, 3) = f_mod_loc.segment(6 * i_node + 3, 3);
        }
        // note: - sign
        R.segment(off, m_num_coords_vel_boundary + m_num_coords_modal) -= c * f_mod;

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
                O_F.block<3, 3>(6 * i_bou, 6 * i_bou) = ChStarMatrix33<>(floating_frame_F.GetAngVelLocal());

            ChMatrixDynamic<> mat_OF = GetCorotationalTransformation(O_F * M_red);

            /// g_quadvel: the quadratic velocity term of the reduced modal superelement
            ChVectorDynamic<> g_quadvel(num_coords_vel_bou_mod);
            g_quadvel = mat_OF * v_mod;

            if (!m_use_linear_inertial_term) {
                ChMatrixDynamic<> V;
                V.setZero(num_coords_vel_bou_mod, 6);
                for (unsigned int i_bou = 0; i_bou < (m_num_coords_vel_boundary / 6.); i_bou++) {
                    V.block<3, 3>(6 * i_bou, 3) =
                        ChStarMatrix33<>(floating_frame_F.GetRot().RotateBack(v_mod.segment(6 * i_bou, 3)));
                }
                ChMatrixDynamic<> mat_M = GetCorotationalTransformation(M_red * V * P_F * Q_0);
                g_quadvel += (mat_M - mat_M.transpose()) * v_mod;

                //// leading to divergence. DO NOT use it.
                // ChMatrixDynamic<> O_B;
                // O_B.setZero(num_coords_vel_bou_mod, num_coords_vel_bou_mod);
                // for (unsigned int i_bou = 0; i_bou < (m_num_coords_vel_boundary / 6.); i_bou++) {
                //     O_B.block(6 * i_bou + 3, 6 * i_bou + 3, 3, 3) = ChStarMatrix33<>(v_mod.segment(6 * i_bou + 3,
                //     3));
                // }
                // ChMatrixDynamic<> mat_OB = GetCorotationalTransformation(O_B * M_red);
                // g_quadvel += mat_OB * v_mod;
            }

            // note: - sign
            R.segment(off, m_num_coords_vel_boundary + m_num_coords_modal) -= c * g_quadvel;
        }

        // 3-
        // Update the external forces imposed on the internal nodes.
        // Note: the below code requires that the internal bodies and internal nodes are inserted in sequence.
        {
            unsigned int offset_loc = 0;
            for (unsigned int ip = 0; ip < internal_bodylist.size(); ++ip) {
                m_full_forces_internal.segment(offset_loc, 3) = internal_bodylist[ip]->GetAccumulatedForce().eigen();
                m_full_forces_internal.segment(offset_loc + 3, 3) =
                    internal_bodylist[ip]->GetAccumulatedTorque().eigen();
                offset_loc += internal_bodylist[ip]->GetNumCoordsVelLevel();
            }
            for (unsigned int ip = 0; ip < internal_meshlist.size(); ++ip) {
                for (auto& node : internal_meshlist[ip]->GetNodes()) {
                    if (auto xyz = std::dynamic_pointer_cast<ChNodeFEAxyz>(node)) {
                        m_full_forces_internal.segment(offset_loc, 3) = xyz->GetForce().eigen();
                        offset_loc += xyz->GetNumCoordsVelLevel();
                    }
                    if (auto xyzrot = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(node)) {
                        m_full_forces_internal.segment(offset_loc, 3) = xyzrot->GetForce().eigen();
                        m_full_forces_internal.segment(offset_loc + 3, 3) = xyzrot->GetTorque().eigen();
                        offset_loc += xyzrot->GetNumCoordsVelLevel();
                    }
                }
            }
        }

        // 4-
        // Update the gravitational force on the internal bodies and nodes
        if (m_modal_automatic_gravity && GetSystem()->GetGravitationalAcceleration().Length2()) {
            ChVectorDynamic<> g_acc_loc;
            g_acc_loc.setZero(m_num_coords_vel_boundary + m_num_coords_vel_internal);

            unsigned int offset_loc = 0;
            ChVector3d gloc = floating_frame_F.GetRot().RotateBack(GetSystem()->GetGravitationalAcceleration());
            // boundary bodies
            for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
                g_acc_loc.segment(offset_loc, 3) = gloc.eigen();
                offset_loc += bodylist[ip]->GetNumCoordsVelLevel();
            }
            // boundary nodes
            for (unsigned int ip = 0; ip < meshlist.size(); ++ip) {
                for (auto& node : meshlist[ip]->GetNodes()) {
                    if (auto xyz = std::dynamic_pointer_cast<ChNodeFEAxyz>(node)) {
                        g_acc_loc.segment(offset_loc, xyz->GetNumCoordsVelLevel()) = gloc.eigen();
                        offset_loc += xyz->GetNumCoordsVelLevel();
                    }
                    if (auto xyzrot = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(node)) {
                        g_acc_loc.segment(offset_loc, 3) = gloc.eigen();
                        offset_loc += xyzrot->GetNumCoordsVelLevel();
                    }
                }
            }
            // internal bodies
            for (unsigned int ip = 0; ip < internal_bodylist.size(); ++ip) {
                g_acc_loc.segment(offset_loc, 3) = gloc.eigen();
                offset_loc += internal_bodylist[ip]->GetNumCoordsVelLevel();
            }
            // internal nodes
            for (unsigned int ip = 0; ip < internal_meshlist.size(); ++ip) {
                for (auto& node : internal_meshlist[ip]->GetNodes()) {
                    if (auto xyz = std::dynamic_pointer_cast<ChNodeFEAxyz>(node)) {
                        g_acc_loc.segment(offset_loc, xyz->GetNumCoordsVelLevel()) = gloc.eigen();
                        offset_loc += xyz->GetNumCoordsVelLevel();
                    }
                    if (auto xyzrot = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(node)) {
                        g_acc_loc.segment(offset_loc, 3) = gloc.eigen();
                        offset_loc += xyzrot->GetNumCoordsVelLevel();
                    }
                }
            }

            // gravitational forces for all boundary and internal items
            ChVectorDynamic<> f_gravity;
            f_gravity.setZero(m_num_coords_vel_boundary + m_num_coords_vel_internal);
            ChVectorDynamic<> f_gravity_loc = full_M_loc * g_acc_loc;
            for (unsigned int i_node = 0; i_node < f_gravity.size() / 6; ++i_node)
                f_gravity.segment(6 * i_node, 3) =
                    floating_frame_F.GetRot().Rotate(f_gravity_loc.segment(6 * i_node, 3)).eigen();

            // only add the gravitational forces for internal part since it has been inherited from the parent methods
            // for the boundary part
            m_full_forces_internal.tail(m_num_coords_vel_internal) += f_gravity.tail(m_num_coords_vel_internal);

            // add the gravity load for boundary bodies and nodes.
            R.segment(off, m_num_coords_vel_boundary) += c * f_gravity.head(m_num_coords_vel_boundary);

            // remove the duplicated gravity load of boundary bodies (ChBoby) since it has been included in the parent
            // method ChAssembly::IntLoadResidual_F().
            for (auto& body : bodylist) {
                if (body->IsActive())
                    R.segment(displ_v + body->GetOffset_w(), 3) -=
                        c * body->GetMass() * GetSystem()->GetGravitationalAcceleration().eigen();
            }
        }

        // 5-
        // Add the collected forces of internal items (bodies, nodes) on the generalized coordinates
        if (!m_full_forces_internal.isZero()) {
            ChVectorDynamic<> f_loc(m_num_coords_vel_internal);
            for (unsigned int i_node = 0; i_node < m_num_coords_vel_internal / 6; ++i_node) {
                f_loc.segment(6 * i_node, 3) =
                    floating_frame_F.GetRot().RotateBack(m_full_forces_internal.segment(6 * i_node, 3)).eigen();
                f_loc.segment(6 * i_node + 3, 3) = m_full_forces_internal.segment(6 * i_node + 3, 3);
            }

            ChVectorDynamic<> f_reduced(m_num_coords_vel_boundary + m_num_coords_modal);
            f_reduced.setZero();

            // the static contribution of the external forces imposed on the internal nodes
            ChVectorDynamic<> f_loc_static = Psi_S.transpose() * f_loc;
            for (unsigned int i_node = 0; i_node < m_num_coords_vel_boundary / 6; ++i_node) {
                f_reduced.segment(6 * i_node, 3) =
                    floating_frame_F.GetRot().Rotate(f_loc_static.segment(6 * i_node, 3)).eigen();
                f_reduced.segment(6 * i_node + 3, 3) = f_loc_static.segment(6 * i_node + 3, 3);
            }

            // the dynamic contribution of the external forces imposed on the internal nodes
            f_reduced.segment(m_num_coords_vel_boundary, m_num_coords_modal - m_num_coords_static_correction) =
                Psi_D.transpose() * f_loc;

            // the static correction part
            if (m_num_coords_static_correction) {
                // update the static correction mode
                this->UpdateStaticCorrectionMode();

                f_reduced.tail(m_num_coords_static_correction) = Psi_Cor.transpose() * f_loc;
            }

            R.segment(off, m_num_coords_vel_boundary + m_num_coords_modal) += c * f_reduced;
        }

        // todo: add gyroscopic torques for internal bodies and internal meshes
    }
}

void ChModalAssembly::IntLoadResidual_Mv(const unsigned int off,      ///< offset in R residual
                                         ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                         const ChVectorDynamic<>& w,  ///< the w vector
                                         const double c               ///< a scaling factor
) {
    if (!m_is_model_reduced) {
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
        ChVectorDynamic<> w_modal = w.segment(off, m_num_coords_vel_boundary + m_num_coords_modal);
        R.segment(off, m_num_coords_vel_boundary + m_num_coords_modal) += c * (this->modal_M * w_modal);
    }
}

void ChModalAssembly::IntLoadLumpedMass_Md(const unsigned int off, ChVectorDynamic<>& Md, double& err, const double c) {
    unsigned int displ_v = off - this->offset_w;

    if (!m_is_model_reduced) {
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
        Md.segment(off, m_num_coords_vel_boundary + m_num_coords_modal) += c * this->modal_M.diagonal();

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

    if (!m_is_model_reduced) {
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
        // there is no internal Lagrange multiplier in the reduced state
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

    if (!m_is_model_reduced) {
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

    if (!m_is_model_reduced) {
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

    if (!m_is_model_reduced) {
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
        this->modal_variables->State() = v.segment(off_v + m_num_coords_vel_boundary, m_num_coords_modal);
        this->modal_variables->Force() = R.segment(off_v + m_num_coords_vel_boundary, m_num_coords_modal);
    }
}

void ChModalAssembly::IntFromDescriptor(const unsigned int off_v,
                                        ChStateDelta& v,
                                        const unsigned int off_L,
                                        ChVectorDynamic<>& L) {
    ChAssembly::IntFromDescriptor(off_v, v, off_L, L);  // parent

    unsigned int displ_L = off_L - this->offset_L;
    unsigned int displ_v = off_v - this->offset_w;

    if (!m_is_model_reduced) {
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
        v.segment(off_v + m_num_coords_vel_boundary, m_num_coords_modal) = this->modal_variables->State();
    }
}

// -----------------------------------------------------------------------------

void ChModalAssembly::InjectVariables(ChSystemDescriptor& descriptor) {
    ChAssembly::InjectVariables(descriptor);

    if (m_is_model_reduced) {
        descriptor.InsertVariables(this->modal_variables);
    } else {
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
    }
}

void ChModalAssembly::InjectConstraints(ChSystemDescriptor& descriptor) {
    ChAssembly::InjectConstraints(descriptor);  // parent

    if (!m_is_model_reduced) {
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
        // there is no internal Lagrange multiplier in the reduced state
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
        // there is no internal Lagrange multiplier in the reduced state
    }
}

void ChModalAssembly::InjectKRMMatrices(ChSystemDescriptor& descriptor) {
    if (m_is_model_reduced) {
        descriptor.InsertKRMBlock(&this->modal_Hblock);
    } else {
        ChAssembly::InjectKRMMatrices(descriptor);

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
    }
}

void ChModalAssembly::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
    if (m_is_model_reduced) {
        ComputeModalKRMmatricesGlobal(Kfactor, Rfactor, Mfactor);
        this->modal_Hblock.GetMatrix() = this->modal_K * Kfactor + this->modal_R * Rfactor + this->modal_M * Mfactor;
    } else {
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
    archive_out << CHNVP(m_modal_reduction_type, "m_modal_reduction_type");
    archive_out << CHNVP(m_is_model_reduced, "m_is_model_reduced");
    archive_out << CHNVP(m_internal_nodes_update, "m_internal_nodes_update");
    archive_out << CHNVP(m_modal_automatic_gravity, "m_modal_automatic_gravity");
    archive_out << CHNVP(modal_q, "modal_q");
    archive_out << CHNVP(modal_q_dt, "modal_q_dt");
    archive_out << CHNVP(modal_q_dtdt, "modal_q_dtdt");
    archive_out << CHNVP(m_full_forces_internal, "m_full_forces_internal");
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

    archive_in >> CHNVP(m_modal_reduction_type, "m_modal_reduction_type");
    archive_in >> CHNVP(m_is_model_reduced, "m_is_model_reduced");
    archive_in >> CHNVP(m_internal_nodes_update, "m_internal_nodes_update");
    archive_in >> CHNVP(m_modal_automatic_gravity, "m_modal_automatic_gravity");
    archive_in >> CHNVP(modal_q, "modal_q");
    archive_in >> CHNVP(modal_q_dt, "modal_q_dt");
    archive_in >> CHNVP(modal_q_dtdt, "modal_q_dtdt");
    archive_in >> CHNVP(m_full_forces_internal, "m_full_forces_internal");

    // Recompute statistics, offsets, etc.
    Setup();
}

}  // end namespace modal

}  // end namespace chrono