// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include "chrono/physics/ChLoadHydrodynamics.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtils.h"

////#define DEBUG_PRINT

namespace chrono {

ChLoadHydrodynamics::ChLoadHydrodynamics(const ChBodyAddedMassBlocks& body_blocks)
    : m_verbose(false), m_frame(AddedMassFrame::INERTIAL), m_body_blocks(body_blocks), m_system_size(0), m_blocks_modified(false) {
    // Traverse list of hydro bodies, check added mass block size, and collect list of variables
    std::vector<ChVariables*> variables;
    auto num_bodies = (int)body_blocks.size();
    for (const auto& b : body_blocks) {
        if (b.block.rows() != 6 || b.block.cols() != 6 * num_bodies) {
            std::cerr << "Incorrect added mass block size for body " << b.body->GetName() << std::endl;
            throw std::runtime_error("Incorrect added mass block size");
        }
        variables.push_back(&b.body->Variables());
    }

    // Set variables for the KRM block. The block spans only the hydrodynamic bodies, so its matrix is
    // 6*num_bodies square and is indexed in the order in which those variables are declared here, which
    // is the order of the body blocks. It is *not* indexed by system offsets.
    m_KRM.SetVariables(variables);
    m_KRM.GetMatrix().setZero(6 * num_bodies, 6 * num_bodies);
    m_M.setZero(6 * num_bodies, 6 * num_bodies);

    // Indicate that the KRM block only includes the mass component
    m_KRM.SetNoKRComponents();
}

ChLoadHydrodynamics::ChLoadHydrodynamics(const ChLoadHydrodynamics& other) {
    m_body_blocks = other.m_body_blocks;
    m_M = other.m_M;
    m_KRM = other.m_KRM;
    m_verbose = other.m_verbose;
    m_frame = other.m_frame;
    m_system_size = other.m_system_size;
    m_blocks_modified = other.m_blocks_modified;
    m_inverse_rotations = other.m_inverse_rotations;
}

ChLoadHydrodynamics::~ChLoadHydrodynamics() {}

void ChLoadHydrodynamics::SetAddedMassFrame(AddedMassFrame frame) {
    if (frame != m_frame) {
        m_frame = frame;
        m_blocks_modified = true;
    }
    OnBlocksModified();
}

void ChLoadHydrodynamics::SetBodyAddedMassBlocks(const std::vector<ChMatrixDynamic<>>& blocks) {
    auto num_bodies = m_body_blocks.size();
    ChAssertAlways(blocks.size() == num_bodies);
    for (size_t i = 0; i < num_bodies; i++) {
        ChAssertAlways(m_body_blocks[i].block.rows() == blocks[i].rows() && m_body_blocks[i].block.cols() == blocks[i].cols());
        if (m_body_blocks[i].block != blocks[i]) {
            m_body_blocks[i].block = blocks[i];
            m_blocks_modified = true;
        }
    }
    OnBlocksModified();
}

void ChLoadHydrodynamics::UpdateBodyAddedMassBlocks(const std::vector<ChMatrix66d>& blocks) {
    auto num_bodies = m_body_blocks.size();
    ChAssertAlways(blocks.size() == num_bodies);
    for (size_t i = 0; i < num_bodies; i++) {
        auto self_block = m_body_blocks[i].block.block(0, 6 * i, 6, 6);
        if (self_block != blocks[i]) {
            self_block = blocks[i];
            m_blocks_modified = true;
        }
    }
    OnBlocksModified();
}

bool ChLoadHydrodynamics::NeedsMassInverse() const {
    // Schur complement-based solvers need the inverse of the total mass matrix. That is the only
    // system-wide work done by this load: the KRM block it injects spans the hydrodynamic bodies alone
    // and is filled in LoadKRMMatrices.
    // PSOR is not included: it does not support systems with KRM blocks and throws for such problems.
    auto solver_type = system->GetSolverType();
    return (solver_type == ChSolver::Type::APGD ||  //
            solver_type == ChSolver::Type::BARZILAIBORWEIN);
}

void ChLoadHydrodynamics::OnBlocksModified() {
    // The system updates its physics items only after the solve of each step, so waiting for the next
    // call to Update would let the next step be solved with a stale mass inverse. If a mass inverse was
    // already computed (and therefore the system is set up and the solver needs it), recompute it now.
    // Otherwise, it is computed during the first call to Update.
    if (m_blocks_modified && system && m_system_size > 0 && NeedsMassInverse())
        ComputeMassInverse();
}

void ChLoadHydrodynamics::ComputeMassInverse() {
    m_system_size = system->GetNumCoordsVelLevel();
    m_blocks_modified = false;
    m_inverse_rotations.clear();
    for (const auto& b : m_body_blocks)
        m_inverse_rotations.push_back(b.body->GetRot());

    if (m_verbose)
        std::cout << "Assemble total mass matrix at t = " << system->GetChTime() << std::endl;

    // Load the total mass matrix. GetMassMatrix includes the body inertia as well as the M part of all
    // KRM blocks, including the one injected by this load, so the added mass blocks are already
    // accounted for and must not be added again here.
    ChSparseMatrix M_sparse;
    system->DescriptorPrepareInject();
    system->GetMassMatrix(M_sparse);

    // Calculate inverse of total mass
    if (m_verbose)
        std::cout << "  compute inverse of total mass matrix" << std::endl;

    auto M = M_sparse.toDense();
#ifdef DEBUG_PRINT
    std::cout << "Mass matrix + Added mass" << std::endl;
    std::cout << M << std::endl;
#endif

    auto Minv = M.inverse();
#ifdef DEBUG_PRINT
    std::cout << "Inverse" << std::endl;
    std::cout << Minv << std::endl;
    std::cout << "Check" << std::endl;
    std::cout << M * Minv << std::endl;
#endif

    system->GetSystemDescriptor()->SetMassInverse(Minv);
}

bool ChLoadHydrodynamics::BodiesRotated() const {
    if (m_inverse_rotations.size() != m_body_blocks.size())
        return true;
    for (size_t i = 0; i < m_body_blocks.size(); i++) {
        if (m_body_blocks[i].body->GetRot() != m_inverse_rotations[i])
            return true;
    }
    return false;
}

void ChLoadHydrodynamics::Update(double time, UpdateFlags update_flags) {
    // Recompute the mass inverse (if needed) whenever the system problem size changes, the added mass
    // blocks were modified, or a hydrodynamic body rotated since it was last computed (the added mass
    // matrix in Chrono velocity coordinates depends on the body orientations). The blocks are only flagged
    // as modified when their values actually change, so setting the same coefficients again (e.g.,
    // constant added mass resent at every coupling step) does not trigger a recomputation.
    // Since Update is called at the end of a step, the mass inverse corresponds to the configuration at
    // the beginning of the next step, which is the configuration at which the solver uses it.
    if (NeedsMassInverse() && (m_system_size != system->GetNumCoordsVelLevel() || m_blocks_modified || BodiesRotated()))
        ComputeMassInverse();

    // Overloading of base class:
    ChPhysicsItem::Update(time, update_flags);
}

ChMatrix66d ChLoadHydrodynamics::ComputeTransformation(const ChBodyAddedMassBlock& b) const {
    // With v the COM velocity (absolute axes), w the angular velocity (COM frame axes), R_com and R_ref the orientations
    // of the COM and reference frames, and d = p_com - p_ref (absolute axes), the velocity of the reference frame origin
    // is
    //    v_ref = v + (R_com w) x (-d) = v + [d] R_com w
    // and the angular velocity in absolute axes is R_com w. Therefore:
    // - INERTIAL: [v_ref; R_com w]               = [I, [d] R_com; 0, R_com] [v; w]
    // - BODY:     [R_ref^T v_ref; R_ref^T R_com w] = diag(R_ref^T, R_ref^T) * T_INERTIAL [v; w]
    // For a ChBody, the reference frame is the COM frame, so d = 0 and R_ref = R_com.
    const auto& frame_ref = b.body->GetFrameRefToAbs();
    const auto& R_com = b.body->GetRotMat();
    ChStarMatrix33<> d_tilde(b.body->GetPos() - frame_ref.GetPos());

    ChMatrix66d T = ChMatrix66d::Zero();
    T.block<3, 3>(0, 0).setIdentity();
    T.block<3, 3>(0, 3) = d_tilde * R_com;
    T.block<3, 3>(3, 3) = R_com;

    if (m_frame == AddedMassFrame::BODY) {
        const auto& R_ref = frame_ref.GetRotMat();
        T.topRows<3>() = R_ref.transpose() * T.topRows<3>();
        T.bottomRows<3>() = R_ref.transpose() * T.bottomRows<3>();
    }

    return T;
}

ChMatrix66d ChLoadHydrodynamics::ComputeShiftTransformation(const ChBody& body, const ChVector3d& point, AddedMassFrame frame) {
    // With v_ref and w the velocity of the reference frame origin and the angular velocity, both expressed in the axes of
    // the frame convention, and d the location of the point relative to the reference frame origin in the same axes, the
    // velocity of the point is
    //    v_P = v_ref + w x d = v_ref - [d] w
    // so that [v_P; w] = [I, -[d]; 0, I] [v_ref; w].
    ChVector3d d = point;
    if (frame == AddedMassFrame::INERTIAL)
        d = body.GetFrameRefToAbs().GetRotMat() * point;

    ChMatrix66d S = ChMatrix66d::Identity();
    S.block<3, 3>(0, 3) = -ChStarMatrix33<>(d);

    return S;
}

void ChLoadHydrodynamics::ShiftBlocks(std::vector<ChMatrixDynamic<>>& blocks, const std::vector<ChMatrix66d>& shifts) {
    auto num_bodies = (int)blocks.size();
    ChAssertAlways((int)shifts.size() == num_bodies);
    for (int i = 0; i < num_bodies; i++) {
        ChAssertAlways(blocks[i].rows() == 6 && blocks[i].cols() == 6 * num_bodies);
        for (int j = 0; j < num_bodies; j++) {
            ChMatrix66d A_ij = shifts[i].transpose() * blocks[i].block<6, 6>(0, 6 * j) * shifts[j];
            blocks[i].block<6, 6>(0, 6 * j) = A_ij;
        }
    }
}

void ChLoadHydrodynamics::ComputeAddedMassMatrix() {
    auto num_bodies = m_body_blocks.size();

    std::vector<ChMatrix66d> T(num_bodies);
    for (size_t i = 0; i < num_bodies; i++)
        T[i] = ComputeTransformation(m_body_blocks[i]);

    for (size_t i = 0; i < num_bodies; i++) {
        for (size_t j = 0; j < num_bodies; j++) {
            m_M.block<6, 6>(6 * i, 6 * j) = T[i].transpose() * m_body_blocks[i].block.block<6, 6>(0, 6 * j) * T[j];
        }
    }
}

void ChLoadHydrodynamics::IntLoadResidual_Mv(const unsigned int off, ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) {
    auto num_bodies = m_body_blocks.size();

    // Added mass matrix at the current body configuration
    ComputeAddedMassMatrix();

    // Compress w to entries corresponding to the hydrodynamic bodies
    ChVectorDynamic<> w1(6 * num_bodies);
    int i = 0;
    for (const auto& b : m_body_blocks) {
        auto offset = b.body->GetOffset_w();
        w1(Eigen::seq(i, i + 5)) = w(Eigen::seq(offset, offset + 5));
        i += 6;
    }

    // Update R += c * M * w
    for (size_t k = 0; k < num_bodies; k++) {
        auto offset = m_body_blocks[k].body->GetOffset_w();
        R(Eigen::seq(offset, offset + 5)) += c * m_M.middleRows(6 * k, 6) * w1;
    }
}

void ChLoadHydrodynamics::InjectKRMMatrices(ChSystemDescriptor& descriptor) {
    descriptor.InsertKRMBlock(&m_KRM);
}

void ChLoadHydrodynamics::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
    // The KRM block is indexed in the order in which its variables were declared, i.e. the order of the
    // body blocks, so it is simply the added mass matrix at the current body configuration. Indexing it by
    // system offsets instead is correct only when the hydrodynamic bodies happen to occupy the leading
    // offsets of the system, in that same order.
    ComputeAddedMassMatrix();
    m_KRM.GetMatrix() = Mfactor * m_M;
}

}  // end namespace chrono
