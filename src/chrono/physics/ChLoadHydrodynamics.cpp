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

////#define DEBUG_PRINT

namespace chrono {

ChLoadHydrodynamics::ChLoadHydrodynamics(const ChBodyAddedMassBlocks& body_blocks) : m_body_blocks(body_blocks), m_verbose(false), m_system_size(0) {
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

    // Indicate that the KRM block only includes the mass component
    m_KRM.SetNoKRComponents();
}

ChLoadHydrodynamics::ChLoadHydrodynamics(const ChLoadHydrodynamics& other) {
    m_body_blocks = other.m_body_blocks;
    m_KRM = other.m_KRM;
    m_verbose = other.m_verbose;
    m_system_size = other.m_system_size;
}

ChLoadHydrodynamics::~ChLoadHydrodynamics() {}

void ChLoadHydrodynamics::SetBodyAddedMassBlocks(const std::vector<ChMatrixDynamic<>>& blocks) {
    auto num_bodies = m_body_blocks.size();
    ChAssertAlways(blocks.size() == num_bodies);
    for (size_t i = 0; i < num_bodies; i++) {
        ChAssertAlways(m_body_blocks[i].block.rows() == blocks[i].rows() && m_body_blocks[i].block.cols() == blocks[i].cols());
        m_body_blocks[i].block = blocks[i];
    }
}

void ChLoadHydrodynamics::UpdateBodyAddedMassBlocks(const std::vector<ChMatrix66d>& blocks) {
    auto num_bodies = m_body_blocks.size();
    ChAssertAlways(blocks.size() == num_bodies);
    for (size_t i = 0; i < num_bodies; i++) {
        m_body_blocks[i].block.block(0, 6 * i, 6, 6) = blocks[i];
    }
}

void ChLoadHydrodynamics::Update(double time, UpdateFlags update_flags) {
    // Check whether the solver needs the inverse of the total mass matrix (Schur complement-based
    // solvers). That is the only system-wide work here: the KRM block this load injects spans the
    // hydrodynamic bodies alone and is filled in LoadKRMMatrices.
    auto solver_type = system->GetSolverType();
    bool calc_M_inv = (solver_type == ChSolver::Type::APGD ||             //
                       solver_type == ChSolver::Type::BARZILAIBORWEIN ||  //
                       solver_type == ChSolver::Type::PSOR);              //

    // Recompute whenever the system problem size changes.
    auto size = GetSystem()->GetNumCoordsVelLevel();
    if (calc_M_inv && m_system_size != size) {
        m_system_size = size;

        if (m_verbose)
            std::cout << "Assemble total mass matrix at t = " << time << std::endl;

        // Load mass matrix with body inertia (sparse, block-diagonal)
        ChSparseMatrix M_sparse;
        system->DescriptorPrepareInject();
        system->GetMassMatrix(M_sparse);
#ifdef DEBUG_PRINT
        std::cout << "Mass matrix" << std::endl;
        std::cout << total_mass << std::endl;
#endif

        // Add the added mass blocks. These are indexed by system offsets, since M_sparse is system-wide.
        for (const auto& b1 : m_body_blocks) {
            auto row = b1.body->GetOffset_w();
            if (m_verbose)
                std::cout << "- process blocks for body '" << b1.body->GetName() << "'" << std::endl;

            int i = 0;
            for (const auto& b2 : m_body_blocks) {
                auto col = b2.body->GetOffset_w();
                if (m_verbose)
                    std::cout << "  add 6x6 block starting at (" << row << "," << col << ")" << std::endl;

                PasteMatrix(M_sparse, b1.block(Eigen::seq(0, 5), Eigen::seq(i, i + 5)), row, col, false);
                i += 6;
            }
        }

        // Calculate inverse of total mass
        {
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
    }

    // Overloading of base class:
    ChPhysicsItem::Update(time, update_flags);
}

void ChLoadHydrodynamics::IntLoadResidual_Mv(const unsigned int off, ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) {
    auto num_bodies = m_body_blocks.size();

    // Compress w to entries corresponding to the hydrodynamic bodies
    ChVectorDynamic<> w1(6 * num_bodies);
    int i = 0;
    for (const auto& b : m_body_blocks) {
        auto offset = b.body->GetOffset_w();
        w1(Eigen::seq(i, i + 5)) = w(Eigen::seq(offset, offset + 5));
        i += 6;
    }

    // Update R += c * M * w
    for (const auto& b : m_body_blocks) {
        auto offset = b.body->GetOffset_w();
        R(Eigen::seq(offset, offset + 5)) += c * b.block * w1;
    }
}

void ChLoadHydrodynamics::InjectKRMMatrices(ChSystemDescriptor& descriptor) {
    descriptor.InsertKRMBlock(&m_KRM);
}

void ChLoadHydrodynamics::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
    // The KRM block is indexed in the order in which its variables were declared, i.e. the order of the
    // body blocks, so it is simply the stack of the per-body 6 x 6*num_bodies blocks. Indexing it by
    // system offsets instead is correct only when the hydrodynamic bodies happen to occupy the leading
    // offsets of the system, in that same order.
    auto num_bodies = m_body_blocks.size();
    for (size_t i = 0; i < num_bodies; i++)
        m_KRM.GetMatrix().block(6 * i, 0, 6, 6 * num_bodies) = Mfactor * m_body_blocks[i].block;
}

}  // end namespace chrono
