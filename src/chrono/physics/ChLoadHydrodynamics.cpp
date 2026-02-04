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

ChLoadHydrodynamics::ChLoadHydrodynamics(const ChBodyAddedMassBlocks& body_blocks)
    : m_body_blocks(body_blocks), m_verbose(false) {
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

    // Set variables for KRM block
    m_KRM.SetVariables(variables);

    // Indicate that the KRM block only includes the mass component
    m_KRM.SetNoKRComponents();
}

ChLoadHydrodynamics::ChLoadHydrodynamics(const ChLoadHydrodynamics& other) {
    m_body_blocks = other.m_body_blocks;
    m_KRM = other.m_KRM;
}

ChLoadHydrodynamics::~ChLoadHydrodynamics() {}

void ChLoadHydrodynamics::Update(double time, UpdateFlags update_flags) {
    // If the system problem size has changed, recompute the system-wide added mass matrix
    auto size = GetSystem()->GetNumCoordsVelLevel();
    if (m_added_mass.rows() != size) {
        if (m_verbose)
            std::cout << "Resize added_mass matrix at t = " << time << std::endl;

        m_KRM.GetMatrix().resize(size, size);
        m_added_mass.resize(size, size);
        m_added_mass.setZero();

        // Check if inverse of mass matrix is required (when a Schur complement-based solver is used)
        auto solver_type = system->GetSolverType();
        bool calc_M_inv = (solver_type == ChSolver::Type::APGD ||             //
                           solver_type == ChSolver::Type::BARZILAIBORWEIN ||  //
                           solver_type == ChSolver::Type::PSOR);              //
           
        // Load mass matrix with body inertia (sparse, block-diagonal)
        ChSparseMatrix M_sparse;
        if (calc_M_inv) {
            system->DescriptorPrepareInject();
            system->GetMassMatrix(M_sparse);
#ifdef DEBUG_PRINT
            std::cout << "Mass matrix" << std::endl;
            std::cout << total_mass << std::endl;
#endif
        }

        m_added_mass.setZero();
        for (const auto& b1 : m_body_blocks) {
            auto row = b1.body->GetOffset_w();
            if (m_verbose)
                std::cout << "- process blocks for body '" << b1.body->GetName() << "'" << std::endl;

            int i = 0;
            for (const auto& b2 : m_body_blocks) {
                auto col = b2.body->GetOffset_w();
                if (m_verbose)
                    std::cout << "  add 6x6 block starting at (" << row << "," << col << ")" << std::endl;

                // Current block (at row x col)
                auto block = b1.block(Eigen::seq(0, 5), Eigen::seq(i, i + 5));

                // Set block in added mass matrix
                m_added_mass.block(row, col, 6, 6) = block;

                if (calc_M_inv) {
                    // Add block to sparse total mass matrix
                    PasteMatrix(M_sparse, block, row, col, false);
                }
                i += 6;
            }
        }

        // Calculate inverse of total mass
        if (calc_M_inv) {
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

void ChLoadHydrodynamics::IntLoadResidual_Mv(const unsigned int off,
                                             ChVectorDynamic<>& R,
                                             const ChVectorDynamic<>& w,
                                             const double c) {
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
    ChVectorDynamic<> cMw1 = c * m_added_mass * w1;
    i = 0;
    for (const auto& b : m_body_blocks) {
        auto offset = b.body->GetOffset_w();
        R(Eigen::seq(offset, offset + 5)) += cMw1(Eigen::seq(i, i + 5));
        i += 6;
    }
}

void ChLoadHydrodynamics::InjectKRMMatrices(ChSystemDescriptor& descriptor) {
    descriptor.InsertKRMBlock(&m_KRM);
}

void ChLoadHydrodynamics::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
    m_KRM.GetMatrix() = Mfactor * m_added_mass;
}

}  // end namespace chrono
