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

namespace chrono {

ChLoadHydrodynamics::ChLoadHydrodynamics(const ChBodyAddedMassBlocks& body_blocks)
    : m_body_blocks(body_blocks), m_verbose(false) {
    // Traverse list of hydro bodies, check added mass block size, and collect list of variables
    std::vector<ChVariables*> variables;
    auto num_bodies = (int)body_blocks.size();
    for (const auto& b : body_blocks) {
        if (b.second.rows() != 6 || b.second.cols() != 6 * num_bodies) {
            std::cerr << "Incorrect added mass block size for body " << b.first->GetName() << std::endl;
            throw std::runtime_error("Incorrect added mass block size");
        }
        variables.push_back(&b.first->Variables());
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

void ChLoadHydrodynamics::Update(double time, bool update_assets) {
    // If the system problem size has changed, recompute the system-wide added mass matrix
    auto size = GetSystem()->GetNumCoordsVelLevel();
    if (m_added_mass.rows() != size) {
        if (m_verbose)
            std::cout << "Resize added_mass matrix at t = " << time << std::endl;

        m_KRM.GetMatrix().resize(size, size);
        m_added_mass.resize(size, size);

        m_added_mass.setZero();
        for (const auto& b1 : m_body_blocks) {
            const auto& row_block = b1.second;  // 6 x (6*n)
            auto row = b1.first->GetOffset_w();
            int i = 0;
            for (const auto& b2 : m_body_blocks) {
                auto col = b2.first->GetOffset_w();
                if (m_verbose)
                    std::cout << "  add 6x6 block starting at (" << row << "," << col << ")" << std::endl;
                m_added_mass.block(row, col, 6, 6) = row_block(Eigen::seq(0, 5), Eigen::seq(i, i + 5));
                i += 6;
            }
        }
    }

    // Overloading of base class:
    ChPhysicsItem::Update(time, update_assets);
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
        auto offset = b.first->GetOffset_w();
        w1(Eigen::seq(i, i + 5)) = w(Eigen::seq(offset, offset + 5));
        i += 6;
    }

    // Update R += c * M * w
    ChVectorDynamic<> cMw1 = c * m_added_mass * w1;
    i = 0;
    for (const auto& b : m_body_blocks) {
        auto offset = b.first->GetOffset_w();
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
