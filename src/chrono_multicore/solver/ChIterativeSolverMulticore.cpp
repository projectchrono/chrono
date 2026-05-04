// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: This class calls the multicore solver, used as an intermediate
// between chrono's solver interface and the multicore solver interface.
//
// =============================================================================

#include "chrono_multicore/solver/ChIterativeSolverMulticore.h"

#include "chrono/physics/ChBody.h"

using namespace chrono;

ChIterativeSolverMulticore::ChIterativeSolverMulticore(ChMulticoreDataManager* dc) : data_manager(dc) {
    m_tolerance = 1e-7;
    m_warm_start = false;
    solver = new ChSolverMulticoreAPGD();
    bilateral_solver = new ChSolverMulticoreMinRes();
    data_manager->rigid_rigid = new ChConstraintRigidRigid();
    data_manager->bilateral = new ChConstraintBilateral();
}

ChIterativeSolverMulticore::~ChIterativeSolverMulticore() {
    delete solver;
    delete bilateral_solver;
}

void ChIterativeSolverMulticore::ComputeInvMassMatrix() {
    uint num_bodies = data_manager->num_rigid_bodies;
    uint num_shafts = data_manager->num_shafts;
    uint num_motors = data_manager->num_motors;
    uint num_particles = data_manager->num_particles;
    uint num_dof = data_manager->num_dof;
    bool use_full_inertia_tensor = data_manager->settings.solver.use_full_inertia_tensor;
    const custom_vector<real>& shaft_inr = data_manager->host_data.shaft_inr;

    std::vector<std::shared_ptr<ChBody>>* body_list = data_manager->body_list;

    const VectorType& hf = data_manager->host_data.hf;
    const VectorType& v = data_manager->host_data.v;

    VectorType& M_invk = data_manager->host_data.M_invk;
    SparseMatrixType& M_inv = data_manager->host_data.M_inv;

    // The mass matrix is square and each rigid body has 6 DOF
    // Shafts have one DOF
    M_inv.resize(num_dof, num_dof);
    // Each rigid object has 3 mass entries and 9 inertia entries
    // Each shaft has one inertia entry
    // Each motor has one "mass" entry
    M_inv.reserve(num_bodies * 12 + num_shafts * 1 + num_motors * 1 + num_particles * 3);

    for (int i = 0; i < (signed)num_bodies; i++) {
        if (data_manager->host_data.active_rigid[i]) {
            real inv_mass = 1.0 / body_list->at(i)->GetMass();
            const ChMatrix33<>& body_inv_inr = body_list->at(i)->GetInvInertia();

            M_inv.insert(i * 6 + 0, i * 6 + 0) = inv_mass;
            M_inv.insert(i * 6 + 1, i * 6 + 1) = inv_mass;
            M_inv.insert(i * 6 + 2, i * 6 + 2) = inv_mass;

            M_inv.insert(i * 6 + 3, i * 6 + 3) = body_inv_inr(0, 0);
            if (use_full_inertia_tensor) {
                M_inv.insert(i * 6 + 3, i * 6 + 4) = body_inv_inr(0, 1);
                M_inv.insert(i * 6 + 3, i * 6 + 5) = body_inv_inr(0, 2);
            }
            if (use_full_inertia_tensor) {
                M_inv.insert(i * 6 + 4, i * 6 + 3) = body_inv_inr(1, 0);
            }
            M_inv.insert(i * 6 + 4, i * 6 + 4) = body_inv_inr(1, 1);
            if (use_full_inertia_tensor) {
                M_inv.insert(i * 6 + 4, i * 6 + 5) = body_inv_inr(1, 2);
            }
            if (use_full_inertia_tensor) {
                M_inv.insert(i * 6 + 5, i * 6 + 3) = body_inv_inr(2, 0);
                M_inv.insert(i * 6 + 5, i * 6 + 4) = body_inv_inr(2, 1);
            }
            M_inv.insert(i * 6 + 5, i * 6 + 5) = body_inv_inr(2, 2);
        }
    }

    for (int i = 0; i < (signed)num_shafts; i++) {
        M_inv.insert(num_bodies * 6 + i, num_bodies * 6 + i) = shaft_inr[i];
    }

    for (int i = 0; i < (signed)num_motors; i++) {
        M_inv.insert(num_bodies * 6 + num_shafts + i, num_bodies * 6 + num_shafts + i) = 1.0;
    }

    M_inv.makeCompressed();

    int offset = num_bodies * 6 + num_shafts + num_motors;
    data_manager->node_container->ComputeInvMass(offset);

    M_invk = v + M_inv * hf;
}

void ChIterativeSolverMulticore::ComputeMassMatrix() {
    uint num_bodies = data_manager->num_rigid_bodies;
    uint num_shafts = data_manager->num_shafts;
    uint num_motors = data_manager->num_motors;
    uint num_particles = data_manager->num_particles;
    uint num_dof = data_manager->num_dof;
    bool use_full_inertia_tensor = data_manager->settings.solver.use_full_inertia_tensor;
    const custom_vector<real>& shaft_inr = data_manager->host_data.shaft_inr;

    std::vector<std::shared_ptr<ChBody>>* body_list = data_manager->body_list;

    SparseMatrixType& M = data_manager->host_data.M;

    // The mass matrix is square and each rigid body has 6 DOF
    // Shafts have one DOF
    M.resize(num_dof, num_dof);
    // Each rigid object has 3 mass entries and 9 inertia entries
    // Each shaft has one inertia entry
    // Each motor has one "mass" entry
    M.reserve(num_bodies * 12 + num_shafts * 1 + num_motors * 1 + num_particles * 3);

    for (int i = 0; i < (signed)num_bodies; i++) {
        if (data_manager->host_data.active_rigid[i]) {
            real mass = body_list->at(i)->GetMass();
            const ChMatrix33<>& body_inr = body_list->at(i)->GetInertia();

            M.insert(i * 6 + 0, i * 6 + 0) = mass;
            M.insert(i * 6 + 1, i * 6 + 1) = mass;
            M.insert(i * 6 + 2, i * 6 + 2) = mass;

            M.insert(i * 6 + 3, i * 6 + 3) = body_inr(0, 0);
            if (use_full_inertia_tensor) {
                M.insert(i * 6 + 3, i * 6 + 4) = body_inr(0, 1);
                M.insert(i * 6 + 3, i * 6 + 5) = body_inr(0, 2);
            }
            if (use_full_inertia_tensor) {
                M.insert(i * 6 + 4, i * 6 + 3) = body_inr(1, 0);
            }
            M.insert(i * 6 + 4, i * 6 + 4) = body_inr(1, 1);
            if (use_full_inertia_tensor) {
                M.insert(i * 6 + 4, i * 6 + 5) = body_inr(1, 2);
            }
            if (use_full_inertia_tensor) {
                M.insert(i * 6 + 5, i * 6 + 3) = body_inr(2, 0);
                M.insert(i * 6 + 5, i * 6 + 4) = body_inr(2, 1);
            }
            M.insert(i * 6 + 5, i * 6 + 5) = body_inr(2, 2);
        }
    }

    for (int i = 0; i < (signed)num_shafts; i++) {
        M.insert(num_bodies * 6 + i, num_bodies * 6 + i) = 1.0 / shaft_inr[i];
    }

    for (int i = 0; i < (signed)num_motors; i++) {
        M.insert(num_bodies * 6 + num_shafts + i, num_bodies * 6 + num_shafts + i) = 1.0;
    }

    M.makeCompressed();

    int offset = num_bodies * 6 + num_shafts + num_motors;
    data_manager->node_container->ComputeMass(offset);
}

void ChIterativeSolverMulticore::PerformStabilization() {
    const VectorType& R_full = data_manager->host_data.R_full;
    VectorType& gamma = data_manager->host_data.gamma;
    uint num_unilaterals = data_manager->num_unilaterals;
    uint num_bilaterals = data_manager->num_bilaterals;

    data_manager->system_timer.start("ChIterativeSolverMulticore_Stab");

    if (data_manager->settings.solver.max_iteration_bilateral > 0 && num_bilaterals > 0) {
        const VectorType R_b = R_full.segment(num_unilaterals, num_bilaterals);
        VectorType gamma_b = gamma.segment(num_unilaterals, num_bilaterals);

        data_manager->measures.solver.total_iteration += bilateral_solver->Solve(SchurProductBilateral,                                  //
                                                                                 ProjectNone,                                            //
                                                                                 data_manager->settings.solver.max_iteration_bilateral,  //
                                                                                 num_bilaterals,                                         //
                                                                                 R_b,                                                    //
                                                                                 gamma_b);                                               //
        gamma.segment(num_unilaterals, num_bilaterals) = gamma_b;
    }

    data_manager->system_timer.stop("ChIterativeSolverMulticore_Stab");
}

real ChIterativeSolverMulticore::GetResidual() const {
    return data_manager->measures.solver.maxd_hist.size() > 0 ? data_manager->measures.solver.maxd_hist.back() : 0.0;
}
