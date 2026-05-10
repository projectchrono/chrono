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

static Eigen::VectorXi MassNNZ(int num_dof, int num_bodies, int num_shafts, int num_motors,
                                int num_particles, bool full_inertia,
                                const custom_vector<char>& active) {
    Eigen::VectorXi nnz(num_dof);
    nnz.setZero();
    const int k = full_inertia ? 3 : 1;
    for (int i = 0; i < num_bodies; i++) {
        if (active[i]) {
            nnz[i*6+0] = nnz[i*6+1] = nnz[i*6+2] = 1;
            nnz[i*6+3] = nnz[i*6+4] = nnz[i*6+5] = k;
        }
    }
    const int s = num_bodies * 6, m = s + num_shafts, p = m + num_motors;
    for (int i = 0; i < num_shafts;    i++) nnz[s + i] = 1;
    for (int i = 0; i < num_motors;    i++) nnz[m + i] = 1;
    for (int i = 0; i < num_particles; i++)
        nnz[p + i*3] = nnz[p + i*3+1] = nnz[p + i*3+2] = 1;
    return nnz;
}


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
    uint num_bodies    = data_manager->num_rigid_bodies;
    uint num_shafts    = data_manager->num_shafts;
    uint num_motors    = data_manager->num_motors;
    uint num_particles = data_manager->num_particles;
    uint num_dof       = data_manager->num_dof;
    bool full_inr      = data_manager->settings.solver.use_full_inertia_tensor;

    const custom_vector<real>& shaft_inr = data_manager->host_data.shaft_inr;
    std::vector<std::shared_ptr<ChBody>>* body_list = data_manager->body_list;
    const VectorType& hf = data_manager->host_data.hf;
    const VectorType& v  = data_manager->host_data.v;
    VectorType& M_invk   = data_manager->host_data.M_invk;
    SparseMatrixType& M_inv = data_manager->host_data.M_inv;

    const bool needs_build = !M_inv.isCompressed() || M_inv.rows() != (int)num_dof;

    if (needs_build) {
        M_inv = SparseMatrixType(num_dof, num_dof);
        M_inv.reserve(MassNNZ((int)num_dof, (int)num_bodies, (int)num_shafts,
                              (int)num_motors, (int)num_particles,
                              full_inr, data_manager->host_data.active_rigid));
    }

    auto set = [&](int r, int c, real val) {
        if (needs_build) M_inv.insert(r, c)   = val;
        else             M_inv.coeffRef(r, c) = val;
    };

    for (int i = 0; i < (signed)num_bodies; i++) {
        if (data_manager->host_data.active_rigid[i]) {
            const real inv_mass = 1.0 / body_list->at(i)->GetMass();
            const ChMatrix33<>& J = body_list->at(i)->GetInvInertia();
            const int b = i * 6;
            set(b, b, inv_mass);
            set(b+1, b+1, inv_mass);
            set(b+2, b+2, inv_mass);
            set(b+3, b+3, J(0,0));
            if (full_inr) {
                set(b+3, b+4, J(0,1));
                set(b+3, b+5, J(0,2));
                set(b+4, b+3, J(1,0));
            }
            set(b+4, b+4, J(1,1));
            if (full_inr) {
                set(b+4, b+5, J(1,2));
                set(b+5, b+3, J(2,0));
                set(b+5, b+4, J(2,1));
            }
            set(b+5, b+5, J(2,2));
        }
    }

    const int s = (int)num_bodies * 6, m = s + (int)num_shafts;
    for (int i = 0; i < (signed)num_shafts; i++) set(s + i, s + i, shaft_inr[i]);
    for (int i = 0; i < (signed)num_motors; i++) set(m + i, m + i, 1.0);

    data_manager->node_container->ComputeInvMass(m + (int)num_motors);
    if (needs_build) M_inv.makeCompressed();

    M_invk.noalias() = v + M_inv * hf;
}

void ChIterativeSolverMulticore::ComputeMassMatrix() {
    uint num_bodies    = data_manager->num_rigid_bodies;
    uint num_shafts    = data_manager->num_shafts;
    uint num_motors    = data_manager->num_motors;
    uint num_particles = data_manager->num_particles;
    uint num_dof       = data_manager->num_dof;
    bool full_inr      = data_manager->settings.solver.use_full_inertia_tensor;

    const custom_vector<real>& shaft_inr = data_manager->host_data.shaft_inr;
    std::vector<std::shared_ptr<ChBody>>* body_list = data_manager->body_list;
    SparseMatrixType& M = data_manager->host_data.M;

    const bool needs_build = !M.isCompressed() || M.rows() != (int)num_dof;

    if (needs_build) {
        M = SparseMatrixType(num_dof, num_dof);
        M.reserve(MassNNZ((int)num_dof, (int)num_bodies, (int)num_shafts,
                          (int)num_motors, (int)num_particles,
                          full_inr, data_manager->host_data.active_rigid));
    }

    auto set = [&](int r, int c, real val) {
        if (needs_build) M.insert(r, c)   = val;
        else             M.coeffRef(r, c) = val;
    };

    for (int i = 0; i < (signed)num_bodies; i++) {
        if (data_manager->host_data.active_rigid[i]) {
            const real mass = body_list->at(i)->GetMass();
            const ChMatrix33<>& J = body_list->at(i)->GetInertia();
            const int b = i * 6;
            set(b, b, mass);
            set(b+1, b+1, mass);
            set(b+2, b+2, mass);
            set(b+3, b+3, J(0,0));
            if (full_inr) {
                set(b+3, b+4, J(0,1));
                set(b+3, b+5, J(0,2));
                set(b+4, b+3, J(1,0));
            }
            set(b+4, b+4, J(1,1));
            if (full_inr) {
                set(b+4, b+5, J(1,2));
                set(b+5, b+3, J(2,0));
                set(b+5, b+4, J(2,1));
            }
            set(b+5, b+5, J(2,2));
        }
    }

    const int s = (int)num_bodies * 6, m = s + (int)num_shafts;
    for (int i = 0; i < (signed)num_shafts; i++) set(s + i, s + i, 1.0 / shaft_inr[i]);
    for (int i = 0; i < (signed)num_motors; i++) set(m + i, m + i, 1.0);

    data_manager->node_container->ComputeMass(m + (int)num_motors);
    if (needs_build) M.makeCompressed();
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

        data_manager->measures.solver.total_iteration += bilateral_solver->Solve(SchurProductBilateral,
                                                                                 ProjectNone,
                                                                                 data_manager->settings.solver.max_iteration_bilateral,
                                                                                 num_bilaterals,
                                                                                 R_b,
                                                                                 gamma_b);
        gamma.segment(num_unilaterals, num_bilaterals) = gamma_b;
    }

    data_manager->system_timer.stop("ChIterativeSolverMulticore_Stab");
}

real ChIterativeSolverMulticore::GetResidual() const {
    return data_manager->measures.solver.maxd_hist.size() > 0 ? data_manager->measures.solver.maxd_hist.back() : 0.0;
}
