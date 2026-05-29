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
    const int num_bodies    = (int)data_manager->num_rigid_bodies;
    const int num_shafts    = (int)data_manager->num_shafts;
    const int num_motors    = (int)data_manager->num_motors;
    const int num_particles = (int)data_manager->num_particles;
    const int num_dof       = (int)data_manager->num_dof;
    const bool full_inr     = data_manager->settings.solver.use_full_inertia_tensor;

    const custom_vector<real>& shaft_inr = data_manager->host_data.shaft_inr;
    std::vector<std::shared_ptr<ChBody>>& body_list = *data_manager->body_list;
    const VectorType& hf = data_manager->host_data.hf;
    const VectorType& v  = data_manager->host_data.v;
    VectorType& M_invk   = data_manager->host_data.M_invk;
    SparseMatrixType& M_inv = data_manager->host_data.M_inv;
    const char* active = data_manager->host_data.active_rigid.data();

    const bool needs_build = !M_inv.isCompressed() || M_inv.rows() != num_dof;
    const int s = num_bodies * 6, m = s + num_shafts;

    if (needs_build) {
        M_inv = SparseMatrixType(num_dof, num_dof);
        M_inv.reserve(MassNNZ(num_dof, num_bodies, num_shafts, num_motors, num_particles,
                              full_inr, data_manager->host_data.active_rigid));

        for (int i = 0; i < num_bodies; i++) {
            if (active[i]) {
                ChBody* body = body_list[i].get();
                const real inv_mass = 1.0 / body->GetMass();
                const ChMatrix33<>& J = body->GetInvInertia();
                const int b = i * 6;
                M_inv.insert(b,   b  ) = inv_mass;
                M_inv.insert(b+1, b+1) = inv_mass;
                M_inv.insert(b+2, b+2) = inv_mass;
                M_inv.insert(b+3, b+3) = J(0,0);
                if (full_inr) {
                    M_inv.insert(b+3, b+4) = J(0,1);
                    M_inv.insert(b+3, b+5) = J(0,2);
                }
                if (full_inr) { M_inv.insert(b+4, b+3) = J(1,0); }
                M_inv.insert(b+4, b+4) = J(1,1);
                if (full_inr) { M_inv.insert(b+4, b+5) = J(1,2); }
                if (full_inr) {
                    M_inv.insert(b+5, b+3) = J(2,0);
                    M_inv.insert(b+5, b+4) = J(2,1);
                }
                M_inv.insert(b+5, b+5) = J(2,2);
            }
        }
        for (int i = 0; i < num_shafts; i++) M_inv.insert(s+i, s+i) = shaft_inr[i];
        for (int i = 0; i < num_motors; i++) M_inv.insert(m+i, m+i) = 1.0;

        data_manager->node_container->ComputeInvMass(m + num_motors);
        M_inv.makeCompressed();
    } else {
        // in row major CSR, outerIndexPtr[r] is the start index of row r in valuePtr.
        // pattern is fixed, so we can bypass coeffRef's binary search with direct buffer writes.
        real* vals       = M_inv.valuePtr();
        const int* outer = M_inv.outerIndexPtr();

        for (int i = 0; i < num_bodies; i++) {
            if (active[i]) {
                ChBody* body = body_list[i].get();
                const real inv_mass = 1.0 / body->GetMass();
                const ChMatrix33<>& J = body->GetInvInertia();
                const int b = i * 6;
                vals[outer[b]]   = inv_mass;
                vals[outer[b+1]] = inv_mass;
                vals[outer[b+2]] = inv_mass;
                if (full_inr) {
                    vals[outer[b+3]] = J(0,0); vals[outer[b+3]+1] = J(0,1); vals[outer[b+3]+2] = J(0,2);
                    vals[outer[b+4]] = J(1,0); vals[outer[b+4]+1] = J(1,1); vals[outer[b+4]+2] = J(1,2);
                    vals[outer[b+5]] = J(2,0); vals[outer[b+5]+1] = J(2,1); vals[outer[b+5]+2] = J(2,2);
                } else {
                    vals[outer[b+3]] = J(0,0);
                    vals[outer[b+4]] = J(1,1);
                    vals[outer[b+5]] = J(2,2);
                }
            }
        }
        for (int i = 0; i < num_shafts; i++) vals[outer[s+i]] = shaft_inr[i];
        for (int i = 0; i < num_motors; i++) vals[outer[m+i]] = 1.0;

        data_manager->node_container->ComputeInvMass(m + num_motors);
    }

    M_invk.noalias() = v + M_inv * hf;
}

void ChIterativeSolverMulticore::ComputeMassMatrix() {
    const int num_bodies    = (int)data_manager->num_rigid_bodies;
    const int num_shafts    = (int)data_manager->num_shafts;
    const int num_motors    = (int)data_manager->num_motors;
    const int num_particles = (int)data_manager->num_particles;
    const int num_dof       = (int)data_manager->num_dof;
    const bool full_inr     = data_manager->settings.solver.use_full_inertia_tensor;

    const custom_vector<real>& shaft_inr = data_manager->host_data.shaft_inr;
    std::vector<std::shared_ptr<ChBody>>& body_list = *data_manager->body_list;
    SparseMatrixType& M = data_manager->host_data.M;
    const char* active = data_manager->host_data.active_rigid.data();

    const bool needs_build = !M.isCompressed() || M.rows() != num_dof;
    const int s = num_bodies * 6, m = s + num_shafts;

    if (needs_build) {
        M = SparseMatrixType(num_dof, num_dof);
        M.reserve(MassNNZ(num_dof, num_bodies, num_shafts, num_motors, num_particles,
                          full_inr, data_manager->host_data.active_rigid));

        for (int i = 0; i < num_bodies; i++) {
            if (active[i]) {
                ChBody* body = body_list[i].get();
                const real mass = body->GetMass();
                const ChMatrix33<>& J = body->GetInertia();
                const int b = i * 6;
                M.insert(b,   b  ) = mass;
                M.insert(b+1, b+1) = mass;
                M.insert(b+2, b+2) = mass;
                M.insert(b+3, b+3) = J(0,0);
                if (full_inr) {
                    M.insert(b+3, b+4) = J(0,1);
                    M.insert(b+3, b+5) = J(0,2);
                }
                if (full_inr) { M.insert(b+4, b+3) = J(1,0); }
                M.insert(b+4, b+4) = J(1,1);
                if (full_inr) { M.insert(b+4, b+5) = J(1,2); }
                if (full_inr) {
                    M.insert(b+5, b+3) = J(2,0);
                    M.insert(b+5, b+4) = J(2,1);
                }
                M.insert(b+5, b+5) = J(2,2);
            }
        }
        for (int i = 0; i < num_shafts; i++) M.insert(s+i, s+i) = 1.0 / shaft_inr[i];
        for (int i = 0; i < num_motors; i++) M.insert(m+i, m+i) = 1.0;

        data_manager->node_container->ComputeMass(m + num_motors);
        M.makeCompressed();
    } else {
        real* vals       = M.valuePtr();
        const int* outer = M.outerIndexPtr();

        for (int i = 0; i < num_bodies; i++) {
            if (active[i]) {
                ChBody* body = body_list[i].get();
                const real mass = body->GetMass();
                const ChMatrix33<>& J = body->GetInertia();
                const int b = i * 6;
                vals[outer[b]]   = mass;
                vals[outer[b+1]] = mass;
                vals[outer[b+2]] = mass;
                if (full_inr) {
                    vals[outer[b+3]] = J(0,0); vals[outer[b+3]+1] = J(0,1); vals[outer[b+3]+2] = J(0,2);
                    vals[outer[b+4]] = J(1,0); vals[outer[b+4]+1] = J(1,1); vals[outer[b+4]+2] = J(1,2);
                    vals[outer[b+5]] = J(2,0); vals[outer[b+5]+1] = J(2,1); vals[outer[b+5]+2] = J(2,2);
                } else {
                    vals[outer[b+3]] = J(0,0);
                    vals[outer[b+4]] = J(1,1);
                    vals[outer[b+5]] = J(2,2);
                }
            }
        }
        for (int i = 0; i < num_shafts; i++) vals[outer[s+i]] = 1.0 / shaft_inr[i];
        for (int i = 0; i < num_motors; i++) vals[outer[m+i]] = 1.0;

        data_manager->node_container->ComputeMass(m + num_motors);
    }
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
