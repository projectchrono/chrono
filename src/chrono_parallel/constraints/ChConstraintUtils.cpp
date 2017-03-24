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
// Constraint utility functions
// =============================================================================

#include "chrono_parallel/constraints/ChConstraintUtils.h"

namespace chrono {

CH_PARALLEL_API
void Orthogonalize(real3& Vx, real3& Vy, real3& Vz) {
    real3 mVsingular = real3(0, 1, 0);
    Vz = Cross(Vx, mVsingular);
    real mzlen = Length(Vz);
    // was near singularity? change singularity reference vector!
    if (mzlen < real(0.0001)) {
        mVsingular = real3(1, 0, 0);
        Vz = Cross(Vx, mVsingular);
        mzlen = Length(Vz);
    }
    Vz = Vz / mzlen;
    Vy = Cross(Vz, Vx);
}

CH_PARALLEL_API
void Compute_Jacobian(const quaternion& quat,
                      const real3& U,
                      const real3& V,
                      const real3& W,
                      const real3& point,
                      real3& T1,
                      real3& T2,
                      real3& T3) {
    quaternion quaternion_conjugate = ~quat;
    real3 sbar = Rotate(point, quaternion_conjugate);

    T1 = Cross(Rotate(U, quaternion_conjugate), sbar);
    T2 = Cross(Rotate(V, quaternion_conjugate), sbar);
    T3 = Cross(Rotate(W, quaternion_conjugate), sbar);
}
CH_PARALLEL_API
void Compute_Jacobian_Rolling(const quaternion& quat,
                              const real3& U,
                              const real3& V,
                              const real3& W,
                              real3& T1,
                              real3& T2,
                              real3& T3) {
    quaternion quaternion_conjugate = ~quat;

    T1 = Rotate(U, quaternion_conjugate);
    T2 = Rotate(V, quaternion_conjugate);
    T3 = Rotate(W, quaternion_conjugate);
}
CH_PARALLEL_API
bool Cone_generalized_rigid(real& gamma_n, real& gamma_u, real& gamma_v, const real& mu) {
    real f_tang = sqrt(gamma_u * gamma_u + gamma_v * gamma_v);

    // inside upper cone? keep untouched!
    if (f_tang < (mu * gamma_n)) {
        return false;
    }

    // inside lower cone? reset  normal,u,v to zero!
    if ((f_tang) < -(1.0 / mu) * gamma_n || (fabs(gamma_n) < 10e-15)) {
        gamma_n = 0;
        gamma_u = 0;
        gamma_v = 0;
        return false;
    }

    // remaining case: project orthogonally to generator segment of upper cone

    gamma_n = (f_tang * mu + gamma_n) / (mu * mu + 1);
    real tproj_div_t = (gamma_n * mu) / f_tang;
    gamma_u *= tproj_div_t;
    gamma_v *= tproj_div_t;

    return true;
}
CH_PARALLEL_API
void AppendRigidFluidBoundary(const real contact_mu,
                              const uint num_fluid_bodies,
                              const uint body_offset,
                              const uint start_boundary,
                              ChParallelDataManager* data_manager) {
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    uint num_rigid_fluid_contacts = data_manager->num_rigid_fluid_contacts;
    if (num_rigid_fluid_contacts > 0) {
        int index_t = 0;

        custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
        custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;

        Loop_Over_Rigid_Neighbors(int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];
                                  AppendRow6(D_T, start_boundary + index + 0, rigid * 6, 0);
                                  AppendRow3(D_T, start_boundary + index + 0, body_offset + p * 3, 0);
                                  D_T.finalize(start_boundary + index + 0););
        if (contact_mu != 0) {
            Loop_Over_Rigid_Neighbors(
                int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];

                AppendRow6(D_T, start_boundary + num_rigid_fluid_contacts + index * 2 + 0, rigid * 6, 0);
                AppendRow3(D_T, start_boundary + num_rigid_fluid_contacts + index * 2 + 0, body_offset + p * 3, 0);
                D_T.finalize(start_boundary + num_rigid_fluid_contacts + index * 2 + 0);

                AppendRow6(D_T, start_boundary + num_rigid_fluid_contacts + index * 2 + 1, rigid * 6, 0);
                AppendRow3(D_T, start_boundary + num_rigid_fluid_contacts + index * 2 + 1, body_offset + p * 3, 0);

                D_T.finalize(start_boundary + num_rigid_fluid_contacts + index * 2 + 1););
        }
    }
}
CH_PARALLEL_API
void ProjectRigidFluidBoundary(const real contact_mu,
                               const real contact_cohesion,
                               const uint num_fluid_bodies,
                               const uint start_boundary,
                               real* gamma,
                               ChParallelDataManager* data_manager) {
    custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
    custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;
    uint num_rigid_fluid_contacts = data_manager->num_rigid_fluid_contacts;

    if (contact_mu == 0) {
#pragma omp parallel for
        Loop_Over_Rigid_Neighbors(
            int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];  // rigid is stored in the first index
            real cohesion = Max((data_manager->host_data.cohesion_data[rigid] + contact_cohesion) * .5, 0.0); real3 gam;
            gam.x = gamma[start_boundary + index];     //
            gam.x += cohesion;                         //
            gam.x = gam.x < 0 ? 0 : gam.x - cohesion;  //
            gamma[start_boundary + index] = gam.x;);
    } else {
#pragma omp parallel for
        Loop_Over_Rigid_Neighbors(
            int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];  // rigid is stored in the first index
            real rigid_fric = data_manager->host_data.fric_data[rigid].x;
            real cohesion = Max((data_manager->host_data.cohesion_data[rigid] + contact_cohesion) * .5, 0.0);
            real friction = (rigid_fric == 0 || contact_mu == 0) ? 0 : (rigid_fric + contact_mu) * .5;

            real3 gam;                              //
            gam.x = gamma[start_boundary + index];  //
            gam.y = gamma[start_boundary + num_rigid_fluid_contacts + index * 2 + 0];
            gam.z = gamma[start_boundary + num_rigid_fluid_contacts + index * 2 + 1];

            gam.x += cohesion;  //

            real mu = friction;  //
            if (mu == 0) {
                gam.x = gam.x < 0 ? 0 : gam.x - cohesion;  //
                gam.y = gam.z = 0;                         //

                gamma[start_boundary + index] = gam.x;
                gamma[start_boundary + num_rigid_fluid_contacts + index * 2 + 0] = gam.y;
                gamma[start_boundary + num_rigid_fluid_contacts + index * 2 + 1] = gam.z;
                continue;
            }

            if (Cone_generalized_rigid(gam.x, gam.y, gam.z, mu)) {}

            gamma[start_boundary + index] = gam.x - cohesion;  //
            gamma[start_boundary + num_rigid_fluid_contacts + index * 2 + 0] = gam.y;
            gamma[start_boundary + num_rigid_fluid_contacts + index * 2 + 1] = gam.z;);
    }
}
CH_PARALLEL_API
void ComplianceRigidFluidBoundary(const real contact_mu,
                                  const real contact_compliance,
                                  const real alpha,
                                  const uint start_boundary,
                                  ChParallelDataManager* data_manager) {
    DynamicVector<real>& E = data_manager->host_data.E;
    uint num_rigid_fluid_contacts = data_manager->num_rigid_fluid_contacts;
    real inv_h = 1.0 / data_manager->settings.step_size;
    real inv_hpa = 1.0 / (data_manager->settings.step_size + alpha);
    real inv_hhpa = inv_h * inv_hpa;
    real com = 0;
    if (alpha) {
        com = inv_hhpa * contact_compliance;
    }
    if (num_rigid_fluid_contacts > 0) {
        if (contact_mu == 0) {
#pragma omp parallel for
            for (int index = 0; index < (signed)num_rigid_fluid_contacts; index++) {
                E[start_boundary + index + 0] = com;
            }
        } else {
#pragma omp parallel for
            for (int index = 0; index < (signed)num_rigid_fluid_contacts; index++) {
                E[start_boundary + index + 0] = com;
                E[start_boundary + num_rigid_fluid_contacts + index * 2 + 0] = 0;
                E[start_boundary + num_rigid_fluid_contacts + index * 2 + 1] = 0;
            }
        }
    }
}
CH_PARALLEL_API
void CorrectionRigidFluidBoundary(const real contact_mu,
                                  const real contact_cohesion,
                                  const real alpha,
                                  const real contact_recovery_speed,
                                  const uint num_fluid_bodies,
                                  const uint start_boundary,
                                  ChParallelDataManager* data_manager) {
    real inv_h = 1 / data_manager->settings.step_size;
    real inv_hpa = 1.0 / (data_manager->settings.step_size + alpha);
    real inv_hhpa = inv_h * inv_hpa;

    real dt = data_manager->settings.step_size;
    DynamicVector<real>& b = data_manager->host_data.b;
    custom_vector<real>& dpth_rigid_fluid = data_manager->host_data.dpth_rigid_fluid;
    uint num_rigid_fluid_contacts = data_manager->num_rigid_fluid_contacts;
    if (num_rigid_fluid_contacts > 0) {
        custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
        custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;

        if (contact_mu == 0) {
#pragma omp parallel for
            Loop_Over_Rigid_Neighbors(real depth = dpth_rigid_fluid[p * max_rigid_neighbors + i];              //
                                      real bi = 0;                                                             //
                                      if (contact_cohesion) { depth = Min(depth, 0); }                         //
                                      if (alpha) { bi = std::max(inv_hpa * depth, -contact_recovery_speed); }  //
                                      else { bi = std::max(real(1.0) / dt * depth, -contact_recovery_speed); }

                                      b[start_boundary + index + 0] = bi;  //
                                      // printf("Depth: %f %f %f\n", depth, kernel_radius, bi);
                                      );
        } else {
#pragma omp parallel for
            Loop_Over_Rigid_Neighbors(real depth = dpth_rigid_fluid[p * max_rigid_neighbors + i];              //
                                      real bi = 0;                                                             //
                                      if (contact_cohesion) { depth = Min(depth, 0); }                         //
                                      if (alpha) { bi = std::max(inv_hpa * depth, -contact_recovery_speed); }  //
                                      else { bi = std::max(real(1.0) / dt * depth, -contact_recovery_speed); }

                                      b[start_boundary + index + 0] = bi;
                                      b[start_boundary + num_rigid_fluid_contacts + index * 2 + 0] = 0;
                                      b[start_boundary + num_rigid_fluid_contacts + index * 2 + 1] = 0;);
        }
    }
}
CH_PARALLEL_API
void BuildRigidFluidBoundary(const real contact_mu,
                             const uint num_fluid_bodies,
                             const uint body_offset,
                             const uint start_boundary,
                             ChParallelDataManager* data_manager) {
    uint num_rigid_fluid_contacts = data_manager->num_rigid_fluid_contacts;
    if (num_rigid_fluid_contacts > 0) {
        CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
        custom_vector<real3>& pos_rigid = data_manager->host_data.pos_rigid;
        custom_vector<quaternion>& rot_rigid = data_manager->host_data.rot_rigid;

        // custom_vector<vec2>& bids = data_manager->host_data.bids_rigid_fluid;
        custom_vector<real3>& cpta = data_manager->host_data.cpta_rigid_fluid;
        custom_vector<real3>& norm = data_manager->host_data.norm_rigid_fluid;
        custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
        custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;

        if (contact_mu == 0) {
#pragma omp parallel for
            Loop_Over_Rigid_Neighbors(
                int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];
                real3 U = norm[p * max_rigid_neighbors + i]; real3 V; real3 W; Orthogonalize(U, V, W);  //
                real3 T1; real3 T2; real3 T3;                                                           //
                Compute_Jacobian(rot_rigid[rigid], U, V, W, cpta[p * max_rigid_neighbors + i] - pos_rigid[rigid], T1,
                                 T2, T3);

                SetRow6Check(D_T, start_boundary + index + 0, rigid * 6, -U, T1);
                SetRow3Check(D_T, start_boundary + index + 0, body_offset + p * 3, U););
        } else {
#pragma omp parallel for
            Loop_Over_Rigid_Neighbors(
                int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];
                real3 U = norm[p * max_rigid_neighbors + i]; real3 V; real3 W;  //
                Orthogonalize(U, V, W);                                         //
                real3 T1; real3 T2; real3 T3;                                   //
                Compute_Jacobian(rot_rigid[rigid], U, V, W, cpta[p * max_rigid_neighbors + i] - pos_rigid[rigid], T1,
                                 T2, T3);

                SetRow6Check(D_T, start_boundary + index + 0, rigid * 6, -U, T1);
                SetRow6Check(D_T, start_boundary + num_rigid_fluid_contacts + index * 2 + 0, rigid * 6, -V, T2);
                SetRow6Check(D_T, start_boundary + num_rigid_fluid_contacts + index * 2 + 1, rigid * 6, -W, T3);

                SetRow3Check(D_T, start_boundary + index + 0, body_offset + p * 3, U);
                SetRow3Check(D_T, start_boundary + num_rigid_fluid_contacts + index * 2 + 0, body_offset + p * 3, V);
                SetRow3Check(D_T, start_boundary + num_rigid_fluid_contacts + index * 2 + 1, body_offset + p * 3, W););
        }
    }
}
}
