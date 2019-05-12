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

#include <algorithm>
#include <cmath>

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/physics/Ch3DOFContainer.h"
#include "chrono_parallel/physics/ChMPM.cuh"
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/constraints/ChConstraintUtils.h"
#include "chrono_parallel/collision/ChCollision.h"

#include "chrono_parallel/math/other_types.h"  // for uint, vec2, vec3
#include "chrono_parallel/math/real.h"         // for real
#include "chrono_parallel/math/real2.h"        // for real2
#include "chrono_parallel/math/real3.h"        // for real3
#include "chrono_parallel/math/real4.h"        // for quaternion, real4
#include "chrono_parallel/math/matrix.h"       // for quaternion, real4

namespace chrono {

using namespace collision;
using namespace geometry;

ChParticleContainer::ChParticleContainer() {
    body_offset = 0;
    compliance = 0;
    mu = 0;
    cohesion = 0;
    mass = 0.037037;
    start_boundary = 0;
    start_contact = 0;
    mpm_iterations = 0;

    nu = .2;
    youngs_modulus = 1.4e5;
    hardening_coefficient = 10;
    lame_lambda = youngs_modulus * nu / ((1. + nu) * (1. - 2. * nu));
    lame_mu = youngs_modulus / (2. * (1. + nu));
    theta_s = 7.5e-3;
    theta_c = 2.5e-2;
    alpha_flip = .95;
    mpm_init = false;
}

void ChParticleContainer::AddBodies(const std::vector<real3>& positions, const std::vector<real3>& velocities) {
    custom_vector<real3>& pos_fluid = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_fluid = data_manager->host_data.vel_3dof;

    pos_fluid.insert(pos_fluid.end(), positions.begin(), positions.end());
    vel_fluid.insert(vel_fluid.end(), velocities.begin(), velocities.end());
    // In case the number of velocities provided were not enough, resize to the number of fluid bodies
    vel_fluid.resize(pos_fluid.size());
    data_manager->num_fluid_bodies = (uint)pos_fluid.size();
}
void ChParticleContainer::Update(double ChTime) {
    uint num_fluid_bodies = data_manager->num_fluid_bodies;
    uint num_rigid_bodies = data_manager->num_rigid_bodies;
    uint num_shafts = data_manager->num_shafts;
    uint num_motors = data_manager->num_motors;
    real3 h_gravity = data_manager->settings.step_size * mass * data_manager->settings.gravity;
#ifdef CHRONO_PARALLEL_USE_CUDA
    if (mpm_init) {
        temp_settings.dt = (float)data_manager->settings.step_size;
        temp_settings.kernel_radius = (float)kernel_radius;
        temp_settings.inv_radius = float(1.0 / kernel_radius);
        temp_settings.bin_edge = float(kernel_radius * 2);
        temp_settings.inv_bin_edge = float(1.0 / (kernel_radius * 2.0));
        temp_settings.max_velocity = (float)max_velocity;
        temp_settings.mu = (float)lame_mu;
        temp_settings.lambda = (float)lame_lambda;
        temp_settings.hardening_coefficient = (float)hardening_coefficient;
        temp_settings.theta_c = (float)theta_c;
        temp_settings.theta_s = (float)theta_s;
        temp_settings.alpha_flip = (float)alpha_flip;
        temp_settings.youngs_modulus = (float)youngs_modulus;
        temp_settings.poissons_ratio = (float)nu;
        temp_settings.num_mpm_markers = data_manager->num_fluid_bodies;
        temp_settings.mass = (float)mass;
        temp_settings.yield_stress = (float)yield_stress;
        temp_settings.num_iterations = mpm_iterations;

        if (mpm_iterations > 0) {
            mpm_pos.resize(data_manager->num_fluid_bodies * 3);
            mpm_vel.resize(data_manager->num_fluid_bodies * 3);
            mpm_jejp.resize(data_manager->num_fluid_bodies * 2);
            for (int i = 0; i < (signed)data_manager->num_fluid_bodies; i++) {
                mpm_pos[i * 3 + 0] = (float)data_manager->host_data.pos_3dof[i].x;
                mpm_pos[i * 3 + 1] = (float)data_manager->host_data.pos_3dof[i].y;
                mpm_pos[i * 3 + 2] = (float)data_manager->host_data.pos_3dof[i].z;
            }
            for (int i = 0; i < (signed)data_manager->num_fluid_bodies; i++) {
                mpm_vel[i * 3 + 0] = (float)data_manager->host_data.vel_3dof[i].x;
                mpm_vel[i * 3 + 1] = (float)data_manager->host_data.vel_3dof[i].y;
                mpm_vel[i * 3 + 2] = (float)data_manager->host_data.vel_3dof[i].z;
            }

            MPM_UpdateDeformationGradient(std::ref(temp_settings), std::ref(mpm_pos), std::ref(mpm_vel),
                                          std::ref(mpm_jejp));

            mpm_thread = std::thread(MPM_Solve, std::ref(temp_settings), std::ref(mpm_pos), std::ref(mpm_vel));

            //            for (int i = 0; i < data_manager->num_fluid_bodies; i++) {
            //                data_manager->host_data.vel_3dof[i].x = mpm_vel[i * 3 + 0];
            //                data_manager->host_data.vel_3dof[i].y = mpm_vel[i * 3 + 1];
            //                data_manager->host_data.vel_3dof[i].z = mpm_vel[i * 3 + 2];
            //            }
        }
    }
#endif

    uint offset = num_rigid_bodies * 6 + num_shafts + num_motors;
#pragma omp parallel for
    for (int i = 0; i < (signed)num_fluid_bodies; i++) {
        // This was moved to after fluid collision detection
        // real3 vel = vel_fluid[i];
        // data_manager->host_data.v[offset + i * 3 + 0] = vel.x;
        // data_manager->host_data.v[offset + i * 3 + 1] = vel.y;
        // data_manager->host_data.v[offset + i * 3 + 2] = vel.z;

        data_manager->host_data.hf[offset + i * 3 + 0] = h_gravity.x;
        data_manager->host_data.hf[offset + i * 3 + 1] = h_gravity.y;
        data_manager->host_data.hf[offset + i * 3 + 2] = h_gravity.z;
    }
}

void ChParticleContainer::UpdatePosition(double ChTime) {
    uint num_fluid_bodies = data_manager->num_fluid_bodies;
    uint num_rigid_bodies = data_manager->num_rigid_bodies;
    uint num_shafts = data_manager->num_shafts;
    uint num_motors = data_manager->num_motors;

    custom_vector<real3>& pos_fluid = data_manager->host_data.pos_3dof;
    custom_vector<real3>& sorted_pos_fluid = data_manager->host_data.sorted_pos_3dof;
    custom_vector<real3>& vel_fluid = data_manager->host_data.vel_3dof;

    uint offset = num_rigid_bodies * 6 + num_shafts + num_motors;
#pragma omp parallel for
    for (int i = 0; i < (signed)num_fluid_bodies; i++) {
        real3 vel;
        int original_index = data_manager->host_data.particle_indices_3dof[i];
        // these are sorted so we have to unsort them
        vel.x = data_manager->host_data.v[offset + i * 3 + 0];
        vel.y = data_manager->host_data.v[offset + i * 3 + 1];
        vel.z = data_manager->host_data.v[offset + i * 3 + 2];

        real speed = Length(vel);
        if (speed > max_velocity) {
            vel = vel * max_velocity / speed;
        }
        vel_fluid[original_index] = vel;
        pos_fluid[original_index] += vel * data_manager->settings.step_size;
        // sorted_pos_fluid[i] = pos_fluid[original_index];
    }

    //    if (num_fluid_bodies != 0) {
    //        data_manager->narrowphase->DispatchRigidFluid();
    //
    //        custom_vector<real3>& cpta = data_manager->host_data.cpta_rigid_fluid;
    //        custom_vector<real3>& norm = data_manager->host_data.norm_rigid_fluid;
    //        custom_vector<real>& dpth = data_manager->host_data.dpth_rigid_fluid;
    //        custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
    //        custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;
    //        // This treats all rigid neighbors as fixed. This correction should usually be pretty small if the
    //        timestep
    //        // isnt too large.
    //
    //        if (data_manager->num_rigid_fluid_contacts > 0) {
    //#pragma omp parallel for
    //            for (int p = 0; p < num_fluid_bodies; p++) {
    //                int start = contact_counts[p];
    //                int end = contact_counts[p + 1];
    //                real3 delta = real3(0);
    //                real weight = 0;
    //                for (int index = start; index < end; index++) {
    //                    int i = index - start;
    //                    // int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];
    //                    // if (data_manager->host_data.active_rigid[rigid] == false) {
    //                    real3 U = norm[p * max_rigid_neighbors + i];
    //                    real depth = dpth[p * max_rigid_neighbors + i];
    //                    if (depth < 0) {
    //                        real w = 1.0;  // mass / (mass + data_manager->host_data.mass_rigid[rigid]);
    //                        delta -= w * depth * U;
    //                        weight++;
    //                    }
    //                    //}
    //                }
    //                if (weight > 0) {
    //                    sorted_pos_fluid[p] += delta / weight;
    //                }
    //            }
    //        }
    //        real inv_dt = 1.0 / data_manager->settings.step_size;
    //#pragma omp parallel for
    //        for (int p = 0; p < num_fluid_bodies; p++) {
    //            int original_index = data_manager->host_data.particle_indices_3dof[p];
    //            real3 vv = real3((sorted_pos_fluid[p] - pos_fluid[original_index]) * inv_dt);
    //            if (contact_counts[p + 1] - contact_counts[p] > 0) {
    //                pos_fluid[original_index] = sorted_pos_fluid[p];
    //                vel_fluid[original_index] += vv;
    //            }
    //        }
    //    }
}

int ChParticleContainer::GetNumConstraints() {
    int num_fluid_fluid = 0;
    if (mu == 0) {
        num_fluid_fluid = (data_manager->num_fluid_contacts - data_manager->num_fluid_bodies) / 2;
    } else {
        num_fluid_fluid = (data_manager->num_fluid_contacts - data_manager->num_fluid_bodies) / 2 * 3;
    }

    if (contact_mu == 0) {
        num_fluid_fluid += data_manager->num_rigid_fluid_contacts;
    } else {
        num_fluid_fluid += data_manager->num_rigid_fluid_contacts * 3;
    }

    return num_fluid_fluid;
}

int ChParticleContainer::GetNumNonZeros() {
    int nnz_fluid_fluid = 0;
    if (mu == 0) {
        nnz_fluid_fluid = (data_manager->num_fluid_contacts - data_manager->num_fluid_bodies) / 2 * 6;

    } else {
        nnz_fluid_fluid = (data_manager->num_fluid_contacts - data_manager->num_fluid_bodies) / 2 * 6 * 3;
    }

    if (contact_mu == 0) {
        nnz_fluid_fluid += 9 * data_manager->num_rigid_fluid_contacts;
    } else {
        nnz_fluid_fluid += 9 * 3 * data_manager->num_rigid_fluid_contacts;
    }

    return nnz_fluid_fluid;
}

void ChParticleContainer::ComputeInvMass(int offset) {
    CompressedMatrix<real>& M_inv = data_manager->host_data.M_inv;
    uint num_fluid_bodies = data_manager->num_fluid_bodies;

    real inv_mass = 1.0 / mass;
    for (int i = 0; i < (float)num_fluid_bodies; i++) {
        M_inv.append(offset + i * 3 + 0, offset + i * 3 + 0, inv_mass);
        M_inv.finalize(offset + i * 3 + 0);
        M_inv.append(offset + i * 3 + 1, offset + i * 3 + 1, inv_mass);
        M_inv.finalize(offset + i * 3 + 1);
        M_inv.append(offset + i * 3 + 2, offset + i * 3 + 2, inv_mass);
        M_inv.finalize(offset + i * 3 + 2);
    }
}

void ChParticleContainer::ComputeMass(int offset) {
    CompressedMatrix<real>& M = data_manager->host_data.M;
    uint num_fluid_bodies = data_manager->num_fluid_bodies;

    real fluid_mass = mass;
    for (int i = 0; i < (float)num_fluid_bodies; i++) {
        M.append(offset + i * 3 + 0, offset + i * 3 + 0, fluid_mass);
        M.finalize(offset + i * 3 + 0);
        M.append(offset + i * 3 + 1, offset + i * 3 + 1, fluid_mass);
        M.finalize(offset + i * 3 + 1);
        M.append(offset + i * 3 + 2, offset + i * 3 + 2, fluid_mass);
        M.finalize(offset + i * 3 + 2);
    }
}

void ChParticleContainer::Setup(int start_constraint) {
    Ch3DOFContainer::Setup(start_constraint);

    start_boundary = start_constraint;
    if (contact_mu == 0) {
        start_contact = start_constraint + num_rigid_fluid_contacts;
    } else {
        start_contact = start_constraint + num_rigid_fluid_contacts * 3;
    }
    body_offset = num_rigid_bodies * 6 + num_shafts + num_motors;

    num_rigid_contacts = (num_fluid_contacts - num_fluid_bodies) / 2;
}

void ChParticleContainer::Initialize() {
#ifdef CHRONO_PARALLEL_USE_CUDA
    temp_settings.dt = (float)data_manager->settings.step_size;
    temp_settings.kernel_radius = (float)kernel_radius;
    temp_settings.inv_radius = float(1.0 / kernel_radius);
    temp_settings.bin_edge = float(kernel_radius * 2);
    temp_settings.inv_bin_edge = float(1.0 / (kernel_radius * 2.0));
    temp_settings.max_velocity = (float)max_velocity;
    temp_settings.mu = (float)lame_mu;
    temp_settings.lambda = (float)lame_lambda;
    temp_settings.hardening_coefficient = (float)hardening_coefficient;
    temp_settings.theta_c = (float)theta_c;
    temp_settings.theta_s = (float)theta_s;
    temp_settings.alpha_flip = (float)alpha_flip;
    temp_settings.youngs_modulus = (float)youngs_modulus;
    temp_settings.poissons_ratio = (float)nu;
    temp_settings.num_mpm_markers = data_manager->num_fluid_bodies;
    temp_settings.mass = (float)mass;
    temp_settings.yield_stress = (float)yield_stress;
    temp_settings.num_iterations = mpm_iterations;
    if (mpm_iterations > 0) {
        mpm_pos.resize(data_manager->num_fluid_bodies * 3);

        for (int i = 0; i < (signed)data_manager->num_fluid_bodies; i++) {
            mpm_pos[i * 3 + 0] = (float)data_manager->host_data.pos_3dof[i].x;
            mpm_pos[i * 3 + 1] = (float)data_manager->host_data.pos_3dof[i].y;
            mpm_pos[i * 3 + 2] = (float)data_manager->host_data.pos_3dof[i].z;
        }

        MPM_Initialize(temp_settings, mpm_pos);
    }
    mpm_init = true;
#endif
}

void ChParticleContainer::Build_D() {
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

    LOG(INFO) << "ChParticleContainer::Build_D"
              << " " << D_T.rows() << " " << D_T.columns();
    BuildRigidFluidBoundary(contact_mu, num_fluid_bodies, body_offset, start_boundary, data_manager);

    if (num_rigid_contacts > 0) {
        int index = 0;
        custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;

        if (mu == 0) {
            Loop_Over_Fluid_Neighbors(                                                       //
                real3 U = -Normalize(xij); real3 V; real3 W;                                 //
                Orthogonalize(U, V, W);                                                      //
                SetRow3Check(D_T, start_contact + index + 0, body_offset + body_a * 3, -U);  //
                SetRow3Check(D_T, start_contact + index + 0, body_offset + body_b * 3, U);   //
                );

        } else {
            Loop_Over_Fluid_Neighbors(
                real3 U = -Normalize(xij); real3 V; real3 W;

                Orthogonalize(U, V, W);

                // printf("set normal: [%d] [%d]\n", start_contact + index, body_offset + body_a * 3);

                SetRow3Check(D_T, start_contact + index + 0, body_offset + body_a * 3, -U);
                SetRow3Check(D_T, start_contact + num_rigid_contacts + index * 2 + 0, body_offset + body_a * 3, -V);
                SetRow3Check(D_T, start_contact + num_rigid_contacts + index * 2 + 1, body_offset + body_a * 3, -W);
                // printf("set normal: [%d] [%d]\n", start_contact + index, body_offset + body_b * 3);

                SetRow3Check(D_T, start_contact + index + 0, body_offset + body_b * 3, U);
                SetRow3Check(D_T, start_contact + num_rigid_contacts + index * 2 + 0, body_offset + body_b * 3, V);
                SetRow3Check(D_T, start_contact + num_rigid_contacts + index * 2 + 1, body_offset + body_b * 3, W););
        }
    }

    LOG(INFO) << "ChConstraintRigid3DOF::JACOBIAN OF RIGID";
}

void ChParticleContainer::Build_b() {
    real inv_hpa = 1.0 / (data_manager->settings.step_size + alpha);

    DynamicVector<real>& b = data_manager->host_data.b;

    CorrectionRigidFluidBoundary(contact_mu, contact_cohesion, alpha, contact_recovery_speed, num_fluid_bodies,
                                 start_boundary, data_manager);

    if (num_rigid_contacts > 0) {
        int index = 0;  // incremented in Loop_Over_Fluid_Neighbors()

        custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;

        if (mu == 0) {
            Loop_Over_Fluid_Neighbors(real depth = Length(xij) - kernel_radius;                 //
                                      real bi = 0;                                              //
                                      if (cohesion != 0) { depth = Min(depth, 0); }             //
                                      bi = std::max(inv_hpa * depth, -contact_recovery_speed);  //
                                      b[start_contact + index + 0] = bi;                        //
                                      );
        } else {
            Loop_Over_Fluid_Neighbors(real depth = Length(xij) - kernel_radius;                   //
                                      real bi = 0;                                                //
                                      if (cohesion != 0) { depth = Min(depth, 0); }               //
                                      bi = std::max(inv_hpa * depth, -contact_recovery_speed);    //
                                      b[start_contact + index + 0] = bi;                          //
                                      b[start_contact + num_rigid_contacts + index * 2 + 0] = 0;  //
                                      b[start_contact + num_rigid_contacts + index * 2 + 1] = 0;  //
                                      );
        }
    }
}

void ChParticleContainer::Build_E() {
    DynamicVector<real>& E = data_manager->host_data.E;

    ComplianceRigidFluidBoundary(contact_mu, contact_compliance, alpha, start_boundary, data_manager);
    real inv_h = 1.0 / data_manager->settings.step_size;
    real inv_hpa = 1.0 / (data_manager->settings.step_size + alpha);
    real inv_hhpa = inv_h * inv_hpa;
    real com = 0;

    if (alpha) {
        com = inv_hhpa * compliance;
    }
    if (num_rigid_contacts > 0) {
        if (mu == 0) {
#pragma omp parallel for
            for (int index = 0; index < (signed)num_rigid_contacts; index++) {
                E[start_contact + index + 0] = com;
            }
        } else {
#pragma omp parallel for
            for (int index = 0; index < (signed)num_rigid_contacts; index++) {
                E[start_contact + index + 0] = com;
                E[start_contact + num_rigid_contacts + index * 2 + 0] = 0;
                E[start_contact + num_rigid_contacts + index * 2 + 1] = 0;
            }
        }
    }
}

void ChParticleContainer::Project(real* gamma) {
    ProjectRigidFluidBoundary(contact_mu, contact_cohesion, num_fluid_bodies, start_boundary, gamma, data_manager);

    if (mu == 0) {
#pragma omp parallel for
        for (int index = 0; index < (signed)num_rigid_contacts; index++) {
            real3 gam;
            gam.x = gamma[start_contact + index];
            gam.x += cohesion;
            gam.x = gam.x < 0 ? 0 : gam.x - cohesion;
            gamma[start_contact + index] = gam.x;
        }
    } else {
#pragma omp parallel for
        for (int index = 0; index < (signed)num_rigid_contacts; index++) {
            real3 gam;
            gam.x = gamma[start_contact + index];
            gam.y = gamma[start_contact + num_rigid_contacts + index * 2 + 0];
            gam.z = gamma[start_contact + num_rigid_contacts + index * 2 + 1];

            gam.x += cohesion;

            Cone_generalized_rigid(gam.x, gam.y, gam.z, mu);

            gamma[start_contact + index] = gam.x - cohesion;
            gamma[start_contact + num_rigid_contacts + index * 2 + 0] = gam.y;
            gamma[start_contact + num_rigid_contacts + index * 2 + 1] = gam.z;
        }
    }
}

void ChParticleContainer::GenerateSparsity() {
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    LOG(INFO) << "ChParticleContainer::GenerateSparsity";
    AppendRigidFluidBoundary(contact_mu, num_fluid_bodies, body_offset, start_boundary, data_manager);

    if (num_rigid_contacts > 0) {
        int index_n = 0;
        int index_t = 0;
        for (int body_a = 0; body_a < (signed)num_fluid_bodies; body_a++) {
            for (int i = 0; i < data_manager->host_data.c_counts_3dof_3dof[body_a]; i++) {
                int body_b = data_manager->host_data.neighbor_3dof_3dof[body_a * max_neighbors + i];
                if (body_a == body_b || body_a > body_b) {
                    continue;
                }

                AppendRow3(D_T, start_contact + index_n + 0, body_offset + body_a * 3, 0);
                AppendRow3(D_T, start_contact + index_n + 0, body_offset + body_b * 3, 0);

                D_T.finalize(start_contact + index_n + 0);
                index_n++;
            }
        }
        if (mu != 0) {
            for (int body_a = 0; body_a < (signed)num_fluid_bodies; body_a++) {
                for (int i = 0; i < data_manager->host_data.c_counts_3dof_3dof[body_a]; i++) {
                    int body_b = data_manager->host_data.neighbor_3dof_3dof[body_a * max_neighbors + i];
                    if (body_a == body_b || body_a > body_b) {
                        continue;
                    }

                    AppendRow3(D_T, start_contact + num_rigid_contacts + index_t * 2 + 0, body_offset + body_a * 3, 0);
                    AppendRow3(D_T, start_contact + num_rigid_contacts + index_t * 2 + 0, body_offset + body_b * 3, 0);

                    D_T.finalize(start_contact + num_rigid_contacts + index_t * 2 + 0);

                    AppendRow3(D_T, start_contact + num_rigid_contacts + index_t * 2 + 1, body_offset + body_a * 3, 0);
                    AppendRow3(D_T, start_contact + num_rigid_contacts + index_t * 2 + 1, body_offset + body_b * 3, 0);

                    D_T.finalize(start_contact + num_rigid_contacts + index_t * 2 + 1);

                    index_t++;
                }
            }
        }
    }
}

void ChParticleContainer::CalculateContactForces() {
    uint num_contacts = data_manager->num_rigid_fluid_contacts;
    if (num_contacts <= 0) {
        return;
    }
    LOG(INFO) << "ChParticleContainer::CalculateContactForces() ";

    DynamicVector<real>& gamma = data_manager->host_data.gamma;
    SubVectorType gamma_n = subvector(gamma, start_boundary, _num_rf_c_);

    contact_forces = submatrix(data_manager->host_data.D, 0, start_boundary, _num_dof_, _num_rf_c_) * gamma_n /
                     data_manager->settings.step_size;

    if (contact_mu != 0) {
        SubVectorType gamma_t = subvector(gamma, start_boundary + _num_rf_c_, 2 * _num_rf_c_);
        contact_forces +=
            submatrix(data_manager->host_data.D, 0, start_boundary + _num_rf_c_, _num_dof_, 2 * _num_rf_c_) * gamma_t /
            data_manager->settings.step_size;
    }
}

real3 ChParticleContainer::GetBodyContactForce(uint body_id) {
    if (data_manager->num_rigid_fluid_contacts <= 0) {
        return real3(0);
    }
    return real3(contact_forces[body_id * 6 + 0], contact_forces[body_id * 6 + 1], contact_forces[body_id * 6 + 2]);
}

real3 ChParticleContainer::GetBodyContactTorque(uint body_id) {
    if (data_manager->num_rigid_fluid_contacts <= 0) {
        return real3(0);
    }
    return real3(contact_forces[body_id * 6 + 3], contact_forces[body_id * 6 + 4], contact_forces[body_id * 6 + 5]);
}

void ChParticleContainer::PreSolve() {
#ifdef CHRONO_PARALLEL_USE_CUDA
    if (mpm_thread.joinable()) {
        mpm_thread.join();
#pragma omp parallel for
        for (int p = 0; p < (signed)num_fluid_bodies; p++) {
            int index = data_manager->host_data.reverse_mapping_3dof[p];
            data_manager->host_data.v[body_offset + index * 3 + 0] = mpm_vel[p * 3 + 0];
            data_manager->host_data.v[body_offset + index * 3 + 1] = mpm_vel[p * 3 + 1];
            data_manager->host_data.v[body_offset + index * 3 + 2] = mpm_vel[p * 3 + 2];
        }
    }
#endif
}

void ChParticleContainer::PostSolve() {}

void ChParticleContainer::GetFluidForce(custom_vector<real3>& forc) {
    forc.resize(num_fluid_bodies);

    DynamicVector<real>& gamma = data_manager->host_data.gamma;

    SubVectorType gamma_n = subvector(gamma, start_contact, num_rigid_contacts);

    DynamicVector<real> pressure_forces =
        submatrix(data_manager->host_data.D, body_offset, start_contact, num_fluid_bodies * 3, num_rigid_contacts) *
        gamma_n / data_manager->settings.step_size;

    for (int i = 0; i < (signed)num_fluid_bodies; i++) {
        forc[i] = real3(pressure_forces[i * 3 + 0], pressure_forces[i * 3 + 1], pressure_forces[i * 3 + 2]);
    }
}

}  // end namespace chrono
