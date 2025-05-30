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

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/physics/Ch3DOFContainer.h"
#include "chrono_multicore/ChDataManager.h"
#include "chrono_multicore/constraints/ChConstraintUtils.h"

#include "chrono/multicore_math/thrust.h"
#include "chrono/multicore_math/matrix.h"

namespace chrono {

ChParticleContainer::ChParticleContainer() {
    body_offset = 0;
    compliance = 0;
    mu = 0;
    cohesion = 0;
    mass = 0.037037;
    start_boundary = 0;
    start_contact = 0;

    nu = .2;
    youngs_modulus = 1.4e5;
    hardening_coefficient = 10;
    lame_lambda = youngs_modulus * nu / ((1. + nu) * (1. - 2. * nu));
    lame_mu = youngs_modulus / (2. * (1. + nu));
    theta_s = 7.5e-3;
    theta_c = 2.5e-2;
    alpha_flip = .95;
}

void ChParticleContainer::AddBodies(const std::vector<real3>& positions, const std::vector<real3>& velocities) {
    custom_vector<real3>& pos_3dof = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_3dof = data_manager->host_data.vel_3dof;

    pos_3dof.insert(pos_3dof.end(), positions.begin(), positions.end());
    vel_3dof.insert(vel_3dof.end(), velocities.begin(), velocities.end());
    // In case the number of velocities provided were not enough, resize to the number of particle velocities
    vel_3dof.resize(pos_3dof.size());
    data_manager->num_particles = (uint)pos_3dof.size();
}
void ChParticleContainer::Update3DOF(double time) {
    num_particles = data_manager->num_particles;
    num_rigid_bodies = data_manager->num_rigid_bodies;
    num_shafts = data_manager->num_shafts;
    num_motors = data_manager->num_motors;
    real3 h_gravity = data_manager->settings.step_size * mass * data_manager->settings.gravity;

    uint offset = num_rigid_bodies * 6 + num_shafts + num_motors;
#pragma omp parallel for
    for (int i = 0; i < (signed)num_particles; i++) {
        data_manager->host_data.hf[offset + i * 3 + 0] = h_gravity.x;
        data_manager->host_data.hf[offset + i * 3 + 1] = h_gravity.y;
        data_manager->host_data.hf[offset + i * 3 + 2] = h_gravity.z;
    }
}

void ChParticleContainer::UpdatePosition(double time) {
    num_particles = data_manager->num_particles;
    num_rigid_bodies = data_manager->num_rigid_bodies;
    num_shafts = data_manager->num_shafts;
    num_motors = data_manager->num_motors;

    custom_vector<real3>& pos_3dof = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_3dof = data_manager->host_data.vel_3dof;

    uint offset = num_rigid_bodies * 6 + num_shafts + num_motors;
#pragma omp parallel for
    for (int i = 0; i < (signed)num_particles; i++) {
        real3 vel;
        int original_index = data_manager->cd_data->particle_indices_3dof[i];
        // these are sorted so we have to unsort them
        vel.x = data_manager->host_data.v[offset + i * 3 + 0];
        vel.y = data_manager->host_data.v[offset + i * 3 + 1];
        vel.z = data_manager->host_data.v[offset + i * 3 + 2];

        real speed = Length(vel);
        if (speed > max_velocity) {
            vel = vel * max_velocity / speed;
        }
        vel_3dof[original_index] = vel;
        pos_3dof[original_index] += vel * data_manager->settings.step_size;
    }
}

unsigned int ChParticleContainer::GetNumConstraints() {
    num_particle_contacts = data_manager->cd_data->num_particle_contacts;
    int num_particle_particle = 0;
    if (mu == 0) {
        num_particle_particle = (num_particle_contacts - data_manager->num_particles) / 2;
    } else {
        num_particle_particle = (num_particle_contacts - data_manager->num_particles) / 2 * 3;
    }

    if (contact_mu == 0) {
        num_particle_particle += data_manager->cd_data->num_rigid_particle_contacts;
    } else {
        num_particle_particle += data_manager->cd_data->num_rigid_particle_contacts * 3;
    }

    return num_particle_particle;
}

unsigned int ChParticleContainer::GetNumNonZeros() {
    num_particle_contacts = data_manager->cd_data->num_particle_contacts;
    int nnz_particle_particle = 0;
    if (mu == 0) {
        nnz_particle_particle = (num_particle_contacts - data_manager->num_particles) / 2 * 6;

    } else {
        nnz_particle_particle = (num_particle_contacts - data_manager->num_particles) / 2 * 6 * 3;
    }

    if (contact_mu == 0) {
        nnz_particle_particle += 9 * data_manager->cd_data->num_rigid_particle_contacts;
    } else {
        nnz_particle_particle += 9 * 3 * data_manager->cd_data->num_rigid_particle_contacts;
    }

    return nnz_particle_particle;
}

void ChParticleContainer::ComputeInvMass(int offset) {
    num_particles = data_manager->num_particles;
    CompressedMatrix<real>& M_inv = data_manager->host_data.M_inv;

    real inv_mass = 1.0 / mass;
    for (uint i = 0; i < num_particles; i++) {
        M_inv.append(offset + i * 3 + 0, offset + i * 3 + 0, inv_mass);
        M_inv.finalize(offset + i * 3 + 0);
        M_inv.append(offset + i * 3 + 1, offset + i * 3 + 1, inv_mass);
        M_inv.finalize(offset + i * 3 + 1);
        M_inv.append(offset + i * 3 + 2, offset + i * 3 + 2, inv_mass);
        M_inv.finalize(offset + i * 3 + 2);
    }
}

void ChParticleContainer::ComputeMass(int offset) {
    num_particles = data_manager->num_particles;
    CompressedMatrix<real>& M = data_manager->host_data.M;

    for (uint i = 0; i < num_particles; i++) {
        M.append(offset + i * 3 + 0, offset + i * 3 + 0, mass);
        M.finalize(offset + i * 3 + 0);
        M.append(offset + i * 3 + 1, offset + i * 3 + 1, mass);
        M.finalize(offset + i * 3 + 1);
        M.append(offset + i * 3 + 2, offset + i * 3 + 2, mass);
        M.finalize(offset + i * 3 + 2);
    }
}

void ChParticleContainer::Setup3DOF(int start_constraint) {
    Ch3DOFContainer::Setup3DOF(start_constraint);

    start_boundary = start_constraint;
    if (contact_mu == 0) {
        start_contact = start_constraint + num_rigid_particle_contacts;
    } else {
        start_contact = start_constraint + num_rigid_particle_contacts * 3;
    }
    body_offset = num_rigid_bodies * 6 + num_shafts + num_motors;

    num_rigid_contacts = (num_particle_contacts - num_particles) / 2;
}

void ChParticleContainer::Initialize() {
    CreateVisualization(0.5 * kernel_radius, ChColor(0.65f, 0.40f, 0.10f));
    Ch3DOFContainer::Initialize();
}

void ChParticleContainer::Build_D() {
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    BuildRigidParticleBoundary(contact_mu, num_particles, body_offset, start_boundary, data_manager);

    if (num_rigid_contacts > 0) {
        int index = 0;
        custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;

        if (mu == 0) {
            Loop_Over_Particle_Neighbors(                                                       //
                real3 U = -Normalize(xij); real3 V; real3 W;                                 //
                Orthogonalize(U, V, W);                                                      //
                SetRow3Check(D_T, start_contact + index + 0, body_offset + body_a * 3, -U);  //
                SetRow3Check(D_T, start_contact + index + 0, body_offset + body_b * 3, U);   //
            );

        } else {
            Loop_Over_Particle_Neighbors(
                real3 U = -Normalize(xij); real3 V; real3 W; Orthogonalize(U, V, W);

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
}

void ChParticleContainer::Build_b() {
    real inv_hpa = 1.0 / (data_manager->settings.step_size + alpha);

    DynamicVector<real>& b = data_manager->host_data.b;

    CorrectionRigidParticleBoundary(contact_mu, contact_cohesion, alpha, contact_recovery_speed, num_particles,
                                 start_boundary, data_manager);

    if (num_rigid_contacts > 0) {
        int index = 0;  // incremented in Loop_Over_Particle_Neighbors()

        custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;

        if (mu == 0) {
            Loop_Over_Particle_Neighbors(real depth = Length(xij) - kernel_radius;                 //
                                      real bi = 0;                                              //
                                      if (cohesion != 0) { depth = Min(depth, 0); }             //
                                      bi = std::max(inv_hpa * depth, -contact_recovery_speed);  //
                                      b[start_contact + index + 0] = bi;                        //
            );
        } else {
            Loop_Over_Particle_Neighbors(real depth = Length(xij) - kernel_radius;                   //
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

    ComplianceRigidParticleBoundary(contact_mu, contact_compliance, alpha, start_boundary, data_manager);
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
    ProjectRigidParticleBoundary(contact_mu, contact_cohesion, num_particles, start_boundary, gamma, data_manager);

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
    AppendRigidParticleBoundary(contact_mu, num_particles, body_offset, start_boundary, data_manager);

    if (num_rigid_contacts > 0) {
        int index_n = 0;
        int index_t = 0;
        for (int body_a = 0; body_a < (signed)num_particles; body_a++) {
            for (int i = 0; i < data_manager->cd_data->c_counts_3dof_3dof[body_a]; i++) {
                int body_b = data_manager->cd_data->neighbor_3dof_3dof[body_a * ChNarrowphase::max_neighbors + i];
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
            for (int body_a = 0; body_a < (signed)num_particles; body_a++) {
                for (int i = 0; i < data_manager->cd_data->c_counts_3dof_3dof[body_a]; i++) {
                    int body_b = data_manager->cd_data->neighbor_3dof_3dof[body_a * ChNarrowphase::max_neighbors + i];
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
    if (data_manager->cd_data->num_rigid_particle_contacts <= 0) {
        return;
    }

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

real3 ChParticleContainer::GetBodyContactForce(std::shared_ptr<ChBody> body) {
    if (data_manager->cd_data->num_rigid_particle_contacts <= 0) {
        return real3(0);
    }
    auto body_id = body->GetIndex();
    return real3(contact_forces[body_id * 6 + 0], contact_forces[body_id * 6 + 1], contact_forces[body_id * 6 + 2]);
}

real3 ChParticleContainer::GetBodyContactTorque(std::shared_ptr<ChBody> body) {
    if (data_manager->cd_data->num_rigid_particle_contacts <= 0) {
        return real3(0);
    }
    auto body_id = body->GetIndex();
    return real3(contact_forces[body_id * 6 + 3], contact_forces[body_id * 6 + 4], contact_forces[body_id * 6 + 5]);
}

void ChParticleContainer::PreSolve() {}

void ChParticleContainer::PostSolve() {}

void ChParticleContainer::GetPressureForce(custom_vector<real3>& forc) {
    forc.resize(num_particles);

    DynamicVector<real>& gamma = data_manager->host_data.gamma;

    SubVectorType gamma_n = subvector(gamma, start_contact, num_rigid_contacts);

    DynamicVector<real> pressure_forces =
        submatrix(data_manager->host_data.D, body_offset, start_contact, num_particles * 3, num_rigid_contacts) *
        gamma_n / data_manager->settings.step_size;

    for (int i = 0; i < (signed)num_particles; i++) {
        forc[i] = real3(pressure_forces[i * 3 + 0], pressure_forces[i * 3 + 1], pressure_forces[i * 3 + 2]);
    }
}

}  // end namespace chrono
