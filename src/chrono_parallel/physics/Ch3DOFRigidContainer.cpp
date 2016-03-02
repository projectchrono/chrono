
#include <algorithm>
#include <math.h>
#include "chrono_parallel/physics/ChSystemParallel.h"
#include <chrono_parallel/physics/Ch3DOFContainer.h>
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/constraints/ChConstraintUtils.h"

#include "chrono_parallel/math/other_types.h"  // for uint, int2, int3
#include "chrono_parallel/math/real.h"         // for real
#include "chrono_parallel/math/real2.h"        // for real2
#include "chrono_parallel/math/real3.h"        // for real3
#include "chrono_parallel/math/real4.h"        // for quaternion, real4
#include "chrono_parallel/math/matrix.h"        // for quaternion, real4

namespace chrono {

using namespace collision;
using namespace geometry;

Ch3DOFRigidContainer::Ch3DOFRigidContainer(ChSystemParallelDVI* physics_system) {
    data_manager = physics_system->data_manager;
    data_manager->Add3DOFContainer(this);
    body_offset = 0;
    compliance = 0;
    mu = 0;
    cohesion = 0;
    mass = 0.037037;
    start_boundary = 0;
    start_contact = 0;
}
Ch3DOFRigidContainer::~Ch3DOFRigidContainer() {}

void Ch3DOFRigidContainer::AddBodies(const std::vector<real3>& positions, const std::vector<real3>& velocities) {
    custom_vector<real3>& pos_fluid = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_fluid = data_manager->host_data.vel_3dof;

    pos_fluid.insert(pos_fluid.end(), positions.begin(), positions.end());
    vel_fluid.insert(vel_fluid.end(), velocities.begin(), velocities.end());
    // In case the number of velocities provided were not enough, resize to the number of fluid bodies
    vel_fluid.resize(pos_fluid.size());
    data_manager->num_fluid_bodies = pos_fluid.size();
}
void Ch3DOFRigidContainer::Update(double ChTime) {
    uint num_fluid_bodies = data_manager->num_fluid_bodies;
    uint num_rigid_bodies = data_manager->num_rigid_bodies;
    uint num_shafts = data_manager->num_shafts;
    real3 h_gravity = data_manager->settings.step_size * mass * data_manager->settings.gravity;
    for (int i = 0; i < num_fluid_bodies; i++) {
        // This was moved to after fluid collision detection
        // real3 vel = vel_fluid[i];
        // data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 0] = vel.x;
        // data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 1] = vel.y;
        // data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 2] = vel.z;

        data_manager->host_data.hf[num_rigid_bodies * 6 + num_shafts + i * 3 + 0] = h_gravity.x;
        data_manager->host_data.hf[num_rigid_bodies * 6 + num_shafts + i * 3 + 1] = h_gravity.y;
        data_manager->host_data.hf[num_rigid_bodies * 6 + num_shafts + i * 3 + 2] = h_gravity.z;
    }
}

void Ch3DOFRigidContainer::UpdatePosition(double ChTime) {
    uint num_fluid_bodies = data_manager->num_fluid_bodies;
    uint num_rigid_bodies = data_manager->num_rigid_bodies;
    uint num_shafts = data_manager->num_shafts;

    custom_vector<real3>& pos_fluid = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_fluid = data_manager->host_data.vel_3dof;
#pragma omp parallel for
    for (int i = 0; i < num_fluid_bodies; i++) {
        real3 vel;
        int original_index = data_manager->host_data.particle_indices_3dof[i];
        // these are sorted so we have to unsort them
        vel.x = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 0];
        vel.y = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 1];
        vel.z = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 2];

        real speed = Length(vel);
        if (speed > max_velocity) {
            vel = vel * max_velocity / speed;
        }
        vel_fluid[original_index] = vel;
        pos_fluid[original_index] += vel * data_manager->settings.step_size;
    }
}

int Ch3DOFRigidContainer::GetNumConstraints() {
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
int Ch3DOFRigidContainer::GetNumNonZeros() {
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

void Ch3DOFRigidContainer::ComputeInvMass(int offset) {
    CompressedMatrix<real>& M_inv = data_manager->host_data.M_inv;
    uint num_fluid_bodies = data_manager->num_fluid_bodies;

    real inv_mass = 1.0 / mass;
    for (int i = 0; i < num_fluid_bodies; i++) {
        M_inv.append(offset + i * 3 + 0, offset + i * 3 + 0, inv_mass);
        M_inv.finalize(offset + i * 3 + 0);
        M_inv.append(offset + i * 3 + 1, offset + i * 3 + 1, inv_mass);
        M_inv.finalize(offset + i * 3 + 1);
        M_inv.append(offset + i * 3 + 2, offset + i * 3 + 2, inv_mass);
        M_inv.finalize(offset + i * 3 + 2);
    }
}
void Ch3DOFRigidContainer::ComputeMass(int offset) {
    CompressedMatrix<real>& M = data_manager->host_data.M;
    uint num_fluid_bodies = data_manager->num_fluid_bodies;

    real fluid_mass = mass;
    for (int i = 0; i < num_fluid_bodies; i++) {
        M.append(offset + i * 3 + 0, offset + i * 3 + 0, fluid_mass);
        M.finalize(offset + i * 3 + 0);
        M.append(offset + i * 3 + 1, offset + i * 3 + 1, fluid_mass);
        M.finalize(offset + i * 3 + 1);
        M.append(offset + i * 3 + 2, offset + i * 3 + 2, fluid_mass);
        M.finalize(offset + i * 3 + 2);
    }
}
void Ch3DOFRigidContainer::Setup(int start_constraint) {
    Ch3DOFContainer::Setup(start_constraint);

    start_boundary = start_constraint;
    if (contact_mu == 0) {
        start_contact = start_constraint + num_rigid_fluid_contacts;
    } else {
        start_contact = start_constraint + num_rigid_fluid_contacts * 3;
    }
    body_offset = num_rigid_bodies * 6 + num_shafts;

    num_rigid_contacts = (num_fluid_contacts - num_fluid_bodies) / 2;
}

void Ch3DOFRigidContainer::Initialize() {
    printf("Computed mass: %f\n", mass);
}

void Ch3DOFRigidContainer::Build_D() {
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

    LOG(INFO) << "ChConstraintRigidFluid::Build_D_Fluid"
              << " " << D_T.rows() << " " << D_T.columns();
    if (num_rigid_fluid_contacts > 0) {
        custom_vector<real3>& pos_rigid = data_manager->host_data.pos_rigid;
        custom_vector<quaternion>& rot_rigid = data_manager->host_data.rot_rigid;

        // custom_vector<int2>& bids = data_manager->host_data.bids_rigid_fluid;
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

void Ch3DOFRigidContainer::Build_b() {
    real dt = data_manager->settings.step_size;
    DynamicVector<real>& b = data_manager->host_data.b;
    if (num_rigid_fluid_contacts > 0) {
        custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
        custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;

        if (contact_mu == 0) {
#pragma omp parallel for
            Loop_Over_Rigid_Neighbors(
                real depth = data_manager->host_data.dpth_rigid_fluid[p * max_rigid_neighbors + i];  //

                real bi = 0;  //
                if (contact_cohesion) { depth = Min(depth, 0); } else if (depth > 0) { bi = 0; } else {
                    real bi = std::max(real(1.0) / dt * depth, -contact_recovery_speed);
                }  //
                b[start_boundary + index + 0] = bi;);
        } else {
#pragma omp parallel for
            Loop_Over_Rigid_Neighbors(
                real depth = data_manager->host_data.dpth_rigid_fluid[p * max_rigid_neighbors + i]; real bi = 0;  //
                if (contact_cohesion) { depth = Min(depth, 0); } else if (depth > 0) { bi = 0; } else {
                    real bi = std::max(real(1.0) / dt * depth, -contact_recovery_speed);
                }  //
                b[start_boundary + index + 0] = bi;
                b[start_boundary + num_rigid_fluid_contacts + index * 2 + 0] = 0;
                b[start_boundary + num_rigid_fluid_contacts + index * 2 + 1] = 0;);
        }
    }
    if (num_rigid_contacts > 0) {
        int index = 0;
        custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;

        if (mu == 0) {
            Loop_Over_Fluid_Neighbors(real depth = Length(xij) - kernel_radius;  //
                                      real bi = 0;                               //
                                      if (cohesion) { depth = Min(depth, 0); } else if (depth > 0) { bi = 0; } else {
                                          real bi = std::max(real(1.0) / dt * depth, -contact_recovery_speed);
                                      }

                                      b[start_contact + index + 0] = bi;);
        } else {
            Loop_Over_Fluid_Neighbors(real depth = Length(xij) - kernel_radius;  //
                                      real bi = 0;                               //
                                      if (cohesion) { depth = Min(depth, 0); } else if (depth > 0) { bi = 0; } else {
                                          real bi = std::max(real(1.0) / dt * depth, -contact_recovery_speed);
                                      }                                                           //
                                      b[start_contact + index + 0] = bi;                          //
                                      b[start_contact + num_rigid_contacts + index * 2 + 0] = 0;  //
                                      b[start_contact + num_rigid_contacts + index * 2 + 1] = 0;);
        }
    }
}
void Ch3DOFRigidContainer::Build_E() {
    DynamicVector<real>& E = data_manager->host_data.E;

    if (num_rigid_fluid_contacts > 0) {
        if (contact_mu == 0) {
#pragma omp parallel for
            for (int index = 0; index < num_rigid_fluid_contacts; index++) {
                E[start_boundary + index + 0] = 0;
            }
        } else {
#pragma omp parallel for
            for (int index = 0; index < num_rigid_fluid_contacts; index++) {
                E[start_boundary + index + 0] = 0;
                E[start_boundary + num_rigid_fluid_contacts + index * 2 + 0] = 0;
                E[start_boundary + num_rigid_fluid_contacts + index * 2 + 1] = 0;
            }
        }
    }

    if (num_rigid_contacts > 0) {
        if (mu == 0) {
#pragma omp parallel for
            for (int index = 0; index < num_rigid_contacts; index++) {
                E[start_contact + index + 0] = 0;
            }
        } else {
#pragma omp parallel for
            for (int index = 0; index < num_rigid_contacts; index++) {
                E[start_contact + index + 0] = 0;
                E[start_contact + num_rigid_contacts + index * 2 + 0] = 0;
                E[start_contact + num_rigid_contacts + index * 2 + 1] = 0;
            }
        }
    }
}

void Ch3DOFRigidContainer::Project(real* gamma) {
    custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
    custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;

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
    if (mu == 0) {
#pragma omp parallel for
        for (int index = 0; index < num_rigid_contacts; index++) {
            real3 gam;
            gam.x = gamma[start_contact + index];
            gam.x += cohesion;
            gam.x = gam.x < 0 ? 0 : gam.x - cohesion;
            gamma[start_contact + index] = gam.x;
        }
    } else {
#pragma omp parallel for
        for (int index = 0; index < num_rigid_contacts; index++) {
            real3 gam;
            gam.x = gamma[start_contact + index];
            gam.y = gamma[start_contact + num_rigid_contacts + index * 2 + 0];
            gam.z = gamma[start_contact + num_rigid_contacts + index * 2 + 1];

            gam.x += cohesion;

            if (mu == 0) {
                gam.x = gam.x < 0 ? 0 : gam.x - cohesion;
                gam.y = gam.z = 0;

                gamma[start_contact + index] = gam.x;
                gamma[start_contact + num_rigid_contacts + index * 2 + 0] = gam.y;
                gamma[start_contact + num_rigid_contacts + index * 2 + 1] = gam.z;

                continue;
            }

            if (Cone_generalized_rigid(gam.x, gam.y, gam.z, mu)) {
            }

            gamma[start_contact + index] = gam.x - cohesion;
            gamma[start_contact + num_rigid_contacts + index * 2 + 0] = gam.y;
            gamma[start_contact + num_rigid_contacts + index * 2 + 1] = gam.z;
        }
    }
}
void Ch3DOFRigidContainer::GenerateSparsity() {
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

    if (num_rigid_fluid_contacts > 0) {
        LOG(INFO) << "ChConstraintRigidFluid::GenerateSparsity";

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

    if (num_rigid_contacts > 0) {
        int index_n = 0;
        int index_t = 0;
        for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
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
            for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
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

void Ch3DOFRigidContainer::PostSolve() {}

void Ch3DOFRigidContainer::CalculateContactForces() {
    uint num_contacts = data_manager->num_rigid_fluid_contacts;
    if (num_contacts <= 0) {
        return;
    }

    DynamicVector<real>& gamma = data_manager->host_data.gamma;
    SubVectorType gamma_n = subvector(gamma, start_boundary, _num_rf_c_);
    SubVectorType gamma_t = subvector(gamma, start_boundary + _num_rf_c_, 2 * _num_rf_c_);

    contact_forces = submatrix(data_manager->host_data.D, 0, start_boundary, _num_dof_, _num_rf_c_) * gamma_n /
                     data_manager->settings.step_size;

    if (contact_mu != 0) {
        contact_forces =
            contact_forces +
            submatrix(data_manager->host_data.D, 0, start_boundary + _num_rf_c_, _num_dof_, 2 * _num_rf_c_) * gamma_t /
                data_manager->settings.step_size;
    }
}

real3 Ch3DOFRigidContainer::GetBodyContactForce(uint body_id) {
    if (data_manager->num_rigid_fluid_contacts <= 0) {
        return real3(0);
    }
    return real3(contact_forces[body_id * 6 + 0], contact_forces[body_id * 6 + 1], contact_forces[body_id * 6 + 2]);
}

real3 Ch3DOFRigidContainer::GetBodyContactTorque(uint body_id) {
    if (data_manager->num_rigid_fluid_contacts <= 0) {
        return real3(0);
    }
    return real3(contact_forces[body_id * 6 + 3], contact_forces[body_id * 6 + 4], contact_forces[body_id * 6 + 5]);
}

void Ch3DOFRigidContainer::PreSolve() {
    if (max_iterations == 0) {
        return;
    }
}
}  // END_OF_NAMESPACE____

/////////////////////
