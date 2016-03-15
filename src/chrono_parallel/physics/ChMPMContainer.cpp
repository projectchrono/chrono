
#include <algorithm>
#include "chrono_parallel/physics/ChSystemParallel.h"
#include <chrono_parallel/physics/Ch3DOFContainer.h>
#include "chrono_parallel/physics/MPMUtils.h"
#include <chrono_parallel/collision/ChBroadphaseUtils.h>
#include <thrust/transform_reduce.h>
#include <thrust/sort.h>
#include <thrust/count.h>
#include <thrust/iterator/constant_iterator.h>
#include "chrono_parallel/constraints/ChConstraintUtils.h"
#include "chrono_parallel/solver/ChSolverParallel.h"
#include "chrono_parallel/math/other_types.h"  // for uint, int2, vec3
#include "chrono_parallel/math/real.h"         // for real
#include "chrono_parallel/math/real3.h"        // for real3
#include "chrono_parallel/math/matrix.h"       // for quaternion, real4
#include "chrono_parallel/physics/ChMPM.cuh"
#include "chrono_parallel/collision/ChCollision.h"
#include "chrono_parallel/constraints/ChConstraintFluidFluidUtils.h"

namespace chrono {

using namespace collision;
using namespace geometry;

#define USE_FLUID 0
//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR A 3DOF FLUID NODE
//

ChMPMContainer::ChMPMContainer(ChSystemParallelDVI* physics_system) {
    data_manager = physics_system->data_manager;
    data_manager->AddMPMContainer(this);
    num_mpm_markers = 0;
    num_mpm_nodes = 0;
    max_iterations = 10;
    real mass = 1;
    real mu = 1;
    real hardening_coefficient = 1;
    real lambda = 1;
    real theta_s = 1;
    real theta_c = 1;
    real alpha_flip = 1;
    cohesion = 0;
}
ChMPMContainer::~ChMPMContainer() {}

void ChMPMContainer::Setup(int start_constraint) {
    Ch3DOFContainer::Setup(start_constraint);
    body_offset = num_rigid_bodies * 6 + num_shafts;
    min_bounding_point = data_manager->measures.collision.mpm_min_bounding_point;
    max_bounding_point = data_manager->measures.collision.mpm_max_bounding_point;
    num_mpm_contacts = (num_fluid_contacts - num_fluid_bodies) / 2;
    start_boundary = start_constraint;

    if (contact_mu == 0) {
        start_contact = start_constraint + num_rigid_fluid_contacts;
    } else {
        start_contact = start_constraint + num_rigid_fluid_contacts * 3;
    }
}

void ChMPMContainer::AddNodes(const std::vector<real3>& positions, const std::vector<real3>& velocities) {
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_3dof;

    pos_marker.insert(pos_marker.end(), positions.begin(), positions.end());
    vel_marker.insert(vel_marker.end(), velocities.begin(), velocities.end());
    // In case the number of velocities provided were not enough, resize to the number of fluid bodies
    vel_marker.resize(pos_marker.size());
    data_manager->num_fluid_bodies = pos_marker.size();
    num_mpm_markers = pos_marker.size();
}
void ChMPMContainer::ComputeDOF() {}

void ChMPMContainer::Update(double ChTime) {
    Setup(0);
    ComputeDOF();

    real3 g_acc = data_manager->settings.gravity;
    real3 h_gravity = data_manager->settings.step_size * mass * g_acc;
#pragma omp parallel for
    for (int i = 0; i < num_mpm_markers; i++) {
        // Velocity already set in the fluid fluid contact function
        data_manager->host_data.hf[body_offset + i * 3 + 0] = h_gravity.x;
        data_manager->host_data.hf[body_offset + i * 3 + 1] = h_gravity.y;
        data_manager->host_data.hf[body_offset + i * 3 + 2] = h_gravity.z;
    }
}

void ChMPMContainer::UpdatePosition(double ChTime) {
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_3dof;
    custom_vector<real3>& sorted_pos_fluid = data_manager->host_data.sorted_pos_3dof;

    printf("Update_Particle_Velocities\n");

#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        real3 new_vel;
        int original_index = data_manager->host_data.particle_indices_3dof[p];
        new_vel.x = data_manager->host_data.v[body_offset + p * 3 + 0];
        new_vel.y = data_manager->host_data.v[body_offset + p * 3 + 1];
        new_vel.z = data_manager->host_data.v[body_offset + p * 3 + 2];

        real speed = Length(new_vel);
        if (speed > max_velocity) {
            new_vel = new_vel * max_velocity / speed;
        }

        vel_marker[original_index] = new_vel;
        pos_marker[original_index] += new_vel * data_manager->settings.step_size;
        sorted_pos_fluid[p] = pos_marker[original_index];
    }
#if 1
    custom_vector<real3> new_pos = sorted_pos_fluid;

    if (num_mpm_markers != 0) {
        data_manager->narrowphase->DispatchRigidFluid();

        custom_vector<real3>& cpta = data_manager->host_data.cpta_rigid_fluid;
        custom_vector<real3>& norm = data_manager->host_data.norm_rigid_fluid;
        custom_vector<real>& dpth = data_manager->host_data.dpth_rigid_fluid;
        custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
        custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;
        // This treats all rigid neighbors as fixed. This correction should usually be pretty small if the timestep
        // isnt too large.

        if (data_manager->num_rigid_fluid_contacts > 0) {
#pragma omp parallel for
            for (int p = 0; p < num_fluid_bodies; p++) {
                int start = contact_counts[p];
                int end = contact_counts[p + 1];
                real3 delta = real3(0);
                real weight = 0;
                for (int index = start; index < end; index++) {
                    int i = index - start;
                    // int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];
                    // if (data_manager->host_data.active_rigid[rigid] == false) {
                    real3 U = norm[p * max_rigid_neighbors + i];
                    real depth = dpth[p * max_rigid_neighbors + i];
                    if (depth < 0) {
                        real w = 1.0;  // mass / (mass + data_manager->host_data.mass_rigid[rigid]);
                        delta -= w * depth * U;
                        weight++;
                    }
                    //}
                }
                if (weight > 0) {
                    new_pos[p] = new_pos[p] + delta / weight;
                }
            }
            real inv_dt = 1.0 / data_manager->settings.step_size;
#pragma omp parallel for
            for (int p = 0; p < num_fluid_bodies; p++) {
                int original_index = data_manager->host_data.particle_indices_3dof[p];
                real3 vv = real3((new_pos[p] - sorted_pos_fluid[p]) * inv_dt);
                if (contact_counts[p + 1] - contact_counts[p] > 0) {
                    // vel_marker[original_index] = vv;
                    pos_marker[original_index] = new_pos[p];
                }
            }
        }
    }
#endif
}

int ChMPMContainer::GetNumConstraints() {
#if USE_FLUID
    int num_fluid_fluid = data_manager->num_fluid_bodies;
#else
    int num_fluid_fluid = (data_manager->num_fluid_contacts - data_manager->num_fluid_bodies) / 2;
#endif
    if (contact_mu == 0) {
        num_fluid_fluid += data_manager->num_rigid_fluid_contacts;
    } else {
        num_fluid_fluid += data_manager->num_rigid_fluid_contacts * 3;
    }
    return num_fluid_fluid;
}

int ChMPMContainer::GetNumNonZeros() {
    int nnz_fluid_fluid = 0;

#if USE_FLUID
    nnz_fluid_fluid = data_manager->num_fluid_bodies * 6 * max_neighbors;
#else
    nnz_fluid_fluid = (data_manager->num_fluid_contacts - data_manager->num_fluid_bodies) / 2 * 6;
#endif

    if (contact_mu == 0) {
        nnz_fluid_fluid += 9 * data_manager->num_rigid_fluid_contacts;
    } else {
        nnz_fluid_fluid += 9 * 3 * data_manager->num_rigid_fluid_contacts;
    }
    return nnz_fluid_fluid;
}

void ChMPMContainer::ComputeInvMass(int offset) {
    CompressedMatrix<real>& M_inv = data_manager->host_data.M_inv;
    real inv_mass = 1.0 / mass;
    for (int i = 0; i < num_mpm_markers; i++) {
        M_inv.append(offset + i * 3 + 0, offset + i * 3 + 0, inv_mass);
        M_inv.finalize(offset + i * 3 + 0);
        M_inv.append(offset + i * 3 + 1, offset + i * 3 + 1, inv_mass);
        M_inv.finalize(offset + i * 3 + 1);
        M_inv.append(offset + i * 3 + 2, offset + i * 3 + 2, inv_mass);
        M_inv.finalize(offset + i * 3 + 2);
    }
}
void ChMPMContainer::ComputeMass(int offset) {
    CompressedMatrix<real>& M = data_manager->host_data.M;
    for (int i = 0; i < num_mpm_markers; i++) {
        M.append(offset + i * 3 + 0, offset + i * 3 + 0, mass);
        M.finalize(offset + i * 3 + 0);
        M.append(offset + i * 3 + 1, offset + i * 3 + 1, mass);
        M.finalize(offset + i * 3 + 1);
        M.append(offset + i * 3 + 2, offset + i * 3 + 2, mass);
        M.finalize(offset + i * 3 + 2);
    }
}

void ChMPMContainer::Initialize() {
    printf("ChMPMContainer::Initialize()\n");
    ComputeDOF();
    MPM_Settings temp_settings;
    temp_settings.dt = data_manager->settings.step_size;
    temp_settings.kernel_radius = kernel_radius;
    temp_settings.inv_radius = 1.0 / kernel_radius;
    temp_settings.bin_edge = kernel_radius * 2;
    temp_settings.inv_bin_edge = 1.0 / (kernel_radius * 2.0);
    temp_settings.max_velocity = max_velocity;
    temp_settings.mu = mu;
    temp_settings.lambda = lambda;
    temp_settings.hardening_coefficient = hardening_coefficient;
    temp_settings.theta_c = theta_c;
    temp_settings.theta_s = theta_s;
    temp_settings.alpha_flip = alpha_flip;
    temp_settings.youngs_modulus = youngs_modulus;
    temp_settings.poissons_ratio = nu;
    temp_settings.num_mpm_markers = num_mpm_markers;
    temp_settings.mass = mass;
    temp_settings.num_iterations = max_iterations;
    if (max_iterations > 0) {
        MPM_Initialize(temp_settings, data_manager->host_data.pos_3dof);
    }
}

void ChMPMContainer::Build_D() {
    LOG(INFO) << "ChMPMContainer::Build_D " << num_rigid_fluid_contacts << " " << num_mpm_contacts;
    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;
    const real dt = data_manager->settings.step_size;
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

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
            Loop_Over_Rigid_Neighbors(int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];
                                      real3 U = norm[p * max_rigid_neighbors + i]; real3 V; real3 W;  //
                                      Orthogonalize(U, V, W); real3 T1; real3 T2; real3 T3;           //
                                      Compute_Jacobian(rot_rigid[rigid], U, V, W,
                                                       cpta[p * max_rigid_neighbors + i] - pos_rigid[rigid], T1, T2,
                                                       T3);

                                      SetRow6Check(D_T, start_boundary + index + 0, rigid * 6, -U, T1);
                                      SetRow3Check(D_T, start_boundary + index + 0, body_offset + p * 3, U););
        } else {
#pragma omp parallel for
            Loop_Over_Rigid_Neighbors(
                int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];
                real3 U = norm[p * max_rigid_neighbors + i]; real3 V; real3 W;  //
                Orthogonalize(U, V, W); real3 T1; real3 T2; real3 T3;           //
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

    if (num_mpm_contacts > 0) {
#if USE_FLUID
        density.resize(num_fluid_bodies);
        printf("FUID JACOB %d \n", num_fluid_bodies);
        real h = kernel_radius;
        real inv_density = 1.0 / rho;
        real mass_over_density = mass * inv_density;
        uint start_density = start_contact;
        custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;
        //#pragma omp parallel for
        for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
            real dens = 0;
            real3 dcon_diag = real3(0.0);
            real3 pos_p = sorted_pos[body_a];
            int d_ind = 0;
            for (int i = 0; i < data_manager->host_data.c_counts_3dof_3dof[body_a]; i++) {
                int body_b = data_manager->host_data.neighbor_3dof_3dof[body_a * max_neighbors + i];
                if (body_a == body_b) {
                    dens += mass * CPOLY6 * H6;
                    d_ind = i;
                    continue;
                }
                int column = body_offset + body_b * 3;
                real3 xij = pos_p - sorted_pos[body_b];
                real dist = Length(xij);

                dens += mass * KPOLY6;

                real3 kernel_xij = KGSPIKY * xij;
                real3 dcon_od = mass_over_density * kernel_xij;  // off diagonal
                dcon_diag -= dcon_od;                            // diagonal is sum
                // den_con_jac[body_a * max_neighbors + i] = dcon_od;
                SetRow3Check(D_T, start_density + body_a, body_offset + body_b * 3, dcon_od);
            }
            // den_con_jac[body_a * max_neighbors + d_ind] = dcon_diag;
            SetRow3Check(D_T, start_density + body_a, body_offset + body_a * 3, dcon_diag);
            density[body_a] = dens;
        }
        // normalize
        printf("FUID normalize %d \n", num_fluid_bodies);
#pragma omp parallel for
        for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
            real dens = 0;
            real3 diag = real3(0);
            real3 pos_p = sorted_pos[body_a];
            for (int i = 0; i < data_manager->host_data.c_counts_3dof_3dof[body_a]; i++) {
                int body_b = data_manager->host_data.neighbor_3dof_3dof[body_a * max_neighbors + i];
                if (body_a == body_b) {
                    dens += mass / density[body_b] * CPOLY6 * H6;
                    continue;
                }
                real3 xij = pos_p - sorted_pos[body_b];
                real dist = Length(xij);
                dens += (mass / density[body_b]) * KPOLY6;
            }
            density[body_a] = density[body_a] / dens;
        }

#else
        int index = 0;
        custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;

        Loop_Over_Fluid_Neighbors(real3 U = -Normalize(xij); real3 V; real3 W;  //
                                  Orthogonalize(U, V, W);
                                  SetRow3Check(D_T, start_contact + index + 0, body_offset + body_a * 3, -U);
                                  SetRow3Check(D_T, start_contact + index + 0, body_offset + body_b * 3, U););
#endif
    }
}
void ChMPMContainer::Build_b() {
    LOG(INFO) << "ChMPMContainer::Build_b";
    real dt = data_manager->settings.step_size;
    DynamicVector<real>& b = data_manager->host_data.b;
    if (num_rigid_fluid_contacts > 0) {
        custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
        custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;

        if (contact_mu == 0) {
#pragma omp parallel for
            Loop_Over_Rigid_Neighbors(real depth =
                                          data_manager->host_data.dpth_rigid_fluid[p * max_rigid_neighbors + i];  //
                                      real bi = 0;                                                                //
                                      if (contact_cohesion) { depth = Min(depth, 0); } else {
                                          bi = std::max(real(1.0) / dt * depth, -contact_recovery_speed);
                                      }

                                      b[start_boundary + index + 0] = bi;

                                      // printf("bi: %f\n", bi);

                                      );
        } else {
#pragma omp parallel for
            Loop_Over_Rigid_Neighbors(real depth =
                                          data_manager->host_data.dpth_rigid_fluid[p * max_rigid_neighbors + i];  //
                                      real bi = 0;                                                                //
                                      if (contact_cohesion) { depth = Min(depth, 0); } else {
                                          bi = std::max(real(1.0) / dt * depth, -contact_recovery_speed);
                                      }

                                      b[start_boundary + index + 0] = bi;
                                      b[start_boundary + num_rigid_fluid_contacts + index * 2 + 0] = 0;
                                      b[start_boundary + num_rigid_fluid_contacts + index * 2 + 1] = 0;);
        }
    }

#if USE_FLUID
    uint start_density = start_contact;
    if (num_fluid_bodies > 0) {
#pragma omp parallel for
        for (int index = 0; index < num_fluid_bodies; index++) {
            b[start_density + index] = -(density[index] / rho - 1.0);
        }
    }
#else
    if (num_mpm_contacts > 0) {
        int index = 0;
        custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;

        Loop_Over_Fluid_Neighbors(real depth = Length(xij) - kernel_radius;  //
                                  real bi = 0;                               //
                                  if (cohesion) {
                                      depth = Min(depth, 0);  //
                                      bi = std::max(real(1.0) / dt * depth, -contact_recovery_speed);
                                  } else { real bi = std::max(real(1.0) / dt * depth, -contact_recovery_speed); }  //
                                  b[start_contact + index + 0] = bi;);
    }
#endif
}
void ChMPMContainer::Build_E() {
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

#if USE_FLUID
    uint start_density = start_contact;
    real zeta = 1.0 / (1.0 + 4.0 * tau / data_manager->settings.step_size);
    real compliance = 4.0 / (data_manager->settings.step_size * data_manager->settings.step_size) * (epsilon * zeta);
    if (num_fluid_bodies > 0) {
#pragma omp parallel for
        for (int index = 0; index < num_fluid_bodies; index++) {
            E[start_density + index] = compliance;
        }
    }
#else
    if (num_mpm_contacts > 0) {
#pragma omp parallel for
        for (int index = 0; index < num_mpm_contacts; index++) {
            E[start_contact + index + 0] = 0;
        }
    }
#endif
}

void ChMPMContainer::Project(real* gamma) {
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
#if USE_FLUID
#else
#pragma omp parallel for
    for (int index = 0; index < num_mpm_contacts; index++) {
        real3 gam;
        gam.x = gamma[start_contact + index];
        gam.x += cohesion;
        gam.x = gam.x < 0 ? 0 : gam.x - cohesion;
        gamma[start_contact + index] = gam.x;
    }
#endif
}

void ChMPMContainer::GenerateSparsity() {
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

    LOG(INFO) << "ChMPMContainer::GenerateSparsity";
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
#if USE_FLUID
    uint start_density = start_contact;
    if (num_mpm_contacts > 0) {
        for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
            for (int i = 0; i < data_manager->host_data.c_counts_3dof_3dof[body_a]; i++) {
                int body_b = data_manager->host_data.neighbor_3dof_3dof[body_a * max_neighbors + i];
                AppendRow3(D_T, start_density + body_a, body_offset + body_b * 3, 0);
            }
            D_T.finalize(start_density + body_a);
        }
    }
#else
    if (num_mpm_contacts > 0) {
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
    }
#endif
}
void ChMPMContainer::PreSolve() {
    LOG(INFO) << "ChMPMContainer::PreSolve()";
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_3dof;
    const real dt = data_manager->settings.step_size;

    MPM_Settings temp_settings;
    temp_settings.dt = dt;
    temp_settings.kernel_radius = kernel_radius;
    temp_settings.inv_radius = 1.0 / kernel_radius;
    temp_settings.bin_edge = kernel_radius * 2;
    temp_settings.inv_bin_edge = 1.0 / (kernel_radius * 2.0);
    temp_settings.max_velocity = max_velocity;
    temp_settings.mu = mu;
    temp_settings.lambda = lambda;
    temp_settings.hardening_coefficient = hardening_coefficient;
    temp_settings.theta_c = theta_c;
    temp_settings.theta_s = theta_s;
    temp_settings.alpha_flip = alpha_flip;
    temp_settings.youngs_modulus = youngs_modulus;
    temp_settings.poissons_ratio = nu;
    temp_settings.num_mpm_markers = num_mpm_markers;
    temp_settings.mass = mass;
    temp_settings.num_iterations = max_iterations;
    if (max_iterations > 0) {
        MPM_Solve(temp_settings, data_manager->host_data.pos_3dof, data_manager->host_data.vel_3dof);
    }
#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        int index = data_manager->host_data.reverse_mapping_3dof[p];
        data_manager->host_data.v[body_offset + index * 3 + 0] = data_manager->host_data.vel_3dof[p].x;
        data_manager->host_data.v[body_offset + index * 3 + 1] = data_manager->host_data.vel_3dof[p].y;
        data_manager->host_data.v[body_offset + index * 3 + 2] = data_manager->host_data.vel_3dof[p].z;
    }

    LOG(INFO) << "ChMPMContainer::DonePreSolve()";
}
void ChMPMContainer::PostSolve() {}
void ChMPMContainer::Solve(const DynamicVector<real>& r, DynamicVector<real>& gamma) {}
void ChMPMContainer::UpdateRhs() {}

}  // END_OF_NAMESPACE____

/////////////////////
