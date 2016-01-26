
#include <algorithm>
#include <math.h>
#include "chrono_parallel/physics/ChSystemParallel.h"
#include <chrono_parallel/physics/Ch3DOFContainer.h>
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/constraints/ChConstraintFluidFluidUtils.h"
#include "chrono_parallel/constraints/ChConstraintUtils.h"

#include "chrono_parallel/math/other_types.h"         // for uint, int2, int3
#include "chrono_parallel/math/real.h"                // for real
#include "chrono_parallel/math/real2.h"               // for real2
#include "chrono_parallel/math/real3.h"               // for real3
#include "chrono_parallel/math/real4.h"               // for quaternion, real4
#include "chrono_parallel/math/mat33.h"               // for quaternion, real4

namespace chrono {

using namespace collision;
using namespace geometry;

ChFluidContainer::ChFluidContainer(ChSystemParallelDVI* physics_system) {
    data_manager = physics_system->data_manager;
    data_manager->Add3DOFContainer(this);
    body_offset = 0;
    compliance = 0;
    epsilon = 10e-3;
    tau = 4 * .01;

    rho = 1000;
    mass = 0.037037;

    viscosity = 0;

    artificial_pressure = false;
    artificial_pressure_k = 0.1;
    artificial_pressure_dq = .2 * kernel_radius;
    artificial_pressure_n = 4;
    enable_viscosity = false;
    initialize_mass = false;
}
ChFluidContainer::~ChFluidContainer() {}

void ChFluidContainer::AddFluid(const std::vector<real3>& positions, const std::vector<real3>& velocities) {
    custom_vector<real3>& pos_fluid = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_fluid = data_manager->host_data.vel_3dof;

    pos_fluid.insert(pos_fluid.end(), positions.begin(), positions.end());
    vel_fluid.insert(vel_fluid.end(), velocities.begin(), velocities.end());
    // In case the number of velocities provided were not enough, resize to the number of fluid bodies
    vel_fluid.resize(pos_fluid.size());
    data_manager->num_fluid_bodies = pos_fluid.size();
}
void ChFluidContainer::Update(double ChTime) {
    uint num_fluid_bodies = data_manager->num_fluid_bodies;
    uint num_rigid_bodies = data_manager->num_rigid_bodies;
    uint num_shafts = data_manager->num_shafts;
    custom_vector<real3>& pos_fluid = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_fluid = data_manager->host_data.vel_3dof;
    real3 g_acc = data_manager->settings.gravity;
    real3 h_gravity = data_manager->settings.step_size * mass * g_acc;
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

void ChFluidContainer::UpdatePosition(double ChTime) {
    uint num_fluid_bodies = data_manager->num_fluid_bodies;
    uint num_rigid_bodies = data_manager->num_rigid_bodies;
    uint num_shafts = data_manager->num_shafts;

    custom_vector<real3>& pos_fluid = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_fluid = data_manager->host_data.vel_3dof;

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

int ChFluidContainer::GetNumConstraints() {
    int num_fluid_fluid = data_manager->num_fluid_bodies + data_manager->num_rigid_fluid_contacts * 3;

    if (enable_viscosity) {
        num_fluid_fluid += data_manager->num_fluid_bodies * 3;
    }

    // printf("ChFluidContainer::GetNumConstraints() %d\n", num_fluid_fluid);
    return num_fluid_fluid;
}
int ChFluidContainer::GetNumNonZeros() {
    int nnz_fluid_fluid =
        data_manager->num_fluid_bodies * 6 * max_neighbors + 9 * 3 * data_manager->num_rigid_fluid_contacts;

    if (enable_viscosity) {
        nnz_fluid_fluid += data_manager->num_fluid_bodies * 18 * max_neighbors;
    }
    // printf("ChFluidContainer::GetNumNonZeros() %d\n", nnz_fluid_fluid);
    return nnz_fluid_fluid;
}

void ChFluidContainer::ComputeInvMass(int offset) {
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
void ChFluidContainer::ComputeMass(int offset) {
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
void ChFluidContainer::Setup(int start_constraint) {
    Ch3DOFContainer::Setup(start_constraint);

    start_boundary = start_constraint;
    start_density = start_constraint + num_rigid_fluid_contacts * 3;
    start_viscous = start_constraint + num_rigid_fluid_contacts * 3 + num_fluid_bodies;

    body_offset = num_rigid_bodies * 6 + num_shafts;
}

void ChFluidContainer::Initialize() {
    density.resize(num_fluid_bodies);

    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;
    real h = kernel_radius;
    real vol = 4.0 / 3.0 * CH_C_PI * h * h * h;

#pragma omp parallel for
    for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
        real dens = 0;
        real3 pos_p = sorted_pos[body_a];
        for (int i = 0; i < data_manager->host_data.c_counts_3dof_3dof[body_a]; i++) {
            int body_b = data_manager->host_data.neighbor_3dof_3dof[body_a * max_neighbors + i];
            if (body_a == body_b) {
                dens += mass * CPOLY6 * H6;
                continue;
            }
            real3 xij = pos_p - sorted_pos[body_b];
            real dist = Length(xij);
            dens += mass * KPOLY6;
        }
        density[body_a] = dens;
    }

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
    real total_density = 0;
    for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
        total_density += density[body_a];
    }
    real avg_density = total_density / num_fluid_bodies;
    real scale = rho / avg_density;

    mass = mass * scale;
    printf("Computed mass: %f\n", mass);
}
bool init_mass = false;

void ChFluidContainer::Density_Fluid() {
    if (initialize_mass && init_mass == false) {
        Initialize();
        init_mass = true;
    }
    LOG(INFO) << "ChConstraintFluidFluid::Density_Fluid";
    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;
    real h = kernel_radius;
    real envel = data_manager->settings.collision.collision_envelope;
    real inv_density = 1.0 / rho;
    real mass_over_density = mass * inv_density;
    real eta = .01;
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

#pragma omp parallel for
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
}
void ChFluidContainer::Normalize_Density_Fluid() {
    real h = kernel_radius;
    custom_vector<real3>& pos = data_manager->host_data.pos_3dof;
    real inv_density = 1.0 / rho;
    real mass_over_density = mass * inv_density;
    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;

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
}

void ChFluidContainer::Build_D() {
    LOG(INFO) << "ChConstraintFluidFluid::Build_D_Fluid";

    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

    if (num_rigid_fluid_contacts > 0) {
        custom_vector<real3>& pos_rigid = data_manager->host_data.pos_rigid;
        custom_vector<quaternion>& rot_rigid = data_manager->host_data.rot_rigid;

        // custom_vector<int2>& bids = data_manager->host_data.bids_rigid_fluid;
        custom_vector<real3>& cpta = data_manager->host_data.cpta_rigid_fluid;
        custom_vector<real3>& norm = data_manager->host_data.norm_rigid_fluid;
        custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
        custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;
        int index = 0;
        for (int p = 0; p < num_fluid_bodies; p++) {
            for (int i = 0; i < contact_counts[p]; i++) {
                int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];
                int fluid = p;  // fluid body is in second index
                real3 U = norm[p * max_rigid_neighbors + i], V, W;
                Orthogonalize(U, V, W);
                real3 T1, T2, T3;
                Compute_Jacobian(rot_rigid[rigid], U, V, W, cpta[p * max_rigid_neighbors + i] - pos_rigid[rigid], T1,
                                 T2, T3);

                SetRow6Check(D_T, start_boundary + index + 0, rigid * 6, -U, T1);
                SetRow6Check(D_T, start_boundary + num_rigid_fluid_contacts + index * 2 + 0, rigid * 6, -V, T2);
                SetRow6Check(D_T, start_boundary + num_rigid_fluid_contacts + index * 2 + 1, rigid * 6, -W, T3);

                SetRow3Check(D_T, start_boundary + index + 0, body_offset + fluid * 3, U);
                SetRow3Check(D_T, start_boundary + num_rigid_fluid_contacts + index * 2 + 0, body_offset + fluid * 3,
                             V);
                SetRow3Check(D_T, start_boundary + num_rigid_fluid_contacts + index * 2 + 1, body_offset + fluid * 3,
                             W);
                index++;
            }
        }
    }
    if (data_manager->num_fluid_contacts >= 0) {
        real h = kernel_radius;
        real envel = data_manager->settings.collision.collision_envelope;
        real inv_density = 1.0 / rho;
        real mass_over_density = mass * inv_density;
        real eta = .01;

        // custom_vector<real3>& vel = data_manager->host_data.vel_3dof;
        custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;

        //=======COMPUTE DENSITY OF FLUID
        density.resize(num_fluid_bodies);
        Density_Fluid();
        Normalize_Density_Fluid();

        real visca = viscosity;
        real viscb = viscosity;
        const real mass_2 = mass * mass;
        const real eta_2 = eta * eta;
        if (enable_viscosity) {
#pragma omp parallel for
            for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
                real3 pos_p = sorted_pos[body_a];
                real3 vmat_row1(0);
                real3 vmat_row2(0);
                real3 vmat_row3(0);
                int d_ind = 0;
                for (int i = 0; i < data_manager->host_data.c_counts_3dof_3dof[body_a]; i++) {
                    int body_b = data_manager->host_data.neighbor_3dof_3dof[body_a * max_neighbors + i];
                    if (body_a == body_b) {
                        d_ind = i;
                        continue;
                    }
                    int column = body_offset + body_b * 3;
                    real3 xij = pos_p - sorted_pos[body_b];
                    real dist = Length(xij);
                    real3 kernel_xij = KGSPIKY * xij;
                    //
                    real density_a = density[body_a];
                    real density_b = density[body_b];
                    real part_a = (8.0 / (density_a + density_b));
                    real part_b = visca + viscb;  //(visca / density_a + viscb / density_b);  // /
                    real part_c = 1.0 / (h * ((dist * dist / (H2)) + eta_2));
                    real scalar = -mass_2 * part_a * part_b * part_c;

                    real3 r1 = xij[0] * kernel_xij * scalar;
                    real3 r2 = xij[1] * kernel_xij * scalar;
                    real3 r3 = xij[2] * kernel_xij * scalar;

                    SetRow3Check(D_T, start_viscous + body_a * 3 + 0, body_offset + body_b * 3, r1);
                    SetRow3Check(D_T, start_viscous + body_a * 3 + 1, body_offset + body_b * 3, r2);
                    SetRow3Check(D_T, start_viscous + body_a * 3 + 2, body_offset + body_b * 3, r3);

                    vmat_row1 -= r1;
                    vmat_row2 -= r2;
                    vmat_row3 -= r3;
                }

                SetRow3Check(D_T, start_viscous + body_a * 3 + 0, body_offset + body_a * 3, vmat_row1);
                SetRow3Check(D_T, start_viscous + body_a * 3 + 1, body_offset + body_a * 3, vmat_row2);
                SetRow3Check(D_T, start_viscous + body_a * 3 + 2, body_offset + body_a * 3, vmat_row3);
            }
        }

        // Add more entries for viscosity
        // Code is repeated because there are three rows per viscosity constraint
    }
    LOG(INFO) << "ChConstraintFluidFluid::JACOBIAN OF FLUID";
}
void ChFluidContainer::Build_b() {
    real dt = data_manager->settings.step_size;
    real h = kernel_radius;
    real zeta = 1.0 / (1.0 + 4.0 * tau / h);
    DynamicVector<real>& b = data_manager->host_data.b;
    if (num_rigid_fluid_contacts > 0) {
        custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
        custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;

        int index = 0;
        for (int p = 0; p < num_fluid_bodies; p++) {
            for (int i = 0; i < contact_counts[p]; i++) {
                real depth = data_manager->host_data.dpth_rigid_fluid[p * max_rigid_neighbors + i];

                real bi = std::max(real(1.0) / dt * depth, -data_manager->node_container->contact_recovery_speed);
                b[start_boundary + index + 0] = bi;
                b[start_boundary + num_rigid_fluid_contacts + index * 2 + 0] = 0;
                b[start_boundary + num_rigid_fluid_contacts + index * 2 + 1] = 0;
                index++;
            }
        }
    }
    if (num_fluid_bodies > 0) {
#pragma omp parallel for
        for (int index = 0; index < num_fluid_bodies; index++) {
            b[start_density + index] = -(density[index] / rho - 1.0);
        }
    }
}
void ChFluidContainer::Build_E() {
    DynamicVector<real>& E = data_manager->host_data.E;

    if (num_rigid_fluid_contacts > 0) {
#pragma omp parallel for
        for (int index = 0; index < num_rigid_fluid_contacts; index++) {
            E[start_boundary + index + 0] = 0;
            E[start_boundary + num_rigid_fluid_contacts + index * 2 + 0] = 0;
            E[start_boundary + num_rigid_fluid_contacts + index * 2 + 1] = 0;
        }
    }

    real step_size = data_manager->settings.step_size;
    real zeta = 1.0 / (1.0 + 4.0 * tau / step_size);
    real compliance = 4.0 / (step_size * step_size) * (epsilon * zeta);

    if (num_fluid_bodies > 0) {
#pragma omp parallel for
        for (int index = 0; index < num_fluid_bodies; index++) {
            E[start_density + index] = 0;  // compliance;
            if (enable_viscosity) {
                E[start_viscous + index * 3 + 0] = 0;
                E[start_viscous + index * 3 + 1] = 0;
                E[start_viscous + index * 3 + 2] = 0;
            }
        }
    }
}

bool Cone_generalized_rf(real& gamma_n, real& gamma_u, real& gamma_v, const real& mu) {
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

void ChFluidContainer::Project(real* gamma) {
    // custom_vector<int2>& bids = data_manager->host_data.bids_rigid_fluid;
    real mu = data_manager->node_container->contact_mu;
    real coh = data_manager->node_container->contact_cohesion;

    custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
    custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;

    //#pragma omp parallel for
    int index = 0;
    for (int p = 0; p < num_fluid_bodies; p++) {
        for (int i = 0; i < contact_counts[p]; ++i) {
            int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];  // rigid is stored in the first index
            int fluid = p;                                                  // fluid body is in second index
            real rigid_fric = data_manager->host_data.fric_data[rigid].x;
            real cohesion = Max((data_manager->host_data.cohesion_data[rigid] + coh) * .5, 0.0);
            real friction = (rigid_fric == 0 || mu == 0) ? 0 : (rigid_fric + mu) * .5;

            real3 gam;
            gam.x = gamma[start_boundary + index];
            gam.y = gamma[start_boundary + num_rigid_fluid_contacts + index * 2 + 0];
            gam.z = gamma[start_boundary + num_rigid_fluid_contacts + index * 2 + 1];

            gam.x += cohesion;

            real mu = friction;
            if (mu == 0) {
                gam.x = gam.x < 0 ? 0 : gam.x - cohesion;
                gam.y = gam.z = 0;

                gamma[start_boundary + index] = gam.x;
                gamma[start_boundary + num_rigid_fluid_contacts + index * 2 + 0] = gam.y;
                gamma[start_boundary + num_rigid_fluid_contacts + index * 2 + 1] = gam.z;
                index++;
                continue;
            }

            if (Cone_generalized_rf(gam.x, gam.y, gam.z, mu)) {
            }

            gamma[start_boundary + index] = gam.x - cohesion;
            gamma[start_boundary + num_rigid_fluid_contacts + index * 2 + 0] = gam.y;
            gamma[start_boundary + num_rigid_fluid_contacts + index * 2 + 1] = gam.z;
            index++;
        }
    }
}
void ChFluidContainer::GenerateSparsity() {
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

    if (num_rigid_fluid_contacts > 0) {
        LOG(INFO) << "ChConstraintRigidFluid::GenerateSparsity";

        int index_n = 0;
        int index_t = 0;

        custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
        custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;

        for (int p = 0; p < num_fluid_bodies; p++) {
            for (int i = 0; i < contact_counts[p]; i++) {
                int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];
                int fluid = p;

                AppendRow6(D_T, start_boundary + index_n + 0, rigid * 6, 0);
                AppendRow3(D_T, start_boundary + index_n + 0, body_offset + fluid * 3, 0);
                D_T.finalize(start_boundary + index_n + 0);
                index_n++;
            }
        }
        for (int p = 0; p < num_fluid_bodies; p++) {
            for (int i = 0; i < contact_counts[p]; i++) {
                int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];
                int fluid = p;

                AppendRow6(D_T, start_boundary + num_rigid_fluid_contacts + index_t * 2 + 0, rigid * 6, 0);
                AppendRow3(D_T, start_boundary + num_rigid_fluid_contacts + index_t * 2 + 0, body_offset + fluid * 3,
                           0);
                D_T.finalize(start_boundary + num_rigid_fluid_contacts + index_t * 2 + 0);

                AppendRow6(D_T, start_boundary + num_rigid_fluid_contacts + index_t * 2 + 1, rigid * 6, 0);
                AppendRow3(D_T, start_boundary + num_rigid_fluid_contacts + index_t * 2 + 1, body_offset + fluid * 3,
                           0);

                D_T.finalize(start_boundary + num_rigid_fluid_contacts + index_t * 2 + 1);
                index_t++;
            }
        }
    }
    if (data_manager->num_fluid_contacts > 0) {
        LOG(INFO) << "ChConstraintFluidFluid::GenerateSparsity";

        for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
            for (int i = 0; i < data_manager->host_data.c_counts_3dof_3dof[body_a]; i++) {
                int body_b = data_manager->host_data.neighbor_3dof_3dof[body_a * max_neighbors + i];
                AppendRow3(D_T, start_density + body_a, body_offset + body_b * 3, 0);
            }
            D_T.finalize(start_density + body_a);
        }
        // Add more entries for viscosity
        // Code is repeated because there are three rows per viscosity constraint
        if (enable_viscosity) {
            for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
                for (int i = 0; i < data_manager->host_data.c_counts_3dof_3dof[body_a]; i++) {
                    int body_b = data_manager->host_data.neighbor_3dof_3dof[body_a * max_neighbors + i];
                    AppendRow3(D_T, start_viscous + body_a * 3 + 0, body_offset + body_b * 3, 0);
                }
                D_T.finalize(start_viscous + body_a * 3 + 0);
                //
                for (int i = 0; i < data_manager->host_data.c_counts_3dof_3dof[body_a]; i++) {
                    int body_b = data_manager->host_data.neighbor_3dof_3dof[body_a * max_neighbors + i];
                    AppendRow3(D_T, start_viscous + body_a * 3 + 1, body_offset + body_b * 3, 0);
                }
                D_T.finalize(start_viscous + body_a * 3 + 1);
                //
                for (int i = 0; i < data_manager->host_data.c_counts_3dof_3dof[body_a]; i++) {
                    int body_b = data_manager->host_data.neighbor_3dof_3dof[body_a * max_neighbors + i];
                    AppendRow3(D_T, start_viscous + body_a * 3 + 2, body_offset + body_b * 3, 0);
                }
                D_T.finalize(start_viscous + body_a * 3 + 2);
            }
        }
    }
}

void ChFluidContainer::PostSolve() {
    if (artificial_pressure == false) {
        return;
    }
    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;
    custom_vector<real3>& sorted_vel = data_manager->host_data.sorted_vel_3dof;
    real inv_density = 1.0 / rho;
    real h = kernel_radius;
    real k = artificial_pressure_k;
    real dq = artificial_pressure_dq;
    real n = artificial_pressure_n;
    real dt = data_manager->settings.step_size;
#pragma omp parallel for
    for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
        real corr = 0;
        real3 vorticity_grad(0);
        real3 pos_a = sorted_pos[body_a];
        for (int i = 0; i < data_manager->host_data.c_counts_3dof_3dof[body_a]; i++) {
            int body_b = data_manager->host_data.neighbor_3dof_3dof[body_a * max_neighbors + i];
            if (body_a == body_b) {
                continue;
            }
            real3 xij = (pos_a - sorted_pos[body_b]);

            real dist = Length(xij);
            corr += k * Pow(KERNEL(dist, h) / KERNEL(dq, h), n);
        }

        data_manager->host_data.gamma[start_density + body_a] += corr;
    }
}

void ChFluidContainer::CalculateContactForces() {
    uint num_contacts = data_manager->num_fluid_contacts;
    if (num_contacts <= 0) {
        return;
    }

    DynamicVector<real>& gamma = data_manager->host_data.gamma;
    SubVectorType gamma_n = subvector(gamma, start_boundary, _num_rf_c_);
    SubVectorType gamma_t = subvector(gamma, start_boundary + _num_rf_c_, 2 * _num_rf_c_);

    contact_forces =
        submatrix(data_manager->host_data.D, 0, start_boundary, _num_dof_, _num_rf_c_) * gamma_n / data_manager->settings.step_size +
        submatrix(data_manager->host_data.D, 0, start_boundary + _num_rf_c_, _num_dof_, 2 * _num_rf_c_) * gamma_t / data_manager->settings.step_size;

    // contact_forces
}

real3 ChFluidContainer::GetBodyContactForce(uint body_id) {
    if (data_manager->num_fluid_contacts <= 0) {
        return real3(0);
    }
    return real3(contact_forces[body_id * 6 + 0], contact_forces[body_id * 6 + 1], contact_forces[body_id * 6 + 2]);
}

real3 ChFluidContainer::GetBodyContactTorque(uint body_id) {
    if (data_manager->num_fluid_contacts <= 0) {
        return real3(0);
    }
    return real3(contact_forces[body_id * 6 + 3], contact_forces[body_id * 6 + 4], contact_forces[body_id * 6 + 5]);
}

}  // END_OF_NAMESPACE____

/////////////////////
