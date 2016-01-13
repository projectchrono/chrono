
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include "chrono_parallel/physics/ChSystemParallel.h"
#include <chrono_parallel/physics/Ch3DOFContainer.h>
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/constraints/ChConstraintFluidFluidUtils.h"
#include "chrono_parallel/constraints/ChConstraintUtils.h"
namespace chrono {

using namespace collision;
using namespace geometry;

ChFluidContainer::ChFluidContainer(ChSystemParallelDVI* physics_system) {
    data_manager = physics_system->data_manager;
    data_manager->Add3DOFContainer(this);
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
    real3 h_gravity = data_manager->settings.step_size * data_manager->settings.fluid.mass * g_acc;
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
        if (speed > data_manager->settings.fluid.max_velocity) {
            vel = vel * data_manager->settings.fluid.max_velocity / speed;
        }
        vel_fluid[original_index] = vel;
        pos_fluid[original_index] += vel * data_manager->settings.step_size;
    }
}

int ChFluidContainer::GetNumConstraints() {
    int num_fluid_fluid = data_manager->num_fluid_bodies;

    if (data_manager->settings.fluid.enable_viscosity) {
        num_fluid_fluid += data_manager->num_fluid_bodies * 3;
    }
    return num_fluid_fluid;
}
int ChFluidContainer::GetNumNonZeros() {
    int nnz_fluid_fluid = data_manager->num_fluid_bodies * 6 * max_neighbors;

    if (data_manager->settings.fluid.enable_viscosity) {
        nnz_fluid_fluid += data_manager->num_fluid_bodies * 18 * max_neighbors;
    }
    return nnz_fluid_fluid;
}

void ChFluidContainer::ComputeInvMass(int offset) {
    CompressedMatrix<real>& M_inv = data_manager->host_data.M_inv;
    uint num_fluid_bodies = data_manager->num_fluid_bodies;

    real inv_mass = 1.0 / data_manager->settings.fluid.mass;
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

    real fluid_mass = data_manager->settings.fluid.mass;
    for (int i = 0; i < num_fluid_bodies; i++) {
        M.append(offset + i * 3 + 0, offset + i * 3 + 0, fluid_mass);
        M.finalize(offset + i * 3 + 0);
        M.append(offset + i * 3 + 1, offset + i * 3 + 1, fluid_mass);
        M.finalize(offset + i * 3 + 1);
        M.append(offset + i * 3 + 2, offset + i * 3 + 2, fluid_mass);
        M.finalize(offset + i * 3 + 2);
    }
}
void ChFluidContainer::Setup() {
    num_fluid_contacts = data_manager->num_fluid_contacts;
    num_fluid_bodies = data_manager->num_fluid_bodies;
    num_rigid_bodies = data_manager->num_rigid_bodies;
    num_rigid_fluid_contacts = data_manager->num_rigid_fluid_contacts;
    num_unilaterals = data_manager->num_unilaterals;
    num_bilaterals = data_manager->num_bilaterals;
    num_shafts = data_manager->num_shafts;

    index_offset = num_unilaterals + num_bilaterals + num_rigid_fluid_contacts * 3;
    body_offset = num_rigid_bodies * 6 + num_shafts;
}

void ChFluidContainer::Initialize() {
    density.resize(num_fluid_bodies);

    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;
    real h = data_manager->settings.fluid.kernel_radius;
    real density_fluid = data_manager->settings.fluid.density;
    real mass_fluid = data_manager->settings.fluid.mass;
    real vol = 4.0 / 3.0 * CH_C_PI * h * h * h;

#pragma omp parallel for
    for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
        real dens = 0;
        real3 pos_p = sorted_pos[body_a];
        for (int i = 0; i < data_manager->host_data.c_counts_3dof_3dof[body_a]; i++) {
            int body_b = data_manager->host_data.neighbor_3dof_3dof[body_a * max_neighbors + i];
            if (body_a == body_b) {
                dens += mass_fluid * CPOLY6 * H6;
                continue;
            }
            real3 xij = pos_p - sorted_pos[body_b];
            real dist = Length(xij);
            dens += mass_fluid * KPOLY6;
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
                dens += mass_fluid / density[body_b] * CPOLY6 * H6;
                continue;
            }
            real3 xij = pos_p - sorted_pos[body_b];
            real dist = Length(xij);
            dens += (mass_fluid / density[body_b]) * KPOLY6;
        }
        density[body_a] = density[body_a] / dens;
    }
    real total_density = 0;
    for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
        total_density += density[body_a];
    }
    real avg_density = total_density / num_fluid_bodies;
    real scale = density_fluid / avg_density;

    data_manager->settings.fluid.mass = mass_fluid * scale;
    printf("Computed mass: %f\n", data_manager->settings.fluid.mass);
}
bool init_mass = false;

void ChFluidContainer::Density_Fluid() {
    if (data_manager->settings.fluid.initialize_mass && init_mass == false) {
        Initialize();
        init_mass = true;
    }
    LOG(INFO) << "ChConstraintFluidFluid::Density_Fluid";
    real mass_fluid = data_manager->settings.fluid.mass;
    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;
    real viscosity = data_manager->settings.fluid.viscosity;
    real mass = data_manager->settings.fluid.mass;
    real h = data_manager->settings.fluid.kernel_radius;
    real envel = data_manager->settings.collision.collision_envelope;
    real density_fluid = data_manager->settings.fluid.density;
    real inv_density = 1.0 / density_fluid;
    real mass_over_density = mass * inv_density;
    real eta = .01;

    den_con_jac.resize(num_fluid_bodies * max_neighbors);

#pragma omp parallel for
    for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
        int rdoff = index_offset + body_a;
        int rvoff = index_offset + num_fluid_bodies + body_a * 3;
        real dens = 0;
        real3 dcon_diag = real3(0.0);
        real3 pos_p = sorted_pos[body_a];
        int d_ind = 0;
        for (int i = 0; i < data_manager->host_data.c_counts_3dof_3dof[body_a]; i++) {
            int body_b = data_manager->host_data.neighbor_3dof_3dof[body_a * max_neighbors + i];
            if (body_a == body_b) {
                dens += mass_fluid * CPOLY6 * H6;
                d_ind = i;
                continue;
            }
            int column = body_offset + body_b * 3;
            real3 xij = pos_p - sorted_pos[body_b];
            real dist = Length(xij);

            dens += mass_fluid * KPOLY6;

            real3 kernel_xij = KGSPIKY * xij;
            real3 dcon_od = mass_over_density * kernel_xij;  // off diagonal
            dcon_diag -= dcon_od;                            // diagonal is sum
            den_con_jac[body_a * max_neighbors + i] = dcon_od;
        }
        den_con_jac[body_a * max_neighbors + d_ind] = dcon_diag;
        density[body_a] = dens;
    }
}
void ChFluidContainer::Normalize_Density_Fluid() {
    real h = data_manager->settings.fluid.kernel_radius;
    custom_vector<real3>& pos = data_manager->host_data.pos_3dof;
    real mass_fluid = data_manager->settings.fluid.mass;
    real density_fluid = data_manager->settings.fluid.density;
    real inv_density = 1.0 / density_fluid;
    real mass = data_manager->settings.fluid.mass;
    real mass_over_density = mass_fluid * inv_density;
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
    if (num_fluid_contacts <= 0) {
        return;
    }
    real viscosity = data_manager->settings.fluid.viscosity;
    real mass = data_manager->settings.fluid.mass;
    real h = data_manager->settings.fluid.kernel_radius;
    real envel = data_manager->settings.collision.collision_envelope;
    real density_fluid = data_manager->settings.fluid.density;
    real inv_density = 1.0 / density_fluid;
    real mass_over_density = mass * inv_density;
    real eta = .01;

    // custom_vector<real3>& vel = data_manager->host_data.vel_3dof;
    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;

    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

    //=======COMPUTE DENSITY OF FLUID
    density.resize(num_fluid_bodies);
    Density_Fluid();
    Normalize_Density_Fluid();

    visc1_jac.resize(num_fluid_bodies * max_neighbors);
    visc2_jac.resize(num_fluid_bodies * max_neighbors);
    visc3_jac.resize(num_fluid_bodies * max_neighbors);

    real visca = data_manager->settings.fluid.viscosity;
    real viscb = data_manager->settings.fluid.viscosity;
    const real mass_2 = mass * mass;
    const real eta_2 = eta * eta;
    if (data_manager->settings.fluid.enable_viscosity) {
#pragma omp parallel for
        for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
            int rdoff = index_offset + body_a;
            int rvoff = index_offset + num_fluid_bodies + body_a * 3;
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

                visc1_jac[body_a * max_neighbors + i] = r1;
                visc2_jac[body_a * max_neighbors + i] = r2;
                visc3_jac[body_a * max_neighbors + i] = r3;

                vmat_row1 -= r1;
                vmat_row2 -= r2;
                vmat_row3 -= r3;
            }
            visc1_jac[body_a * max_neighbors + d_ind] = vmat_row1;
            visc2_jac[body_a * max_neighbors + d_ind] = vmat_row2;
            visc3_jac[body_a * max_neighbors + d_ind] = vmat_row3;
        }
    }

    for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
        // D_T.reserve(index_offset + body_a, data_manager->settings.fluid.max_interactions * 3);
        for (int i = 0; i < data_manager->host_data.c_counts_3dof_3dof[body_a]; i++) {
            int body_b = data_manager->host_data.neighbor_3dof_3dof[body_a * max_neighbors + i];
            AppendRow3(D_T, index_offset + body_a, body_offset + body_b * 3, den_con_jac[body_a * max_neighbors + i]);
        }
        D_T.finalize(index_offset + body_a);
    }
    // Add more entries for viscosity
    // Code is repeated because there are three rows per viscosity constraint
    if (data_manager->settings.fluid.enable_viscosity) {
        for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
            // D_T.reserve(index_offset + num_fluid_bodies + body_a * 3 + 0,
            //            data_manager->settings.fluid.max_interactions * 3);
            for (int i = 0; i < data_manager->host_data.c_counts_3dof_3dof[body_a]; i++) {
                int body_b = data_manager->host_data.neighbor_3dof_3dof[body_a * max_neighbors + i];
                AppendRow3(D_T, index_offset + num_fluid_bodies + body_a * 3 + 0, body_offset + body_b * 3,
                           visc1_jac[body_a * max_neighbors + i]);
            }
            D_T.finalize(index_offset + num_fluid_bodies + body_a * 3 + 0);
            // D_T.reserve(index_offset + num_fluid_bodies + body_a * 3 + 1,
            //            data_manager->settings.fluid.max_interactions * 3);
            for (int i = 0; i < data_manager->host_data.c_counts_3dof_3dof[body_a]; i++) {
                int body_b = data_manager->host_data.neighbor_3dof_3dof[body_a * max_neighbors + i];
                AppendRow3(D_T, index_offset + num_fluid_bodies + body_a * 3 + 1, body_offset + body_b * 3,
                           visc2_jac[body_a * max_neighbors + i]);
            }
            D_T.finalize(index_offset + num_fluid_bodies + body_a * 3 + 1);
            // D_T.reserve(index_offset + num_fluid_bodies + body_a * 3 + 2,
            //            data_manager->settings.fluid.max_interactions * 3);
            for (int i = 0; i < data_manager->host_data.c_counts_3dof_3dof[body_a]; i++) {
                int body_b = data_manager->host_data.neighbor_3dof_3dof[body_a * max_neighbors + i];
                AppendRow3(D_T, index_offset + num_fluid_bodies + body_a * 3 + 2, body_offset + body_b * 3,
                           visc3_jac[body_a * max_neighbors + i]);
            }
            D_T.finalize(index_offset + num_fluid_bodies + body_a * 3 + 2);
        }
    }

    LOG(INFO) << "ChConstraintFluidFluid::JACOBIAN OF FLUID";
}
void ChFluidContainer::Build_b() {
    real dt = data_manager->settings.step_size;

    real density_fluid = data_manager->settings.fluid.density;
    real epsilon = data_manager->settings.fluid.epsilon;
    real tau = data_manager->settings.fluid.tau;
    real h = data_manager->settings.fluid.kernel_radius;
    real zeta = 1.0 / (1.0 + 4.0 * tau / h);

    SubVectorType b_sub = blaze::subvector(data_manager->host_data.b, index_offset, num_fluid_bodies);
    DynamicVector<real> g(num_fluid_bodies);
    SubVectorType v_sub = blaze::subvector(data_manager->host_data.v, body_offset, num_fluid_bodies * 3);
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    SubMatrixType D_T_sub = submatrix(D_T, index_offset, body_offset, num_fluid_bodies, num_fluid_bodies * 3);

#pragma omp parallel for
    for (int index = 0; index < num_fluid_bodies; index++) {
        b_sub[index] = -(density[index] / density_fluid - 1.0);
        // printf("b: %f\n", b_sub[index]);
        // g[index] = (density[index] / density_fluid - 1.0) ;
    }
    // b_sub = -4.0 / dt * zeta * g + zeta * D_T_sub * v_sub;
}
void ChFluidContainer::Build_E() {
    if (num_fluid_bodies <= 0) {
        return;
    }

    DynamicVector<real>& E = data_manager->host_data.E;
    real step_size = data_manager->settings.step_size;
    real epsilon = data_manager->settings.fluid.epsilon;
    real tau = data_manager->settings.fluid.tau;

    real zeta = 1.0 / (1.0 + 4.0 * tau / step_size);
    real compliance = 4.0 / (step_size * step_size) * (epsilon * zeta);

#pragma omp parallel for
    for (int index = 0; index < num_fluid_bodies; index++) {
        E[index_offset + index] = compliance;
        if (data_manager->settings.fluid.enable_viscosity) {
            E[index_offset + num_fluid_bodies + index * 3 + 0] = 0;
            E[index_offset + num_fluid_bodies + index * 3 + 1] = 0;
            E[index_offset + num_fluid_bodies + index * 3 + 2] = 0;
        }
    }
}
void ChFluidContainer::Project(real* gamma) {}
void ChFluidContainer::GenerateSparsity() {
    if (data_manager->num_fluid_contacts <= 0) {
        return;
    }

    LOG(INFO) << "ChConstraintFluidFluid::GenerateSparsityFluid";
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

    //    for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
    //        // D_T.reserve(index_offset + body_a, data_manager->settings.fluid.max_interactions * 3);
    //
    //        for (int i = 0; i < data_manager->host_data.c_counts_fluid_fluid[body_a]; i++) {
    //            int body_b = data_manager->host_data.neighbor_fluid_fluid[body_a * max_neighbors + i];
    //            AppendRow3(D_T, index_offset + body_a, body_offset + body_b * 3, 0);
    //        }
    //        D_T.finalize(index_offset + body_a);
    //    }
    //    // Add more entries for viscosity
    //    // Code is repeated because there are three rows per viscosity constraint
    //    if (data_manager->settings.fluid.enable_viscosity) {
    //        for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
    //            // D_T.reserve(index_offset + num_fluid_bodies + body_a * 3 + 0,
    //            //            data_manager->settings.fluid.max_interactions * 3);
    //            for (int i = 0; i < data_manager->host_data.c_counts_fluid_fluid[body_a]; i++) {
    //                int body_b = data_manager->host_data.neighbor_fluid_fluid[body_a * max_neighbors + i];
    //                AppendRow3(D_T, index_offset + num_fluid_bodies + body_a * 3 + 0, body_offset + body_b * 3, 0);
    //            }
    //            D_T.finalize(index_offset + num_fluid_bodies + body_a * 3 + 0);
    //            // D_T.reserve(index_offset + num_fluid_bodies + body_a * 3 + 1,
    //            //            data_manager->settings.fluid.max_interactions * 3);
    //            for (int i = 0; i < data_manager->host_data.c_counts_fluid_fluid[body_a]; i++) {
    //                int body_b = data_manager->host_data.neighbor_fluid_fluid[body_a * max_neighbors + i];
    //                AppendRow3(D_T, index_offset + num_fluid_bodies + body_a * 3 + 1, body_offset + body_b * 3, 0);
    //            }
    //            D_T.finalize(index_offset + num_fluid_bodies + body_a * 3 + 1);
    //            // D_T.reserve(index_offset + num_fluid_bodies + body_a * 3 + 2,
    //            //            data_manager->settings.fluid.max_interactions * 3);
    //            for (int i = 0; i < data_manager->host_data.c_counts_fluid_fluid[body_a]; i++) {
    //                int body_b = data_manager->host_data.neighbor_fluid_fluid[body_a * max_neighbors + i];
    //                AppendRow3(D_T, index_offset + num_fluid_bodies + body_a * 3 + 2, body_offset + body_b * 3, 0);
    //            }
    //            D_T.finalize(index_offset + num_fluid_bodies + body_a * 3 + 2);
    //        }
    //    }
}

void ChFluidContainer::PostSolve() {
    if (data_manager->settings.fluid.artificial_pressure == false) {
        return;
    }
    if (data_manager->settings.fluid.fluid_is_rigid == false) {
        custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;
        custom_vector<real3>& sorted_vel = data_manager->host_data.sorted_vel_3dof;
        real mass_fluid = data_manager->settings.fluid.mass;
        real inv_density = 1.0 / data_manager->settings.fluid.density;
        real h = data_manager->settings.fluid.kernel_radius;
        real k = data_manager->settings.fluid.artificial_pressure_k;
        real dq = data_manager->settings.fluid.artificial_pressure_dq;
        real n = data_manager->settings.fluid.artificial_pressure_n;
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

            data_manager->host_data.gamma[index_offset + body_a] += corr;
        }
    }
}
}  // END_OF_NAMESPACE____

/////////////////////
