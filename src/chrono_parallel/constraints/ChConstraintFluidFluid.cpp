#include "chrono_parallel/constraints/ChConstraintFluidFluid.h"
#include "chrono_parallel/constraints/ChConstraintFluidFluidUtils.h"
#include "chrono_parallel/constraints/ChConstraintUtils.h"
#include <thrust/iterator/constant_iterator.h>
#include <thrust/sort.h>

namespace chrono {
// Perform projection on the constraints associated with the fluid==========================================
// Do a standard frictionless conic projection for granular material
// No projection for constraint fluids
void ChConstraintFluidFluid::Project(real* gamma) {
    if (data_manager->settings.fluid.fluid_is_rigid == false) {
    } else {
#pragma omp parallel for
        for (int index = 0; index < num_fluid_contacts; index++) {
            real cohesion = data_manager->settings.fluid.cohesion;
            real gam = gamma[index_offset + index];
            gam = gam + cohesion;
            gam = gam < 0 ? 0 : gam - cohesion;
            gamma[index_offset + index] = gam;
        }
    }
}
// Compute the jacobians for rigid "fluid" bodies===========================================================

void ChConstraintFluidFluid::Build_D_Rigid() {
    //    LOG(INFO) << "ChConstraintFluidFluid::Build_D_Rigid";
    //
    //    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    //    custom_vector<real3>& pos_fluid = data_manager->host_data.pos_fluid;
    //    real step_size = data_manager->settings.step_size;
    //    custom_vector<int2>& bids = data_manager->host_data.bids_fluid_fluid;
    //
    //    if (num_fluid_contacts <= 0) {
    //        return;
    //    }
    //
    //#pragma omp parallel for
    //    for (int index = 0; index < num_fluid_contacts; index++) {
    //        int2 bid = bids[index];
    //        real3 n = normalize(pos_fluid[bid.y] - pos_fluid[bid.x]);
    //        D_T(index_offset + index, body_offset + bid.x * 3 + 0) = -n.x;
    //        D_T(index_offset + index, body_offset + bid.x * 3 + 1) = -n.y;
    //        D_T(index_offset + index, body_offset + bid.x * 3 + 2) = -n.z;
    //
    //        D_T(index_offset + index, body_offset + bid.y * 3 + 0) = n.x;
    //        D_T(index_offset + index, body_offset + bid.y * 3 + 1) = n.y;
    //        D_T(index_offset + index, body_offset + bid.y * 3 + 2) = n.z;
    //    }
}

// =========================================================================================================
// FLUID CODE:

// Compute the fluid density================================================================================

void ChConstraintFluidFluid::Density_Fluid() {
    LOG(INFO) << "ChConstraintFluidFluid::Density_Fluid";
    real mass_fluid = data_manager->settings.fluid.mass;
    real h = data_manager->settings.fluid.kernel_radius;
    custom_vector<real>& density = data_manager->host_data.den_fluid;
    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_fluid;

    const real h_3 = h * h * h;
    const real h_6 = h_3 * h_3;
    const real h_9 = h_3 * h_3 * h_3;
    const real KGSPIKY = 315.0 / (64.0 * F_PI * h_9);

#pragma omp parallel for
    for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
        real dens = 0;
        real3 pos_p = sorted_pos[body_a];
        for (int i = 0; i < data_manager->host_data.c_counts_fluid_fluid[body_a]; i++) {
            int body_b = data_manager->host_data.neighbor_fluid_fluid[body_a * max_neighbors + i];
            if (body_a == body_b) {
                dens += mass_fluid * KGSPIKY * h_6;
                continue;
            }
            real3 xij = pos_p - sorted_pos[body_b];
            real dist = Length(xij);

            dens += mass_fluid * KGSPIKY * pow((h * h - dist * dist), 3);
        }
        density[body_a] = dens;
    }
}
// void ChConstraintFluidFluid::Solve_Density(real volume, real tolerance) {
//  real density_fluid = data_manager->settings.fluid.density;
//  custom_vector<real>& density = data_manager->host_data.den_fluid;
//  density.resize(num_fluid_bodies);
//  uint num_fluid_bodies = data_manager->num_fluid_bodies;
//
//  std::cout << "avg_Density: " << avg_density << std::endl;
//
//  real& mass = data_manager->settings.fluid.mass;
//  real density_fluid = data_manager->settings.fluid.density;
//
//  real vol = volume / num_fluid_bodies;
//  mass = density_fluid * vol;
//
//  Density_Fluid();
//  Normalize_Density_Fluid();
//
//  real avg_density = Thrust_Total(data_manager->host_data.den_fluid) / num_fluid_bodies;
//
//  if (avg_density - density_fluid < tolerance){
//
//  }
//}
void ChConstraintFluidFluid::Normalize_Density_Fluid() {
    //    real h = data_manager->settings.fluid.kernel_radius;
    //    custom_vector<real3>& pos = data_manager->host_data.pos_fluid;
    //    real mass_fluid = data_manager->settings.fluid.mass;
    //    custom_vector<real>& density = data_manager->host_data.den_fluid;
    //    real density_fluid = data_manager->settings.fluid.density;
    //    real inv_density = 1.0 / density_fluid;
    //    real mass_over_density = mass_fluid * inv_density;
    //    const real h_3 = h * h * h;
    //    const real h_6 = h_3 * h_3;
    //    const real h_9 = h_3 * h_3 * h_3;
    //    const real KPOLY6 = 315.0 / (64.0 * F_PI * h_9);
    //
    //#pragma omp parallel for
    //    for (int i = 0; i < last_body; i++) {
    //        int2 bid;
    //        uint start = fluid_start_index[i], end = fluid_start_index[i + 1];
    //        bid.x = fluid_contact_idA_start[i];
    //        real3 posa = pos[bid.x];
    //        real dens = 0;
    //        for (int index = start; index < end; index++) {
    //            bid.y = fluid_contact_idB[index];
    //            if (bid.x == bid.y) {
    //                dens += mass_fluid / density[bid.y] * KPOLY6 * h_6;
    //                continue;
    //            }
    //            real3 xij = (posa - pos[bid.y]);
    //            real dist = Length(xij);
    //
    //            dens += (mass_fluid / density[bid.y]) * KPOLY6 * pow((h * h - dist * dist), 3);
    //        }
    //        density[bid.x] = density[bid.x] / dens;
    //    }
}
template <typename T>
static void inline CheckSetRow3(T& D, const int row, const int col, const real x, const real y, const real z) {
    if (x != 0) {
        D.set(row, col + 0, x);
    }
    if (y != 0) {
        D.set(row, col + 1, y);
    }
    if (z != 0) {
        D.set(row, col + 2, z);
    }
}

void ChConstraintFluidFluid::Build_D_Fluid() {
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

    custom_vector<real>& density = data_manager->host_data.den_fluid;
    // custom_vector<real3>& vel = data_manager->host_data.vel_fluid;
    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_fluid;

    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

    //=======COMPUTE DENSITY OF FLUID
    density.resize(num_fluid_bodies);
    // den_con.resize(num_fluid_contacts);
    Density_Fluid();
    const real h_2 = h * h;
    const real h_3 = h * h * h;
    const real h_6 = h_3 * h_3;
    const real h_9 = h_3 * h_3 * h_3;
    const real KGSPIKY = -45.0 / (F_PI * h_6);
    const real KPOLY6 = 315.0 / (64.0 * F_PI * h_9);
    real visca = data_manager->settings.fluid.viscosity;
    real viscb = data_manager->settings.fluid.viscosity;
    const real mass_2 = mass * mass;
    const real eta_2 = eta * eta;

    #pragma omp parallel for
    for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
        real dens = 0;
        real3 diag = real3(0);
        real3 pos_p = sorted_pos[body_a];
        for (int i = 0; i < data_manager->host_data.c_counts_fluid_fluid[body_a]; i++) {
            int body_b = data_manager->host_data.neighbor_fluid_fluid[body_a * max_neighbors + i];
            if (body_a == body_b) {
                dens += mass / density[body_b] * KPOLY6 * h_6;
                continue;
            }
            real3 xij = pos_p - sorted_pos[body_b];
            real dist = Length(xij);
            real3 off_diag = mass_over_density * KGSPIKY * pow(h - dist, 2) * xij;
            CheckSetRow3(D_T, index_offset + body_a, body_offset + body_b * 3, off_diag.x, off_diag.y, off_diag.z);
            diag += off_diag;
            dens += (mass / density[body_b]) * KPOLY6 * pow((h * h - dist * dist), 3);
        }
        density[body_a] = density[body_a] / dens;
        CheckSetRow3(D_T, index_offset + body_a, body_offset + body_a * 3, -diag.x, -diag.y, -diag.z);
    }

    if (data_manager->settings.fluid.enable_viscosity) {
        #pragma omp parallel for
        for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
            // printf("a: %d ", body_a);
            Mat33 visc_mat;
            real3 pos_p = sorted_pos[body_a];
            for (int i = 0; i < data_manager->host_data.c_counts_fluid_fluid[body_a]; i++) {
                int body_b = data_manager->host_data.neighbor_fluid_fluid[body_a * max_neighbors + i];
                // printf("b: %d ", body_b);
                if (body_a == body_b) {
                    continue;
                }
                real3 xij = (pos_p - sorted_pos[body_b]);
                real dist = Length(xij);
                real density_a = density[body_a];
                real density_b = density[body_b];

                real part_a = (8.0 / (density_a + density_b));
                real part_b = visca + viscb;  //(visca / density_a + viscb / density_b);  // /
                real part_c = 1.0 / (h * ((dist * dist / h_2) + eta_2));
                real scalar = -mass_2 * part_a * part_b * part_c;
                real kernel = KGSPIKY * pow(h - dist, 2);
                // kernel = 45.0 / (F_PI * h_6) * (h - dist);;
                Mat33 matrix = OuterProduct(xij, kernel * xij) * scalar;

                CheckSetRow3(D_T, index_offset + num_fluid_bodies + body_a * 3 + 0, body_offset + body_b * 3,
                             matrix.cols[0].x, matrix.cols[1].x, matrix.cols[2].x);
                CheckSetRow3(D_T, index_offset + num_fluid_bodies + body_a * 3 + 1, body_offset + body_b * 3,
                             matrix.cols[0].y, matrix.cols[1].y, matrix.cols[2].y);
                CheckSetRow3(D_T, index_offset + num_fluid_bodies + body_a * 3 + 2, body_offset + body_b * 3,
                             matrix.cols[0].z, matrix.cols[1].z, matrix.cols[2].z);

                visc_mat = visc_mat + matrix;
            }
            // printf("\n");
            CheckSetRow3(D_T, index_offset + num_fluid_bodies + body_a * 3 + 0, body_offset + body_a * 3,
                         -visc_mat.cols[0].x, -visc_mat.cols[1].x, -visc_mat.cols[2].x);
            CheckSetRow3(D_T, index_offset + num_fluid_bodies + body_a * 3 + 1, body_offset + body_a * 3,
                         -visc_mat.cols[0].y, -visc_mat.cols[1].y, -visc_mat.cols[2].y);
            CheckSetRow3(D_T, index_offset + num_fluid_bodies + body_a * 3 + 2, body_offset + body_a * 3,
                         -visc_mat.cols[0].z, -visc_mat.cols[1].z, -visc_mat.cols[2].z);
        }
    }
    LOG(INFO) << "ChConstraintFluidFluid::JACOBIAN OF FLUID";
}
void ChConstraintFluidFluid::Build_D() {
    if (data_manager->settings.fluid.fluid_is_rigid == false) {
        Build_D_Fluid();
    } else {
        Build_D_Rigid();
    }
}

void ChConstraintFluidFluid::Build_b() {
    real step_size = data_manager->settings.step_size;
    if (data_manager->settings.fluid.fluid_is_rigid == false) {
        SubVectorType b_sub = blaze::subvector(data_manager->host_data.b, index_offset, num_fluid_bodies);
        SubVectorType v_sub = blaze::subvector(data_manager->host_data.v, body_offset, num_fluid_bodies * 3);
        CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
        SubMatrixType D_T_sub = submatrix(D_T, index_offset, body_offset, num_fluid_bodies, num_fluid_bodies * 3);

        real density_fluid = data_manager->settings.fluid.density;
        custom_vector<real>& density = data_manager->host_data.den_fluid;
        DynamicVector<real> g(num_fluid_bodies);
        real epsilon = data_manager->settings.fluid.epsilon;
        real tau = data_manager->settings.fluid.tau;
        real h = data_manager->settings.fluid.kernel_radius;
        real zeta = 1.0 / (1.0 + 4.0 * tau / h);

#pragma omp parallel for
        for (int index = 0; index < num_fluid_bodies; index++) {
            b_sub[index] = -(density[index] / data_manager->settings.fluid.density - 1.0);
        }
    }
    //    else {
    //        SubVectorType b_sub = blaze::subvector(data_manager->host_data.b, index_offset, num_fluid_contacts);
    //        custom_vector<real3>& pos = data_manager->host_data.pos_fluid;
    //        custom_vector<int2>& bids = data_manager->host_data.bids_fluid_fluid;
    //        real h = data_manager->settings.fluid.kernel_radius;
    //        real inv_hpa = 1.0 / (step_size + data_manager->settings.solver.alpha);
    //        real inv_hhpa = 1.0 / (step_size * (step_size + data_manager->settings.solver.alpha));
    //#pragma omp parallel for
    //        for (int index = 0; index < num_fluid_contacts; index++) {
    //            int2 bid = bids[index];
    //            real3 n = (pos[bid.y] - pos[bid.x]);
    //            real depth = length(n) - h * 2;
    //            real bi = 0;
    //            if (data_manager->settings.solver.alpha > 0) {
    //                bi = inv_hpa * depth;
    //            } else {
    //                if (data_manager->settings.solver.contact_recovery_speed < 0) {
    //                    bi = real(1.0) / step_size * depth;
    //                } else {
    //                    bi = std::max(real(1.0) / step_size * depth,
    //                    -data_manager->settings.fluid.contact_recovery_speed);
    //                }
    //            }
    //            b_sub[index] = bi;
    //        }
    //    }
}
void ChConstraintFluidFluid::Build_E() {
    DynamicVector<real>& E = data_manager->host_data.E;
    real step_size = data_manager->settings.step_size;
    if (data_manager->settings.fluid.fluid_is_rigid == false) {
        if (num_fluid_bodies <= 0) {
            return;
        }

        real epsilon = data_manager->settings.fluid.epsilon;
        real tau = data_manager->settings.fluid.tau;
        real h = data_manager->settings.fluid.kernel_radius;

        real zeta = 1.0 / (1.0 + 4.0 * tau / step_size);
        real compliance = 4.0 / (step_size * step_size) * (epsilon * zeta);
#pragma omp parallel for
        for (int index = 0; index < num_fluid_bodies; index++) {
            E[index_offset + index] = compliance;

            E[index_offset + num_fluid_bodies + index * 3 + 0] = 0;
            E[index_offset + num_fluid_bodies + index * 3 + 1] = 0;
            E[index_offset + num_fluid_bodies + index * 3 + 2] = 0;
        }

        //#pragma omp parallel for
        //    for (int index = 0; index < num_fluid_bodies; index++) {
        //      E[index_offset + num_fluid_bodies + index * 3 + 0] = 0;  // 1.0/step_size * 1;    // compliance;
        //      E[index_offset + num_fluid_bodies + index * 3 + 1] = 0;  // 1.0/step_size * 1;    // compliance;
        //      E[index_offset + num_fluid_bodies + index * 3 + 2] = 0;  // 1.0/step_size * 1;    // compliance;
        //    }

    } else {
        if (num_fluid_contacts <= 0) {
            return;
        }
        real compliance = data_manager->settings.fluid.compliance;
        real inv_hhpa = 1.0 / (step_size * (step_size + data_manager->settings.solver.alpha));
#pragma omp parallel for
        for (int index = 0; index < num_fluid_contacts; index++) {
            E[index_offset + index] = inv_hhpa * compliance;
        }
    }
}
void ChConstraintFluidFluid::GenerateSparsityRigid() {
    LOG(INFO) << "ChConstraintFluidFluid::GenerateSparsityRigid";

    // custom_vector<int2>& bids = data_manager->host_data.bids_fluid_fluid;

    //    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    //    for (int index = 0; index < num_fluid_contacts; index++) {
    //        int2 bid = bids[index];
    //
    //        D_T.append(index_offset + index, body_offset + bid.x * 3 + 0, 1);
    //        D_T.append(index_offset + index, body_offset + bid.x * 3 + 1, 1);
    //        D_T.append(index_offset + index, body_offset + bid.x * 3 + 2, 1);
    //
    //        D_T.append(index_offset + index, body_offset + bid.y * 3 + 0, 1);
    //        D_T.append(index_offset + index, body_offset + bid.y * 3 + 1, 1);
    //        D_T.append(index_offset + index, body_offset + bid.y * 3 + 2, 1);
    //        D_T.finalize(index_offset + index);
    //    }
}
void ChConstraintFluidFluid::GenerateSparsityFluid() {
    LOG(INFO) << "ChConstraintFluidFluid::GenerateSparsityFluid";
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

    for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
        for (int i = 0; i < data_manager->host_data.c_counts_fluid_fluid[body_a]; i++) {
            int body_b = data_manager->host_data.neighbor_fluid_fluid[body_a * max_neighbors + i];
            // printf("index: [%d, %d]\n", body_a, body_b);

            D_T.append(index_offset + body_a, body_offset + body_b * 3 + 0, 1);
            D_T.append(index_offset + body_a, body_offset + body_b * 3 + 1, 1);
            D_T.append(index_offset + body_a, body_offset + body_b * 3 + 2, 1);
        }
        D_T.finalize(index_offset + body_a);
    }
    // Add more entries for viscosity
    // Code is repeated because there are three rows per viscosity constraint
    if (data_manager->settings.fluid.enable_viscosity) {
        for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
            for (int i = 0; i < data_manager->host_data.c_counts_fluid_fluid[body_a]; i++) {
                int body_b = data_manager->host_data.neighbor_fluid_fluid[body_a * max_neighbors + i];
                D_T.append(index_offset + num_fluid_bodies + body_a * 3 + 0, body_offset + body_b * 3 + 0, 1);
                D_T.append(index_offset + num_fluid_bodies + body_a * 3 + 0, body_offset + body_b * 3 + 1, 1);
                D_T.append(index_offset + num_fluid_bodies + body_a * 3 + 0, body_offset + body_b * 3 + 2, 1);
            }
            D_T.finalize(index_offset + num_fluid_bodies + body_a * 3 + 0);

            for (int i = 0; i < data_manager->host_data.c_counts_fluid_fluid[body_a]; i++) {
                int body_b = data_manager->host_data.neighbor_fluid_fluid[body_a * max_neighbors + i];
                D_T.append(index_offset + num_fluid_bodies + body_a * 3 + 1, body_offset + body_b * 3 + 0, 1);
                D_T.append(index_offset + num_fluid_bodies + body_a * 3 + 1, body_offset + body_b * 3 + 1, 1);
                D_T.append(index_offset + num_fluid_bodies + body_a * 3 + 1, body_offset + body_b * 3 + 2, 1);
            }
            D_T.finalize(index_offset + num_fluid_bodies + body_a * 3 + 1);

            for (int i = 0; i < data_manager->host_data.c_counts_fluid_fluid[body_a]; i++) {
                int body_b = data_manager->host_data.neighbor_fluid_fluid[body_a * max_neighbors + i];
                D_T.append(index_offset + num_fluid_bodies + body_a * 3 + 2, body_offset + body_b * 3 + 0, 1);
                D_T.append(index_offset + num_fluid_bodies + body_a * 3 + 2, body_offset + body_b * 3 + 1, 1);
                D_T.append(index_offset + num_fluid_bodies + body_a * 3 + 2, body_offset + body_b * 3 + 2, 1);
            }
            D_T.finalize(index_offset + num_fluid_bodies + body_a * 3 + 2);
        }
    }
}
void ChConstraintFluidFluid::GenerateSparsity() {
    if (data_manager->num_fluid_contacts <= 0) {
        return;
    }

    if (data_manager->settings.fluid.fluid_is_rigid == false) {
        //        real t1, t2;
        //        ChTimer<> temp;
        //        temp.start();
        //        // DetermineNeighbors();
        //        temp.stop();
        //        t1 = temp();
        //        temp.reset();
        //        temp.start();
        GenerateSparsityFluid();
        //        temp.stop();
        //        t2 = temp();
        //        temp.reset();
        // printf("Neigh, sparse [%f, %f]\n", t1, t2);
    } else {
        GenerateSparsityRigid();
    }
}

void ChConstraintFluidFluid::DetermineNeighbors() {
    if (num_fluid_bodies == 0) {
        return;
    }
}
void ChConstraintFluidFluid::ArtificialPressure() {
    if (data_manager->settings.fluid.artificial_pressure == false) {
        return;
    }
    if (data_manager->settings.fluid.fluid_is_rigid == false) {
        custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_fluid;
        real mass_fluid = data_manager->settings.fluid.mass;
        real h = data_manager->settings.fluid.kernel_radius;
        real k = data_manager->settings.fluid.artificial_pressure_k;
        real dq = data_manager->settings.fluid.artificial_pressure_dq;
        real n = data_manager->settings.fluid.artificial_pressure_n;
#pragma omp parallel for
        for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
            real corr = 0;
            for (int i = 0; i < data_manager->host_data.c_counts_fluid_fluid[body_a]; i++) {
                int body_b = data_manager->host_data.neighbor_fluid_fluid[body_a * max_neighbors + i];

                real3 xij = (sorted_pos[body_a] - sorted_pos[body_b]);
                real dist = Length(xij);
                corr += k * pow(KERNEL(dist, h) / KERNEL(dq, h), n);
            }
            data_manager->host_data.gamma[index_offset + body_a] += corr;
        }
    }
}
void ChConstraintFluidFluid::Dx(const DynamicVector<real>& x, DynamicVector<real>& output) {
    //    custom_vector<int>& neighbor_fluid_fluid = data_manager->host_data.neighbor_fluid_fluid;
    //    custom_vector<int>& c_counts_fluid_fluid = data_manager->host_data.c_counts_fluid_fluid;
    //    custom_vector<int>& particle_indices_fluid = data_manager->host_data.particle_indices_fluid;
    //    custom_vector<real3>& pos = data_manager->host_data.pos_fluid;
    //
    //    real viscosity = data_manager->settings.fluid.viscosity;
    //    real mass = data_manager->settings.fluid.mass;
    //    real h = data_manager->settings.fluid.kernel_radius;
    //    real envel = data_manager->settings.collision.collision_envelope;
    //    real density_fluid = data_manager->settings.fluid.density;
    //    real inv_density = 1.0 / density_fluid;
    //    real mass_over_density = mass * inv_density;
    //    real eta = .01;
    //
    //    const real h_3 = h * h * h;
    //    const real h_6 = h_3 * h_3;
    //    const real h_9 = h_3 * h_3 * h_3;
    //    const real KGSPIKY = 315.0 / (64.0 * F_PI * h_9);
    //#pragma omp parallel
    //    for (int a = 0; a < num_fluid_bodies; a++) {
    //        int p = particle_indices_fluid[a];
    //        real3 diag = real3(0, 0, 0);
    //        real3 pos_p = pos[p];
    //        real3 res = real3(0, 0, 0);
    //        for (int i = 0; i < c_counts_fluid_fluid[a]; i++) {
    //            int b = neighbor_fluid_fluid[a * max_neighbors + i];
    //
    //            int q = particle_indices_fluid[b];
    //            real4 xij_dist = xij_dist_fluid_fluid[a * max_neighbors + i];
    //
    //            if (xij_dist.w == 0) {
    //                continue;
    //            }
    //            real3 off_diag =
    //                mass_over_density * KGSPIKY * pow(h - xij_dist.w, 2) * real3(xij_dist.x, xij_dist.y, xij_dist.z);
    //            diag += off_diag;
    //            res += off_diag * x[q];
    //        }
    //        output[p * 3 + 0] = diag.x * x[p] + res.x;
    //        output[p * 3 + 1] = diag.y * x[p] + res.y;
    //        output[p * 3 + 2] = diag.z * x[p] + res.z;
    //    }
}
void ChConstraintFluidFluid::D_Tx(const DynamicVector<real>& x, DynamicVector<real>& output) {
    //    custom_vector<int>& neighbor_fluid_fluid = data_manager->host_data.neighbor_fluid_fluid;
    //    custom_vector<int>& c_counts_fluid_fluid = data_manager->host_data.c_counts_fluid_fluid;
    //    custom_vector<int>& particle_indices_fluid = data_manager->host_data.particle_indices_fluid;
    //    custom_vector<real3>& pos = data_manager->host_data.pos_fluid;
    //
    //    real viscosity = data_manager->settings.fluid.viscosity;
    //    real mass = data_manager->settings.fluid.mass;
    //    real h = data_manager->settings.fluid.kernel_radius;
    //    real envel = data_manager->settings.collision.collision_envelope;
    //    real density_fluid = data_manager->settings.fluid.density;
    //    real inv_density = 1.0 / density_fluid;
    //    real mass_over_density = mass * inv_density;
    //    real eta = .01;
    //
    //    const real h_3 = h * h * h;
    //    const real h_6 = h_3 * h_3;
    //    const real h_9 = h_3 * h_3 * h_3;
    //    const real KGSPIKY = 315.0 / (64.0 * F_PI * h_9);
    //#pragma omp parallel
    //    for (int a = 0; a < num_fluid_bodies; a++) {
    //        int p = particle_indices_fluid[a];
    //        real3 diag = real3(0, 0, 0);
    //        // real3 pos_p = pos[p];
    //        real res = 0;
    //        for (int i = 0; i < c_counts_fluid_fluid[a]; i++) {
    //            real4 xij_dist = xij_dist_fluid_fluid[a * max_neighbors + i];
    //            int b = neighbor_fluid_fluid[a * max_neighbors + i];
    //            int q = particle_indices_fluid[b];
    //            if (xij_dist.w == 0) {
    //                continue;
    //            }
    //            real3 off_diag =
    //                mass_over_density * KGSPIKY * pow(h - xij_dist.w, 2) * real3(xij_dist.x, xij_dist.y, xij_dist.z);
    //            diag += off_diag;
    //
    //            res += off_diag.x * x[q * 3 + 0] + off_diag.y * x[q * 3 + 1] + off_diag.z * x[q * 3 + 2];
    //        }
    //        output[p] = res + diag.x * x[p * 3 + 0] + diag.y * x[p * 3 + 1] + diag.z * x[p * 3 + 2];
    //    }
}
}
