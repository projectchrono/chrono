#include "chrono_parallel/constraints/ChConstraintFluidFluid.h"
#include "chrono_parallel/constraints/ChConstraintFluidFluidUtils.h"
#include "chrono_parallel/constraints/ChConstraintUtils.h"
#include <thrust/iterator/constant_iterator.h>
#include <thrust/sort.h>

namespace chrono {
// Perform projection on the constraints associated with the fluid==========================================
// Do a standard frictionless conic projection for granular material
// No projection for constraint fluids
void ChConstraintFluidFluid::Project(real* gamma) {}

void ChConstraintFluidFluid::Initialize() {
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

// Compute the fluid density================================================================================
void ChConstraintFluidFluid::Density_Fluid() {
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
void ChConstraintFluidFluid::Normalize_Density_Fluid() {
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

void ChConstraintFluidFluid::Build_D() {
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

void ChConstraintFluidFluid::Build_b() {
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
void ChConstraintFluidFluid::Build_E() {
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

void ChConstraintFluidFluid::GenerateSparsity() {
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

void ChConstraintFluidFluid::ArtificialPressure() {
    if (data_manager->settings.fluid.artificial_pressure == false) {
        return;
    }
    if (data_manager->settings.fluid.fluid_is_rigid == false) {
        custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;
        custom_vector<real3>& sorted_vel = data_manager->host_data.sorted_vel_3dof;
        real mass_fluid = data_manager->settings.fluid.mass;
        real inv_density = 1.0 / data_manager->settings.fluid.density;
        real vorticity_confinement = data_manager->settings.fluid.vorticity_confinement;
        real h = data_manager->settings.fluid.kernel_radius;
        real k = data_manager->settings.fluid.artificial_pressure_k;
        real dq = data_manager->settings.fluid.artificial_pressure_dq;
        real n = data_manager->settings.fluid.artificial_pressure_n;
        real dt = data_manager->settings.step_size;
//        custom_vector<real4> curl(num_fluid_bodies);
//        if (vorticity_confinement > 0 && data_manager->settings.fluid.enable_viscosity) {
//#pragma omp parallel for
//            for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
//                real3 vorticity(0);
//                real3 pos_a = sorted_pos[body_a];
//                real3 vel_a = sorted_vel[body_a];
//                for (int i = 0; i < data_manager->host_data.c_counts_fluid_fluid[body_a]; i++) {
//                    int body_b = data_manager->host_data.neighbor_fluid_fluid[body_a * max_neighbors + i];
//                    if (body_a == body_b) {
//                        continue;
//                    }
//                    real3 xij = (pos_a - sorted_pos[body_b]);
//                    real3 vij = (vel_a - sorted_vel[body_b]);
//
//                    real dist = Length(xij);
//                    vorticity += Cross(vij, GRAD_KERNEL(xij, dist, h));
//                }
//                curl[body_a] = real4(vorticity.x, vorticity.y, vorticity.z, Length(vorticity));
//            }
//        }
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
                //                if (body_a != body_b) {
                //                    vorticity_grad += curl[body_b].w * GRAD_KERNEL(xij, dist, h);
                //                }
            }
            //            if (vorticity_confinement > 0 && data_manager->settings.fluid.enable_viscosity) {
            //                real3 curl_a = real3(curl[body_a].x, curl[body_a].y, curl[body_a].z);
            //                real3 corr_vort = dt * inv_density * vorticity_confinement *
            //                Cross(Normalize(vorticity_grad), curl_a);
            //                data_manager->host_data.gamma[index_offset + num_fluid_bodies + body_a * 3 + 0] +=
            //                -corr_vort.x;
            //                data_manager->host_data.gamma[index_offset + num_fluid_bodies + body_a * 3 + 1] +=
            //                -corr_vort.y;
            //                data_manager->host_data.gamma[index_offset + num_fluid_bodies + body_a * 3 + 2] +=
            //                -corr_vort.z;
            //            }
            data_manager->host_data.gamma[index_offset + body_a] += corr;
        }
    }
}

void ChConstraintFluidFluid::XSPHViscosity() {
    return;
    if (data_manager->settings.fluid.fluid_is_rigid == false) {
        custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;
        custom_vector<real3>& sorted_vel = data_manager->host_data.sorted_vel_3dof;
        real mass_fluid = data_manager->settings.fluid.mass;
        real inv_density = 1.0 / data_manager->settings.fluid.density;
        real vorticity_confinement = data_manager->settings.fluid.vorticity_confinement;
        real h = data_manager->settings.fluid.kernel_radius;
        real k = data_manager->settings.fluid.artificial_pressure_k;
        real dq = data_manager->settings.fluid.artificial_pressure_dq;
        real n = data_manager->settings.fluid.artificial_pressure_n;
        real dt = data_manager->settings.step_size;
        real c = .01;

#pragma omp parallel for
        for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
            real3 corr = real3(0);
            real3 pos_a = sorted_pos[body_a];
            real3 vel_a, vel_b;
            vel_a = sorted_vel[body_a];
            //			vela.x = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + body_a * 3 + 0];
            //			vela.y = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + body_a * 3 + 1];
            //			vela.z = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + body_a * 3 + 2];

            for (int i = 0; i < data_manager->host_data.c_counts_3dof_3dof[body_a]; i++) {
                int body_b = data_manager->host_data.neighbor_3dof_3dof[body_a * max_neighbors + i];
                //				if(body_a==body_b) {
                //					continue;
                //				}

                //				velb.x = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + body_b * 3 + 0];
                //				velb.y = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + body_b * 3 + 1];
                //				velb.z = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + body_b * 3 + 2];

                real3 xij = (pos_a - sorted_pos[body_b]);
                real3 vij = (vel_a - sorted_vel[body_b]);

                real dist = Length(xij);

                corr += c * vij * KERNEL(dist, h);
            }

            data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + body_a * 3 + 0] += corr.x;
            data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + body_a * 3 + 1] += corr.y;
            data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + body_a * 3 + 2] += corr.z;
            // data_manager->host_data.gamma[index_offset + body_a] += corr;
        }
    }
}

void ChConstraintFluidFluid::Dx(const DynamicVector<real>& x, DynamicVector<real>& output) {
    custom_vector<int>& neighbor_fluid_fluid = data_manager->host_data.neighbor_3dof_3dof;
    custom_vector<int>& c_counts_fluid_fluid = data_manager->host_data.c_counts_3dof_3dof;
    custom_vector<int>& particle_indices_fluid = data_manager->host_data.particle_indices_3dof;
    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;

    real viscosity = data_manager->settings.fluid.viscosity;
    real mass = data_manager->settings.fluid.mass;
    real h = data_manager->settings.fluid.kernel_radius;
    real envel = data_manager->settings.collision.collision_envelope;
    real density_fluid = data_manager->settings.fluid.density;
    real inv_density = 1.0 / density_fluid;
    real mass_over_density = mass * inv_density;
    real eta = .01;

#pragma omp parallel
    for (int p = 0; p < num_fluid_bodies; p++) {
        real3 diag = real3(0);
        real3 pos_p = sorted_pos[p];
        real3 res = real3(0);
        for (int i = 0; i < c_counts_fluid_fluid[p]; i++) {
            int q = neighbor_fluid_fluid[p * max_neighbors + i];
            if (p == q) {
                continue;
            }
            real3 pos_q = sorted_pos[p];
            real3 xij = pos_p - pos_q;
            real dist = Length(xij);
            real3 off_diag = mass_over_density * KGSPIKY * xij;
            diag += off_diag;
            res += off_diag * x[q];
        }
        output[p * 3 + 0] = diag.x * x[p] + res.x;
        output[p * 3 + 1] = diag.y * x[p] + res.y;
        output[p * 3 + 2] = diag.z * x[p] + res.z;
    }
}
void ChConstraintFluidFluid::D_Tx(const DynamicVector<real>& x, DynamicVector<real>& output) {
    custom_vector<int>& neighbor_fluid_fluid = data_manager->host_data.neighbor_3dof_3dof;
    custom_vector<int>& c_counts_fluid_fluid = data_manager->host_data.c_counts_3dof_3dof;
    custom_vector<int>& particle_indices_fluid = data_manager->host_data.particle_indices_3dof;
    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;

    real viscosity = data_manager->settings.fluid.viscosity;
    real mass = data_manager->settings.fluid.mass;
    real h = data_manager->settings.fluid.kernel_radius;
    real envel = data_manager->settings.collision.collision_envelope;
    real density_fluid = data_manager->settings.fluid.density;
    real inv_density = 1.0 / density_fluid;
    real mass_over_density = mass * inv_density;
    real eta = .01;

#pragma omp parallel
    for (int p = 0; p < num_fluid_bodies; p++) {
        real3 diag = real3(0, 0, 0);
        real3 pos_p = sorted_pos[p];
        real res = 0;
        for (int i = 0; i < c_counts_fluid_fluid[p]; i++) {
            int q = neighbor_fluid_fluid[p * max_neighbors + i];
            if (p == q) {
                continue;
            }
            real3 pos_q = sorted_pos[q];
            real3 xij = pos_p - pos_q;
            real dist = Length(xij);
            real3 off_diag = mass_over_density * KGSPIKY * xij;
            diag += off_diag;

            res += off_diag.x * x[q * 3 + 0] + off_diag.y * x[q * 3 + 1] + off_diag.z * x[q * 3 + 2];
        }
        output[p] = res + diag.x * x[p * 3 + 0] + diag.y * x[p * 3 + 1] + diag.z * x[p * 3 + 2];
    }
}
}
