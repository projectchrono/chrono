#include "chrono_parallel/constraints/ChConstraintFluidFluid.h"
#include "chrono_parallel/constraints/ChConstraintFluidFluidUtils.h"
#include <thrust/iterator/constant_iterator.h>

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
  LOG(INFO) << "ChConstraintFluidFluid::Build_D_Rigid";

  CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
  host_vector<real3>& pos_fluid = data_manager->host_data.pos_fluid;
  real step_size = data_manager->settings.step_size;
  custom_vector<int2>& bids = data_manager->host_data.bids_fluid_fluid;

  if (num_fluid_contacts <= 0) {
    return;
  }

#pragma omp paralle for
  for (int index = 0; index < num_fluid_contacts; index++) {
    int2 bid = bids[index];
    real3 n = normalize(pos_fluid[bid.y] - pos_fluid[bid.x]);
    D_T(index_offset + index, body_offset + bid.x * 3 + 0) = -n.x;
    D_T(index_offset + index, body_offset + bid.x * 3 + 1) = -n.y;
    D_T(index_offset + index, body_offset + bid.x * 3 + 2) = -n.z;

    D_T(index_offset + index, body_offset + bid.y * 3 + 0) = n.x;
    D_T(index_offset + index, body_offset + bid.y * 3 + 1) = n.y;
    D_T(index_offset + index, body_offset + bid.y * 3 + 2) = n.z;
  }
}

// =========================================================================================================
// FLUID CODE:

// Compute the fluid density================================================================================

void ChConstraintFluidFluid::Density_Fluid() {
  LOG(INFO) << "ChConstraintFluidFluid::Density_Fluid";
  real mass_fluid = data_manager->settings.fluid.mass;
  real h = data_manager->settings.fluid.kernel_radius;
  host_vector<real3>& vel = data_manager->host_data.vel_fluid;
  host_vector<real3>& pos = data_manager->host_data.pos_fluid;
  host_vector<real>& density = data_manager->host_data.den_fluid;

  const real h_3 = h * h * h;
  const real h_6 = h_3 * h_3;
  const real h_9 = h_3 * h_3 * h_3;
  const real KGSPIKY = 315.0 / (64.0 * F_PI * h_9);

#pragma omp parallel for
  for (int i = 0; i < last_body; i++) {
    uint start = fluid_start_index[i], end = fluid_start_index[i + 1];
    int2 bid;
    bid.x = fluid_contact_idA_start[i];
    real dens = 0;
    for (int index = start; index < end; index++) {
      bid.y = fluid_contact_idB[index];
      if (bid.x == bid.y) {
        dens += mass_fluid * KGSPIKY * h_6;
        continue;
      }
      real3 xij = (pos[bid.x] - pos[bid.y]);
      real dist = length(xij);
      dens += mass_fluid * KGSPIKY * pow((h * h - dist * dist), 3);
    }
    density[bid.x] = dens;
  }
}
// void ChConstraintFluidFluid::Solve_Density(real volume, real tolerance) {
//  real density_fluid = data_manager->settings.fluid.density;
//  host_vector<real>& density = data_manager->host_data.den_fluid;
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
  real h = data_manager->settings.fluid.kernel_radius;
  host_vector<real3>& pos = data_manager->host_data.pos_fluid;
  real mass_fluid = data_manager->settings.fluid.mass;
  host_vector<real>& density = data_manager->host_data.den_fluid;
  real density_fluid = data_manager->settings.fluid.density;
  real inv_density = 1.0 / density_fluid;
  real mass_over_density = mass_fluid * inv_density;
  const real h_3 = h * h * h;
  const real h_6 = h_3 * h_3;
  const real h_9 = h_3 * h_3 * h_3;
  const real KPOLY6 = 315.0 / (64.0 * F_PI * h_9);

#pragma omp parallel for
  for (int i = 0; i < last_body; i++) {
    int2 bid;
    uint start = fluid_start_index[i], end = fluid_start_index[i + 1];
    bid.x = fluid_contact_idA_start[i];
    real3 posa = pos[bid.x];
    real dens = 0;
    for (int index = start; index < end; index++) {
      bid.y = fluid_contact_idB[index];
      if (bid.x == bid.y) {
        dens += mass_fluid / density[bid.y] * KPOLY6 * h_6;
        continue;
      }
      real3 xij = (posa - pos[bid.y]);
      real dist = length(xij);

      dens += (mass_fluid / density[bid.y]) * KPOLY6 * pow((h * h - dist * dist), 3);
    }
    density[bid.x] = density[bid.x] / dens;
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

  host_vector<real>& density = data_manager->host_data.den_fluid;
  host_vector<real3>& vel = data_manager->host_data.vel_fluid;
  host_vector<real3>& pos = data_manager->host_data.pos_fluid;

  CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

  //=======COMPUTE DENSITY OF FLUID
  density.resize(num_fluid_bodies);
  den_con.resize(num_fluid_contacts * 2 + num_fluid_bodies);
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

// host_vector<real> dist_temp(fluid_contact_idB.size());

#pragma omp parallel for
  for (int i = 0; i < last_body; i++) {
    uint start = fluid_start_index[i], end = fluid_start_index[i + 1];
    int body_a = fluid_contact_idA_start[i];
    real3 posa = pos[body_a];
    real3 diag = 0;
    int diag_index = 0;
    real dens = 0;

    for (int index = start; index < end; index++) {
      int body_b = fluid_contact_idB[index];
      if (body_a == body_b) {
        diag_index = index;
        dens += mass / density[body_b] * KPOLY6 * h_6;
        continue;
      }
      real3 xij = (posa - pos[body_b]);
      real dist = length(xij);
      real3 off_diag = mass_over_density * KGSPIKY * pow(h - dist, 2) * xij;
      den_con[index] = off_diag;
      diag += off_diag;

      dens += (mass / density[body_b]) * KPOLY6 * pow((h * h - dist * dist), 3);
    }
    density[body_a] = density[body_a] / dens;
    den_con[diag_index] = -diag;
  }
  if (data_manager->settings.fluid.enable_viscosity) {
    viscosity_row_1.clear();
    viscosity_row_2.clear();
    viscosity_row_3.clear();
    viscosity_row_1.resize(num_fluid_contacts * 2 + num_fluid_bodies);
    viscosity_row_2.resize(num_fluid_contacts * 2 + num_fluid_bodies);
    viscosity_row_3.resize(num_fluid_contacts * 2 + num_fluid_bodies);

// shear_tensor.clear();
// shear_tensor.resize(num_fluid_bodies);
//    shear_trace.clear();
//    shear_trace.resize(num_fluid_bodies);

//#pragma omp parallel for
//    for (int i = 0; i < last_body; i++) {
//      uint start = fluid_start_index[i], end = fluid_start_index[i + 1];
//      int2 bid;
//      bid.x = fluid_contact_idA_start[i];
//      M33 tensor;
//      real dens;
//      for (int index = start; index < end; index++) {
//        bid.y = fluid_contact_idB[index];
//        if (bid.x != bid.y) {
//          real3 xij = (pos[bid.x] - pos[bid.y]);
//          real dist = length(xij);
//          real3 vij = (vel[bid.x] - vel[bid.y]);
//          real3 grad_kernel = GRAD_KERNEL(xij, dist, h);
//          tensor = tensor + ComputeShearTensor(mass / density[bid.y], grad_kernel, vij);
//        }
//      }
//      // look at the jop reference for how to do a norm of the tensor
//      //||T|| = (0.5*Tij*Tij)^0.5
//      real norm = sqrt(0.5 * Trace(tensor * Transpose(tensor)));
//      shear_trace[bid.x] = norm;
//      // std::cout << norm << std::endl;
//    }

#pragma omp parallel for
    for (int i = 0; i < last_body; i++) {
      uint start = fluid_start_index[i], end = fluid_start_index[i + 1];
      int body_a = fluid_contact_idA_start[i];
      real3 posa = pos[body_a];
      M33 visc_mat;
      int diag_index = 0;
      for (int index = start; index < end; index++) {
        int body_b = fluid_contact_idB[index];
        if (body_a == body_b) {
          diag_index = index;
          continue;
        }
        real3 xij = (posa - pos[body_b]);
        real dist = length(xij);
        real density_a = density[body_a];
        real density_b = density[body_b];

        //        real norm_shear_a = shear_trace[body_a];
        //        real norm_shear_b = shear_trace[body_b];
        //        real mu_b = 290.8;
        //        real tau_b = 374.0;
        //        real corr = 1e-5;
        //        visca = (mu_b + tau_b / (corr + norm_shear_a));
        //        viscb = (mu_b + tau_b / (corr + norm_shear_b));
        //
        //        std::cout << norm_shear_a << " " << norm_shear_b << " " << visca << " " << viscb << std::endl;

        real part_a = (8.0 / (density_a + density_b));
        real part_b = visca + viscb;  //(visca / density_a + viscb / density_b);  // /
        real part_c = 1.0 / (h * ((dist * dist / h_2) + eta_2));
        real scalar = -mass_2 * part_a * part_b * part_c;
        real kernel = KGSPIKY * pow(h - dist, 2);
        // kernel = 45.0 / (F_PI * h_6) * (h - dist);;
        M33 matrix = VectorxVector(xij, kernel * xij) * scalar;

        viscosity_row_1[index] = R3(matrix.U.x, matrix.V.x, matrix.W.x);
        viscosity_row_2[index] = R3(matrix.U.y, matrix.V.y, matrix.W.y);
        viscosity_row_3[index] = R3(matrix.U.z, matrix.V.z, matrix.W.z);

        visc_mat = visc_mat + matrix;
      }

      viscosity_row_1[diag_index] = R3(visc_mat.U.x, visc_mat.V.x, visc_mat.W.x) * -1;
      viscosity_row_2[diag_index] = R3(visc_mat.U.y, visc_mat.V.y, visc_mat.W.y) * -1;
      viscosity_row_3[diag_index] = R3(visc_mat.U.z, visc_mat.V.z, visc_mat.W.z) * -1;
    }
  }
  LOG(INFO) << "ChConstraintFluidFluid::JACOBIAN OF FLUID";
  for (int i = 0; i < last_body; i++) {
    uint start = fluid_start_index[i];
    uint end = fluid_start_index[i + 1];
    int body_a = fluid_contact_idA_start[i];
    for (int index = start; index < end; index++) {
      int body_b = fluid_contact_idB[index];
      real3 density_constraint = den_con[index];
      D_T.append(index_offset + body_a, body_offset + body_b * 3 + 0, density_constraint.x);
      D_T.append(index_offset + body_a, body_offset + body_b * 3 + 1, density_constraint.y);
      D_T.append(index_offset + body_a, body_offset + body_b * 3 + 2, density_constraint.z);
    }
    D_T.finalize(index_offset + body_a);
  }
  if (data_manager->settings.fluid.enable_viscosity) {
    for (int i = 0; i < last_body; i++) {
      uint start = fluid_start_index[i];
      uint end = fluid_start_index[i + 1];
      int body_a = fluid_contact_idA_start[i];
      for (int index = start; index < end; index++) {
        int body_b = fluid_contact_idB[index];
        real3 row_1 = viscosity_row_1[index];

        D_T.append(index_offset + num_fluid_bodies + body_a * 3 + 0, body_offset + body_b * 3 + 0, row_1.x);
        D_T.append(index_offset + num_fluid_bodies + body_a * 3 + 0, body_offset + body_b * 3 + 1, row_1.y);
        D_T.append(index_offset + num_fluid_bodies + body_a * 3 + 0, body_offset + body_b * 3 + 2, row_1.z);
      }
      D_T.finalize(index_offset + num_fluid_bodies + body_a * 3 + 0);
      for (int index = start; index < end; index++) {
        int body_b = fluid_contact_idB[index];
        real3 row_2 = viscosity_row_2[index];

        D_T.append(index_offset + num_fluid_bodies + body_a * 3 + 1, body_offset + body_b * 3 + 0, row_2.x);
        D_T.append(index_offset + num_fluid_bodies + body_a * 3 + 1, body_offset + body_b * 3 + 1, row_2.y);
        D_T.append(index_offset + num_fluid_bodies + body_a * 3 + 1, body_offset + body_b * 3 + 2, row_2.z);
      }
      D_T.finalize(index_offset + num_fluid_bodies + body_a * 3 + 1);
      for (int index = start; index < end; index++) {
        int body_b = fluid_contact_idB[index];
        real3 row_3 = viscosity_row_3[index];

        D_T.append(index_offset + num_fluid_bodies + body_a * 3 + 2, body_offset + body_b * 3 + 0, row_3.x);
        D_T.append(index_offset + num_fluid_bodies + body_a * 3 + 2, body_offset + body_b * 3 + 1, row_3.y);
        D_T.append(index_offset + num_fluid_bodies + body_a * 3 + 2, body_offset + body_b * 3 + 2, row_3.z);
      }
      D_T.finalize(index_offset + num_fluid_bodies + body_a * 3 + 2);
    }
  }
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
    host_vector<real>& density = data_manager->host_data.den_fluid;
    host_vector<real3>& pos = data_manager->host_data.pos_fluid;
    DynamicVector<real> g(num_fluid_bodies);
    real epsilon = data_manager->settings.fluid.epsilon;
    real tau = data_manager->settings.fluid.tau;
    real h = data_manager->settings.fluid.kernel_radius;
    real zeta = 1.0 / (1.0 + 4.0 * tau / h);

#pragma omp parallel for
    for (int index = 0; index < num_fluid_bodies; index++) {
      // g[index] = density[index] / density_fluid - 1.0;
      b_sub[index] = -(density[index] / data_manager->settings.fluid.density - 1.0);
      // std::cout << g[index] << " " << density[index] << std::endl;
    }
    // b_sub = -4.0 / step_size * zeta * g + zeta * D_T_sub * v_sub;
    // b_sub = -g + zeta * D_T_sub * v_sub;
    // b_sub = -g;  // + D_T_sub * v_sub;
  } else {
    SubVectorType b_sub = blaze::subvector(data_manager->host_data.b, index_offset, num_fluid_contacts);
    custom_vector<real3>& pos = data_manager->host_data.pos_fluid;
    custom_vector<int2>& bids = data_manager->host_data.bids_fluid_fluid;
    real h = data_manager->settings.fluid.kernel_radius;
    real inv_hpa = 1.0 / (step_size + data_manager->settings.solver.alpha);
    real inv_hhpa = 1.0 / (step_size * (step_size + data_manager->settings.solver.alpha));
#pragma omp parallel for
    for (int index = 0; index < num_fluid_contacts; index++) {
      int2 bid = bids[index];
      real3 n = (pos[bid.y] - pos[bid.x]);
      real depth = length(n) - h * 2;
      real bi = 0;
      if (data_manager->settings.solver.alpha > 0) {
        bi = inv_hpa * depth;
      } else {
        if (data_manager->settings.solver.contact_recovery_speed < 0) {
          bi = real(1.0) / step_size * depth;
        } else {
          bi = std::max(real(1.0) / step_size * depth, -data_manager->settings.fluid.contact_recovery_speed);
        }
      }
      b_sub[index] = bi;
    }
  }
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

  custom_vector<int2>& bids = data_manager->host_data.bids_fluid_fluid;

  CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
  for (int index = 0; index < num_fluid_contacts; index++) {
    int2 bid = bids[index];

    D_T.append(index_offset + index, body_offset + bid.x * 3 + 0, 1);
    D_T.append(index_offset + index, body_offset + bid.x * 3 + 1, 1);
    D_T.append(index_offset + index, body_offset + bid.x * 3 + 2, 1);

    D_T.append(index_offset + index, body_offset + bid.y * 3 + 0, 1);
    D_T.append(index_offset + index, body_offset + bid.y * 3 + 1, 1);
    D_T.append(index_offset + index, body_offset + bid.y * 3 + 2, 1);
    D_T.finalize(index_offset + index);
  }
}
void ChConstraintFluidFluid::GenerateSparsityFluid() {
  LOG(INFO) << "ChConstraintFluidFluid::GenerateSparsityFluid";
  CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

  // density constraints
  for (int i = 0; i < last_body; i++) {
    uint start = fluid_start_index[i];
    uint end = fluid_start_index[i + 1];
    int2 bid;
    bid.x = fluid_contact_idA_start[i];
    for (int index = start; index < end; index++) {
      bid.y = fluid_contact_idB[index];
      D_T.append(index_offset + bid.x, body_offset + bid.y * 3 + 0, 1);
      D_T.append(index_offset + bid.x, body_offset + bid.y * 3 + 1, 1);
      D_T.append(index_offset + bid.x, body_offset + bid.y * 3 + 2, 1);
    }
    D_T.finalize(index_offset + bid.x);
  }
  // Add more entries for viscosity
  // Code is repeated because there are three rows per viscosity constraint

  //  for (int i = 0; i < last_body; i++) {
  //    uint start = (i == 0) ? 0 : fluid_start_index[i - 1];
  //    uint end = fluid_start_index[i];
  //    int2 bid;
  //    bid.x = fluid_contact_idA[i];
  //    for (int index = start; index < end; index++) {
  //      bid.y = fluid_contact_idB[index];
  //      D_T.append(index_offset + num_fluid_bodies + bid.x * 3 + 0, body_offset + bid.y * 3 + 0, 1);
  //      D_T.append(index_offset + num_fluid_bodies + bid.x * 3 + 0, body_offset + bid.y * 3 + 1, 1);
  //      D_T.append(index_offset + num_fluid_bodies + bid.x * 3 + 0, body_offset + bid.y * 3 + 2, 1);
  //    }
  //    D_T.finalize(index_offset + num_fluid_bodies + bid.x * 3 + 0);
  //
  //    for (int index = start; index < end; index++) {
  //      bid.y = fluid_contact_idB[index];
  //      D_T.append(index_offset + num_fluid_bodies + bid.x * 3 + 1, body_offset + bid.y * 3 + 0, 1);
  //      D_T.append(index_offset + num_fluid_bodies + bid.x * 3 + 1, body_offset + bid.y * 3 + 1, 1);
  //      D_T.append(index_offset + num_fluid_bodies + bid.x * 3 + 1, body_offset + bid.y * 3 + 2, 1);
  //    }
  //    D_T.finalize(index_offset + num_fluid_bodies + bid.x * 3 + 1);
  //
  //    for (int index = start; index < end; index++) {
  //      bid.y = fluid_contact_idB[index];
  //      D_T.append(index_offset + num_fluid_bodies + bid.x * 3 + 2, body_offset + bid.y * 3 + 0, 1);
  //      D_T.append(index_offset + num_fluid_bodies + bid.x * 3 + 2, body_offset + bid.y * 3 + 1, 1);
  //      D_T.append(index_offset + num_fluid_bodies + bid.x * 3 + 2, body_offset + bid.y * 3 + 2, 1);
  //    }
  //    D_T.finalize(index_offset + num_fluid_bodies + bid.x * 3 + 2);
  //  }
}
void ChConstraintFluidFluid::GenerateSparsity() {
  if (data_manager->num_fluid_contacts <= 0) {
    return;
  }

  if (data_manager->settings.fluid.fluid_is_rigid == false) {
    DetermineNeighbors();
    // GenerateSparsityFluid();
  } else {
    GenerateSparsityRigid();
  }
}

void ChConstraintFluidFluid::DetermineNeighbors() {
  if(num_fluid_bodies==0){
    return;
  }
  LOG(INFO) << "ChConstraintFluidFluid::DetermineNeighbors";
  // get a reference to the contact body ID data
  host_vector<int2>& bids = data_manager->host_data.bids_fluid_fluid;
  // host_vector<real>& dist_fluid_fluid = data_manager->host_data.dist_fluid_fluid;

  fluid_contact_idA.resize(num_fluid_contacts * 2 + num_fluid_bodies);
  fluid_contact_idB.resize(num_fluid_contacts * 2 + num_fluid_bodies);
  fluid_contact_idA_start.resize(num_fluid_contacts * 2 + num_fluid_bodies);
// dist_fluid_fluid.resize(num_fluid_contacts * 2 + num_fluid_bodies);

// For each contact in the list that is a fluid contact
#pragma omp parallel for
  for (int index = 0; index < num_fluid_bodies; index++) {
    fluid_contact_idA[index] = index;
    fluid_contact_idB[index] = index;
    // dist_fluid_fluid[index] = 0;
  }
#pragma omp parallel for
  for (int index = 0; index < num_fluid_contacts; index++) {
    int2 body_id = bids[index];
    fluid_contact_idA[index + num_fluid_bodies] = body_id.x;
    fluid_contact_idA[index + num_fluid_bodies + num_fluid_contacts] = body_id.y;

    fluid_contact_idB[index + num_fluid_bodies] = body_id.y;
    fluid_contact_idB[index + num_fluid_bodies + num_fluid_contacts] = body_id.x;
  }
  LOG(INFO) << "ChConstraintFluidFluid::Thrust_Sort_By_Key";

  Thrust_Sort_By_Key(fluid_contact_idB, fluid_contact_idA);
  Thrust_Sort_By_Key(fluid_contact_idA, fluid_contact_idB);

  fluid_start_index.resize(num_fluid_bodies);
  LOG(INFO) << "ChConstraintFluidFluid::Thrust_Reduce_By_Key";
  last_body = Run_Length_Encode(fluid_contact_idA, fluid_contact_idA_start, fluid_start_index);
  fluid_start_index.resize(last_body + 1);
  fluid_start_index[last_body] = 0;
  LOG(INFO) << "ChConstraintFluidFluid::Thrust_Exclusive_Scan";
  Thrust_Exclusive_Scan(fluid_start_index);
}
void ChConstraintFluidFluid::ArtificialPressure() {
  if (data_manager->settings.fluid.artificial_pressure == false) {
    return;
  }
  if (data_manager->settings.fluid.fluid_is_rigid == false) {
    host_vector<real3>& pos = data_manager->host_data.pos_fluid;
    real mass_fluid = data_manager->settings.fluid.mass;
    real h = data_manager->settings.fluid.kernel_radius;
    real k = data_manager->settings.fluid.artificial_pressure_k;
    real dq = data_manager->settings.fluid.artificial_pressure_dq;
    real n = data_manager->settings.fluid.artificial_pressure_n;
#pragma omp parallel for
    for (int i = 0; i < last_body; i++) {
      uint start = fluid_start_index[i], end = fluid_start_index[i + 1];
      int2 bid;
      bid.x = fluid_contact_idA_start[i];
      real corr = 0;
      for (int index = start; index < end; index++) {
        bid.y = fluid_contact_idB[index];
        real3 xij = (pos[bid.x] - pos[bid.y]);
        real dist = length(xij);
        corr += k * pow(KERNEL(dist, h) / KERNEL(dq, h), n);
      }
      // std::cout << gamma[index_offset + bid.x] << " " << corr << std::endl;
      data_manager->host_data.gamma[index_offset + bid.x] += corr;
    }
  }
}
}
