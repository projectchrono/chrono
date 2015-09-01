// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
// Implementation of methods specific to the parallel DEM solver.
//
// These functions implement the basic time update for a multibody system using
// a penalty-based approach for including frictional contact. It is assumed that
// geometric contact information has been already computed and is available.
// The current algorithm is based on a semi-implicit Euler scheme and projection
// on the velocity manifold of the bilateral constraints.
// =============================================================================

#include "chrono_parallel/lcp/ChLcpSolverParallel.h"

using namespace chrono;

// -----------------------------------------------------------------------------
// Main worker function for calculating contact forces. Calculates the contact
// force and torque for the contact pair identified by 'index' and stores them
// in the 'extended' output arrays. The calculated force and torque vectors are
// therefore duplicated in the output arrays, once for each body involved in the
// contact (with opposite signs for the two bodies).
// -----------------------------------------------------------------------------
void function_CalcContactForces(
    int index,                              // index of this contact pair
    CONTACTFORCEMODEL force_model,          // contact force model
    TANGENTIALDISPLACEMENTMODE displ_mode,  // type of tangential displacement history
    bool use_mat_props,                     // flag specifying how coefficients are obtained
    real char_vel,                          // characteristic velocity (Hooke)
    real min_slip_vel,                      // threshold tangential velocity
    real dT,                                // integration time step
    real* mass,                             // body masses
    real3* pos,                             // body positions
    real4* rot,                             // body orientations
    real* vel,                              // body linear and angular velocities
    real2* elastic_moduli,                  // Young's modulus (per body)
    real* cr,                               // coefficient of restitution (per body)
    real4* dem_coeffs,                      // stiffness and damping coefficients (per body)
    real* mu,                               // coefficient of friction (per body)
    real* cohesion,                         // cohesion force (per body)
    int2* body_id,                          // body IDs (per contact)
    int2* shape_id,                         // shape IDs (per contact)
    real3* pt1,                             // point on shape 1 (per contact)
    real3* pt2,                             // point on shape 2 (per contact)
    real3* normal,                          // contact normal (per contact)
    real* depth,                            // penetration depth (per contact)
    real* eff_radius,                       // effective contact radius (per contact)
    int3* shear_neigh,                      // neighbor list of contacting bodies and shapes (max_shear per body)
    bool* shear_touch,                      // flag if contact in neighbor list is persistent (max_shear per body)
    real3* shear_disp,                      // accumulated shear displacement for each neighbor (max_shear per body)
    int* ext_body_id,                       // [output] body IDs (two per contact)
    real3* ext_body_force,                  // [output] body force (two per contact)
    real3* ext_body_torque)                 // [output] body torque (two per contact)
{
  // Identify the two bodies in contact.
  int body1 = body_id[index].x;
  int body2 = body_id[index].y;

  // If the two contact shapes are actually separated, set zero forces and torques.
  if (depth[index] >= 0) {
    ext_body_id[2 * index] = body1;
    ext_body_id[2 * index + 1] = body2;
    ext_body_force[2 * index] = ZERO_VECTOR;
    ext_body_force[2 * index + 1] = ZERO_VECTOR;
    ext_body_torque[2 * index] = ZERO_VECTOR;
    ext_body_torque[2 * index + 1] = ZERO_VECTOR;

    return;
  }

  // Kinematic information
  // ---------------------

  // Express contact point locations in local frames
  //   s' = At * s = At * (rP - r)
  real3 pt1_loc = TransformParentToLocal(pos[body1], rot[body1], pt1[index]);
  real3 pt2_loc = TransformParentToLocal(pos[body2], rot[body2], pt2[index]);

  // Calculate velocities of the contact points (in global frame)
  //   vP = v + omg x s = v + A * (omg' x s')
  real3 v_body1 = real3(vel[body1 * 6 + 0], vel[body1 * 6 + 1], vel[body1 * 6 + 2]);
  real3 v_body2 = real3(vel[body2 * 6 + 0], vel[body2 * 6 + 1], vel[body2 * 6 + 2]);

  real3 o_body1 = real3(vel[body1 * 6 + 3], vel[body1 * 6 + 4], vel[body1 * 6 + 5]);
  real3 o_body2 = real3(vel[body2 * 6 + 3], vel[body2 * 6 + 4], vel[body2 * 6 + 5]);

  real3 vel1 = v_body1 + quatRotateMat(cross(o_body1, pt1_loc), rot[body1]);
  real3 vel2 = v_body2 + quatRotateMat(cross(o_body2, pt2_loc), rot[body2]);

  // Calculate relative velocity (in global frame)
  // Note that relvel_n_mag is a signed quantity, while relvel_t_mag is an
  // actual magnitude (always positive).
  real3 relvel = vel2 - vel1;
  real relvel_n_mag = dot(relvel, normal[index]);
  real3 relvel_n = relvel_n_mag * normal[index];
  real3 relvel_t = relvel - relvel_n;
  real relvel_t_mag = length(relvel_t);

  // Calculate composite material properties
  // ---------------------------------------

  real m_eff = mass[body1] * mass[body2] / (mass[body1] + mass[body2]);

  real mu_eff = std::min(mu[body1], mu[body2]);
  real cohesion_eff = std::min(cohesion[body1], cohesion[body2]);

  real E_eff, G_eff, cr_eff;
  real user_kn, user_kt, user_gn, user_gt;

  if (use_mat_props) {
    real Y1 = elastic_moduli[body1].x;
    real Y2 = elastic_moduli[body2].x;
    real nu1 = elastic_moduli[body1].y;
    real nu2 = elastic_moduli[body2].y;
    real inv_E = (1 - nu1 * nu1) / Y1 + (1 - nu2 * nu2) / Y2;
    real inv_G = 2 * (2 + nu1) * (1 - nu1) / Y1 + 2 * (2 + nu2) * (1 - nu2) / Y2;

    E_eff = 1 / inv_E;
    G_eff = 1 / inv_G;
    cr_eff = (cr[body1] + cr[body2]) / 2;
  }
  else {
    user_kn = (dem_coeffs[body1].w + dem_coeffs[body2].w) / 2;
    user_kt = (dem_coeffs[body1].x + dem_coeffs[body2].x) / 2;
    user_gn = (dem_coeffs[body1].y + dem_coeffs[body2].y) / 2;
    user_gt = (dem_coeffs[body1].z + dem_coeffs[body2].z) / 2;
  }

  // Contact force
  // -------------

  // All models use the following formulas for normal and tangential forces:
  //     Fn = kn * delta_n - gn * v_n
  //     Ft = kt * delta_t - gt * v_t
  // The stiffness and damping coefficients are obtained differently, based
  // on the force model and on how coefficients are specified.
  real kn;
  real kt;
  real gn;
  real gt;

  real delta_n = -depth[index];
  real3 delta_t = R3(0, 0, 0);

  int i;
  int contact_id;
  int shear_body1;
  int shear_body2;
  int shear_shape1;
  int shear_shape2;
  bool newcontact = true;

  if (displ_mode == ONE_STEP) {
    delta_t = relvel_t * dT;

  } else if (displ_mode == MULTI_STEP) {
    delta_t = relvel_t * dT;

    // Contact history information should be stored on the body with
    // the smaller shape in contact or the body with larger index.
    // Currently, it is assumed that the smaller shape is on the body
    // with larger ID.
    // We call this body shear_body1.
    int shape1 = shape_id[index].x;
    int shape2 = shape_id[index].y;

    shear_body1 = std::max(body1, body2);
    shear_body2 = std::min(body1, body2);
    shear_shape1 = std::max(shape1, shape2);
    shear_shape2 = std::min(shape1, shape2);

    // Check if contact history already exists.
    // If not, initialize new contact history.
    for (i = 0; i < max_shear; i++) {
      if (shear_neigh[max_shear * shear_body1 + i].x == shear_body2
        && shear_neigh[max_shear * shear_body1 + i].y == shear_shape1
        && shear_neigh[max_shear * shear_body1 + i].z == shear_shape2) {
        contact_id = i;
        newcontact = false;
        break;
      }
    }
    if (newcontact == true) {
      for (i = 0; i < max_shear; i++) {
        if (shear_neigh[max_shear * shear_body1 + i].x == -1) {
          contact_id = i;
          shear_neigh[max_shear * shear_body1 + i].x = shear_body2;
          shear_neigh[max_shear * shear_body1 + i].y = shear_shape1;
          shear_neigh[max_shear * shear_body1 + i].z = shear_shape2;
          shear_disp[max_shear * shear_body1 + i].x = 0;
          shear_disp[max_shear * shear_body1 + i].y = 0;
          shear_disp[max_shear * shear_body1 + i].z = 0;
          break;
        }
      }
    }

    // Record that these two bodies are really in contact at this time.
    shear_touch[max_shear * shear_body1 + contact_id] = true;

    // Increment stored contact history tangential (shear) displacement vector
    // and project it onto the <current> contact plane.

    if (shear_body1 == body1) {
      shear_disp[max_shear * shear_body1 + contact_id] += delta_t;
      shear_disp[max_shear * shear_body1 + contact_id] -=
        dot(shear_disp[max_shear * shear_body1 + contact_id], normal[index])
        * normal[index];
      delta_t = shear_disp[max_shear * shear_body1 + contact_id];
    }
    else {
      shear_disp[max_shear * shear_body1 + contact_id] -= delta_t;
      shear_disp[max_shear * shear_body1 + contact_id] -=
        dot(shear_disp[max_shear * shear_body1 + contact_id], normal[index])
        * normal[index];
      delta_t = -shear_disp[max_shear * shear_body1 + contact_id];
    }
  }

  switch (force_model) {
  case HOOKE:
    if (use_mat_props) {
      double tmp_k = (16.0 / 15) * sqrt(eff_radius[index]) * E_eff;
      double v2 = char_vel * char_vel;
      double tmp_g = 1 + pow(CH_C_PI / log(cr_eff), 2);
      kn = tmp_k * pow(m_eff * v2 / tmp_k, 1.0 / 5);
      kt = kn;
      gn = std::sqrt(4 * m_eff * kn / tmp_g);
      gt = gn;
    }
    else {
      kn = user_kn;
      kt = user_kt;
      gn = m_eff * user_gn;
      gt = m_eff * user_gt;
    }

    break;

  case HERTZ:
    if (use_mat_props) {
      double sqrt_Rd = sqrt(eff_radius[index] * delta_n);
      double Sn = 2 * E_eff * sqrt_Rd;
      double St = 8 * G_eff * sqrt_Rd;
      double loge = log(cr_eff);
      double beta = loge / sqrt(loge * loge + CH_C_PI * CH_C_PI);
      kn = (2.0 / 3) * Sn;
      kt = St;
      gn = -2 * sqrt(5.0 / 6) * beta * sqrt(Sn * m_eff);
      gt = -2 * sqrt(5.0 / 6) * beta * sqrt(St * m_eff);
    }
    else {
      double tmp = eff_radius[index] * std::sqrt(delta_n);
      kn = tmp * user_kn;
      kt = tmp * user_kt;
      gn = tmp * m_eff * user_gn;
      gt = tmp * m_eff * user_gt;
    }

    break;
  }

  // Calculate the the normal and tangential contact forces.
  // The normal force is a magnitude, and it will be applied along the contact
  // normal direction (negative relative to body1 & positive relative to body2).
  // The tangential force is a vector with two parts: one depends on the stored
  // contact history tangential (or shear) displacement vector delta_t, and the
  // other depends on the current relative velocity vector (for viscous damping).
  real forceN_mag = kn * delta_n - gn * relvel_n_mag;
  real3 forceT_stiff = kt * delta_t;
  real3 forceT_damp = gt * relvel_t;

  // If the resulting normal force is negative, then the two shapes are
  // moving away from each other so fast that no contact force is generated.
  if (forceN_mag < 0) {
    forceN_mag = 0;
    forceT_stiff.x = 0;
    forceT_stiff.y = 0;
    forceT_stiff.z = 0;
    forceT_damp.x = 0;
    forceT_damp.y = 0;
    forceT_damp.z = 0;
  }

  // Include cohesion force.
  // (This is a very simple model, which can perhaps be improved later.)
  forceN_mag -= cohesion_eff;

  // Apply Coulomb friction law.
  // We must enforce force_T_mag <= mu_eff * |forceN_mag|.
  // If force_T_mag > mu_eff * |forceN_mag| and there is shear displacement
  // due to contact history, then the shear displacement is scaled so that
  // the tangential force will be correct if force_T_mag subsequently drops
  // below the Coulomb limit.  Also, if there is sliding, then there is no
  // viscous damping in the tangential direction (to keep the Coulomb limit
  // strict, and independent of velocity).
  //  real forceT_mag = length(forceT_stiff + forceT_damp);  // This seems correct
  real forceT_mag = length(forceT_stiff);  // This is what LAMMPS/LIGGGHTS does
  real delta_t_mag = length(delta_t);
  real forceT_slide = mu_eff * fabsf(forceN_mag);
  if (forceT_mag > forceT_slide) {
    if (delta_t_mag != 0.0) {
      real forceT_stiff_mag = length(forceT_stiff);
      double ratio = forceT_slide / forceT_stiff_mag;
      forceT_stiff *= ratio;
      if (displ_mode == MULTI_STEP) {
        if (shear_body1 == body1) {
          shear_disp[max_shear * shear_body1 + contact_id] = forceT_stiff / kt;
        } else {
          shear_disp[max_shear * shear_body1 + contact_id] = -forceT_stiff / kt;
        }
      }
    } else {
      forceT_stiff.x = 0.0;
      forceT_stiff.y = 0.0;
      forceT_stiff.z = 0.0;
    }
    forceT_damp.x = 0.0;
    forceT_damp.y = 0.0;
    forceT_damp.z = 0.0;
  }

  // Accumulate normal and tangential forces
  real3 force = forceN_mag * normal[index];
  force -= forceT_stiff;
  force -= forceT_damp;

  // Body forces (in global frame) & torques (in local frame)
  // --------------------------------------------------------

  // Convert force into the local body frames and calculate induced torques
  //    n' = s' x F' = s' x (A*F)
  real3 torque1_loc = cross(pt1_loc, quatRotateMatT(force, rot[body1]));
  real3 torque2_loc = cross(pt2_loc, quatRotateMatT(force, rot[body2]));

  // Store body forces and torques, duplicated for the two bodies.
  ext_body_id[2 * index] = body1;
  ext_body_id[2 * index + 1] = body2;
  ext_body_force[2 * index] = -force;
  ext_body_force[2 * index + 1] = force;
  ext_body_torque[2 * index] = -torque1_loc;
  ext_body_torque[2 * index + 1] = torque2_loc;
}

// -----------------------------------------------------------------------------
// Calculate contact forces and torques for all contact pairs.
// -----------------------------------------------------------------------------
void ChLcpSolverParallelDEM::host_CalcContactForces(custom_vector<int>& ext_body_id,
                                                    custom_vector<real3>& ext_body_force,
                                                    custom_vector<real3>& ext_body_torque,
                                                    custom_vector<int2>& shape_pairs,
                                                    custom_vector<bool>& shear_touch) {
#pragma omp parallel for
  for (int index = 0; index < data_manager->num_rigid_contacts; index++) {
    function_CalcContactForces(index,
                               data_manager->settings.solver.contact_force_model,
                               data_manager->settings.solver.tangential_displ_mode,
                               data_manager->settings.solver.use_material_properties,
                               data_manager->settings.solver.characteristic_vel,
                               data_manager->settings.solver.min_slip_vel,
                               data_manager->settings.step_size,
                               data_manager->host_data.mass_rigid.data(),
                               data_manager->host_data.pos_rigid.data(),
                               data_manager->host_data.rot_rigid.data(),
                               data_manager->host_data.v.data(),
                               data_manager->host_data.elastic_moduli.data(),
                               data_manager->host_data.cr.data(),
                               data_manager->host_data.dem_coeffs.data(),
                               data_manager->host_data.mu.data(),
                               data_manager->host_data.cohesion_data.data(),
                               data_manager->host_data.bids_rigid_rigid.data(),
                               shape_pairs.data(),
                               data_manager->host_data.cpta_rigid_rigid.data(),
                               data_manager->host_data.cptb_rigid_rigid.data(),
                               data_manager->host_data.norm_rigid_rigid.data(),
                               data_manager->host_data.dpth_rigid_rigid.data(),
                               data_manager->host_data.erad_rigid_rigid.data(),
                               data_manager->host_data.shear_neigh.data(),
                               shear_touch.data(),
                               data_manager->host_data.shear_disp.data(),
                               ext_body_id.data(),
                               ext_body_force.data(),
                               ext_body_torque.data());
  }
}

// -----------------------------------------------------------------------------
// Include contact impulses (linear and rotational) for all bodies that are
// involved in at least one contact. For each such body, the corresponding
// entries in the arrays 'ct_body_force' and 'ct_body_torque' contain the
// cummulative force and torque, respectively, over all contacts involving that
// body.
// -----------------------------------------------------------------------------
void ChLcpSolverParallelDEM::host_AddContactForces(uint ct_body_count, const custom_vector<int>& ct_body_id) {
  const custom_vector<real3>& ct_body_force = data_manager->host_data.ct_body_force;
  const custom_vector<real3>& ct_body_torque = data_manager->host_data.ct_body_torque;

#pragma omp parallel for
  for (int index = 0; index < ct_body_count; index++) {
    real3 contact_force = data_manager->settings.step_size * ct_body_force[index];
    real3 contact_torque = data_manager->settings.step_size * ct_body_torque[index];
    data_manager->host_data.hf[ct_body_id[index] * 6 + 0] += contact_force.x;
    data_manager->host_data.hf[ct_body_id[index] * 6 + 1] += contact_force.y;
    data_manager->host_data.hf[ct_body_id[index] * 6 + 2] += contact_force.z;
    data_manager->host_data.hf[ct_body_id[index] * 6 + 3] += contact_torque.x;
    data_manager->host_data.hf[ct_body_id[index] * 6 + 4] += contact_torque.y;
    data_manager->host_data.hf[ct_body_id[index] * 6 + 5] += contact_torque.z;
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLcpSolverParallelDEM::host_SetContactForcesMap(uint ct_body_count, const custom_vector<int>& ct_body_id) {
  custom_vector<int>& ct_body_map = data_manager->host_data.ct_body_map;

#pragma omp parallel for
  for (int index = 0; index < ct_body_count; index++) {
    ct_body_map[ct_body_id[index]] = index;
  }
}

// -----------------------------------------------------------------------------
// Process contact information reported by the narrowphase collision detection,
// generate contact forces, and update the (linear and rotational) impulses for
// all bodies involved in at least one contact.
// -----------------------------------------------------------------------------
void ChLcpSolverParallelDEM::ProcessContacts() {
  // 1. Calculate contact forces and torques - per contact basis
  //    For each pair of contact shapes that overlap, we calculate and store the
  //    IDs of the two corresponding bodies and the resulting contact forces and
  //    torques on the two bodies.
  custom_vector<int> ext_body_id(2 * data_manager->num_rigid_contacts);
  custom_vector<real3> ext_body_force(2 * data_manager->num_rigid_contacts);
  custom_vector<real3> ext_body_torque(2 * data_manager->num_rigid_contacts);
  custom_vector<int2> shape_pairs;
  custom_vector<bool> shear_touch;

  if (data_manager->settings.solver.tangential_displ_mode == MULTI_STEP) {
    shape_pairs.resize(data_manager->num_rigid_contacts);
    shear_touch.resize(max_shear * data_manager->num_rigid_bodies);
    thrust::fill(thrust_parallel, shear_touch.begin(), shear_touch.end(), false);
#pragma omp parallel for
    for (int i = 0; i < data_manager->num_rigid_contacts; i++) {
      int2 pair = I2(int(data_manager->host_data.pair_rigid_rigid[i] >> 32),
                     int(data_manager->host_data.pair_rigid_rigid[i] & 0xffffffff));
      shape_pairs[i] = pair;
    }
  }

  host_CalcContactForces(ext_body_id, ext_body_force, ext_body_torque, shape_pairs, shear_touch);

  if (data_manager->settings.solver.tangential_displ_mode == MULTI_STEP) {
#pragma omp parallel for
    for (int index = 0; index < data_manager->num_rigid_bodies; index++) {
      for (int i = 0; i < max_shear; i++) {
        if (shear_touch[max_shear * index + i] == false)
          data_manager->host_data.shear_neigh[max_shear * index + i].x = -1;
      }
    }
  }

  // 2. Calculate contact forces and torques - per body basis
  //    Accumulate the contact forces and torques for all bodies that are
  //    involved in at least one contact, by reducing the contact forces and
  //    torques from all contacts these bodies are involved in. The number of
  //    bodies that experience at least one contact is 'ct_body_count'.
  thrust::sort_by_key(thrust_parallel,
                      ext_body_id.begin(),
                      ext_body_id.end(),
                      thrust::make_zip_iterator(thrust::make_tuple(ext_body_force.begin(), ext_body_torque.begin())));

  custom_vector<int> ct_body_id(data_manager->num_rigid_bodies);
  custom_vector<real3>& ct_body_force = data_manager->host_data.ct_body_force;
  custom_vector<real3>& ct_body_torque = data_manager->host_data.ct_body_torque;

  ct_body_force.resize(data_manager->num_rigid_bodies);
  ct_body_torque.resize(data_manager->num_rigid_bodies);

  // Reduce contact forces from all contacts and count bodies currently involved
  // in contact. We do this simultaneously for contact forces and torques, using
  // zip iterators.
  uint ct_body_count =
      thrust::reduce_by_key(
          ext_body_id.begin(),
          ext_body_id.end(),
          thrust::make_zip_iterator(thrust::make_tuple(ext_body_force.begin(), ext_body_torque.begin())),
          ct_body_id.begin(),
          thrust::make_zip_iterator(thrust::make_tuple(ct_body_force.begin(), ct_body_torque.begin())),
          thrust::equal_to<int>(),
          sum_tuples()).first -
      ct_body_id.begin();

  ct_body_force.resize(ct_body_count);
  ct_body_torque.resize(ct_body_count);

  // 3. Add contact forces and torques to existing forces (impulses):
  //    For all bodies involved in a contact, update the body forces and torques
  //    (scaled by the integration time step).
  host_AddContactForces(ct_body_count, ct_body_id);

  // 4. Set up map from all bodies in the system to bodies involved in a contact.
  host_SetContactForcesMap(ct_body_count, ct_body_id);
}

void ChLcpSolverParallelDEM::ComputeD() {
  uint num_constraints = data_manager->num_constraints;
  if (num_constraints <= 0) {
    return;
  }

  uint num_bodies = data_manager->num_rigid_bodies;
  uint num_shafts = data_manager->num_shafts;
  uint num_dof = data_manager->num_dof;
  uint num_contacts = data_manager->num_rigid_contacts;
  uint num_bilaterals = data_manager->num_bilaterals;
  uint nnz_bilaterals = data_manager->nnz_bilaterals;

  CompressedMatrix<real>& D_b_T = data_manager->host_data.D_b_T;
  clear(D_b_T);

  D_b_T.reserve(nnz_bilaterals);

  D_b_T.resize(num_constraints, num_dof, false);

  bilateral.GenerateSparsity();
  bilateral.Build_D();
}

void ChLcpSolverParallelDEM::ComputeE() {
  if (data_manager->num_constraints <= 0) {
    return;
  }

  data_manager->host_data.E.resize(data_manager->num_constraints);
  reset(data_manager->host_data.E);

  bilateral.Build_E();
}

void ChLcpSolverParallelDEM::ComputeR() {
  if (data_manager->num_constraints <= 0) {
    return;
  }

  data_manager->host_data.b.resize(data_manager->num_constraints);
  reset(data_manager->host_data.b);
  bilateral.Build_b();

  data_manager->host_data.R_full =
      -data_manager->host_data.b - data_manager->host_data.D_b_T * data_manager->host_data.M_invk;
}

// -----------------------------------------------------------------------------
// This is the main function for advancing the system state in time. On entry,
// geometric contact information is available as calculated by the narrowphase
// collision detection. This function calculates contact forces, updates the
// generalized velocities, then enforces the velocity-level constraints for any
// bilateral (joint) constraints present in the system.
// -----------------------------------------------------------------------------
void ChLcpSolverParallelDEM::RunTimeStep() {
  // This is the total number of constraints, note that there are no contacts
  data_manager->num_constraints = data_manager->num_bilaterals;
  data_manager->num_unilaterals = 0;

  // Calculate contact forces (impulses) and append them to the body forces
  data_manager->host_data.ct_body_map.resize(data_manager->num_rigid_bodies);
  thrust::fill(data_manager->host_data.ct_body_map.begin(), data_manager->host_data.ct_body_map.end(), -1);

  if (data_manager->num_rigid_contacts > 0) {
    data_manager->system_timer.start("ChLcpSolverParallelDEM_ProcessContact");
    ProcessContacts();
    data_manager->system_timer.stop("ChLcpSolverParallelDEM_ProcessContact");
  }

  // Generate the mass matrix and compute M_inv_k
  ComputeMassMatrix();

  // If there are (bilateral) constraints, calculate Lagrange multipliers.
  if (data_manager->num_constraints != 0) {
    data_manager->system_timer.start("ChLcpSolverParallel_Setup");

    bilateral.Setup(data_manager);

    solver->current_iteration = 0;
    data_manager->measures.solver.total_iteration = 0;
    data_manager->measures.solver.maxd_hist.clear();            ////
    data_manager->measures.solver.maxdeltalambda_hist.clear();  ////  currently not used

    solver->bilateral = &bilateral;
    solver->Setup(data_manager);

    // Set the initial guess for the iterative solver to zero.
    data_manager->host_data.gamma.resize(data_manager->num_constraints);
    data_manager->host_data.gamma.reset();

    // Compute the jacobian matrix, the compliance matrix and the right hand side
    ComputeD();
    ComputeE();
    ComputeR();

    data_manager->system_timer.stop("ChLcpSolverParallel_Setup");

    // Solve for the Lagrange multipliers associated with bilateral constraints.
    PerformStabilization();
  }

  // Update velocity (linear and angular)
  ComputeImpulses();

  for (int i = 0; i < data_manager->measures.solver.maxd_hist.size(); i++) {
    AtIterationEnd(data_manager->measures.solver.maxd_hist[i],
                   data_manager->measures.solver.maxdeltalambda_hist[i],
                   i);
  }
  tot_iterations = data_manager->measures.solver.maxd_hist.size();
}

void ChLcpSolverParallelDEM::ComputeImpulses() {
  DynamicVector<real>& v = data_manager->host_data.v;
  const DynamicVector<real>& M_invk = data_manager->host_data.M_invk;
  const DynamicVector<real>& gamma = data_manager->host_data.gamma;
  const CompressedMatrix<real>& M_invD_b = data_manager->host_data.M_invD_b;

  uint num_unilaterals = data_manager->num_unilaterals;
  uint num_bilaterals = data_manager->num_bilaterals;

  if (data_manager->num_constraints > 0) {
    ConstSubVectorType gamma_b =
        blaze::subvector(gamma, num_unilaterals, num_bilaterals);
    v = M_invk + M_invD_b * gamma_b;
  } else {
    v = M_invk;
  }
}
