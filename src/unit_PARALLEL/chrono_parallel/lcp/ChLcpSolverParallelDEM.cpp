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
#include "chrono_parallel/math/ChThrustLinearAlgebra.h"

using namespace chrono;

// -----------------------------------------------------------------------------
// Main worker function for calculating contact forces. Calculates the contact
// force and torque for the contact pair identified by 'index' and stores them
// in the 'extended' output arrays. The calculated force and torque vectors are
// therefore duplicated in the output arrays, once for each body involved in the
// contact (with opposite signs for the two bodies).
//
// TODO(Radu):
//   Currently hard-coded for Hunt-Crossley with simple sliding friction.
// -----------------------------------------------------------------------------
__host__ __device__
void
function_CalcContactForces(
    int          index,            // index of this contact pair
    real         dT,               // integration time step
    real3*       pos,              // body positions
    real4*       rot,              // body orientations
    real*        vel,              // body linear velocity (global frame), angular velocity (local frame)
    real2*       elastic_moduli,   // Young's modulus (per body)
    real*        mu,               // coefficient of friction (per body)
    real*        alpha,            // disipation coefficient (per body)
    real*        cr,               // coefficient of restitution (per body)
    real*        cohesion,         // cohesion force (per body)
    int2*        body_id,          // body IDs (per contact)
    real3*       pt1,              // point on shape 1 (per contact)
    real3*       pt2,              // point on shape 2 (per contact)
    real3*       normal,           // contact normal (per contact)
    real*        depth,            // penetration depth (per contact)
    real*        eff_radius,       // effective contact radius (per contact)
    int*         ext_body_id,      // [output] body IDs (two per contact)
    real3*       ext_body_force,   // [output] body force (two per contact)
    real3*       ext_body_torque)  // [output] body torque (two per contact)
{
  // Identify the two bodies in contact.
  int body1 = body_id[index].x;
  int body2 = body_id[index].y;

  // If the two contact shapes are actually separated, set zero forces and
  // torques.
  if (depth[index] >= 0) {
    ext_body_id[2*index] = body1;
    ext_body_id[2*index+1] = body2;
    ext_body_force[2*index] = ZERO_VECTOR;
    ext_body_force[2*index+1] = ZERO_VECTOR;
    ext_body_torque[2*index] = ZERO_VECTOR;
    ext_body_torque[2*index+1] = ZERO_VECTOR;

    return;
  }

  // Kinematic information
  // ---------------------

  // Express contact point locations in local frames
  //   s' = At * s = At * (rP - r)
  real3 pt1_loc = TransformParentToLocal(pos[body1], rot[body1], pt1[index]);
  real3 pt2_loc = TransformParentToLocal(pos[body2], rot[body2], pt2[index]);

  // Calculate relative velocity (in global frame)
  //   vP = v + omg x s = v + A * (omg' x s')
  real3 v_body1 = real3(vel[body1]*6+0, vel[body1]*6+1, vel[body1]*6+2);
  real3 v_body2 = real3(vel[body2]*6+0, vel[body2]*6+1, vel[body2]*6+2);

  real3 o_body1 = real3(vel[body1]*6+3, vel[body1]*6+4, vel[body1]*6+5);
  real3 o_body2 = real3(vel[body2]*6+3, vel[body2]*6+4, vel[body2]*6+5);

  real3 vel1 = v_body1 + quatRotateMat(cross(o_body1, pt1_loc), rot[body1]);
  real3 vel2 = v_body2 + quatRotateMat(cross(o_body2, pt2_loc), rot[body2]);
  real3 relvel = vel2 - vel1;
  real  relvel_n_mag = dot(relvel, normal[index]);
  real3 relvel_n = relvel_n_mag * normal[index];
  real3 relvel_t = relvel - relvel_n;
  real  relvel_t_mag = length(relvel_t);

  // Calculate composite material properties
  // ---------------------------------------

  real Y1 = elastic_moduli[body1].x;
  real Y2 = elastic_moduli[body2].x;
  real nu1 = elastic_moduli[body1].y;
  real nu2 = elastic_moduli[body2].y;
  real inv_E = (1 - nu1 * nu1) / Y1 + (1 - nu2 * nu2) / Y2;
  real inv_G = 2 * (2 + nu1) * (1 - nu1) / Y1 + 2 * (2 + nu2) * (1 - nu2) / Y2;

  real E_eff = 1 / inv_E;
  real G_eff = 1 / inv_G;

  real mu_eff = std::min(mu[body1], mu[body2]);
  //real cr_eff = (cr[body1] + cr[body2]) / 2;
  real alpha_eff = (alpha[body1] + alpha[body2]) / 2;

  real cohesion_eff = std::min(cohesion[body1], cohesion[body2]);

  // Contact force
  // -------------

  // Normal force: Hunt-Crossley
  real delta = -depth[index];
  real kn = (4.0 / 3) * E_eff * sqrt(eff_radius[index]);
  real forceN_elastic = kn * delta * sqrt(delta);
  real forceN_dissipation = 1.5 * alpha_eff * forceN_elastic * relvel_n_mag;
  real forceN = forceN_elastic - forceN_dissipation;

  // If the resulting force is negative, the two shapes are moving away from
  // each other so fast that no contact force is generated.
  if (forceN < 0)
    forceN = 0;

  // Include cohesion force
  forceN -= cohesion_eff;

  real3 force = forceN * normal[index];

  // Tangential force: Simple Coulomb sliding
  if (relvel_t_mag > 1e-4) {
    real forceT = mu_eff * fabs(forceN);

    force -= (forceT / relvel_t_mag) * relvel_t;
  }

  // Body forces (in global frame) & torques (in local frame)
  // --------------------------------------------------------

  // Convert force into the local body frames and calculate induced torques
  //    n' = s' x F' = s' x (A*F)
  real3 torque1_loc = cross(pt1_loc, quatRotateMatT(force, rot[body1]));
  real3 torque2_loc = cross(pt2_loc, quatRotateMatT(force, rot[body2]));

  // Store body forces and torques, duplicated for the two bodies.
  ext_body_id[2*index]   = body1;
  ext_body_id[2*index+1] = body2;
  ext_body_force[2*index]   = -force;
  ext_body_force[2*index+1] =  force;
  ext_body_torque[2*index]   = -torque1_loc;
  ext_body_torque[2*index+1] =  torque2_loc;
}

// -----------------------------------------------------------------------------
// Calculate contact forces and torques for all contact pairs.
// -----------------------------------------------------------------------------
void
ChLcpSolverParallelDEM::host_CalcContactForces(
    custom_vector<int>&    ext_body_id,
    custom_vector<real3>&  ext_body_force,
    custom_vector<real3>&  ext_body_torque)
{
#pragma omp parallel for
  for (int index = 0; index < data_container->num_contacts; index++) {
    function_CalcContactForces(
      index,
      data_container->settings.step_size,
      data_container->host_data.pos_data.data(),
      data_container->host_data.rot_data.data(),
      data_container->host_data.v.data(),
      data_container->host_data.elastic_moduli.data(),
      data_container->host_data.mu.data(),
      data_container->host_data.alpha.data(),
      data_container->host_data.cr.data(),
      data_container->host_data.cohesion_data.data(),
      data_container->host_data.bids_rigid_rigid.data(),
      data_container->host_data.cpta_rigid_rigid.data(),
      data_container->host_data.cptb_rigid_rigid.data(),
      data_container->host_data.norm_rigid_rigid.data(),
      data_container->host_data.dpth_rigid_rigid.data(),
      data_container->host_data.erad_rigid_rigid.data(),
      ext_body_id.data(),
      ext_body_force.data(),
      ext_body_torque.data());
  }
}

// -----------------------------------------------------------------------------
// Include contact impulses (linear and rotational) for all bodies that are
// involved in at least one contact. For each such body, the corresponding
// entries in the input arrays 'ct_body_force' and 'ct_body_torque' contain the
// cummulative force and torque, respectively, over all contacts involving that
// body.
// -----------------------------------------------------------------------------
void
ChLcpSolverParallelDEM::host_AddContactForces(
    uint                         ct_body_count,
    const custom_vector<int>&    ct_body_id,
    const custom_vector<real3>&  ct_body_force,
    const custom_vector<real3>&  ct_body_torque)
{
#pragma omp parallel for
  for (int index = 0; index < ct_body_count; index++) {
	real3 contact_force = data_container->settings.step_size * ct_body_force[index];
	real3 contact_torque = data_container->settings.step_size * ct_body_torque[index];
    data_container->host_data.hf[ct_body_id[index]*6+0] += contact_force.x;
    data_container->host_data.hf[ct_body_id[index]*6+1] += contact_force.x;
    data_container->host_data.hf[ct_body_id[index]*6+2] += contact_force.x;
    data_container->host_data.hf[ct_body_id[index]*6+3] += contact_torque.x;
    data_container->host_data.hf[ct_body_id[index]*6+4] += contact_torque.x;
    data_container->host_data.hf[ct_body_id[index]*6+5] += contact_torque.x;
   }
}

// -----------------------------------------------------------------------------
// Process contact information reported by the narrowphase collision detection,
// generate contact forces, and update the (linear and rotational) impulses for
// all bodies involved in at least one contact.
// -----------------------------------------------------------------------------
void ChLcpSolverParallelDEM::ProcessContacts()
{
  // 0. If the narrowphase collision detection does not set the effective
  //    contact radius, fill it with the value 1.
  if (!data_container->erad_is_set)
    data_container->host_data.erad_rigid_rigid.resize(data_container->num_contacts, 1.0);

  // 1. Calculate contact forces and torques - per contact basis
  //    For each pair of contact shapes that overlap, we calculate and store the
  //    IDs of the two corresponding bodies and the resulting contact forces and
  //    torques on the two bodies.
  custom_vector<int>   ext_body_id(2 * data_container->num_contacts);
  custom_vector<real3> ext_body_force(2 * data_container->num_contacts);
  custom_vector<real3> ext_body_torque(2 * data_container->num_contacts);

  host_CalcContactForces(ext_body_id, ext_body_force, ext_body_torque);

  // 2. Calculate contact forces and torques - per body basis
  //    Accumulate the contact forces and torques for all bodies that are
  //    involved in at least one contact, by reducing the contact forces and
  //    torques from all contacts these bodies are involved in. The number of
  //    bodies that experience at least one contact is calculated in
  //    'ct_body_count'.
  thrust::sort_by_key(thrust_parallel,
    ext_body_id.begin(), ext_body_id.end(),
    thrust::make_zip_iterator(thrust::make_tuple(ext_body_force.begin(), ext_body_torque.begin())));

  custom_vector<int>   ct_body_id(data_container->num_bodies);
  custom_vector<real3> ct_body_force(data_container->num_bodies);
  custom_vector<real3> ct_body_torque(data_container->num_bodies);

  // Reduce contact forces from all contacts and count bodies currently involved
  // in contact. We do this simultaneously for contact forces and torques, using
  // zip iterators.
  uint ct_body_count = thrust::reduce_by_key(
    ext_body_id.begin(),
    ext_body_id.end(),
    thrust::make_zip_iterator(thrust::make_tuple(ext_body_force.begin(), ext_body_torque.begin())),
    ct_body_id.begin(),
    thrust::make_zip_iterator(thrust::make_tuple(ct_body_force.begin(), ct_body_torque.begin())),
    thrust::equal_to<int>(),
    sum_tuples()
    ).first - ct_body_id.begin();

  // 3. Add contact forces and torques to existing forces (impulses):
  //    For all bodies involved in a contact, update the body forces and torques
  //    (scaled by the integration time step).
  host_AddContactForces(ct_body_count, ct_body_id, ct_body_force, ct_body_torque);
}


void ChLcpSolverParallelDEM::ComputeD() {

  CompressedMatrix<real>& D_T = data_container->host_data.D_T;

  uint& num_constraints = data_container->num_constraints;
  uint& num_bodies = data_container->num_bodies;
  uint& num_shafts = data_container->num_shafts;
  uint& num_dof = data_container->num_dof;
  uint& num_contacts = data_container->num_contacts;
  uint& num_bilaterals = data_container->num_bilaterals;
  if (num_constraints <= 0) {
    return;
  }
  clear(D_T);

  int unilateral_reserve = 0;

  int constraint_reserve = num_bilaterals * 6 * 2;

  if (D_T.capacity() < constraint_reserve) {
    D_T.reserve(constraint_reserve * 1.2);
  }

  D_T.resize(num_constraints, num_dof, false);
  bilateral.GenerateSparsity(data_container->settings.solver.solver_mode);
  bilateral.Build_D();

  data_container->host_data.D = trans(data_container->host_data.D_T);
  data_container->host_data.M_invD = data_container->host_data.M_inv * data_container->host_data.D;
}

void ChLcpSolverParallelDEM::ComputeE() {

  uint& num_constraints = data_container->num_constraints;
  if (num_constraints <= 0) {
    return;
  }
  DynamicVector<real>& E = data_container->host_data.E;
  E.resize(num_constraints);
  reset(E);

  bilateral.Build_E();
}


void ChLcpSolverParallelDEM::ComputeR(SOLVERMODE mode) {
  if (data_container->num_constraints <= 0) {
    return;
  }
  data_container->host_data.b.resize(data_container->num_constraints);
  reset(data_container->host_data.b);
  bilateral.Build_b();

  data_container->host_data.R = -data_container->host_data.b - data_container->host_data.D_T * data_container->host_data.M_invk;
}


// -----------------------------------------------------------------------------
// This is the main function for advancing the system state in time. On entry,
// geometric contact information is available as calculated by the narrowphase
// collision detection. This function calculates contact forces, updates the
// generalized velocities, then enforces the velocity-level constraints for any
// bilateral (joint) constraints present in the system.
// -----------------------------------------------------------------------------
void
ChLcpSolverParallelDEM::RunTimeStep(real step)
{
   data_container->settings.step_size = step;

  data_container->num_unilaterals = 0;
  // This is the total number of constraints, note that there are no contacts
  data_container->num_constraints = data_container->num_bilaterals;

  // This is the total number of degrees of freedom in the system
  data_container->num_dof = data_container->num_bodies  * 6 + data_container->num_shafts;

  // Calculate contact forces (impulses) and append them to the body forces
  if (data_container->num_contacts > 0) {
    data_container->system_timer.start("ChLcpSolverParallelDEM_ProcessContact");
    ProcessContacts();
    data_container->system_timer.stop("ChLcpSolverParallelDEM_ProcessContact");
  }

  // Generate the mass matrix and compute M_inv_k
  ComputeMassMatrix();

  // If there are (bilateral) constraints, calculate Lagrange multipliers.
  if (data_container->num_constraints == 0) {

    // Perform stabilization of the bilateral constraints. Currently, we only 
    // project the velocities onto the velocity constraint manifold. This is done
    // with an oblique projection (using the M-norm).
    data_container->system_timer.start("ChLcpSolverParallel_Setup");

    bilateral.Setup(data_container);

    solver->current_iteration = 0;
    data_container->measures.solver.total_iteration = 0;
    data_container->measures.solver.maxd_hist.clear();               ////
    data_container->measures.solver.maxdeltalambda_hist.clear();     ////  currently not used
    data_container->measures.solver.iter_hist.clear();               ////

    solver->bilateral = &bilateral;
    solver->Setup(data_container);

    // Set the initial guess for the iterative solver to zero.
    data_container->host_data.gamma.resize(data_container->num_constraints);
    data_container->host_data.gamma.reset();

    //Compute the jacobian matrix, the compliance matrix and the right hand side
    ComputeD();
    ComputeE();
    ComputeR(NORMAL);

    data_container->system_timer.stop("ChLcpSolverParallel_Setup");

    ////First copy the gamma's from the previous timestep into the gamma vector
    ////Currently because the initial guess is set to zero, this doesn't do anything so it has been commented out
    //#pragma omp parallel for
    //  for (int i = 0; i < data_container->num_bilaterals; i++) {
    //    data_container->host_data.gamma[i + data_container->num_unilaterals] = data_container->host_data.gamma_bilateral[i];
    //  }

    // This will solve the system for only the bilaterals
    PerformStabilization();

    // Copy multipliers in the vector specific to bilaterals (these are the values
    // used in calculating reaction forces)
    data_container->host_data.gamma_bilateral.resize(data_container->num_bilaterals);

#pragma omp parallel for
    for (int i = 0; i < data_container->num_bilaterals; i++) {
      data_container->host_data.gamma_bilateral[i] = data_container->host_data.gamma[i + data_container->num_unilaterals];
    }
  }

  // Update velocity (linear and angular)
  ComputeImpulses();

  for (int i = 0; i <  data_container->measures.solver.iter_hist.size(); i++) {
    AtIterationEnd( data_container->measures.solver.maxd_hist[i],  data_container->measures.solver.maxdeltalambda_hist[i],  data_container->measures.solver.iter_hist[i]);
  }
}
