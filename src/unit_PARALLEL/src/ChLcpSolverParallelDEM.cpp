#include "ChLcpSolverParallel.h"
#include "math/ChThrustLinearAlgebra.h"


using namespace chrono;


//// TODO:  For now, this is hard-coded for Hunt-Crossley with Simple Sliding Friction.
__host__ __device__
void function_CalcContactForces(int&       index,           // index of this contact pair
                                real&      dT,              // integration time step
                                real3*     pos,             // body positions
                                real4*     rot,             // body orientations
                                real3*     vel,             // body linear velocity (global frame)
                                real3*     omg,             // body angular velocity (local frame)
                                real2*     elastic_moduli,  // Young's modulus (per body)
                                real*      mu,              // coefficient of friction (per body)
                                real*      alpha,           // disipation coefficient (per body)
                                real*      cr,              // coefficient of restitution (per body)
                                real*      cohesion,        // cohesion force (per body)
                                long long* pairs,           // shape IDs (per contact pairs)
                                uint*      body_id,         // body IDs (per shape)
                                real3*     pt1,             // point on shape 1 (per contact)
                                real3*     pt2,             // point on shape 2 (per contact)
                                real3*     normal,          // contact normal (per contact)
                                real*      depth,           // penetration depth (per contact)
                                real*      eff_radius,      // effective contact radius (per contact)
                                int*       ext_body_id,     // [output] body IDs (two per contact)
                                real3*     ext_body_force,  // [output] body force (two per contact)
                                real3*     ext_body_torque) // [output] body torque (two per contact)
{
	// Identify the two bodies in contact. First, find the IDs of the shapes in
	// contact, then find the IDs of the associated bodies.
	int2 pair = I2(int(pairs[index] >> 32), int(pairs[index] & 0xffffffff));
	int body1 = body_id[pair.x];
	int body2 = body_id[pair.y];

	// If the two contact shapes are actually separated, set zero forces and torques
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
	real3 vel1 = vel[body1] + quatRotateMat(cross(omg[body1], pt1_loc), rot[body1]);
	real3 vel2 = vel[body2] + quatRotateMat(cross(omg[body2], pt2_loc), rot[body2]);
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

	real mu_eff = min(mu[body1], mu[body2]);
	//real cr_eff = (cr[body1] + cr[body2]) / 2;
	real alpha_eff = (alpha[body1] + alpha[body2]) / 2;

	real cohesion_eff = min(cohesion[body1], cohesion[body2]);

	// Contact force
	// -------------

	// Normal force: Hunt-Crossley
	real delta = -depth[index];
	real kn = (4.0 / 3) * E_eff * sqrt(eff_radius[index]);
	real forceN_elastic = kn * delta * sqrt(delta);
	real forceN_dissipation = 1.5 * alpha_eff * forceN_elastic * relvel_n_mag;
	real forceN = forceN_elastic - forceN_dissipation;

	// Cohesion force
	forceN -= cohesion_eff;

	real3 force = forceN * normal[index];

	// Tangential force: Simple Coulomb Sliding
	if (relvel_t_mag > 1e-4) {
		real forceT = mu_eff * abs(forceN);
	
		force -= (forceT / relvel_t_mag) * relvel_t;
	}

	// Body forces (in global frame) & torques (in local frame)
	// --------------------------------------------------------

	// Convert force into the local body frames and calculate induced torques
	//    n' = s' x F' = s' x (A*F)
	real3 torque1_loc = cross(pt1_loc, quatRotateMatT(force, rot[body1]));
	real3 torque2_loc = cross(pt2_loc, quatRotateMatT(force, rot[body2]));

	// Store body forces and torques
	ext_body_id[2*index]   = body1;
	ext_body_id[2*index+1] = body2;
	ext_body_force[2*index]   = -force;
	ext_body_force[2*index+1] =  force;
	ext_body_torque[2*index]   = -torque1_loc;
	ext_body_torque[2*index+1] =  torque2_loc;
}


void ChLcpSolverParallelDEM::host_CalcContactForces(custom_vector<int>&   ext_body_id,
                                                    custom_vector<real3>& ext_body_force,
                                                    custom_vector<real3>& ext_body_torque)
{
#pragma omp parallel for
	for (int index = 0; index < data_container->number_of_rigid_rigid; index++) {
		function_CalcContactForces(
			index,
			step_size,
			data_container->host_data.pos_data.data(),
			data_container->host_data.rot_data.data(),
			data_container->host_data.vel_data.data(),
			data_container->host_data.omg_data.data(),
			data_container->host_data.elastic_moduli.data(),
			data_container->host_data.mu.data(),
			data_container->host_data.alpha.data(),
			data_container->host_data.cr.data(),
			data_container->host_data.cohesion_data.data(),
			data_container->host_data.pair_rigid_rigid.data(),
			data_container->host_data.id_rigid.data(),
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


void ChLcpSolverParallelDEM::host_AddContactForces(uint                        ct_body_count,
                                                   const custom_vector<int>&   ct_body_id,
                                                   const custom_vector<real3>& ct_body_force,
                                                   const custom_vector<real3>& ct_body_torque)
{
#pragma omp parallel for
	for (int index = 0; index < ct_body_count; index++) {
		data_container->host_data.frc_data[ct_body_id[index]] += step_size * ct_body_force[index];
		data_container->host_data.trq_data[ct_body_id[index]] += step_size * ct_body_torque[index];
	}
}


void ChLcpSolverParallelDEM::ProcessContacts()
{
	// 0. If the narrowphase collision detection does not set the effective contact
	//    radius, fill it with the value 1.
	if (!data_container->erad_is_set)
		data_container->host_data.erad_rigid_rigid.resize(data_container->number_of_rigid_rigid, 1.0);

	// 1. Calculate contact forces and torques - per contact basis
	//    For each pair of contact shapes that overlap, we calculate and store
	//    the IDs of the two corresponding bodies and the resulting contact
	//    forces and torques on the two bodies.
	custom_vector<int>   ext_body_id(2 * data_container->number_of_rigid_rigid);
	custom_vector<real3> ext_body_force(2 * data_container->number_of_rigid_rigid);
	custom_vector<real3> ext_body_torque(2 * data_container->number_of_rigid_rigid);

	host_CalcContactForces(ext_body_id, ext_body_force, ext_body_torque);

	// 2. Calculate contact forces and torques - per body basis
	//    Accumulate the contact forces and torques for all bodies that are involved
	//    in at least one contact, by reducing the contact forces and torques from
	//    all contacts these bodies are involved in. The number of bodies that
	//    experience at least one contact is calculated in 'ct_body_count'.
	thrust::sort_by_key(
		ext_body_id.begin(), ext_body_id.end(),
		thrust::make_zip_iterator(thrust::make_tuple(ext_body_force.begin(), ext_body_torque.begin())));

	custom_vector<int>   ct_body_id(data_container->number_of_rigid);
	custom_vector<real3> ct_body_force(data_container->number_of_rigid);
	custom_vector<real3> ct_body_torque(data_container->number_of_rigid);

	// Reduce contact forces from all contacts and count bodies currently involved in contact
	uint ct_body_count = thrust::reduce_by_key(
		ext_body_id.begin(),
		ext_body_id.end(),
		thrust::make_zip_iterator(thrust::make_tuple(ext_body_force.begin(), ext_body_torque.begin())),
		ct_body_id.begin(),
		thrust::make_zip_iterator(thrust::make_tuple(ct_body_force.begin(), ct_body_torque.begin())),
		thrust::equal_to<int>(),
		sum_tuples()
		).first - ct_body_id.begin();

	// 3. Add contact forces and torques to existing forces (impulses)
	//    For all bodies involved in a contact, update the body forces and torques,
	//    scaled by the integration time step.
	host_AddContactForces(ct_body_count, ct_body_id, ct_body_force, ct_body_torque);
}


void ChLcpSolverParallelDEM::RunTimeStep(real step)
{
	step_size = step;
	data_container->step_size = step;

	number_of_constraints = data_container->number_of_bilaterals;
	number_of_objects = data_container->number_of_rigid;

	// Calculate contact forces (impulses) and append them to the body forces
	if (data_container->number_of_rigid_rigid)
		ProcessContacts();

	// Include forces and torques (update derivatives: v += m_inv * h * f)
	Preprocess();


	//// TODO:  check and clean up everything that has to do with bilateral constraints...


	data_container->host_data.rhs_data.resize(number_of_constraints);
	data_container->host_data.diag.resize(number_of_constraints);

	data_container->host_data.gamma_data.resize((number_of_constraints));

#pragma omp parallel for
	for (int i = 0; i < number_of_constraints; i++) {
		data_container->host_data.gamma_data[i] = 0;
	}

	bilateral.Setup(data_container);

	solver.current_iteration = 0;
	solver.total_iteration = 0;
	solver.maxd_hist.clear();
	solver.maxdeltalambda_hist.clear();
	solver.iter_hist.clear();

	solver.SetMaxIterations(max_iteration);
	solver.SetTolerance(tolerance);

	//solver.SetComplianceAlpha(alpha);
	solver.SetContactRecoverySpeed(contact_recovery_speed);
	solver.lcp_omega_bilateral = lcp_omega_bilateral;
	//solver.rigid_rigid = &rigid_rigid;
	solver.bilateral = &bilateral;
	//solver.lcp_omega_contact = lcp_omega_contact;
	solver.do_stab = do_stab;
	solver.collision_inside = collision_inside;
	solver.Initial(step, data_container);

	bilateral.ComputeJacobians();
	bilateral.ComputeRHS();

	if (max_iter_bilateral > 0) {
		custom_vector<real> rhs_bilateral(data_container->number_of_bilaterals);
		thrust::copy_n(data_container->host_data.rhs_data.begin(), data_container->number_of_bilaterals, rhs_bilateral.begin());
		solver.SolveStab(data_container->host_data.gamma_bilateral, rhs_bilateral, max_iter_bilateral);
	}

	thrust::copy_n(data_container->host_data.gamma_bilateral.begin(), data_container->number_of_bilaterals, data_container->host_data.gamma_data.begin());
}

