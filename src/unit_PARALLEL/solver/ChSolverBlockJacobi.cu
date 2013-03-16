#include "ChSolverBlockJacobi.h"
using namespace chrono;

__host__ __device__ void function_process_contacts2(
		uint &index,
		real& step_size,
		real& lcp_contact_factor,
		uint& number_of_contacts,
		real& lcp_omega_contact,
		real3* JXYZA,
		real3* JXYZB,
		real3* JUVWA,
		real3* JUVWB,
		real* contactDepth,
		int2* ids,
		real3* G,
		real* dG,
		real* inv_mass,
		real* fric,
		real3* inv_inertia,
		real4* rot,
		real3* vel,
		real3* omega,
		real3* pos,
		real3* updateV,
		real3* updateO,
		uint* offset) {

	int2 body_id = ids[index];
	real depth = -fabs(contactDepth[index]);

	real bi = fmaxf((depth / (step_size)), -lcp_contact_factor);
	real3 W1 = omega[body_id.x];
	real3 W2 = omega[body_id.y];
	real3 V1 = vel[body_id.x];
	real3 V2 = vel[body_id.y];
	real3 gamma;
	gamma.x = dot(JUVWA[index + number_of_contacts * 0], W1) + dot(JXYZA[index + number_of_contacts * 0], V1) + dot(JUVWB[index + number_of_contacts * 0], W2)
			+ dot(JXYZB[index + number_of_contacts * 0], V2) + bi /*+ cfm * gamma_old.x*/; //+bi

	gamma.y = dot(JUVWA[index + number_of_contacts * 1], W1) + dot(JXYZA[index + number_of_contacts * 1], V1) + dot(JUVWB[index + number_of_contacts * 1], W2)
			+ dot(JXYZB[index + number_of_contacts * 1], V2) /*+ cfmT * gamma_old.y*/;

	gamma.z = dot(JUVWA[index + number_of_contacts * 2], W1) + dot(JXYZA[index + number_of_contacts * 2], V1) + dot(JUVWB[index + number_of_contacts * 2], W2)
			+ dot(JXYZB[index + number_of_contacts * 2], V2) /*+ cfmT * gamma_old.z*/;
	real3 In1 = inv_inertia[body_id.x]; // bring in the inertia attributes; to be used to compute \eta
	real3 In2 = inv_inertia[body_id.y]; // bring in the inertia attributes; to be used to compute \eta

	real eta = dot(JUVWA[index + number_of_contacts * 0] * JUVWA[index + number_of_contacts * 0], In1) + dot(JUVWA[index + number_of_contacts * 1] * JUVWA[index + number_of_contacts * 1], In1)
			+ dot(JUVWA[index + number_of_contacts * 2] * JUVWA[index + number_of_contacts * 2], In1); // update expression of eta

	eta += dot(JUVWB[index + number_of_contacts * 0] * JUVWB[index + number_of_contacts * 0], In2) + dot(JUVWB[index + number_of_contacts * 1] * JUVWB[index + number_of_contacts * 1], In2)
			+ dot(JUVWB[index + number_of_contacts * 2] * JUVWB[index + number_of_contacts * 2], In2);

	eta += dot(JXYZA[index + number_of_contacts * 0], JXYZA[index + number_of_contacts * 0]) * inv_mass[body_id.x]
			+ dot(JXYZA[index + number_of_contacts * 1], JXYZA[index + number_of_contacts * 1]) * inv_mass[body_id.x]
			+ dot(JXYZA[index + number_of_contacts * 2], JXYZA[index + number_of_contacts * 2]) * inv_mass[body_id.x];

	eta += dot(JXYZB[index + number_of_contacts * 0], JXYZB[index + number_of_contacts * 0]) * inv_mass[body_id.y]
			+ dot(JXYZB[index + number_of_contacts * 1], JXYZB[index + number_of_contacts * 1]) * inv_mass[body_id.y]
			+ dot(JXYZB[index + number_of_contacts * 2], JXYZB[index + number_of_contacts * 2]) * inv_mass[body_id.y];

	eta = 3 * lcp_omega_contact / eta; // final value of eta
	real3 gamma_old = G[index];
	gamma = eta * gamma; // perform gamma *= omega*eta
	gamma = gamma_old - gamma; // perform gamma = gamma_old - gamma ;  in place.
	/// ---- perform projection of 'a8' onto friction cone  --------

	real f_tang = sqrtf(gamma.y * gamma.y + gamma.z * gamma.z);
	real mu = (fric[body_id.x] + fric[body_id.y]) * .5f;

	if (f_tang > (mu * gamma.x)) { // inside upper cone? keep untouched!
		if ((f_tang) < -(1.0 / mu) * gamma.x || (fabsf(gamma.x) < 0)) { // inside lower cone? reset  normal,u,v to zero!
			gamma = R3(0);
		} else { // remaining case: project orthogonally to generator segment of upper cone
			gamma.x = (f_tang * mu + gamma.x) / (mu * mu + 1);
			real tproj_div_t = (gamma.x * mu) / f_tang; //  reg = tproj_div_t
			gamma.y *= tproj_div_t;
			gamma.z *= tproj_div_t;
		}
	} else if (mu == 0) {
		gamma.y = gamma.z = 0;
	}

	G[index] = gamma; // store gamma_new
	gamma -= gamma_old; // compute delta_gamma = gamma_new - gamma_old   = delta_gamma.
	dG[index] = length(gamma);
	real3 vB1 = JXYZA[index + number_of_contacts * 0] * gamma.x + JXYZA[index + number_of_contacts * 1] * gamma.y + JXYZA[index + number_of_contacts * 2] * gamma.z;
	real3 vB2 = JXYZB[index + number_of_contacts * 0] * gamma.x + JXYZB[index + number_of_contacts * 1] * gamma.y + JXYZB[index + number_of_contacts * 2] * gamma.z;

	int offset1 = offset[index];
	int offset2 = offset[index + number_of_contacts];
	updateV[offset1] = real3(vB1 * inv_mass[body_id.x]); // compute and store dv1
	updateO[offset1] = real3((JUVWA[index + number_of_contacts * 0] * gamma.x + JUVWA[index + number_of_contacts * 1] * gamma.y + JUVWA[index + number_of_contacts * 2] * gamma.z) * In1); // compute dw1 =  Inert.1' * J1w^ * deltagamma  and store  dw1
	updateV[offset2] = real3(vB2 * inv_mass[body_id.y]); // compute and store dv2
	updateO[offset2] = real3((JUVWB[index + number_of_contacts * 0] * gamma.x + JUVWB[index + number_of_contacts * 1] * gamma.y + JUVWB[index + number_of_contacts * 2] * gamma.z) * In2); // compute dw2 =  Inert.2' * J2w^ * deltagamma  and store  dw2
}

__host__ __device__ void function_Bilaterals2(
		uint& index,
		uint& number_of_bilaterals,
		uint& number_of_contacts,
		real& lcp_omega_bilateral,
		real4* bilaterals,
		real* mass,
		real3* inertia,
		real4* rot,
		real3* vel,
		real3* omega,
		real3* pos,
		real3* updateV,
		real3* updateO,
		uint* offset,
		real* dG) {
	real4 vA;
	real3 vB;
	real gamma_new = 0, gamma_old = 0;
	int B1_index = 0, B2_index = 0;
	B1_index = bilaterals[index].w;
	B2_index = bilaterals[index + number_of_bilaterals].w;
	real mass1 = mass[B1_index];
	real mass2 = mass[B2_index];
	// ---- perform   gamma_new = ([J1 J2] {v1 | v2}^ + b)
	{
		vA = bilaterals[index]; // line 0
		vB = vel[B1_index]; // v1
		gamma_new += dot3(vA, vB);
		vA = bilaterals[index + 2 * number_of_bilaterals]; // line 2
		vB = omega[B1_index]; // w1
		gamma_new += dot3(vA, vB);
	}
	{
		vA = bilaterals[index + number_of_bilaterals]; // line 1
		vB = vel[B2_index]; // v2
		gamma_new += dot3(vA, vB);
		vA = bilaterals[index + 3 * number_of_bilaterals]; // line 3
		vB = omega[B2_index]; // w2
		gamma_new += dot3(vA, vB);
	}
	vA = bilaterals[index + 4 * number_of_bilaterals]; // line 4   (eta, b, gamma, 0)
	gamma_new += vA.y; // add known term     + b
	gamma_old = vA.z; // old gamma
	/// ---- perform gamma_new *= omega/g_i
	gamma_new *= lcp_omega_bilateral; // lcp_omega_const is in constant memory
	gamma_new *= vA.x; // eta = 1/g_i;
	/// ---- perform gamma_new = gamma_old - gamma_new ; in place.
	gamma_new = gamma_old - gamma_new;

	/// ---- perform projection of 'a' (only if simple unilateral behavior C>0 is requested)
	if (vA.w && gamma_new < 0.) {
		gamma_new = 0.;
	}

	// ----- store gamma_new
	vA.z = gamma_new;
	bilaterals[index + 4 * number_of_bilaterals] = vA;
	/// ---- compute delta in multipliers: gamma_new = gamma_new - gamma_old   = delta_gamma    , in place.
	gamma_new -= gamma_old;
	dG[number_of_contacts + index] = (gamma_new);
	/// ---- compute dv1 =  invInert.18 * J1^ * deltagamma
	vB = inertia[B1_index]; // iJ iJ iJ im
	vA = (bilaterals[index]) * mass1 * gamma_new; // line 0: J1(x)
	int offset1 = offset[2 * number_of_contacts + index];
	int offset2 = offset[2 * number_of_contacts + index + number_of_bilaterals];
	updateV[offset1] = make_real3(vA); //  ---> store  v1 vel. in reduction buffer
	updateO[offset1] = make_real3((bilaterals[index + 2 * number_of_bilaterals]) * R4(vB) * gamma_new); // line 2:  J1(w)// ---> store  w1 vel. in reduction buffer
	vB = inertia[B2_index]; // iJ iJ iJ im
	vA = (bilaterals[index + number_of_bilaterals]) * mass2 * gamma_new; // line 1: J2(x)
	updateV[offset2] = make_real3(vA); //  ---> store  v2 vel. in reduction buffer
	updateO[offset2] = make_real3((bilaterals[index + 3 * number_of_bilaterals]) * R4(vB) * gamma_new); // line 3:  J2(w)// ---> store  w2 vel. in reduction buffer
}

ChSolverJacobi::ChSolverJacobi() {
}

void ChSolverJacobi::Solve() {

//#ifdef SIM_ENABLE_GPU_MODE
//	COPY_TO_CONST_MEM(compliance);
//	COPY_TO_CONST_MEM(complianceT);
//	COPY_TO_CONST_MEM(alpha);
//	COPY_TO_CONST_MEM(lcp_omega_bilateral);
//	COPY_TO_CONST_MEM(lcp_omega_contact);
//	COPY_TO_CONST_MEM(lcp_contact_factor);
//	COPY_TO_CONST_MEM(number_of_updates);
//
//	cudaFuncSetCacheConfig(device_process_contacts, cudaFuncCachePreferL1);
//	cudaFuncSetCacheConfig(device_Bilaterals, cudaFuncCachePreferL1);
//	cudaFuncSetCacheConfig(device_Reduce_Speeds, cudaFuncCachePreferL1);
//#else
//#endif
//
//	if (number_of_constraints != 0) {
//		for (iteration_number = 0; iteration_number < max_iterations; iteration_number++) {
//#ifdef SIM_ENABLE_GPU_MODE
//			device_process_contacts CUDA_KERNEL_DIM(BLOCKS(number_of_contacts), THREADS)(CASTR3(gpu_data.device_JXYZA_data), CASTR3(gpu_data.device_JXYZB_data), CASTR3(gpu_data.device_JUVWA_data),
//					CASTR3(gpu_data.device_JUVWB_data), CASTR1(gpu_data.device_dpth_data), CASTI2(gpu_data.device_bids_data), CASTR3(gpu_data.device_gam_data), CASTR1(gpu_data.device_dgm_data),
//					CASTR1(gpu_data.device_mass_data), CASTR1(gpu_data.device_fric_data), CASTR3(gpu_data.device_inr_data), CASTR4(gpu_data.device_rot_data), CASTR3(gpu_data.device_vel_data),
//					CASTR3(gpu_data.device_omg_data), CASTR3(gpu_data.device_pos_data), CASTR3(gpu_data.vel_update), CASTR3(gpu_data.omg_update), CASTU1(gpu_data.update_offset));
//#else
//			host_process_contacts(
//					gpu_data.device_JXYZA_data.data(),
//					gpu_data.device_JXYZB_data.data(),
//					gpu_data.device_JUVWA_data.data(),
//					gpu_data.device_JUVWB_data.data(),
//					gpu_data.device_dpth_data.data(),
//					gpu_data.device_bids_data.data(),
//					gpu_data.device_gam_data.data(),
//					gpu_data.device_dgm_data.data(),
//					gpu_data.device_mass_data.data(),
//					gpu_data.device_fric_data.data(),
//					gpu_data.device_inr_data.data(),
//					gpu_data.device_rot_data.data(),
//					gpu_data.device_vel_data.data(),
//					gpu_data.device_omg_data.data(),
//					gpu_data.device_pos_data.data(),
//					gpu_data.vel_update.data(),
//					gpu_data.omg_update.data(),
//					gpu_data.update_offset.data());
//#endif
//#ifdef SIM_ENABLE_GPU_MODE
//			device_Bilaterals CUDA_KERNEL_DIM(BLOCKS(number_of_bilaterals), THREADS)(CASTR4(gpu_data.device_bilateral_data), CASTR1(gpu_data.device_mass_data), CASTR3(gpu_data.device_inr_data),
//					CASTR4(gpu_data.device_rot_data), CASTR3(gpu_data.device_vel_data), CASTR3(gpu_data.device_omg_data), CASTR3(gpu_data.device_pos_data), CASTR3(gpu_data.vel_update),
//					CASTR3(gpu_data.omg_update), CASTU1(gpu_data.update_offset), CASTR1(gpu_data.device_dgm_data));
//#else
//			host_Bilaterals(
//					gpu_data.device_bilateral_data.data(),
//					gpu_data.device_mass_data.data(),
//					gpu_data.device_inr_data.data(),
//					gpu_data.device_rot_data.data(),
//					gpu_data.device_vel_data.data(),
//					gpu_data.device_omg_data.data(),
//					gpu_data.device_pos_data.data(),
//					gpu_data.vel_update.data(),
//					gpu_data.omg_update.data(),
//					gpu_data.update_offset.data(),
//					gpu_data.device_dgm_data.data());
//#endif
//#ifdef SIM_ENABLE_GPU_MODE
//			device_Reduce_Speeds CUDA_KERNEL_DIM(BLOCKS(number_of_updates), THREADS)(CASTB1(gpu_data.device_active_data), CASTR1(gpu_data.device_mass_data), CASTR3(gpu_data.device_vel_data),
//					CASTR3(gpu_data.device_omg_data), CASTR3(gpu_data.vel_update), CASTR3(gpu_data.omg_update), CASTU1(gpu_data.body_number), CASTU1(gpu_data.offset_counter),
//					CASTR3(gpu_data.device_fap_data));
//#else
//			host_Reduce_Speeds(
//					gpu_data.device_active_data.data(),
//					gpu_data.device_mass_data.data(),
//					gpu_data.device_vel_data.data(),
//					gpu_data.device_omg_data.data(),
//					gpu_data.vel_update.data(),
//					gpu_data.omg_update.data(),
//					gpu_data.body_number.data(),
//					gpu_data.offset_counter.data(),
//					gpu_data.device_fap_data.data());
//#endif
//
//			if (tolerance != 0) {
//				if (iteration_number > 20 && iteration_number % 20 == 0) {
//					real gam = Thrust_Max(gpu_data.device_dgm_data);
//
//					if (fabsf(old_gam - gam) < tolerance) {
//						break;
//					}
//
//					old_gam = gam;
//				}
//			}
//		}
//	}

}
