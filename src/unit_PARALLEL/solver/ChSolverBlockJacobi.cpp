#include "ChSolverGPU.h"
using namespace chrono;

__constant__ real lcp_omega_bilateral_const;
__constant__ real lcp_omega_contact_const;
__constant__ real contact_recovery_speed_const;
__constant__ uint number_of_objects_const;
__constant__ uint number_of_contacts_const;
__constant__ uint number_of_constraints_const;
__constant__ uint number_of_bilaterals_const;
__constant__ uint number_of_updates_const;
__constant__ real step_size_const;
__constant__ real compliance_const;
__constant__ real complianceT_const;
__constant__ real alpha_const; // [R]=alpha*[K]

__host__ __device__ void function_Project_jacobi(uint & index, real3 & gamma, real* fric, int2* ids) {
	int2 body_id = ids[index];

	real f_tang = sqrt(gamma.y * gamma.y + gamma.z * gamma.z);
	real mu = (fric[body_id.x] == 0 || fric[body_id.y] == 0) ? 0 : (fric[body_id.x] + fric[body_id.y]) * .5;

	if (mu == 0) {
		gamma.x = gamma.x < 0 ? 0 : gamma.x;
		gamma.y = gamma.z = 0;
		return;
	}
	if (f_tang < (mu * gamma.x)) {
		return;
	}
	if ((f_tang) < -(1.0 / mu) * gamma.x || (fabs(gamma.x) < 10e-15)) {
		gamma = R3(0);
		return;
	}
	gamma.x = (f_tang * mu + gamma.x) / (mu * mu + 1);
	real tproj_div_t = (gamma.x * mu) / f_tang;
	gamma.y *= tproj_div_t;
	gamma.z *= tproj_div_t;

}

__host__ __device__ void function_process_contacts(
		uint &index, real& step_size, real& contact_recovery_speed, uint& number_of_contacts, real& lcp_omega_contact, real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB, real * rhs,
		real* contactDepth, bool* active, int2* ids, real* G, real* dG, real* inv_mass, real* fric, real3* inv_inertia, real4* rot, real3* vel, real3* omega, real3* pos, real3* updateV,
		real3* updateO, uint* offset) {

	int2 body_id = ids[index];

	real bi = fmax(contactDepth[index] / (step_size), -contact_recovery_speed);

	real3 W1 = omega[body_id.x];
	real3 W2 = omega[body_id.y];
	real3 V1 = vel[body_id.x];
	real3 V2 = vel[body_id.y];
	real3 gamma = R3(0), gamma_old;
	bool active_a = active[body_id.x];
	bool active_b = active[body_id.y];
	real eta = 0;
//	gamma.x = -rhs[index + number_of_contacts * 0];
//	gamma.y = -rhs[index + number_of_contacts * 1];
//	gamma.z = -rhs[index + number_of_contacts * 2];

	real3 In1 = inv_inertia[body_id.x]; // bring in the inertia attributes; to be used to compute \eta
	real3 In2 = inv_inertia[body_id.y]; // bring in the inertia attributes; to be used to compute \eta

	if (active_a) {
		gamma.x += dot(JUVWA[index + number_of_contacts * 0], W1) + dot(JXYZA[index + number_of_contacts * 0], V1);
		gamma.y += dot(JUVWA[index + number_of_contacts * 1], W1) + dot(JXYZA[index + number_of_contacts * 1], V1);
		gamma.z += dot(JUVWA[index + number_of_contacts * 2], W1) + dot(JXYZA[index + number_of_contacts * 2], V1);

		eta += dot(JUVWA[index + number_of_contacts * 0] * JUVWA[index + number_of_contacts * 0], In1) + dot(JUVWA[index + number_of_contacts * 1] * JUVWA[index + number_of_contacts * 1], In1)
				+ dot(JUVWA[index + number_of_contacts * 2] * JUVWA[index + number_of_contacts * 2], In1);

		eta += dot(JXYZA[index + number_of_contacts * 0], JXYZA[index + number_of_contacts * 0]) * inv_mass[body_id.x]
				+ dot(JXYZA[index + number_of_contacts * 1], JXYZA[index + number_of_contacts * 1]) * inv_mass[body_id.x]
				+ dot(JXYZA[index + number_of_contacts * 2], JXYZA[index + number_of_contacts * 2]) * inv_mass[body_id.x];

	}
	if (active_b) {
		gamma.x += dot(JUVWB[index + number_of_contacts * 0], W2) + dot(JXYZB[index + number_of_contacts * 0], V2); // + bi; // + .01 * G[index + number_of_contacts * 0]; //+bi
		gamma.y += dot(JUVWB[index + number_of_contacts * 1], W2) + dot(JXYZB[index + number_of_contacts * 1], V2) /*+ cfmT * gamma_old.y*/;
		gamma.z += dot(JUVWB[index + number_of_contacts * 2], W2) + dot(JXYZB[index + number_of_contacts * 2], V2) /*+ cfmT * gamma_old.z*/;

		eta += dot(JUVWB[index + number_of_contacts * 0] * JUVWB[index + number_of_contacts * 0], In2) + dot(JUVWB[index + number_of_contacts * 1] * JUVWB[index + number_of_contacts * 1], In2)
				+ dot(JUVWB[index + number_of_contacts * 2] * JUVWB[index + number_of_contacts * 2], In2);

		eta += dot(JXYZB[index + number_of_contacts * 0], JXYZB[index + number_of_contacts * 0]) * inv_mass[body_id.y]
				+ dot(JXYZB[index + number_of_contacts * 1], JXYZB[index + number_of_contacts * 1]) * inv_mass[body_id.y]
				+ dot(JXYZB[index + number_of_contacts * 2], JXYZB[index + number_of_contacts * 2]) * inv_mass[body_id.y];
	}
	gamma.x += bi;

	//std::cout << gamma.x << " ";
	//std::cout << gamma.y << " ";
	//std::cout << gamma.z << std::endl;

	dG[index + number_of_contacts * 0] = fabs(fmin(real(0.0), gamma.x));
	dG[index + number_of_contacts * 1] = gamma.y;
	dG[index + number_of_contacts * 2] = gamma.z;

	gamma = 3.0 * lcp_omega_contact / (eta) * -gamma; // perform gamma *= omega*eta

	gamma_old.x = G[index + number_of_contacts * 0]; // store gamma_new];
	gamma_old.y = G[index + number_of_contacts * 1]; // store gamma_new];
	gamma_old.z = G[index + number_of_contacts * 2]; // store gamma_new];

	//std::cout << gamma.x << std::endl;
	//std::cout << gamma.y << " ";
	//std::cout << gamma.z << std::endl;

	gamma = gamma_old + gamma; // perform gamma = gamma_old - gamma ;  in place.

	/// ---- perform projection of 'a8' onto friction cone  --------

	function_Project_jacobi(index, gamma, fric, ids);

	//if(index ==158){
	//	cout<<"GPU"<<endl;
	//					std::cout<<gamma.x<<std::endl;
	//					std::cout<<gamma.y<<std::endl;
	//					std::cout<<gamma.z<<std::endl;
	//					}

	G[index + number_of_contacts * 0] = gamma.x; // store gamma_new
	G[index + number_of_contacts * 1] = gamma.y; // store gamma_new
	G[index + number_of_contacts * 2] = gamma.z; // store gamma_new
	gamma -= gamma_old; // compute delta_gamma = gamma_new - gamma_old   = delta_gamma.

	real3 vB1 = JXYZA[index + number_of_contacts * 0] * gamma.x + JXYZA[index + number_of_contacts * 1] * gamma.y + JXYZA[index + number_of_contacts * 2] * gamma.z;
	real3 vB2 = JXYZB[index + number_of_contacts * 0] * gamma.x + JXYZB[index + number_of_contacts * 1] * gamma.y + JXYZB[index + number_of_contacts * 2] * gamma.z;

	int offset1 = offset[index];
	int offset2 = offset[index + number_of_contacts];
	updateV[offset1] = real3(vB1 * inv_mass[body_id.x]); // compute and store dv1
	updateO[offset1] = real3((JUVWA[index + number_of_contacts * 0] * gamma.x + JUVWA[index + number_of_contacts * 1] * gamma.y + JUVWA[index + number_of_contacts * 2] * gamma.z) * In1); // compute dw1 =  Inert.1' * J1w^ * deltagamma  and store  dw1
	updateV[offset2] = real3(vB2 * inv_mass[body_id.y]); // compute and store dv2
	updateO[offset2] = real3((JUVWB[index + number_of_contacts * 0] * gamma.x + JUVWB[index + number_of_contacts * 1] * gamma.y + JUVWB[index + number_of_contacts * 2] * gamma.z) * In2); // compute dw2 =  Inert.2' * J2w^ * deltagamma  and store  dw2

}

//  Kernel for a single iteration of the LCP over all contacts
//  Version 2.0 - Tasora
//  Version 2.2- Hammad (optimized, cleaned etc)


void ChSolverGPU::host_process_contacts(
		real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB, real * rhs, real* contactDepth, bool * active, int2* ids, real* gamma, real* dG, real* mass, real* fric, real3* inertia, real4* rot,
		real3* vel, real3* omega, real3* pos, real3* updateV, real3* updateO, uint* offset) {
#pragma omp parallel for schedule(guided)

	for (uint index = 0; index < number_of_rigid_rigid; index++) {
		function_process_contacts(
				index,
				step_size,
				contact_recovery_speed,
				number_of_rigid_rigid,
				lcp_omega_contact,
				JXYZA,
				JXYZB,
				JUVWA,
				JUVWB,
				rhs,
				contactDepth,
				active,
				ids,
				gamma,
				dG,
				mass,
				fric,
				inertia,
				rot,
				vel,
				omega,
				pos,
				updateV,
				updateO,
				offset);
	}
}

///////////////////////////////////////////////////////////////////////////////////
// Kernel for a single iteration of the LCP over all scalar bilateral contacts
// (a bit similar to the ChKernelLCPiteration above, but without projection etc.)
// Version 2.0 - Tasora
//

__host__ __device__ void function_Bilaterals(
		uint& index, uint& number_of_bilaterals, uint& number_of_contacts, real& lcp_omega_bilateral, real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB, int2* bids, real* gamma, real* eta,
		real* bi, real* mass, real3* inertia, real4* rot, real3* vel, real3* omega, real3* pos, real3* updateV, real3* updateO, uint* offset, real* dG) {
	real3 vA;
	real3 vB;
	real gamma_new = 0, gamma_old = 0;
	int B1_index = 0, B2_index = 0;
	B1_index = bids[index].x;
	B2_index = bids[index].y;
	real mass1 = mass[B1_index];
	real mass2 = mass[B2_index];
	// ---- perform   gamma_new = ([J1 J2] {v1 | v2}^ + b)
	{
		vA = JXYZA[index]; // line 0
		vB = vel[B1_index]; // v1
		gamma_new += dot3(vA, vB);
		vA = JUVWA[index]; // line 2
		vB = omega[B1_index]; // w1
		gamma_new += dot3(vA, vB);
	}
	{
		vA = JXYZB[index]; // line 1
		vB = vel[B2_index]; // v2
		gamma_new += dot3(vA, vB);
		vA = JUVWB[index]; // line 3
		vB = omega[B2_index]; // w2
		gamma_new += dot3(vA, vB);
	}
	//real4 temp = bilaterals[index + 4 * number_of_bilaterals]; // line 4   (eta, b, gamma, 0)
	//vA = R3(temp.x, temp.y, temp.z);
	gamma_new += bi[index]; // add known term     + b
	gamma_old = gamma[index]; // old gamma
	/// ---- perform gamma_new *= omega/g_i
	gamma_new *= lcp_omega_bilateral; // lcp_omega_const is in constant memory
	gamma_new *= eta[index]; // eta = 1/g_i;
	/// ---- perform gamma_new = gamma_old - gamma_new ; in place.
	gamma_new = gamma_old - gamma_new;

	/// ---- perform projection of 'a' (only if simple unilateral behavior C>0 is requested)
//	if (temp.w && gamma_new < 0.) {
//		gamma_new = 0.;
//	}

	// ----- store gamma_new
	gamma[index] = gamma_new;
	//bilaterals[index + 4 * number_of_bilaterals] = R4(0, vA.x, vA.y, vA.z);
	/// ---- compute delta in multipliers: gamma_new = gamma_new - gamma_old   = delta_gamma    , in place.
	gamma_new -= gamma_old;
	dG[number_of_contacts + index] = (gamma_new);
	/// ---- compute dv1 =  invInert.18 * J1^ * deltagamma
	vB = inertia[B1_index]; // iJ iJ iJ im
	vA = (JXYZA[index]) * mass1 * gamma_new; // line 0: J1(x)
	int offset1 = offset[2 * number_of_contacts + index];
	int offset2 = offset[2 * number_of_contacts + index + number_of_bilaterals];
	updateV[offset1] = vA; //  ---> store  v1 vel. in reduction buffer
	updateO[offset1] = JUVWA[index] * vB * gamma_new; // line 2:  J1(w)// ---> store  w1 vel. in reduction buffer
	vB = inertia[B2_index]; // iJ iJ iJ im
	vA = (JXYZB[index]) * mass2 * gamma_new; // line 1: J2(x)
	updateV[offset2] = vA; //  ---> store  v2 vel. in reduction buffer
	updateO[offset2] = JUVWB[index] * vB * gamma_new; // line 3:  J2(w)// ---> store  w2 vel. in reduction buffer
}


void ChSolverGPU::host_Bilaterals(
		real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB, int2* bids, real* gamma, real* eta, real* bi, real* mass, real3* inertia, real4* rot, real3* vel, real3* omega, real3* pos,
		real3* updateV, real3* updateO, uint* offset, real* dG) {
	for (uint index = 0; index < number_of_bilaterals; index++) {
		function_Bilaterals(
				index,
				number_of_bilaterals,
				number_of_rigid_rigid,
				lcp_omega_bilateral,
				JXYZA,
				JXYZB,
				JUVWA,
				JUVWB,
				bids,
				gamma,
				eta,
				bi,
				mass,
				inertia,
				rot,
				vel,
				omega,
				pos,
				updateV,
				updateO,
				offset,
				dG);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Updates the speeds in the body buffer with values accumulated in the
// reduction buffer:   V_new = V_old + delta_speeds

__host__ __device__ void function_Reduce_Speeds(uint& index, bool* active, real* mass, real3* vel, real3* omega, real3* updateV, real3* updateO, uint* d_body_num, uint* counter) {
	int start = (index == 0) ? 0 : counter[index - 1], end = counter[index];
	int id = d_body_num[end - 1], j;

	if (active[id] == 0) {
		return;
	}
	real3 mUpdateV = R3(0);
	real3 mUpdateO = R3(0);

	for (j = 0; j < end - start; j++) {
		mUpdateV = mUpdateV + updateV[j + start];
		mUpdateO = mUpdateO + updateO[j + start];
	}
	vel[id] += (mUpdateV);
	omega[id] += (mUpdateO);
}


void ChSolverGPU::host_Reduce_Speeds(bool* active, real* mass, real3* vel, real3* omega, real3* updateV, real3* updateO, uint* d_body_num, uint* counter) {
#pragma omp parallel for

	for (uint index = 0; index < number_of_updates; index++) {
		function_Reduce_Speeds(index, active, mass, vel, omega, updateV, updateO, d_body_num, counter);
	}
}
//__global__ void device_Offsets(int2* ids, real4* bilaterals, uint* Body) {
//	uint index = blockIdx.x * blockDim.x + threadIdx.x;
//
//	if (index < number_of_contacts_const) {
//		int2 temp_id = ids[index];
//		Body[index] = temp_id.x;
//		Body[index + number_of_contacts_const] = temp_id.y;
//	}
//
//	if (index < number_of_bilaterals_const) {
//		Body[2 * number_of_contacts_const + index] = bilaterals[index].w;
//		Body[2 * number_of_contacts_const + index + number_of_bilaterals_const] = bilaterals[index + number_of_bilaterals_const].w;
//	}
//}
//
//void ChSolverJacobi::host_Offsets(int2* ids, real4* bilaterals, uint* Body) {
//	for (uint index = 0; index < number_of_contacts+number_of_bilaterals; index++) {
//		if (index < number_of_contacts) {
//			int2 temp_id = ids[index];
//			Body[index] = temp_id.x;
//			Body[index + number_of_contacts] = temp_id.y;
//		}
//
//		if (index < number_of_bilaterals) {
//			Body[2 * number_of_contacts + index] = bilaterals[index].w;
//			Body[2 * number_of_contacts + index + number_of_bilaterals] = bilaterals[index + number_of_bilaterals].w;
//		}
//	}
//}

void ChSolverGPU::SolveJacobi() {

#ifdef SIM_ENABLE_GPU_MODE
	COPY_TO_CONST_MEM(number_of_rigid_rigid);
	COPY_TO_CONST_MEM(number_of_constraints);
	COPY_TO_CONST_MEM(number_of_bilaterals);
	COPY_TO_CONST_MEM(number_of_objects);
	COPY_TO_CONST_MEM(step_size);
	COPY_TO_CONST_MEM(compliance);
	COPY_TO_CONST_MEM(complianceT);
	COPY_TO_CONST_MEM(alpha);
	COPY_TO_CONST_MEM(lcp_omega_bilateral);
	COPY_TO_CONST_MEM(lcp_omega_contact);
	COPY_TO_CONST_MEM(contact_recovery_speed);

	cudaFuncSetCacheConfig(device_process_contacts, cudaFuncCachePreferL1);
	cudaFuncSetCacheConfig(device_Bilaterals, cudaFuncCachePreferL1);
	cudaFuncSetCacheConfig(device_Reduce_Speeds, cudaFuncCachePreferL1);
	cudaFuncSetCacheConfig(device_Offsets, cudaFuncCachePreferL1);
#else
#endif

#ifdef SIM_ENABLE_GPU_MODE
	COPY_TO_CONST_MEM(number_of_updates);
#endif
	if (number_of_rigid_rigid + number_of_bilaterals != 0) {
		for (current_iteration = 0; current_iteration < max_iteration; current_iteration++) {

#ifdef SIM_ENABLE_GPU_MODE
			device_process_contacts CUDA_KERNEL_DIM(BLOCKS(number_of_rigid_rigid), THREADS)(
					CASTR3(data_container->device_data.device_JXYZA_data),
					CASTR3(data_container->device_data.device_JXYZB_data),
					CASTR3(data_container->device_data.device_JUVWA_data),
					CASTR3(data_container->device_data.device_JUVWB_data),
					CASTR1(data_container->device_rhs_data),
					CASTR1(data_container->device_data.device_dpth_data),
					CASTB1(data_container->device_data.device_active_data),
					CASTI2(data_container->device_data.device_bids_data),
					CASTR1(data_container->device_data.device_gam_data),
					CASTR1(data_container->device_data.device_dgm_data),
					CASTR1(data_container->device_data.device_mass_data),
					CASTR1(data_container->device_data.device_fric_data),
					CASTR3(data_container->device_data.device_inr_data),
					CASTR4(data_container->device_data.device_rot_data),
					CASTR3(data_container->device_data.device_vel_data),
					CASTR3(data_container->device_data.device_omg_data),
					CASTR3(data_container->device_data.device_pos_data),
					CASTR3(data_container->device_data.vel_update),
					CASTR3(data_container->device_data.omg_update),
					CASTU1(data_container->device_data.update_offset));
#else
			host_process_contacts(
					data_container->device_data.device_JXYZA_data.data(),
					data_container->device_data.device_JXYZB_data.data(),
					data_container->device_data.device_JUVWA_data.data(),
					data_container->device_data.device_JUVWB_data.data(),
					data_container->device_data.device_rhs_data.data(),
					data_container->device_data.device_dpth_data.data(),
					data_container->device_data.device_active_data.data(),
					data_container->device_data.device_bids_data.data(),
					data_container->device_data.device_gam_data.data(),
					data_container->device_data.device_dgm_data.data(),
					data_container->device_data.device_mass_data.data(),
					data_container->device_data.device_fric_data.data(),
					data_container->device_data.device_inr_data.data(),
					data_container->device_data.device_rot_data.data(),
					data_container->device_data.device_vel_data.data(),
					data_container->device_data.device_omg_data.data(),
					data_container->device_data.device_pos_data.data(),
					data_container->device_data.vel_update.data(),
					data_container->device_data.omg_update.data(),
					data_container->device_data.update_offset.data());
#endif
#ifdef SIM_ENABLE_GPU_MODE
			device_Bilaterals CUDA_KERNEL_DIM(BLOCKS(number_of_bilaterals), THREADS)(
					CASTR4(data_container->device_data.device_bilateral_data),
					CASTR1(data_container->device_data.device_mass_data),
					CASTR3(data_container->device_data.device_inr_data),
					CASTR4(data_container->device_data.device_rot_data),
					CASTR3(data_container->device_data.device_vel_data),
					CASTR3(data_container->device_data.device_omg_data),
					CASTR3(data_container->device_data.device_pos_data),
					CASTR3(data_container->device_data.vel_update),
					CASTR3(data_container->device_data.omg_update),
					CASTU1(data_container->device_data.update_offset),
					CASTR1(data_container->device_data.device_dgm_data));
#else
			host_Bilaterals(
					data_container->device_data.device_JXYZA_bilateral.data(),
					data_container->device_data.device_JXYZB_bilateral.data(),
					data_container->device_data.device_JUVWA_bilateral.data(),
					data_container->device_data.device_JUVWB_bilateral.data(),
					data_container->device_data.device_bids_bilateral.data(),
					data_container->device_data.device_gamma_bilateral.data(),
					data_container->device_data.device_correction_bilateral.data(),
					data_container->device_data.device_residual_bilateral.data(),
					data_container->device_data.device_mass_data.data(),
					data_container->device_data.device_inr_data.data(),
					data_container->device_data.device_rot_data.data(),
					data_container->device_data.device_vel_data.data(),
					data_container->device_data.device_omg_data.data(),
					data_container->device_data.device_pos_data.data(),
					data_container->device_data.vel_update.data(),
					data_container->device_data.omg_update.data(),
					data_container->device_data.update_offset.data(),
					data_container->device_data.device_dgm_data.data());
#endif

#ifdef SIM_ENABLE_GPU_MODE
			device_Reduce_Speeds CUDA_KERNEL_DIM(BLOCKS(number_of_updates), THREADS)(
					CASTB1(data_container->device_data.device_active_data),
					CASTR1(data_container->device_data.device_mass_data),
					CASTR3(data_container->device_data.device_vel_data),
					CASTR3(data_container->device_data.device_omg_data),
					CASTR3(data_container->device_data.vel_update),
					CASTR3(data_container->device_data.omg_update),
					CASTU1(data_container->device_data.body_number),
					CASTU1(data_container->device_data.offset_counter));
#else
			host_Reduce_Speeds(
					data_container->device_data.device_active_data.data(),
					data_container->device_data.device_mass_data.data(),
					data_container->device_data.device_vel_data.data(),
					data_container->device_data.device_omg_data.data(),
					data_container->device_data.vel_update.data(),
					data_container->device_data.omg_update.data(),
					data_container->device_data.body_number.data(),
					data_container->device_data.offset_counter.data());
#endif
			//residual = CompRes(data_container->gpu_data.device_dgm_data, number_of_contacts);
			residual = data_container->device_data.device_dgm_data[thrust::max_element(
					data_container->device_data.device_dgm_data.begin(),
					data_container->device_data.device_dgm_data.begin() + number_of_rigid_rigid) - data_container->device_data.device_dgm_data.begin()];

			AtIterationEnd(residual, 0, current_iteration);

			if (residual < tolerance) {
				break;
			}
		}
	}
}
