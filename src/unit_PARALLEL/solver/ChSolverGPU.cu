#include "ChSolverGPU.h"
using namespace chrono;

__constant__ uint number_of_constraints_const;
__constant__ uint number_of_contacts_const;
__constant__ real step_size_const;
__constant__ real alpha_const;
__constant__ real compliance_const;
__constant__ real inv_hpa_const;
__constant__ real inv_hhpa_const;
__constant__ real contact_recovery_speed_const;

__device__ double atomicAdd(double* address, double val) {
	unsigned long long int* address_as_ull = (unsigned long long int*) address;
	unsigned long long int old = *address_as_ull, assumed;
	do {
		assumed = old;
		old = atomicCAS(address_as_ull, assumed, __double_as_longlong(val + __longlong_as_double(assumed)));
	} while (assumed != old);
	return __longlong_as_double(old);
}


__host__ __device__ void function_Project(uint &index, uint number_of_contacts, int2 *ids, real *fric, real *gam) {
	int2 body_id = ids[index];
	real3 gamma;
	gamma.x = gam[index + number_of_contacts * 0];
	gamma.y = gam[index + number_of_contacts * 1];
	gamma.z = gam[index + number_of_contacts * 2];
	real f_tang = sqrtf(gamma.y * gamma.y + gamma.z * gamma.z);
	real mu = (fric[body_id.x] + fric[body_id.y]) * .5f;
	if (mu == 0) {
		gamma.y = gamma.z = 0;
	} else if (f_tang > (mu * gamma.x)) { // inside upper cone? keep untouched!
		if ((f_tang) < -(1.0 / mu) * gamma.x || (fabsf(gamma.x) < 0)) { // inside lower cone? reset  normal,u,v to zero!
			gamma = R3(0);
		} else { // remaining case: project orthogonally to generator segment of upper cone
			gamma.x = (f_tang * mu + gamma.x) / (mu * mu + 1);
			real tproj_div_t = (gamma.x * mu) / f_tang; //  reg = tproj_div_t
			gamma.y *= tproj_div_t;
			gamma.z *= tproj_div_t;
		}
	}
	//  if (gamma.x < 0) {
	//      gamma.x = 0;
	//  }
	//  gamma.y = gamma.z = 0;
	gam[index + number_of_contacts * 0] = gamma.x;
	gam[index + number_of_contacts * 1] = gamma.y;
	gam[index + number_of_contacts * 2] = gamma.z;
}

__global__ void device_Project(int2 *ids, real *friction, real *gamma) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_contacts_const);
	function_Project(index, number_of_contacts_const, ids, friction, gamma);
}

void ChSolverGPU::host_Project(int2 *ids, real *friction, real *gamma) {
	for (uint index = 0; index < number_of_contacts; index++) {
		function_Project(index, number_of_contacts, ids, friction, gamma);
	}
}
void ChSolverGPU::Project(custom_vector<real> & gamma) {
	timer_project.start();

#ifdef SIM_ENABLE_GPU_MODE
		device_Project CUDA_KERNEL_DIM(BLOCKS(number_of_contacts), THREADS)(
				CASTI2(gpu_data->device_bids_data),
				CASTR1(gpu_data->device_fric_data),
				CASTR1(gamma));
#else
		host_Project (
				gpu_data->device_bids_data.data(),
				gpu_data->device_fric_data.data(),
				gamma.data());
#endif

		timer_project.stop();
		time_project += timer_project();
	}

__host__ __device__ void function_shurA(uint &index, int2 *ids, bool *active, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *gamma, real3 *QXYZ, real3 *QUVW) {
	real q_x = 0, q_y = 0, q_z = 0, q_u = 0, q_v = 0, q_w = 0;
	uint b1 = ids[index].x;
	uint b2 = ids[index].y;
	if (active[b1] != 0) {
		q_x = JXYZA[index].x * gamma[index];
		q_y = JXYZA[index].y * gamma[index];
		q_z = JXYZA[index].z * gamma[index];
		q_u = JUVWA[index].x * gamma[index];
		q_v = JUVWA[index].y * gamma[index];
		q_w = JUVWA[index].z * gamma[index];
		QXYZ[b1] += R3(q_x, q_y, q_z);
		QUVW[b1] += R3(q_u, q_v, q_w);
	}
	if (active[b2] != 0) {
		q_x = JXYZB[index].x * gamma[index];
		q_y = JXYZB[index].y * gamma[index];
		q_z = JXYZB[index].z * gamma[index];
		q_u = JUVWB[index].x * gamma[index];
		q_v = JUVWB[index].y * gamma[index];
		q_w = JUVWB[index].z * gamma[index];
		QXYZ[b2] += R3(q_x, q_y, q_z);
		QUVW[b2] += R3(q_u, q_v, q_w);
	}
}
__global__ void device_shurA(int2 *ids, bool *active, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *gamma, real3 *QXYZ, real3 *QUVW) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_constraints_const);
	real q_x = 0, q_y = 0, q_z = 0, q_u = 0, q_v = 0, q_w = 0;
	real temp = 0, m;
	uint b1 = ids[index].x;
	uint b2 = ids[index].y;
	if (active[b1] != 0) {
		q_x = JXYZA[index].x * gamma[index];
		atomicAdd(&QXYZ[b1].x, q_x);
		q_y = JXYZA[index].y * gamma[index];
		atomicAdd(&QXYZ[b1].y, q_y);
		q_z = JXYZA[index].z * gamma[index];
		atomicAdd(&QXYZ[b1].z, q_z);

		q_u = JUVWA[index].x * gamma[index];
		atomicAdd(&QUVW[b1].x, q_u);
		q_v = JUVWA[index].y * gamma[index];
		atomicAdd(&QUVW[b1].y, q_v);
		q_w = JUVWA[index].z * gamma[index];
		atomicAdd(&QUVW[b1].z, q_w);

		//QXYZ[b1] += R3(q_x, q_y, q_z);
		//QUVW[b1] += R3(q_u, q_v, q_w);
	}
	if (active[b2] != 0) {
		q_x = JXYZB[index].x * gamma[index];
		atomicAdd(&QXYZ[b2].x, q_x);
		q_y = JXYZB[index].y * gamma[index];
		atomicAdd(&QXYZ[b2].y, q_y);
		q_z = JXYZB[index].z * gamma[index];
		atomicAdd(&QXYZ[b2].z, q_z);

		q_u = JUVWB[index].x * gamma[index];
		atomicAdd(&QUVW[b2].x, q_u);
		q_v = JUVWB[index].y * gamma[index];
		atomicAdd(&QUVW[b2].y, q_v);
		q_w = JUVWB[index].z * gamma[index];
		atomicAdd(&QUVW[b2].z, q_w);

		//QXYZ[b2] += R3(q_x, q_y, q_z);
		//QUVW[b2] += R3(q_u, q_v, q_w);
	}
}

void ChSolverGPU::host_shurA(int2 *ids, bool *active, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *gamma, real3 *QXYZ, real3 *QUVW) {
	for (uint index = 0; index < number_of_constraints; index++) {
		function_shurA(index, ids, active, JXYZA, JXYZB, JUVWA, JUVWB, gamma, QXYZ, QUVW);
	}
}
void ChSolverGPU::shurA(custom_vector<real> &x) {
#ifdef SIM_ENABLE_GPU_MODE
		device_shurA CUDA_KERNEL_DIM(BLOCKS(number_of_constraints), THREADS)(
				CASTI2(temp_bids),
				CASTB1(gpu_data->device_active_data),
				CASTR3(gpu_data->device_JXYZA_data),
				CASTR3(gpu_data->device_JXYZB_data),
				CASTR3(gpu_data->device_JUVWA_data),
				CASTR3(gpu_data->device_JUVWB_data),
				CASTR1(x),
				CASTR3(gpu_data->device_QXYZ_data),
				CASTR3(gpu_data->device_QUVW_data));
		cudaDeviceSynchronize();
#else
		host_shurA(
				temp_bids.data(),
				gpu_data->device_active_data.data(),
				gpu_data->device_JXYZA_data.data(),
				gpu_data->device_JXYZB_data.data(),
				gpu_data->device_JUVWA_data.data(),
				gpu_data->device_JUVWB_data.data(),
				x.data(),
				gpu_data->device_QXYZ_data.data(),
				gpu_data->device_QUVW_data.data());
#endif
	}

__host__ __device__ void function_shurB(
		uint &index,
		int2 *ids,
		bool *active,
		real *inv_mass,
		real3 *inv_inertia,
		real * gamma,
		real3 *JXYZA,
		real3 *JXYZB,
		real3 *JUVWA,
		real3 *JUVWB,
		real3 *QXYZ,
		real3 *QUVW,
		const real &alpha,
		const real &compliance,
		const real & inv_hhpa,
		real *AX) {
	real temp = 0, m;
	uint b1 = ids[index].x;
	uint b2 = ids[index].y;
	real m1 = inv_mass[b1];
	real m2 = inv_mass[b2];
	real3 inertia1 = inv_inertia[b1];
	real3 inertia2 = inv_inertia[b2];
	if (active[b1] != 0) {
		temp += QXYZ[b1].x * JXYZA[index].x * m1;
		temp += QXYZ[b1].y * JXYZA[index].y * m1;
		temp += QXYZ[b1].z * JXYZA[index].z * m1;
		temp += QUVW[b1].x * JUVWA[index].x * inertia1.x;
		temp += QUVW[b1].y * JUVWA[index].y * inertia1.y;
		temp += QUVW[b1].z * JUVWA[index].z * inertia1.z;
	}
	if (active[b2] != 0) {
		temp += QXYZ[b2].x * JXYZB[index].x * m2;
		temp += QXYZ[b2].y * JXYZB[index].y * m2;
		temp += QXYZ[b2].z * JXYZB[index].z * m2;
		temp += QUVW[b2].x * JUVWB[index].x * inertia2.x;
		temp += QUVW[b2].y * JUVWB[index].y * inertia2.y;
		temp += QUVW[b2].z * JUVWB[index].z * inertia2.z;
	}
	AX[index] = temp + gamma[index] * inv_hhpa * compliance;
}

__global__ void device_shurB(int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real * gamma, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real3 *QXYZ, real3 *QUVW, real *AX) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_constraints_const);
	function_shurB(index, ids, active, inv_mass, inv_inertia, gamma, JXYZA, JXYZB, JUVWA, JUVWB, QXYZ, QUVW, alpha_const, compliance_const, inv_hhpa_const, AX);
}

void ChSolverGPU::host_shurB(int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real * gamma, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real3 *QXYZ, real3 *QUVW, real *AX) {
#pragma omp parallel for schedule(guided)
	for (uint index = 0; index < number_of_constraints; index++) {
		function_shurB(index, ids, active, inv_mass, inv_inertia, gamma, JXYZA, JXYZB, JUVWA, JUVWB, QXYZ, QUVW, alpha, compliance, inv_hhpa, AX);
	}
}

void ChSolverGPU::shurB(custom_vector<real> &x) {
#ifdef SIM_ENABLE_GPU_MODE
		device_shurB CUDA_KERNEL_DIM(BLOCKS(number_of_constraints), THREADS)(
				CASTI2(temp_bids),
				CASTB1(gpu_data->device_active_data),
				CASTR1(gpu_data->device_mass_data),
				CASTR3(gpu_data->device_inr_data),
				CASTR1(x),
				CASTR3(gpu_data->device_JXYZA_data),
				CASTR3(gpu_data->device_JXYZB_data),
				CASTR3(gpu_data->device_JUVWA_data),
				CASTR3(gpu_data->device_JUVWB_data),
				CASTR3(gpu_data->device_QXYZ_data),
				CASTR3(gpu_data->device_QUVW_data),
				CASTR1(AX));
		cudaDeviceSynchronize();
#else
		host_shurB(
				temp_bids.data(),
				gpu_data->device_active_data.data(),
				gpu_data->device_mass_data.data(),
				gpu_data->device_inr_data.data(),
				x.data(),
				gpu_data->device_JXYZA_data.data(),
				gpu_data->device_JXYZB_data.data(),
				gpu_data->device_JUVWA_data.data(),
				gpu_data->device_JUVWB_data.data(),
				gpu_data->device_QXYZ_data.data(),
				gpu_data->device_QUVW_data.data(),
				AX.data());
#endif
	}

__host__ __device__ void function_RHS(
		uint &index,
		real &step_size,
		int2 *ids,
		real *correction,
		real & recovery_speed,
		real3 *vel,
		real3 *omega,
		real3 *JXYZA,
		real3 *JXYZB,
		real3 *JUVWA,
		real3 *JUVWB,
		const real & inv_hpa,
		real *rhs) {
	uint b1 = ids[index].x;
	uint b2 = ids[index].y;
	real temp = 0;

	temp += dot(JXYZA[index], vel[b1]);
	temp += dot(JUVWA[index], omega[b1]);
	temp += dot(JXYZB[index], vel[b2]);
	temp += dot(JUVWB[index], omega[b2]);

//	temp -= JXYZA[index].x * vel[b1].x;
//	temp -= JXYZA[index].y * vel[b1].y;
//	temp -= JXYZA[index].z * vel[b1].z;
//	temp -= JUVWA[index].x * omega[b1].x;
//	temp -= JUVWA[index].y * omega[b1].y;
//	temp -= JUVWA[index].z * omega[b1].z;
//	temp -= JXYZB[index].x * vel[b2].x;
//	temp -= JXYZB[index].y * vel[b2].y;
//	temp -= JXYZB[index].z * vel[b2].z;
//	temp -= JUVWB[index].x * omega[b2].x;
//	temp -= JUVWB[index].y * omega[b2].y;
//	temp -= JUVWB[index].z * omega[b2].z;
	//rhs[index] = -((-temp) + fmaxf(inv_hpa * -fabs(correction[index]), 0));
	rhs[index] = -(temp + fmax(inv_hpa * correction[index], real(-recovery_speed)));

}

__global__ void device_RHS(int2 *ids, real *correction, real3 *vel, real3 *omega, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *rhs) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_constraints_const);
	function_RHS(index, step_size_const, ids, correction, contact_recovery_speed_const, vel, omega, JXYZA, JXYZB, JUVWA, JUVWB, inv_hpa_const, rhs);
}

void ChSolverGPU::host_RHS(int2 *ids, real *correction, real3 *vel, real3 *omega, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *rhs) {
#pragma omp parallel for schedule(guided)
	for (uint index = 0; index < number_of_constraints; index++) {
		function_RHS(index, step_size, ids, correction, contact_recovery_speed, vel, omega, JXYZA, JXYZB, JUVWA, JUVWB, inv_hpa, rhs);
	}
}

void ChSolverGPU::ComputeRHS() {
	timer_rhs.start();

	Thrust_Fill(gpu_data->device_QXYZ_data, R3(0));
	Thrust_Fill(gpu_data->device_QUVW_data, R3(0));
#ifdef SIM_ENABLE_GPU_MODE
	device_RHS CUDA_KERNEL_DIM(BLOCKS(number_of_constraints), THREADS)(
			CASTI2(temp_bids),
			CASTR1(correction),
			CASTR3(gpu_data->device_vel_data),
			CASTR3(gpu_data->device_omg_data),
			CASTR3(gpu_data->device_JXYZA_data),
			CASTR3(gpu_data->device_JXYZB_data),
			CASTR3(gpu_data->device_JUVWA_data),
			CASTR3(gpu_data->device_JUVWB_data),
			CASTR1(rhs));
	cudaDeviceSynchronize();
#else
	host_RHS(
			temp_bids.data(),
			correction.data(),
			gpu_data->device_vel_data.data(),
			gpu_data->device_omg_data.data(),
			gpu_data->device_JXYZA_data.data(),
			gpu_data->device_JXYZB_data.data(),
			gpu_data->device_JUVWA_data.data(),
			gpu_data->device_JUVWB_data.data(),
			rhs.data());
#endif

	timer_rhs.stop();
	time_rhs = timer_rhs();
}

void ChSolverGPU::Setup() {
	time_rhs = time_shurcompliment = time_project = time_integrate = 0;

	number_of_constraints = gpu_data->number_of_contacts * 3 + gpu_data->number_of_bilaterals;
	number_of_contacts = gpu_data->number_of_contacts;
	number_of_bilaterals = gpu_data->number_of_bilaterals;
	number_of_objects = gpu_data->number_of_objects;
	temp_bids.resize(number_of_constraints);
	correction.resize(number_of_constraints);
	rhs.resize(number_of_constraints);
	gpu_data->device_gam_data.resize((number_of_constraints));
	AX.resize(number_of_constraints);

	gpu_data->device_QXYZ_data.resize(number_of_objects);
	gpu_data->device_QUVW_data.resize(number_of_objects);

	///
	Thrust_Fill(gpu_data->device_gam_data, 0);
	Thrust_Fill(correction, 0);
	///
	thrust::copy_n(gpu_data->device_dpth_data.begin(), gpu_data->number_of_contacts, correction.begin() + gpu_data->number_of_contacts * 0);
	///
	thrust::copy_n(gpu_data->device_bids_data.begin(), gpu_data->number_of_contacts, temp_bids.begin() + gpu_data->number_of_contacts * 0);
	thrust::copy_n(gpu_data->device_bids_data.begin(), gpu_data->number_of_contacts, temp_bids.begin() + gpu_data->number_of_contacts * 1);
	thrust::copy_n(gpu_data->device_bids_data.begin(), gpu_data->number_of_contacts, temp_bids.begin() + gpu_data->number_of_contacts * 2);

	inv_hpa = 1.0 / (step_size + alpha);
	inv_hhpa = 1.0 / (step_size * (step_size + alpha));

#ifdef SIM_ENABLE_GPU_MODE
	COPY_TO_CONST_MEM(number_of_constraints);
	COPY_TO_CONST_MEM(number_of_contacts);
	COPY_TO_CONST_MEM(step_size);
	COPY_TO_CONST_MEM(alpha);
	COPY_TO_CONST_MEM(compliance);
	COPY_TO_CONST_MEM(inv_hpa);
	COPY_TO_CONST_MEM(inv_hhpa);
	COPY_TO_CONST_MEM(contact_recovery_speed);

	cudaFuncSetCacheConfig(device_Project, cudaFuncCachePreferL1);
	cudaFuncSetCacheConfig(device_shurA, cudaFuncCachePreferL1);
	cudaFuncSetCacheConfig(device_shurB, cudaFuncCachePreferL1);
	cudaFuncSetCacheConfig(device_RHS, cudaFuncCachePreferL1);

#else
#endif
}

custom_vector<real> ChSolverGPU::ShurProduct( custom_vector<real> &x) {
	timer_shurcompliment.start();

	Thrust_Fill(gpu_data->device_QXYZ_data, R3(0));
	Thrust_Fill(gpu_data->device_QUVW_data, R3(0));
	AX.resize(x.size());
	Thrust_Fill(AX, 0);
	shurA(x);
	shurB(x);
	timer_shurcompliment.stop();
	time_shurcompliment +=timer_shurcompliment();
	return AX;
}

void ChSolverGPU::ComputeImpulses() {
	Thrust_Fill(gpu_data->device_QXYZ_data, R3(0));
	Thrust_Fill(gpu_data->device_QUVW_data, R3(0));
	shurA(gpu_data->device_gam_data);
	gpu_data->device_QXYZ_data *= gpu_data->device_mass_data;
	gpu_data->device_QUVW_data *= gpu_data->device_inr_data;
}

void ChSolverGPU::Solve(GPUSOLVERTYPE solver_type, real step, gpu_container &gpu_data_) {
	timer_solver.start();
	gpu_data = &gpu_data_;
	step_size = step;
	Setup();
	if (number_of_constraints > 0) {
		ComputeRHS();

		if (solver_type == STEEPEST_DESCENT) {
			SolveSD(gpu_data->device_gam_data, rhs, max_iteration);
		} else if (solver_type == GRADIENT_DESCENT) {
			SolveGD(gpu_data->device_gam_data, rhs, max_iteration);
		} else if (solver_type == CONJUGATE_GRADIENT) {
			SolveCG(gpu_data->device_gam_data, rhs, max_iteration);
		} else if (solver_type == CONJUGATE_GRADIENT_SQUARED) {
			SolveCGS(gpu_data->device_gam_data, rhs, max_iteration);
		} else if (solver_type == BICONJUGATE_GRADIENT) {
			SolveBiCG(gpu_data->device_gam_data, rhs, max_iteration);
		} else if (solver_type == BICONJUGATE_GRADIENT_STAB) {
			SolveBiCGStab(gpu_data->device_gam_data, rhs, max_iteration);
		} else if (solver_type == MINIMUM_RESIDUAL) {
			SolveMinRes(gpu_data->device_gam_data, rhs, max_iteration);
		}
		//else if(solver_type==QUASAI_MINIMUM_RESIDUAL){SolveQMR(gpu_data->device_gam_data, rhs, max_iteration);}
		else if (solver_type == ACCELERATED_PROJECTED_GRADIENT_DESCENT) {
			SolveAPGD(gpu_data->device_gam_data, rhs, max_iteration);
		}

		ComputeImpulses();
		gpu_data->device_vel_data += gpu_data->device_QXYZ_data;
		gpu_data->device_omg_data += gpu_data->device_QUVW_data;

		timer_solver.stop();
		time_solver = timer_solver();
	}
}

//
//__host__ __device__ void function_InvNDiag(uint index, int b1, int b2, int2* ids, real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB, real * inv_mass, real3 * inv_inertia, real & inv_n_diag) {
//
//  real Jx1 = JXYZA[index].x;
//  real Jy1 = JXYZA[index].y;
//  real Jz1 = JXYZA[index].z;
//  real Ju1 = JUVWA[index].x;
//  real Jv1 = JUVWA[index].y;
//  real Jw1 = JUVWA[index].z;
//  //
//  real Jx2 = JXYZB[index].x;
//  real Jy2 = JXYZB[index].y;
//  real Jz2 = JXYZB[index].z;
//  real Ju2 = JUVWB[index].x;
//  real Jv2 = JUVWB[index].y;
//  real Jw2 = JUVWB[index].z;
//  //
//  real m1 = inv_mass[b1];
//  real m2 = inv_mass[b2];
//  //
//  real3 i1 = inv_inertia[b1];
//  real3 i2 = inv_inertia[b2];
//
//  real part1 = dot(JXYZA[index], JXYZA[index]) * m1;
//  real part2 = dot(JUVWA[index] * JUVWA[index], i1);
//  real part3 = dot(JXYZB[index], JXYZB[index]) * m2;
//  real part4 = dot(JUVWB[index] * JUVWB[index], i2);
//  inv_n_diag = (part1 + part2 + part3 + part4);
//}
