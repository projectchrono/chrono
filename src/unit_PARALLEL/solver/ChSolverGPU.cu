#include "ChSolverGPU.cuh"
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
	real f_tang = sqrt(gamma.y * gamma.y + gamma.z * gamma.z);
	real mu = (fric[body_id.x] == 0 || fric[body_id.y] == 0) ? 0 : (fric[body_id.x] + fric[body_id.y]) * .5;
	if (mu == 0) {
		gamma.x = gamma.x < 0 ? 0 : gamma.x;
		gamma.y = gamma.z = 0;

		gam[index + number_of_contacts * 0] = gamma.x;
		gam[index + number_of_contacts * 1] = gamma.y;
		gam[index + number_of_contacts * 2] = gamma.z;

		return;

	}
	// inside upper cone? keep untouched!
	if (f_tang < (mu * gamma.x)) {
		return;
	}

	// inside lower cone? reset  normal,u,v to zero!

	if ((f_tang) < -(1.0 / mu) * gamma.x || (fabs(gamma.x) < 10e-15)) {
		gamma = R3(0);
		gam[index + number_of_contacts * 0] = gamma.x;
		gam[index + number_of_contacts * 1] = gamma.y;
		gam[index + number_of_contacts * 2] = gamma.z;

		return;
	}

	// remaining case: project orthogonally to generator segment of upper cone

	gamma.x = (f_tang * mu + gamma.x) / (mu * mu + 1);
	real tproj_div_t = (gamma.x * mu) / f_tang;
	gamma.y *= tproj_div_t;
	gamma.z *= tproj_div_t;

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

__host__ __device__ void function_shurA(uint &index, int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *gamma, real3 *QXYZ,
		real3 *QUVW) {
	real q_x = 0, q_y = 0, q_z = 0, q_u = 0, q_v = 0, q_w = 0;
	uint b1 = ids[index].x;
	uint b2 = ids[index].y;

	// if (active[b1] != 0) {
	real m1 = inv_mass[b1];
	real3 inertia1 = inv_inertia[b1];
	q_x = JXYZA[index].x * gamma[index] * m1;
	q_y = JXYZA[index].y * gamma[index] * m1;
	q_z = JXYZA[index].z * gamma[index] * m1;
	q_u = JUVWA[index].x * gamma[index] * inertia1.x;
	q_v = JUVWA[index].y * gamma[index] * inertia1.y;
	q_w = JUVWA[index].z * gamma[index] * inertia1.z;

	//cout << q_x << " " << q_y << " " << q_z << " " << q_u << " " << q_v << " " << q_w << endl;
	QXYZ[b1] += R3(q_x, q_y, q_z) * active[b1];
	QUVW[b1] += R3(q_u, q_v, q_w) * active[b1];
	//}
	//if (active[b2] != 0) {
	real m2 = inv_mass[b2];
	real3 inertia2 = inv_inertia[b2];
	q_x = JXYZB[index].x * gamma[index] * m2;
	q_y = JXYZB[index].y * gamma[index] * m2;
	q_z = JXYZB[index].z * gamma[index] * m2;
	q_u = JUVWB[index].x * gamma[index] * inertia2.x;
	q_v = JUVWB[index].y * gamma[index] * inertia2.y;
	q_w = JUVWB[index].z * gamma[index] * inertia2.z;

	//cout << q_x << " " << q_y << " " << q_z << " " << q_u << " " << q_v << " " << q_w << endl;

	QXYZ[b2] += R3(q_x, q_y, q_z) * active[b2];
	QUVW[b2] += R3(q_u, q_v, q_w) * active[b2];
	//}
}
__global__ void device_shurA(int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *gamma, real3 *QXYZ, real3 *QUVW) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_constraints_const);
	real q_x = 0, q_y = 0, q_z = 0, q_u = 0, q_v = 0, q_w = 0;
	uint b1 = ids[index].x;
	uint b2 = ids[index].y;
	real m1 = inv_mass[b1];
	real m2 = inv_mass[b2];
	real3 inertia1 = inv_inertia[b1];
	real3 inertia2 = inv_inertia[b2];
	if (active[b1] != 0) {
		q_x = JXYZA[index].x * gamma[index] * m1;
		atomicAdd(&QXYZ[b1].x, q_x);
		q_y = JXYZA[index].y * gamma[index] * m1;
		atomicAdd(&QXYZ[b1].y, q_y);
		q_z = JXYZA[index].z * gamma[index] * m1;
		atomicAdd(&QXYZ[b1].z, q_z);

		q_u = JUVWA[index].x * gamma[index] * inertia1.x;
		atomicAdd(&QUVW[b1].x, q_u);
		q_v = JUVWA[index].y * gamma[index] * inertia1.y;
		atomicAdd(&QUVW[b1].y, q_v);
		q_w = JUVWA[index].z * gamma[index] * inertia1.z;
		atomicAdd(&QUVW[b1].z, q_w);

		//QXYZ[b1] += R3(q_x, q_y, q_z);
		//QUVW[b1] += R3(q_u, q_v, q_w);
	}
	if (active[b2] != 0) {
		q_x = JXYZB[index].x * gamma[index] * m2;
		atomicAdd(&QXYZ[b2].x, q_x);
		q_y = JXYZB[index].y * gamma[index] * m2;
		atomicAdd(&QXYZ[b2].y, q_y);
		q_z = JXYZB[index].z * gamma[index] * m2;
		atomicAdd(&QXYZ[b2].z, q_z);

		q_u = JUVWB[index].x * gamma[index] * inertia2.x;
		atomicAdd(&QUVW[b2].x, q_u);
		q_v = JUVWB[index].y * gamma[index] * inertia2.y;
		atomicAdd(&QUVW[b2].y, q_v);
		q_w = JUVWB[index].z * gamma[index] * inertia2.z;
		atomicAdd(&QUVW[b2].z, q_w);

		//QXYZ[b2] += R3(q_x, q_y, q_z);
		//QUVW[b2] += R3(q_u, q_v, q_w);
	}
}

void ChSolverGPU::host_shurA(int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *gamma, real3 *QXYZ, real3 *QUVW) {
//for (uint index = 0; index < number_of_constraints; index++) {
	// function_shurA(index, ids, active, inv_mass, inv_inertia, JXYZA, JXYZB, JUVWA, JUVWB, gamma, QXYZ, QUVW);

	/*for (uint index = 0; index < number_of_constraints; index++) {
	 real gam = gamma[index];
	 uint b1 = ids[index].x;
	 if (active[b1] != 0) {
	 real m1 = inv_mass[b1];
	 QXYZ[b1].x += JXYZA[index].x * gam * m1;
	 QXYZ[b1].y += JXYZA[index].y * gam * m1;
	 QXYZ[b1].z += JXYZA[index].z * gam * m1;

	 real3 inertia1 = inv_inertia[b1];
	 QUVW[b1].x += JUVWA[index].x * gam * inertia1.x;
	 QUVW[b1].y += JUVWA[index].y * gam * inertia1.y;
	 QUVW[b1].z += JUVWA[index].z * gam * inertia1.z;
	 }
	 uint b2 = ids[index].y;
	 if (active[b2] != 0) {
	 real m2 = inv_mass[b2];
	 QXYZ[b2].x += JXYZB[index].x * gam * m2;
	 QXYZ[b2].y += JXYZB[index].y * gam * m2;
	 QXYZ[b2].z += JXYZB[index].z * gam * m2;

	 real3 inertia2 = inv_inertia[b2];
	 QUVW[b2].x += JUVWB[index].x * gam * inertia2.x;
	 QUVW[b2].y += JUVWB[index].y * gam * inertia2.y;
	 QUVW[b2].z += JUVWB[index].z * gam * inertia2.z;
	 }
	 }*/

	for (int index = 0; index < number_of_constraints; index++) {
		real gam = gamma[index];
		uint b1 = ids[index].x;
		real m1 = inv_mass[b1];
		real3 inertia1 = inv_inertia[b1];
		QXYZ[b1] += JXYZA[index] * gam * m1 * active[b1];
		QUVW[b1] += JUVWA[index] * gam * inertia1 * active[b1];
		uint b2 = ids[index].y;
		real m2 = inv_mass[b2];
		real3 inertia2 = inv_inertia[b2];
		QXYZ[b2] += JXYZB[index] * gam * m2 * active[b2];
		QUVW[b2] += JUVWB[index] * gam * inertia2 * active[b2];
	}
}
void ChSolverGPU::shurA(custom_vector<real> &x) {
#ifdef SIM_ENABLE_GPU_MODE
		device_shurA CUDA_KERNEL_DIM(BLOCKS(number_of_constraints), THREADS)(
				CASTI2(temp_bids),
				CASTB1(gpu_data->device_active_data),
				CASTR1(gpu_data->device_mass_data),
				CASTR3(gpu_data->device_inr_data),
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
				gpu_data->device_mass_data.data(),
				gpu_data->device_inr_data.data(),
				gpu_data->device_JXYZA_data.data(),
				gpu_data->device_JXYZB_data.data(),
				gpu_data->device_JUVWA_data.data(),
				gpu_data->device_JUVWB_data.data(),
				x.data(),
				gpu_data->device_QXYZ_data.data(),
				gpu_data->device_QUVW_data.data());
#endif
	}

__host__ __device__
void function_shurB(uint &index, int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real * gamma, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real3 *QXYZ, real3 *QUVW,
		const real &alpha, const real &compliance, const real & inv_hhpa, real *AX) {
	real temp = 0;
	uint b1 = ids[index].x;
	uint b2 = ids[index].y;
	if (active[b1] != 0) {
		temp += QXYZ[b1].x * JXYZA[index].x;
		temp += QXYZ[b1].y * JXYZA[index].y;
		temp += QXYZ[b1].z * JXYZA[index].z;
		temp += QUVW[b1].x * JUVWA[index].x;
		temp += QUVW[b1].y * JUVWA[index].y;
		temp += QUVW[b1].z * JUVWA[index].z;
	}
	if (active[b2] != 0) {
		temp += QXYZ[b2].x * JXYZB[index].x;
		temp += QXYZ[b2].y * JXYZB[index].y;
		temp += QXYZ[b2].z * JXYZB[index].z;
		temp += QUVW[b2].x * JUVWB[index].x;
		temp += QUVW[b2].y * JUVWB[index].y;
		temp += QUVW[b2].z * JUVWB[index].z;
	}
	AX[index] = temp; //+ gamma[index] * inv_hhpa * compliance;
}

__global__ void device_shurB(int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real * gamma, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real3 *QXYZ, real3 *QUVW, real *AX) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_constraints_const);
	function_shurB(index, ids, active, inv_mass, inv_inertia, gamma, JXYZA, JXYZB, JUVWA, JUVWB, QXYZ, QUVW, alpha_const, compliance_const, inv_hhpa_const, AX);
}

void ChSolverGPU::host_shurB(int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real * gamma, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real3 *QXYZ, real3 *QUVW, real *AX) {
	//function_shurB(index, ids, active, inv_mass, inv_inertia, gamma, JXYZA, JXYZB, JUVWA, JUVWB, QXYZ, QUVW, alpha, compliance, inv_hhpa, AX);
#pragma omp parallel
	{
#pragma omp for
		for (int index = 0; index < number_of_constraints; index++) {
			real temp = 0;
			uint b1 = ids[index].x;
			temp += QXYZ[b1].x * JXYZA[index].x;
			temp += QXYZ[b1].y * JXYZA[index].y;
			temp += QXYZ[b1].z * JXYZA[index].z;
			temp += QUVW[b1].x * JUVWA[index].x;
			temp += QUVW[b1].y * JUVWA[index].y;
			temp += QUVW[b1].z * JUVWA[index].z;
			AX[index] = temp * active[b1]; //+ gamma[index] * inv_hhpa * compliance;

		}

#pragma omp for
		for (int index = 0; index < number_of_constraints; index++) {
			real temp = 0;
			uint b2 = ids[index].y;
			temp += QXYZ[b2].x * JXYZB[index].x;
			temp += QXYZ[b2].y * JXYZB[index].y;
			temp += QXYZ[b2].z * JXYZB[index].z;
			temp += QUVW[b2].x * JUVWB[index].x;
			temp += QUVW[b2].y * JUVWB[index].y;
			temp += QUVW[b2].z * JUVWB[index].z;
			AX[index] += temp * active[b2]; //+ gamma[index] * inv_hhpa * compliance;
		}
	}
}

void ChSolverGPU::shurB(custom_vector<real> &x, custom_vector<real> &out) {
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
				out.data());
#endif
	}

__host__ __device__
void function_bi(uint &index, uint & num_contacts, real &step_size, real *correction, real & recovery_speed, const real & alpha, real *bi) {
	//std::cout<<real(1.0)/step_size* correction[index]<<std::endl;
	bi[index + num_contacts * 0] = fmax(real(1.0) / step_size * correction[index], -recovery_speed);
	bi[index + num_contacts * 1] = 0;
	bi[index + num_contacts * 2] = 0;
}
__global__ void device_bi(real *correction, real* bi) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_contacts_const);
	function_bi(index, number_of_contacts_const, step_size_const, correction, contact_recovery_speed_const, alpha_const, bi);
}

void ChSolverGPU::host_bi(real *correction, real* bi) {
#pragma omp parallel for
	for (uint index = 0; index < number_of_contacts; index++) {
		function_bi(index, number_of_contacts, step_size, correction, contact_recovery_speed, alpha, bi);
	}
}

__host__ __device__ void function_RHS(uint &index, real &step_size, int2 *ids, real *bi, real & recovery_speed, bool* active, real3 *vel, real3 *omega, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA,
		real3 *JUVWB, const real & alpha, real *rhs) {
	uint b1 = ids[index].x;
	uint b2 = ids[index].y;
	real temp = 0;
	if (active[b1]) {
		temp += dot(JXYZA[index], vel[b1]);
		temp += dot(JUVWA[index], omega[b1]);
	}
	if (active[b2]) {
		temp += dot(JXYZB[index], vel[b2]);
		temp += dot(JUVWB[index], omega[b2]);
	}

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
	rhs[index] = -temp - bi[index]; //(temp + fmax(inv_hpa * correction[index], real(-recovery_speed)));

}

__global__ void device_RHS(int2 *ids, real *bi, bool * active, real3 *vel, real3 *omega, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *rhs) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_constraints_const);
	function_RHS(index, step_size_const, ids, bi, contact_recovery_speed_const, active, vel, omega, JXYZA, JXYZB, JUVWA, JUVWB, alpha_const, rhs);
}

void ChSolverGPU::host_RHS(int2 *ids, real *bi, bool * active, real3 *vel, real3 *omega, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *rhs) {
#pragma omp parallel for schedule(guided)
	for (uint index = 0; index < number_of_constraints; index++) {
		function_RHS(index, step_size, ids, bi, contact_recovery_speed, active, vel, omega, JXYZA, JXYZB, JUVWA, JUVWB, alpha, rhs);
	}
}

void ChSolverGPU::ComputeRHS() {
	timer_rhs.start();
	memset(&gpu_data->device_QXYZ_data[0], 0, sizeof(gpu_data->device_QXYZ_data[0]) * gpu_data->device_QXYZ_data.size());
	memset(&gpu_data->device_QUVW_data[0], 0, sizeof(gpu_data->device_QUVW_data[0]) * gpu_data->device_QUVW_data.size());

	//Thrust_Fill(gpu_data->device_QXYZ_data, R3(0));
	//Thrust_Fill(gpu_data->device_QUVW_data, R3(0));

#ifdef SIM_ENABLE_GPU_MODE
	device_bi CUDA_KERNEL_DIM(BLOCKS(number_of_constraints), THREADS)(
			CASTR1(correction),
			CASTR1(bi));
	cudaDeviceSynchronize();
#else
	host_bi(correction.data(), bi.data());
#endif

#ifdef SIM_ENABLE_GPU_MODE
	device_RHS CUDA_KERNEL_DIM(BLOCKS(number_of_constraints), THREADS)(
			CASTI2(temp_bids),
			CASTR1(bi),
			CASTB1(gpu_data->device_active_data),
			CASTR3(gpu_data->device_vel_data),
			CASTR3(gpu_data->device_omg_data),
			CASTR3(gpu_data->device_JXYZA_data),
			CASTR3(gpu_data->device_JXYZB_data),
			CASTR3(gpu_data->device_JUVWA_data),
			CASTR3(gpu_data->device_JUVWB_data),
			CASTR1(rhs));
	cudaDeviceSynchronize();
#else
	host_RHS(temp_bids.data(), bi.data(), gpu_data->device_active_data.data(), gpu_data->device_vel_data.data(), gpu_data->device_omg_data.data(), gpu_data->device_JXYZA_data.data(),
			gpu_data->device_JXYZB_data.data(), gpu_data->device_JUVWA_data.data(), gpu_data->device_JUVWB_data.data(), rhs.data());
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
	bi.resize(number_of_constraints);
	gpu_data->device_gam_data.resize((number_of_constraints));

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

void ChSolverGPU::ShurProduct(custom_vector<real> &x, custom_vector<real> & output) {
	timer_shurcompliment.start();

	memset(&gpu_data->device_QXYZ_data[0], 0, sizeof(gpu_data->device_QXYZ_data[0]) * gpu_data->device_QXYZ_data.size());
	memset(&gpu_data->device_QUVW_data[0], 0, sizeof(gpu_data->device_QUVW_data[0]) * gpu_data->device_QUVW_data.size());
	memset(&output[0], 0, sizeof(output[0]) * output.size());
	//Thrust_Fill(gpu_data->device_QXYZ_data, R3(0));
	//Thrust_Fill(gpu_data->device_QUVW_data, R3(0));
	//Thrust_Fill(AX, 0);

		shurA(x);
		shurB(x,output);
		timer_shurcompliment.stop();
		time_shurcompliment +=timer_shurcompliment();
	}

void ChSolverGPU::ComputeImpulses() {
	Thrust_Fill(gpu_data->device_QXYZ_data, R3(0));
	Thrust_Fill(gpu_data->device_QUVW_data, R3(0));
	shurA(gpu_data->device_gam_data);

	//for(int i=0;  i<gpu_data->device_QXYZ_data.size(); i++){
	//cout<<gpu_data->device_QXYZ_data[i].x<<" "<<gpu_data->device_QXYZ_data[i].y<<" "<<gpu_data->device_QXYZ_data[i].z<<endl;
	//cout<<gpu_data->device_QUVW_data[i].x<<" "<<gpu_data->device_QUVW_data[i].y<<" "<<gpu_data->device_QUVW_data[i].z<<endl;
	//}

	//gpu_data->device_QXYZ_data *= gpu_data->device_mass_data;
	//gpu_data->device_QUVW_data *= gpu_data->device_inr_data;
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
