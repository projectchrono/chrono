#include "ChSolverGPU.cuh"
using namespace chrono;

__constant__ uint number_of_constraints_const;
__constant__ uint number_of_contacts_const;
__constant__ uint number_of_bilaterals_const;
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

__host__ __device__ void function_Reduce_Shur(uint& index, bool* active, real3* QXYZ, real3* QUVW, real *inv_mass, real3 *inv_inertia, real3* updateQXYZ, real3* updateQUVW, uint* d_body_num,
		uint* counter) {
	int start = (index == 0) ? 0 : counter[index - 1], end = counter[index];
	int id = d_body_num[end - 1], j;

	if (active[id] == 0) {
		return;
	}
	real3 mUpdateV = R3(0);
	real3 mUpdateO = R3(0);

	for (j = 0; j < end - start; j++) {
		mUpdateV += updateQXYZ[j + start];
		mUpdateO += updateQUVW[j + start];
	}
	QXYZ[id] = mUpdateV * inv_mass[id];
	QUVW[id] = mUpdateO * inv_inertia[id];
}

void ChSolverGPU::host_Reduce_Shur(bool* active, real3* QXYZ, real3* QUVW, real *inv_mass, real3 *inv_inertia, real3* updateQXYZ, real3* updateQUVW, uint* d_body_num, uint* counter) {
#pragma omp parallel for
	for (uint index = 0; index < number_of_updates; index++) {
		function_Reduce_Shur(index, active, QXYZ, QUVW, inv_mass, inv_inertia, updateQXYZ, updateQUVW, d_body_num, counter);
	}
}

__global__ void device_shurA(int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *gamma, real3 *QXYZ, real3 *QUVW) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_constraints_const);
}

void ChSolverGPU::host_shurA(int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *gamma, real3 *updateV, real3 *updateO,
		real3* QXYZ, real3* QUVW, uint* offset) {
#pragma omp parallel for
	for (int index = 0; index < number_of_contacts; index++) {
		real3 gam;
		gam.x = gamma[index + number_of_contacts * 0];
		gam.y = gamma[index + number_of_contacts * 1];
		gam.z = gamma[index + number_of_contacts * 2];
		uint b1 = ids[index].x;

		int offset1 = offset[index];
		int offset2 = offset[index + number_of_contacts];

		if (active[b1] != 0) {
			updateV[offset1] = JXYZA[index + number_of_contacts * 0] * gam.x + JXYZA[index + number_of_contacts * 1] * gam.y + JXYZA[index + number_of_contacts * 2] * gam.z;
			updateO[offset1] = JUVWA[index + number_of_contacts * 0] * gam.x + JUVWA[index + number_of_contacts * 1] * gam.y + JUVWA[index + number_of_contacts * 2] * gam.z;
		}
		uint b2 = ids[index].y;
		if (active[b2] != 0) {
			updateV[offset2] = JXYZB[index + number_of_contacts * 0] * gam.x + JXYZB[index + number_of_contacts * 1] * gam.y + JXYZB[index + number_of_contacts * 2] * gam.z;
			updateO[offset2] = JUVWB[index + number_of_contacts * 0] * gam.x + JUVWB[index + number_of_contacts * 1] * gam.y + JUVWB[index + number_of_contacts * 2] * gam.z;
		}
	}
//#pragma omp parallel for
//	for (int index = 0; index < number_of_bilaterals; index++) {
//		real gam;
//		gam = gamma[index + number_of_contacts * 3];
//		uint b1 = ids[index].x;
//
//		int offset1 = offset[2 * number_of_contacts + index];
//		int offset2 = offset[2 * number_of_contacts + index + number_of_bilaterals];
//
//		if (active[b1] != 0) {
//			updateV[offset1] = JXYZA[index + number_of_contacts * 0] * gam.x + JXYZA[index + number_of_contacts * 1] * gam.y + JXYZA[index + number_of_contacts * 2] * gam.z;
//			updateO[offset1] = JUVWA[index + number_of_contacts * 0] * gam.x + JUVWA[index + number_of_contacts * 1] * gam.y + JUVWA[index + number_of_contacts * 2] * gam.z;
//		}
//		uint b2 = ids[index].y;
//		if (active[b2] != 0) {
//			updateV[offset2] = JXYZB[index + number_of_contacts * 0] * gam.x + JXYZB[index + number_of_contacts * 1] * gam.y + JXYZB[index + number_of_contacts * 2] * gam.z;
//			updateO[offset2] = JUVWB[index + number_of_contacts * 0] * gam.x + JUVWB[index + number_of_contacts * 1] * gam.y + JUVWB[index + number_of_contacts * 2] * gam.z;
//		}
//	}
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
				gpu_data->vel_update.data(),
				gpu_data->omg_update.data(),
				gpu_data->device_QXYZ_data.data(),
				gpu_data->device_QUVW_data.data(),
				gpu_data->update_offset.data());
#endif

		host_Reduce_Shur(
				gpu_data->device_active_data.data(),
				gpu_data->device_QXYZ_data.data(),
				gpu_data->device_QUVW_data.data(),
				gpu_data->device_mass_data.data(),
				gpu_data->device_inr_data.data(),
				gpu_data->vel_update.data(),
				gpu_data->omg_update.data(),
				gpu_data->body_number.data(),
				gpu_data->offset_counter.data());

	}

__host__ __device__
void function_shurB(uint &index, int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real * gamma, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real3 *QXYZ, real3 *QUVW,
		const real &alpha, const real &compliance, const real & inv_hhpa, real *AX) {
	real temp = 0;
	uint b1 = ids[index].x;
	uint b2 = ids[index].y;
	real3 JXYZ = JXYZA[index];
	if (active[b1] != 0) {
		temp += dot(QXYZ[b1], JXYZ);
		temp += dot(QUVW[b1], JUVWA[index]);
	}
	if (active[b2] != 0) {
		temp += dot(QXYZ[b2], -JXYZ);
		temp += dot(QUVW[b2], JUVWB[index]);
	}
	AX[index] = temp; //+ gamma[index] * inv_hhpa * compliance;
}

__global__ void device_shurB(int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real * gamma, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real3 *QXYZ, real3 *QUVW, real *AX) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_constraints_const);
	function_shurB(index, ids, active, inv_mass, inv_inertia, gamma, JXYZA, JXYZB, JUVWA, JUVWB, QXYZ, QUVW, alpha_const, compliance_const, inv_hhpa_const, AX);
}

void ChSolverGPU::host_shurB(int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real * gamma, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real3 *QXYZ, real3 *QUVW, real *AX) {

//#pragma omp parallel for
//	for (uint index = 0; index < number_of_constraints; index++) {
//		function_shurB(index, ids, active, inv_mass, inv_inertia, gamma, JXYZA, JXYZB, JUVWA, JUVWB, QXYZ, QUVW, alpha, compliance, inv_hhpa, AX);
//	}

#pragma omp parallel for
	for (uint index = 0; index < number_of_contacts; index++) {
		real3 temp = R3(0);
		int2 id_ = ids[index];
		uint b1 = id_.x;
		uint b2 = id_.y;

		if (active[b1] != 0) {
			real3 XYZ = QXYZ[b1];
			real3 UVW = QUVW[b1];

			temp.x += dot(XYZ, JXYZA[index + number_of_contacts * 0]);
			temp.x += dot(UVW, JUVWA[index + number_of_contacts * 0]);

			temp.y += dot(XYZ, JXYZA[index + number_of_contacts * 1]);
			temp.y += dot(UVW, JUVWA[index + number_of_contacts * 1]);

			temp.z += dot(XYZ, JXYZA[index + number_of_contacts * 2]);
			temp.z += dot(UVW, JUVWA[index + number_of_contacts * 2]);

		}
		if (active[b2] != 0) {
			real3 XYZ = QXYZ[b2];
			real3 UVW = QUVW[b2];

			temp.x += dot(XYZ, JXYZB[index + number_of_contacts * 0]);
			temp.x += dot(UVW, JUVWB[index + number_of_contacts * 0]);

			temp.y += dot(XYZ, JXYZB[index + number_of_contacts * 1]);
			temp.y += dot(UVW, JUVWB[index + number_of_contacts * 1]);

			temp.z += dot(XYZ, JXYZB[index + number_of_contacts * 2]);
			temp.z += dot(UVW, JUVWB[index + number_of_contacts * 2]);
		}
		AX[index + number_of_contacts * 0] += temp.x + gamma[index + number_of_contacts * 0] * inv_hhpa * compliance;
		AX[index + number_of_contacts * 1] += temp.y + gamma[index + number_of_contacts * 1] * inv_hhpa * complianceT;
		AX[index + number_of_contacts * 2] += temp.z + gamma[index + number_of_contacts * 2] * inv_hhpa * complianceT;
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
				CASTR1(out));
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

		if (compliance) {
			bi[index + number_of_contacts * 0] = inv_hpa * correction[index];
			bi[index + number_of_contacts * 1] = 0;
			bi[index + number_of_contacts * 2] = 0;
		} else {
			bi[index + number_of_contacts * 0] = fmax(real(1.0) / step_size * correction[index], -contact_recovery_speed);
			bi[index + number_of_contacts * 1] = 0;
			bi[index + number_of_contacts * 2] = 0;
		}

		//function_bi(index, number_of_contacts, step_size, correction, contact_recovery_speed, alpha, bi);
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
#pragma omp parallel for
	for (uint index = 0; index < number_of_constraints; index++) {
		function_RHS(index, step_size, ids, bi, contact_recovery_speed, active, vel, omega, JXYZA, JXYZB, JUVWA, JUVWB, alpha, rhs);
	}
}

void ChSolverGPU::ComputeRHS() {
	timer_rhs.start();
	//Thrust_Fill(gpu_data->device_QXYZ_data, R3(0));
	//Thrust_Fill(gpu_data->device_QUVW_data, R3(0));

#ifdef SIM_ENABLE_GPU_MODE
	device_bi CUDA_KERNEL_DIM(BLOCKS(number_of_constraints), THREADS)(CASTR1(correction), CASTR1(bi));
	cudaDeviceSynchronize();
#else
	host_bi(correction.data(), bi.data());
#endif

#ifdef SIM_ENABLE_GPU_MODE
	device_RHS CUDA_KERNEL_DIM(BLOCKS(number_of_constraints), THREADS)(CASTI2(temp_bids), CASTR1(bi), CASTB1(gpu_data->device_active_data), CASTR3(gpu_data->device_vel_data),
			CASTR3(gpu_data->device_omg_data), CASTR3(gpu_data->device_JXYZA_data), CASTR3(gpu_data->device_JXYZB_data), CASTR3(gpu_data->device_JUVWA_data), CASTR3(gpu_data->device_JUVWB_data),
			CASTR1(rhs));
	cudaDeviceSynchronize();
#else
	host_RHS(temp_bids.data(), bi.data(), gpu_data->device_active_data.data(), gpu_data->device_vel_data.data(), gpu_data->device_omg_data.data(), gpu_data->device_JXYZA_data.data(),
			gpu_data->device_JXYZB_data.data(), gpu_data->device_JUVWA_data.data(), gpu_data->device_JUVWB_data.data(), rhs.data());
#endif

	timer_rhs.stop();
	time_rhs = timer_rhs();
}

__global__ void device_Offsets(int2* ids, real4* bilaterals, uint* Body) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;

	if (index < number_of_contacts_const) {
		int2 temp_id = ids[index];
		Body[index] = temp_id.x;
		Body[index + number_of_contacts_const] = temp_id.y;
	}

	if (index < number_of_bilaterals_const) {
		Body[2 * number_of_contacts_const + index] = bilaterals[index].w;
		Body[2 * number_of_contacts_const + index + number_of_bilaterals_const] = bilaterals[index + number_of_bilaterals_const].w;
	}
}

void ChSolverGPU::host_Offsets(int2* ids, real4* bilaterals, uint* Body) {
	for (uint index = 0; index < number_of_contacts + number_of_bilaterals; index++) {
		if (index < number_of_contacts) {
			int2 temp_id = ids[index];
			Body[index] = temp_id.x;
			Body[index + number_of_contacts] = temp_id.y;
		}

		if (index < number_of_bilaterals) {
			Body[2 * number_of_contacts + index] = bilaterals[index].w;
			Body[2 * number_of_contacts + index + number_of_bilaterals] = bilaterals[index + number_of_bilaterals].w;
		}
	}
}

void ChSolverGPU::Setup() {
	time_rhs = time_shurcompliment = time_project = time_integrate = 0;
	///
	maxd_hist.clear();
	maxdeltalambda_hist.clear();
	iter_hist.clear();
	///
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
	Thrust_Fill(gpu_data->device_QXYZ_data, R3(0));
	Thrust_Fill(gpu_data->device_QUVW_data, R3(0));
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
	COPY_TO_CONST_MEM(number_of_bilaterals);
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
	//cudaFuncSetCacheConfig(device_Offsets, cudaFuncCachePreferL1);

#else
#endif
	uint number_of_cont_bilat = number_of_contacts + number_of_bilaterals;

	if (number_of_cont_bilat > 0) {
		update_number.resize((number_of_cont_bilat) * 2, 0);
		gpu_data->offset_counter.resize((number_of_cont_bilat) * 2, 0);
		gpu_data->update_offset.resize((number_of_cont_bilat) * 2, 0);
		body_num.resize((number_of_cont_bilat) * 2, 0);
		gpu_data->device_dgm_data.resize((number_of_constraints));
		gpu_data->device_gam_data.resize((number_of_constraints));
		Thrust_Fill(gpu_data->device_dgm_data, 1);
		gpu_data->vel_update.resize((number_of_cont_bilat) * 2);
		gpu_data->omg_update.resize((number_of_cont_bilat) * 2);
#ifdef SIM_ENABLE_GPU_MODE
		device_Offsets CUDA_KERNEL_DIM(BLOCKS(number_of_cont_bilat), THREADS)(CASTI2(gpu_data->device_bids_data), CASTR4(gpu_data->device_bilateral_data), CASTU1(body_num));
#else
		host_Offsets(gpu_data->device_bids_data.data(), gpu_data->device_bilateral_data.data(), body_num.data());
#endif
		Thrust_Sequence(update_number);
		Thrust_Sequence(gpu_data->update_offset);
		Thrust_Fill(gpu_data->offset_counter, 0);
		Thrust_Sort_By_Key(body_num, update_number);
		Thrust_Sort_By_Key(update_number, gpu_data->update_offset);
		gpu_data->body_number = body_num;
		Thrust_Reduce_By_KeyB(gpu_data->number_of_updates, body_num, update_number, gpu_data->offset_counter);
		Thrust_Inclusive_Scan(gpu_data->offset_counter);
	}
	number_of_updates = gpu_data->number_of_updates;
}

void ChSolverGPU::ShurProduct(custom_vector<real> &x, custom_vector<real> & output) {
	timer_shurcompliment.start();
	Thrust_Fill(output, 0);
	shurA(x);
	shurB(x,output);
	timer_shurcompliment.stop();
	time_shurcompliment +=timer_shurcompliment();
}

void ChSolverGPU::ComputeImpulses() {
	shurA(gpu_data->device_gam_data);
	gpu_data->device_vel_data += gpu_data->device_QXYZ_data;
	gpu_data->device_omg_data += gpu_data->device_QUVW_data;
}

void ChSolverGPU::Solve(GPUSOLVERTYPE solver_type, real step, gpu_container &gpu_data_) {
	timer_solver.start();
	gpu_data = &gpu_data_;
	step_size = step;
	Setup();
	total_iteration = 0;
	if (number_of_constraints > 0) {
		ComputeRHS();

		if (solver_type == STEEPEST_DESCENT) {
			total_iteration += SolveSD(gpu_data->device_gam_data, rhs, max_iteration);
		} else if (solver_type == GRADIENT_DESCENT) {
			total_iteration += SolveGD(gpu_data->device_gam_data, rhs, max_iteration);
		} else if (solver_type == CONJUGATE_GRADIENT) {
			total_iteration += SolveCG(gpu_data->device_gam_data, rhs, max_iteration);
		} else if (solver_type == CONJUGATE_GRADIENT_SQUARED) {
			total_iteration += SolveCGS(gpu_data->device_gam_data, rhs, max_iteration);
		} else if (solver_type == BICONJUGATE_GRADIENT) {
			total_iteration += SolveBiCG(gpu_data->device_gam_data, rhs, max_iteration);
		} else if (solver_type == BICONJUGATE_GRADIENT_STAB) {
			total_iteration += SolveBiCGStab(gpu_data->device_gam_data, rhs, max_iteration);
		} else if (solver_type == MINIMUM_RESIDUAL) {
			total_iteration += SolveMinRes(gpu_data->device_gam_data, rhs, max_iteration);
		}
		//else if(solver_type==QUASAI_MINIMUM_RESIDUAL){SolveQMR(gpu_data->device_gam_data, rhs, max_iteration);}
		else if (solver_type == ACCELERATED_PROJECTED_GRADIENT_DESCENT) {
			total_iteration += SolveAPGD(gpu_data->device_gam_data, rhs, max_iteration);
		}
		current_iteration = total_iteration;
		ComputeImpulses();
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
