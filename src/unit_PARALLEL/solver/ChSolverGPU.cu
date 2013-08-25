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

//=================================================================================================================================
__host__ __device__ void function_Project(uint &index, uint number_of_contacts, int2 *ids, real *fric, real* cohesion, real *gam) {
	int2 body_id = ids[index];
	real3 gamma;
	gamma.x = gam[index + number_of_contacts * 0];
	gamma.y = gam[index + number_of_contacts * 1];
	gamma.z = gam[index + number_of_contacts * 2];

	real coh = (cohesion[body_id.x] + cohesion[body_id.y]) * .5;
	if (coh < 0) {
		coh = 0;
	}
	gamma.x += coh;
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

	gam[index + number_of_contacts * 0] = gamma.x - coh;
	gam[index + number_of_contacts * 1] = gamma.y;
	gam[index + number_of_contacts * 2] = gamma.z;
}

__global__ void device_Project(int2 *ids, real *friction, real* cohesion, real *gamma) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_contacts_const);
	function_Project(index, number_of_contacts_const, ids, friction, cohesion, gamma);
}

void ChSolverGPU::host_Project(int2 *ids, real *friction, real* cohesion, real *gamma) {
#pragma omp parallel for
	for (uint index = 0; index < number_of_contacts; index++) {
		function_Project(index, number_of_contacts, ids, friction, cohesion, gamma);
	}
}
void ChSolverGPU::Project(custom_vector<real> & gamma) {
	timer_project.start();

#ifdef SIM_ENABLE_GPU_MODE
		device_Project CUDA_KERNEL_DIM(BLOCKS(number_of_contacts), THREADS)(
				CASTI2(data_container->device_data.device_bids_data),
				CASTR1(data_container->device_data.device_fric_data),
				CASTR1(data_container->device_data.device_cohesion_data),
				CASTR1(gamma));
#else
		host_Project (
				data_container->host_data.bids_data.data(),
				data_container->host_data.fric_data.data(),
				data_container->host_data.cohesion_data.data(),
				gamma.data());
#endif
		timer_project.stop();
		time_project += timer_project();
	}
//=================================================================================================================================

__host__ __device__ void function_Reduce_Shur(
		uint& index,
		bool* active,
		real3* QXYZ,
		real3* QUVW,
		real *inv_mass,
		real3 *inv_inertia,
		real3* updateQXYZ,
		real3* updateQUVW,
		uint* d_body_num,
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

void ChSolverGPU::host_Reduce_Shur(
		bool* active,
		real3* QXYZ,
		real3* QUVW,
		real *inv_mass,
		real3 *inv_inertia,
		real3* updateQXYZ,
		real3* updateQUVW,
		uint* d_body_num,
		uint* counter) {
#pragma omp parallel for
	for (uint index = 0; index < number_of_updates; index++) {
		function_Reduce_Shur(index, active, QXYZ, QUVW, inv_mass, inv_inertia, updateQXYZ, updateQUVW, d_body_num, counter);
	}
}
//=================================================================================================================================
__global__ void device_shurA(
		int2 *ids,
		bool *active,
		real *inv_mass,
		real3 *inv_inertia,
		real3 *JXYZA,
		real3 *JXYZB,
		real3 *JUVWA,
		real3 *JUVWB,
		real *gamma,
		real3 *QXYZ,
		real3 *QUVW) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_constraints_const);
}

void ChSolverGPU::host_shurA_contacts(
		int2 *ids,
		bool *active,
		real *inv_mass,
		real3 *inv_inertia,
		real3 *JXYZA,
		real3 *JXYZB,
		real3 *JUVWA,
		real3 *JUVWB,
		real *gamma,
		real3 *updateV,
		real3 *updateO,
		real3* QXYZ,
		real3* QUVW,
		uint* offset) {
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
			updateV[offset1] = JXYZA[index + number_of_contacts * 0] * gam.x + JXYZA[index + number_of_contacts * 1] * gam.y
					+ JXYZA[index + number_of_contacts * 2] * gam.z;
			updateO[offset1] = JUVWA[index + number_of_contacts * 0] * gam.x + JUVWA[index + number_of_contacts * 1] * gam.y
					+ JUVWA[index + number_of_contacts * 2] * gam.z;
		}
		uint b2 = ids[index].y;
		if (active[b2] != 0) {
			updateV[offset2] = JXYZB[index + number_of_contacts * 0] * gam.x + JXYZB[index + number_of_contacts * 1] * gam.y
					+ JXYZB[index + number_of_contacts * 2] * gam.z;
			updateO[offset2] = JUVWB[index + number_of_contacts * 0] * gam.x + JUVWB[index + number_of_contacts * 1] * gam.y
					+ JUVWB[index + number_of_contacts * 2] * gam.z;
		}
	}
}

void ChSolverGPU::host_shurA_bilaterals(
		int2 *ids,
		bool *active,
		real *inv_mass,
		real3 *inv_inertia,
		real3 *JXYZA,
		real3 *JXYZB,
		real3 *JUVWA,
		real3 *JUVWB,
		real *gamma,
		real3 *updateV,
		real3 *updateO,
		real3* QXYZ,
		real3* QUVW,
		uint* offset) {

#pragma omp parallel for
	for (int index = 0; index < number_of_bilaterals; index++) {
		real gam;
		gam = gamma[index + number_of_contacts * 3];
		uint b1 = ids[index + number_of_contacts * 3].x;
		int offset1 = offset[2 * number_of_contacts + index];
		int offset2 = offset[2 * number_of_contacts + index + number_of_bilaterals];

		if (active[b1] != 0) {
			updateV[offset1] = JXYZA[index + number_of_contacts * 3] * gam;
			updateO[offset1] = JUVWA[index + number_of_contacts * 3] * gam;
		}

		uint b2 = ids[index + number_of_contacts * 3].y;
		if (active[b2] != 0) {
			updateV[offset2] = JXYZB[index + number_of_contacts * 3] * gam;
			updateO[offset2] = JUVWB[index + number_of_contacts * 3] * gam;
		}
	}
}
void ChSolverGPU::shurA(custom_vector<real> &x) {

#ifdef SIM_ENABLE_GPU_MODE
		device_shurA CUDA_KERNEL_DIM(BLOCKS(number_of_constraints), THREADS)(
				CASTI2(data_container->device_data.device_bidlist_data),
				CASTB1(data_container->device_data.device_active_data),
				CASTR1(data_container->device_data.device_mass_data),
				CASTR3(data_container->device_data.device_inr_data),
				CASTR3(data_container->device_data.device_JXYZA_data),
				CASTR3(data_container->device_data.device_JXYZB_data),
				CASTR3(data_container->device_data.device_JUVWA_data),
				CASTR3(data_container->device_data.device_JUVWB_data),
				CASTR1(x),
				CASTR3(data_container->device_data.device_QXYZ_data),
				CASTR3(data_container->device_data.device_QUVW_data));
		cudaDeviceSynchronize();
#else
		host_shurA_contacts(
				data_container->host_data.bidlist_data.data(),
				data_container->host_data.active_data.data(),
				data_container->host_data.mass_data.data(),
				data_container->host_data.inr_data.data(),
				data_container->host_data.JXYZA_data.data(),
				data_container->host_data.JXYZB_data.data(),
				data_container->host_data.JUVWA_data.data(),
				data_container->host_data.JUVWB_data.data(),
				x.data(),
				data_container->host_data.vel_update.data(),
				data_container->host_data.omg_update.data(),
				data_container->host_data.QXYZ_data.data(),
				data_container->host_data.QUVW_data.data(),
				data_container->host_data.update_offset.data());
		host_shurA_bilaterals(
				data_container->host_data.bidlist_data.data(),
				data_container->host_data.active_data.data(),
				data_container->host_data.mass_data.data(),
				data_container->host_data.inr_data.data(),
				data_container->host_data.JXYZA_data.data(),
				data_container->host_data.JXYZB_data.data(),
				data_container->host_data.JUVWA_data.data(),
				data_container->host_data.JUVWB_data.data(),
				x.data(),
				data_container->host_data.vel_update.data(),
				data_container->host_data.omg_update.data(),
				data_container->host_data.QXYZ_data.data(),
				data_container->host_data.QUVW_data.data(),
				data_container->host_data.update_offset.data());

		host_Reduce_Shur(
				data_container->host_data.active_data.data(),
				data_container->host_data.QXYZ_data.data(),
				data_container->host_data.QUVW_data.data(),
				data_container->host_data.mass_data.data(),
				data_container->host_data.inr_data.data(),
				data_container->host_data.vel_update.data(),
				data_container->host_data.omg_update.data(),
				data_container->host_data.body_number.data(),
				data_container->host_data.offset_counter.data());

#endif

	}
//=================================================================================================================================
__host__ __device__
void function_shurB(
		uint &index,
		int2 *ids,
		bool *active,
		real *inv_mass,
		real3 *inv_inertia,
		real * compliance,
		real * gamma,
		real3 *JXYZA,
		real3 *JXYZB,
		real3 *JUVWA,
		real3 *JUVWB,
		real3 *QXYZ,
		real3 *QUVW,
		const real &alpha,
		const real & inv_hhpa,
		real *AX) {
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
	AX[index] = temp + gamma[index] * inv_hhpa * compliance[index];
}

__global__ void device_shurB(
		int2 *ids,
		bool *active,
		real *inv_mass,
		real3 *inv_inertia,
		real * compliance,
		real * gamma,
		real3 *JXYZA,
		real3 *JXYZB,
		real3 *JUVWA,
		real3 *JUVWB,
		real3 *QXYZ,
		real3 *QUVW,
		real *AX) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_constraints_const);
	function_shurB(
			index,
			ids,
			active,
			inv_mass,
			inv_inertia,
			compliance,
			gamma,
			JXYZA,
			JXYZB,
			JUVWA,
			JUVWB,
			QXYZ,
			QUVW,
			alpha_const,
			inv_hhpa_const,
			AX);
}

void ChSolverGPU::host_shurB_contacts(
		int2 *ids,
		bool *active,
		real *inv_mass,
		real3 *inv_inertia,
		real * compliance,
		real * gamma,
		real3 *JXYZA,
		real3 *JXYZB,
		real3 *JUVWA,
		real3 *JUVWB,
		real3 *QXYZ,
		real3 *QUVW,
		real *AX) {

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
		AX[index + number_of_contacts * 0] += temp.x + gamma[index + number_of_contacts * 0] * inv_hhpa * compliance[index];
		AX[index + number_of_contacts * 1] += temp.y + gamma[index + number_of_contacts * 1] * inv_hhpa * compliance[index];
		AX[index + number_of_contacts * 2] += temp.z + gamma[index + number_of_contacts * 2] * inv_hhpa * compliance[index];
	}

}
void ChSolverGPU::host_shurB_bilaterals(
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
		real *AX) {

#pragma omp parallel for
	for (uint index = 0; index < number_of_bilaterals; index++) {

		real temp = 0;
		uint b1 = ids[index + number_of_contacts * 3].x;
		if (active[b1] != 0) {
			real3 XYZ = QXYZ[b1];
			real3 UVW = QUVW[b1];
			temp += dot(XYZ, JXYZA[index + number_of_contacts * 3]);
			temp += dot(UVW, JUVWA[index + number_of_contacts * 3]);

		}
		uint b2 = ids[index + number_of_contacts * 3].y;
		if (active[b2] != 0) {

			real3 XYZ = QXYZ[b2];
			real3 UVW = QUVW[b2];
			temp += dot(XYZ, JXYZB[index + number_of_contacts * 3]);
			temp += dot(UVW, JUVWB[index + number_of_contacts * 3]);

		}

		AX[index + number_of_contacts * 3] = temp;
	}
}
void ChSolverGPU::shurB(custom_vector<real> &x, custom_vector<real> &out) {
#ifdef SIM_ENABLE_GPU_MODE
		device_shurB CUDA_KERNEL_DIM(BLOCKS(number_of_constraints), THREADS)(
				CASTI2(data_container->device_data.device_bidlist_data),
				CASTB1(data_container->device_data.device_active_data),
				CASTR1(data_container->device_data.device_mass_data),
				CASTR3(data_container->device_data.device_inr_data),
				CASTR1(data_container->device_data.device_compliance_data),
				CASTR1(data_container->device_data.device_complianceT_data),
				CASTR1(x),
				CASTR3(data_container->device_data.device_JXYZA_data),
				CASTR3(data_container->device_data.device_JXYZB_data),
				CASTR3(data_container->device_data.device_JUVWA_data),
				CASTR3(data_container->device_data.device_JUVWB_data),
				CASTR3(data_container->device_data.device_QXYZ_data),
				CASTR3(data_container->device_data.device_QUVW_data),
				CASTR1(out));
#else
		host_shurB_contacts(
				data_container->host_data.bidlist_data.data(),
				data_container->host_data.active_data.data(),
				data_container->host_data.mass_data.data(),
				data_container->host_data.inr_data.data(),
				data_container->host_data.comp_data.data(),
				x.data(),
				data_container->host_data.JXYZA_data.data(),
				data_container->host_data.JXYZB_data.data(),
				data_container->host_data.JUVWA_data.data(),
				data_container->host_data.JUVWB_data.data(),
				data_container->host_data.QXYZ_data.data(),
				data_container->host_data.QUVW_data.data(),
				out.data());
		host_shurB_bilaterals(
				data_container->host_data.bidlist_data.data(),
				data_container->host_data.active_data.data(),
				data_container->host_data.mass_data.data(),
				data_container->host_data.inr_data.data(),
				x.data(),
				data_container->host_data.JXYZA_data.data(),
				data_container->host_data.JXYZB_data.data(),
				data_container->host_data.JUVWA_data.data(),
				data_container->host_data.JUVWB_data.data(),
				data_container->host_data.QXYZ_data.data(),
				data_container->host_data.QUVW_data.data(),
				out.data());
#endif
	}
void ChSolverGPU::ShurProduct(custom_vector<real> &x, custom_vector<real> & output) {
	timer_shurcompliment.start();
	Thrust_Fill(output, 0);
#ifdef SIM_ENABLE_GPU_MODE
		Thrust_Fill(data_container->device_data.device_QXYZ_data, R3(0));
		Thrust_Fill(data_container->device_data.device_QUVW_data, R3(0));
#else
#pragma omp parallel for
		for (int i = 0; i < number_of_objects; i++) {
			data_container->host_data.QXYZ_data[i] = R3(0);
			data_container->host_data.QUVW_data[i] = R3(0);
		}
#endif
		shurA(x);
		shurB(x,output);
		timer_shurcompliment.stop();
		time_shurcompliment +=timer_shurcompliment();
	}
#define JXYZA_bilateral data_container->host_data.JXYZA_bilateral
#define JXYZB_bilateral data_container->host_data.JXYZB_bilateral
#define JUVWA_bilateral data_container->host_data.JUVWA_bilateral
#define JUVWB_bilateral data_container->host_data.JUVWB_bilateral
//=================================================================================================================================
void ChSolverGPU::ShurBilaterals(custom_vector<real> &x_t, custom_vector<real> & AX) {

	for (int i = 0; i < number_of_objects; i++) {
		data_container->host_data.QXYZ_data[i] = R3(0);
		data_container->host_data.QUVW_data[i] = R3(0);
	}

	for (int index = 0; index < number_of_bilaterals; index++) {
		real gam;
		gam = x_t[index];
		uint b1 = data_container->host_data.bidlist_data[index + number_of_contacts * 3].x;

		if (data_container->host_data.active_data[b1] != 0) {
			data_container->host_data.QXYZ_data[b1] += JXYZA_bilateral[index] * gam;
			data_container->host_data.QUVW_data[b1] += JUVWA_bilateral[index] * gam;
		}

		uint b2 = data_container->host_data.bidlist_data[index + number_of_contacts * 3].y;
		if (data_container->host_data.active_data[b2] != 0) {
			data_container->host_data.QXYZ_data[b2] += JXYZB_bilateral[index ] * gam;
			data_container->host_data.QUVW_data[b2] += JUVWB_bilateral[index ] * gam;
		}
	}

#pragma omp parallel for
		for (uint index = 0; index < number_of_bilaterals; index++) {

			real temp = 0;
			uint b1 = data_container->host_data.bidlist_data[index + number_of_contacts * 3].x;
			if (data_container->host_data.active_data[b1] != 0) {
				real3 XYZ = data_container->host_data.QXYZ_data[b1]* data_container->host_data.mass_data[b1];
				real3 UVW = data_container->host_data.QUVW_data[b1]* data_container->host_data.inr_data[b1];
				temp += dot(XYZ, JXYZA_bilateral[index ]);
				temp += dot(UVW, JUVWA_bilateral[index ]);

			}
			uint b2 = data_container->host_data.bidlist_data[index + number_of_contacts * 3].y;
			if (data_container->host_data.active_data[b2] != 0) {

				real3 XYZ = data_container->host_data.QXYZ_data[b2]* data_container->host_data.mass_data[b2];;
				real3 UVW = data_container->host_data.QUVW_data[b2]* data_container->host_data.inr_data[b2];
				temp += dot(XYZ, JXYZB_bilateral[index ]);
				temp += dot(UVW, JUVWB_bilateral[index ]);

			}
			AX[index ] = temp;
		}

	}
//=================================================================================================================================
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

void ChSolverGPU::host_Offsets(int2* ids_contacts, int2* ids_bilaterals, uint* Body) {
	for (uint index = 0; index < number_of_contacts + number_of_bilaterals; index++) {
		if (index < number_of_contacts) {
			int2 temp_id = ids_contacts[index];
			Body[index] = temp_id.x;
			Body[index + number_of_contacts] = temp_id.y;
		}

		if (index < number_of_bilaterals) {
			int2 temp_id = ids_bilaterals[index];
			Body[2 * number_of_contacts + index] = temp_id.x;
			Body[2 * number_of_contacts + index + number_of_bilaterals] = temp_id.y;
		}
	}
}
//=================================================================================================================================
void ChSolverGPU::Setup() {
	time_shurcompliment = time_project = time_integrate = 0;
	///////////////////////////////////////
	maxd_hist.clear();
	maxdeltalambda_hist.clear();
	iter_hist.clear();
	///////////////////////////////////////
	Initialize();

	data_container->host_data.gam_data.resize((number_of_constraints));
	data_container->host_data.QXYZ_data.resize(number_of_objects);
	data_container->host_data.QUVW_data.resize(number_of_objects);

	///////////////////////////////////////
#ifdef SIM_ENABLE_GPU_MODE
	Thrust_Fill(data_container->device_data.device_QXYZ_data, R3(0));
	Thrust_Fill(data_container->device_data.device_QUVW_data, R3(0));
	Thrust_Fill(data_container->device_data.device_gam_data, 0);
	Thrust_Fill(correction, 0);

#else
#pragma omp parallel for
	for (int i = 0; i < number_of_objects; i++) {
		data_container->host_data.QXYZ_data[i] = R3(0);
		data_container->host_data.QUVW_data[i] = R3(0);
	}
#pragma omp parallel for
	for (int i = 0; i < number_of_constraints; i++) {
		data_container->host_data.gam_data[i] = 0;
	}
#endif

	///////////////////////////////////////

	///////////////////////////////////////
	thrust::copy_n(
			data_container->host_data.gamma_bilateral.begin(),
			data_container->number_of_bilaterals,
			data_container->host_data.gam_data.begin() + data_container->number_of_contacts * 3);

	///////////////////////////////////////

#ifdef SIM_ENABLE_GPU_MODE
	COPY_TO_CONST_MEM(number_of_constraints);
	COPY_TO_CONST_MEM(number_of_contacts);
	COPY_TO_CONST_MEM(number_of_bilaterals);
	COPY_TO_CONST_MEM(step_size);
	COPY_TO_CONST_MEM(alpha);
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
		data_container->host_data.offset_counter.resize((number_of_cont_bilat) * 2, 0);
		data_container->host_data.update_offset.resize((number_of_cont_bilat) * 2, 0);
		body_num.resize((number_of_cont_bilat) * 2, 0);
		data_container->host_data.vel_update.resize((number_of_cont_bilat) * 2);
		data_container->host_data.omg_update.resize((number_of_cont_bilat) * 2);
#ifdef SIM_ENABLE_GPU_MODE
		device_Offsets CUDA_KERNEL_DIM(BLOCKS(number_of_cont_bilat), THREADS)(CASTI2(data_container->device_data.device_bids_data), CASTR4(data_container->device_data.device_bilateral_data), CASTU1(body_num));
#else
		host_Offsets(data_container->host_data.bids_data.data(), data_container->host_data.bids_bilateral.data(), body_num.data());
#endif
		Thrust_Sequence(update_number);
		Thrust_Sequence(data_container->host_data.update_offset);
		Thrust_Fill(data_container->host_data.offset_counter, 0);
		Thrust_Sort_By_Key(body_num, update_number);
		Thrust_Sort_By_Key(update_number, data_container->host_data.update_offset);
		data_container->host_data.body_number = body_num;
		Thrust_Reduce_By_KeyB(data_container->number_of_updates, body_num, update_number, data_container->host_data.offset_counter);
		Thrust_Inclusive_Scan(data_container->host_data.offset_counter);
	}
	number_of_updates = data_container->number_of_updates;

}

void ChSolverGPU::ComputeImpulses() {
	shurA(data_container->host_data.gam_data);
	data_container->host_data.vel_data += data_container->host_data.QXYZ_data;
	data_container->host_data.omg_data += data_container->host_data.QUVW_data;
}

void ChSolverGPU::Solve(GPUSOLVERTYPE solver_type, real step, ChGPUDataManager *data_container_) {
	timer_solver.start();
	data_container = data_container_;
	step_size = step;

	Setup();
	total_iteration = 0;
	if (number_of_constraints > 0) {

		if (solver_type == STEEPEST_DESCENT) {
			total_iteration += SolveSD(data_container->host_data.gam_data, data_container->host_data.rhs_data, max_iteration);
		} else if (solver_type == GRADIENT_DESCENT) {
			total_iteration += SolveGD(data_container->host_data.gam_data, data_container->host_data.rhs_data, max_iteration);
		} else if (solver_type == CONJUGATE_GRADIENT) {
			total_iteration += SolveCG(data_container->host_data.gam_data, data_container->host_data.rhs_data, max_iteration);
		} else if (solver_type == CONJUGATE_GRADIENT_SQUARED) {
			total_iteration += SolveCGS(data_container->host_data.gam_data, data_container->host_data.rhs_data, max_iteration);
		} else if (solver_type == BICONJUGATE_GRADIENT) {
			total_iteration += SolveBiCG(data_container->host_data.gam_data, data_container->host_data.rhs_data, max_iteration);
		} else if (solver_type == BICONJUGATE_GRADIENT_STAB) {
			total_iteration += SolveBiCGStab(data_container->host_data.gam_data, data_container->host_data.rhs_data, max_iteration);
		} else if (solver_type == MINIMUM_RESIDUAL) {
			total_iteration += SolveMinRes(data_container->host_data.gam_data, data_container->host_data.rhs_data, max_iteration);
		}
		//else if(solver_type==QUASAI_MINIMUM_RESIDUAL){SolveQMR(data_container->gpu_data.device_gam_data, rhs, max_iteration);}
		else if (solver_type == ACCELERATED_PROJECTED_GRADIENT_DESCENT) {
			if (do_stab) {
				custom_vector<real> rhs_bilateral(data_container->number_of_bilaterals);
				thrust::copy_n(data_container->host_data.rhs_data.begin() + data_container->number_of_contacts * 3, data_container->number_of_bilaterals, rhs_bilateral.begin());

				for (int i = 0; i < max_iteration; i += 4) {
					total_iteration += SolveAPGD(data_container->host_data.gam_data, data_container->host_data.rhs_data, 4);
					thrust::copy_n(
							data_container->host_data.gam_data.begin() + data_container->number_of_contacts * 3,
							data_container->number_of_bilaterals,
							data_container->host_data.gamma_bilateral.begin());

					SolveStab(data_container->host_data.gamma_bilateral, rhs_bilateral, 50);

					thrust::copy_n(
							data_container->host_data.gamma_bilateral.begin(),
							data_container->number_of_bilaterals,
							data_container->host_data.gam_data.begin() + data_container->number_of_contacts * 3);

					total_iteration += SolveAPGD(data_container->host_data.gam_data, data_container->host_data.rhs_data, 4);
				}
			} else {
				total_iteration += SolveAPGD(data_container->host_data.gam_data, data_container->host_data.rhs_data, max_iteration);
			}
		} else if (solver_type == BLOCK_JACOBI) {
			SolveJacobi();
		}
		thrust::copy_n(
				data_container->host_data.gam_data.begin() + data_container->number_of_contacts * 3,
				data_container->number_of_bilaterals,
				data_container->host_data.gamma_bilateral.begin());

		current_iteration = total_iteration;
		ComputeImpulses();
		timer_solver.stop();
		time_solver = timer_solver();
	}
}

void ChSolverGPU::VelocityStabilization(ChGPUDataManager *data_container_) {

}

uint ChSolverGPU::SolveStab(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter) {
	uint N = b.size();
	custom_vector<real> v(N, 0), v_hat(x.size()), w(N, 0), w_old, xMR, v_old, Av(x.size()), w_oold;
	real beta, c = 1, eta, norm_rMR, norm_r0, c_old = 1, s_old = 0, s = 0, alpha, beta_old, c_oold, s_oold, r1_hat, r1, r2, r3;
	ShurBilaterals(x,v_hat);
	v_hat = b - v_hat;
	beta = Norm(v_hat);
	w_old = w;
	eta = beta;
	xMR = x;
	norm_rMR = beta;
	norm_r0 = beta;

	if (beta == 0 || norm_rMR / norm_r0 < tolerance) {return 0;}

	for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
		//// Lanczos
		v_old = v;
		v = 1.0 / beta * v_hat;
		ShurBilaterals(v,Av);
		alpha = Dot(v, Av);
		v_hat = Av - alpha * v - beta * v_old;
		beta_old = beta;
		beta = Norm(v_hat);
		//// QR factorization
		c_oold = c_old;
		c_old = c;
		s_oold = s_old;
		s_old = s;
		r1_hat = c_old * alpha - c_oold * s_old * beta_old;
		r1 = 1 / sqrt(r1_hat * r1_hat + beta * beta);
		r2 = s_old * alpha + c_oold * c_old * beta_old;
		r3 = s_oold * beta_old;
		//// Givens Rotation
		c = r1_hat * r1;
		s = beta * r1;
		//// update
		w_oold = w_old;
		w_old = w;
		w = r1 * (v - r3 * w_oold - r2 * w_old);
		x = x + c * eta * w;
		norm_rMR = norm_rMR * abs(s);
		eta = -s * eta;
		residual = norm_rMR / norm_r0;

		if (residual < tolerance) {break;}
	}

	return current_iteration;

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
