#include "ChSolverCG.h"
using namespace chrono;

__constant__ uint number_of_constraints_const;

ChSolverCG::ChSolverCG() {

}



//
__host__ __device__ void function_shurA(
		uint& index,
		real3* JXYZA,
		real3* JXYZB,
		real3* JUVWA,
		real3* JUVWB,
		real * gamma,
		int2* ids,
		real* inv_mass,
		real3* inv_inertia,
		bool* active,
		real3 * QXYZ,
		real3 * QUVW) {
	real q_x, q_y, q_z, q_u, q_v, q_w;
	real temp = 0, m;
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
__global__ void device_shurA(real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB, real * gamma, int2* ids, real* inv_mass, real3* inv_inertia, bool* active, real3 * QXYZ, real3 * QUVW) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_constraints_const);
	function_shurA(index, JXYZA, JXYZB, JUVWA, JUVWB, gamma, ids, inv_mass, inv_inertia,active, QXYZ, QUVW);
}

void ChSolverCG::host_shurA(real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB, real * gamma, int2* ids, real* inv_mass, real3* inv_inertia, bool* active, real3 * QXYZ, real3 * QUVW) {
//#pragma omp parallel for schedule(guided)
	for (uint index = 0; index < number_of_constraints; index++) {
		function_shurA(index, JXYZA, JXYZB, JUVWA, JUVWB, gamma, ids, inv_mass, inv_inertia, active, QXYZ, QUVW);
	}
}

__host__ __device__ void function_shurB(
		uint& index,
		real3* JXYZA,
		real3* JXYZB,
		real3* JUVWA,
		real3* JUVWB,
		real * gamma,
		real * inv_mass,
		real3 * inv_inertia,
		int2* ids,
		bool * active,
		real3 * QXYZ,
		real3 * QUVW,
		real * AX) {
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
		temp += QXYZ[b2].x * JXYZB[index].x * m1;
		temp += QXYZ[b2].y * JXYZB[index].y * m1;
		temp += QXYZ[b2].z * JXYZB[index].z * m1;
		temp += QUVW[b2].x * JUVWB[index].x * inertia1.x;
		temp += QUVW[b2].y * JUVWB[index].y * inertia1.y;
		temp += QUVW[b2].z * JUVWB[index].z * inertia1.z;
	}

	AX[index] += temp;
}

__global__ void device_shurB(real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB, real * gamma, int2* ids, real* inv_mass, real3* inv_inertia, bool* active, real3 * QXYZ, real3 * QUVW, real * AX) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_constraints_const);
	function_shurB(index, JXYZA, JXYZB, JUVWA, JUVWB, gamma, inv_mass, inv_inertia, ids,active,  QXYZ, QUVW, AX);
}

void ChSolverCG::host_shurB(real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB, real * gamma, int2* ids, real* inv_mass, real3* inv_inertia, bool* active, real3 * QXYZ, real3 * QUVW, real * AX) {
#pragma omp parallel for schedule(guided)
	for (uint index = 0; index < number_of_constraints; index++) {
		function_shurB(index, JXYZA, JXYZB, JUVWA, JUVWB, gamma, inv_mass, inv_inertia, ids,active,  QXYZ, QUVW, AX);
	}
}

__host__ __device__ void function_RHS(
		uint &index,
		int2* ids,
		real * inv_mass,
		real3* JXYZA,
		real3* JXYZB,
		real3* JUVWA,
		real3* JUVWB,
		real3* vel,
		real3* omega,
		real* correction,
		real & step_size,
		real * rhs) {
	uint b1 = ids[index].x;
	uint b2 = ids[index].y;
	real m1 = inv_mass[b1];
	real m2 = inv_mass[b2];

	real temp = 0;
	temp -= JXYZA[index].x * vel[b1].x;
	temp -= JXYZA[index].y * vel[b1].y;
	temp -= JXYZA[index].z * vel[b1].z;
	temp -= JUVWA[index].x * omega[b1].x;
	temp -= JUVWA[index].y * omega[b1].y;
	temp -= JUVWA[index].z * omega[b1].z;

	temp -= JXYZB[index].x * vel[b2].x;
	temp -= JXYZB[index].y * vel[b2].y;
	temp -= JXYZB[index].z * vel[b2].z;
	temp -= JUVWB[index].x * omega[b2].x;
	temp -= JUVWB[index].y * omega[b2].y;
	temp -= JUVWB[index].z * omega[b2].z;

	rhs[index] = temp - fmaxf(1.0 / step_size * correction[index], 0);

}

__global__ void device_RHS(int2* ids, real * inv_mass, real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB, real3* vel, real3* omega, real* correction, real step_size, real * rhs) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_constraints_const);
	function_RHS(index, ids, inv_mass, JXYZA, JXYZB, JUVWA, JUVWB, vel, omega, correction, step_size, rhs);
}

void ChSolverCG::host_RHS(int2* ids, real * inv_mass, real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB, real3* vel, real3* omega, real* correction, real step_size, real * rhs) {
#pragma omp parallel for schedule(guided)
	for (uint index = 0; index < number_of_constraints; index++) {
		function_RHS(index, ids, inv_mass, JXYZA, JXYZB, JUVWA, JUVWB, vel, omega, correction, step_size, rhs);
	}
}

//
//__host__ __device__ void function_InvNDiag(uint index, int b1, int b2, int2* ids, real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB, real * inv_mass, real3 * inv_inertia, real & inv_n_diag) {
//
//	real Jx1 = JXYZA[index].x;
//	real Jy1 = JXYZA[index].y;
//	real Jz1 = JXYZA[index].z;
//	real Ju1 = JUVWA[index].x;
//	real Jv1 = JUVWA[index].y;
//	real Jw1 = JUVWA[index].z;
//	//
//	real Jx2 = JXYZB[index].x;
//	real Jy2 = JXYZB[index].y;
//	real Jz2 = JXYZB[index].z;
//	real Ju2 = JUVWB[index].x;
//	real Jv2 = JUVWB[index].y;
//	real Jw2 = JUVWB[index].z;
//	//
//	real m1 = inv_mass[b1];
//	real m2 = inv_mass[b2];
//	//
//	real3 i1 = inv_inertia[b1];
//	real3 i2 = inv_inertia[b2];
//
//	real part1 = dot(JXYZA[index], JXYZA[index]) * m1;
//	real part2 = dot(JUVWA[index] * JUVWA[index], i1);
//	real part3 = dot(JXYZB[index], JXYZB[index]) * m2;
//	real part4 = dot(JUVWB[index] * JUVWB[index], i2);
//	inv_n_diag = (part1 + part2 + part3 + part4);
//}


custom_vector<real> ChSolverCG::ShurProduct(const custom_vector<real> &x_t, const custom_vector<real> &A)
{
	if (x_t.size() == 0) {
		custom_vector<real> result(0, 0);
		return result;
	}
#ifdef SIM_ENABLE_GPU_MODE
	COPY_TO_CONST_MEM(number_of_constraints);
#endif

#ifdef SIM_ENABLE_GPU_MODE

	device_shurA CUDA_KERNEL_DIM(BLOCKS(number_of_constraints), THREADS)(
			CASTR3(gpu_data->device_JXYZA_data),
			CASTR3(gpu_data->device_JXYZB_data),
			CASTR3(gpu_data->device_JUVWA_data),
			CASTR3(gpu_data->device_JUVWB_data),
			CASTR1(gpu_data->device_gam_data),
			CASTI2(temp_bids),
			CASTR3( gpu_data->device_QXYZ_data),
			CASTR3( gpu_data->device_QUVW_data));

	device_shurB(
			CASTR3(gpu_data->device_JXYZA_data),
			CASTR3(gpu_data->device_JXYZB_data),
			CASTR3(gpu_data->device_JUVWA_data),
			CASTR3(gpu_data->device_JUVWB_data),
			CASTR1(gamma),
			real * inv_mass,
			real3 * inv_inertia,
			int2 * ids,
			real3 * QXYZ,
			real3 * QUVW,
			real * AX);

#else

	host_shurA(
			gpu_data->device_JXYZA_data.data(),
			gpu_data->device_JXYZB_data.data(),
			gpu_data->device_JUVWA_data.data(),
			gpu_data->device_JUVWB_data.data(),
			gpu_data->device_dgm_data.data(),
			temp_bids.data(),
			gpu_data->device_mass_data.data(),
			gpu_data->device_inr_data.data(),
			gpu_data->device_active_data.data(),
			gpu_data->device_QXYZ_data.data(),
			gpu_data->device_QUVW_data.data());
	host_shurB(
			gpu_data->device_JXYZA_data.data(),
			gpu_data->device_JXYZB_data.data(),
			gpu_data->device_JUVWA_data.data(),
			gpu_data->device_JUVWB_data.data(),
			gpu_data->device_dgm_data.data(),
			temp_bids.data(),
			gpu_data->device_mass_data.data(),
			gpu_data->device_inr_data.data(),
			gpu_data->device_active_data.data(),
			gpu_data->device_QXYZ_data.data(),
			gpu_data->device_QUVW_data.data(),
			AX.data());

#endif
}

void ChSolverCG::Solve(real step, gpu_container& gpu_data_) {
	gpu_data=&gpu_data_;

	number_of_constraints = gpu_data->number_of_contacts * 3 + gpu_data->number_of_bilaterals;

	temp_bids.resize(number_of_constraints);
	AX.resize(number_of_constraints);

	thrust::copy_n(gpu_data->device_bids_data.begin(), gpu_data->number_of_contacts, temp_bids.begin() + gpu_data->number_of_contacts * 0);
	thrust::copy_n(gpu_data->device_bids_data.begin(), gpu_data->number_of_contacts, temp_bids.begin() + gpu_data->number_of_contacts * 1);
	thrust::copy_n(gpu_data->device_bids_data.begin(), gpu_data->number_of_contacts, temp_bids.begin() + gpu_data->number_of_contacts * 2);


}

uint ChSolverCG::SolveCG(custom_vector<real> &x, const custom_vector<real> &b, const custom_vector<real> &A, const uint max_iter) {
	custom_vector<real> r(x.size()), p, Ap;
	real rsold, alpha, rsnew = 0, normb = Norm(b);
	if (normb == 0.0) {
		normb = 1;
	}
	p = r = b - ShurProduct(x, A);
	rsold = Dot(r, r);
	normb = 1.0 / normb;
	if (sqrt(rsold) * normb <= tolerance) {
		return 0;
	}
	for (current_iteration = 0; current_iteration < max_iteration; current_iteration++) {
		Ap = ShurProduct(p, A);
		alpha = rsold / Dot(p, Ap);
		rsnew = 0;
#pragma omp parallel for reduction(+:rsnew)
		for (int i = 0; i < x.size(); i++) {
			x[i] = x[i] + alpha * p[i];
			r[i] = r[i] - alpha * Ap[i];
			rsnew += r[i] * r[i];
		}
		residual = sqrtf(rsnew) * normb;
		if (residual < tolerance) {
			break;
		}
		SEAXPY(rsnew / rsold, p, r, p); //p = r + rsnew / rsold * p;
		rsold = rsnew;
	}
	return current_iteration;
}
