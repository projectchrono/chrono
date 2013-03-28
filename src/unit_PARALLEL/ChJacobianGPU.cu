#include "ChJacobianGPU.h"
using namespace chrono;
__constant__ uint number_of_contacts_const;
__host__ __device__ void inline Compute_Jacobian(
		const real4& quaternion_rotation,
		const real3& normal,
		const real3& tangent_u,
		const real3& tangent_w,
		const real3& point,
		real3& T3,
		real3& T4,
		real3& T5) {
	real t1 = quaternion_rotation.x * quaternion_rotation.y;
	real t2 = quaternion_rotation.w * quaternion_rotation.z;
	real t3 = 2 * t1 - 2 * t2;
	real t4 = quaternion_rotation.x * quaternion_rotation.x;
	real t5 = quaternion_rotation.y * quaternion_rotation.y;
	real t6 = quaternion_rotation.w * quaternion_rotation.w;
	real t7 = quaternion_rotation.z * quaternion_rotation.z;
	real t8 = t6 - t4 + t5 - t7;
	real t9 = quaternion_rotation.y * quaternion_rotation.z;
	real t10 = quaternion_rotation.w * quaternion_rotation.x;
	real t11 = 2 * t9 + 2 * t10;
	real t12 = normal.x * t3 + normal.y * t8 + normal.z * t11;
	real t13 = quaternion_rotation.x * quaternion_rotation.z;
	real t14 = quaternion_rotation.w * quaternion_rotation.y;
	real t15 = 2 * t13 + 2 * t14;
	t9 = 2 * t9 - 2 * t10;
	t10 = t6 - t4 - t5 + t7;
	real t16 = t15 * point.x + t9 * point.y + t10 * point.z;
	real t17 = normal.x * t15 + normal.y * t9 + normal.z * t10;
	real t18 = -t3 * point.x - t8 * point.y - t11 * point.z;
	t4 = t6 + t4 - t5 - t7;
	t1 = 2 * t1 + 2 * t2;
	t2 = 2 * t13 - 2 * t14;
	t5 = normal.x * t4 + normal.y * t1 + normal.z * t2;
	t6 = -t16;
	t7 = t4 * point.x + t1 * point.y + t2 * point.z;
	t13 = -t18;
	t14 = -t7;
	real t19 = tangent_u.x * t3 + tangent_u.y * t8 + tangent_u.z * t11;
	real t20 = tangent_u.x * t15 + tangent_u.y * t9 + tangent_u.z * t10;
	real t21 = tangent_u.x * t4 + tangent_u.y * t1 + tangent_u.z * t2;
	t3 = tangent_w.x * t3 + tangent_w.y * t8 + tangent_w.z * t11;
	t8 = tangent_w.x * t15 + tangent_w.y * t9 + tangent_w.z * t10;
	t1 = tangent_w.x * t4 + tangent_w.y * t1 + tangent_w.z * t2;
	T3.x = t12 * t16 + t17 * t18;
	T3.y = t5 * t6 + t17 * t7;
	T3.z = t5 * t13 + t12 * t14;
	T4.x = t19 * t16 + t20 * t18;
	T4.y = t21 * t6 + t20 * t7;
	T4.z = t21 * t13 + t19 * t14;
	T5.x = t3 * t16 + t8 * t18;
	T5.y = t1 * t6 + t8 * t7;
	T5.z = t1 * t13 + t3 * t14;
}
__host__ __device__ void inline function_ContactJacobians(
		uint& index,
		uint& num_contacts,
		real3* norm,
		real3* ptA,
		real3* ptB,
		real3* pos,
		real4* rot,
		int2* ids,
		real3* JXYZA,
		real3* JXYZB,
		real3* JUVWA,
		real3* JUVWB) {
	real3 U = norm[index];
	real3 W = fabs(U);
	real3 V = R3(0, U.z, -U.y);

	if (W.x > W.y) {
		V = R3(-U.z, 0, U.x);
	}

	if (W.y > W.z) {
		V = R3(U.y, -U.x, 0);
	}

	V = normalize(V);
	W = cross(U, V);

	JXYZA[index + num_contacts * 0] = -U;
	JXYZA[index + num_contacts * 1] = -V;
	JXYZA[index + num_contacts * 2] = -W;

	JXYZB[index + num_contacts * 0] = U;
	JXYZB[index + num_contacts * 1] = V;
	JXYZB[index + num_contacts * 2] = W;

	int2 body_id = ids[index];
	real3 T3, T4, T5, T6, T7, T8;

	real3 sbar = ptA[index] - pos[body_id.x];
	real4 E1 = rot[body_id.x];
	Compute_Jacobian(E1, U, V, W, sbar, T3, T4, T5);

	sbar = ptB[index] - pos[body_id.y];
	real4 E2 = rot[body_id.y];
	Compute_Jacobian(E2, U, V, W, sbar, T6, T7, T8);
	T6 = -T6;
	T7 = -T7;
	T8 = -T8;

	JUVWA[index + num_contacts * 0] = T3;
	JUVWA[index + num_contacts * 1] = T4;
	JUVWA[index + num_contacts * 2] = T5;

	JUVWB[index + num_contacts * 0] = T6;
	JUVWB[index + num_contacts * 1] = T7;
	JUVWB[index + num_contacts * 2] = T8;

//    std::cout << "[" << N.x << " " << N.y << " " << N.z << "]" << "[" << -N.x << " " << -N.y << " " << -N.z << "]" << std::endl;
//    std::cout << "[" << U.x << " " << U.y << " " << U.z << "]" << "[" << -U.x << " " << -U.y << " " << -U.z << "]" << std::endl;
//    std::cout << "[" << W.x << " " << W.y << " " << W.z << "]" << "[" << -W.x << " " << -W.y << " " << -W.z << "]" << std::endl;
//    std::cout << "------" << std::endl;
//    std::cout << "[" << T3.x << " " << T3.y << " " << T3.z << "]" << "[" << T6.x << " " << T6.y << " " << T6.z << "]" << std::endl;
//    std::cout << "[" << T4.x << " " << T4.y << " " << T4.z << "]" << "[" << T7.x << " " << T7.y << " " << T7.z << "]" << std::endl;
//    std::cout << "[" << T5.x << " " << T5.y << " " << T5.z << "]" << "[" << T8.x << " " << T8.y << " " << T8.z << "]" << std::endl;
//    std::cout << "****************" << std::endl;
}

__global__ void device_ContactJacobians(real3* norm, real3* ptA, real3* ptB, int2* ids, real4* rot, real3* pos, real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_contacts_const);
	function_ContactJacobians(index, number_of_contacts_const, norm, ptA, ptB, pos, rot, ids, JXYZA, JXYZB, JUVWA, JUVWB);
}
void ChJacobianGPU::host_ContactJacobians(real3* norm, real3* ptA, real3* ptB, int2* ids, real4* rot, real3* pos, real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB) {
	// #pragma omp parallel for schedule(guided)
	for (uint index = 0; index < number_of_contacts; index++) {
		function_ContactJacobians(index, number_of_contacts, norm, ptA, ptB, pos, rot, ids, JXYZA, JXYZB, JUVWA, JUVWB);
	}
}

void ChJacobianGPU::Setup() {
	number_of_constraints = gpu_data->number_of_contacts * 3 + gpu_data->number_of_bilaterals;
	number_of_contacts = gpu_data->number_of_contacts;
	number_of_bilaterals = gpu_data->number_of_bilaterals;
	number_of_objects = gpu_data->number_of_objects;

	gpu_data->device_JXYZA_data.resize(number_of_constraints);
	gpu_data->device_JXYZB_data.resize(number_of_constraints);
	gpu_data->device_JUVWA_data.resize(number_of_constraints);
	gpu_data->device_JUVWB_data.resize(number_of_constraints);
#ifdef SIM_ENABLE_GPU_MODE
	COPY_TO_CONST_MEM(number_of_contacts);
#endif

}

void ChJacobianGPU::ComputeJacobians(gpu_container& gpu_data_) {
	gpu_data = &gpu_data_;

	Setup();

#ifdef SIM_ENABLE_GPU_MODE
	device_ContactJacobians CUDA_KERNEL_DIM(BLOCKS(number_of_contacts), THREADS)(
			CASTR3(gpu_data->device_norm_data),
			CASTR3(gpu_data->device_cpta_data),
			CASTR3(gpu_data->device_cptb_data),
			CASTI2(gpu_data->device_bids_data),
			CASTR4(gpu_data->device_rot_data),
			CASTR3(gpu_data->device_pos_data),
			CASTR3(gpu_data->device_JXYZA_data),
			CASTR3(gpu_data->device_JXYZB_data),
			CASTR3(gpu_data->device_JUVWA_data),
			CASTR3(gpu_data->device_JUVWB_data));

#else
	host_ContactJacobians(
			gpu_data->device_norm_data.data(),
			gpu_data->device_cpta_data.data(),
			gpu_data->device_cptb_data.data(),
			gpu_data->device_bids_data.data(),
			gpu_data->device_rot_data.data(),
			gpu_data->device_pos_data.data(),
			gpu_data->device_JXYZA_data.data(),
			gpu_data->device_JXYZB_data.data(),
			gpu_data->device_JUVWA_data.data(),
			gpu_data->device_JUVWB_data.data()
	);
#endif

}
