#include "ChComputeRHSGPU.h"
using namespace chrono;

void ChComputeRHSGPU::Setup() {
	Initialize();
	time_rhs = 0;
	correction.resize(number_of_constraints);
	data_container->gpu_data.device_rhs_data.resize(number_of_constraints);
	bi.resize(number_of_constraints);
	Thrust_Fill(correction, 0);
	thrust::copy_n(data_container->gpu_data.device_dpth_data.begin(), data_container->number_of_contacts, correction.begin() + data_container->number_of_contacts * 0);
}

__host__ __device__ void function_bi(uint &index, uint & num_contacts, real &step_size, real *correction, real & recovery_speed, const real & alpha, real *bi) {
	bi[index + num_contacts * 0] = fmax(real(1.0) / step_size * correction[index], -recovery_speed);
	bi[index + num_contacts * 1] = 0;
	bi[index + num_contacts * 2] = 0;
}

void ChComputeRHSGPU::host_bi(real *correction, real * compliance, real* bi) {
#pragma omp parallel for
	for (uint index = 0; index < number_of_contacts; index++) {
		if (compliance[index]) {
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
//=================================================================================================================================
__host__ __device__ void function_RHS(
		uint &index, real &step_size, int2 *ids, real *bi, real & recovery_speed, bool* active, real3 *vel, real3 *omega, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, const real & alpha,
		real *rhs) {
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
	//rhs[index] = -((-temp) + fmaxf(inv_hpa * -fabs(correction[index]), 0));
	rhs[index] = -temp - bi[index]; //(temp + fmax(inv_hpa * correction[index], real(-recovery_speed)));

}

void ChComputeRHSGPU::host_RHS(int2 *ids, real *bi, bool * active, real3 *vel, real3 *omega, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *rhs) {
#pragma omp parallel for
	for (uint index = 0; index < number_of_contacts * 3; index++) {
		function_RHS(index, step_size, ids, bi, contact_recovery_speed, active, vel, omega, JXYZA, JXYZB, JUVWA, JUVWB, alpha, rhs);
	}

#pragma omp parallel for
	for (uint index = 0; index < number_of_bilaterals; index++) {
		uint b1 = ids[index + number_of_contacts * 3].x;
		uint b2 = ids[index + number_of_contacts * 3].y;
		real temp = 0;
		if (active[b1]) {
			temp += dot(JXYZA[index + number_of_contacts * 3], vel[b1]);
			temp += dot(JUVWA[index + number_of_contacts * 3], omega[b1]);
			real3 vA = vel[b1];
			vA = omega[b1];

		}
		if (active[b2]) {
			temp += dot(JXYZB[index + number_of_contacts * 3], vel[b2]);
			temp += dot(JUVWB[index + number_of_contacts * 3], omega[b2]);

			real3 vA = vel[b1];
			vA = omega[b1];

		}
		rhs[index + number_of_contacts * 3] = -temp - bi[index + number_of_contacts * 3]; //(temp + fmax(inv_hpa * correction[index], real(-recovery_speed)));
	}

}

void ChComputeRHSGPU::ComputeRHS(ChGPUDataManager *data_container_) {
	timer_rhs.start();
	data_container = data_container_;
	Setup();
	//Thrust_Fill(data_container->gpu_data.device_QXYZ_data, R3(0));
	//Thrust_Fill(data_container->gpu_data.device_QUVW_data, R3(0));

	data_container->gpu_data.device_comp_data.resize(number_of_contacts);
	for (int i = 0; i < number_of_contacts; i++) {
		uint b1 = data_container->gpu_data.device_bidlist_data[i].x;
		uint b2 = data_container->gpu_data.device_bidlist_data[i].y;
		real compb1 = data_container->gpu_data.device_compliance_data[b1];
		real compb2 = data_container->gpu_data.device_compliance_data[b2];

		real comp = (compb1 == 0 || compb2 == 0) ? 0 : (compb1 + compb2) * .5;
		data_container->gpu_data.device_comp_data[i] = comp;
	}
	thrust::copy_n(data_container->gpu_data.device_residual_bilateral.begin(), number_of_bilaterals, bi.begin() + number_of_contacts * 3);

	host_bi(correction.data(), data_container->gpu_data.device_comp_data.data(), bi.data());

	host_RHS(
			data_container->gpu_data.device_bidlist_data.data(),
			bi.data(),
			data_container->gpu_data.device_active_data.data(),
			data_container->gpu_data.device_vel_data.data(),
			data_container->gpu_data.device_omg_data.data(),
			data_container->gpu_data.device_JXYZA_data.data(),
			data_container->gpu_data.device_JXYZB_data.data(),
			data_container->gpu_data.device_JUVWA_data.data(),
			data_container->gpu_data.device_JUVWB_data.data(),
			data_container->gpu_data.device_rhs_data.data());

	timer_rhs.stop();
	time_rhs = timer_rhs();
}
