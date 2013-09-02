#include "ChComputeRHSGPU.h"
using namespace chrono;

void ChComputeRHSGPU::Setup() {
	Initialize();
	time_rhs = 0;
	correction.resize(number_of_rigid_rigid);
	data_container->host_data.rhs_data.resize(number_of_constraints);
	bi.resize(number_of_rigid_rigid);
	Thrust_Fill(correction, 0);
	thrust::copy_n(
			data_container->host_data.dpth_rigid_rigid.begin(),
			data_container->number_of_rigid_rigid,
			correction.begin() + data_container->number_of_rigid_rigid * 0);
}

__host__ __device__ void function_bi(
		uint &index,
		uint & num_contacts,
		real &step_size,
		real *correction,
		real & recovery_speed,
		const real & alpha,
		real *bi) {
	bi[index + num_contacts * 0] = fmax(real(1.0) / step_size * correction[index], -recovery_speed);
}

void ChComputeRHSGPU::host_bi(real *correction, real * compliance, real* bi) {
#pragma omp parallel for
	for (uint index = 0; index < number_of_rigid_rigid; index++) {
		if (compliance[index]) {
			bi[index + number_of_rigid_rigid * 0] = inv_hpa * correction[index];
		} else {
			bi[index + number_of_rigid_rigid * 0] = fmax(real(1.0) / step_size * correction[index], -contact_recovery_speed);
		}

		//function_bi(index, number_of_contacts, step_size, correction, contact_recovery_speed, alpha, bi);
	}
}
//=================================================================================================================================
__host__ __device__ void function_RHS(
		uint &index,
		uint number_of_contacts,
		real &step_size,
		int2 *ids,
		real *bi,
		real & recovery_speed,
		bool* active,
		real3 *vel,
		real3 *omega,
		real3 *JXYZA,
		real3 *JXYZB,
		real3 *JUVWA,
		real3 *JUVWB,
		const real & alpha,
		real *rhs) {
	uint b1 = ids[index].x;
	uint b2 = ids[index].y;
	real3 temp = R3(0);
	if (active[b1]) {
		temp.x += dot(JXYZA[index + number_of_contacts * 0], vel[b1]) + dot(JUVWA[index + number_of_contacts * 0], omega[b1]);
		temp.y += dot(JXYZA[index + number_of_contacts * 1], vel[b1]) + dot(JUVWA[index + number_of_contacts * 1], omega[b1]);
		temp.z += dot(JXYZA[index + number_of_contacts * 2], vel[b1]) + dot(JUVWA[index + number_of_contacts * 2], omega[b1]);
	}
	if (active[b2]) {
		temp.x += dot(JXYZB[index + number_of_contacts * 0], vel[b2]) + dot(JUVWB[index + number_of_contacts * 0], omega[b2]);
		temp.y += dot(JXYZB[index + number_of_contacts * 1], vel[b2]) + dot(JUVWB[index + number_of_contacts * 1], omega[b2]);
		temp.z += dot(JXYZB[index + number_of_contacts * 2], vel[b1]) + dot(JUVWB[index + number_of_contacts * 2], omega[b2]);
	}
	//rhs[index] = -((-temp) + fmaxf(inv_hpa * -fabs(correction[index]), 0));
	rhs[index + number_of_contacts * 0] = -temp.x - bi[index];     //(temp + fmax(inv_hpa * correction[index], real(-recovery_speed)));
	rhs[index + number_of_contacts * 1] = -temp.y;
	rhs[index + number_of_contacts * 2] = -temp.z;

}

void ChComputeRHSGPU::host_RHS_contacts(
		int2 *ids,
		real *bi,
		bool * active,
		real3 *vel,
		real3 *omega,
		real3 *JXYZA,
		real3 *JXYZB,
		real3 *JUVWA,
		real3 *JUVWB,
		real *rhs) {
#pragma omp parallel for
	for (uint index = 0; index < number_of_rigid_rigid; index++) {
		function_RHS(
				index,
				number_of_rigid_rigid,
				step_size,
				ids,
				bi,
				contact_recovery_speed,
				active,
				vel,
				omega,
				JXYZA,
				JXYZB,
				JUVWA,
				JUVWB,
				alpha,
				rhs);
	}

}
void ChComputeRHSGPU::host_RHS_bilaterals(
		int2 *ids,
		real *bi,
		bool * active,
		real3 *vel,
		real3 *omega,
		real3 *JXYZA,
		real3 *JXYZB,
		real3 *JUVWA,
		real3 *JUVWB,
		real *rhs) {

#pragma omp parallel for
	for (uint index = 0; index < number_of_bilaterals; index++) {
		uint b1 = ids[index].x;
		uint b2 = ids[index].y;
		real temp = 0;
		if (active[b1]) {
			temp += dot(JXYZA[index], vel[b1]);
			temp += dot(JUVWA[index], omega[b1]);
			real3 vA = vel[b1];
			vA = omega[b1];

		}
		if (active[b2]) {
			temp += dot(JXYZB[index], vel[b2]);
			temp += dot(JUVWB[index], omega[b2]);

			real3 vA = vel[b1];
			vA = omega[b1];

		}
		rhs[index + number_of_rigid_rigid * 3] = -temp - bi[index];     //(temp + fmax(inv_hpa * correction[index], real(-recovery_speed)));
	}

}

void ChComputeRHSGPU::ComputeRHS(ChGPUDataManager *data_container_) {
	timer_rhs.start();
	data_container = data_container_;
	Setup();
	//Thrust_Fill(data_container->gpu_data.device_QXYZ_data, R3(0));
	//Thrust_Fill(data_container->gpu_data.device_QUVW_data, R3(0));

	data_container->host_data.compliance_rigid_rigid.resize(number_of_rigid_rigid);
	for (int i = 0; i < number_of_rigid_rigid; i++) {
		uint b1 = data_container->host_data.bids_rigid_rigid[i].x;
		uint b2 = data_container->host_data.bids_rigid_rigid[i].y;
		real compb1 = data_container->host_data.compliance_data[b1];
		real compb2 = data_container->host_data.compliance_data[b2];

		real comp = (compb1 == 0 || compb2 == 0) ? 0 : (compb1 + compb2) * .5;
		data_container->host_data.compliance_rigid_rigid[i] = comp;
	}

	host_bi(correction.data(), data_container->host_data.compliance_rigid_rigid.data(), bi.data());

	host_RHS_contacts(
			data_container->host_data.bids_rigid_rigid.data(),
			bi.data(),
			data_container->host_data.active_data.data(),
			data_container->host_data.vel_data.data(),
			data_container->host_data.omg_data.data(),
			data_container->host_data.JXYZA_data.data(),
			data_container->host_data.JXYZB_data.data(),
			data_container->host_data.JUVWA_data.data(),
			data_container->host_data.JUVWB_data.data(),
			data_container->host_data.rhs_data.data());
	host_RHS_bilaterals(
			data_container->host_data.bids_bilateral.data(),
			data_container->host_data.residual_bilateral.data(),
			data_container->host_data.active_data.data(),
			data_container->host_data.vel_data.data(),
			data_container->host_data.omg_data.data(),
			data_container->host_data.JXYZA_bilateral.data(),
			data_container->host_data.JXYZB_bilateral.data(),
			data_container->host_data.JUVWA_bilateral.data(),
			data_container->host_data.JUVWB_bilateral.data(),
			data_container->host_data.rhs_data.data());
	timer_rhs.stop();
	time_rhs = timer_rhs();
}
