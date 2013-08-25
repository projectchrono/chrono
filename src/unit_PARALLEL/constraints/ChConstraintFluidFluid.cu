#include "ChConstraintFluidFluid.h"
using namespace chrono;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintFluidFluid::Project(custom_vector<real> & gamma) {
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void ChConstraintFluidFluid::host_RHS(int2 *ids, real3 *pos, real3 *vel, real3 *JXYZA, real3 *JXYZB, real* compliance, real *rhs) {

#pragma omp parallel for
	for (uint index = 0; index < number_of_fluid_fluid; index++) {
		uint b1 = ids[index].x;
		uint b2 = ids[index].y;

		real temp = 0;
		temp += dot(JXYZA[index], vel[b1]);
		temp += dot(JXYZB[index], vel[b2]);

		real3 posA = pos[b1];
		real3 posB = pos[b2];

		real3 relPos = posA - posB;
		real dist = length(relPos);
		real bi = 0;

		if (compliance[index]) {
			bi = inv_hpa * -dist;
		} else {
			bi = fmax(real(1.0) / step_size * dist, -contact_recovery_speed);
		}
		rhs[index + number_of_rigid_rigid * 3 + number_of_rigid_fluid] = -temp - bi;
	}

}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintFluidFluid::ComputeRHS() {

	host_RHS(
			data_container->device_data.device_bids_fluid_fluid.data(),
			data_container->device_data.device_pos_fluid.data(),
			data_container->device_data.device_vel_fluid.data(),
			device_JXYZA_fluid_fluid.data(),
			device_JXYZB_fluid_fluid.data(),
			device_comp_fluid_fluid.data(),
			data_container->device_data.device_rhs_data.data());

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintFluidFluid::host_Jacobians(int2* ids, real3* pos, real3* JXYZA, real3* JXYZB) {

	for (uint index = 0; index < number_of_fluid_fluid; index++) {

		int2 body_id = ids[index];

		real3 posA = pos[body_id.x];
		real3 posB = pos[body_id.y];

		real3 relPos = posA - posB;
		real dist = length(relPos);
		real3 N = relPos / dist;

		JXYZA[index] = -N;
		JXYZB[index] = N;

	}

}
void ChConstraintFluidFluid::ComputeJacobians() {
	device_JXYZA_fluid_fluid.resize(number_of_fluid_fluid);
	device_JXYZB_fluid_fluid.resize(number_of_fluid_fluid);
	device_comp_fluid_fluid.resize(number_of_fluid_fluid);
	host_Jacobians(
			data_container->device_data.device_bids_fluid_fluid.data(),
			data_container->device_data.device_pos_fluid.data(),
			device_JXYZA_fluid_fluid.data(),
			device_JXYZB_fluid_fluid.data());
#pragma omp parallel for
	for (int i = 0; i < number_of_fluid_fluid; i++) {
		real comp = 0;
		device_comp_fluid_fluid[i] = inv_hhpa * comp;
	}
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintFluidFluid::host_shurA(int2 *ids, real *inv_mass, real3 *JXYZA, real3 *JXYZB, real *gamma, real3* QXYZ) {

	for (uint index = 0; index < number_of_fluid_fluid; index++) {
		real gam = gamma[index];

		uint b1 = ids[index].x;
		uint b2 = ids[index].y;

		QXYZ[b1] += JXYZA[index] * gam * inv_mass[b1];
		QXYZ[b2] += JXYZB[index] * gam * inv_mass[b2];
	}
}

void ChConstraintFluidFluid::host_shurB(
		int2 *ids,
		real *inv_mass_fluid,
		real * gamma,
		real * compliance,
		real3 *JXYZA,
		real3 *JXYZB,
		real3 *QXYZ,
		real *AX) {

	for (uint index = 0; index < number_of_fluid_fluid; index++) {
		uint b1 = ids[index].x;
		uint b2 = ids[index].y;
		real temp = 0;

		temp += dot(QXYZ[b1], JXYZA[index]);
		temp += dot(QXYZ[b2], JXYZB[index]);

		AX[index] += temp + gamma[index] * compliance[index];
	}

}
void ChConstraintFluidFluid::ShurA(custom_vector<real> &x) {

	gamma.resize(number_of_fluid_fluid);
	thrust::copy_n(
			x.begin()+number_of_rigid_rigid*3+number_of_rigid_fluid,
			number_of_fluid_fluid,
			gamma.begin());

	host_shurA(
			data_container->device_data.device_bids_fluid_fluid.data(),
			data_container->device_data.device_mass_fluid.data(),
			device_JXYZA_fluid_fluid.data(),
			device_JXYZB_fluid_fluid.data(),
			gamma.data(),
			data_container->device_data.device_QXYZ_fluid.data());

}
void ChConstraintFluidFluid::ShurB(custom_vector<real> &x, custom_vector<real> & output) {
	gamma.resize(number_of_fluid_fluid);
	ax.resize(number_of_fluid_fluid);
	thrust::copy_n(
			x.begin()+number_of_rigid_rigid*3+number_of_rigid_fluid ,
			number_of_fluid_fluid,
			gamma.begin());
	host_shurB(
			data_container->device_data.device_bids_fluid_fluid.data(),
			data_container->device_data.device_mass_fluid.data(),
			gamma.data(),
			device_comp_fluid_fluid.data(),
			device_JXYZA_fluid_fluid.data(),
			device_JXYZB_fluid_fluid.data(),
			data_container->device_data.device_QXYZ_fluid.data(),
			ax.data());
	thrust::copy_n(
			ax.begin() ,
			number_of_fluid_fluid,
			output.begin()+number_of_rigid_rigid*3+number_of_rigid_fluid);
}
