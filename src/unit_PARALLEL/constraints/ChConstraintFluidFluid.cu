#include "ChConstraintFluidFluid.h"
using namespace chrono;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

real W_Poly6(real r, real h) {

	return 315.0 / (64.0 * PI * pow(h, 9)) * pow((h * h - r * r), 3);
}

void ChConstraintFluidFluid::host_Project(int2 *ids, real *gam) {
#pragma omp parallel for
	for (uint index = 0; index < number_of_fluid_fluid; index++) {
		real gamma = gam[index + number_of_rigid_rigid * 3 + number_of_rigid_fluid];
		gamma = gamma < 0 ? 0 : gamma;
		gam[index + number_of_rigid_rigid * 3 + number_of_rigid_fluid] = gamma;
	}
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintFluidFluid::Project(custom_vector<real> & gamma) {
	host_Project(
			data_container->gpu_data.device_bids_fluid_fluid.data(),
			gamma.data());
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void ChConstraintFluidFluid::host_RHS(int2 *ids, real3 *position, real *rad, real * compliance, real3 *vel, real3 *JXYZA, real3 *JXYZB, real *rhs) {

#pragma omp parallel for
	for (uint index = 0; index < number_of_fluid_fluid; index++) {
		uint b1 = ids[index].x;
		uint b2 = ids[index].y;
		real3 temp = R3(0);

		temp.x += dot(JXYZA[index], vel[b1]);
		temp.x += dot(JXYZB[index], vel[b2]);

		real3 pos_a = position[b1];
		real3 pos_b = position[b2];
		real totalrad = rad[b1] + rad[b2];
		real correction = length(pos_b - pos_a) - totalrad;
		real bi = 0;
		if (compliance[index]) {
			bi = inv_hpa * correction;
		} else {
			bi = fmax(real(1.0) / step_size * -correction, -contact_recovery_speed);
		}
		rhs[index + number_of_rigid_rigid * 3 + number_of_rigid_fluid] =  -temp.x - bi;
	}
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintFluidFluid::ComputeRHS() {

	host_RHS(
			data_container->gpu_data.device_bids_fluid_fluid.data(),
			data_container->gpu_data.device_pos_fluid.data(),
			data_container->gpu_data.device_rad_fluid.data(),
			device_comp_fluid_fluid.data(),
			data_container->gpu_data.device_vel_fluid.data(),
			device_JXYZA_fluid_fluid.data(),
			device_JXYZB_fluid_fluid.data(),
			data_container->gpu_data.device_rhs_data.data());

}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintFluidFluid::host_Jacobians(int2* ids, real3* pos, real3* JXYZA, real3* JXYZB) {
#pragma omp parallel for
	for (uint index = 0; index < number_of_fluid_fluid; index++) {
		uint b1 = ids[index].x;
		uint b2 = ids[index].y;
		real3 pos_a = pos[b1];
		real3 pos_b = pos[b2];

		real3 relPos = (pos_b - pos_a);
		real3 U = relPos / length(relPos);

		JXYZA[index] = R3(0);  -U;
		JXYZB[index] = R3(0); U;

	}
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintFluidFluid::ComputeJacobians() {

	device_JXYZA_fluid_fluid.resize(number_of_fluid_fluid);
	device_JXYZB_fluid_fluid.resize(number_of_fluid_fluid);

	host_Jacobians(
			data_container->gpu_data.device_bids_fluid_fluid.data(),
			data_container->gpu_data.device_pos_fluid.data(),
			device_JXYZA_fluid_fluid.data(),
			device_JXYZB_fluid_fluid.data());

	device_comp_fluid_fluid.resize(number_of_fluid_fluid);
#pragma omp parallel for
	for (int i = 0; i < number_of_fluid_fluid; i++) {
		real comp = 1e-3;
		device_comp_fluid_fluid[i] = inv_hhpa * comp;
	}

}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintFluidFluid::host_shurA(int2 *ids, real *inv_mass, real3 *JXYZA, real3 *JXYZB, real *gamma, real3* QXYZ) {

	for (int index = 0; index < number_of_fluid_fluid; index++) {
		real3 gam;
		gam.x = gamma[index + number_of_rigid_rigid * 3 + number_of_rigid_fluid];

		uint b1 = ids[index].x;
		uint b2 = ids[index].y;
		QXYZ[b1] += JXYZA[index] * gam.x * inv_mass[b1];
		QXYZ[b2] += JXYZB[index] * gam.x * inv_mass[b2];
	}
}

void ChConstraintFluidFluid::host_shurB(
		int2 *ids,
		real *inv_mass,
		real * compliance,
		real * gamma,
		real3 *JXYZA,
		real3 *JXYZB,
		real3 *QXYZ,
		real *AX) {

#pragma omp parallel for
	for (uint index = 0; index < number_of_fluid_fluid; index++) {
		real3 temp = R3(0);
		int2 id_ = ids[index];
		uint b1 = id_.x;
		uint b2 = id_.y;

		temp.x += dot(QXYZ[b1], JXYZA[index]);
		temp.x += dot(QXYZ[b2], JXYZB[index]);

		AX[index + number_of_rigid_rigid * 3 + number_of_rigid_fluid] = temp.x
				+ gamma[index + number_of_rigid_rigid * 3 + number_of_rigid_fluid] * compliance[index];

	}

}
void ChConstraintFluidFluid::ShurA(custom_vector<real> &x) {

	host_shurA(
			data_container->gpu_data.device_bids_fluid_fluid.data(),
			data_container->gpu_data.device_mass_fluid.data(),
			device_JXYZA_fluid_fluid.data(),
			device_JXYZB_fluid_fluid.data(),
			x.data(),
			data_container->gpu_data.device_QXYZ_fluid.data());

}
void ChConstraintFluidFluid::ShurB(custom_vector<real> &x, custom_vector<real> & output) {

	host_shurB(
			data_container->gpu_data.device_bids_fluid_fluid.data(),
			data_container->gpu_data.device_mass_fluid.data(),
			device_comp_fluid_fluid.data(),
			x.data(),
			device_JXYZA_fluid_fluid.data(),
			device_JXYZB_fluid_fluid.data(),
			data_container->gpu_data.device_QXYZ_fluid.data(),
			output.data());
}
