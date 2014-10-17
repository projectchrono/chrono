#include "chrono_parallel/constraints/ChConstraintRigidFluid.h"
using namespace chrono;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void func_Project_rigid_fluid(uint &index, uint number_of_contacts, int2 *ids, real *gam) {
	int2 body_id = ids[index];

}

void ChConstraintRigidFluid::host_Project(int2 *ids, real *gam) {
	for (int index = 0; index < number_of_rigid_fluid; index++) {
		real3 gamma;
		gamma.x = gam[index + number_of_rigid_rigid * 3];
		gamma.x = gamma.x < 0 ? 0 : gamma.x;
		gamma.y = gamma.z = 0;
		gam[index + number_of_rigid_rigid * 3] = gamma.x;
	}
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidFluid::Project(custom_vector<real> & gamma) {
	host_Project(
			data_container->gpu_data.device_bids_rigid_fluid.data(),
			gamma.data());
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void ChConstraintRigidFluid::host_RHS(
		int2 *ids,
		real *correction,
		bool * active,
		real3 *vel,
		real3 *omega,
		real3 *vel_fluid,
		real3 *JXYZA,
		real3 *JXYZB,
		real3 *JUVWB,
		real *rhs) {

//#pragma omp parallel for
	for (int index = 0; index < number_of_rigid_fluid; index++) {
		uint b1 = ids[index].x;
		uint b2 = ids[index].y;
		real temp = 0;
//cout<<b1<<" "<<b2<<endl;
		temp += dot(JXYZA[index], vel_fluid[b1]);

		if (active[b2]) {
			temp += dot(JXYZB[index], vel[b2]) + dot(JUVWB[index], omega[b2]);

		}
		real bi = fmax(real(1.0) / step_size * correction[index], -contact_recovery_speed);

		//rhs[index] = -((-temp) + fmaxf(inv_hpa * -fabs(correction[index]), 0));
		rhs[index + number_of_rigid_rigid * 3] = -temp - bi; //(temp + fmax(inv_hpa * correction[index], real(-recovery_speed)));

		//cout<<correction[index]<<endl;

	}

}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidFluid::ComputeRHS() {

	host_RHS(
			data_container->gpu_data.device_bids_rigid_fluid.data(),
			data_container->gpu_data.device_dpth_rigid_fluid.data(),
			data_container->gpu_data.device_active_data.data(),
			data_container->gpu_data.device_vel_data.data(),
			data_container->gpu_data.device_omg_data.data(),
			data_container->gpu_data.device_vel_fluid.data(),
			device_JXYZA_rigid_fluid.data(),
			device_JXYZB_rigid_fluid.data(),
			device_JUVWB_rigid_fluid.data(),
			data_container->gpu_data.device_rhs_data.data());


}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

__host__ __device__ void inline Compute_Jacobian(const real4& quaternion_rotation, const real3& normal, const real3& point, real3& T) {
	real3 Pl = MatTMult(AMat(quaternion_rotation), point);
	T = cross(Pl, normal);
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidFluid::host_Jacobians(
		real3* norm,
		real3* ptA,
		real3* ptB,
		int2* ids,
		real4* rot,
		real3* pos,
		real3* JXYZA,
		real3* JXYZB,
		real3* JUVWB) {
#pragma omp parallel for
	for (int index = 0; index < number_of_rigid_fluid; index++) {
		real3 U = norm[index];

		if (U == R3(0, 0, 0)) {
			U = R3(1, 0, 0);
		} else {
			U = normalize(U);
		}
		U = normalize(U);

		JXYZA[index] = -U;
		JXYZB[index] = U;

		int2 body_id = ids[index];

		real3 T;

		real3 sbar = ptB[index] - pos[body_id.y];
		real4 E2 = rot[body_id.y];
		Compute_Jacobian(E2, U, sbar, T);
		T = -T;

		JUVWB[index] = T;
	}
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidFluid::ComputeJacobians() {

	device_JXYZA_rigid_fluid.resize(number_of_rigid_fluid);
	device_JXYZB_rigid_fluid.resize(number_of_rigid_fluid);
	device_JUVWB_rigid_fluid.resize(number_of_rigid_fluid);

	host_Jacobians(
			data_container->gpu_data.device_norm_rigid_fluid.data(),
			data_container->gpu_data.device_cpta_rigid_fluid.data(),
			data_container->gpu_data.device_cptb_rigid_fluid.data(),
			data_container->gpu_data.device_bids_rigid_fluid.data(),
			data_container->gpu_data.device_rot_data.data(),
			data_container->gpu_data.device_pos_data.data(),
			device_JXYZA_rigid_fluid.data(),
			device_JXYZB_rigid_fluid.data(),
			device_JUVWB_rigid_fluid.data());

	device_comp_rigid_fluid.resize(number_of_rigid_fluid);

	for (int i = 0; i < number_of_rigid_fluid; i++) {
		device_comp_rigid_fluid[i] = inv_hhpa * 1e-3;
	}

}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidFluid::host_shurA(
		int2 *ids,
		bool *active,
		real *inv_mass,
		real3 *inv_inertia,
		real *inv_mass_fluid,
		real3 *JXYZA,
		real3 *JXYZB,
		real3 *JUVWB,
		real *gamma,
		real3* QXYZ,
		real3* QUVW,
		real3* QXYZ_fluid) {
//#pragma omp parallel for
	for (int index = 0; index < number_of_rigid_fluid; index++) {
		real gam;
		gam = gamma[index + number_of_rigid_rigid * 3];
		uint b1 = ids[index].x;
		QXYZ_fluid[b1] += JXYZA[index] * gam * inv_mass_fluid[b1];

		uint b2 = ids[index].y;
		if (active[b2] != 0) {
			QXYZ[b2] += JXYZB[index] * gam * inv_mass[b2];
			QUVW[b2] += JUVWB[index] * gam * inv_inertia[b2];
		}
	}
}

void ChConstraintRigidFluid::host_shurB(
		int2 *ids,
		bool *active,
		real *inv_mass,
		real3 *inv_inertia,
		real *inv_mass_fluid,
		real * compliance,
		real * gamma,
		real3 *JXYZA,
		real3 *JXYZB,
		real3 *JUVWB,
		real3 *QXYZ,
		real3 *QUVW,
		real3* QXYZ_fluid,
		real *AX) {

//#pragma omp parallel for
	for (int index = 0; index < number_of_rigid_fluid; index++) {
		real temp = 0;
		int2 id_ = ids[index];
		uint b1 = id_.x;
		uint b2 = id_.y;

		real3 XYZ = QXYZ_fluid[b1];
		temp += dot(XYZ, JXYZA[index]);

		if (active[b2] != 0) {
			real3 XYZ = QXYZ[b2];
			real3 UVW = QUVW[b2];

			temp += dot(XYZ, JXYZB[index]);
			temp += dot(UVW, JUVWB[index]);
		}
		AX[index + number_of_rigid_rigid * 3] = temp; //+ gamma[index+ number_of_rigid_rigid * 3] * inv_hhpa*1e-6;

	}

}
void ChConstraintRigidFluid::ShurA(custom_vector<real> &x) {
		host_shurA(
				data_container->gpu_data.device_bids_rigid_fluid.data(),
				data_container->gpu_data.device_active_data.data(),
				data_container->gpu_data.device_mass_data.data(),
				data_container->gpu_data.device_inr_data.data(),
				data_container->gpu_data.device_mass_fluid.data(),
				device_JXYZA_rigid_fluid.data(),
				device_JXYZB_rigid_fluid.data(),
				device_JUVWB_rigid_fluid.data(),
				x.data(),
				data_container->gpu_data.device_QXYZ_data.data(),
				data_container->gpu_data.device_QUVW_data.data(),
				data_container->gpu_data.device_QXYZ_fluid.data());

	}
void ChConstraintRigidFluid::ShurB(custom_vector<real> &x, custom_vector<real> & output) {
		host_shurB(
				data_container->gpu_data.device_bids_rigid_fluid.data(),
				data_container->gpu_data.device_active_data.data(),
				data_container->gpu_data.device_mass_data.data(),
				data_container->gpu_data.device_inr_data.data(),
				data_container->gpu_data.device_mass_fluid.data(),
				device_comp_rigid_fluid.data(),
				x.data(),
				device_JXYZA_rigid_fluid.data(),
				device_JXYZB_rigid_fluid.data(),
				device_JUVWB_rigid_fluid.data(),
				data_container->gpu_data.device_QXYZ_data.data(),
				data_container->gpu_data.device_QUVW_data.data(),
				data_container->gpu_data.device_QXYZ_fluid.data(),
				output.data());

	}
