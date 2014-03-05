#include "ChConstraintBilateral.h"
using namespace chrono;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintBilateral::Project(custom_vector<real> & gamma) {

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void ChConstraintBilateral::host_RHS(int2 *ids, real *bi, bool * active, real3 *vel, real3 *omega, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *rhs) {
#pragma omp parallel for
	for (int index = 0; index < number_of_bilaterals; index++) {
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
		rhs[index + number_of_rigid_rigid * 6] = -temp - bi[index];     //(temp + fmax(inv_hpa * correction[index], real(-recovery_speed)));
	}

}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintBilateral::ComputeRHS() {
	host_RHS(
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
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintBilateral::ComputeJacobians() {

}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintBilateral::host_Offsets(int2* ids, uint* Body) {
#pragma omp parallel for
	for (int index = 0; index < number_of_bilaterals; index++) {
		if (index < number_of_bilaterals) {
			int2 temp_id = ids[index];
			Body[index] = temp_id.x;
			Body[index + number_of_bilaterals] = temp_id.y;
		}
	}
}

__host__ __device__ void function_Reduce_Shur_Bilateral(int& index,bool* active, real3* QXYZ, real3* QUVW, real *inv_mass, real3 *inv_inertia, real3* updateQXYZ, real3* updateQUVW, uint* d_body_num, uint* counter) {
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
	QXYZ[id] += mUpdateV * inv_mass[id];
	QUVW[id] += mUpdateO * inv_inertia[id];
}

void ChConstraintBilateral::host_Reduce_Shur(bool* active, real3* QXYZ, real3* QUVW, real *inv_mass, real3 *inv_inertia, real3* updateQXYZ, real3* updateQUVW, uint* d_body_num, uint* counter) {
#pragma omp  for
	for (int index = 0; index < number_of_updates; index++) {
		function_Reduce_Shur_Bilateral(index, active, QXYZ, QUVW, inv_mass, inv_inertia, updateQXYZ, updateQUVW, d_body_num, counter);
	}
}

void ChConstraintBilateral::host_shurA(
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
#pragma omp  for
	for (int index = 0; index < number_of_bilaterals; index++) {
		real gam;
		gam = gamma[index + number_of_rigid_rigid * 6];
		uint b1 = ids[index].x;

		int offset1 = offset[index];
		int offset2 = offset[index + number_of_bilaterals];

		if (active[b1] != 0) {
			updateV[offset1] = JXYZA[index] * gam;
			updateO[offset1] = JUVWA[index] * gam;
		}

		uint b2 = ids[index].y;
		if (active[b2] != 0) {
			updateV[offset2] = JXYZB[index] * gam;
			updateO[offset2] = JUVWB[index] * gam;
		}
	}
}

void ChConstraintBilateral::host_shurB(
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

#pragma omp  for
	for (int index = 0; index < number_of_bilaterals; index++) {

		real temp = 0;
		uint b1 = ids[index].x;
		if (active[b1] != 0) {
			real3 XYZ = QXYZ[b1];
			real3 UVW = QUVW[b1];
			temp += dot(XYZ, JXYZA[index]);
			temp += dot(UVW, JUVWA[index]);

		}
		uint b2 = ids[index].y;
		if (active[b2] != 0) {

			real3 XYZ = QXYZ[b2];
			real3 UVW = QUVW[b2];
			temp += dot(XYZ, JXYZB[index]);
			temp += dot(UVW, JUVWB[index]);

		}

		AX[index + number_of_rigid_rigid * 6] = temp;
	}

}
void ChConstraintBilateral::ShurA(real* x) {
	host_shurA(
			data_container->host_data.bids_bilateral.data(),
			data_container->host_data.active_data.data(),
			data_container->host_data.mass_data.data(),
			data_container->host_data.inr_data.data(),
			data_container->host_data.JXYZA_bilateral.data(),
			data_container->host_data.JXYZB_bilateral.data(),
			data_container->host_data.JUVWA_bilateral.data(),
			data_container->host_data.JUVWB_bilateral.data(),
			x,
			vel_update.data(),
			omg_update.data(),
			data_container->host_data.QXYZ_data.data(),
			data_container->host_data.QUVW_data.data(),
			update_offset.data());

	host_Reduce_Shur(
			data_container->host_data.active_data.data(),
			data_container->host_data.QXYZ_data.data(),
			data_container->host_data.QUVW_data.data(),
			data_container->host_data.mass_data.data(),
			data_container->host_data.inr_data.data(),
			vel_update.data(),
			omg_update.data(),
			body_number.data(),
			offset_counter.data());

}
void ChConstraintBilateral::ShurB(real*x, real* output) {
	host_shurB(
			data_container->host_data.bids_bilateral.data(),
			data_container->host_data.active_data.data(),
			data_container->host_data.mass_data.data(),
			data_container->host_data.inr_data.data(),
			x,
			data_container->host_data.JXYZA_bilateral.data(),
			data_container->host_data.JXYZB_bilateral.data(),
			data_container->host_data.JUVWA_bilateral.data(),
			data_container->host_data.JUVWB_bilateral.data(),
			data_container->host_data.QXYZ_data.data(),
			data_container->host_data.QUVW_data.data(),
			output);

}

void ChConstraintBilateral::ShurBilaterals(custom_vector<real> &x_t, custom_vector<real> & AX) {

	for (int i = 0; i < number_of_rigid; i++) {
		data_container->host_data.QXYZ_data[i] = R3(0);
		data_container->host_data.QUVW_data[i] = R3(0);
	}

	for (int index = 0; index < number_of_bilaterals; index++) {
		real gam;
		gam = x_t[index];
		uint b1 = data_container->host_data.bids_bilateral[index].x;

		if (data_container->host_data.active_data[b1] != 0) {
			data_container->host_data.QXYZ_data[b1] += data_container->host_data.JXYZA_bilateral[index] * gam;
			data_container->host_data.QUVW_data[b1] += data_container->host_data.JUVWA_bilateral[index] * gam;
		}

		uint b2 = data_container->host_data.bids_bilateral[index].y;
		if (data_container->host_data.active_data[b2] != 0) {
			data_container->host_data.QXYZ_data[b2] += data_container->host_data.JXYZB_bilateral[index] * gam;
			data_container->host_data.QUVW_data[b2] += data_container->host_data.JUVWB_bilateral[index] * gam;
		}
	}

#pragma omp parallel for
		for (int index = 0; index < number_of_bilaterals; index++) {

			real temp = 0;
			uint b1 = data_container->host_data.bids_bilateral[index].x;
			if (data_container->host_data.active_data[b1] != 0) {
				real3 XYZ = data_container->host_data.QXYZ_data[b1]* data_container->host_data.mass_data[b1];
				real3 UVW = data_container->host_data.QUVW_data[b1]* data_container->host_data.inr_data[b1];
				temp += dot(XYZ, data_container->host_data.JXYZA_bilateral[index ]);
				temp += dot(UVW, data_container->host_data.JUVWA_bilateral[index ]);

			}
			uint b2 = data_container->host_data.bids_bilateral[index].y;
			if (data_container->host_data.active_data[b2] != 0) {

				real3 XYZ = data_container->host_data.QXYZ_data[b2]* data_container->host_data.mass_data[b2];;
				real3 UVW = data_container->host_data.QUVW_data[b2]* data_container->host_data.inr_data[b2];
				temp += dot(XYZ, data_container->host_data.JXYZB_bilateral[index ]);
				temp += dot(UVW, data_container->host_data.JUVWB_bilateral[index ]);

			}
			AX[index ] = temp;
		}

	}
void ChConstraintBilateral::host_Diag(int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real* diag) {
#pragma omp parallel for
	for (int index = 0; index < number_of_bilaterals; index++) {
		real3 temp = R3(0);
		int2 id_ = ids[index];
		uint b1 = id_.x;
		uint b2 = id_.y;
		real3 eta = R3(0);
		if (active[b1] != 0) {

			real inverse_mass = inv_mass[b1];
			eta.x += dot(JXYZA[index], JXYZA[index]) * inverse_mass;

			real3 inverse_inertia = inv_inertia[b1];
			eta.x += dot(JUVWA[index] * JUVWA[index], inverse_inertia);

		}
		if (active[b2] != 0) {

			real inverse_mass = inv_mass[b2];
			eta.x += dot(JXYZB[index], JXYZB[index]) * inverse_mass;

			real3 inverse_inertia = inv_inertia[b2];
			eta.x += dot(JUVWB[index] * JUVWB[index], inverse_inertia);

		}
		diag[index + number_of_rigid_rigid * 6] = eta.x;

	}
}

void ChConstraintBilateral::Diag() {
	host_Diag(
			data_container->host_data.bids_bilateral.data(),
			data_container->host_data.active_data.data(),
			data_container->host_data.mass_data.data(),
			data_container->host_data.inr_data.data(),
			data_container->host_data.JXYZA_bilateral.data(),
			data_container->host_data.JXYZB_bilateral.data(),
			data_container->host_data.JUVWA_bilateral.data(),
			data_container->host_data.JUVWB_bilateral.data(),
			data_container->host_data.diag.data());
}
