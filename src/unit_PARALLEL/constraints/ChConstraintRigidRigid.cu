#include "ChConstraintRigidRigid.h"
using namespace chrono;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void func_Project(uint &index, uint number_of_contacts, int2 *ids, real *fric, real* cohesion, real *gam) {
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

	real mu = (fric[body_id.x] == 0 || fric[body_id.y] == 0) ? 0 : (fric[body_id.x] + fric[body_id.y]) * .5;
	if (mu == 0) {
		gamma.x = gamma.x < 0 ? 0 : gamma.x - coh;
		gamma.y = gamma.z = 0;

		gam[index + number_of_contacts * 0] = gamma.x;
		gam[index + number_of_contacts * 1] = gamma.y;
		gam[index + number_of_contacts * 2] = gamma.z;

		return;

	}

	real f_tang = sqrt(gamma.y * gamma.y + gamma.z * gamma.z);
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
void ChConstraintRigidRigid::host_Project(int2 *ids, real *friction, real* cohesion, real *gamma) {
#pragma omp parallel for
	for (uint index = 0; index < number_of_rigid_rigid; index++) {
		func_Project(index, number_of_rigid_rigid, ids, friction, cohesion, gamma);
	}
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidRigid::Project(custom_vector<real> & gamma) {
	host_Project(
			data_container->host_data.bids_rigid_rigid.data(),
			data_container->host_data.fric_data.data(),
			data_container->host_data.cohesion_data.data(),
			gamma.data());
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void ChConstraintRigidRigid::host_RHS(int2 *ids, real *correction, real * compliance, bool * active, real3 *vel, real3 *omega, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *rhs) {

#pragma omp parallel for
	for (uint index = 0; index < number_of_rigid_rigid; index++) {
		uint b1 = ids[index].x;
		uint b2 = ids[index].y;
		real3 temp = R3(0);
		if (active[b1]) {
			temp.x += dot(JXYZA[index + number_of_rigid_rigid * 0], vel[b1]) + dot(JUVWA[index + number_of_rigid_rigid * 0], omega[b1]);
			temp.y += dot(JXYZA[index + number_of_rigid_rigid * 1], vel[b1]) + dot(JUVWA[index + number_of_rigid_rigid * 1], omega[b1]);
			temp.z += dot(JXYZA[index + number_of_rigid_rigid * 2], vel[b1]) + dot(JUVWA[index + number_of_rigid_rigid * 2], omega[b1]);
		}
		if (active[b2]) {
			temp.x += dot(JXYZB[index + number_of_rigid_rigid * 0], vel[b2]) + dot(JUVWB[index + number_of_rigid_rigid * 0], omega[b2]);
			temp.y += dot(JXYZB[index + number_of_rigid_rigid * 1], vel[b2]) + dot(JUVWB[index + number_of_rigid_rigid * 1], omega[b2]);
			temp.z += dot(JXYZB[index + number_of_rigid_rigid * 2], vel[b2]) + dot(JUVWB[index + number_of_rigid_rigid * 2], omega[b2]);
		}
		real bi = 0;

		if (compliance[index]) {
			bi = inv_hpa * correction[index];
		} else {
			bi = fmax(real(1.0) / step_size * correction[index], -contact_recovery_speed);
		}

		//rhs[index] = -((-temp) + fmaxf(inv_hpa * -fabs(correction[index]), 0));
		rhs[index + number_of_rigid_rigid * 0] = -temp.x - bi;     //(temp + fmax(inv_hpa * correction[index], real(-recovery_speed)));
		rhs[index + number_of_rigid_rigid * 1] = -temp.y - 0;
		rhs[index + number_of_rigid_rigid * 2] = -temp.z - 0;
	}

}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidRigid::ComputeRHS() {

	host_RHS(
			data_container->host_data.bids_rigid_rigid.data(),
			data_container->host_data.dpth_rigid_rigid.data(),
			comp_rigid_rigid.data(),
			data_container->host_data.active_data.data(),
			data_container->host_data.vel_data.data(),
			data_container->host_data.omg_data.data(),
			JXYZA_rigid_rigid.data(),
			JXYZB_rigid_rigid.data(),
			JUVWA_rigid_rigid.data(),
			JUVWB_rigid_rigid.data(),
			data_container->host_data.rhs_data.data());

//#pragma omp parallel for
//	for (uint index = 0; index < number_of_rigid_rigid; index++) {
//		uint b1 = data_container->host_data.bids_rigid_rigid[index].x;
//		uint b2 = data_container->host_data.bids_rigid_rigid[index].y;
//
//
//
//
//	}

}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

__host__ __device__ void inline Compute_Jacobian(
		const real4& quaternion_rotation,
		const real3& normal,
		const real3& tangent_u,
		const real3& tangent_w,
		const real3& point,
		real3& T1,
		real3& T2,
		real3& T3) {

	M33 contact_plane;

	contact_plane.U = normal;
	contact_plane.V = tangent_u;
	contact_plane.W = tangent_w;

	real3 Pl = MatTMult(AMat(quaternion_rotation), point);

	M33 Ps = XMatrix(Pl);
	M33 Jtemp = MatMult(AMat(quaternion_rotation), Ps);
	M33 Jr = MatTMult(contact_plane, Jtemp);

	T1 = R3(Jr.U.x, Jr.V.x, Jr.W.x);
	T2 = R3(Jr.U.y, Jr.V.y, Jr.W.y);
	T3 = R3(Jr.U.z, Jr.V.z, Jr.W.z);

}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidRigid::host_Jacobians(real3* norm, real3* ptA, real3* ptB, int2* ids, real4* rot, real3* pos, real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB) {
#pragma omp parallel for
	for (uint index = 0; index < number_of_rigid_rigid; index++) {
		real3 U = norm[index];

		if (U == R3(0, 0, 0)) {
			U = R3(1, 0, 0);
		} else {
			U = normalize(U);
		}
		U = normalize(U);
		real3 W = cross(U, R3(0, 1, 0));
		real mzlen = length(W);

		if (mzlen < 0.0001) {     // was near singularity? change singularity reference vector!
			real3 mVsingular = R3(0, 1, 0);
			if (mVsingular.x < 0.9) {
				mVsingular = R3(0, 0, 1);
			}
			if (mVsingular.y < 0.9) {
				mVsingular = R3(0, 1, 0);
			}
			if (mVsingular.z < 0.9) {
				mVsingular = R3(1, 0, 0);
			}

			W = cross(U, mVsingular);
			mzlen = length(W);
		}
		W = W * 1.0 / mzlen;
		real3 V = cross(W, U);

		JXYZA[index + number_of_rigid_rigid * 0] = -U;
		JXYZA[index + number_of_rigid_rigid * 1] = -V;
		JXYZA[index + number_of_rigid_rigid * 2] = -W;

		JXYZB[index + number_of_rigid_rigid * 0] = U;
		JXYZB[index + number_of_rigid_rigid * 1] = V;
		JXYZB[index + number_of_rigid_rigid * 2] = W;

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

		JUVWA[index + number_of_rigid_rigid * 0] = T3;
		JUVWA[index + number_of_rigid_rigid * 1] = T4;
		JUVWA[index + number_of_rigid_rigid * 2] = T5;

		JUVWB[index + number_of_rigid_rigid * 0] = T6;
		JUVWB[index + number_of_rigid_rigid * 1] = T7;
		JUVWB[index + number_of_rigid_rigid * 2] = T8;
	}
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidRigid::ComputeJacobians() {

	JXYZA_rigid_rigid.resize(number_of_rigid_rigid * 3);
	JXYZB_rigid_rigid.resize(number_of_rigid_rigid * 3);
	JUVWA_rigid_rigid.resize(number_of_rigid_rigid * 3);
	JUVWB_rigid_rigid.resize(number_of_rigid_rigid * 3);

	host_Jacobians(
			data_container->host_data.norm_rigid_rigid.data(),
			data_container->host_data.cpta_rigid_rigid.data(),
			data_container->host_data.cptb_rigid_rigid.data(),
			data_container->host_data.bids_rigid_rigid.data(),
			data_container->host_data.rot_data.data(),
			data_container->host_data.pos_data.data(),
			JXYZA_rigid_rigid.data(),
			JXYZB_rigid_rigid.data(),
			JUVWA_rigid_rigid.data(),
			JUVWB_rigid_rigid.data());

	comp_rigid_rigid.resize(number_of_rigid_rigid);
#pragma omp parallel for
	for (int i = 0; i < number_of_rigid_rigid; i++) {
		uint b1 = data_container->host_data.bids_rigid_rigid[i].x;
		uint b2 = data_container->host_data.bids_rigid_rigid[i].y;
		real compb1 = data_container->host_data.compliance_data[b1];
		real compb2 = data_container->host_data.compliance_data[b2];

		real comp = (compb1 == 0 || compb2 == 0) ? 0 : (compb1 + compb2) * .5;
		comp_rigid_rigid[i] = inv_hhpa * comp;
	}

}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidRigid::host_shurA(
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
#pragma omp parallel for schedule(dynamic, 100)
	for (int index = 0; index < number_of_rigid_rigid; index++) {
		real3 gam;
		gam.x = gamma[index + number_of_rigid_rigid * 0];
		gam.y = gamma[index + number_of_rigid_rigid * 1];
		gam.z = gamma[index + number_of_rigid_rigid * 2];
		uint b1 = ids[index].x;

		int offset1 = offset[index];
		int offset2 = offset[index + number_of_rigid_rigid];

		if (active[b1] != 0) {
			updateV[offset1] = (JXYZA[index + number_of_rigid_rigid * 0] * gam.x + JXYZA[index + number_of_rigid_rigid * 1] * gam.y + JXYZA[index + number_of_rigid_rigid * 2] * gam.z);
			updateO[offset1] = (JUVWA[index + number_of_rigid_rigid * 0] * gam.x + JUVWA[index + number_of_rigid_rigid * 1] * gam.y + JUVWA[index + number_of_rigid_rigid * 2] * gam.z);
		}
		uint b2 = ids[index].y;
		if (active[b2] != 0) {
			updateV[offset2] = (JXYZB[index + number_of_rigid_rigid * 0] * gam.x + JXYZB[index + number_of_rigid_rigid * 1] * gam.y + JXYZB[index + number_of_rigid_rigid * 2] * gam.z);
			updateO[offset2] = (JUVWB[index + number_of_rigid_rigid * 0] * gam.x + JUVWB[index + number_of_rigid_rigid * 1] * gam.y + JUVWB[index + number_of_rigid_rigid * 2] * gam.z);
		}
	}

}

void ChConstraintRigidRigid::host_shurB(
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

#pragma omp parallel for schedule(dynamic, 100)
	for (uint index = 0; index < number_of_rigid_rigid; index++) {
		real3 temp = R3(0);
		int2 id_ = ids[index];
		uint b1 = id_.x;
		uint b2 = id_.y;

		if (active[b1] != 0) {
			real3 XYZ = QXYZ[b1];
			real3 UVW = QUVW[b1];

			temp.x += dot(XYZ, JXYZA[index + number_of_rigid_rigid * 0]);
			temp.x += dot(UVW, JUVWA[index + number_of_rigid_rigid * 0]);

			temp.y += dot(XYZ, JXYZA[index + number_of_rigid_rigid * 1]);
			temp.y += dot(UVW, JUVWA[index + number_of_rigid_rigid * 1]);

			temp.z += dot(XYZ, JXYZA[index + number_of_rigid_rigid * 2]);
			temp.z += dot(UVW, JUVWA[index + number_of_rigid_rigid * 2]);

		}
		if (active[b2] != 0) {
			real3 XYZ = QXYZ[b2];
			real3 UVW = QUVW[b2];

			temp.x += dot(XYZ, JXYZB[index + number_of_rigid_rigid * 0]);
			temp.x += dot(UVW, JUVWB[index + number_of_rigid_rigid * 0]);

			temp.y += dot(XYZ, JXYZB[index + number_of_rigid_rigid * 1]);
			temp.y += dot(UVW, JUVWB[index + number_of_rigid_rigid * 1]);

			temp.z += dot(XYZ, JXYZB[index + number_of_rigid_rigid * 2]);
			temp.z += dot(UVW, JUVWB[index + number_of_rigid_rigid * 2]);
		}
		AX[index + number_of_rigid_rigid * 0] = temp.x + gamma[index + number_of_rigid_rigid * 0] * compliance[index];
		AX[index + number_of_rigid_rigid * 1] = temp.y + gamma[index + number_of_rigid_rigid * 1] * compliance[index];
		AX[index + number_of_rigid_rigid * 2] = temp.z + gamma[index + number_of_rigid_rigid * 2] * compliance[index];
	}
}

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

void ChConstraintRigidRigid::host_Reduce_Shur(bool* active, real3* QXYZ, real3* QUVW, real *inv_mass, real3 *inv_inertia, real3* updateQXYZ, real3* updateQUVW, uint* d_body_num, uint* counter) {
#pragma omp parallel for schedule(dynamic, 100)
	for (uint index = 0; index < number_of_updates; index++) {
		function_Reduce_Shur(index, active, QXYZ, QUVW, inv_mass, inv_inertia, updateQXYZ, updateQUVW, d_body_num, counter);
	}
}

void ChConstraintRigidRigid::host_Offsets(int2* ids, uint* Body) {
#pragma omp parallel for
	for (uint index = 0; index < number_of_rigid_rigid; index++) {
		if (index < number_of_rigid_rigid) {
			int2 temp_id = ids[index];
			Body[index] = temp_id.x;
			Body[index + number_of_rigid_rigid] = temp_id.y;
		}
	}
}

void ChConstraintRigidRigid::ShurA(custom_vector<real> &x) {

	host_shurA(
			data_container->host_data.bids_rigid_rigid.data(),
			data_container->host_data.active_data.data(),
			data_container->host_data.mass_data.data(),
			data_container->host_data.inr_data.data(),
			JXYZA_rigid_rigid.data(),
			JXYZB_rigid_rigid.data(),
			JUVWA_rigid_rigid.data(),
			JUVWB_rigid_rigid.data(),
			x.data(),
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
void ChConstraintRigidRigid::ShurB(custom_vector<real> &x, custom_vector<real> & output) {
	host_shurB(
			data_container->host_data.bids_rigid_rigid.data(),
			data_container->host_data.active_data.data(),
			data_container->host_data.mass_data.data(),
			data_container->host_data.inr_data.data(),
			comp_rigid_rigid.data(),
			x.data(),
			JXYZA_rigid_rigid.data(),
			JXYZB_rigid_rigid.data(),
			JUVWA_rigid_rigid.data(),
			JUVWB_rigid_rigid.data(),
			data_container->host_data.QXYZ_data.data(),
			data_container->host_data.QUVW_data.data(),
			output.data());
}
void ChConstraintRigidRigid::host_Diag(int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real* diag) {
#pragma omp parallel for schedule(dynamic, 100)
	for (uint index = 0; index < number_of_rigid_rigid; index++) {
		real3 temp = R3(0);
		int2 id_ = ids[index];
		uint b1 = id_.x;
		uint b2 = id_.y;
		real3 eta = R3(0);
		if (active[b1] != 0) {

			real inverse_mass = inv_mass[b1];
			eta.x += dot(JXYZA[index + number_of_rigid_rigid * 0], JXYZA[index + number_of_rigid_rigid * 0]) * inverse_mass;
			eta.y += dot(JXYZA[index + number_of_rigid_rigid * 1], JXYZA[index + number_of_rigid_rigid * 1]) * inverse_mass;
			eta.z += dot(JXYZA[index + number_of_rigid_rigid * 2], JXYZA[index + number_of_rigid_rigid * 2]) * inverse_mass;

			real3 inverse_inertia = inv_inertia[b1];
			eta.x += dot(JUVWA[index + number_of_rigid_rigid * 0] * JUVWA[index + number_of_rigid_rigid * 0], inverse_inertia);
			eta.y += dot(JUVWA[index + number_of_rigid_rigid * 1] * JUVWA[index + number_of_rigid_rigid * 1], inverse_inertia);
			eta.z += dot(JUVWA[index + number_of_rigid_rigid * 2] * JUVWA[index + number_of_rigid_rigid * 2], inverse_inertia);

		}
		if (active[b2] != 0) {

			real inverse_mass = inv_mass[b2];
			eta.x += dot(JXYZB[index + number_of_rigid_rigid * 0], JXYZB[index + number_of_rigid_rigid * 0]) * inverse_mass;
			eta.y += dot(JXYZB[index + number_of_rigid_rigid * 1], JXYZB[index + number_of_rigid_rigid * 1]) * inverse_mass;
			eta.z += dot(JXYZB[index + number_of_rigid_rigid * 2], JXYZB[index + number_of_rigid_rigid * 2]) * inverse_mass;

			real3 inverse_inertia = inv_inertia[b2];
			eta.x += dot(JUVWB[index + number_of_rigid_rigid * 0] * JUVWB[index + number_of_rigid_rigid * 0], inverse_inertia);
			eta.y += dot(JUVWB[index + number_of_rigid_rigid * 1] * JUVWB[index + number_of_rigid_rigid * 1], inverse_inertia);
			eta.z += dot(JUVWB[index + number_of_rigid_rigid * 2] * JUVWB[index + number_of_rigid_rigid * 2], inverse_inertia);

		}
		diag[index + number_of_rigid_rigid * 0] = eta.x;
		diag[index + number_of_rigid_rigid * 1] = eta.y;
		diag[index + number_of_rigid_rigid * 2] = eta.z;
	}
}

void ChConstraintRigidRigid::Diag() {
	host_Diag(
			data_container->host_data.bids_rigid_rigid.data(),
			data_container->host_data.active_data.data(),
			data_container->host_data.mass_data.data(),
			data_container->host_data.inr_data.data(),
			JXYZA_rigid_rigid.data(),
			JXYZB_rigid_rigid.data(),
			JUVWA_rigid_rigid.data(),
			JUVWB_rigid_rigid.data(),
			data_container->host_data.diag.data());
}
