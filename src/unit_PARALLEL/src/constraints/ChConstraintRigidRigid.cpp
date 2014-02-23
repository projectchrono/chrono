#include "ChConstraintRigidRigid.h"
using namespace chrono;

void Orthogonalize(real3 &U, real3 &V, real3 &W) {
//	if (U == R3(0, 0, 0)) {
//		U = R3(1, 0, 0);
//	} else {
//		U = normalize(U);
//	}
	W = cross(U, R3(0, 1, 0));
	real mzlen = length(W);

	if (mzlen < 0.0001) {     // was near singularity? change singularity reference vector!
		real3 mVsingular = R3(1, 0, 0);
		W = cross(U, mVsingular);
		mzlen = length(W);
	}
	W = W * 1.0 / mzlen;
	V = cross(W, U);

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool Cone_generalized(real & gamma_n, real & gamma_u, real & gamma_v, const real & mu) {

	real f_tang = sqrt(gamma_u * gamma_u + gamma_v * gamma_v);

	// inside upper cone? keep untouched!
	if (f_tang < (mu * gamma_n)) {
		return false;
	}

	// inside lower cone? reset  normal,u,v to zero!
	if ((f_tang) < -(1.0 / mu) * gamma_n || (fabs(gamma_n) < 10e-15)) {
		gamma_n = 0;
		gamma_u = 0;
		gamma_v = 0;
		return false;
	}

	// remaining case: project orthogonally to generator segment of upper cone

	gamma_n = (f_tang * mu + gamma_n) / (mu * mu + 1);
	real tproj_div_t = (gamma_n * mu) / f_tang;
	gamma_u *= tproj_div_t;
	gamma_v *= tproj_div_t;

	return true;
}

void Cone_single(real & gamma_n, real & gamma_s, const real & mu) {

	real f_tang = abs(gamma_s);

	// inside upper cone? keep untouched!
	if (f_tang < (mu * gamma_n)) {
		return;
	}

	// inside lower cone? reset  normal,u,v to zero!
	if ((f_tang) < -(1.0 / mu) * gamma_n || (fabs(gamma_n) < 10e-15)) {
		gamma_n = 0;
		gamma_s = 0;
		return;
	}

	// remaining case: project orthogonally to generator segment of upper cone

	gamma_n = (f_tang * mu + gamma_n) / (mu * mu + 1);
	real tproj_div_t = (gamma_n * mu) / f_tang;
	gamma_s *= tproj_div_t;

}

void func_Project(int &index, uint number_of_contacts, int2 *ids, real3 *fric, real* cohesion, real *gam) {
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

	real3 f_a = fric[body_id.x];
	real3 f_b = fric[body_id.y];

	real mu = (f_a.x == 0 || f_b.x == 0) ? 0 : (f_a.x + f_b.x) * .5;
	if (mu == 0) {
		gamma.x = gamma.x < 0 ? 0 : gamma.x - coh;
		gamma.y = gamma.z = 0;

		gam[index + number_of_contacts * 0] = gamma.x;
		gam[index + number_of_contacts * 1] = gamma.y;
		gam[index + number_of_contacts * 2] = gamma.z;

		return;
	}

	if (Cone_generalized(gamma.x, gamma.y, gamma.z, mu)) {
	}

	gam[index + number_of_contacts * 0] = gamma.x - coh;
	gam[index + number_of_contacts * 1] = gamma.y;
	gam[index + number_of_contacts * 2] = gamma.z;

}
void func_Project_rolling(int &index, uint number_of_contacts, int2 *ids, real3 *fric, real* cohesion, real *gam, bool solve_spinning) {
	real3 gamma_roll = R3(0);
	int2 body_id = ids[index];
	real3 f_a = fric[body_id.x];
	real3 f_b = fric[body_id.y];

	real rollingfriction = (f_a.y == 0 || f_b.y == 0) ? 0 : (f_a.y + f_b.y) * .5;
	real spinningfriction = (f_a.z == 0 || f_b.z == 0) ? 0 : (f_a.z + f_b.z) * .5;

//	if(rollingfriction||spinningfriction){
//		gam[index + number_of_contacts * 1] = 0;
//		gam[index + number_of_contacts * 2] = 0;
//	}

	real gamma_n = abs(gam[index + number_of_contacts * 0]);
	real gamma_s = gam[index + number_of_contacts * 3];
	real gamma_tu = gam[index + number_of_contacts * 4];
	real gamma_tv = gam[index + number_of_contacts * 5];

	if (spinningfriction == 0) {
		gamma_s = 0;

	} else {
		Cone_single(gamma_n, gamma_s, spinningfriction);
	}

	if (rollingfriction == 0) {
		gamma_tu = 0;
		gamma_tv = 0;
//		if (gamma_n < 0) {
//			gamma_n = 0;
//		}
	} else {
		Cone_generalized(gamma_n, gamma_tu, gamma_tv, rollingfriction);
	}
	//gam[index + number_of_contacts * 0] = gamma_n;
	gam[index + number_of_contacts * 3] = gamma_s;
	gam[index + number_of_contacts * 4] = gamma_tu;
	gam[index + number_of_contacts * 5] = gamma_tv;

}

void ChConstraintRigidRigid::host_Project(int2 *ids, real3 *friction, real* cohesion, real *gamma) {
#pragma omp parallel for
	for (int index = 0; index < number_of_rigid_rigid; index++) {
		//always project normal

		if (solve_sliding) {
			func_Project(index, number_of_rigid_rigid, ids, friction, cohesion, gamma);
		} else {
			real gamma_x = gamma[index + number_of_rigid_rigid * 0];
			int2 body_id = ids[index];
			real coh = (cohesion[body_id.x] + cohesion[body_id.y]) * .5;
			if (coh < 0) {
				coh = 0;
			}
			gamma_x += coh;

			gamma_x = gamma_x < 0 ? 0 : gamma_x - coh;

			gamma[index + number_of_rigid_rigid * 0] = gamma_x;
			gamma[index + number_of_rigid_rigid * 1] = 0;
			gamma[index + number_of_rigid_rigid * 2] = 0;

		}
		if (solve_spinning) {
			func_Project_rolling(index, number_of_rigid_rigid, ids, friction, cohesion, gamma, solve_spinning);
		}
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
void ChConstraintRigidRigid::host_RHS(int2 *ids, real *correction, real4 * compliance, bool * active, real3* norm, real3* ptA, real3* ptB, real3 *vel, real3 *omega, real3 *JXYZA, real3 *JXYZB,
		real3 *JUVWA, real3 *JUVWB, real *rhs) {

#pragma omp parallel for
	for (int index = 0; index < number_of_rigid_rigid; index++) {
		uint b1 = ids[index].x;
		uint b2 = ids[index].y;
		real3 temp = R3(0);

		real3 U = norm[index], V, W;
		Orthogonalize(U, V, W);     //read 3 float

		if (active[b1]) {
			real3 omega_b1 = omega[b1];
			real3 vel_b1 = vel[b1];
			temp.x += dot(-U, vel_b1) + dot(JUVWA[index + number_of_rigid_rigid * 0], omega_b1);
			if (solve_sliding) {
				temp.y += dot(-V, vel_b1) + dot(JUVWA[index + number_of_rigid_rigid * 1], omega_b1);
				temp.z += dot(-W, vel_b1) + dot(JUVWA[index + number_of_rigid_rigid * 2], omega_b1);
			}
		}
		if (active[b2]) {
			real3 omega_b2 = omega[b2];
			real3 vel_b2 = vel[b2];
			temp.x += dot(U, vel_b2) + dot(JUVWB[index + number_of_rigid_rigid * 0], omega_b2);
			if (solve_sliding) {
				temp.y += dot(V, vel_b2) + dot(JUVWB[index + number_of_rigid_rigid * 1], omega_b2);
				temp.z += dot(W, vel_b2) + dot(JUVWB[index + number_of_rigid_rigid * 2], omega_b2);
			}
		}
		real bi = 0;
		if (compliance[index].x) {
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

void ChConstraintRigidRigid::host_RHS_spinning(int2 *ids, real *correction, real4 * compliance, bool * active, real3* norm, real3* ptA, real3* ptB, real3 *vel, real3 *omega, real3 *JXYZA,
		real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *rhs) {

#pragma omp parallel for
	for (int index = 0; index < number_of_rigid_rigid; index++) {
		uint b1 = ids[index].x;
		uint b2 = ids[index].y;
		real3 temp = R3(0);

		if (solve_spinning) {
			if (active[b1]) {
				real3 omega_b1 = omega[b1];
				temp.x += dot(JUVWA[index + number_of_rigid_rigid * 3], omega_b1);
				temp.y += dot(JUVWA[index + number_of_rigid_rigid * 4], omega_b1);
				temp.z += dot(JUVWA[index + number_of_rigid_rigid * 5], omega_b1);
			}
			if (active[b2]) {
				real3 omega_b2 = omega[b2];
				temp.x += dot(JUVWB[index + number_of_rigid_rigid * 3], omega_b2);
				temp.y += dot(JUVWB[index + number_of_rigid_rigid * 4], omega_b2);
				temp.z += dot(JUVWB[index + number_of_rigid_rigid * 5], omega_b2);
			}
			//cout << temp.x << " " << temp.y << " " << temp.z << endl;
		}
		rhs[index + number_of_rigid_rigid * 3] = -temp.x;
		rhs[index + number_of_rigid_rigid * 4] = -temp.y;
		rhs[index + number_of_rigid_rigid * 5] = -temp.z;
	}
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidRigid::ComputeRHS() {
	comp_rigid_rigid.resize(number_of_rigid_rigid);
#pragma omp parallel for
	for (int i = 0; i < number_of_rigid_rigid; i++) {
		uint b1 = data_container->host_data.bids_rigid_rigid[i].x;
		uint b2 = data_container->host_data.bids_rigid_rigid[i].y;

		real4 compb1 = data_container->host_data.compliance_data[b1];
		real4 compb2 = data_container->host_data.compliance_data[b2];
		real4 compab = R4(0.0);

		compab.x = (compb1.x == 0 || compb2.x == 0) ? 0 : (compb1.x + compb2.x) * .5;
		compab.y = (compb1.y == 0 || compb2.y == 0) ? 0 : (compb1.y + compb2.y) * .5;
		compab.z = (compb1.z == 0 || compb2.z == 0) ? 0 : (compb1.z + compb2.z) * .5;
		compab.w = (compb1.w == 0 || compb2.w == 0) ? 0 : (compb1.w + compb2.w) * .5;
		comp_rigid_rigid[i] = inv_hhpa * compab;
	}
	host_RHS(data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.dpth_rigid_rigid.data(), comp_rigid_rigid.data(), data_container->host_data.active_data.data(),
			data_container->host_data.norm_rigid_rigid.data(), data_container->host_data.cpta_rigid_rigid.data(), data_container->host_data.cptb_rigid_rigid.data(),
			data_container->host_data.vel_data.data(), data_container->host_data.omg_data.data(), JXYZA_rigid_rigid.data(), JXYZB_rigid_rigid.data(), JUVWA_rigid_rigid.data(),
			JUVWB_rigid_rigid.data(), data_container->host_data.rhs_data.data());

	host_RHS_spinning(data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.dpth_rigid_rigid.data(), comp_rigid_rigid.data(), data_container->host_data.active_data.data(),
			data_container->host_data.norm_rigid_rigid.data(), data_container->host_data.cpta_rigid_rigid.data(), data_container->host_data.cptb_rigid_rigid.data(),
			data_container->host_data.vel_data.data(), data_container->host_data.omg_data.data(), JXYZA_rigid_rigid.data(), JXYZB_rigid_rigid.data(), JUVWA_rigid_rigid.data(),
			JUVWB_rigid_rigid.data(), data_container->host_data.rhs_data.data());

//#pragma omp parallel for
//	for (int index = 0; index < number_of_rigid_rigid; index++) {
//		uint b1 = data_container->host_data.bids_rigid_rigid[index].x;
//		uint b2 = data_container->host_data.bids_rigid_rigid[index].y;
//
//
//
//
//	}

}

void ChConstraintRigidRigid::UpdateRHS() {
	host_RHS(data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.dpth_rigid_rigid.data(), comp_rigid_rigid.data(), data_container->host_data.active_data.data(),
			data_container->host_data.norm_rigid_rigid.data(), data_container->host_data.cpta_rigid_rigid.data(), data_container->host_data.cptb_rigid_rigid.data(),
			data_container->host_data.vel_new_data.data(), data_container->host_data.omg_new_data.data(), JXYZA_rigid_rigid.data(), JXYZB_rigid_rigid.data(), JUVWA_rigid_rigid.data(),
			JUVWB_rigid_rigid.data(), data_container->host_data.rhs_data.data());

	host_RHS_spinning(data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.dpth_rigid_rigid.data(), comp_rigid_rigid.data(), data_container->host_data.active_data.data(),
			data_container->host_data.norm_rigid_rigid.data(), data_container->host_data.cpta_rigid_rigid.data(), data_container->host_data.cptb_rigid_rigid.data(),
			data_container->host_data.vel_new_data.data(), data_container->host_data.omg_new_data.data(), JXYZA_rigid_rigid.data(), JXYZB_rigid_rigid.data(), JUVWA_rigid_rigid.data(),
			JUVWB_rigid_rigid.data(), data_container->host_data.rhs_data.data());

//#pragma omp parallel for
//	for (int index = 0; index < number_of_rigid_rigid; index++) {
//		uint b1 = data_container->host_data.bids_rigid_rigid[index].x;
//		uint b2 = data_container->host_data.bids_rigid_rigid[index].y;
//
//
//
//
//	}

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

__host__ __device__ void inline Compute_Jacobian(const real4& quaternion_rotation, const real3& normal, const real3& tangent_u, const real3& tangent_w, const real3& point, real3& T1, real3& T2,
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

__host__ __device__ void inline Compute_Jacobian_Rolling(const real4& quaternion_rotation, const real3& normal, const real3& tangent_u, const real3& tangent_w, real3& T1, real3& T2, real3& T3) {

	M33 contact_plane;

	contact_plane.U = normal;
	contact_plane.V = tangent_u;
	contact_plane.W = tangent_w;

	M33 Jr = MatTMult(contact_plane, AMat(quaternion_rotation));

	T1 = R3(Jr.U.x, Jr.V.x, Jr.W.x);
	T2 = R3(Jr.U.y, Jr.V.y, Jr.W.y);
	T3 = R3(Jr.U.z, Jr.V.z, Jr.W.z);

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void JacobianNormal() {

}
void ChConstraintRigidRigid::host_Jacobians(real3* norm, real3* ptA, real3* ptB, int2* ids, real4* rot, real3* pos, real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB) {
#pragma omp parallel for
	for (int index = 0; index < number_of_rigid_rigid; index++) {
		real3 U = norm[index], V, W;
		Orthogonalize(U, V, W);     //read 3 float

		//write 18 float
//		JXYZA[index + number_of_rigid_rigid * 0] = -U;
//		JXYZA[index + number_of_rigid_rigid * 1] = -V;
//		JXYZA[index + number_of_rigid_rigid * 2] = -W;
//
//		JXYZB[index + number_of_rigid_rigid * 0] = U;
//		JXYZB[index + number_of_rigid_rigid * 1] = V;
//		JXYZB[index + number_of_rigid_rigid * 2] = W;

		int2 body_id = ids[index];     //load 2 int
		real3 T3, T4, T5, T6, T7, T8;

		real3 sbar = ptA[index] - pos[body_id.x];     //read 6 float
		real4 E1 = rot[body_id.x];     //read 4 float
		Compute_Jacobian(E1, U, V, W, sbar, T3, T4, T5);

		sbar = ptB[index] - pos[body_id.y];     //read 6 float
		real4 E2 = rot[body_id.y];     //read 4 float
		Compute_Jacobian(E2, U, V, W, sbar, T6, T7, T8);
		T6 = -T6;
		T7 = -T7;
		T8 = -T8;
		//write 18 float
		JUVWA[index + number_of_rigid_rigid * 0] = T3;
		JUVWA[index + number_of_rigid_rigid * 1] = T4;
		JUVWA[index + number_of_rigid_rigid * 2] = T5;

		JUVWB[index + number_of_rigid_rigid * 0] = T6;
		JUVWB[index + number_of_rigid_rigid * 1] = T7;
		JUVWB[index + number_of_rigid_rigid * 2] = T8;

		//total read: 23
		//total write 36
	}
}

void ChConstraintRigidRigid::host_Jacobians_Rolling(real3* norm, real3* ptA, real3* ptB, int2* ids, real4* rot, real3* pos, real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB) {
#pragma omp parallel for
	for (int index = 0; index < number_of_rigid_rigid; index++) {
		real3 U = norm[index];

		if (U == R3(0, 0, 0)) {
			U = R3(1, 0, 0);
		} else {
			U = normalize(U);
		}
		real3 W = cross(U, R3(0, 1, 0));
		real mzlen = length(W);

		if (mzlen < 0.0001) {     // was near singularity? change singularity reference vector!
			real3 mVsingular = R3(1, 0, 0);
			W = cross(U, mVsingular);
			mzlen = length(W);
		}
		W = W * 1.0f / mzlen;
		real3 V = cross(W, U);

		int2 body_id = ids[index];
		real3 T3, T4, T5, T6, T7, T8;

		real4 E1 = rot[body_id.x];
		Compute_Jacobian_Rolling(E1, U, V, W, T3, T4, T5);
		T3 = -T3;
		T4 = -T4;
		T5 = -T5;
		real4 E2 = rot[body_id.y];
		Compute_Jacobian_Rolling(E2, U, V, W, T6, T7, T8);

		JUVWA[index + number_of_rigid_rigid * 3] = T3;
		JUVWA[index + number_of_rigid_rigid * 4] = T4;
		JUVWA[index + number_of_rigid_rigid * 5] = T5;

		JUVWB[index + number_of_rigid_rigid * 3] = T6;
		JUVWB[index + number_of_rigid_rigid * 4] = T7;
		JUVWB[index + number_of_rigid_rigid * 5] = T8;
	}
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidRigid::ComputeJacobians() {

	//JXYZA_rigid_rigid.resize(number_of_rigid_rigid * 3);
	//JXYZB_rigid_rigid.resize(number_of_rigid_rigid * 3);
	JUVWA_rigid_rigid.resize(number_of_rigid_rigid * 6);
	JUVWB_rigid_rigid.resize(number_of_rigid_rigid * 6);

	host_Jacobians(data_container->host_data.norm_rigid_rigid.data(), data_container->host_data.cpta_rigid_rigid.data(), data_container->host_data.cptb_rigid_rigid.data(),
			data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.rot_data.data(), data_container->host_data.pos_data.data(), JXYZA_rigid_rigid.data(), JXYZB_rigid_rigid.data(),
			JUVWA_rigid_rigid.data(), JUVWB_rigid_rigid.data());

	host_Jacobians_Rolling(data_container->host_data.norm_rigid_rigid.data(), data_container->host_data.cpta_rigid_rigid.data(), data_container->host_data.cptb_rigid_rigid.data(),
			data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.rot_data.data(), data_container->host_data.pos_data.data(), JXYZA_rigid_rigid.data(), JXYZB_rigid_rigid.data(),
			JUVWA_rigid_rigid.data(), JUVWB_rigid_rigid.data());

}

void ChConstraintRigidRigid::UpdateJacobians() {

	JXYZA_rigid_rigid.resize(number_of_rigid_rigid * 3);
	JXYZB_rigid_rigid.resize(number_of_rigid_rigid * 3);
	JUVWA_rigid_rigid.resize(number_of_rigid_rigid * 6);
	JUVWB_rigid_rigid.resize(number_of_rigid_rigid * 6);

	host_Jacobians(data_container->host_data.norm_rigid_rigid.data(), data_container->host_data.cpta_rigid_rigid.data(), data_container->host_data.cptb_rigid_rigid.data(),
			data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.rot_new_data.data(), data_container->host_data.pos_new_data.data(), JXYZA_rigid_rigid.data(),
			JXYZB_rigid_rigid.data(), JUVWA_rigid_rigid.data(), JUVWB_rigid_rigid.data());

	host_Jacobians_Rolling(data_container->host_data.norm_rigid_rigid.data(), data_container->host_data.cpta_rigid_rigid.data(), data_container->host_data.cptb_rigid_rigid.data(),
			data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.rot_new_data.data(), data_container->host_data.pos_new_data.data(), JXYZA_rigid_rigid.data(),
			JXYZB_rigid_rigid.data(), JUVWA_rigid_rigid.data(), JUVWB_rigid_rigid.data());

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidRigid::host_shurA_normal(int2 *ids, bool *active, real3* norm, real3* ptA, real3* ptB, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *gamma, real3 *updateV,
		real3 *updateO, uint* offset) {
#pragma omp parallel for
	for (int index = 0; index < number_of_rigid_rigid; index++) {
		real gam;
		gam = gamma[index + number_of_rigid_rigid * 0];

		int offset1 = offset[index];
		int offset2 = offset[index + number_of_rigid_rigid];

		real3 U = norm[index];
//		if (U == R3(0, 0, 0)) {
//			U = R3(1, 0, 0);
//		} else {
//			U = normalize(U);
//		}

		uint b1 = ids[index].x;
		if (active[b1] != 0) {
			updateV[offset1] = -U * gam;
			updateO[offset1] = JUVWA[index + number_of_rigid_rigid * 0] * gam;
		}
		uint b2 = ids[index].y;
		if (active[b2] != 0) {
			updateV[offset2] = U * gam;
			updateO[offset2] = JUVWB[index + number_of_rigid_rigid * 0] * gam;
		}
	}
}

void ChConstraintRigidRigid::host_shurA_sliding(int2 *ids, bool *active, real3* norm, real3* ptA, real3* ptB, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *gamma, real3 *updateV,
		real3 *updateO, uint* offset) {
#pragma omp parallel for
	for (int index = 0; index < number_of_rigid_rigid; index++) {
		real3 gam;
		gam.x = gamma[index + number_of_rigid_rigid * 0];
		gam.y = gamma[index + number_of_rigid_rigid * 1];
		gam.z = gamma[index + number_of_rigid_rigid * 2];

		int offset1 = offset[index];
		int offset2 = offset[index + number_of_rigid_rigid];

		real3 U = norm[index], V, W;
		Orthogonalize(U, V, W);

		//read 39
		//read 21
		uint b1 = ids[index].x;
		if (active[b1] != 0) {
			updateV[offset1] = -U * gam.x - V * gam.y - W * gam.z;
			updateO[offset1] = JUVWA[index + number_of_rigid_rigid * 0] * gam.x + JUVWA[index + number_of_rigid_rigid * 1] * gam.y + JUVWA[index + number_of_rigid_rigid * 2] * gam.z;
		}
		uint b2 = ids[index].y;
		if (active[b2] != 0) {

			updateV[offset2] = U * gam.x + V * gam.y + W * gam.z;
			updateO[offset2] = JUVWB[index + number_of_rigid_rigid * 0] * gam.x + JUVWB[index + number_of_rigid_rigid * 1] * gam.y + JUVWB[index + number_of_rigid_rigid * 2] * gam.z;

		}
	}
}

void ChConstraintRigidRigid::host_shurA_spinning(int2 *ids, bool *active, real3* norm, real3* ptA, real3* ptB, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *gamma, real3 *updateV,
		real3 *updateO, uint* offset) {
#pragma omp parallel for
	for (int index = 0; index < number_of_rigid_rigid; index++) {
		real3 gam, gam_roll;
		gam.x = gamma[index + number_of_rigid_rigid * 0];
		gam.y = gamma[index + number_of_rigid_rigid * 1];
		gam.z = gamma[index + number_of_rigid_rigid * 2];
		gam_roll.x = gamma[index + number_of_rigid_rigid * 3];
		gam_roll.y = gamma[index + number_of_rigid_rigid * 4];
		gam_roll.z = gamma[index + number_of_rigid_rigid * 5];

		uint b1 = ids[index].x;

		real3 U = norm[index], V, W;
		Orthogonalize(U, V, W);

		int offset1 = offset[index];
		int offset2 = offset[index + number_of_rigid_rigid];
		if (active[b1] != 0) {
			updateV[offset1] = -U * gam.x - V * gam.y - W * gam.z;
			updateO[offset1] = JUVWA[index + number_of_rigid_rigid * 0] * gam.x + JUVWA[index + number_of_rigid_rigid * 1] * gam.y + JUVWA[index + number_of_rigid_rigid * 2] * gam.z
					+ JUVWA[index + number_of_rigid_rigid * 3] * gam_roll.x + JUVWA[index + number_of_rigid_rigid * 4] * gam_roll.y + JUVWA[index + number_of_rigid_rigid * 5] * gam_roll.z;
		}
		uint b2 = ids[index].y;
		if (active[b2] != 0) {
			updateV[offset2] = U * gam.x + V * gam.y + W * gam.z;
			updateO[offset2] = JUVWB[index + number_of_rigid_rigid * 0] * gam.x + JUVWB[index + number_of_rigid_rigid * 1] * gam.y + JUVWB[index + number_of_rigid_rigid * 2] * gam.z
					+ JUVWB[index + number_of_rigid_rigid * 3] * gam_roll.x + JUVWB[index + number_of_rigid_rigid * 4] * gam_roll.y + JUVWB[index + number_of_rigid_rigid * 5] * gam_roll.z;
		}

	}
}

void ChConstraintRigidRigid::host_shurB_normal(int2 *ids, bool *active, real3* norm, real3* ptA, real3* ptB, real *inv_mass, real3 *inv_inertia, real4 * compliance, real * gamma, real3 *JXYZA,
		real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real3 *QXYZ, real3 *QUVW, real *AX) {

#pragma omp parallel for
	for (int index = 0; index < number_of_rigid_rigid; index++) {
		real3 temp = R3(0);

		int2 id_ = ids[index];
		uint b1 = id_.x;
		uint b2 = id_.y;

		real3 U = norm[index];

		if (active[b1] != 0) {
			real3 XYZ = QXYZ[b1];
			real3 UVW = QUVW[b1];

			temp.x += dot(XYZ, -U);
			temp.x += dot(UVW, JUVWA[index + number_of_rigid_rigid * 0]);

		}
		if (active[b2] != 0) {
			real3 XYZ = QXYZ[b2];
			real3 UVW = QUVW[b2];

			temp.x += dot(XYZ, U);
			temp.x += dot(UVW, JUVWB[index + number_of_rigid_rigid * 0]);

		}
		real4 comp = compliance[index];

		AX[index + number_of_rigid_rigid * 0] = temp.x + gamma[index + number_of_rigid_rigid * 0] * comp.x;
		AX[index + number_of_rigid_rigid * 1] = 0;
		AX[index + number_of_rigid_rigid * 2] = 0;
		AX[index + number_of_rigid_rigid * 3] = 0;
		AX[index + number_of_rigid_rigid * 4] = 0;
		AX[index + number_of_rigid_rigid * 5] = 0;

	}
}
void ChConstraintRigidRigid::host_shurB_sliding(int2 *ids, bool *active, real3* norm, real3* ptA, real3* ptB, real *inv_mass, real3 *inv_inertia, real4 * compliance, real * gamma, real3 *JXYZA,
		real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real3 *QXYZ, real3 *QUVW, real *AX) {

#pragma omp parallel for
	for (int index = 0; index < number_of_rigid_rigid; index++) {
		real3 temp = R3(0);

		int2 id_ = ids[index];
		uint b1 = id_.x;
		uint b2 = id_.y;

		real3 U = norm[index], V, W;
		Orthogonalize(U, V, W);

		if (active[b1] != 0) {
			real3 XYZ = QXYZ[b1];
			real3 UVW = QUVW[b1];

			temp.x += dot(XYZ, -U);
			temp.x += dot(UVW, JUVWA[index + number_of_rigid_rigid * 0]);

			temp.y += dot(XYZ, -V);
			temp.y += dot(UVW, JUVWA[index + number_of_rigid_rigid * 1]);

			temp.z += dot(XYZ, -W);
			temp.z += dot(UVW, JUVWA[index + number_of_rigid_rigid * 2]);
		}
		if (active[b2] != 0) {
			real3 XYZ = QXYZ[b2];
			real3 UVW = QUVW[b2];

			temp.x += dot(XYZ, U);
			temp.x += dot(UVW, JUVWB[index + number_of_rigid_rigid * 0]);

			temp.y += dot(XYZ, V);
			temp.y += dot(UVW, JUVWB[index + number_of_rigid_rigid * 1]);

			temp.z += dot(XYZ, W);
			temp.z += dot(UVW, JUVWB[index + number_of_rigid_rigid * 2]);

		}
		real4 comp = compliance[index];

		AX[index + number_of_rigid_rigid * 0] = temp.x + gamma[index + number_of_rigid_rigid * 0] * comp.x;
		AX[index + number_of_rigid_rigid * 1] = temp.y + gamma[index + number_of_rigid_rigid * 1] * comp.y;
		AX[index + number_of_rigid_rigid * 2] = temp.z + gamma[index + number_of_rigid_rigid * 2] * comp.y;
		AX[index + number_of_rigid_rigid * 3] = 0;
		AX[index + number_of_rigid_rigid * 4] = 0;
		AX[index + number_of_rigid_rigid * 5] = 0;

	}
}
void ChConstraintRigidRigid::host_shurB_spinning(int2 *ids, bool *active, real3* norm, real3* ptA, real3* ptB, real *inv_mass, real3 *inv_inertia, real4 * compliance, real * gamma, real3 *JXYZA,
		real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real3 *QXYZ, real3 *QUVW, real *AX) {

#pragma omp parallel for
	for (int index = 0; index < number_of_rigid_rigid; index++) {
		real3 temp = R3(0);
		real3 temp_roll = R3(0);

		int2 id_ = ids[index];
		uint b1 = id_.x;
		uint b2 = id_.y;

		real3 U = norm[index], V, W;
		Orthogonalize(U, V, W);

		if (active[b1] != 0) {
			real3 XYZ = QXYZ[b1];
			real3 UVW = QUVW[b1];

			temp.x += dot(XYZ, -U);
			temp.x += dot(UVW, JUVWA[index + number_of_rigid_rigid * 0]);

			temp.y += dot(XYZ, -V);
			temp.y += dot(UVW, JUVWA[index + number_of_rigid_rigid * 1]);

			temp.z += dot(XYZ, -W);
			temp.z += dot(UVW, JUVWA[index + number_of_rigid_rigid * 2]);

			temp_roll.x += dot(UVW, JUVWA[index + number_of_rigid_rigid * 3]);
			temp_roll.y += dot(UVW, JUVWA[index + number_of_rigid_rigid * 4]);
			temp_roll.z += dot(UVW, JUVWA[index + number_of_rigid_rigid * 5]);

		}
		if (active[b2] != 0) {
			real3 XYZ = QXYZ[b2];
			real3 UVW = QUVW[b2];

			temp.x += dot(XYZ, U);
			temp.x += dot(UVW, JUVWB[index + number_of_rigid_rigid * 0]);

			temp.y += dot(XYZ, V);
			temp.y += dot(UVW, JUVWB[index + number_of_rigid_rigid * 1]);

			temp.z += dot(XYZ, W);
			temp.z += dot(UVW, JUVWB[index + number_of_rigid_rigid * 2]);

			temp_roll.x += dot(UVW, JUVWB[index + number_of_rigid_rigid * 3]);
			temp_roll.y += dot(UVW, JUVWB[index + number_of_rigid_rigid * 4]);
			temp_roll.z += dot(UVW, JUVWB[index + number_of_rigid_rigid * 5]);

		}
		real4 comp = compliance[index];

		AX[index + number_of_rigid_rigid * 0] = temp.x + gamma[index + number_of_rigid_rigid * 0] * comp.x;
		AX[index + number_of_rigid_rigid * 1] = temp.y + gamma[index + number_of_rigid_rigid * 1] * comp.y;
		AX[index + number_of_rigid_rigid * 2] = temp.z + gamma[index + number_of_rigid_rigid * 2] * comp.y;

		AX[index + number_of_rigid_rigid * 3] = temp_roll.x + gamma[index + number_of_rigid_rigid * 3] * comp.w;
		AX[index + number_of_rigid_rigid * 4] = temp_roll.y + gamma[index + number_of_rigid_rigid * 4] * comp.z;
		AX[index + number_of_rigid_rigid * 5] = temp_roll.z + gamma[index + number_of_rigid_rigid * 5] * comp.z;

	}
}

__host__ __device__ void function_Reduce_Shur(int& index, bool* active, real3* QXYZ, real3* QUVW, real *inv_mass, real3 *inv_inertia, real3* updateQXYZ, real3* updateQUVW, uint* d_body_num,
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
#pragma omp parallel for
	for (int index = 0; index < number_of_updates; index++) {
		function_Reduce_Shur(index, active, QXYZ, QUVW, inv_mass, inv_inertia, updateQXYZ, updateQUVW, d_body_num, counter);
	}
}

void ChConstraintRigidRigid::host_Offsets(int2* ids, uint* Body) {
#pragma omp parallel for
	for (int index = 0; index < number_of_rigid_rigid; index++) {
		if (index < number_of_rigid_rigid) {
			int2 temp_id = ids[index];
			Body[index] = temp_id.x;
			Body[index + number_of_rigid_rigid] = temp_id.y;
		}
	}
}

void ChConstraintRigidRigid::ShurA(custom_vector<real> &x) {
	//ChTimer<double> timer_A;
	//timer_A.start();

	if(solve_spinning) {
		host_shurA_spinning(
		data_container->host_data.bids_rigid_rigid.data(),
		data_container->host_data.active_data.data(),
		data_container->host_data.norm_rigid_rigid.data(),
		data_container->host_data.cpta_rigid_rigid.data(),
		data_container->host_data.cptb_rigid_rigid.data(),
		JXYZA_rigid_rigid.data(),
		JXYZB_rigid_rigid.data(),
		JUVWA_rigid_rigid.data(),
		JUVWB_rigid_rigid.data(),
		x.data(),
		vel_update.data(),
		omg_update.data(),
		update_offset.data());
	} else if(solve_sliding) {
		host_shurA_sliding(
		data_container->host_data.bids_rigid_rigid.data(),
		data_container->host_data.active_data.data(),
		data_container->host_data.norm_rigid_rigid.data(),
		data_container->host_data.cpta_rigid_rigid.data(),
		data_container->host_data.cptb_rigid_rigid.data(),
		JXYZA_rigid_rigid.data(),
		JXYZB_rigid_rigid.data(),
		JUVWA_rigid_rigid.data(),
		JUVWB_rigid_rigid.data(),
		x.data(),
		vel_update.data(),
		omg_update.data(),
		update_offset.data());
	} else {
		host_shurA_normal(
		data_container->host_data.bids_rigid_rigid.data(),
		data_container->host_data.active_data.data(),
		data_container->host_data.norm_rigid_rigid.data(),
		data_container->host_data.cpta_rigid_rigid.data(),
		data_container->host_data.cptb_rigid_rigid.data(),
		JXYZA_rigid_rigid.data(),
		JXYZB_rigid_rigid.data(),
		JUVWA_rigid_rigid.data(),
		JUVWB_rigid_rigid.data(),
		x.data(),
		vel_update.data(),
		omg_update.data(),
		update_offset.data());

	}

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
//timer_A.stop();
//cout<<"A: "<<timer_A()<<endl;

}
void ChConstraintRigidRigid::ShurB(custom_vector<real> &x, custom_vector<real> & output) {
	//ChTimer<double> timer_B;
	//	timer_B.start();
	if(solve_spinning) {
		host_shurB_spinning(
		data_container->host_data.bids_rigid_rigid.data(),
		data_container->host_data.active_data.data(),
		data_container->host_data.norm_rigid_rigid.data(),
		data_container->host_data.cpta_rigid_rigid.data(),
		data_container->host_data.cptb_rigid_rigid.data(),
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

	} else if (solve_sliding) {
		host_shurB_sliding(
		data_container->host_data.bids_rigid_rigid.data(),
		data_container->host_data.active_data.data(),
		data_container->host_data.norm_rigid_rigid.data(),
		data_container->host_data.cpta_rigid_rigid.data(),
		data_container->host_data.cptb_rigid_rigid.data(),
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

	} else {
		host_shurB_normal(
		data_container->host_data.bids_rigid_rigid.data(),
		data_container->host_data.active_data.data(),
		data_container->host_data.norm_rigid_rigid.data(),
		data_container->host_data.cpta_rigid_rigid.data(),
		data_container->host_data.cptb_rigid_rigid.data(),
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

	//timer_B.stop();
	//cout<<"B: "<<timer_B()<<endl;

}
void ChConstraintRigidRigid::host_Diag(int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real* diag) {
#pragma omp parallel for
	for (int index = 0; index < number_of_rigid_rigid; index++) {
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
	host_Diag(data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.active_data.data(), data_container->host_data.mass_data.data(), data_container->host_data.inr_data.data(),
			JXYZA_rigid_rigid.data(), JXYZB_rigid_rigid.data(), JUVWA_rigid_rigid.data(), JUVWB_rigid_rigid.data(), data_container->host_data.diag.data());
}

void ChConstraintRigidRigid::Build_N() {
	//M^-1 * D^T
	//
	//D = eacc row is a constraint

	//m bodies, n constraints
	//M: mxm
	//D:mxn
	//M*D^T = mxn
	//D*M*D^T = nxn

	//each constraint has 6 J values, 3 pos, 3 rot
	//each row of D has two entries with 6 values each, so 12 values
	//D has 12*n entries
	//M*D^T has 2*n

}
