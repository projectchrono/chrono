#include "ChConstraintRigidRigid.h"
using namespace chrono;
#define  _index_  index*6
void inline Orthogonalize(real3 &U, real3 &V, real3 &W) {
	W = cross(U, R3(0, 1, 0));
	real mzlen = length(W);
	if (mzlen < .0001) {     // was near singularity? change singularity reference custom_vector!
		real3 mVsingular = R3(1, 0, 0);
		W = cross(U, mVsingular);
		mzlen = length(W);
	}
	W /= mzlen;
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

void func_Project(int &index, int2 *ids, real3 *fric, real* cohesion, real *gam) {
	int2 body_id = ids[index];
	real3 gamma;
	gamma.x = gam[_index_ + 0];
	gamma.y = gam[_index_ + 1];
	gamma.z = gam[_index_ + 2];

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

		gam[_index_ + 0] = gamma.x;
		gam[_index_ + 1] = gamma.y;
		gam[_index_ + 2] = gamma.z;

		return;
	}

	if (Cone_generalized(gamma.x, gamma.y, gamma.z, mu)) {
	}

	gam[_index_ + 0] = gamma.x - coh;
	gam[_index_ + 1] = gamma.y;
	gam[_index_ + 2] = gamma.z;

}
void func_Project_rolling(int &index, int2 *ids, real3 *fric, real *gam) {
	//real3 gamma_roll = R3(0);
	int2 body_id = ids[index];
	real3 f_a = fric[body_id.x];
	real3 f_b = fric[body_id.y];

	real rollingfriction = (f_a.y == 0 || f_b.y == 0) ? 0 : (f_a.y + f_b.y) * .5;
	real spinningfriction = (f_a.z == 0 || f_b.z == 0) ? 0 : (f_a.z + f_b.z) * .5;

//	if(rollingfriction||spinningfriction){
//		gam[index + number_of_contacts * 1] = 0;
//		gam[index + number_of_contacts * 2] = 0;
//	}

	real gamma_n = abs(gam[_index_ + 0]);
	real gamma_s = gam[_index_ + 3];
	real gamma_tu = gam[_index_ + 4];
	real gamma_tv = gam[_index_ + 5];

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
	gam[_index_ + 3] = gamma_s;
	gam[_index_ + 4] = gamma_tu;
	gam[_index_ + 5] = gamma_tv;

}

void ChConstraintRigidRigid::host_Project(int2 *ids, real3 *friction, real* cohesion, real *gamma) {
#pragma omp for
	for (int index = 0; index < number_of_rigid_rigid; index++) {
		//always project normal

		if (solve_sliding) {
			func_Project(index, ids, friction, cohesion, gamma);
		} else {
			real gamma_x = gamma[_index_ + 0];
			int2 body_id = ids[index];
			real coh = (cohesion[body_id.x] + cohesion[body_id.y]) * .5;
			if (coh < 0) {
				coh = 0;
			}
			gamma_x += coh;

			gamma_x = gamma_x < 0 ? 0 : gamma_x - coh;

			gamma[_index_ + 0] = gamma_x;
			gamma[_index_ + 1] = 0;
			gamma[_index_ + 2] = 0;

		}
		if (solve_spinning) {
			func_Project_rolling(index, ids, friction, gamma);
		}
	}
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidRigid::Project(real* gamma) {
	host_Project(data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.fric_data.data(), data_container->host_data.cohesion_data.data(), gamma);
}
void ChConstraintRigidRigid::Project_NoPar(real* gamma) {
	host_Project(data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.fric_data.data(), data_container->host_data.cohesion_data.data(), gamma);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void ChConstraintRigidRigid::host_RHS(int2 *ids, real *correction, real4 * compliance, bool * active, real3* norm, real3 *vel, real3 *omega, real3 *JUA, real3 *JUB, real3 *JVA,
		real3 *JVB, real3 *JWA, real3 *JWB, real *rhs) {

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
			temp.x = dot(-U, vel_b1) + dot(JUA[index], omega_b1);
			if (solve_sliding) {
				temp.y = dot(-V, vel_b1) + dot(JVA[index], omega_b1);
				temp.z = dot(-W, vel_b1) + dot(JWA[index], omega_b1);
			}
		}
		if (active[b2]) {
			real3 omega_b2 = omega[b2];
			real3 vel_b2 = vel[b2];
			temp.x += dot(U, vel_b2) + dot(JUB[index], omega_b2);
			if (solve_sliding) {
				temp.y += dot(V, vel_b2) + dot(JVB[index], omega_b2);
				temp.z += dot(W, vel_b2) + dot(JWB[index], omega_b2);
			}
		}
		real bi = 0;
		if (compliance[index].x) {
			bi = inv_hpa * correction[index];
		} else {
			bi = fmax(real(1.0) / step_size * correction[index], -contact_recovery_speed);
		}

		rhs[_index_ + 0] = -temp.x - bi;
		rhs[_index_ + 1] = -temp.y - 0;
		rhs[_index_ + 2] = -temp.z - 0;
	}
}

void ChConstraintRigidRigid::host_RHS_spinning(int2 *ids, bool * active, real3 *omega, real3 *JTA, real3 *JTB, real3 *JSA, real3 *JSB, real3 *JRA, real3 *JRB, real *rhs) {

#pragma omp parallel for
	for (int index = 0; index < number_of_rigid_rigid; index++) {
		uint b1 = ids[index].x;
		uint b2 = ids[index].y;
		real3 temp = R3(0);

		if (solve_spinning) {
			if (active[b1]) {
				real3 omega_b1 = omega[b1];
				temp.x = dot(JTA[index], omega_b1);
				temp.y = dot(JSA[index], omega_b1);
				temp.z = dot(JRA[index], omega_b1);
			}
			if (active[b2]) {
				real3 omega_b2 = omega[b2];
				temp.x += dot(JTB[index], omega_b2);
				temp.y += dot(JSB[index], omega_b2);
				temp.z += dot(JRB[index], omega_b2);
			}
		}
		rhs[_index_ + 3] = -temp.x;
		rhs[_index_ + 4] = -temp.y;
		rhs[_index_ + 5] = -temp.z;
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
	host_RHS(data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.dpth_rigid_rigid.data(), comp_rigid_rigid.data(),
			data_container->host_data.active_data.data(), data_container->host_data.norm_rigid_rigid.data(), data_container->host_data.vel_data.data(),
			data_container->host_data.omg_data.data(), JUA_rigid_rigid.data(), JUB_rigid_rigid.data(), JVA_rigid_rigid.data(), JVB_rigid_rigid.data(), JWA_rigid_rigid.data(),
			JWB_rigid_rigid.data(), data_container->host_data.rhs_data.data());

	host_RHS_spinning(data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.active_data.data(), data_container->host_data.omg_data.data(),
			JTA_rigid_rigid.data(), JTB_rigid_rigid.data(), JSA_rigid_rigid.data(), JSB_rigid_rigid.data(), JRA_rigid_rigid.data(), JRB_rigid_rigid.data(),
			data_container->host_data.rhs_data.data());
}

void ChConstraintRigidRigid::UpdateRHS() {
	host_RHS(data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.dpth_rigid_rigid.data(), comp_rigid_rigid.data(),
			data_container->host_data.active_data.data(), data_container->host_data.norm_rigid_rigid.data(),

			data_container->host_data.vel_new_data.data(), data_container->host_data.omg_new_data.data(), JUA_rigid_rigid.data(), JUB_rigid_rigid.data(), JVA_rigid_rigid.data(),
			JVB_rigid_rigid.data(), JWA_rigid_rigid.data(), JWB_rigid_rigid.data(), data_container->host_data.rhs_data.data());

	host_RHS_spinning(data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.active_data.data(), data_container->host_data.omg_new_data.data(),
			JTA_rigid_rigid.data(), JTB_rigid_rigid.data(), JSA_rigid_rigid.data(), JSB_rigid_rigid.data(), JRA_rigid_rigid.data(), JRB_rigid_rigid.data(),
			data_container->host_data.rhs_data.data());

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

void inline Compute_Jacobian(const real4& quaternion_rotation, const real3& normal, const real3& tangent_u, const real3& tangent_w, const real3& point, real3& T1, real3& T2,
		real3& T3) {
	real4 quaternion_conjugate = ~quaternion_rotation;

	real3 sbar = quatRotate(point, quaternion_conjugate);

	T1 = cross(quatRotate(normal, quaternion_conjugate), sbar);
	T2 = cross(quatRotate(tangent_u, quaternion_conjugate), sbar);
	T3 = cross(quatRotate(tangent_w, quaternion_conjugate), sbar);

}

void inline Compute_Jacobian_Rolling(const real4& quaternion_rotation, const real3& normal, const real3& tangent_u, const real3& tangent_w, real3& T1, real3& T2, real3& T3) {

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
void ChConstraintRigidRigid::host_Jacobians(real3* norm, real3* ptA, real3* ptB, int2* ids, real4* rot, real3* JUA, real3* JUB, real3* JVA, real3* JVB, real3* JWA, real3* JWB) {
#pragma omp parallel for
	for (int index = 0; index < number_of_rigid_rigid; index++) {
		real3 U = norm[index], V, W;
		Orthogonalize(U, V, W);

		int2 body_id = ids[index];

		real3 T3, T4, T5, T6, T7, T8;
		Compute_Jacobian(rot[body_id.x], U, V, W, ptA[index], T3, T4, T5);
		Compute_Jacobian(rot[body_id.y], U, V, W, ptB[index], T6, T7, T8);
		T6 = -T6;
		T7 = -T7;
		T8 = -T8;

		JUA[index] = T3;
		JVA[index] = T4;
		JWA[index] = T5;

		JUB[index] = T6;
		JVB[index] = T7;
		JWB[index] = T8;
	}
}

void ChConstraintRigidRigid::host_Jacobians_Rolling(real3* norm, int2* ids, real4* rot, real3 *JTA, real3 *JTB, real3 *JSA, real3 *JSB, real3 *JRA, real3 *JRB) {
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

		if (mzlen < 0.0001) {     // was near singularity? change singularity reference custom_vector!
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

		JTA[index] = T3;
		JSA[index] = T4;
		JRA[index] = T5;

		JTB[index] = T6;
		JSB[index] = T7;
		JRB[index] = T8;
	}
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidRigid::ComputeJacobians() {
	JUA_rigid_rigid.resize(number_of_rigid_rigid);
	JUB_rigid_rigid.resize(number_of_rigid_rigid);

	JVA_rigid_rigid.resize(number_of_rigid_rigid);
	JVB_rigid_rigid.resize(number_of_rigid_rigid);

	JWA_rigid_rigid.resize(number_of_rigid_rigid);
	JWB_rigid_rigid.resize(number_of_rigid_rigid);

	JTA_rigid_rigid.resize(number_of_rigid_rigid);
	JTB_rigid_rigid.resize(number_of_rigid_rigid);

	JSA_rigid_rigid.resize(number_of_rigid_rigid);
	JSB_rigid_rigid.resize(number_of_rigid_rigid);

	JRA_rigid_rigid.resize(number_of_rigid_rigid);
	JRB_rigid_rigid.resize(number_of_rigid_rigid);

	host_Jacobians(data_container->host_data.norm_rigid_rigid.data(), data_container->host_data.cpta_rigid_rigid.data(), data_container->host_data.cptb_rigid_rigid.data(),
			data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.rot_data.data(), JUA_rigid_rigid.data(), JUB_rigid_rigid.data(), JVA_rigid_rigid.data(),
			JVB_rigid_rigid.data(), JWA_rigid_rigid.data(), JWB_rigid_rigid.data());

	host_Jacobians_Rolling(data_container->host_data.norm_rigid_rigid.data(), data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.rot_data.data(),
			JTA_rigid_rigid.data(), JTB_rigid_rigid.data(), JSA_rigid_rigid.data(), JSB_rigid_rigid.data(), JRA_rigid_rigid.data(), JRB_rigid_rigid.data());

}

void ChConstraintRigidRigid::UpdateJacobians() {

	JUA_rigid_rigid.resize(number_of_rigid_rigid);
	JUB_rigid_rigid.resize(number_of_rigid_rigid);

	JVA_rigid_rigid.resize(number_of_rigid_rigid);
	JVB_rigid_rigid.resize(number_of_rigid_rigid);

	JWA_rigid_rigid.resize(number_of_rigid_rigid);
	JWB_rigid_rigid.resize(number_of_rigid_rigid);

	JTA_rigid_rigid.resize(number_of_rigid_rigid);
	JTB_rigid_rigid.resize(number_of_rigid_rigid);

	JSA_rigid_rigid.resize(number_of_rigid_rigid);
	JSB_rigid_rigid.resize(number_of_rigid_rigid);

	JRA_rigid_rigid.resize(number_of_rigid_rigid);
	JRB_rigid_rigid.resize(number_of_rigid_rigid);
	host_Jacobians(data_container->host_data.norm_rigid_rigid.data(), data_container->host_data.cpta_rigid_rigid.data(), data_container->host_data.cptb_rigid_rigid.data(),
			data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.rot_new_data.data(), JUA_rigid_rigid.data(), JUB_rigid_rigid.data(),
			JVA_rigid_rigid.data(), JVB_rigid_rigid.data(), JWA_rigid_rigid.data(), JWB_rigid_rigid.data());

	host_Jacobians_Rolling(data_container->host_data.norm_rigid_rigid.data(), data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.rot_new_data.data(),
			JTA_rigid_rigid.data(), JTB_rigid_rigid.data(), JSA_rigid_rigid.data(), JSB_rigid_rigid.data(), JRA_rigid_rigid.data(), JRB_rigid_rigid.data());

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidRigid::host_shurA_normal(real * gamma, real3* norm, real3 * JUA, real3 * JUB, real3 * updateV, real3 * updateO) {
//	double start = omp_get_wtime();
#pragma omp  for schedule(static, 4)
	for (int index = 0; index < number_of_rigid_rigid; index++) {
		real gam = gamma[index * 6];
		real3 U = norm[index];
		updateV[index] = -U * gam;
		//updateV[index + number_of_rigid_rigid] = U * gam;
		updateO[index] = JUA[index] * gam;
		updateO[index + number_of_rigid_rigid] = JUB[index] * gam;

	}
//	double end = omp_get_wtime();
//
//	double total_time_omp = (end - start) * 1000;
//	double total_flops = 12 * number_of_rigid_rigid / ((end - start)) / 1e9;
//	double total_memory = (7 * 4 * 4 + 1 * 4) * number_of_rigid_rigid / ((end - start)) / 1024.0 / 1024.0 / 1024.0;
//
//	cout<<"SA_STAT: "<<total_time_omp<<" "<<total_flops<<" "<<total_memory<<endl;

}

void ChConstraintRigidRigid::host_shurA_sliding(bool2* contact_active, real3* norm, real3 * JUA, real3 * JUB, real3 * JVA, real3 * JVB, real3 * JWA, real3 * JWB, real * gamma,
		real3 * updateV, real3 * updateO) {
#pragma omp  for schedule(static, 4)
	for (size_t index = 0; index < number_of_rigid_rigid; index++) {
		real3 gam(_mm_loadu_ps(&gamma[_index_]));
		real3 U = norm[index], V, W;
		Orthogonalize(U, V, W);
		bool2 active = contact_active[index];
		updateV[index] = -U * gam.x - V * gam.y - W * gam.z;
		if (active.x != 0) {
			updateO[index] = JUA[index] * gam.x + JVA[index] * gam.y + JWA[index] * gam.z;
		}
		if (active.y != 0) {
			//updateV[index + number_of_rigid_rigid] = U * gam.x + V * gam.y + W * gam.z;
			updateO[index + number_of_rigid_rigid] = JUB[index] * gam.x + JVB[index] * gam.y + JWB[index] * gam.z;

		}
	}

}

void ChConstraintRigidRigid::host_shurA_spinning(bool2* contact_active, real3* norm, real3 * JUA, real3 * JUB, real3 * JVA, real3 * JVB, real3 * JWA, real3 * JWB, real3 * JTA,
		real3 * JTB, real3 * JSA, real3 * JSB, real3 * JRA, real3 * JRB, real * gamma, real3 * updateV, real3 * updateO) {
#pragma omp  for schedule(static, 4)
	for (int index = 0; index < number_of_rigid_rigid; index++) {

		real3 gam(_mm_loadu_ps(&gamma[_index_]));
		real3 gam_roll(_mm_loadu_ps(&gamma[_index_ + 3]));

		real3 U = norm[index], V, W;
		Orthogonalize(U, V, W);
		bool2 active = contact_active[index];
		updateV[index] = -U * gam.x - V * gam.y - W * gam.z;
		if (active.x != 0) {
			updateO[index] = JUA[index] * gam.x + JVA[index] * gam.y + JWA[index] * gam.z + JTA[index] * gam_roll.x + JSA[index] * gam_roll.y + JRA[index] * gam_roll.z;
		}
		if (active.y != 0) {
			//updateV[index + number_of_rigid_rigid] = U * gam.x + V * gam.y + W * gam.z;
			updateO[index + number_of_rigid_rigid] = JUB[index] * gam.x + JVB[index] * gam.y + JWB[index] * gam.z + JTB[index] * gam_roll.x + JSB[index] * gam_roll.y
					+ JRB[index] * gam_roll.z;
		}
	}
}

void ChConstraintRigidRigid::host_shurB_normal(int2 * ids, bool2* contact_active, real3* norm, real4 * compliance, real * gamma, real3 * JUA, real3 * JUB, real3 * QXYZ,
		real3 * QUVW, real * AX) {

#pragma omp  for schedule(static, 4)
	for (size_t index = 0; index < number_of_rigid_rigid; index++) {
		real3 temp = R3(0);

		int2 id_ = ids[index];
		real3 XYZ, UVW;
		real3 U = norm[index];
		bool2 active = contact_active[index];
		if (active.x != 0) {
			XYZ = QXYZ[id_.x];
			UVW = QUVW[id_.x];
			temp.x = dot(XYZ, -U) + dot(UVW, JUA[index]);
		}
		if (active.y != 0) {
			XYZ = QXYZ[id_.y];
			UVW = QUVW[id_.y];
			temp.x += dot(XYZ, U) + dot(UVW, JUB[index]);
		}

		real4 comp = compliance[index];

		AX[_index_ + 0] = temp.x + gamma[_index_] * comp.x;
//		_mm_storeu_ps(&AX[_index_ + 1], _mm_setzero_ps());
//		AX[_index_ + 5] = 0;

	}
}
void ChConstraintRigidRigid::host_shurB_sliding(int2 * ids, bool2* contact_active, real3* norm, real4 * compliance, real * gamma, real3 * JUA, real3 * JUB, real3 * JVA,
		real3 * JVB, real3 * JWA, real3 * JWB, real3 * QXYZ, real3 * QUVW, real * AX) {

#pragma omp  for schedule(static, 4)
	for (int index = 0; index < number_of_rigid_rigid; index++) {
		real3 temp = R3(0);

		int2 id_ = ids[index];
		uint b1 = id_.x;
		uint b2 = id_.y;
		real3 XYZ, UVW;
		real3 U = norm[index], V, W;
		Orthogonalize(U, V, W);

		bool2 active = contact_active[index];
		if (active.x != 0) {
			XYZ = QXYZ[b1];
			UVW = QUVW[b1];

			temp.x = dot(XYZ, -U) + dot(UVW, JUA[index]);
			temp.y = dot(XYZ, -V) + dot(UVW, JVA[index]);
			temp.z = dot(XYZ, -W) + dot(UVW, JWA[index]);
		}

		if (active.y != 0) {
			XYZ = QXYZ[b2];
			UVW = QUVW[b2];

			temp.x += dot(XYZ, U) + dot(UVW, JUB[index]);
			temp.y += dot(XYZ, V) + dot(UVW, JVB[index]);
			temp.z += dot(XYZ, W) + dot(UVW, JWB[index]);

		}

		real4 comp = compliance[index];
		real3 gam(_mm_loadu_ps(&gamma[_index_]));

		AX[_index_ + 0] = temp.x + gam.x * comp.x;
		AX[_index_ + 1] = temp.y + gam.y * comp.y;
		AX[_index_ + 2] = temp.z + gam.z * comp.y;
//		AX[_index_ + 3] = 0;
//		AX[_index_ + 4] = 0;
//		AX[_index_ + 5] = 0;

	}
}
void ChConstraintRigidRigid::host_shurB_spinning(int2 * ids, bool2* contact_active, real3* norm, real4 * compliance, real * gamma, real3 * JUA, real3 * JUB, real3 * JVA,
		real3 * JVB, real3 * JWA, real3 * JWB, real3 * JTA, real3 * JTB, real3 * JSA, real3 * JSB, real3 * JRA, real3 * JRB, real3 * QXYZ, real3 * QUVW, real * AX) {

#pragma omp  for schedule(static, 4)
	for (int index = 0; index < number_of_rigid_rigid; index++) {
		real3 temp = R3(0);
		real3 temp_roll = R3(0);

		int2 id_ = ids[index];
		uint b1 = id_.x;
		uint b2 = id_.y;

		real3 U = norm[index], V, W;
		Orthogonalize(U, V, W);
		bool2 active = contact_active[index];
		if (active.x != 0) {
			real3 XYZ = QXYZ[b1];
			real3 UVW = QUVW[b1];

			temp.x += dot(XYZ, -U);
			temp.x += dot(UVW, JUA[index]);

			temp.y += dot(XYZ, -V);
			temp.y += dot(UVW, JVA[index]);

			temp.z += dot(XYZ, -W);
			temp.z += dot(UVW, JWA[index]);

			temp_roll.x += dot(UVW, JTA[index]);
			temp_roll.y += dot(UVW, JSA[index]);
			temp_roll.z += dot(UVW, JRA[index]);

		}
		if (active.y != 0) {
			real3 XYZ = QXYZ[b2];
			real3 UVW = QUVW[b2];

			temp.x += dot(XYZ, U);
			temp.x += dot(UVW, JUB[index]);

			temp.y += dot(XYZ, V);
			temp.y += dot(UVW, JVB[index]);

			temp.z += dot(XYZ, W);
			temp.z += dot(UVW, JWB[index]);

			temp_roll.x += dot(UVW, JTB[index]);
			temp_roll.y += dot(UVW, JSB[index]);
			temp_roll.z += dot(UVW, JRB[index]);

		}
		real4 comp = compliance[index];
		real3 gam(_mm_loadu_ps(&gamma[_index_]));
		real3 gam_roll(_mm_loadu_ps(&gamma[_index_ + 3]));
		AX[_index_ + 0] = temp.x + gam.x * comp.x;
		AX[_index_ + 1] = temp.y + gam.y * comp.y;
		AX[_index_ + 2] = temp.z + gam.z * comp.y;

		AX[_index_ + 3] = temp_roll.x + gam.x * comp.w;
		AX[_index_ + 4] = temp_roll.y + gam.y * comp.z;
		AX[_index_ + 5] = temp_roll.z + gam.z * comp.z;

	}
}

void ChConstraintRigidRigid::host_Reduce_Shur(bool* active, real3* QXYZ, real3* QUVW, real * inv_mass, real3 * inv_inertia, real3* updateQXYZ, real3* updateQUVW, int* d_body_num,
		int* counter, int* reverse_offset) {
#pragma omp  for
	for (int index = 0; index < number_of_updates; index++) {

		int start = (index == 0) ? 0 : counter[index - 1], end = counter[index];
		int id = d_body_num[end - 1], j;

		if (active[id] != 0) {

			real3 mUpdateV = R3(0);
			real3 mUpdateO = R3(0);

			for (j = 0; j < end - start; j++) {
				int contact_num = reverse_offset[j + start];
				if (contact_num >= number_of_rigid_rigid) {
					mUpdateV += -updateQXYZ[contact_num - number_of_rigid_rigid];
				} else {
					mUpdateV += updateQXYZ[contact_num];
				}
				mUpdateO += updateQUVW[contact_num];
			}
			QXYZ[id] = mUpdateV * inv_mass[id];
			QUVW[id] = mUpdateO * inv_inertia[id];
		}
	}
}

void ChConstraintRigidRigid::host_Offsets(int2* ids, int* Body) {
#pragma omp parallel for
	for (int index = 0; index < number_of_rigid_rigid; index++) {
		if (index < number_of_rigid_rigid) {
			int2 temp_id = ids[index];
			Body[index] = temp_id.x;
			Body[index + number_of_rigid_rigid] = temp_id.y;
		}
	}
}

void ChConstraintRigidRigid::ShurA(real* x) {

	if (solve_spinning) {
#pragma omp master
		{
			data_container->system_timer.SetMemory("shurA_spinning", 19 * 4 * 4 * number_of_rigid_rigid);
			data_container->system_timer.start("shurA_spinning");
		}
		host_shurA_spinning(contact_active_pairs.data(), data_container->host_data.norm_rigid_rigid.data(),
		JUA_rigid_rigid.data(), JUB_rigid_rigid.data(), JVA_rigid_rigid.data(), JVB_rigid_rigid.data(), JWA_rigid_rigid.data(), JWB_rigid_rigid.data(), JTA_rigid_rigid.data(),
				JTB_rigid_rigid.data(), JSA_rigid_rigid.data(), JSB_rigid_rigid.data(), JRA_rigid_rigid.data(), JRB_rigid_rigid.data(), x, vel_update.data(), omg_update.data());
#pragma omp master
		{
			data_container->system_timer.stop("shurA_spinning");
		}
	} else if (solve_sliding) {
#pragma omp master
		{
			data_container->system_timer.SetMemory("shurA_sliding", 12 * 4 * 4 * number_of_rigid_rigid);
			data_container->system_timer.start("shurA_sliding");
		}
		host_shurA_sliding(contact_active_pairs.data(), data_container->host_data.norm_rigid_rigid.data(), JUA_rigid_rigid.data(), JUB_rigid_rigid.data(), JVA_rigid_rigid.data(),
				JVB_rigid_rigid.data(), JWA_rigid_rigid.data(), JWB_rigid_rigid.data(), x, vel_update.data(), omg_update.data());
#pragma omp master
		{
			data_container->system_timer.stop("shurA_sliding");
		}
	} else {
#pragma omp master
		{
			data_container->system_timer.SetMemory("shurA_normal", (6 * 4 * 4 + 1 * 4) * number_of_rigid_rigid);
			data_container->system_timer.start("shurA_normal");
		}
		host_shurA_normal(x, data_container->host_data.norm_rigid_rigid.data(), JUA_rigid_rigid.data(), JUB_rigid_rigid.data(), vel_update.data(), omg_update.data());
#pragma omp master
		{
			data_container->system_timer.stop("shurA_normal");
		}
	}
#pragma omp master
		{
			data_container->system_timer.start("shurA_reduce");
		}
	host_Reduce_Shur(data_container->host_data.active_data.data(), data_container->host_data.QXYZ_data.data(), data_container->host_data.QUVW_data.data(),
			data_container->host_data.mass_data.data(), data_container->host_data.inr_data.data(), vel_update.data(), omg_update.data(), body_number.data(), offset_counter.data(),
			update_offset_bodies.data());
#pragma omp master
		{
			data_container->system_timer.stop("shurA_reduce");
		}
}
void ChConstraintRigidRigid::ShurB(real*x, real* output) {

	if (solve_spinning) {

		host_shurB_spinning(data_container->host_data.bids_rigid_rigid.data(), contact_active_pairs.data(), data_container->host_data.norm_rigid_rigid.data(),
				comp_rigid_rigid.data(), x, JUA_rigid_rigid.data(), JUB_rigid_rigid.data(), JVA_rigid_rigid.data(), JVB_rigid_rigid.data(), JWA_rigid_rigid.data(),
				JWB_rigid_rigid.data(), JTA_rigid_rigid.data(), JTB_rigid_rigid.data(), JSA_rigid_rigid.data(), JSB_rigid_rigid.data(), JRA_rigid_rigid.data(),
				JRB_rigid_rigid.data(), data_container->host_data.QXYZ_data.data(), data_container->host_data.QUVW_data.data(), output);

	} else if (solve_sliding) {

		host_shurB_sliding(data_container->host_data.bids_rigid_rigid.data(), contact_active_pairs.data(), data_container->host_data.norm_rigid_rigid.data(),
				comp_rigid_rigid.data(), x, JUA_rigid_rigid.data(), JUB_rigid_rigid.data(), JVA_rigid_rigid.data(), JVB_rigid_rigid.data(), JWA_rigid_rigid.data(),
				JWB_rigid_rigid.data(), data_container->host_data.QXYZ_data.data(), data_container->host_data.QUVW_data.data(), output);

	} else {

		host_shurB_normal(data_container->host_data.bids_rigid_rigid.data(), contact_active_pairs.data(), data_container->host_data.norm_rigid_rigid.data(),
				comp_rigid_rigid.data(), x, JUA_rigid_rigid.data(), JUB_rigid_rigid.data(), data_container->host_data.QXYZ_data.data(), data_container->host_data.QUVW_data.data(),
				output);

	}
}

void ChConstraintRigidRigid::Diag() {
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
