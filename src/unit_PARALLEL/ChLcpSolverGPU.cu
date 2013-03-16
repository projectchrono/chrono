#include "ChLcpSolverGPU.h"
#include "ChThrustLinearAlgebra.cuh"

using namespace chrono;

__constant__ real lcp_omega_bilateral_const;
__constant__ real lcp_omega_contact_const;
__constant__ real lcp_contact_factor_const;
__constant__ uint number_of_objects_const;
__constant__ uint number_of_contacts_const;
__constant__ uint number_of_bilaterals_const;
__constant__ uint number_of_updates_const;
__constant__ real step_size_const;
__constant__ real compliance_const;
__constant__ real complianceT_const;
__constant__ real alpha_const; // [R]=alpha*[K]

//__constant__ real force_factor_const;
//__constant__ real negated_recovery_speed_const;
////////////////////////////////////////////////////////////////////////////////////////////////////

__host__ __device__ void inline Compute_Jacobian(const real4& quaternion_rotation, const real3& normal, const real3& tangent_u, const real3& tangent_w, const real3& point, real3& T3, real3& T4,
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
__host__ __device__ void inline function_ContactJacobians(uint& index, uint& num_contacts, real3* norm, real3* ptA, real3* ptB, real3* pos, real4* rot, int2* ids, real3* JXYZA, real3* JXYZB,
		real3* JUVWA, real3* JUVWB) {
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
void ChLcpSolverGPU::host_ContactJacobians(real3* norm, real3* ptA, real3* ptB, int2* ids, real4* rot, real3* pos, real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB) {
	// #pragma omp parallel for schedule(guided)
	for (uint index = 0; index < number_of_contacts; index++) {
		function_ContactJacobians(index, number_of_contacts, norm, ptA, ptB, pos, rot, ids, JXYZA, JXYZB, JUVWA, JUVWB);
	}
}
__host__ __device__ void function_shurA(uint& index, real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB, real * gamma, int2* ids, real3 * QXYZ, real3 * QUVW) {
	real q_x, q_y, q_z, q_u, q_v, q_w;
	real temp = 0, m;
	uint b1 = ids[index].x;
	uint b2 = ids[index].y;

	if (index != b1) {
		q_x = JXYZA[index].x * gamma[index];
		q_y = JXYZA[index].y * gamma[index];
		q_z = JXYZA[index].z * gamma[index];
		q_u = JUVWA[index].x * gamma[index];
		q_v = JUVWA[index].y * gamma[index];
		q_w = JUVWA[index].z * gamma[index];

		QXYZ[b1] += R3(q_x, q_y, q_z);
		QUVW[b1] += R3(q_u, q_v, q_w);

	}

	if (index != b2) {
		q_x = JXYZB[index].x * gamma[index];
		q_y = JXYZB[index].y * gamma[index];
		q_z = JXYZB[index].z * gamma[index];
		q_u = JUVWB[index].x * gamma[index];
		q_v = JUVWB[index].y * gamma[index];
		q_w = JUVWB[index].z * gamma[index];

		QXYZ[b2] += R3(q_x, q_y, q_z);
		QUVW[b2] += R3(q_u, q_v, q_w);
	}
}
__host__ __device__ void function_shurB(uint& index, real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB, real * gamma, real * inv_mass, real3 * inv_inertia, int2* ids, real3 * QXYZ, real3 * QUVW,
		real * AX) {
	real temp = 0, m;
	uint b1 = ids[index].x;
	uint b2 = ids[index].y;
	real m1 = inv_mass[b1];
	real m2 = inv_mass[b2];

	real3 inertia1 = inv_inertia[b1];
	real3 inertia2 = inv_inertia[b2];
	if (index != b1) {
		temp += QXYZ[b1].x * JXYZA[index].x * m1;
		temp += QXYZ[b1].y * JXYZA[index].y * m1;
		temp += QXYZ[b1].z * JXYZA[index].z * m1;
		temp += QUVW[b1].x * JUVWA[index].x * inertia1.x;
		temp += QUVW[b1].y * JUVWA[index].y * inertia1.y;
		temp += QUVW[b1].z * JUVWA[index].z * inertia1.z;
	}

	if (index != b2) {
		temp += QXYZ[b2].x * JXYZB[index].x * m1;
		temp += QXYZ[b2].y * JXYZB[index].y * m1;
		temp += QXYZ[b2].z * JXYZB[index].z * m1;
		temp += QUVW[b2].x * JUVWB[index].x * inertia1.x;
		temp += QUVW[b2].y * JUVWB[index].y * inertia1.y;
		temp += QUVW[b2].z * JUVWB[index].z * inertia1.z;
	}

	AX[index] += temp;
}
__host__ __device__ void function_RHS(uint &index, int2* ids, real * inv_mass, real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB, real3* vel, real3* omega, real* correction, real & step_size,
		real * rhs) {
	uint b1 = ids[index].x;
	uint b2 = ids[index].y;
	real m1 = inv_mass[b1];
	real m2 = inv_mass[b2];

	real temp = 0;
	temp -= JXYZA[index].x * vel[b1].x;
	temp -= JXYZA[index].y * vel[b1].y;
	temp -= JXYZA[index].z * vel[b1].z;
	temp -= JUVWA[index].x * omega[b1].x;
	temp -= JUVWA[index].y * omega[b1].y;
	temp -= JUVWA[index].z * omega[b1].z;

	temp -= JXYZB[index].x * vel[b2].x;
	temp -= JXYZB[index].y * vel[b2].y;
	temp -= JXYZB[index].z * vel[b2].z;
	temp -= JUVWB[index].x * omega[b2].x;
	temp -= JUVWB[index].y * omega[b2].y;
	temp -= JUVWB[index].z * omega[b2].z;

	rhs[index] = temp - fmaxf(1 / step_size * correction[index], 0);

}
__host__ __device__ void function_InvNDiag(uint index, int b1, int b2,int2* ids, real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB, real * inv_mass, real3 * inv_inertia, real & inv_n_diag) {

	real Jx1 = JXYZA[index].x;
	real Jy1 = JXYZA[index].y;
	real Jz1 = JXYZA[index].z;
	real Ju1 = JUVWA[index].x;
	real Jv1 = JUVWA[index].y;
	real Jw1 = JUVWA[index].z;
	//
	real Jx2 = JXYZB[index].x;
	real Jy2 = JXYZB[index].y;
	real Jz2 = JXYZB[index].z;
	real Ju2 = JUVWB[index].x;
	real Jv2 = JUVWB[index].y;
	real Jw2 = JUVWB[index].z;
	//
	real m1 = inv_mass[b1];
	real m2 = inv_mass[b2];
	//
	real3 i1 = inv_inertia[b1];
	real3 i2 = inv_inertia[b2];

	real part1 = dot(JXYZA[index], JXYZA[index]) * m1;
	real part2 = dot(JUVWA[index] * JUVWA[index], i1);
	real part3 = dot(JXYZB[index], JXYZB[index]) * m2;
	real part4 = dot(JUVWB[index] * JUVWB[index], i2);
	inv_n_diag = (part1 + part2 + part3 + part4);
}
__host__ __device__ void function_Project(uint & index, real3 & gamma, real* fric, int2* ids) {
	int2 body_id = ids[index];

	real f_tang = sqrtf(gamma.y * gamma.y + gamma.z * gamma.z);
	real mu = (fric[body_id.x] + fric[body_id.y]) * .5f;

	if (f_tang > (mu * gamma.x)) { // inside upper cone? keep untouched!
		if ((f_tang) < -(1.0 / mu) * gamma.x || (fabsf(gamma.x) < 0)) { // inside lower cone? reset  normal,u,v to zero!
			gamma = R3(0);
		} else { // remaining case: project orthogonally to generator segment of upper cone
			gamma.x = (f_tang * mu + gamma.x) / (mu * mu + 1);
			real tproj_div_t = (gamma.x * mu) / f_tang; //  reg = tproj_div_t
			gamma.y *= tproj_div_t;
			gamma.z *= tproj_div_t;
		}
	} else if (mu == 0) {
		gamma.y = gamma.z = 0;
	}

}

__host__ __device__ void function_process_contacts(uint &index, real& step_size, real& lcp_contact_factor, uint& number_of_contacts, real& lcp_omega_contact, real3* JXYZA, real3* JXYZB, real3* JUVWA,
		real3* JUVWB, real* contactDepth, int2* ids, real3* G, real* dG, real* inv_mass, real* fric, real3* inv_inertia, real4* rot, real3* vel, real3* omega, real3* pos, real3* updateV,
		real3* updateO, uint* offset) {

	int2 body_id = ids[index];
	real depth = -fabs(contactDepth[index]);

	real bi = depth / (step_size);//fmaxf((), -lcp_contact_factor);
	real3 W1 = omega[body_id.x];
	real3 W2 = omega[body_id.y];
	real3 V1 = vel[body_id.x];
	real3 V2 = vel[body_id.y];
	real3 gamma;
	gamma.x = dot(JUVWA[index + number_of_contacts * 0], W1) + dot(JXYZA[index + number_of_contacts * 0], V1) + dot(JUVWB[index + number_of_contacts * 0], W2)
			+ dot(JXYZB[index + number_of_contacts * 0], V2) + bi /*+ cfm * gamma_old.x*/; //+bi

	gamma.y = dot(JUVWA[index + number_of_contacts * 1], W1) + dot(JXYZA[index + number_of_contacts * 1], V1) + dot(JUVWB[index + number_of_contacts * 1], W2)
			+ dot(JXYZB[index + number_of_contacts * 1], V2) /*+ cfmT * gamma_old.y*/;

	gamma.z = dot(JUVWA[index + number_of_contacts * 2], W1) + dot(JXYZA[index + number_of_contacts * 2], V1) + dot(JUVWB[index + number_of_contacts * 2], W2)
			+ dot(JXYZB[index + number_of_contacts * 2], V2) /*+ cfmT * gamma_old.z*/;
	real3 In1 = inv_inertia[body_id.x]; // bring in the inertia attributes; to be used to compute \eta
	real3 In2 = inv_inertia[body_id.y]; // bring in the inertia attributes; to be used to compute \eta

  	real eta =
			dot(JUVWA[index + number_of_contacts * 0] * JUVWA[index + number_of_contacts * 0], In1) +
			dot(JUVWA[index + number_of_contacts * 1] * JUVWA[index + number_of_contacts * 1], In1)+
			dot(JUVWA[index + number_of_contacts * 2] * JUVWA[index + number_of_contacts * 2], In1); // update expression of eta

	eta += dot(JUVWB[index + number_of_contacts * 0] * JUVWB[index + number_of_contacts * 0], In2) +
		   dot(JUVWB[index + number_of_contacts * 1] * JUVWB[index + number_of_contacts * 1], In2)+
		   dot(JUVWB[index + number_of_contacts * 2] * JUVWB[index + number_of_contacts * 2], In2);

	eta += dot(JXYZA[index + number_of_contacts * 0], JXYZA[index + number_of_contacts * 0])* inv_mass[body_id.x] +
			dot(JXYZA[index + number_of_contacts * 1], JXYZA[index + number_of_contacts * 1])* inv_mass[body_id.x]+
			dot(JXYZA[index + number_of_contacts * 2], JXYZA[index + number_of_contacts * 2])* inv_mass[body_id.x] ;

	eta += dot(JXYZB[index + number_of_contacts * 0], JXYZB[index + number_of_contacts * 0])* inv_mass[body_id.y] +
			dot(JXYZB[index + number_of_contacts * 1], JXYZB[index + number_of_contacts * 1])* inv_mass[body_id.y]+
			dot(JXYZB[index + number_of_contacts * 2], JXYZB[index + number_of_contacts * 2])* inv_mass[body_id.y] ;

	eta = 3*lcp_omega_contact / eta; // final value of eta
	real3 gamma_old = G[index];
	gamma = eta * gamma; // perform gamma *= omega*eta
	gamma = gamma_old - gamma; // perform gamma = gamma_old - gamma ;  in place.
	/// ---- perform projection of 'a8' onto friction cone  --------

	function_Project(index, gamma, fric, ids);

	G[index] = gamma; // store gamma_new
	gamma -= gamma_old; // compute delta_gamma = gamma_new - gamma_old   = delta_gamma.
	dG[index] = length(gamma);
	real3 vB1 = JXYZA[index + number_of_contacts * 0] * gamma.x + JXYZA[index + number_of_contacts * 1] * gamma.y + JXYZA[index + number_of_contacts * 2] * gamma.z;
	real3 vB2 = JXYZB[index + number_of_contacts * 0] * gamma.x + JXYZB[index + number_of_contacts * 1] * gamma.y + JXYZB[index + number_of_contacts * 2] * gamma.z;

	int offset1 = offset[index];
	int offset2 = offset[index + number_of_contacts];
	updateV[offset1] = real3(vB1 * inv_mass[body_id.x]); // compute and store dv1
	updateO[offset1] = real3((JUVWA[index + number_of_contacts * 0] * gamma.x + JUVWA[index + number_of_contacts * 1] * gamma.y + JUVWA[index + number_of_contacts * 2] * gamma.z) * In1); // compute dw1 =  Inert.1' * J1w^ * deltagamma  and store  dw1
	updateV[offset2] = real3(vB2 * inv_mass[body_id.y]); // compute and store dv2
	updateO[offset2] = real3((JUVWB[index + number_of_contacts * 0] * gamma.x + JUVWB[index + number_of_contacts * 1] * gamma.y + JUVWB[index + number_of_contacts * 2] * gamma.z) * In2); // compute dw2 =  Inert.2' * J2w^ * deltagamma  and store  dw2
}

//  Kernel for a single iteration of the LCP over all contacts
//  Version 2.0 - Tasora
//  Version 2.2- Hammad (optimized, cleaned etc)
__global__ void device_process_contacts(real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB, real* contactDepth, int2* ids, real3* G, real* dG, real* mass, real* fric, real3* inertia, real4* rot,
		real3* vel, real3* omega, real3* pos, real3* updateV, real3* updateO, uint* offset) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_contacts_const);
	function_process_contacts(index, step_size_const, lcp_contact_factor_const, number_of_contacts_const, lcp_omega_contact_const, JXYZA, JXYZB, JUVWA, JUVWB, contactDepth, ids, G, dG, mass, fric,
			inertia, rot, vel, omega, pos, updateV, updateO, offset);
}

void ChLcpSolverGPU::host_process_contacts(real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB, real* contactDepth, int2* ids, real3* G, real* dG, real* mass, real* fric, real3* inertia,
		real4* rot, real3* vel, real3* omega, real3* pos, real3* updateV, real3* updateO, uint* offset) {
#pragma omp parallel for schedule(guided)

	for (uint index = 0; index < number_of_contacts; index++) {
		function_process_contacts(index, step_size, lcp_contact_factor, number_of_contacts, lcp_omega_contact, JXYZA, JXYZB, JUVWA, JUVWB, contactDepth, ids, G, dG, mass, fric, inertia, rot, vel,
				omega, pos, updateV, updateO, offset);
	}
}

///////////////////////////////////////////////////////////////////////////////////
// Kernel for a single iteration of the LCP over all scalar bilateral contacts
// (a bit similar to the ChKernelLCPiteration above, but without projection etc.)
// Version 2.0 - Tasora
//

__host__ __device__ void function_Bilaterals(uint& index, uint& number_of_bilaterals, uint& number_of_contacts, real& lcp_omega_bilateral, real4* bilaterals, real* mass, real3* inertia, real4* rot,
		real3* vel, real3* omega, real3* pos, real3* updateV, real3* updateO, uint* offset, real* dG) {
	real4 vA;
	real3 vB;
	real gamma_new = 0, gamma_old = 0;
	int B1_index = 0, B2_index = 0;
	B1_index = bilaterals[index].w;
	B2_index = bilaterals[index + number_of_bilaterals].w;
	real mass1 = mass[B1_index];
	real mass2 = mass[B2_index];
	// ---- perform   gamma_new = ([J1 J2] {v1 | v2}^ + b)
	{
		vA = bilaterals[index]; // line 0
		vB = vel[B1_index]; // v1
		gamma_new += dot3(vA, vB);
		vA = bilaterals[index + 2 * number_of_bilaterals]; // line 2
		vB = omega[B1_index]; // w1
		gamma_new += dot3(vA, vB);
	}
	{
		vA = bilaterals[index + number_of_bilaterals]; // line 1
		vB = vel[B2_index]; // v2
		gamma_new += dot3(vA, vB);
		vA = bilaterals[index + 3 * number_of_bilaterals]; // line 3
		vB = omega[B2_index]; // w2
		gamma_new += dot3(vA, vB);
	}
	vA = bilaterals[index + 4 * number_of_bilaterals]; // line 4   (eta, b, gamma, 0)
	gamma_new += vA.y; // add known term     + b
	gamma_old = vA.z; // old gamma
	/// ---- perform gamma_new *= omega/g_i
	gamma_new *= lcp_omega_bilateral; // lcp_omega_const is in constant memory
	gamma_new *= vA.x; // eta = 1/g_i;
	/// ---- perform gamma_new = gamma_old - gamma_new ; in place.
	gamma_new = gamma_old - gamma_new;

	/// ---- perform projection of 'a' (only if simple unilateral behavior C>0 is requested)
	if (vA.w && gamma_new < 0.) {
		gamma_new = 0.;
	}

	// ----- store gamma_new
	vA.z = gamma_new;
	bilaterals[index + 4 * number_of_bilaterals] = vA;
	/// ---- compute delta in multipliers: gamma_new = gamma_new - gamma_old   = delta_gamma    , in place.
	gamma_new -= gamma_old;
	dG[number_of_contacts + index] = (gamma_new);
	/// ---- compute dv1 =  invInert.18 * J1^ * deltagamma
	vB = inertia[B1_index]; // iJ iJ iJ im
	vA = (bilaterals[index]) * mass1 * gamma_new; // line 0: J1(x)
	int offset1 = offset[2 * number_of_contacts + index];
	int offset2 = offset[2 * number_of_contacts + index + number_of_bilaterals];
	updateV[offset1] = make_real3(vA); //  ---> store  v1 vel. in reduction buffer
	updateO[offset1] = make_real3((bilaterals[index + 2 * number_of_bilaterals]) * R4(vB) * gamma_new); // line 2:  J1(w)// ---> store  w1 vel. in reduction buffer
	vB = inertia[B2_index]; // iJ iJ iJ im
	vA = (bilaterals[index + number_of_bilaterals]) * mass2 * gamma_new; // line 1: J2(x)
	updateV[offset2] = make_real3(vA); //  ---> store  v2 vel. in reduction buffer
	updateO[offset2] = make_real3((bilaterals[index + 3 * number_of_bilaterals]) * R4(vB) * gamma_new); // line 3:  J2(w)// ---> store  w2 vel. in reduction buffer
}

__global__ void device_Bilaterals(real4* bilaterals, real* mass, real3* inertia, real4* rot, real3* vel, real3* omega, real3* pos, real3* updateV, real3* updateO, uint* offset, real* dG) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_bilaterals_const);
	function_Bilaterals(index, number_of_bilaterals_const, number_of_contacts_const, lcp_omega_bilateral_const, bilaterals, mass, inertia, rot, vel, omega, pos, updateV, updateO, offset, dG);
}
void ChLcpSolverGPU::host_Bilaterals(real4* bilaterals, real* mass, real3* inertia, real4* rot, real3* vel, real3* omega, real3* pos, real3* updateV, real3* updateO, uint* offset, real* dG) {
	for (uint index = 0; index < number_of_bilaterals; index++) {
		function_Bilaterals(index, number_of_bilaterals, number_of_contacts, lcp_omega_bilateral, bilaterals, mass, inertia, rot, vel, omega, pos, updateV, updateO, offset, dG);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////
// Kernel for adding invmass*force*step_size_const to body speed vector.
// This kernel must be applied to the stream of the body buffer.

__host__ __device__ void function_addForces(uint& index, bool* active, real* mass, real3* inertia, real3* forces, real3* torques, real3* vel, real3* omega) {
	if (active[index] != 0) {
		// v += m_inv * h * f
		vel[index] += forces[index] * mass[index];
		// w += J_inv * h * c
		//omega[index] += torques[index] * inertia[index];
	}
}
__global__ void device_addForces(bool* active, real* mass, real3* inertia, real3* forces, real3* torques, real3* vel, real3* omega) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_objects_const);
	function_addForces(index, active, mass, inertia, forces, torques, vel, omega);
}

void ChLcpSolverGPU::host_addForces(bool* active, real* mass, real3* inertia, real3* forces, real3* torques, real3* vel, real3* omega) {
#pragma omp parallel for schedule(guided)

	for (uint index = 0; index < number_of_objects; index++) {
		function_addForces(index, active, mass, inertia, forces, torques, vel, omega);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Updates the speeds in the body buffer with values accumulated in the
// reduction buffer:   V_new = V_old + delta_speeds

__host__ __device__ void function_Reduce_Speeds(uint& index, bool* active, real* mass, real3* vel, real3* omega, real3* updateV, real3* updateO, uint* d_body_num, uint* counter, real3* fap) {
	int start = (index == 0) ? 0 : counter[index - 1], end = counter[index];
	int id = d_body_num[end - 1], j;
	//real3 auxd = active[id];

	if (active[id] == 0) {
		return;
	}

	real3 mUpdateV = R3(0);
	real3 mUpdateO = R3(0);

	for (j = 0; j < end - start; j++) {
		mUpdateV = mUpdateV + updateV[j + start];
		mUpdateO = mUpdateO + updateO[j + start];
	}

	//fap[id] += (mUpdateV / mass[id]) / step_size_const;
	vel[id] += (mUpdateV);
	omega[id] += (mUpdateO);
}

__global__ void device_Reduce_Speeds(bool* active, real* mass, real3* vel, real3* omega, real3* updateV, real3* updateO, uint* d_body_num, uint* counter, real3* fap) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_updates_const);
	function_Reduce_Speeds(index, active, mass, vel, omega, updateV, updateO, d_body_num, counter, fap);
}
void ChLcpSolverGPU::host_Reduce_Speeds(bool* active, real* mass, real3* vel, real3* omega, real3* updateV, real3* updateO, uint* d_body_num, uint* counter, real3* fap) {
#pragma omp parallel for schedule(guided)

	for (uint index = 0; index < number_of_updates; index++) {
		function_Reduce_Speeds(index, active, mass, vel, omega, updateV, updateO, d_body_num, counter, fap);
	}
}
__host__ __device__ void function_Integrate_Timestep(uint& index, real& step_size, bool* active, real3* acc, real4* rot, real3* vel, real3* omega, real3* pos, real3* lim) {
	real3 velocity = vel[index];

	if (active[index] == 0) {
		return;
	}

	// Do 1st order integration of quaternion position as q[t+dt] = qw_abs^(dt) * q[dt] = q[dt] * qw_local^(dt)
	// where qw^(dt) is the quaternion { cos(0.5|w|), wx/|w| sin(0.5|w|), wy/|w| sin(0.5|w|), wz/|w| sin(0.5|w|)}^dt
	// that is argument of sine and cosines are multiplied by dt.
	real3 omg = omega[index];
	real3 limits = lim[index];
	real wlen = length(omg);
//    if (limits.x == 1) {
//        real w = 2.0 * wlen;
//
//        if (w > limits.z) {
//            omg = omg * limits.z / w;
//            wlen = sqrtf(dot3(omg, omg));
//        }
//
//        real v = length(velocity);
//
//        if (v > limits.y) {
//            velocity = velocity * limits.y / v;
//        }
//
//        vel[index] = velocity;
//        omega[index] = omg;
//    }
	pos[index] = pos[index] + velocity * step_size; // Do 1st order integration of linear speeds
	real4 Rw = (fabs(wlen) > 10e-10) ? Q_from_AngAxis(step_size * wlen, omg / wlen) : R4(1, 0, 0, 0); // to avoid singularity for near zero angular speed
	Rw = normalize(Rw);
	real4 mq = mult(rot[index], Rw);
	mq = mq * rsqrtf(dot(mq, mq));
	rot[index] = mq;
	acc[index] = (velocity - acc[index]) / step_size;
}
__global__ void device_Integrate_Timestep(bool* active, real3* acc, real4* rot, real3* vel, real3* omega, real3* pos, real3* lim) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_objects_const);
	function_Integrate_Timestep(index, step_size_const, active, acc, rot, vel, omega, pos, lim);
}
void ChLcpSolverGPU::host_Integrate_Timestep(bool* active, real3* acc, real4* rot, real3* vel, real3* omega, real3* pos, real3* lim) {
#pragma omp parallel for schedule(guided)

	for (uint index = 0; index < number_of_objects; index++) {
		function_Integrate_Timestep(index, step_size, active, acc, rot, vel, omega, pos, lim);
	}
}

__host__ __device__ void function_ComputeGyro(uint& index, real3* omega, real3* inertia, real3* gyro, real3* torque) {
	real3 body_inertia = inertia[index];
	body_inertia = R3(1.0 / body_inertia.x, 1.0 / body_inertia.y, 1.0 / body_inertia.z);
	real3 body_omega = omega[index];
	real3 gyr = cross(body_omega, body_inertia * body_omega);
	gyro[index] = gyr;
}

__global__ void device_ComputeGyro(real3* omega, real3* inertia, real3* gyro, real3* torque) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_objects_const);
	function_ComputeGyro(index, omega, inertia, gyro, torque);
}

void ChLcpSolverGPU::host_ComputeGyro(real3* omega, real3* inertia, real3* gyro, real3* torque) {
#pragma omp parallel for schedule(guided)

	for (uint index = 0; index < number_of_objects; index++) {
		function_ComputeGyro(index, omega, inertia, gyro, torque);
	}
}

__global__ void device_Offsets(int2* ids, real4* bilaterals, uint* Body) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;

	if (index < number_of_contacts_const) {
		int2 temp_id = ids[index];
		Body[index] = temp_id.x;
		Body[index + number_of_contacts_const] = temp_id.y;
	}

	if (index < number_of_bilaterals_const) {
		Body[2 * number_of_contacts_const + index] = bilaterals[index].w;
		Body[2 * number_of_contacts_const + index + number_of_bilaterals_const] = bilaterals[index + number_of_bilaterals_const].w;
	}
}

void ChLcpSolverGPU::host_Offsets(int2* ids, real4* bilaterals, uint* Body) {
	for (uint index = 0; index < number_of_constraints; index++) {
		if (index < number_of_contacts) {
			int2 temp_id = ids[index];
			Body[index] = temp_id.x;
			Body[index + number_of_contacts] = temp_id.y;
		}

		if (index < number_of_bilaterals) {
			Body[2 * number_of_contacts + index] = bilaterals[index].w;
			Body[2 * number_of_contacts + index + number_of_bilaterals] = bilaterals[index + number_of_bilaterals].w;
		}
	}
}

void ChLcpSolverGPU::Preprocess(gpu_container& gpu_data) {
	gpu_data.number_of_updates = 0;
	gpu_data.device_gyr_data.resize(number_of_objects);
	custom_vector<uint> body_num;
	custom_vector<uint> update_number;
	gpu_data.device_gam_data.resize(number_of_constraints);
#ifdef SIM_ENABLE_GPU_MODE
	device_ComputeGyro CUDA_KERNEL_DIM(BLOCKS(number_of_objects), THREADS)(CASTR3(gpu_data.device_omg_data), CASTR3(gpu_data.device_inr_data), CASTR3(gpu_data.device_gyr_data),
			CASTR3(gpu_data.device_trq_data));
#else
	host_ComputeGyro(
			gpu_data.device_omg_data.data(),
			gpu_data.device_inr_data.data(),
			gpu_data.device_gyr_data.data(),
			gpu_data.device_trq_data.data());
#endif
#ifdef SIM_ENABLE_GPU_MODE
	device_addForces CUDA_KERNEL_DIM(BLOCKS(number_of_objects), THREADS)(CASTB1(gpu_data.device_active_data), CASTR1(gpu_data.device_mass_data), CASTR3(gpu_data.device_inr_data),
			CASTR3(gpu_data.device_frc_data), CASTR3(gpu_data.device_trq_data), CASTR3(gpu_data.device_vel_data), CASTR3(gpu_data.device_omg_data));
#else
	host_addForces(
			gpu_data.device_active_data.data(),
			gpu_data.device_mass_data.data(),
			gpu_data.device_inr_data.data(),
			gpu_data.device_frc_data.data(),
			gpu_data.device_trq_data.data(),
			gpu_data.device_vel_data.data(),
			gpu_data.device_omg_data.data());
#endif
	gpu_data.device_fap_data.resize(number_of_objects);
	Thrust_Fill(gpu_data.device_fap_data, R3(0));

	if (number_of_constraints > 0) {
		update_number.resize((number_of_constraints) * 2, 0);
		gpu_data.offset_counter.resize((number_of_constraints) * 2, 0);
		gpu_data.update_offset.resize((number_of_constraints) * 2, 0);
		body_num.resize((number_of_constraints) * 2, 0);
		gpu_data.device_dgm_data.resize((number_of_constraints));
		Thrust_Fill(gpu_data.device_dgm_data, 1);
		gpu_data.vel_update.resize((number_of_constraints) * 2);
		gpu_data.omg_update.resize((number_of_constraints) * 2);
#ifdef SIM_ENABLE_GPU_MODE
		device_Offsets CUDA_KERNEL_DIM(BLOCKS(number_of_constraints), THREADS)(CASTI2(gpu_data.device_bids_data), CASTR4(gpu_data.device_bilateral_data), CASTU1(body_num));
#else
		host_Offsets(gpu_data.device_bids_data.data(), gpu_data.device_bilateral_data.data(), body_num.data());
#endif
		Thrust_Sequence(update_number);
		Thrust_Sequence(gpu_data.update_offset);
		Thrust_Fill(gpu_data.offset_counter, 0);
		Thrust_Sort_By_Key(body_num, update_number);
		Thrust_Sort_By_Key(update_number, gpu_data.update_offset);
		gpu_data.body_number = body_num;
		Thrust_Reduce_By_KeyB(gpu_data.number_of_updates, body_num, update_number, gpu_data.offset_counter);
//        host_vector<uint> body_num_t=body_num;
//        host_vector<uint> update_number_t=update_number;
//        host_vector<uint> offset_counter_t=gpu_data.offset_counter;
//        Thrust_Reduce_By_KeyB(gpu_data.number_of_updates, body_num_t, update_number_t, offset_counter_t);
//        body_num=body_num_t;
//        update_number=update_number_t;
//        gpu_data.offset_counter=offset_counter_t;
		Thrust_Inclusive_Scan(gpu_data.offset_counter);
	}
}

void ChLcpSolverGPU::Iterate(gpu_container& gpu_data) {
}
void ChLcpSolverGPU::Reduce(gpu_container& gpu_data) {
}
void ChLcpSolverGPU::Integrate(gpu_container& gpu_data) {
#ifdef SIM_ENABLE_GPU_MODE
	device_Integrate_Timestep CUDA_KERNEL_DIM(BLOCKS(number_of_objects), THREADS)(CASTB1(gpu_data.device_active_data), CASTR3(gpu_data.device_acc_data), CASTR4(gpu_data.device_rot_data),
			CASTR3(gpu_data.device_vel_data), CASTR3(gpu_data.device_omg_data), CASTR3(gpu_data.device_pos_data), CASTR3(gpu_data.device_lim_data));
#else
	host_Integrate_Timestep(
			gpu_data.device_active_data.data(),
			gpu_data.device_acc_data.data(),
			gpu_data.device_rot_data.data(),
			gpu_data.device_vel_data.data(),
			gpu_data.device_omg_data.data(),
			gpu_data.device_pos_data.data(),
			gpu_data.device_lim_data.data());
#endif
}
void ChLcpSolverGPU::RunTimeStep(real step, gpu_container& gpu_data) {
	lcp_omega_contact = omega;
	step_size = step;
	number_of_constraints = gpu_data.number_of_contacts + gpu_data.number_of_bilaterals;
	number_of_contacts = gpu_data.number_of_contacts;
	number_of_bilaterals = 0; ///gpu_data.number_of_bilaterals;
	number_of_objects = gpu_data.number_of_objects;
	//cout << number_of_constraints << " " << number_of_contacts << " " << number_of_bilaterals << " " << number_of_objects << endl;
#ifdef SIM_ENABLE_GPU_MODE
	COPY_TO_CONST_MEM(number_of_contacts);
	COPY_TO_CONST_MEM(number_of_bilaterals);
	COPY_TO_CONST_MEM(number_of_objects);
	COPY_TO_CONST_MEM(step_size);
	cudaFuncSetCacheConfig(device_ComputeGyro, cudaFuncCachePreferL1);
	cudaFuncSetCacheConfig(device_addForces, cudaFuncCachePreferL1);
	cudaFuncSetCacheConfig(device_Offsets, cudaFuncCachePreferL1);
#else
#endif
	Preprocess(gpu_data);
	number_of_updates = gpu_data.number_of_updates;
#ifdef SIM_ENABLE_GPU_MODE
	COPY_TO_CONST_MEM(compliance);
	COPY_TO_CONST_MEM(complianceT);
	COPY_TO_CONST_MEM(alpha);
	COPY_TO_CONST_MEM(lcp_omega_bilateral);
	COPY_TO_CONST_MEM(lcp_omega_contact);
	COPY_TO_CONST_MEM(lcp_contact_factor);
	COPY_TO_CONST_MEM(number_of_updates);
	cudaFuncSetCacheConfig(device_process_contacts, cudaFuncCachePreferL1);
	cudaFuncSetCacheConfig(device_Bilaterals, cudaFuncCachePreferL1);
	cudaFuncSetCacheConfig(device_Reduce_Speeds, cudaFuncCachePreferL1);
	cudaFuncSetCacheConfig(device_Integrate_Timestep, cudaFuncCachePreferL1);
#else
#endif
	real old_gam = 1;
	//COMPUTE JACOBIANS

	gpu_data.device_QXYZ_data.resize(number_of_objects);
	gpu_data.device_QUVW_data.resize(number_of_objects);

	gpu_data.device_JXYZA_data.resize(number_of_contacts * 3 + number_of_constraints);
	gpu_data.device_JXYZB_data.resize(number_of_contacts * 3 + number_of_constraints);
	gpu_data.device_JUVWA_data.resize(number_of_contacts * 3 + number_of_constraints);
	gpu_data.device_JUVWB_data.resize(number_of_contacts * 3 + number_of_constraints);
#ifdef SIM_ENABLE_GPU_MODE
	device_ContactJacobians CUDA_KERNEL_DIM(BLOCKS(number_of_contacts), THREADS)(CASTR3(gpu_data.device_norm_data), CASTR3(gpu_data.device_cpta_data), CASTR3(gpu_data.device_cptb_data),
			CASTI2(gpu_data.device_bids_data), CASTR4(gpu_data.device_rot_data), CASTR3(gpu_data.device_pos_data), CASTR3(gpu_data.device_JXYZA_data), CASTR3(gpu_data.device_JXYZB_data),
			CASTR3(gpu_data.device_JUVWA_data), CASTR3(gpu_data.device_JUVWB_data));

#else
	host_ContactJacobians(
			gpu_data.device_norm_data.data(),
			gpu_data.device_cpta_data.data(),
			gpu_data.device_cptb_data.data(),
			gpu_data.device_bids_data.data(),
			gpu_data.device_rot_data.data(),
			gpu_data.device_pos_data.data(),
			gpu_data.device_JXYZA_data.data(),
			gpu_data.device_JXYZB_data.data(),
			gpu_data.device_JUVWA_data.data(),
			gpu_data.device_JUVWB_data.data()
	);
#endif
cout<<max_iterations<<" "<<tolerance<<endl;
	if (number_of_constraints != 0) {
		for (iteration_number = 0; iteration_number < 500; iteration_number++) {
#ifdef SIM_ENABLE_GPU_MODE
			device_process_contacts CUDA_KERNEL_DIM(BLOCKS(number_of_contacts), THREADS)(CASTR3(gpu_data.device_JXYZA_data), CASTR3(gpu_data.device_JXYZB_data), CASTR3(gpu_data.device_JUVWA_data),
					CASTR3(gpu_data.device_JUVWB_data), CASTR1(gpu_data.device_dpth_data), CASTI2(gpu_data.device_bids_data), CASTR3(gpu_data.device_gam_data), CASTR1(gpu_data.device_dgm_data),
					CASTR1(gpu_data.device_mass_data), CASTR1(gpu_data.device_fric_data), CASTR3(gpu_data.device_inr_data), CASTR4(gpu_data.device_rot_data), CASTR3(gpu_data.device_vel_data),
					CASTR3(gpu_data.device_omg_data), CASTR3(gpu_data.device_pos_data), CASTR3(gpu_data.vel_update), CASTR3(gpu_data.omg_update), CASTU1(gpu_data.update_offset));
#else
			host_process_contacts(
					gpu_data.device_JXYZA_data.data(),
					gpu_data.device_JXYZB_data.data(),
					gpu_data.device_JUVWA_data.data(),
					gpu_data.device_JUVWB_data.data(),
					gpu_data.device_dpth_data.data(),
					gpu_data.device_bids_data.data(),
					gpu_data.device_gam_data.data(),
					gpu_data.device_dgm_data.data(),
					gpu_data.device_mass_data.data(),
					gpu_data.device_fric_data.data(),
					gpu_data.device_inr_data.data(),
					gpu_data.device_rot_data.data(),
					gpu_data.device_vel_data.data(),
					gpu_data.device_omg_data.data(),
					gpu_data.device_pos_data.data(),
					gpu_data.vel_update.data(),
					gpu_data.omg_update.data(),
					gpu_data.update_offset.data());
#endif
#ifdef SIM_ENABLE_GPU_MODE
			device_Bilaterals CUDA_KERNEL_DIM(BLOCKS(number_of_bilaterals), THREADS)(CASTR4(gpu_data.device_bilateral_data), CASTR1(gpu_data.device_mass_data), CASTR3(gpu_data.device_inr_data),
					CASTR4(gpu_data.device_rot_data), CASTR3(gpu_data.device_vel_data), CASTR3(gpu_data.device_omg_data), CASTR3(gpu_data.device_pos_data), CASTR3(gpu_data.vel_update),
					CASTR3(gpu_data.omg_update), CASTU1(gpu_data.update_offset), CASTR1(gpu_data.device_dgm_data));
#else
			host_Bilaterals(
					gpu_data.device_bilateral_data.data(),
					gpu_data.device_mass_data.data(),
					gpu_data.device_inr_data.data(),
					gpu_data.device_rot_data.data(),
					gpu_data.device_vel_data.data(),
					gpu_data.device_omg_data.data(),
					gpu_data.device_pos_data.data(),
					gpu_data.vel_update.data(),
					gpu_data.omg_update.data(),
					gpu_data.update_offset.data(),
					gpu_data.device_dgm_data.data());
#endif
#ifdef SIM_ENABLE_GPU_MODE
			device_Reduce_Speeds CUDA_KERNEL_DIM(BLOCKS(number_of_updates), THREADS)(CASTB1(gpu_data.device_active_data), CASTR1(gpu_data.device_mass_data), CASTR3(gpu_data.device_vel_data),
					CASTR3(gpu_data.device_omg_data), CASTR3(gpu_data.vel_update), CASTR3(gpu_data.omg_update), CASTU1(gpu_data.body_number), CASTU1(gpu_data.offset_counter),
					CASTR3(gpu_data.device_fap_data));
#else
			host_Reduce_Speeds(
					gpu_data.device_active_data.data(),
					gpu_data.device_mass_data.data(),
					gpu_data.device_vel_data.data(),
					gpu_data.device_omg_data.data(),
					gpu_data.vel_update.data(),
					gpu_data.omg_update.data(),
					gpu_data.body_number.data(),
					gpu_data.offset_counter.data(),
					gpu_data.device_fap_data.data());
#endif

			if (tolerance != 0) {
				if (iteration_number > 20 && iteration_number % 20 == 0) {
					real gam = Max_DeltaGamma(gpu_data.device_dgm_data);

					if (fabsf(old_gam - gam) < 1e-8) {
						break;
					}

					old_gam = gam;
				}
			}
		}
	}
cout<<" "<<iteration_number<<endl;
	Integrate(gpu_data);
}
real ChLcpSolverGPU::Max_DeltaGamma(custom_vector<real>& device_dgm_data) {
	return Thrust_Max(device_dgm_data);
}
real ChLcpSolverGPU::Min_DeltaGamma(custom_vector<real>& device_dgm_data) {
	return Thrust_Min(device_dgm_data);
}
real ChLcpSolverGPU::Avg_DeltaGamma(uint number_of_constraints, custom_vector<real>& device_dgm_data) {
	real gamma = (Thrust_Total(device_dgm_data)) / real(number_of_constraints);
	//cout << gamma << endl;
		return gamma;
	}
__global__ void Compute_KE(real3* vel, real* mass, real* ke) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_objects_const);
	real3 velocity = vel[index];
	ke[index] = .5 / mass[index] * dot(velocity, velocity);
}
real ChLcpSolverGPU::Total_KineticEnergy(gpu_container& gpu_data) {
	custom_vector<real> device_ken_data;
	device_ken_data.resize(gpu_data.number_of_objects);
	Compute_KE CUDA_KERNEL_DIM(BLOCKS(gpu_data.number_of_objects), THREADS)(CASTR3(gpu_data.device_vel_data), CASTR1(gpu_data.device_mass_data), CASTR1(device_ken_data));
	return (Thrust_Total(device_ken_data));
}

