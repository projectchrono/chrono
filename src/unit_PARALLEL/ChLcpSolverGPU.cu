#include "ChLcpSolverGPU.h"

using namespace chrono;

//helper functions
template<class T, class U> // dot product of the first three elements of float3/float4 values
inline __host__ __device__ float dot3(const T & a, const U & b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

__constant__ float lcp_omega_bilateral_const;
__constant__ float lcp_omega_contact_const;
__constant__ float lcp_contact_factor_const;
__constant__ unsigned int number_of_objects_const;
__constant__ unsigned int number_of_contacts_const;
__constant__ unsigned int number_of_bilaterals_const;
__constant__ unsigned int number_of_updates_const;
//__constant__ float force_factor_const; // usually, the step size
//__constant__ float negated_recovery_speed_const;
__constant__ float step_size_const;
__constant__ float compliance_const;
__constant__ float complianceT_const;
__constant__ float alpha_const; // [R]=alpha*[K]
////////////////////////////////////////////////////////////////////////////////////////////////////
// Creates a quaternion as a function of a vector of rotation and an angle (the vector is assumed already
// normalized, and angle is in radians).

__host__ __device__ inline float4 Quat_from_AngAxis(const float &angle, const float3 & v) {
	float sinhalf = sinf(angle * 0.5f);
	float4 quat;
	quat.x = cosf(angle * 0.5f);
	quat.y = v.x * sinhalf;
	quat.z = v.y * sinhalf;
	quat.w = v.z * sinhalf;
	return quat;
}

/// The quaternion becomes the quaternion product of the two quaternions A and B:
/// following the classic Hamilton rule:  this=AxB
/// This is the true, typical quaternion product. It is NOT commutative.

__host__ __device__ inline float4 Quaternion_Product(const float4 &qa, const float4 &qb) {
	float4 quat;
	quat.x = qa.x * qb.x - qa.y * qb.y - qa.z * qb.z - qa.w * qb.w;
	quat.y = qa.x * qb.y + qa.y * qb.x - qa.w * qb.z + qa.z * qb.w;
	quat.z = qa.x * qb.z + qa.z * qb.x + qa.w * qb.y - qa.y * qb.w;
	quat.w = qa.x * qb.w + qa.w * qb.x - qa.z * qb.y + qa.y * qb.z;
	return quat;
}
__device__ void inline Compute_Jacobian(float3 &T3, float3 &T4, float3 &T5, float4 Eb, const float3 &N, const float3 &U, const float3 &W, const float3 &pb) {

	float t1 = Eb.y * Eb.z;
	float t2 = Eb.x * Eb.w;
	float t3 = 2 * t1 - 2 * t2;
	float t4 = Eb.y * Eb.w;
	float t5 = Eb.x * Eb.z;
	float t6 = 2 * t4 + 2 * t5;
	float t7 = Eb.z * Eb.w;
	float t8 = Eb.x * Eb.y;
	float t9 = 2 * t7 - 2 * t8;
	float t10 = Eb.x * Eb.x;
	float t11 = Eb.y * Eb.y;
	float t12 = Eb.w * Eb.w;
	float t13 = Eb.z * Eb.z;
	float t14 = t10 - t11 - t13 + t12;
	float t15 = t6 * pb.x + t9 * pb.y + t14 * pb.z;
	float t16 = t10 - t11 + t13 - t12;
	t7 = 2 * t7 + 2 * t8;
	t8 = -t3 * pb.x - t16 * pb.y - t7 * pb.z;
	float t17 = t3 * t15 + t6 * t8;
	float t18 = t16 * t15 + t9 * t8;
	float t19 = t7 * t15 + t14 * t8;
	t10 = t10 + t11 - t13 - t12;
	t11 = -t15;
	t1 = 2 * t1 + 2 * t2;
	t2 = 2 * t4 - 2 * t5;
	t4 = t10 * pb.x + t1 * pb.y + t2 * pb.z;
	t5 = t10 * t11 + t6 * t4;
	t6 = t1 * t11 + t9 * t4;
	t9 = t2 * t11 + t14 * t4;
	t8 = -t8;
	t4 = -t4;
	t3 = t10 * t8 + t3 * t4;
	t1 = t1 * t8 + t16 * t4;
	t2 = t2 * t8 + t7 * t4;
	T3.x = N.x * t17 + N.y * t18 + N.z * t19;
	T3.y = N.x * t5 + N.y * t6 + N.z * t9;
	T3.z = N.x * t3 + N.y * t1 + N.z * t2;
	T4.x = U.x * t17 + U.y * t18 + U.z * t19;
	T4.y = U.x * t5 + U.y * t6 + U.z * t9;
	T4.z = U.x * t3 + U.y * t1 + U.z * t2;
	T5.x = W.x * t17 + W.y * t18 + W.z * t19;
	T5.y = W.x * t5 + W.y * t6 + W.z * t9;
	T5.z = W.x * t3 + W.y * t1 + W.z * t2;
}

// 	Kernel for a single iteration of the LCP over all contacts
//   	Version 2.0 - Tasora
//	Version 2.2- Hammad (optimized, cleaned etc)
__global__ void LCP_Iteration_Contacts(
		float3* norm,
		float3* ptA,
		float3* ptB,
		float* contactDepth,
		int2* ids,
		float3* G,
		float* dG,
		float3* aux,
		float3* inertia,
		float4* rot,
		float3* vel,
		float3* omega,
		float3* pos,
		float4* updateV,
		float4* updateO,
		uint* offset) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= number_of_contacts_const) {
		return;
	}

	float3 gamma, T3, T4, T5, T6, T7, T8;
	int2 temp_id = ids[i];
	float depth = -fabs(contactDepth[i]);

	int B1_i = temp_id.x;
	int B2_i = temp_id.y;

	float3 N = norm[i]; //assume: normalized, and if depth=0 norm=(1,0,0)

	float3 W = fabs(N); //Gramm Schmidt; work with the global axis that is "most perpendicular" to the contact normal vector;effectively this means looking for the smallest component (in abs) and using the corresponding direction to carry out cross product.
	float3 U = F3(0, N.z, -N.y); //it turns out that X axis is closest to being perpendicular to contact vector;
	{
		if (W.x > W.y) {
			U = F3(-N.z, 0, N.x);
		} //it turns out that Y axis is closest to being perpendicular to contact vector;
		if (W.y > W.z) {
			U = F3(N.y, -N.x, 0);
		} //it turns out that Z axis is closest to being perpendicular to contact vector;
		U = normalize(U); //normalize the local contact Y,Z axis
		W = cross(N, U); //carry out the last cross product to find out the contact local Z axis : multiply the contact normal by the local Y component
	}

	{
		float3 sbar = ptA[i] - pos[B1_i]; //Contact Point on A - Position of A
		float4 E1 = rot[B1_i]; //bring in the Euler parameters associated with body 1;
		Compute_Jacobian(T3, T4, T5, E1, N, U, W, sbar); //A_i,p'*A_A*(sbar~_i,A)
	}
	{
		float3 sbar = ptB[i] - pos[B2_i]; //Contact Point on B - Position of B
		float4 E2 = rot[B2_i]; //bring in the Euler parameters associated with body 2;
		Compute_Jacobian(T6, T7, T8, E2, N, U, W, sbar); //A_i,p'*A_B*(sbar~_i,B)
	}

	T6 = -T6;
	T7 = -T7;
	T8 = -T8;

//	float normV = dot(N, ((V2 - V1)));
//	normV = (normV > 0) ? 0 : normV;
	float cfm = 0, cfmT = 0;
//	if (compliance_const) {
//		float h = step_size_const;
//		float inv_hpa = 1.0 / (h + alpha_const); // 1/(h+a)
//		float inv_hhpa = 1.0 / (h * (h + alpha_const)); // 1/(h*(h+a))
//		bi = inv_hpa * depth;
//		cfm = inv_hhpa * compliance_const;
//		cfmT = inv_hhpa * complianceT_const;
//	} else {
	float bi = fmaxf((depth / (step_size_const)), -lcp_contact_factor_const);
//	}

	//c_i = [Cq_i]*q + b_i + cfm_i*l_i
	float3 W1 = omega[B1_i];
	float3 W2 = omega[B2_i];
	float3 V1 = vel[B1_i];
	float3 V2 = vel[B2_i];

	gamma.x = dot(T3, W1) - dot(N, V1) + dot(T6, W2) + dot(N, V2) + bi /*+ cfm * gamma_old.x*/; //+bi
	gamma.y = dot(T4, W1) - dot(U, V1) + dot(T7, W2) + dot(U, V2) /*+ cfmT * gamma_old.y*/;
	gamma.z = dot(T5, W1) - dot(W, V1) + dot(T8, W2) + dot(W, V2) /*+ cfmT * gamma_old.z*/;
	float3 In1 = inertia[B1_i]; // bring in the inertia attributes; to be used to compute \eta
	float3 In2 = inertia[B2_i]; // bring in the inertia attributes; to be used to compute \eta

	float3 aux1 = aux[B1_i];
	float3 aux2 = aux[B2_i];

	float eta = dot(T3 * T3, In1) + dot(T4 * T4, In1) + dot(T5 * T5, In1); // update expression of eta
	eta += dot(T6 * T6, In2) + dot(T7 * T7, In2) + dot(T8 * T8, In2);
	eta += (dot(N, N) + dot(U, U) + dot(W, W)) * (aux1.z + aux2.z); // multiply by inverse of mass matrix of B1 and B2, add contribution from mass and matrix A_c.
	eta = lcp_omega_contact_const / eta; // final value of eta
	float3 gamma_old = G[i];
	gamma = eta * gamma; // perform gamma *= omega*eta
	gamma = gamma_old - gamma; // perform gamma = gamma_old - gamma ;  in place.
	/// ---- perform projection of 'a8' onto friction cone  --------
	float f_tang = sqrtf(gamma.y * gamma.y + gamma.z * gamma.z);
	float mu = (aux1.y + aux2.y) * .5f;

	if (f_tang > (mu * gamma.x)) { // inside upper cone? keep untouched!
		if ((f_tang) < -(1.0 / mu) * gamma.x || (fabsf(gamma.x) < 0)) { // inside lower cone? reset  normal,u,v to zero!
			gamma = F3(0);
		} else { // remaining case: project orthogonally to generator segment of upper cone
			gamma.x = (f_tang * mu + gamma.x) / (mu * mu + 1);
			float tproj_div_t = (gamma.x * mu) / f_tang; //  reg = tproj_div_t
			gamma.y *= tproj_div_t;
			gamma.z *= tproj_div_t;
		}
	} else if (mu == 0) {
		gamma.y = gamma.z = 0;
	}

	G[i] = gamma; // store gamma_new
	gamma -= gamma_old; // compute delta_gamma = gamma_new - gamma_old   = delta_gamma.
	dG[i] = length(gamma);
	float3 vB = N * gamma.x + U * gamma.y + W * gamma.z;
	int offset1 = offset[i];
	int offset2 = offset[i + number_of_contacts_const];
	updateV[offset1] = F4(-vB * aux1.z); // compute and store dv1
	updateO[offset1] = F4((T3 * gamma.x + T4 * gamma.y + T5 * gamma.z) * In1); // compute dw1 =  Inert.1' * J1w^ * deltagamma  and store  dw1
	updateV[offset2] = F4(vB * aux2.z); // compute and store dv2
	updateO[offset2] = F4((T6 * gamma.x + T7 * gamma.y + T8 * gamma.z) * In2); // compute dw2 =  Inert.2' * J2w^ * deltagamma  and store  dw2

}
///////////////////////////////////////////////////////////////////////////////////
// Kernel for a single iteration of the LCP over all scalar bilateral contacts
// (a bit similar to the ChKernelLCPiteration above, but without projection etc.)
// Version 2.0 - Tasora
//
__global__ void LCP_Iteration_Bilaterals(
		CH_REALNUMBER4* bilaterals,
		float3* aux,
		float3* inertia,
		float4* rot,
		float3* vel,
		float3* omega,
		float3* pos,
		float4* updateV,
		float4* updateO,
		uint* offset,
		float * dG) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= number_of_bilaterals_const) {
		return;
	}
	float4 vA;
	float3 vB;
	float gamma_new = 0, gamma_old = 0;
	int B1_index = 0, B2_index = 0;
	B1_index = bilaterals[i].w;
	B2_index = bilaterals[i + number_of_bilaterals_const].w;
	float3 aux1 = aux[B1_index];
	float3 aux2 = aux[B2_index];
	// ---- perform   gamma_new = ([J1 J2] {v1 | v2}^ + b)
	{
		vA = bilaterals[i]; // line 0
		vB = vel[B1_index]; // v1
		gamma_new += dot3(vA, vB);

		vA = bilaterals[i + 2 * number_of_bilaterals_const]; // line 2
		vB = omega[B1_index]; // w1
		gamma_new += dot3(vA, vB);
	}
	{
		vA = bilaterals[i + number_of_bilaterals_const]; // line 1
		vB = vel[B2_index]; // v2
		gamma_new += dot3(vA, vB);

		vA = bilaterals[i + 3 * number_of_bilaterals_const]; // line 3
		vB = omega[B2_index]; // w2
		gamma_new += dot3(vA, vB);
	}
	vA = bilaterals[i + 4 * number_of_bilaterals_const]; // line 4   (eta, b, gamma, 0)
	gamma_new += vA.y; // add known term     + b
	gamma_old = vA.z; // old gamma
	/// ---- perform gamma_new *= omega/g_i
	gamma_new *= lcp_omega_bilateral_const; // lcp_omega_const is in constant memory
	gamma_new *= vA.x; // eta = 1/g_i;
	/// ---- perform gamma_new = gamma_old - gamma_new ; in place.
	gamma_new = gamma_old - gamma_new;
	/// ---- perform projection of 'a' (only if simple unilateral behavior C>0 is requested)
	if (vA.w && gamma_new < 0.) {
		gamma_new = 0.;
	}
	// ----- store gamma_new
	vA.z = gamma_new;
	bilaterals[i + 4 * number_of_bilaterals_const] = vA;
	/// ---- compute delta in multipliers: gamma_new = gamma_new - gamma_old   = delta_gamma    , in place.
	gamma_new -= gamma_old;
	//dG[number_of_contacts_const + i] = (gamma_new);
	/// ---- compute dv1 =  invInert.18 * J1^ * deltagamma
	vB = inertia[B1_index]; // iJ iJ iJ im
	vA = (bilaterals[i]) * aux1.z * gamma_new; // line 0: J1(x)
	int offset1 = offset[2 * number_of_contacts_const + i];
	int offset2 = offset[2 * number_of_contacts_const + i + number_of_bilaterals_const];
	updateV[offset1] = (vA); //  ---> store  v1 vel. in reduction buffer
	updateO[offset1] = F4(F3(bilaterals[i + 2 * number_of_bilaterals_const]) * vB * gamma_new); // line 2:  J1(w)// ---> store  w1 vel. in reduction buffer
	vB = inertia[B2_index]; // iJ iJ iJ im
	vA = (bilaterals[i + number_of_bilaterals_const]) * aux2.z * gamma_new; // line 1: J2(x)
	updateV[offset2] = (vA); //  ---> store  v2 vel. in reduction buffer
	updateO[offset2] = F4(F3(bilaterals[i + 3 * number_of_bilaterals_const]) * vB * gamma_new); // line 3:  J2(w)// ---> store  w2 vel. in reduction buffer
}

__device__ __host__ inline float4 computeRot_dt(float3 & omega, float4 &rot) {
	return mult(F4(0, omega.x, omega.y, omega.z), rot) * .5;
}
__device__ __host__ float3 RelPoint_AbsSpeed(float3 & vel, float3 & omega, float4 & rot, float3 &point) {
	float4 q = mult(computeRot_dt(omega, rot), mult(F4(0, point.x, point.y, point.z), inv(rot)));
	return vel + ((F3(q.y, q.z, q.w)) * 2);
}

__global__ void DEM_Contacts(
		float3* norm,
		float3* ptA,
		float3* ptB,
		float* contactDepth,
		int2* ids,
		float3* aux,
		float3* inertia,
		float4* rot,
		float3* vel,
		float3* omega,
		float3* pos,
		float3* updateV,
		float3* updateO,
		uint* offset) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= number_of_contacts_const) {
		return;
	}
	float mkn = 3924, mgn = 420, mgt = 420, mkt = 2.0f / 7.0f * 3924;
	//long long id=ids[i];
	int2 temp_id = ids[i];
	int B1_i = int(temp_id.x);
	int B2_i = int(temp_id.y);

	float3 vN = norm[i]; //normal
	float3 pA = ptA[i];
	float3 pB = ptB[i];
	float3 B1 = vel[B1_i]; //vel B1
	float3 B2 = vel[B2_i]; //vel B2
	float3 oA = omega[B1_i];
	float3 oB = omega[B2_i];
	float4 rotA = rot[B1_i];
	float4 rotB = rot[B2_i];
	float3 aux1 = aux[B1_i];
	float3 aux2 = aux[B2_i];
	float3 posA = pos[B1_i];
	float3 posB = pos[B2_i];
	float3 f_n = mkn * -fabs(contactDepth[i]) * vN;
	float3 local_pA = quatRotate(pA - posA, inv(rotA));
	float3 local_pB = quatRotate(pB - posB, inv(rotB));

	float3 v_BA = (RelPoint_AbsSpeed(B2, oB, rotB, local_pB)) - (RelPoint_AbsSpeed(B1, oA, rotA, local_pA));
	float3 v_n = normalize(dot(v_BA, vN) * vN);
	float m_eff = (1.0 / aux1.z) * (1.0 / aux2.z) / (1.0 / aux1.z + 1.0 / aux2.z);
	f_n += mgn * m_eff * v_n;

	float mu = (aux1.y + aux2.y) * .5;
	float3 v_t = v_BA - v_n;

	float3 f_t = (mgt * m_eff * v_t) + (mkt * (v_t * step_size_const));

	if (length(f_t) > mu * length(f_n)) {
		f_t *= mu * length(f_n) / length(f_t);
	}

	float3 f_r = f_n + f_t;

	int offset1 = offset[i];
	int offset2 = offset[i + number_of_contacts_const];

	float3 force1_loc = quatRotate(f_r, inv(rotA));
	float3 force2_loc = quatRotate(f_r, inv(rotB));

	float3 trq1 = cross(local_pA, force1_loc);
	float3 trq2 = cross(local_pB, -force2_loc);

	f_r *= step_size_const;

	updateV[offset1] = (f_r) * aux1.z;
	updateV[offset2] = (f_r) * -aux2.z;

	updateO[offset1] = (trq1 * step_size_const) * (inertia[B1_i]);
	updateO[offset2] = (trq2 * step_size_const) * (inertia[B2_i]);
}

////////////////////////////////////////////////////////////////////////////////////////////////
// Kernel for adding invmass*force*step_size_const to body speed vector.
// This kernel must be applied to the stream of the body buffer.

__global__ void ChKernelLCPaddForces(float3* aux, float3* inertia, float3* forces, float3* torques, float3* vel, float3* omega) {
	// Compute the i values used to access data inside the large
	// array using pointer arithmetic.
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i < number_of_objects_const) {
		float3 temp_aux = aux[i];
		if (temp_aux.x != 0) {
			float3 mF, minvMasses = inertia[i];
			// v += m_inv * h * f
			mF = forces[i]; // vector with f (force)
			//mF *= 1;//step_size_const;
			mF *= temp_aux.z;
			vel[i] += mF;
			// w += J_inv * h * c
			mF = torques[i]; // vector with f (torque)
			//mF *= 1;//step_size_const;
			mF.x *= minvMasses.x;
			mF.y *= minvMasses.y;
			mF.z *= minvMasses.z;
			omega[i] += mF;
		}
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// Updates the speeds in the body buffer with values accumulated in the
// reduction buffer:   V_new = V_old + delta_speeds

__global__ void LCP_Reduce_Speeds(float3* aux, float3* vel, float3* omega, float4* updateV, float4* updateO, uint* d_body_num, uint* counter, float3* fap) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= number_of_updates_const) {
		return;
	}
	int start = (i == 0) ? 0 : counter[i - 1], end = counter[i];
	int id = d_body_num[end - 1], j;
	float3 auxd = aux[id];
	if (auxd.x == 0) {
		return;
	}
	float4 mUpdateV = F4(0);
	float4 mUpdateO = F4(0);
	for (j = 0; j < end - start; j++) {
		mUpdateV += updateV[j + start];
		mUpdateO += updateO[j + start];
	}
	//fap[id] += (mUpdateV / auxd.z) / step_size_const;
	vel[id] += F3(mUpdateV);
	omega[id] += F3(mUpdateO);

}
//  Kernel for performing the time step integration (with 1st o;rder Eulero)
//  on the body data stream.
//
//  number of registers: 12 (suggested 320 threads/block)

__global__ void LCP_Integrate_Timestep(float3* aux, float3* acc, float4* rot, float3* vel, float3* omega, float3* pos, float3* lim) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= number_of_objects_const) {
		return;
	}
	float3 velocity = vel[i];
	float3 aux1 = aux[i];
	if (aux1.x == 0) {
		return;
	}

	// Do 1st order integration of quaternion position as q[t+dt] = qw_abs^(dt) * q[dt] = q[dt] * qw_local^(dt)
	// where qw^(dt) is the quaternion { cos(0.5|w|), wx/|w| sin(0.5|w|), wy/|w| sin(0.5|w|), wz/|w| sin(0.5|w|)}^dt
	// that is argument of sine and cosines are multiplied by dt.
	float3 omg = omega[i];

	float3 limits = lim[i];
	float wlen = length(omg);

	if (limits.x == 1) {
		float w = 2.0 * wlen;
		if (w > limits.z) {
			omg *= limits.z / w;
			wlen = sqrtf(dot3(omg, omg));
		}

		float v = length(velocity);
		if (v > limits.y) {
			velocity *= limits.y / v;
		}
		vel[i] = velocity;
		omega[i] = omg;
	}
	pos[i] = pos[i] + velocity * step_size_const; // Do 1st order integration of linear speeds

	float4 Rw = (fabs(wlen) > 10e-10) ? Quat_from_AngAxis(step_size_const * wlen, omg / wlen) : F4(1., 0, 0, 0); // to avoid singularity for near zero angular speed

	float4 mq = mult(rot[i], Rw);
	mq *= rsqrtf(dot(mq, mq));
	rot[i] = mq;
	acc[i] = (velocity - acc[i]) / step_size_const;
}
__global__ void LCP_ComputeGyro(float3* omega, float3* inertia, float3* gyro, float3* torque) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= number_of_objects_const) {
		return;
	}
	float3 body_inertia = inertia[i];
	body_inertia = F3(1 / body_inertia.x, 1 / body_inertia.y, 1 / body_inertia.z);
	float3 body_omega = omega[i];
	float3 gyr = cross(body_omega, body_inertia * body_omega);
	gyro[i] = gyr;
}

__global__ void ChKernelOffsets(int2* ids, CH_REALNUMBER4* bilaterals, uint* Body) {
	uint i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i < number_of_contacts_const) {
		int2 temp_id = ids[i];
		Body[i] = temp_id.x;
		Body[i + number_of_contacts_const] = temp_id.y;
	}
	if (i < number_of_bilaterals_const) {
		Body[2 * number_of_contacts_const + i] = bilaterals[i].w;
		Body[2 * number_of_contacts_const + i + number_of_bilaterals_const] = bilaterals[i + number_of_bilaterals_const].w;
	}
}

void ChLcpSolverGPU::WarmContact(const int & i) {

}

void ChLcpSolverGPU::Preprocess(gpu_container & gpu_data) {
	gpu_data.number_of_updates = 0;
	gpu_data.device_gyr_data.resize(number_of_objects);
	thrust::device_vector<uint> body_num;
	thrust::device_vector<uint> update_number;
	LCP_ComputeGyro CUDA_KERNEL_DIM(BLOCKS(number_of_objects), THREADS)(
			CASTF3(gpu_data.device_omg_data),
			CASTF3(gpu_data.device_inr_data),
			CASTF3(gpu_data.device_gyr_data),
			CASTF3(gpu_data.device_trq_data));
	ChKernelLCPaddForces CUDA_KERNEL_DIM(BLOCKS(number_of_objects), THREADS)(
			CASTF3(gpu_data.device_aux_data),
			CASTF3(gpu_data.device_inr_data),
			CASTF3(gpu_data.device_frc_data),
			CASTF3(gpu_data.device_trq_data),
			CASTF3(gpu_data.device_vel_data),
			CASTF3(gpu_data.device_omg_data));
	gpu_data.device_fap_data.resize(number_of_objects);
	Thrust_Fill(gpu_data.device_fap_data, F3(0));

	if (number_of_constraints > 0) {

		update_number.resize((number_of_constraints) * 2, 0);
		gpu_data.offset_counter.resize((number_of_constraints) * 2, 0);
		gpu_data.update_offset.resize((number_of_constraints) * 2, 0);
		body_num.resize((number_of_constraints) * 2, 0);
		gpu_data.device_dgm_data.resize((number_of_constraints));

		Thrust_Fill(gpu_data.device_dgm_data, 1);
		gpu_data.vel_update.resize((number_of_constraints) * 2);
		gpu_data.omg_update.resize((number_of_constraints) * 2);

		ChKernelOffsets CUDA_KERNEL_DIM(BLOCKS(number_of_constraints), THREADS)(CASTI2(gpu_data.device_bids_data), CASTF4(gpu_data.device_bilateral_data), CASTU1(body_num));

		Thrust_Sequence(update_number);
		Thrust_Sequence(gpu_data.update_offset);
		Thrust_Fill(gpu_data.offset_counter, 0);
		Thrust_Sort_By_Key(body_num, update_number);
		Thrust_Sort_By_Key(update_number, gpu_data.update_offset);
		gpu_data.body_number = body_num;
		Thrust_Reduce_By_KeyB(gpu_data.number_of_updates, body_num, update_number, gpu_data.offset_counter);
		Thrust_Inclusive_Scan(gpu_data.offset_counter);

	}
}

void ChLcpSolverGPU::Iterate(gpu_container & gpu_data) {

}
void ChLcpSolverGPU::Reduce(gpu_container & gpu_data) {

}
void ChLcpSolverGPU::Integrate(gpu_container & gpu_data) {
	LCP_Integrate_Timestep CUDA_KERNEL_DIM( BLOCKS(number_of_objects), THREADS)(
			CASTF3(gpu_data.device_aux_data),
			CASTF3(gpu_data.device_acc_data),
			CASTF4(gpu_data.device_rot_data),
			CASTF3(gpu_data.device_vel_data),
			CASTF3(gpu_data.device_omg_data),
			CASTF3(gpu_data.device_pos_data),
			CASTF3(gpu_data.device_lim_data));
}
void ChLcpSolverGPU::RunTimeStep(float step, gpu_container & gpu_data) {

	lcp_omega_contact = omega;
	step_size = step;
	number_of_constraints = gpu_data.number_of_contacts + gpu_data.number_of_bilaterals;
	number_of_contacts = gpu_data.number_of_contacts;
	number_of_bilaterals = gpu_data.number_of_bilaterals;
	number_of_objects = gpu_data.number_of_objects;

	COPY_TO_CONST_MEM(number_of_contacts);
	COPY_TO_CONST_MEM(number_of_bilaterals);
	COPY_TO_CONST_MEM(number_of_objects);
	COPY_TO_CONST_MEM(step_size);

	cudaFuncSetCacheConfig(LCP_ComputeGyro, cudaFuncCachePreferL1);
	cudaFuncSetCacheConfig(ChKernelLCPaddForces, cudaFuncCachePreferL1);
	cudaFuncSetCacheConfig(ChKernelOffsets, cudaFuncCachePreferL1);

	Preprocess(gpu_data);
	number_of_updates = gpu_data.number_of_updates;

	COPY_TO_CONST_MEM(compliance);
	COPY_TO_CONST_MEM(complianceT);
	COPY_TO_CONST_MEM(alpha);
	COPY_TO_CONST_MEM(lcp_omega_bilateral);
	COPY_TO_CONST_MEM(lcp_omega_contact);
	COPY_TO_CONST_MEM(lcp_contact_factor);
	COPY_TO_CONST_MEM(number_of_updates);
	//cudaFuncSetCacheConfig(LCP_Iteration_Contacts, cudaFuncCachePreferL1);
	//cudaFuncSetCacheConfig(LCP_Iteration_Bilaterals, cudaFuncCachePreferL1);
	//cudaFuncSetCacheConfig(LCP_Reduce_Speeds, cudaFuncCachePreferL1);
	//cudaFuncSetCacheConfig(LCP_Integrate_Timestep, cudaFuncCachePreferL1);

	if (number_of_constraints != 0) {
		for (iteration_number = 0; iteration_number < max_iterations; iteration_number++) {
			LCP_Iteration_Contacts CUDA_KERNEL_DIM(BLOCKS(number_of_contacts), THREADS)(
					CASTF3(gpu_data.device_norm_data),
					CASTF3(gpu_data.device_cpta_data),
					CASTF3(gpu_data.device_cptb_data),
					CASTF1(gpu_data.device_dpth_data),
					CASTI2(gpu_data.device_bids_data),
					CASTF3(gpu_data.device_gam_data),
					CASTF1(gpu_data.device_dgm_data),
					CASTF3(gpu_data.device_aux_data),
					CASTF3(gpu_data.device_inr_data),
					CASTF4(gpu_data.device_rot_data),
					CASTF3(gpu_data.device_vel_data),
					CASTF3(gpu_data.device_omg_data),
					CASTF3(gpu_data.device_pos_data),
					CASTF4(gpu_data.vel_update),
					CASTF4(gpu_data.omg_update),
					CASTU1(gpu_data.update_offset));

			LCP_Iteration_Bilaterals CUDA_KERNEL_DIM(BLOCKS(number_of_bilaterals), THREADS)(
					CASTF4(gpu_data.device_bilateral_data),
					CASTF3(gpu_data.device_aux_data),
					CASTF3(gpu_data.device_inr_data),
					CASTF4(gpu_data.device_rot_data),
					CASTF3(gpu_data.device_vel_data),
					CASTF3(gpu_data.device_omg_data),
					CASTF3(gpu_data.device_pos_data),
					CASTF4(gpu_data.vel_update),
					CASTF4(gpu_data.omg_update),
					CASTU1(gpu_data.update_offset),
					CASTF1(gpu_data.device_dgm_data));

			LCP_Reduce_Speeds CUDA_KERNEL_DIM(BLOCKS( number_of_updates), THREADS)(
					CASTF3(gpu_data.device_aux_data),
					CASTF3(gpu_data.device_vel_data),
					CASTF3(gpu_data.device_omg_data),
					CASTF4(gpu_data.vel_update),
					CASTF4(gpu_data.omg_update),
					CASTU1(gpu_data.body_number),
					CASTU1(gpu_data.offset_counter),
					CASTF3(gpu_data.device_fap_data));

			if (tolerance != 0) {
				if (iteration_number > 5 && iteration_number % 5 == 0) {
					if (Max_DeltaGamma(gpu_data.device_dgm_data) < tolerance) {
						break;
					}
				}
			}
		}
	}

	Integrate(gpu_data);
}
float ChLcpSolverGPU::Max_DeltaGamma(device_vector<float> &device_dgm_data) {
	return Thrust_Max(device_dgm_data);
}
float ChLcpSolverGPU::Min_DeltaGamma(device_vector<float> &device_dgm_data) {
	return Thrust_Min(device_dgm_data);
}
float ChLcpSolverGPU::Avg_DeltaGamma(uint number_of_constraints, device_vector<float> &device_dgm_data) {

	float gamma = (Thrust_Total(device_dgm_data)) / float(number_of_constraints);
	//cout << gamma << endl;
	return gamma;
}
__global__ void Compute_KE(float3* vel, float3* aux, float* ke) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= number_of_objects_const) {
		return;
	}
	float3 velocity = vel[i];
	ke[i] = .5 / aux[i].z * dot(velocity, velocity);
}
float ChLcpSolverGPU::Total_KineticEnergy(gpu_container & gpu_data) {

	thrust::device_vector<float> device_ken_data;
	device_ken_data.resize(gpu_data.number_of_objects);
	Compute_KE CUDA_KERNEL_DIM(BLOCKS(gpu_data.number_of_objects), THREADS)(CASTF3(gpu_data.device_vel_data), CASTF3(gpu_data.device_aux_data), CASTF1(device_ken_data));
	return (Thrust_Total(device_ken_data));
}

