#include "ChLcpIterativeSolverGPU.h"

using namespace chrono;

//helper functions
template<class T, class U> // dot product of the first three elements of float3/float4 values
inline __host__ __device__ float dot3(const T & a, const U & b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

__constant__ float lcp_omega_bilateral_const;
__constant__ float lcp_omega_contact_const;
__constant__ unsigned int number_of_bodies_const;
__constant__ unsigned int number_of_contacts_const;
__constant__ unsigned int number_of_bilaterals_const;
__constant__ unsigned int number_of_updates_const;
__constant__ float force_factor_const; // usually, the step size
__constant__ float negated_recovery_speed_const;
__constant__ float c_factor_const; // usually 1/dt
__constant__ float step_size_const;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Creates a quaternion as a function of a vector of rotation and an angle (the vector is assumed already
// normalized, and angle is in radians).

inline __host__                        __device__ float4 Quat_from_AngAxis(const float &angle, const float3 & v) {
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

inline __host__             __device__ float4 Quaternion_Product(const float4 &qa, const float4 &qb) {
	float4 quat;
	quat.x = qa.x * qb.x - qa.y * qb.y - qa.z * qb.z - qa.w * qb.w;
	quat.y = qa.x * qb.y + qa.y * qb.x - qa.w * qb.z + qa.z * qb.w;
	quat.z = qa.x * qb.z + qa.z * qb.x + qa.w * qb.y - qa.y * qb.w;
	quat.w = qa.x * qb.w + qa.w * qb.x - qa.z * qb.y + qa.y * qb.z;
	return quat;
}

__host__ __device__ void inline Compute_Mat(float3 &A, float3 &B, float3 &C, float4 TT, const float3 &n, const float3 &u, const float3 &w, const float3 &pos) {
	float t00 = pos.z * n.y - pos.y * n.z;
	float t01 = TT.x * TT.x;
	float t02 = TT.y * TT.y;
	float t03 = TT.z * TT.z;
	float t04 = TT.w * TT.w;
	float t05 = t01 + t02 - t03 - t04;
	float t06 = -pos.z * n.x + pos.x * n.z;
	float t07 = TT.y * TT.z;
	float t08 = TT.x * TT.w;
	float t09 = t07 + t08;
	float t10 = pos.y * n.x - pos.x * n.y;
	float t11 = TT.y * TT.w;
	float t12 = TT.x * TT.z;
	float t13 = t11 - t12;
	float t14 = t07 - t08;
	float t15 = t01 - t02 + t03 - t04;
	float t16 = TT.z * TT.w;
	float t17 = TT.x * TT.y;
	float t18 = t16 + t17;
	float t19 = t11 + t12;
	float t20 = t16 - t17;
	float t21 = t01 - t02 - t03 + t04;
	float t22 = pos.z * u.y - pos.y * u.z;
	float t23 = -pos.z * u.x + pos.x * u.z;
	float t24 = pos.y * u.x - pos.x * u.y;
	float t25 = pos.z * w.y - pos.y * w.z;
	float t26 = -pos.z * w.x + pos.x * w.z;
	float t27 = pos.y * w.x - pos.x * w.y;
	A.x = t00 * t05 + 2 * t06 * t09 + 2 * t10 * t13;
	A.y = 2 * t00 * t14 + t06 * t15 + 2 * t10 * t18;
	A.z = 2 * t00 * t19 + 2 * t06 * t20 + t10 * t21;
	B.x = t22 * t05 + 2 * t23 * t09 + 2 * t24 * t13;
	B.y = 2 * t22 * t14 + t23 * t15 + 2 * t24 * t18;
	B.z = 2 * t22 * t19 + 2 * t23 * t20 + t24 * t21;
	C.x = t25 * t05 + 2 * t26 * t09 + 2 * t27 * t13;
	C.y = 2 * t25 * t14 + t26 * t15 + 2 * t27 * t18;
	C.z = 2 * t25 * t19 + 2 * t26 * t20 + t27 * t21;
}
// 	Kernel for a single iteration of the LCP over all contacts
//   	Version 2.0 - Tasora
//	Version 2.2- Hammad (optimized, cleaned etc)
__global__ void LCP_Iteration_Contacts(float3* norm, float3* ptA, float3* ptB, float* contactDepth, int2* ids, float3* G, float* dG, float3* aux, float3* inertia, float4* rot, float3* vel,
		float3* omega, float3* pos, float3* updateV, float3* updateO, uint* offset) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= number_of_contacts_const) {
		return;
	}
	//if(dG[i]<1e-8){dG[i]=0;return;}
	float4 E1, E2;
	float3 vB, gamma, N, U, W, T3, T4, T5, T6, T7, T8, gamma_old, sbar, B1, B2, W1, W2, aux1, aux2;
	float reg, mu, eta;
	//long long id=ids[i];
	int2 temp_id = ids[i];
	reg = -fabs(contactDepth[i]);//c_factor_const;							//Scale contact distance, cfactor is usually 1
	int B1_i = temp_id.x;
	int B2_i = temp_id.y;
	B1 = vel[B1_i];
	B2 = vel[B2_i];
	aux1 = aux[B1_i];
	aux2 = aux[B2_i];
	N = norm[i]; //assume: normalized, and if depth=0 norm=(1,0,0)
	W = fabs(N); //Gramm Schmidt; work with the global axis that is "most perpendicular" to the contact normal vector;effectively this means looking for the smallest component (in abs) and using the corresponding direction to carry out cross product.
	U = F3(0.0, N.z, -N.y); //it turns out that X axis is closest to being perpendicular to contact vector;
	if (W.x > W.y) {
		U = F3(-N.z, 0.0, N.x);
	} //it turns out that Y axis is closest to being perpendicular to contact vector;
	if (W.y > W.z) {
		U = F3(N.y, -N.x, 0.0);
	} //it turns out that Z axis is closest to being perpendicular to contact vector;
	U = normalize(U); //normalize the local contact Y,Z axis
	W = cross(N, U); //carry out the last cross product to find out the contact local Z axis : multiply the contact normal by the local Y component
	float normm = dot(N, ((B2 - B1)));
	//reg=(reg>normm)? reg:normm;

	normm = (normm > 0) ? 0 : normm;
	//reg=min(0.0,max(reg,-1.));
	reg = min((reg + normm), (-step_size_const));
	//if(reg>0){printf("REG: %f\n",reg);}
	sbar = ptA[i] - pos[B1_i]; //Contact Point on A - Position of A
	E1 = rot[B1_i]; //bring in the Euler parameters associated with body 1;
	Compute_Mat(T3, T4, T5, E1, N, U, W, sbar); //A_i,p'*A_A*(sbar~_i,A)

	sbar = ptB[i] - pos[B2_i]; //Contact Point on B - Position of B
	E2 = rot[B2_i]; //bring in the Euler parameters associated with body 2;
	Compute_Mat(T6, T7, T8, E2, N, U, W, sbar); //A_i,p'*A_B*(sbar~_i,B)
	T6 = -T6;
	T7 = -T7;
	T8 = -T8;
	//if(i==0){printf("%f %f %f | %f %f %f | %f %f %f\n",T3.x,T3.y,T3.z,T4.x,T4.y,T4.z,T5.x,T5.y,T5.z);}

	W1 = omega[B1_i];
	W2 = omega[B2_i];

	mu = (aux1.y + aux2.y) * .5;
	gamma.x = dot3(T3, W1) - dot3(N, B1) + dot3(T6, W2) + dot3(N, B2) + reg; //+bi
	gamma.y = dot3(T4, W1) - dot3(U, B1) + dot3(T7, W2) + dot3(U, B2);
	gamma.z = dot3(T5, W1) - dot3(W, B1) + dot3(T8, W2) + dot3(W, B2);
	B1 = inertia[B1_i]; // bring in the inertia attributes; to be used to compute \eta
	B2 = inertia[B2_i]; // bring in the inertia attributes; to be used to compute \eta
	eta = dot3(T3 * T3, B1) + dot3(T4 * T4, B1) + dot3(T5 * T5, B1); // update expression of eta
	eta += dot3(T6 * T6, B2) + dot3(T7 * T7, B2) + dot3(T8 * T8, B2);
	eta += (dot(N, N) + dot(U, U) + dot(W, W)) * (aux1.z + aux2.z); // multiply by inverse of mass matrix of B1 and B2, add contribution from mass and matrix A_c.
	eta = 1.0 / eta; // final value of eta

	gamma *= lcp_omega_contact_const * eta; // perform gamma *= omega*eta
	gamma_old = G[i];
	gamma = gamma_old - gamma; // perform gamma = gamma_old - gamma ;  in place.
	/// ---- perform projection of 'a8' onto friction cone  --------
	reg = sqrtf(gamma.y * gamma.y + gamma.z * gamma.z); // reg = f_tang


	if (reg > (mu * gamma.x)) { // inside upper cone? keep untouched!
		if ((mu * reg) < -gamma.x) { // inside lower cone? reset  normal,u,v to zero!
			gamma = F3(0.f, 0.f, 0.f);
		} else { // remaining case: project orthogonally to generator segment of upper cone
			gamma.x = (reg * mu + gamma.x) / (mu * mu + 1.f);
			reg = (gamma.x * mu) / reg; //  reg = tproj_div_t
			gamma.y *= reg;
			gamma.z *= reg;
		}
	}
	G[i] = gamma; // store gamma_new
	gamma -= gamma_old; // compute delta_gamma = gamma_new - gamma_old   = delta_gamma.
	dG[i] = length(gamma);
	//if(i==0){printf("%f %f %f\n",gamma.x,gamma.y,gamma.z);}
	vB = N * gamma.x + U * gamma.y + W * gamma.z;
	int offset1 = offset[i];
	int offset2 = offset[i + number_of_contacts_const];
	if (aux1.x == 1) {
		updateV[offset1] = -vB * aux1.z; // compute and store dv1
		updateO[offset1] = (T3 * gamma.x + T4 * gamma.y + T5 * gamma.z) * B1; // compute dw1 =  Inert.1' * J1w^ * deltagamma  and store  dw1
	}
	if (aux2.x == 1) {
		updateV[offset2] = vB * aux2.z; // compute and store dv2
		updateO[offset2] = (T6 * gamma.x + T7 * gamma.y + T8 * gamma.z) * B2; // compute dw2 =  Inert.2' * J2w^ * deltagamma  and store  dw2
	}
}
///////////////////////////////////////////////////////////////////////////////////
// Kernel for a single iteration of the LCP over all scalar bilateral contacts
// (a bit similar to the ChKernelLCPiteration above, but without projection etc.)
// Version 2.0 - Tasora
//
__global__ void LCP_Iteration_Bilaterals(CH_REALNUMBER4* bilaterals, float3* aux, float3* inertia, float4* rot, float3* vel, float3* omega, float3* pos, float3* updateV, float3* updateO,
		uint* offset, float * dG) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= number_of_bilaterals_const) {
		return;
	}
	float4 vA;
	float3 vB;
	float gamma_new = 0, gamma_old;
	int B1_index = 0, B2_index = 0;
	B1_index = bilaterals[i].w;
	B2_index = bilaterals[i + number_of_bilaterals_const].w;
	float3 aux1 = aux[B1_index];
	float3 aux2 = aux[B2_index];
	// ---- perform   gamma_new = ([J1 J2] {v1 | v2}^ + b) 
	vA = bilaterals[i]; // line 0
	vB = vel[B1_index]; // v1
	gamma_new += dot3(vA, vB);

	vA = bilaterals[i + 2 * number_of_bilaterals_const];// line 2
	vB = omega[B1_index]; // w1
	gamma_new += dot3(vA, vB);

	vA = bilaterals[i + number_of_bilaterals_const]; // line 1
	vB = vel[B2_index]; // v2
	gamma_new += dot3(vA, vB);

	vA = bilaterals[i + 3 * number_of_bilaterals_const];// line 3
	vB = omega[B2_index]; // w2
	gamma_new += dot3(vA, vB);

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
	dG[number_of_contacts_const + i] = (gamma_new);
	/// ---- compute dv1 =  invInert.18 * J1^ * deltagamma
	vB = inertia[B1_index]; // iJ iJ iJ im
	vA = (bilaterals[i]) * aux1.z * gamma_new; // line 0: J1(x)
	int offset1 = offset[2 * number_of_contacts_const + i];
	int offset2 = offset[2 * number_of_contacts_const + i + number_of_bilaterals_const];
	updateV[offset1] = F3(vA);//  ---> store  v1 vel. in reduction buffer
	updateO[offset1] = F3(bilaterals[i + 2 * number_of_bilaterals_const]) * vB * gamma_new;// line 2:  J1(w)// ---> store  w1 vel. in reduction buffer
	vB = inertia[B2_index]; // iJ iJ iJ im
	vA = (bilaterals[i + number_of_bilaterals_const]) * aux2.z * gamma_new; // line 1: J2(x)
	updateV[offset2] = F3(vA);//  ---> store  v2 vel. in reduction buffer
	updateO[offset2] = F3(bilaterals[i + 3 * number_of_bilaterals_const]) * vB * gamma_new;// line 3:  J2(w)// ---> store  w2 vel. in reduction buffer
}

__device__ __host__ inline float4 computeRot_dt(float3 & omega, float4 &rot) {
	return mult(F4(0, omega.x, omega.y, omega.z), rot) * .5;
}
__device__ __host__ float3 RelPoint_AbsSpeed(float3 & vel, float3 & omega, float4 & rot, float3 &point) {
	float4 q = mult(computeRot_dt(omega, rot), mult(F4(0, point.x, point.y, point.z), inv(rot)));
	return vel + ((F3(q.y, q.z, q.w)) * 2);
}
__global__ void Warm_Contacts(float3* norm, float3* ptA, float3* ptB, float* contactDepth, int2* ids, float3* aux, float3* inertia, float4* rot, float3* vel, float3* omega, float3* pos, float3 *gamma) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= number_of_contacts_const) {
		return;
	}
	if (length(gamma[i]) != 0) {
		return;
	}
	float mkn = 1000, mgn = 200, mgt = 200, mkt = 100;
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
	//float3 local_pA = quatRotate(pA - posA, inv(rotA));
	//float3 local_pB = quatRotate(pB - posB, inv(rotB));

	//float3 v_BA = (B1-B2);//(RelPoint_AbsSpeed(B2, oB, rotB, local_pB)) - (RelPoint_AbsSpeed(B1, oA, rotA, local_pA));
	//float3 v_n = normalize(dot(v_BA, vN) * vN);
	//float m_eff = (1.0 / aux1.z) * (1.0 / aux2.z) / (1.0 / aux1.z + 1.0 / aux2.z);
	//f_n += mgn * m_eff * v_n;

	//float mu = (aux1.y + aux2.y) * .5;
	//float3 v_t = v_BA - v_n;

	//float3 f_t = (mgt * m_eff * v_t) + (mkt * (v_t * step_size_const));

	//if (length(f_t) > mu * length(f_n)) {
	//	f_t *= mu * length(f_n) / length(f_t);
	//}

	float3 f_r = f_n ;//+ f_t;
	f_r *= step_size_const;

	gamma[i] = f_r;

}
__global__ void DEM_Contacts(float3* norm, float3* ptA, float3* ptB, float* contactDepth, int2* ids, float3* aux, float3* inertia, float4* rot, float3* vel, float3* omega, float3* pos,
		float3* updateV, float3* updateO, uint* offset) {
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
	if (i < number_of_bodies_const) {
		float3 temp_aux = aux[i];
		if (temp_aux.x != 0) {
			float3 mF, minvMasses = inertia[i];
			// v += m_inv * h * f
			mF = forces[i]; // vector with f (force)
			mF *= step_size_const;
			mF *= temp_aux.z;
			vel[i] += mF;
			// w += J_inv * h * c
			mF = torques[i]; // vector with f (torque)
			mF *= step_size_const;
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
__device__ __host__ uint nearest_pow(uint num) {
	uint n = num > 0 ? num - 1 : 0;

	n |= n >> 1;
	n |= n >> 2;
	n |= n >> 4;
	n |= n >> 8;
	n |= n >> 16;
	n++;

	return n;
}
__global__ void LCP_Reduce_Speeds(float3* aux, float3* vel, float3* omega, float3* updateV, float3* updateO, uint* d_body_num, uint* counter) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= number_of_updates_const) {
		return;
	}
	int start = (i == 0) ? 0 : counter[i - 1], end = counter[i];
	int id = d_body_num[end - 1], j;
	if (aux[id].x == 1) {
		float3 mUpdateV = F3(0);
		float3 mUpdateO = F3(0);
		for (j = 0; j < end - start; j++) {
			mUpdateV += updateV[j + start];
			mUpdateO += updateO[j + start];
		}
		vel[id] += mUpdateV;
		omega[id] += mUpdateO;
	}
}
//  Kernel for performing the time step integration (with 1st o;rder Eulero)
//  on the body data stream.
//
//  number of registers: 12 (suggested 320 threads/block)

__global__ void LCP_Integrate_Timestep(float3* aux, float3* acc, float4* rot, float3* vel, float3* omega, float3* pos, float3* lim) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= number_of_bodies_const) {
		return;
	}
	float3 velocity = vel[i];
	float3 aux1 = aux[i];
	if (aux1.x == 0) {
		return;
	}

	pos[i] = pos[i] + velocity * step_size_const; // Do 1st order integration of linear speeds

	//bodies[i] = velocity;
	// Do 1st order integration of quaternion position as q[t+dt] = qw_abs^(dt) * q[dt] = q[dt] * qw_local^(dt)
	// where qw^(dt) is the quaternion { cos(0.5|w|), wx/|w| sin(0.5|w|), wy/|w| sin(0.5|w|), wz/|w| sin(0.5|w|)}^dt
	// that is argument of sine and cosines are multiplied by dt.
	float3 omg = omega[i];
	float3 limits = lim[i];
	float wlen = sqrtf(dot3(omg, omg));
	//if (limits.x == 1) {
			float w = 2.0 * wlen;
			if (w > limits.z) {
				omg *= limits.z / w;
			}

			float v = length(velocity);
			if (v > limits.y) {
				velocity *= limits.y / v;
			}
			vel[i] = velocity;
			omega[i] = omg;
	//}



	float4 Rw = (fabs(wlen) > 10e-10) ? Quat_from_AngAxis(step_size_const * wlen, omg / wlen) : F4(1., 0, 0, 0);// to avoid singularity for near zero angular speed

	float4 mq = Quaternion_Product(rot[i], Rw);
	mq *= rsqrtf(dot(mq, mq));
	rot[i] = mq;
	acc[i] = (velocity - acc[i]) / step_size_const;
}
__global__ void LCP_ComputeGyro(float3* omega, float3* inertia, float3* gyro, float3* torque) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= number_of_bodies_const) {
		return;
	}
	float3 body_inertia = inertia[i];
	body_inertia = F3(1 / body_inertia.x, 1 / body_inertia.y, 1 / body_inertia.z);
	float3 body_omega = omega[i];
	float3 gyr = cross(body_omega, body_inertia * body_omega);
	gyro[i] = gyr;
	torque[i] -= gyr;

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
ChLcpIterativeSolverGPU::~ChLcpIterativeSolverGPU() {
	body_number.clear();
	update_number.clear();
	offset_counter.clear();
	update_offset.clear();
	device_bilateral_data.clear();
}
void ChLcpIterativeSolverGPU::WarmContact() {

	if (use_DEM == false) {
Warm_Contacts	<<< BLOCKS(number_of_contacts), THREADS >>>(
			CASTF3(data_container->device_norm_data),
			CASTF3(data_container->device_cpta_data),
			CASTF3(data_container->device_cptb_data),
			CASTF1(data_container->device_dpth_data),
			CASTI2(data_container->device_bids_data),
			CASTF3(data_container->device_aux_data),
			CASTF3(data_container->device_inr_data),
			CASTF4(data_container->device_rot_data),
			CASTF3(data_container->device_vel_data),
			CASTF3(data_container->device_omg_data),
			CASTF3(data_container->device_pos_data),
			CASTF3(data_container->device_gam_data));
}
}
void ChLcpIterativeSolverGPU::RunTimeStep() {
	number_of_bodies = data_container->number_of_objects;
	number_of_contacts = data_container->number_of_contacts;
	number_of_constraints = number_of_contacts + number_of_bilaterals;

	data_container->device_gyr_data.resize(number_of_bodies);

	COPY_TO_CONST_MEM(c_factor);
	COPY_TO_CONST_MEM(negated_recovery_speed);
	COPY_TO_CONST_MEM(lcp_omega_bilateral);
	COPY_TO_CONST_MEM(lcp_omega_contact);
	COPY_TO_CONST_MEM(step_size);
	COPY_TO_CONST_MEM(number_of_contacts);
	COPY_TO_CONST_MEM(number_of_bilaterals);
	COPY_TO_CONST_MEM(number_of_bodies);

	LCP_ComputeGyro<<< BLOCKS(number_of_bodies),THREADS>>>(
			CASTF3(data_container->device_omg_data),
			CASTF3(data_container->device_inr_data),
			CASTF3(data_container->device_gyr_data),
			CASTF3(data_container->device_trq_data));

	ChKernelLCPaddForces<<< BLOCKS(number_of_bodies), THREADS >>>(
			CASTF3(data_container->device_aux_data),
			CASTF3(data_container->device_inr_data),
			CASTF3(data_container->device_frc_data),
			CASTF3(data_container->device_trq_data),
			CASTF3(data_container->device_vel_data),
			CASTF3(data_container->device_omg_data));

	if (number_of_constraints > 0) {

		device_bilateral_data = host_bilateral_data;

		update_number.resize((number_of_constraints) * 2, 0);
		offset_counter.resize((number_of_constraints) * 2, 0);
		update_offset.resize((number_of_constraints) * 2, 0);
		body_number.resize((number_of_constraints) * 2, 0);
		device_dgm_data.resize((number_of_constraints), (1));
		Thrust_Fill(device_dgm_data,1);
		vel_update.resize((number_of_constraints) * 2);
		omg_update.resize((number_of_constraints) * 2);

		ChKernelOffsets<<< BLOCKS(number_of_constraints),THREADS>>>(
				CASTI2(data_container->device_bids_data),
				CASTF4(device_bilateral_data),
				CASTU1(body_number));

		Thrust_Sequence(update_number);
		Thrust_Sequence(update_offset);
		Thrust_Fill(offset_counter,0);
		Thrust_Sort_By_Key(body_number, update_number);
		Thrust_Sort_By_Key(update_number,update_offset);
		thrust::device_vector<uint> body_number2 = body_number;
		Thrust_Reduce_By_KeyB(number_of_updates,body_number,update_number,offset_counter);
		Thrust_Inclusive_Scan(offset_counter);

		COPY_TO_CONST_MEM(number_of_updates);
		//WarmContact();
		for (iteration_number = 0; iteration_number < maximum_iterations; iteration_number++) {
			if (use_DEM == false) {

LCP_Iteration_Contacts			<<< BLOCKS(number_of_contacts), THREADS >>>(
					CASTF3(data_container->device_norm_data),
					CASTF3(data_container->device_cpta_data),
					CASTF3(data_container->device_cptb_data),
					CASTF1(data_container->device_dpth_data),
					CASTI2(data_container->device_bids_data),
					CASTF3(data_container->device_gam_data),
					CASTF1(device_dgm_data),
					CASTF3(data_container->device_aux_data),
					CASTF3(data_container->device_inr_data),
					CASTF4(data_container->device_rot_data),
					CASTF3(data_container->device_vel_data),
					CASTF3(data_container->device_omg_data),
					CASTF3(data_container->device_pos_data),
					CASTF3(vel_update),
					CASTF3(omg_update),
					CASTU1(update_offset));
		} else {
			DEM_Contacts<<< BLOCKS(number_of_contacts), THREADS >>>(
					CASTF3(data_container->device_norm_data),
					CASTF3(data_container->device_cpta_data),
					CASTF3(data_container->device_cptb_data),
					CASTF1(data_container->device_dpth_data),
					CASTI2(data_container->device_bids_data),
					CASTF3(data_container->device_aux_data),
					CASTF3(data_container->device_inr_data),
					CASTF4(data_container->device_rot_data),
					CASTF3(data_container->device_vel_data),
					CASTF3(data_container->device_omg_data),
					CASTF3(data_container->device_pos_data),
					CASTF3(vel_update),
					CASTF3(omg_update),
					CASTU1(update_offset));
		}
		LCP_Iteration_Bilaterals<<< BLOCKS(number_of_bilaterals), THREADS >>>(
				CASTF4(device_bilateral_data),
				CASTF3(data_container->device_aux_data),
				CASTF3(data_container->device_inr_data),
				CASTF4(data_container->device_rot_data),
				CASTF3(data_container->device_vel_data),
				CASTF3(data_container->device_omg_data),
				CASTF3(data_container->device_pos_data),
				CASTF3(vel_update),
				CASTF3(omg_update),
				CASTU1(update_offset),
				CASTF1(device_dgm_data));

		LCP_Reduce_Speeds<<< BLOCKS(number_of_updates), THREADS >>>(
				CASTF3(data_container->device_aux_data),
				CASTF3(data_container->device_vel_data),
				CASTF3(data_container->device_omg_data),
				CASTF3(vel_update),
				CASTF3(omg_update),
				CASTU1(body_number2),
				CASTU1(offset_counter));
		if (use_DEM == true) {break;}
		if(Avg_DeltaGamma()<tolerance) {break;}
	}
}
LCP_Integrate_Timestep<<< BLOCKS(number_of_bodies),THREADS>>>(
		CASTF3(data_container->device_aux_data),
		CASTF3(data_container->device_acc_data),
		CASTF4(data_container->device_rot_data),
		CASTF3(data_container->device_vel_data),
		CASTF3(data_container->device_omg_data),
		CASTF3(data_container->device_pos_data),
		CASTF3(data_container->device_lim_data));
}
float ChLcpIterativeSolverGPU::Max_DeltaGamma() {
	return Thrust_Max(device_dgm_data);
}
float ChLcpIterativeSolverGPU::Min_DeltaGamma() {
	return Thrust_Min(device_dgm_data);
}
float ChLcpIterativeSolverGPU::Avg_DeltaGamma() {
	return (Thrust_Total(device_dgm_data)) / float(number_of_constraints);
}
__global__ void Compute_KE(float3* vel, float3* aux, float* ke) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= number_of_bodies_const) {
		return;
	}
	float3 velocity = vel[i];
	ke[i] = .5 / aux[i].z * dot(velocity, velocity);

}
float ChLcpIterativeSolverGPU::Total_KineticEnergy() {
	device_ken_data.resize(number_of_bodies);
	Compute_KE<<< BLOCKS(number_of_bodies),THREADS>>>(
			CASTF3(data_container->device_vel_data),
			CASTF3(data_container->device_aux_data),
			CASTF1(device_ken_data));
	return (Thrust_Total(device_ken_data));
}
