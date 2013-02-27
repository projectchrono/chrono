#include "ChLcpSolverGPU.h"

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

__device__ void inline Compute_Jacobian(
    const real4 &quaternion_rotation,
    const real3 &normal,
    const real3 &tangent_u,
    const real3 &tangent_w,
    const real3 &point,
    real3 &T3,
    real3 &T4,
    real3 &T5
) {
    real t1 = quaternion_rotation.y * quaternion_rotation.z;
    real t2 = quaternion_rotation.x * quaternion_rotation.w;
    real t3 = 2 * t1 - 2 * t2;
    real t4 = quaternion_rotation.y * quaternion_rotation.w;
    real t5 = quaternion_rotation.x * quaternion_rotation.z;
    real t6 = 2 * t4 + 2 * t5;
    real t7 = quaternion_rotation.z * quaternion_rotation.w;
    real t8 = quaternion_rotation.x * quaternion_rotation.y;
    real t9 = 2 * t7 - 2 * t8;
    real t10 = quaternion_rotation.x * quaternion_rotation.x;
    real t11 = quaternion_rotation.y * quaternion_rotation.y;
    real t12 = quaternion_rotation.w * quaternion_rotation.w;
    real t13 = quaternion_rotation.z * quaternion_rotation.z;
    real t14 = t10 - t11 - t13 + t12;
    real t15 = t6 * point.x + t9 * point.y + t14 * point.z;
    real t16 = t10 - t11 + t13 - t12;
    t7 = 2 * t7 + 2 * t8;
    t8 = -t3 * point.x - t16 * point.y - t7 * point.z;
    real t17 = t3 * t15 + t6 * t8;
    real t18 = t16 * t15 + t9 * t8;
    real t19 = t7 * t15 + t14 * t8;
    t10 = t10 + t11 - t13 - t12;
    t11 = -t15;
    t1 = 2 * t1 + 2 * t2;
    t2 = 2 * t4 - 2 * t5;
    t4 = t10 * point.x + t1 * point.y + t2 * point.z;
    t5 = t10 * t11 + t6 * t4;
    t6 = t1 * t11 + t9 * t4;
    t9 = t2 * t11 + t14 * t4;
    t8 = -t8;
    t4 = -t4;
    t3 = t10 * t8 + t3 * t4;
    t1 = t1 * t8 + t16 * t4;
    t2 = t2 * t8 + t7 * t4;
    T3.x = normal.x * t17 + normal.y * t18 + normal.z * t19;
    T3.y = normal.x * t5 + normal.y * t6 + normal.z * t9;
    T3.z = normal.x * t3 + normal.y * t1 + normal.z * t2;
    T4.x = tangent_u.x * t17 + tangent_u.y * t18 + tangent_u.z * t19;
    T4.y = tangent_u.x * t5 + tangent_u.y * t6 + tangent_u.z * t9;
    T4.z = tangent_u.x * t3 + tangent_u.y * t1 + tangent_u.z * t2;
    T5.x = tangent_w.x * t17 + tangent_w.y * t18 + tangent_w.z * t19;
    T5.y = tangent_w.x * t5 + tangent_w.y * t6 + tangent_w.z * t9;
    T5.z = tangent_w.x * t3 + tangent_w.y * t1 + tangent_w.z * t2;
}

//  Kernel for a single iteration of the LCP over all contacts
//  Version 2.0 - Tasora
//  Version 2.2- Hammad (optimized, cleaned etc)
__global__ void LCP_Iteration_Contacts(
    real3 *norm,
    real3 *ptA,
    real3 *ptB,
    real *contactDepth,
    int2 *ids,
    real3 *G,
    real *dG,
    real3 *aux,
    real3 *inertia,
    real4 *rot,
    real3 *vel,
    real3 *omega,
    real3 *pos,
    real3 *updateV,
    real3 *updateO,
    uint *offset) {
    INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_contacts_const);
    real3 gamma, T3, T4, T5, T6, T7, T8;
    int2 temp_id = ids[index];
    real depth = -fabs(contactDepth[index]);
    int B1_i = temp_id.x;
    int B2_i = temp_id.y;
    real3 N = norm[index]; //assume: normalized, and if depth=0 norm=(1,0,0)
    real3 W = fabs(N); //Gramm Schmidt; work with the global axis that is "most perpendicular" to the contact normal vector;effectively this means looking for the smallest component (in abs) and using the corresponding direction to carry out cross product.
    real3 U = real3(0, N.z, -N.y); //it turns out that X axis is closest to being perpendicular to contact vector;

    if (W.x > W.y) {
        U = real3(-N.z, 0, N.x);
    } //it turns out that Y axis is closest to being perpendicular to contact vector;

    if (W.y > W.z) {
        U = real3(N.y, -N.x, 0);
    } //it turns out that Z axis is closest to being perpendicular to contact vector;

    U = normalize(U); //normalize the local contact Y,Z axis
    W = cross(N, U); //carry out the last cross product to find out the contact local Z axis : multiply the contact normal by the local Y component
    real3 sbar = ptA[index] - pos[B1_i]; //Contact Point on A - Position of A
    real4 E1 = rot[B1_i]; //bring in the Euler parameters associated with body 1;
    Compute_Jacobian(E1, N, U, W, sbar, T3, T4, T5); //A_i,p'*A_A*(sbar~_i,A)
    sbar = ptB[index] - pos[B2_i]; //Contact Point on B - Position of B
    real4 E2 = rot[B2_i]; //bring in the Euler parameters associated with body 2;
    Compute_Jacobian(E2, N, U, W, sbar, T6, T7, T8); //A_i,p'*A_B*(sbar~_i,B)
    T6 = -T6;
    T7 = -T7;
    T8 = -T8;
//  real normV = dot(N, ((V2 - V1)));
//  normV = (normV > 0) ? 0 : normV;
    real cfm = 0, cfmT = 0;
//  if (compliance_const) {
//      real h = step_size_const;
//      real inv_hpa = 1.0 / (h + alpha_const); // 1/(h+a)
//      real inv_hhpa = 1.0 / (h * (h + alpha_const)); // 1/(h*(h+a))
//      bi = inv_hpa * depth;
//      cfm = inv_hhpa * compliance_const;
//      cfmT = inv_hhpa * complianceT_const;
//  } else {
    real bi = fmaxf((depth / (step_size_const)), -lcp_contact_factor_const);
//  }
    //c_i = [Cq_i]*q + b_i + cfm_i*l_i
    real3 W1 = omega[B1_i];
    real3 W2 = omega[B2_i];
    real3 V1 = vel[B1_i];
    real3 V2 = vel[B2_i];
    gamma.x = dot(T3, W1) - dot(N, V1) + dot(T6, W2) + dot(N, V2) + bi /*+ cfm * gamma_old.x*/; //+bi
    gamma.y = dot(T4, W1) - dot(U, V1) + dot(T7, W2) + dot(U, V2) /*+ cfmT * gamma_old.y*/;
    gamma.z = dot(T5, W1) - dot(W, V1) + dot(T8, W2) + dot(W, V2) /*+ cfmT * gamma_old.z*/;
    real3 In1 = inertia[B1_i]; // bring in the inertia attributes; to be used to compute \eta
    real3 In2 = inertia[B2_i]; // bring in the inertia attributes; to be used to compute \eta
    real3 aux1 = aux[B1_i];
    real3 aux2 = aux[B2_i];
    real eta = dot(T3 * T3, In1) + dot(T4 * T4, In1) + dot(T5 * T5, In1); // update expression of eta
    eta += dot(T6 * T6, In2) + dot(T7 * T7, In2) + dot(T8 * T8, In2);
    eta += (dot(N, N) + dot(U, U) + dot(W, W)) * (aux1.z + aux2.z); // multiply by inverse of mass matrix of B1 and B2, add contribution from mass and matrix A_c.
    eta = lcp_omega_contact_const / eta; // final value of eta
    real3 gamma_old = G[index];
    gamma = eta * gamma; // perform gamma *= omega*eta
    gamma = gamma_old - gamma; // perform gamma = gamma_old - gamma ;  in place.
    /// ---- perform projection of 'a8' onto friction cone  --------
    real f_tang = sqrtf(gamma.y * gamma.y + gamma.z * gamma.z);
    real mu = (aux1.y + aux2.y) * .5f;

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

    G[index] = gamma; // store gamma_new
    gamma -= gamma_old; // compute delta_gamma = gamma_new - gamma_old   = delta_gamma.
    dG[index] = length(gamma);
    real3 vB = N * gamma.x + U * gamma.y + W * gamma.z;
    int offset1 = offset[index];
    int offset2 = offset[index + number_of_contacts_const];
    updateV[offset1] = real3(-vB * aux1.z); // compute and store dv1
    updateO[offset1] = real3((T3 * gamma.x + T4 * gamma.y + T5 * gamma.z) * In1); // compute dw1 =  Inert.1' * J1w^ * deltagamma  and store  dw1
    updateV[offset2] = real3(vB * aux2.z); // compute and store dv2
    updateO[offset2] = real3((T6 * gamma.x + T7 * gamma.y + T8 * gamma.z) * In2); // compute dw2 =  Inert.2' * J2w^ * deltagamma  and store  dw2
}
///////////////////////////////////////////////////////////////////////////////////
// Kernel for a single iteration of the LCP over all scalar bilateral contacts
// (a bit similar to the ChKernelLCPiteration above, but without projection etc.)
// Version 2.0 - Tasora
//

__global__ void LCP_Iteration_Bilaterals(
    real4 *bilaterals,
    real3 *aux,
    real3 *inertia,
    real4 *rot,
    real3 *vel,
    real3 *omega,
    real3 *pos,
    real3 *updateV,
    real3 *updateO,
    uint *offset,
    real *dG) {
    INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_bilaterals_const);
    real4 vA;
    real3 vB;
    real gamma_new = 0, gamma_old = 0;
    int B1_index = 0, B2_index = 0;
    B1_index = bilaterals[index].w;
    B2_index = bilaterals[index + number_of_bilaterals_const].w;
    real3 aux1 = aux[B1_index];
    real3 aux2 = aux[B2_index];
    // ---- perform   gamma_new = ([J1 J2] {v1 | v2}^ + b)
    {
        vA = bilaterals[index]; // line 0
        vB = vel[B1_index]; // v1
        gamma_new += dot3(vA, vB);
        vA = bilaterals[index + 2 * number_of_bilaterals_const]; // line 2
        vB = omega[B1_index]; // w1
        gamma_new += dot3(vA, vB);
    }
    {
        vA = bilaterals[index + number_of_bilaterals_const]; // line 1
        vB = vel[B2_index]; // v2
        gamma_new += dot3(vA, vB);
        vA = bilaterals[index + 3 * number_of_bilaterals_const]; // line 3
        vB = omega[B2_index]; // w2
        gamma_new += dot3(vA, vB);
    }
    vA = bilaterals[index + 4 * number_of_bilaterals_const]; // line 4   (eta, b, gamma, 0)
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
    bilaterals[index + 4 * number_of_bilaterals_const] = vA;
    /// ---- compute delta in multipliers: gamma_new = gamma_new - gamma_old   = delta_gamma    , in place.
    gamma_new -= gamma_old;
    dG[number_of_contacts_const + index] = (gamma_new);
    /// ---- compute dv1 =  invInert.18 * J1^ * deltagamma
    vB = inertia[B1_index]; // iJ iJ iJ im
    vA = (bilaterals[index]) * aux1.z * gamma_new; // line 0: J1(x)
    int offset1 = offset[2 * number_of_contacts_const + index];
    int offset2 = offset[2 * number_of_contacts_const + index + number_of_bilaterals_const];
    updateV[offset1] = make_real3(vA); //  ---> store  v1 vel. in reduction buffer
    updateO[offset1] = make_real3((bilaterals[index + 2 * number_of_bilaterals_const]) * make_real4(vB) * gamma_new); // line 2:  J1(w)// ---> store  w1 vel. in reduction buffer
    vB = inertia[B2_index]; // iJ iJ iJ im
    vA = (bilaterals[index + number_of_bilaterals_const]) * aux2.z * gamma_new; // line 1: J2(x)
    updateV[offset2] = make_real3(vA); //  ---> store  v2 vel. in reduction buffer
    updateO[offset2] = make_real3((bilaterals[index + 3 * number_of_bilaterals_const]) * make_real4(vB) * gamma_new); // line 3:  J2(w)// ---> store  w2 vel. in reduction buffer
}

__device__ __host__ inline real4 computeRot_dt(real3 &omega, real4 &rot) {
    return mult(R4(0, omega.x, omega.y, omega.z), rot) * .5;
}
__device__ __host__ real3 RelPoint_AbsSpeed(real3 &vel, real3 &omega, real4 &rot, real3 &point) {
    real4 q = mult(computeRot_dt(omega, rot), mult(R4(0, point.x, point.y, point.z), inv(rot)));
    return vel + ((R3(q.y, q.z, q.w)) * 2);
}

__global__ void DEM_Contacts(
    real3 *norm,
    real3 *ptA,
    real3 *ptB,
    real *contactDepth,
    int2 *ids,
    real3 *aux,
    real3 *inertia,
    real4 *rot,
    real3 *vel,
    real3 *omega,
    real3 *pos,
    real3 *updateV,
    real3 *updateO,
    uint *offset) {
    INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_contacts_const);
    real mkn = 3924, mgn = 420, mgt = 420, mkt = 2.0f / 7.0f * 3924;
    //long long id=ids[index];
    int2 temp_id = ids[index];
    int B1_i = int(temp_id.x);
    int B2_i = int(temp_id.y);
    real3 vN = norm[index]; //normal
    real3 pA = ptA[index];
    real3 pB = ptB[index];
    real3 B1 = vel[B1_i]; //vel B1
    real3 B2 = vel[B2_i]; //vel B2
    real3 oA = omega[B1_i];
    real3 oB = omega[B2_i];
    real4 rotA = rot[B1_i];
    real4 rotB = rot[B2_i];
    real3 aux1 = aux[B1_i];
    real3 aux2 = aux[B2_i];
    real3 posA = pos[B1_i];
    real3 posB = pos[B2_i];
    real3 f_n = mkn * -fabs(contactDepth[index]) * vN;
    real3 local_pA = quatRotate(pA - posA, inv(rotA));
    real3 local_pB = quatRotate(pB - posB, inv(rotB));
    real3 v_BA = (RelPoint_AbsSpeed(B2, oB, rotB, local_pB)) - (RelPoint_AbsSpeed(B1, oA, rotA, local_pA));
    real3 v_n = normalize(dot(v_BA, vN) * vN);
    real m_eff = (1.0 / aux1.z) * (1.0 / aux2.z) / (1.0 / aux1.z + 1.0 / aux2.z);
    f_n += mgn * m_eff * v_n;
    real mu = (aux1.y + aux2.y) * .5;
    real3 v_t = v_BA - v_n;
    real3 f_t = (mgt * m_eff * v_t) + (mkt * (v_t * step_size_const));

    if (length(f_t) > mu * length(f_n)) {
        f_t = f_t * mu * length(f_n) / length(f_t);
    }

    real3 f_r = f_n + f_t;
    int offset1 = offset[index];
    int offset2 = offset[index + number_of_contacts_const];
    real3 force1_loc = quatRotate(f_r, inv(rotA));
    real3 force2_loc = quatRotate(f_r, inv(rotB));
    real3 trq1 = cross(local_pA, force1_loc);
    real3 trq2 = cross(local_pB, -force2_loc);
    f_r = f_r * step_size_const;
    updateV[offset1] = (f_r) * aux1.z;
    updateV[offset2] = (f_r) * -aux2.z;
    updateO[offset1] = (trq1 * step_size_const) * (inertia[B1_i]);
    updateO[offset2] = (trq2 * step_size_const) * (inertia[B2_i]);
}

////////////////////////////////////////////////////////////////////////////////////////////////
// Kernel for adding invmass*force*step_size_const to body speed vector.
// This kernel must be applied to the stream of the body buffer.

__global__ void ChKernelLCPaddForces(real3 *aux, real3 *inertia, real3 *forces, real3 *torques, real3 *vel, real3 *omega) {
    // Compute the index values used to access data inside the large
    // array using pointer arithmetic.
    INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_objects_const);
    real3 temp_aux = aux[index];

    if (temp_aux.x != 0) {
        real3 mF, minvMasses = inertia[index];
        // v += m_inv * h * f
        mF = forces[index]; // vector with f (force)
        //mF *= 1;//step_size_const;
        mF = mF * temp_aux.z;
        vel[index] += mF;
        // w += J_inv * h * c
        mF = torques[index]; // vector with f (torque)
        //mF *= 1;//step_size_const;
        mF.x *= minvMasses.x;
        mF.y *= minvMasses.y;
        mF.z *= minvMasses.z;
        omega[index] += mF;
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// Updates the speeds in the body buffer with values accumulated in the
// reduction buffer:   V_new = V_old + delta_speeds

__global__ void LCP_Reduce_Speeds(real3 *aux, real3 *vel, real3 *omega, real3 *updateV, real3 *updateO, uint *d_body_num, uint *counter, real3 *fap) {
    INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_updates_const);
    int start = (index == 0) ? 0 : counter[index - 1], end = counter[index];
    int id = d_body_num[end - 1], j;
    real3 auxd = aux[id];

    if (auxd.x == 0) {
        return;
    }

    real3 mUpdateV = real3(0);
    real3 mUpdateO = real3(0);

    for (j = 0; j < end - start; j++) {
        mUpdateV = mUpdateV + updateV[j + start];
        mUpdateO = mUpdateO + updateO[j + start];
    }

    fap[id] += (mUpdateV / auxd.z) / step_size_const;
    vel[id] += (mUpdateV);
    omega[id] += (mUpdateO);
}
//  Kernel for performing the time step integration (with 1st o;rder Eulero)
//  on the body data stream.
//
//  number of registers: 12 (suggested 320 threads/block)

__global__ void LCP_Integrate_Timestep(real3 *aux, real3 *acc, real4 *rot, real3 *vel, real3 *omega, real3 *pos, real3 *lim) {
    INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_objects_const);
    real3 velocity = vel[index];
    real3 aux1 = aux[index];

    if (aux1.x == 0) {
        return;
    }

    // Do 1st order integration of quaternion position as q[t+dt] = qw_abs^(dt) * q[dt] = q[dt] * qw_local^(dt)
    // where qw^(dt) is the quaternion { cos(0.5|w|), wx/|w| sin(0.5|w|), wy/|w| sin(0.5|w|), wz/|w| sin(0.5|w|)}^dt
    // that is argument of sine and cosines are multiplied by dt.
    real3 omg = omega[index];
    real3 limits = lim[index];
    real wlen = length(omg);

    if (limits.x == 1) {
        real w = 2.0 * wlen;

        if (w > limits.z) {
            omg = omg * limits.z / w;
            wlen = sqrtf(dot3(omg, omg));
        }

        real v = length(velocity);

        if (v > limits.y) {
            velocity = velocity * limits.y / v;
        }

        vel[index] = velocity;
        omega[index] = omg;
    }

    pos[index] = pos[index] + velocity * step_size_const; // Do 1st order integration of linear speeds
    real4 Rw = (fabs(wlen) > 10e-10) ? Q_from_AngAxis(step_size_const * wlen, omg / wlen) : R4(1., 0, 0, 0); // to avoid singularity for near zero angular speed
    real4 mq = mult(rot[index], Rw);
    mq =mq* rsqrtf(dot(mq, mq));
    rot[index] = mq;
    acc[index] = (velocity - acc[index]) / step_size_const;
}
__global__ void LCP_ComputeGyro(real3 *omega, real3 *inertia, real3 *gyro, real3 *torque) {
    INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_objects_const);
    real3 body_inertia = inertia[index];
    body_inertia = R3(1 / body_inertia.x, 1 / body_inertia.y, 1 / body_inertia.z);
    real3 body_omega = omega[index];
    real3 gyr = cross(body_omega, body_inertia * body_omega);
    gyro[index] = gyr;
}

__global__ void ChKernelOffsets(int2 *ids, real4 *bilaterals, uint *Body) {
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

void ChLcpSolverGPU::WarmContact(const int &index) {
}

void ChLcpSolverGPU::Preprocess(gpu_container &gpu_data) {
    gpu_data.number_of_updates = 0;
    gpu_data.device_gyr_data.resize(number_of_objects);
    thrust::device_vector<uint> body_num;
    thrust::device_vector<uint> update_number;
    LCP_ComputeGyro CUDA_KERNEL_DIM(BLOCKS(number_of_objects), THREADS)(
        CASTR3(gpu_data.device_omg_data),
        CASTR3(gpu_data.device_inr_data),
        CASTR3(gpu_data.device_gyr_data),
        CASTR3(gpu_data.device_trq_data));
    ChKernelLCPaddForces CUDA_KERNEL_DIM(BLOCKS(number_of_objects), THREADS)(
        CASTR3(gpu_data.device_aux_data),
        CASTR3(gpu_data.device_inr_data),
        CASTR3(gpu_data.device_frc_data),
        CASTR3(gpu_data.device_trq_data),
        CASTR3(gpu_data.device_vel_data),
        CASTR3(gpu_data.device_omg_data));
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
        ChKernelOffsets CUDA_KERNEL_DIM(BLOCKS(number_of_constraints), THREADS)(CASTI2(gpu_data.device_bids_data), CASTR4(gpu_data.device_bilateral_data), CASTU1(body_num));
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

void ChLcpSolverGPU::Iterate(gpu_container &gpu_data) {
}
void ChLcpSolverGPU::Reduce(gpu_container &gpu_data) {
}
void ChLcpSolverGPU::Integrate(gpu_container &gpu_data) {
    LCP_Integrate_Timestep CUDA_KERNEL_DIM(BLOCKS(number_of_objects), THREADS)(
        CASTR3(gpu_data.device_aux_data),
        CASTR3(gpu_data.device_acc_data),
        CASTR4(gpu_data.device_rot_data),
        CASTR3(gpu_data.device_vel_data),
        CASTR3(gpu_data.device_omg_data),
        CASTR3(gpu_data.device_pos_data),
        CASTR3(gpu_data.device_lim_data));
}
void ChLcpSolverGPU::RunTimeStep(real step, gpu_container &gpu_data) {
    DBG("");
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
    cudaFuncSetCacheConfig(LCP_Iteration_Contacts, cudaFuncCachePreferL1);
    cudaFuncSetCacheConfig(LCP_Iteration_Bilaterals, cudaFuncCachePreferL1);
    cudaFuncSetCacheConfig(LCP_Reduce_Speeds, cudaFuncCachePreferL1);
    cudaFuncSetCacheConfig(LCP_Integrate_Timestep, cudaFuncCachePreferL1);
    real old_gam = 1;

    if (number_of_constraints != 0) {
        for (iteration_number = 0; iteration_number < max_iterations; iteration_number++) {
            LCP_Iteration_Contacts CUDA_KERNEL_DIM(BLOCKS(number_of_contacts), THREADS)(
                CASTR3(gpu_data.device_norm_data),
                CASTR3(gpu_data.device_cpta_data),
                CASTR3(gpu_data.device_cptb_data),
                CASTR1(gpu_data.device_dpth_data),
                CASTI2(gpu_data.device_bids_data),
                CASTR3(gpu_data.device_gam_data),
                CASTR1(gpu_data.device_dgm_data),
                CASTR3(gpu_data.device_aux_data),
                CASTR3(gpu_data.device_inr_data),
                CASTR4(gpu_data.device_rot_data),
                CASTR3(gpu_data.device_vel_data),
                CASTR3(gpu_data.device_omg_data),
                CASTR3(gpu_data.device_pos_data),
                CASTR3(gpu_data.vel_update),
                CASTR3(gpu_data.omg_update),
                CASTU1(gpu_data.update_offset));
            LCP_Iteration_Bilaterals CUDA_KERNEL_DIM(BLOCKS(number_of_bilaterals), THREADS)(
                CASTR4(gpu_data.device_bilateral_data),
                CASTR3(gpu_data.device_aux_data),
                CASTR3(gpu_data.device_inr_data),
                CASTR4(gpu_data.device_rot_data),
                CASTR3(gpu_data.device_vel_data),
                CASTR3(gpu_data.device_omg_data),
                CASTR3(gpu_data.device_pos_data),
                CASTR3(gpu_data.vel_update),
                CASTR3(gpu_data.omg_update),
                CASTU1(gpu_data.update_offset),
                CASTR1(gpu_data.device_dgm_data));
            LCP_Reduce_Speeds CUDA_KERNEL_DIM(BLOCKS(number_of_updates), THREADS)(
                CASTR3(gpu_data.device_aux_data),
                CASTR3(gpu_data.device_vel_data),
                CASTR3(gpu_data.device_omg_data),
                CASTR3(gpu_data.vel_update),
                CASTR3(gpu_data.omg_update),
                CASTU1(gpu_data.body_number),
                CASTU1(gpu_data.offset_counter),
                CASTR3(gpu_data.device_fap_data));

            if (tolerance != 0) {
                if (iteration_number > 20 && iteration_number % 20 == 0) {
                    real gam = Max_DeltaGamma(gpu_data.device_dgm_data);

                    if (fabsf(old_gam - gam) < tolerance) {
                        break;
                    }

                    old_gam = gam;
                }
            }
        }
    }

    Integrate(gpu_data);
}
real ChLcpSolverGPU::Max_DeltaGamma(device_vector<real> &device_dgm_data) {
    return Thrust_Max(device_dgm_data);
}
real ChLcpSolverGPU::Min_DeltaGamma(device_vector<real> &device_dgm_data) {
    return Thrust_Min(device_dgm_data);
}
real ChLcpSolverGPU::Avg_DeltaGamma(uint number_of_constraints, device_vector<real> &device_dgm_data) {
    real gamma = (Thrust_Total(device_dgm_data)) / real(number_of_constraints);
    //cout << gamma << endl;
    return gamma;
}
__global__ void Compute_KE(real3 *vel, real3 *aux, real *ke) {
    INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_objects_const);
    real3 velocity = vel[index];
    ke[index] = .5 / aux[index].z * dot(velocity, velocity);
}
real ChLcpSolverGPU::Total_KineticEnergy(gpu_container &gpu_data) {
    thrust::device_vector<real> device_ken_data;
    device_ken_data.resize(gpu_data.number_of_objects);
    Compute_KE CUDA_KERNEL_DIM(BLOCKS(gpu_data.number_of_objects), THREADS)(CASTR3(gpu_data.device_vel_data), CASTR3(gpu_data.device_aux_data), CASTR1(device_ken_data));
    return (Thrust_Total(device_ken_data));
}




