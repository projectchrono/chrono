#include "ChLcpSolverGPU.h"
#include "ChThrustLinearAlgebra.cuh"
#include "solver/ChSolverBlockJacobi.h"
#include "solver/ChSolverGPU.h"

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

__host__ __device__ void inline Compute_Jacobian(
		const real4& quaternion_rotation,
		const real3& normal,
		const real3& tangent_u,
		const real3& tangent_w,
		const real3& point,
		real3& T3,
		real3& T4,
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
__host__ __device__ void inline function_ContactJacobians(
		uint& index,
		uint& num_contacts,
		real3* norm,
		real3* ptA,
		real3* ptB,
		real3* pos,
		real4* rot,
		int2* ids,
		real3* JXYZA,
		real3* JXYZB,
		real3* JUVWA,
		real3* JUVWB) {
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



void ChLcpSolverGPU::Preprocess(gpu_container& gpu_data) {
	gpu_data.number_of_updates = 0;
	gpu_data.device_gyr_data.resize(number_of_objects);

#ifdef SIM_ENABLE_GPU_MODE
	device_ComputeGyro CUDA_KERNEL_DIM(BLOCKS(number_of_objects), THREADS)(CASTR3(gpu_data.device_omg_data), CASTR3(gpu_data.device_inr_data), CASTR3(gpu_data.device_gyr_data),
			CASTR3(gpu_data.device_trq_data));
	device_addForces CUDA_KERNEL_DIM(BLOCKS(number_of_objects), THREADS)(CASTB1(gpu_data.device_active_data), CASTR1(gpu_data.device_mass_data), CASTR3(gpu_data.device_inr_data),
				CASTR3(gpu_data.device_frc_data), CASTR3(gpu_data.device_trq_data), CASTR3(gpu_data.device_vel_data), CASTR3(gpu_data.device_omg_data));
#else
	host_ComputeGyro(
			gpu_data.device_omg_data.data(),
			gpu_data.device_inr_data.data(),
			gpu_data.device_gyr_data.data(),
			gpu_data.device_trq_data.data());

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
	number_of_constraints = gpu_data.number_of_contacts * 3  + gpu_data.number_of_bilaterals;
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

#else
#endif
	Preprocess(gpu_data);

#ifdef SIM_ENABLE_GPU_MODE
	COPY_TO_CONST_MEM(compliance);
	COPY_TO_CONST_MEM(complianceT);
	COPY_TO_CONST_MEM(alpha);
	COPY_TO_CONST_MEM(lcp_omega_bilateral);
	COPY_TO_CONST_MEM(lcp_omega_contact);
	COPY_TO_CONST_MEM(lcp_contact_factor);
	COPY_TO_CONST_MEM(number_of_updates);

	cudaFuncSetCacheConfig(device_Integrate_Timestep, cudaFuncCachePreferL1);
#else
#endif
	real old_gam = 1;
	//COMPUTE JACOBIANS

	gpu_data.device_QXYZ_data.resize(number_of_objects);
	gpu_data.device_QUVW_data.resize(number_of_objects);

	gpu_data.device_JXYZA_data.resize(number_of_constraints);
	gpu_data.device_JXYZB_data.resize(number_of_constraints);
	gpu_data.device_JUVWA_data.resize(number_of_constraints);
	gpu_data.device_JUVWB_data.resize(number_of_constraints);
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
	//ChSolverJacobi solver;
	ChSolverGPU solver;
	solver.Solve(ChSolverGPU::CONJUGATE_GRADIENT,step, gpu_data);
	cout<<solver.GetIteration()<<" "<<solver.GetResidual()<<"\t["<<solver.time_rhs<<"\t"<<solver.time_shurcompliment<<"\t"<<solver.time_project<<"\t"<<solver.time_solver<<"]"<<endl;

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

