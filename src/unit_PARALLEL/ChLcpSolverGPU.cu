#include "ChLcpSolverGPU.h"
#include "ChThrustLinearAlgebra.cuh"
#include "solver/ChSolverBlockJacobi.h"
#include "solver/ChSolverGPU.h"
#include "ChJacobianGPU.h"
#include "ChIntegratorGPU.h"
using namespace chrono;
__constant__ uint number_of_objects_const;
__constant__ uint number_of_contacts_const;
__constant__ uint number_of_bilaterals_const;
__constant__ uint number_of_updates_const;
__constant__ real step_size_const;

//__constant__ real force_factor_const;
//__constant__ real negated_recovery_speed_const;
////////////////////////////////////////////////////////////////////////////////////////////////
// Kernel for adding invmass*force*step_size_const to body speed vector.
// This kernel must be applied to the stream of the body buffer.

__host__ __device__ void function_addForces(uint& index, bool* active, real* mass, real3* inertia, real3* forces, real3* torques, real3* vel, real3* omega) {
	if (active[index] != 0) {
		// v += m_inv * h * f
		vel[index] += forces[index] * mass[index];
		// w += J_inv * h * c
		omega[index] += torques[index] * inertia[index];
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
	COPY_TO_CONST_MEM(number_of_objects);

	device_ComputeGyro CUDA_KERNEL_DIM(BLOCKS(number_of_objects), THREADS)(
			CASTR3(gpu_data.device_omg_data),
			CASTR3(gpu_data.device_inr_data),
			CASTR3(gpu_data.device_gyr_data),
			CASTR3(gpu_data.device_trq_data));
	device_addForces CUDA_KERNEL_DIM(BLOCKS(number_of_objects), THREADS)(
			CASTB1(gpu_data.device_active_data),
			CASTR1(gpu_data.device_mass_data),
			CASTR3(gpu_data.device_inr_data),
			CASTR3(gpu_data.device_frc_data),
			CASTR3(gpu_data.device_trq_data),
			CASTR3(gpu_data.device_vel_data),
			CASTR3(gpu_data.device_omg_data));
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

void ChLcpSolverGPU::RunTimeStep(real step, gpu_container& gpu_data) {
	step_size = step;
	number_of_constraints = gpu_data.number_of_contacts * 3 + gpu_data.number_of_bilaterals;
	number_of_contacts = gpu_data.number_of_contacts;
	number_of_bilaterals = 0; ///gpu_data.number_of_bilaterals;
	number_of_objects = gpu_data.number_of_objects;
	//cout << number_of_constraints << " " << number_of_contacts << " " << number_of_bilaterals << " " << number_of_objects << endl;

	Preprocess(gpu_data);
	ChJacobianGPU jacobian_compute;
	jacobian_compute.ComputeJacobians(gpu_data);
	if (solver_type == BLOCK_JACOBI) {
		ChSolverJacobi jacobi_solver;
		jacobi_solver.SetMaxIterations(max_iteration);

		jacobi_solver.SetTolerance(tolerance);
		jacobi_solver.SetComplianceParameters(alpha, compliance, complianceT);
		jacobi_solver.SetContactRecoverySpeed(-contact_recovery_speed);
		jacobi_solver.lcp_omega_bilateral = lcp_omega_bilateral;
		jacobi_solver.lcp_omega_contact = lcp_omega_contact;
		jacobi_solver.Solve(step, gpu_data);
		current_iteration = jacobi_solver.GetIteration();
		residual = jacobi_solver.GetResidual();

	} else {
		ChSolverGPU solver;
		solver.SetMaxIterations(max_iteration);
		solver.SetTolerance(tolerance);
		solver.SetComplianceParameters(alpha, compliance, complianceT);
		solver.SetContactRecoverySpeed(-contact_recovery_speed);
		solver.Solve(solver_type, step, gpu_data);
		current_iteration = solver.GetIteration();
		residual = solver.GetResidual();

	}

	ChIntegratorGPU integrator;
	integrator.IntegrateSemiImplicit(step, gpu_data);
}

