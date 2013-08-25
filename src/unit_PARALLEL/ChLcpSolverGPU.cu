#include "ChLcpSolverGPU.h"
#include "ChThrustLinearAlgebra.cuh"
#include "solver/ChSolverGPU.cuh"
#include "ChJacobianGPU.h"
#include "ChIntegratorGPU.h"
#include "ChComputeRHSGPU.h"

using namespace chrono;
__constant__ uint number_of_objects_const;
__constant__ uint number_of_contacts_const;
__constant__ uint number_of_bilaterals_const;
__constant__ uint number_of_updates_const;
__constant__ real step_size_const;

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
#pragma omp parallel for

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
#pragma omp parallel for

	for (uint index = 0; index < number_of_objects; index++) {
		function_ComputeGyro(index, omega, inertia, gyro, torque);
	}
}

void ChLcpSolverGPU::Preprocess() {
	data_container->number_of_updates = 0;
	data_container->host_data.gyr_data.resize(number_of_objects);

#ifdef SIM_ENABLE_GPU_MODE
	COPY_TO_CONST_MEM(number_of_objects);

	device_ComputeGyro CUDA_KERNEL_DIM(BLOCKS(number_of_objects), THREADS)(
			CASTR3(data_container->device_data.device_omg_data),
			CASTR3(data_container->device_data.device_inr_data),
			CASTR3(data_container->device_data.device_gyr_data),
			CASTR3(data_container->device_data.device_trq_data));
	device_addForces CUDA_KERNEL_DIM(BLOCKS(number_of_objects), THREADS)(
			CASTB1(data_container->device_data.device_active_data),
			CASTR1(data_container->device_data.device_mass_data),
			CASTR3(data_container->device_data.device_inr_data),
			CASTR3(data_container->device_data.device_frc_data),
			CASTR3(data_container->device_data.device_trq_data),
			CASTR3(data_container->device_data.device_vel_data),
			CASTR3(data_container->device_data.device_omg_data));
#else
	host_ComputeGyro(
			data_container->host_data.omg_data.data(),
			data_container->host_data.inr_data.data(),
			data_container->host_data.gyr_data.data(),
			data_container->host_data.trq_data.data());

	host_addForces(
			data_container->host_data.active_data.data(),
			data_container->host_data.mass_data.data(),
			data_container->host_data.inr_data.data(),
			data_container->host_data.frc_data.data(),
			data_container->host_data.trq_data.data(),
			data_container->host_data.vel_data.data(),
			data_container->host_data.omg_data.data());
#endif
}

void ChLcpSolverGPU::RunTimeStep(real step) {
	step_size = step;
	data_container->step_size = step;

	number_of_constraints = data_container->number_of_contacts * 3 + data_container->number_of_bilaterals;
	number_of_contacts = data_container->number_of_contacts;
	number_of_bilaterals = 0; ///data_container->number_of_bilaterals;
	number_of_objects = data_container->number_of_objects;
	//cout << number_of_constraints << " " << number_of_contacts << " " << number_of_bilaterals << " " << number_of_objects << endl;
#ifdef PRINT_DEBUG_GPU
	cout << "Preprocess: " << endl;
#endif
	Preprocess();
#ifdef PRINT_DEBUG_GPU
	cout << "Jacobians: " << endl;
#endif
	ChJacobianGPU jacobian_compute;
	jacobian_compute.ComputeJacobians(data_container);

	ChComputeRHSGPU rhs_compute;
	rhs_compute.ComputeRHS(data_container);

#ifdef PRINT_DEBUG_GPU
	cout << "Solve: " << endl;
#endif

		ChSolverGPU solver;
		solver.SetMaxIterations(max_iteration);
		solver.SetTolerance(tolerance);
		solver.SetComplianceParameters(alpha, compliance, complianceT);
		solver.SetContactRecoverySpeed(contact_recovery_speed);
		solver.lcp_omega_bilateral = lcp_omega_bilateral;
		solver.lcp_omega_contact = lcp_omega_contact;
		solver.do_stab = do_stab;
		solver.Solve(solver_type, step, data_container);

		tot_iterations = solver.GetIteration();
		residual = solver.GetResidual();
		rhs = data_container->host_data.rhs_data;
		lambda = data_container->host_data.gam_data;

		for (int i = 0; i < solver.iter_hist.size(); i++) {
			AtIterationEnd(solver.maxd_hist[i], solver.maxdeltalambda_hist[i], solver.iter_hist[i]);
		}
	//stabilization  code
	//jacobian_compute.ComputeJacobians(data_container);
	//rhs_compute.ComputeRHS(data_container);

	//solver.VelocityStabilization(data_container);

#ifdef PRINT_DEBUG_GPU
	cout << "Solve Done: "<<residual << endl;
#endif
	//ChIntegratorGPU integrator;
	//integrator.IntegrateSemiImplicit(step, data_container->gpu_data);
}

