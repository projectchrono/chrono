#include "ChLcpSolverGPU.h"
#include "ChThrustLinearAlgebra.cuh"
#include "solver/ChSolverGPU.cuh"
#include "ChJacobianGPU.h"
#include "ChIntegratorGPU.h"
#include "ChComputeRHSGPU.h"
#include "constraints/ChConstraintRigidRigid.h"
#include "constraints/ChConstraintBilateral.h"

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
	COPY_TO_CONST_MEM(number_of_rigid);

	device_ComputeGyro CUDA_KERNEL_DIM(BLOCKS(number_of_rigid), THREADS)(
			CASTR3(data_container->device_data.device_omg_data),
			CASTR3(data_container->device_data.device_inr_data),
			CASTR3(data_container->device_data.device_gyr_data),
			CASTR3(data_container->device_data.device_trq_data));
	device_addForces CUDA_KERNEL_DIM(BLOCKS(number_of_rigid), THREADS)(
			CASTB1(data_container->device_data.device_active_data),
			CASTR1(data_container->device_data.device_mass_data),
			CASTR3(data_container->device_data.device_inr_data),
			CASTR3(data_container->device_data.device_frc_data),
			CASTR3(data_container->device_data.device_trq_data),
			CASTR3(data_container->device_data.device_vel_data),
			CASTR3(data_container->device_data.device_omg_data));
#else
//	host_ComputeGyro(
//			data_container->host_data.omg_data.data(),
//			data_container->host_data.inr_data.data(),
//			data_container->host_data.gyr_data.data(),
//			data_container->host_data.trq_data.data());

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

	number_of_constraints = data_container->number_of_rigid_rigid * 6 + data_container->number_of_bilaterals;
	number_of_bilaterals = 0;     ///data_container->number_of_bilaterals;
	number_of_objects = data_container->number_of_rigid;
	//cout << number_of_constraints << " " << number_of_contacts << " " << number_of_bilaterals << " " << number_of_objects << endl;
#ifdef PRINT_DEBUG_GPU
	cout << "Preprocess: " << endl;
#endif
	Preprocess();
#ifdef PRINT_DEBUG_GPU
	cout << "Jacobians: " << endl;
#endif
//	ChJacobianGPU jacobian_compute;
//	jacobian_compute.ComputeJacobians(data_container);
//
//	ChComputeRHSGPU rhs_compute;
//	rhs_compute.ComputeRHS(data_container);

	data_container->host_data.rhs_data.resize(number_of_constraints);
	data_container->host_data.diag.resize(number_of_constraints);

	//bilateral.Diag();

	data_container->host_data.gamma_data.resize((number_of_constraints));

#pragma omp parallel for
	for (int i = 0; i < number_of_constraints; i++) {
		data_container->host_data.gamma_data[i] = 0;
	}
	if (warm_start) {
		//cout<<"WARM"<<endl;
		RunWarmStartPreprocess();
	}

	ChConstraintRigidRigid rigid_rigid(data_container);
	ChConstraintBilateral bilateral(data_container);

	ChSolverGPU solver;
	solver.SetMaxIterations(max_iteration);
	solver.SetTolerance(tolerance);

	solver.SetComplianceParameters(alpha, compliance, complianceT);
	solver.SetContactRecoverySpeed(contact_recovery_speed);
	solver.lcp_omega_bilateral = lcp_omega_bilateral;
	solver.rigid_rigid = &rigid_rigid;
	solver.bilateral = &bilateral;
	solver.lcp_omega_contact = lcp_omega_contact;
	solver.do_stab = do_stab;
	solver.Initial(step, data_container);
//solve initial
	//solver.SetComplianceParameters(.2, 1e-3, 1e-3);
	rigid_rigid.solve_sliding = true;
	rigid_rigid.ComputeJacobians();
	//rigid_rigid.ComputeRHS();
	bilateral.ComputeJacobians();
	bilateral.ComputeRHS();
	//solver.SetMaxIterations(5);
	//solver.Solve(solver_type);

//solve normal
	solver.SetMaxIterations(max_iteration);
	solver.SetComplianceParameters(alpha, compliance, complianceT);
	rigid_rigid.solve_sliding = false;
	rigid_rigid.solve_spinning = false;
	rigid_rigid.ComputeRHS();
	solver.Solve(solver_type);

	//solve full
	solver.SetMaxIterations(max_iteration/2.0);
	rigid_rigid.solve_sliding = true;
	rigid_rigid.solve_spinning = false;
	rigid_rigid.ComputeRHS();
	solver.Solve(solver_type);

	solver.SetMaxIterations(max_iteration/2.0);
	rigid_rigid.solve_sliding = true;
	rigid_rigid.solve_spinning = true;
	rigid_rigid.ComputeRHS();
	solver.Solve(solver_type);
	solver.ComputeImpulses();

	tot_iterations = solver.GetIteration();
	residual = solver.GetResidual();
	data_container->host_data.old_gamma_data = data_container->host_data.gamma_data;
	//rhs = data_container->host_data.rhs_data;
	//lambda = data_container->host_data.gam_data;

	for (int i = 0; i < solver.iter_hist.size(); i++) {
		AtIterationEnd(solver.maxd_hist[i], solver.maxdeltalambda_hist[i], solver.iter_hist[i]);
	}
	if (warm_start) {
		//RunWarmStartPostProcess();
	}
	//determine binning

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

void ChLcpSolverGPU::RunWarmStartPostProcess() {
	if (data_container->number_of_rigid_rigid == 0) {
		return;
	}

	num_bins_per_axis = I3(20, 40, 20);
	int l = num_bins_per_axis.x;
	int h = num_bins_per_axis.y;
	int w = num_bins_per_axis.z;
	uint N = data_container->number_of_rigid_rigid;

	thrust::host_vector<real3> points(N);
	thrust::host_vector<uint> counter((l + 1) * (w + 1) * (h + 1), 0);

	data_container->host_data.bin_number.resize((l + 1) * (w + 1) * (h + 1));
	thrust::fill(data_container->host_data.bin_number.begin(), data_container->host_data.bin_number.end(), 0);

	points = (data_container->host_data.cpta_rigid_rigid + data_container->host_data.cptb_rigid_rigid);
	points = .5 * points;

	real3 max_point = R3(-FLT_MAX);
	real3 min_point = R3(FLT_MAX);
	real3 origin;

	for (int i = 0; i < N; i++) {
		max_point.x = max(points[i].x, max_point.x);
		max_point.y = max(points[i].y, max_point.y);
		max_point.z = max(points[i].z, max_point.z);

		min_point.x = min(points[i].x, min_point.x);
		min_point.y = min(points[i].y, min_point.y);
		min_point.z = min(points[i].z, min_point.z);
	}

	origin = min_point;
	real3 bin_size_vec = (fabs(max_point - origin));
	bin_size_vec.x = bin_size_vec.x / real(l);
	bin_size_vec.y = bin_size_vec.y / real(h);
	bin_size_vec.z = bin_size_vec.z / real(w);

	for (int i = 0; i < N; i++) {
		points[i] = points[i] - origin;

		int3 temp;
		temp.x = floor(points[i].x / bin_size_vec.x);
		temp.y = floor(points[i].y / bin_size_vec.y);
		temp.z = floor(points[i].z / bin_size_vec.z);
		data_container->host_data.bin_number[temp.x + temp.y * w + temp.z * w * h] += data_container->host_data.gamma_data[i];
		counter[temp.x + temp.y * w + temp.z * w * h]++;
	}
	for (int i = 0; i < counter.size(); i++) {
		if (counter[i] > 0) {
			data_container->host_data.bin_number[i] = data_container->host_data.bin_number[i] / counter[i];
		}
	}
}
void ChLcpSolverGPU::RunWarmStartPreprocess() {
	if (data_container->number_of_rigid_rigid == 0) {
		return;
	}

	if (data_container->host_data.old_pair_rigid_rigid.size() != 0) {

//		cout << "SORTEDN: " << thrust::is_sorted(data_container->host_data.pair_rigid_rigid.begin(), data_container->host_data.pair_rigid_rigid.end())<<endl;
//		cout << "SORTEDO: " << thrust::is_sorted(data_container->host_data.old_pair_rigid_rigid.begin(), data_container->host_data.old_pair_rigid_rigid.end())<<endl;
//		return;

		thrust::host_vector<long long> res1(data_container->host_data.pair_rigid_rigid.size());

		uint numP = thrust::set_intersection(
				thrust::omp::par,
				data_container->host_data.pair_rigid_rigid.begin(),
				data_container->host_data.pair_rigid_rigid.end(),
				data_container->host_data.old_pair_rigid_rigid.begin(),
				data_container->host_data.old_pair_rigid_rigid.end(),
				res1.begin()) - res1.begin();     //list of persistent contacts

		if (numP > 0) {
			res1.resize(numP);
			thrust::host_vector<uint> temporaryA(numP);
			thrust::host_vector<uint> temporaryB(numP);

			thrust::lower_bound(
					thrust::omp::par,
					data_container->host_data.old_pair_rigid_rigid.begin(),
					data_container->host_data.old_pair_rigid_rigid.end(),
					res1.begin(),
					res1.end(),
					temporaryA.begin());     //return index of common new contact
			thrust::lower_bound(thrust::omp::par, data_container->host_data.pair_rigid_rigid.begin(), data_container->host_data.pair_rigid_rigid.end(), res1.begin(), res1.end(), temporaryB.begin());     //return index of common new contact

			uint number_of_rigid_rigid = data_container->number_of_rigid_rigid;
			uint old_number_of_rigid_rigid = data_container->old_number_of_rigid_rigid;
#pragma omp parallel for
			for (int i = 0; i < numP; i++) {

				M33 contact_plane_old;

				{
					real3 U = data_container->host_data.old_norm_rigid_rigid[temporaryA[i]];

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

					contact_plane_old.U = U;
					contact_plane_old.V = V;
					contact_plane_old.W = W;
				}

				real3 old_gamma = R3(0), old_gamma_spinning = R3(0);
				old_gamma.x = data_container->host_data.old_gamma_data[temporaryA[i] + old_number_of_rigid_rigid * 0];
				old_gamma.y = data_container->host_data.old_gamma_data[temporaryA[i] + old_number_of_rigid_rigid * 1];
				old_gamma.z = data_container->host_data.old_gamma_data[temporaryA[i] + old_number_of_rigid_rigid * 2];

				old_gamma_spinning.x = data_container->host_data.old_gamma_data[temporaryA[i] + old_number_of_rigid_rigid * 3];
				old_gamma_spinning.y = data_container->host_data.old_gamma_data[temporaryA[i] + old_number_of_rigid_rigid * 4];
				old_gamma_spinning.z = data_container->host_data.old_gamma_data[temporaryA[i] + old_number_of_rigid_rigid * 5];

				real3 global_gamma = MatMult(contact_plane_old, old_gamma);
				real3 global_gamma_spin = MatMult(contact_plane_old, old_gamma_spinning);

				M33 contact_plane_new;

				{
					real3 U = data_container->host_data.norm_rigid_rigid[temporaryB[i]];

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

					contact_plane_new.U = U;
					contact_plane_new.V = V;
					contact_plane_new.W = W;
				}
				real3 new_gamma = MatTMult(contact_plane_new, global_gamma);
				real3 new_gamma_spin = MatTMult(contact_plane_new, global_gamma_spin);

				data_container->host_data.gamma_data[temporaryB[i] + number_of_rigid_rigid * 0] = new_gamma.x * .4;
				data_container->host_data.gamma_data[temporaryB[i] + number_of_rigid_rigid * 1] = new_gamma.y * .4;
				data_container->host_data.gamma_data[temporaryB[i] + number_of_rigid_rigid * 2] = new_gamma.z * .4;

				data_container->host_data.gamma_data[temporaryB[i] + number_of_rigid_rigid * 3] = new_gamma_spin.x * .4;
				data_container->host_data.gamma_data[temporaryB[i] + number_of_rigid_rigid * 4] = new_gamma_spin.y * .4;
				data_container->host_data.gamma_data[temporaryB[i] + number_of_rigid_rigid * 5] = new_gamma_spin.z * .4;

			}
		}
	}

//
//	num_bins_per_axis = I3(20,40,20);
//	int l = num_bins_per_axis.x;
//	int h = num_bins_per_axis.y;
//	int w = num_bins_per_axis.z;
//	uint N = data_container->number_of_rigid_rigid;
//
//	thrust::host_vector<real3> points(N);
//
//	points = (data_container->host_data.cpta_rigid_rigid + data_container->host_data.cptb_rigid_rigid);
//	points = .5 * points;
//
//	real3 max_point = R3(-FLT_MAX);
//	real3 min_point = R3(FLT_MAX);
//
//	for (int i = 0; i < N; i++) {
//		max_point.x = max(points[i].x, max_point.x);
//		max_point.y = max(points[i].y, max_point.y);
//		max_point.z = max(points[i].z, max_point.z);
//
//		min_point.x = min(points[i].x, min_point.x);
//		min_point.y = min(points[i].y, min_point.y);
//		min_point.z = min(points[i].z, min_point.z);
//	}
//
//	origin = min_point;
//	bin_size_vec = (fabs(max_point - origin));
//	bin_size_vec.x = bin_size_vec.x / real(l);
//	bin_size_vec.y = bin_size_vec.y / real(h);
//	bin_size_vec.z = bin_size_vec.z / real(w);
//
//	for (int i = 0; i < N; i++) {
//		points[i] = points[i] - origin;
//
//		int3 temp;
//		temp.x = floor(points[i].x / bin_size_vec.x);
//		temp.y = floor(points[i].y / bin_size_vec.y);
//		temp.z = floor(points[i].z / bin_size_vec.z);
//		data_container->host_data.gamma_data[i] = data_container->host_data.bin_number[temp.x + temp.y * w + temp.z * w * h];
//
//	}

}

