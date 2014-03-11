#include "ChLcpSolverParallel.h"
#include "math/ChThrustLinearAlgebra.h"


using namespace chrono;


void ChLcpSolverParallelDVI::RunTimeStep(real step)
{
	step_size = step;
	data_container->step_size = step;

	number_of_constraints = data_container->number_of_rigid_rigid * 6 + data_container->number_of_bilaterals;
	number_of_bilaterals = 0;     ///data_container->number_of_bilaterals;
	number_of_objects = data_container->number_of_rigid;
	//cout << number_of_constraints << " " << number_of_contacts << " " << number_of_bilaterals << " " << number_of_objects << endl;

	Preprocess();

	data_container->host_data.rhs_data.resize(number_of_constraints);
	data_container->host_data.diag.resize(number_of_constraints);

	//bilateral.Diag();

	data_container->host_data.gamma_data.resize((number_of_constraints));

#pragma omp parallel for
	for (int i = 0; i < number_of_constraints; i++) {
		data_container->host_data.gamma_data[i] = 0;
	}
	if (warm_start) {
		RunWarmStartPreprocess();
	}
	data_container->system_timer.start("solve_setup");
	rigid_rigid.Setup(data_container);
	bilateral.Setup(data_container);

	solver.current_iteration = 0;
	solver.total_iteration = 0;
	solver.maxd_hist.clear();
	solver.maxdeltalambda_hist.clear();
	solver.iter_hist.clear();

	solver.SetMaxIterations(max_iteration);
	solver.SetTolerance(tolerance);

	solver.SetComplianceAlpha(alpha);
	solver.SetContactRecoverySpeed(contact_recovery_speed);
	solver.lcp_omega_bilateral = lcp_omega_bilateral;
	solver.rigid_rigid = &rigid_rigid;
	solver.bilateral = &bilateral;
	solver.lcp_omega_contact = lcp_omega_contact;
	solver.do_stab = do_stab;
	solver.collision_inside = collision_inside;
	solver.Initial(step, data_container);
	data_container->system_timer.stop("solve_setup");
	if (collision_inside) {
		data_container->host_data.vel_new_data = data_container->host_data.vel_data;
		data_container->host_data.omg_new_data = data_container->host_data.omg_data;

		data_container->host_data.pos_new_data = data_container->host_data.pos_data;
		data_container->host_data.rot_new_data = data_container->host_data.rot_data;
	}

	//solve initial
	//solver.SetComplianceParameters(.2, 1e-3, 1e-3);
	//rigid_rigid.solve_sliding = true;
	data_container->system_timer.start("jacobians");
	rigid_rigid.ComputeJacobians();

	bilateral.ComputeJacobians();
	data_container->system_timer.stop("jacobians");
	data_container->system_timer.start("rhs");
	bilateral.ComputeRHS();
	data_container->system_timer.stop("rhs");


	if (max_iter_bilateral > 0) {
		data_container->system_timer.start("stab");
		custom_vector<real> rhs_bilateral(data_container->number_of_bilaterals);
		thrust::copy_n(data_container->host_data.rhs_data.begin() + data_container->number_of_rigid_rigid * 6, data_container->number_of_bilaterals, rhs_bilateral.begin());
		//thrust::copy_n(data_container->host_data.gamma_data.begin() + data_container->number_of_rigid_rigid * 6, data_container->number_of_bilaterals, data_container->host_data.gamma_bilateral.begin());
		solver.SolveStab(data_container->host_data.gamma_bilateral, rhs_bilateral, max_iter_bilateral);
		data_container->system_timer.stop("stab");
	}
	thrust::copy_n(data_container->host_data.gamma_bilateral.begin(), data_container->number_of_bilaterals, data_container->host_data.gamma_data.begin() + data_container->number_of_rigid_rigid * 6);

	//cout<<"Solve normal"<<endl;
	//solve normal
	if (max_iter_normal > 0) {
		solver.SetMaxIterations(max_iter_normal);
		solver.SetComplianceAlpha(alpha);
		rigid_rigid.solve_sliding = false;
		rigid_rigid.solve_spinning = false;
		data_container->system_timer.start("rhs");
		rigid_rigid.ComputeRHS();
		data_container->system_timer.stop("rhs");
		solver.Solve(solver_type);
	}
	//cout<<"Solve sliding"<<endl;
	//solve full
	if (max_iter_sliding > 0) {
		solver.SetMaxIterations(max_iter_sliding);
		rigid_rigid.solve_sliding = true;
		rigid_rigid.solve_spinning = false;
		data_container->system_timer.start("rhs");
		rigid_rigid.ComputeRHS();
		data_container->system_timer.stop("rhs");
		solver.Solve(solver_type);
	}
	if (max_iter_spinning > 0) {
		//cout<<"Solve Full"<<endl;
		solver.SetMaxIterations(max_iter_spinning);
		rigid_rigid.solve_sliding = true;
		rigid_rigid.solve_spinning = true;
		data_container->system_timer.start("rhs");
		rigid_rigid.ComputeRHS();
		data_container->system_timer.stop("rhs");
		solver.Solve(solver_type);
	}

	thrust::copy_n(data_container->host_data.gamma_data.begin() + data_container->number_of_rigid_rigid * 6, data_container->number_of_bilaterals, data_container->host_data.gamma_bilateral.begin());
	for (int i = 0; i < data_container->number_of_bilaterals; i++) {
		data_container->host_data.gamma_bilateral[i] *= .5;
	}

//	if (max_iter_bilateral > 0) {
//		thrust::copy_n(data_container->host_data.rhs_data.begin() + data_container->number_of_rigid_rigid * 6, data_container->number_of_bilaterals, rhs_bilateral.begin());
//		thrust::copy_n(data_container->host_data.gamma_data.begin() + data_container->number_of_rigid_rigid * 6, data_container->number_of_bilaterals,
//				data_container->host_data.gamma_bilateral.begin());
//		solver.SolveStab(data_container->host_data.gamma_bilateral, rhs_bilateral, max_iter_bilateral);
//		thrust::copy_n(data_container->host_data.gamma_bilateral.begin(), data_container->number_of_bilaterals,
//				data_container->host_data.gamma_data.begin() + data_container->number_of_rigid_rigid * 6);
//	}
	solver.ComputeImpulses();

	tot_iterations = solver.GetIteration();
	residual = solver.GetResidual();
	//data_container->host_data.old_gamma_data = data_container->host_data.gamma_data;
	//rhs = data_container->host_data.rhs_data;
	//lambda = data_container->host_data.gam_data;

	for (int i = 0; i < solver.iter_hist.size(); i++) {
		AtIterationEnd(solver.maxd_hist[i], solver.maxdeltalambda_hist[i], solver.iter_hist[i]);
	}
	//if (warm_start) {
	//RunWarmStartPostProcess();
	//}

#ifdef PRINT_DEBUG_GPU
	cout << "Solve Done: "<<residual << endl;
#endif
	//ChIntegratorGPU integrator;
	//integrator.IntegrateSemiImplicit(step, data_container->gpu_data);
}


void ChLcpSolverParallelDVI::RunWarmStartPostProcess()
{
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


void ChLcpSolverParallelDVI::RunWarmStartPreprocess()
{
	if (data_container->number_of_rigid_rigid == 0) {
		return;
	}

	if (data_container->host_data.old_pair_rigid_rigid.size() != 0) {

//		cout << "SORTEDN: " << thrust::is_sorted(data_container->host_data.pair_rigid_rigid.begin(), data_container->host_data.pair_rigid_rigid.end())<<endl;
//		cout << "SORTEDO: " << thrust::is_sorted(data_container->host_data.old_pair_rigid_rigid.begin(), data_container->host_data.old_pair_rigid_rigid.end())<<endl;
//		return;

		thrust::host_vector<long long> res1(data_container->host_data.pair_rigid_rigid.size());

		uint numP = thrust::set_intersection(thrust::omp::par, data_container->host_data.pair_rigid_rigid.begin(), data_container->host_data.pair_rigid_rigid.end(),
				data_container->host_data.old_pair_rigid_rigid.begin(), data_container->host_data.old_pair_rigid_rigid.end(), res1.begin()) - res1.begin();     //list of persistent contacts

		if (numP > 0) {
			res1.resize(numP);
			thrust::host_vector<uint> temporaryA(numP);
			thrust::host_vector<uint> temporaryB(numP);

			thrust::lower_bound(thrust::omp::par, data_container->host_data.old_pair_rigid_rigid.begin(), data_container->host_data.old_pair_rigid_rigid.end(), res1.begin(), res1.end(),
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

