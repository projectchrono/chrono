#include "ChSolverParallel.h"

using namespace chrono;

void ChSolverParallel::Project(real* gamma) {

	rigid_rigid->Project(gamma);

}

void ChSolverParallel::Project_NoPar(real* gamma) {
	timer_project.start();

	rigid_rigid->Project_NoPar(gamma);

	timer_project.stop();
	time_project += timer_project();
}
//=================================================================================================================================

void ChSolverParallel::shurA(real* x) {

#pragma omp for
	for (int i = 0; i < number_of_rigid; i++) {
		data_container->host_data.QXYZ_data[i] = R3(0);
		data_container->host_data.QUVW_data[i] = R3(0);
	}

	rigid_rigid->ShurA(x);

	bilateral->ShurA(x);

}
//=================================================================================================================================

void ChSolverParallel::shurB(real*x, real*out) {

	rigid_rigid->ShurB(x, out);
	bilateral->ShurB(x, out);

}
void ChSolverParallel::ShurProduct(custom_vector<real> &x, custom_vector<real> & output) {

		//Thrust_Fill(output, 0);
#pragma omp master
		{
		data_container->system_timer.start("shurA");
		}
		shurA(x.data());
#pragma omp master
		{
		data_container->system_timer.stop("shurA");
		}

		//timer_shurcompliment.start();
#pragma omp master
		{
		data_container->system_timer.start("shurB");
		}
		shurB(x.data(),output.data());
#pragma omp master
		{
		data_container->system_timer.stop("shurB");
		}
		//timer_shurcompliment.stop();
		//time_shurcompliment +=timer_shurcompliment();

}
//=================================================================================================================================
void ChSolverParallel::ShurBilaterals(custom_vector<real> &x_t, custom_vector<real> & AX) {
	bilateral->ShurBilaterals(x_t,AX);
}
//=================================================================================================================================
//=================================================================================================================================
void ChSolverParallel::Setup() {
	time_shurcompliment = time_project = time_integrate = 0;
	///////////////////////////////////////
	//maxd_hist.clear();
	//maxdeltalambda_hist.clear();
	//iter_hist.clear();
	///////////////////////////////////////
	Initialize();
	//data_container->host_data.gamma_data.resize((number_of_constraints));
	data_container->host_data.QXYZ_data.resize(number_of_rigid);
	data_container->host_data.QUVW_data.resize(number_of_rigid);

	///////////////////////////////////////
#ifdef SIM_ENABLE_GPU_MODE
	Thrust_Fill(data_container->device_data.device_QXYZ_data, R3(0));
	Thrust_Fill(data_container->device_data.device_QUVW_data, R3(0));
	Thrust_Fill(data_container->device_data.device_gam_data, 0);
	Thrust_Fill(correction, 0);

#else

//	memset(data_container->host_data.QXYZ_data.data(), 0, data_container->host_data.QXYZ_data.size() * sizeof(real3));
//	memset(data_container->host_data.QUVW_data.data(), 0, data_container->host_data.QUVW_data.size() * sizeof(real3));
//	memset(data_container->host_data.gam_data.data(), 0, data_container->host_data.gam_data.size() * sizeof(real));

#pragma omp parallel for
	for (int i = 0; i < number_of_rigid; i++) {
		data_container->host_data.QXYZ_data[i] = R3(0);
		data_container->host_data.QUVW_data[i] = R3(0);
	}
#pragma omp parallel for
	for (int i = 0; i < number_of_constraints; i++) {
		//data_container->host_data.gamma_data[i] = 0;
	}
#endif
	//Thrust_Fill(data_container->host_data.gamma_bilateral, 0);
	///////////////////////////////////////
	//data_container->host_data.gamma_bilateral = data_container->host_data.gamma_bilateral*0;
	//thrust::copy_n(data_container->host_data.gamma_bilateral.begin(), data_container->number_of_bilaterals, data_container->host_data.gamma_data.begin() + data_container->number_of_rigid_rigid * 3);

}
void ChSolverParallel::UpdatePosition(custom_vector<real> &x) {
	if (rigid_rigid->solve_sliding == true|| rigid_rigid->solve_spinning ==true) {
		return;
	}
	shurA(x.data());

	data_container->host_data.vel_new_data = data_container->host_data.vel_data+data_container->host_data.QXYZ_data;
	data_container->host_data.omg_new_data + data_container->host_data.omg_data+data_container->host_data.QUVW_data;

#pragma omp parallel for
	for (int i = 0; i < data_container->number_of_rigid; i++) {

		data_container->host_data.pos_new_data[i] = data_container->host_data.pos_data[i] + data_container->host_data.vel_new_data[i] * step_size;
		//real3 dp = data_container->host_data.pos_new_data[i]-data_container->host_data.pos_data[i];
		//cout<<dp<<endl;
		real4 moldrot = data_container->host_data.rot_data[i];
		real3 newwel = data_container->host_data.omg_new_data[i];

		M33 A = AMat(moldrot);
		real3 newwel_abs = MatMult(A, newwel);
		real mangle = length(newwel_abs) * step_size;
		newwel_abs = normalize(newwel_abs);
		real4 mdeltarot = Q_from_AngAxis(mangle, newwel_abs);
		real4 mnewrot = mdeltarot % moldrot;
		data_container->host_data.rot_new_data[i] = mnewrot;
	}
}

void ChSolverParallel::UpdateContacts() {
	if (rigid_rigid->solve_sliding == true || rigid_rigid->solve_spinning == true) {
		return;
	}

	//// TODO:  This ASSUMES that we are using an MPR narrowphase!!
	////        Instead of constructing a narrowphaseMPR object here,
	////        modify so that we can use the CHCNarrowphase object 
	////        from the CollisionSystemParallel.
	collision::ChCNarrowphaseMPR narrowphase;
	narrowphase.SetCollisionEnvelope(data_container->collision_envelope);
	narrowphase.Update(data_container);

	rigid_rigid->UpdateJacobians();
	rigid_rigid->UpdateRHS();

}
void ChSolverParallel::ComputeImpulses() {
#pragma omp parallel
	{
		shurA(data_container->host_data.gamma_data.data());
	}
	data_container->host_data.vel_data += data_container->host_data.QXYZ_data;
	data_container->host_data.omg_data += data_container->host_data.QUVW_data;

}
void ChSolverParallel::Initial(real step, ChParallelDataManager *data_container_) {
	data_container = data_container_;
	step_size = step;
	Setup();

}
void ChSolverParallel::Solve(GPUSOLVERTYPE solver_type) {
	timer_solver.start();

	if (number_of_constraints > 0) {
		//total_iteration += SolveSD(data_container->host_data.gamma_data, data_container->host_data.rhs_data, 10);
		if (solver_type == STEEPEST_DESCENT) {
			total_iteration += SolveSD(data_container->host_data.gamma_data, data_container->host_data.rhs_data, max_iteration);
		} else if (solver_type == GRADIENT_DESCENT) {
			total_iteration += SolveGD(data_container->host_data.gamma_data, data_container->host_data.rhs_data, max_iteration);
		} else if (solver_type == CONJUGATE_GRADIENT) {
			total_iteration += SolveCG(data_container->host_data.gamma_data, data_container->host_data.rhs_data, max_iteration);
		} else if (solver_type == CONJUGATE_GRADIENT_SQUARED) {
			total_iteration += SolveCGS(data_container->host_data.gamma_data, data_container->host_data.rhs_data, max_iteration);
		} else if (solver_type == BICONJUGATE_GRADIENT) {
			total_iteration += SolveBiCG(data_container->host_data.gamma_data, data_container->host_data.rhs_data, max_iteration);
		} else if (solver_type == BICONJUGATE_GRADIENT_STAB) {
			total_iteration += SolveBiCGStab(data_container->host_data.gamma_data, data_container->host_data.rhs_data, max_iteration);
		} else if (solver_type == MINIMUM_RESIDUAL) {
			total_iteration += SolveMinRes(data_container->host_data.gamma_data, data_container->host_data.rhs_data, max_iteration);
		}
		//else if(solver_type==QUASAI_MINIMUM_RESIDUAL){SolveQMR(data_container->gpu_data.device_gam_data, rhs, max_iteration);}
		else if (solver_type == ACCELERATED_PROJECTED_GRADIENT_DESCENT || solver_type == APGDRS) {

			InitAPGD(data_container->host_data.gamma_data);

			if (do_stab) {
				custom_vector<real> rhs_bilateral(data_container->number_of_bilaterals);
				thrust::copy_n(data_container->host_data.rhs_data.begin() + data_container->number_of_rigid_rigid * 6, data_container->number_of_bilaterals, rhs_bilateral.begin());

				for (int i = 0; i < max_iteration; i += 8) {
					total_iteration += SolveAPGD(data_container->host_data.gamma_data, data_container->host_data.rhs_data, 4);
					thrust::copy_n(
							data_container->host_data.gamma_data.begin() + data_container->number_of_rigid_rigid * 6,
							data_container->number_of_bilaterals,
							data_container->host_data.gamma_bilateral.begin());

					SolveStab(data_container->host_data.gamma_bilateral, rhs_bilateral, 10);

					thrust::copy_n(
							data_container->host_data.gamma_bilateral.begin(),
							data_container->number_of_bilaterals,
							data_container->host_data.gamma_data.begin() + data_container->number_of_rigid_rigid * 3);

					total_iteration += SolveAPGD(data_container->host_data.gamma_data, data_container->host_data.rhs_data, 4);
				}
			} else {
				if(solver_type == ACCELERATED_PROJECTED_GRADIENT_DESCENT) {
					total_iteration += SolveAPGD(data_container->host_data.gamma_data, data_container->host_data.rhs_data, max_iteration);
				} else {
					total_iteration += SolveAPGDRS(data_container->host_data.gamma_data, data_container->host_data.rhs_data, max_iteration, number_of_constraints);
				}

			}
		} else if (solver_type == BLOCK_JACOBI) {
			//SolveJacobi();
		}
//		thrust::copy_n(
//				data_container->host_data.gamma_data.begin() + data_container->number_of_rigid_rigid * 3,
//				data_container->number_of_bilaterals,
//				data_container->host_data.gamma_bilateral.begin());

		//cout<<time_shurcompliment<<endl;

		current_iteration = total_iteration;

		timer_solver.stop();
		time_solver = timer_solver();
	}
}

void ChSolverParallel::VelocityStabilization(ChParallelDataManager *data_container_) {

}

uint ChSolverParallel::SolveStab(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter) {
	custom_vector<real> r(x.size()), p, Ap(x.size());
	real rsold, alpha, rsnew = 0, normb = Norm(b);
	if (normb == 0.0) {
		normb = 1;
	}
	ShurBilaterals(x,r);
	p = r = b - r;
	rsold = Dot(r, r);
	normb = 1.0 / normb;
	if (sqrt(rsold) * normb <= tolerance) {
		return 0;
	}
	for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
		ShurBilaterals(p,Ap);
		alpha = rsold / Dot(p, Ap);
		rsnew = 0;
#pragma omp parallel for reduction(+:rsnew)
		for (int i = 0; i < x.size(); i++) {
			x[i] = x[i] + alpha * p[i];
			real _r = r[i] - alpha * Ap[i];
			r[i] = _r;
			rsnew += _r*_r;
		}
		residual = sqrt(rsnew) * normb;
		if (residual < tolerance) {
			break;
		}
		SEAXPY(rsnew / rsold, p, r, p);     //p = r + rsnew / rsold * p;
		rsold = rsnew;

	}
	total_iteration +=current_iteration;
	current_iteration = total_iteration;
	return current_iteration;
}

//
//__host__ __device__ void function_InvNDiag(uint index, int b1, int b2, int2* ids, real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB, real * inv_mass, real3 * inv_inertia, real & inv_n_diag) {
//
//  real Jx1 = JXYZA[index].x;
//  real Jy1 = JXYZA[index].y;
//  real Jz1 = JXYZA[index].z;
//  real Ju1 = JUVWA[index].x;
//  real Jv1 = JUVWA[index].y;
//  real Jw1 = JUVWA[index].z;
//  //
//  real Jx2 = JXYZB[index].x;
//  real Jy2 = JXYZB[index].y;
//  real Jz2 = JXYZB[index].z;
//  real Ju2 = JUVWB[index].x;
//  real Jv2 = JUVWB[index].y;
//  real Jw2 = JUVWB[index].z;
//  //
//  real m1 = inv_mass[b1];
//  real m2 = inv_mass[b2];
//  //
//  real3 i1 = inv_inertia[b1];
//  real3 i2 = inv_inertia[b2];
//
//  real part1 = dot(JXYZA[index], JXYZA[index]) * m1;
//  real part2 = dot(JUVWA[index] * JUVWA[index], i1);
//  real part3 = dot(JXYZB[index], JXYZB[index]) * m2;
//  real part4 = dot(JUVWB[index] * JUVWB[index], i2);
//  inv_n_diag = (part1 + part2 + part3 + part4);
//}
