#include "ChSolverGPU.cuh"
using namespace chrono;

__constant__ uint number_of_constraints_const;
__constant__ uint number_of_contacts_const;
__constant__ uint number_of_bilaterals_const;
__constant__ real step_size_const;
__constant__ real alpha_const;
__constant__ real compliance_const;
__constant__ real inv_hpa_const;
__constant__ real inv_hhpa_const;
__constant__ real contact_recovery_speed_const;

__device__ double atomicAdd(double* address, double val) {
	unsigned long long int* address_as_ull = (unsigned long long int*) address;
	unsigned long long int old = *address_as_ull, assumed;
	do {
		assumed = old;
		old = atomicCAS(address_as_ull, assumed, __double_as_longlong(val + __longlong_as_double(assumed)));
	} while (assumed != old);
	return __longlong_as_double(old);
}

void ChSolverGPU::Project(custom_vector<real> & gamma) {
	timer_project.start();

	rigid_rigid->Project(gamma);

	timer_project.stop();
	time_project += timer_project();
}
//=================================================================================================================================

void ChSolverGPU::shurA(custom_vector<real> &x) {
//#pragma omp parallel for
//		for (int i = 0; i < number_of_rigid; i++) {
//			data_container->host_data.QXYZ_data[i] = R3(0);
//			data_container->host_data.QUVW_data[i] = R3(0);
//		}
		rigid_rigid->ShurA(x);
		bilateral->ShurA(x);

	}
//=================================================================================================================================

void ChSolverGPU::shurB(custom_vector<real> &x, custom_vector<real> &out) {
	rigid_rigid->ShurB(x,out);
	bilateral->ShurB(x,out);
}
void ChSolverGPU::ShurProduct(custom_vector<real> &x, custom_vector<real> & output) {
	timer_shurcompliment.start();
	Thrust_Fill(output, 0);
	shurA(x);
	shurB(x,output);
	timer_shurcompliment.stop();
	time_shurcompliment +=timer_shurcompliment();
}
#define JXYZA_bilateral data_container->host_data.JXYZA_bilateral
#define JXYZB_bilateral data_container->host_data.JXYZB_bilateral
#define JUVWA_bilateral data_container->host_data.JUVWA_bilateral
#define JUVWB_bilateral data_container->host_data.JUVWB_bilateral
//=================================================================================================================================
void ChSolverGPU::ShurBilaterals(custom_vector<real> &x_t, custom_vector<real> & AX) {
	bilateral->ShurBilaterals(x_t,AX);
}
//=================================================================================================================================
//=================================================================================================================================
void ChSolverGPU::Setup() {
	time_shurcompliment = time_project = time_integrate = 0;
	///////////////////////////////////////
	maxd_hist.clear();
	maxdeltalambda_hist.clear();
	iter_hist.clear();
	///////////////////////////////////////
	Initialize();

	data_container->host_data.gam_data.resize((number_of_constraints));
	data_container->host_data.QXYZ_data.resize(number_of_rigid);
	data_container->host_data.QUVW_data.resize(number_of_rigid);

	///////////////////////////////////////
#ifdef SIM_ENABLE_GPU_MODE
	Thrust_Fill(data_container->device_data.device_QXYZ_data, R3(0));
	Thrust_Fill(data_container->device_data.device_QUVW_data, R3(0));
	Thrust_Fill(data_container->device_data.device_gam_data, 0);
	Thrust_Fill(correction, 0);

#else
#pragma omp parallel for
	for (int i = 0; i < number_of_rigid; i++) {
		data_container->host_data.QXYZ_data[i] = R3(0);
		data_container->host_data.QUVW_data[i] = R3(0);
	}
#pragma omp parallel for
	for (int i = 0; i < number_of_constraints; i++) {
		data_container->host_data.gam_data[i] = 0;
	}
#endif

	///////////////////////////////////////
	thrust::copy_n(
			data_container->host_data.gamma_bilateral.begin(),
			data_container->number_of_bilaterals,
			data_container->host_data.gam_data.begin() + data_container->number_of_rigid_rigid * 3);

}

void ChSolverGPU::ComputeImpulses() {
	shurA(data_container->host_data.gam_data);
	data_container->host_data.vel_data += data_container->host_data.QXYZ_data;
	data_container->host_data.omg_data += data_container->host_data.QUVW_data;
}

void ChSolverGPU::Solve(GPUSOLVERTYPE solver_type, real step, ChGPUDataManager *data_container_) {
	timer_solver.start();
	data_container = data_container_;
	step_size = step;

	Setup();
	total_iteration = 0;
	if (number_of_constraints > 0) {

		if (solver_type == STEEPEST_DESCENT) {
			total_iteration += SolveSD(data_container->host_data.gam_data, data_container->host_data.rhs_data, max_iteration);
		} else if (solver_type == GRADIENT_DESCENT) {
			total_iteration += SolveGD(data_container->host_data.gam_data, data_container->host_data.rhs_data, max_iteration);
		} else if (solver_type == CONJUGATE_GRADIENT) {
			total_iteration += SolveCG(data_container->host_data.gam_data, data_container->host_data.rhs_data, max_iteration);
		} else if (solver_type == CONJUGATE_GRADIENT_SQUARED) {
			total_iteration += SolveCGS(data_container->host_data.gam_data, data_container->host_data.rhs_data, max_iteration);
		} else if (solver_type == BICONJUGATE_GRADIENT) {
			total_iteration += SolveBiCG(data_container->host_data.gam_data, data_container->host_data.rhs_data, max_iteration);
		} else if (solver_type == BICONJUGATE_GRADIENT_STAB) {
			total_iteration += SolveBiCGStab(data_container->host_data.gam_data, data_container->host_data.rhs_data, max_iteration);
		} else if (solver_type == MINIMUM_RESIDUAL) {
			total_iteration += SolveMinRes(data_container->host_data.gam_data, data_container->host_data.rhs_data, max_iteration);
		}
		//else if(solver_type==QUASAI_MINIMUM_RESIDUAL){SolveQMR(data_container->gpu_data.device_gam_data, rhs, max_iteration);}
		else if (solver_type == ACCELERATED_PROJECTED_GRADIENT_DESCENT) {

			InitAPGD(data_container->host_data.gam_data);

			if (do_stab) {
				custom_vector<real> rhs_bilateral(data_container->number_of_bilaterals);
				thrust::copy_n(data_container->host_data.rhs_data.begin() + data_container->number_of_rigid_rigid * 3, data_container->number_of_bilaterals, rhs_bilateral.begin());

				for (int i = 0; i < max_iteration; i += 4) {
					total_iteration += SolveAPGD(data_container->host_data.gam_data, data_container->host_data.rhs_data, 4);
					thrust::copy_n(
							data_container->host_data.gam_data.begin() + data_container->number_of_rigid_rigid * 3,
							data_container->number_of_bilaterals,
							data_container->host_data.gamma_bilateral.begin());

					SolveStab(data_container->host_data.gamma_bilateral, rhs_bilateral, 50);

					thrust::copy_n(
							data_container->host_data.gamma_bilateral.begin(),
							data_container->number_of_bilaterals,
							data_container->host_data.gam_data.begin() + data_container->number_of_rigid_rigid * 3);

					total_iteration += SolveAPGD(data_container->host_data.gam_data, data_container->host_data.rhs_data, 4);
				}
			} else {
				total_iteration += SolveAPGD(data_container->host_data.gam_data, data_container->host_data.rhs_data, max_iteration);
			}
		} else if (solver_type == BLOCK_JACOBI) {
			SolveJacobi();
		}
		thrust::copy_n(
				data_container->host_data.gam_data.begin() + data_container->number_of_rigid_rigid * 3,
				data_container->number_of_bilaterals,
				data_container->host_data.gamma_bilateral.begin());

		current_iteration = total_iteration;
		ComputeImpulses();
		timer_solver.stop();
		time_solver = timer_solver();
	}
}

void ChSolverGPU::VelocityStabilization(ChGPUDataManager *data_container_) {

}

uint ChSolverGPU::SolveStab(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter) {
	uint N = b.size();
	custom_vector<real> v(N, 0), v_hat(x.size()), w(N, 0), w_old, xMR, v_old, Av(x.size()), w_oold;
	real beta, c = 1, eta, norm_rMR, norm_r0, c_old = 1, s_old = 0, s = 0, alpha, beta_old, c_oold, s_oold, r1_hat, r1, r2, r3;
	ShurBilaterals(x,v_hat);
	v_hat = b - v_hat;
	beta = Norm(v_hat);
	w_old = w;
	eta = beta;
	xMR = x;
	norm_rMR = beta;
	norm_r0 = beta;

	if (beta == 0 || norm_rMR / norm_r0 < tolerance) {return 0;}

	for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
		//// Lanczos
		v_old = v;
		v = 1.0 / beta * v_hat;
		ShurBilaterals(v,Av);
		alpha = Dot(v, Av);
		v_hat = Av - alpha * v - beta * v_old;
		beta_old = beta;
		beta = Norm(v_hat);
		//// QR factorization
		c_oold = c_old;
		c_old = c;
		s_oold = s_old;
		s_old = s;
		r1_hat = c_old * alpha - c_oold * s_old * beta_old;
		r1 = 1 / sqrt(r1_hat * r1_hat + beta * beta);
		r2 = s_old * alpha + c_oold * c_old * beta_old;
		r3 = s_oold * beta_old;
		//// Givens Rotation
		c = r1_hat * r1;
		s = beta * r1;
		//// update
		w_oold = w_old;
		w_old = w;
		w = r1 * (v - r3 * w_oold - r2 * w_old);
		x = x + c * eta * w;
		norm_rMR = norm_rMR * abs(s);
		eta = -s * eta;
		residual = norm_rMR / norm_r0;

		if (residual < tolerance) {break;}
	}

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
