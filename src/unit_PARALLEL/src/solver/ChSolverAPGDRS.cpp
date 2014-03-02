#include "ChSolverParallel.h"
using namespace chrono;

real Func1(const int &SIZE, real* __restrict__ x, real* __restrict__ mb_tmp) {
	real mb_tmp_norm = 0;
	real _mb_tmp_;

	real *x_ = (real*) __builtin_assume_aligned(x, 16);
	real *mb_tmp_ = (real*) __builtin_assume_aligned(mb_tmp, 16);
//#pragma omp parallel for reduction(+:mb_tmp_norm)
	for (int i = 0; i < SIZE; i++) {
		_mb_tmp_ = x_[i] - 1.0f;
		mb_tmp_norm += _mb_tmp_ * _mb_tmp_;
		mb_tmp_[i] = _mb_tmp_;
	}
	mb_tmp_norm = sqrt(mb_tmp_norm);
	return mb_tmp_norm;
}
void Func2(const size_t &SIZE, const real t_k, real* __restrict__ x, real* __restrict__ mg_tmp1, real* __restrict__ b, real* __restrict__ mg, real* __restrict__ mx,
		real* __restrict__ my) {
//#pragma omp parallel for
	for (int i = 0; i < SIZE; i++) {
		real _mg_ = mg_tmp1[i] - b[i];
		mg[i] = _mg_;
		mx[i] = -t_k * _mg_ + my[i];
	}
}

void Func3(const size_t &SIZE, real* __restrict__ mg_tmp, real* __restrict__ mg_tmp1, real* __restrict__ b, real* __restrict__ mg, real* __restrict__ mx, real* __restrict__ my,
		real & obj1, real & obj2, real & dot_mg_ms, real & norm_ms) {
	obj1 = 0.0;
	obj2 = 0.0;

	dot_mg_ms = 0;
	norm_ms = 0;
	real _mg_tmp_, _b_, _ms_, _mx_, _my_;
//#pragma omp parallel for reduction(+:obj1,obj2,dot_mg_ms,norm_ms)
	for (int i = 0; i < SIZE; i++) {
		_mg_tmp_ = mg_tmp[i];
		_b_ = b[i];
		_ms_ = .5 * _mg_tmp_ - _b_;
		_mx_ = mx[i];
		_my_ = my[i];
		obj1 += _mx_ * _ms_;

		_ms_ = .5 * mg_tmp1[i] - _b_;
		obj2 += _my_ * _ms_;
		_ms_ = _mx_ - _my_;
		dot_mg_ms += mg[i] * _ms_;
		norm_ms += _ms_ * _ms_;
	}
	norm_ms = sqrt(norm_ms);
}
uint ChSolverParallel::SolveAPGDRS(custom_vector<real> &x, custom_vector<real> &b, const uint max_iter,const int SIZE) {
	real gdiff = 1e-6;

	real lastgoodres=10e30;
	real theta_k=init_theta_k;
	real theta_k1=theta_k;
	real beta_k1=0.0;
	real L_k;

	Project(x.data());
	ShurProduct(x,mg);
	real mb_tmp_norm = Func1(SIZE,x.data(),mb_tmp.data());

	ShurProduct(mb_tmp,mg_tmp);

	if (mb_tmp_norm == 0) {
		L_k = 1;
	} else {
		L_k =Norm(mg_tmp) / mb_tmp_norm;
	}

	real t_k = 1.0 / L_k;
	my = x;
	mx = x;

	real obj1=0.0;
	real obj2=0.0;

	for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
		ShurProduct(my,mg_tmp1);

		Func2(SIZE, t_k, x.data(), mg_tmp1.data(), b.data(), mg.data(), mx.data(),
		my.data());

		Project(mx.data());
		ShurProduct(mx,mg_tmp);
		obj1=0.0;
		obj2=0.0;

		real dot_mg_ms = 0;
		real norm_ms = 0;

		Func3(SIZE, mg_tmp.data(), mg_tmp1.data(), b.data(), mg.data(), mx.data(), my.data(),
		obj1, obj2, dot_mg_ms, norm_ms);

		while (obj1 > obj2 + dot_mg_ms + 0.5 * L_k * powf(norm_ms, 2.0)) {
			L_k = step_grow * L_k;
			t_k = 1.0 / L_k;
			SEAXPY(-t_k, mg, my, mx);     // mx = my + mg*(t_k);
		Project(mx.data());

		ShurProduct(mx,mg_tmp);
		obj1 = dot_mg_ms = norm_ms =0;
#pragma omp parallel for reduction(+:obj1,dot_mg_ms,norm_ms)
		for(int i=0; i<SIZE; i++) {
			real _mg_tmp_ = mg_tmp[i];
			real _b_ = b[i];
			real _ms_ = .5*_mg_tmp_-_b_;
			real _mx_ = mx[i];
			obj1+=_mx_*_ms_;
			_ms_ = _mx_-my[i];
			dot_mg_ms+=mg[i]*_ms_;
			norm_ms+=_ms_*_ms_;
		}
		norm_ms = sqrt(norm_ms);
	}
	theta_k1 = (-pow(theta_k, 2) + theta_k * sqrt(pow(theta_k, 2) + 4)) / 2.0;
	beta_k1 = theta_k * (1.0 - theta_k) / (pow(theta_k, 2) + theta_k1);
	real temp_sum=0;
#pragma omp parallel for reduction(+:temp_sum)
		for(int i=0; i<SIZE; i++) {
			real _mx_ = mx[i];
			real _ms_ = _mx_-x[i];
			my[i] = beta_k1*_ms_+_mx_;
			temp_sum+=mg[i]*_ms_;
		}

		if (temp_sum > 0) {
			my = mx;
			theta_k1 = 1.0;
		}
		L_k = step_shrink * L_k;
		t_k = 1.0 / L_k;
		theta_k = theta_k1;
		if(current_iteration%5==0) {
			real g_proj_norm = Res4(mg_tmp, b, mx, mb_tmp);
			if(g_proj_norm < lastgoodres) {
				lastgoodres = g_proj_norm;
				ml_candidate = mx;
			}
			residual=lastgoodres;
			real maxdeltalambda = CompRes(b,number_of_rigid_rigid);     //NormInf(ms);

			AtIterationEnd(residual, maxdeltalambda, current_iteration);
			if(collision_inside) {
				UpdatePosition(ml_candidate);
				UpdateContacts();
			}
		}
		if (residual < tolerance) {
			break;
		}
	}
	x=ml_candidate;
	return current_iteration;
}

