#include "ChSolverParallel.h"
using namespace chrono;
uint ChSolverParallel::SolveAPGDRS(custom_vector<real> &x, custom_vector<real> &b, const uint max_iter,const int SIZE) {
	real gdiff = 1e-6;

	real lastgoodres=10e30;
	real theta_k=init_theta_k;
	real theta_k1=theta_k;
	real beta_k1=0.0;
	real L_k;

	Project(x.data());
	ShurProduct(x,mg);
	real mb_tmp_norm = 0;

#pragma omp parallel for reduction(+:mb_tmp_norm)
		for (int i = 0; i < SIZE; i++) {
			real _mb_tmp_ = x.data()[i] - 1.0f;
			mb_tmp_norm += _mb_tmp_ * _mb_tmp_;
			mb_tmp.data()[i] = _mb_tmp_;
		}
		mb_tmp_norm = sqrt(mb_tmp_norm);

		ShurProduct(mb_tmp,mg_tmp);

		if (mb_tmp_norm == 0) {
			L_k = 1;
		} else {
			L_k =Norm(mg_tmp) / mb_tmp_norm;
		}

		real t_k = 1.0 / L_k;
		my = x;
		mx = x;

		real obj1=0.0, obj2=0.0;

		for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
			ShurProduct(my,mg_tmp1);
#pragma omp parallel
		{

#pragma omp for
		for (int i = 0; i < SIZE; i++) {
			real _mg_ = mg_tmp1.data()[i] - b.data()[i];
			mg.data()[i] = _mg_;
			mx.data()[i] = -t_k * _mg_ + my.data()[i];
		}

		Project_NoPar(mx.data());
	}

	ShurProduct(mx,mg_tmp);
	obj1=obj2=0.0;

	real dot_mg_ms = 0, norm_ms = 0;

#pragma omp parallel for reduction(+:obj1,obj2,dot_mg_ms,norm_ms)
		for(int i=0; i<SIZE; i++) {
			real _mg_tmp_ = mg_tmp.data()[i];
			real _b_ = b.data()[i];
			real _ms_ = .5*_mg_tmp_-_b_;
			real _mx_ = mx.data()[i];
			real _my_ = my.data()[i];
			obj1+=_mx_*_ms_;

			_ms_ = .5*mg_tmp1.data()[i]-_b_;
			obj2+=_my_*_ms_;
			_ms_ = _mx_-_my_;
			dot_mg_ms+=mg.data()[i]*_ms_;
			norm_ms+=_ms_*_ms_;
		}
		norm_ms = sqrt(norm_ms);
		while (obj1 > obj2 + dot_mg_ms + 0.5 * L_k * powf(norm_ms, 2.0)) {
			L_k = step_grow * L_k;
			t_k = 1.0 / L_k;

#pragma omp parallel
		{
#pragma omp  for
		for(int i=0; i<SIZE; i++) {
			mx.data()[i] = -t_k*mg.data()[i]+ my.data()[i];
		}
		Project_NoPar(mx.data());
	}

	ShurProduct(mx,mg_tmp);
	obj1 = dot_mg_ms = norm_ms =0;
#pragma omp parallel for reduction(+:obj1,dot_mg_ms,norm_ms)
		for(int i=0; i<SIZE; i++) {
			real _mg_tmp_ = mg_tmp.data()[i];
			real _b_ = b.data()[i];
			real _ms_ = .5*_mg_tmp_-_b_;
			real _mx_ = mx.data()[i];
			obj1+=_mx_*_ms_;
			_ms_ = _mx_-my.data()[i];
			dot_mg_ms+=mg.data()[i]*_ms_;
			norm_ms+=_ms_*_ms_;
		}
		norm_ms = sqrt(norm_ms);
	}
	theta_k1 = (-pow(theta_k, 2) + theta_k * sqrt(pow(theta_k, 2) + 4)) / 2.0;
	beta_k1 = theta_k * (1.0 - theta_k) / (pow(theta_k, 2) + theta_k1);
	real temp_sum=0;
#pragma omp parallel for reduction(+:temp_sum)
		for(int i=0; i<SIZE; i++) {
			real _mx_ = mx.data()[i];
			real _ms_ = _mx_-x.data()[i];
			my[i] = beta_k1*_ms_+_mx_;
			temp_sum+=mg.data()[i]*_ms_;
		}

		if (temp_sum > 0) {
			my = mx;
			theta_k1 = 1.0;
		}
		L_k = step_shrink * L_k;
		t_k = 1.0 / L_k;
		theta_k = theta_k1;
		if(current_iteration%5==0) {
			real g_proj_norm = Res4(SIZE, mg_tmp.data(), b.data(), x.data(), mb_tmp.data());
			if(g_proj_norm < lastgoodres) {
				lastgoodres = g_proj_norm;
				ml_candidate = mx;
			}
			residual=lastgoodres;
			real maxdeltalambda = 0;     //CompRes(b,number_of_rigid_rigid);     //NormInf(ms);

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

