#include "ChSolverParallel.h"
using namespace chrono;
uint ChSolverParallel::SolveAPGDRS(custom_vector<real> &x, custom_vector<real> &b, const uint max_iter,const int SIZE) {
	real lastgoodres=10e30;
	real theta_k=init_theta_k;
	real theta_k1=theta_k;
	real beta_k1=0.0;
	real L_k, t_k;
	real mb_tmp_norm = 0, mg_tmp_norm = 0;
	real obj1=0.0, obj2=0.0;
	real dot_mg_ms = 0, norm_ms = 0;
	mg.resize(SIZE);
	mg_tmp.resize(SIZE);
	mg_tmp1.resize(SIZE);
	mg_tmp2.resize(SIZE);
	ml.resize(SIZE);

//#pragma omp  parallel for
//		for(int i=0; i<SIZE; i++) {
//			ml[i] = 0;
//		}
		ml = x;
		Project(ml.data());
		ml_candidate = ml;
		ShurProduct(ml,mg);//mg is never used, only re-written

#pragma omp parallel for reduction(+:mb_tmp_norm)
		for (int i = 0; i < SIZE; i++) {
			real _mb_tmp_ = -1.0f + ml[i];
			mb_tmp_norm += _mb_tmp_ * _mb_tmp_;
			mb_tmp[i] = _mb_tmp_;
			mg[i] = mg[i]-b[i];
		}

		ShurProduct(mb_tmp,mg_tmp);
		mb_tmp_norm = sqrt(mb_tmp_norm);

		if (mb_tmp_norm == 0) {
			L_k = 1;
		} else {
			L_k =Norm(mg_tmp) / mb_tmp_norm;
		}

		t_k = 1.0 / L_k;
		my = ml;
		mx = ml;

		for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
			obj1=obj2=0.0;
			dot_mg_ms = 0;
			norm_ms = 0;

			ShurProduct(my,mg_tmp1);

#pragma omp parallel for
		for (int i = 0; i < SIZE; i++) {
			real _mg_ = mg_tmp1[i] - b[i];
			mg[i] = _mg_;
			mx[i] = -t_k * _mg_ + my[i];
		}

		Project(mx.data());
		ShurProduct(mx,mg_tmp);

#pragma omp parallel for reduction(+:obj1,obj2,dot_mg_ms,norm_ms)
		for(int i=0; i<SIZE; i++) {
			real _mg_tmp_ = mg_tmp[i];
			real _b_ = b[i];
			real _ms_ = .5*_mg_tmp_-_b_;
			real _mx_ = mx[i];
			real _my_ = my[i];
			obj1+=_mx_*_ms_;
			mg_tmp2[i] = _mg_tmp_ -_b_;
			_ms_ = .5*mg_tmp1[i]-_b_;
			obj2+=_my_*_ms_;
			_ms_ = _mx_-_my_;
			dot_mg_ms+=mg[i]*_ms_;
			norm_ms+=_ms_*_ms_;
		}

		norm_ms = sqrt(norm_ms);

		while (obj1 > obj2 + dot_mg_ms + 0.5 * L_k * powf(norm_ms, 2.0)) {
			L_k = step_grow * L_k;
			t_k = 1.0 / L_k;
			obj1 = dot_mg_ms = norm_ms =0;

#pragma omp parallel for
		for(int i=0; i<SIZE; i++) {
			mx[i] = -t_k*mg[i]+ my[i];
		}
		Project(mx.data());
		ShurProduct(mx,mg_tmp);
#pragma omp  parallelfor reduction(+:obj1,dot_mg_ms,norm_ms)
		for(int i=0; i<SIZE; i++) {
			real _mg_tmp_ = mg_tmp[i];
			real _b_ = b[i];
			real _ms_ = .5*_mg_tmp_-_b_;
			real _mx_ = mx[i];
			obj1+=_mx_*_ms_;
			_ms_ = _mx_-my[i];
			dot_mg_ms+=mg[i]*_ms_;
			norm_ms+=_ms_*_ms_;
			mg_tmp2[i] = _mg_tmp_ -_b_;
		}
	}

	norm_ms = sqrt(norm_ms);

	theta_k1 = (-pow(theta_k, 2) + theta_k * sqrt(pow(theta_k, 2) + 4)) / 2.0;
	beta_k1 = theta_k * (1.0 - theta_k) / (pow(theta_k, 2) + theta_k1);
	real dot_mg_ms=0;
#pragma omp parallel for reduction(+:dot_mg_ms)
		for(int i=0; i<SIZE; i++) {
			real _mx_ = mx[i];
			real _ms_ = _mx_-ml[i];
			my[i] = beta_k1*_ms_+_mx_;
			dot_mg_ms+=mg[i]*_ms_;
		}

		if (dot_mg_ms > 0) {
			my = mx;
			theta_k1 = 1.0;
		}
		L_k = step_shrink * L_k;
		t_k = 1.0 / L_k;
		ml = mx;

		theta_k = theta_k1;
		//if(current_iteration%2==0) {
		real g_proj_norm = Res4(number_of_rigid_rigid*6, mg_tmp.data(), b.data(), ml.data(), mb_tmp.data());

		if(number_of_bilaterals>0) {

			real resid_bilat=-1;

			for (int i = number_of_rigid_rigid*6; i < x.size(); i++) {
				resid_bilat = max(resid_bilat, fabs(mg_tmp2[i]));
			}
			g_proj_norm = max(g_proj_norm,resid_bilat );
			//cout<<resid_bilat<<endl;
		}

		if(g_proj_norm < lastgoodres) {
			lastgoodres = g_proj_norm;
			ml_candidate = ml;
		}
		residual=lastgoodres;
		real maxdeltalambda = 0;     //CompRes(b,number_of_rigid_rigid);     //NormInf(ms);

		AtIterationEnd(residual, maxdeltalambda, current_iteration);
		if(collision_inside) {
			UpdatePosition(ml_candidate);
			UpdateContacts();
		}
		//}
		if (residual < tolerance) {
			break;
		}

	}
	x=ml_candidate;
	return current_iteration;
}

