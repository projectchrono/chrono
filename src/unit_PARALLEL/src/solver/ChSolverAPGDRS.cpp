#include "ChSolverParallel.h"
using namespace chrono;

uint ChSolverParallel::SolveAPGDRS(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter) {
	bool verbose = false;
	real gdiff = 1;

	real lastgoodres=10e30;
	real theta_k=init_theta_k;
	real theta_k1=theta_k;
	real beta_k1=0.0;
	custom_vector<real> x_initial =x;

	ml = x;
	Project(x);
	ShurProduct(x,mg);
	real norm_mb_tmp = 0;
#pragma omp parallel for reduction(+:norm_mb_tmp)
		for(int i=0; i<mg.size(); i++) {
			mg[i] = mg[i]-b[i];
			real _mb_tmp_ = -1.0+x[i];
			norm_mb_tmp +=_mb_tmp_*_mb_tmp_;
			mb_tmp[i] = _mb_tmp_;

		}
		norm_mb_tmp = sqrt(norm_mb_tmp);
		ShurProduct(mb_tmp,mg_tmp);
		real L_k;
		real mb_tmp_norm = norm_mb_tmp;
		if (mb_tmp_norm == 0) {
			L_k = 1;
		} else {
			L_k =Norm(mg_tmp) / mb_tmp_norm;
		}

		real t_k = 1.0 / L_k;
		if (verbose) cout << "L_k:" << L_k << " t_k:" << -t_k << "\n";
		my = x;
		mx = x;

		real obj1=0.0;
		real obj2=0.0;

		for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
			ShurProduct(my,mg_tmp1);
#pragma omp parallel for
		for(int i=0; i<mg.size(); i++) {
			real _mg_ = mg_tmp1[i]-b[i];
			mg[i] = _mg_;
			mx[i] = -t_k*_mg_+my[i];
		}
		Project(mx);
		ShurProduct(mx,mg_tmp);
		obj1=0.0;
		obj2=0.0;

		real dot_mg_ms = 0;
		real norm_ms = 0;

#pragma omp parallel for reduction(+:obj1,obj2,dot_mg_ms,norm_ms)
		for(int i=0; i<mg_tmp.size(); i++) {
			real _mg_tmp_ = mg_tmp[i];
			real _b_ = b[i];
			//mg_tmp2[i] = _mg_tmp_-_b_;
		real _ms_ = .5*_mg_tmp_-_b_;
		real _mx_ = mx[i];
		real _my_ = my[i];
		obj1+=_mx_*_ms_;

		_ms_ = .5*mg_tmp1[i]-_b_;
		obj2+=_my_*_ms_;
		_ms_ = _mx_-_my_;
		dot_mg_ms+=mg[i]*_ms_;
		norm_ms+=_ms_*_ms_;
	}
	norm_ms = sqrt(norm_ms);
	//cout<<obj1<<" "<<obj2<<endl;
	while (obj1 > obj2 + dot_mg_ms + 0.5 * L_k * powf(norm_ms, 2.0)) {
		L_k = step_grow * L_k;
		t_k = 1.0 / L_k;
		SEAXPY(-t_k, mg, my, mx);     // mx = my + mg*(t_k);
		Project(mx);

		ShurProduct(mx,mg_tmp);
		obj1 = dot_mg_ms = norm_ms =0;
#pragma omp parallel for reduction(+:obj1,dot_mg_ms,norm_ms)
		for(int i=0; i<x.size(); i++) {
			real _mg_tmp_ = mg_tmp[i];
			real _b_ = b[i];
			//mg_tmp2[i] = _mg_tmp_-_b_;
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
		for(int i=0; i<ms.size(); i++) {
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

		//ml = mx;
		x = mx;
		theta_k = theta_k1;

		//this is res1
		//real g_proj_norm=fmax(real(0.0),-min_val);
		//real g_proj_norm=0;//CompRes(mg_tmp2,number_of_rigid_rigid);
		//real temp_norm = 0;

		//temp_norm = mg_tmp2[thrust::min_element(thrust::omp::par,mg_tmp2.begin(),mg_tmp2.end())-mg_tmp2.begin()];
		//cout<<"norm1: "<<temp_norm<<endl;
		//g_proj_norm = fmax(real(0.0),-temp_norm);
		//temp_norm = mg_tmp2[thrust::min_element(thrust::omp::par,mg_tmp2.begin()+number_of_rigid_rigid*6,mg_tmp2.end())-mg_tmp2.begin()];
		//cout<<"norm2: "<<temp_norm<<endl;
		//g_proj_norm = fmax(g_proj_norm,-temp_norm);
		//
		//			int count_resolved = mg_tmp2.size();
		//
		//			for(int i=0; i<mg_tmp2.size(); i++) {
		//				if(mg_tmp2[i]<tolerance) {
		//
		//					count_resolved --;
		//				}
		//			}
		//			cout<<"resolved "<<count_resolved<<endl;
		//cout<<"MINVAL "<<g_proj_norm<<" "<<fmax(real(0.0),-min_val)<<endl;
		//this is res4
		if(current_iteration%5==0) {
			Sub(mg_tmp2,mg_tmp,b);
			real g_proj_norm = Res4(mg_tmp2, x, mb_tmp);
			if(g_proj_norm < lastgoodres) {
				lastgoodres = g_proj_norm;
				//ml_candidate = ml;
			}
			residual=lastgoodres;
			real maxdeltalambda = CompRes(b,number_of_rigid_rigid);     //NormInf(ms);

			AtIterationEnd(residual, maxdeltalambda, current_iteration);
		}

		//custom_vector<real> error = (x_initial-x)/x;
		//x_initial = x;

		//for(int i=0; i<x.size(); i++) {
		//	if(x[i]==0) {
		//		error[i]=0;
		//	}
		//}
		//summary_stats_data<real> result = Statistics(error);
		//cout<<"current_iteration: "<<current_iteration<<" Max: "<<result.max<<" Mean: "<<result.mean<<" StdDev: "<<std::sqrt(result.variance_n())<<" Variance: "<<result.variance()<<endl;
		if (residual < tolerance) {
			break;
		}
	}
//cout<<x<<endl;
//x=ml_candidate;
	return current_iteration;
}

