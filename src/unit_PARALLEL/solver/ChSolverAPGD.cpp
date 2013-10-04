#include "ChSolverParallel.h"
using namespace chrono;

void ChSolverParallel::InitAPGD(custom_vector<real> &x) {
	ms.resize(x.size());
	mg_tmp2.resize(x.size());
	mb_tmp.resize(x.size());
	mg_tmp.resize(x.size());
	mg_tmp1.resize(x.size());
	mg.resize(x.size());
	ml.resize(x.size());
	mx.resize(x.size());
	my.resize(x.size());

}

uint ChSolverParallel::SolveAPGD(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter) {
	bool verbose = false;
	real gdiff = 0.000001;

	real lastgoodres=10e30;
	real theta_k=1.0;
	real theta_k1=theta_k;
	real beta_k1=0.0;
	ml = x;
	Project(x);
	ShurProduct(x,mg);
	Sub(mg,mg,b);     //mg = mg -b;

		thrust::fill(mb_tmp.begin(),mb_tmp.end(),-1.0);
		PLUS_EQ(mb_tmp,x);//mb_tmp+=x;
		ShurProduct(mb_tmp,mg_tmp);
		real L_k;
		real mb_tmp_norm = Norm(mb_tmp);
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
			Sub(mg,mg_tmp1,b);     //mg = mg_tmp1-b;
			SEAXPY(-t_k, mg, my, mx);// mx = my + mg*(t_k);
			Project(mx);
			ShurProduct(mx,mg_tmp);
			Sub(mg_tmp2,mg_tmp,b);//mg_tmp2 = mg_tmp-b
			SEAXMY(.5,mg_tmp,b,ms);//use ms as a temp variable
			obj1 = Dot(mx, ms);
			SEAXMY(.5,mg_tmp1,b,ms);//use ms as a temp variable
			obj2 = Dot(my, ms);
			Sub(ms,mx,my);//ms = mx - my;

			while (obj1 > obj2 + Dot(mg, ms) + 0.5 * L_k * powf(Norm(ms), 2.0)) {
				L_k = 2 * L_k;
				t_k = 1.0 / L_k;
				SEAXPY(-t_k, mg, my, mx);     // mx = my + mg*(t_k);
				Project(mx);

				ShurProduct(mx,mg_tmp);
				Sub(mg_tmp2,mg_tmp,b);//mg_tmp2 = mg_tmp-b
				SEAXMY(.5,mg_tmp,b,ms);//use ms as a temp variable
				obj1 = Dot(mx, ms);
				Sub(ms,mx,my);//ms = mx - my;
			}
			theta_k1 = (-pow(theta_k, 2) + theta_k * sqrt(pow(theta_k, 2) + 4)) / 2.0;
			beta_k1 = theta_k * (1.0 - theta_k) / (pow(theta_k, 2) + theta_k1);

			Sub(ms,mx,ml);     //ms = mx - ml;
			SEAXPY(beta_k1,ms,mx,my);//my = mx + beta_k1 * (ms);

			if (Dot(mg, ms) > 0) {
				my = mx;
				theta_k1 = 1.0;
			}
			L_k = 0.9 * L_k;
			t_k = 1.0 / L_k;

			ml = mx;
			x = mx;
			theta_k = theta_k1;

			//this is res1
			//real g_proj_norm=fmax(real(0.0),-min_val);
			real g_proj_norm=0;//CompRes(mg_tmp2,number_of_rigid_rigid);
			real temp_norm = 0;

			temp_norm = mg_tmp2[thrust::min_element(thrust::omp::par,mg_tmp2.begin(),mg_tmp2.end())-mg_tmp2.begin()];
			//cout<<"norm1: "<<temp_norm<<endl;
			//g_proj_norm = fmax(real(0.0),-temp_norm);
			//temp_norm = mg_tmp2[thrust::min_element(thrust::omp::par,mg_tmp2.begin()+number_of_rigid_rigid*6,mg_tmp2.end())-mg_tmp2.begin()];
			//cout<<"norm2: "<<temp_norm<<endl;
			g_proj_norm = fmax(g_proj_norm,-temp_norm);
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
			//SEAXPY(-gdiff, mg_tmp2, x, mb_tmp); //mb_tmp=x+mg_tmp2*(-gdiff)
			//Project(mb_tmp);
			//d01=mb_tmp-x;
			//mb_tmp = (-1.0/gdiff)*d01;
			//real g_proj_norm = Norm(mb_tmp);

			if(g_proj_norm < lastgoodres) {
				lastgoodres = g_proj_norm;
				//ml_candidate = ml;
			}

			residual=lastgoodres;
			real maxdeltalambda = CompRes(b,number_of_rigid_rigid);     //NormInf(ms);

			AtIterationEnd(residual, maxdeltalambda, current_iteration);

			if (residual < tolerance) {
				break;
			}
		}
		//cout<<x<<endl;
//x=ml_candidate;
		return current_iteration;
	}

