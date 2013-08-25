#include "ChSolverGPU.cuh"
using namespace chrono;

void ChSolverGPU::InitAPGD(custom_vector<real> &x) {
	ms.resize(x.size());
	mg_tmp2.resize(x.size());
	mb_tmp.resize(x.size());
	mg_tmp.resize(x.size());
	mg_tmp1.resize(x.size());

	mg_tmp.resize(x.size());
	mg_tmp.resize(x.size());

	mg.resize(x.size());

	ml.resize(x.size());

}

uint ChSolverGPU::SolveAPGD(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter) {
	bool verbose = false;
	real gdiff = 0.000001;

	real lastgoodres=10e30;
	real theta_k=1.0;
	real theta_k1=theta_k;
	real beta_k1=0.0;
	ml = x;
	Project(x);
	ShurProduct(x,mg);
	mg=mg- b;

	Thrust_Fill(mb_tmp, -1.0);
	mb_tmp+=x;
	ShurProduct(mb_tmp,mg_tmp);
	real L_k;
	if (Norm(mb_tmp) == 0) {
		L_k = 1;
		std::cout<<"HERE"<<std::endl;
	} else {
		L_k =Norm(mg_tmp) / Norm(mb_tmp);
	}

	real t_k = 1.0 / L_k;
	if (verbose) cout << "L_k:" << L_k << " t_k:" << -t_k << "\n";
	custom_vector<real> my = x;
	custom_vector<real> mx = x;

	real obj1=0.0;
	real obj2=0.0;

	for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
		ShurProduct(my,mg_tmp1);
		mg = mg_tmp1-b;
		SEAXPY(-t_k, mg, my, mx);     // mx = my + mg*(t_k);
		Project(mx);
		ShurProduct(mx,mg_tmp);
		mg_tmp2 = mg_tmp-b;
		SEAXMY(.5,mg_tmp,b,ms);//use ms as a temp variable
		obj1 = Dot(mx, ms);
		SEAXMY(.5,mg_tmp1,b,ms);//use ms as a temp variable
		obj2 = Dot(my, ms);
		ms = mx - my;

		while (obj1 > obj2 + Dot(mg, ms) + 0.5 * L_k * powf(Norm(ms), 2.0)) {
			L_k = 2 * L_k;
			t_k = 1.0 / L_k;
			SEAXPY(-t_k, mg, my, mx);     // mx = my + mg*(t_k);
			Project(mx);

			ShurProduct(mx,mg_tmp);
			mg_tmp2 = mg_tmp-b;
			SEAXMY(.5,mg_tmp,b,ms);	//use ms as a temp variable
			obj1 = Dot(mx, ms);
			ms = mx - my;
		}
		theta_k1 = (-pow(theta_k, 2) + theta_k * sqrt(pow(theta_k, 2) + 4)) / 2.0;
		beta_k1 = theta_k * (1.0 - theta_k) / (pow(theta_k, 2) + theta_k1);

		ms = mx - ml;
		SEAXPY(beta_k1,ms,mx,my);     //my = mx + beta_k1 * (ms);

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
		real g_proj_norm=CompRes(mg_tmp2,number_of_rigid_rigid);
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
		real maxdeltalambda = 0;     //NormInf(ms);

		AtIterationEnd(residual, maxdeltalambda, current_iteration);

		if (residual < tolerance) {
			break;
		}
	}
	//cout<<x<<endl;
//x=ml_candidate;
	return current_iteration;
}

