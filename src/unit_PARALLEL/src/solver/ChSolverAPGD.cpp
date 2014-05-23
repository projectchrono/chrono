#include "ChSolverParallel.h"
using namespace chrono;

void ChSolverParallel::SetAPGDParams(
      real theta_k,
      real shrink,
      real grow) {
   init_theta_k = theta_k;
   step_shrink = shrink;
   step_grow = grow;

}

real ChSolverParallel::Res4(
      const int SIZE,
      real* mg_tmp,
      const real* b,
      real*x,
      real* mb_tmp) {
   real gdiff = 1e-6;
   real sum = 0;

#pragma omp  parallel for
   for (int i = 0; i < SIZE; i++) {
      real _mg_tmp2_ = mg_tmp[i] - b[i];
      mb_tmp[i] = -gdiff * _mg_tmp2_ + x[i];
   }

   Project(mb_tmp);
   //ms = mb_tmp - x;
   //mb_tmp = (-1.0 / (gdiff)) * ms;

#pragma omp  parallel for reduction(+:sum)
   for (int i = 0; i < SIZE; i++) {
      real _ms_ = mb_tmp[i] - x[i];
      real _mb_tmp_ = (-1.0f / (gdiff)) * _ms_;
      sum += _mb_tmp_ * _mb_tmp_;
   }

   return sqrt(sum);

}
uint ChSolverParallel::SolveAPGD(const uint max_iter,const uint size,const custom_vector<real> &b,custom_vector<real> &x) {
   ms.resize(size);
   mg_tmp2.resize(size);
   mb_tmp.resize(size);
   mg_tmp.resize(size);
   mg_tmp1.resize(size);
   mg.resize(size);
   ml.resize(size);
   mx.resize(size);
   my.resize(size);

   bool verbose = false;
   real lastgoodres = 10e30;
   real theta_k = init_theta_k;
   real theta_k1 = theta_k;
   real beta_k1 = 0.0;
   //custom_vector<real> x_initial =x;

   ml = x;

//#pragma omp  parallel for
//		for(int i=0; i<x.size(); i++) {
//			ml[i] = 0;
//		}

   Project(ml.data());

   ml_candidate = ml;

   ShurProduct(ml, mg);
   Sub(mg, mg, b); /*mg = mg -b;*/

   thrust::fill(mb_tmp.begin(), mb_tmp.end(), -1.0);
   mb_tmp += ml;
   ShurProduct(mb_tmp, mg_tmp);
   real L_k;
   real mb_tmp_norm = Norm(mb_tmp);
   if (mb_tmp_norm == 0) {
      L_k = 1;
   } else {
      L_k = Norm(mg_tmp) / mb_tmp_norm;
   }

   real t_k = 1.0 / L_k;
   if (verbose)
      cout << "L_k:" << L_k << " t_k:" << -t_k << "\n";
   my = ml;
   mx = ml;

   real obj1 = 0.0;
   real obj2 = 0.0;

   for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
      ShurProduct(my, mg_tmp1);
      Sub(mg, mg_tmp1, b);     //mg = mg_tmp1-b;
      SEAXPY(-t_k, mg, my, mx);     // mx = my + mg*(t_k);
      Project(mx.data());
      ShurProduct(mx, mg_tmp);
      Sub(mg_tmp2, mg_tmp, b);     //mg_tmp2 = mg_tmp-b
      SEAXMY(.5, mg_tmp, b, ms);     //use ms as a temp variable
      obj1 = Dot(mx, ms);
      SEAXMY(.5, mg_tmp1, b, ms);     //use ms as a temp variable
      obj2 = Dot(my, ms);
      Sub(ms, mx, my);     //ms = mx - my;
      //cout<<obj1<<" "<<obj2<<endl;
      while (obj1 > obj2 + Dot(mg, ms) + 0.5 * L_k * powf(Norm(ms), 2.0)) {
         L_k = step_grow * L_k;
         t_k = 1.0 / L_k;
         SEAXPY(-t_k, mg, my, mx);     // mx = my + mg*(t_k);
         Project(mx.data());

         ShurProduct(mx, mg_tmp);
         Sub(mg_tmp2, mg_tmp, b);     //mg_tmp2 = mg_tmp-b
         SEAXMY(.5, mg_tmp, b, ms);     //use ms as a temp variable
         obj1 = Dot(mx, ms);
         Sub(ms, mx, my);     //ms = mx - my;
      }
      theta_k1 = (-pow(theta_k, 2) + theta_k * sqrt(pow(theta_k, 2) + 4)) / 2.0;
      beta_k1 = theta_k * (1.0 - theta_k) / (pow(theta_k, 2) + theta_k1);

      Sub(ms, mx, ml);     //ms = mx - ml;
      SEAXPY(beta_k1, ms, mx, my);     //my = mx + beta_k1 * (ms);

      if (Dot(mg, ms) > 0) {
         my = mx;
         theta_k1 = 1.0;
      }
      L_k = step_shrink * L_k;
      t_k = 1.0 / L_k;

      ml = mx;
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

      real g_proj_norm = Res4(x.size(), mg_tmp.data(), b.data(), ml.data(), mb_tmp.data());

      if (num_bilaterals > 0) {

         real resid_bilat = -1;

         for (int i = number_of_rigid_rigid * 6; i < x.size(); i++) {
            resid_bilat = max(resid_bilat, fabs(mg_tmp2[i]));
         }
         g_proj_norm = max(g_proj_norm, resid_bilat);
         //cout<<resid_bilat<<endl;
      }

      if (g_proj_norm < lastgoodres) {
         lastgoodres = g_proj_norm;
         ml_candidate = ml;
      }

      residual = lastgoodres;
      real maxdeltalambda = CompRes(b, number_of_rigid_rigid);     //NormInf(ms);

      AtIterationEnd(residual, maxdeltalambda, current_iteration);
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
   x = ml_candidate;
   return current_iteration;
}

