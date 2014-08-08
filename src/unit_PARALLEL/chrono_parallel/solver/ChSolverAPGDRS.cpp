#include "chrono_parallel/ChConfigParallel.h"
#include "ChSolverParallel.h"

using namespace chrono;
uint ChSolverParallel::SolveAPGDRS(
                                   const uint max_iter,
                                   const uint size,
                                   custom_vector<real> &rhs,
                                   custom_vector<real> &x) {
   ms.resize(size);
   mg_tmp2.resize(size);
   mb_tmp.resize(size);
   mg_tmp.resize(size);
   mg_tmp1.resize(size);
   mg.resize(size);
   ml.resize(size);
   mx.resize(size);
   my.resize(size);

   data_container->system_timer.start("ChSolverParallel_solverA");
   real lastgoodres = 10e30;
   real theta_k = init_theta_k;
   real theta_k1 = theta_k;
   real beta_k1 = 0.0;
   real L_k, t_k;
   real mb_tmp_norm = 0, mg_tmp_norm = 0;
   real obj1 = 0.0, obj2 = 0.0;
   real dot_mg_ms = 0, norm_ms = 0;

   ml = x;

   custom_vector<real3> vel_data, omg_data;
   custom_vector<real> b = rhs;
//#pragma omp  parallel for
//		for(int i=0; i<SIZE; i++) {
//			ml[i] = 0;
//		}

   Project(ml.data());
   ml_candidate = ml;
   ShurProduct(ml, mg);  //mg is never used, only re-written

#pragma omp parallel for reduction(+:mb_tmp_norm)
   for (int i = 0; i < size; i++) {
      real _mb_tmp_ = -1.0f + ml[i];
      mb_tmp_norm += _mb_tmp_ * _mb_tmp_;
      mb_tmp[i] = _mb_tmp_;
      mg[i] = mg[i] - b[i];
   }

   ShurProduct(mb_tmp, mg_tmp);
   mb_tmp_norm = sqrt(mb_tmp_norm);

   if (mb_tmp_norm == 0) {
      L_k = 1;
   } else {
      L_k = Norm(mg_tmp) / mb_tmp_norm;
   }

   t_k = 1.0 / L_k;
   my = ml;
   mx = ml;
   data_container->system_timer.stop("ChSolverParallel_solverA");

   for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {

      data_container->system_timer.start("ChSolverParallel_solverB");
      obj1 = obj2 = 0.0;
      dot_mg_ms = 0;
      norm_ms = 0;

      ShurProduct(my, mg_tmp1);
    data_container->system_timer.stop("ChSolverParallel_solverB");
    data_container->system_timer.start("ChSolverParallel_solverC");
#pragma omp parallel for
      for (int i = 0; i < size; i++) {
         real _mg_ = mg_tmp1[i] - b[i];
         mg[i] = _mg_;
         mx[i] = -t_k * _mg_ + my[i];
      }

      Project(mx.data());
      ShurProduct(mx, mg_tmp);
      data_container->system_timer.stop("ChSolverParallel_solverC");
      data_container->system_timer.start("ChSolverParallel_solverD");
#pragma omp parallel for reduction(+:obj1,obj2,dot_mg_ms,norm_ms)
      for (int i = 0; i < size; i++) {
         real _mg_tmp_ = mg_tmp[i];
         real _b_ = b[i];
         real _ms_ = .5 * _mg_tmp_ - _b_;
         real _mx_ = mx[i];
         real _my_ = my[i];
         obj1 += _mx_ * _ms_;
         mg_tmp2[i] = _mg_tmp_ - _b_;
         _ms_ = .5 * mg_tmp1[i] - _b_;
         obj2 += _my_ * _ms_;
         _ms_ = _mx_ - _my_;
         dot_mg_ms += mg[i] * _ms_;
         norm_ms += _ms_ * _ms_;
      }

      norm_ms = sqrt(norm_ms);
      data_container->system_timer.stop("ChSolverParallel_solverD");
      while (obj1 > obj2 + dot_mg_ms + 0.5 * L_k * powf(norm_ms, 2.0)) {
         data_container->system_timer.start("ChSolverParallel_solverE");
         L_k = step_grow * L_k;
         t_k = 1.0 / L_k;
         obj1 = dot_mg_ms = norm_ms = 0;

#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
         for (int i = 0; i < size; i++) {
            mx[i] = -t_k * mg[i] + my[i];
         }
         Project(mx.data());
         ShurProduct(mx, mg_tmp);
#pragma omp  parallel for reduction(+:obj1,dot_mg_ms,norm_ms)
         for (int i = 0; i < size; i++) {
            real _mg_tmp_ = mg_tmp[i];
            real _b_ = b[i];
            real _ms_ = .5 * _mg_tmp_ - _b_;
            real _mx_ = mx[i];
            obj1 += _mx_ * _ms_;
            _ms_ = _mx_ - my[i];
            dot_mg_ms += mg[i] * _ms_;
            norm_ms += _ms_ * _ms_;
            mg_tmp2[i] = _mg_tmp_ - _b_;
         }
         norm_ms = sqrt(norm_ms);
         step_grow += .5;

         data_container->system_timer.stop("ChSolverParallel_solverE");
      }
      data_container->system_timer.start("ChSolverParallel_solverF");

      theta_k1 = (-pow(theta_k, 2) + theta_k * sqrt(pow(theta_k, 2) + 4)) / 2.0;
      beta_k1 = theta_k * (1.0 - theta_k) / (pow(theta_k, 2) + theta_k1);
      real dot_mg_ms = 0;
#pragma omp parallel for reduction(+:dot_mg_ms)
      for (int i = 0; i < size; i++) {
         real _mx_ = mx[i];
         real _ms_ = _mx_ - ml[i];
         my[i] = beta_k1 * _ms_ + _mx_;
         dot_mg_ms += mg[i] * _ms_;
      }

      if (dot_mg_ms > 0) {
         my = mx;
         theta_k1 = 1.0;
      }
      L_k = step_shrink * L_k;
      t_k = 1.0 / L_k;
      ml = mx;
      step_grow = 2.0;
      theta_k = theta_k1;
      if (current_iteration % 2 == 0) {
         real g_proj_norm = Res4(num_unilaterals, mg_tmp.data(), b.data(), ml.data(), mb_tmp.data());
         if (num_bilaterals > 0) {
            real resid_bilat = -1;
            for (int i = num_unilaterals; i < x.size(); i++) {
               resid_bilat = max(resid_bilat, fabs(mg_tmp2[i]));
            }
            g_proj_norm = max(g_proj_norm, resid_bilat);
         }

         if (g_proj_norm < lastgoodres) {
            lastgoodres = g_proj_norm;
            ml_candidate = ml;
         }

         residual = lastgoodres;
         //CompRes(b,num_contacts);     //NormInf(ms);
         if (update_rhs) {
            ComputeSRhs(ml_candidate, rhs, vel_data, omg_data, b);
         }
         if (collision_inside) {
            UpdatePosition(ml_candidate);
            UpdateContacts();
         }
      }
      real maxdeltalambda = GetObjective(x, b);
      AtIterationEnd(residual, maxdeltalambda, iter_hist.size());
      if (residual < tolerance) {
         break;
      }
      data_container->system_timer.stop("ChSolverParallel_solverF");
   }
   x = ml_candidate;
   return current_iteration;
}

