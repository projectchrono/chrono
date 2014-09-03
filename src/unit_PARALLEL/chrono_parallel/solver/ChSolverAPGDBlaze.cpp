#include "ChSolverAPGDBlaze.h"

using namespace chrono;

void ChSolverAPGDBlaze::SetAPGDParams(real theta_k,
                                      real shrink,
                                      real grow) {
   init_theta_k = theta_k;
   step_shrink = shrink;
   step_grow = grow;

}

real ChSolverAPGDBlaze::Res4(const int SIZE,
                             blaze::DynamicVector<real> & mg_tmp2,
                             blaze::DynamicVector<real> &x,
                             blaze::DynamicVector<real> & mb_tmp) {
   real gdiff = 1e-6;
   real sum = 0;

//#pragma omp  parallel for
//   for (int i = 0; i < SIZE; i++) {
//      mb_tmp[i] = -gdiff * mg_tmp2[i] + x[i];
//   }

   mb_tmp = -gdiff * mg_tmp2 + x;

   Project(mb_tmp.data());
   //ms = mb_tmp - x;
   //mb_tmp = (-1.0 / (gdiff)) * ms;

//#pragma omp  parallel for reduction(+:sum)
//   for (int i = 0; i < SIZE; i++) {
//      real _ms_ = mb_tmp[i] - x[i];
//      real _mb_tmp_ = (-1.0f / (gdiff)) * _ms_;
//      sum += _mb_tmp_ * _mb_tmp_;
//   }
   mb_tmp = (-1.0f / (gdiff)) * (mb_tmp - x);
   sum = (mb_tmp, trans(mb_tmp));

   return sqrt(sum);

}

uint ChSolverAPGDBlaze::SolveAPGDBlaze(const uint max_iter,
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
   mb.resize(size);
   mso.resize(size);

   data_container->system_timer.start("ChSolverParallel_solverA");
   real lastgoodres = 10e30;
   real theta_k = init_theta_k;
   real theta_k1 = theta_k;
   real beta_k1 = 0.0;
   real L_k, t_k;
   real mb_tmp_norm = 0, mg_tmp_norm = 0;
   real obj1 = 0.0, obj2 = 0.0;
   real dot_mg_ms = 0, norm_ms = 0;
   real delta_obj = 1e8;
   real maxdeltalambda = 0;
   for (int i = 0; i < size; i++) {
      ml[i] = x[i];
      mb[i] = rhs[i];
   }

   custom_vector<real3> vel_data, omg_data;
   custom_vector<real> b = rhs;
//#pragma omp  parallel for
//    for(int i=0; i<SIZE; i++) {
//       ml[i] = 0;
//    }

   Project(ml.data());
   ml_candidate = ml;
   //ShurProduct(ml, mg);  //mg is never used, only re-written
   mg = data_container->host_data.Nshur * ml;
#pragma omp parallel for reduction(+:mb_tmp_norm)
   for (int i = 0; i < size; i++) {
      real _mb_tmp_ = -1.0 + ml[i];
      mb_tmp_norm += _mb_tmp_ * _mb_tmp_;
      mb_tmp[i] = _mb_tmp_;
      mg[i] = mg[i] - b[i];
   }

   //ShurProduct(mb_tmp, mg_tmp);
   mg_tmp = data_container->host_data.Nshur * mb_tmp;
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

      //ShurProduct(my, mg_tmp1);
      mg_tmp1 = data_container->host_data.Nshur * my;
      data_container->system_timer.stop("ChSolverParallel_solverB");
      data_container->system_timer.start("ChSolverParallel_solverC");
//#pragma omp parallel for
//      for (int i = 0; i < size; i++) {
//         real _mg_ = mg_tmp1[i] - b[i];
//         mg[i] = _mg_;
//         mx[i] = -t_k * _mg_ + my[i];
//      }

      mg = mg_tmp1 - mb;
      mx = -t_k * mg + my;

      Project(mx.data());
      //ShurProduct(mx, mg_tmp);
      mg_tmp = data_container->host_data.Nshur * mx;
      data_container->system_timer.stop("ChSolverParallel_solverC");
      data_container->system_timer.start("ChSolverParallel_solverD");

//      #pragma omp parallel for reduction(+:obj1,obj2,dot_mg_ms,norm_ms)
//      for (int i = 0; i < size; i++) {
//         real _mg_tmp_ = mg_tmp[i];
//         real _b_ = b[i];
//         real _ms_ = .5 * _mg_tmp_ - _b_;
//         real _mx_ = mx[i];
//         real _my_ = my[i];
//         obj1 += _mx_ * _ms_;
//         mg_tmp2[i] = _mg_tmp_ - _b_;
//         _ms_ = .5 * mg_tmp1[i] - _b_;
//         obj2 += _my_ * _ms_;
//         _ms_ = _mx_ - _my_;
//         dot_mg_ms += mg[i] * _ms_;
//         norm_ms += _ms_ * _ms_;
//      }
      mso = .5 * mg_tmp - mb;
      obj1 = (mx, trans(mso));
      mg_tmp2 = mg_tmp - mb;
      ms = .5 * mg_tmp1 - mb;
      obj2 = (my, trans(ms));
      ms = mx - my;
      dot_mg_ms = (mg, trans(ms));
      norm_ms = (ms, trans(ms));

      data_container->system_timer.stop("ChSolverParallel_solverD");
      while (obj1 > obj2 + dot_mg_ms + 0.5 * L_k * norm_ms) {
         data_container->system_timer.start("ChSolverParallel_solverE");
         L_k = step_grow * L_k;
         t_k = 1.0 / L_k;
         obj1 = dot_mg_ms = norm_ms = 0;

//#pragma omp parallel for
//         for (int i = 0; i < size; i++) {
//            mx[i] = -t_k * mg[i] + my[i];
//         }

         mx = -t_k * mg + my;

         Project(mx.data());
         //ShurProduct(mx, mg_tmp);
         mg_tmp = data_container->host_data.Nshur * mx;
//#pragma omp  parallel for reduction(+:obj1,dot_mg_ms,norm_ms)
//         for (int i = 0; i < size; i++) {
//            real _mg_tmp_ = mg_tmp[i];
//            real _b_ = b[i];
//            real _ms_ = .5 * _mg_tmp_ - _b_;
//            real _mx_ = mx[i];
//            obj1 += _mx_ * _ms_;
//            _ms_ = _mx_ - my[i];
//            dot_mg_ms += mg[i] * _ms_;
//            norm_ms += _ms_ * _ms_;
//            //mg_tmp2[i] = _mg_tmp_ - _b_;
//         }

         mso = .5 * mg_tmp - mb;
         obj1 = (mx, trans(mso));
         mg_tmp2 = mg_tmp - mb;
         ms = mx - my;
         dot_mg_ms = (mg, trans(ms));
         norm_ms = (ms, trans(ms));

         //step_grow += .5;

         data_container->system_timer.stop("ChSolverParallel_solverE");
      }
      data_container->system_timer.start("ChSolverParallel_solverF");

      theta_k1 = (-pow(theta_k, 2) + theta_k * sqrt(pow(theta_k, 2) + 4)) / 2.0;
      beta_k1 = theta_k * (1.0 - theta_k) / (pow(theta_k, 2) + theta_k1);
      real dot_mg_ms = 0;
//#pragma omp parallel for reduction(+:dot_mg_ms)
//      for (int i = 0; i < size; i++) {
//         real _mx_ = mx[i];
//         real _ms_ = _mx_ - ml[i];
//         my[i] = beta_k1 * _ms_ + _mx_;
//         dot_mg_ms += mg[i] * _ms_;
//      }
      ms = mx - ml;
      my = beta_k1 * ms + mx;
      dot_mg_ms = (mg, trans(ms));

      if (dot_mg_ms > 0) {
         my = mx;
         theta_k1 = 1.0;
      }
      L_k = step_shrink * L_k;
      t_k = 1.0 / L_k;
      ml = mx;
      step_grow = 2.0;
      theta_k = theta_k1;
      //if (current_iteration % 2 == 0) {
      real g_proj_norm = Res4(num_unilaterals, mg_tmp2, ml, mb_tmp);
      //real objective = 100;
//      if(current_iteration==0){
//         old_objective = GetObjective(ml,b);
//      }else{
//         objective = GetObjective(ml,b);
//         g_proj_norm = fabs(objective - old_objective);
//         old_objective = objective;
//      }

      if (num_bilaterals > 0) {
         real resid_bilat = -1;
         for (int i = num_unilaterals; i < x.size(); i++) {
            resid_bilat = std::max(resid_bilat, std::abs(mg_tmp2[i]));
         }
         g_proj_norm = std::max(g_proj_norm, resid_bilat);
      }

      bool update = false;
      if (g_proj_norm < lastgoodres) {
         lastgoodres = g_proj_norm;
         ml_candidate = ml;
         maxdeltalambda = (ml_candidate, mso);
         //maxdeltalambda = GetObjectiveBlaze(ml_candidate, mb);
         update = true;

      }

      residual = lastgoodres;
      //CompRes(b,num_contacts);     //NormInf(ms);
      //if (update_rhs) {
      //ComputeSRhs(ml_candidate, rhs, vel_data, omg_data, b);
      //}
//      if (collision_inside) {
//         //UpdatePosition(ml_candidate);
//         //UpdateContacts();
//      }
      //}

      AtIterationEnd(residual, maxdeltalambda, iter_hist.size());
      //cout << "delta_obj " << delta_obj<<" "<< residual<<" "<<current_iteration<< endl;
      if (tol_objective) {
         if (maxdeltalambda <= tolerance) {
            break;
         }
      } else {
         if (residual < tolerance) {
            break;
         }
      }
      data_container->system_timer.stop("ChSolverParallel_solverF");
   }
   for (int i = 0; i < size; i++) {
      x[i] = ml_candidate[i];
   }
   return current_iteration;
}
