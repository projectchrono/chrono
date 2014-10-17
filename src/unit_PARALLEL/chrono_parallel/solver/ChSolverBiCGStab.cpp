#include "chrono_parallel/solver/ChSolverBiCGStab.h"

using namespace chrono;

uint ChSolverBiCGStab::SolveBiCGStab(const uint max_iter,
                                     const uint size,
                                     const custom_vector<real> &b,
                                     custom_vector<real> &x) {
   real rho_1, rho_2, alpha = 1, beta, omega = 1;
   ml.resize(size);
   mb.resize(size);
#pragma omp parallel for
   for (int i = 0; i < size; i++) {
      ml[i] = x[i];
      mb[i] = b[i];
   }

   r.resize(size);
   t.resize(size);
   v.resize(size);
   real normb = sqrt((mb, mb));

   r = data_container->host_data.D_T * (data_container->host_data.M_invD * ml);
   p = r = mb - r;
   rtilde = r;

   if (normb == 0.0) {
      normb = 1;
   }

   if ((residual = sqrt((r, r)) / normb) <= data_container->settings.solver.tolerance) {
      return 0;
   }

   for (current_iteration = 0; current_iteration <= max_iter; current_iteration++) {
      rho_1 = Dot(rtilde, r);

      if (rho_1 == 0) {
         break;
      }

      if (current_iteration > 0) {
         beta = (rho_1 / rho_2) * (alpha / omega);
         p = r + beta * (p - omega * v);
      }

      phat = p;
      v = data_container->host_data.D_T * (data_container->host_data.M_invD * phat);
      alpha = rho_1 / Dot(rtilde, v);
      s = r - alpha * v;  //SEAXPY(-alpha,v,r,s);//
      residual = sqrt((s, s)) / normb;

      if (residual < data_container->settings.solver.tolerance) {

         ml = ml + alpha * phat;
         break;
      }

      shat = s;
      t = data_container->host_data.D_T * (data_container->host_data.M_invD * shat);
      omega = Dot(t, s) / Dot(t, t);
      ml = ml + alpha * phat + omega * shat;
      r = s - omega * t;
      rho_2 = rho_1;
      residual = Norm(r) / normb;

      objective_value = GetObjectiveBlaze(ml, mb);
      AtIterationEnd(residual, objective_value, iter_hist.size());

      if (residual < data_container->settings.solver.tolerance || omega == 0) {
         break;
      }
   }
#pragma omp parallel for
   for (int i = 0; i < size; i++) {
      x[i] = ml[i];
   }
   return current_iteration;
}
