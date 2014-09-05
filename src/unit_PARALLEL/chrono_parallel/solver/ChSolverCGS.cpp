#include "ChSolverCGS.h"

using namespace chrono;

uint ChSolverCGS::SolveCGS(const uint max_iter,
                           const uint size,
                           const custom_vector<real> &b,
                           custom_vector<real> &x) {

   r.resize(size);
   qhat.resize(size);
   vhat.resize(size);
   uhat.resize(size);
   ml.resize(size);
   mb.resize(size);
#pragma omp parallel for
   for (int i = 0; i < size; i++) {
      ml[i] = x[i];
      mb[i] = b[i];
   }

   r = data_container->host_data.Nshur * ml;
   r = mb - r;
   p = r;
   q = r;

   u = r;

   real normb = sqrt((mb, mb));
   rtilde = r;

   if (normb == 0.0) {
      normb = 1;
   }

   if ((sqrt((r, r)) / normb) <= tolerance) {
      return 0;
   }

   for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
      rho_1 = Dot(rtilde, r);

      if (rho_1 == 0) {
         break;
      }

      if (current_iteration > 0) {
         beta = rho_1 / rho_2;

         u = r + beta * q;
         p = u + beta * (q + beta * p);
      }

      phat = p;
      vhat = data_container->host_data.Nshur * phat;
      alpha = rho_1 / Dot(rtilde, vhat);
      q = u - alpha * vhat;
      uhat = (u + q);
      ml = ml + alpha * uhat;
      qhat = data_container->host_data.Nshur * uhat;
      r = r - alpha * qhat;
      rho_2 = rho_1;
      residual = (sqrt((r, r)) / normb);

      objective_value = GetObjectiveBlaze(ml, mb);
      AtIterationEnd(residual, objective_value, iter_hist.size());

      if (residual < tolerance) {
         break;
      }
   }
   Project(ml.data());

#pragma omp parallel for
   for (int i = 0; i < size; i++) {
      x[i] = ml[i];
   }

   return current_iteration;
}
