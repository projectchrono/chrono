#include "chrono_parallel/solver/ChSolverSD.h"

using namespace chrono;

uint ChSolverSD::SolveSD(const uint max_iter,
                         const uint size,
                         const custom_vector<real> &b,
                         custom_vector<real> &x) {
   r.resize(size);
   temp.resize(size);
   ml.resize(size);
   mb.resize(size);
#pragma omp parallel for
   for (int i = 0; i < size; i++) {
      ml[i] = x[i];
      mb[i] = b[i];
   }

   r = data_container->host_data.D_T * (data_container->host_data.M_invD * ml);

   r = mb - r;
   real resold = 1, resnew, normb = sqrt((mb, mb)), alpha;
   if (normb == 0.0) {
      normb = 1;
   }
   for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
      temp = data_container->host_data.D_T * (data_container->host_data.M_invD * r);
      alpha = Dot(r, r) / Dot(r, temp);
      ml = ml + alpha * r;
      r = data_container->host_data.D_T * (data_container->host_data.M_invD * ml);
      r = mb - r;
      resnew = sqrt((ml, ml));
      residual = std::abs(resnew - resold);

      AtIterationEnd(residual, GetObjectiveBlaze(ml, mb), iter_hist.size());
      if (residual < data_container->settings.solver.tolerance) {
         break;
      }
      resold = resnew;
   }
   Project(ml.data());

#pragma omp parallel for
   for (int i = 0; i < size; i++) {
      x[i] = ml[i];
   }
   return current_iteration;
}
