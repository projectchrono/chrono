#include "ChSolverCG.h"

using namespace chrono;

uint ChSolverCG::SolveCG(const uint max_iter,
                         const uint size,
                         const custom_vector<real> &b,
                         custom_vector<real> &x) {
   r.resize(size), Ap.resize(size);
   ml.resize(size);
   mb.resize(size);
#pragma omp parallel for
   for (int i = 0; i < size; i++) {
      ml[i] = x[i];
      mb[i] = b[i];
   }

   real rsold, alpha, rsnew = 0, normb = sqrt((mb, mb));
   if (normb == 0.0) {
      normb = 1;
   }
   r = data_container->host_data.Nshur * ml;
   p = r = mb - r;
   rsold = (r, r);
   normb = 1.0 / normb;
   if (sqrt(rsold) * normb <= data_container->settings.solver.tolerance) {
      return 0;
   }
   for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
      Ap = data_container->host_data.Nshur * p;
      alpha = rsold / Dot(p, Ap);
      rsnew = 0;
      ml = alpha * p + ml;
      r = -alpha * Ap + r;
      rsnew = (r, r);

      residual = sqrt(rsnew) * normb;
      if (residual < data_container->settings.solver.tolerance) {
         break;
      }
      p = rsnew / rsold * p + r;
      rsold = rsnew;

      objective_value = GetObjectiveBlaze(ml, mb);
      AtIterationEnd(residual, objective_value, iter_hist.size());
   }
   Project(ml.data());
#pragma omp parallel for
   for (int i = 0; i < size; i++) {
      x[i] = ml[i];
   }
   return current_iteration;
}
