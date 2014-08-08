#include "ChSolverCG.h"

using namespace chrono;

uint ChSolverCG::SolveCG(const uint max_iter,
                         const uint size,
                         const custom_vector<real> &b,
                         custom_vector<real> &x) {
   r.resize(size), Ap.resize(size);
   real rsold, alpha, rsnew = 0, normb = Norm(b);
   if (normb == 0.0) {
      normb = 1;
   }
   ShurProduct(x, r);
   p = r = b - r;
   rsold = Dot(r, r);
   normb = 1.0 / normb;
   if (sqrt(rsold) * normb <= tolerance) {
      return 0;
   }
   for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
      ShurProduct(p, Ap);
      alpha = rsold / Dot(p, Ap);
      rsnew = 0;
#ifdef SIM_ENABLE_GPU_MODE
      SEAXPY(alpha, p, x, x);
      SEAXPY(-alpha, Ap, r, r);
      rsnew=Dot(r,r);
#else
#pragma omp parallel for reduction(+:rsnew)
      for (int i = 0; i < size; i++) {
         x[i] = x[i] + alpha * p[i];
         real _r = r[i] - alpha * Ap[i];
         r[i] = _r;
         rsnew += _r * _r;
      }
#endif
      residual = sqrt(rsnew) * normb;
      if (residual < tolerance) {
         break;
      }
      SEAXPY(rsnew / rsold, p, r, p);  //p = r + rsnew / rsold * p;
      rsold = rsnew;

      AtIterationEnd(residual, 0, current_iteration);

   }
   Project(x.data());
   return current_iteration;
}
