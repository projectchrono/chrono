#include "ChSolverParallel.h"
using namespace chrono;


uint ChSolverParallel::SolveCG(std::vector<real> &x, const std::vector<real> &b, const uint max_iter) {
	std::vector<real> r(x.size()), p, Ap(x.size());
    real rsold, alpha, rsnew = 0, normb = Norm(b);
    if (normb == 0.0) {
        normb = 1;
    }
    ShurProduct(x,r);
    p = r = b - r;
    rsold = Dot(r, r);
    normb = 1.0 / normb;
    if (sqrt(rsold) * normb <= tolerance) {
        return 0;
    }
    for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
        ShurProduct(p,Ap);
        alpha = rsold / Dot(p, Ap);
        rsnew = 0;
#ifdef SIM_ENABLE_GPU_MODE
        SEAXPY(alpha, p, x, x);
        SEAXPY(-alpha, Ap, r, r);
        rsnew=Dot(r,r);
#else
        #pragma omp parallel for reduction(+:rsnew)
        for (int i = 0; i < x.size(); i++) {
            x[i] = x[i] + alpha * p[i];
            real _r = r[i] - alpha * Ap[i];
            r[i] = _r;
            rsnew += _r*_r;
        }
#endif
        residual = sqrt(rsnew) * normb;
        if (residual < tolerance) {
            break;
        }
        SEAXPY(rsnew / rsold, p, r, p); //p = r + rsnew / rsold * p;
        rsold = rsnew;

        AtIterationEnd(residual, 0, current_iteration);

    }
    Project(x.data());
    return current_iteration;
}
