#include "ChSolverParallel.h"
using namespace chrono;

uint ChSolverParallel::SolveGD(std::vector<real> &x, const std::vector<real> &b, const uint max_iter) {
	real eps = step_size;
	std::vector<real> r(x.size());
	ShurProduct(x,r);
	r = b - r;
	real resold = 1, resnew, normb = Norm(b);
	if (normb == 0.0) {
		normb = 1;
	};
	for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
		SEAXPY(eps, r, x, x); //x = x + eps *r;
		ShurProduct(x,r);
		r = b - r;
		resnew = Norm(x);
		residual = abs(resnew - resold);
		if (residual < tolerance) {
			break;
		}
		resold = resnew;
	}
	Project(x.data());
	return current_iteration;
}
