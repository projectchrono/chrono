#include "ChSolverGD.h"

using namespace chrono;

uint ChSolverGD::SolveGD(const uint max_iter,const uint size,const custom_vector<real> &b,custom_vector<real> &x) {
	real eps = step_size;
	custom_vector<real> r(size);
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
