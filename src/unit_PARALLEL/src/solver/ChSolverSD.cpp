#include "ChSolverParallel.h"
using namespace chrono;

uint ChSolverParallel::SolveSD(const uint max_iter,const uint size,const custom_vector<real> &b,custom_vector<real> &x) {
	custom_vector<real> r(size),temp(size);
	ShurProduct(x,r);
	r = b - r;
	real resold = 1, resnew, normb = Norm(b), alpha;
	if (normb == 0.0) {
		normb = 1;
	}
	for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
		ShurProduct(r,temp);
		alpha = Dot(r, r) / Dot(r, temp);
		SEAXPY(alpha, r, x, x);     //x = x + eps *r;
		ShurProduct(x,r);
		r = b - r;
		resnew = Norm(x);
		residual = abs(resnew - resold);
		real maxdeltalambda = CompRes(b,num_contacts);//NormInf(ms);
		AtIterationEnd(residual, maxdeltalambda, current_iteration);
		if (residual < tolerance) {
			break;
		}
		resold = resnew;
	}
	Project(x.data());
	return current_iteration;
}
