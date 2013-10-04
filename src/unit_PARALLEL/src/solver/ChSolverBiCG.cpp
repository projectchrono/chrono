#include "ChSolverParallel.h"
using namespace chrono;

uint ChSolverParallel::SolveBiCG(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter) {
	real rho_1, rho_2, alpha, beta;
	custom_vector<real> z, ztilde, p, ptilde, q(x.size()), qtilde(x.size()),r(x.size());
	real normb = Norm(b);
	ShurProduct(x,r);
	r= b - r;

	custom_vector<real> rtilde = r;

	if (normb == 0.0) {normb = 1;}

	if ((residual = Norm(r) / normb) <= tolerance) {
		return 0;
	}

	for (current_iteration = 0; current_iteration <= max_iter; current_iteration++) {
		z = (r);
		ztilde = (rtilde);
		rho_1 = Dot(z, rtilde);

		if (rho_1 == 0) {
			break;
		}

		if (current_iteration == 0) {
			p = z;
			ptilde = ztilde;
		} else {
			beta = rho_1 / rho_2;
			p = z + beta * p;
			ptilde = ztilde + beta * ptilde;
		}

		ShurProduct(p,q);
		ShurProduct(ptilde,qtilde);
		alpha = rho_1 / Dot(ptilde, q);
		x = x + alpha * p;
		r = r - alpha * q;
		rtilde = rtilde - alpha * qtilde;
		rho_2 = rho_1;
		residual = Norm(r) / normb;

		if (residual < tolerance) {break;}
	}

	return current_iteration;

}
