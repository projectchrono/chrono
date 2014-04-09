#include "ChSolverParallel.h"
using namespace chrono;
//Copyright 2011, Kenny Erleben, DIKU
// Copyright 2012, Michael Andersen, DIKU

custom_vector<real> & fischer(std::vector<real> y, std::vector<real> x) {

	// Auxiliary function used by the Fischer-Newton method
	// Copyright 2011, Kenny Erleben, DIKU
	custom_vector<real> phi (x.size());

	for(int i=0; i<x.size(); i++) {
		phi[i] = powf(y[i] * y[i] + x[i] * x[i], .5) - y[i] - x[i];

	}

	return phi;
}

std::vector<real> & phi_lambda(std::vector<real> a,std::vector<real> b,real lambda) {

	std::vector<real> phi_l (a.size());
	//phi_l = lambda*fischer(a,b)+(1-lambda)*(max(0,a)*max(0,b));
	return phi_l;

}

uint ChSolverParallel::SolveFN(std::vector<real> &x0, const std::vector<real> &b, const uint max_iter) {
	int N = x0.size();
	std::vector<real> Ax(N), phi(N);

	real tol_rel = 0.0001;
	real eps = .0001;
	real lambda = 0.95;
	real tol_abs = 10*eps;
	int flag = 2;
	real h = 1e-7;     // Fixed constant used to evaluate the directional detivative

		real alpha = 0.5;// Step reduction parameter for projected Armijo backtracking line search
		real beta = 0.001;// Sufficent decrease parameter for projected Armijo backtracking line search
		real gamma = 1e-28;// Perturbation values used to fix near singular points in derivative
		real rho = eps;// Descent direction test parameter used to test if the Newton direction does a good enough job.

		real err = 1e20;// Current error measure
		std::vector<real> x = x0;// Current iterate
		int iter = 1;// Iteration counter

		while (iter <= max_iter ) {
			ShurProduct(x,Ax);
			std::vector<real> y = Ax + b;

			//--- Test all stopping criteria used ------------------------------------
			phi = phi_lambda(y, x, lambda);// Calculate fischer function
			real old_err = err;
			real err = 0.5*(Dot(phi,phi));// Natural merit function

			if ((abs(err - old_err) / abs(old_err)) < tol_rel ) {     // Relative stopping criteria
				flag = 3;
				break;
			}

			if (err < tol_abs) {     // Absolute stopping criteria
				flag = 4;
				break;
			}

			//--- Solve the Newton system --------------------------------------------
			//--- First we find the Jacobian matrix. We wish to avoid singular points
			//--- in Jacobian of the Fischer function
			//dx = zeros(N,1);// Allocate space for computing the Newton direction

		}

		return current_iteration;
	}

