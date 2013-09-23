#include "ChSolverGPU.cuh"
using namespace chrono;

uint ChSolverGPU::SolveMinRes(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter) {
	uint N = b.size();
		custom_vector<real> v(N, 0), v_hat(x.size()), w(N, 0), w_old, xMR, v_old, Av(x.size()), w_oold;
		real beta, c = 1, eta, norm_rMR, norm_r0, c_old = 1, s_old = 0, s = 0, alpha, beta_old, c_oold, s_oold, r1_hat, r1, r2, r3;
		ShurProduct(x,v_hat);
		v_hat = b - v_hat;
		beta = Norm(v_hat);
		w_old = w;
		eta = beta;
		xMR = x;
		norm_rMR = beta;
		norm_r0 = beta;

		if (beta == 0 || norm_rMR / norm_r0 < tolerance) {return 0;}

		for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
			//// Lanczos
			v_old = v;
			v = 1.0 / beta * v_hat;
			ShurProduct(v,Av);
			alpha = Dot(v, Av);
			v_hat = Av - alpha * v - beta * v_old;
			beta_old = beta;
			beta = Norm(v_hat);
			//// QR factorization
			c_oold = c_old;
			c_old = c;
			s_oold = s_old;
			s_old = s;
			r1_hat = c_old * alpha - c_oold * s_old * beta_old;
			r1 = 1 / sqrt(r1_hat * r1_hat + beta * beta);
			r2 = s_old * alpha + c_oold * c_old * beta_old;
			r3 = s_oold * beta_old;
			//// Givens Rotation
			c = r1_hat * r1;
			s = beta * r1;
			//// update
			w_oold = w_old;
			w_old = w;
			w = r1 * (v - r3 * w_oold - r2 * w_old);
			x = x + c * eta * w;
			norm_rMR = norm_rMR * abs(s);
			eta = -s * eta;
			residual = norm_rMR / norm_r0;

			real maxdeltalambda = CompRes(b,number_of_rigid_rigid);     //NormInf(ms);
			AtIterationEnd(residual, maxdeltalambda, current_iteration);


			if (residual < tolerance) {break;}
		}
		Project(x);
		return current_iteration;

}
