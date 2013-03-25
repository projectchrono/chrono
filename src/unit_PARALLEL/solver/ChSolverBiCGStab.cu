#include "ChSolverBiCGStab.h"
using namespace chrono;


ChSolverBiCGStab::ChSolverBiCGStab() {
}

void ChSolverBiCGStab::Solve(real step, gpu_container &gpu_data_) {
    gpu_data = &gpu_data_;
    step_size = step;
    Setup();
    if (number_of_constraints > 0) {
        ComputeRHS();
        SolveBiCGStab(gpu_data->device_gam_data, rhs, 100);
        ComputeImpulses();
        gpu_data->device_vel_data += gpu_data->device_QXYZ_data;
        gpu_data->device_omg_data += gpu_data->device_QUVW_data;
    }
}

uint ChSolverBiCGStab::SolveBiCGStab(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter) {
	real rho_1, rho_2, alpha = 1, beta, omega = 1;
	custom_vector<real> p, r, phat, s, shat, t, v;
		real normb = Norm(b);
		p = r = b - ShurProduct(x);
		custom_vector<real> rtilde = r;

		if (normb == 0.0) {normb = 1;}

		if ((residual = Norm(r) / normb) <= tolerance) {return 0;}

		for (current_iteration = 0; current_iteration <= max_iter; current_iteration++) {
			rho_1 = Dot(rtilde, r);

			if (rho_1 == 0) {break;}

			if (current_iteration > 0) {
				beta = (rho_1 / rho_2) * (alpha / omega);
				p = r + beta * (p - omega * v);
			}

			phat = p;
			v = ShurProduct(phat);
			alpha = rho_1 / Dot(rtilde, v);
			s = r - alpha * v;//SEAXPY(-alpha,v,r,s);//
			residual = Norm(s) / normb;

			if (residual < tolerance) {

				SEAXPY(alpha, phat, x, x); //x = x + alpha * phat;
				break;
			}

			shat = s;
			t = ShurProduct(shat);
			omega = Dot(t, s) / Dot(t, t);
			SEAXPY(alpha, phat, x, x);
			SEAXPY(omega, shat, x, x); //x = x + alpha * phat + omega * shat;
			SEAXPY(-omega, t, s, r); //r = s - omega * t;
			rho_2 = rho_1;
			residual = Norm(r) / normb;


			if (residual < tolerance || omega == 0) {break;}
		}

		return current_iteration;
}
