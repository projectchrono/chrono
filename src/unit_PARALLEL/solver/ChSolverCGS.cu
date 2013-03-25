#include "ChSolverCGS.h"
using namespace chrono;


ChSolverCGS::ChSolverCGS() {
}

void ChSolverCGS::Solve(real step, gpu_container &gpu_data_) {
    gpu_data = &gpu_data_;
    step_size = step;
    Setup();
    if (number_of_constraints > 0) {
        ComputeRHS();
        SolveCGS(gpu_data->device_gam_data, rhs, 100);
        ComputeImpulses();
        gpu_data->device_vel_data += gpu_data->device_QXYZ_data;
        gpu_data->device_omg_data += gpu_data->device_QUVW_data;
    }
}

uint ChSolverCGS::SolveCGS(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter) {
	real rho_1, rho_2, alpha, beta;
	custom_vector<real> r = b - ShurProduct(x);
		custom_vector<real> p = r, phat, q = r, qhat, vhat, u = r, uhat;
		real normb = Norm(b);
		custom_vector<real> rtilde = r;

		if (normb == 0.0) {normb = 1;}

		if ((Norm(r) / normb) <= tolerance) {return 0;}

		for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
			rho_1 = Dot(rtilde, r);

			if (rho_1 == 0) {break;}

			if (current_iteration > 0) {
				beta = rho_1 / rho_2;
				#pragma omp parallel for schedule(guided)

				for (int i = 0; i < x.size(); i++) {
					u[i] = r[i] + beta * q[i]; //u = r + beta * q;
					p[i] = u[i] + beta * (q[i] + beta * p[i]); //p = u + beta * (q + beta * p);
				}
			}

			phat = p;
			vhat = ShurProduct(phat);
			alpha = rho_1 / Dot(rtilde, vhat);
			SEAXPY(-alpha, vhat, u, q); //q = u - alpha * vhat;
			uhat = (u + q);
			SEAXPY(alpha, uhat, x, x);  //x = x + alpha * uhat;
			qhat = ShurProduct(uhat);
			SEAXPY(-alpha, qhat, r, r); //r = r - alpha * qhat;
			rho_2 = rho_1;
			residual = (Norm(r) / normb);

			if (residual < tolerance) {
				break;
			}
		}
		Project(x);
		return current_iteration;
}
