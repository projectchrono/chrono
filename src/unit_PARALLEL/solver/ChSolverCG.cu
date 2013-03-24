#include "ChSolverCG.h"
using namespace chrono;


ChSolverCG::ChSolverCG() {
	tolerance = 1e-6;

}

void ChSolverCG::Solve(real step, gpu_container &gpu_data_) {

	gpu_data = &gpu_data_;
	step_size = step;
	number_of_constraints = gpu_data->number_of_contacts * 3 + gpu_data->number_of_bilaterals;
	if (number_of_constraints > 0) {
		number_of_contacts = gpu_data->number_of_contacts;
		temp_bids.resize(number_of_constraints);
		correction.resize(number_of_constraints);
		rhs.resize(number_of_constraints);
		gpu_data->device_gam_data.resize((number_of_constraints));
		Thrust_Fill(gpu_data->device_gam_data, 0);
		Thrust_Fill(correction, 0);
		thrust::copy_n(gpu_data->device_dpth_data.begin(), gpu_data->number_of_contacts, correction.begin() + gpu_data->number_of_contacts * 0);

		AX.resize(number_of_constraints);
		thrust::copy_n(gpu_data->device_bids_data.begin(), gpu_data->number_of_contacts, temp_bids.begin() + gpu_data->number_of_contacts * 0);
		thrust::copy_n(gpu_data->device_bids_data.begin(), gpu_data->number_of_contacts, temp_bids.begin() + gpu_data->number_of_contacts * 1);
		thrust::copy_n(gpu_data->device_bids_data.begin(), gpu_data->number_of_contacts, temp_bids.begin() + gpu_data->number_of_contacts * 2);

#ifdef SIM_ENABLE_GPU_MODE
		COPY_TO_CONST_MEM(number_of_constraints);
		COPY_TO_CONST_MEM(number_of_contacts);
#else
		host_RHS(
				temp_bids.data(),
				gpu_data->device_JXYZA_data.data(),
				gpu_data->device_JXYZB_data.data(),
				gpu_data->device_JUVWA_data.data(),
				gpu_data->device_JUVWB_data.data(),
				gpu_data->device_vel_data.data(),
				gpu_data->device_omg_data.data(),
				correction.data(),
				step_size,
				rhs.data());
#endif

		SolveSD(gpu_data->device_gam_data, rhs, 100);

		Thrust_Fill(gpu_data->device_QXYZ_data, R3(0));
		Thrust_Fill(gpu_data->device_QUVW_data, R3(0));
		host_shurA(
				gpu_data->device_JXYZA_data.data(),
				gpu_data->device_JXYZB_data.data(),
				gpu_data->device_JUVWA_data.data(),
				gpu_data->device_JUVWB_data.data(),
				gpu_data->device_gam_data.data(),
				temp_bids.data(),
				gpu_data->device_mass_data.data(),
				gpu_data->device_inr_data.data(),
				gpu_data->device_active_data.data(),
				gpu_data->device_QXYZ_data.data(),
				gpu_data->device_QUVW_data.data());
		gpu_data->device_QXYZ_data *= gpu_data->device_mass_data;
		gpu_data->device_QUVW_data *= gpu_data->device_inr_data;
		gpu_data->device_vel_data += gpu_data->device_QXYZ_data;
		gpu_data->device_omg_data += gpu_data->device_QUVW_data;
	}
}

uint ChSolverCG::SolveCG(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter) {
	custom_vector<real> r(x.size()), p, Ap;
	real rsold, alpha, rsnew = 0, normb = Norm(b);
	if (normb == 0.0) {
		normb = 1;
	}
	p = r = b - ShurProduct(x);
	rsold = Dot(r, r);
	normb = 1.0 / normb;
	if (sqrt(rsold) * normb <= tolerance) {
		return 0;
	}
	for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
		Ap = ShurProduct(p);
		alpha = rsold / Dot(p, Ap);
		rsnew = 0;
#pragma omp parallel for reduction(+:rsnew)
		for (int i = 0; i < x.size(); i++) {
			x[i] = x[i] + alpha * p[i];
			r[i] = r[i] - alpha * Ap[i];
			rsnew += r[i] * r[i];
		}
		residual = sqrtf(rsnew) * normb;

		if (residual < tolerance) {
			break;
		}
		SEAXPY(rsnew / rsold, p, r, p); //p = r + rsnew / rsold * p;
		rsold = rsnew;
	}
	host_Project(x.data(),
			gpu_data->device_fric_data.data(),
			temp_bids.data());
	return current_iteration;
}
uint ChSolverCG::SolveGD(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter) {
	real eps=step_size;
	custom_vector<real> r;
	r = b - ShurProduct(x);
	real resold = 1, resnew, normb = Norm(b);
	if (normb == 0.0) {normb = 1;};
	for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
		SEAXPY(eps, r, x, x); //x = x + eps *r;
		r = b - ShurProduct(x);
		resnew = Norm(x);
		residual = abs(resnew - resold);
		if (residual < tolerance) {break;}

		resold = resnew;
	}
	host_Project(x.data(),
			gpu_data->device_fric_data.data(),
			temp_bids.data());
	return current_iteration;

}
uint ChSolverCG::SolveSD(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter) {
	custom_vector<real> r;
	r = b - ShurProduct(x);
	real resold = 1, resnew, normb = Norm(b), alpha;
	if (normb == 0.0) {normb = 1;}
	for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
		alpha = Dot(r, r) /Dot(r, ShurProduct(r));
		SEAXPY(alpha, r, x, x); //x = x + eps *r;
		r = b - ShurProduct(x);
		resnew = Norm(x);
		residual = abs(resnew - resold);
		if (residual < tolerance) {break;}
		resold = resnew;
	}
	host_Project(x.data(),
			gpu_data->device_fric_data.data(),
			temp_bids.data());

	return current_iteration;

}
