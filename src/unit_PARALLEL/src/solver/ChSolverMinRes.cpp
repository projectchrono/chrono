#include "ChSolverParallel.h"
using namespace chrono;

uint ChSolverParallel::SolveMinRes(std::vector<real> &x, const std::vector<real> &mb, const uint max_iter) {
	uint N = mb.size();
	bool verbose = true;
	std::vector<real> mr(N, 0), ml(N,0), mp(N,0), mz(N,0), mNMr(N,0), mNp(N,0), mMNp(N,0), mtmp(N,0);
	std::vector<real> mz_old;
	std::vector<real> mNMr_old;
	real grad_diffstep = 0.01;
	double rel_tol = tolerance;
	double abs_tol = tolerance;

	double rel_tol_b = NormInf(mb) * rel_tol;
	//ml = x;

			ShurProduct(ml,mr);

		mr = mb-mr;

		mr=mr*grad_diffstep+ml;

			Project(mr.data());
		
		mr=(mr-ml)*(1.0/grad_diffstep);

		mp=mr;
		mz=mp;

			ShurProduct(mz,mNMr);
			ShurProduct(mp,mNp);
		
		for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
			mMNp = mNp;

			double zNMr = Dot(mz,mNMr);
			double MNpNp = Dot(mMNp, mNp);
			if (fabs(MNpNp)<10e-30){
				if (verbose) {cout << "Iter=" << current_iteration << " Rayleygh quotient alpha breakdown: " << zNMr << " / " << MNpNp << "\n";}
				MNpNp=10e-12;
			}
			double alpha = zNMr/MNpNp;
			mtmp = mp*alpha;
			ml=ml+mtmp;
			double maxdeltalambda = Norm(mtmp);

			Project(ml.data());
			ShurProduct(ml,mr);
		
		mr = mb-mr;

		mr=mr*grad_diffstep+ml;

			Project(mr.data());
		

		mr=(mr-ml)*(1.0/grad_diffstep);

		double r_proj_resid = Norm(mr);

		if (r_proj_resid < max(rel_tol_b, abs_tol) ) {
			if (verbose) {
				cout << "Iter=" << current_iteration << " P(r)-converged!  |P(r)|=" << r_proj_resid << "\n";
			}
			break;
		}

		mz_old = mz;
		mz = mr;
		mNMr_old = mNMr;

			ShurProduct(mz,mNMr);
		
		double numerator = Dot(mz,mNMr-mNMr_old);
		double denominator = Dot(mz_old,mNMr_old);
		double beta =numerator /numerator;

		if (fabs(denominator)<10e-30 || fabs(numerator)<10e-30) {
			if (verbose) {
				cout << "Iter=" << current_iteration << " Ribiere quotient beta restart: " << numerator << " / " << denominator << "\n";
			}
			beta =0;
		}

		mtmp = mp*beta;
		mp = mz+mtmp;
		mNp = mNp*beta+mNMr;

		AtIterationEnd(r_proj_resid, maxdeltalambda, current_iteration);

	}
	x=ml;

	//	uint N = b.size();
//	custom_vector<real> v(N, 0), v_hat(x.size()), w(N, 0), w_old, xMR, v_old, Av(x.size()), w_oold;
//	real beta, c = 1, eta, norm_rMR, norm_r0, c_old = 1, s_old = 0, s = 0, alpha, beta_old, c_oold, s_oold, r1_hat, r1, r2, r3;
//	ShurProduct(x,v_hat);
//	v_hat = b - v_hat;
//	beta = Norm(v_hat);
//	w_old = w;
//	eta = beta;
//	xMR = x;
//	norm_rMR = beta;
//	norm_r0 = beta;
//
//	if (beta == 0 || norm_rMR / norm_r0 < tolerance) {return 0;}
//
//	for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
//		//// Lanczos
//		v_old = v;
//		v = 1.0 / beta * v_hat;
//		ShurProduct(v,Av);
//		alpha = Dot(v, Av);
//		v_hat = Av - alpha * v - beta * v_old;
//		beta_old = beta;
//		beta = Norm(v_hat);
//		//// QR factorization
//		c_oold = c_old;
//		c_old = c;
//		s_oold = s_old;
//		s_old = s;
//		r1_hat = c_old * alpha - c_oold * s_old * beta_old;
//		r1 = 1 / sqrt(r1_hat * r1_hat + beta * beta);
//		r2 = s_old * alpha + c_oold * c_old * beta_old;
//		r3 = s_oold * beta_old;
//		//// Givens Rotation
//		c = r1_hat * r1;
//		s = beta * r1;
//		//// update
//		w_oold = w_old;
//		w_old = w;
//		w = r1 * (v - r3 * w_oold - r2 * w_old);
//		x = x + c * eta * w;
//		norm_rMR = norm_rMR * abs(s);
//		eta = -s * eta;
//		residual = norm_rMR / norm_r0;
//
//		real maxdeltalambda = CompRes(b,number_of_rigid_rigid);//NormInf(ms);
//		AtIterationEnd(residual, maxdeltalambda, current_iteration);
//
//
//		if (residual < tolerance) {break;}
//	}
//	Project(x.data());
		return current_iteration;

	}
