#include "chrono_parallel/solver/ChSolverMinRes.h"

using namespace chrono;

uint ChSolverMinRes::SolveMinRes(const uint max_iter, const uint size, blaze::DynamicVector<real>& mb, blaze::DynamicVector<real>& ml) {
  mr.resize(size, 0);
  mp.resize(size, 0);
  mz.resize(size, 0);
  mNMr.resize(size, 0);
  mNp.resize(size, 0);
  mMNp.resize(size, 0);
  mtmp.resize(size, 0);

  real grad_diffstep = 0.01;
  double rel_tol = data_container->settings.solver.tolerance;
  double abs_tol = data_container->settings.solver.tolerance;

  real max_element = -1e8;
  for (int i = 0; i < size; i++) {
    max_element = std::max(max_element, mb[i]);
  }
  double rel_tol_b = max_element * rel_tol;
  // ml = x;

  ShurProduct(ml, mr);    // mr = data_container->host_data.D_T *
                          // (data_container->host_data.M_invD * ml);

  mr = mb - mr;

  mr = mr * grad_diffstep + ml;

  Project(mr.data());

  mr = (mr - ml) * (1.0 / grad_diffstep);

  mp = mr;
  mz = mp;

  ShurProduct(mz, mNMr);    // mNMr = data_container->host_data.D_T *
                            // (data_container->host_data.M_invD * mz);
  ShurProduct(mp, mNp);     // mNp = data_container->host_data.D_T *
                            // (data_container->host_data.M_invD * mp);

  for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
    mMNp = mNp;

    double zNMr = (mz, mNMr);
    double MNpNp = (mMNp, mNp);
    if (std::abs(MNpNp) < 10e-30) {
      if (data_container->settings.solver.verbose) {
        std::cout << "Iter=" << current_iteration << " Rayleygh quotient alpha breakdown: " << zNMr << " / " << MNpNp << "\n";
      }
      MNpNp = 10e-12;
    }
    double alpha = zNMr / MNpNp;
    mtmp = mp * alpha;
    ml = ml + mtmp;

    Project(ml.data());
    ShurProduct(ml, mr);    // mr = data_container->host_data.D_T *
                            // (data_container->host_data.M_invD * ml);
    mr = mb - mr;
    mr = mr * grad_diffstep + ml;
    Project(mr.data());
    mr = (mr - ml) * (1.0 / grad_diffstep);
    residual = sqrt((mr, mr));

    if (residual < std::max(rel_tol_b, abs_tol)) {
      if (data_container->settings.solver.verbose) {
        std::cout << "Iter=" << current_iteration << " P(r)-converged!  |P(r)|=" << residual << "\n";
      }
      break;
    }

    mz_old = mz;
    mz = mr;
    mNMr_old = mNMr;

    ShurProduct(mz, mNMr);    // mNMr = data_container->host_data.D_T *
                              // (data_container->host_data.M_invD * mz);

    double numerator = (mz, mNMr - mNMr_old);
    double denominator = (mz_old, mNMr_old);
    double beta = numerator / numerator;

    if (std::abs(denominator) < 10e-30 || std::abs(numerator) < 10e-30) {
      if (data_container->settings.solver.verbose) {
        std::cout << "Iter=" << current_iteration << " Ribiere quotient beta restart: " << numerator << " / " << denominator << "\n";
      }
      beta = 0;
    }

    mtmp = mp * beta;
    mp = mz + mtmp;
    mNp = mNp * beta + mNMr;

    AtIterationEnd(residual, GetObjectiveBlaze(ml, mb), iter_hist.size());
  }

  //	uint N = b.size();
  //	custom_vector<real> v(N, 0), v_hat(size), w(N, 0), w_old, xMR, v_old,
  // Av(size), w_oold;
  //	real beta, c = 1, eta, norm_rMR, norm_r0, c_old = 1, s_old = 0, s = 0,
  // alpha, beta_old, c_oold, s_oold, r1_hat, r1, r2, r3;
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
  //	for (current_iteration = 0; current_iteration < max_iter;
  // current_iteration++) {
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
  //		norm_rMR = norm_rMR * std::abs(s);
  //		eta = -s * eta;
  //		residual = norm_rMR / norm_r0;
  //
  //		real maxdeltalambda = CompRes(b,num_contacts);//NormInf(ms);
  //		AtIterationEnd(residual, maxdeltalambda, current_iteration);
  //
  //
  //		if (residual < tolerance) {break;}
  //	}
  //	Project(x.data());
  return current_iteration;
}
