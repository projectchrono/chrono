#include "chrono_parallel/solver/ChSolverAPGDREF.h"
#include <blaze/math/CompressedVector.h>

using namespace chrono;

real ChSolverAPGDREF::Res4(blaze::DynamicVector<real> & gamma,
    blaze::DynamicVector<real> & tmp) {

  real gdiff = 1.0/pow(num_constraints,2.0);
  SchurComplementProduct(gamma, tmp);
  tmp = tmp + r;
  tmp = gamma - gdiff * (tmp);
  Project(tmp.data());
  tmp = (1.0 / gdiff) * (gamma - tmp);

  return sqrt((double) (tmp, tmp));
}

void ChSolverAPGDREF::SchurComplementProduct(blaze::DynamicVector<real> & src,
    blaze::DynamicVector<real> & dst) {
  dst = data_container->host_data.D_T
      * (data_container->host_data.M_invD * src);
}

uint ChSolverAPGDREF::SolveAPGDREF(const uint max_iter, const uint size,
		 const blaze::DynamicVector<real>& b,
		 blaze::DynamicVector<real>& x) {
  bool verbose = false;
  bool useWarmStarting = true;
  if(verbose) std::cout << "Number of constraints: " << size << "\nNumber of variables  : " << data_container->num_bodies << std::endl;

  real L, t;
  real theta;
  real thetaNew;
  real Beta;
  real obj1, obj2;

  gamma_hat.resize(size);
  gammaNew.resize(size);
  g.resize(size);
  y.resize(size);
  gamma.resize(size);
  yNew.resize(size);
  r.resize(size);
  tmp.resize(size);

  residual = 10e30;

#pragma omp parallel for
  for (int i = 0; i < size; i++) {
    // (1) gamma_0 = zeros(nc,1)
    if(!useWarmStarting) gamma[i] = 0;

    // (2) gamma_hat_0 = ones(nc,1)
    gamma_hat[i] = 1.0;

    // (3) y_0 = gamma_0
    y[i] = gamma[i];
    r[i] = -b[i]; // convert r to a blaze vector //TODO: WHY DO I NEED A MINUS SIGN?
  }

  // (4) theta_0 = 1
  theta = 1.0;

  thetaNew = theta;
  Beta = 0.0;
  obj1 = 0.0, obj2 = 0.0;

  // (5) L_k = norm(N * (gamma_0 - gamma_hat_0)) / norm(gamma_0 - gamma_hat_0)
  tmp = gamma - gamma_hat;
  L = sqrt((double) (tmp, tmp));
  SchurComplementProduct(tmp, tmp);
  L = sqrt((double) (tmp, tmp)) / L;

  // (6) t_k = 1 / L_k
  t = 1.0 / L;

  // (7) for k := 0 to N_max
  for (current_iteration = 0; current_iteration < max_iter;
      current_iteration++) {
    // (8) g = N * y_k - r
    SchurComplementProduct(y, g);
    g = g + r;

    // (9) gamma_(k+1) = ProjectionOperator(y_k - t_k * g)
    gammaNew = y - t * g;
    Project(gammaNew.data());

    // (10) while 0.5 * gamma_(k+1)' * N * gamma_(k+1) - gamma_(k+1)' * r >= 0.5 * y_k' * N * y_k - y_k' * r + g' * (gamma_(k+1) - y_k) + 0.5 * L_k * norm(gamma_(k+1) - y_k)^2
    SchurComplementProduct(gammaNew, tmp); // Here tmp is equal to N*gammaNew;
    obj1 = 0.5 * (gammaNew, tmp) + (gammaNew, r);
    SchurComplementProduct(y, tmp); // Here tmp is equal to N*y;
    obj2 = 0.5 * (y, tmp) + (y, r);
    tmp = gammaNew - y; // Here tmp is equal to gammaNew - y
    obj2 = obj2 + (g, tmp) + 0.5 * L * (tmp, tmp);

    while (obj1 >= obj2) {
      // (11) L_k = 2 * L_k
      L = 2.0 * L;

      // (12) t_k = 1 / L_k
      t = 1.0 / L;

      // (13) gamma_(k+1) = ProjectionOperator(y_k - t_k * g)
      gammaNew = y - t * g;
      Project(gammaNew.data());

      // Update the components of the while condition
      SchurComplementProduct(gammaNew, tmp); // Here tmp is equal to N*gammaNew;
      obj1 = 0.5 * (gammaNew, tmp) + (gammaNew, r);
      SchurComplementProduct(y, tmp); // Here tmp is equal to N*y;
      obj2 = 0.5 * (y, tmp) + (y, r);
      tmp = gammaNew - y; // Here tmp is equal to gammaNew - y
      obj2 = obj2 + (g, tmp) + 0.5 * L * (tmp, tmp);

      // (14) endwhile
    }

    // (15) theta_(k+1) = (-theta_k^2 + theta_k * sqrt(theta_k^2 + 4)) / 2;
    thetaNew = (-pow(theta, 2.0) + theta * sqrt(pow(theta, 2.0) + 4.0)) / 2.0;

    // (16) Beta_(k+1) = theta_k * (1 - theta_k) / (theta_k^2 + theta_(k+1))
    Beta = theta * (1.0 - theta) / (pow(theta, 2) + thetaNew);

    // (17) y_(k+1) = gamma_(k+1) + Beta_(k+1) * (gamma_(k+1) - gamma_k)
    yNew = gammaNew + Beta * (gammaNew - gamma);

    // (18) r = r(gamma_(k+1))
    real res = Res4(gammaNew, tmp);

    // (19) if r < epsilon_min
    if (res < residual) {
      // (20) r_min = r
      residual = res;

      // (21) gamma_hat = gamma_(k+1)
      gamma_hat = gammaNew;

      // (22) endif
    }

    // (23) if r < Tau
    if (residual < tol_speed) {
      // (24) break
      break;

      // (25) endif
    }

    // (26) if g' * (gamma_(k+1) - gamma_k) > 0
    if ((g, gammaNew - gamma) > 0) {
      // (27) y_(k+1) = gamma_(k+1)
      yNew = gammaNew;

      // (28) theta_(k+1) = 1
      thetaNew = 1.0;

      // (29) endif
    }

    // (30) L_k = 0.9 * L_k
    L = 0.9 * L;

    // (31) t_k = 1 / L_k
    t = 1.0 / L;

    // Update iterates
    theta = thetaNew;
    gamma = gammaNew;
    y = yNew;

    // perform some tasks at the end of the iteration
    AtIterationEnd(residual, objective_value, iter_hist.size());

    // (32) endfor
  }
  if(verbose) std::cout << "Residual: " << residual << ", Iter: " << current_iteration << std::endl;

  // (33) return Value at time step t_(l+1), gamma_(l+1) := gamma_hat
  if(useWarmStarting) gamma = gamma_hat;
#pragma omp parallel for
  for (int i = 0; i < size; i++) {
    x[i] = gamma_hat[i];
  }

  return current_iteration;
}

