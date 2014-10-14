#include "ChSolverPDIP.h"
#include <blaze/math/CompressedVector.h>

#include "core/ChFileutils.h"
#include "core/ChStream.h"

#include <algorithm>

using namespace chrono;

real MU = 0.5;

real ChSolverPDIP::Res4(blaze::DynamicVector<real> & gamma, blaze::DynamicVector<real> & tmp) {
  real gdiff = 1e-6;
  SchurComplementProduct(gamma, tmp);
  tmp = tmp + r;
  tmp = gamma - gdiff * (tmp);
  Project(tmp.data());
  tmp = (1.0 / gdiff) * (gamma - tmp);

  return sqrt((double) (tmp, tmp));
}

void ChSolverPDIP::SchurComplementProduct(blaze::DynamicVector<real> & src, blaze::DynamicVector<real> & dst) {
  dst = data_container->host_data.D_T * (data_container->host_data.M_invD * src);
}

void ChSolverPDIP::getConstraintVector(blaze::DynamicVector<real> & src, blaze::DynamicVector<real> & dst, const uint size)
{
#pragma omp parallel for
  for (int i = 0; i < size/3; i++) {
    dst[i] = 0.5*(pow(src[3*i+1],2)+pow(src[3*i+2],2)-pow(MU,2)*pow(src[3*i],2));
    dst[i+size/3] = -src[3*i];
  }
}

void ChSolverPDIP::initializeConstraintGradient(blaze::DynamicVector<real> & src, const uint size)
{
//#pragma omp parallel for
  for (int i = 0; i < size/3; i++) {
    grad_f.append(i, 3*i+0, -pow(MU,2)*src[3*i+0]);
    grad_f.append(i, 3*i+1, src[3*i+1]);
    grad_f.append(i, 3*i+2, src[3*i+2]);
    grad_f.finalize(i);
  }
  for (int i = 0; i < size/3; i++) {
    grad_f.append(i+size/3, 3*i, -1);
    grad_f.finalize(i+size/3);
  }
}

void ChSolverPDIP::updateConstraintGradient(blaze::DynamicVector<real> & src, const uint size)
{
#pragma omp parallel for
  for (int i = 0; i < size/3; i++) {
    grad_f(i, 3*i+0) = -pow(MU,2)*src[3*i+0];
    grad_f(i, 3*i+1) = src[3*i+1];
    grad_f(i, 3*i+2) = src[3*i+2];
  }
#pragma omp parallel for
  for (int i = 0; i < size/3; i++) {
    grad_f(i+size/3, 3*i) = -1;
  }
}

void ChSolverPDIP::initializeNewtonStepMatrix(blaze::DynamicVector<real> & gamma, blaze::DynamicVector<real> & lambda, blaze::DynamicVector<real> & f, const uint size)
{
  for (int i = 0; i < size/3; i++) {
    M_hat.append(3*i+0, 3*i+0, -pow(MU,2)*lambda[i]);
    M_hat.finalize(3*i+0);
    M_hat.append(3*i+1, 3*i+1, lambda[i]);
    M_hat.finalize(3*i+1);
    M_hat.append(3*i+2, 3*i+2, lambda[i]);
    M_hat.finalize(3*i+2);
  }
  updateConstraintGradient(gamma, size);
  B = trans(grad_f);

  for (int i = 0; i < 2*size/3; i++) {
    diaglambda.append(i, i, lambda[i]);
    diaglambda.finalize(i);

    Dinv.append(i, i, -1/f[i]);
    Dinv.finalize(i);
  }
}

void ChSolverPDIP::updateNewtonStepMatrix(blaze::DynamicVector<real> & gamma, blaze::DynamicVector<real> & lambda, blaze::DynamicVector<real> & f, const uint size)
{
#pragma omp parallel for
  for (int i = 0; i < size/3; i++) {
    M_hat(3*i+0, 3*i+0) = -pow(MU,2)*lambda[i];
    M_hat(3*i+1, 3*i+1) = lambda[i];
    M_hat(3*i+2, 3*i+2) = lambda[i];
  }
  updateConstraintGradient(gamma, size);
  B = trans(grad_f);

#pragma omp parallel for
  for (int i = 0; i < 2*size/3; i++) {
    diaglambda(i, i) = lambda[i];

    Dinv(i, i) = -1/f[i];
  }
}

void ChSolverPDIP::updateNewtonStepVector(blaze::DynamicVector<real> & gamma, blaze::DynamicVector<real> & lambda, blaze::DynamicVector<real> & f, real t, const uint size)
{
  updateConstraintGradient(gamma, size);
  r_d = data_container->host_data.D_T * (data_container->host_data.M_invD * gamma) + r + B * lambda;
  r_g = -diaglambda * f - (1/t) * ones;
}

void ChSolverPDIP::conjugateGradient(blaze::DynamicVector<real> & x)
{
  real rsold_cg = 0;
  real rsnew_cg = 0;
  real alpha_cg = 0;
  r_cg = (B*Dinv*r_g-r_d) - (data_container->host_data.D_T*data_container->host_data.M_invD+M_hat+B*Dinv*diaglambda*grad_f) * x;
  p_cg = r_cg;
  rsold_cg = (r_cg, r_cg);

  for(int i=0;i<1e6;i++)
  {
    Ap_cg = (data_container->host_data.D_T*data_container->host_data.M_invD+M_hat+B*Dinv*diaglambda*grad_f) * p_cg;
    alpha_cg = rsold_cg/(p_cg,Ap_cg);
    x = x + alpha_cg * p_cg;
    r_cg = r_cg - alpha_cg * Ap_cg;
    rsnew_cg = (r_cg, r_cg);
    if(sqrt(rsnew_cg)<data_container->settings.solver.tolerance/100)
    {
      return;
    }
    p_cg = r_cg + rsnew_cg/rsold_cg * p_cg;
    rsold_cg = rsnew_cg;
  }
}

uint ChSolverPDIP::SolvePDIP(const uint max_iter, const uint size, custom_vector<real> &b, custom_vector<real> &x)
{
  bool verbose = true;
  custom_vector<real> residualHistory;

  // Initialize scalars
  real mu = 10;
  real alpha = 0.01; // should be [0.01, 0.1]
  real beta = 0.3; // should be [0.3, 0.8]
  real eta_hat = 0;
  real obj = 0;
  real newobj = 0;
  real t = 0;
  real s = 1;
  real s_max = 1;
  real norm_rt = 0;

  // Initialize vectors
  gamma.resize(size);
  gamma_tmp.resize(size);
  r.resize(size);
  f.resize(2*size/3);
  lambda.resize(2*size/3);
  lambda_tmp.resize(2*size/3);
  ones.resize(2*size/3);
  r_d.resize(size);
  r_g.resize(2*size/3);
  delta_gamma.resize(size);
  delta_lambda.resize(size);

  r_cg.resize(size);
  p_cg.resize(size);
  Ap_cg.resize(size);

  // Initialize matrices
  grad_f.reset();
  grad_f.resize(2*size/3, size);
  grad_f.reserve(4*size/3); // there are (4*size/3) nonzero entries

  M_hat.reset();
  M_hat.resize(size, size);
  M_hat.reserve(size); // there are (size) nonzero entries

  B.reset();
  B.resize(size, 2*size/3);
  B.reserve(4*size/3); // there are (4*size/3) nonzero entries

  diaglambda.reset();
  diaglambda.resize(2*size/3, 2*size/3);
  diaglambda.reserve(2*size/3); // there are (size+size/3) nonzero entries

  Dinv.reset();
  Dinv.resize(2*size/3, 2*size/3);
  Dinv.reserve(2*size/3); // there are (size+size/3) nonzero entries

#pragma omp parallel for
  for (int i = 0; i < size; i++) {

    gamma[i] = 0.0;
    if (i % 3 == 0) gamma[i] = 1.0; // provide an initial guess!
    delta_gamma[i] = 0.0;
    //if (i % 3 == 0) delta_gamma[i] = 1.0; // provide an initial guess!

    r[i] = -b[i]; // convert r to a blaze vector
  }

  // (1) f = f(gamma_0)
  getConstraintVector(gamma,f,size);

  // (2) lambda_0 = -1/f
#pragma omp parallel for
  for (int i = 0; i < 2*size/3; i++) {
    ones[i] = 1.0;
    lambda[i] = -1/f[i];
  }
  initializeConstraintGradient(gamma,size);
  initializeNewtonStepMatrix(gamma,lambda,f,size);

  // (3) for k := 0 to N_max
  for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
    // (4) f = f(gamma_k)
    getConstraintVector(gamma,f,size);

    // (5) eta_hat = -f^T * lambda_k
    eta_hat = -(f, lambda);

    // (6) t = mu*m/eta_hat
    t = mu * (2*size/3) / eta_hat;

    // (7) A = A(gamma_k, lambda_k, f)
    updateConstraintGradient(gamma,size);
    updateNewtonStepMatrix(gamma,lambda,f,size);

    // (8) r_t = r_t(gamma_k, lambda_k, t)
    updateNewtonStepVector(gamma,lambda,f,t,size);

    // (9) Solve the linear system A * y = -r_t
    conjugateGradient(delta_gamma);
    delta_lambda = Dinv*(diaglambda*grad_f*delta_gamma-r_g);

    // (10) s_max = sup{s in [0,1]|lambda+s*delta_lambda>=0} = min{1,min{-lambda_i/delta_lambda_i|delta_lambda_i < 0 }}
#pragma omp parallel for
    for (int i = 0; i < 2*size/3; i++) {
      lambda_tmp[i] = -lambda[i]/delta_lambda[i];
      if(delta_lambda[i] > 0) lambda_tmp[i] = 1.0;
    }
    s_max = std::fmin(1.0,min(lambda_tmp));

    // (11) s = 0.99 * s_max
    s = 0.99 * s_max;

    // (12) while max(f(gamma_k + s * delta_gamma) > 0)
    gamma_tmp = gamma+s*delta_gamma;
    getConstraintVector(gamma_tmp,lambda_tmp,size);
    while(max(lambda_tmp)>0)
    {
      // (13) s = beta * s
      s = beta * s;
      gamma_tmp = gamma+s*delta_gamma;
      getConstraintVector(gamma_tmp,lambda_tmp,size);

      // (14) endwhile
    }

    // (15) while norm(r_t(gamma_k + s * delta_gamma, lambda_k + s * delta_lambda),2) > (1-alpha*s)*norm(r_t,2)
    norm_rt = sqrt((r_d, r_d) + (r_g,r_g));
    lambda_tmp = lambda + s*delta_lambda;
    getConstraintVector(gamma_tmp,f,size);
    updateNewtonStepVector(gamma_tmp,lambda_tmp,f,t,size);
    while( sqrt((r_d, r_d) + (r_g,r_g)) > (1-alpha*s)*norm_rt)
    {
      // (16) s = beta * s
      s = beta * s;
      gamma_tmp = gamma+s*delta_gamma;
      lambda_tmp = lambda + s*delta_lambda;
      getConstraintVector(gamma_tmp,f,size);
      updateNewtonStepVector(gamma_tmp,lambda_tmp,f,t,size);

      // (17) endwhile
    }

    // (18) gamma_(k+1) = gamma_k + s * delta_gamma
    gamma = gamma + s*delta_gamma;

    // (19) lambda_(k+1) = lamda_k + s * delta_lambda
    lambda = lambda + s*delta_lambda;

    // (20) r = r(gamma_(k+1))
    residual = Res4(gamma, gamma_tmp);
    residualHistory.push_back(residual);

    SchurComplementProduct(gamma, gamma_tmp);
    objective_value = 0.5*(gamma_tmp,gamma) + (r,gamma);
    AtIterationEnd(residual, objective_value, iter_hist.size());
    if (data_container->settings.solver.tolerance_objective) {
      if (objective_value <= data_container->settings.solver.tolerance) {
        break;
      }
    } else {
      // (21) if r < tau
      if (residual < data_container->settings.solver.tolerance) {
        // (22) break
        break;

        // (23) endif
      }
    }

    // (24) endfor
  }

  // (25) return Value at time step t_(l+1), gamma_(l+1) := gamma_(k+1)
#pragma omp parallel for
   for (int i = 0; i < size; i++) {
      x[i] = gamma[i];
   }
   data_container->system_timer.stop("ChSolverParallel_Solve");

  return current_iteration;
}

void ChSolverPDIP::ComputeImpulses() {
  std::cout << "COMPUTE IMPULSES (BLAZE)" << std::endl;
  blaze::CompressedVector<real> velocities = data_container->host_data.M_invD
      * gamma;

#pragma omp parallel for
  for (int i = 0; i < data_container->num_bodies; i++) {
    real3 vel, omg;

    vel = R3(velocities[i * 6 + 0], velocities[i * 6 + 1],
        velocities[i * 6 + 2]);
    omg = R3(velocities[i * 6 + 3], velocities[i * 6 + 4],
        velocities[i * 6 + 5]);

    data_container->host_data.vel_data[i] += vel;
    data_container->host_data.omg_data[i] += omg;
  }
}

// OUTPUT FUNCTIONS FOR DEBUGGING

int ChSolverPDIP::OutputState(std::string stateId) {
//  const std::string state_dir = "../DIRECTSHEAR_DVI/STATES/";
//  ChFileutils::MakeDirectory(state_dir.c_str());
//
//  // output D_T
//  const std::string filename = state_dir + "D_T" + stateId + ".dat";
//  OutputBlazeMatrix(data_container->host_data.D_T, filename);
//
//  // output MinvD
//  filename = state_dir + "MinvD" + stateId + ".dat";
//  OutputBlazeMatrix(data_container->host_data.M_invD, filename);

//  // output Minvk
//  filename = state_dir + "Minvk" + stateId + ".dat";
//  OutputBlazeVector(data_container->host_data.M_invk, filename);
//
//  // output b
//  filename = state_dir + "b" + stateId + ".dat";
//  OutputBlazeVector(data_container->host_data.b, filename);
//
//  // output k
//  filename = state_dir + "k" + stateId + ".dat";
//  OutputBlazeVector(data_container->host_data.k, filename);

//  // output gamma_hat
//  filename = state_dir + "gamma_hat" + stateId + ".dat";
//  OutputBlazeVector(gamma_hat, filename);
//
//  // output gammaNew
//  filename = state_dir + "gammaNew" + stateId + ".dat";
//  OutputBlazeVector(gammaNew, filename);
//
//  // output g
//  filename = state_dir + "g" + stateId + ".dat";
//  OutputBlazeVector(g, filename);
//
//  // output y
//  filename = state_dir + "y" + stateId + ".dat";
//  OutputBlazeVector(y, filename);
//
//  // output gamma
//  filename = state_dir + "gamma" + stateId + ".dat";
//  OutputBlazeVector(gamma, filename);
//
//  // output yNew
//  filename = state_dir + "yNew" + stateId + ".dat";
//  OutputBlazeVector(yNew, filename);
//
//  // output r
//  filename = state_dir + "r" + stateId + ".dat";
//  OutputBlazeVector(r, filename);
//
//  // output tmp
//  filename = state_dir + "tmp" + stateId + ".dat";
//  OutputBlazeVector(tmp, filename);

  return 0;
}

int ChSolverPDIP::OutputBlazeVector(blaze::DynamicVector<real> src,
    std::string filename) {

  const char* numformat = "%.16g";
  ChStreamOutAsciiFile stream(filename.c_str());
  stream.SetNumFormat(numformat);

  for (int i = 0; i < src.size(); i++)
    stream << src[i] << "\n";
  return 0;
}

int ChSolverPDIP::OutputBlazeMatrix(blaze::CompressedMatrix<real> src,
    std::string filename) {

  const char* numformat = "%.16g";
  ChStreamOutAsciiFile stream(filename.c_str());
  stream.SetNumFormat(numformat);

  for (int i = 0; i < src.rows(); ++i) {
    for (CompressedMatrix<real>::Iterator it = src.begin(i); it != src.end(i);
        ++it) {
      stream << i << " " << it->index() << " " << it->value() << "\n";
    }
  }

  return 0;
}
