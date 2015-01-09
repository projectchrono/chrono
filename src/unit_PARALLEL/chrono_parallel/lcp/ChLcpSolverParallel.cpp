#include "chrono_parallel/lcp/ChLcpSolverParallel.h"
#include "chrono_parallel/math/ChThrustLinearAlgebra.h"

using namespace chrono;

ChLcpSolverParallel::ChLcpSolverParallel() {
  tolerance = 1e-7;
  record_violation_history = true;
  warm_start = false;
  residual = 0;
  data_container = 0;
}

void ChLcpSolverParallel::ComputeMassMatrix() {
  uint& num_bodies = data_container->num_bodies;
  custom_vector<real>& inv_mass_data = data_container->host_data.inv_mass_data;
  custom_vector<M33>& inr_data = data_container->host_data.inr_data;

  CompressedMatrix<real>& M_inv = data_container->host_data.M_inv;
  clear(M_inv);

  // Each object has 3 mass entries and 9 inertia entries
  M_inv.reserve(num_bodies * 12);
  // The mass matrix is square and each rigid body has 6 DOF
  M_inv.resize(num_bodies * 6, num_bodies * 6);

  for (int i = 0; i < num_bodies; i++) {
    if (data_container->host_data.active_data[i]) {
      M_inv.append(i * 6 + 0, i * 6 + 0, inv_mass_data[i]);
      M_inv.finalize(i * 6 + 0);
      M_inv.append(i * 6 + 1, i * 6 + 1, inv_mass_data[i]);
      M_inv.finalize(i * 6 + 1);
      M_inv.append(i * 6 + 2, i * 6 + 2, inv_mass_data[i]);
      M_inv.finalize(i * 6 + 2);

      M_inv.append(i * 6 + 3, i * 6 + 3, inr_data[i].U.x);
      M_inv.append(i * 6 + 3, i * 6 + 4, inr_data[i].V.x);
      M_inv.append(i * 6 + 3, i * 6 + 5, inr_data[i].W.x);
      M_inv.finalize(i * 6 + 3);
      M_inv.append(i * 6 + 4, i * 6 + 3, inr_data[i].U.y);
      M_inv.append(i * 6 + 4, i * 6 + 4, inr_data[i].V.y);
      M_inv.append(i * 6 + 4, i * 6 + 5, inr_data[i].W.y);
      M_inv.finalize(i * 6 + 4);
      M_inv.append(i * 6 + 5, i * 6 + 3, inr_data[i].U.z);
      M_inv.append(i * 6 + 5, i * 6 + 4, inr_data[i].V.z);
      M_inv.append(i * 6 + 5, i * 6 + 5, inr_data[i].W.z);
      M_inv.finalize(i * 6 + 5);
    }
  }

  data_container->host_data.M_invk = data_container->host_data.v + M_inv * data_container->host_data.hf;
}

void ChLcpSolverParallel::ComputeImpulses() {
  if (data_container->num_constraints > 0) {
	  //Compute new velocity based on the lagrange multipliers
    data_container->host_data.v = data_container->host_data.M_invk + data_container->host_data.M_invD * data_container->host_data.gamma;
  } else {
	  //When there are no constraints we need to still apply gravity!
    data_container->host_data.v = data_container->host_data.M_invk;
  }
}
