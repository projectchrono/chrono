#include "chrono_parallel/solver/ChSolverPGS.h"
#include <blaze/math/SparseRow.h>
#include <blaze/math/CompressedVector.h>
using namespace chrono;

uint ChSolverPGS::SolvePGS(const uint max_iter, const uint size, blaze::DynamicVector<real>& mb, blaze::DynamicVector<real>& ml) {
  real& residual = data_container->measures.solver.residual;
  real& objective_value = data_container->measures.solver.objective_value;
  custom_vector<real>& iter_hist = data_container->measures.solver.iter_hist;

  real rmax = 0, flimit, aux;
  diagonal.resize(size, false);

  for (size_t i = 0; i < size; ++i) {
    const real tmp(data_container->host_data.Nshur(i, i));
    diagonal[i] = real(1) / tmp;
  }

  Project(ml.data());

  for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
    const size_t N(size / 3);
    size_t j;
    real omega = 1.0;

    for (size_t i = 0; i < data_container->num_contacts; ++i) {
      j = i * 3;
      real Dinv = 1.0 / (diagonal[j + 0] + diagonal[j + 1] + diagonal[j + 2]);

      ml[j + 0] = ml[j + 0] - omega * Dinv * ((row(data_container->host_data.Nshur, j + 0), ml) - mb[j + 0]);
      ml[j + 1] = ml[j + 1] - omega * Dinv * ((row(data_container->host_data.Nshur, j + 1), ml) - mb[j + 1]);
      ml[j + 2] = ml[j + 2] - omega * Dinv * ((row(data_container->host_data.Nshur, j + 2), ml) - mb[j + 2]);

      Project_Single(i, ml.data());
    }
    residual = Res4Blaze(ml, mb);
    objective_value = GetObjective(ml, mb);
    AtIterationEnd(residual, objective_value);
  }

  return current_iteration;
}
