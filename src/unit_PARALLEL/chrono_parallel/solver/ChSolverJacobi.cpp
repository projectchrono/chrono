#include "chrono_parallel/solver/ChSolverJacobi.h"
#include <blaze/math/SparseRow.h>
#include <blaze/math/CompressedVector.h>
using namespace chrono;

uint ChSolverJacobi::SolveJacobi(const uint max_iter, const uint size, blaze::DynamicVector<real>& mb, blaze::DynamicVector<real>& ml) {
  diagonal.resize(size, false);
  ml_old = ml;
  data_container->host_data.Nshur = data_container->host_data.D_T * data_container->host_data.M_invD;

  for (size_t i = 0; i < size; ++i) {
    const real tmp(data_container->host_data.Nshur(i, i));
    diagonal[i] = real(1) / tmp;
  }

  Project(ml.data());

  for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
    const size_t N(size / 3);
    size_t j;
    real omega = 1.0;
#pragma omp parallel for
    for (int i = 0; i < data_container->num_contacts; ++i) {
      j = i * 3;
      real Dinv = 1.0 / (diagonal[j + 0] + diagonal[j + 1] + diagonal[j + 2]);
      real E1 = data_container->host_data.E[j + 0];
      real E2 = data_container->host_data.E[j + 1];
      real E3 = data_container->host_data.E[j + 2];
      ml[j + 0] = ml[j + 0] - omega * Dinv * ((row(data_container->host_data.Nshur, j + 0), ml_old) + E1 * ml_old[j + 0] - mb[j + 0]);
      ml[j + 1] = ml[j + 1] - omega * Dinv * ((row(data_container->host_data.Nshur, j + 1), ml_old) + E2 * ml_old[j + 1] - mb[j + 1]);
      ml[j + 2] = ml[j + 2] - omega * Dinv * ((row(data_container->host_data.Nshur, j + 2), ml_old) + E3 * ml_old[j + 2] - mb[j + 2]);

      Project_Single(i, ml.data());
    }
    Project(ml.data());
    ml_old = ml;
    residual = Res4Blaze(ml, mb);
    AtIterationEnd(residual, GetObjectiveBlaze(ml, mb), iter_hist.size());
  }

  return current_iteration;
}
