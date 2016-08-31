#include "chrono_parallel/solver/ChSolverParallelJacobi.h"
#if BLAZE_MAJOR_VERSION == 2
#include <blaze/math/SparseRow.h>
#endif
#include <blaze/math/CompressedMatrix.h>

using namespace chrono;

uint ChSolverParallelJacobi::SolveJacobi(const uint max_iter,
                                         const uint size,
                                         DynamicVector<real>& mb,
                                         DynamicVector<real>& ml) {
  real& residual = data_manager->measures.solver.residual;
  real& objective_value = data_manager->measures.solver.objective_value;

  uint num_contacts = data_manager->num_rigid_contacts;
  diagonal.resize(size, false);
  ml_old = ml;
  CompressedMatrix<real> Nshur_n = data_manager->host_data.D_n_T * data_manager->host_data.M_invD_n;
  CompressedMatrix<real> Nshur_t = data_manager->host_data.D_t_T * data_manager->host_data.M_invD_t;

  for (size_t i = 0; i < num_contacts; ++i) {
    diagonal[i * 1 + 0] = Nshur_n(i, i);
    diagonal[num_contacts + i * 2 + 0] = Nshur_t(i * 2 + 0, i * 2 + 0);
    diagonal[num_contacts + i * 2 + 1] = Nshur_t(i * 2 + 1, i * 2 + 1);
  }

  Project(ml.data());
  //
  for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
    real omega = 1.0;
#pragma omp parallel for
    for (int i = 0; i < num_contacts; ++i) {
      int a = i * 1 + 0;
      int b = num_contacts + i * 2 + 0;
      int c = num_contacts + i * 2 + 1;

      real Dinv = 3.0 / (diagonal[a] + diagonal[b] + diagonal[c]);
      real E1 = data_manager->host_data.E[a];
      real E2 = data_manager->host_data.E[b];
      real E3 = data_manager->host_data.E[c];
      ml[a] = ml[a] -
              omega * Dinv *
                  ((row(Nshur_n, i * 1 + 0), blaze::subvector(ml_old, 0, 1 * num_contacts)) + E1 * ml_old[a] - mb[a]);
      ml[b] = ml[b] -
              omega * Dinv * ((row(Nshur_t, i * 2 + 0), blaze::subvector(ml_old, num_contacts, 2 * num_contacts)) +
                              E2 * ml_old[b] - mb[b]);
      ml[c] = ml[c] -
              omega * Dinv * ((row(Nshur_t, i * 2 + 1), blaze::subvector(ml_old, num_contacts, 2 * num_contacts)) +
                              E3 * ml_old[c] - mb[c]);

      // Project_Single(i, ml.data());
    }
    Project(ml.data());
    ml_old = ml;
    residual = 0;  // Res4Blaze(ml, mb);
    objective_value = 0;  // GetObjective(ml, mb);
    AtIterationEnd(residual, objective_value);
  }

  return current_iteration;
  return 0;
}
