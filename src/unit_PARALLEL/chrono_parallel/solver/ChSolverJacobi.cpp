#include "ChSolverJacobi.h"
#include <blaze/math/SparseRow.h>
#include <blaze/math/CompressedVector.h>
using namespace chrono;

uint ChSolverJacobi::SolveJacobi(const uint max_iter,
                              const uint size,
                              const custom_vector<real> &b,
                              custom_vector<real> &x) {

   real rmax = 0;
   diagonal.resize(size, false);
   ml.resize(size);
   ml_old.resize(size);
   mb.resize(size);

#pragma omp parallel for
   for (int i = 0; i < size; i++) {
      ml[i] = ml_old[i] = x[i];
      mb[i] = b[i];
   }

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
      for (size_t i = 0; i < data_container->num_contacts; ++i) {
         j = i * 3;
         real Dinv = 1.0 / (diagonal[j + 0] + diagonal[j + 1] + diagonal[j + 2]);

         ml[j + 0] = ml[j + 0] - omega * Dinv * ((row(data_container->host_data.Nshur, j + 0), ml_old) - mb[j + 0]);
         ml[j + 1] = ml[j + 1] - omega * Dinv * ((row(data_container->host_data.Nshur, j + 1), ml_old) - mb[j + 1]);
         ml[j + 2] = ml[j + 2] - omega * Dinv * ((row(data_container->host_data.Nshur, j + 2), ml_old) - mb[j + 2]);

         Project_Single(i, ml.data());
      }
      Project(ml.data());
      ml_old = ml;

      AtIterationEnd(rmax, GetObjectiveBlaze(ml, mb), current_iteration);

#pragma omp parallel for
      for (int i = 0; i < size; i++) {
         x[i] = ml[i];
      }
   }
   return current_iteration;
}

