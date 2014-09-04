#include "ChSolverPGS.h"

using namespace chrono;

uint ChSolverPGS::SolvePGS(const uint max_iter,
                           const uint size,
                           const custom_vector<real> &b,
                           custom_vector<real> &x) {

   const size_t n = size;
   real rmax = 0, residual, flimit, aux;
   diagonal.resize(n, false);
   ml.resize(n);
   mb.resize(n);

#pragma omp parallel for
   for (int i = 0; i < size; i++) {
      ml[i] = x[i];
      mb[i] = b[i];
   }

   for (size_t i = 0; i < n; ++i) {
      const real tmp(data_container->host_data.Nshur(i, i));
      diagonal[i] = real(1) / tmp;
   }

   Project(ml.data());

   for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
      const size_t N(size / 3);
      size_t j;
      real xold, yold, zold, maxResidual = 0;

//      for (size_t i = 0; i < data_container->num_contacts; ++i) {
//         const real residualx(-mb[i*3+0] - (data_container->host_data.Nshur * ml)[i*3+0]);
//         const real residualy(-mb[i*3+1] - (data_container->host_data.Nshur * ml)[i*3+1]);
//         const real residualz(-mb[i*3+2] - (data_container->host_data.Nshur * ml)[i*3+2]);
//
//
//         // Updating and projecting the unknown
//         xold = ml[i*3+0];
//         yold = ml[i*3+1];
//         zold = ml[i*3+2];
//
//         ml[i*3+0] += diagonal[i*3+0] * residualx;
//         ml[i*3+1] += diagonal[i*3+1] * residualy;
//         ml[i*3+2] += diagonal[i*3+2] * residualz;
//
//         Project_Single(i, ml.data());
//         maxResidual = std::max(maxResidual, std::fabs(xold - ml[i*3+0]));
//      }

      for (size_t i = 0; i < N; ++i) {

         j = i * 3;
         residual = -mb[j] - (data_container->host_data.Nshur * ml)[j];
         aux = std::max(0.0, ml[j] - diagonal[j] * residual);
         rmax = std::max(rmax, std::fabs(ml[j] - aux));
         ml[j] = aux;

         flimit = data_container->host_data.fric_rigid_rigid[i].x * ml[j];

         ++j;
         residual = -mb[j] - (data_container->host_data.Nshur * ml)[j];
         aux = std::max(-flimit, std::min(flimit, ml[j] - diagonal[j] * residual));
         rmax = std::max(rmax, std::fabs(ml[j] - aux));
         ml[j] = aux;

         ++j;
         residual = -mb[j] - (data_container->host_data.Nshur * ml)[j];
         aux = std::max(-flimit, std::min(flimit, ml[j] - diagonal[j] * residual));
         rmax = std::max(rmax, std::fabs(ml[j] - aux));
         ml[j] = aux;
      }

      AtIterationEnd(rmax, GetObjectiveBlaze(ml, mb), current_iteration);

#pragma omp parallel for
      for (int i = 0; i < size; i++) {
         x[i] = ml[i];
      }

   }

   return current_iteration;
}

