// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// ChronoParallel unit test for MPR collision detection
// =============================================================================

#include <stdio.h>
#include <vector>
#include <cmath>
#include "unit_testing.h"
#include "chrono_parallel/collision/ChCNarrowphaseMPR.h"
#include "chrono_parallel/collision/ChCNarrowphaseMPRUtils.h"
#include "chrono_parallel/constraints/ChConstraintRigidRigid.h"

#include "collision/ChCCollisionModel.h"
#include "core/ChMathematics.h"

#include "chrono_utils/ChUtilsInputOutput.h"
#include <blaze/math/CompressedMatrix.h>
#include <blaze/math/DynamicVector.h>
#include "test_matvec.h"

using blaze::CompressedMatrix;
using blaze::DynamicVector;
using namespace chrono;
using namespace chrono::collision;
using namespace chrono::utils;
double timestep = .001;

int main(int argc,
         char* argv[]) {
   omp_set_num_threads(12);
   ChSystemParallelDVI * system_gpu = new ChSystemParallelDVI;
   system_gpu->SetIntegrationType(ChSystem::INT_ANITESCU);
   int size = 5;
   if (argc > 1) {
      size = atoi(argv[1]);
   }
   std::stringstream ss;
   ss << "container_checkpoint_" << size << "_settled.txt";

   ChCollisionSystemBulletParallel * bullet_coll = new ChCollisionSystemBulletParallel();
   system_gpu->ChangeCollisionSystem(bullet_coll);
   system_gpu->SetStep(timestep);
   system_gpu->SetMaxPenetrationRecoverySpeed(10000);

   system_gpu->GetSettings()->solver.solver_mode = SLIDING;
   system_gpu->GetSettings()->solver.max_iteration_normal = 1;
   system_gpu->GetSettings()->solver.max_iteration_sliding = 1;
   system_gpu->GetSettings()->solver.max_iteration_spinning = 0;
   system_gpu->GetSettings()->solver.alpha = 0;
   system_gpu->GetSettings()->solver.contact_recovery_speed = 1;
   system_gpu->ChangeSolverType(APGDBLAZE);

   utils::ReadCheckpoint(system_gpu, ss.str());
   system_gpu->DoStepDynamics(0.0025);

   CompressedMatrix<real> Mass_inv(system_gpu->data_manager->host_data.M_inv);

   CompressedMatrix<real> D_t = system_gpu->data_manager->host_data.D_T;
   CompressedMatrix<real> Mass_invsqt = system_gpu->data_manager->host_data.M_inv;

   CompressedMatrix<real> D = trans(D_t);
   CompressedMatrix<real> MinvD = Mass_inv * D;
   CompressedMatrix<real> MinvsqrtD = Mass_invsqt * D;
   CompressedMatrix<real> MinvsqrtD_t = trans(MinvsqrtD);
   CompressedMatrix<real> N = D_t * MinvD;
   CompressedMatrix<real> N_sym = D_t * (Mass_inv * D);

   //      cout << D.rows() << " " << D.columns() << endl;
   //      cout << D_t.rows() << " " << D_t.columns() << endl;
   //      cout << Mass_inv.rows() << " " << Mass_inv.columns() << endl;

   DynamicVector<real> rhs_vector(system_gpu->data_manager->host_data.rhs_data.size());
   DynamicVector<real> result_1, result_2, result_3, result_4;

   for (unsigned int ic = 0; ic < system_gpu->data_manager->host_data.rhs_data.size(); ic++) {
      rhs_vector[ic] = system_gpu->data_manager->host_data.rhs_data[ic];
   }
   result_1 = result_2 = result_3 = result_4 = rhs_vector;
   std::cout << N.rows() << " " << N.columns() << std::endl;
   int runs = 100;
   ChTimer<double> timer;
   timer.start();
   for (int i = 0; i < runs; i++) {
      result_1 = MinvsqrtD_t * MinvsqrtD * rhs_vector;
   }
   timer.stop();
   std::cout << timer() << " ";
   timer.start();
   for (int i = 0; i < runs; i++) {
      result_2 = N * rhs_vector;
   }
   timer.stop();
   std::cout << timer() << " ";
   timer.start();
   for (int i = 0; i < runs; i++) {
      result_3 = N_sym * rhs_vector;
   }
   timer.stop();
   std::cout << timer() << " ";
   timer.start();
   for (int i = 0; i < runs; i++) {
      result_4 = D_t * MinvD * rhs_vector;
   }
   timer.stop();
   std::cout << timer() << " ";

   thrust::host_vector<int> h_row;
   thrust::host_vector<int> h_col;
   thrust::host_vector<real> h_val;
   thrust::host_vector<real> h_rhs = system_gpu->data_manager->host_data.rhs_data;
   thrust::host_vector<real> h_x = h_rhs;
   //int counter = 0;
   for (int i = 0; i < N.rows(); i++) {
      for (blaze::CompressedMatrix<real>::Iterator it = N.begin(i); it != N.end(i); ++it) {
         h_row.push_back(i);
         h_col.push_back(it->index());
         h_val.push_back(it->value());
         //std::cout<<h_val[counter]<<" "<<N(i,it->index())<<std::endl;
         //counter++;
      }

   }

   //std::cout << "START CUDA" << std::endl;

   //timer.start();
   mat_vec_cusparse(h_row, h_col, h_val, h_rhs, h_x, N.rows(), N.columns(), h_row.size());

   real sum_1 = 0, sum_2 = 0, sum_3 = 0, sum_4 = 0, sum_5 = 0;

   //timer.stop();
   //std::cout << "N * rhs_vector _ GPU: " << timer() << std::endl;
//   for (unsigned int ic = 0; ic < system_gpu->data_manager->host_data.rhs_data.size(); ic++) {
//
//      sum_1 += result_1[ic];
//      sum_2 += result_2[ic];
//      sum_3 += result_3[ic];
//      sum_4 += result_4[ic];
//      sum_5 += h_x[ic];
//
//   }
//
//   std::cout << sum_1 << " " << sum_2 << " " << sum_3 << " " << sum_4 << " " << sum_5 << std::endl;

   return 0;
}
