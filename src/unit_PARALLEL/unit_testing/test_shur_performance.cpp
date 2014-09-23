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

#include "chrono_utils/ChUtilsCreators.h"
#include "chrono_utils/ChUtilsInputOutput.h"
#include <blaze/math/CompressedMatrix.h>
#include <blaze/math/DynamicVector.h>
#include <blaze/math/SymmetricMatrix.h>
using blaze::CompressedMatrix;
using blaze::DynamicVector;
using namespace std;
using namespace chrono;
using namespace chrono::collision;
using namespace chrono::utils;
double timestep = .001;
double factor = 1.0 / timestep;

int main(int argc,
         char* argv[]) {
   omp_set_num_threads(8);
   ChSystemParallelDVI * system_gpu = new ChSystemParallelDVI;
   system_gpu->SetIntegrationType(ChSystem::INT_ANITESCU);

   std::stringstream ss;
   ss << "container_checkpoint_50_settled.txt";

   ChCollisionSystemBulletParallel * bullet_coll = new ChCollisionSystemBulletParallel();
   system_gpu->ChangeCollisionSystem(bullet_coll);
   system_gpu->SetStep(timestep);
   system_gpu->SetMaxPenetrationRecoverySpeed(10000);
   utils::ReadCheckpoint(system_gpu, ss.str());
   system_gpu->AssembleSystem();
   ChContactContainer* container = (ChContactContainer *) system_gpu->GetContactContainer();

   std::vector<ChLcpConstraint*>& mconstraints = system_gpu->GetLcpSystemDescriptor()->GetConstraintsList();
   std::vector<ChLcpVariables*>& mvariables = system_gpu->GetLcpSystemDescriptor()->GetVariablesList();
   size_t nOfVars = mvariables.size();
   size_t nOfConstraints = mconstraints.size();

   ChMatrixDynamic<> mb(nOfConstraints, 1);

//#########
   for (unsigned int ic = 0; ic < nOfConstraints; ic++) {
      mconstraints[ic]->Update_auxiliary();
   }

   // Average all g_i for the triplet of contact constraints n,u,v.
   //  Can be used for the fixed point phase and/or by preconditioner.
   int j_friction_comp = 0;
   double gi_values[3];
   for (unsigned int ic = 0; ic < nOfConstraints; ic++) {
      if (mconstraints[ic]->GetMode() == CONSTRAINT_FRIC) {
         gi_values[j_friction_comp] = mconstraints[ic]->Get_g_i();
         j_friction_comp++;
         if (j_friction_comp == 3) {
            double average_g_i = (gi_values[0] + gi_values[1] + gi_values[2]) / 3.0;
            mconstraints[ic - 2]->Set_g_i(average_g_i);
            mconstraints[ic - 1]->Set_g_i(average_g_i);
            mconstraints[ic - 0]->Set_g_i(average_g_i);
            j_friction_comp = 0;
         }
      }
   }

   for (unsigned int iv = 0; iv < nOfVars; iv++) {
      if (mvariables[iv]->IsActive()) {
         mvariables[iv]->Compute_invMb_v(mvariables[iv]->Get_qb(), mvariables[iv]->Get_fb());  // q = [M]'*fb
      }
   }
   ChMatrixDynamic<> mb_tmp(nOfConstraints, 1);
   int s_i = 0;
   for (unsigned int ic = 0; ic < nOfConstraints; ic++)
      if (mconstraints[ic]->IsActive()) {
         mb(s_i, 0) = -mconstraints[ic]->Compute_Cq_q();
         ++s_i;
      }
   system_gpu->GetLcpSystemDescriptor()->BuildBiVector(mb_tmp);   // b_i   =   -c   = phi/h
   mb.MatrDec(mb_tmp);
   ChLcpSystemDescriptor* sysd = system_gpu->GetLcpSystemDescriptor();

   //ChMatrixDynamic<> mq;
   //sysd->FromVariablesToVector(mq, true);
   //#########
   chrono::ChSparseMatrix mdM;
   chrono::ChSparseMatrix mdCq;
   chrono::ChSparseMatrix mdE;
   chrono::ChMatrixDynamic<double> mdf;
   chrono::ChMatrixDynamic<double> mdb;
   chrono::ChMatrixDynamic<double> mdfric;
   chrono::ChMatrixDynamic<double> mdgamma;
   sysd->ConvertToMatrixForm(&mdCq, &mdM, &mdE, &mdf, &mdb, &mdfric);

   cout << mdM.GetRows() << " " << mdM.GetColumns() << endl;
   cout << mdCq.GetRows() << " " << mdCq.GetColumns() << endl;

//   int mn_c = 0;
//   for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
//      if (mconstraints[ic]->IsActive())
//         if (!((mconstraints[ic]->GetMode() == CONSTRAINT_FRIC)))
//            if (!((dynamic_cast<ChLcpConstraintTwoFrictionT*>(mconstraints[ic])))) {
//               mn_c++;
//            }
//   }

   // Count active variables, by scanning through all variable blocks,
   // and set offsets

   int n_q = sysd->CountActiveVariables();

   CompressedMatrix<double> Mass_inv(n_q, n_q), D_t(mdCq.GetRows(), mdCq.GetColumns());
   CompressedMatrix<double> Mass_invsqt(n_q, n_q);

   for (int ii = 0; ii < mdM.GetRows(); ii++) {
      for (int jj = 0; jj < mdM.GetColumns(); jj++) {
         double elVal = mdM.GetElement(ii, jj);
         if (elVal || (ii + 1 == mdM.GetRows() && jj + 1 == mdM.GetColumns())) {
            Mass_inv.insert(ii, jj, 1.0 / elVal);
            Mass_invsqt.insert(ii, jj, 1.0 / sqrt(elVal));
         }
      }
   }

   for (int ii = 0; ii < mdCq.GetRows(); ii++) {
      for (int jj = 0; jj < mdCq.GetColumns(); jj++) {
         double elVal = mdCq.GetElement(ii, jj);
         if (elVal || (ii + 1 == mdCq.GetRows() && jj + 1 == mdCq.GetColumns())) {
            D_t.insert(ii, jj, elVal);
         }
      }
   }
   CompressedMatrix<double> D = trans(D_t);
   CompressedMatrix<double> MinvD = Mass_inv * D;
   CompressedMatrix<double> MinvsqrtD = Mass_invsqt * D;
   CompressedMatrix<double> MinvsqrtD_t = trans(MinvsqrtD);
   CompressedMatrix<double> N = D_t * Mass_inv * D;
   blaze::SymmetricMatrix<CompressedMatrix<double> > N_sym = D_t * Mass_inv * D;
//
//      cout << D.rows() << " " << D.columns() << endl;
//      cout << D_t.rows() << " " << D_t.columns() << endl;
//      cout << Mass_inv.rows() << " " << Mass_inv.columns() << endl;

   DynamicVector<double> rhs_vector(nOfConstraints);

   for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
      rhs_vector[ic] = mb(ic, 0);
   }

   cout << N.rows() << " " << N.columns() << endl;

   ChTimer<double> timer;
   timer.start();
   DynamicVector<double> result_1 = MinvsqrtD_t * (MinvsqrtD * rhs_vector);
   timer.stop();
   std::cout << "MinvsqrtD_t * MinvsqrtD * rhs_vector: " << timer() << std::endl;
   timer.start();
   DynamicVector<double> result_2 = N * rhs_vector;
   timer.stop();
   std::cout << "N * rhs_vector: " << timer() << std::endl;
   timer.start();
   DynamicVector<double> result_3 = N_sym * rhs_vector;
   timer.stop();
   std::cout << "N_sym * rhs_vector: " << timer() << std::endl;
   timer.start();
   DynamicVector<double> result_4 = D_t * (MinvD * rhs_vector);
   timer.stop();
   std::cout << "D_t * MinvD * rhs_vector: " << timer() << std::endl;

   return 0;

}

