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
   stringstream ss;
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

   int mn_c = 0;
   for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
      if (mconstraints[ic]->IsActive())
         if (!((mconstraints[ic]->GetMode() == CONSTRAINT_FRIC)))
            if (!((dynamic_cast<ChLcpConstraintTwoFrictionT*>(mconstraints[ic])))) {
               mn_c++;
            }
   }

   // Count active variables, by scanning through all variable blocks,
   // and set offsets

   int n_q = sysd->CountActiveVariables();

   CompressedMatrix<double> Mass_inv(n_q, n_q), D(mdCq.GetRows(), mdCq.GetColumns());

   for (int ii = 0; ii < mdM.GetRows(); ii++) {
      for (int jj = 0; jj < mdM.GetColumns(); jj++) {
         double elVal = mdM.GetElement(ii, jj);
         if (elVal || (ii + 1 == mdM.GetRows() && jj + 1 == mdM.GetColumns())) {
            Mass_inv.insert(ii, jj, 1.0 / elVal);
         }
      }
   }

   for (int ii = 0; ii < mdCq.GetRows(); ii++) {
      for (int jj = 0; jj < mdCq.GetColumns(); jj++) {
         double elVal = mdCq.GetElement(ii, jj);
         if (elVal || (ii + 1 == mdCq.GetRows() && jj + 1 == mdCq.GetColumns())) {
            D.insert(ii, jj, elVal);
         }
      }
   }

   CompressedMatrix<double> D_t = trans(D);


//   cout << D.rows() << " " << D.columns() << endl;
//   cout << D_t.rows() << " " << D_t.columns() << endl;
//   cout << Mass_inv.rows() << " " << Mass_inv.columns() << endl;
   ChTimer<double> timer_shur, timer_shur_2;
   timer_shur.start();
   CompressedMatrix<double> N = D * Mass_inv * D_t;
   timer_shur.stop();
   cout << N.rows() << " " << N.columns() << endl;
   cout << "shur: " << timer_shur() << endl;

   DynamicVector<double> rhs_vector(nOfConstraints);

   for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
      rhs_vector[ic] = mb(ic, 0);
   }
   ChTimer<double> timer_mv;
   timer_mv.start();
   for(int i=0; i<882; i++){
      DynamicVector<double> Mv = N*rhs_vector;

   }
   timer_mv.stop();
   cout<<"Timer_mv: "<<timer_mv()<<endl;
   ChTimer<double> timer;
   timer.start();
   double time_shur_2 = 0;
   int total_matvec = 0;
//#########
   double max_iterations = 212;
   double SIZE = nOfConstraints;
   double lastgoodres = 10e30;
   double theta_k = 1.0;
   double theta_k1 = theta_k;
   double beta_k1 = 0.0;
   double L_k = 0.0;
   double t_k = 0.0;

   ChMatrixDynamic<> ml(nOfConstraints, 1);
   ChMatrixDynamic<> ml_candidate(nOfConstraints, 1);
   ChMatrixDynamic<> mg(nOfConstraints, 1);
   ChMatrixDynamic<> mg_tmp(nOfConstraints, 1);
   ChMatrixDynamic<> my(nOfConstraints, 1);
   ChMatrixDynamic<> mx(nOfConstraints, 1);
   ChMatrixDynamic<> mg_tmp1(nOfConstraints, 1);
   ChMatrixDynamic<> mg_tmp2(nOfConstraints, 1);
   ChMatrixDynamic<> ms(nOfConstraints, 1);
   ml.FillElem(0);
   sysd->ConstraintsProject(ml);
   ml_candidate = ml;
   timer_shur_2.start();
   sysd->ShurComplementProduct(mg, &ml, 0);
   total_matvec++;
   timer_shur_2.stop();
   time_shur_2 += timer_shur_2();
   mg = mg - mb;

   mb_tmp.FillElem(-1.0);
   mb_tmp += ml;
   timer_shur_2.start();
   sysd->ShurComplementProduct(mg_tmp, &mb_tmp, 0);  // 1)  g = N*l ...        #### MATR.MULTIPLICATION!!!###
   timer_shur_2.stop();
   total_matvec++;
   time_shur_2 += timer_shur_2();
   if (mb_tmp.NormTwo() == 0) {
      L_k = 1;
   } else {
      L_k = mg_tmp.NormTwo() / mb_tmp.NormTwo();
   }
   t_k = 1.0 / L_k;

   double obj1 = 0;
   double obj2 = 0;

   my = ml;
   mx = ml;

   for (int iter = 0; iter < max_iterations; iter++) {
      timer_shur_2.start();
      sysd->ShurComplementProduct(mg_tmp1, &my, 0);
      timer_shur_2.stop();
      total_matvec++;
      time_shur_2 += timer_shur_2();
      mg = mg_tmp1 - mb;
      mx = mg * -t_k + my;
      sysd->ConstraintsProject(mx);
      timer_shur_2.start();
      sysd->ShurComplementProduct(mg_tmp, &mx, 0);
      total_matvec++;
      timer_shur_2.stop();
      time_shur_2 += timer_shur_2();
      mg_tmp2 = mg_tmp - mb;
      ms = mg_tmp * 0.5 - mb;
      obj1 = mx.MatrDot(&mx, &ms);
      ms = mg_tmp1 * 0.5 - mb;
      obj2 = my.MatrDot(&my, &ms);

      ms = mx - my;
      while (obj1 > obj2 + mg.MatrDot(&mg, &ms) + 0.5 * L_k * pow(ms.NormTwo(), 2.0)) {
         L_k = 2.0 * L_k;
         t_k = 1.0 / L_k;

         mx = mg * -t_k + my;
         sysd->ConstraintsProject(mx);
         timer_shur_2.start();
         sysd->ShurComplementProduct(mg_tmp, &mx, 0);
         timer_shur_2.stop();
         total_matvec++;
         time_shur_2 += timer_shur_2();
         mg_tmp2 = mg_tmp - mb;
         ms = mg_tmp * 0.5 - mb;
         obj1 = mx.MatrDot(&mx, &ms);
         ms = mx - my;
      }

      theta_k1 = (-pow(theta_k, 2) + theta_k * sqrt(pow(theta_k, 2) + 4)) / 2.0;
      beta_k1 = theta_k * (1.0 - theta_k) / (pow(theta_k, 2) + theta_k1);

      ms = (mx - ml);
      my = ms * beta_k1 + mx;

      if (mg.MatrDot(&mg, &ms) > 0) {
         my = mx;
         theta_k1 = 1.0;
      }

      L_k = 0.9 * L_k;
      t_k = 1.0 / L_k;

      ml = mx;
      theta_k = theta_k1;

      //========
      ChMatrixDynamic<> testres(nOfConstraints, 1);
      timer_shur_2.start();
      sysd->ShurComplementProduct(testres, &ml, 0);
      timer_shur_2.stop();
      total_matvec++;
      time_shur_2 += timer_shur_2();
      testres = testres - mb;

      double gdiff = .1;

      ChMatrixDynamic<> inside = ml - testres * gdiff;
      sysd->ConstraintsProject(inside);

      ChMatrixDynamic<> resid = (ml - inside) * (1.0 / nOfConstraints * gdiff);
      double g_proj_norm = resid.NormTwo();

      //========

      if (g_proj_norm < lastgoodres) {
         lastgoodres = g_proj_norm;
         ml_candidate = ml;
      }

      //========
      // f_p = 0.5*l_candidate'*N*l_candidate - l_candidate'*b  = l_candidate'*(0.5*Nl_candidate - b);
      timer_shur_2.start();
      sysd->ShurComplementProduct(mg_tmp, &ml_candidate, 0);    // 1)  g_tmp = N*l_candidate ...        #### MATR.MULTIPLICATION!!!###
      timer_shur_2.stop();
      total_matvec++;
      time_shur_2 += timer_shur_2();
      mg_tmp = mg_tmp * 0.5 - mb;                                   // 2)  g_tmp = 0.5*N*l_candidate                                    // 3)  g_tmp = 0.5*N*l_candidate-b_shur
      double m_objective = ml_candidate.MatrDot(&ml_candidate, &mg_tmp);     // 4)  mf_p  = l_candidate'*(0.5*N*l_candidate-b_shur)
      //========
      double maxdeltalambda = ms.NormInf();
      double maxd = lastgoodres;
      //cout << "  iter=" << iter << "   f=" << m_objective << "  |d|=" << maxd << "  |s|=" << maxdeltalambda << "\n";
   }
   ml = ml_candidate;
   timer.stop();
   cout << timer() << endl;

   cout << "total_shur: " << time_shur_2 << endl;
   cout<<total_matvec<<endl;
//#########
   cout << system_gpu->GetNbodies() << " " << container->GetNcontacts() << endl;

   return 0;

}

