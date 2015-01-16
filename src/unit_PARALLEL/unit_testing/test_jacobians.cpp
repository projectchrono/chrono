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


using namespace chrono;
using namespace chrono::collision;
using namespace chrono::utils;
int main(int argc,
         char* argv[]) {
   ChSystemParallelDVI * system_gpu = new ChSystemParallelDVI;
   system_gpu->SetIntegrationType(ChSystem::INT_ANITESCU);
   std::stringstream ss;
   ss << "Jacobian_checkpoint_1.txt";

   system_gpu->ChangeCollisionSystem(COLLSYS_BULLET_PARALLEL);

   utils::ReadCheckpoint(system_gpu, ss.str());

   system_gpu->AssembleSystem();

   ChContactContainer* container = (ChContactContainer *) system_gpu->GetContactContainer();

   std::vector<ChLcpConstraint*>& mconstraints = system_gpu->GetLcpSystemDescriptor()->GetConstraintsList();
   std::vector<real> J_a(mconstraints.size() * 6);
   std::vector<real> J_b(mconstraints.size() * 6);
   ChMatrixDynamic<> mb(mconstraints.size(), 1);
   for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
      J_a[ic * 6 + 0] = ((ChLcpConstraintTwoBodies*) mconstraints[ic])->Get_Cq_a()->ElementN(0);
      J_a[ic * 6 + 1] = ((ChLcpConstraintTwoBodies*) mconstraints[ic])->Get_Cq_a()->ElementN(1);
      J_a[ic * 6 + 2] = ((ChLcpConstraintTwoBodies*) mconstraints[ic])->Get_Cq_a()->ElementN(2);
      J_a[ic * 6 + 3] = ((ChLcpConstraintTwoBodies*) mconstraints[ic])->Get_Cq_a()->ElementN(3);
      J_a[ic * 6 + 4] = ((ChLcpConstraintTwoBodies*) mconstraints[ic])->Get_Cq_a()->ElementN(4);
      J_a[ic * 6 + 5] = ((ChLcpConstraintTwoBodies*) mconstraints[ic])->Get_Cq_a()->ElementN(5);

      J_b[ic * 6 + 0] = ((ChLcpConstraintTwoBodies*) mconstraints[ic])->Get_Cq_b()->ElementN(0);
      J_b[ic * 6 + 1] = ((ChLcpConstraintTwoBodies*) mconstraints[ic])->Get_Cq_b()->ElementN(1);
      J_b[ic * 6 + 2] = ((ChLcpConstraintTwoBodies*) mconstraints[ic])->Get_Cq_b()->ElementN(2);
      J_b[ic * 6 + 3] = ((ChLcpConstraintTwoBodies*) mconstraints[ic])->Get_Cq_b()->ElementN(3);
      J_b[ic * 6 + 4] = ((ChLcpConstraintTwoBodies*) mconstraints[ic])->Get_Cq_b()->ElementN(4);
      J_b[ic * 6 + 5] = ((ChLcpConstraintTwoBodies*) mconstraints[ic])->Get_Cq_b()->ElementN(5);
   }

   std::cout << container->GetNcontacts() << " " << mconstraints.size() << std::endl;

   int counter = 0;
   std::list<ChContact*> m_list = container->GetContactList();
   for (std::list<ChContact *>::iterator it = m_list.begin(); it != m_list.end(); ++it) {
      ChModelBulletBody * model_A = (ChModelBulletBody *) (*it)->GetModelA();
      ChModelBulletBody * model_B = (ChModelBulletBody *) (*it)->GetModelB();

      ChBody * body_A = model_A->GetBody();
      ChBody * body_B = model_B->GetBody();

      ChVector<real> point_on_A = (*it)->GetContactP1();
      ChVector<real> point_on_B = (*it)->GetContactP2();

      ChVector<real> Normal = (*it)->GetContactNormal();
      real Depth = (*it)->GetContactDistance();

      real3 n = ToReal3(Normal), v, w;
      real3 p1 = ToReal3(point_on_A);
      real3 p2 = ToReal3(point_on_B);

      if (n == R3(0)) {
         n = R3(0, 1, 0);
      } else {
         n = normalize(n);
      }

      Orthogonalize(n, v, w);

      ChVector<real> Vx, Vy, Vz;
      ChVector<real> VN = Normal;
      VN.DirToDxDyDz(Vx, Vy, Vz);

      //std::cout << n << v << w;
      //std::cout << ToReal3(Vx) << ToReal3(Vy) << ToReal3(Vz);
      WeakEqual(n, ToReal3(Vx));
      WeakEqual(v, ToReal3(Vy));
      WeakEqual(w, ToReal3(Vz));

      real4 A_R = ToReal4(body_A->GetRot());
      real4 B_R = ToReal4(body_B->GetRot());

      std::cout << std::endl;
      real3 T3, T4, T5;
      Compute_Jacobian(A_R, n, v, w, p1, T3, T4, T5);

      real3 T6, T7, T8;
      Compute_Jacobian(B_R, n, v, w, p2, T6, T7, T8);

      ChMatrix33<real> Jx1, Jx2, Jr1, Jr2;
      ChMatrix33<real> Ps1, Ps2, Jtemp;
      ChMatrix33<real> A1 = ChMatrix33<real>(ToChQuaternion(A_R));
      ChMatrix33<real> A2 = ChMatrix33<real>(ToChQuaternion(B_R));

      ChVector<real> Pl1 = ChTransform<real>::TransformParentToLocal(ToChVector(p1), ChVector<real>(0, 0, 0), A1);
      ChVector<real> Pl2 = ChTransform<real>::TransformParentToLocal(ToChVector(p2), ChVector<real>(0, 0, 0), A2);

      Ps1.Set_X_matrix(Pl1);
      Ps2.Set_X_matrix(Pl2);

      ChMatrix33<real> contact_plane;
      contact_plane.Set_A_axis(Vx, Vy, Vz);

      Jx1.CopyFromMatrixT(contact_plane);
      Jx2.CopyFromMatrixT(contact_plane);
      Jx1.MatrNeg();

      Jtemp.MatrMultiply(A1, Ps1);
      Jr1.MatrTMultiply(contact_plane, Jtemp);

      Jtemp.MatrMultiply(A2, Ps2);

      Jr2.MatrTMultiply(contact_plane, Jtemp);
      Jr2.MatrNeg();

      Jx1.MatrTranspose();
      Jx2.MatrTranspose();
      Jr1.MatrTranspose();
      Jr2.MatrTranspose();

      std::cout << J_a[(counter+0) * 6 + 3] << " " << J_a[(counter+0) * 6 + 4] << " " << J_a[(counter+0) * 6 + 5] << " " << std::endl;
      std::cout << -T3 << std::endl;

      std::cout << J_a[(counter+1) * 6 + 3] << " " << J_a[(counter+1) * 6 + 4] << " " << J_a[(counter+1) * 6 + 5] << " " << std::endl;
      std::cout << -T4 << std::endl;

      std::cout << J_a[(counter+2) * 6 + 3] << " " << J_a[(counter+2) * 6 + 4] << " " << J_a[(counter+2) * 6 + 5] << " " << std::endl;
      std::cout << -T5 << std::endl;

      //std::cout << J_a[(counter+0) * 6 + 3] << " " << J_a[(counter+0) * 6 + 4] << " " << J_a[(counter+0) * 6 + 5] << " " << endl;
      //std::cout<<J_a[counter*9 +3]<<" "<<J_a[counter*9 +4]<<" "<<J_a[counter*9 +5]<<" "<<endl;
      //std::cout<<J_a[counter*9 +6]<<" "<<J_a[counter*9 +7]<<" "<<J_a[counter*9 +8]<<" "<<endl;

      //std::cout<<J_b[counter +0]<<" "<<J_a[counter +1]<<" "<<J_a[counter +2]<<" "<<endl;

      //std::cout << T3 << endl;

      //std::cout << n << v << w;
      //std::cout << ToReal3(Jx1.ClipVector(0, 0)) << ToReal3(Jx1.ClipVector(0, 1)) << ToReal3(Jx1.ClipVector(0, 2));
      //std::cout<<ToReal3(contact_plane.ClipVector(0,0))<<ToReal3(contact_plane.ClipVector(0,1))<<ToReal3(contact_plane.ClipVector(0,2));
      WeakEqual(-n, ToReal3(Jx1.ClipVector(0, 0)));
      WeakEqual(-v, ToReal3(Jx1.ClipVector(0, 1)));
      WeakEqual(-w, ToReal3(Jx1.ClipVector(0, 2)));

      WeakEqual(n, ToReal3(Jx2.ClipVector(0, 0)));
      WeakEqual(v, ToReal3(Jx2.ClipVector(0, 1)));
      WeakEqual(w, ToReal3(Jx2.ClipVector(0, 2)));

      WeakEqual(T3, ToReal3(Jr1.ClipVector(0, 0)));
      WeakEqual(T4, ToReal3(Jr1.ClipVector(0, 1)));
      WeakEqual(T5, ToReal3(Jr1.ClipVector(0, 2)));

      WeakEqual(-T6, ToReal3(Jr2.ClipVector(0, 0)));
      WeakEqual(-T7, ToReal3(Jr2.ClipVector(0, 1)));
      WeakEqual(-T8, ToReal3(Jr2.ClipVector(0, 2)));
      counter += 3;
   }
   return 0;

}

