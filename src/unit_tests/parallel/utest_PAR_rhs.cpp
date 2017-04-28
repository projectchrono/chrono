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

#include <cstdio>
#include <vector>
#include <cmath>

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/collision/ChCNarrowphaseMPR.h"
#include "chrono_parallel/constraints/ChConstraintRigidRigid.h"
#include "chrono_parallel/collision/ChNarrowphaseUtils.h"

#include "chrono/collision/ChCCollisionModel.h"
#include "chrono/core/ChMathematics.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "unit_testing.h"
using namespace chrono;
using namespace chrono::collision;
using namespace chrono::utils;

real timestep = .001;
real factor = 1.0 / timestep;

int main(int argc, char* argv[]) {
  ChSystemParallelNSC* system = new ChSystemParallelNSC;
  system->SetIntegrationType(ChSystem::INT_ANITESCU);

  std::stringstream ss;
  ss << "container_checkpoint_2_settled.txt";

  system->ChangeCollisionSystem(COLLSYS_BULLET_PARALLEL);
  system->SetStep(timestep);
  system->SetMaxPenetrationRecoverySpeed(10000);
  utils::ReadCheckpoint(system, ss.str());

  system->AssembleSystem();

  std::vector<ChConstraint*>& mconstraints = system->GetSystemDescriptor()->GetConstraintsList();
  std::vector<ChVariables*>& mvariables = system->GetSystemDescriptor()->GetVariablesList();

  std::vector<real> bi_a(mconstraints.size());
  std::vector<real> bi_b(mconstraints.size());
  std::vector<real> J_a(mconstraints.size());
  std::vector<real> J_b(mconstraints.size());
  ChMatrixDynamic<> mb(mconstraints.size(), 1);
  ChMatrixDynamic<> mb_tmp(mconstraints.size(), 1);
  system->GetSystemDescriptor()->BuildBiVector(mb_tmp);

  for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
    bi_a[ic] = mb_tmp(ic, 0);
    J_a[ic] = ((ChConstraintTwoBodies*)mconstraints[ic])->Get_Cq_a()->ElementN(ic);
  }
  {
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
      mconstraints[ic]->Update_auxiliary();

    size_t nOfVars = mvariables.size();
    for (unsigned int iv = 0; iv < nOfVars; iv++) {
      if (mvariables[iv]->IsActive()) {
        mvariables[iv]->Compute_invMb_v(mvariables[iv]->Get_qb(), mvariables[iv]->Get_fb());  // q = [M]'*fb
      }
    }

    ChMatrixDynamic<> mb_tmp(mconstraints.size(), 1);
    int s_i = 0;
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
      if (mconstraints[ic]->IsActive()) {
        mb(s_i, 0) = -mconstraints[ic]->Compute_Cq_q();
        ++s_i;
      }

    // ..and finally do   b_shur = b_shur - c
    system->GetSystemDescriptor()->BuildBiVector(mb_tmp);  // b_i   =   -c   = phi/h
    mb.MatrDec(mb_tmp);
  }
  auto container = std::dynamic_pointer_cast<ChContactContainerParallel>(system->GetContactContainer());

  std::list<ChContactContainerParallel::ChContact_6_6*> m_list = container->GetContactList();
  ChTimer<real> timer;

  std::vector<real> b_b(container->GetNcontacts() * 3);
  int ic = 0;
  timer.start();
  for (std::list<ChContactContainerParallel::ChContact_6_6*>::iterator it = m_list.begin(); it != m_list.end(); ++it) {

    ChBody* body_A = (ChBody*)(*it)->GetObjA();
    ChBody* body_B = (ChBody*)(*it)->GetObjB();

    ChVector<real> point_on_A = (*it)->GetContactP1();
    ChVector<real> point_on_B = (*it)->GetContactP2();

    ChVector<real> N = (*it)->GetContactNormal();
    real d = (*it)->GetContactDistance();

    real3 U = (ToReal3(N));
    real3 p1 = ToReal3(point_on_A), p2 = ToReal3(point_on_B);

    real3 V, W;

    real4 A_R = ToReal4(body_A->GetRot());
    real4 B_R = ToReal4(body_B->GetRot());

    real3 A_V = real3(body_A->Variables().Get_qb().GetElementN(0),
                      body_A->Variables().Get_qb().GetElementN(1),
                      body_A->Variables().Get_qb().GetElementN(2));
    real3 B_V = real3(body_B->Variables().Get_qb().GetElementN(0),
                      body_B->Variables().Get_qb().GetElementN(1),
                      body_B->Variables().Get_qb().GetElementN(2));

    real3 A_O = real3(body_A->Variables().Get_qb().GetElementN(3),
                      body_A->Variables().Get_qb().GetElementN(4),
                      body_A->Variables().Get_qb().GetElementN(5));
    real3 B_O = real3(body_B->Variables().Get_qb().GetElementN(3),
                      body_B->Variables().Get_qb().GetElementN(4),
                      body_B->Variables().Get_qb().GetElementN(5));

    real3 temp = R3(0);
    Orthogonalize(U, V, W);  // read 3 real

    //      ChVector<real> Vx, Vy, Vz;
    //      ChVector<real> VN = ToChVector(U);
    //      VN.DirToDxDyDz(Vx, Vy, Vz);
    //      U = ToReal3(Vx);
    //      V = ToReal3(Vy);
    //      W = ToReal3(Vz);

    if (body_A->IsActive()) {
      real3 omega_b1 = A_O;
      real3 vel_b1 = A_V;
      real3 T3, T4, T5;
      Compute_Jacobian(A_R, U, V, W, p1 - ToReal3(body_A->GetPos()), T3, T4, T5);
      temp.x = dot(-U, vel_b1) + dot(T3, omega_b1);
      temp.y = dot(-V, vel_b1) + dot(T4, omega_b1);
      temp.z = dot(-W, vel_b1) + dot(T5, omega_b1);
    }
    if (body_B->IsActive()) {
      real3 omega_b2 = B_O;
      real3 vel_b2 = B_V;
      real3 T6, T7, T8;
      Compute_Jacobian(B_R, U, V, W, p2 - ToReal3(body_B->GetPos()), T6, T7, T8);
      temp.x += dot(U, vel_b2) + dot(-T6, omega_b2);
      temp.y += dot(V, vel_b2) + dot(-T7, omega_b2);
      temp.z += dot(W, vel_b2) + dot(-T8, omega_b2);
    }
    real bi = factor * d;

    temp = -temp;
    // bi_b[ic * 3 + 0] = bi;
    b_b[ic * 3 + 0] = temp.x - bi;
    b_b[ic * 3 + 1] = temp.y;
    b_b[ic * 3 + 2] = temp.z;
    ic++;
  }
  // timer.stop();
  // cout << timer() << endl;

  for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
    // std::cout << bi_a[ic] << " " << mb(ic, 0) << " " << bi_b[ic] << " " << b_b[ic] << " " << ic << std::endl;
    // WeakEqual(mb(ic, 0), b_b[ic],ZERO_EPSILON*10);
  }
  ChMatrixDynamic<> mg_tmp1(mconstraints.size(), 1);
  system->GetSystemDescriptor()->ShurComplementProduct(mg_tmp1, &mb, 0);
  for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
    std::cout << mg_tmp1(ic, 0) << std::endl;
  }
  return 0;
}
