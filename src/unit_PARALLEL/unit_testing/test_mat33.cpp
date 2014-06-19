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
#include "chrono_parallel/math/mat33.h"

#include "chrono_utils/ChUtilsCreators.h"
#include "chrono_utils/ChUtilsInputOutput.h"

using namespace std;
using namespace chrono;
using namespace chrono::collision;
using namespace chrono::utils;
int main(
      int argc,
      char* argv[]) {
   real3 n = normalize(real3(rand(), rand(), rand()));
   real4 R1 = normalize(real4(rand(), rand(), rand(), rand()));
   real4 R2 = normalize(real4(rand(), rand(), rand(), rand()));

   {
      cout << "Cross Matrix\n";

      real3 v, w;
      Orthogonalize(n, v, w);

      M33 cross_m1 = XMatrix(n);

      ChMatrix33<float> cross_m2;
      cross_m2.Set_X_matrix(ToChVector(n));

      //cout << cross_m1.U << cross_m1.V << cross_m1.W << endl;
      //cout << ToReal3(cross_m2.ClipVector(0, 0)) << ToReal3(cross_m2.ClipVector(0, 1)) << ToReal3(cross_m2.ClipVector(0, 2)) << endl;

      StrictEqual(cross_m1, ToM33(cross_m2));
   }

   {
      cout << "A Matrix\n";
      M33 A1 = AMat(R1);
      ChMatrix33<float> A2 = ChMatrix33<float>(ToChQuaternion(R1));
      StrictEqual(A1, ToM33(A2));
   }

   {
      cout << "A Matrix T\n";
      M33 A1 = AMatT(R1);
      ChMatrix33<float> A2 = ChMatrix33<float>(ToChQuaternion(R1));
      A2.MatrTranspose();
      StrictEqual(A1, ToM33(A2));
   }

   M33 A1 = AMat(R1);
   M33 A2 = AMat(R2);
   ChMatrix33<float> B1 = ChMatrix33<float>(ToChQuaternion(R1));
   ChMatrix33<float> B2 = ChMatrix33<float>(ToChQuaternion(R2));

   {
      cout << "Multiply Matrix\n";
      M33 Res1 = A1 * A2;
      ChMatrix33<float> Res2 = B1 * B2;
      StrictEqual(Res1, ToM33(Res2));
   }

   {
      cout << "Multiply T Matrix \n";

      M33 Res1 = MatTMult(A1, A2);

      ChMatrix33<float> Res2;
      Res2.MatrTMultiply(B1, B2);

      StrictEqual(Res1, ToM33(Res2));
   }

   {
      cout << "Multiply Matrix T\n";

      M33 Res1 = MatMultT(A1, A2);

      ChMatrix33<float> Res2;
      Res2.MatrMultiplyT(B1, B2);

      StrictEqual(Res1, ToM33(Res2));
   }

   return 0;
}

