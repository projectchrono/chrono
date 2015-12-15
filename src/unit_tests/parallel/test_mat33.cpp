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

#include "chrono_parallel/constraints/ChConstraintRigidRigid.h"
#include "chrono_parallel/math/mat33.h"
#include "chrono_parallel/constraints/ChConstraintUtils.h"

#include "chrono/collision/ChCCollisionModel.h"
#include "chrono/core/ChMathematics.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::utils;
int main(int argc, char* argv[]) {
  real3 n = Normalize(real3(rand(), rand(), rand()));
  quaternion R1 = Normalize(quaternion(rand(), rand(), rand(), rand()));
  quaternion R2 = Normalize(quaternion(rand(), rand(), rand(), rand()));

  {
    std::cout << "Cross Matrix\n";

    real3 v, w;
    chrono::Orthogonalize(n, v, w);

    Mat33 cross_m1 = SkewSymmetric(n);

    ChMatrix33<real> cross_m2;
    cross_m2.Set_X_matrix(ToChVector(n));

    // std::cout << cross_m1.U << cross_m1.V << cross_m1.W << endl;
    // std::cout << ToReal3(cross_m2.ClipVector(0, 0)) << ToReal3(cross_m2.ClipVector(0, 1)) <<
    // ToReal3(cross_m2.ClipVector(0, 2)) << endl;

    WeakEqual(cross_m1, ToMat33(cross_m2));
  }

  {
    std::cout << "A Matrix\n";
    Mat33 A1(R1);
    ChMatrix33<real> A2 = ChMatrix33<real>(ToChQuaternion(R1));
    WeakEqual(A1, ToMat33(A2), C_EPSILON * 2);
  }

//  {
//    std::cout << "A Matrix T\n";
//    Mat33 A1 = Transpose(Mat33(R1));
//    ChMatrix33<real> A2 = ChMatrix33<real>(ToChQuaternion(R1));
//    A2.MatrTranspose();
//    WeakEqual(A1, ToMat33(A2), C_EPSILON * 2);
//  }

  Mat33 A1(R1);
  Mat33 A2(R2);
  ChMatrix33<real> B1 = ChMatrix33<real>(ToChQuaternion(R1));
  ChMatrix33<real> B2 = ChMatrix33<real>(ToChQuaternion(R2));

  {
    std::cout << "Multiply Matrix\n";
    Mat33 Res1 = A1 * A2;
    ChMatrix33<real> Res2 = B1 * B2;
    WeakEqual(Res1, ToMat33(Res2));
  }

  {
    std::cout << "Multiply T Matrix \n";

    Mat33 Res1 = TransposeMult(A1, A2);

    ChMatrix33<real> Res2;
    Res2.MatrTMultiply(B1, B2);

    WeakEqual(Res1, ToMat33(Res2), C_EPSILON * 2);
  }

  {
    std::cout << "Multiply Matrix T\n";

    Mat33 Res1 = MultTranspose(A1, A2);

    ChMatrix33<real> Res2;
    Res2.MatrMultiplyT(B1, B2);

    WeakEqual(Res1, ToMat33(Res2), C_EPSILON * 2);
  }

  return 0;
}
