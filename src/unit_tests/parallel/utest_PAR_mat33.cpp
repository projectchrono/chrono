// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
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

#include "chrono_parallel/constraints/ChConstraintRigidRigid.h"
#include "chrono_parallel/math/matrix.h"

#include "chrono/collision/ChCCollisionModel.h"
#include "chrono/core/ChMathematics.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "unit_testing.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::utils;
int main(int argc, char* argv[]) {
  real3 n = normalize(real3(rand(), rand(), rand()));
  real4 R1 = normalize(real4(rand(), rand(), rand(), rand()));
  real4 R2 = normalize(real4(rand(), rand(), rand(), rand()));

  {
    std::cout << "Cross Matrix\n";

    real3 v, w;
    Orthogonalize(n, v, w);

    M33 cross_m1 = XMatrix(n);

    ChMatrix33<real> cross_m2;
    cross_m2.Set_X_matrix(ToChVector(n));

    // std::cout << cross_m1.U << cross_m1.V << cross_m1.W << endl;
    // std::cout << ToReal3(cross_m2.ClipVector(0, 0)) << ToReal3(cross_m2.ClipVector(0, 1)) <<
    // ToReal3(cross_m2.ClipVector(0, 2)) << endl;

    WeakEqual(cross_m1, ToM33(cross_m2));
  }

  {
    std::cout << "A Matrix\n";
    M33 A1 = AMat(R1);
    ChMatrix33<real> A2 = ChMatrix33<real>(ToChQuaternion(R1));
    WeakEqual(A1, ToM33(A2));
  }

  {
    std::cout << "A Matrix T\n";
    M33 A1 = AMatT(R1);
    ChMatrix33<real> A2 = ChMatrix33<real>(ToChQuaternion(R1));
    A2.MatrTranspose();
    WeakEqual(A1, ToM33(A2));
  }

  M33 A1 = AMat(R1);
  M33 A2 = AMat(R2);
  ChMatrix33<real> B1 = ChMatrix33<real>(ToChQuaternion(R1));
  ChMatrix33<real> B2 = ChMatrix33<real>(ToChQuaternion(R2));

  {
    std::cout << "Multiply Matrix\n";
    M33 Res1 = A1 * A2;
    ChMatrix33<real> Res2 = B1 * B2;
    WeakEqual(Res1, ToM33(Res2));
  }

  {
    std::cout << "Multiply T Matrix \n";

    M33 Res1 = MatTMult(A1, A2);

    ChMatrix33<real> Res2;
    Res2.MatrTMultiply(B1, B2);

    WeakEqual(Res1, ToM33(Res2));
  }

  {
    std::cout << "Multiply Matrix T\n";

    M33 Res1 = MatMultT(A1, A2);

    ChMatrix33<real> Res2;
    Res2.MatrMultiplyT(B1, B2);

    WeakEqual(Res1, ToM33(Res2));
  }

  return 0;
}
