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

using namespace std;
using namespace chrono;
using namespace chrono::collision;
using namespace chrono::utils;
int main(
      int argc,
      char* argv[]) {

   {
      real3 n;
      real d = 0;
      real3 p1, p2;
      real4 A_R = normalize(real4(rand(), rand(), rand(), rand()));
      real4 B_R = normalize(real4(rand(), rand(), rand(), rand()));
      SphereSphere(real3(1, 1, 0), real3(2, 0, 0), real3(1, 0, 0), real3(1, 0, 0), n, d, p1, p2);

      real3 v, w;
      Orthogonalize(n, v, w);

      ChVector<double> Vx, Vy, Vz;
      ChVector<double> singul(VECT_Y);
      ChVector<> VN = ToChVector(n);
      XdirToDxDyDz(&VN, &singul, &Vx, &Vy, &Vz);

      //cout << n << v << w;
      //cout << ToReal3(Vx) << ToReal3(Vy) << ToReal3(Vz);
      cout << "Orthogonalize Sphere Sphere\n";
      WeakEqual(n, ToReal3(Vx));
      WeakEqual(v, ToReal3(Vy));
      WeakEqual(w, ToReal3(Vz));

      //====================================================================

      real3 T3, T4, T5;
      Compute_Jacobian(A_R, n, v, w, p1, T3, T4, T5);
      real3 T6, T7, T8;
      Compute_Jacobian(B_R, n, v, w, p2, T6, T7, T8);

      ChMatrix33<float> Jx1, Jx2, Jr1, Jr2;
      ChMatrix33<float> Ps1, Ps2, Jtemp;
      ChMatrix33<float> A1 = ChMatrix33<float>(ToChQuaternion(A_R));
      ChMatrix33<float> A2 = ChMatrix33<float>(ToChQuaternion(B_R));

      ChVector<float> Pl1 = ChTransform<float>::TransformParentToLocal(ToChVector(p1), ChVector<float>(0, 0, 0), A1);
      ChVector<float> Pl2 = ChTransform<float>::TransformParentToLocal(ToChVector(p2), ChVector<float>(0, 0, 0), A2);

      Ps1.Set_X_matrix(Pl1);
      Ps2.Set_X_matrix(Pl2);

      ChMatrix33<float> contact_plane;
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

      cout << "Contact Planes Sphere Sphere\n";

      //cout << n << v << w;
      //cout << ToReal3(Jx1.ClipVector(0, 0)) << ToReal3(Jx1.ClipVector(0, 1)) << ToReal3(Jx1.ClipVector(0, 2));
      //cout<<ToReal3(contact_plane.ClipVector(0,0))<<ToReal3(contact_plane.ClipVector(0,1))<<ToReal3(contact_plane.ClipVector(0,2));
      WeakEqual(-n, ToReal3(Jx1.ClipVector(0, 0)));
      WeakEqual(-v, ToReal3(Jx1.ClipVector(0, 1)));
      WeakEqual(-w, ToReal3(Jx1.ClipVector(0, 2)));

      WeakEqual(n, ToReal3(Jx2.ClipVector(0, 0)));
      WeakEqual(v, ToReal3(Jx2.ClipVector(0, 1)));
      WeakEqual(w, ToReal3(Jx2.ClipVector(0, 2)));

      cout << "Jacobians Sphere Sphere\n";

      //cout << T3 << T4 << T5 << endl;
      //cout << ToReal3(Jr1.ClipVector(0, 0)) << ToReal3(Jr1.ClipVector(0, 1)) << ToReal3(Jr1.ClipVector(0, 2)) << endl;
      //cout<<ToReal3(Jr2.ClipVector(0,0))<<ToReal3(Jr2.ClipVector(0,1))<<ToReal3(Jr2.ClipVector(0,2));

      WeakEqual(T3, ToReal3(Jr1.ClipVector(0, 0)), FLT_EPSILON * 10);
      WeakEqual(T4, ToReal3(Jr1.ClipVector(0, 1)), FLT_EPSILON * 10);
      WeakEqual(T5, ToReal3(Jr1.ClipVector(0, 2)), FLT_EPSILON * 10);

      WeakEqual(-T6, ToReal3(Jr2.ClipVector(0, 0)), FLT_EPSILON * 10);
      WeakEqual(-T7, ToReal3(Jr2.ClipVector(0, 1)), FLT_EPSILON * 10);
      WeakEqual(-T8, ToReal3(Jr2.ClipVector(0, 2)), FLT_EPSILON * 10);

      //====================================================================

      real3 TA, TB, TC;
      Compute_Jacobian_Rolling(A_R, n, v, w, TA, TB, TC);

      real3 TD, TE, TF;
      Compute_Jacobian_Rolling(B_R, n, v, w, TD, TE, TF);

      ChMatrix33<> Jro1, Jro2;

      Jro1.MatrTMultiply(contact_plane, A1);
      Jro2.MatrTMultiply(contact_plane, A2);
      Jro1.MatrNeg();

      Jro1.MatrTranspose();
      Jro2.MatrTranspose();
      //cout << -TA << -TB << -TC;
      //cout << ToReal3(Jro1.ClipVector(0, 0)) << ToReal3(Jro1.ClipVector(0, 1)) << ToReal3(Jro1.ClipVector(0, 2));
      //cout << ToReal3(Jro2.ClipVector(0, 0)) << ToReal3(Jro2.ClipVector(0, 1)) << ToReal3(Jro2.ClipVector(0, 2));

      cout << "Rolling Sphere Sphere\n";
      WeakEqual(-TA, ToReal3(Jro1.ClipVector(0, 0)), FLT_EPSILON * 5);
      WeakEqual(-TB, ToReal3(Jro1.ClipVector(0, 1)), FLT_EPSILON * 5);
      WeakEqual(-TC, ToReal3(Jro1.ClipVector(0, 2)), FLT_EPSILON * 5);

      WeakEqual(TD, ToReal3(Jro2.ClipVector(0, 0)), FLT_EPSILON * 5);
      WeakEqual(TE, ToReal3(Jro2.ClipVector(0, 1)), FLT_EPSILON * 5);
      WeakEqual(TF, ToReal3(Jro2.ClipVector(0, 2)), FLT_EPSILON * 5);

   }

   {

      real3 p, n(0, 0, 0);
      real d = 0;

      ShapeType A_T = ShapeType::SPHERE;
      real3 A_X = real3(0.629447, -1.45809, 0.045702643641292666);
      real3 A_Y = real3(0.1, 0, 0);
      real3 A_Z = real3(0);
      real4 A_R = normalize(real4(2, 2, 1, 1));

      ShapeType B_T = ShapeType::BOX;
      real3 B_X = real3(0, 0, -0.1);
      real3 B_Y = real3(5, 2, 0.1);
      real3 B_Z = real3(0);
      real4 B_R = normalize(real4(1, 0, 0, 0));

      CollideAndFindPoint(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, d);
      real3 p1, p2;
      GetPoints(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, p1, p2);
      d = dot(n, p2 - p1);

      real3 v, w;
      Orthogonalize(n, v, w);

      ChVector<double> Vx, Vy, Vz;
      ChVector<double> singul(VECT_Y);
      ChVector<> VN = ToChVector(n);
      XdirToDxDyDz(&VN, &singul, &Vx, &Vy, &Vz);

      //cout << n << v << w;
      //cout << ToReal3(Vx) << ToReal3(Vy) << ToReal3(Vz);
      cout << "Orthogonalize Sphere Box\n";
      WeakEqual(n, ToReal3(Vx), FLT_EPSILON);
      WeakEqual(v, ToReal3(Vy), FLT_EPSILON);
      WeakEqual(w, ToReal3(Vz), FLT_EPSILON);
      real3 T3, T4, T5;
      Compute_Jacobian(A_R, n, v, w, p1, T3, T4, T5);
      real3 T6, T7, T8;
      Compute_Jacobian(B_R, n, v, w, p2, T6, T7, T8);

      ChMatrix33<> Jx1, Jx2, Jr1, Jr2;
      ChMatrix33<> Ps1, Ps2, Jtemp;

      ChMatrix33<float> A1 = ChMatrix33<>(ToChQuaternion(A_R));
      ChMatrix33<float> A2 = ChMatrix33<>(ToChQuaternion(B_R));

      ChVector<float> Pl1 = ChTransform<float>::TransformParentToLocal(ToChVector(p1), ChVector<float>(0, 0, 0), A1);
      ChVector<float> Pl2 = ChTransform<float>::TransformParentToLocal(ToChVector(p2), ChVector<float>(0, 0, 0), A2);

      Ps1.Set_X_matrix(Pl1);
      Ps2.Set_X_matrix(Pl2);

      ChMatrix33<float> contact_plane;
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

      // cout << n << v << w<<endl;
      // cout<< -ToReal3(contact_plane.ClipVector(0, 0)) << -ToReal3(contact_plane.ClipVector(0, 1)) << -ToReal3(contact_plane.ClipVector(0, 2))<<endl;
      // cout<<ToReal3(contact_plane.ClipVector(0,0))<<ToReal3(contact_plane.ClipVector(0,1))<<ToReal3(contact_plane.ClipVector(0,2))<<endl;
      cout << "Jacobians Sphere Box\n";
      WeakEqual(-n, ToReal3(Jx1.ClipVector(0, 0)), FLT_EPSILON * 2);
      WeakEqual(-v, ToReal3(Jx1.ClipVector(0, 1)), FLT_EPSILON * 2);
      WeakEqual(-w, ToReal3(Jx1.ClipVector(0, 2)), FLT_EPSILON * 2);

      WeakEqual(n, ToReal3(Jx2.ClipVector(0, 0)), FLT_EPSILON * 2);
      WeakEqual(v, ToReal3(Jx2.ClipVector(0, 1)), FLT_EPSILON * 2);
      WeakEqual(w, ToReal3(Jx2.ClipVector(0, 2)), FLT_EPSILON * 2);

      //cout << T3 << T4 << T5 << endl;
     // cout << ToReal3(Jr1.ClipVector(0, 0)) << ToReal3(Jr1.ClipVector(0, 1)) << ToReal3(Jr1.ClipVector(0, 2)) << endl;

      //cout << T6 << T7 << T8 << endl;
      //cout << ToReal3(Jr2.ClipVector(0, 0)) << ToReal3(Jr2.ClipVector(0, 1)) << ToReal3(Jr2.ClipVector(0, 2)) << endl;

      WeakEqual(T3, ToReal3(Jr1.ClipVector(0, 0)), FLT_EPSILON * 2);
      WeakEqual(T4, ToReal3(Jr1.ClipVector(0, 1)), FLT_EPSILON * 2);
      WeakEqual(T5, ToReal3(Jr1.ClipVector(0, 2)), FLT_EPSILON * 2);

      WeakEqual(-T6, ToReal3(Jr2.ClipVector(0, 0)), FLT_EPSILON * 2);
      WeakEqual(-T7, ToReal3(Jr2.ClipVector(0, 1)), FLT_EPSILON * 2);
      WeakEqual(-T8, ToReal3(Jr2.ClipVector(0, 2)), FLT_EPSILON * 2);

      real3 TA, TB, TC;
      Compute_Jacobian_Rolling(A_R, n, v, w, TA, TB, TC);

      real3 TD, TE, TF;
      Compute_Jacobian_Rolling(B_R, n, v, w, TD, TE, TF);

      ChMatrix33<> Jro1, Jro2;

      Jro1.MatrTMultiply(contact_plane, A1);
      Jro2.MatrTMultiply(contact_plane, A2);
      Jro1.MatrNeg();

      Jro1.MatrTranspose();
      Jro2.MatrTranspose();
      //cout << -TA << -TB << -TC;
      //cout << ToReal3(Jro1.ClipVector(0, 0)) << ToReal3(Jro1.ClipVector(0, 1)) << ToReal3(Jro1.ClipVector(0, 2));
      //cout << ToReal3(Jro2.ClipVector(0, 0)) << ToReal3(Jro2.ClipVector(0, 1)) << ToReal3(Jro2.ClipVector(0, 2));

      cout << "Rolling Sphere Box\n";
      WeakEqual(-TA, ToReal3(Jro1.ClipVector(0, 0)), FLT_EPSILON* 2);
      WeakEqual(-TB, ToReal3(Jro1.ClipVector(0, 1)), FLT_EPSILON* 2);
      WeakEqual(-TC, ToReal3(Jro1.ClipVector(0, 2)), FLT_EPSILON* 2);

      WeakEqual(TD, ToReal3(Jro2.ClipVector(0, 0)), FLT_EPSILON* 2);
      WeakEqual(TE, ToReal3(Jro2.ClipVector(0, 1)), FLT_EPSILON* 2);
      WeakEqual(TF, ToReal3(Jro2.ClipVector(0, 2)), FLT_EPSILON* 2);

   }

   return 0;
}

