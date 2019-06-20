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
// Authors: Radu Serban
// =============================================================================
//
// Demo for working with matrices & vectors and linear algebra operations.
//
// =============================================================================

#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChMatrixMBD.h"
#include "chrono/core/ChTransform.h"
#include "chrono/core/ChVector.h"

using std::cout;
using std::endl;
using namespace chrono;

int main(int argc, char* argv[]) {
    cout << "\n=== Creation and assignment ===\n" << endl;
    {
        // Matrix allocated on the heap.
        ChMatrixDynamic<> Md1(5, 7);
        ChMatrixDynamic<> Md2(4, 4);

        // Matrix allocated on the stack.
        ChMatrixNM<double, 4, 4> Ms;

        // 3x3 matrices, mostly used for coordinate transformations.
        ChMatrix33<> ma;

        Ms.setConstant(0.1);  // Fill a matrix with an element
        cout << Ms << endl;

        Md1.setRandom();         // initialize with random numbers
        Md1.transposeInPlace();  // transpose the matrix in place
        Md1.resize(2, 2);        // resize
        Md1.setZero();           // set all elements to zero

        Md2.setIdentity();  // Create a diagonal matrix
        Md2 *= 3;

        Md2(0, 0) = 10;  // Use the () operator to reference single elements
        Md2(1, 2) = Md2(0, 0) + Md2(1, 1);
        Md2(2, 2) = 4;

        ChMatrixDynamic<> M1(Md2);   // Copy constructor
        ChMatrixDynamic<> M2(-Ms);   // The - unary operator returns a negated matrix
        ChMatrixDynamic<> M3(8, 4);  // 8x4 uninitialized matrix
        M3 = M1;                     // Copy (resize as needed)
        M3 = M1.transpose();         // transposed copy.
    }

    cout << "\n=== Matrix operations ===\n" << endl;
    {
        ChMatrixNM<double, 2, 3> A;
        ChMatrixDynamic<double> B(2, 3);
        A.setRandom();
        B.setRandom();
        ChMatrixDynamic<> result(A + B);
        result = A + B;
        result = A;
        result += B;

        // Different ways to do subtraction..
        result = A - B;
        result = A;
        result -= B;

        // Multiplication with scalar
        result = A * 10;
        result = 10 * A;
        cout << result << endl;

        result = A;
        result *= 10;
        cout << result << endl;

        // Matrix multiplications
        ChMatrixDynamic<> C(3, 2);
        ChMatrixDynamic<> D(2, 3);
        ChMatrixDynamic<double> CD = C * D;
        ChMatrixDynamic<double> CD_t = D.transpose() * C.transpose();

        ChMatrix33<> R(Q_from_AngX(CH_C_PI / 3));
        cout << "rot * matrix\n" << R * C << endl;
        cout << "matrix * rot\n" << D * R << endl;
    }

    cout << "\n=== Chrono extensions to Eigen::MatrixBase ===\n" << endl;
    {
        ChMatrixNM<double, 2, 3> A;
        A.setRandom();
        cout << "random 2x3 matrix A:\n" << A << endl;
        A.fillDiagonal(10.1);
        cout << "fill diagonal with 10.1:\n" << A << endl;
        A.fill(2.1);
        cout << "fill entire matrix with 2.1:\n" << A << endl;

        ChMatrixNM<double, 2, 3> B(A);
        B(1, 2) += 0.01;
        cout << "matrix B = A with B(1,2) incremented by 0.01\n" << B << endl;
        cout << "|A-B| < 0.1?   " << A.equals(B, 0.1) << endl;
        cout << "|A-B| < 0.001? " << A.equals(B, 0.001) << endl;

        ChVectorDynamic<> v(3);
        v << 2, 3, 4;
        ChVectorDynamic<> w(3);
        w << 0.1, 0.2, 0.3;
        cout << "||v||_wrms, w = " << v.wrmsNorm(w) << endl;
        cout << "||v + v||_wrms, w = " << (v + v).wrmsNorm(w) << endl;

        cout << "v + v + 1: " << (v + v) + 1 << endl;
        cout << "1 + v: " << 1 + v << endl;
    }

    cout << "\n=== Matrix comparison tests ===\n" << endl;
    {
        ChMatrixNM<double, 2, 3> A;
        A.setRandom();
        ChMatrixDynamic<> B(A);
        if (A == B) {
            cout << "Matrices are exactly equal" << endl;
        }

        // Tolerance comparison
        B(1, 2) += 0.001;
        if (A.equals(B, 0.002)) {
            cout << "Matrices are equal within tol 0.002 \n";
        }
    }

    cout << "\n=== Pasting matrices and matrix-blocks ===\n" << endl;
    {
        ChMatrixDynamic<> A(4, 6);
        A.setRandom();
        cout << A << "\n\n";

        ChMatrixNM<double, 2, 3> B;
        B << 1, 2, 3, 4, 5, 6;
        cout << B << "\n\n";

        ChMatrixNM<double, 5, 7> C;
        C.setZero();
        C.block<2, 3>(1, 1) << 10, 20, 30, 40, 50, 60;
        cout << C << "\n\n";

        ChMatrixDynamic<> X = A;
        X.block<2, 3>(1, 2) = B;
        cout << X << "\n\n";

        X.block<2, 3>(1, 2) += C.block<2, 3>(1, 1);
        cout << X << "\n\n";
    }

    cout << "\n=== 3x3 matrix times vector ===\n" << endl;
    {
        ChVector<> v(1, 2, 3);
        ChMatrix33<> A;
        A.setRandom();

        ChMatrix33<> B(1);
        cout << B << endl;

        // Vector transformation, typical product [A] * v
        ChVector<> res1 = A * v;

        // Inverse vector transformation, [A]' * v
        ChVector<> res2 = A.transpose() * v;

        cout << A << endl;
        cout << res1 << endl;
        cout << res2 << endl;
    }

    cout << "\n=== Custom 3x4, 4x3, and 4x4 matrices ===\n" << endl;
    {
        ChMatrix34<> G = Eigen::Matrix<double, 3, 4>::Ones(3, 4);
        ChQuaternion<> q(1, 2, 3, 4);
        ChVector<> v = G * q;
        cout << v << endl;

        ChMatrix43<> Gt = 2 * G.transpose();
        cout << Gt << endl;
        q = Gt * v;
        cout << q << endl;
        q = G.transpose() * v;
        cout << q << endl;

        ChMatrix33<> rot(Q_from_AngX(CH_C_PI / 6));
        cout << rot << endl;
        cout << rot.transpose() << endl;

        ChMatrix34<> res1 = rot * G;
        ChMatrix34<> res2 = rot * Gt.transpose();
        ChMatrix43<> res3 = Gt * rot;
        ChMatrix43<> res4 = G.transpose() * rot;

        ChMatrix44<> A44;
        A44.setRandom();
        ChQuaternion<> q2 = A44 * q;

        ChStarMatrix44<> X(ChQuaternion<>(1, 2, 3, 4));
        cout << "4x4 star matrix X:\n" << X << endl;
        X.semiTranspose();
        cout << "Semi-transpose X:\n" << X << endl;
        X.semiNegate();
        cout << "Semi-negate X:\n" << X << endl;
    }

    cout << "\n=== Use of ChTransform ===\n" << endl;
    {
        ChVector<> vl(2, 3, 4);        // local point to transform
        ChVector<> t(5, 6, 7);         // translation of coord system
        ChQuaternion<> q(1, 3, 4, 5);  // rotation of coord system (quaternion)
        q.Normalize();                 // as unit quaternion, must be normalized
        ChMatrix33<> R;                // rotation of coord system (rotation matrix)
        R.Set_A_quaternion(q);         // set from quaternion
        Coordsys csys(t, q);           // coordinate system representing translation + rotation

        // Perform the transformation: v = t + [R] * v'
        // NOTE: all the following ways will give the same result, so you can use them equivalently!
        auto va1 = ChTransform<>::TransformLocalToParent(vl, t, R);
        auto va2 = ChTransform<>::TransformLocalToParent(vl, t, q);
        auto va3 = csys.TransformLocalToParent(vl);
        auto va4 = t + R * vl;
        cout << va2.Equals(va1, 1e-6) << " " << va3.Equals(va1, 1e-6) << " " << va4.Equals(va1, 1e-6) << endl;

        // Inverse transformation
        auto vl1 = ChTransform<>::TransformParentToLocal(va1, t, q);
        auto vl2 = ChTransform<>::TransformParentToLocal(va1, t, R);
        auto vl3 = csys.TransformParentToLocal(va1);
        auto vl4 = R.transpose() * (va1 - t);
        cout << vl1.Equals(vl, 1e-6) << " " << vl2.Equals(vl, 1e-6) << " " << vl3.Equals(vl, 1e-6) << " "
             << vl4.Equals(vl, 1e-6) << endl;
    }

    cout << "\n=== Linear systems ===\n" << endl;
    {
        ChMatrixNM<double, 3, 3> A;
        Eigen::Vector3d b;
        A << 1, 2, 3, 4, 5, 6, 7, 8, 10;
        b << 3, 3, 4;
        cout << "matrix A:\n" << A << endl;
        cout << "vector b:\n" << b << endl;
        Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
        cout << "solution:\n" << x << endl;
        cout << "Ax-b:\n" << A * x - b << endl;
    }

    return 0;
}
