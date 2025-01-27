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
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChRotation.h"

int main(int argc, char* argv[]) {
    ///! [Basic operations with matrices]
    std::cout << "\n=== Creation and assignment ===\n" << std::endl;
    {
        // Matrix allocated on the heap.
        chrono::ChMatrixDynamic<> Md1(5, 7);
        chrono::ChMatrixDynamic<> Md2(4, 4);

        // Matrix allocated on the stack.
        chrono::ChMatrixNM<double, 4, 4> Ms;

        // 3x3 matrices, mostly used for coordinate transformations.
        chrono::ChMatrix33<> ma;

        Ms.setConstant(0.1);  // Fill a matrix with an element
        std::cout << Ms << std::endl;

        Md1.setRandom();         // initialize with random numbers
        Md1.transposeInPlace();  // transpose the matrix in place
        Md1.resize(2, 2);        // resize
        Md1.setZero();           // set all elements to zero

        Md2.setIdentity();  // Create a diagonal matrix
        Md2 *= 3;

        Md2(0, 0) = 10;  // Use the () operator to reference single elements
        Md2(1, 2) = Md2(0, 0) + Md2(1, 1);
        Md2(2, 2) = 4;

        chrono::ChMatrixDynamic<> M1(Md2);   // Copy constructor
        chrono::ChMatrixDynamic<> M2(-Ms);   // The - unary operator returns a negated matrix
        chrono::ChMatrixDynamic<> M3(8, 4);  // 8x4 uninitialized matrix
        M3 = M1;                             // Copy (resize as needed)
        M3 = M1.transpose();                 // transposed copy.
    }

    std::cout << "\n=== Matrix operations ===\n" << std::endl;
    {
        chrono::ChMatrixNM<double, 2, 3> A;
        chrono::ChMatrixDynamic<double> B(2, 3);
        A.setRandom();
        B.setRandom();
        chrono::ChMatrixDynamic<> result(A + B);
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
        std::cout << result << std::endl;

        result = A;
        result *= 10;
        std::cout << result << std::endl;

        // Matrix multiplications
        chrono::ChMatrixDynamic<> C(3, 2);
        chrono::ChMatrixDynamic<> D(2, 3);
        chrono::ChMatrixDynamic<double> CD = C * D;
        chrono::ChMatrixDynamic<double> CD_t = D.transpose() * C.transpose();

        chrono::ChMatrix33<> R(chrono::QuatFromAngleX(chrono::CH_PI / 3));
        std::cout << "rot * matrix\n" << R * C << std::endl;
        std::cout << "matrix * rot\n" << D * R << std::endl;
    }

    std::cout << "\n=== Chrono extensions to Eigen::MatrixBase ===\n" << std::endl;
    {
        chrono::ChMatrixNM<double, 2, 3> A;
        A.setRandom();
        std::cout << "random 2x3 matrix A:\n" << A << std::endl;
        A.fillDiagonal(10.1);
        std::cout << "fill diagonal with 10.1:\n" << A << std::endl;
        A.fill(2.1);
        std::cout << "fill entire matrix with 2.1:\n" << A << std::endl;

        chrono::ChMatrixNM<double, 2, 3> B(A);
        B(1, 2) += 0.01;
        std::cout << "matrix B = A with B(1,2) incremented by 0.01\n" << B << std::endl;
        std::cout << "|A-B| < 0.1?   " << A.isApprox(B, 0.1) << std::endl;
        std::cout << "|A-B| < 0.001? " << A.isApprox(B, 0.001) << std::endl;

        chrono::ChVectorDynamic<> v(3);
        v << 2, 3, 4;
        chrono::ChVectorDynamic<> w(3);
        w << 0.1, 0.2, 0.3;
        std::cout << "||v||_wrms, w = " << v.wrmsNorm(w) << std::endl;
        std::cout << "||v + v||_wrms, w = " << (v + v).wrmsNorm(w) << std::endl;

        std::cout << "v + v + 1: " << (v + v) + 1 << std::endl;
        std::cout << "1 + v: " << 1 + v << std::endl;
    }

    std::cout << "\n=== Matrix comparison tests ===\n" << std::endl;
    {
        chrono::ChMatrixNM<double, 2, 3> A;
        A.setRandom();
        chrono::ChMatrixDynamic<> B(A);
        if (A == B) {
            std::cout << "Matrices are exactly equal" << std::endl;
        }

        // Tolerance comparison
        B(1, 2) += 0.001;
        if (A.isApprox(B, 0.002)) {
            std::cout << "Matrices are equal within tol 0.002 \n";
        }
    }

    std::cout << "\n=== Pasting matrices and matrix-blocks ===\n" << std::endl;
    {
        chrono::ChMatrixDynamic<> A(4, 6);
        A.setRandom();
        std::cout << A << "\n\n";

        chrono::ChMatrixNM<double, 2, 3> B;
        B << 1, 2, 3, 4, 5, 6;
        std::cout << B << "\n\n";

        chrono::ChMatrixNM<double, 5, 7> C;
        C.setZero();
        C.block<2, 3>(1, 1) << 10, 20, 30, 40, 50, 60;
        std::cout << C << "\n\n";

        chrono::ChMatrixDynamic<> X = A;
        X.block<2, 3>(1, 2) = B;
        std::cout << X << "\n\n";

        X.block<2, 3>(1, 2) += C.block<2, 3>(1, 1);
        std::cout << X << "\n\n";
    }

    std::cout << "\n=== 3x3 matrix times vector ===\n" << std::endl;
    {
        chrono::ChVector3d v(1, 2, 3);
        chrono::ChMatrix33<> A;
        A.setRandom();

        chrono::ChMatrix33<> B(1);
        std::cout << B << std::endl;

        // Vector transformation, typical product [A] * v
        chrono::ChVector3d res1 = A * v;

        // Inverse vector transformation, [A]' * v
        chrono::ChVector3d res2 = A.transpose() * v;

        std::cout << A << std::endl;
        std::cout << res1 << std::endl;
        std::cout << res2 << std::endl;
    }

    std::cout << "\n=== Custom 3x4, 4x3, and 4x4 matrices ===\n" << std::endl;
    {
        chrono::ChMatrix34<> G = Eigen::Matrix<double, 3, 4>::Ones(3, 4);
        chrono::ChQuaternion<> q(1, 2, 3, 4);
        chrono::ChVector3d v = G * q;
        std::cout << v << std::endl;

        chrono::ChMatrix43<> Gt = 2 * G.transpose();
        std::cout << Gt << std::endl;
        q = Gt * v;
        std::cout << q << std::endl;
        q = G.transpose() * v;
        std::cout << q << std::endl;

        chrono::ChMatrix33<> rot(chrono::QuatFromAngleX(chrono::CH_PI / 6));
        std::cout << rot << std::endl;
        std::cout << rot.transpose() << std::endl;

        chrono::ChMatrix34<> res1 = rot * G;
        chrono::ChMatrix34<> res2 = rot * Gt.transpose();
        chrono::ChMatrix43<> res3 = Gt * rot;
        chrono::ChMatrix43<> res4 = G.transpose() * rot;
        std::cout << "rot * G:\n" << res1 << std::endl;
        std::cout << "rot * (G').transpose:\n" << res2 << std::endl;
        std::cout << "G' * rot:\n" << res3 << std::endl;
        std::cout << "G.transpose * rot:\n" << res4 << std::endl;

        chrono::ChMatrix44<> A44;
        A44.setRandom();
        chrono::ChQuaternion<> q2 = A44 * q;
        std::cout << "Random 4x4 * q:\n" << q2 << std::endl;

        chrono::ChStarMatrix44<> X(chrono::ChQuaternion<>(1, 2, 3, 4));
        std::cout << "4x4 star matrix X:\n" << X << std::endl;
        X.semiTranspose();
        std::cout << "Semi-transpose X:\n" << X << std::endl;
        X.semiNegate();
        std::cout << "Semi-negate X:\n" << X << std::endl;
    }

    std::cout << "\n=== Frame transformations ===\n" << std::endl;
    {
        chrono::ChVector3d vl(2, 3, 4);        // local point to transform
        chrono::ChVector3d t(5, 6, 7);         // translation of coord system
        chrono::ChQuaternion<> q(1, 3, 4, 5);  // rotation of coord system (quaternion)
        q.Normalize();                         // as unit quaternion, must be normalized
        chrono::ChMatrix33<> R;                // rotation of coord system (rotation matrix)
        R.SetFromQuaternion(q);                // set from quaternion
        chrono::ChCoordsysd csys(t, q);        // coordinate system representing translation + rotation

        // Perform the transformation: v = t + [R] * v'
        // NOTE: all the following ways will give the same result, so you can use them equivalently!
        auto va1 = csys.TransformPointLocalToParent(vl);
        auto va2 = t + R * vl;
        std::cout << va1.Equals(va2, 1e-6) << std::endl;

        // Inverse transformation
        auto vl3 = csys.TransformPointParentToLocal(va1);
        auto vl4 = R.transpose() * (va1 - t);
        std::cout << vl3.Equals(vl, 1e-6) << " " << vl4.Equals(vl, 1e-6) << std::endl;
    }

    std::cout << "\n=== Linear systems ===\n" << std::endl;
    {
        chrono::ChMatrixNM<double, 3, 3> A;
        Eigen::Vector3d b;
        A << 1, 2, 3, 4, 5, 6, 7, 8, 10;
        b << 3, 3, 4;
        std::cout << "matrix A:\n" << A << std::endl;
        std::cout << "vector b:\n" << b << std::endl;
        Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
        std::cout << "solution:\n" << x << std::endl;
        std::cout << "Ax-b:\n" << A * x - b << std::endl;
    }
    ///! [Basic operations with matrices]

    return 0;
}
