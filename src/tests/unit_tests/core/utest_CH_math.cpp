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
//   Demo on how to use Chrono mathematical
//   functions (vector math, linear algebra, etc)
// =============================================================================

#include "chrono/core/ChTransform.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChLog.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuadrature.h"
#include "chrono/core/ChException.h"

using namespace chrono;

int main(int argc, char* argv[]) {
    GetLog() << "CHRONO foundation classes test: math\n\n";

    //
    // Some examples of math, using Chrono::Engine core features.
    //

    // Use ChMatrixDynamic<> to create a matrix with generic size, say 12 rows x 4 columns.
    // Thank to the template argument, you can tell that you are using elements of
    // type 'double' floating-point precision (note, leaving <> defaults to <double> )
    // If you do not worry about performance, this is the type of matrix that you can
    // use to do everything.
    chrono::ChMatrixDynamic<double> mh(12, 4);

    // Use ChMatrixNM<> to create matrices that do not need to be resized, and
    // whose size is known at compile-time. Columns x rows are passed in template <..> brackets.
    // Although the functionalities are the same of ChMatrixDynamic<>, this type
    // of matrix has better performance (but avoid using it for large sizes, because
    // it is allocated on stack).
    chrono::ChMatrixNM<double, 4, 4> mm;

    // Use ChMatrix33<> to create 3x3 matrices, mostly used for coordinate transformations.
    // It inherits the same high-performance features of ChMatrixNM<>, but also offers special
    // functions for coordinate and rotation operations.
    chrono::ChMatrix33<> ma;

    mm.FillElem(0.1);                    // Fill a matrix with an element
    mm.StreamOUT(GetLog());              // Print a matrix to cout (ex. the console, if open)
    chrono::ChMatrixDynamic<> me(5, 7);  // Create a 5x7 random matrix
    me.FillRandom(-1, 2);
    me.MatrTranspose();  // Transpose the matrix in place
    me.Reset(2, 2);      // Resets the matrix to zero (and modify size, if necessary).
    chrono::ChMatrixDynamic<> md(4, 4);
    md.FillDiag(3);  // Create a diagonal matrix

    md(0, 0) = 10;  // Use the () operator to reference single elements
    md(1, 2) = md(0, 0) + md(1, 1);
    md.Element(2, 2) = 4;  // The md.Element(.,.) function has the same effect as md(.,.)

    chrono::ChMatrixDynamic<> ma1(md);   // Copy constructor
    chrono::ChMatrixDynamic<> ma2(-mm);  // The - unary operator returns a negated matrix
    chrono::ChMatrixDynamic<> ma3(8, 4);
    ma3.CopyFromMatrix(ma1);  // A late copy - matrix will be resized if necessary

    // This is faster than doing: ma3.CopyFromMatrix(ma2); ma3.MatrTranspose();
    ma3.CopyFromMatrixT(ma1);  // Transposed copy.

    // Use the + * -  *= += -=  operators to perform matrix algebra.
    // WARNING: + - *  operators may introduce overhead because they may instance temporary
    //          matrix objects as intermediate results. Use *= -= +=  operators whenever possible,
    //          or use the specific Add() Multiply() functions, for max computational speed.

    chrono::ChMatrixDynamic<> result(ma1 +
                                     ma2);  // Note: size of result is automatically set because of copy constructor
    result = ma1 + ma2;                     // Using the assignment operator (size of result may be automatically reset)

    // Another way to do operations (more intricated, but allows higher performances)
    // Note that, doing this, you must prepare 'result' with already exact column/row size!
    result.MatrAdd(ma1, ma2);

    // You can also use the += operator (often this minimizes the need of intermediate temp.matrices)
    result = ma1;
    result += ma2;

    // Different ways to do subtraction..
    result = ma1 - ma2;
    result.MatrSub(ma1, ma2);
    result = ma1;
    result -= ma2;

    // Multiplications between two matrices, different methods..
    result = ma1 * ma2;
    result.MatrMultiply(ma1, ma2);

    // Multiplication between matrix and scalars, different methods..

    result = ma1 * 10;
    result = ma1;
    result.MatrScale(10);
    GetLog() << result;
    result = ma1;
    result *= 10;
    GetLog() << result;

    // The dot multiplication
    chrono::ChMatrixDynamic<> mmv1(5, 1);
    chrono::ChMatrixDynamic<> mmv2(5, 1);
    mmv1.FillRandom(1, 3);
    mmv2.FillRandom(2, 4);
    double mdot = chrono::ChMatrix<>::MatrDot(mmv1, mmv2);

    // The comparison
    chrono::ChMatrixDynamic<> mmv3(mmv2);
    if (mmv2 == mmv3) {
        GetLog() << "Matrices are exactly equal \n";
    } else {
        return 0;
    }
    // Tolerance comparison
    mmv3.Element(2, 0) += 0.001;
    if (mmv2.Equals(mmv3, 0.002)) {
        GetLog() << "Matrices are equal within tol 0.002 \n";
    } else {
        return 0;
    }
    chrono::Vector mvect(1, 2, 3);
    chrono::Quaternion mquat(1, 2, 3, 4);
    chrono::ChMatrix33<> mta1;
    mta1.FillRandom(-1, 2);

    // Vector transformation, typical product  [A]*v
    chrono::Vector vres = mta1.Matr_x_Vect(mvect);

    // Also with more compact syntax: operator * between matrix and vector..
    chrono::Vector vres2 = mta1 * mvect;
    if (vres == vres2) {
        GetLog() << "vectors are equal \n";
    } else {
        return 0;
    }
    // .. same, but transposed matrix
    vres = mta1.MatrT_x_Vect(mvect);

    // Custom multiplication functions for 3x4 matrices and quaternions:
    chrono::ChMatrixNM<double, 3, 4> mgl;
    mgl.FillRandom(-1, 2);
    vres = mgl.Matr34_x_Quat(mquat);

    chrono::Quaternion qres = mgl.Matr34T_x_Vect(mvect);

    chrono::ChMatrixNM<double, 4, 4> mxq;
    mxq.FillRandom(-1, 2);
    qres = mxq.Matr44_x_Quat(mquat);

    // How to use the ChTransform or ChCoordsys functions to transform points
    // from/to local coordinates in 3D:

    chrono::Vector mvect2;
    chrono::Vector mvect2b;
    chrono::Vector mvect1(2, 3, 4);        // local point to transform
    chrono::Vector vtraslA(5, 6, 7);       // translation of coordsystem
    chrono::Quaternion qrotA(1, 3, 4, 5);  // rotation of coordsys.(as unit quaternion, must be normalized)
    qrotA.Normalize();

    // given a quaternion, sets the corresponding [A] rotation matrix
    chrono::ChMatrix33<> mrotA;
    mrotA.Set_A_quaternion(qrotA);

    // ..you may use also a coordsystem object, representing both
    // translation and rotation.
    chrono::Coordsys csysA(vtraslA, qrotA);

    // Now perform the transformation, like in v'=t+[A]*v
    // NOTE: all the following ways will give the same result, so you
    // can use them equivalently!
    // (the 1st method however is a bit faster..)

    mvect2 = chrono::ChTransform<>::TransformLocalToParent(mvect1, vtraslA, mrotA);

    mvect2 = chrono::ChTransform<>::TransformLocalToParent(mvect1, vtraslA, qrotA);

    mvect2 = csysA.TransformLocalToParent(mvect1);

    mvect2 = vtraslA + mrotA * mvect1;

    // How to use Gauss-Legendre quadrature to compute integrals
    // of functions in 1D/2D/3D

    // Define a y=f(x) function by inheriting ChIntegrable1D:
    class MySine1d : public ChIntegrable1D<double> {
      public:
        void Evaluate(double& result, const double x) { result = sin(x); }
    };
    // Create an object from the function class
    MySine1d mfx;
    // Invoke 6th order Gauss-Legendre quadrature on 0..PI interval:
    double qresult = 0;
    ChQuadrature::Integrate1D<double>(qresult, mfx, 0, CH_C_PI, 6);

    GetLog() << "Quadrature 1d result:" << qresult << " (analytic solution: 2.0) \n";

    // Other quadrature tests, this time in 2D

    class MySine2d : public ChIntegrable2D<double> {
      public:
        void Evaluate(double& result, const double x, const double y) { result = sin(x); }
    };

    MySine2d mfx2d;
    qresult = 0;
    ChQuadrature::Integrate2D<double>(qresult, mfx2d, 0, CH_C_PI, -1, 1, 6);
    GetLog() << "Quadrature 2d result:" << qresult << " (analytic solution: 4.0) \n";

    // Other quadrature tests, this time with vector function
    // (that is, integrates 2x1 matrix)

    class MySine2dM : public ChIntegrable2D<ChMatrixNM<double, 2, 1> > {
      public:
        void Evaluate(ChMatrixNM<double, 2, 1>& result, const double x, const double y) {
            result(0) = x * y;
            result(1) = 0.5 * y * y;
        }
    };

    MySine2dM mfx2dM;
    ChMatrixNM<double, 2, 1> resultM;
    resultM.Reset();
    ChQuadrature::Integrate2D<ChMatrixNM<double, 2, 1> >(resultM, mfx2dM, 0, 1, 0, 3, 6);
    GetLog() << "Quadrature 2d matrix result:" << resultM << " (analytic solution: 2.25, 4.5) \n";

    GetLog() << "\n  CHRONO execution terminated.";

    return 0;
}
