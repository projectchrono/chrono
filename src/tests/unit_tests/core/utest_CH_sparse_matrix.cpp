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
// Tests for Chrono sparse matrix classes.
//
// =============================================================================

#include <iostream>
#include <cmath>

#include "chrono/core/ChLinkedListMatrix.h"
#include "chrono/core/ChMapMatrix.h"
#include "chrono/core/ChMatrixDynamic.h"

using namespace chrono;

using std::cout;
using std::endl;

bool testLinkedListMatrix() {
    cout << endl << "---  LinkedListMatrix ---" << endl << endl;

    bool passed = true;
    double tolerance = 1e-10;

    // General linear system solution
    cout << "Test solution of general system Ax=b" << endl;
    {
        ChLinkedListMatrix A(5, 5);
        A.SetElement(1, 0, 0.130);
        A.SetElement(3, 0, 0.012);
        A.SetElement(2, 1, 1);
        A.SetElement(0, 2, -1);
        A.SetElement(3, 2, 0.337);
        A.SetElement(1, 3, 0.569);
        A.SetElement(4, 3, -0.1);
        A.SetElement(2, 4, 0.469);
        A.SetElement(4, 4, 1);

        double det_ref = 0.006828;

        ChMatrixDynamic<> x_ref(5, 1);
        x_ref(0, 0) = 0.34;
        x_ref(1, 0) = 0.58;
        x_ref(2, 0) = 0.23;
        x_ref(3, 0) = 0.75;
        x_ref(4, 0) = 0.25;

        ChMatrixDynamic<> b(5, 1);
        b(0, 0) = -0.23;
        b(1, 0) = 0.47095;
        b(2, 0) = 0.69725;
        b(3, 0) = 0.08159;
        b(4, 0) = 0.1750;

        ChMatrixDynamic<> x(5, 1);
        int err = A.SolveGeneral(b, x);

        if (err) {
            cout << "  Error in factorization = " << err << endl;
            passed = false;
        }

        double det = A.GetDeterminant();

        cout << "   det_ref = " << det_ref << "  det = " << det << endl;
        cout << "   x_ref = [ " << x_ref(0, 0) << "  " << x_ref(1, 0) << "  " << x_ref(2, 0) << "  " << x_ref(3, 0)
             << "  " << x_ref(4, 0) << " ]" << endl;
        cout << "   x     = [ " << x(0, 0) << "  " << x(1, 0) << "  " << x(2, 0) << "  " << x(3, 0) << "  " << x(4, 0)
             << " ]" << endl;

        if (!x.Equals(x_ref, tolerance) || std::abs(det - det_ref) > tolerance)
            passed = false;
    }

    // Check LU factorization of singular matrix.
    cout << "Test attempt to LU factorize singular matrix" << endl;
    {
        ChLinkedListMatrix A(4, 4);
        A.SetElement(0, 0, 0.5);
        A.SetElement(0, 1, 0.3);
        A.SetElement(0, 2, -0.7);
        A.SetElement(1, 2, 0.2);
        A.SetElement(3, 2, -0.6);
        A.SetElement(3, 3, 0.5);

        int err = A.Setup_LU();
        double det = A.GetDeterminant();

        if (err != 3)
            passed = false;

        cout << "   error code = " << err << endl;
        cout << "   det = " << det << endl;
    }

    // Symmetric linear system solution
    cout << "Test solution of symmetric system Ax=b" << endl;
    {
        ChLinkedListMatrix A(5, 5);
        A.SetElement(0, 0, 0.32);
        A.SetElement(1, 1, -0.14);
        A.SetElement(2, 2, 0.54);
        A.SetElement(3, 3, -0.40);
        A.SetElement(4, 4, 0.38);

        A.SetElement(0, 1, 0.06);
        A.SetElement(0, 4, -0.08);
        A.SetElement(1, 4, -0.82);

        double det_ref = 0.048555072;

        ChMatrixDynamic<> x_ref(5, 1);
        x_ref(0, 0) = 0.34;
        x_ref(1, 0) = 0.58;
        x_ref(2, 0) = 0.23;
        x_ref(3, 0) = 0.75;
        x_ref(4, 0) = 0.25;

        ChMatrixDynamic<> b(5, 1);
        b(0, 0) = 0.1236;
        b(1, 0) = -0.2658;
        b(2, 0) = 0.1242;
        b(3, 0) = -0.3;
        b(4, 0) = -0.4078;

        ChMatrixDynamic<> x(5, 1);
        int err = A.SolveSymmetric(b, x);

        if (err) {
            cout << "  Error in factorization = " << err << endl;
            passed = false;
        }

        double det = A.GetDeterminant();

        cout << "   det_ref = " << det_ref << "  det = " << det << endl;
        cout << "   x_ref = [ " << x_ref(0, 0) << "  " << x_ref(1, 0) << "  " << x_ref(2, 0) << "  " << x_ref(3, 0)
             << "  " << x_ref(4, 0) << " ]" << endl;
        cout << "   x     = [ " << x(0, 0) << "  " << x(1, 0) << "  " << x(2, 0) << "  " << x(3, 0) << "  " << x(4, 0)
             << " ]" << endl;

        if (!x.Equals(x_ref, tolerance) || std::abs(det - det_ref) > tolerance)
            passed = false;
    }

    // Check LDL factorization of singular matrix.
    cout << "Test attempt to LDL factorize singular matrix" << endl;
    {
        ChLinkedListMatrix A(4, 4);
        A.SetElement(0, 0, 0.5);
        A.SetElement(2, 2, 0.3);
        A.SetElement(3, 3, 0.5);

        A.SetElement(0, 2, -0.7);
        A.SetElement(2, 3, -0.6);

        int err = A.Setup_LDL();
        double det = A.GetDeterminant();

        if (err != 1)
            passed = false;

        cout << "   error code = " << err << endl;
        cout << "   det = " << det << endl;
    }

    return passed;
}

bool testMapMatrix() {
    bool passed = true;
    double tolerance = 1e-10;

    cout << endl << "---  MapMatrix ---" << endl;

    // Set and get elements
    cout << "\nTest set and get elements" << endl;
    cout << "-------------------------" << endl;
    ChMapMatrix A(5, 5);
    A.SetElement(1, 0, 0.130);
    A.SetElement(3, 0, 0.012);
    A.SetElement(2, 1, 1);
    A.SetElement(0, 2, -1);
    A.SetElement(3, 2, 0.337);
    A.SetElement(1, 3, 0.569);
    A.SetElement(4, 3, -0.1);
    A.SetElement(2, 4, 0.469);
    A.SetElement(4, 4, 1);
    A.StreamOUTsparseMatlabFormat(GetLog());
    A.StreamOUT(GetLog());

    cout << "\nTest changing existing element" << endl;
    cout << "------------------------------" << endl;
    cout << "   replace A(3,2) = -2" << endl;
    A.SetElement(3, 2, -2);
    A.StreamOUTsparseMatlabFormat(GetLog());
    cout << "   increment A(1,3) += 1" << endl;
    A.SetElement(1, 3, 1, false);
    A.StreamOUTsparseMatlabFormat(GetLog());

    cout << "\nTest copy constructor" << endl;
    cout << "---------------------" << endl;
    ChMapMatrix B(A);
    B.StreamOUTsparseMatlabFormat(GetLog());

    cout << "\nTest conversion to CSR" << endl;
    cout << "----------------------" << endl;
    std::vector<int> ia;
    std::vector<int> ja;
    std::vector<double> a;
    A.ConvertToCSR(ia, ja, a);
    cout << " IA: ";
    for (auto i : ia)
        cout << i << "  ";
    cout << endl << " JA: ";
    for (auto i : ja)
        cout << i << "  ";
    cout << endl << "  A: ";
    for (auto i : a)
        cout << i << "  ";
    cout << endl;

    cout << "\nTest conversion to dense matrix" << endl;
    cout << "-------------------------------" << endl;
    ChMatrixDynamic<double> Afull;
    A.ConvertToDense(Afull);
    Afull.StreamOUT(GetLog());

    cout << "\nTest constructor from dense matrix" << endl;
    cout << "----------------------------------" << endl;
    ChMapMatrix C(Afull);
    C.StreamOUTsparseMatlabFormat(GetLog());

    return passed;
}

int main(int argc, char* argv[]) {
    bool passed = true;

    passed &= testLinkedListMatrix();
    passed &= testMapMatrix();

    cout << endl << "UNIT TEST " << (passed ? "PASSED" : "FAILED") << endl;
    return !passed;
}
