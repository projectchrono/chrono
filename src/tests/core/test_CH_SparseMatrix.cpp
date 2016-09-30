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
// Authors: Radu Serban
// =============================================================================

#include <vector>
#include <random>
#include <functional>
#include <algorithm>
#include <iostream>

#include "chrono/core/ChTimer.h"
#include "chrono/core/ChMatrixDynamic.h"
#include "chrono/core/ChLinkedListMatrix.h"
#include "chrono/core/ChMapMatrix.h"
#include "chrono/core/ChCSR3Matrix.h"

using namespace chrono;
using std::cout;
using std::endl;

void timeSetElement() {
    cout << "-----------------------------------------------------" << endl;
    cout << "SparseMatrix Test: inserting elements in random order" << endl;
    cout << "-----------------------------------------------------" << endl;

    ChTimer<> timer;

    // Generate randomized row-column indices in a sparse matrix.
    int n = 10000;
    int nnz = (n * n) * 0.05;

    cout << "N   = " << n << endl;
    cout << "NNZ = " << nnz << endl << endl;

    std::random_device rd;  // random device
    std::mt19937 mt(rd());  // random engine

    std::uniform_int_distribution<int> dist(0, n - 1);
    auto gen = std::bind(dist, mt);
    std::vector<int> row_indices(nnz);
    std::vector<int> col_indices(nnz);

    for (int i = 0; i < nnz; i++) {
        row_indices[i] = gen();
        col_indices[i] = gen();
    }

    /*
    // Insert non-zero elements in a LinkedList sparse matrix.
    ChLinkedListMatrix A(n, n);
    timer.reset();
    timer.start();
    for (int i = 0; i < nnz; i++) {
    A.SetElement(row_indices[i], col_indices[i], 1.0);
    }
    timer.stop();
    cout << "Time LinkedList matrix: " << timer() << endl;
    */


    {
        // -----------------------------------------
        // Insert non-zero elements in a CSR3 matrix
        // without using sparsity pattern lock
        // -----------------------------------------
        cout << "ChCSR3Matrix" << endl;
        ChCSR3Matrix B(n, n);

        cout << "   First insertion: " << nnz << " values" << endl;
        timer.reset();
        timer.start();
        //B.ExportToDatFile("a", 6);
        for (int i = 0; i < nnz; i++) {
            B.SetElement(row_indices[i], col_indices[i], 1.0);
        }
        B.Compress();
        timer.stop();
        cout << "      NNZ:  " << B.GetNNZ() << endl;
        cout << "      Time: " << timer() << endl;

        cout << "   Second insertion w/o sparsity lock: " << nnz << " values" << endl;
        B.Reset(n, n);
        timer.reset();
        timer.start();
        //B.ExportToDatFile("b", 6);
        for (int i = 0; i < nnz; i++) {
            B.SetElement(row_indices[i], col_indices[i], 2.0);
        }
        B.Compress();
        timer.stop();
        cout << "      NNZ:  " << B.GetNNZ() << endl;
        cout << "      Time: " << timer() << endl;
        cout << endl;

    }

    {
        // -----------------------------------------
        // Insert non-zero elements in a CSR3 matrix
        // using sparsity pattern lock
        // -----------------------------------------
        cout << "ChCSR3Matrix" << endl;
        ChCSR3Matrix B(n, n);

        cout << "   First insertion: " << nnz << " values" << endl;
        timer.reset();
        timer.start();
        //B.ExportToDatFile("a", 6);
        for (int i = 0; i < nnz; i++) {
            B.SetElement(row_indices[i], col_indices[i], 1.0);
        }
        B.Compress();
        timer.stop();
        cout << "      NNZ:  " << B.GetNNZ() << endl;
        cout << "      Time: " << timer() << endl;

        B.SetSparsityPatternLock(true);


        cout << "   Second insertion with sparsity lock: " << nnz << " values" << endl;
        B.Reset(n, n);
        timer.reset();
        timer.start();
        //B.ExportToDatFile("b", 6);
        for (int i = 0; i < nnz; i++) {
            B.SetElement(row_indices[i], col_indices[i], 2.0);
        }
        B.Compress();
        timer.stop();
        cout << "      NNZ:  " << B.GetNNZ() << endl;
        cout << "      Time: " << timer() << endl;
        cout << endl;

    }

    {
        // -----------------------------------------
        // Insert non-zero elements in a ChSparsityPatternLearner matrix
        // and then to CSR3 with sparsity pattern lock
        // -----------------------------------------
        cout << "ChSparsityPatternLearner" << endl;
        ChSparsityPatternLearner SL(n, n);

        cout << " ...inserting into ChSparsityPatternLearner: " << nnz << " values" << endl;
        timer.reset();
        timer.start();
        //B.ExportToDatFile("a", 6);
        for (int i = 0; i < nnz; i++) {
            SL.SetElement(row_indices[i], col_indices[i], 1.0);
        }
        SL.Compress();
        timer.stop();
        cout << "      NNZ:  " << SL.GetNNZ() << endl;
        cout << "      Time: " << timer() << endl;


        ChCSR3Matrix B(n, n);
        B.LoadSparsityPattern(SL);

        B.SetSparsityPatternLock(true);

        cout << " ...inserting into ChCSR3Matrix with sparsity lock: " << nnz << " values" << endl;
        B.Reset(n, n);
        timer.reset();
        timer.start();
        //B.ExportToDatFile("b", 6);
        for (int i = 0; i < nnz; i++) {
            B.SetElement(row_indices[i], col_indices[i], 2.0);
        }
        B.Compress();
        timer.stop();
        cout << "      NNZ:  " << B.GetNNZ() << endl;
        cout << "      Time: " << timer() << endl;
        cout << endl;
    }

    {
        // -----------------------------------------
        // Insert non-zero elements in a Map matrix
        // -----------------------------------------
        cout << "MapMatrix" << endl;
        ChMapMatrix C(n, n);
        std::vector<int> ia;
        std::vector<int> ja;
        std::vector<double> vals;

        cout << "   First insertion: " << nnz << " values" << endl;
        timer.reset();
        timer.start();
        for (int i = 0; i < nnz; i++) {
            C.SetElement(row_indices[i], col_indices[i], 1.0);
        }
        C.ConvertToCSR(ia, ja, vals);
        timer.stop();
        cout << "      NNZ:  " << C.GetNNZ() << endl;
        cout << "      Time: " << timer() << endl;

        cout << "   Second insertion: " << nnz << " values" << endl;
        C.Reset(n, n);
        timer.reset();
        timer.start();
        for (int i = 0; i < nnz; i++) {
            C.SetElement(row_indices[i], col_indices[i], 2.0);
        }
        C.ConvertToCSR(ia, ja, vals);
        timer.stop();
        cout << "      NNZ:  " << C.GetNNZ() << endl;
        cout << "      Time: " << timer() << endl;
        cout << endl;
    }

}



int main() {
    timeSetElement();
}
