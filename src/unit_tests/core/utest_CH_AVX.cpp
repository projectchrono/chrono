//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
// =============================================================================
// Authors: Milad Rakhsha, Radu Serban
// =============================================================================
// unit test for MatrMultiplyAVX and MatrMultiplyTAVX.
// =============================================================================
#include "core/ChTransform.h"
#include "core/ChMatrix.h"
#include "core/ChLog.h"
#include "core/ChVector.h"
#include "core/ChQuadrature.h"
#include "core/ChException.h"
#include "core/ChTimer.h"
using namespace chrono;
void FillRand(ChMatrix<double>& matra) {
    int A_Nrow = matra.GetRows();
    int A_NCol = matra.GetColumns();

    for (int rowA = 0; rowA < A_Nrow; rowA++) {
        for (int colA = 0; colA < A_NCol; colA++) {
            matra.SetElement(rowA, colA, double(rand() % 100) / 100);
        }
    }
}

int main(int argc, char* argv[]) {
    // If you want to pollute the cache line to see if this affects the speed up or not
    bool polluteCache = false;
    // If you want to print the difference between AVX implementation and non-AVX implementation
    // This should print zeros since you are printing results of (AVX - nonAVX) implementation.
    bool printMul = true;
    bool MatrMultiply = true;
    bool MatrMultiplyT = true;
    // This is the number of repetition of the operation. Remember that timing only one operation is not accurate
    int ITERATION = 10000;

    ChTimer<double> timer;
    srand(time(NULL));
    if (MatrMultiply) {
        int A_row = 24;
        int A_col = 6;
        int B_row = A_col;
        int B_col = 24;
        ChMatrixDynamic<double> A(A_row, A_col);
        ChMatrixDynamic<double> B(B_row, B_col);
        ChMatrixDynamic<double> AmulB(A_row, B_col);
        ChMatrixDynamic<double> AmulB_ref(A_row, B_col);

        if (polluteCache) {
            GetLog() << "\nCache is polluted by changing the element of matrices.\n";
            GetLog() << "No report for the correctness of the operation...\n";
            GetLog() << "==================================================\n\n";
            GetLog() << "Testing the FillRand()... \n";
            GetLog() << "The first call to FillRand(A): ";
            FillRand(A);
            (A).StreamOUT(GetLog());  // Print a matrix to cout (ex. the console, if open)
            GetLog() << "The second call to FillRand(A): ";
            FillRand(A);
            (A).StreamOUT(GetLog());  // Print a matrix to cout (ex. the console, if open)
        }
        GetLog() << "-----------------MatrMultiply---------------------- \n";
        timer.reset();
        timer.start();
        for (int j = 0; j < ITERATION; j++) {
            if (polluteCache) {
                timer.stop();
                FillRand(A);
                FillRand(B);
                timer.start();
            }
            AmulB.MatrMultiply(A, B);
        }
        timer.stop();

        double tempTime = timer();
        GetLog() << "The MatrMultiply results in " << timer() << " (s), ";
        timer.reset();
        timer.start();
        for (int j = 0; j < ITERATION; j++) {
            if (polluteCache) {
                timer.stop();
                FillRand(A);
                FillRand(B);
                timer.start();
            }
            AmulB.MatrMultiplyAVX(A, B);
        }
        timer.stop();
        double AVXTime = timer();
        GetLog() << "The AVX results in " << timer() << " (s) \n";
        GetLog() << "Speed up =  " << tempTime / AVXTime << "x \n";

        // Let's not compare the results if polluteCache=true; since A and B elements have been changed...
        if (printMul && !polluteCache) {
            GetLog() << "--------------------------------------- \n";
            GetLog() << "AVX-Ref result is : ";
            (AmulB - AmulB_ref).StreamOUT(GetLog());  // Print a matrix to cout (ex. the console, if open)
            GetLog() << "--------------------------------------- \n";
        }
        // if polluteCache=true let's assume the AVX works Okay
        if (AmulB_ref == AmulB && !polluteCache) {
            GetLog() << "MatrMultiplyAVX is Ok ... \n\n";
        } else if (!polluteCache) {
            GetLog() << "MatrMultiplyTAVX is not Ok! \n\n";
        }
    }

    if (MatrMultiplyT) {
        int A_row = 5;
        int A_col = 20;
        int B_row = 5;
        int B_col = A_col;

        ChMatrixDynamic<double> A(A_row, A_col);
        ChMatrixDynamic<double> B(B_row, B_col);
        ChMatrixDynamic<double> AmulB(A_row, B_row);
        ChMatrixDynamic<double> AmulB_ref(A_row, B_row);

        A.FillRandom(10, -10);
        B.FillRandom(10, -10);
        GetLog() << "-----------------MatrMultiplyT---------------------- \n";
        timer.reset();
        timer.start();
        for (int j = 0; j < ITERATION; j++) {
            if (polluteCache) {
                timer.stop();
                FillRand(A);
                FillRand(B);
                timer.start();
            }
            AmulB_ref.MatrMultiplyT(A, B);
        }
        timer.stop();

        double tempTime = timer();
        GetLog() << "The MatrMultiplyT results in " << timer() << " (s), ";
        timer.reset();
        timer.start();
        for (int j = 0; j < ITERATION; j++) {
            if (polluteCache) {
                timer.stop();
                FillRand(A);
                FillRand(B);
                timer.start();
            }
            AmulB.MatrMultiplyTAVX(A, B);
        }
        timer.stop();
        double AVXTime = timer();
        GetLog() << "The AVX results in " << timer() << " (s) \n";
        GetLog() << "Speed up =  " << tempTime / AVXTime << "x \n";
        // Let's not compare the results if polluteCache=true; since A and B elements have been changed...
        if (printMul && !polluteCache) {
            GetLog() << "--------------------------------------- \n";
            GetLog() << "AVX-Ref result is : ";
            (AmulB - AmulB_ref).StreamOUT(GetLog());
            GetLog() << "--------------------------------------- \n";
        }
        // if polluteCache=true let's assume the AVX works Okay
        if ((AmulB_ref == AmulB && !polluteCache)) {
            GetLog() << "MatrMultiplyTAVX is Ok ... \n\n";
        } else if (!polluteCache) {
            GetLog() << "MatrMultiplyTAVX is not Ok!? \n\n";
        }
    }

    return 0;
}
