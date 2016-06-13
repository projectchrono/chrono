//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   Demo on how to use Chrono mathematical
//   functions (vector math, linear algebra, etc)
//
//	 CHRONO
//   ------
//   Multibody dinamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
#include "core/ChTransform.h"
#include "core/ChMatrix.h"
#include "core/ChLog.h"
#include "core/ChVector.h"
#include "core/ChQuadrature.h"
#include "core/ChException.h"
#include "core/ChTimer.h"

using namespace chrono;

int main(int argc, char* argv[]) {
    GetLog() << "CHRONO foundation classes test: math\n\n";
    ChTimer<double> timer;

    bool printMul = true;
    bool MatrMultiply = true;
    bool MatrMultiplyT = true;

    int ITERATION = 10000;

    if (MatrMultiply) {
        int A_row = 33;
        int A_col = 9;
        int B_row = A_col;
        int B_col = 33;
        ChMatrixDynamic<double> A(A_row, A_col);
        ChMatrixDynamic<double> B(B_row, B_col);  // For Multiplication
        ChMatrixDynamic<double> C(A_row, A_col);  // For add/sub
        ChMatrixDynamic<double> AmulB(A_row, B_col);
        ChMatrixDynamic<double> AmulB_ref(A_row, B_col);
        ChMatrixDynamic<double> AAddC(A_row, A_col);
        ChMatrixDynamic<double> AAddC_ref(A_row, A_col);

        A.FillRandom(10, -10);  // Fill a matrix with an element
        B.FillRandom(10, -10);  // Fill a matrix with an element

        GetLog() << "-----------------MatrMultiply---------------------- \n";
        timer.start();
        for (int j = 0; j < ITERATION; j++)
            AmulB_ref.MatrMultiply(A, B);
        timer.stop();

        double tempTime = timer();
        GetLog() << "The MatrMultiply results in " << timer() << " (s), ";
        timer.reset();
        timer.start();
        for (int j = 0; j < ITERATION; j++) {
            AmulB.MatrMultiplyAVX(A, B);
        }
        timer.stop();
        double AVXTime = timer();
        GetLog() << "The AVX results in " << timer() << " (s) \n";
        GetLog() << "Speed up =  " << tempTime / AVXTime << "x \n";

        if (printMul) {
            GetLog() << "--------------------------------------- \n";
            GetLog() << "AVX-Ref result is : ";
            (AmulB - AmulB_ref).StreamOUT(GetLog());  // Print a matrix to cout (ex. the console, if open)
            GetLog() << "--------------------------------------- \n";
        }
        if (AmulB_ref == AmulB) {
            GetLog() << "MatrMultiplyAVX is Ok ... \n\n";
        } else {
            GetLog() << "MatrMultiplyTAVX is not Ok! \n\n";
        }
    }

    if (MatrMultiplyT) {
        int A_row = 5;
        int A_col = 20;
        int B_row = 5;
        int B_col = A_col;

        ChMatrixDynamic<double> A(A_row, A_col);
        ChMatrixDynamic<double> B(B_row, B_col);  // For Multiplication
        ChMatrixDynamic<double> C(A_row, A_col);  // For add/sub
        ChMatrixDynamic<double> AmulB(A_row, B_row);
        ChMatrixDynamic<double> AmulB_ref(A_row, B_row);

        A.FillRandom(10, -10);  // Fill a matrix with an element
        B.FillRandom(10, -10);  // Fill a matrix with an element
        GetLog() << "-----------------MatrMultiplyT---------------------- \n";
        timer.reset();
        timer.start();
        for (int j = 0; j < ITERATION; j++)
            AmulB_ref.MatrMultiplyT(A, B);
        timer.stop();

        double tempTime = timer();
        GetLog() << "The MatrMultiplyT results in " << timer() << " (s), ";
        timer.reset();
        timer.start();
        for (int j = 0; j < ITERATION; j++)
            AmulB.MatrMultiplyTAVX(A, B);
        timer.stop();
        double AVXTime = timer();
        GetLog() << "The AVX results in " << timer() << " (s) \n";
        GetLog() << "Speed up =  " << tempTime / AVXTime << "x \n";

        if (printMul) {
            GetLog() << "--------------------------------------- \n";
            GetLog() << "AVX-Ref result is : ";
            (AmulB - AmulB_ref).StreamOUT(GetLog());  // Print a matrix to cout (ex. the console, if open)
                                                      //        GetLog() << "Reference result is : ";
            //        AmulB_ref.StreamOUT(GetLog());
            GetLog() << "--------------------------------------- \n";
        }
        if ((AmulB_ref == AmulB)) {
            GetLog() << "MatrMultiplyTAVX is Ok ... \n\n";
        } else {
            GetLog() << "MatrMultiplyTAVX is not Ok! \n\n";
        }
    }

    return 0;
}
