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
// Authors: Dario Mangoni
// =============================================================================
//
// Basic demo and test for MUMPS Engine
//
// The demo provides basic usage of the ChMumpsEngine (that wraps Mumps)
// also with null pivot detection.
//
// =============================================================================

#include "chrono_mumps/ChMumpsEngine.h"
#include "chrono/core/ChMatrixDynamic.h"

using namespace chrono;

int test_basic() {
    // Define the problem data structures and size:
    // - main matrix in ChCOOMatrix format
    // - rhs/solution vector
    int n = 3;
    ChCOOMatrix mat(n, n, true);
    ChMatrixDynamic<double> rhs_sol(n, 1);
    ChMumpsEngine mumps_engine;

    // Store test rhs values in the vector
    rhs_sol(0) = 2;
    rhs_sol(1) = 1;
    rhs_sol(2) = 3;

    // Store non-zero coefficient in the sparse matrix
    //  compress it after the insertions
    mat.SetElement(0, 0, 1.3);
    mat.SetElement(1, 1, 2.7);
    mat.SetElement(2, 2, 3.9);
    mat.Compress();

    // Inform the Mumps Engine about the problem data structures being used
    mumps_engine.SetProblem(mat, rhs_sol);

    // Launch the solver
    auto return_value = mumps_engine.MumpsCall(ChMumpsEngine::mumps_JOB::COMPLETE);
    mumps_engine.PrintINFOG();
    printf("Mumps return code: %d\n", return_value);

    // Print the solution that has been stored in the same vector where there was the rhs
    for (int i = 0; i < n; i++) {
        printf("%f ", rhs_sol(i));
    }
    printf("\n");

    return return_value;
}

int test_null_pivot() {
    int n = 3;
    ChCOOMatrix mat(n, n, true);
    ChMatrixDynamic<double> rhs_sol(n, 1);
    ChMumpsEngine mumps_engine;

    rhs_sol(0) = 2;
    rhs_sol(1) = 1;
    rhs_sol(2) = 3;

    mat.SetElement(0, 0, 1.3);
    mat.SetElement(1, 1, 2.7);
    mat.SetElement(2, 2, 0);  // make the matrix rank-deficient
    mat.Compress();

    mumps_engine.SetProblem(mat, rhs_sol);
    mumps_engine.SetNullPivotDetection(true);  // activate the null pivot detection
    auto return_value = mumps_engine.MumpsCall(ChMumpsEngine::mumps_JOB::COMPLETE);
    mumps_engine.PrintINFOG();
    printf("Mumps return code: %d\n", return_value);

    for (int i = 0; i < n; i++) {
        printf("%f ", rhs_sol(i));
    }
    printf("\n");

    return return_value;
}

int main() {
    auto error_test_basic = test_basic();
    auto error_test_null_pivot = test_null_pivot();

    return error_test_basic || error_test_null_pivot;
}