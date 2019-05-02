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
// Authors: Dario Mangoni, Radu Serban
// =============================================================================

#include "gtest/gtest.h"

#include "chrono/core/ChCSMatrix.h"
#include "chrono/core/ChMatrixDynamic.h"

using namespace chrono;

void PrintMatrix(const ChMatrixDynamic<>& mat) {
    for (int ii = 0; ii < mat.GetRows(); ii++) {
        for (int jj = 0; jj < mat.GetColumns(); jj++)
            std::cout << mat.GetElement(ii, jj) << "\t";
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void PrintMatrix(ChSparseMatrix& mat) {
    for (int ii = 0; ii < mat.GetNumRows(); ii++) {
        for (int jj = 0; jj < mat.GetNumColumns(); jj++)
            std::cout << mat.GetElement(ii, jj) << "\t";
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void PrintMatrixCS(const ChCSMatrix& mat) {
    int ncols = mat.GetNumColumns();
    int nrows = mat.GetNumRows();
    int* rowindex = mat.GetCS_LeadingIndexArray();
    int* colindex = mat.GetCS_TrailingIndexArray();
    double* values = mat.GetCS_ValueArray();
    int nnz = rowindex[nrows];

    std::cout << "Num. rows: " << nrows << std::endl;
    std::cout << "Num. cols: " << ncols << std::endl;
    std::cout << "NNZ:       " << nnz << std::endl;
    std::cout << "Row indexes: ";
    for (int i = 0; i <= nrows; i++)
        std::cout << rowindex[i] << "  ";
    std::cout << std::endl;
    std::cout << "Column indexes: ";
    for (int i = 0; i < nnz; i++)
        std::cout << colindex[i] << "  ";
    std::cout << std::endl;
    std::cout << "Values: ";
    for (int i = 0; i < nnz; i++)
        std::cout << values[i] << "  ";
    std::cout << std::endl << std::endl;
}

// true if matrices equal
template <typename mat_t>
bool CompareMatrix(const ChCSMatrix& mat1, const mat_t& mat2) {
    for (int i = 0; i < mat1.GetNumRows(); i++)
        for (int j = 0; j < mat1.GetNumColumns(); j++) {
            if (mat1.GetElement(i, j) != mat2.GetElement(i, j)) {
                return false;
            }
        }

    return true;
}

// true if matrices equal
bool CompareMatrix(const ChCSMatrix& mat1, const ChCSMatrix& mat2, bool tolerate_uncompressed) {
    auto rows = mat1.GetNumRows();
    auto rows_temp = mat2.GetNumRows();

    if (rows_temp != rows) {
        std::cout << "Number of rows do not match: ";
        std::cout << "   mat1 -> " << rows;
        std::cout << "   mat2 -> " << rows_temp << std::endl;
        return false;
    }

    if (!CompareMatrix(mat1, mat2))
        return false;

    for (int cont = 0; cont <= rows; cont++) {
        if (mat1.GetCS_LeadingIndexArray()[cont] != mat2.GetCS_LeadingIndexArray()[cont]) {
            std::cout << "Row indexes do not match at entry " << cont << ":";
            std::cout << "   mat1 -> " << mat1.GetCS_LeadingIndexArray()[cont];
            std::cout << "   mat2 -> " << mat2.GetCS_LeadingIndexArray()[cont] << std::endl;
            return false;
        }
    }

    for (int cont = 0; cont < mat1.GetCS_LeadingIndexArray()[rows]; cont++) {
        if (mat1.GetCS_TrailingIndexArray()[cont] != mat2.GetCS_TrailingIndexArray()[cont]) {
            std::cout << "Column indexes do not match at entry " << cont << ":";
            std::cout << "   mat1 -> " << mat1.GetCS_TrailingIndexArray()[cont];
            std::cout << "   mat2 -> " << mat2.GetCS_TrailingIndexArray()[cont] << std::endl;
            return false;
        }

        if (mat1.GetCS_TrailingIndexArray()[cont] != -1 && mat2.GetCS_TrailingIndexArray()[cont] != -1) {
            if (mat1.GetCS_ValueArray()[cont] != mat2.GetCS_ValueArray()[cont]) {
                std::cout << "Values do not match at entry " << cont << ":";
                std::cout << "   mat1 -> " << mat1.GetCS_ValueArray()[cont];
                std::cout << "   mat2 -> " << mat2.GetCS_ValueArray()[cont] << std::endl;
                return false;
            }
        } else if (!tolerate_uncompressed)
            return false;
    }

    return true;
}

// ------------------------------

TEST(ChCSMatrixTest, sparsity_lock) {
    const int n = 4;
    ChCSMatrix mat(n, n, true, 15);
    mat.SetSparsityPatternLock(true);

    mat.SetElement(0, 0, 10.0);
    mat.SetElement(0, 1, 0.1);
    mat.SetElement(1, 1, 1.1);
    mat.SetElement(1, 2, 1.2);
    mat.SetElement(2, 1, 2.1);
    mat.SetElement(2, 2, 2.2);
    mat.SetElement(3, 0, 3.0);

    std::vector<int> rowIndex_hardcoded = {0, 2, 4, 6, 7};
    std::vector<int> colIndex_hardcoded = {0, 1, 1, 2, 1, 2, 0};
    std::vector<double> values_hardcoded = {10, 0.1, 1.1, 1.2, 2.1, 2.2, 3};

    mat.Compress();

    double* a = mat.GetCS_ValueArray();
    int* ia = mat.GetCS_LeadingIndexArray();
    int* ja = mat.GetCS_TrailingIndexArray();

    for (auto row_sel = 0; row_sel <= n; ++row_sel) {
        ASSERT_EQ(ia[row_sel], rowIndex_hardcoded[row_sel]);
    }

    for (auto col_sel = 0; col_sel < rowIndex_hardcoded[n]; ++col_sel) {
        ASSERT_EQ(ja[col_sel], colIndex_hardcoded[col_sel]);
        ASSERT_EQ(a[col_sel], values_hardcoded[col_sel]);
    }
}

TEST(ChCSMatrixTest, compress) {
    ChCSMatrix mat(3, 3, true, 6);
    mat.SetSparsityPatternLock(true);

    mat.SetElement(0, 0, 10.0);
    mat.SetElement(2, 2, 2.2);
    mat.SetElement(1, 1, 1.1);
    mat.SetElement(0, 1, 0.1);
    mat.SetElement(2, 1, 2.1);
    mat.SetElement(1, 2, 1.2);

    ChMatrixDynamic<double> matDYN(3, 3);
    matDYN.Reset();
    
    matDYN.SetElement(0, 0, 10.0);
    matDYN.SetElement(2, 2, 2.2);
    matDYN.SetElement(1, 1, 1.1);
    matDYN.SetElement(0, 1, 0.1);
    matDYN.SetElement(2, 1, 2.1);
    matDYN.SetElement(1, 2, 1.2);

    ASSERT_TRUE(CompareMatrix(mat, matDYN));

    mat.Compress();

    ASSERT_TRUE(CompareMatrix(mat, matDYN));
}

TEST(ChCSMatrixTest, column_major) {
    int n = 3;

    ChCSMatrix matCM(n, n, false);
    matCM.SetElement(0, 0, 3);
    matCM.SetElement(0, 1, 5);
    matCM.SetElement(1, 0, 7);
    matCM.SetElement(1, 2, 9);
    matCM.SetElement(2, 2, 11);

    ChCSMatrix matRM(n, n, true);
    matRM.SetElement(0, 0, 3);
    matRM.SetElement(0, 1, 5);
    matRM.SetElement(1, 0, 7);
    matRM.SetElement(1, 2, 9);
    matRM.SetElement(2, 2, 11);

    ////PrintMatrix(matCM);
    ////PrintMatrix(matRM);

    ASSERT_TRUE(CompareMatrix(matCM, matRM));

    matRM.Compress();
    matCM.Compress();

    ASSERT_TRUE(CompareMatrix(matCM, matRM));
}

TEST(ChCSMatrixTest, mat_mult) {
    ChMatrixDynamic<double> matB(3, 2);
    matB.SetElement(0, 0, 7.1);
    matB.SetElement(1, 0, 3.7);
    matB.SetElement(2, 0, 9.4);
    matB.SetElement(0, 1, 6.2);
    matB.SetElement(1, 1, 5.2);
    matB.SetElement(2, 1, 7.1);

    ChCSMatrix matA_rm(3, 3, true);   // row-major format
    matA_rm.SetElement(0, 0, 3.1);
    matA_rm.SetElement(0, 1, 2.4);
    matA_rm.SetElement(1, 1, 9.2);
    matA_rm.SetElement(2, 2, 5.6);

    ChCSMatrix matA_cm(3, 3, false);  // column-major format
    matA_cm = matA_rm;

    ASSERT_TRUE(CompareMatrix(matA_cm, matA_rm));

    // Test A * B
    ChMatrixDynamic<double> mat_outR(3, 2), mat_outC(3, 2);
    matA_rm.MatrMultiply(matB, mat_outR, false);
    matA_cm.MatrMultiply(matB, mat_outC, false);
    ////PrintMatrix(mat_outR);
    ////PrintMatrix(mat_outC);
    ASSERT_TRUE(mat_outR.Equals(mat_outC));

    // Test A' * B
    matA_rm.MatrMultiply(matB, mat_outR, true);
    matA_cm.MatrMultiply(matB, mat_outC, true);
    ////PrintMatrix(mat_outR);
    ////PrintMatrix(mat_outC);
    ASSERT_TRUE(mat_outR.Equals(mat_outC));
}

TEST(ChCSMatrixTest, mat_mult_clipped) {
    ChCSMatrix matA_cm(6, 4);

    matA_cm.SetElement(0, 0, 1.7);
    matA_cm.SetElement(0, 1, 1.3);
    matA_cm.SetElement(0, 3, 0.6);

    matA_cm.SetElement(1, 0, 0.7);
    matA_cm.SetElement(1, 1, 0.8);
    matA_cm.SetElement(1, 2, 0.1);

    matA_cm.SetElement(2, 2, 2.4);

    matA_cm.SetElement(3, 2, 5.1);
    matA_cm.SetElement(4, 2, 8.2);
    matA_cm.SetElement(5, 2, 4.4);

    ChCSMatrix matA_cmT(4, 6);

    matA_cmT.SetElement(0, 0, 1.7);
    matA_cmT.SetElement(1, 0, 1.3);
    matA_cmT.SetElement(3, 0, 0.6);

    matA_cmT.SetElement(0, 1, 0.7);
    matA_cmT.SetElement(1, 1, 0.8);
    matA_cmT.SetElement(2, 1, 0.1);

    matA_cmT.SetElement(2, 2, 2.4);

    matA_cmT.SetElement(2, 3, 5.1);
    matA_cmT.SetElement(2, 4, 8.2);
    matA_cmT.SetElement(2, 5, 4.4);

    ChMatrixDynamic<double> matB(6, 8);
    matB.Reset();

    matB.SetElement(2, 0, 1.7);
    matB.SetElement(3, 0, 1.3);
    matB.SetElement(5, 0, 0.6);

    matB.SetElement(2, 1, 0.7);
    matB.SetElement(3, 1, 0.8);
    matB.SetElement(4, 1, 0.1);

    matB.SetElement(4, 2, 2.4);

    matB.SetElement(4, 3, 5.1);
    matB.SetElement(4, 4, 8.2);
    matB.SetElement(4, 5, 4.4);

    ////std::cout << "matA_cm" << std::endl;
    ////PrintMatrix(matA_cm);
    ////std::cout << "matA_cmT" << std::endl;
    ////PrintMatrix(matA_cmT);
    ////std::cout << "matB" << std::endl;
    ////PrintMatrix(matB);

    ChMatrixDynamic<double> mat_out1(8, 6);
    mat_out1.Reset();
    matA_cm.MatrMultiplyClipped(matB, mat_out1, 1, 3, 1, 2, 3, 1, false, 1, 3, 1);
    ////std::cout << "mat_out1" << std::endl;
    ////PrintMatrix(mat_out1);

    ChMatrixDynamic<double> mat_out2(8, 6);
    mat_out2.Reset();
    matA_cm.MatrMultiplyClipped(matB, mat_out2, 1, 3, 1, 2, 3, 1, true, 1, 3, 1);
    ////std::cout << "mat_out2" << std::endl;
    PrintMatrix(mat_out2);

    ChMatrixDynamic<double> mat_out3(8, 6);
    mat_out3.Reset();
    matA_cmT.MatrMultiplyClipped(matB, mat_out3, 1, 3, 1, 2, 3, 1, false, 1, 3, 1);
    ////std::cout << "mat_out3" << std::endl;
    PrintMatrix(mat_out3);

    ASSERT_TRUE(mat_out2.Equals(mat_out3));
}
