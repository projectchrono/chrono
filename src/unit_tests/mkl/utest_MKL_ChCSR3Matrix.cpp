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
// Authors: Dario Mangoni, Radu Serban
// =============================================================================

#include "chrono_mkl/ChCSR3Matrix.h"
#include "chrono/core/ChMatrixDynamic.h"

using namespace chrono;

template <class matrix_t>
void PrintMatrix(matrix_t& mat) {
    for (int ii = 0; ii < mat.GetRows(); ii++) {
        for (int jj = 0; jj < mat.GetColumns(); jj++)
            std::cout << mat.GetElement(ii, jj) << "\t";
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void PrintMatrixCSR(const ChCSR3Matrix& mat) {
    int ncols = mat.GetColumns();
    int nrows = mat.GetRows();
    int* rowindex = mat.GetRowIndexAddress();
    int* colindex = mat.GetColIndexAddress();
    double* values = mat.GetValuesAddress();
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

bool CompareArrays(const ChCSR3Matrix& mat1, const ChCSR3Matrix& mat2, bool tolerate_uncompressed) {
    int rows = mat1.GetRows();
    int rows_temp = mat2.GetRows();

    if (rows_temp != rows) {
        std::cout << "Number of rows do not match: ";
        std::cout << "   mat1 -> " << rows;
        std::cout << "   mat2 -> " << rows_temp << std::endl;
        return false;
    }

    for (int cont = 0; cont <= rows; cont++) {
        if (mat1.GetRowIndexAddress()[cont] != mat2.GetRowIndexAddress()[cont]) {
            std::cout << "Row indexes do not match at entry " << cont << ":";
            std::cout << "   mat1 -> " << mat1.GetRowIndexAddress()[cont];
            std::cout << "   mat2 -> " << mat2.GetRowIndexAddress()[cont] << std::endl;
            return false;
        }
    }

    for (int cont = 0; cont < mat1.GetRowIndexAddress()[rows]; cont++) {

        if (mat1.GetColIndexAddress()[cont] != mat2.GetColIndexAddress()[cont]) {
            std::cout << "Column indexes do not match at entry " << cont << ":";
            std::cout << "   mat1 -> " << mat1.GetColIndexAddress()[cont];
            std::cout << "   mat2 -> " << mat2.GetColIndexAddress()[cont] << std::endl;
            return false;
        }

        if (mat1.GetColIndexAddress()[cont] != -1 && mat2.GetColIndexAddress()[cont] != -1)
        {
            if (mat1.GetValuesAddress()[cont] != mat2.GetValuesAddress()[cont]) {
                std::cout << "Values do not match at entry " << cont << ":";
                std::cout << "   mat1 -> " << mat1.GetValuesAddress()[cont];
                std::cout << "   mat2 -> " << mat2.GetValuesAddress()[cont] << std::endl;
                return false;
            }
        }
        else
            if (!tolerate_uncompressed)
                return false;

    }

    return true;
}

template <class matrixIN, class matrixOUT>
void FillMatrix(matrixOUT& mat_out, const matrixIN& mat_in) {
    // assert(mat_out.GetRows() == mat_in.GetRows());
    // assert(mat_out.GetColumns() == mat_in.GetColumns());
    for (int m_sel = 0; m_sel < mat_in.GetRows(); m_sel++) {
        for (int n_sel = 0; n_sel < mat_in.GetColumns(); n_sel++) {
            if (mat_in.GetElement(m_sel, n_sel) != 0) {
                ////std::cout << "(" << m_sel << "," << n_sel << ") = " << mat_in.GetElement(m_sel, n_sel) << std::endl;
                mat_out.SetElement(m_sel, n_sel, mat_in.GetElement(m_sel, n_sel));
            }
        }
    }
}

int main() {
    int* nonzeros_vector = nullptr;
    int m = 5;
    int n = 3;

    ChMatrixDynamic<double> ref_matrix(m, n);
    ref_matrix(0, 1) = 0.1;
    ref_matrix(1, 2) = 1.2;
    ref_matrix(2, 0) = 2.0;
    ref_matrix(2, 1) = 2.1;
    ref_matrix(3, 1) = 3.1;
    ref_matrix(4, 0) = 4.0;
    ref_matrix(4, 1) = 4.1;

    std::cout << "Reference matrix (dense)" << std::endl;
    PrintMatrix(ref_matrix);

    // ----------------------------------------------------------------------------

    std::cout << "*********** Test 1: compare dense against CSR3 matrix ***********" << std::endl;
    ChCSR3Matrix matCSR3(m, n);
    FillMatrix(matCSR3, ref_matrix);

    std::cout << "matCSR3" << std::endl;
    PrintMatrix(matCSR3);
    PrintMatrixCSR(matCSR3);

    std::cout << "Compare base matrix and matCSR3" << std::endl;
    bool test1 = true;
    for (int m_sel = 0; m_sel < m; m_sel++)
		for (int n_sel = 0; n_sel < n; n_sel++)
			if (ref_matrix(m_sel, n_sel) != matCSR3.GetElement(m_sel, n_sel))
				test1 = false;
    std::cout << "Test 1: " << (test1 ? "passed" : "NOT passed") << std::endl << std::endl << std::endl;

    // ----------------------------------------------------------------------------

    std::cout << "*********** Test 2: initialization from nonzeros distribution vector ***********" << std::endl;
    nonzeros_vector = new int[matCSR3.GetRows()];
    matCSR3.GetNonZerosDistribution(nonzeros_vector);
    std::cout << "NNZ distribution in matCSR3:  " << std::endl;
    for (int i = 0; i < matCSR3.GetRows(); i++)
        std::cout << nonzeros_vector[i] << "  ";
    std::cout << std::endl;

    ChCSR3Matrix matCSR3_1(m, n, nonzeros_vector);
    delete nonzeros_vector;
    FillMatrix(matCSR3_1, ref_matrix);
  
    std::cout << "Check matrix builded from nonzeros distribution vector" << std::endl;
    bool test2 = CompareArrays(matCSR3_1, matCSR3, true);
    if (!test2)
    {
        std::cout << "Compare matCSR3 vs matCSR3_1" << std::endl;
        PrintMatrixCSR(matCSR3);
        PrintMatrixCSR(matCSR3_1);
    }
    std::cout << "Test 2: " << (test2 ? "passed" : "NOT passed") << std::endl << std::endl << std::endl;

    // ----------------------------------------------------------------------------

    std::cout << "*********** Test 3: matrix compression invariance **********" << std::endl;
    matCSR3.Compress();
    matCSR3_1.Compress();

    bool test3a = true;
    for (int m_sel = 0; m_sel < m; m_sel++)
        for (int n_sel = 0; n_sel < n; n_sel++)
            if (ref_matrix(m_sel, n_sel) != matCSR3.GetElement(m_sel, n_sel))
                test3a = false;

    std::cout << "Check matrices after compression" << std::endl;
    bool test3b = CompareArrays(matCSR3_1, matCSR3, false);
    if (!test3b)
    {
        std::cout << "Compare matCSR3 vs matCSR3_1" << std::endl;
        PrintMatrixCSR(matCSR3);
        PrintMatrixCSR(matCSR3_1);
    }
    bool test3 = test3a & test3b;
    std::cout << "Test 3: " << (test3 ? "passed" : "NOT passed") << std::endl << std::endl << std::endl;

    // ----------------------------------------------------------------------------

    bool passed = test1 && test2 && test3;
    return !passed;
}