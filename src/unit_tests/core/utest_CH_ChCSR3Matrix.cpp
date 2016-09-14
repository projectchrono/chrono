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

#include "chrono/core/ChCSR3Matrix.h"
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

void PrintMatrixCSR(const ChCSR3Matrix& mat) {
    int ncols = mat.GetNumColumns();
    int nrows = mat.GetNumRows();
    int* rowindex = mat.GetCSR_LeadingIndexArray();
    int* colindex = mat.GetCSR_TrailingIndexArray();
    double* values = mat.GetCSR_ValueArray();
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
    int rows = mat1.GetNumRows();
    int rows_temp = mat2.GetNumRows();

    if (rows_temp != rows) {
        std::cout << "Number of rows do not match: ";
        std::cout << "   mat1 -> " << rows;
        std::cout << "   mat2 -> " << rows_temp << std::endl;
        return false;
    }

    for (int cont = 0; cont <= rows; cont++) {
        if (mat1.GetCSR_LeadingIndexArray()[cont] != mat2.GetCSR_LeadingIndexArray()[cont]) {
            std::cout << "Row indexes do not match at entry " << cont << ":";
            std::cout << "   mat1 -> " << mat1.GetCSR_LeadingIndexArray()[cont];
            std::cout << "   mat2 -> " << mat2.GetCSR_LeadingIndexArray()[cont] << std::endl;
            return false;
        }
    }

    for (int cont = 0; cont < mat1.GetCSR_LeadingIndexArray()[rows]; cont++) {

        if (mat1.GetCSR_TrailingIndexArray()[cont] != mat2.GetCSR_TrailingIndexArray()[cont]) {
            std::cout << "Column indexes do not match at entry " << cont << ":";
            std::cout << "   mat1 -> " << mat1.GetCSR_TrailingIndexArray()[cont];
            std::cout << "   mat2 -> " << mat2.GetCSR_TrailingIndexArray()[cont] << std::endl;
            return false;
        }

        if (mat1.GetCSR_TrailingIndexArray()[cont] != -1 && mat2.GetCSR_TrailingIndexArray()[cont] != -1)
        {
            if (mat1.GetCSR_ValueArray()[cont] != mat2.GetCSR_ValueArray()[cont]) {
                std::cout << "Values do not match at entry " << cont << ":";
                std::cout << "   mat1 -> " << mat1.GetCSR_ValueArray()[cont];
                std::cout << "   mat2 -> " << mat2.GetCSR_ValueArray()[cont] << std::endl;
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


int testColumnMajor()
{
	int n = 3;
	ChCSR3Matrix matCM(n, n, false);
	matCM.SetElement(0, 0, 3);
	matCM.SetElement(0, 1, 5);
	matCM.SetElement(1, 0, 7);
	matCM.SetElement(1, 2, 9);
	matCM.SetElement(2, 2, 11);

	ChCSR3Matrix matRM(n, n, true);
	matRM.SetElement(0, 0, 3);
	matRM.SetElement(0, 1, 5);
	matRM.SetElement(1, 0, 7);
	matRM.SetElement(1, 2, 9);
	matRM.SetElement(2, 2, 11);

	bool test_passed = true;
	for (int i = 0; i < n; i++)
		for (int j = 0; j < n; j++)
		{
			if (matRM.GetElement(i, j) != matCM.GetElement(i, j))
			{
				test_passed = false;
				return test_passed;
			}	
		}

	return test_passed;

}

void basic_test()
{
	ChCSR3Matrix mat(3, 3, true, 6);
	mat.SetSparsityPatternLock(true);

	mat.SetElement(0, 0, 10.0);
	mat.SetElement(2,2, 2.2);
	mat.SetElement(1,1, 1.1);
	mat.SetElement(0, 1, 0.1);
	PrintMatrix(mat);

	mat.SetElement(2, 1, 2.1); // force insert forward of 1 element
	PrintMatrix(mat);

	mat.SetElement(1, 2, 1.2);
	PrintMatrix(mat);

	mat.Compress();
	PrintMatrix(mat);

	mat.Reset(3, 3);
	PrintMatrix(mat);

}

void basic_test2()
{
	ChCSR3Matrix mat(3,3, true, 6);
	//mat.SetSparsityPatternLock(true);

	mat.SetElement(2, 1, 2.1);
	mat.SetElement(2, 2, 2.2);
	PrintMatrix(mat);

	mat.SetElement(2, 0, 2.0);
	PrintMatrix(mat);

	mat.SetElement(1, 2, 1.2);
	PrintMatrix(mat);

	mat.SetElement(1, 1, 1.1);
	mat.SetElement(0, 2, 0.2);
	mat.SetElement(0, 1, 0.1);
	mat.SetElement(0, 0, 10.0);
	PrintMatrix(mat);

	mat.Compress();
	PrintMatrix(mat);

}

int main() {


	basic_test2();
	return 0;
}