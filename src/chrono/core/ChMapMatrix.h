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

#ifndef CH_MAP_MATRIX_H
#define CH_MAP_MATRIX_H

#include <vector>
#include <unordered_map>

#include "chrono/core/ChSparseMatrix.h"
#include "chrono/core/ChMatrixDynamic.h"

namespace chrono {

// -----------------------
// SPARSE MAP MATRIX CLASS
// -----------------------

/// This class defines a sparse matrix, implemented using unordered_maps for each row.
class ChApi ChMapMatrix : public ChSparseMatrix {
  public:
    /// Create a sparse matrix with given dimensions.
    ChMapMatrix(int nrows = 1, int ncols = 1);

    /// Create a sparse matrix from a given dense matrix.
    ChMapMatrix(const ChMatrix<>& mat);

    /// Copy constructor.
    ChMapMatrix(const ChMapMatrix& other);

    /// Destructor.
    ~ChMapMatrix();

    /// Resize this matrix.
    virtual bool Resize(int nrows, int ncols, int nonzeros = 0) override;

    /// Reset to null matrix and (if needed) changes the size.
    virtual void Reset(int nrows, int ncols, int nonzeros = 0) override;

    /// Set/update the specified matrix element.
    virtual void SetElement(int row, int col, double elem, bool overwrite = true) override;

    /// Get the element at the specified location.
    virtual double GetElement(int row, int col) const override;

    /// Return the row index array in the CSR representation of this matrix.
    virtual int* GetCS_LeadingIndexArray() const override;

    /// Return the column index array in the CSR representation of this matrix.
    virtual int* GetCS_TrailingIndexArray() const override;

    /// Return the array of matrix values in the CSR representation of this matrix.
    virtual double* GetCS_ValueArray() const override;

    /// Convert to dense matrix.
    void ConvertToDense(ChMatrixDynamic<double>& mat);

    /// Convert to CSR arrays.
	void ConvertToCSR(std::vector<int>& ia, std::vector<int>& ja, std::vector<double>& a) const;

    /// Method to allow serializing transient data into in ASCII stream (e.g., a file) as a
    /// Matlab sparse matrix format; each row in file has three elements: {row, column, value}.
    /// Note: the row and column indexes start from 1.
    void StreamOUTsparseMatlabFormat(ChStreamOutAscii& mstream);

    /// Write first few rows and columns to the console.
    /// Method to allow serializing transient data into in ASCII format.
    void StreamOUT(ChStreamOutAscii& mstream);

  private:
    struct MatrixRow {
        MatrixRow() : m_nnz(0) {}
		int m_nnz;                               ///< number of non-zero elements in row
        std::unordered_map<int, double> m_data;  ///< column - value pairs in row
    };

    std::vector<MatrixRow> m_rows;  ///< vector of data structures for each row

    // Storage for CSR arrays. These are allocated only if needed.
    mutable std::vector<int> m_ia;    ///< CSR row index array
    mutable std::vector<int> m_ja;    ///< CSR column index array
    mutable std::vector<double> m_a;  ///< CSR values array

    mutable bool m_CSR_current;  ///< flag indicating whether or not the CSR arrays are up-to-date
};

}  // end namespace chrono

#endif
