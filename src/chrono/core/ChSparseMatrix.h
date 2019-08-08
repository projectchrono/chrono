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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHSPARSEMATRIX_H
#define CHSPARSEMATRIX_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMatrix.h"

#define SPM_DEF_FULLNESS 0.1       ///< default predicted density (in [0,1])
#define SPM_DEF_MAXELEMENTS 10000  ///< default limit on initial number of off-diagonal elements

namespace chrono {
class ChSparsityPatternLearner;

/// Base class for all sparse matrices.
class ChApi ChSparseMatrix {
  public:
    /// Symmetry type of the matrix
    enum SymmetryType {
        GENERAL,              ///< unsymmetric matrix
        SYMMETRIC_POSDEF,     ///< symmetric positive definite
        SYMMETRIC_INDEF,      ///< symmetric indefinite
        STRUCTURAL_SYMMETRIC  ///< structurally symmetric
    };

    /// Construct a sparse matrix with \a nrows and \a ncols and with \a nnz non-zero elements.
    /// By default, the matrix type is GENERAL (i.e., unsymmetric) and the sparsity pattern is unlocked.
    ChSparseMatrix(int nrows = 0, int ncols = 0, int nnz = 0) : m_num_rows(nrows), m_num_cols(ncols), m_nnz(nnz) {}

    ChSparseMatrix(const ChSparseMatrix& other) {
        m_num_rows = other.m_num_rows;
        m_num_cols = other.m_num_cols;
        m_nnz = other.m_nnz;
        m_type = other.m_type;
        m_lock = other.m_lock;
        m_update_sparsity_pattern = other.m_update_sparsity_pattern;
    }

    virtual ~ChSparseMatrix() {}

    /// Get the number of rows of this matrix.
    int GetNumRows() const { return m_num_rows; }

    /// Get the number of columns of this matrix.
    int GetNumColumns() const { return m_num_cols; }

    /// Get the number of non-zero elements stored in this matrix.
    virtual int GetNNZ() const { return m_nnz; }

    /// Set the symmetry type for this sparse matrix (default: GENERAL).
    /// A derived class should always support GENERAL (i.e. unsymmetric matrices), but is free
    /// to perform optimizations for symmetric or structurally symmetric matrices.
    void SetType(SymmetryType type) { m_type = type; }

    /// Return the symmetry type of this matrix.
    SymmetryType GetType() const { return m_type; }

    /// Enable/disable a lock on the matrix sparsity pattern (default: false).
    void SetSparsityPatternLock(bool val) { m_lock = val; }

    /// (Optional) Force the update of the sparsity pattern
    /// Depending on the internal data structure, this can highly speed up the insertion of elements in the matrix.
    /// Suggested for matrix with dimension >1e5
    virtual void LoadSparsityPattern(ChSparsityPatternLearner& sparsity_learner) {}

    /// Set the value of the element with index (\a insrow, \a inscol) to \a insval.
    /// \param[in] insrow row index of the element;
    /// \param[in] inscol column index of the element;
    /// \param[in] insval value of the element;
    /// \param[in] overwrite tells if the new element should overwrite an existing element or be summed to it.
    virtual void SetElement(int insrow, int inscol, double insval, bool overwrite = true) = 0;

    /// Returns the value of the element with index (\a row, \a col).
    /// Returns \c zero if an element is not stored.
    virtual double GetElement(int row, int col) const = 0;

    virtual void Reset(int row, int col, int nonzeros = 0) = 0;
    virtual bool Resize(int nrows, int ncols, int nonzeros = 0) = 0;

    /// Optional compression method, typically invoked after all elements have been inserted.
    /// Depending on the internal data structures, a derived class may perform additional operations
    /// for improved space or speed performance. A typical implementation should respect the sparsity
    /// pattern lock status. This function should return true if it makes any modifications.
    virtual bool Compress() { return false; }

    /// Paste a given matrix into \a this sparse matrix at position (\a insrow, \a inscol).
    /// The matrix \a matra will be copied into \a this[insrow : insrow + \a matra.GetRows()][[inscol : inscol +
    /// matra.GetColumns()]] \param[in] matra The source matrix that will be copied; \param[in] insrow The row index
    /// where the first element will be copied; \param[in] inscol The column index where the first element will be
    /// copied; \param[in] overwrite Tells if the copied element will overwrite an existing element or be summed to it;
    virtual void PasteMatrix(ChMatrixConstRef matra, int insrow, int inscol, bool overwrite = true) {
        for (auto i = 0; i < matra.rows(); i++) {
            for (auto j = 0; j < matra.cols(); j++) {
                this->SetElement(insrow + i, inscol + j, matra(i, j), overwrite);
            }
        }
    }

    /// Paste a clipped portion of the given matrix into \a this sparse matrix at position (\a insrow, \a inscol).
    /// So the clipped portion \a matra[cliprow : cliprow + nrows][[clipcol : clipcol + ncolumns]]
    /// will be copied into \a this[insrow : insrow + nrows][[inscol : inscol + ncolumns]]
    /// \param[in] matra The source matrix that will be copied;
    /// \param[in] cliprow The row index of the first element of source matrix that will be copied;
    /// \param[in] clipcol The column index of the first element of source matrix that will be copied;
    /// \param[in] nrows The number of rows that will be copied;
    /// \param[in] ncolumns The number of columns that will be copied;
    /// \param[in] insrow The row index where the first element will be copied;
    /// \param[in] inscol The column index where the first element will be copied;
    /// \param[in] overwrite Tells if the copied element will overwrite an existing element or be summed to it.
    virtual void PasteClippedMatrix(ChMatrixConstRef matra,
                                    int cliprow,
                                    int clipcol,
                                    int nrows,
                                    int ncols,
                                    int insrow,
                                    int inscol,
                                    bool overwrite = true) {
        for (auto i = 0; i < nrows; ++i)
            for (auto j = 0; j < ncols; ++j)
                this->SetElement(insrow + i, inscol + j, matra(i + cliprow, j + clipcol), overwrite);
    }

    /// Return the row|column index array in the CSR|CSC representation of this matrix.
    virtual int* GetCS_LeadingIndexArray() const { return nullptr; }

    /// Return the column|row index array in the CSR|CSC representation of this matrix.
    virtual int* GetCS_TrailingIndexArray() const { return nullptr; }

    /// Return the array of matrix values in the CSR|CSC representation of this matrix.
    virtual double* GetCS_ValueArray() const { return nullptr; }

    // Wrapper functions

    /// Same as #PasteMatrix(), but with \a overwrite set to \c true and \a transp set to \c true.
    /// The source matrix will be transposed and pasted into \a this matrix, overwriting existing elements.
    virtual void PasteTranspMatrix(ChMatrixConstRef matra, int insrow, int inscol) {
        PasteMatrix(matra.transpose(), insrow, inscol, true);
    }

    /// Same as #PasteMatrix(), but with \a overwrite set to \c false and \a transp set to \c false.
    /// The source matrix will be summed to the current matrix and not transposed.
    virtual void PasteSumMatrix(ChMatrixConstRef matra, int insrow, int inscol) {
        PasteMatrix(matra, insrow, inscol, false);
    }

    /// Same as #PasteMatrix(), but with \a overwrite set to \c false and \a transp set to \c true.
    /// The source matrix will be transposed and summed to the \a this matrix.
    virtual void PasteSumTranspMatrix(ChMatrixConstRef matra, int insrow, int inscol) {
        PasteMatrix(matra.transpose(), insrow, inscol, false);
    }

    /// Same as #PasteClippedMatrix(), but with \a overwrite set to \c false.
    /// The clipped portion of the source matrix will be summed to \a this matrix.
    virtual void PasteSumClippedMatrix(ChMatrixConstRef matra,
                               int cliprow,
                               int clipcol,
                               int nrows,
                               int ncols,
                               int insrow,
                               int inscol) {
        PasteClippedMatrix(matra, cliprow, clipcol, nrows, ncols, insrow, inscol, false);
    }

    /// Method to allow serializing transient data into in ASCII stream (e.g., a file) as a
    /// Matlab sparse matrix format; each row in file has three elements: {row, column, value}.
    /// Note: the row and column indexes start from 1.
    virtual void StreamOUTsparseMatlabFormat(ChStreamOutAscii& mstream) {
        for (int ii = 0; ii < m_num_rows; ii++) {
            for (int jj = 0; jj < m_num_cols; jj++) {
                double elVal = GetElement(ii, jj);
                if (elVal || (ii + 1 == m_num_rows && jj + 1 == m_num_cols)) {
                    mstream << ii + 1 << " " << jj + 1 << " " << elVal << "\n";
                }
            }
        }
    }

    virtual void StreamOUT(ChStreamOutAscii& mstream) {
        mstream << "\n"
            << "Matrix " << m_num_rows << " rows, " << m_num_cols << " columns."
            << "\n";
        for (int i = 0; i < std::min(m_num_rows, 8); i++) {
            for (int j = 0; j < std::min(m_num_cols, 8); j++)
                mstream << GetElement(i, j) << "  ";
            if (m_num_cols > 8)
                mstream << "...";
            mstream << "\n";
        }
        if (m_num_rows > 8)
            mstream << "... \n\n";
    }

  protected:
    int m_num_rows;                          ///< number of rows
    int m_num_cols;                          ///< number of columns
    int m_nnz;                               ///< number of non-zero elements
    SymmetryType m_type = GENERAL;           ///< matrix type
    bool m_lock = false;                     ///< indicate whether or not the matrix sparsity pattern should be locked
    bool m_update_sparsity_pattern = false;  ///< let the matrix acquire the sparsity pattern
};

}  // end namespace chrono

#endif
