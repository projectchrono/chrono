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
    enum SymmetryType {
        GENERAL,              ///< unsymmetric matrix
        SYMMETRIC_POSDEF,     ///< symmetric positive definite
        SYMMETRIC_INDEF,      ///< symmetric indefinite
        STRUCTURAL_SYMMETRIC  ///< structurally symmetric
    };

    /// Construct a sparse matrix with 'nrows' and 'ncols' and with 'nnz' non-zero elements.
    /// By default, the matrix type is GENERAL (i.e., unsymmetric) and the sparsity pattern is unlocked.
    ChSparseMatrix(int nrows = 0, int ncols = 0, int nnz = 0)
        : m_num_rows(nrows), m_num_cols(ncols), m_nnz(nnz) {}

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

    /// Get the number of non-zero elements in this matrix.
    virtual int GetNNZ() const { return m_nnz; }

    /// Set the symmetry type for this sparse matrix (default: GENERAL).
    /// A derived class should always support GENERAL (i.e. unsymmetric matrices), but is free
    /// to perform optimizations for symmetric or structurally symmetric matrices.
    void SetType(SymmetryType type) { m_type = type; }

    /// Return the symnmetery type of this matrix.
    SymmetryType GetType() const { return m_type; }

    /// Enable/disable a lock on the matrix sparsity pattern (default: false).
    void SetSparsityPatternLock(bool val) { m_lock = val; }

	/// (Optional) Force the update of the sparsity pattern
    /// Depending on the internal data structure, this can highly speed up the insertion of elements in the matrix.
    /// Suggested for matrix with dimension >1e5
    virtual void LoadSparsityPattern(ChSparsityPatternLearner& sparsity_learner){}

    virtual void SetElement(int insrow, int inscol, double insval, bool overwrite = true) = 0;
    virtual double GetElement(int row, int col) const = 0;

    virtual void Reset(int row, int col, int nonzeros = 0) = 0;
    virtual bool Resize(int nrows, int ncols, int nonzeros = 0) = 0;

    /// Optional compression method, typically invoked after all elements have been inserted.
    /// Depending on the internal data structures, a derived class may perform additional operations
    /// for improved space or speed performance. A typical implementation should respect the sparsity
    /// pattern lock status. This function should return true if it makes any modifications.
    virtual bool Compress() { return false; }

    /// Paste the specified matrix into this sparse matrix at (insrow,inscol).
    virtual void PasteMatrix(const ChMatrix<>& matra, int insrow, int inscol, bool overwrite = true, bool transp = false) {
        auto maxrows = matra.GetRows();
        auto maxcols = matra.GetColumns();

        if (transp) {
            for (auto i = 0; i < maxcols; i++) {
                for (auto j = 0; j < maxrows; j++) {
                    this->SetElement(insrow + i, inscol + j, matra(j, i), overwrite);
                }
            }
        } else {
            for (auto i = 0; i < maxrows; i++) {
                for (auto j = 0; j < maxcols; j++) {
                    this->SetElement(insrow + i, inscol + j, matra(i, j), overwrite);
                }
            }
        }
    }

    /// Paste a clipped portion of the specified matrix into this sparse matrix at (insrow,inscol).
    virtual void PasteClippedMatrix(const ChMatrix<>& matra,
                                    int cliprow,
                                    int clipcol,
                                    int nrows,
                                    int ncolumns,
                                    int insrow,
                                    int inscol,
                                    bool overwrite = true) {
        for (auto i = 0; i < nrows; ++i)
            for (auto j = 0; j < ncolumns; ++j)
                this->SetElement(insrow + i, inscol + j, matra.GetElement(i + cliprow, j + clipcol), overwrite);
    }

    /// Return the row index array in the CSR representation of this matrix.
    virtual int* GetCSR_LeadingIndexArray() const { return nullptr; }

    /// Return the column index array in the CSR representation of this matrix.
    virtual int* GetCSR_TrailingIndexArray() const { return nullptr; }

    /// Return the array of matrix values in the CSR representation of this matrix.
    virtual double* GetCSR_ValueArray() const { return nullptr; }

    // Wrapper functions

    virtual void PasteTranspMatrix(const ChMatrix<>& matra, int insrow, int inscol) {
        PasteMatrix(matra, insrow, inscol, true, true);
    }
    virtual void PasteSumMatrix(const ChMatrix<>& matra, int insrow, int inscol) {
        PasteMatrix(matra, insrow, inscol, false, false);
    }
    virtual void PasteSumTranspMatrix(const ChMatrix<>& matra, int insrow, int inscol) {
        PasteMatrix(matra, insrow, inscol, false, true);
    }
    virtual void PasteSumClippedMatrix(const ChMatrix<>& matra,
                                       int cliprow,
                                       int clipcol,
                                       int nrows,
                                       int ncolumns,
                                       int insrow,
                                       int inscol) {
        PasteClippedMatrix(matra, cliprow, clipcol, nrows, ncolumns, insrow, inscol, false);
    }

  protected:
      int m_num_rows;                               ///< number of rows
      int m_num_cols;                               ///< number of columns
      int m_nnz;                                    ///< number of non-zero elements
      SymmetryType m_type = GENERAL;                ///< matrix type
      bool m_lock = false;                          ///< indicate whether or not the matrix sparsity pattern should be locked
      bool m_update_sparsity_pattern = false;	    ///< let the matrix acquire the sparsity pattern
};

}  // end namespace chrono

#endif
