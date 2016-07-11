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

/// Base class for all sparse matrices.
class ChApi ChSparseMatrix {
  protected:
    int rows;
    int columns;

  public:
    ChSparseMatrix() : rows(-1), columns(-1) {}
    ChSparseMatrix(int nrows, int ncols) : rows(nrows), columns(ncols) {}
    virtual ~ChSparseMatrix() {}

    int GetRows() const { return rows; }
    int GetColumns() const { return columns; }

    virtual void SetElement(int insrow, int inscol, double insval, bool overwrite = true) = 0;
    virtual double GetElement(int row, int col) = 0;

    virtual void Reset(int row, int col, int nonzeros = 0) = 0;
    virtual bool Resize(int nrows, int ncols, int nonzeros = 0) = 0;

    /// Paste the specified matrix into this sparse matrix at (insrow,inscol).
    virtual void PasteMatrix(ChMatrix<>* matra, int insrow, int inscol, bool overwrite = true, bool transp = false) {
        int maxrows = matra->GetRows();
        int maxcols = matra->GetColumns();
        int i, j;

        if (transp) {
            for (i = 0; i < maxcols; i++) {
                for (j = 0; j < maxrows; j++) {
                    if ((*matra)(j, i) != 0)
                        this->SetElement(insrow + i, inscol + j, (*matra)(j, i), overwrite);
                }
            }
        } else {
            for (i = 0; i < maxrows; i++) {
                for (j = 0; j < maxcols; j++) {
                    if ((*matra)(i, j) != 0)
                        this->SetElement(insrow + i, inscol + j, (*matra)(i, j), overwrite);
                }
            }
        }
    }

    /// Paste a clipped portion of the specified matrix into this sparse matrix at (insrow,inscol).
    virtual void PasteClippedMatrix(ChMatrix<>* matra,
                                    int cliprow,
                                    int clipcol,
                                    int nrows,
                                    int ncolumns,
                                    int insrow,
                                    int inscol,
                                    bool overwrite = true) {
        for (int i = 0; i < nrows; ++i)
            for (int j = 0; j < ncolumns; ++j)
                this->SetElement(insrow + i, inscol + j, matra->GetElement(i + cliprow, j + clipcol), overwrite);
    }

    // Wrapper functions

    virtual void PasteTranspMatrix(ChMatrix<>* matra, int insrow, int inscol) {
        PasteMatrix(matra, insrow, inscol, true, true);
    }
    virtual void PasteSumMatrix(ChMatrix<>* matra, int insrow, int inscol) {
        PasteMatrix(matra, insrow, inscol, false, false);
    }
    virtual void PasteSumTranspMatrix(ChMatrix<>* matra, int insrow, int inscol) {
        PasteMatrix(matra, insrow, inscol, false, true);
    }
    virtual void PasteSumClippedMatrix(ChMatrix<>* matra,
                                       int cliprow,
                                       int clipcol,
                                       int nrows,
                                       int ncolumns,
                                       int insrow,
                                       int inscol) {
        PasteClippedMatrix(matra, cliprow, clipcol, nrows, ncolumns, insrow, inscol, false);
    }
};

}  // end namespace chrono

#endif
