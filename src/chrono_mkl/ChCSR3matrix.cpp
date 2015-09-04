#include "ChCSR3matrix.h"

namespace chrono {

void ChEigenMatrix::PasteMatrix(ChMatrix<>* matra, int insrow, int inscol) {
    int maxrows = matra->GetRows();
    int maxcols = matra->GetColumns();
    int i, j;

    // can't use triplets because they expect a compressed matrix with
    // non existing entries

    for (i = 0; i < maxrows; i++) {
        for (j = 0; j < maxcols; j++) {
            if ((*matra)(i, j) != 0)
                this->SetElement<1>(insrow + i, inscol + j, (*matra)(i, j));
        }
    }
}

void ChEigenMatrix::PasteMatrixFloat(ChMatrix<float>* matra, int insrow, int inscol) {
    int maxrows = matra->GetRows();
    int maxcols = matra->GetColumns();
    int i, j;

    // can't use triplets because they expect a compressed matrix with
    // non existing entries

    for (i = 0; i < maxrows; i++) {
        for (j = 0; j < maxcols; j++) {
            if ((*matra)(i, j) != 0)
                this->SetElement<1>(insrow + i, inscol + j, (*matra)(i, j));
        }
    }
}

void ChEigenMatrix::PasteSumMatrix(ChMatrix<>* matra, int insrow, int inscol) {
    int maxrows = matra->GetRows();
    int maxcols = matra->GetColumns();
    int i, j;

    // can't use triplets because they expect a compressed matrix with
    // non existing entries

    for (i = 0; i < maxrows; i++) {
        for (j = 0; j < maxcols; j++) {
            if ((*matra)(i, j) != 0)
                this->SetElement<0>(insrow + i, inscol + j, (*matra)(i, j));
        }
    }
}

void ChEigenMatrix::PasteTranspMatrix(ChMatrix<>* matra, int insrow, int inscol) {
    int maxrows = matra->GetRows();
    int maxcols = matra->GetColumns();
    int i, j;

    // can't use triplets because they expect a compressed matrix with
    // non existing entries

    for (i = 0; i < maxcols; i++) {
        for (j = 0; j < maxrows; j++) {
            if ((*matra)(j, i) != 0)
                this->SetElement<1>(insrow + i, inscol + j, (*matra)(j, i));
        }
    }
}

void ChEigenMatrix::PasteSumTranspMatrix(ChMatrix<>* matra, int insrow, int inscol) {
    int maxrows = matra->GetRows();
    int maxcols = matra->GetColumns();
    int i, j;

    // can't use triplets because they expect a compressed matrix with
    // non existing entries

    for (i = 0; i < maxcols; i++) {
        for (j = 0; j < maxrows; j++) {
            if ((*matra)(j, i) != 0)
                this->SetElement<0>(insrow + i, inscol + j, (*matra)(j, i));
        }
    }
}

void ChEigenMatrix::PasteTranspMatrixFloat(ChMatrix<float>* matra, int insrow, int inscol) {
    int maxrows = matra->GetRows();
    int maxcols = matra->GetColumns();
    int i, j;

    // can't use triplets because they expect a compressed matrix with
    // non existing entries

    for (i = 0; i < maxcols; i++) {
        for (j = 0; j < maxrows; j++) {
            if ((*matra)(j, i) != 0)
                this->SetElement<1>(insrow + i, inscol + j, (*matra)(j, i));
        }
    }
}

void ChEigenMatrix::PasteClippedMatrix(ChMatrix<>* matra,
                                       int cliprow,
                                       int clipcol,
                                       int nrows,
                                       int ncolumns,
                                       int insrow,
                                       int inscol) {
    /*#pragma omp parallel for if (nrows > CH_OMP_MATR)*/
    for (int i = 0; i < nrows; ++i)
        for (int j = 0; j < ncolumns; ++j)
            this->SetElement<1>(insrow + i, inscol + j, matra->GetElement(i + cliprow, j + clipcol));
}

void ChEigenMatrix::PasteSumClippedMatrix(ChMatrix<>* matra,
                                          int cliprow,
                                          int clipcol,
                                          int nrows,
                                          int ncolumns,
                                          int insrow,
                                          int inscol) {
    /*#pragma omp parallel for if (nrows > CH_OMP_MATR)*/
    for (int i = 0; i < nrows; ++i)
        for (int j = 0; j < ncolumns; ++j)
            this->SetElement<0>(insrow + i, inscol + j, matra->GetElement(i + cliprow, j + clipcol));
}

}  // end namespace chrono
