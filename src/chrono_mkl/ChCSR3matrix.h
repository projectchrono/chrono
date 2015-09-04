#ifndef CHCSR3MATRIX_H
#define CHCSR3MATRIX_H

#include <Eigen/Sparse>
#include <mkl.h>
#include "core/ChSpmatrix.h"
#include "collision/bullet/LinearMath/btQuaternion.h"

namespace chrono {

class ChEigenMatrix : public ChSparseMatrixBase, public Eigen::SparseMatrix<double, Eigen::RowMajor, int> {
  public:
    ChEigenMatrix() : Eigen::SparseMatrix<double, Eigen::RowMajor, int>(){};
    ChEigenMatrix(int rows, int cols, double reallocRatio = 0.025)
        : Eigen::SparseMatrix<double, Eigen::RowMajor, int>(rows, cols) {
        // resizing is already performed by Eigen
        std::vector<int> reserveSize(rows, static_cast<int>(cols * reallocRatio));
        reserve(reserveSize);
    };

    template <class SizesType>
    ChEigenMatrix(SizesType& reserveSize)
        : Eigen::SparseMatrix<double, Eigen::RowMajor, int>(rows, cols) {
        reserve(reserveSize);
    };

    ChEigenMatrix(int dimension) : ChEigenMatrix(dimension, dimension){};

    virtual ~ChEigenMatrix(){};

    /*template<typename OtherDerived>
    ChEigenMatrix(const Eigen::SparseMatrixBase<SparseMatrix<OtherDerived>>& other) : Eigen::SparseMatrix<other>(){};

    template<typename OtherDerived>
    ChEigenMatrix& operator= (const Eigen::SparseMatrixBase<SparseMatrix<OtherDerived>>& other)	{
    this->Eigen::SparseMatrix<double>::operator=(other);
    return *this;
    };*/

    /// Virtualizabile functions
    virtual void SetElement(int insrow, int inscol, double insval) override { coeffRef(insrow, inscol) = insval; };

    virtual void PasteMatrix(ChMatrix<>* matra, int insrow, int inscol) override;
    virtual void PasteMatrixFloat(ChMatrix<float>* matra, int insrow, int inscol) override;
    virtual void PasteSumMatrix(ChMatrix<>* matra, int insrow, int inscol) override;
    virtual void PasteTranspMatrix(ChMatrix<>* matra, int insrow, int inscol) override;
    virtual void PasteSumTranspMatrix(ChMatrix<>* matra, int insrow, int inscol) override;
    virtual void PasteTranspMatrixFloat(ChMatrix<float>* matra, int insrow, int inscol) override;
    virtual void PasteClippedMatrix(ChMatrix<>* matra,
                                    int cliprow,
                                    int clipcol,
                                    int nrows,
                                    int ncolumns,
                                    int insrow,
                                    int inscol) override;
    virtual void PasteSumClippedMatrix(ChMatrix<>* matra,
                                       int cliprow,
                                       int clipcol,
                                       int nrows,
                                       int ncolumns,
                                       int insrow,
                                       int inscol) override;

    /// Templatized functions
    template <bool overwrite = 1>
    void SetElement(int insrow, int inscol, double insval) {
        if (overwrite)
            coeffRef(insrow, inscol) = insval;
        else
            coeffRef(insrow, inscol) += insval;
    };

    inline double& Element(int row, int col) { return coeffRef(row, col); };

    inline double* GetValueArray() { return valuePtr(); }

    inline int* GetColumnIndex() { return innerIndexPtr(); }

    inline int* GetRowIndex() { return outerIndexPtr(); }

    template <bool overwrite = 1, class ChMatrixIN>
    void PasteMatrix(ChMatrixIN* matra, int insrow, int inscol) {
        int maxrows = matra->GetRows();
        int maxcols = matra->GetColumns();
        int i, j;

        // can't use triplets because they expect a compressed matrix with
        // non existing entries

        for (i = 0; i < maxrows; i++) {
            for (j = 0; j < maxcols; j++) {
                if ((*matra)(i, j) != 0)
                    this->SetElement<overwrite>(insrow + i, inscol + j, (*matra)(i, j));
            }
        }
    }

    template <bool overwrite = 1, class ChMatrixIN>
    void PasteMatrixFloat(ChMatrixIN* matra, int insrow, int inscol) {
        PasteMatrix(matra, insrow, inscol);
    };

    template <class ChMatrixIN>
    void PasteSumMatrix(ChMatrixIN* matra, int insrow, int inscol) {
        PasteMatrix<0>(matra, insrow, inscol);
    };

    template <bool overwrite = 1, class ChMatrixIN>
    void PasteTranspMatrix(ChMatrixIN* matra, int insrow, int inscol) {
        int maxrows = matra->GetRows();
        int maxcols = matra->GetColumns();
        int i, j;

        // can't use triplets because they expect a compressed matrix with
        // non existing entries

        for (i = 0; i < maxcols; i++) {
            for (j = 0; j < maxrows; j++) {
                if ((*matra)(j, i) != 0)
                    this->SetElement<overwrite>(insrow + i, inscol + j, (*matra)(j, i));
            };
        };
    };

    template <class ChMatrixIN>
    void PasteSumTranspMatrix(ChMatrixIN* matra, int insrow, int inscol) {
        PasteTranspMatrix<0>(matra, insrow, inscol);
    };

    template <class ChMatrixIN>
    void PasteTranspMatrixFloat(ChMatrixIN* matra, int insrow, int inscol) {
        PasteTranspMatrix(matra, insrow, inscol);
    };

    // GetElement returns 0 also if indexes are out of bound!
    //***TODO*** see http://eigen.tuxfamily.org/dox/group__TutorialSparse.html for a better way to find elements
    double GetElement(const int row, const int col) const { return this->coeff(row, col); };

    inline int GetRows() const { return static_cast<int>(this->rows()); }

    inline int GetColumns() const { return static_cast<int>(this->cols()); }

    template <bool overwrite = 1, class ChMatrixIN>
    void
    PasteClippedMatrix(ChMatrixIN* matra, int cliprow, int clipcol, int nrows, int ncolumns, int insrow, int inscol) {
        /*#pragma omp parallel for if (nrows > CH_OMP_MATR)*/
        for (int i = 0; i < nrows; ++i)
            for (int j = 0; j < ncolumns; ++j)
                this->SetElement<overwrite>(insrow + i, inscol + j, matra->GetElement(i + cliprow, j + clipcol));
    }

    template <class ChMatrixIN>
    void PasteSumClippedMatrix(ChMatrixIN* matra,
                               int cliprow,
                               int clipcol,
                               int nrows,
                               int ncolumns,
                               int insrow,
                               int inscol) {
        PasteClippedMatrix<0>(matra, cliprow, clipcol, nrows, ncolumns, insrow, inscol);
    }

    double& operator()(const int row, const int col) { return Element(row, col); }
    double& operator()(const int index) { return Element((int)index / cols(), index % cols()); }

    void Reset(int new_rows, int new_cols) {
        // TODO verify if it allocates memory or it should be done with resizeNonZeros
        if ((new_rows != this->rows()) || (new_cols != this->cols()))
            resize(new_rows, new_cols);
    }

    void Reset(int mat_size) { Reset(mat_size, mat_size); }

    // Import function from ChSparseMatrix format; this function resets the matrix!
    void LoadFromChSparseMatrix(ChSparseMatrix* mat) {
        // Create the CSR3 matrix with just the right amount of elements
        int mat_rows = mat->GetRows();
        resize(mat_rows, mat->GetColumns());     // rectangular matrices allowed; matrix still empty
        std::vector<int> reserveSize(mat_rows);  // will take the number of elements for each row
        mat->CountNonZeros<std::vector<int>>(reserveSize);
        reserve(reserveSize);

        // Import values from ChSparseMatrix
        ChMelement* elarray = mat->GetElarrayDereferenced();
        ChMelement* el_temp;
        for (int i = 0; i < mat_rows; i++) {
            el_temp = &elarray[i];
            while (el_temp) {
                if (el_temp->val != 0)
                    SetElement(i, el_temp->col, el_temp->val);
                el_temp = el_temp->next;
            };
        };

    }  // END LoadFromChSparseMatrix;

    void LoadFromChSparseMatrix(ChSparseMatrix* M, ChSparseMatrix* Cq, ChSparseMatrix* E) {
        // Create the CSR3 matrix
        int M_rows = M->GetRows();
        int Cq_rows = Cq->GetRows();
        int mat_rows = M_rows + Cq_rows;
        resize(mat_rows, mat_rows);  // only square matrices allowed

        // Preallocate a ChEigenMatrix with the exact number of non-zeros of the ChSparseMatrix
        std::vector<int> reserveSize(mat_rows);  // will take the number of elements for each row
        ChMelement* Cq_elarray = Cq->GetElarrayDereferenced();
        ChMelement* el_temp;

        // scan Cq matrix for non-zeros
        for (int i = 0; i < Cq_rows; i++) {
            el_temp = &Cq_elarray[i];
            while (el_temp) {
                if (el_temp->val != 0) {
                    reserveSize[M_rows + i]++;
                    reserveSize[el_temp->col]++;
                };
                el_temp = el_temp->next;
            }
        }

        M->CountNonZeros<std::vector<int>>(reserveSize);
        E->CountNonZeros<std::vector<int>>(reserveSize, M_rows);

        reserve(reserveSize);

        // Import values from ChSparseMatrix
        for (int i = 0; i < Cq_rows; i++) {
            el_temp = &Cq_elarray[i];
            while (el_temp) {
                if (el_temp->val != 0) {
                    SetElement(M_rows + i, el_temp->col, el_temp->val);  // sets the Cq
                    SetElement(el_temp->col, M_rows + i, el_temp->val);  // sets the Cq'
                };
                el_temp = el_temp->next;
            }
        }

        ChMelement* M_elarray = M->GetElarrayDereferenced();
        for (int i = 0; i < M_rows; i++) {
            el_temp = &M_elarray[i];
            while (el_temp) {
                if (el_temp->val != 0)
                    SetElement(i, el_temp->col, el_temp->val);
                el_temp = el_temp->next;
            }
        }

        ChMelement* E_elarray = E->GetElarrayDereferenced();
        for (int i = 0; i < Cq_rows; i++) {
            el_temp = &E_elarray[i];
            while (el_temp) {
                if (el_temp->val != 0)
                    SetElement(i + M_rows, M_rows + el_temp->col, -el_temp->val);
                el_temp = el_temp->next;
            }
        }
    }  // END LoadFromChSparseMatrix

};  // END class ChEigenMatrix

// -----------------------------------------------------------------------------
// Specializations of ChEigenMatrix::SetElement()

template <>
inline void ChEigenMatrix::SetElement<1>(int insrow, int inscol, double insval) {
    coeffRef(insrow, inscol) = insval;
}

template <>
inline void ChEigenMatrix::SetElement<0>(int insrow, int inscol, double insval) {
    coeffRef(insrow, inscol) += insval;
}

// -----------------------------------------------------------------------------

class ChEigenMatrixNEW : public ChSparseMatrixBase {
  public:
    ChEigenMatrixNEW();
    ChEigenMatrixNEW(int insrow, int inscol, double insnonzero_ratio);
    virtual ~ChEigenMatrixNEW() override;

    virtual void SetElement(int insrow, int inscol, double insval) override;
    virtual double GetElement(int row, int col) override;

  protected:
    bool reallocate(int new_rows, int new_cols, double nonzero_ratio);
    void prune(
        double epsilon = DBL_EPSILON);  // or could simply remove the elements that has col index that equals -1!!!

  private:
    double* values;
    double* colIndex;
    double* rowIndex;
    int mat_rows;
    int mat_cols;
    double nonzero_ratio;
    int storage_dimension;  // could differ from 'rowIndex[mat_rows]' if has been allocated a greater space
};

inline ChEigenMatrixNEW::ChEigenMatrixNEW() {
}

inline ChEigenMatrixNEW::ChEigenMatrixNEW(int insrow, int inscol, double insnonzero_ratio) {
    assert(insrow > 0 && inscol > 0 && insnonzero_ratio > 0);
    mat_rows = insrow;
    mat_cols = inscol;
    storage_dimension = static_cast<int>(inscol * insrow * insnonzero_ratio);
    values = static_cast<double*>(mkl_malloc(storage_dimension * sizeof(double), 64));
    colIndex = static_cast<double*>(mkl_malloc(storage_dimension * sizeof(double), 64));
    rowIndex = static_cast<double*>(mkl_malloc(mat_rows + 1, 64));
    for (int row_sel = 0; row_sel < mat_rows; row_sel++) {
        rowIndex[row_sel] = row_sel * round(storage_dimension / mat_cols);
    }
    for (int col_sel = 0; col_sel < storage_dimension; col_sel++) {
        colIndex[col_sel] = -1;
    }
    rowIndex[mat_rows] = storage_dimension;
}

inline ChEigenMatrixNEW::~ChEigenMatrixNEW() {
}

inline void ChEigenMatrixNEW::SetElement(int insrow, int inscol, double insval) {
    assert(insrow < mat_rows && inscol < mat_cols);
    assert(insrow >= 0 && inscol >= 0);
    bool overwrite = 1;
    int col_sel;
    for (col_sel = rowIndex[insrow];
         col_sel < rowIndex[insrow + 1] && colIndex[col_sel] < inscol && colIndex[col_sel] != -1; col_sel++) {
    }

    // if the element is already in memory is overwritten. TODO: let the user choose if overwrite or add
    if (colIndex[col_sel] == inscol || colIndex[col_sel] == -1) {
        if (colIndex[col_sel] == -1) {
            values[col_sel] = insval;
            colIndex[col_sel] = inscol;
        } else {
            if (overwrite)
                values[col_sel] = insval;
            else
                values[col_sel] += insval;
        }

        return;
    }

    // at this time col_sel should point one space to the right on where 'insval' should be stored

    // check if the memory space should be inflated
    if (storage_dimension <= rowIndex[mat_rows + 1]) {
        // allocated_size = allocated_size + std::max(ceil(sizeof(double) / 64), 1.0) * 64;
        storage_dimension = storage_dimension + 1;  // TODO: probably it is better to add more spaces for a double
        values = static_cast<double*>(
            mkl_realloc(values, storage_dimension * 64));  // WARNING: does it preserve 64byte alignment or only 32?
        colIndex = static_cast<double*>(
            mkl_realloc(colIndex, storage_dimension * 64));  // WARNING: does it preserve 64byte alignment or only 32?
    }

    // in any case elements from 'col_sel' and to its right must be moved
    for (int col_sel_temp = rowIndex[mat_rows + 1]; col_sel_temp >= col_sel; col_sel_temp--) {
        colIndex[col_sel_temp] = colIndex[col_sel_temp - 1];
        values[col_sel_temp] = values[col_sel_temp - 1];
    }

    // update rowIndex
    for (int i = insrow; i <= mat_rows; i++) {
        rowIndex[i]++;
    }

    values[col_sel - 1] = insval;
    colIndex[col_sel - 1] = inscol;
}

inline double ChEigenMatrixNEW::GetElement(int row, int col) {
    assert(row < mat_rows && col < mat_cols);
    assert(row >= 0 && col >= 0);
    for (int col_sel = rowIndex[row]; col_sel < rowIndex[row + 1]; col_sel++) {
        if (colIndex[col_sel] == col) {
            return values[col_sel];
        }
    }
    return 0;
}

};  // END namespace chrono

#endif
