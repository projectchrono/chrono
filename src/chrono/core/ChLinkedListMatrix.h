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

#ifndef CHLLMATRIX_H
#define CHLLMATRIX_H

#include <vector>

#include "chrono/core/ChLists.h"
#include "chrono/core/ChSparseMatrix.h"

namespace chrono {

/// Generic element of a sparse matrix ChLinkedListMatrix.
class ChApi ChMelement {
  public:
    double val;
    ChMelement* next;
    ChMelement* prev;
    int row;
    int col;

    ChMelement(double mval, ChMelement* mnext, ChMelement* mprev, int mrow, int mcol)
        : val(mval), next(mnext), prev(mprev), row(mrow), col(mcol) {}

    void Initialize(double mval, ChMelement* mnext, ChMelement* mprev, int mrow, int mcol) {
        val = mval;
        next = mnext;
        prev = mprev;
        col = mcol;
        row = mrow;
    }
};

// -------------------------------
// SPARSE LINKED LIST MATRIX CLASS
// -------------------------------

/// This class defines a sparse matrix, implemented using linked lists of
/// non-zero elements in each row.
class ChApi ChLinkedListMatrix : public ChSparseMatrix {
  private:
    ChMelement** elarray;  // array of 1st column elements

    ChList<ChMelement> mbufferlist;  // list of pointers to Melement* buffers
    ChMelement* mnextel;             // address of next writable element in mbufferlist

    int mtot_size;      // total allocated space for elements (maybe in noncontiguous buffers list)
	int mbuffer_size;   // size of currently used buffer
	int mbuffer_added;  // mbuffer_added grows all time a new element is set non-zero,
    // until it reaches mbuffer_size (if so, allocation of another buffer is needed). Handled internally.

    std::vector<int> m_pindices;
    double m_determinant;

  public:
    /// Default constructor.
    ChLinkedListMatrix();

    /// Create a sparse matrix with given dimensions and fill-in.
    ChLinkedListMatrix(int nrows, int ncols, double fill_in = SPM_DEF_FULLNESS);

    /// Create a sparse matrix from a given dense matrix.
    ChLinkedListMatrix(const ChMatrix<>& mat);

    /// Copy constructor.
    ChLinkedListMatrix(const ChLinkedListMatrix& other);

    /// Destructor.
    ~ChLinkedListMatrix();

    /// Copy this sparse matrix to a dense matrix.
    void CopyToMatrix(ChMatrix<>& mat);

    /// Optimized SetElement,  returning the fetched Melement*
    ChMelement* SetElement(int row, int col, double val, ChMelement* guess);
    /// Optimized GetElement,  returning the fetched Melement*
    ChMelement* GetElement(int row, int col, double* val, ChMelement* guess);

    /// Add another buffer in buffer list, bigger of last
    /// (max total buffer size = col x rows) as in newbuffer = lastbuffer * inflate
    /// (use inflate >1, suggested 2)     [mostly used internally]
    void MoreBuffer(double inflate);

    /// Overloadings of standard matrix functions
    virtual bool Resize(int nrows, int ncols, int nonzeros = 0) override;

    /// Reset to null matrix.
    void Reset();
    /// Reset to null matrix and (if needed) changes the size.
    virtual void Reset(int nrows, int ncols, int nonzeros = 0) override;
    /// If size changes, is like the above, otherwise just sets the elements to zero.
    void ResetBlocks(int nrows, int ncols);

    virtual void SetElement(int row, int col, double elem, bool overwrite = true) override;
    virtual double GetElement(int row, int col) const override;

    // Customized functions, speed-optimized for sparse matrices:

    virtual void PasteMatrix(const ChMatrix<>& matra, int insrow, int inscol, bool overwrite, bool transp) override;
    virtual void PasteTranspMatrix(const ChMatrix<>& matra, int insrow, int inscol) override;
    virtual void PasteClippedMatrix(const ChMatrix<>& matra,
                                    int cliprow,
                                    int clipcol,
                                    int nrows,
                                    int ncolumns,
                                    int insrow,
                                    int inscol,
                                    bool overwrite) override;
    virtual void PasteSumClippedMatrix(const ChMatrix<>& matra,
                                       int cliprow,
                                       int clipcol,
                                       int nrows,
                                       int ncolumns,
                                       int insrow,
                                       int inscol) override;
    virtual void PasteSumMatrix(const ChMatrix<>& matra, int insrow, int inscol) override;
    virtual void PasteSumTranspMatrix(const ChMatrix<>& matra, int insrow, int inscol) override;

    // Specialized functions

    void PasteMatrix(const ChLinkedListMatrix& matra, int insrow, int inscol);
    void PasteTranspMatrix(const ChLinkedListMatrix& matra, int insrow, int inscol);

    // Matrix operations

    void MatrScale(double factor);
    void Neg();

    // Linear algebra functions

    /// Perform in-place LU factorization with partial (row) pivoting.
    /// The matrix determinant is calculated as a by-product of the factorization and
    /// can be obtained with GetDeterminant().
    int Setup_LU();

    /// Solve the system A*x = b, using an existing LU factorization.
    void Solve_LU(const ChMatrix<>& b, ChMatrix<>& x);

    /// Solve the general system A*x = b.
    /// Note that the matrix is modified in-place to contain the LU factors.
    int SolveGeneral(const ChMatrix<>& b, ChMatrix<>& x);

    /// Perform in-place LDL factorization with full pivoting.
    /// Note that this factorization can only be performed for symmetric matrices.
    /// During the factorization, only the upper triangular part of A is accessed.
    int Setup_LDL();

    /// Solve the symmetric system A*x=b, using an existing LDL factorization.
    void Solve_LDL(const ChMatrix<>& b, ChMatrix<>& x);

    /// Solve the symmetric system A*x = b.
    /// Note that the matrix is modified in-place to contain the LU factors.
    int SolveSymmetric(const ChMatrix<>& b, ChMatrix<>& x);

    /// Get the pivot indexes after the last factorization.
    const std::vector<int>& GetPivots() const { return m_pindices; }

    /// Get matrix determinant.
    /// Available after a factorization.
    double GetDeterminant() const { return m_determinant; }

    // Output functions (for debugging)

    /// Method to allow serializing transient data into in ASCII stream (e.g., a file) as a
    /// Matlab sparse matrix format; each row in file has three elements: {row, column, value}.
    /// Note: the row and column indexes start from 1.
    void StreamOUTsparseMatlabFormat(ChStreamOutAscii& mstream);

    /// Write first few rows and columns to the console.
    /// Method to allow serializing transient data into in ASCII format.
    void StreamOUT(ChStreamOutAscii& mstream);

    // Functions used to convert to CSR3 format (ChCSMatrix)

    /// Used to convert to CSR3 format (ChCSMatrix).
    /// Returns the array to 1st column elements.
    ChMelement* GetElarrayDereferenced() { return *elarray; }  // used to scan the matrix faster than GetElement

    /// Used to convert to CSR3 format (ChCSMatrix).
    /// Returns a pointer to an array/vector of type \a reserveSizeType.
    /// reserveSizeType[i] gives the number of non-zero elements in the i-th row;
    /// \a reserveSize must have the same row-dimension as ChLinkedListMatrix instance.
    template <typename reserveSizeType>
    void CountNonZeros(reserveSizeType& reserveSize, int offset = 0) {
        ChMelement* el_temp;
        // from the first element of each row scan until there's no "next" linked element
        for (int i = 0; i < m_num_rows; i++) {  // for each row
            el_temp = elarray[i];              // start from the element [i,0]
            while (el_temp) {
                if (el_temp->val != 0)
                    reserveSize[i + offset]++;  // assert("i" should be equal to el_temp->row)
                el_temp = el_temp->next;        // move right until no "next" element is found
            }
            // there is no backward search because we always start from guess[j]
            // that points to (j-th row, 1st column) element also if it is zero
        }
    }

  private:
    ChMelement** GetElarray() { return elarray; }
    ChMelement* GetElarrayMel(int num) { return (*(elarray + num)); }
    double GetElarrayN(int num) { return (*(elarray + num))->val; }
    void SetElarrayN(double val, int num) { (*(elarray + num))->val = val; }
    ChMelement* NewElement(double mval, ChMelement* mnext, ChMelement* mprev, int mrow, int mcol) {
        if (mbuffer_added >= mbuffer_size) {
            MoreBuffer(2.0);
        }

        ChMelement* newel = mnextel;
        mnextel->Initialize(mval, mnext, mprev, mrow, mcol);

        mbuffer_added++;
        mnextel++;

        return (newel);
    }

    void Build(double fill_in);

    void SwapColumns(int a, int b);
    void SwapRows(int a, int b);

    int BestPivotRow(int current);
    int BestPivotDiag(int current);
    void DiagPivotSymmetric(int rowA, int rowB);
};

}  // end namespace chrono

#endif
