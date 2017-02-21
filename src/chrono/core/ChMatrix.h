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

#ifndef CHMATRIX_H
#define CHMATRIX_H

#include <immintrin.h>

#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChException.h"
#include "chrono/ChConfig.h"
#include "chrono/serialization/ChArchive.h"
#include "chrono/serialization/ChArchiveAsciiDump.h"

namespace chrono {

//
// FAST MACROS TO SPEEDUP CODE
//

#define Set33Element(a, b, val) SetElementN(((a * 3) + (b)), val)
#define Get33Element(a, b) GetElementN((a * 3) + (b))

#define Set34Element(a, b, val) SetElementN(((a * 4) + (b)), val)
#define Get34Element(a, b) GetElementN((a * 4) + (b))
#define Set34Row(ma, a, val0, val1, val2, val3) \
    ma.SetElementN((a * 4), val0);              \
    ma.SetElementN((a * 4) + 1, val1);          \
    ma.SetElementN((a * 4) + 2, val2);          \
    ma.SetElementN((a * 4) + 3, val3);

#define Set44Element(a, b, val) SetElementN(((a * 4) + (b)), val)
#define Get44Element(a, b) GetElementN((a * 4) + (b))

// forward declaration
template <class Real = double>
class ChMatrixDynamic;

///
/// ChMatrix:
///
///  A base class for matrix objects (tables of NxM numbers).
/// To access elements, the indexes start from zero, and
/// you must indicate first row, then column, that is: m(2,4)
/// means the element at 3rd row, 5th column.
///  This is an abstract class, so you cannot instantiate
/// objects from it: you must rather create matrices using the
/// specialized child classes like ChMatrixDynamic, ChMatrixNM
/// ChMatrix33 and so on; all of them have this same base class.
///  Warning: for optimization reasons, not all functions will
/// check about boundaries of element indexes and matrix sizes (in
/// some cases, if sizes are wrong, debug asserts are used).
///
/// Further info at the @ref mathematical_objects manual page.

template <class Real = double>
class ChMatrix {
  protected:
    //
    // DATA
    //

    int rows;
    int columns;
    Real* address;

  public:
    //
    // CONSTRUCTORS (none - abstract class that must be implemented with child classes)
    //

    virtual ~ChMatrix() {}

    //
    // OPERATORS OVERLOADING
    //

    /// Parenthesis () operator, to access a single element of the matrix, by
    /// supplying the row and the column (indexes start from 0).
    /// For example: m(3,5) gets the element at the 4th row, 6th column.
    /// Value is returned by reference, so it can be modified, like in m(1,2)=10.
    Real& operator()(const int row, const int col) {
        assert(row >= 0 && col >= 0 && row < rows && col < columns);
        return (*(address + col + (row * columns)));
    }
    const Real& operator()(const int row, const int col) const {
        assert(row >= 0 && col >= 0 && row < rows && col < columns);
        return (*(address + col + (row * columns)));
    }

    /// Parenthesis () operator, to access a single element of the matrix, by
    /// supplying the ordinal of the element (indexes start from 0).
    /// For example: m(3) gets the 4th element, counting row by row.
    /// Mostly useful if the matrix is Nx1 sized (i.e. a N-element vector).
    /// Value is returned by reference, so it can be modified, like in m(1,2)=10.
    Real& operator()(const int el) {
        assert(el >= 0 && el < rows * columns);
        return (*(address + el));
    }
    const Real& operator()(const int el) const {
        assert(el >= 0 && el < rows * columns);
        return (*(address + el));
    }

    /// The [] operator returns the address of the n-th row. This is mostly
    /// for compatibility with old matrix programming styles (2d array-like)
    /// where to access an element at row i, column j, one can write mymatrix[i][j].
    Real* operator[](const int row) {
        assert(row >= 0 && row < rows);
        return ((address + (row * columns)));
    }
    const Real* operator[](const int row) const {
        assert(row >= 0 && row < rows);
        return ((address + (row * columns)));
    }

    /// Multiplies this matrix by a factor, in place
    ChMatrix<Real>& operator*=(const Real factor) {
        MatrScale(factor);
        return *this;
    }

    /// Increments this matrix by another matrix, in place
    template <class RealB>
    ChMatrix<Real>& operator+=(const ChMatrix<RealB>& matbis) {
        MatrInc(matbis);
        return *this;
    }

    /// Decrements this matrix by another matrix, in place
    template <class RealB>
    ChMatrix<Real>& operator-=(const ChMatrix<RealB>& matbis) {
        MatrDec(matbis);
        return *this;
    }

    /// Matrices are equal?
    bool operator==(const ChMatrix<Real>& other) { return Equals(other); }
    /// Matrices are not equal?
    bool operator!=(const ChMatrix<Real>& other) { return !Equals(other); }

    /// Assignment operator
    ChMatrix<Real>& operator=(const ChMatrix<Real>& matbis) {
        if (&matbis != this)
            CopyFromMatrix(matbis);
        return *this;
    }
    template <class RealB>
    ChMatrix<Real>& operator=(const ChMatrix<RealB>& matbis) {
        CopyFromMatrix(matbis);
        return *this;
    }

    //
    // FUNCTIONS
    //

    /// Sets the element at row,col position. Indexes start with zero.
    void SetElement(int row, int col, Real elem) {
        assert(row >= 0 && col >= 0 && row < rows && col < columns);  // boundary checks
        *(address + col + (row * columns)) = elem;
    }

    /// Gets the element at row,col position. Indexes start with zero.
    /// The return value is a copy of original value. Use Element() instead if you
    /// want to access directly by reference the original element.
    Real GetElement(int row, int col) {
        assert(row >= 0 && col >= 0 && row < rows && col < columns);  // boundary checks
        return (*(address + col + (row * columns)));
    }
    Real GetElement(int row, int col) const {
        assert(row >= 0 && col >= 0 && row < rows && col < columns);  // boundary checks
        return (*(address + col + (row * columns)));
    }

    /// Sets the Nth element, counting row after row.
    void SetElementN(int index, Real elem) {
        assert(index >= 0 && index < (rows * columns));  // boundary checks
        *(address + index) = elem;
    }

    /// Gets the Nth element, counting row after row.
    Real GetElementN(int index) {
        assert(index >= 0 && index < (rows * columns));
        return (*(address + index));
    }
    const Real GetElementN(int index) const {
        assert(index >= 0 && index < (rows * columns));
        return (*(address + index));
    }

    /// Access a single element of the matrix, by
    /// supplying the row and the column (indexes start from 0).
    /// Value is returned by reference, so it can be modified, like in m.Element(1,2)=10.
    Real& Element(int row, int col) {
        assert(row >= 0 && col >= 0 && row < rows && col < columns);
        return (*(address + col + (row * columns)));
    }
    const Real& Element(int row, int col) const {
        assert(row >= 0 && col >= 0 && row < rows && col < columns);
        return (*(address + col + (row * columns)));
    }
    /// Access a single element of the matrix, the Nth element, counting row after row.
    /// Value is returned by reference, so it can be modified, like in m.Element(5)=10.
    Real& ElementN(int index) {
        assert(index >= 0 && index < (rows * columns));
        return (*(address + index));
    }
    const Real& ElementN(int index) const {
        assert(index >= 0 && index < (rows * columns));
        return (*(address + index));
    }

    /// Access directly the "Real* address" buffer. Warning! this is a low level
    /// function, it should be used in rare cases, if really needed!
    Real* GetAddress() { return address; }
    const Real* GetAddress() const { return address; }

    /// Gets the number of rows
    int GetRows() const { return rows; }

    /// Gets the number of columns
    int GetColumns() const { return columns; }

    /// Reallocate memory for a new size. VIRTUAL! Must be implemented by child classes!
    virtual void Resize(int nrows, int ncols) {}

    /// Swaps the columns a and b
    void SwapColumns(int a, int b) {
        Real temp;
        for (int i = 0; i < rows; i++) {
            temp = GetElement(i, a);
            SetElement(i, a, GetElement(i, b));
            SetElement(i, b, temp);
        }
    }

    /// Swap the rows a and b
    void SwapRows(int a, int b) {
        Real temp;
        for (int i = 0; i < columns; i++) {
            temp = GetElement(a, i);
            SetElement(a, i, GetElement(b, i));
            SetElement(b, i, temp);
        }
    }

    /// Fill the diagonal elements, given a sample.
    /// Note that the matrix must already be square (no check for
    /// rectangular matrices!), and the extradiagonal elements are
    /// not modified -this function does not set them to 0-
    void FillDiag(Real sample) {
        for (int i = 0; i < rows; ++i)
            SetElement(i, i, sample);
    }

    /// Fill the matrix with the same value in all elements
    void FillElem(Real sample) {
        for (int i = 0; i < rows * columns; ++i)
            SetElementN(i, sample);
    }

    /// Fill the matrix with random float numbers, falling within the
    /// "max"/"min" range.
    void FillRandom(Real max, Real min) {
        for (int i = 0; i < rows * columns; ++i)
            SetElementN(i, min + (Real)ChRandom() * (max - min));
    }

    /// Resets the matrix to zero  (warning: simply sets memory to 0 bytes!)
    void Reset() {
        // SetZero(rows*columns); //memset(address, 0, sizeof(Real) * rows * columns);
        for (int i = 0; i < rows * columns; ++i)
            this->address[i] = 0;
    }

    /// Reset to zeroes and (if needed) changes the size to have row and col
    void Reset(int nrows, int ncols) {
        Resize(nrows, ncols);
        // SetZero(rows*columns); //memset(address, 0, sizeof(Real) * rows * columns);
        for (int i = 0; i < rows * columns; ++i)
            this->address[i] = 0;
    }

    /// Reset to identity matrix (ones on diagonal, zero elsewhere)
    void SetIdentity() {
        Reset();
        FillDiag(1.0);
    }

    /// Copy a matrix "matra" into this matrix. Note that
    /// the destination matrix will be resized if necessary.
    template <class RealB>
    void CopyFromMatrix(const ChMatrix<RealB>& matra) {
        Resize(matra.GetRows(), matra.GetColumns());
        // ElementsCopy(address, matra.GetAddress(), rows*columns);
        // memcpy (address, matra.address, (sizeof(Real) * rows * columns));
        for (int i = 0; i < rows * columns; ++i)
            address[i] = (Real)matra.GetAddress()[i];
    }

    /// Copy the transpose of matrix "matra" into this matrix. Note that
    /// the destination matrix will be resized if necessary.
    template <class RealB>
    void CopyFromMatrixT(const ChMatrix<RealB>& matra) {
        Resize(matra.GetColumns(), matra.GetRows());
        for (int i = 0; i < matra.GetRows(); ++i)
            for (int j = 0; j < matra.GetColumns(); ++j)
                SetElement(j, i, (Real)matra.Element(i, j));
    }

    /// Copy the transposed upper triangular part of "matra" in the lower triangular
    /// part of this matrix. (matra must be square)
    /// Note that the destination matrix will be resized if necessary.
    template <class RealB>                            //    _______                       //
    void CopyTUpMatrix(const ChMatrix<RealB>& matra)  //    \      |          |\          //
    {                                                 //      \  A'|   --->   |  \        //
        Resize(matra.GetRows(), matra.GetColumns());  //        \  |          |this\      //
        for (int i = 0; i < matra.GetRows(); i++) {   //          \|          |______\    //
            for (int j = 0; j < matra.GetRows(); j++)
                SetElement(j, i, (Real)matra.GetElement(i, j));
        }
    }

    /// Copy the transposed lower triangulat part of "matra" in the upper triangular
    /// part of this matrix. (matra must be square)
    /// Note that the destination matrix will be resized if necessary.
    template <class RealB>                            //                      _______     //
    void CopyTLwMatrix(const ChMatrix<RealB>& matra)  //    |\                \      |    //
    {                                                 //    |  \      --->      \this|    //
        Resize(matra.GetRows(), matra.GetColumns());  //    |A'  \                \  |    //
        for (int i = 0; i < matra.GetRows(); i++) {   //    |______\                \|    //
            for (int j = 0; j < matra.GetRows(); j++)
                SetElement(i, j, (Real)matra.GetElement(j, i));
        }
    }

    //
    // STREAMING
    //
    /// Method to allow serialization of transient data in archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) {
        // suggested: use versioning
        marchive.VersionWrite(1);

        // stream out all member data
        marchive << make_ChNameValue("rows", rows);
        marchive << make_ChNameValue("columns", columns);

        // custom output of matrix data as array
        if (ChArchiveAsciiDump* mascii = dynamic_cast<ChArchiveAsciiDump*>(&marchive)) {
            // CUSTOM row x col 'intuitive' table-like log when using ChArchiveAsciiDump:

            for (int i = 0; i < rows; i++) {
                mascii->indent();
                for (int j = 0; j < columns; j++) {
                    mascii->GetStream()->operator<<(Element(i, j));
                    mascii->GetStream()->operator<<(", ");
                }
                mascii->GetStream()->operator<<("\n");
            }
        } else {
            // NORMAL array-based serialization:

            int tot_elements = GetRows() * GetColumns();
            marchive.out_array_pre("data", tot_elements, typeid(Real).name());
            for (int i = 0; i < tot_elements; i++) {
                marchive << CHNVP(ElementN(i), "");
                marchive.out_array_between(tot_elements, typeid(Real).name());
            }
            marchive.out_array_end(tot_elements, typeid(Real).name());
        }
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) {
        // suggested: use versioning
        int version = marchive.VersionRead();

        // stream in all member data
        int m_row, m_col;
        marchive >> make_ChNameValue("rows", m_row);
        marchive >> make_ChNameValue("columns", m_col);

        Reset(m_row, m_col);

        // custom input of matrix data as array
        size_t tot_elements = GetRows() * GetColumns();
        marchive.in_array_pre("data", tot_elements);
        for (int i = 0; i < tot_elements; i++) {
            marchive >> CHNVP(ElementN(i));
            marchive.in_array_between("data");
        }
        marchive.in_array_end("data");
    }

    /// Method to allow serializing transient data into in ascii
    /// as a readable item, for example   "chrono::GetLog() << myobject;"
    /// ***OBSOLETE***
    void StreamOUT(ChStreamOutAscii& mstream) {
        mstream << "\n"
                << "Matrix " << GetRows() << " rows, " << GetColumns() << " columns."
                << "\n";
        for (int i = 0; i < ChMin(GetRows(), 8); i++) {
            for (int j = 0; j < ChMin(GetColumns(), 8); j++)
                mstream << GetElement(i, j) << "  ";
            if (GetColumns() > 8)
                mstream << "...";
            mstream << "\n";
        }
        if (GetRows() > 8)
            mstream << "... \n\n";
    }

    /// Method to allow serializing transient data into an ascii stream (ex. a file)
    /// as a Matlab .dat file (all numbers in a row, separated by space, then CR)
    void StreamOUTdenseMatlabFormat(ChStreamOutAscii& mstream) {
        for (int ii = 0; ii < this->GetRows(); ii++) {
            for (int jj = 0; jj < this->GetColumns(); jj++) {
                mstream << this->GetElement(ii, jj);
                if (jj < (this->GetColumns() - 1))
                    mstream << " ";
            }
            mstream << "\n";
        }
    }

    //
    // MATH MEMBER FUNCTIONS.
    // For speed reasons, sometimes size checking of operands is left to the user!
    //

    /// Changes the sign of all the elements of this matrix, in place.
    void MatrNeg() {
        for (int nel = 0; nel < rows * columns; ++nel)
            ElementN(nel) = -ElementN(nel);
    }

    /// Sum two matrices, and stores the result in "this" matrix: [this]=[A]+[B].
    template <class RealB, class RealC>
    void MatrAdd(const ChMatrix<RealB>& matra, const ChMatrix<RealC>& matrb) {
        assert(matra.GetColumns() == matrb.GetColumns() && matra.rows == matrb.GetRows());
        assert(this->columns == matrb.GetColumns() && this->rows == matrb.GetRows());
        for (int nel = 0; nel < rows * columns; ++nel)
            ElementN(nel) = (Real)(matra.ElementN(nel) + matrb.ElementN(nel));
    }

    /// Subtract two matrices, and stores the result in "this" matrix: [this]=[A]-[B].
    template <class RealB, class RealC>
    void MatrSub(const ChMatrix<RealB>& matra, const ChMatrix<RealC>& matrb) {
        assert(matra.GetColumns() == matrb.GetColumns() && matra.rows == matrb.GetRows());
        assert(this->columns == matrb.GetColumns() && this->rows == matrb.GetRows());
        for (int nel = 0; nel < rows * columns; ++nel)
            ElementN(nel) = (Real)(matra.ElementN(nel) - matrb.ElementN(nel));
    }

    /// Increments this matrix with another matrix A, as: [this]+=[A]
    template <class RealB>
    void MatrInc(const ChMatrix<RealB>& matra) {
        assert(matra.GetColumns() == columns && matra.GetRows() == rows);
        for (int nel = 0; nel < rows * columns; ++nel)
            ElementN(nel) += (Real)matra.ElementN(nel);
    }

    /// Decrements this matrix with another matrix A, as: [this]-=[A]
    template <class RealB>
    void MatrDec(const ChMatrix<RealB>& matra) {
        assert(matra.GetColumns() == columns && matra.GetRows() == rows);
        for (int nel = 0; nel < rows * columns; ++nel)
            ElementN(nel) -= (Real)matra.ElementN(nel);
    }

    /// Scales a matrix, multiplying all elements by a constant value: [this]*=f
    void MatrScale(Real factor) {
        for (int nel = 0; nel < rows * columns; ++nel)
            ElementN(nel) *= factor;
    }
    /// Scales a matrix, multiplying all element by all oter elements of
    /// matra (it is not the classical matrix multiplication!)
    template <class RealB>
    void MatrScale(const ChMatrix<RealB>& matra) {
        assert(matra.GetColumns() == columns && matra.GetRows() == rows);
        for (int nel = 0; nel < rows * columns; ++nel)
            ElementN(nel) *= (Real)matra.ElementN(nel);
    }

    /// Scales a matrix, dividing all elements by a constant value: [this]/=f
    void MatrDivScale(Real factor) {
        for (int nel = 0; nel < rows * columns; ++nel)
            ElementN(nel) /= factor;
    }

    /// Scales a matrix, dividing all element by all oter elements of
    /// matra (it is not the classical matrix multiplication!)
    template <class RealB>
    void MatrDivScale(const ChMatrix<RealB>& matra) {
        assert(matra.GetColumns() == columns && matra.GetRows() == rows);
        for (int nel = 0; nel < rows * columns; ++nel)
            ElementN(nel) /= (Real)matra.ElementN(nel);
    }

    /// Multiplies two matrices, and stores the result in "this" matrix: [this]=[A]*[B].
    template <class RealB, class RealC>
    void MatrMultiply(const ChMatrix<RealB>& matra, const ChMatrix<RealC>& matrb) {
        assert(matra.GetColumns() == matrb.GetRows());
        assert(this->rows == matra.GetRows());
        assert(this->columns == matrb.GetColumns());
        int col, row, colres;
        Real sum;
        for (colres = 0; colres < matrb.GetColumns(); ++colres) {
            for (row = 0; row < matra.GetRows(); ++row) {
                sum = 0;
                for (col = 0; col < matra.GetColumns(); ++col)
                    sum += (Real)(matra.Element(row, col) * matrb.Element(col, colres));
                SetElement(row, colres, sum);
            }
        }
    }

#ifdef CHRONO_HAS_AVX
    /// Multiplies two matrices, and stores the result in "this" matrix: [this]=[A]*[B].
    /// AVX implementation: The speed up is marginal if size of the matrices are small, e.g. 3*3
    /// Generally, as the matra.GetColumns() increases the method performs better
    void MatrMultiplyAVX(const ChMatrix<double>& matra, const ChMatrix<double>& matrb) {
        assert(matra.GetColumns() == matrb.GetRows());
        assert(this->rows == matra.GetRows());
        assert(this->columns == matrb.GetColumns());
        int A_Nrow = matra.GetRows();
        int B_Nrow = matrb.GetRows();
        int A_NCol = matra.GetColumns();
        int B_NCol = matrb.GetColumns();
        const double* A_add = matra.GetAddress();
        const double* B_add = matrb.GetAddress();
        double* this_Add = this->GetAddress();
        for (int rowA = 0; rowA < A_Nrow; rowA++) {
            for (int colB = 0; colB < B_NCol; colB += 4) {
                __m256d sum = _mm256_setzero_pd();
                for (int elem = 0; elem < A_NCol; elem++) {
                    __m256d ymmA = _mm256_broadcast_sd(A_add + A_NCol * rowA + elem);
                    __m256d ymmB = _mm256_loadu_pd(B_add + elem * B_NCol + colB);
                    __m256d prod = _mm256_mul_pd(ymmA, ymmB);
                    sum = _mm256_add_pd(sum, prod);
                }
                _mm256_storeu_pd(this_Add + rowA * B_NCol + colB, sum);
            }
        }
    }

    /// Multiplies two matrices (the second is considered transposed): [this]=[A]*[B]'
    /// Note: This method is faster than MatrMultiplyT if matra.GetColumns()%4=0 && matra.GetColumns()>8
    /// It is still fast if matra.GetColumns() is large enough even if matra.GetColumns()%4!=0
    void MatrMultiplyTAVX(const ChMatrix<double>& matra, const ChMatrix<double>& matrb) {
        assert(matra.GetColumns() == matrb.GetColumns());
        assert(this->GetRows() == matra.GetRows());
        assert(this->GetColumns() == matrb.GetRows());
        int A_Nrow = matra.GetRows();
        int B_Nrow = matrb.GetRows();
        int A_NCol = matra.GetColumns();
        int B_NCol = matrb.GetColumns();
        const double* A_add = matra.GetAddress();
        const double* B_add = matrb.GetAddress();
        bool NeedsPadding = (B_NCol % 4 != 0);
        int CorrectFAT = ((B_NCol >> 2) << 2);
        for (int rowA = 0; rowA < A_Nrow; rowA++) {
            for (int rowB = 0; rowB < B_Nrow; rowB++) {
                int colB;
                double temp_sum = 0.0;
                __m256d sum = _mm256_setzero_pd();
                for (colB = 0; colB < CorrectFAT; colB += 4) {
                    __m256d ymmA = _mm256_loadu_pd(A_add + rowA * A_NCol + colB);
                    __m256d ymmB = _mm256_loadu_pd(B_add + rowB * B_NCol + colB);
                    __m256d prod = _mm256_mul_pd(ymmA, ymmB);
                    sum = _mm256_add_pd(sum, prod);
                }
                sum = _mm256_hadd_pd(sum, sum);
                temp_sum = ((double*)&sum)[0] + ((double*)&sum)[2];
                if (NeedsPadding)
                    for (colB = CorrectFAT; colB < B_NCol; colB++) {
                        temp_sum += (matra.Element(rowA, colB) * matrb.Element(rowB, colB));
                    }
                SetElement(rowA, rowB, temp_sum);
            }
        }
    }

#endif

    /// Multiplies two matrices (the second is considered transposed): [this]=[A]*[B]'
    /// Faster than doing B.MatrTranspose(); result.MatrMultiply(A,B);
    /// Note: no check on mistaken size of this!
    template <class RealB, class RealC>
    void MatrMultiplyT(const ChMatrix<RealB>& matra, const ChMatrix<RealC>& matrb) {
        assert(matra.GetColumns() == matrb.GetColumns());
        assert(this->rows == matra.GetRows());
        assert(this->columns == matrb.GetRows());
        int col, row, colres;
        Real sum;
        for (colres = 0; colres < matrb.GetRows(); ++colres) {
            for (row = 0; row < matra.GetRows(); ++row) {
                sum = 0;
                for (col = 0; col < matra.GetColumns(); ++col)
                    sum += (Real)(matra.Element(row, col) * matrb.Element(colres, col));
                SetElement(row, colres, sum);
            }
        }
    }

    /// Multiplies two matrices (the first is considered transposed): [this]=[A]'*[B]
    /// Faster than doing A.MatrTranspose(); result.MatrMultiply(A,B);
    template <class RealB, class RealC>
    void MatrTMultiply(const ChMatrix<RealB>& matra, const ChMatrix<RealC>& matrb) {
        assert(matra.GetRows() == matrb.GetRows());
        assert(this->rows == matra.GetColumns());
        assert(this->columns == matrb.GetColumns());
        int col, row, colres;
        Real sum;
        for (colres = 0; colres < matrb.GetColumns(); ++colres) {
            for (row = 0; row < matra.GetColumns(); ++row) {
                sum = 0;
                for (col = 0; col < (matra.GetRows()); ++col)
                    sum += (Real)(matra.Element(col, row) * matrb.Element(col, colres));
                SetElement(row, colres, sum);
            }
        }
    }

    /// Computes dot product between two column-matrices (vectors) with
    /// same size. Returns a scalar value.
    template <class RealB, class RealC>
    static Real MatrDot(const ChMatrix<RealB>& ma, const ChMatrix<RealC>& mb) {
        assert(ma.GetColumns() == mb.GetColumns() && ma.GetRows() == mb.GetRows());
        Real tot = 0;
        for (int i = 0; i < ma.GetRows(); ++i)
            tot += (Real)(ma.ElementN(i) * mb.ElementN(i));
        return tot;
    }

    /// Transpose this matrix in place
    void MatrTranspose() {
        if (columns == rows)  // Square transp.is optimized
        {
            for (int row = 0; row < rows; ++row)
                for (int col = row; col < columns; ++col)
                    if (row != col) {
                        Real temp = Element(row, col);
                        Element(row, col) = Element(col, row);
                        Element(col, row) = temp;
                    }
            int tmpr = rows;
            rows = columns;
            columns = tmpr;
        } else  // Naive implementation for rectangular case. Not in-place. Slower.
        {
            ChMatrixDynamic<Real> matrcopy(*this);
            int tmpr = rows;
            rows = columns;
            columns = tmpr;  // dont' realloc buffer, anyway
            for (int row = 0; row < rows; ++row)
                for (int col = 0; col < columns; ++col)
                    Element(row, col) = matrcopy.Element(col, row);
        }
    }

    /// Returns the determinant of the matrix.
    /// Note! This method must be used only with max 4x4 matrices,
    /// otherwise it throws an exception.
    Real Det() {
        assert(this->GetRows() == this->GetColumns());
        assert(this->GetRows() <= 4);

        if (this->GetRows() != this->GetColumns())
            throw("Cannot compute matrix determinant because rectangular matrix");
        if (this->GetRows() > 4)
            throw("Cannot compute matrix determinant because matr. larger than 3x3");
        Real det = 0;
        switch (this->GetRows()) {
            case 1:
                det = (*this)(0, 0);
                break;
            case 2:
                det = (*this)(0, 0) * (*this)(1, 1) - (*this)(0, 1) * (*this)(1, 0);
                break;
            case 3:
                det = (*this)(0, 0) * (*this)(1, 1) * (*this)(2, 2) + (*this)(0, 1) * (*this)(1, 2) * (*this)(2, 0) +
                      (*this)(0, 2) * (*this)(1, 0) * (*this)(2, 1) - (*this)(2, 0) * (*this)(1, 1) * (*this)(0, 2) -
                      (*this)(2, 1) * (*this)(1, 2) * (*this)(0, 0) - (*this)(2, 2) * (*this)(1, 0) * (*this)(0, 1);
                break;
            case 4:
                det = (*this)(0, 0) * (*this)(1, 1) * (*this)(2, 2) * (*this)(3, 3) +
                      (*this)(0, 0) * (*this)(1, 2) * (*this)(2, 3) * (*this)(3, 1) +
                      (*this)(0, 0) * (*this)(1, 3) * (*this)(2, 1) * (*this)(3, 2) +
                      (*this)(0, 1) * (*this)(1, 0) * (*this)(2, 3) * (*this)(3, 2) +
                      (*this)(0, 1) * (*this)(1, 2) * (*this)(2, 0) * (*this)(3, 3) +
                      (*this)(0, 1) * (*this)(1, 3) * (*this)(2, 2) * (*this)(3, 0) +
                      (*this)(0, 2) * (*this)(1, 0) * (*this)(2, 1) * (*this)(3, 3) +
                      (*this)(0, 2) * (*this)(1, 1) * (*this)(2, 3) * (*this)(3, 0) +
                      (*this)(0, 2) * (*this)(1, 3) * (*this)(2, 0) * (*this)(3, 1) +
                      (*this)(0, 3) * (*this)(1, 0) * (*this)(2, 2) * (*this)(3, 1) +
                      (*this)(0, 3) * (*this)(1, 1) * (*this)(2, 0) * (*this)(3, 2) +
                      (*this)(0, 3) * (*this)(1, 2) * (*this)(2, 1) * (*this)(3, 0) -
                      (*this)(0, 0) * (*this)(1, 1) * (*this)(2, 3) * (*this)(3, 2) -
                      (*this)(0, 0) * (*this)(1, 2) * (*this)(2, 1) * (*this)(3, 3) -
                      (*this)(0, 0) * (*this)(1, 3) * (*this)(2, 2) * (*this)(3, 1) -
                      (*this)(0, 1) * (*this)(1, 0) * (*this)(2, 2) * (*this)(3, 3) -
                      (*this)(0, 1) * (*this)(1, 2) * (*this)(2, 3) * (*this)(3, 0) -
                      (*this)(0, 1) * (*this)(1, 3) * (*this)(2, 0) * (*this)(3, 2) -
                      (*this)(0, 2) * (*this)(1, 0) * (*this)(2, 3) * (*this)(3, 1) -
                      (*this)(0, 2) * (*this)(1, 1) * (*this)(2, 0) * (*this)(3, 3) -
                      (*this)(0, 2) * (*this)(1, 3) * (*this)(2, 1) * (*this)(3, 0) -
                      (*this)(0, 3) * (*this)(1, 0) * (*this)(2, 1) * (*this)(3, 2) -
                      (*this)(0, 3) * (*this)(1, 1) * (*this)(2, 2) * (*this)(3, 0) -
                      (*this)(0, 3) * (*this)(1, 2) * (*this)(2, 0) * (*this)(3, 1);
                break;
        }
        return det;
    }

    /// Returns the inverse of the matrix.
    /// Note! This method must be used only with max 4x4 matrices,
    /// otherwise it throws an exception.
    void MatrInverse() {
        assert(this->GetRows() == this->GetColumns());
        assert(this->GetRows() <= 4);
        assert(this->Det() != 0);

        if (this->GetRows() != this->GetColumns())
            throw("Cannot compute matrix inverse because rectangular matrix");
        if (this->GetRows() > 4)
            throw("Cannot compute matrix inverse because matr. larger than 4x4");
        if (this->Det() == 0)
            throw("Cannot compute matrix inverse because singular matrix");

        switch (this->GetRows()) {
            case 1:
                (*this)(0, 0) = (1 / (*this)(0, 0));
                break;
            case 2: {
                ChMatrixDynamic<Real> inv(2, 2);
                inv(0, 0) = (*this)(1, 1);
                inv(0, 1) = -(*this)(0, 1);
                inv(1, 1) = (*this)(0, 0);
                inv(1, 0) = -(*this)(1, 0);
                inv.MatrDivScale(this->Det());
                this->CopyFromMatrix(inv);
                break;
            }
            case 3: {
                ChMatrixDynamic<Real> inv(3, 3);
                inv(0, 0) = (*this)(1, 1) * (*this)(2, 2) - (*this)(1, 2) * (*this)(2, 1);
                inv(0, 1) = (*this)(2, 1) * (*this)(0, 2) - (*this)(0, 1) * (*this)(2, 2);
                inv(0, 2) = (*this)(0, 1) * (*this)(1, 2) - (*this)(0, 2) * (*this)(1, 1);
                inv(1, 0) = (*this)(1, 2) * (*this)(2, 0) - (*this)(1, 0) * (*this)(2, 2);
                inv(1, 1) = (*this)(2, 2) * (*this)(0, 0) - (*this)(2, 0) * (*this)(0, 2);
                inv(1, 2) = (*this)(0, 2) * (*this)(1, 0) - (*this)(1, 2) * (*this)(0, 0);
                inv(2, 0) = (*this)(1, 0) * (*this)(2, 1) - (*this)(1, 1) * (*this)(2, 0);
                inv(2, 1) = (*this)(0, 1) * (*this)(2, 0) - (*this)(0, 0) * (*this)(2, 1);
                inv(2, 2) = (*this)(0, 0) * (*this)(1, 1) - (*this)(0, 1) * (*this)(1, 0);
                inv.MatrDivScale(this->Det());
                this->CopyFromMatrix(inv);
                break;
            }
            case 4: {
                ChMatrixDynamic<Real> inv(4, 4);
                inv.SetElement(
                    0, 0,
                    (*this)(1, 2) * (*this)(2, 3) * (*this)(3, 1) - (*this)(1, 3) * (*this)(2, 2) * (*this)(3, 1) +
                        (*this)(1, 3) * (*this)(2, 1) * (*this)(3, 2) - (*this)(1, 1) * (*this)(2, 3) * (*this)(3, 2) -
                        (*this)(1, 2) * (*this)(2, 1) * (*this)(3, 3) + (*this)(1, 1) * (*this)(2, 2) * (*this)(3, 3));
                inv.SetElement(
                    0, 1,
                    (*this)(0, 3) * (*this)(2, 2) * (*this)(3, 1) - (*this)(0, 2) * (*this)(2, 3) * (*this)(3, 1) -
                        (*this)(0, 3) * (*this)(2, 1) * (*this)(3, 2) + (*this)(0, 1) * (*this)(2, 3) * (*this)(3, 2) +
                        (*this)(0, 2) * (*this)(2, 1) * (*this)(3, 3) - (*this)(0, 1) * (*this)(2, 2) * (*this)(3, 3));
                inv.SetElement(
                    0, 2,
                    (*this)(0, 2) * (*this)(1, 3) * (*this)(3, 1) - (*this)(0, 3) * (*this)(1, 2) * (*this)(3, 1) +
                        (*this)(0, 3) * (*this)(1, 1) * (*this)(3, 2) - (*this)(0, 1) * (*this)(1, 3) * (*this)(3, 2) -
                        (*this)(0, 2) * (*this)(1, 1) * (*this)(3, 3) + (*this)(0, 1) * (*this)(1, 2) * (*this)(3, 3));
                inv.SetElement(
                    0, 3,
                    (*this)(0, 3) * (*this)(1, 2) * (*this)(2, 1) - (*this)(0, 2) * (*this)(1, 3) * (*this)(2, 1) -
                        (*this)(0, 3) * (*this)(1, 1) * (*this)(2, 2) + (*this)(0, 1) * (*this)(1, 3) * (*this)(2, 2) +
                        (*this)(0, 2) * (*this)(1, 1) * (*this)(2, 3) - (*this)(0, 1) * (*this)(1, 2) * (*this)(2, 3));
                inv.SetElement(
                    1, 0,
                    (*this)(1, 3) * (*this)(2, 2) * (*this)(3, 0) - (*this)(1, 2) * (*this)(2, 3) * (*this)(3, 0) -
                        (*this)(1, 3) * (*this)(2, 0) * (*this)(3, 2) + (*this)(1, 0) * (*this)(2, 3) * (*this)(3, 2) +
                        (*this)(1, 2) * (*this)(2, 0) * (*this)(3, 3) - (*this)(1, 0) * (*this)(2, 2) * (*this)(3, 3));
                inv.SetElement(
                    1, 1,
                    (*this)(0, 2) * (*this)(2, 3) * (*this)(3, 0) - (*this)(0, 3) * (*this)(2, 2) * (*this)(3, 0) +
                        (*this)(0, 3) * (*this)(2, 0) * (*this)(3, 2) - (*this)(0, 0) * (*this)(2, 3) * (*this)(3, 2) -
                        (*this)(0, 2) * (*this)(2, 0) * (*this)(3, 3) + (*this)(0, 0) * (*this)(2, 2) * (*this)(3, 3));
                inv.SetElement(
                    1, 2,
                    (*this)(0, 3) * (*this)(1, 2) * (*this)(3, 0) - (*this)(0, 2) * (*this)(1, 3) * (*this)(3, 0) -
                        (*this)(0, 3) * (*this)(1, 0) * (*this)(3, 2) + (*this)(0, 0) * (*this)(1, 3) * (*this)(3, 2) +
                        (*this)(0, 2) * (*this)(1, 0) * (*this)(3, 3) - (*this)(0, 0) * (*this)(1, 2) * (*this)(3, 3));
                inv.SetElement(
                    1, 3,
                    (*this)(0, 2) * (*this)(1, 3) * (*this)(2, 0) - (*this)(0, 3) * (*this)(1, 2) * (*this)(2, 0) +
                        (*this)(0, 3) * (*this)(1, 0) * (*this)(2, 2) - (*this)(0, 0) * (*this)(1, 3) * (*this)(2, 2) -
                        (*this)(0, 2) * (*this)(1, 0) * (*this)(2, 3) + (*this)(0, 0) * (*this)(1, 2) * (*this)(2, 3));
                inv.SetElement(
                    2, 0,
                    (*this)(1, 1) * (*this)(2, 3) * (*this)(3, 0) - (*this)(1, 3) * (*this)(2, 1) * (*this)(3, 0) +
                        (*this)(1, 3) * (*this)(2, 0) * (*this)(3, 1) - (*this)(1, 0) * (*this)(2, 3) * (*this)(3, 1) -
                        (*this)(1, 1) * (*this)(2, 0) * (*this)(3, 3) + (*this)(1, 0) * (*this)(2, 1) * (*this)(3, 3));
                inv.SetElement(
                    2, 1,
                    (*this)(0, 3) * (*this)(2, 1) * (*this)(3, 0) - (*this)(0, 1) * (*this)(2, 3) * (*this)(3, 0) -
                        (*this)(0, 3) * (*this)(2, 0) * (*this)(3, 1) + (*this)(0, 0) * (*this)(2, 3) * (*this)(3, 1) +
                        (*this)(0, 1) * (*this)(2, 0) * (*this)(3, 3) - (*this)(0, 0) * (*this)(2, 1) * (*this)(3, 3));
                inv.SetElement(
                    2, 2,
                    (*this)(0, 1) * (*this)(1, 3) * (*this)(3, 0) - (*this)(0, 3) * (*this)(1, 1) * (*this)(3, 0) +
                        (*this)(0, 3) * (*this)(1, 0) * (*this)(3, 1) - (*this)(0, 0) * (*this)(1, 3) * (*this)(3, 1) -
                        (*this)(0, 1) * (*this)(1, 0) * (*this)(3, 3) + (*this)(0, 0) * (*this)(1, 1) * (*this)(3, 3));
                inv.SetElement(
                    2, 3,
                    (*this)(0, 3) * (*this)(1, 1) * (*this)(2, 0) - (*this)(0, 1) * (*this)(1, 3) * (*this)(2, 0) -
                        (*this)(0, 3) * (*this)(1, 0) * (*this)(2, 1) + (*this)(0, 0) * (*this)(1, 3) * (*this)(2, 1) +
                        (*this)(0, 1) * (*this)(1, 0) * (*this)(2, 3) - (*this)(0, 0) * (*this)(1, 1) * (*this)(2, 3));
                inv.SetElement(
                    3, 0,
                    (*this)(1, 2) * (*this)(2, 1) * (*this)(3, 0) - (*this)(1, 1) * (*this)(2, 2) * (*this)(3, 0) -
                        (*this)(1, 2) * (*this)(2, 0) * (*this)(3, 1) + (*this)(1, 0) * (*this)(2, 2) * (*this)(3, 1) +
                        (*this)(1, 1) * (*this)(2, 0) * (*this)(3, 2) - (*this)(1, 0) * (*this)(2, 1) * (*this)(3, 2));
                inv.SetElement(
                    3, 1,
                    (*this)(0, 1) * (*this)(2, 2) * (*this)(3, 0) - (*this)(0, 2) * (*this)(2, 1) * (*this)(3, 0) +
                        (*this)(0, 2) * (*this)(2, 0) * (*this)(3, 1) - (*this)(0, 0) * (*this)(2, 2) * (*this)(3, 1) -
                        (*this)(0, 1) * (*this)(2, 0) * (*this)(3, 2) + (*this)(0, 0) * (*this)(2, 1) * (*this)(3, 2));
                inv.SetElement(
                    3, 2,
                    (*this)(0, 2) * (*this)(1, 1) * (*this)(3, 0) - (*this)(0, 1) * (*this)(1, 2) * (*this)(3, 0) -
                        (*this)(0, 2) * (*this)(1, 0) * (*this)(3, 1) + (*this)(0, 0) * (*this)(1, 2) * (*this)(3, 1) +
                        (*this)(0, 1) * (*this)(1, 0) * (*this)(3, 2) - (*this)(0, 0) * (*this)(1, 1) * (*this)(3, 2));
                inv.SetElement(
                    3, 3,
                    (*this)(0, 1) * (*this)(1, 2) * (*this)(2, 0) - (*this)(0, 2) * (*this)(1, 1) * (*this)(2, 0) +
                        (*this)(0, 2) * (*this)(1, 0) * (*this)(2, 1) - (*this)(0, 0) * (*this)(1, 2) * (*this)(2, 1) -
                        (*this)(0, 1) * (*this)(1, 0) * (*this)(2, 2) + (*this)(0, 0) * (*this)(1, 1) * (*this)(2, 2));
                inv.MatrDivScale(this->Det());
                this->CopyFromMatrix(inv);
                break;
            }
        }
    }

    /// Returns true if vector is identical to other matrix
    bool Equals(const ChMatrix<Real>& other) { return Equals(other, 0.0); }

    /// Returns true if vector equals another vector, within a tolerance 'tol'
    bool Equals(const ChMatrix<Real>& other, Real tol) {
        if ((other.GetColumns() != this->columns) || (other.GetRows() != this->rows))
            return false;
        for (int nel = 0; nel < rows * columns; ++nel)
            if (fabs(ElementN(nel) - other.ElementN(nel)) > tol)
                return false;
        return true;
    }

    /// Multiplies this 3x4 matrix by a quaternion, as v=[G]*q
    /// The matrix must be 3x4.
    ///  \return The result of the multiplication, i.e. a vector.
    template <class RealB>
    ChVector<Real> Matr34_x_Quat(const ChQuaternion<RealB>& qua) {
        assert((rows == 3) && (columns == 4));
        return ChVector<Real>(Get34Element(0, 0) * (Real)qua.e0() + Get34Element(0, 1) * (Real)qua.e1() +
                                  Get34Element(0, 2) * (Real)qua.e2() + Get34Element(0, 3) * (Real)qua.e3(),
                              Get34Element(1, 0) * (Real)qua.e0() + Get34Element(1, 1) * (Real)qua.e1() +
                                  Get34Element(1, 2) * (Real)qua.e2() + Get34Element(1, 3) * (Real)qua.e3(),
                              Get34Element(2, 0) * (Real)qua.e0() + Get34Element(2, 1) * (Real)qua.e1() +
                                  Get34Element(2, 2) * (Real)qua.e2() + Get34Element(2, 3) * (Real)qua.e3());
    }

    /// Multiplies this 3x4 matrix (transposed) by a vector, as q=[G]'*v
    /// The matrix must be 3x4.
    ///  \return The result of the multiplication, i.e. a quaternion.
    template <class RealB>
    ChQuaternion<Real> Matr34T_x_Vect(const ChVector<RealB>& va) {
        assert((rows == 3) && (columns == 4));
        return ChQuaternion<Real>(
            Get34Element(0, 0) * (Real)va.x() + Get34Element(1, 0) * (Real)va.y() + Get34Element(2, 0) * (Real)va.z(),
            Get34Element(0, 1) * (Real)va.x() + Get34Element(1, 1) * (Real)va.y() + Get34Element(2, 1) * (Real)va.z(),
            Get34Element(0, 2) * (Real)va.x() + Get34Element(1, 2) * (Real)va.y() + Get34Element(2, 2) * (Real)va.z(),
            Get34Element(0, 3) * (Real)va.x() + Get34Element(1, 3) * (Real)va.y() + Get34Element(2, 3) * (Real)va.z());
    }

    /// Multiplies this 4x4 matrix (transposed) by a quaternion,
    /// The matrix must be  4x4.
    ///  \return The result of the multiplication, i.e. a quaternion.
    template <class RealB>
    ChQuaternion<Real> Matr44_x_Quat(const ChQuaternion<RealB>& qua) {
        assert((rows == 4) && (columns == 4));
        return ChQuaternion<Real>(Get44Element(0, 0) * (Real)qua.e0() + Get44Element(0, 1) * (Real)qua.e1() +
                                      Get44Element(0, 2) * (Real)qua.e2() + Get44Element(0, 3) * (Real)qua.e3(),
                                  Get44Element(1, 0) * (Real)qua.e0() + Get44Element(1, 1) * (Real)qua.e1() +
                                      Get44Element(1, 2) * (Real)qua.e2() + Get44Element(1, 3) * (Real)qua.e3(),
                                  Get44Element(2, 0) * (Real)qua.e0() + Get44Element(2, 1) * (Real)qua.e1() +
                                      Get44Element(2, 2) * (Real)qua.e2() + Get44Element(2, 3) * (Real)qua.e3(),
                                  Get44Element(3, 0) * (Real)qua.e0() + Get44Element(3, 1) * (Real)qua.e1() +
                                      Get44Element(3, 2) * (Real)qua.e2() + Get44Element(3, 3) * (Real)qua.e3());
    }

    /// Transposes only the lower-right 3x3 submatrix of a hemisimetric 4x4 matrix,
    /// used when the 4x4 matrix is a "star" matrix [q] coming from a quaternion q:
    /// the non commutative quat. product is:
    ///     q1 x q2  =  [q1]*q2  =  [q2st]*q1
    /// where [q2st] is the "semitranspose of [q2].
    void MatrXq_SemiTranspose() {
        SetElement(1, 2, -GetElement(1, 2));
        SetElement(1, 3, -GetElement(1, 3));
        SetElement(2, 1, -GetElement(2, 1));
        SetElement(2, 3, -GetElement(2, 3));
        SetElement(3, 1, -GetElement(3, 1));
        SetElement(3, 2, -GetElement(3, 2));
    }

    /// Change the sign of the 2nd, 3rd and 4th columns of a 4x4 matrix,
    /// The product between a quaternion q1 and the coniugate of q2 (q2'), is:
    ///    q1 x q2'  = [q1]*q2'   = [q1sn]*q2
    /// where [q1sn] is the seminegation of the 4x4 matrix [q1].
    void MatrXq_SemiNeg() {
        for (int i = 0; i < rows; ++i)
            for (int j = 1; j < columns; ++j)
                SetElement(i, j, -GetElement(i, j));
    }

    /// Gets the norm infinite of the matrix, i.e. the max.
    /// of its elements in absolute value.
    Real NormInf() {
        Real norm = 0;
        for (int nel = 0; nel < rows * columns; ++nel)
            if ((fabs(ElementN(nel))) > norm)
                norm = fabs(ElementN(nel));
        return norm;
    }

    /// Gets the norm two of the matrix, i.e. the square root
    /// of the sum of the elements squared.
    Real NormTwo() {
        Real norm = 0;
        for (int nel = 0; nel < rows * columns; ++nel)
            norm += ElementN(nel) * ElementN(nel);
        return (sqrt(norm));
    }

    /// Finds max value among the values of the matrix
    Real Max() {
        Real mmax = GetElement(0, 0);
        for (int nel = 0; nel < rows * columns; ++nel)
            if (ElementN(nel) > mmax)
                mmax = ElementN(nel);
        return mmax;
    }

    /// Finds min value among the values of the matrix
    Real Min() {
        Real mmin = GetElement(0, 0);
        for (int nel = 0; nel < rows * columns; ++nel)
            if (ElementN(nel) < mmin)
                mmin = ElementN(nel);
        return mmin;
    }

    /// Linear interpolation of two matrices. Parameter mx must be 0...1.
    /// [this] =(1-x)[A]+ (x)[B]    Matrices must have the same size!!
    void LinInterpolate(const ChMatrix<Real>& matra, const ChMatrix<Real>& matrb, Real mx) {
        assert(matra.columns == matrb.columns && matra.rows == matrb.rows);
        for (int nel = 0; nel < rows * columns; nel++)
            ElementN(nel) = matra.ElementN(nel) * (1 - mx) + matrb.ElementN(nel) * (mx);
    }

    /// Fills a matrix or a vector with a bilinear interpolation,
    /// from corner values (as a u-v patch).
    void RowColInterp(Real vmin, Real vmax, Real umin, Real umax) {
        for (int iu = 0; iu < GetColumns(); iu++)
            for (int iv = 0; iv < GetRows(); iv++) {
                if (GetRows() > 1)
                    Element(iv, iu) = vmin + (vmax - vmin) * ((Real)iv / ((Real)(GetRows() - 1)));
                if (GetColumns() > 1)
                    Element(iv, iu) += umin + (umax - umin) * ((Real)iu / ((Real)(GetColumns() - 1)));
            }
    }

    //
    // BOOKKEEPING
    //

    /// Paste a matrix "matra" into "this", inserting at location insrow-inscol.
    /// Normal copy for insrow=inscol=0
    template <class RealB>
    void PasteMatrix(const ChMatrix<RealB>& matra, int insrow, int inscol) {
        for (int i = 0; i < matra.GetRows(); ++i)
            for (int j = 0; j < matra.GetColumns(); ++j)
                Element(i + insrow, j + inscol) = (Real)matra.Element(i, j);
    }

    /// Paste a matrix "matra" into "this", inserting at location insrow-inscol
    /// and performing a sum with the preexisting values.
    template <class RealB>
    void PasteSumMatrix(const ChMatrix<RealB>& matra, int insrow, int inscol) {
        for (int i = 0; i < matra.GetRows(); ++i)
            for (int j = 0; j < matra.GetColumns(); ++j)
                Element(i + insrow, j + inscol) += (Real)matra.Element(i, j);
    }

    /// Paste a matrix "matra", transposed, into "this", inserting at location insrow-inscol.
    /// Normal copy for insrow=inscol=0
    template <class RealB>
    void PasteTranspMatrix(const ChMatrix<RealB>& matra, int insrow, int inscol) {
        for (int i = 0; i < matra.GetRows(); ++i)
            for (int j = 0; j < matra.GetColumns(); ++j)
                Element(j + insrow, i + inscol) = (Real)matra.Element(i, j);
    }

    /// Paste a matrix "matra", transposed, into "this", inserting at location insrow-inscol
    /// and performing a sum with the preexisting values.
    template <class RealB>
    void PasteSumTranspMatrix(const ChMatrix<RealB>& matra, int insrow, int inscol) {
        for (int i = 0; i < matra.GetRows(); ++i)
            for (int j = 0; j < matra.GetColumns(); ++j)
                Element(j + insrow, i + inscol) += (Real)matra.Element(i, j);
    }

    /// Paste a clipped portion of the matrix "matra" into "this",
    /// inserting the clip (of size nrows, ncolumns) at the location insrow-inscol.
    template <class RealB>
    void PasteClippedMatrix(const ChMatrix<RealB>& matra,
                            int cliprow,
                            int clipcol,
                            int nrows,
                            int ncolumns,
                            int insrow,
                            int inscol) {
        for (int i = 0; i < nrows; ++i)
            for (int j = 0; j < ncolumns; ++j)
                Element(i + insrow, j + inscol) = (Real)matra.Element(i + cliprow, j + clipcol);
    }
    /// Paste a clipped portion of the matrix "matra" into "this", where "this"
    /// is a vector (of ChMatrix type),
    /// inserting the clip (of size nrows, ncolumns) at the location insindex.
    template <class RealB>
    void PasteClippedMatrixToVector(const ChMatrix<RealB>& matra,
                                    int cliprow,
                                    int clipcol,
                                    int nrows,
                                    int ncolumns,
                                    int insindex) {
        for (int i = 0; i < nrows; ++i)
            for (int j = 0; j < ncolumns; ++j)
                ElementN(insindex + i * ncolumns + j) = (Real)matra.Element(cliprow + i, clipcol + j);
    }

    /// Paste a clipped portion of a vector into "this", where "this"
    /// is a matrix (of ChMatrix type),
    /// inserting the clip (of size nrows, ncolumns) at the location insindex.
    template <class RealB>
    void PasteClippedVectorToMatrix(const ChMatrix<RealB>& matra,
                                    int cliprow,
                                    int clipcol,
                                    int nrows,
                                    int ncolumns,
                                    int insindex) {
        for (int i = 0; i < nrows; ++i)
            for (int j = 0; j < ncolumns; ++j)
                Element(i + cliprow, j + clipcol) = (Real)matra.ElementN(insindex + i * ncolumns + j);
    }

    /// Paste a clipped portion of the matrix "matra" into "this", performing a sum with preexisting values,
    /// inserting the clip (of size nrows, ncolumns) at the location insrow-inscol.
    template <class RealB>
    void PasteSumClippedMatrix(const ChMatrix<RealB>& matra,
                               int cliprow,
                               int clipcol,
                               int nrows,
                               int ncolumns,
                               int insrow,
                               int inscol) {
        for (int i = 0; i < nrows; ++i)
            for (int j = 0; j < ncolumns; ++j)
#pragma omp atomic
                Element(i + insrow, j + inscol) += (Real)matra.Element(i + cliprow, j + clipcol);
    }

    /// Paste a vector "va" into the matrix.
    template <class RealB>
    void PasteVector(const ChVector<RealB>& va, int insrow, int inscol) {
        SetElement(insrow + 0, inscol, (Real)va.x());
        SetElement(insrow + 1, inscol, (Real)va.y());
        SetElement(insrow + 2, inscol, (Real)va.z());
    }

    /// Paste a vector "va" into the matrix, summing it with preexisting values.
    template <class RealB>
    void PasteSumVector(const ChVector<RealB>& va, int insrow, int inscol) {
        Element(insrow + 0, inscol) += (Real)va.x();
        Element(insrow + 1, inscol) += (Real)va.y();
        Element(insrow + 2, inscol) += (Real)va.z();
    }

    /// Paste a vector "va" into the matrix, subtracting it from preexisting values.
    template <class RealB>
    void PasteSubVector(const ChVector<RealB>& va, int insrow, int inscol) {
        Element(insrow + 0, inscol) -= (Real)va.x();
        Element(insrow + 1, inscol) -= (Real)va.y();
        Element(insrow + 2, inscol) -= (Real)va.z();
    }

    /// Paste a quaternion into the matrix.
    template <class RealB>
    void PasteQuaternion(const ChQuaternion<RealB>& qa, int insrow, int inscol) {
        SetElement(insrow + 0, inscol, (Real)qa.e0());
        SetElement(insrow + 1, inscol, (Real)qa.e1());
        SetElement(insrow + 2, inscol, (Real)qa.e2());
        SetElement(insrow + 3, inscol, (Real)qa.e3());
    }

    /// Paste a quaternion into the matrix, summing it with preexisting values.
    template <class RealB>
    void PasteSumQuaternion(const ChQuaternion<RealB>& qa, int insrow, int inscol) {
        Element(insrow + 0, inscol) += (Real)qa.e0();
        Element(insrow + 1, inscol) += (Real)qa.e1();
        Element(insrow + 2, inscol) += (Real)qa.e2();
        Element(insrow + 3, inscol) += (Real)qa.e3();
    }

    /// Paste a coordsys into the matrix.
    template <class RealB>
    void PasteCoordsys(const ChCoordsys<RealB>& cs, int insrow, int inscol) {
        PasteVector(cs.pos, insrow, inscol);
        PasteQuaternion(cs.rot, insrow + 3, inscol);
    }

    /// Returns the vector clipped from insrow, inscol.
    ChVector<Real> ClipVector(int insrow, int inscol) const {
        return ChVector<Real>(Element(insrow, inscol), Element(insrow + 1, inscol), Element(insrow + 2, inscol));
    }

    /// Returns the quaternion clipped from insrow, inscol.
    ChQuaternion<Real> ClipQuaternion(int insrow, int inscol) const {
        return ChQuaternion<Real>(Element(insrow, inscol), Element(insrow + 1, inscol), Element(insrow + 2, inscol),
                                  Element(insrow + 3, inscol));
    }

    /// Returns the coordsys clipped from insrow, inscol.
    ChCoordsys<Real> ClipCoordsys(int insrow, int inscol) const {
        return ChCoordsys<Real>(ClipVector(insrow, inscol), ClipQuaternion(insrow + 3, inscol));
    }

    //
    // MULTIBODY SPECIFIC MATH FUCTION
    //

    /// Fills a 4x4 matrix as the "star" matrix, representing quaternion cross product.
    /// That is, given two quaternions a and b, aXb= [Astar]*b
    template <class RealB>
    void Set_Xq_matrix(const ChQuaternion<RealB>& q) {
        Set44Element(0, 0, (Real)q.e0());
        Set44Element(0, 1, -(Real)q.e1());
        Set44Element(0, 2, -(Real)q.e2());
        Set44Element(0, 3, -(Real)q.e3());
        Set44Element(1, 0, (Real)q.e1());
        Set44Element(1, 1, (Real)q.e0());
        Set44Element(1, 2, -(Real)q.e3());
        Set44Element(1, 3, (Real)q.e2());
        Set44Element(2, 0, (Real)q.e2());
        Set44Element(2, 1, (Real)q.e3());
        Set44Element(2, 2, (Real)q.e0());
        Set44Element(2, 3, -(Real)q.e1());
        Set44Element(3, 0, (Real)q.e3());
        Set44Element(3, 1, -(Real)q.e2());
        Set44Element(3, 2, (Real)q.e1());
        Set44Element(3, 3, (Real)q.e0());
    }
};

}  // end namespace chrono

#endif
