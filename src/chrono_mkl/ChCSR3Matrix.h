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

#ifndef CHCSR3MATRIX_H
#define CHCSR3MATRIX_H

#include <limits>
#include <string>

#include "chrono/core/ChSparseMatrix.h"
#include "chrono_mkl/ChApiMkl.h"

namespace chrono {

/// @addtogroup mkl_module
/// @{

/* ChCSR3Matrix is a class that implements CSR3 sparse matrix format;
* - The more useful constructor specifies rows, columns and nonzeros
* - The argument "nonzeros": if 0<nonzeros<=1 specifies non-zeros/(rows*columns);
*                            if nonzeros>1 specifies exactly the number non-zeros in the matrix.
* - It's better to overestimate the number of non-zero elements to avoid reallocations in memory.
* - Each of the 3 arrays is stored contiguously in memory (e.g. as needed by MKL Pardiso).
* - The array of column indexes (colIndex) is initialized with "-1": that means that the corrisponing element in the
"values" array
*   doesn't hold any significant number, so it can be overwritten.
* - It's preferrable to insert elements in the matrix in increasing column order to avoid rearranging.
* - When a new element should be inserted the algorithm seeks the nearest not-initialized location (i.e. with "-1" in
colIndex);
    if it has to search too far ("max_shifts" exceeded) or if it finds no available spaces THEN it reallocates the
arrays
* It's better to use GetElement to read from matrix; Element() creates the space if the element does not exist.
*/

// The CSR3 format for a 3x3 matrix is like this:
//  | 1.1  1.2  1.3 |    values =   { 1.1, 1.2, 1.3, 2.2, 2.3, 3.3 };
//  |  0   2.2  2.3 |	 colIndex = {  0,   1,   2,   1,   2,   2  };
//  |  0    0   3.3 |	 rowIndex = {  0,             3,        5  , 6};
// but it's difficult to have an exact estimate of how many nonzero element there will be before actually storing them;
// so how many location should be preallocated? an overestimation is usually preferred to avoid further reallocations.
// Let's say that we would like to allocate all the 9 elements: (NI means Not Initialized)
//  | 1.1  1.2  1.3 |    values =   { 1.1, 1.2, 1.3, 2.2, 2.3, NI, 3.3, NI, NI };
//  |  0   2.2  2.3 |	 colIndex = {  0,   1,   2,   1,   2,  -1,  2,  -1, -1 };
//  |  0    0   3.3 |	 rowIndex = {  0,             3,            6,          , 9 };
// So, if a new element should be stored (e.g. the [2,0] element) only one insignificant arrangement should be done
// instead of reallocating the arrays:
// the algorithm, starting from colIndex[6] will find the nearest uninitialized space (i.e. a colIndex cell that has
// "-1" in it) and moves the elements
// in order to let the new element to be written in that place!
// When all the writing operations are performed the matrix can be "compressed" (i.e. call Compress()): all the
// uninitialized locations are purged.

/*
* Reset VS Resize
* Reset() function initializes arrays to their default values. Always succesfull.
* Resize() always preserve data in the arrays. The return value tells the user if the resizing has been done.
*
* Reset() and Resize() eventually expands the arrays dimension (increase occupancy)
* but they DO NOT REDUCE the occupancy. Eventually it has to be done manually with Trim().
*/

class ChApiMkl ChCSR3Matrix : public ChSparseMatrix {
  private:
    const int array_alignment = 64;
    bool isCompressed = false;
    int max_shifts = std::numeric_limits<int>::max();

    // CSR matrix arrays.
    // Note that m_capacity may be larger than NNZ before a call to Trim()
    int m_capacity;  ///< actual size of 'colIndex' and 'values' arrays in memory
    double* values;  ///< array of matrix values (length: m_capacity)
    int* colIndex;   ///< array of column indices (length: m_capacity)
    int* rowIndex;   ///< array of row indices (length: m_num_rows+1)

    bool m_lock_broken = false;  ///< true if a modification was made that overrules m_lock

  protected:
    void insert(int insrow, int inscol, double insval, int& col_sel);
    void initialize(int colIndex_length = 0);
    void initialize(int* nonzeros_vector);
    void initialize_ValuesColIndex();
    void copy(double* values_temp,
              int* colIndex_temp,
              bool to_internal_arrays,
              int insrow = 0,
              int col_sel = 0,
              int shifts = 0);

  public:
    ChCSR3Matrix(int nrows = 1, int ncols = 1, int nonzeros = 1);
    ChCSR3Matrix(int nrows, int ncols, int* nonzeros);
    virtual ~ChCSR3Matrix();

    virtual void SetElement(int insrow, int inscol, double insval, bool overwrite = true) override;
    virtual double GetElement(int row, int col) override;

    //double& Element(int row, int col);
    double& Element(int row, int col);
    double& operator()(int row, int col) { return Element(row, col); }
    double& operator()(int index) { return Element(index / m_num_cols, index % m_num_cols); }

    // Size manipulation
    virtual void Reset(int nrows, int ncols, int nonzeros = 0) override;
    virtual bool Resize(int nrows, int ncols, int nonzeros = 0) override {
        Reset(nrows, ncols, nonzeros);
        return true;
    }

    /// Get the number of non-zero elements in this matrix.
    virtual int GetNNZ() const override { return rowIndex[m_num_rows]; }

    /// Return the row index array in the CSR representation of this matrix.
    virtual int* GetCSR_RowIndexArray() const override { return rowIndex; }

    /// Return the column index array in the CSR representation of this matrix.
    virtual int* GetCSR_ColIndexArray() const override { return colIndex; }

    /// Return the array of matrix values in the CSR representation of this matrix.
    virtual double* GetCSR_ValueArray() const override { return values; }

    /// Compress the internal arrays and purge all uninitialized elements.
    virtual bool Compress() override;

    /// Trims the internal arrays to have exactly the dimension needed, nothing more.
    /// Data arrays are not moved.
    void Trim();
    void Prune(double pruning_threshold = 0);

    // Auxiliary functions
    int GetColIndexLength() const { return rowIndex[m_num_rows]; }
    int GetColIndexCapacity() const { return m_capacity; }
    void GetNonZerosDistribution(int* nonzeros_vector) const;
    void SetMaxShifts(int max_shifts_new = std::numeric_limits<int>::max()) { max_shifts = max_shifts_new; }
    bool IsCompressed() const { return isCompressed; }

    // Testing functions
    bool CheckArraysAlignment(int alignment = 0) const;
    void GetMemoryInfo() const;
    int VerifyMatrix() const;
    int VerifyMatrixByMKL() const;

    // Import/Export functions
    void ImportFromDatFile(std::string filepath);
    void ExportToDatFile(std::string filepath, int precision = 12) const;
};

/// @} mkl_module

};  // end namespace chrono

#endif
