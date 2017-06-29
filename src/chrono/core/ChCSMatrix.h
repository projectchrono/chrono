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
// Authors: Dario Mangoni, Radu Serban
// =============================================================================

#ifndef CHCSMATRIX_H
#define CHCSMATRIX_H

#define ALIGNED_ALLOCATORS

#include <limits>

#include "chrono/core/ChAlignedAllocator.h"
#include "chrono/core/ChSparseMatrix.h"

namespace chrono {

/** \class ChSparsityPatternLearner
\brief A dummy matrix that acquires the sparsity pattern.

ChSparsityPatternLearner estimates the sparsity pattern without actually allocating any value, but the elements indexes.
Other matrices (like ChCSMatrix) can acquire the sparsity pattern information from this matrix.
*/
class ChApi ChSparsityPatternLearner : public ChSparseMatrix {
  protected:
    std::vector<std::list<int>> leadDim_list;
    bool row_major_format = true;
    int* leading_dimension;
    int* trailing_dimension;

  public:
    ChSparsityPatternLearner(int nrows, int ncols, bool row_major_format_in = true) : ChSparseMatrix(nrows, ncols) {
        row_major_format = row_major_format_in;
        leading_dimension = row_major_format ? &m_num_rows : &m_num_cols;
        trailing_dimension = row_major_format ? &m_num_cols : &m_num_rows;
        leadDim_list.resize(*leading_dimension);
    }

    virtual ~ChSparsityPatternLearner() {}

    void SetElement(int insrow, int inscol, double insval, bool overwrite = true) override {
        row_major_format ? leadDim_list[insrow].push_back(inscol) : leadDim_list[inscol].push_back(insrow);
    }

    double GetElement(int row, int col) const override { return 0.0; }

    void Reset(int row, int col, int nonzeros = 0) override {
        *leading_dimension = row_major_format ? row : col;
        *trailing_dimension = row_major_format ? col : row;
        leadDim_list.clear();
        leadDim_list.resize(*leading_dimension);
    }

    bool Resize(int nrows, int ncols, int nonzeros = 0) override {
        Reset(nrows, ncols, nonzeros);
        return true;
    }

    std::vector<std::list<int>>& GetSparsityPattern() {
        for (auto list_iter = leadDim_list.begin(); list_iter != leadDim_list.end(); ++list_iter) {
            list_iter->sort();
            list_iter->unique();
        }
        return leadDim_list;
    }

    bool IsRowMajor() const { return row_major_format; }

    int GetNNZ() const override {
        int nnz_temp = 0;
        for (auto list_iter = leadDim_list.begin(); list_iter != leadDim_list.end(); ++list_iter)
            nnz_temp += static_cast<int>(list_iter->size());

        const_cast<ChSparsityPatternLearner*>(this)->m_nnz = nnz_temp;
        return nnz_temp;
    }
};

/** \class ChCSMatrix
    \brief Class that implements Compressed Sparse matrix format in either Row (CSR) or Column (CSC) major order;

    The arrays are stored contiguously in memory (as needed by Intel MKL Pardiso) with array_alignment byte alignment.

    The class is equipped with a <em>sparsity lock</em> feature.\n
    When this feature is enabled, the matrix will hold the \e position of the elements - i.e. the sparsity pattern -
   even after calling a#Reset(). So, if an element already existed in a given position, the next call to SetElement() in
   that position will be much faster.

    \warning When the <em>sparsity lock</em> feature is turned on, also non-zero elements will be stored in the matrix
   in order to maximize the efficiency.

    While building the matrix, the most demanding operation is the \e new elements insertion (specifically, those
   elements whose indexes are not stored in the matrix yet).\n The performance is slow when: (most demanding first) 1.
   the arrays capacity is not enough to fit the matrix; 2. the arrays size is not enough to fit the matrix; 3. a new
   element must be inserted in a row that already contains elements with greater column index (swap 'column' and 'row'
   for CSC).

    Some suggestions:
    - it is better to overestimate the number of non-zero, rather than underestimate;
    - it is better to store the elements in increasing column order, even at the cost of jumping from a row to another
   (swap 'column' and 'row' for CSC); - use <em>sparsity lock</em> feature whenever possible.
*/

class ChApi ChCSMatrix : public ChSparseMatrix {
  private:
    const bool row_major_format =
        true;  ///< if \c true the arrays are stored according to CSR format, otherwise in CSC format
    const static int array_alignment = 64;  ///< alignment of arrays [byte]
    bool isCompressed = false;  ///< if \c true the arrays are compressed, so there is no not-initialized element
    int max_shifts = std::numeric_limits<int>::max();

// Compressed Storage arrays typedefs
#ifdef ALIGNED_ALLOCATORS
    typedef std::vector<int, aligned_allocator<int, array_alignment>>
        index_vector_t;  ///< type for #leadIndex and #trailIndex vectors (aligned)
    typedef std::vector<double, aligned_allocator<double, array_alignment>>
        values_vector_t;  ///< type for #value vector (aligned)
#else
    typedef std::vector<int> index_vector_t;      ///< type for #leadIndex and #trailIndex vectors (not aligned)
    typedef std::vector<double> values_vector_t;  ///< type for #value vector (not aligned)
#endif

    index_vector_t leadIndex;   ///< CS vector: #leadIndex[i] tells that #trailIndex[#leadIndex[i]] is the first element
                                ///of the i-th row (if row-major)
    index_vector_t trailIndex;  ///< CS vector: #trailIndex[j] tells the column index of #values[j] (if row-major)
    values_vector_t values;     ///< CS vector: non-zero values
    std::vector<bool> initialized_element;  ///< flag if a space in #trailIndex is initialized or not
    int* leading_dimension = nullptr;       ///< points to #m_num_rows (CSR) or #m_num_cols (CSC)
    int* trailing_dimension = nullptr;      ///< points to #m_num_cols (CSR) or #m_num_rows (CSC)

    bool m_lock_broken = false;  ///< true if a modification was made that overrules m_lock

  protected:
    /// (internal) The \a vector elements will contain equally spaced indexes, going from \a initial_number to \a
    /// final_number.
    void static distribute_integer_range_on_vector(index_vector_t& vector, int initial_number, int final_number);

    /// (internal) Really reset the internal arrays to the given dimensions.
    /// It checks if the dimensions are consistent, but no smart choice are made here.
    void reset_arrays(int lead_dim, int trail_dim, int nonzeros);

    /// (internal) Insert a non existing element in the position \a trai_i, given the row(CSR) or column(CSC) \a
    /// lead_sel
    void insert(int& trail_i, const int& lead_sel);

    /// (internal) Copy arrays from source vectors to destination vectors (source and destination vectors can coincide).
    /// In the meantime it puts a given number of not-initialized spaces (\p storage_augm) in the matrix, equally
    /// distributed between the rows.
    void copy_and_distribute(
        const index_vector_t& trailIndex_src,              ///< trailing dimension index (source)
        const values_vector_t& values_src,                 ///< values array (source)
        const std::vector<bool>& initialized_element_src,  ///< array with initialization flags (source)
        index_vector_t& trailIndex_dest,                   ///< trailing dimension index (destination)
        values_vector_t& values_dest,                      ///< values array (destination)
        std::vector<bool>& initialized_element_dest,       ///< array with initialization flags (destination)
        int& trail_ins,                                    ///< the position in the trailIndex where a new element must be inserted (the element that
                                                           ///caused the insertion)
        int lead_ins,                                      ///< the position, referred to the leading dimension, where a new element must be inserted (the
                                                           ///element that caused the insertion)
        int storage_augm                                   ///< number of not-initialized spaces to add
    );

  public:
    /// Create a CS matrix with the given dimensions.
    ChCSMatrix(int nrows = 1,                    ///< number of rows
               int ncols = 1,                    ///< number of columns
               bool row_major_format_on = true,  ///< create a Row Major matrix?
               int nonzeros = 1                  ///< number of non-zeros
    );

    /// Destructor
    ~ChCSMatrix() override {}

    void SetElement(int row_sel, int col_sel, double insval, bool overwrite = true) override;

    double GetElement(int row_sel, int col_sel) const override;

    /// Create the element with index (\a row_sel, \a col_sel) (if it doesn't exist) and return its reference.
    double& Element(int row_sel, int col_sel);

    /// Create the element with index (\a row_sel, \a col_sel)(if it doesn't exist) and return its reference.
    double& operator()(int row_sel, int col_sel) { return Element(row_sel, col_sel); }

    /// Create the element with index \a index (if it doesn't exist) and return its reference.
    double& operator()(int index) { return Element(index / m_num_cols, index % m_num_cols); }

    /// Resize the matrix in order to have \a nrows rows and \a ncols columns.
    /// The \a nonzeros_hint value is just a \e hint.\n
    /// The matrix can be\e fully reset, or \e partially reset (i.e. maintaining the sparsity pattern).\n
    /// For \e partial reset the following must apply:
    /// - \a nrows and \a ncols must not differ from the current ones;
    /// - \a nonzeros_hint must not be provided (or must equal 0);
    /// - #m_lock must be set (see ChSparseMatrix::SetSparsityPatternLock())
    /// otherwise a \e full reset will occur.
    void Reset(int nrows, int ncols, int nonzeros_hint = 0) override;

    /// Equivalent to #Reset().
    bool Resize(int nrows, int ncols, int nonzeros_hint = 0) override {
        Reset(nrows, ncols, nonzeros_hint);
        return true;
    };

    int GetNNZ() const override { return GetTrailingIndexLength(); }

    int* GetCS_LeadingIndexArray() const override;

    int* GetCS_TrailingIndexArray() const override;

    double* GetCS_ValueArray() const override;

    /// Compress the internal arrays and purge all uninitialized elements.
    bool Compress() override;

    /// Add not-initialized element to the matrix. If many new element should be inserted
    /// it can improve the speed.
    int Inflate(int storage_augm, int lead_sel = 0, int trail_sel = -1);

    /// Trims the internal arrays to have exactly the dimension needed, nothing more.
    void Trim();

    /// The same as #Compress(), but also removes elements whose absolute value is less or equal to \a
    /// pruning_threshold.
    void Prune(double pruning_threshold = 0);

    /// Get the length of the trailing-index array (e.g. column index if row major, row index if column major)
    int GetTrailingIndexLength() const { return leadIndex[*leading_dimension]; }

    /// Get the capacity of the trailing-index array (e.g. column index if row major, row index if column major)
    int GetTrailingIndexCapacity() const { return static_cast<int>(trailIndex.capacity()); }

    /// Advanced use. While inserting new elements in the matrix SetMaxShifts() tells how far in the array the internal
    /// algorithm should look for a not-initialized space.
    void SetMaxShifts(int max_shifts_new = std::numeric_limits<int>::max()) { max_shifts = max_shifts_new; }

    /// Check if the matrix is compressed i.e. if the matrix elements are stored contiguously in the arrays.
    bool IsCompressed() const { return isCompressed; }

    /// Check if the matrix is stored in row major format.
    bool IsRowMajor() const { return row_major_format; }

    /// Load the sparsity pattern from \a sparsity_learner matrix; the internal arrays will be reshaped
    /// in order to accommodate the sparsity pattern
    void LoadSparsityPattern(ChSparsityPatternLearner& sparsity_learner) override;

    /// Verify if the matrix respects the Compressed Sparse Row|Column standard.\n
    ///  3 - warning message: the row (CSR) | column (CSC) is empty\n
    ///  1 - warning message: the matrix is not compressed\n
    ///  0 - all good!\n
    /// -1 - error message: leadIndex is not strictly ascending\n
    /// -2 - error message: there are not-initialized element NOT at the end of the row (CSR) | column (CSC)\n
    /// -4 - error message: trailIndex has not ascending indexes within the rows (CSR) | columns (CSC)\n
    int VerifyMatrix() const;

    // Import/Export functions
    /// Import data from separated file (a.dat, ia.dat, ja.dat) given the format.
    void ImportFromDatFile(std::string filepath = "", bool row_major_format_on = true);
    /// Export data to separated file (a.dat, ia.dat, ja.dat) with the given precision \a precision.
    void ExportToDatFile(std::string filepath = "", int precision = 6) const;
};

};  // end namespace chrono

#endif
