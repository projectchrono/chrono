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

#define ALIGNED_ALLOCATORS

#include <limits>

#include "chrono/core/ChSparseMatrix.h"
#include "chrono/core/ChAlignedAllocator.h"

namespace chrono {

	/** \class ChSparsityPatternLearner
	\brief A dummy matrix that gets only sparsity pattern infos.

	ChSparsityPatternLearner estimates the sparsity pattern without actually allocating any value.
	It is meant to be used together with ChCSR3Matrix in order to improve the building of the latter.
	*/
	class ChApi ChSparsityPatternLearner : public ChSparseMatrix
	{
	protected:
		std::vector<std::list<int>> row_lists;
		bool row_major_format = true;
		int* leading_dimension;
		int* trailing_dimension;

	public:
		ChSparsityPatternLearner(int nrows, int ncols, bool row_major_format_in = true) :
			ChSparseMatrix(nrows, ncols)
		{
			row_major_format = row_major_format_in;
			leading_dimension = row_major_format ? &m_num_rows : &m_num_cols;
			trailing_dimension = row_major_format ? &m_num_cols : &m_num_rows;
			row_lists.resize(*leading_dimension);
		}

		virtual ~ChSparsityPatternLearner() {}

		void SetElement(int insrow, int inscol, double insval, bool overwrite = true) override
		{
			row_lists[insrow].push_back(inscol);
		}

		double GetElement(int row, int col) const override { return 0.0; }

		void Reset(int row, int col, int nonzeros = 0) override
		{
			*leading_dimension = row_major_format ? row : col;
			*trailing_dimension = row_major_format ? col : row;
			row_lists.clear();
			row_lists.resize(*leading_dimension);
		}

		bool Resize(int nrows, int ncols, int nonzeros = 0) override
		{
			Reset(nrows, ncols, nonzeros);
			return true;
		}

		std::vector<std::list<int>>& GetSparsityPattern()
		{
			for (auto list_iter = row_lists.begin(); list_iter!= row_lists.end(); ++list_iter)
			{
                list_iter->sort();
                list_iter->unique();
			}
			return row_lists;
		}

		bool isRowMajor() const { return row_major_format; }

		int GetNNZ() const override {
			int nnz_temp = 0;
            for (auto list_iter = row_lists.begin(); list_iter != row_lists.end(); ++list_iter)
				nnz_temp += static_cast<int>(list_iter->size());

			const_cast<ChSparsityPatternLearner*>(this)->m_nnz = nnz_temp;
			return nnz_temp;
		}
	};

	/// @addtogroup chrono
/// @{

/** \class ChCSR3Matrix
\brief ChCSR3Matrix is a class that implements CSR3 sparse matrix format;

 Each of the 3 CSR arrays is stored contiguously in memory (as needed by Intel MKL Pardiso).

 Building of the matrix is faster if the sparsity pattern of the matrix does not change (or change just a little).
 In order to let the matrix know that, set ChSparseMatrix::SetSparsityPatternLock(true). From now on, the position of the elements is kept in memory.
 Please mind that, when Reset() is called, only #values will be cleaned. Nothing will happen to other arrays.
 This means that if the sparsity pattern will change, many zeros will pollute your matrix. Do Prune() in this case.

 Moreover, if the ChCSR3Matrix will be built from a ChSystemDescriptor, you can provide it to the matrix through #BindToChSystemDescriptor().
 In the next call to Reset(), the ChCSR3Matrix will adapt itself to the sparsity pattern of ChSystemDescriptor.
 The sparsity pattern will be acquired from ChSystemDescriptor only once. To force the update call #ForceSparsityPatternUpdate()

 Hints:
 - It's far better to overestimate the number of non-zero elements to avoid reallocations in memory.
 - It's preferrable to insert elements in the matrix in increasing column order (if row major) to minimize re-sorting of the elements.
 - It's better to use GetElement to read from matrix; Element() creates the space if the element does not exist.
*/

class ChApi ChCSR3Matrix : public ChSparseMatrix {
  private:
    const bool row_major_format = true;
    const static int array_alignment = 64;
    bool isCompressed = false;
    int max_shifts = std::numeric_limits<int>::max();

    // CSR matrix arrays typedefs
#ifdef ALIGNED_ALLOCATORS
	typedef std::vector<int, aligned_allocator<int, array_alignment>> index_vector_t;
	typedef std::vector<double, aligned_allocator<double, array_alignment>> values_vector_t;
#else
	typedef std::vector<int> index_vector_t;
	typedef std::vector<double> values_vector_t;
#endif

	index_vector_t leadIndex;    ///< CSR vector: leadIndex[i] tells that trailIndex[leadIndex[i]] is the first element of the i-th row (if row-major)
	index_vector_t trailIndex;   ///< CSR vector: trailIndex[j] tells the column index of values[j]
	values_vector_t values;      ///< CSR vector: non-zero valuess
	std::vector<bool> initialized_element;    ///< flag if a space in #trailIndex is initialized or not
	int* leading_dimension = nullptr;    ///< points to m_num_rows or m_num_cols depending on format
    int* trailing_dimension = nullptr;    ///< points to m_num_cols or m_num_rows depending on format

    bool m_lock_broken = false;  ///< true if a modification was made that overrules m_lock

  protected:
	void static distribute_integer_range_on_vector(index_vector_t& vector, int initial_number, int final_number);
	void reset_arrays(int lead_dim, int trail_dim, int nonzeros);
	void insert(int& trail_sel, const int& lead_sel);
	void copy_and_distribute(const index_vector_t& trailIndex_src,
							 const values_vector_t& values_src,
							 const std::vector<bool>& initialized_element_src,
							 index_vector_t& trailIndex_dest,
							 values_vector_t& values_dest,
							 std::vector<bool>& initialized_element_dest,
							 int& trail_ins, int lead_ins,
							 int storage_augm);

  public:
    ChCSR3Matrix(int nrows = 1, int ncols = 1, bool row_major_format_on = true, int nonzeros = 1);
    ~ChCSR3Matrix() override {}

    void SetElement(int row_sel, int col_sel, double insval, bool overwrite = true) override;
    double GetElement(int row_sel, int col_sel) const override;

    double& Element(int row_sel, int col_sel);
    double& operator()(int row_sel, int col_sel) { return Element(row_sel, col_sel); }
    double& operator()(int index) { return Element(index / m_num_cols, index % m_num_cols); }

    void Reset(int nrows, int ncols, int nonzeros_hint = 0) override;
    bool Resize(int nrows, int ncols, int nonzeros_hint = 0) override {
        Reset(nrows, ncols, nonzeros_hint);
        return true;
    };

    /// Get the number of non-zero elements in this matrix.
    int GetNNZ() const override { return GetTrailingIndexLength(); }

    /// Return the row index array in the CSR representation of this matrix if in row major format.
	/// Return the column index array if in col major format.
	int* GetCSR_LeadingIndexArray() const override;

    /// Return the column index array in the CSR representation of this matrix if in row major format.
	/// Return the row index array if in col major format.
	int* GetCSR_TrailingIndexArray() const override;

    /// Return the array of matrix values in the CSR representation of this matrix.
	double* GetCSR_ValueArray() const override;

    /// Compress the internal arrays and purge all uninitialized elements.
    bool Compress() override;

    /// Trims the internal arrays to have exactly the dimension needed, nothing more.
	/// The underlying vectors are not resized (see Trim() for this), nor moved. 
    void Trim();

    /// The same as Compress(), but also removes elements below \p pruning_threshold.
    void Prune(double pruning_threshold = 0);

	/// Get the length of the trailing-index array (e.g. column index if row major, row index if column major)
	int GetTrailingIndexLength() const { return leadIndex[*leading_dimension]; }

	/// Get the capacity of the trailing-index array (e.g. column index if row major, row index if column major)
    int GetTrailingIndexCapacity() const { return static_cast<int>(trailIndex.capacity()); }

    /// Advanced use. While setting new elements in the matrix, SetMaxShifts() tells how far the internal algorithm
    /// should look for not-initialized elements.
    void SetMaxShifts(int max_shifts_new = std::numeric_limits<int>::max()) { max_shifts = max_shifts_new; }

    /// Check if the matrix is compressed i.e. the matrix elements are stored contiguously in the arrays.
    bool IsCompressed() const { return isCompressed; }

    /// Check if the matrix is stored in row major format.
    bool IsRowMajor() const { return row_major_format; }

    void LoadSparsityPattern(ChSparsityPatternLearner& sparsity_learner) override;

    int VerifyMatrix() const;

    // Import/Export functions
	void ImportFromDatFile(std::string filepath = "", bool row_major_format_on = true);
    void ExportToDatFile(std::string filepath = "", int precision = 6) const;

};

	/// @} chrono

};  // end namespace chrono

#endif
