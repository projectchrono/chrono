#ifndef CHCSR3MATRIX_H
#define CHCSR3MATRIX_H

#include <limits>

#include "chrono/core/ChSparseMatrix.h"
#include "chrono_mkl/ChApiMkl.h"


#define ALIGNMENT_REQUIRED true

namespace chrono{
	

	/* ChCSR3Matrix is a class that implements CSR3 sparse matrix format;
	* - The more useful constructor specifies rows, columns and nonzeros
	* - The argument "nonzeros": if 0<nonzeros<=1 specifies non-zeros/(rows*columns);
	*                            if nonzeros>1 specifies exactly the number non-zeros in the matrix.
	* - It's better to overestimate the number of non-zero elements to avoid reallocations in memory.
	* - Each of the 3 arrays is stored contiguously in memory (e.g. as needed by MKL Pardiso).
	* - The array of column indexes (colIndex) is initialized with "-1": that means that the corrisponing element in the "values" array
	*   doesn't hold any significant number, so it can be overwritten.
	* - It's preferrable to insert elements in the matrix in increasing column order to avoid rearranging.
	* - When a new element should be inserted the algorithm seeks the nearest not-initialized location (i.e. with "-1" in colIndex);
	    if it has to search too far ("max_shifts" exceeded) or if it finds no available spaces THEN it reallocates the arrays
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
	// So, if a new element should be stored (e.g. the [2,0] element) only one insignificant arrangement should be done instead of reallocating the arrays:
	// the algorithm, starting from colIndex[6] will find the nearest uninitialized space (i.e. a colIndex cell that has "-1" in it) and moves the elements
	// in order to let the new element to be written in that place!
	// When all the writing operations are performed the matrix can be "compressed" (i.e. call Compress()): all the uninitialized locations are purged.


	/*
	* Reset VS Resize
	* Reset() function initializes arrays to their default values. Always succesfull.
	* Resize() always preserve data in the arrays. The return value tells the user if the resizing has been done.
	* 
	* Reset() and Resize() eventually expands the arrays dimension (increase occupancy)
	* but they DO NOT REDUCE the occupancy. Eventually it has to be done manually with Trim().
	*/
	
	
	class ChApiMkl ChCSR3Matrix : public ChSparseMatrix
	{

	private:
		bool reallocation_occurred;
		const int array_alignment;
		bool isCompressed;
		int max_shifts;
		double* values;
		int* colIndex;
		int* rowIndex;
		int colIndex_occupancy; ///< effective occupancy of \c values (and so of \c colIndex) arrays in memory;
		///< \c colIndex_occupancy differs from \c rowIndex[rows] when a \c Compress(), \c Reset() or \c Resize occurred without a \c Trim();
		int rowIndex_occupancy;
		///< \c rowIndex_occupancy differs from \c rowIndex[rows] when a \c Compress(), \c Reset() or \c Resize occurred without a \c Trim();
		bool rowIndex_lock; ///< TRUE if the matrix should always keep the same number of element for each row
		bool colIndex_lock; ///< TRUE if the matrix elements should keep always the same position
		bool rowIndex_lock_broken;
		bool colIndex_lock_broken;
		enum symmetry_type
		{
			NO_SYMMETRY = 11,
			UPPER_SYMMETRY_POSDEF = 2,
			UPPER_SYMMETRY_INDEF = -2,
			LOWER_SYMMETRY = 20,
			STRUCTURAL_SYMMETRY = 1
		} symmetry;

		



	protected:
		void insert(int insrow, int inscol, double insval, int& col_sel);
		void initialize(int colIndex_length = 0);
		void initialize(int* nonzeros_vector);
		void initialize_ValuesColIndex();
		void copy(double* values_temp, int* colIndex_temp, bool to_internal_arrays, int insrow = 0, int col_sel = 0, int shifts = 0);

	public:
		ChCSR3Matrix(int insrow = 3, int inscol = 3, int nonzeros = 0);
		ChCSR3Matrix(int insrow, int inscol, int* nonzeros);
		virtual ~ChCSR3Matrix();

		double* GetValuesAddress() const { return values; };
		int* GetColIndexAddress() const { return colIndex; };
		int* GetRowIndexAddress() const { return rowIndex; };

		virtual void SetElement(int insrow, int inscol, double insval, bool overwrite = true) override;
		virtual double GetElement(int row, int col) override;
		double& Element(int row, int col);
		double& operator()(int row, int col) { return Element(row, col); }
		double& operator()(int index) { return Element(index / GetColumns(), index % GetColumns()); }

		virtual void PasteMatrix(ChMatrix<>* matra, int insrow, int inscol, bool overwrite = true, bool transp = false) override;
		virtual void PasteMatrixFloat(ChMatrix<float>* matra, int insrow, int inscol, bool overwrite = true, bool transp = false) override;
		virtual void PasteClippedMatrix(ChMatrix<>* matra, int cliprow, int clipcol, int nrows, int ncolumns, int insrow, int inscol, bool overwrite = true) override;

		// Size manipulation
		virtual void Reset(int nrows, int ncols, int nonzeros = 0) override;
		virtual bool Resize(int nrows, int ncols, int nonzeros = 0) override;
		void Compress(); // purge the matrix from all the unininitialized elements
		void Trim(); // trims the arrays so to have exactly the dimension needed, nothing more. (arrays are not moved)
		void Prune(double pruning_threshold = 0);

		// Auxiliary functions
		int GetColIndexLength() const { return rowIndex[rows]; };
		int GetColIndexMemOccupancy() const { return colIndex_occupancy; };
		int GetRowIndexMemOccupancy() const { return rowIndex_occupancy; };
		void GetNonZerosDistribution(int* nonzeros_vector) const;
		void SetMaxShifts(int max_shifts_new = std::numeric_limits<int>::max()) { max_shifts = max_shifts_new; };
		void SetRowIndexLock(bool on_off){ rowIndex_lock = on_off; }
		void SetColIndexLock(bool on_off){ colIndex_lock = on_off; }
		bool IsCompressed() const { return isCompressed; }
		bool IsRowIndexLockBroken() const { return rowIndex_lock_broken; }
		bool IsColIndexLockBroken() const { return colIndex_lock_broken; }
		void SetSymmetry(symmetry_type sym) { symmetry = sym; }
		symmetry_type GetSymmetry() const { return symmetry; }

		// Testing functions
		bool CheckArraysAlignment(int alignment = 0) const;
		void GetMemoryInfo() const;
		int VerifyMatrix() const;

		// Import/Export functions
		void ImportFromDatFile(std::string filepath);
		void ExportToDatFile(std::string filepath, int precision = 12) const;


	};

}; // END namespace chrono

#endif
