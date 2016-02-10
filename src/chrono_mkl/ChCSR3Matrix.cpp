#include "ChCSR3Matrix.h"
#include <algorithm>
#include <mkl.h>

namespace chrono {

	ChCSR3Matrix::ChCSR3Matrix(int insrow, int inscol, int nonzeros) :
		reallocation_occurred(false),
		array_alignment(64),
		isCompressed(false),
		max_shifts(std::numeric_limits<int>::max()),
		rowIndex_lock(false),
		colIndex_lock(false),
		rowIndex_lock_broken(false),
		colIndex_lock_broken(false),
		symmetry(NO_SYMMETRY)
	{
		assert(insrow > 0 && inscol > 0 && nonzeros >= 0);

		rows = insrow;
		columns = inscol;
		

		if (nonzeros == 0) nonzeros = static_cast<int>(rows*columns*SPM_DEF_FULLNESS);

		colIndex_occupancy = std::max(rows, nonzeros);
		rowIndex_occupancy = rows + 1;
		
		
		values = static_cast<double*>(mkl_malloc(colIndex_occupancy*sizeof(double), array_alignment));
		colIndex = static_cast<int*>(mkl_malloc(colIndex_occupancy*sizeof(int), array_alignment));
		rowIndex = static_cast<int*>(mkl_malloc(rowIndex_occupancy*sizeof(int), array_alignment));

		initialize();

	}

	ChCSR3Matrix::ChCSR3Matrix(int insrow, int inscol, int* nonzeros_vector) :
		reallocation_occurred(false),
		array_alignment(64),
		isCompressed(false),
		max_shifts(std::numeric_limits<int>::max()),
		rowIndex_lock(false),
		colIndex_lock(false),
		rowIndex_lock_broken(false),
		colIndex_lock_broken(false),
		symmetry(NO_SYMMETRY)
	{
		assert(insrow > 0 && inscol > 0);

		rows = insrow;
		columns = inscol;

		colIndex_occupancy = 0;
		for (int row_sel = 0; row_sel < rows; row_sel++)
		{
			colIndex_occupancy += nonzeros_vector[row_sel];
		}

		rowIndex_occupancy = rows + 1;

		
		values = static_cast<double*>(mkl_malloc(colIndex_occupancy*sizeof(double), array_alignment));
		colIndex = static_cast<int*>(mkl_malloc(colIndex_occupancy*sizeof(int), array_alignment));
		rowIndex = static_cast<int*>(mkl_malloc(rowIndex_occupancy*sizeof(int), array_alignment));

		initialize(nonzeros_vector);

	}



	ChCSR3Matrix::~ChCSR3Matrix()
	{
		mkl_free(values);
		mkl_free(colIndex);
		mkl_free(rowIndex);
	}


	void ChCSR3Matrix::SetElement(int insrow, int inscol, double insval, bool overwrite)
	{
		assert(insrow < rows && inscol < columns);
		assert(insrow >= 0 && inscol >= 0);

		if ((symmetry == UPPER_SYMMETRY_POSDEF || symmetry == UPPER_SYMMETRY_INDEF) && insrow<inscol ||
			symmetry == LOWER_SYMMETRY && insrow>inscol)
			return;

		int col_sel = rowIndex[insrow];
		while (1)
		{
			// case: element not found in the row OR another element with a higher col number is already been stored
			if (col_sel >= rowIndex[insrow + 1] || colIndex[col_sel] > inscol){
				if (insval!=0) // avoid to insert zero elements
					insert(insrow, inscol, insval, col_sel);
				break;
			}

			// case: empty space
			if (colIndex[col_sel] == -1)
			{
				if (insval != 0) // avoid to insert zero elements
				{
					values[col_sel] = insval;
					colIndex[col_sel] = inscol;
				}
				break;
			}

			// case: element already allocated
			if (colIndex[col_sel] == inscol)
			{
				if (overwrite) // allows to write zeros
					values[col_sel] = insval;
				else
					values[col_sel] += insval;
				break;
			}
			col_sel++;
		}

	}



	/// This function arranges the space to let the new element be put in the arrays.
	void ChCSR3Matrix::insert(int insrow, int inscol, double insval, int& col_sel)
	{
		colIndex_lock_broken = true;
		int col_shift = 1; // an offset from the current position that points to the empty location found
		int col_sel_empty = col_sel; // the location in which the new element will be put (if a space will be found it will be different from col_sel)

		/****************** STEP 1 ******************/

		// "only-one-uninitialized-cell" row check: it starts from the row following/preceding the one you are in;
		// this is because if you are inserting in your own row there is no interest to check for "only-one-uninitialized-cell" row 
		int row_sel_bw = insrow - 1;
		int row_sel_fw = insrow + 1;
			
		// STEP 1: find an empty space in the array so part of the array can be shifted in order to give space to the new element
		// There are 3 While cycles; they all search for the NEAREST empty space (i.e. in which the colIndex array has a "-1"); no rearrangement is done at this stage.
		// 1st While: it scans both Backward and Forward, but only until at least ONE of the limits of the colIndex is reached;
		// 2nd While: it scans only Backward, but only if the 1st cycle did not find anything AND the beginning of colIndex is not reached yet;
		// 3rd While: it scans only Forward, but only if the 1st cycle did not find anything AND the end of colIndex is not reached yet;
		// These 3 cycles can be made one introducing a check on the limits of the array (in the IFs in the first While), but
		// this will introduce another 2 more condition check that have to be done at every iteration also if they'll be hit very rarely.
		while (col_shift < max_shifts && col_sel - col_shift>-1 && col_sel + col_shift < rowIndex[rows]) // 1st While
		{
			if (colIndex[col_sel - col_shift] == -1 && !rowIndex_lock) // backward check
			{
				// This part is very specific: it avoids to write to another row that has only one element that it's uninitialized;
				for (; rowIndex[row_sel_bw] > col_sel - col_shift && row_sel_bw >= 0; row_sel_bw--) {}
				if (rowIndex[row_sel_bw] == col_sel - col_shift)
				{
					if (row_sel_bw == 0)
						break;
					col_shift++;
					continue;
				}

				col_sel_empty = col_sel - col_shift;
				break;
			}

			if (colIndex[col_sel + col_shift] == -1) // forward check
			{
				// This part is very specific: it avoids to write to another row that has only one element that it's uninitialized;
				for (; rowIndex[row_sel_fw] < col_sel + col_shift && row_sel_fw <= rows; row_sel_fw++) {}
				if (rowIndex[row_sel_fw] == col_sel + col_shift)
				{
					if (row_sel_fw == rows)
						break;
					col_shift++;
					continue;
				}

				col_sel_empty = col_sel + col_shift;
				break;
			}

			col_shift++;
		} // end 1st While
		

		// 2nd While: scan the last elements not already checked to the left (backward)
		while (!rowIndex_lock && col_sel_empty == col_sel && col_shift < max_shifts && col_sel - col_shift>-1)
		{
			if (colIndex[col_sel - col_shift] == -1) // backward check
			{
				// This part is very specific: it avoids to write to another row that has only one element that it's uninitialized;
				for (; rowIndex[row_sel_bw] > col_sel - col_shift && row_sel_bw >= 0; row_sel_bw--) {}
				if (rowIndex[row_sel_bw] == col_sel - col_shift)
				{
					if (row_sel_bw == 0)
						break;
					col_shift++;
					continue;
				}

				col_sel_empty = col_sel - col_shift;
				break;
			}
			col_shift++;
		}
		

		// 3rd While: scan the last elements not already checked to the right (forward)
		while (col_sel_empty == col_sel && col_shift < max_shifts && col_sel + col_shift < rowIndex[rows])
		{
			if (colIndex[col_sel + col_shift] == -1) // forward check
			{
				// This part is very specific: it avoids to write to another row that has only one element that it's uninitialized;
				for (; rowIndex[row_sel_fw] < col_sel + col_shift && row_sel_fw <= rows; row_sel_fw++) {}
				if (rowIndex[row_sel_fw] == col_sel + col_shift)
				{
					if (row_sel_fw == rows)
						break;
					col_shift++;
					continue;
				}

				col_sel_empty = col_sel + col_shift;
				break;
			}
			col_shift++;
		}
		
		// If an uninitialized location is found "col_sel_empty" should point at it, so it would be different from "col_sel".

		/****************** STEP 2 ******************/
		
		// STEP 2: shift the array to make space for the new element; eventually update "col_sel"
		// case 1: the uninitialized location is found forward;
		// case 2: the uninitialized location is found backward;
		// case 3: the location is not found in the neighborhood ("max_shifts") of the "col_sel" cell; a reallocation is needed.

		// case 1
		if (col_sel_empty > col_sel && col_sel + col_shift < rowIndex[rows] && col_shift < max_shifts)
		{
			for (int col_sel_temp = col_sel_empty; col_sel_temp > col_sel; col_sel_temp--)
			{
				values[col_sel_temp] = values[col_sel_temp - 1];
				colIndex[col_sel_temp] = colIndex[col_sel_temp - 1];
			}

			for (int row_sel = insrow + 1; rowIndex[row_sel] < col_sel_empty; row_sel++)
			{
				rowIndex[row_sel]++;
			}
		}
		// case 2
		else if (col_sel_empty < col_sel && col_sel - col_shift>-1 && col_shift < max_shifts)
		{
			assert(!rowIndex_lock);
			col_sel--;
			for (int col_sel_temp = col_sel_empty; col_sel_temp < col_sel; col_sel_temp++)
			{
				values[col_sel_temp] = values[col_sel_temp + 1];
				colIndex[col_sel_temp] = colIndex[col_sel_temp + 1];
			}

			for (int row_sel = insrow; row_sel > -1 && rowIndex[row_sel] > col_sel_empty; row_sel--)
			{
				rowIndex[row_sel]--;
			}
		}
		// case 3
		else
		{
			rowIndex_lock_broken = true;

			if (colIndex_occupancy > GetColIndexLength()) // that happens when a Compress() or a Reset() is not followed by a Trim()
			{
				copy(values, colIndex, false, insrow, col_sel, 1);
			}
			else
			{ // effective reallocation

				int storage_augmentation = 4;
				colIndex_occupancy = colIndex_occupancy + storage_augmentation;

				
				if (ALIGNMENT_REQUIRED)
				{
					double* new_values = static_cast<double*>(mkl_malloc(colIndex_occupancy*sizeof(double), array_alignment));
					int* new_colIndex = static_cast<int*>(mkl_malloc(colIndex_occupancy*sizeof(int), array_alignment));
					reallocation_occurred = true;
					copy(new_values, new_colIndex, false, insrow, col_sel, storage_augmentation);
					if (new_values != values) mkl_free(values);
					if (new_colIndex != colIndex) mkl_free(colIndex);
					values = new_values;
					colIndex = new_colIndex;
				}
				else
				{
					values = static_cast<double*>(mkl_realloc(values, colIndex_occupancy*sizeof(double)));
					colIndex = static_cast<int*>(mkl_realloc(colIndex, colIndex_occupancy*sizeof(int)));
					reallocation_occurred = true;
					copy(values, colIndex, false, insrow, col_sel, storage_augmentation);
				}

				
			} // end effective reallocation

			
		} // end case 3

		// In any case the new location should has been found; write the new values
		values[col_sel] = insval;
		colIndex[col_sel] = inscol;
	}




	/** Initialize the arrays giving, for each row, the exact space needed;
	* \param[in] nonzeros_vector array of integer; length equal to row number;
	*	\a nonzeros_vector[i] tells how many nonzeros there will be on row \a i
	*/
	void ChCSR3Matrix::initialize(int* nonzeros_vector)
	{
		// rowIndex is initialized based on nonzeros_vector specification
		rowIndex[0] = 0;
		for (int row_sel = 0; row_sel < rows; row_sel++) // rowIndex is initialized with equally spaced indexes
		{
			rowIndex[row_sel + 1] = rowIndex[row_sel] + nonzeros_vector[row_sel];
		}

		initialize_ValuesColIndex();
	}

	void ChCSR3Matrix::initialize(int colIndex_length)
	{
		if (!rowIndex_lock || rowIndex_lock_broken)
		{
			if (colIndex_length == 0)
				colIndex_length = colIndex_occupancy;

			// rowIndex is initialized with equally spaced indexes
			for (int row_sel = 0; row_sel <= rows; row_sel++)
				rowIndex[row_sel] = static_cast<int>(ceil(static_cast<double>(row_sel)* (static_cast<double>(colIndex_length)+1.0) / (static_cast<double>(rows)+1.0)));

			isCompressed = false;
		}

		initialize_ValuesColIndex();
	}

	void ChCSR3Matrix::initialize_ValuesColIndex()
	{
		if (colIndex_lock && !colIndex_lock_broken)
			for (int col_sel = 0; col_sel < GetColIndexLength(); col_sel++)
				values[col_sel] = 0;

		else
		{
			// colIndex is initialized with -1; it means that the cell has been stored but contains an uninitialized value
			for (int col_sel = 0; col_sel < GetColIndexLength(); col_sel++)
				colIndex[col_sel] = -1;

			isCompressed = false;
		}
		
	}

	/** Copies from/to \c values and \c colIndex arrays to/from the specified arrays \c values_temp and \c colIndex_temp.
	* Meanwhile he is coping it can also shift the destination array in this way:
	*	- every row > \c insrow is copied into the new array shifted by \c shifts location
	*	- the \c insrow row is modified (if \shifts >0):
	*		1. at \c col_sel a new space is created;
	*		2. at the end of the row the remaining \c shifts-1 location are created.
	*	- every row < \c insrow is copied as is
	*/

	void ChCSR3Matrix::copy(double* values_temp, int* colIndex_temp, bool to_internal_arrays, int insrow, int col_sel, int shifts)
	{
		double* values_destination, *values_source;
		int* colIndex_destination, *colIndex_source;

		// set the destination and source arrays depending on "to_internal_arrays"
		if (to_internal_arrays)
		{
			values_destination = values;
			colIndex_destination = colIndex;
			values_source = values_temp;
			colIndex_source = colIndex_temp;
		}
		else
		{
			values_destination = values_temp;
			colIndex_destination = colIndex_temp;
			values_source = values;
			colIndex_source = colIndex;
		}
		
		// Suppose this is a piece of colIndex and we should put a new element in position 7 and increase the size of 4 spaces in total
		// (1 space for the new element + 3 not initialized yet); || indicates new row, | new column
		//                                                |<-- down here the new element should be put (where now there's 14)
		// The source array			||11|19|23|42|56||12|14|17|47||26||31|49||21|39|44||~|				<-- won't be touched (unless we are performing a selfcopy)
		// The destination array	|| | | | | || | | | ||  ||  |  ||  |  |  ||  | | | | |		<-- 4 more allocated spaces to allow expansion
		int remaining_shifts = shifts;
		int col_sel_destination;
		for (col_sel_destination = rowIndex[rows] + shifts - 1; col_sel_destination - remaining_shifts >= rowIndex[insrow + 1]; col_sel_destination--)
		{
			values_destination[col_sel_destination] = values_source[col_sel_destination - remaining_shifts];
			colIndex_destination[col_sel_destination] = colIndex_source[col_sel_destination - remaining_shifts];
		}
		// The destination array	|| | | | | || | | | ||  ||  |  ||  |26|31||49|21|39|44|~|	
		
		for (remaining_shifts = shifts; remaining_shifts>1; remaining_shifts--, col_sel_destination--)
		{
			colIndex_destination[col_sel_destination] = -1;
		}
		// The destination array	|| | | | | || | | | ||  ||-1|-1||-1|26|31||49|21|39|44|~|	

		while (col_sel_destination - remaining_shifts >= col_sel)
		{
			values_destination[col_sel_destination] = values_source[col_sel_destination - remaining_shifts];
			colIndex_destination[col_sel_destination] = colIndex_source[col_sel_destination - remaining_shifts];
			col_sel_destination--;
		}
		// The destination array	|| | | | | || | |14|17||47||-1|-1||-1|26|31||49|21|39|44|~|
			

		if (values_destination!=values_source || colIndex_destination!=colIndex_source)
		{
			while (col_sel_destination>= 0)
			{
				values_destination[col_sel_destination] = values_source[col_sel_destination];
				colIndex_destination[col_sel_destination] = colIndex_source[col_sel_destination];
				col_sel_destination--;
			}
		}
		// The destination array	||11|19|23|42|56||12|14|14|17|47|-1|-1|-1||26||31|49||21|39|44||~|

		// update of rowIndex
		if (shifts > 0)
		{
			int row_sel = insrow + 1;
			while (row_sel <= rows)
			{
				rowIndex[row_sel] += shifts;
				row_sel++;
			}
		}
		// The destination array	||11|19|23|42|56||12|14|14|17|47|-1|-1|-1||26||31|12||21|39|15||~|
			
	}

	void ChCSR3Matrix::GetNonZerosDistribution(int* nonzeros_vector) const
	{
		for (int row_sel = 0; row_sel < rows; row_sel++)
			nonzeros_vector[row_sel] = rowIndex[row_sel + 1] - rowIndex[row_sel];
	}

	bool ChCSR3Matrix::CheckArraysAlignment(int alignment) const
	{
		if (alignment == 0)
			alignment = array_alignment;
		double integ_part_dummy = 0;
		double dec_part_a = modf(static_cast<double>(reinterpret_cast<uintptr_t>(values)) / alignment, &integ_part_dummy);
		double dec_part_ia = modf(static_cast<double>(reinterpret_cast<uintptr_t>(rowIndex)) / alignment, &integ_part_dummy);
		double dec_part_ja = modf(static_cast<double>(reinterpret_cast<uintptr_t>(colIndex)) / alignment, &integ_part_dummy);

		return (dec_part_a == 0 && dec_part_ia == 0 && dec_part_ja == 0) ? true : false;
	}

	void ChCSR3Matrix::GetMemoryInfo() const
	{
		printf("\nMemory allocated: %.2f MB", static_cast<double>( (2*colIndex_occupancy*sizeof(double) + rowIndex_occupancy* sizeof(int)) )/1000000  );
	}

	// Verify Matrix; output:
	//  3 - warning message: in the row there are no initialized elements
	//  1 - warning message: the matrix is not compressed
	//  0 - all good!
	// -1 - error message: rowIndex is not strictly ascending
	// -2 - error message: there's a row that has some an uninitialized element NOT at the end of its space in colIndex
	// -4 - error message: colIndex has not ascending indexes within the rows

	int ChCSR3Matrix::VerifyMatrix() const
	{
		bool uninitialized_elements_found = false;
		for (int row_sel = 0; row_sel < rows; row_sel++)
		{
			// Check ascending order of rowIndex
			if (rowIndex[row_sel] >= rowIndex[row_sel + 1])
				return -1;

			bool initialized_elements_found = false;

			int col_sel = rowIndex[row_sel + 1];
			while (col_sel>rowIndex[row_sel])
			{
				col_sel--;
				if (colIndex[col_sel] == -1)
				{
					uninitialized_elements_found = true;
					if (initialized_elements_found)
						return -2;
				}
				else
				{
					initialized_elements_found = true;

					if (col_sel>rowIndex[row_sel] && colIndex[col_sel] <= colIndex[col_sel-1])
						return -4;
				}
			}
			if (!initialized_elements_found)
				return 3;

		}
		return (uninitialized_elements_found)? 1:0;
	}

	void ChCSR3Matrix::ImportFromDatFile(std::string path)
	{
		std::ifstream a_file, ia_file, ja_file;
		a_file.open(path+"a.dat");
		ja_file.open(path+"ja.dat");
		ia_file.open(path+"ia.dat");

		if (!a_file.is_open())
			assert(0);

		int row_sel = -1;
		for (row_sel = 0; row_sel<=rows; row_sel++)
			ia_file >> rowIndex[row_sel];
		row_sel--;

		Reset(rows, columns);
		
		ia_file.seekg(0);

		row_sel = -1;
		for (row_sel = 0; row_sel <= rows; row_sel++)
			ia_file >> rowIndex[row_sel];
		row_sel--;

		int col_sel = -1;
		for (col_sel = 0; col_sel < rowIndex[row_sel]; col_sel++)
		{
			a_file >> values[col_sel];
			ja_file >> colIndex[col_sel];
		}

		if (col_sel!= rowIndex[row_sel])
			assert(0);

		a_file.close();
		ja_file.close();
		ia_file.close();

	}

	double ChCSR3Matrix::GetElement(int row, int col)
	{
		assert(row < rows && col < columns);
		assert(row >= 0 && col >= 0);
		for (int col_sel = rowIndex[row]; col_sel < rowIndex[row + 1]; col_sel++)
		{
			if (colIndex[col_sel] == col)
			{
				return values[col_sel];
			}
		}
		return 0;
	}

	double& ChCSR3Matrix::Element(int row, int col)
	{
		assert(row < rows && col < columns);
		assert(row >= 0 && col >= 0);

		// It scans the array SINCE it finds the place in which the element should be;
		// case 1a: the beginning of the next row is reached: a rearrangement is needed
		// case 1b: an already-stored element is found with a bigger column index: a rearrangement is needed
		// case 2: an empty location is found: no arrangement needed. (colIndex from the beginning of the row until col_sel should be non-"-1" i.e. should have stored element)
		// case 3: an element with exactly the same column index is found
		// IN ANY CASE, exiting the WHILE cycle, "col_sel" should point to the place where the new element should be!

		int col_sel = rowIndex[row];
		while (1)
		{
			if (col_sel >= rowIndex[row + 1] || colIndex[col_sel] > col){ // case 1a and 1b
				insert(row, col, 0.0, col_sel);
				//TODO: if Element is called as a constant i.e. as a rvalue you could not-write the value and return 0 without storing any extra space
				break;
			}

			if (colIndex[col_sel] == -1) // case 2
			{
				values[col_sel] = 0.0;
				//TODO: if Element is called as a constant i.e. as a rvalue you could not-write the value and return 0 without storing any extra space
				colIndex[col_sel] = col;
				break;
			}

			if (colIndex[col_sel] == col)
			{
				break;
			}
			col_sel++;
		}

		return values[col_sel];
	}


	/** Resize. If it will be asked to:
	*  - LeaveStoreAsIs: \c nonzeros==0    the algorithm doesn't change the occupancy of the \c values and \c colIndex arrays
	*        -# if rows++ (row_increment) a certain amount of space is allocated for each new row
	*        -# if rows== nothing will happen
	*  - SetStorage: \c nonzeros>0    the new dimension is prescribed.
	*        -# if rows++ it's verified that the prescribed dimension fits the row increment (there must be at least one element for each new row)
	*        -# if rows== ONLY the last row is expanded and filled with uninitialized spaces
	*/

	bool ChCSR3Matrix::Resize(int nrows, int ncols, int nonzeros)
	{
		assert(nrows > 0 && ncols > 0 && nonzeros >=0);
		if (rowIndex_lock)
			rowIndex_lock_broken = true;

		// we can't preserve data if any row will be cut
		if (nrows < rows)
			return false;

		// STEP1: figure out the new storage dimension
		int new_mat_rows = nrows;
		int new_mat_cols = ncols;

		int new_colIndex_occupancy = 0;
		int storage_augmentation_foreachrow = 4;


		if (nonzeros == 0) // case LeaveStoreAsIs
		{
			if (rows == new_mat_rows) // case LeaveStoreAsIs&row==
				return true;

			if (new_mat_rows > rows) // case LeaveStoreAsIs&row++
				new_colIndex_occupancy = colIndex_occupancy + (new_mat_rows - rows)*storage_augmentation_foreachrow;
		}
		else // case SetStorage
		{
			new_colIndex_occupancy = nonzeros;

			// if the new number of rows require more space than the one imposed by the user
			if (new_mat_rows > rows && new_colIndex_occupancy < colIndex_occupancy + (new_mat_rows - rows) * 1)
				return false;
		}


		// if the nonzeros size requested would led to data losses then stop and return FALSE
		if (new_colIndex_occupancy < rowIndex[rows] - 1) 
			return false;


		// STEP 2: find the space for the new storage and paste the arrays in their new location

		
		// if new the size exceeds the current storage size a reallocation is required
		if (new_colIndex_occupancy > colIndex_occupancy)
		{
			if (ALIGNMENT_REQUIRED)
			{
				double* new_values = static_cast<double*>(mkl_malloc(new_colIndex_occupancy*sizeof(double), array_alignment));
				int* new_colIndex = static_cast<int*>(mkl_malloc(new_colIndex_occupancy*sizeof(int), array_alignment));
				reallocation_occurred = true;
				copy(new_values, new_colIndex, false);
				if (new_values != values) mkl_free(values);
				if (new_colIndex != colIndex) mkl_free(colIndex);
				values = new_values;
				colIndex = new_colIndex;
			}
			else
			{
				values = static_cast<double*>(mkl_realloc(values, new_colIndex_occupancy * sizeof(double)));
				colIndex = static_cast<int*>(mkl_realloc(colIndex, new_colIndex_occupancy * sizeof(int)));
				reallocation_occurred = true;
			}
		}
		// if the new size is between the current storage size and the effective length of the arrays there is no need to reallocate
		else if (new_colIndex_occupancy < colIndex_occupancy)
		{
			values = static_cast<double*>(mkl_realloc(values, new_colIndex_occupancy * sizeof(double)));
			colIndex = static_cast<int*>(mkl_realloc(colIndex, new_colIndex_occupancy * sizeof(int)));
		}


		// STEP 3: expand the arrays. Update rowIndex
		// case row++
		if (new_mat_rows > rows)
		{
			if (new_mat_rows + 1 > rowIndex_occupancy)
			{
				if (ALIGNMENT_REQUIRED)
				{
					int* new_rowIndex = static_cast<int*>(mkl_malloc((new_mat_rows + 1)*sizeof(int), array_alignment));
					reallocation_occurred = true;
					for (int row_sel = 0; row_sel <= rows; row_sel++)
						new_rowIndex[row_sel] = rowIndex[row_sel];
					if (new_rowIndex != rowIndex) mkl_free(rowIndex);
					rowIndex = new_rowIndex;
				}
				else
				{
					rowIndex = static_cast<int*>(mkl_realloc(rowIndex, (new_mat_rows + 1) * sizeof(int)));
					reallocation_occurred = true;
				}

				rowIndex_occupancy = new_mat_rows + 1;
			}

			
			// the newly acquired space is equally distributed to the new rows
			double effective_augmentation_foreachrow = (static_cast<double>(new_colIndex_occupancy) - static_cast<double>(rowIndex[rows]) ) / static_cast<double>(new_mat_rows - rows);

			for (int row_sel = 1; row_sel <= new_mat_rows - rows; row_sel++)
			{
				rowIndex[rows+row_sel] = rowIndex[rows] + static_cast<int>(ceil(static_cast<double>(row_sel) *effective_augmentation_foreachrow));
			}

		}
		// case row==
		else if (rows == new_mat_rows)
			rowIndex[new_mat_rows] = new_colIndex_occupancy+1;


		

		// Update colInde. Set the new elements in colIndex as "not-initialized" i.e. "-1"
		for (int col_sel = rowIndex[rows]; col_sel < rowIndex[new_mat_rows]; col_sel++)
		{
			colIndex[col_sel] = -1;
		}


		colIndex_occupancy = new_colIndex_occupancy;
		rows = new_mat_rows;
		columns = new_mat_cols;
		rowIndex_lock_broken = true;
		colIndex_lock_broken = true;
		isCompressed = false;

		return true;


	}  // Resize



	void ChCSR3Matrix::Reset(int nrows, int ncols, int nonzeros)
	{
		assert(nrows > 0 && ncols > 0 && nonzeros >= 0);

		// if nonzeros are not specified then the current size is kept
		int nonzeros_old = GetColIndexLength();
		if (nonzeros == 0)
			nonzeros = nonzeros_old;

		// exception: if the current size is lower than the new number of rows then it must be updated
		nonzeros = std::max(nrows, nonzeros);

		if (nrows != rows || ncols != columns || nonzeros != nonzeros_old)
		{
			isCompressed = false;
			rowIndex_lock_broken = true;
			colIndex_lock_broken = true;
		}


		/* Size update */
		// after this stage the length of all the arrays must be at LEAST as long as needed;
		// all the memory-descripting variables are updated;
		if (nrows > rows)
		{
			mkl_free(rowIndex);
			rowIndex = static_cast<int*>(mkl_malloc((nrows + 1)*sizeof(int), array_alignment));
			reallocation_occurred = true;
			rowIndex_occupancy = nrows + 1;
		}


		if (nonzeros > colIndex_occupancy)
		{
			mkl_free(values);
			mkl_free(colIndex);
			values = static_cast<double*>(mkl_malloc(nonzeros*sizeof(double), array_alignment));
			colIndex = static_cast<int*>(mkl_malloc(nonzeros*sizeof(int), array_alignment));
			reallocation_occurred = true;
			colIndex_occupancy = nonzeros;
		}


		/* Values initialization */
		// at the end of this stage colIndex will be initialized with '-1' or kept unchanged, depending on sparsity lock status;
		// values are left as they are or zeroed, depending on sparsity lock status;
		// rowIndex is filled so that each row is ready to hold the same number of nonzeros or kept unchanged, depending on sparsity lock status;
		// rows and columns are updated;

		rows = nrows;
		columns = ncols;

		initialize(nonzeros);

		rowIndex_lock_broken = false;
		colIndex_lock_broken = false;

	} // Reset

	void ChCSR3Matrix::Compress()
	{
		int col_sel_new = 0;
		int row_sel = 0;

		for (int col_sel = 0; col_sel < rowIndex[rows]; col_sel++)
		{
			// if an element is not initialized it would simply skip its copy
			if (colIndex[col_sel]>-1)
			{
				colIndex[col_sel_new] = colIndex[col_sel];
				values[col_sel_new] = values[col_sel];
				col_sel_new++;
			}

			// check for "all-zeros" line; it adds a dummy 0 on the diagonal (or as near as possible to the diagonal for rectangular matrices)
			if (col_sel == rowIndex[row_sel])
			{
				if (colIndex[col_sel] == -1)
				{
					colIndex[col_sel_new] = std::min(row_sel, columns);
					values[col_sel_new] = 0;
					col_sel_new++;
				}
				rowIndex[row_sel] = col_sel_new - 1;
				row_sel++;
			}
		}

		rowIndex[row_sel] = col_sel_new;
		isCompressed = true;
		rowIndex_lock_broken = false;
		colIndex_lock_broken = false;
	}

	void ChCSR3Matrix::Prune(double pruning_threshold)
	{
		if (pruning_threshold == 0)
		{
			for (int col_sel = 0; col_sel < rowIndex[rows]; col_sel++)
			{
				if (values[col_sel] == 0)
					colIndex[col_sel] = -1;
			}
		}
		else
		{
			for (int col_sel = 0; col_sel < rowIndex[rows]; col_sel++)
			{
				if (std::abs(values[col_sel]) < pruning_threshold)
					colIndex[col_sel] = -1;
			}
		}
		
	}

	void ChCSR3Matrix::Trim()
	{
		if (colIndex_occupancy > rowIndex[rows])
		{
			double* old_values = values;
			int* old_colIndex = colIndex;
			values = static_cast<double*>(mkl_realloc(values, rowIndex[rows] * sizeof(double)));
			colIndex = static_cast<int*>(mkl_realloc(colIndex, rowIndex[rows] * sizeof(int)));
			reallocation_occurred = true;
			colIndex_occupancy = rowIndex[rows];
			assert(old_values == values && old_colIndex);
		}

		if (rowIndex_occupancy>rows + 1)
		{
			int* old_rowIndex = rowIndex;
			rowIndex = static_cast<int*>(mkl_realloc(rowIndex, (rows + 1) * sizeof(int)));
			rowIndex_occupancy = rows + 1;
			reallocation_occurred = true;
			assert(old_rowIndex == rowIndex);
		}

		
	}

	void ChCSR3Matrix::PasteMatrix(ChMatrix<>* matra, int insrow, int inscol, bool overwrite, bool transp)
	{
		int maxrows = matra->GetRows();
		int maxcols = matra->GetColumns();
		int i, j;

		if (transp)
		{
			for (i = 0; i < maxcols; i++){
				for (j = 0; j < maxrows; j++){
					if ((*matra)(j, i) != 0) this->SetElement(insrow + i, inscol + j, (*matra)(j, i), overwrite);
				}
			}
		}
		else
		{
			for (i = 0; i < maxrows; i++){
				for (j = 0; j < maxcols; j++){
					if ((*matra)(i, j) != 0) this->SetElement(insrow + i, inscol + j, (*matra)(i, j), overwrite);
				}
			}
		}
	}

	void ChCSR3Matrix::PasteMatrixFloat(ChMatrix<float>* matra, int insrow, int inscol, bool overwrite, bool transp)
	{
		int maxrows = matra->GetRows();
		int maxcols = matra->GetColumns();
		int i, j;

		if (transp)
		{
			for (i = 0; i < maxcols; i++){
				for (j = 0; j < maxrows; j++){
					if ((*matra)(j, i) != 0) this->SetElement(insrow + i, inscol + j, (*matra)(j, i), overwrite);
				}
			}
		}
		else
		{
			for (i = 0; i < maxrows; i++){
				for (j = 0; j < maxcols; j++){
					if ((*matra)(i, j) != 0) this->SetElement(insrow + i, inscol + j, (*matra)(i, j), overwrite);
				}
			}
		}
	};

	void ChCSR3Matrix::PasteClippedMatrix(ChMatrix<>* matra, int cliprow, int clipcol, int nrows, int ncolumns, int insrow, int inscol, bool overwrite)
	{
		for (int i = 0; i < nrows; ++i)
			for (int j = 0; j < ncolumns; ++j)
				this->SetElement(insrow + i, inscol + j, matra->GetElement(i + cliprow, j + clipcol), overwrite);
	}

	void ChCSR3Matrix::ExportToDatFile(std::string filepath, int precision) const
	{
		std::ofstream a_file, ia_file, ja_file;
		a_file.open(filepath+"a.dat");
		ja_file.open(filepath+"ja.dat");
		ia_file.open(filepath+"ia.dat");
		a_file << std::scientific << std::setprecision(precision);
		ja_file << std::scientific << std::setprecision(precision);
		ia_file << std::scientific << std::setprecision(precision);

		double* a = GetValuesAddress();
		int* ja = GetColIndexAddress();
		int* ia = GetRowIndexAddress();

		for (int col_sel = 0; col_sel < ia[GetRows()]; col_sel++)
		{
			a_file << a[col_sel] << "\n";
			ja_file << ja[col_sel] << "\n";
		}

		for (int row_sel = 0; row_sel <= GetRows(); row_sel++)
		{
			ia_file << ia[row_sel] << "\n";
		}

		a_file.close();
		ja_file.close();
		ia_file.close();
	}



}  // end namespace chrono
