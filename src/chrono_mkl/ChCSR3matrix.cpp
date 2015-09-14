#include "ChCSR3matrix.h"
#include <algorithm>

namespace chrono {

	ChCSR3Matrix::ChCSR3Matrix(int insrow, int inscol, int nonzeros):
		max_shifts(std::numeric_limits<int>::max())
	{
		assert(insrow > 0 && inscol > 0 && nonzeros >= 0);
		mat_rows = insrow;
		mat_cols = inscol;
		reallocation_occurred = false;
		isCompressed = false;

		if (nonzeros == 0)
			nonzeros = static_cast<int>( static_cast<double>(mat_rows*mat_cols)*SPM_DEF_FULLNESS );

		colIndex_occupancy = std::max(mat_rows, static_cast<int>(nonzeros));
		rowIndex_occupancy = mat_rows + 1;
		
		if (TESTING_CSR3){
			mkl_peak_mem_usage(MKL_PEAK_MEM_ENABLE);
			mkl_peak_mem_CSR3 = 0;
			mkl_peak_mem_usage(MKL_PEAK_MEM_RESET);
		}

		values = static_cast<double*>(mkl_malloc(colIndex_occupancy*sizeof(double), array_alignment));
		colIndex = static_cast<int*>(mkl_malloc(colIndex_occupancy*sizeof(int), array_alignment));
		rowIndex = static_cast<int*>(mkl_malloc(rowIndex_occupancy*sizeof(int), array_alignment));

		initialize();

		if (TESTING_CSR3){
			mkl_peak_mem_CSR3 = max(mkl_peak_mem_CSR3, mkl_peak_mem_usage(MKL_PEAK_MEM_RESET));
		}
	}


	ChCSR3Matrix::ChCSR3Matrix()
	{
		ChCSR3Matrix(3, 3, 3);
	}

	ChCSR3Matrix::~ChCSR3Matrix()
	{
		mkl_free(values);
		mkl_free(colIndex);
		mkl_free(rowIndex);
	}


	void ChCSR3Matrix::SetElement(int insrow, int inscol, double insval, bool overwrite)
	{
		assert(insrow < mat_rows && inscol < mat_cols);
		assert(insrow >= 0 && inscol >= 0);

		int col_sel = rowIndex[insrow];
		while (1)
		{
			// case: element not found in the row OR another element with a higher col number is already been stored
			if (col_sel >= rowIndex[insrow + 1] || colIndex[col_sel] > inscol){
				insert(insrow, inscol, insval, col_sel);
				break;
			}

			// case: empty space
			if (colIndex[col_sel] == -1)
			{
				values[col_sel] = insval;
				colIndex[col_sel] = inscol;
				break;
			}

			if (colIndex[col_sel] == inscol)
			{
				if (overwrite)
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

		int col_shift = 1;
		int col_sel_empty = col_sel;

		// STEP 1: find an empty space in the array so part of the array can be shifted in order to give space to the new element
		// There are 3 While cycles; they all search for the NEAREST empty space (i.e. in which the colIndex array has a "-1"); no rearrangement is done at this stage.
		// 1st While: it scans both Backward and Forward, but only until at least ONE of the limits of the colIndex is reached;
		// 2nd While: it scans only Backward, but only if the 1st cycle did not find anything AND the beginning of colIndex is not reached yet;
		// 3rd While: it scans only Forward, but only if the 1st cycle did not find anything AND the end of colIndex is not reached yet;
		// These 3 cycles can be made one introducing a check on the limits of the array (in the IFs in the first While), but
		// this will introduce another 2 more condition check that have to be done at every iteration also if they'll be hit very rarely.
		while (col_shift < max_shifts && col_sel - col_shift>-1 && col_sel + col_shift < rowIndex[mat_rows])
		{
			if (colIndex[col_sel - col_shift] == -1) // backward check
			{
				// This part is very specific: it avoids to write to another row that has only one element that it's uninitialized;
				int row_sel = 0;
				for (row_sel = insrow; col_sel - col_shift < rowIndex[row_sel] && row_sel >= 0; row_sel--) {}
				if (rowIndex[row_sel] == col_sel - col_shift)
				{
					if (row_sel == 0)
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
				int row_sel = 0;
				for (row_sel = insrow; col_sel + col_shift > rowIndex[row_sel] && row_sel <= mat_rows; row_sel++) {}
				if (rowIndex[row_sel] == col_sel + col_shift)
				{
					if (row_sel == mat_rows)
						break;
					col_shift++;
					continue;
				}

				col_sel_empty = col_sel + col_shift;
				break;
			}
			col_shift++;
		}

		while (col_sel_empty == col_sel && col_shift < max_shifts && col_sel - col_shift>-1 ) // scan the last elements not already checked to the left
		{
			if (colIndex[col_sel - col_shift] == -1) // backward check
			{
				// This part is very specific: it avoids to write to another row that has only one element that it's uninitialized;
				int row_sel = 0;
				for (row_sel = insrow; col_sel - col_shift < rowIndex[row_sel] && row_sel >= 0; row_sel--) {}
				if (rowIndex[row_sel] == col_sel - col_shift)
				{
					if (row_sel == 0)
						break;
					col_shift++;
					continue;
				}

				col_sel_empty = col_sel - col_shift;
				break;
			}
			col_shift++;
		}

		while (col_sel_empty == col_sel && col_shift < max_shifts && col_sel + col_shift < rowIndex[mat_rows]) // scan the last elements not already checked to the right
		{
			if (colIndex[col_sel + col_shift] == -1) // forward check
			{
				// This part is very specific: it avoids to write to another row that has only one element that it's uninitialized;
				int row_sel = 0;
				for (row_sel = insrow; col_sel + col_shift > rowIndex[row_sel] && row_sel <= mat_rows; row_sel++) {}
				if (rowIndex[row_sel] == col_sel + col_shift)
				{
					if (row_sel == mat_rows)
						break;
					col_shift++;
					continue;
				}

				col_sel_empty = col_sel + col_shift;
				break;
			}
			col_shift++;
		}


		// If an uninitialized location is found "col_sel_empty" should point at it, so it would be different from col_sel.
		
		// STEP 2: shift the array to make space for the new element; eventually update "col_sel"
		// case 1: the uninitialized location is found forward;
		// case 2: the uninitialized location is found backward;
		// case 3: the location is not found in the neighborhood ("max_shifts") of the "col_sel" cell; a reallocation is needed.

		// case 1
		if (col_sel_empty > col_sel && col_sel + col_shift < rowIndex[mat_rows] && col_shift < max_shifts)
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
			if (colIndex_occupancy >= rowIndex[mat_rows]) // that happens when a Compress() or a Reset() is not followed by a Trim()
			{
				copy(values, colIndex, rowIndex, col_sel, 1);
			}
			else
			{ // effective reallocation

				int storage_augmentation = 4;
				colIndex_occupancy = colIndex_occupancy + storage_augmentation;

				if (TESTING_CSR3) mkl_peak_mem_usage(MKL_PEAK_MEM_RESET);

				if (ALIGNMENT_REQUIRED)
				{
					double* new_values = static_cast<double*>(mkl_malloc(colIndex_occupancy*sizeof(double), array_alignment));
					int* new_colIndex = static_cast<int*>(mkl_malloc(colIndex_occupancy*sizeof(int), array_alignment));
					reallocation_occurred = true;
					copy(new_values, new_colIndex, false, col_sel, storage_augmentation);
					if (new_values != values) mkl_free(values);
					if (new_colIndex != colIndex) mkl_free(colIndex);
					values = new_values;
					colIndex = new_colIndex;
				}
				else
				{
					values = static_cast<double*>(mkl_realloc(values, colIndex_occupancy * sizeof(double)));
					colIndex = static_cast<int*>(mkl_realloc(colIndex, colIndex_occupancy * sizeof(int)));
					reallocation_occurred = true;
					copy(values, colIndex, false, col_sel, storage_augmentation);
				}

				if (TESTING_CSR3) mkl_peak_mem_CSR3 = std::max(mkl_peak_mem_CSR3, mkl_peak_mem_usage(MKL_PEAK_MEM_RESET));
				
			} // end effective reallocation


			
		} // end case 3

		// In any case the new location should has been found; write the new values
		values[col_sel] = insval;
		colIndex[col_sel] = inscol;
	}

	void ChCSR3Matrix::initialize()
	{
		// TODO: we're taking for granted that rowIndex should span all storage dimension. Is it always that case?
		// initialize arrays
		for (int row_sel = 0; row_sel <= mat_rows; row_sel++) // rowIndex is initialized with equally spaced indexes
		{
			rowIndex[row_sel] = static_cast<int>(round(static_cast<double>(row_sel) * (static_cast<double>(colIndex_occupancy)) / static_cast<double>(mat_rows)));
		}

		for (int col_sel = 0; col_sel < rowIndex[mat_rows]; col_sel++) // colIndex is initialized with -1; it means that the cell has been stored but contains an uninitialized value
		{
			colIndex[col_sel] = -1;
		}
	}


	void ChCSR3Matrix::copy(double* values_temp, int* colIndex_temp, bool to_internal_arrays, int col_sel, int shifts)
	{
		double* values_destination, *values_source;
		int* colIndex_destination, *colIndex_source;

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
		

	// this first loop copies the "values" and "colIndex" arrays; when the loop arrives at the location
	// where the new space should be inserted it jumps few (precisely "shifts") elements forward so there will be a "shifts" amount
	// of space not initialized; the following elements will be located "shifts" positions forward;
		int col_sel_temp;
		int temp_shifts = 0;

		// it avoids the copy of the arrays on themselves for the first elements before "col_sel"
		if (values == values_temp && colIndex == colIndex_temp)
			col_sel_temp = col_sel;
		else
			col_sel_temp = 0;

		while( col_sel_temp < rowIndex[mat_rows] )
		{
			if (col_sel_temp == col_sel)
				temp_shifts = shifts;
			values_destination[col_sel_temp + temp_shifts] = values_source[col_sel_temp];
			colIndex_destination[col_sel_temp + temp_shifts] = colIndex_source[col_sel_temp];
			col_sel_temp++;
		}

		// this loop initializes the memory locations not initialized (could start from col_sel+1)
		for (col_sel_temp = col_sel; col_sel_temp < col_sel + shifts; col_sel_temp++)
		{
			values_destination[col_sel_temp] = 0;
			colIndex_destination[col_sel_temp] = -1;
		}

		// update of rowIndex
		int row_sel;
		for (row_sel = 0; rowIndex[row_sel] < col_sel; row_sel++){} // it points to the next row after col_sel
		for (;row_sel<=mat_rows;row_sel++)
		{
			rowIndex[row_sel] += shifts;
		}

	}

	bool ChCSR3Matrix::CheckArraysAlignment(int alignment)
	{
		if (alignment == 0)
			alignment = array_alignment;
		double integ_part_dummy = 0;
		double dec_part_a = modf(static_cast<double>(reinterpret_cast<uintptr_t>(values)) / alignment, &integ_part_dummy);
		double dec_part_ia = modf(static_cast<double>(reinterpret_cast<uintptr_t>(rowIndex)) / alignment, &integ_part_dummy);
		double dec_part_ja = modf(static_cast<double>(reinterpret_cast<uintptr_t>(colIndex)) / alignment, &integ_part_dummy);

		return (dec_part_a == 0 && dec_part_ia == 0 && dec_part_ja == 0) ? true : false;
	}

	void ChCSR3Matrix::GetMemoryInfo()
	{
		if (TESTING_CSR3)
			printf("\nPeak memory in CSR3 class (bytes): %lld", mkl_peak_mem_CSR3);
		printf("\nMemory allocated: %.2f MB", static_cast<double>( (2*colIndex_occupancy*sizeof(double) + rowIndex_occupancy* sizeof(int)) )/1000000  );
	}

	// Verify Matrix; output:
	//  3 - warning message: in the row there are no initialized elements
	//  0 - all good!
	// -1 - error message: rowIndex is not strictly ascending
	// -2 - error message: colIndex has not ascending indexes within the rows
	int ChCSR3Matrix::VerifyMatrix()
	{
		for (int row_sel = 0; row_sel < mat_rows; row_sel++)
		{
			// Check ascending order of rowIndex
			if (rowIndex[row_sel] >= rowIndex[row_sel + 1]) 
				return -1;

			bool valid_row = false;
			for (int col_sel = rowIndex[row_sel]; col_sel < rowIndex[row_sel + 1]-1; col_sel++)
			{
				// Checking if there is at least one allocated element for each row (it's not properly an error)
				if (colIndex[col_sel] != -1)
					valid_row = true;
				// Check ascending order of colIndex in each row
				if (colIndex[col_sel] != -1 && colIndex[col_sel + 1] != -1 && colIndex[col_sel] >= colIndex[col_sel + 1]) 
					return -2;
			}
			if (colIndex[rowIndex[row_sel + 1]-1] != -1)
				valid_row = true;
			if (!valid_row)
				return 3;
		}
		return 0;
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
		for (row_sel = 0; row_sel<=mat_rows; row_sel++)
			ia_file >> rowIndex[row_sel];
		row_sel--;

		Reset(mat_rows, mat_cols);
		
		ia_file.seekg(0);

		row_sel = -1;
		for (row_sel = 0; row_sel <= mat_rows; row_sel++)
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
		assert(row < mat_rows && col < mat_cols);
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
		assert(row < mat_rows && col < mat_cols);
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


	/* Resize HOW-TO. If asked to:
	*  LeaveStoreAsIs: nonzeros==0    the algorithm doesn't change the occupancy of the "values" and "colIndex" arrays
	*        if rows++ (row_increment) a certain amount of space is allocated for each new row
	*        if rows== nothing will happen
	*  SetStorage: nonzeros>0    the new dimension is prescribed.
	*        if rows++ it's verified that the prescribed dimension fits the row increment (there must be at least one element for each new row)
	*        if rows== ONLY the last row is expanded and filled with uninitialized spaces
	*/

	bool ChCSR3Matrix::Resize(int nrows, int ncols, int nonzeros)
	{
		assert(nrows > 0 && ncols > 0 && nonzeros >=0);

		// we can't preserve data if any row will be cut
		if (nrows < mat_rows)
			return false;

		// STEP1: figure out the new storage dimension
		int new_mat_rows = nrows;
		int new_mat_cols = ncols;

		int new_colIndex_occupancy = 0;
		int storage_augmentation_foreachrow = 4;

		if (nonzeros == 0) // case LeaveStoreAsIs
		{
			if (mat_rows == new_mat_rows) // case LeaveStoreAsIs&row==
				return true;

			if (new_mat_rows > mat_rows) // case LeaveStoreAsIs&row++
				new_colIndex_occupancy = colIndex_occupancy + (new_mat_rows - mat_rows)*storage_augmentation_foreachrow; // decision about new dimension
		}
		else // case SetStorage
		{
			new_colIndex_occupancy = nonzeros;

			if (new_mat_rows > mat_rows && new_colIndex_occupancy < colIndex_occupancy + (new_mat_rows - mat_rows) * 1)
				return false;
		}


		// if the size requested would led to data losses then stop and return FALSE
		if (new_colIndex_occupancy < rowIndex[mat_rows] - 1) 
			return false;

		// STEP 2: find the space for the new storage and paste the arrays in their new location

		if (TESTING_CSR3) mkl_peak_mem_usage(MKL_PEAK_MEM_RESET);
		
		// if new the size exceeds the current storage size a reallocation is required
		if (new_colIndex_occupancy > colIndex_occupancy)
		{
			if (ALIGNMENT_REQUIRED)
			{
				double* new_values = static_cast<double*>(mkl_malloc(new_colIndex_occupancy*sizeof(double), array_alignment));
				int* new_colIndex = static_cast<int*>(mkl_malloc(new_colIndex_occupancy*sizeof(int), array_alignment));
				reallocation_occurred = true;
				copy(new_values, new_colIndex, false, 0, 0);
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
		if (new_mat_rows > mat_rows)
		{
			if (new_mat_rows + 1 > rowIndex_occupancy)
			{
				if (ALIGNMENT_REQUIRED)
				{
					int* new_rowIndex = static_cast<int*>(mkl_malloc((new_mat_rows + 1)*sizeof(int), array_alignment));
					reallocation_occurred = true;
					for (int row_sel = 0; row_sel <= mat_rows; row_sel++)
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
			double effective_augmentation_foreachrow = (static_cast<double>(new_colIndex_occupancy) - static_cast<double>(rowIndex[mat_rows]) ) / static_cast<double>(new_mat_rows - mat_rows);

			for (int row_sel = 1; row_sel <= new_mat_rows - mat_rows; row_sel++)
			{
				rowIndex[mat_rows+row_sel] = rowIndex[mat_rows] + round(static_cast<double>(row_sel) *effective_augmentation_foreachrow);
			}

		}
		// case row==
		else if (mat_rows == new_mat_rows)
			rowIndex[new_mat_rows] = new_colIndex_occupancy+1;


		if (TESTING_CSR3) mkl_peak_mem_CSR3 = std::max(mkl_peak_mem_CSR3, mkl_peak_mem_usage(MKL_PEAK_MEM_RESET));


		// Update colInde. Set the new elements in colIndex as "not-initialized" i.e. "-1"
		for (int col_sel = rowIndex[mat_rows]; col_sel < rowIndex[new_mat_rows]; col_sel++)
		{
			colIndex[col_sel] = -1;
		}


		colIndex_occupancy = new_colIndex_occupancy;
		mat_rows = new_mat_rows;
		mat_cols = new_mat_cols;
		
		return true;


	}  // Resize


	void ChCSR3Matrix::Reset(int nrows, int ncols, int nonzeros)
	{
		assert(nrows > 0 && ncols > 0 && nonzeros >= 0);

		if (TESTING_CSR3){
			mkl_peak_mem_usage(MKL_PEAK_MEM_RESET);
		}

		if (nonzeros == 0)
			nonzeros = GetColIndexLength();

		if (nrows > mat_rows)
		{
			mkl_free(rowIndex);
			rowIndex = static_cast<int*>(mkl_malloc((nrows + 1)*sizeof(int), array_alignment));
			reallocation_occurred = true;
			rowIndex_occupancy = nrows + 1;
		}


		if (nonzeros>rowIndex[mat_rows])
		{
			mkl_free(values);
			mkl_free(colIndex);
			values = static_cast<double*>(mkl_malloc(nonzeros*sizeof(double), array_alignment));
			colIndex = static_cast<int*>(mkl_malloc(nonzeros*sizeof(int), array_alignment));
			reallocation_occurred = true;
			colIndex_occupancy = nonzeros;
		}

		if (TESTING_CSR3){
			mkl_peak_mem_CSR3 = std::max(mkl_peak_mem_CSR3, mkl_peak_mem_usage(MKL_PEAK_MEM_RESET));
		}


		mat_rows = nrows;
		mat_cols = ncols;

		initialize();
	}


	void ChCSR3Matrix::Compress(bool trim_after_compressing)
	{
		int col_sel_new = 0;
		int row_sel = 0;

		for (int col_sel = 0; col_sel < rowIndex[mat_rows]; col_sel++)
		{
			if (colIndex[col_sel]>-1)
			{
				colIndex[col_sel_new] = colIndex[col_sel];
				values[col_sel_new] = values[col_sel];
				col_sel_new++;
			}

			// check for all-zeros line; it adds a dummy 0 on the diagonal (or as near as possible to the diagonal for rectangular matrices)
			if (col_sel == rowIndex[row_sel])
			{
				if (colIndex[col_sel] == -1)
				{
					colIndex[col_sel_new] = std::min(row_sel, mat_cols);
					values[col_sel_new] = 0;
					col_sel_new++;
				}
				rowIndex[row_sel] = col_sel_new - 1;
				row_sel++;
			}
		}

		rowIndex[row_sel] = col_sel_new;
		if (trim_after_compressing) Trim();
		isCompressed = true;
	}

	void ChCSR3Matrix::Prune(double pruning_threshold) //TODO: use the same technique as Compress also to avoid zero-element lines
	{
		for (int col_sel = 0; col_sel < rowIndex[mat_rows]; col_sel++)
		{
			if (std::abs(values[col_sel]) < pruning_threshold)
				colIndex[col_sel] == -1;
		}
	}

	void ChCSR3Matrix::Trim()
	{
		if (colIndex_occupancy > rowIndex[mat_rows])
		{
			double* old_values = values;
			int* old_colIndex = colIndex;
			values = static_cast<double*>(mkl_realloc(values, rowIndex[mat_rows] * sizeof(double)));
			colIndex = static_cast<int*>(mkl_realloc(colIndex, rowIndex[mat_rows] * sizeof(int)));
			reallocation_occurred = true;
			colIndex_occupancy = rowIndex[mat_rows];
			assert(old_values == values && old_colIndex);
		}

		if (rowIndex_occupancy>mat_rows + 1)
		{
			int* old_rowIndex = rowIndex;
			rowIndex = static_cast<int*>(mkl_realloc(rowIndex, (mat_rows + 1) * sizeof(int)));
			rowIndex_occupancy = mat_rows + 1;
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

	void ChCSR3Matrix::ExportToDatFile(std::string filepath, int precision)
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
