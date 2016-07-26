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

#include <mkl.h>
#include <algorithm>
#include <cmath>

#include "chrono_mkl/ChCSR3Matrix.h"

#define ALIGNMENT_REQUIRED true

namespace chrono {

ChCSR3Matrix::ChCSR3Matrix(int nrows, int ncols, int nonzeros)
    : ChSparseMatrix(nrows, ncols) {
    assert(nrows > 0 && ncols > 0 && nonzeros >= 0);

    if (nonzeros == 0) nonzeros = static_cast<int>(m_num_rows*(m_num_cols*SPM_DEF_FULLNESS));

    m_capacity = std::max(m_num_rows, nonzeros);

    values = static_cast<double*>(mkl_malloc(m_capacity * sizeof(double), array_alignment));
    colIndex = static_cast<int*>(mkl_malloc(m_capacity * sizeof(int), array_alignment));
    rowIndex = static_cast<int*>(mkl_malloc((m_num_rows + 1) * sizeof(int), array_alignment));

    initialize();
}

ChCSR3Matrix::ChCSR3Matrix(int nrows, int ncols, int* nonzeros_vector)
    : ChSparseMatrix(nrows, ncols) {
    assert(nrows > 0 && ncols > 0);

    m_capacity = 0;
    for (int row_sel = 0; row_sel < m_num_rows; row_sel++) {
        m_capacity += nonzeros_vector[row_sel];
    }

    values = static_cast<double*>(mkl_malloc(m_capacity * sizeof(double), array_alignment));
    colIndex = static_cast<int*>(mkl_malloc(m_capacity * sizeof(int), array_alignment));
    rowIndex = static_cast<int*>(mkl_malloc((m_num_rows + 1) * sizeof(int), array_alignment));

    initialize(nonzeros_vector);
}

ChCSR3Matrix::~ChCSR3Matrix() {
    mkl_free(values);
    mkl_free(colIndex);
    mkl_free(rowIndex);
}

void ChCSR3Matrix::SetElement(int insrow, int inscol, double insval, bool overwrite) {
    assert(insrow < m_num_rows && inscol < m_num_cols);
    assert(insrow >= 0 && inscol >= 0);

    if ((m_type == SYMMETRIC_POSDEF || m_type == SYMMETRIC_INDEF) && insrow < inscol)
        return;

    // WARNING: if you fill the matrix with too many zeros Pardiso will give unpredictable behavior,
    // at least up until MKL 11.3 update 3
    // if you want to avoid the insertion of zeros just turn off the sparsity pattern lock
    int col_sel = rowIndex[insrow];
    while (1) {
        // case: element not found in the row OR another element with a higher col number is already been stored
        if (col_sel >= rowIndex[insrow + 1] || colIndex[col_sel] > inscol) {
            if (insval != 0 || m_lock)  // avoid to insert zero elements
                insert(insrow, inscol, insval, col_sel);
            break;
        }

        // case: empty space
        if (colIndex[col_sel] == -1) {
            if (insval != 0 || m_lock)  // avoid to insert zero elements
            {
                values[col_sel] = insval;
                colIndex[col_sel] = inscol;
            }
            break;
        }

        // case: element already allocated
        if (colIndex[col_sel] == inscol) {
            if (overwrite)  // allows to write zeros
                values[col_sel] = insval;
            else
                values[col_sel] += insval;
            break;
        }
        col_sel++;
    }
}

/// This function arranges the space to let the new element be put in the arrays.
void ChCSR3Matrix::insert(int insrow, int inscol, double insval, int& col_sel) {
    m_lock_broken = true;
    int col_shift = 1;            // an offset from the current position that points to the empty location found
    int col_sel_empty = col_sel;  // the location in which the new element will be put (if a space will be found it
                                  // will be different from col_sel)

    /****************** STEP 1 ******************/

    // "only-one-uninitialized-cell" row check: it starts from the row following/preceding the one you are in;
    // this is because if you are inserting in your own row there is no interest to check for
    // "only-one-uninitialized-cell" row
    int row_sel_bw = insrow - 1;
    int row_sel_fw = insrow + 1;

    // STEP 1: find an empty space in the array so part of the array can be shifted in order to give space to the new
    // element
    // There are 3 While cycles; they all search for the NEAREST empty space (i.e. in which the colIndex array has a
    // "-1"); no rearrangement is done at this stage.
    // 1st While: it scans both Backward and Forward, but only until at least ONE of the limits of the colIndex is
    // reached;
    // 2nd While: it scans only Backward, but only if the 1st cycle did not find anything AND the beginning of colIndex
    // is not reached yet;
    // 3rd While: it scans only Forward, but only if the 1st cycle did not find anything AND the end of colIndex is not
    // reached yet;
    // These 3 cycles can be made one introducing a check on the limits of the array (in the IFs in the first While),
    // but
    // this will introduce another 2 more condition check that have to be done at every iteration also if they'll be hit
    // very rarely.
    while (col_shift < max_shifts && col_sel - col_shift > -1 && col_sel + col_shift < rowIndex[m_num_rows]) {
        // backward check
        if (colIndex[col_sel - col_shift] == -1 && !m_lock) {
            // This part is very specific: it avoids to write to another row that has only one element that it's
            // uninitialized;
            for (; rowIndex[row_sel_bw] > col_sel - col_shift && row_sel_bw >= 0; row_sel_bw--) {
            }
            if (rowIndex[row_sel_bw] == col_sel - col_shift) {
                if (row_sel_bw == 0)
                    break;
                col_shift++;
                continue;
            }

            col_sel_empty = col_sel - col_shift;
            break;
        }

        // forward check
        if (colIndex[col_sel + col_shift] == -1) {
            // This part is very specific: it avoids to write to another row that has only one element that it's
            // uninitialized;
            for (; rowIndex[row_sel_fw] < col_sel + col_shift && row_sel_fw <= m_num_rows; row_sel_fw++) {
            }
            if (rowIndex[row_sel_fw] == col_sel + col_shift) {
                if (row_sel_fw == m_num_rows)
                    break;
                col_shift++;
                continue;
            }

            col_sel_empty = col_sel + col_shift;
            break;
        }

        col_shift++;

    }  // end 1st While

    // 2nd While: scan the last elements not already checked to the left (backward)
    while (!m_lock && col_sel_empty == col_sel && col_shift < max_shifts && col_sel - col_shift > -1) {
        // backward check
        if (colIndex[col_sel - col_shift] == -1) {
            // This part is very specific: it avoids to write to another row that has only one element that it's
            // uninitialized;
            for (; rowIndex[row_sel_bw] > col_sel - col_shift && row_sel_bw >= 0; row_sel_bw--) {
            }
            if (rowIndex[row_sel_bw] == col_sel - col_shift) {
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
    while (col_sel_empty == col_sel && col_shift < max_shifts && col_sel + col_shift < rowIndex[m_num_rows]) {
        // forward check
        if (colIndex[col_sel + col_shift] == -1) {
            // This part is very specific: it avoids to write to another row that has only one element that it's
            // uninitialized;
            for (; rowIndex[row_sel_fw] < col_sel + col_shift && row_sel_fw <= m_num_rows; row_sel_fw++) {
            }
            if (rowIndex[row_sel_fw] == col_sel + col_shift) {
                if (row_sel_fw == m_num_rows)
                    break;
                col_shift++;
                continue;
            }

            col_sel_empty = col_sel + col_shift;
            break;
        }
        col_shift++;
    }

    // If an uninitialized location is found "col_sel_empty" should point at it, so it would be different from
    // "col_sel".

    /****************** STEP 2 ******************/

    // STEP 2: shift the array to make space for the new element; eventually update "col_sel"
    // case 1: the uninitialized location is found forward;
    // case 2: the uninitialized location is found backward;
    // case 3: the location is not found in the neighborhood ("max_shifts") of the "col_sel" cell;
    // the array must be shifted (and, if the case, reallocated).

    if (col_sel_empty > col_sel && col_sel + col_shift < rowIndex[m_num_rows] && col_shift < max_shifts) {
        // case 1
        for (int col_sel_temp = col_sel_empty; col_sel_temp > col_sel; col_sel_temp--) {
            values[col_sel_temp] = values[col_sel_temp - 1];
            colIndex[col_sel_temp] = colIndex[col_sel_temp - 1];
        }

        for (int row_sel = insrow + 1; rowIndex[row_sel] < col_sel_empty; row_sel++) {
            rowIndex[row_sel]++;
        }
    } else if (col_sel_empty < col_sel && col_sel - col_shift > -1 && col_shift < max_shifts) {
        // case 2
        assert(!m_lock);
        col_sel--;
        for (int col_sel_temp = col_sel_empty; col_sel_temp < col_sel; col_sel_temp++) {
            values[col_sel_temp] = values[col_sel_temp + 1];
            colIndex[col_sel_temp] = colIndex[col_sel_temp + 1];
        }

        for (int row_sel = insrow; row_sel > -1 && rowIndex[row_sel] > col_sel_empty; row_sel--) {
            rowIndex[row_sel]--;
        }
    } else {
        // case 3
        m_lock_broken = true;

        // If you are reading this comment it means that the arrays must be shifted and this is the most expensive operation.
        // If you think that many other elements must be inserted 
        // then you probably want to make room not for just the current element
        // but, in the meantime, also for some more...
        // The more elements you think they will be inserted, the higher 'array_shifting' should be.
        const int array_shifting = 4;

        if (m_capacity > GetColIndexLength()) {
            // the new element will be inserted in 'col_sel'
            // but first of all the arrays must shift in order to give room to the new element
            copy(values, colIndex, false, insrow, col_sel, std::min(array_shifting, m_capacity - GetColIndexLength()));
        } else {
            // Actual reallocation

            int new_capacity = ceil(1.75*m_capacity);
            int storage_augmentation = new_capacity - m_capacity;
            m_capacity = new_capacity;

            //int storage_augmentation = 4;
            //m_capacity = m_capacity + storage_augmentation;

            if (ALIGNMENT_REQUIRED) {
                double* new_values = static_cast<double*>(mkl_malloc(m_capacity * sizeof(double), array_alignment));
                int* new_colIndex = static_cast<int*>(mkl_malloc(m_capacity * sizeof(int), array_alignment));
                copy(new_values, new_colIndex, false, insrow, col_sel, std::min(array_shifting, storage_augmentation));
                if (new_values != values)
                    mkl_free(values);
                if (new_colIndex != colIndex)
                    mkl_free(colIndex);
                values = new_values;
                colIndex = new_colIndex;
            } else {
                values = static_cast<double*>(mkl_realloc(values, m_capacity * sizeof(double)));
                colIndex = static_cast<int*>(mkl_realloc(colIndex, m_capacity * sizeof(int)));
                copy(values, colIndex, false, insrow, col_sel, storage_augmentation);
            }

        }  // end effective reallocation
    }

    // In any case the new location should has been found; write the new values
    values[col_sel] = insval;
    colIndex[col_sel] = inscol;
}

/** Initialize the arrays giving, for each row, the exact space needed;
* \param[in] nonzeros_vector array of integer; length equal to row number;
*	\a nonzeros_vector[i] tells how many nonzeros there will be on row \a i
*/
void ChCSR3Matrix::initialize(int* nonzeros_vector) {
    // rowIndex is initialized based on nonzeros_vector specification
    rowIndex[0] = 0;
    for (int row_sel = 0; row_sel < m_num_rows; row_sel++)  // rowIndex is initialized with equally spaced indexes
    {
        rowIndex[row_sel + 1] = rowIndex[row_sel] + nonzeros_vector[row_sel];
    }

    initialize_ValuesColIndex();
}

void ChCSR3Matrix::initialize(int colIndex_length) {
    if (!m_lock || m_lock_broken) {
        if (colIndex_length == 0)
            colIndex_length = m_capacity;

        // rowIndex is initialized with equally spaced indexes
        for (int row_sel = 0; row_sel <= m_num_rows; row_sel++) {
            rowIndex[row_sel] =
                static_cast<int>(ceil(static_cast<double>(row_sel) * (static_cast<double>(colIndex_length) + 1.0) /
                                      (static_cast<double>(m_num_rows) + 1.0)));
        }

        isCompressed = false;
    }

    initialize_ValuesColIndex();
}

void ChCSR3Matrix::initialize_ValuesColIndex() {
    if (m_lock && !m_lock_broken) {
        for (int col_sel = 0; col_sel < GetColIndexLength(); col_sel++)
            values[col_sel] = 0;
    } else {
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

void ChCSR3Matrix::copy(double* values_temp,
                        int* colIndex_temp,
                        bool to_internal_arrays,
                        int insrow,
                        int col_sel,
                        int shifts) {
    double *values_destination, *values_source;
    int *colIndex_destination, *colIndex_source;

    // set the destination and source arrays depending on "to_internal_arrays"
    if (to_internal_arrays) {
        values_destination = values;
        colIndex_destination = colIndex;
        values_source = values_temp;
        colIndex_source = colIndex_temp;
    } else {
        values_destination = values_temp;
        colIndex_destination = colIndex_temp;
        values_source = values;
        colIndex_source = colIndex;
    }

    // Suppose this is a piece of colIndex and we should put a new element in position 7 and increase the size of 4
    // spaces in total
    // (1 space for the new element + 3 not initialized yet); || indicates new row, | new column
    //                                                |<-- down here the new element should be put (where now there's
    //                                                14)
    // The source array			||11|19|23|42|56||12|14|17|47||26||31|49||21|39|44||~|				<-- won't be touched (unless we are
    // performing a selfcopy)
    // The destination array	|| | | | | || | | | ||  ||  |  ||  |  |  ||  | | | | |		<-- 4 more allocated spaces to
    // allow expansion
    int remaining_shifts = shifts;
    int col_sel_destination;
    for (col_sel_destination = rowIndex[m_num_rows] + shifts - 1;
         col_sel_destination - remaining_shifts >= rowIndex[insrow + 1]; col_sel_destination--) {
        values_destination[col_sel_destination] = values_source[col_sel_destination - remaining_shifts];
        colIndex_destination[col_sel_destination] = colIndex_source[col_sel_destination - remaining_shifts];
    }
    // The destination array	|| | | | | || | | | ||  ||  |  ||  |26|31||49|21|39|44|~|

    for (remaining_shifts = shifts; remaining_shifts > 1; remaining_shifts--, col_sel_destination--) {
        colIndex_destination[col_sel_destination] = -1;
    }
    // The destination array	|| | | | | || | | | ||  ||-1|-1||-1|26|31||49|21|39|44|~|

    while (col_sel_destination - remaining_shifts >= col_sel) {
        values_destination[col_sel_destination] = values_source[col_sel_destination - remaining_shifts];
        colIndex_destination[col_sel_destination] = colIndex_source[col_sel_destination - remaining_shifts];
        col_sel_destination--;
    }
    // The destination array	|| | | | | || | |14|17||47||-1|-1||-1|26|31||49|21|39|44|~|

    if (values_destination != values_source || colIndex_destination != colIndex_source) {
        while (col_sel_destination >= 0) {
            values_destination[col_sel_destination] = values_source[col_sel_destination];
            colIndex_destination[col_sel_destination] = colIndex_source[col_sel_destination];
            col_sel_destination--;
        }
    }
    // The destination array	||11|19|23|42|56||12|14|14|17|47|-1|-1|-1||26||31|49||21|39|44||~|

    // update of rowIndex
    if (shifts > 0) {
        int row_sel = insrow + 1;
        while (row_sel <= m_num_rows) {
            rowIndex[row_sel] += shifts;
            row_sel++;
        }
    }
    // The destination array	||11|19|23|42|56||12|14|14|17|47|-1|-1|-1||26||31|12||21|39|15||~|
}

void ChCSR3Matrix::GetNonZerosDistribution(int* nonzeros_vector) const {
    for (int row_sel = 0; row_sel < m_num_rows; row_sel++) {
        nonzeros_vector[row_sel] = rowIndex[row_sel + 1] - rowIndex[row_sel];
    }
}

bool ChCSR3Matrix::CheckArraysAlignment(int alignment) const {
    if (alignment == 0)
        alignment = array_alignment;
    double integ_part_dummy = 0;
    double dec_part_a = modf(static_cast<double>(reinterpret_cast<uintptr_t>(values)) / alignment, &integ_part_dummy);
    double dec_part_ia =
        modf(static_cast<double>(reinterpret_cast<uintptr_t>(rowIndex)) / alignment, &integ_part_dummy);
    double dec_part_ja =
        modf(static_cast<double>(reinterpret_cast<uintptr_t>(colIndex)) / alignment, &integ_part_dummy);

    return (dec_part_a == 0 && dec_part_ia == 0 && dec_part_ja == 0) ? true : false;
}

void ChCSR3Matrix::GetMemoryInfo() const {
    size_t sizeMB = (m_capacity + m_num_rows + 1) * sizeof(int) + m_capacity * sizeof(double);
    printf("\nMemory allocated: %.2f MB", sizeMB / 1000000.0);
}

// Verify Matrix; output:
//  3 - warning message: in the row there are no initialized elements
//  1 - warning message: the matrix is not compressed
//  0 - all good!
// -1 - error message: rowIndex is not strictly ascending
// -2 - error message: there's a row that has some an uninitialized element NOT at the end of its space in colIndex
// -4 - error message: colIndex has not ascending indexes within the rows

int ChCSR3Matrix::VerifyMatrix() const {
    bool uninitialized_elements_found = false;
    for (int row_sel = 0; row_sel < m_num_rows; row_sel++) {
        // Check ascending order of rowIndex
        if (rowIndex[row_sel] >= rowIndex[row_sel + 1])
            return -1;

        bool initialized_elements_found = false;

        int col_sel = rowIndex[row_sel + 1];
        while (col_sel > rowIndex[row_sel]) {
            col_sel--;
            if (colIndex[col_sel] == -1) {
                uninitialized_elements_found = true;
                if (initialized_elements_found)
                    return -2;
            } else {
                initialized_elements_found = true;

                if (col_sel > rowIndex[row_sel] && colIndex[col_sel] <= colIndex[col_sel - 1])
                    return -4;
            }
        }
        if (!initialized_elements_found)
            return 3;
    }
    return (uninitialized_elements_found) ? 1 : 0;
}

int ChCSR3Matrix::VerifyMatrixByMKL() const {
    sparse_struct mat_sparse;
    mat_sparse.n = m_num_rows;
    mat_sparse.csr_ia = rowIndex;
    mat_sparse.csr_ja = colIndex;
    mat_sparse.indexing = MKL_ZERO_BASED;
    mat_sparse.matrix_structure = MKL_GENERAL_STRUCTURE;
    mat_sparse.matrix_format = MKL_CSR;
    mat_sparse.message_level = MKL_PRINT;
    mat_sparse.print_style = MKL_C_STYLE;

    sparse_matrix_checker_init(&mat_sparse);

    return sparse_matrix_checker(&mat_sparse);
}

void ChCSR3Matrix::ImportFromDatFile(std::string path) {
    std::ifstream a_file, ia_file, ja_file;
    a_file.open(path + "a.dat");
    ja_file.open(path + "ja.dat");
    ia_file.open(path + "ia.dat");

    if (!a_file.is_open())
        assert(0);

    int row_sel = -1;
    for (row_sel = 0; row_sel <= m_num_rows; row_sel++)
        ia_file >> rowIndex[row_sel];
    row_sel--;

    Reset(m_num_rows, m_num_cols);

    ia_file.seekg(0);

    row_sel = -1;
    for (row_sel = 0; row_sel <= m_num_rows; row_sel++)
        ia_file >> rowIndex[row_sel];
    row_sel--;

    int col_sel = -1;
    for (col_sel = 0; col_sel < rowIndex[row_sel]; col_sel++) {
        a_file >> values[col_sel];
        ja_file >> colIndex[col_sel];
    }

    if (col_sel != rowIndex[row_sel])
        assert(0);

    a_file.close();
    ja_file.close();
    ia_file.close();
}

double ChCSR3Matrix::GetElement(int row, int col) {
    assert(row < m_num_rows && col < m_num_cols);
    assert(row >= 0 && col >= 0);
    for (int col_sel = rowIndex[row]; col_sel < rowIndex[row + 1]; col_sel++) {
        if (colIndex[col_sel] == col) {
            return values[col_sel];
        }
    }
    return 0;
}

double& ChCSR3Matrix::Element(int row, int col) {
    assert(row < m_num_rows && col < m_num_cols);
    assert(row >= 0 && col >= 0);

    // It scans the array SINCE it finds the place in which the element should be;
    // case 1a: the beginning of the next row is reached: a rearrangement is needed
    // case 1b: an already-stored element is found with a bigger column index: a rearrangement is needed
    // case 2: an empty location is found: no arrangement needed. (colIndex from the beginning of the row until col_sel
    // should be non-"-1" i.e. should have stored element)
    // case 3: an element with exactly the same column index is found
    // IN ANY CASE, exiting the WHILE cycle, "col_sel" should point to the place where the new element should be!

    int col_sel = rowIndex[row];
    while (1) {
        if (col_sel >= rowIndex[row + 1] || colIndex[col_sel] > col) {  // case 1a and 1b
            insert(row, col, 0.0, col_sel);
            // TODO: if Element is called as a constant i.e. as a rvalue you could not-write the value and return 0
            // without storing any extra space
            break;
        }

        if (colIndex[col_sel] == -1)  // case 2
        {
            values[col_sel] = 0.0;
            // TODO: if Element is called as a constant i.e. as a rvalue you could not-write the value and return 0
            // without storing any extra space
            colIndex[col_sel] = col;
            break;
        }

        if (colIndex[col_sel] == col) {
            break;
        }
        col_sel++;
    }

    return values[col_sel];
}

void ChCSR3Matrix::Reset(int nrows, int ncols, int nonzeros) {
    assert(nrows > 0 && ncols > 0 && nonzeros >= 0);

    // if nonzeros are not specified then the current size is kept
    int nonzeros_old = GetColIndexCapacity();
    if (nonzeros == 0)
        nonzeros = nonzeros_old;

    // exception: if the current size is lower than the new number of rows then it must be updated
    nonzeros = std::max(nrows, nonzeros);

    if (nrows != m_num_rows || ncols != m_num_cols || nonzeros != nonzeros_old) {
        isCompressed = false;
        m_lock_broken = true;
    }

    /* Size update */
    // after this stage the length of all the arrays must be at LEAST as long as needed;
    // all the memory-descripting variables are updated;
    if (nrows > m_num_rows) {
        mkl_free(rowIndex);
        rowIndex = static_cast<int*>(mkl_malloc((nrows + 1) * sizeof(int), array_alignment));
    }

    if (nonzeros > m_capacity) {
        mkl_free(values);
        mkl_free(colIndex);
        values = static_cast<double*>(mkl_malloc(nonzeros * sizeof(double), array_alignment));
        colIndex = static_cast<int*>(mkl_malloc(nonzeros * sizeof(int), array_alignment));
        m_capacity = nonzeros;
    }

    /* Values initialization */
    // at the end of this stage colIndex will be initialized with '-1' or kept unchanged, depending on sparsity lock
    // status;
    // values are left as they are or zeroed, depending on sparsity lock status;
    // rowIndex is filled so that each row is ready to hold the same number of nonzeros or kept unchanged, depending on
    // sparsity lock status;
    // rows and columns are updated;

    m_num_rows = nrows;
    m_num_cols = ncols;

    initialize(nonzeros);

    m_lock_broken = false;
}

bool ChCSR3Matrix::Compress() {
    // Do nothing if the sparsity pattern is locked and the locks are not broken.
    if (m_lock && !m_lock_broken)
        return false;

    // Compress the matrix
    int col_sel_new = 0;
    int row_sel = 0;

    for (int col_sel = 0; col_sel < rowIndex[m_num_rows]; col_sel++) {
        // if an element is not initialized it would simply skip its copy
        if (colIndex[col_sel] > -1) {
            colIndex[col_sel_new] = colIndex[col_sel];
            values[col_sel_new] = values[col_sel];
            col_sel_new++;
        }

        // check for "all-zeros" line; it adds a dummy 0 on the diagonal (or as near as possible to the diagonal for
        // rectangular matrices)
        if (col_sel == rowIndex[row_sel]) {
            if (colIndex[col_sel] == -1) {
                colIndex[col_sel_new] = std::min(row_sel, m_num_cols);
                values[col_sel_new] = 0;
                col_sel_new++;
            }
            rowIndex[row_sel] = col_sel_new - 1;
            row_sel++;
        }
    }

    rowIndex[row_sel] = col_sel_new;
    isCompressed = true;
    m_lock_broken = false;

    return true;
}

void ChCSR3Matrix::Prune(double pruning_threshold) {
    if (pruning_threshold == 0) {
        for (int col_sel = 0; col_sel < rowIndex[m_num_rows]; col_sel++) {
            if (values[col_sel] == 0)
                colIndex[col_sel] = -1;
        }
    } else {
        for (int col_sel = 0; col_sel < rowIndex[m_num_rows]; col_sel++) {
            if (std::abs(values[col_sel]) < pruning_threshold)
                colIndex[col_sel] = -1;
        }
    }
}

void ChCSR3Matrix::Trim() {
    if (m_capacity > rowIndex[m_num_rows]) {
        double* old_values = values;
        int* old_colIndex = colIndex;
        values = static_cast<double*>(mkl_realloc(values, rowIndex[m_num_rows] * sizeof(double)));
        colIndex = static_cast<int*>(mkl_realloc(colIndex, rowIndex[m_num_rows] * sizeof(int)));
        m_capacity = rowIndex[m_num_rows];
        assert(old_values == values && old_colIndex);
    }
}

void ChCSR3Matrix::ExportToDatFile(std::string filepath, int precision) const {
    std::ofstream a_file, ia_file, ja_file;
    a_file.open(filepath + "a.dat");
    ja_file.open(filepath + "ja.dat");
    ia_file.open(filepath + "ia.dat");
    a_file << std::scientific << std::setprecision(precision);
    ja_file << std::scientific << std::setprecision(precision);
    ia_file << std::scientific << std::setprecision(precision);

    for (int col_sel = 0; col_sel < rowIndex[m_num_rows]; col_sel++) {
        a_file << values[col_sel] << "\n";
        ja_file << colIndex[col_sel] << "\n";
    }

    for (int row_sel = 0; row_sel <= m_num_rows; row_sel++) {
        ia_file << rowIndex[row_sel] << "\n";
    }

    a_file.close();
    ja_file.close();
    ia_file.close();
}

}  // end namespace chrono
