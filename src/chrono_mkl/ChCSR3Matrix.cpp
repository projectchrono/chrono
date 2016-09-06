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

#include <algorithm>
#include "chrono_mkl/ChCSR3Matrix.h"

#define ALIGNMENT_REQUIRED true

namespace chrono {

ChCSR3Matrix::ChCSR3Matrix(int nrows, int ncols, int nonzeros, bool row_major_format_on)
    : ChSparseMatrix(nrows, ncols), row_major_format(row_major_format_on) {
    assert(nrows > 0 && ncols > 0 && nonzeros >= 0);

    leading_dimension = row_major_format ? m_num_rows : m_num_cols;

    if (nonzeros == 0) nonzeros = static_cast<int>(m_num_rows*(m_num_cols*SPM_DEF_FULLNESS));

    size_t new_capacity = std::max(leading_dimension, nonzeros);

    values_vect.resize(new_capacity);
    trailIndex_vect.resize(new_capacity);
    leadIndex_vect.resize(leading_dimension + 1);

    initialize();
}

ChCSR3Matrix::ChCSR3Matrix(int nrows, int ncols, int* nonzeros_vector, bool row_major_format_on)
    : ChSparseMatrix(nrows, ncols), row_major_format(row_major_format_on) {
    assert(nrows > 0 && ncols > 0);

    leading_dimension = row_major_format ? m_num_rows : m_num_cols;

    int new_capacity = 0;
    for (int ld_sel = 0; ld_sel < leading_dimension; ld_sel++) {
        new_capacity += nonzeros_vector[ld_sel];
    }

    values_vect.resize(new_capacity);
    trailIndex_vect.resize(new_capacity);
    leadIndex_vect.resize(leading_dimension + 1);

    initialize(nonzeros_vector);
}


void ChCSR3Matrix::SetElement(int insrow, int inscol, double insval, bool overwrite) {
    assert(insrow < m_num_rows && inscol < m_num_cols);
    assert(insrow >= 0 && inscol >= 0);

    if ((m_type == SYMMETRIC_POSDEF || m_type == SYMMETRIC_INDEF) && insrow < inscol)
        return;

    int insleaddim = row_major_format ? insrow : inscol;
    int instraildim = row_major_format ? inscol : insrow;

    // WARNING: you MUST check if insval!=0 because of known issues of current release of Pardiso (11.2 Update 2);
    // if the matrix is filled with too many zeros it gives unpredictable behaviour
    // if you need to insert a 0 use Element() instead
    int traildim_sel = leadIndex_vect[insleaddim];
    while (1) {
        // case: element not found in the row OR another element with a higher col number is already been stored
        if (traildim_sel >= leadIndex_vect[insleaddim + 1] || trailIndex_vect[traildim_sel] > instraildim) {
            if (insval != 0 || m_lock)  // avoid to insert zero elements
                insert(insleaddim, instraildim, insval, traildim_sel);
            break;
        }

        // case: empty space
        if (trailIndex_vect[traildim_sel] == -1) {
            if (insval != 0 || m_lock)  // avoid to insert zero elements
            {
                values_vect[traildim_sel] = insval;
                trailIndex_vect[traildim_sel] = instraildim;
            }
            break;
        }

        // case: element already allocated
        if (trailIndex_vect[traildim_sel] == instraildim) {
            if (overwrite)  // allows to write zeros
                values_vect[traildim_sel] = insval;
            else
                values_vect[traildim_sel] += insval;
            break;
        }
        traildim_sel++;
    }
}

/// This function arranges the space to let the new element be put in the arrays.
void ChCSR3Matrix::insert(int insleaddim, int instraildim, double insval, int& traildim_sel) {
    m_lock_broken = true;
    int traildim_shift = 1;            // an offset from the current position that points to the empty location found
    int traildim_sel_empty = traildim_sel;  // the location in which the new element will be put (if a space will be found it
                                  // will be different from traildim_sel)

    /****************** STEP 1 ******************/

    // "only-one-uninitialized-cell" row check: it starts from the row following/preceding the one you are in;
    // this is because if you are inserting in your own row there is no interest to check for
    // "only-one-uninitialized-cell" row
    int leaddim_sel_bw = insleaddim - 1;
    int traildim_sel_fw = insleaddim + 1;

    // STEP 1: find an empty space in the array so part of the array can be shifted in order to give space to the new
    // element
    // There are 3 While cycles; they all search for the NEAREST empty space (i.e. in which the trailIndex_vect array has a
    // "-1"); no rearrangement is done at this stage.
    // 1st While: it scans both Backward and Forward, but only until at least ONE of the limits of the trailIndex_vect is
    // reached;
    // 2nd While: it scans only Backward, but only if the 1st cycle did not find anything AND the beginning of trailIndex_vect
    // is not reached yet;
    // 3rd While: it scans only Forward, but only if the 1st cycle did not find anything AND the end of trailIndex_vect is not
    // reached yet;
    // These 3 cycles can be made one introducing a check on the limits of the array (in the IFs in the first While),
    // but
    // this will introduce another 2 more condition check that have to be done at every iteration also if they'll be hit
    // very rarely.
    while (traildim_shift < max_shifts && traildim_sel - traildim_shift > -1 && traildim_sel + traildim_shift < leadIndex_vect[m_num_rows]) {
        // backward check
        if (trailIndex_vect[traildim_sel - traildim_shift] == -1 && !m_lock) {
            // This part is very specific: it avoids to write to another row that has only one element that it's
            // uninitialized;
            for (; leadIndex_vect[leaddim_sel_bw] > traildim_sel - traildim_shift && leaddim_sel_bw >= 0; leaddim_sel_bw--) {
            }
            if (leadIndex_vect[leaddim_sel_bw] == traildim_sel - traildim_shift) {
                if (leaddim_sel_bw == 0)
                    break;
                traildim_shift++;
                continue;
            }

            traildim_sel_empty = traildim_sel - traildim_shift;
            break;
        }

        // forward check
        if (trailIndex_vect[traildim_sel + traildim_shift] == -1) {
            // This part is very specific: it avoids to write to another row that has only one element that it's
            // uninitialized;
            for (; leadIndex_vect[traildim_sel_fw] < traildim_sel + traildim_shift && traildim_sel_fw <= m_num_rows; traildim_sel_fw++) {
            }
            if (leadIndex_vect[traildim_sel_fw] == traildim_sel + traildim_shift) {
                if (traildim_sel_fw == m_num_rows)
                    break;
                traildim_shift++;
                continue;
            }

            traildim_sel_empty = traildim_sel + traildim_shift;
            break;
        }

        traildim_shift++;

    }  // end 1st While

    // 2nd While: scan the last elements not already checked to the left (backward)
    while (!m_lock && traildim_sel_empty == traildim_sel && traildim_shift < max_shifts && traildim_sel - traildim_shift > -1) {
        // backward check
        if (trailIndex_vect[traildim_sel - traildim_shift] == -1) {
            // This part is very specific: it avoids to write to another row that has only one element that it's
            // uninitialized;
            for (; leadIndex_vect[leaddim_sel_bw] > traildim_sel - traildim_shift && leaddim_sel_bw >= 0; leaddim_sel_bw--) {
            }
            if (leadIndex_vect[leaddim_sel_bw] == traildim_sel - traildim_shift) {
                if (leaddim_sel_bw == 0)
                    break;
                traildim_shift++;
                continue;
            }

            traildim_sel_empty = traildim_sel - traildim_shift;
            break;
        }
        traildim_shift++;
    }

    // 3rd While: scan the last elements not already checked to the right (forward)
    while (traildim_sel_empty == traildim_sel && traildim_shift < max_shifts && traildim_sel + traildim_shift < leadIndex_vect[m_num_rows]) {
        // forward check
        if (trailIndex_vect[traildim_sel + traildim_shift] == -1) {
            // This part is very specific: it avoids to write to another row that has only one element that it's
            // uninitialized;
            for (; leadIndex_vect[traildim_sel_fw] < traildim_sel + traildim_shift && traildim_sel_fw <= m_num_rows; traildim_sel_fw++) {
            }
            if (leadIndex_vect[traildim_sel_fw] == traildim_sel + traildim_shift) {
                if (traildim_sel_fw == m_num_rows)
                    break;
                traildim_shift++;
                continue;
            }

            traildim_sel_empty = traildim_sel + traildim_shift;
            break;
        }
        traildim_shift++;
    }

    // If an uninitialized location is found "traildim_sel_empty" should point at it, so it would be different from
    // "traildim_sel".

    /****************** STEP 2 ******************/

    // STEP 2: shift the array to make space for the new element; eventually update "traildim_sel"
    // case 1: the uninitialized location is found forward;
    // case 2: the uninitialized location is found backward;
    // case 3: the location is not found in the neighborhood ("max_shifts") of the "traildim_sel" cell; a reallocation is
    // needed.

    if (traildim_sel_empty > traildim_sel && traildim_sel + traildim_shift < leadIndex_vect[m_num_rows] && traildim_shift < max_shifts) {
        // case 1
        for (int traildim_sel_temp = traildim_sel_empty; traildim_sel_temp > traildim_sel; traildim_sel_temp--) {
            values_vect[traildim_sel_temp] = values_vect[traildim_sel_temp - 1];
            trailIndex_vect[traildim_sel_temp] = trailIndex_vect[traildim_sel_temp - 1];
        }

        for (int leaddim_sel = insleaddim + 1; leadIndex_vect[leaddim_sel] < traildim_sel_empty; leaddim_sel++) {
            leadIndex_vect[leaddim_sel]++;
        }
    } else if (traildim_sel_empty < traildim_sel && traildim_sel - traildim_shift > -1 && traildim_shift < max_shifts) {
        // case 2
        assert(!m_lock);
        traildim_sel--;
        for (int traildim_sel_temp = traildim_sel_empty; traildim_sel_temp < traildim_sel; traildim_sel_temp++) {
            values_vect[traildim_sel_temp] = values_vect[traildim_sel_temp + 1];
            trailIndex_vect[traildim_sel_temp] = trailIndex_vect[traildim_sel_temp + 1];
        }

        for (int leaddim_sel = insleaddim; leaddim_sel > -1 && leadIndex_vect[leaddim_sel] > traildim_sel_empty; leaddim_sel--) {
            leadIndex_vect[leaddim_sel]--;
        }
    } else {
        // case 3
        m_lock_broken = true;

        if (trailIndex_vect.capacity() > GetLeadingIndexLength()) {
            copy(values_vect.data(), trailIndex_vect.data(), false, insleaddim, traildim_sel, 1);
        } else {
            // Actual reallocation
            std::vector<double, aligned_allocator<double, 64>> new_values_vect;
            std::vector<int, aligned_allocator<int, 64>> new_leaddim_vect_vect;

            int new_capacity = trailIndex_vect.capacity()*1.75;
            new_values_vect.resize(new_capacity);
            new_leaddim_vect_vect.resize(new_capacity);

            copy(new_values_vect.data(), new_leaddim_vect_vect.data(), false, insleaddim, traildim_sel, new_capacity-trailIndex_vect.capacity());
            
            values_vect = std::move(new_values_vect);
            trailIndex_vect = std::move(new_leaddim_vect_vect);

        }  // end effective reallocation
    }

    // In any case the new location should has been found; write the new values_vect
    values_vect[traildim_sel] = insval;
    trailIndex_vect[traildim_sel] = instraildim;
}

/** Initialize the arrays giving, for each row, the exact space needed;
* \param[in] nonzeros_vector array of integer; length equal to row number;
*	\a nonzeros_vector[i] tells how many nonzeros there will be on row \a i
*/
void ChCSR3Matrix::initialize(int* nonzeros_vector) {
    // leadIndex_vect is initialized based on nonzeros_vector specification
    leadIndex_vect[0] = 0;
    for (int leaddim_sel = 0; leaddim_sel < m_num_rows; leaddim_sel++)  // leadIndex_vect is initialized with equally spaced indexes
    {
        leadIndex_vect[leaddim_sel + 1] = leadIndex_vect[leaddim_sel] + nonzeros_vector[leaddim_sel];
    }

    initialize_ValuesColIndex();
}

void ChCSR3Matrix::initialize(int leaddim_vect_length) {
    if (!m_lock || m_lock_broken) {
        if (leaddim_vect_length == 0)
            leaddim_vect_length = trailIndex_vect.capacity();

        // leadIndex_vect is initialized with equally spaced indexes
        for (int leaddim_sel = 0; leaddim_sel <= m_num_rows; leaddim_sel++) {
            leadIndex_vect[leaddim_sel] = static_cast<int>(ceil(static_cast<double>(leaddim_sel) * (static_cast<double>(leaddim_vect_length) + 1.0) /
                                      (static_cast<double>(m_num_rows) + 1.0)));
        }

        isCompressed = false;
    }

    initialize_ValuesColIndex();
}

void ChCSR3Matrix::initialize_ValuesColIndex() {
    if (m_lock && !m_lock_broken)
        for (int traildim_sel = 0; traildim_sel < leadIndex_vect[m_num_rows]; traildim_sel++)
            values_vect[traildim_sel] = 0;

    else {
        // trailIndex_vect is initialized with -1; it means that the cell has been stored but contains an uninitialized value
        for (int traildim_sel = 0; traildim_sel < leadIndex_vect[m_num_rows]; traildim_sel++)
            trailIndex_vect[traildim_sel] = -1;

        isCompressed = false;
    }
}

/** Copies from/to \c values_vect and \c trailIndex_vect arrays to/from the specified arrays \c values_temp and \c leaddim_vect_temp.
* Meanwhile he is coping it can also shift the destination array in this way:
*	- every row > \c insrow is copied into the new array shifted by \c shifts location
*	- the \c insrow row is modified (if \shifts >0):
*		1. at \c traildim_sel a new space is created;
*		2. at the end of the row the remaining \c shifts-1 location are created.
*	- every row < \c insrow is copied as is
*/

void ChCSR3Matrix::copy(double* values_temp,
                        int* leaddim_vect_temp,
                        bool to_internal_arrays,
                        int insleaddim,
                        int traildim_sel,
                        int shifts) {
    double *values_destination, *values_source;
    int *leaddim_vect_destination, *leaddim_vect_source;

    // set the destination and source arrays depending on "to_internal_arrays"
    if (to_internal_arrays) {
        values_destination = values_vect.data();
        leaddim_vect_destination = trailIndex_vect.data();
        values_source = values_temp;
        leaddim_vect_source = leaddim_vect_temp;
    } else {
        values_destination = values_temp;
        leaddim_vect_destination = leaddim_vect_temp;
        values_source = values_vect.data();
        leaddim_vect_source = trailIndex_vect.data();
    }


    // Suppose this is a piece of trailIndex_vect and we should put a new element in position 7 and increase the size of 4
    // spaces in total
    // (1 space for the new element + 3 not initialized yet); || indicates new row, | new column
    //                                                |<-- down here the new element should be put (where now there's
    //                                                14)
    // The source array			||11|19|23|42|56||12|14|17|47||26||31|49||21|39|44||~|				<-- won't be touched (unless we are
    // performing a selfcopy)
    // The destination array	|| | | | | || | | | ||  ||  |  ||  |  |  ||  | | | | |		<-- 4 more allocated spaces to
    // allow expansion
    int remaining_shifts = shifts;
    int traildim_sel_destination;
    for (traildim_sel_destination = leadIndex_vect[m_num_rows] + shifts - 1;
         traildim_sel_destination - remaining_shifts >= leadIndex_vect[insleaddim + 1]; traildim_sel_destination--) {
        values_destination[traildim_sel_destination] = values_source[traildim_sel_destination - remaining_shifts];
        leaddim_vect_destination[traildim_sel_destination] = leaddim_vect_source[traildim_sel_destination - remaining_shifts];
    }
    // The destination array	|| | | | | || | | | ||  ||  |  ||  |26|31||49|21|39|44|~|

    for (remaining_shifts = shifts; remaining_shifts > 1; remaining_shifts--, traildim_sel_destination--) {
        leaddim_vect_destination[traildim_sel_destination] = -1;
    }
    // The destination array	|| | | | | || | | | ||  ||-1|-1||-1|26|31||49|21|39|44|~|

    while (traildim_sel_destination - remaining_shifts >= traildim_sel) {
        values_destination[traildim_sel_destination] = values_source[traildim_sel_destination - remaining_shifts];
        leaddim_vect_destination[traildim_sel_destination] = leaddim_vect_source[traildim_sel_destination - remaining_shifts];
        traildim_sel_destination--;
    }
    // The destination array	|| | | | | || | |14|17||47||-1|-1||-1|26|31||49|21|39|44|~|

    if (values_destination != values_source || leaddim_vect_destination != leaddim_vect_source) {
        while (traildim_sel_destination >= 0) {
            values_destination[traildim_sel_destination] = values_source[traildim_sel_destination];
            leaddim_vect_destination[traildim_sel_destination] = leaddim_vect_source[traildim_sel_destination];
            traildim_sel_destination--;
        }
    }
    // The destination array	||11|19|23|42|56||12|14|14|17|47|-1|-1|-1||26||31|49||21|39|44||~|

    // update of leadIndex_vect
    if (shifts > 0) {
        int leaddim_sel = insleaddim + 1;
        while (leaddim_sel <= leading_dimension) {
            leadIndex_vect[leaddim_sel] += shifts;
            leaddim_sel++;
        }
    }
    // The destination array	||11|19|23|42|56||12|14|14|17|47|-1|-1|-1||26||31|12||21|39|15||~|
}

void ChCSR3Matrix::GetNonZerosDistribution(int* nonzeros_vector) const {
    for (int leaddim_sel = 0; leaddim_sel < leading_dimension; leaddim_sel++) {
        nonzeros_vector[leaddim_sel] = leadIndex_vect[leaddim_sel + 1] - leadIndex_vect[leaddim_sel];
    }
}

bool ChCSR3Matrix::CheckArraysAlignment(int alignment) const {
    if (alignment == 0)
        alignment = array_alignment;
    double integ_part_dummy = 0;
    double dec_part_a = modf(static_cast<double>(reinterpret_cast<uintptr_t>(values_vect.data())) / alignment, &integ_part_dummy);
    double dec_part_ia =
        modf(static_cast<double>(reinterpret_cast<uintptr_t>(leadIndex_vect.data())) / alignment, &integ_part_dummy);
    double dec_part_ja =
        modf(static_cast<double>(reinterpret_cast<uintptr_t>(trailIndex_vect.data())) / alignment, &integ_part_dummy);

    return (dec_part_a == 0 && dec_part_ia == 0 && dec_part_ja == 0) ? true : false;
}

void ChCSR3Matrix::GetMemoryInfo() const {
    size_t sizeMB = (trailIndex_vect.capacity() + leading_dimension + 1) * sizeof(int) + trailIndex_vect.capacity() * sizeof(double);
    printf("\nMemory allocated: %.2f MB", sizeMB / 1000000.0);
}

// Verify Matrix; output:
//  3 - warning message: in the row there are no initialized elements
//  1 - warning message: the matrix is not compressed
//  0 - all good!
// -1 - error message: leadIndex_vect is not strictly ascending
// -2 - error message: there's a row that has some an uninitialized element NOT at the end of its space in trailIndex_vect
// -4 - error message: trailIndex_vect has not ascending indexes within the rows

int ChCSR3Matrix::VerifyMatrix() const {
    bool uninitialized_elements_found = false;
    for (int leaddim_sel = 0; leaddim_sel < leading_dimension; leaddim_sel++) {
        // Check ascending order of leadIndex_vect
        if (leadIndex_vect[leaddim_sel] >= leadIndex_vect[leaddim_sel + 1])
            return -1;

        bool initialized_elements_found = false;

        int traildim_sel = leadIndex_vect[leaddim_sel + 1];
        while (traildim_sel > leadIndex_vect[leaddim_sel]) {
            traildim_sel--;
            if (trailIndex_vect[traildim_sel] == -1) {
                uninitialized_elements_found = true;
                if (initialized_elements_found)
                    return -2;
            } else {
                initialized_elements_found = true;

                if (traildim_sel > leadIndex_vect[leaddim_sel] && trailIndex_vect[traildim_sel] <= trailIndex_vect[traildim_sel - 1])
                    return -4;
            }
        }
        if (!initialized_elements_found)
            return 3;
    }
    return (uninitialized_elements_found) ? 1 : 0;
}


void ChCSR3Matrix::ImportFromDatFile(std::string path) {
    std::ifstream a_file, ia_file, ja_file;
    a_file.open(path + "a.dat");
    ja_file.open(path + "ja.dat");
    ia_file.open(path + "ia.dat");

    if (!a_file.is_open())
        assert(0);

    int leaddim_sel;
    for (leaddim_sel = 0; leaddim_sel <= leading_dimension; leaddim_sel++)
        ia_file >> leadIndex_vect[leaddim_sel];

    Reset(m_num_rows, m_num_cols);

    ia_file.seekg(0);

    for (leaddim_sel = 0; leaddim_sel <= leading_dimension; leaddim_sel++)
        ia_file >> leadIndex_vect[leaddim_sel];
    leaddim_sel--;

    int traildim_sel;
    for (traildim_sel = 0; traildim_sel < leadIndex_vect[leaddim_sel]; traildim_sel++) {
        a_file >> values_vect[traildim_sel];
        ja_file >> trailIndex_vect[traildim_sel];
    }

    if (traildim_sel != leadIndex_vect[leaddim_sel])
        assert(0);

    a_file.close();
    ja_file.close();
    ia_file.close();
}

double ChCSR3Matrix::GetElement(int row, int col) {
    assert(row < m_num_rows && col < m_num_cols);
    assert(row >= 0 && col >= 0);

    int leaddimsel = row_major_format ? row : col;
    int traildim_sel = row_major_format ? col : row;

    for (int traildim_sel = leadIndex_vect[leaddimsel]; traildim_sel < leadIndex_vect[leaddimsel + 1]; traildim_sel++) {
        if (trailIndex_vect[traildim_sel] == traildim_sel) {
            return values_vect[traildim_sel];
        }
    }
    return 0;
}

double& ChCSR3Matrix::Element(int row, int col) {
    assert(row < m_num_rows && col < m_num_cols);
    assert(row >= 0 && col >= 0);

    int leaddim = row_major_format ? row : col;
    int traildim = row_major_format ? col : row;

    // It scans the array SINCE it finds the place in which the element should be;
    // case 1a: the beginning of the next row is reached: a rearrangement is needed
    // case 1b: an already-stored element is found with a bigger column index: a rearrangement is needed
    // case 2: an empty location is found: no arrangement needed. (trailIndex_vect from the beginning of the row until traildim_sel
    // should be non-"-1" i.e. should have stored element)
    // case 3: an element with exactly the same column index is found
    // IN ANY CASE, exiting the WHILE cycle, "traildim_sel" should point to the place where the new element should be!

    int traildim_sel = leadIndex_vect[leaddim];
    while (1) {
        if (traildim_sel >= leadIndex_vect[leaddim + 1] || trailIndex_vect[traildim_sel] > traildim) {  // case 1a and 1b
            insert(leaddim, traildim, 0.0, traildim_sel);
            // TODO: if Element is called as a constant i.e. as a rvalue you could not-write the value and return 0
            // without storing any extra space
            break;
        }

        if (trailIndex_vect[traildim_sel] == -1)  // case 2
        {
            values_vect[traildim_sel] = 0.0;
            // TODO: if Element is called as a constant i.e. as a rvalue you could not-write the value and return 0
            // without storing any extra space
            trailIndex_vect[traildim_sel] = traildim;
            break;
        }

        if (trailIndex_vect[traildim_sel] == traildim) {
            break;
        }
        traildim_sel++;
    }

    return values_vect[traildim_sel];
}


void ChCSR3Matrix::Reset(int nrows, int ncols, int nonzeros) {
    assert(nrows > 0 && ncols > 0 && nonzeros >= 0);

    // if nonzeros are not specified then the current size is kept
    int nonzeros_old = GetTrailingIndexCapacity();
    if (nonzeros == 0)
        nonzeros = nonzeros_old;

    int leaddim = row_major_format ? nrows : ncols;
    int traildim = row_major_format ? ncols : nrows;


    // exception: if the current size is lower than the new number of rows then it must be updated
    nonzeros = std::max(leaddim, nonzeros);

    if (leaddim != leading_dimension || traildim != trailing_dimension || nonzeros != nonzeros_old) {
        isCompressed = false;
        m_lock_broken = true;
    }

    /* Size update */
    // after this stage the length of all the arrays must be at LEAST as long as needed;
    // all the memory-descripting variables are updated;
    if (leaddim > leading_dimension) {
        leadIndex_vect.resize(leaddim +1);
    }

    if (nonzeros > trailIndex_vect.capacity()) {
        values_vect.resize(nonzeros);
        trailIndex_vect.resize(nonzeros);
    }

    /* Values initialization */
    // at the end of this stage trailIndex_vect will be initialized with '-1' or kept unchanged, depending on sparsity lock
    // status;
    // values_vect are left as they are or zeroed, depending on sparsity lock status;
    // leadIndex_vect is filled so that each row is ready to hold the same number of nonzeros or kept unchanged, depending on
    // sparsity lock status;
    // rows and columns are updated;

    leading_dimension = nrows;
    trailing_dimension = ncols;

    initialize(nonzeros);

    m_lock_broken = false;
}

bool ChCSR3Matrix::Compress() {
    // Do nothing if the sparsity pattern is locked and the locks are not broken.
    if (m_lock && !m_lock_broken)
        return false;

    // Compress the matrix
    int traildim_sel_new = 0;
    int leaddim_sel = 0;

    for (int traildim_sel = 0; traildim_sel < leadIndex_vect[m_num_rows]; traildim_sel++) {
        // if an element is not initialized it would simply skip its copy
        if (trailIndex_vect[traildim_sel] > -1) {
            trailIndex_vect[traildim_sel_new] = trailIndex_vect[traildim_sel];
            values_vect[traildim_sel_new] = values_vect[traildim_sel];
            traildim_sel_new++;
        }

        // check for "all-zeros" line; it adds a dummy 0 on the diagonal (or as near as possible to the diagonal for
        // rectangular matrices)
        if (traildim_sel == leadIndex_vect[leaddim_sel]) {
            if (trailIndex_vect[traildim_sel] == -1) {
                trailIndex_vect[traildim_sel_new] = std::min(leaddim_sel, m_num_cols);
                values_vect[traildim_sel_new] = 0;
                traildim_sel_new++;
            }
            leadIndex_vect[leaddim_sel] = traildim_sel_new - 1;
            leaddim_sel++;
        }
    }

    leadIndex_vect[leaddim_sel] = traildim_sel_new;
    isCompressed = true;
    m_lock_broken = false;

    return true;
}

void ChCSR3Matrix::Prune(double pruning_threshold) {
    if (pruning_threshold == 0) {
        for (int traildim_sel = 0; traildim_sel < leadIndex_vect[m_num_rows]; traildim_sel++) {
            if (values_vect[traildim_sel] == 0)
                trailIndex_vect[traildim_sel] = -1;
        }
    } else {
        for (int traildim_sel = 0; traildim_sel < leadIndex_vect[m_num_rows]; traildim_sel++) {
            if (std::abs(values_vect[traildim_sel]) < pruning_threshold)
                trailIndex_vect[traildim_sel] = -1;
        }
    }
}

void ChCSR3Matrix::Trim() {
    trailIndex_vect.shrink_to_fit();
    values_vect.shrink_to_fit();
    leadIndex_vect.shrink_to_fit();
}

void ChCSR3Matrix::ExportToDatFile(std::string filepath, int precision) const {
    std::ofstream a_file, ia_file, ja_file;
    a_file.open(filepath + "a.dat");
    ja_file.open(filepath + "ja.dat");
    ia_file.open(filepath + "ia.dat");
    a_file << std::scientific << std::setprecision(precision);
    ja_file << std::scientific << std::setprecision(precision);
    ia_file << std::scientific << std::setprecision(precision);

    for (int traildim_sel = 0; traildim_sel < leadIndex_vect[m_num_rows]; traildim_sel++) {
        a_file << values_vect[traildim_sel] << "\n";
        ja_file << trailIndex_vect[traildim_sel] << "\n";
    }

    for (int leaddim_sel = 0; leaddim_sel <= m_num_rows; leaddim_sel++) {
        ia_file << leadIndex_vect[leaddim_sel] << "\n";
    }

    a_file.close();
    ja_file.close();
    ia_file.close();
}

}  // end namespace chrono
