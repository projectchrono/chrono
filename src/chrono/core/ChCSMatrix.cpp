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

#include <algorithm>

#include "chrono/core/ChCSMatrix.h"
#include "chrono/core/ChMapMatrix.h"
#include "chrono/solver/ChSystemDescriptor.h"

namespace chrono {

ChCSMatrix::ChCSMatrix(int nrows, int ncols, bool row_major_format_on, int nonzeros)
    : ChSparseMatrix(nrows, ncols), row_major_format(row_major_format_on) {
    // link dimensions to rows and column depending on format
    leading_dimension = row_major_format ? &m_num_rows : &m_num_cols;
    trailing_dimension = row_major_format ? &m_num_cols : &m_num_rows;

    reset_arrays(*leading_dimension, *trailing_dimension, nonzeros);
}

void ChCSMatrix::SetElement(int row_sel, int col_sel, double insval, bool overwrite) {
    auto lead_sel = row_major_format ? row_sel : col_sel;
    auto trail_sel = row_major_format ? col_sel : row_sel;

    if (insval == 0 && !m_lock)
        return;

    int trail_i;
    for (trail_i = leadIndex[lead_sel]; trail_i < leadIndex[lead_sel + 1]; ++trail_i) {
        // the requested element DOES NOT exist yet, BUT
        // NO other elements with greater index have been stored yet, SO
        // we can just place the new element here
        if (!initialized_element[trail_i]) {
            initialized_element[trail_i] = true;
            trailIndex[trail_i] = trail_sel;
            values[trail_i] = insval;
            return;
        }

        // the requested element DOES NOT exist yet AND
        // another element with greater index has already been stored, SO
        // that element has to be pushed further!
        if (trailIndex[trail_i] > trail_sel) {
            // insertion needed
            break;
        }

        // the requested element already exists
        if (trailIndex[trail_i] == trail_sel) {
            (overwrite) ? values[trail_i] = insval : values[trail_i] += insval;
            return;
        }
    }

    // if the loop ends without break means 'row full' (the insertion will also move the other row, for sure!)

    // insertion needed
    insert(trail_i, lead_sel);
    initialized_element[trail_i] = true;
    trailIndex[trail_i] = trail_sel;
    values[trail_i] = insval;
}

double ChCSMatrix::GetElement(int row_sel, int col_sel) const {
    auto lead_sel = row_major_format ? row_sel : col_sel;
    auto trail_sel = row_major_format ? col_sel : row_sel;

    for (auto trail_i = leadIndex[lead_sel]; trail_i < leadIndex[lead_sel + 1]; ++trail_i) {
        if (trailIndex[trail_i] == trail_sel)
            return values[trail_i];
    }

    return 0.0;
}

double& ChCSMatrix::Element(int row_sel, int col_sel) {
    auto lead_sel = row_major_format ? row_sel : col_sel;
    auto trail_sel = row_major_format ? col_sel : row_sel;

    int trail_i;
    for (trail_i = leadIndex[lead_sel]; trail_i < leadIndex[lead_sel + 1]; ++trail_i) {
        // the requested element DOES NOT exist yet, BUT
        // NO other elements with greater index have been stored yet, SO
        // we can just place the new element here
        if (!initialized_element[trail_i]) {
            initialized_element[trail_i] = true;
            trailIndex[trail_i] = trail_sel;
            return values[trail_i];
        }

        // the requested element DOES NOT exist yet AND
        // another element with greater index has already been stored, SO
        // that element has to be pushed further!
        if (trailIndex[trail_i] > trail_sel) {
            // insertion needed
            break;
        }

        // the requested element already exists
        if (trailIndex[trail_i] == trail_sel) {
            return values[trail_i];
        }
    }

    // if the loop ends without break means 'row full' (the insertion will also move the other row, for sure!)

    // insertion needed
    insert(trail_i, lead_sel);
    initialized_element[trail_i] = true;
    trailIndex[trail_i] = trail_sel;
    return values[trail_i];
}

void ChCSMatrix::Reset(int nrows, int ncols, int nonzeros_hint) {
    auto lead_dim_new = row_major_format ? nrows : ncols;
    auto trail_dim_new = row_major_format ? ncols : nrows;

    if (nonzeros_hint == 0 && lead_dim_new == *leading_dimension && trail_dim_new == *trailing_dimension && m_lock) {
        std::fill(values.begin(), values.begin() + leadIndex[*leading_dimension] - 1, 0);
    } else {
        if (nonzeros_hint == 0)
            nonzeros_hint = GetTrailingIndexLength();
        reset_arrays(lead_dim_new, trail_dim_new, nonzeros_hint);  // breaks also the sparsity lock
    }

    m_num_rows = nrows;
    m_num_cols = ncols;
}

int* ChCSMatrix::GetCS_LeadingIndexArray() const {
    if (!isCompressed)
        const_cast<ChCSMatrix*>(this)->Compress();
    return const_cast<int*>(leadIndex.data());
}

int* ChCSMatrix::GetCS_TrailingIndexArray() const {
    if (!isCompressed)
        const_cast<ChCSMatrix*>(this)->Compress();
    return const_cast<int*>(trailIndex.data());
}

double* ChCSMatrix::GetCS_ValueArray() const {
    if (!isCompressed)
        const_cast<ChCSMatrix*>(this)->Compress();
    return const_cast<double*>(values.data());
}

bool ChCSMatrix::Compress() {
    if (isCompressed)
        return false;

    int trail_i_dest = 0;
    int trail_i = 0;
    for (auto lead_i = 0; lead_i < *leading_dimension; ++lead_i) {
        for (; trail_i < leadIndex[lead_i + 1]; ++trail_i) {
            if (initialized_element[trail_i]) {
                values[trail_i_dest] = values[trail_i];
                trailIndex[trail_i_dest] = trailIndex[trail_i];
                ++trail_i_dest;
            }
        }
        leadIndex[lead_i + 1] = trail_i_dest;
    }

    std::fill(initialized_element.begin(), initialized_element.begin() + leadIndex[*leading_dimension], true);
    isCompressed = true;
    m_lock_broken = false;
    return trail_i_dest != trail_i;
}

int ChCSMatrix::Inflate(int storage_augm, int lead_sel, int trail_sel) {
    assert(lead_sel >= 0 && lead_sel < m_num_rows && "Cannot inflate a row that does not exist");
    if (trail_sel == -1)
        trail_sel = leadIndex[lead_sel + 1];

    assert(trail_sel >= 0 && trail_sel <= leadIndex[*leading_dimension] &&
           "Cannot inflate the values and trail-dimension index array in the given position");

    auto new_size = trailIndex.size() + storage_augm;
    if (new_size >= trailIndex.capacity())  // the space required does NOT fit into the current array capacity
    {
        // create new arrays in which the new elements will be stored
        index_vector_t trailIndex_new;
        values_vector_t values_new;
        std::vector<bool> initialized_element_new;

        // resize to desired values //TODO: do not initialize elements
        trailIndex_new.resize(new_size);
        values_new.resize(new_size);
        initialized_element_new.assign(new_size, false);

        // copy the array values over new vectors
        copy_and_distribute(trailIndex, values, initialized_element, trailIndex_new, values_new,
                            initialized_element_new, trail_sel, lead_sel, storage_augm);

        // move
        values = std::move(values_new);
        trailIndex = std::move(trailIndex_new);
        initialized_element = std::move(initialized_element_new);
    } else {
        // resize to desired values
        trailIndex.resize(new_size);
        values.resize(new_size);
        initialized_element.resize(new_size, false);
        copy_and_distribute(trailIndex, values, initialized_element, trailIndex, values, initialized_element, trail_sel,
                            lead_sel, storage_augm);
    }

    return trail_sel;
}

void ChCSMatrix::Trim() {
    trailIndex.resize(leadIndex[*leading_dimension]);
    values.resize(leadIndex[*leading_dimension]);
    initialized_element.resize(leadIndex[*leading_dimension]);

    leadIndex.shrink_to_fit();
    trailIndex.shrink_to_fit();
    values.shrink_to_fit();
    initialized_element.shrink_to_fit();
}

void ChCSMatrix::Prune(double pruning_threshold) {
    int trail_i_dest = 0;
    for (auto lead_i = 0; lead_i < *leading_dimension; ++lead_i) {
        for (auto trail_i = leadIndex[lead_i]; trail_i < leadIndex[lead_i + 1]; ++trail_i) {
            if (initialized_element[trail_i] && std::abs(values[trail_i]) > pruning_threshold) {
                values[trail_i] = values[trail_i_dest];
                trailIndex[trail_i] = trailIndex[trail_i_dest];
                ++trail_i_dest;
            }
        }
        leadIndex[lead_i + 1] = trail_i_dest;
    }
    std::fill(initialized_element.begin(), initialized_element.begin() + leadIndex[*leading_dimension], true);
    m_lock_broken = false;
    isCompressed = true;
}

int ChCSMatrix::VerifyMatrix() const {
    bool uninitialized_elements_found = false;
    for (int lead_sel = 0; lead_sel < *leading_dimension; lead_sel++) {
        // Check ascending order of leadIndex
        if (leadIndex[lead_sel] >= leadIndex[lead_sel + 1]) {
            std::cout << "ERROR: leadIndex is not strictly ascending."
                      << " ROW(" << lead_sel << "): " << leadIndex[lead_sel] << "; ROW(" << lead_sel + 1
                      << ") :" << leadIndex[lead_sel + 1] << std::endl;
            return -1;
        }

        bool initialized_elements_found = false;

        int trail_sel = leadIndex[lead_sel + 1];
        while (trail_sel > leadIndex[lead_sel]) {
            trail_sel--;
            if (initialized_element[trail_sel] == false) {
                uninitialized_elements_found = true;
                if (initialized_elements_found) {
                    std::cout << "ERROR: trailIndex as an invalid not-initialized element at POS: " << trail_sel
                              << "; ROW: " << lead_sel << std::endl;
                    return -2;
                }
            } else {
                initialized_elements_found = true;
                if (trail_sel > leadIndex[lead_sel] && trailIndex[trail_sel] <= trailIndex[trail_sel - 1]) {
                    std::cout << "ERROR: trailIndex at POS: " << trail_sel - 1
                              << " is not greater than the following element."
                              << " COL(" << trail_sel - 1 << "): " << trailIndex[trail_sel] << "; COL(" << trail_sel
                              << ") :" << trailIndex[trail_sel] << "; ROW: " << lead_sel << std::endl;
                    return -4;
                }
            }
        }
        if (!initialized_elements_found) {
            std::cout << "WARNING: no elements in ROW: " << lead_sel << std::endl;
            return 3;
        }
    }

    if (uninitialized_elements_found)
        std::cout << "INFO: the matrix is not compressed" << std::endl;
    else
        std::cout << "OK: matrix verified" << std::endl;

    return (uninitialized_elements_found) ? 1 : 0;
}

void ChCSMatrix::ImportFromDatFile(std::string path, bool row_major_format_on) {
    std::ifstream a_file, ia_file, ja_file;
    a_file.open(path + "/a.dat");
    ja_file.open(path + "/ja.dat");
    ia_file.open(path + "/ia.dat");

    if (!a_file.is_open())
        assert(0);

    int leadInd_sel;
    for (leadInd_sel = 0; leadInd_sel <= *leading_dimension; leadInd_sel++)
        ia_file >> leadIndex[leadInd_sel];

    Reset(m_num_rows, m_num_cols);

    ia_file.seekg(0);

    for (leadInd_sel = 0; leadInd_sel <= *leading_dimension; leadInd_sel++)
        ia_file >> leadIndex[leadInd_sel];
    leadInd_sel--;

    int trailInd_sel;
    for (trailInd_sel = 0; trailInd_sel < leadIndex[leadInd_sel]; trailInd_sel++) {
        a_file >> values[trailInd_sel];
        ja_file >> trailIndex[trailInd_sel];
    }

    if (trailInd_sel != leadIndex[leadInd_sel])
        assert(0);

    a_file.close();
    ja_file.close();
    ia_file.close();
}

void ChCSMatrix::ExportToDatFile(std::string filepath, int precision) const {
    std::ofstream a_file, ia_file, ja_file;
    a_file.open(filepath + "/a.dat");
    ja_file.open(filepath + "/ja.dat");
    ia_file.open(filepath + "/ia.dat");
    a_file << std::scientific << std::setprecision(precision);
    ja_file << std::scientific << std::setprecision(precision);
    ia_file << std::scientific << std::setprecision(precision);

    for (auto trailInd_sel = 0; trailInd_sel < leadIndex[*leading_dimension]; trailInd_sel++) {
        a_file << values[trailInd_sel] << "\n";
        ja_file << trailIndex[trailInd_sel] << "\n";
    }

    for (auto leadInd_sel = 0; leadInd_sel <= *leading_dimension; leadInd_sel++) {
        ia_file << leadIndex[leadInd_sel] << "\n";
    }

    a_file.close();
    ja_file.close();
    ia_file.close();
}

void ChCSMatrix::LoadSparsityPattern(ChSparsityPatternLearner& sparsity_learner) {
    auto& row_lists = sparsity_learner.GetSparsityPattern();
    *leading_dimension =
        sparsity_learner.IsRowMajor() ? sparsity_learner.GetNumRows() : sparsity_learner.GetNumColumns();
    *trailing_dimension =
        sparsity_learner.IsRowMajor() ? sparsity_learner.GetNumColumns() : sparsity_learner.GetNumRows();
    auto nnz = sparsity_learner.GetNNZ();
    leadIndex.resize(*leading_dimension + 1);
    trailIndex.resize(nnz);

    leadIndex[0] = 0;
    for (auto lead_sel = 0; lead_sel < *leading_dimension; ++lead_sel) {
        leadIndex[lead_sel + 1] = leadIndex[lead_sel] + static_cast<int>(row_lists[lead_sel].size());
        std::copy(row_lists[lead_sel].begin(), row_lists[lead_sel].end(), trailIndex.begin() + leadIndex[lead_sel]);
    }

    values.resize(nnz);
    initialized_element.assign(nnz, true);
    m_lock_broken = false;
    isCompressed = true;
}

void ChCSMatrix::distribute_integer_range_on_vector(index_vector_t& vector, int initial_number, int final_number) {
    double delta = static_cast<double>(final_number - initial_number) / (vector.size() - 1);
    for (auto el_sel = 0; el_sel < vector.size(); el_sel++) {
        vector[el_sel] = static_cast<int>(std::round(delta * el_sel));
    }
}

void ChCSMatrix::reset_arrays(int lead_dim, int trail_dim, int nonzeros) {
    // break sparsity lock
    m_lock_broken = true;

    // update dimensions (redundant if called from constructor)
    *leading_dimension = lead_dim;
    *trailing_dimension = trail_dim;

    // check if there is at least one element per row
    nonzeros = std::max(nonzeros, lead_dim);

    // allocate the arrays
    leadIndex.resize(*leading_dimension + 1);
    trailIndex.resize(nonzeros);
    values.resize(nonzeros);
    initialized_element.assign(nonzeros, false);

    // make leadIndex span over available space
    distribute_integer_range_on_vector(leadIndex, 0, nonzeros);

    //// set the number of max shifts
    // max_shifts = GetNNZ() * 2 / *leading_dimension;

    isCompressed = false;
}

void ChCSMatrix::insert(int& trail_i_sel, const int& lead_sel) {
    isCompressed = false;
    m_lock_broken = true;

    bool OK_also_out_of_row = true;  // look for viable positions also in other rows respect to the one selected
    bool OK_also_onelement_rows = false;

    auto trailIndexlength = leadIndex[*leading_dimension];
    int shift_fw = 0;  // 0 means no viable position found forward
    int shift_bw = 0;  // 0 means no viable position found backward

    // TODO: optimize?
    // look for not-initialized elements FORWARD
    auto lead_sel_fw = lead_sel;
    for (auto trail_i = trail_i_sel + 1; trail_i < trailIndexlength && (trail_i - trail_i_sel) < max_shifts;
         ++trail_i) {
        if (!initialized_element[trail_i])  // look for not initialized elements
        {
            // check if it is out of row
            if (!OK_also_out_of_row && trail_i >= leadIndex[lead_sel + 1])
                break;

            // check if it is in 1element row
            if (!OK_also_onelement_rows)  // check if we are out of the starting row
            {
                // find the beginning of row that follows the one in which we have found an initialized space
                for (; leadIndex[lead_sel_fw] <= trail_i; ++lead_sel_fw) {
                }
                if (leadIndex[lead_sel_fw - 1] + 1 >= leadIndex[lead_sel_fw])
                    continue;
            }
            shift_fw = trail_i - trail_i_sel;
            break;
        }
    }

    // look for not-initialized elements BACWARD
    auto lead_sel_bw = lead_sel;
    if (OK_also_out_of_row)  // do it only if 'out of row' insertions are allowed
    {
        for (auto trail_i = trail_i_sel - 1;
             trail_i >= 0 && (trail_i_sel - trail_i) < std::min(max_shifts, std::max(shift_fw, 1)); --trail_i) {
            if (!initialized_element[trail_i])  // look for not initialized elements
            {
                // check if it is in 1element row
                if (!OK_also_onelement_rows)  // check if we are out of the starting row
                {
                    // find the beginning of row that follows the one in which we have found an initialized space
                    for (; leadIndex[lead_sel_bw] > trail_i; --lead_sel_bw) {
                    }
                    if (leadIndex[lead_sel_bw + 1] - 1 <= leadIndex[lead_sel_bw])
                        continue;
                }
                shift_bw = trail_i - trail_i_sel;
                break;
            }
        }
    }

    if (shift_bw == 0 && shift_fw == 0) {
        // no viable position found
        // some space has to be made right where trail_i_sel is pointing
        // meanwhile we create some additional space also to all the other rows
        // in order to reduce the risk of another redistribution
        // so trail_i_sel WILL CHANGE

        auto desired_storage_augmentation = static_cast<int>(std::max(GetTrailingIndexLength() * 0.2, 1.0));
        trail_i_sel = Inflate(desired_storage_augmentation, lead_sel, trail_i_sel);

    } else {
        // shift the elements in order to have a not initialized position where trail_i_sel points
        // trail_i_sel WILL CHANGE if backward, WON'T CHANGE if forward
        // WARNING! move_backward is actually forward (towards the end of the array)
        if (shift_bw < 0 && -shift_bw < shift_fw) {
            if (shift_bw < -1) {
                // move is not actually 'moving'; it's just for cleaner code!
                std::move(trailIndex.begin() + trail_i_sel + shift_bw + 1, trailIndex.begin() + trail_i_sel,
                          trailIndex.begin() + trail_i_sel + shift_bw);
                std::move(values.begin() + trail_i_sel + shift_bw + 1, values.begin() + trail_i_sel,
                          values.begin() + trail_i_sel + shift_bw);
                std::move(initialized_element.begin() + trail_i_sel + shift_bw + 1,
                          initialized_element.begin() + trail_i_sel,
                          initialized_element.begin() + trail_i_sel + shift_bw);
            }

            for (lead_sel_bw = lead_sel; leadIndex[lead_sel_bw] > trail_i_sel + shift_bw; --lead_sel_bw) {
                leadIndex[lead_sel_bw]--;
            }

            trail_i_sel--;

        } else {
            // move is not actually 'moving'; it's just for cleaner code!
            std::move_backward(trailIndex.begin() + trail_i_sel, trailIndex.begin() + trail_i_sel + shift_fw,
                               trailIndex.begin() + trail_i_sel + shift_fw + 1);
            std::move_backward(values.begin() + trail_i_sel, values.begin() + trail_i_sel + shift_fw,
                               values.begin() + trail_i_sel + shift_fw + 1);
            std::move_backward(initialized_element.begin() + trail_i_sel,
                               initialized_element.begin() + trail_i_sel + shift_fw,
                               initialized_element.begin() + trail_i_sel + shift_fw + 1);

            for (lead_sel_fw = lead_sel + 1; leadIndex[lead_sel_fw] <= trail_i_sel + shift_fw; ++lead_sel_fw) {
                leadIndex[lead_sel_fw]++;
            }
        }
    }
    // let the returning function store the value, takes care of initialize element and so on...

}  // end insert

void ChCSMatrix::copy_and_distribute(const index_vector_t& trailIndex_src,
                                     const values_vector_t& values_src,
                                     const std::vector<bool>& initialized_element_src,
                                     index_vector_t& trailIndex_dest,
                                     values_vector_t& values_dest,
                                     std::vector<bool>& initialized_element_dest,
                                     int& trail_ins,
                                     int lead_ins,
                                     int storage_augm) {
    assert(storage_augm > 0);
    assert(trail_ins <= leadIndex[*leading_dimension]);  // a new element can be added at most at
                                                         // leadIndex[*leading_dimension] and not beyond
    assert(leadIndex[*leading_dimension] + storage_augm <=
           values_dest.size());  // the destination arrays must already be fit to accommodate the new size

    double storage_delta = static_cast<double>(storage_augm - 1) / *leading_dimension;

    // start from ending point of source and destination vectors
    auto trail_i_dest = leadIndex[*leading_dimension] + storage_augm;
    auto trail_i_src = leadIndex[*leading_dimension];
    // update leadIndex
    auto lead_i = *leading_dimension;

    while (trail_i_src > 0) {
        if (trail_i_src == leadIndex[lead_i]) {
            // evaluate until which element we have to store not-initialized elements
            int fill_up_until = static_cast<int>(
                std::round(leadIndex[lead_i] + storage_delta * (lead_i - 1) + (((lead_i - 1) >= lead_ins) ? 1 : 0)));

            // insertion point located: make space
            if (trail_i_src == trail_ins && lead_i == lead_ins) {
                --trail_i_dest;
                trail_ins = trail_i_dest;
            }

            // update leadIndex
            leadIndex[lead_i] = trail_i_dest;

            --trail_i_dest;
            // fill with not-initialized elements
            while (trail_i_dest >= static_cast<int>(fill_up_until)) {
                initialized_element_dest[trail_i_dest] = false;
                --trail_i_dest;
            }
            ++trail_i_dest;
            --lead_i;

            // insertion point located: make space
            if (trail_i_src == trail_ins && lead_i == lead_ins) {
                --trail_i_dest;
                trail_ins = trail_i_dest;
            }

        } else {
            if (trail_i_src == trail_ins && lead_i == lead_ins) {
                --trail_i_dest;
                trail_ins = trail_i_dest;
            }
        }

        // move back and copy elements
        --trail_i_src;
        --trail_i_dest;

        trailIndex_dest[trail_i_dest] = trailIndex_src[trail_i_src];
        initialized_element_dest[trail_i_dest] = initialized_element_src[trail_i_src];
        values_dest[trail_i_dest] = values_src[trail_i_src];
    }
}

}  // end namespace chrono
