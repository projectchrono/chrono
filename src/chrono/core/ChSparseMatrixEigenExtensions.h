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
// Authors: Dario Mangoni
// =============================================================================
//
// Chrono-specific extensions to Eigen::SparseMatrixBase
//
// =============================================================================

#ifndef CHSPARSEMATRIXEIGENEXTENSIONS_H
#define CHSPARSEMATRIXEIGENEXTENSIONS_H

virtual void SetElement(int row, int col, double el, bool overwrite = true) {
    overwrite ? coeffRef(row, col) = el : coeffRef(row, col) += el;
}

void setZeroValues() {
    for (int k = 0; k < outerSize(); ++k) {
        for (InnerIterator it(*this, k); it; ++it) {
            it.valueRef() = 0.0;
        }
    }
}

void ArchiveIn(chrono::ChArchiveIn& archive_in) {
    std::vector<int> reserve_sizes;

    int r;
    int c;
    archive_in >> chrono::make_ChNameValue("rows", r);
    archive_in >> chrono::make_ChNameValue("cols", c);
    resize(r, c);
    archive_in >> chrono::CHNVP(reserve_sizes);

    int totalSize = 0;
    for (int j = 0; j < reserve_sizes.size(); ++j) {
        totalSize += reserve_sizes[j];
    }
    reserve(reserve_sizes);

    std::vector<int> rowIndex;
    archive_in >> chrono::CHNVP(rowIndex);
    std::vector<int> colIndex;
    archive_in >> chrono::CHNVP(colIndex);
    std::vector<Scalar> values;
    archive_in >> chrono::CHNVP(values);

    size_t data_size;
    {
        for (auto i = 0; i < values.size(); ++i) {
            insert(rowIndex[i], colIndex[i]) = values[i];
        }
    }

    makeCompressed();
}

void ArchiveOut(chrono::ChArchiveOut& archive_out) {
    std::vector<int> reserve_sizes;

    makeCompressed();

    int r = rows();
    int c = cols();
    archive_out << chrono::make_ChNameValue("rows", r);
    archive_out << chrono::make_ChNameValue("cols", c);
    reserve_sizes.resize(m_outerSize);
    int totalSize = 0;
    for (int j = 0; j < m_outerSize; ++j) {
        reserve_sizes[j] = m_outerIndex[j + 1] - m_outerIndex[j];
        totalSize += reserve_sizes[j];
    }
    archive_out << chrono::CHNVP(reserve_sizes);

    std::vector<int> rowIndex;
    rowIndex.reserve(totalSize);
    std::vector<int> colIndex;
    colIndex.reserve(totalSize);

    auto data_size = totalSize;
    {
        int* foo = 0;
        chrono::ChValueSpecific<int*> specVal(foo, "rowIndex", 0);
        archive_out.out_array_pre(specVal, data_size);
        for (int os = 0; os < m_outerSize; ++os) {
            for (InnerIterator it(*this, os); it; ++it) {
                int rr = it.row();
                archive_out << chrono::CHNVP(rr, std::to_string(os).c_str());
                archive_out.out_array_between(specVal, data_size);
            }
        }
        archive_out.out_array_end(specVal, data_size);
    }
    {
        int* foo = 0;
        chrono::ChValueSpecific<int*> specVal(foo, "colIndex", 0);
        archive_out.out_array_pre(specVal, data_size);
        for (int os = 0; os < m_outerSize; ++os) {
            for (InnerIterator it(*this, os); it; ++it) {
                int cc = it.col();
                archive_out << chrono::CHNVP(cc, std::to_string(os).c_str());
                archive_out.out_array_between(specVal, data_size);
            }
        }
        archive_out.out_array_end(specVal, data_size);
    }
    {
        Scalar* foo = 0;
        chrono::ChValueSpecific<Scalar*> specVal(foo, "values", 0);
        archive_out.out_array_pre(specVal, data_size);
        for (int os = 0; os < m_outerSize; ++os) {
            for (InnerIterator it(*this, os); it; ++it) {
                archive_out << chrono::CHNVP(it.value(), std::to_string(os).c_str());
                archive_out.out_array_between(specVal, data_size);
            }
        }
        archive_out.out_array_end(specVal, data_size);
    }
}

#endif
