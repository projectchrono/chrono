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

// template <typename T = Derived>
// std::enable_if_t<std::is_same_v<typename T::Scalar, double> && (T::Options & Eigen::RowMajor) &&
//                      std::is_same_v<typename T::StorageIndex, int>,
//                  void>
// ArchiveIn(chrono::ChArchiveIn& archive_in) {
//     bool is_compressed;
//     archive_in >> chrono::CHNVP(is_compressed);
//     if (isCompressed()) {
//         archive_in >> chrono::CHNVP(innerNonZeroPtr);
//         archive_in >> chrono::CHNVP(innerIndexPtr);
//         archive_in >> chrono::CHNVP(outerIndexPtr);
//     } else {
//         archive_in >> chrono::CHNVP(valuePtr);
//         archive_in >> chrono::CHNVP(innerIndexPtr);
//         archive_in >> chrono::CHNVP(outerIndexPtr);
//     }
// }
//
// template <typename T = Derived>
// std::enable_if_t<std::is_same_v<typename T::Scalar, std::complex<double>> && (T::Options & Eigen::ColMajor) &&
//                      std::is_same_v<typename T::StorageIndex, int>,
//                  void>
void ArchiveIn(chrono::ChArchiveIn& archive_in) {
    bool is_compressed;
    archive_in >> chrono::CHNVP(is_compressed);

    // clean up any previous value stored in m_innerNonZeros
    if (!isCompressed()) {
        internal::conditional_aligned_delete_auto<StorageIndex, true>(m_innerNonZeros, m_outerSize);
    }

    size_t data_size;
    size_t inner_size;
    size_t outer_size;
    archive_in >> chrono::make_ChNameValue("data_size", data_size);
    archive_in >> chrono::make_ChNameValue("inner_size", inner_size);
    archive_in >> chrono::make_ChNameValue("outer_size", outer_size);

    m_innerSize = inner_size;
    m_outerSize = outer_size;
    m_data.resize(data_size);

    {
        archive_in.in_array_pre("values", data_size);
        // *valuePtr already resized by m_data.resize(data_size);
        for (size_t i = 0; i < data_size; i++) {
            archive_in >> chrono::CHNVP(valuePtr()[(Eigen::Index)i], std::to_string(i).c_str());
            archive_in.in_array_between("values");
        }
        archive_in.in_array_end("values");
    }

    {
        archive_in.in_array_pre("innerIndices", data_size);
        // *innerIndexPtr already resized by m_data.resize(data_size);
        for (size_t i = 0; i < data_size; i++) {
            archive_in >> chrono::CHNVP(innerIndexPtr()[(Eigen::Index)i], std::to_string(i).c_str());
            archive_in.in_array_between("innerIndices");
        }
        archive_in.in_array_end("innerIndices");
    }

    {
        size_t outerIndicesSize = outer_size + 1;
        archive_in.in_array_pre("outerIndices", outerIndicesSize);
        m_outerIndex = internal::conditional_aligned_new_auto<StorageIndex, true>(outerIndicesSize);
        for (size_t i = 0; i < outerIndicesSize; i++) {
            archive_in >> chrono::CHNVP(outerIndexPtr()[(Eigen::Index)i], std::to_string(i).c_str());
            archive_in.in_array_between("outerIndices");
        }
        archive_in.in_array_end("outerIndices");
    }

    if (!is_compressed) {
        m_innerNonZeros = internal::conditional_aligned_new_auto<StorageIndex, true>(outer_size);
        archive_in.in_array_pre("innerNNZs", outer_size);
        for (size_t i = 0; i < outer_size; i++) {
            archive_in >> chrono::CHNVP(m_innerNonZeros[(Eigen::Index)i], std::to_string(i).c_str());
            archive_in.in_array_between("innerNNZs");
        }
        archive_in.in_array_end("innerNNZs");
    }
}

//
// void ArchiveIn(chrono::ChArchiveIn& archive_in) {
//    bool is_compressed;
//    archive_in >> chrono::CHNVP(is_compressed);
//    if (isCompressed()) {
//        archive_in >> chrono::CHNVP(innerNonZeroPtr);
//        archive_in >> chrono::CHNVP(innerIndexPtr);
//        archive_in >> chrono::CHNVP(outerIndexPtr);
//    } else {
//        archive_in >> chrono::CHNVP(valuePtr);
//        archive_in >> chrono::CHNVP(innerIndexPtr);
//        archive_in >> chrono::CHNVP(outerIndexPtr);
//    }
//}

void ArchiveOut(chrono::ChArchiveOut& archive_out) {
    bool is_compressed = isCompressed();
    archive_out << chrono::CHNVP(is_compressed);

    size_t data_size = m_data.size();
    size_t inner_size = innerSize();
    size_t outer_size = outerSize();
    archive_out << chrono::make_ChNameValue("data_size", data_size);
    archive_out << chrono::make_ChNameValue("inner_size", inner_size);
    archive_out << chrono::make_ChNameValue("outer_size", outer_size);

    {
        Scalar* foo = 0;
        chrono::ChValueSpecific<Scalar*> specVal(foo, "values", 0);
        archive_out.out_array_pre(specVal, data_size);
        for (size_t i = 0; i < data_size; i++) {
            archive_out << chrono::CHNVP(valuePtr()[i], std::to_string(i).c_str());
            archive_out.out_array_between(specVal, data_size);
        }
        archive_out.out_array_end(specVal, data_size);
    }
    {
        StorageIndex* foo = 0;
        chrono::ChValueSpecific<StorageIndex*> specVal(foo, "innerIndices", 0);
        archive_out.out_array_pre(specVal, data_size);
        for (size_t i = 0; i < data_size; i++) {
            archive_out << chrono::CHNVP(innerIndexPtr()[i], std::to_string(i).c_str());
            archive_out.out_array_between(specVal, data_size);
        }
        archive_out.out_array_end(specVal, data_size);
    }
    {
        StorageIndex* foo = 0;
        size_t outerIndicesSize = outer_size + 1;
        chrono::ChValueSpecific<StorageIndex*> specVal(foo, "outerIndices", 0);
        archive_out.out_array_pre(specVal, outerIndicesSize);
        for (size_t i = 0; i < outerIndicesSize; i++) {
            archive_out << chrono::CHNVP(outerIndexPtr()[i], std::to_string(i).c_str());
            archive_out.out_array_between(specVal, outerIndicesSize);
        }
        archive_out.out_array_end(specVal, outerIndicesSize);
    }

    if (!isCompressed()) {
        Scalar* foo = 0;
        chrono::ChValueSpecific<Scalar*> specVal(foo, "innerNNZs", 0);
        archive_out.out_array_pre(specVal, outer_size);
        for (size_t i = 0; i < outer_size; i++) {
            archive_out << chrono::CHNVP(innerNonZeroPtr()[i], std::to_string(i).c_str());
            archive_out.out_array_between(specVal, outer_size);
        }
        archive_out.out_array_end(specVal, outer_size);
    }
}

#endif
