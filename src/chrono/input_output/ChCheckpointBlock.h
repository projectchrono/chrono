// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
//
// Format-independent blocks of checkpoint data, used by physics items, FEA
// elements and FEA nodes to save and restore their internal state.
//
// =============================================================================

#ifndef CH_CHECKPOINT_BLOCK_H
#define CH_CHECKPOINT_BLOCK_H

#include <cstdint>
#include <functional>
#include <string>
#include <type_traits>
#include <vector>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChQuaternion.h"

namespace chrono {

/// @addtogroup chrono_io
/// @{

/// Entry in a block of checkpoint data: either a number or an opaque sequence of bytes.
struct ChCheckpointEntry {
    bool is_bytes = false;            ///< true if this entry holds bytes
    double value = 0;                 ///< numeric value (if not bytes)
    std::vector<std::uint8_t> bytes;  ///< opaque data (if bytes)
};

/// Writer for a block of checkpoint data.
/// A physics item, FEA element or FEA node writes its internal state (data not stored in the system state vectors) to
/// a writer; the checkpoint database stores the resulting block in its own format. The same data must be read back, in
/// the same order, with a ChCheckpointReader. See the developer manual on the physics item lifecycle.
class ChApi ChCheckpointWriter {
  public:
    /// Write a number.
    void Write(double value);

    /// Write an integral number (stored exactly if its magnitude is below 2^53).
    template <typename T, typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
    void Write(T value) {
        Write(static_cast<double>(value));
    }

    /// Write a 3D vector.
    void Write(const ChVector3d& v);

    /// Write a quaternion.
    void Write(const ChQuaterniond& q);

    /// Write a vector of numbers (its size, then its elements).
    void Write(const std::vector<double>& v);

    /// Write an Eigen matrix or vector (its dimensions, then its elements in column-major order).
    template <typename Derived>
    void Write(const Eigen::MatrixBase<Derived>& m) {
        Write(static_cast<double>(m.rows()));
        Write(static_cast<double>(m.cols()));
        for (Eigen::Index j = 0; j < m.cols(); j++)
            for (Eigen::Index i = 0; i < m.rows(); i++)
                Write(static_cast<double>(m(i, j)));
    }

    /// Write an opaque sequence of bytes.
    void WriteBytes(const std::vector<std::uint8_t>& bytes);

    /// Write a nested block.
    void Write(const ChCheckpointWriter& block);

    /// Write, for each index in [0,n), the block produced by the provided function, tagged with that index.
    /// Empty blocks are skipped. Read back with ChCheckpointReader::ReadIndexedBlocks.
    void WriteIndexedBlocks(size_t n, const std::function<void(size_t, ChCheckpointWriter&)>& save);

    /// Return true if nothing was written.
    bool IsEmpty() const { return m_entries.empty(); }

    /// Access the entries written so far.
    const std::vector<ChCheckpointEntry>& GetEntries() const { return m_entries; }

  private:
    std::vector<ChCheckpointEntry> m_entries;
};

/// Reader for a block of checkpoint data written with a ChCheckpointWriter.
/// Every read checks the type of the next entry and throws a std::runtime_error on a mismatch or if the block has no
/// more data, so that a checkpoint that does not match the system fails loudly.
class ChApi ChCheckpointReader {
  public:
    ChCheckpointReader() {}
    explicit ChCheckpointReader(std::vector<ChCheckpointEntry> entries) : m_entries(std::move(entries)) {}

    /// Read a number.
    void Read(double& value);

    /// Read an integral number.
    template <typename T, typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
    void Read(T& value) {
        double v;
        Read(v);
        value = static_cast<T>(v);
    }

    /// Read a 3D vector.
    void Read(ChVector3d& v);

    /// Read a quaternion.
    void Read(ChQuaterniond& q);

    /// Read a vector of numbers. The vector is resized as needed.
    void Read(std::vector<double>& v);

    /// Read an Eigen matrix or vector.
    /// A dynamic-size object is resized as needed; a fixed-size object must have the stored dimensions.
    template <typename Derived>
    void Read(Eigen::PlainObjectBase<Derived>& m) {
        Eigen::Index rows, cols;
        Read(rows);
        Read(cols);
        if ((Derived::RowsAtCompileTime != Eigen::Dynamic && rows != Derived::RowsAtCompileTime) ||
            (Derived::ColsAtCompileTime != Eigen::Dynamic && cols != Derived::ColsAtCompileTime))
            Fail("matrix dimensions do not match");
        m.resize(rows, cols);
        for (Eigen::Index j = 0; j < cols; j++)
            for (Eigen::Index i = 0; i < rows; i++)
                Read(m(i, j));
    }

    /// Read an opaque sequence of bytes.
    void ReadBytes(std::vector<std::uint8_t>& bytes);

    /// Read a nested block.
    void Read(ChCheckpointReader& block);

    /// Read blocks written with ChCheckpointWriter::WriteIndexedBlocks, passing each one to the provided function.
    /// Throws if a stored index is not in [0,n) or if the function does not consume its entire block.
    void ReadIndexedBlocks(size_t n, const std::function<void(size_t, ChCheckpointReader&)>& load);

    /// Return true if all data in this block was read.
    bool IsAtEnd() const { return m_pos == m_entries.size(); }

    /// Throw if not all data in this block was read.
    void CheckAtEnd() const;

  private:
    const ChCheckpointEntry& Next(bool bytes);
    [[noreturn]] void Fail(const std::string& msg) const;

    std::vector<ChCheckpointEntry> m_entries;
    size_t m_pos = 0;
};

/// @} chrono_io

}  // end namespace chrono

#endif
