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

#include <iostream>
#include <stdexcept>

#include "chrono/input_output/ChCheckpointBlock.h"

namespace chrono {

// -----------------------------------------------------------------------------

void ChCheckpointWriter::Write(double value) {
    ChCheckpointEntry entry;
    entry.value = value;
    m_entries.push_back(std::move(entry));
}

void ChCheckpointWriter::Write(const ChVector3d& v) {
    Write(v.x());
    Write(v.y());
    Write(v.z());
}

void ChCheckpointWriter::Write(const ChQuaterniond& q) {
    Write(q.e0());
    Write(q.e1());
    Write(q.e2());
    Write(q.e3());
}

void ChCheckpointWriter::Write(const std::vector<double>& v) {
    Write(v.size());
    for (auto val : v)
        Write(val);
}

void ChCheckpointWriter::WriteBytes(const std::vector<std::uint8_t>& bytes) {
    ChCheckpointEntry entry;
    entry.is_bytes = true;
    entry.bytes = bytes;
    m_entries.push_back(std::move(entry));
}

void ChCheckpointWriter::Write(const ChCheckpointWriter& block) {
    Write(block.m_entries.size());
    m_entries.insert(m_entries.end(), block.m_entries.begin(), block.m_entries.end());
}

void ChCheckpointWriter::WriteIndexedBlocks(size_t n, const std::function<void(size_t, ChCheckpointWriter&)>& save) {
    std::vector<std::pair<size_t, ChCheckpointWriter>> blocks;
    for (size_t i = 0; i < n; i++) {
        ChCheckpointWriter block;
        save(i, block);
        if (!block.IsEmpty())
            blocks.emplace_back(i, std::move(block));
    }

    Write(blocks.size());
    for (const auto& block : blocks) {
        Write(block.first);
        Write(block.second);
    }
}

// -----------------------------------------------------------------------------

void ChCheckpointReader::Fail(const std::string& msg) const {
    std::cerr << "Error: checkpoint data does not match: " << msg << std::endl;
    throw std::runtime_error("Checkpoint data does not match: " + msg);
}

const ChCheckpointEntry& ChCheckpointReader::Next(bool bytes) {
    if (m_pos >= m_entries.size())
        Fail("not enough data");
    const auto& entry = m_entries[m_pos++];
    if (entry.is_bytes != bytes)
        Fail(bytes ? "expected bytes, found a number" : "expected a number, found bytes");
    return entry;
}

void ChCheckpointReader::Read(double& value) {
    value = Next(false).value;
}

void ChCheckpointReader::Read(ChVector3d& v) {
    Read(v.x());
    Read(v.y());
    Read(v.z());
}

void ChCheckpointReader::Read(ChQuaterniond& q) {
    Read(q.e0());
    Read(q.e1());
    Read(q.e2());
    Read(q.e3());
}

void ChCheckpointReader::Read(std::vector<double>& v) {
    size_t n;
    Read(n);
    v.resize(n);
    for (auto& val : v)
        Read(val);
}

void ChCheckpointReader::ReadBytes(std::vector<std::uint8_t>& bytes) {
    bytes = Next(true).bytes;
}

void ChCheckpointReader::Read(ChCheckpointReader& block) {
    size_t n;
    Read(n);
    if (m_pos + n > m_entries.size())
        Fail("nested block extends past the end of its parent block");
    block = ChCheckpointReader(std::vector<ChCheckpointEntry>(m_entries.begin() + m_pos, m_entries.begin() + m_pos + n));
    m_pos += n;
}

void ChCheckpointReader::ReadIndexedBlocks(size_t n, const std::function<void(size_t, ChCheckpointReader&)>& load) {
    size_t num_blocks;
    Read(num_blocks);
    for (size_t b = 0; b < num_blocks; b++) {
        size_t index;
        Read(index);
        if (index >= n)
            Fail("block index " + std::to_string(index) + " out of range (" + std::to_string(n) + " objects)");
        ChCheckpointReader block;
        Read(block);
        load(index, block);
        block.CheckAtEnd();
    }
}

void ChCheckpointReader::CheckAtEnd() const {
    if (!IsAtEnd())
        Fail("unread data left in block");
}

}  // end namespace chrono
