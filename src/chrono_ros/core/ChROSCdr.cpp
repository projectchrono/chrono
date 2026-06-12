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
// Authors: Patrick Chen
// =============================================================================

#include "chrono_ros/core/ChROSCdr.h"

#include <cstring>

namespace chrono {
namespace ros {
namespace core {

bool IsHostLittleEndian() {
    const uint16_t probe = 0x0102;
    uint8_t first;
    std::memcpy(&first, &probe, 1);
    return first == 0x02;
}

// ----------------------------------------------------------------------------
// CdrWriter
// ----------------------------------------------------------------------------

CdrWriter::CdrWriter(std::vector<uint8_t>& out, bool with_encapsulation) : m_out(out), m_start(out.size()) {
    if (!IsHostLittleEndian()) {
        throw CdrError("this codec supports little-endian hosts only");
    }
    if (with_encapsulation) {
        m_out.insert(m_out.end(), kCdrEncapsulation, kCdrEncapsulation + kCdrEncapsulationSize);
    }
    m_origin = m_out.size();
}

void CdrWriter::Align(size_t alignment) {
    const size_t body_pos = m_out.size() - m_origin;
    const size_t remainder = body_pos % alignment;
    if (remainder != 0) {
        m_out.insert(m_out.end(), alignment - remainder, 0);
    }
}

template <typename T>
void CdrWriter::WriteScalar(T v) {
    Align(sizeof(T));
    const size_t pos = m_out.size();
    m_out.resize(pos + sizeof(T));
    std::memcpy(m_out.data() + pos, &v, sizeof(T));
}

void CdrWriter::WriteBool(bool v) {
    WriteScalar<uint8_t>(v ? 1 : 0);
}
void CdrWriter::WriteUInt8(uint8_t v) {
    WriteScalar(v);
}
void CdrWriter::WriteInt8(int8_t v) {
    WriteScalar(v);
}
void CdrWriter::WriteUInt16(uint16_t v) {
    WriteScalar(v);
}
void CdrWriter::WriteInt16(int16_t v) {
    WriteScalar(v);
}
void CdrWriter::WriteUInt32(uint32_t v) {
    WriteScalar(v);
}
void CdrWriter::WriteInt32(int32_t v) {
    WriteScalar(v);
}
void CdrWriter::WriteUInt64(uint64_t v) {
    WriteScalar(v);
}
void CdrWriter::WriteInt64(int64_t v) {
    WriteScalar(v);
}
void CdrWriter::WriteFloat32(float v) {
    WriteScalar(v);
}
void CdrWriter::WriteFloat64(double v) {
    WriteScalar(v);
}

void CdrWriter::WriteString(const std::string& v) {
    if (v.find('\0') != std::string::npos) {
        throw CdrError("string contains an embedded NUL character, which cannot be represented in a CDR string");
    }
    if (v.size() + 1 > UINT32_MAX) {
        throw CdrError("string too long for CDR (length must fit in uint32)");
    }
    WriteUInt32(static_cast<uint32_t>(v.size() + 1));  // length includes the NUL terminator
    const size_t pos = m_out.size();
    m_out.resize(pos + v.size() + 1);
    std::memcpy(m_out.data() + pos, v.data(), v.size());
    m_out[pos + v.size()] = 0;
}

void CdrWriter::WriteBytes(const void* data, size_t size) {
    if (size == 0)
        return;
    if (data == nullptr) {
        throw CdrError("WriteBytes called with null data and non-zero size");
    }
    const size_t pos = m_out.size();
    m_out.resize(pos + size);
    std::memcpy(m_out.data() + pos, data, size);
}

void CdrWriter::WriteBlob(const void* data, size_t size) {
    if (size > UINT32_MAX) {
        throw CdrError("blob too large for CDR (size must fit in uint32)");
    }
    WriteUInt32(static_cast<uint32_t>(size));
    WriteBytes(data, size);
}

// ----------------------------------------------------------------------------
// CdrReader
// ----------------------------------------------------------------------------

CdrReader::CdrReader(const uint8_t* data, size_t size, bool with_encapsulation)
    : m_data(data), m_size(size), m_pos(0) {
    if (data == nullptr && size != 0) {
        throw CdrError("null data with non-zero size");
    }
    if (!IsHostLittleEndian()) {
        throw CdrError("this codec supports little-endian hosts only");
    }
    if (with_encapsulation) {
        Require(kCdrEncapsulationSize);
        // Representation identifier: 0x0000 = CDR_BE, 0x0001 = CDR_LE.
        if (m_data[0] != 0x00 || m_data[1] != 0x01) {
            throw CdrError("unsupported CDR encapsulation (expected little-endian CDR, identifier {0x00,0x01})");
        }
        m_pos = kCdrEncapsulationSize;  // options bytes are ignored per spec
    }
    m_origin = m_pos;
}

void CdrReader::Require(size_t bytes) const {
    if (m_pos + bytes > m_size) {
        throw CdrError("input truncated: need " + std::to_string(bytes) + " byte(s) at offset " +
                       std::to_string(m_pos) + ", but only " + std::to_string(m_size - m_pos) + " remain");
    }
}

void CdrReader::Align(size_t alignment) {
    const size_t body_pos = m_pos - m_origin;
    const size_t remainder = body_pos % alignment;
    if (remainder != 0) {
        Require(alignment - remainder);
        m_pos += alignment - remainder;
    }
}

void CdrReader::Skip(size_t size) {
    Require(size);
    m_pos += size;
}

template <typename T>
T CdrReader::ReadScalar() {
    Align(sizeof(T));
    Require(sizeof(T));
    T v;
    std::memcpy(&v, m_data + m_pos, sizeof(T));
    m_pos += sizeof(T);
    return v;
}

bool CdrReader::ReadBool() {
    return ReadScalar<uint8_t>() != 0;
}
uint8_t CdrReader::ReadUInt8() {
    return ReadScalar<uint8_t>();
}
int8_t CdrReader::ReadInt8() {
    return ReadScalar<int8_t>();
}
uint16_t CdrReader::ReadUInt16() {
    return ReadScalar<uint16_t>();
}
int16_t CdrReader::ReadInt16() {
    return ReadScalar<int16_t>();
}
uint32_t CdrReader::ReadUInt32() {
    return ReadScalar<uint32_t>();
}
int32_t CdrReader::ReadInt32() {
    return ReadScalar<int32_t>();
}
uint64_t CdrReader::ReadUInt64() {
    return ReadScalar<uint64_t>();
}
int64_t CdrReader::ReadInt64() {
    return ReadScalar<int64_t>();
}
float CdrReader::ReadFloat32() {
    return ReadScalar<float>();
}
double CdrReader::ReadFloat64() {
    return ReadScalar<double>();
}

std::string CdrReader::ReadString() {
    const uint32_t length = ReadUInt32();
    if (length == 0) {
        // A zero length (no NUL at all) is produced by some implementations
        // for empty strings; tolerate it on input.
        return std::string();
    }
    Require(length);
    if (m_data[m_pos + length - 1] != 0) {
        throw CdrError("malformed CDR string: missing NUL terminator");
    }
    std::string v(reinterpret_cast<const char*>(m_data + m_pos), length - 1);
    m_pos += length;
    return v;
}

void CdrReader::ReadBytes(void* out, size_t size) {
    if (size == 0)
        return;
    Require(size);
    std::memcpy(out, m_data + m_pos, size);
    m_pos += size;
}

std::vector<uint8_t> CdrReader::ReadBlob() {
    const uint32_t size = ReadUInt32();
    Require(size);
    std::vector<uint8_t> v(m_data + m_pos, m_data + m_pos + size);
    m_pos += size;
    return v;
}

}  // namespace core
}  // namespace ros
}  // namespace chrono
