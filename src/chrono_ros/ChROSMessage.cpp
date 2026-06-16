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

#include "chrono_ros/ChROSMessage.h"

namespace chrono {
namespace ros {

// ----------------------------------------------------------------------------
// ChROSMessage
// ----------------------------------------------------------------------------

ChROSMessage::ChROSMessage(std::shared_ptr<core::MessageBuilder> root) : m_root(std::move(root)) {
    m_node = m_root.get();
}

ChROSMessage::ChROSMessage(std::shared_ptr<core::MessageBuilder> root, core::MessageBuilder* node)
    : m_root(std::move(root)), m_node(node) {}

core::MessageBuilder& ChROSMessage::Node() const {
    return *m_node;
}

void ChROSMessage::SetBool(const std::string& path, bool value) {
    Node().SetBool(path, value);
}
void ChROSMessage::SetInt(const std::string& path, int64_t value) {
    Node().SetInt(path, value);
}
void ChROSMessage::SetUInt(const std::string& path, uint64_t value) {
    Node().SetUInt(path, value);
}
void ChROSMessage::SetDouble(const std::string& path, double value) {
    Node().SetDouble(path, value);
}
void ChROSMessage::SetString(const std::string& path, const std::string& value) {
    Node().SetString(path, value);
}
void ChROSMessage::SetBlob(const std::string& path, const void* data, size_t count) {
    Node().SetBlob(path, data, count);
}
void ChROSMessage::SetBlobCopy(const std::string& path, const void* data, size_t count) {
    Node().SetBlobCopy(path, data, count);
}
void ChROSMessage::SetBlobBytes(const std::string& path, const void* data, size_t nbytes) {
    Node().SetBlobBytes(path, data, nbytes);
}
void ChROSMessage::SetStringArray(const std::string& path, const std::vector<std::string>& value) {
    Node().SetStringArray(path, value);
}

ChROSMessage ChROSMessage::Nested(const std::string& path) {
    return ChROSMessage(m_root, &Node().GetNested(path));
}

ChROSMessage ChROSMessage::AppendMessage(const std::string& path) {
    return ChROSMessage(m_root, &Node().AppendMessage(path));
}

void ChROSMessage::SetTime(const std::string& path, double time_seconds) {
    Node().SetTime(path, time_seconds);
}

void ChROSMessage::Clear() {
    Node().Clear();
}

std::string ChROSMessage::DescribeType() const {
    return Node().GetSchema().ToString();
}

// ----------------------------------------------------------------------------
// ChROSMessageView
// ----------------------------------------------------------------------------

ChROSMessageView::ChROSMessageView(std::shared_ptr<const core::Schema> schema,
                                   std::shared_ptr<const std::vector<uint8_t>> data)
    : m_data(std::move(data)), m_reader(std::move(schema), m_data->data(), m_data->size()) {}

ChROSMessageView::ChROSMessageView(std::shared_ptr<const std::vector<uint8_t>> data, core::MessageReader reader)
    : m_data(std::move(data)), m_reader(std::move(reader)) {}

bool ChROSMessageView::GetBool(const std::string& path) const {
    return m_reader.GetBool(path);
}
int64_t ChROSMessageView::GetInt(const std::string& path) const {
    return m_reader.GetInt(path);
}
uint64_t ChROSMessageView::GetUInt(const std::string& path) const {
    return m_reader.GetUInt(path);
}
double ChROSMessageView::GetDouble(const std::string& path) const {
    return m_reader.GetDouble(path);
}
std::string ChROSMessageView::GetString(const std::string& path) const {
    return m_reader.GetString(path);
}
std::vector<std::string> ChROSMessageView::GetStringArray(const std::string& path) const {
    return m_reader.GetStringArray(path);
}
core::BlobView ChROSMessageView::GetBlob(const std::string& path) const {
    return m_reader.GetBlob(path);
}

std::vector<uint8_t> ChROSMessageView::CopyBlob(const std::string& path) const {
    const core::BlobView view = m_reader.GetBlob(path);
    if (view.count == 0) {
        return {};
    }
    return std::vector<uint8_t>(view.data, view.data + view.SizeBytes());
}

size_t ChROSMessageView::GetCount(const std::string& path) const {
    return m_reader.GetCount(path);
}

ChROSMessageView ChROSMessageView::GetMessage(const std::string& path, size_t index) const {
    return ChROSMessageView(m_data, m_reader.GetMessage(path, index));
}

double ChROSMessageView::GetTimeSec(const std::string& path) const {
    return m_reader.GetTimeSec(path);
}

std::string ChROSMessageView::DescribeType() const {
    return m_reader.GetSchema().ToString();
}

}  // namespace ros
}  // namespace chrono
