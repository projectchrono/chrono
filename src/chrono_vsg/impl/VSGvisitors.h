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
// Screen capture code from https://github.com/vsg-dev/vsgExamples.git
//
// =============================================================================
// Radu Serban, Rainer Gericke
// =============================================================================

#pragma once

// Utility visitor class for accessing the vec3 data in the N-th vertex buffer of an object.
// Note: since VSG v.1.0.8 VertexIndexDraw is used instead of BindVertexBuffers!
template <int N>
class FindVec3BufferData : public vsg::Visitor {
  public:
    FindVec3BufferData() : m_buffer(nullptr) {}
    void apply(vsg::Object& object) override { object.traverse(*this); }
    void apply(vsg::BindVertexBuffers& bvd) override {
        if (bvd.arrays.empty())
            return;
        bvd.arrays[N]->data->accept(*this);
    }
    void apply(vsg::VertexDraw& vd) override {
        if (vd.arrays.empty())
            return;
        vd.arrays[N]->data->accept(*this);
    }
    void apply(vsg::VertexIndexDraw& vid) override {
        if (vid.arrays.empty())
            return;
        vid.arrays[N]->data->accept(*this);
    }
    void apply(vsg::vec3Array& vertices) override {
        if (!m_buffer)
            m_buffer = &vertices;
    }
    vsg::ref_ptr<vsg::vec3Array> getBufferData() {
        vsg::ref_ptr<vsg::vec3Array> data;
        data = const_cast<vsg::vec3Array*>(m_buffer);
        return data;
    }
    vsg::vec3Array* m_buffer;
};

// Utility visitor class for accessing the vec4 data in the N-th vertex buffer of an object.
// Note: since VSG v.1.0.8 VertexIndexDraw is used instead of BindVertexBuffers!
template <int N>
class FindVec4BufferData : public vsg::Visitor {
  public:
    FindVec4BufferData() : m_buffer(nullptr) {}
    void apply(vsg::Object& object) override { object.traverse(*this); }
    void apply(vsg::BindVertexBuffers& bvd) override {
        if (bvd.arrays.empty())
            return;
        bvd.arrays[N]->data->accept(*this);
    }
    void apply(vsg::VertexIndexDraw& vid) override {
        if (vid.arrays.empty())
            return;
        vid.arrays[N]->data->accept(*this);
    }
    void apply(vsg::vec4Array& vertices) override {
        if (!m_buffer)
            m_buffer = &vertices;
    }
    vsg::ref_ptr<vsg::vec4Array> getBufferData() {
        vsg::ref_ptr<vsg::vec4Array> data;
        data = const_cast<vsg::vec4Array*>(m_buffer);
        return data;
    }
    vsg::vec4Array* m_buffer;
};

// Utility visitor for fetching the BufferInfo used by the N-th vertex array binding.
template <int N>
class FindVertexArrayBufferInfo : public vsg::Visitor {
  public:
    void apply(vsg::Object& object) override {
        // Continue traversal until the target binding is discovered
        if (!bufferInfo)
            object.traverse(*this);
    }

    void apply(vsg::BindVertexBuffers& bvb) override {
        // Grab the BufferInfo from legacy BindVertexBuffers nodes when slot N exists
        if (bufferInfo || bvb.arrays.size() <= N)
            return;
        bufferInfo = bvb.arrays[N].cast<vsg::BufferInfo>();
    }

    void apply(vsg::VertexIndexDraw& vid) override {
        // Support modern VertexIndexDraw nodes that replaced BindVertexBuffers in newer VSG
        if (bufferInfo || vid.arrays.size() <= N)
            return;
        bufferInfo = vid.arrays[N].cast<vsg::BufferInfo>();
    }

    vsg::ref_ptr<vsg::BufferInfo> bufferInfo;
};
