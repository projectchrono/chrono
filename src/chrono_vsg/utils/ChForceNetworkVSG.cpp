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
// Authors: Radu Serban
// =============================================================================

#include <algorithm>
#include <cmath>
#include <iostream>

#include "chrono/assets/ChVisualMaterial.h"

#include "chrono_vsg/ChVisualSystemVSG.h"
#include "chrono_vsg/utils/ChForceNetworkVSG.h"
#include "chrono_vsg/utils/ChShaderUtilsVSG.h"
#include "chrono_vsg/utils/ChShapeBuilderVSG.h"

namespace chrono {
namespace vsg3d {

ChForceNetworkVSG::ChForceNetworkVSG(unsigned int capacity)
    : m_capacity(capacity),
      m_mode(Mode::CYLINDERS),
      m_visible(true),
      m_use_colormap(true),
      m_colormap_type(ChColormap::Type::JET),
      m_color_min(ChColor(0.0f, 0.0f, 1.0f)),
      m_color_max(ChColor(1.0f, 0.0f, 0.0f)),
      m_vmin(0),
      m_vmax(1),
      m_radius(1e-3),
      m_map_radius(false),
      m_rmin(0),
      m_rmax(0),
      m_num_divs(12),
      m_num_segments(0),
      m_num_dropped(0),
      m_total_dropped(0) {
    m_colormap = chrono_types::make_unique<ChColormap>(m_colormap_type);
}

ChForceNetworkVSG::~ChForceNetworkVSG() {}

void ChForceNetworkVSG::SetMode(Mode mode) {
    m_mode = mode;
    UpdateMasks();
}

void ChForceNetworkVSG::SetColormap(ChColormap::Type type) {
    m_use_colormap = true;
    if (type != m_colormap_type) {
        m_colormap_type = type;
        m_colormap->Load(type);
    }
}

void ChForceNetworkVSG::SetColorRamp(const ChColor& color_min, const ChColor& color_max) {
    m_use_colormap = false;
    m_color_min = color_min;
    m_color_max = color_max;
}

void ChForceNetworkVSG::SetValueRange(double vmin, double vmax) {
    m_vmin = vmin;
    m_vmax = vmax;
}

void ChForceNetworkVSG::SetRadius(double radius) {
    m_radius = radius;
    m_map_radius = false;
}

void ChForceNetworkVSG::SetRadiusRange(double rmin, double rmax) {
    m_rmin = rmin;
    m_rmax = rmax;
    m_map_radius = true;
}

void ChForceNetworkVSG::SetResolution(int num_divs) {
    m_num_divs = std::max(num_divs, 3);
}

void ChForceNetworkVSG::SetVisibility(bool visible) {
    m_visible = visible;
    UpdateMasks();
}

void ChForceNetworkVSG::UpdateMasks() {
    if (!m_root)
        return;
    m_root->children[0].mask = vsg::boolToMask(m_visible && m_mode == Mode::LINES);
    m_root->children[1].mask = vsg::boolToMask(m_visible && m_mode == Mode::CYLINDERS);
}

void ChForceNetworkVSG::Initialize(ChVisualSystemVSG& vsys) {
    if (m_root)
        return;

    if (vsys.IsInitialized()) {
        std::cerr << "ChForceNetworkVSG: the visual system is already initialized; the force network cannot be added." << std::endl;
        return;
    }

    auto options = vsys.GetOptions();

    // Arrays cannot be empty
    size_t size = std::max(m_capacity, 1u);

    // Cylinders, as instances of a unit cylinder (radius 1, length 1, centered at the origin, along the z axis)
    m_translations = vsg::vec3Array::create(size);
    m_rotations = vsg::quatArray::create(size);
    m_scales = vsg::vec3Array::create(size);
    m_colors = vsg::vec4Array::create(size);
    m_translations->properties.dataVariance = vsg::DYNAMIC_DATA;
    m_rotations->properties.dataVariance = vsg::DYNAMIC_DATA;
    m_scales->properties.dataVariance = vsg::DYNAMIC_DATA;
    m_colors->properties.dataVariance = vsg::DYNAMIC_DATA;

    ShapeBuilder::CylinderShapeData cylinder(m_num_divs);
    auto draw = vsg::InstanceDrawIndexed::create();
    draw->assignArrays(vsg::DataList{cylinder.vertices, cylinder.normals});
    draw->assignIndices(cylinder.indices);
    draw->indexCount = static_cast<uint32_t>(cylinder.indices->size());

    auto cylinder_state = createPbrInstancedStateGroup(options, ChVisualMaterial::Default());
    cylinder_state->addChild(draw);

    m_cylinders = vsg::InstanceNode::create();
    m_cylinders->translations = vsg::BufferInfo::create(m_translations);
    m_cylinders->rotations = vsg::BufferInfo::create(m_rotations);
    m_cylinders->scales = vsg::BufferInfo::create(m_scales);
    m_cylinders->colors = vsg::BufferInfo::create(m_colors);
    m_cylinders->instanceCount = 0;
    m_cylinders->child = cylinder_state;

    // Lines, as a line list with a color per end point
    m_line_vertices = vsg::vec3Array::create(2 * size);
    m_line_colors = vsg::vec3Array::create(2 * size);
    m_line_vertices->properties.dataVariance = vsg::DYNAMIC_DATA;
    m_line_colors->properties.dataVariance = vsg::DYNAMIC_DATA;

    m_lines = vsg::VertexDraw::create();
    m_lines->assignArrays(vsg::DataList{m_line_vertices, m_line_colors});
    m_lines->vertexCount = 0;
    m_lines->instanceCount = 1;

    auto line_state = createLineStateGroup(options, VK_PRIMITIVE_TOPOLOGY_LINE_LIST, 1.0f);
    line_state->addChild(m_lines);

    m_root = vsg::Switch::create();
    m_root->addChild(false, line_state);
    m_root->addChild(false, m_cylinders);
    UpdateMasks();

    vsys.GetVSGScene()->addChild(m_root);
}

vsg::vec4 ChForceNetworkVSG::MapColor(float t) const {
    ChColor c = m_use_colormap ? m_colormap->Get(t) : ChColor::Interp(m_color_min, m_color_max, t);
    return vsg::vec4(c.R, c.G, c.B, 1.0f);
}

void ChForceNetworkVSG::SetSegments(unsigned int num_segments, const float* points1, const float* points2, const float* values, const float* colors, const float* radii) {
    if (!m_root)
        return;

    unsigned int n = std::min(num_segments, m_capacity);
    m_num_dropped = num_segments - n;
    m_total_dropped += m_num_dropped;
    m_num_segments = n;

    // Normalized segment value (in [0, 1])
    float vmin = static_cast<float>(m_vmin);
    float scale = (m_vmax > m_vmin) ? static_cast<float>(1 / (m_vmax - m_vmin)) : 0.0f;
    auto normalized = [&](unsigned int i) { return values ? std::clamp((values[i] - vmin) * scale, 0.0f, 1.0f) : 0.0f; };

    if (m_mode == Mode::CYLINDERS) {
        float rmin = static_cast<float>(m_rmin);
        float rdelta = static_cast<float>(m_rmax - m_rmin);
        float radius = static_cast<float>(m_radius);

        for (unsigned int i = 0; i < n; i++) {
            vsg::vec3 a(points1[3 * i + 0], points1[3 * i + 1], points1[3 * i + 2]);
            vsg::vec3 b(points2[3 * i + 0], points2[3 * i + 1], points2[3 * i + 2]);
            vsg::vec3 d = b - a;
            float length = vsg::length(d);

            // Rotation of the cylinder axis (z) onto the segment direction u. A cylinder is symmetric, so u is flipped
            // to point to positive z; the rotation (axis z x u, angle acos(u_z)) is then never degenerate.
            vsg::quat q(0, 0, 0, 1);
            if (length > 0) {
                vsg::vec3 u = d / length;
                if (u.z < 0)
                    u = -u;
                float w = 1 + u.z;
                float norm = std::sqrt(2 * w);
                q.set(-u.y / norm, u.x / norm, 0, w / norm);
            }

            float t = normalized(i);
            float r = radii ? radii[i] : (m_map_radius ? rmin + t * rdelta : radius);

            (*m_translations)[i] = (a + b) * 0.5f;
            (*m_rotations)[i] = q;
            (*m_scales)[i].set(r, r, length);
            (*m_colors)[i] = colors ? vsg::vec4(colors[3 * i + 0], colors[3 * i + 1], colors[3 * i + 2], 1.0f) : MapColor(t);
        }

        m_cylinders->instanceCount = n;
        m_translations->dirty();
        m_rotations->dirty();
        m_scales->dirty();
        m_colors->dirty();
    } else {
        for (unsigned int i = 0; i < n; i++) {
            vsg::vec3 c;
            if (colors) {
                c.set(colors[3 * i + 0], colors[3 * i + 1], colors[3 * i + 2]);
            } else {
                auto c4 = MapColor(normalized(i));
                c.set(c4.r, c4.g, c4.b);
            }
            (*m_line_vertices)[2 * i + 0].set(points1[3 * i + 0], points1[3 * i + 1], points1[3 * i + 2]);
            (*m_line_vertices)[2 * i + 1].set(points2[3 * i + 0], points2[3 * i + 1], points2[3 * i + 2]);
            (*m_line_colors)[2 * i + 0] = c;
            (*m_line_colors)[2 * i + 1] = c;
        }

        m_lines->vertexCount = 2 * n;
        m_line_vertices->dirty();
        m_line_colors->dirty();
    }
}

}  // namespace vsg3d
}  // namespace chrono
