// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include "chrono/assets/ChVisualModel.h"
#include "chrono/physics/ChPhysicsItem.h"

namespace chrono {

void ChVisualModel::AddShape(std::shared_ptr<ChVisualShape> shape, const ChFrame<>& frame) {
    m_shapes.push_back({shape, frame});
}

void ChVisualModel::AddShapeFEA(std::shared_ptr<ChVisualShapeFEA> shapeFEA) {
    m_shapesFEA.push_back(shapeFEA);
    m_shapes.push_back({shapeFEA->m_trimesh_shape, ChFrame<>()});
    m_shapes.push_back({shapeFEA->m_glyphs_shape, ChFrame<>()});
}

void ChVisualModel::Clear() {
    m_shapes.clear();
    m_shapesFEA.clear();
}

void ChVisualModel::Erase(std::shared_ptr<ChVisualShape> shape) {
    auto it = std::find_if(m_shapes.begin(), m_shapes.end(),
                           [&shape](const ShapeInstance& element) { return element.first == shape; });
    if (it != m_shapes.end())
        m_shapes.erase(it);
}

void ChVisualModel::Update(ChPhysicsItem* owner, const ChFrame<>& frame) {
    for (auto& shape : m_shapes) {
        auto xform = frame >> shape.second;
        shape.first->Update(owner, xform);
    }
    for (auto& shapeFEA : m_shapesFEA) {
        shapeFEA->Update(owner, ChFrame<>());
    }
}

// -----------------------------------------------------------------------------

ChVisualModelInstance::ChVisualModelInstance(std::shared_ptr<ChVisualModel> model) : m_model(model), m_owner(nullptr) {}

void ChVisualModelInstance::Update(const ChFrame<>& frame) {
    m_model->Update(m_owner, frame);
}

}  // namespace chrono
