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
#include "chrono/physics/ChObject.h"

namespace chrono {
CH_FACTORY_REGISTER(ChVisualModel)

void ChVisualModel::AddShape(std::shared_ptr<ChVisualShape> shape, const ChFramed& frame, bool wireframe) {
    m_shapes.push_back({shape, frame, wireframe});
}

void ChVisualModel::AddShapeFEA(std::shared_ptr<ChVisualShapeFEA> shapeFEA) {
    m_shapesFEA.push_back(shapeFEA);
    m_shapes.push_back({shapeFEA->m_trimesh_shape, ChFramed(), false});
    m_shapes.push_back({shapeFEA->m_glyphs_shape, ChFramed(), false});
}

void ChVisualModel::EnableWireframe(bool val) {
    for (auto& shape : m_shapes)
        shape.wireframe = val;
}

void ChVisualModel::Clear() {
    m_shapes.clear();
    m_shapesFEA.clear();
}

void ChVisualModel::Erase(std::shared_ptr<ChVisualShape> shape) {
    auto it = std::find_if(m_shapes.begin(), m_shapes.end(),
                           [&shape](const ChVisualShapeInstance& element) { return element.shape == shape; });
    if (it != m_shapes.end())
        m_shapes.erase(it);
}

void ChVisualModel::Update(ChObj* owner, const ChFrame<>& frame) {
    for (auto& si : m_shapes) {
        auto xform = frame >> si.frame;
        si.shape->Update(owner, xform);
    }
    for (auto& shapeFEA : m_shapesFEA) {
        shapeFEA->Update(owner, ChFrame<>());
    }
}

ChAABB ChVisualModel::GetBoundingBox() const {
    ChAABB aabb;
    for (const auto& si : m_shapes) {
        auto shape_aabb = si.shape->GetBoundingBox();
        aabb += shape_aabb.Transform(si.frame);
    }
    return aabb;
}

void ChVisualModel::ArchiveOut(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChVisualModel>();

    archive_out << CHNVP(m_shapes);
    // archive_out << CHNVP(m_shapesFEA); // TODO: DARIOM enable archive
}

void ChVisualModel::ArchiveIn(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChVisualModel>();

    archive_in >> CHNVP(m_shapes);
    // archive_in >> CHNVP(m_shapesFEA); // TODO: DARIOM enable archive
}

// -----------------------------------------------------------------------------

ChVisualModelInstance::ChVisualModelInstance(std::shared_ptr<ChVisualModel> model) : m_model(model), m_owner(nullptr) {}

void ChVisualModelInstance::Update(const ChFrame<>& frame) {
    m_model->Update(m_owner, frame);
}

// -----------------------------------------------------------------------------

void ChVisualShapeInstance::ArchiveOut(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChVisualShapeInstance>();
    archive_out << CHNVP(shape);
    archive_out << CHNVP(frame);
    archive_out << CHNVP(wireframe);
}

void ChVisualShapeInstance::ArchiveIn(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChVisualShapeInstance>();
    archive_in >> CHNVP(shape);
    archive_in >> CHNVP(frame);
    archive_in >> CHNVP(wireframe);
}

}  // namespace chrono
