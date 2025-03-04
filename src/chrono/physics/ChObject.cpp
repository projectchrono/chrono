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

#include <atomic>

#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChObject.h"

namespace chrono {

ChObj::ChObj() : m_tag(-1), ChTime(0) {
    m_identifier = GenerateUniqueIdentifier();
}

ChObj::ChObj(const ChObj& other) {
    m_identifier = GenerateUniqueIdentifier();

    m_name = other.m_name;
    ChTime = other.ChTime;
}

int ChObj::GenerateUniqueIdentifier() {
    static std::atomic<std::uint32_t> uid{0};
    return ++uid;
}

// -----------------------------------------------------------------------------

void ChObj::AddVisualModel(std::shared_ptr<ChVisualModel> model) {
    vis_model_instance = std::shared_ptr<ChVisualModelInstance>(new ChVisualModelInstance(model));
    vis_model_instance->m_owner = this;
}

std::shared_ptr<ChVisualModel> ChObj::GetVisualModel() const {
    if (!vis_model_instance)
        return nullptr;
    return vis_model_instance->GetModel();
}

void ChObj::AddVisualShape(std::shared_ptr<ChVisualShape> shape, const ChFrame<>& frame) {
    if (!vis_model_instance) {
        auto model = chrono_types::make_shared<ChVisualModel>();
        AddVisualModel(model);
    }
    vis_model_instance->GetModel()->AddShape(shape, frame);
}

std::shared_ptr<ChVisualShape> ChObj::GetVisualShape(unsigned int i) const {
    if (!vis_model_instance)
        return nullptr;
    return vis_model_instance->GetModel()->GetShape(i);
}

void ChObj::AddVisualShapeFEA(std::shared_ptr<ChVisualShapeFEA> shape) {
    shape->obj = this;
    if (!vis_model_instance) {
        auto model = chrono_types::make_shared<ChVisualModel>();
        AddVisualModel(model);
    }
    vis_model_instance->GetModel()->AddShapeFEA(shape);
}

std::shared_ptr<ChVisualShapeFEA> ChObj::GetVisualShapeFEA(unsigned int i) const {
    if (!vis_model_instance)
        return nullptr;
    return vis_model_instance->GetModel()->GetShapeFEA(i);
}

void ChObj::AddCamera(std::shared_ptr<ChCamera> camera) {
    camera->m_owner = this;
    cameras.push_back(camera);
}

// -----------------------------------------------------------------------------

void ChObj::Update(double time, bool update_assets) {
    ChTime = time;

    if (update_assets) {
        UpdateVisualModel();
        for (auto& camera : cameras)
            camera->Update();
    }
}

void ChObj::UpdateVisualModel() {
    if (vis_model_instance)
        vis_model_instance->Update(GetVisualModelFrame());
}

// -----------------------------------------------------------------------------

void ChObj::ArchiveOut(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChObj>();

    archive_out << CHNVP(m_name);
    archive_out << CHNVP(m_tag);
    archive_out << CHNVP(ChTime);

    archive_out << CHNVP(GetVisualModel(), "visual_model");
    archive_out << CHNVP(cameras);
}

void ChObj::ArchiveIn(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChObj>();

    archive_in >> CHNVP(m_name);
    archive_in >> CHNVP(m_tag);
    archive_in >> CHNVP(ChTime);

    std::shared_ptr<ChVisualModel> visual_model;
    archive_in >> CHNVP(visual_model);
    if (visual_model)
        AddVisualModel(visual_model);
    archive_in >> CHNVP(cameras);
}

}  // end namespace chrono
