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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChPhysicsItem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChPhysicsItem)

ChPhysicsItem::ChPhysicsItem(const ChPhysicsItem& other) : ChObj(other) {
    // Do not copy the system; this is initialized at insertion time
    system = NULL;
    offset_x = other.offset_x;
    offset_w = other.offset_w;
    offset_L = other.offset_L;
}

ChPhysicsItem::~ChPhysicsItem() {
    SetSystem(NULL);  // note that this might remove collision model from system
}

void ChPhysicsItem::SetSystem(ChSystem* m_system) {
    if (system == m_system)  // shortcut if no change
        return;
    if (system) {
        if (GetCollide())
            RemoveCollisionModelsFromSystem();
    }
    system = m_system;  // set here
    if (system) {
        if (GetCollide())
            AddCollisionModelsToSystem();
    }
}

void ChPhysicsItem::AddVisualModel(std::shared_ptr<ChVisualModel> model) {
    vis_model_instance = std::shared_ptr<ChVisualModelInstance>(new ChVisualModelInstance(model));
    vis_model_instance->m_owner = this;
}

std::shared_ptr<ChVisualModel> ChPhysicsItem::GetVisualModel() const {
    if (!vis_model_instance)
        return nullptr;
    return vis_model_instance->GetModel();
}

void ChPhysicsItem::AddVisualShape(std::shared_ptr<ChVisualShape> shape, const ChFrame<>& frame) {
    if (!vis_model_instance) {
        auto model = chrono_types::make_shared<ChVisualModel>();
        AddVisualModel(model);
    }
    vis_model_instance->GetModel()->AddShape(shape, frame);
}

std::shared_ptr<ChVisualShape> ChPhysicsItem::GetVisualShape(unsigned int i) const {
    if (!vis_model_instance)
        return nullptr;
    return vis_model_instance->GetModel()->GetShape(i);
}

void ChPhysicsItem::AddVisualShapeFEA(std::shared_ptr<ChVisualShapeFEA> shape) {
    if (!vis_model_instance) {
        auto model = chrono_types::make_shared<ChVisualModel>();
        AddVisualModel(model);
    }
    vis_model_instance->GetModel()->AddShapeFEA(shape);
}

std::shared_ptr<ChVisualShapeFEA> ChPhysicsItem::GetVisualShapeFEA(unsigned int i) const {
    if (!vis_model_instance)
        return nullptr;
    return vis_model_instance->GetModel()->GetShapeFEA(i);
}

void ChPhysicsItem::AddCamera(std::shared_ptr<ChCamera> camera) {
    camera->m_owner = this;
    cameras.push_back(camera);
}

void ChPhysicsItem::GetTotalAABB(ChVector<>& bbmin, ChVector<>& bbmax) {
    bbmin.Set(-1e200, -1e200, -1e200);
    bbmax.Set(1e200, 1e200, 1e200);
}

void ChPhysicsItem::GetCenter(ChVector<>& mcenter) {
    ChVector<> mmin, mmax;
    GetTotalAABB(mmin, mmax);
    mcenter = (mmin + mmax) * 0.5;
}

void ChPhysicsItem::Update(double mytime, bool update_assets) {
    ChTime = mytime;

    if (update_assets) {
        for (auto& camera : cameras)
            camera->Update();
        if (vis_model_instance)
            vis_model_instance->Update(GetVisualModelFrame());
    }
}

void ChPhysicsItem::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChPhysicsItem>();

    // serialize parent class
    ChObj::ArchiveOUT(marchive);

    // serialize all member data:
    // marchive << CHNVP(system); ***TODO***
    // marchive << CHNVP(offset_x);
    // marchive << CHNVP(offset_w);
    // marchive << CHNVP(offset_L);
}

/// Method to allow de serialization of transient data from archives.
void ChPhysicsItem::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChPhysicsItem>();

    // deserialize parent class
    ChObj::ArchiveIN(marchive);

    // stream in all member data:
    // marchive >> CHNVP(system); ***TODO***
    // marchive >> CHNVP(offset_x);
    // marchive >> CHNVP(offset_w);
    // marchive >> CHNVP(offset_L);
}

}  // end namespace chrono
