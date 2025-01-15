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
    system = m_system;
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

ChAABB ChPhysicsItem::GetTotalAABB() const {
    return ChAABB();
}

ChVector3d ChPhysicsItem::GetCenter() const {
    auto bbox = GetTotalAABB();
    return (bbox.min + bbox.max) * 0.5;
}


/// Takes the F force term, scale by c and scale by i-th weight in Wd at node offset, and adds to R at given offset:
///    R += c*F* Wi
/// This is a weighted version of IntLoadResidual_F, where some items are scaled, ex. to simulate splitting a mass.
/// Many children classes do not need to implement this: here a default fallback to IntLoadResidual_F is implemented.
void ChPhysicsItem::IntLoadResidual_F_weighted(const unsigned int off,  ///< offset in R residual
    ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
    const double c,          ///< a scaling factor
    ChVectorDynamic<>& Wd    ///< vector with weights, the node effect is "scaled".
) {
    // default use first Wd value at offset off for scaling the entire physics item. Child classes 
    // should override for finer behavior if contain more items etc.
    if (this->GetNumCoordsVelLevel())
        this->IntLoadResidual_F(off, R, c * Wd(off));
    else
        this->IntLoadResidual_F(off, R, c);
}


/// Takes the M*v term,  multiplying mass by a vector, scale by c and and scale by i-th weight in Wd at node offset
/// and adds to R at given offset:
///    R += c*M*v * Wi
/// This is a weighted version of IntLoadResidual_Mv, where some items are scaled, ex. to simulate splitting a mass.
/// Many children classes do not need to implement this: here a default fallback to IntLoadResidual_Mv is implemented.
void ChPhysicsItem::IntLoadResidual_Mv_weighted(const unsigned int off,  ///< offset in R residual
    ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
    const ChVectorDynamic<>& w,  ///< the w vector
    const double c,              ///< a scaling factor
    ChVectorDynamic<>& Wd        ///< vector with weights, the node mass is "scaled".
) {
    // default use first Wd value at offset off for scaling the entire physics item. Child classes 
    // should override for finer behavior if contain more items etc.
    if (this->GetNumCoordsVelLevel())
        this->IntLoadResidual_Mv(off, R, w, c * Wd(off));
    else
        this->IntLoadResidual_Mv(off, R, w, c);
}


/// Adds the lumped mass to a Md vector, representing a mass diagonal matrix. Used by lumped explicit integrators.
/// If mass lumping is impossible or approximate, adds scalar error to "error" parameter.
///    Md += c*diag(M)* Wi
/// This is a weighted version of IntLoadLumpedMass_Md, where some items are scaled, ex. to simulate splitting a mass.
void ChPhysicsItem::IntLoadLumpedMass_Md_weighted(const unsigned int off,  ///< offset in Md vector
    ChVectorDynamic<>& Md,  ///< result: Md vector, diagonal of the lumped mass matrix
    double& err,            ///< result: not touched if lumping does not introduce errors
    const double c,         ///< a scaling factor
    ChVectorDynamic<>& Wd   ///< vector with weights, the node mass is "scaled".
) {
    // default use first Wd value at offset off for scaling the entire physics item. Child classes 
    // should override for finer behavior if contain more items etc.
    if (this->GetNumCoordsVelLevel())
        this->IntLoadLumpedMass_Md(off, Md, err, c * Wd(off));
    else
        this->IntLoadLumpedMass_Md(off, Md, err, c);
}


void ChPhysicsItem::Update(double time, bool update_assets) {
    ChTime = time;

    if (update_assets) {
        UpdateVisualModel();
        for (auto& camera : cameras)
            camera->Update();
    }
}

void ChPhysicsItem::Update(bool update_assets) {
    Update(ChTime, update_assets);
}

void ChPhysicsItem::UpdateVisualModel() {
    if (vis_model_instance)
        vis_model_instance->Update(GetVisualModelFrame());
}

void ChPhysicsItem::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChPhysicsItem>();

    // serialize parent class
    ChObj::ArchiveOut(archive_out);

    // serialize all member data:
    // archive_out << CHNVP(system); ***TODO***
    archive_out << CHNVP(GetVisualModel(), "visual_model");
    archive_out << CHNVP(cameras);
    // archive_out << CHNVP(offset_x);
    // archive_out << CHNVP(offset_w);
    // archive_out << CHNVP(offset_L);
}

/// Method to allow de serialization of transient data from archives.
void ChPhysicsItem::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChPhysicsItem>();

    // deserialize parent class
    ChObj::ArchiveIn(archive_in);

    // stream in all member data:
    // archive_in >> CHNVP(system); ***TODO***
    std::shared_ptr<ChVisualModel> visual_model;
    archive_in >> CHNVP(visual_model);
    if (visual_model)
        AddVisualModel(visual_model);
    archive_in >> CHNVP(cameras);
    // archive_in >> CHNVP(offset_x);
    // archive_in >> CHNVP(offset_w);
    // archive_in >> CHNVP(offset_L);
}

}  // end namespace chrono
