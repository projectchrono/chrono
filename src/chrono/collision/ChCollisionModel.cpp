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

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/ChCollisionShapeCylinder.h"
#include "chrono/physics/ChBody.h"
#include "chrono/geometry/ChLineSegment.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChCollisionModel)  // NO! Abstract class!

static double default_model_envelope = 0.03;
static double default_safe_margin = 0.01;

ChCollisionModel::ChCollisionModel() : contactable(nullptr), family_group(1), family_mask(0x7FFF), impl(nullptr) {
    model_envelope = (float)default_model_envelope;
    model_safe_margin = (float)default_safe_margin;
}

ChCollisionModel::ChCollisionModel(const ChCollisionModel& other) : contactable(nullptr), impl(nullptr) {
    // Create new shape instances (sharing the collision shapes)
    for (const auto& si : other.m_shape_instances) {
        const auto& shape = si.first;
        const auto& frame = si.second;
        m_shape_instances.push_back({shape, frame});
    }
    model_envelope = other.model_envelope;
    model_safe_margin = other.model_safe_margin;
    family_group = other.family_group;
    family_mask = other.family_mask;
}

ChCollisionModel::~ChCollisionModel() {
    m_shape_instances.clear();
    impl = nullptr;
}

void ChCollisionModel::SetContactable(ChContactable* contactable) {
    this->contactable = contactable;
}

void ChCollisionModel::Clear() {
    m_shape_instances.clear();
}

void ChCollisionModel::SyncPosition() {
    // Sync position of this collision model only if it was processed by the current collision system
    // (through BindAll or BindItem)

    if (impl)
        impl->SyncPosition();
}

ChPhysicsItem* ChCollisionModel::GetPhysicsItem() {
    return contactable->GetPhysicsItem();
}

geometry::ChAABB ChCollisionModel::GetBoundingBox() const {
    // Return an updated bounding box only if this collision model was processed by the current collision system
    // (through BindAll or BindItem)

    if (impl)
        return impl->GetBoundingBox();

    return geometry::ChAABB();
}

void ChCollisionModel::SetDefaultSuggestedEnvelope(double envelope) {
    default_model_envelope = envelope;
}

void ChCollisionModel::SetDefaultSuggestedMargin(double margin) {
    default_safe_margin = margin;
}

// static
double ChCollisionModel::GetDefaultSuggestedEnvelope() {
    return default_model_envelope;
}

// static
double ChCollisionModel::GetDefaultSuggestedMargin() {
    return default_safe_margin;
}

// Set family_group to a power of 2, with the set bit in position 'family'.
void ChCollisionModel::SetFamily(int family) {
    assert(family >= 0 && family < 15);
    family_group = (1 << family);
    if (impl)
        impl->OnFamilyChange(family_group, family_mask);
}

// Return the position of the single bit set in family_group.
int ChCollisionModel::GetFamily() {
    unsigned i = 1;
    int pos = 1;
    while (!(i & family_group)) {
        i = i << 1;
        pos++;
    }
    return pos - 1;
}

// Clear the family_mask bit in position mfamily.
void ChCollisionModel::SetFamilyMaskNoCollisionWithFamily(int family) {
    assert(family >= 0 && family < 15);
    family_mask &= ~(1 << family);
    if (impl)
        impl->OnFamilyChange(family_group, family_mask);
}

// Set the family_mask bit in position mfamily.
void ChCollisionModel::SetFamilyMaskDoCollisionWithFamily(int family) {
    assert(family >= 0 && family < 15);
    family_mask |= (1 << family);
    if (impl)
        impl->OnFamilyChange(family_group, family_mask);
}

// Return true if the family_mask bit in position mfamily is set.
bool ChCollisionModel::GetFamilyMaskDoesCollisionWithFamily(int family) {
    assert(family >= 0 && family < 15);
    return (family_mask & (1 << family)) != 0;
}

// Set the collision family group of this model.
// In order to properly encode a collision family, the value 'group' must be a power of 2.
void ChCollisionModel::SetFamilyGroup(short int group) {
    assert(group > 0 && !(group & (group - 1)));
    family_group = group;
    if (impl)
        impl->OnFamilyChange(family_group, family_mask);
}

// Set the collision mask for this model.
// In order to properly encode a collision mask, the value 'mask' must not exceed 0x7FFFF (i.e. 15 right bits all set)
void ChCollisionModel::SetFamilyMask(short int mask) {
    assert(mask >= 0 && mask <= 0x7FFF);
    family_mask = mask;
    if (impl)
        impl->OnFamilyChange(family_group, family_mask);
}

void ChCollisionModel::AddShape(std::shared_ptr<ChCollisionShape> shape, const ChFrame<>& frame) {
    m_shape_instances.push_back({shape, frame});
}

void ChCollisionModel::AddShapes(std::shared_ptr<ChCollisionModel> model, const ChFrame<>& frame) {
    for (const auto& s : model->m_shape_instances) {
        const auto& shape = s.first;
        const auto& shape_frame = s.second;

        AddShape(shape, frame * shape_frame);
    }
}

void ChCollisionModel::AddCylinder(std::shared_ptr<ChMaterialSurface> material,
                                   double radius,
                                   const ChVector<>& p1,
                                   const ChVector<>& p2) {
    geometry::ChLineSegment seg(p1, p2);
    auto height = seg.GetLength();
    auto frame = seg.GetFrame();

    auto cylinder_shape = chrono_types::make_shared<ChCollisionShapeCylinder>(material, radius, height);
    AddShape(cylinder_shape, frame);
}

void ChCollisionModel::SetAllShapesMaterial(std::shared_ptr<ChMaterialSurface> mat) {
    assert(m_shape_instances.size() == 0 ||
           m_shape_instances[0].first->m_material->GetContactMethod() == mat->GetContactMethod());
    for (auto& shape : m_shape_instances)
        shape.first->m_material = mat;
}

void ChCollisionModel::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionModel>();

    // serialize all member data:
    marchive << CHNVP(model_envelope);
    marchive << CHNVP(model_safe_margin);
    marchive << CHNVP(family_group);
    marchive << CHNVP(family_mask);
    marchive << CHNVP(m_shape_instances);
    marchive << CHNVP(contactable);
}

void ChCollisionModel::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCollisionModel>();

    // stream in all member data:
    marchive >> CHNVP(model_envelope);
    marchive >> CHNVP(model_safe_margin);
    marchive >> CHNVP(family_group);
    marchive >> CHNVP(family_mask);
    marchive >> CHNVP(m_shape_instances);
    marchive >> CHNVP(contactable);
}

// -----------------------------------------------------------------------------

ChCollisionModelImpl::ChCollisionModelImpl(ChCollisionModel* collision_model) : model(collision_model) {
    model->impl = this;
}

ChContactable* ChCollisionModelImpl::GetContactable() {
    return model->GetContactable();
}

}  // end namespace chrono
