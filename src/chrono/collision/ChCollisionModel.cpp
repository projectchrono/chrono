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

class my_enum_mappers {
  public:
    CH_ENUM_MAPPER_BEGIN(ChCollisionSystemType);
    CH_ENUM_VAL(ChCollisionSystemType::BULLET);
    CH_ENUM_VAL(ChCollisionSystemType::CHRONO);
    CH_ENUM_VAL(ChCollisionSystemType::OTHER);
    CH_ENUM_MAPPER_END(ChCollisionSystemType);
};

static double default_model_envelope = 0.03;
static double default_safe_margin = 0.01;

ChCollisionModel::ChCollisionModel() : contactable(nullptr), family_group(1), family_mask(0x7FFF) {
    model_envelope = (float)default_model_envelope;
    model_safe_margin = (float)default_safe_margin;
}

ChCollisionModel::~ChCollisionModel() {
    m_shape_instances.clear();
}

void ChCollisionModel::Clear() {
    // Delete all inserted collision shape instances
    m_shape_instances.clear();

    // Remove this model from the collsion system
    Dissociate();
}

void ChCollisionModel::Build() {
    // Remove this model from the collision system
    Dissociate();

    // Populate model with current list of collision shapes
    Populate();

    // Associate this model with the collision system
    Associate();
}

ChPhysicsItem* ChCollisionModel::GetPhysicsItem() {
    return contactable->GetPhysicsItem();
}

void ChCollisionModel::SetDefaultSuggestedEnvelope(double menv) {
    default_model_envelope = menv;
}

void ChCollisionModel::SetDefaultSuggestedMargin(double mmargin) {
    default_safe_margin = mmargin;
}

// static
double ChCollisionModel::GetDefaultSuggestedEnvelope() {
    return default_model_envelope;
}

// static
double ChCollisionModel::GetDefaultSuggestedMargin() {
    return default_safe_margin;
}

// Set family_group to a power of 2, with the set bit in position mfamily.
void ChCollisionModel::SetFamily(int mfamily) {
    assert(mfamily >= 0 && mfamily < 15);
    family_group = (1 << mfamily);
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
void ChCollisionModel::SetFamilyMaskNoCollisionWithFamily(int mfamily) {
    assert(mfamily >= 0 && mfamily < 15);
    family_mask &= ~(1 << mfamily);
}

// Set the family_mask bit in position mfamily.
void ChCollisionModel::SetFamilyMaskDoCollisionWithFamily(int mfamily) {
    assert(mfamily >= 0 && mfamily < 15);
    family_mask |= (1 << mfamily);
}

// Return true if the family_mask bit in position mfamily is set.
bool ChCollisionModel::GetFamilyMaskDoesCollisionWithFamily(int mfamily) {
    assert(mfamily >= 0 && mfamily < 15);
    return (family_mask & (1 << mfamily)) != 0;
}

// Set the collision family group of this model.
// In order to properly encode a collision family, the value 'group' must be a power of 2.
void ChCollisionModel::SetFamilyGroup(short int group) {
    assert(group > 0 && !(group & (group - 1)));
    family_group = group;
}

// Set the collision mask for this model.
// In order to properly encode a collision mask, the value 'mask' must not exceed 0x7FFFF (i.e. 15 right bits all set)
void ChCollisionModel::SetFamilyMask(short int mask) {
    assert(mask >= 0 && mask <= 0x7FFF);
    family_mask = mask;
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

std::vector<double> ChCollisionModel::GetShapeDimensions(std::shared_ptr<ChCollisionShape> shape) const {
    std::vector<double> dims;
    switch (shape->GetType()) {
        case ChCollisionShape::Type::SPHERE: {
            auto sphere = std::static_pointer_cast<ChCollisionShapeSphere>(shape);
            dims = {sphere->GetRadius()};
            break;
        }
        case ChCollisionShape::Type::BOX: {
            auto box = std::static_pointer_cast<ChCollisionShapeBox>(shape);
            const auto& hdims = box->GetHalflengths();
            dims = {hdims.x(), hdims.y(), hdims.z()};
            break;
        }
        case ChCollisionShape::Type::ELLIPSOID: {
            auto ellipsoid = std::static_pointer_cast<ChCollisionShapeEllipsoid>(shape);
            const auto& r = ellipsoid->GetSemiaxes();
            dims = {r.x(), r.y(), r.z()};
            break;
        }
        case ChCollisionShape::Type::CYLINDER: {
            auto cylinder = std::static_pointer_cast<ChCollisionShapeCylinder>(shape);
            auto height = cylinder->GetHeight();
            auto radius = cylinder->GetRadius();
            dims = {radius, radius, height / 2};
            break;
        }
        case ChCollisionShape::Type::CYLSHELL: {
            auto cylshell = std::static_pointer_cast<ChCollisionShapeCylindricalShell>(shape);
            auto height = cylshell->GetHeight();
            auto radius = cylshell->GetRadius();
            dims = {radius, radius, height / 2};
            break;
        }
        case ChCollisionShape::Type::CONE: {
            auto cone = std::static_pointer_cast<ChCollisionShapeCone>(shape);
            auto height = cone->GetHeight();
            auto radius = cone->GetRadius();
            dims = {radius, radius, height / 2};
            break;
        }
        case ChCollisionShape::Type::CAPSULE: {
            auto capsule = std::static_pointer_cast<ChCollisionShapeCapsule>(shape);
            auto height = capsule->GetHeight();
            auto radius = capsule->GetRadius();
            dims = {radius, radius, height / 2};
            break;
        }
        case ChCollisionShape::Type::ROUNDEDBOX: {
            auto box = std::static_pointer_cast<ChCollisionShapeRoundedBox>(shape);
            const auto& hdims = box->GetHalflengths();
            auto sradius = box->GetSRadius();
            dims = {hdims.x(), hdims.y(), hdims.z(), sradius};
            break;
        }
        case ChCollisionShape::Type::ROUNDEDCYL: {
            auto cylinder = std::static_pointer_cast<ChCollisionShapeRoundedCylinder>(shape);
            auto height = cylinder->GetHeight();
            auto radius = cylinder->GetRadius();
            auto sradius = cylinder->GetSRadius();
            dims = {radius, radius, height / 2, sradius};
            break;
        }
        default:
            break;
    }

    return dims;
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

}  // end namespace chrono
