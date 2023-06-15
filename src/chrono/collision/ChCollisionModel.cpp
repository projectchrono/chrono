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
#include "chrono/physics/ChBody.h"
#include "chrono/geometry/ChLineSegment.h"

namespace chrono {
namespace collision {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChCollisionModel)  // NO! Abstract class!

static double default_model_envelope = 0.03;
static double default_safe_margin = 0.01;

ChCollisionModel::ChCollisionModel() : mcontactable(nullptr), family_group(1), family_mask(0x7FFF) {
    model_envelope = (float)default_model_envelope;
    model_safe_margin = (float)default_safe_margin;
}

void ChCollisionModel::CopyShapes(ChCollisionModel* other) {
    m_shapes = other->m_shapes;
}

ChPhysicsItem* ChCollisionModel::GetPhysicsItem() {
    return mcontactable->GetPhysicsItem();
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

bool ChCollisionModel::AddCylinder(std::shared_ptr<ChMaterialSurface> material,
                                   double radius,
                                   const ChVector<>& p1,
                                   const ChVector<>& p2) {
    geometry::ChLineSegment seg(p1, p2);
    auto height = seg.GetLength();
    auto frame = seg.GetFrame();

    std::cout << height << std::endl;
    std::cout << frame.GetPos() << std::endl;
    std::cout << frame.GetA() << std::endl;

    return AddCylinder(material, radius, height, frame.GetPos(), frame.GetA());
}

bool ChCollisionModel::AddConvexHullsFromFile(std::shared_ptr<ChMaterialSurface> material,
                                              ChStreamInAscii& mstream,
                                              const ChVector<>& pos,
                                              const ChMatrix33<>& rot) {
    std::vector<ChVector<double> > ptlist;

    char bufdata[200];
    int linechar = 0;
    while (!mstream.End_of_stream()) {
        // read one line
        linechar = 0;
        while (!mstream.End_of_stream()) {
            try {
                mstream >> bufdata[linechar];
            } catch (const ChException &) {
            };
            if ((bufdata[linechar] == (char)13) || (bufdata[linechar] == (char)10)) {
                bufdata[linechar] = 0;
                break;
            }
            linechar++;
            if (linechar >= 200)
                throw(ChException("Too long line in parsing"));
        }
        bufdata[linechar + 1] = 0;

        ////bool parsedline = false;
        if (bufdata[0] != *"#") {
            ////parsedline = true;
        }
        if (strcmp(bufdata, "hull") == 0) {
            if (ptlist.size())
                this->AddConvexHull(material, ptlist, pos, rot);
            ptlist.clear();
            ////parsedline = true;
        }
        float vx, vy, vz;
        if (sscanf(bufdata, "%g %g %g", &vx, &vy, &vz) == 3) {
            ptlist.push_back(ChVector<>(vx, vy, vz));
            ////parsedline = true;
        }
    }
    
    if (ptlist.size())
        this->AddConvexHull(material, ptlist, pos, rot);
    ptlist.clear();

    return true;
}

void ChCollisionModel::SetShapeMaterial(int index, std::shared_ptr<ChMaterialSurface> mat) {
    assert(index < GetNumShapes());
    assert(m_shapes[index]->m_material->GetContactMethod() == mat->GetContactMethod());
    m_shapes[index]->m_material = mat;
}

void ChCollisionModel::SetAllShapesMaterial(std::shared_ptr<ChMaterialSurface> mat) {
    assert(GetNumShapes() == 0 || m_shapes[0]->m_material->GetContactMethod() == mat->GetContactMethod());
    for (auto shape : m_shapes)
        shape->m_material = mat;
}

void ChCollisionModel::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionModel>();

    // serialize all member data:
    marchive << CHNVP(model_envelope);
    marchive << CHNVP(model_safe_margin);
}

void ChCollisionModel::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChCollisionModel>();

    // stream in all member data:
    marchive >> CHNVP(model_envelope);
    marchive >> CHNVP(model_safe_margin);
}

}  // end namespace collision
}  // end namespace chrono
