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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/assets/ChVisualShape.h"

namespace chrono {

ChVisualShape::ChVisualShape() : Pos(0), Rot(1), visible(true), is_static(false), fading(0) {
    default_mat = chrono_types::make_shared<ChVisualMaterial>();
    default_mat->SetDiffuseColor(ChVector<float>(1, 1, 1));
}

int ChVisualShape::AddMaterial(std::shared_ptr<ChVisualMaterial> material) {
    material_list.push_back(material);
    return (int)material_list.size();
}

void ChVisualShape::SetColor(const ChColor& col) {
    default_mat->SetDiffuseColor(ChVector<float>(col.R, col.G, col.B));
    default_mat->SetTransparency(col.A);
}

ChColor ChVisualShape::GetColor() const {
    auto RGB = default_mat->GetDiffuseColor();
    auto A = default_mat->GetTransparency();
    return ChColor(RGB[0], RGB[1], RGB[2], A);
}

void ChVisualShape::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChVisualShape>();
    // serialize parent class
    ChAsset::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(Pos);
    ////marchive << CHNVP(Rot);
    marchive << CHNVP(visible);
    marchive << CHNVP(fading);
}

void ChVisualShape::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChVisualShape>();
    // deserialize parent class
    ChAsset::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(Pos);
    ////marchive >> CHNVP(Rot);
    marchive >> CHNVP(visible);
    marchive >> CHNVP(fading);
}

}  // namespace chrono
