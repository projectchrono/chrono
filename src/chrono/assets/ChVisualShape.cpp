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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/assets/ChVisualShape.h"

namespace chrono {

ChVisualShape::ChVisualShape() : visible(true), is_mutable(true) {}

int ChVisualShape::AddMaterial(std::shared_ptr<ChVisualMaterial> material) {
    material_list.push_back(material);
    return (int)material_list.size();
}

void ChVisualShape::SetMaterial(int i, std::shared_ptr<ChVisualMaterial> material) {
    if (i == 0 && material_list.empty()) {
        material_list.push_back(material);
        return;
    }

    if (i < material_list.size()) {
        material_list[i] = material;
    }
}

void ChVisualShape::SetColor(const ChColor& col) {
    // Ensure that material_list[0] is a new material
    if (material_list.empty())
        material_list.push_back(std::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default()));
    else if (material_list[0] == ChVisualMaterial::Default())
        material_list[0] = std::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default());

    material_list[0]->SetDiffuseColor(ChVector<float>(col.R, col.G, col.B));
    material_list[0]->SetOpacity(col.A);
}

ChColor ChVisualShape::GetColor() const {
    ChVector<float> RGB;
    float A;
    if (material_list.empty()) {
        RGB = ChVisualMaterial::Default()->GetDiffuseColor();
        A = ChVisualMaterial::Default()->GetOpacity();
    } else {
        RGB = material_list[0]->GetDiffuseColor();
        A = material_list[0]->GetOpacity();
    }

    return ChColor(RGB[0], RGB[1], RGB[2], A);
}

void ChVisualShape::SetOpacity(float val) {
    // Ensure that material_list[0] is a new material
    if (material_list.empty())
        material_list.push_back(std::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default()));
    else if (material_list[0] == ChVisualMaterial::Default())
        material_list[0] = std::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default());

    material_list[0]->SetOpacity(val);
}

float ChVisualShape::GetOpacity() const {
    float A;
    if (material_list.empty()) {
        A = ChVisualMaterial::Default()->GetOpacity();
    } else {
        A = material_list[0]->GetOpacity();
    }

    return A;
}

void ChVisualShape::SetTexture(const std::string& filename, ChVector2<float> scale) {
    // Ensure that material_list[0] is a new material
    if (material_list.empty())
        material_list.push_back(std::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default()));
    else if (material_list[0] == ChVisualMaterial::Default())
        material_list[0] = std::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default());

    material_list[0]->SetKdTexture(filename, scale);
}

std::string ChVisualShape::GetTexture() const {
    if (material_list.empty())
        return "";
    return material_list[0]->GetKdTexture();
}

void ChVisualShape::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChVisualShape>();
    // serialize all member data:
    marchive << CHNVP(visible);
}

void ChVisualShape::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChVisualShape>();
    // stream in all member data:
    marchive >> CHNVP(visible);
}

}  // namespace chrono
