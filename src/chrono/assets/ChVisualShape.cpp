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
#include "chrono/physics/ChPhysicsItem.h"

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

    material_list[0]->SetDiffuseColor(col);
}

ChColor ChVisualShape::GetColor() const {
    ChColor RGB;
    if (material_list.empty()) {
        RGB = ChVisualMaterial::Default()->GetDiffuseColor();
    } else {
        RGB = material_list[0]->GetDiffuseColor();
    }

    return RGB;
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

void ChVisualShape::SetTexture(const std::string& filename, float scale_x, float scale_y) {
    // Ensure that material_list[0] is a new material
    if (material_list.empty())
        material_list.push_back(std::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default()));
    else if (material_list[0] == ChVisualMaterial::Default())
        material_list[0] = std::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default());

    material_list[0]->SetKdTexture(filename);
    material_list[0]->SetTextureScale(scale_x, scale_y);
}

std::string ChVisualShape::GetTexture() const {
    if (material_list.empty())
        return "";
    return material_list[0]->GetKdTexture();
}

void ChVisualShape::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChVisualShape>();
    // serialize all member data:
    marchive << CHNVP(visible);
    marchive << CHNVP(is_mutable);
    marchive << CHNVP(material_list);
}

void ChVisualShape::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChVisualShape>();
    // stream in all member data:
    marchive >> CHNVP(visible);
    marchive >> CHNVP(is_mutable);
    marchive >> CHNVP(material_list);

}

}  // namespace chrono
