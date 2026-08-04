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

#include <iostream>
#include <mutex>
#include <set>
#include <typeinfo>

#if defined(__GNUC__) && __has_include(<cxxabi.h>)
    #include <cstdlib>
    #include <cxxabi.h>
    #define CH_HAVE_CXA_DEMANGLE
#endif

#include "chrono/assets/ChVisualShape.h"
#include "chrono/physics/ChPhysicsItem.h"

namespace chrono {

ChVisualShape::ChVisualShape() : is_visible(true), is_mutable(false), is_double_faced(false) {}

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

void ChVisualShape::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChVisualShape>();
    // serialize all member data:
    archive_out << CHNVP(is_visible);
    archive_out << CHNVP(is_mutable);
    archive_out << CHNVP(is_double_faced);
    archive_out << CHNVP(material_list);
}

void ChVisualShape::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChVisualShape>();
    // stream in all member data:
    archive_in >> CHNVP(is_visible);
    archive_in >> CHNVP(is_mutable);
    archive_in >> CHNVP(is_double_faced);
    archive_in >> CHNVP(material_list);
}

// Return a readable name for a type. MSVC already yields one; the Itanium ABI yields a mangled name, so demangle it.
static std::string TypeName(const std::type_info& ti) {
#ifdef CH_HAVE_CXA_DEMANGLE
    int status = 0;
    char* demangled = abi::__cxa_demangle(ti.name(), nullptr, nullptr, &status);
    if (status == 0 && demangled) {
        std::string name(demangled);
        std::free(demangled);
        return name;
    }
    std::free(demangled);
#endif
    return ti.name();
}

void ReportUnsupportedVisualShape(const ChVisualShape& shape, const std::string& backend) {
    // Visual models can be populated from more than one thread, so guard the record of what has been reported.
    static std::mutex mutex;
    static std::set<std::string> reported;

    std::string type_name = TypeName(typeid(shape));

    std::lock_guard<std::mutex> lock(mutex);
    if (!reported.insert(backend + "/" + type_name).second)
        return;

    std::cerr << "Warning: " << backend << " cannot render a visual shape of type '" << type_name
              << "'; the shape will not be drawn. Further occurrences of this shape type are not reported."
              << std::endl;
}

}  // namespace chrono
