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
// Authors: Asher Elmquist
// =============================================================================
//
// Class to manage adding, changing, and removing godot scene nodes
//
// =============================================================================

#include <vector>
//#include <scene/resources/material.h>
//#include <scene/resources/mesh.h>
//#include <scene/resources/primitive_meshes.h>

#include "chrono_godot/godot_utils/ChGdAssetManager.h"

namespace chrono {
namespace gd {

ChGdAssetManager::ChGdAssetManager() {}
ChGdAssetManager::~ChGdAssetManager() {}

SphereMesh* ChGdAssetManager::GetSphereMesh() {
    if (m_sphere_exists) {
        return m_sphereMesh;
    }
    m_sphereMesh = memnew(SphereMesh);
    m_sphereMesh->set_radius(1.0);
    m_sphereMesh->set_height(2.0);
    m_sphere_exists = true;
    return m_sphereMesh;
}

CubeMesh* ChGdAssetManager::GetCubeMesh() {
    if (m_cube_exists) {
        return m_cubeMesh;
    }
    m_cubeMesh = memnew(CubeMesh);
    m_cubeMesh->set_size({1, 1, 1});
    m_cube_exists = true;
    return m_cubeMesh;
}

CylinderMesh* ChGdAssetManager::GetCylinderMesh() {
    if (m_cylinder_exists) {
        return m_cylinderMesh;
    }
    m_cylinderMesh = memnew(CylinderMesh);
    m_cylinderMesh->set_height(1.0);
    m_cylinderMesh->set_bottom_radius(1.0);
    m_cylinderMesh->set_top_radius(1.0);
    m_cylinder_exists = true;
    return m_cylinderMesh;
}

SpatialMaterial* ChGdAssetManager::GetColorMaterial(Color color) {
    SpatialMaterial* material = memnew(SpatialMaterial);
    m_colorMaterials.push_back(material);
    material->set_flag(SpatialMaterial::FLAG_SRGB_VERTEX_COLOR, true);
    material->set_flag(SpatialMaterial::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
    //    material->set_metallic(1.0);
    //    material->set_refraction(0.5);

    material->set_albedo(color);
    material->set_script_instance(nullptr);
    SpatialMaterial::flush_changes();
    return material;
}

}  // namespace gd
}  // end namespace chrono
