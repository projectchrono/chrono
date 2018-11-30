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
#ifndef CHGDASSETMANAGER_H
#define CHGDASSETMANAGER_H

//#include <vector>

// godot includes
#include <scene/resources/material.h>
#include <scene/resources/mesh.h>
#include <scene/resources/primitive_meshes.h>

// chrono includes

// ChGodotIncludes

namespace chrono {
namespace gd {

class ChGdAssetManager {
  public:
    ChGdAssetManager();
    ~ChGdAssetManager();

    SphereMesh* GetSphereMesh();
    CubeMesh* GetCubeMesh();
    CylinderMesh* GetCylinderMesh();
    SpatialMaterial* GetColorMaterial(Color color);

  private:
    SphereMesh* m_sphereMesh;
    CubeMesh* m_cubeMesh;
    CylinderMesh* m_cylinderMesh;

    std::vector<SpatialMaterial*> m_colorMaterials;

    bool m_sphere_exists = false;
    bool m_cube_exists = false;
    bool m_cylinder_exists = false;
};

}  // namespace gd
}  // end namespace chrono

#endif
