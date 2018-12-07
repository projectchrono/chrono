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
#ifndef CHGDBODY_H
#define CHGDBODY_H

// #include <iostream>
// #include <memory>

// godot includes
//#include <core/math/quat.h>
#include <scene/3d/mesh_instance.h>
//#include <scene/3d/spatial.h>

// chrono includes
#include "chrono/assets/ChAsset.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCapsuleShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChConeShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChEllipsoidShape.h"
#include "chrono/assets/ChLineShape.h"
#include "chrono/assets/ChPathShape.h"
#include "chrono/assets/ChRoundedBoxShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChVisualization.h"
#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBody.h"

// ChGodotIncludes
#include "chrono_godot/ChApiGodot.h"
#include "chrono_godot/godot_utils/ChGdAssetManager.h"
#include "chrono_godot/godot_utils/ChGdUtils.h"

namespace chrono {
namespace gd {

class CH_GODOT_API ChGdBody {
  public:
    ChGdBody(std::shared_ptr<ChGdAssetManager> assetManager);
    ~ChGdBody();

    void CreateFromChrono(Spatial* parent_node, std::shared_ptr<ChBody> body);
    void UpdatePose();

  private:
    struct geometry_pair {
        MeshInstance* godot_mesh;
        ChVisualization* ch_geo;
    };
    std::shared_ptr<ChBody> m_bodyRef;
    std::shared_ptr<ChGdAssetManager> m_assetManager;

    Spatial* m_bodyNode;
    std::vector<Object*> m_resourceList;
    std::vector<geometry_pair> m_geometries;
};

}  // namespace gd
}  // end namespace chrono

#endif
