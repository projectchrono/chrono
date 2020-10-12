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
// Authors: Rainer Gericke
// =============================================================================
// Header for an helper class defining common methods for shape node classes
// =============================================================================

#ifndef VSG_SIMPLE_BOX_H
#define VSG_SIMPLE_BOX_H

#include <iostream>
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_vsg/core/ChApiVSG.h"
#include "chrono_vsg/shapes/ChVSGSimplePhongIdxMesh.h"

#include <vsg/all.h>

namespace chrono {
namespace vsg3d {

class CH_VSG_API VSGSimpleBox : public ChVSGSimplePhongIdxMesh {
  public:
    VSGSimpleBox(std::shared_ptr<ChBody> body,
                 std::shared_ptr<ChAsset> asset,
                 vsg::ref_ptr<vsg::MatrixTransform> transform);
    virtual void Initialize(vsg::vec3& lightPosition, vsg::vec3& objectColor) override;
};
}  // namespace vsg3d
}  // namespace chrono
#endif
