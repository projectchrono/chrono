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
// Header for a class defining a sphere shape for the vulkan scene graph
// =============================================================================

#ifndef CH_VSG_SPHERE_H
#define CH_VSG_SPHERE_H

#include <iostream>
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_vsg/core/ChApiVSG.h"

#include <vsg/all.h>

namespace chrono {
namespace vsg3d {

class CH_VSG_API ChVSGSphere {
  public:
    ChVSGSphere();
    vsg::ref_ptr<vsg::Node> createTexturedNode(vsg::vec4 color, vsg::ref_ptr<vsg::MatrixTransform> transform);
    void compile(vsg::ref_ptr<vsg::Node> subgraph);

  private:
    vsg::ref_ptr<vsg::CompileTraversal> _compile;
};
}  // namespace vsg3d
}  // namespace chrono
#endif
