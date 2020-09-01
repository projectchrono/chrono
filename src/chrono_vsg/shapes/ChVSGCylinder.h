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
// Cylinder shape
// =============================================================================

#ifndef CH_VSG_CYLINDER_H
#define CH_VSG_CYLINDER_H

#include <iostream>
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_vsg/core/ChApiVSG.h"
#include "chrono_vsg/shapes/ChVSGShape.h"

#include <vsg/all.h>

namespace chrono {
namespace vsg3d {

class CH_VSG_API ChVSGCylinder : public ChVSGShape {
  public:
    ChVSGCylinder();

    virtual vsg::ref_ptr<vsg::Node> createTexturedNode(vsg::vec4 color,
                                                       std::string texFilePath,
                                                       vsg::ref_ptr<vsg::MatrixTransform> transform) override;

    virtual vsg::ref_ptr<vsg::Node> createPhongNode(vsg::vec4 ambientColor,
                                                    vsg::vec4 duffuseColor,
                                                    vsg::vec4 specularColor,
                                                    float shininess,
                                                    float opacity,
                                                    vsg::ref_ptr<vsg::MatrixTransform> transform) override;
};
}  // namespace vsg3d
}  // namespace chrono
#endif
