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

#ifndef CH_VSG_SHAPE_H
#define CH_VSG_SHAPE_H

#include <iostream>
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_vsg/core/ChApiVSG.h"

#include <vsg/all.h>

namespace chrono {
namespace vsg3d {

class CH_VSG_API ChVSGShape {
  public:
    ChVSGShape();
    vsg::ref_ptr<vsg::ShaderStage> readVertexShader(std::string filePath);
    vsg::ref_ptr<vsg::ShaderStage> readFragmentShader(std::string filePath);
    vsg::ref_ptr<vsg::vec4Array2D> createRGBATexture(std::string filePath);

    virtual vsg::ref_ptr<vsg::Node> createTexturedNode(vsg::vec4 color,
                                                       std::string texFilePath,
                                                       vsg::ref_ptr<vsg::MatrixTransform> transform) = 0;

    virtual vsg::ref_ptr<vsg::Node> createPhongNode(vsg::vec3& lightPosition,
                                                    vsg::vec4 ambientColor,
                                                    vsg::vec4 diffuseColor,
                                                    vsg::vec4 specularColor,
                                                    float shininess,
                                                    float opacity,
                                                    vsg::ref_ptr<vsg::MatrixTransform> transform) = 0;
};
}  // namespace vsg3d
}  // namespace chrono
#endif
