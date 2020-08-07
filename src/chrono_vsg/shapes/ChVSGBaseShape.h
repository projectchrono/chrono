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
// Header for an abstract class defining common methods for shape node classes
// =============================================================================

#ifndef CH_VSG_BASESHAPE_H
#define CH_VSG_BASESHAPE_H

#include <iostream>
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_vsg/core/ChApiVSG.h"

#include <vsg/all.h>

namespace chrono {
namespace vsg3d {

class CH_VSG_API ChVSGBaseShape {
  public:
    ChVSGBaseShape();
    virtual vsg::ref_ptr<vsg::Node> createTexturedNode(vsg::vec4 color,
                                                       vsg::ref_ptr<vsg::MatrixTransform> transform) = 0;
    void compile(vsg::ref_ptr<vsg::Node> subgraph);
    vsg::ref_ptr<vsg::ShaderStage> readVertexShader(std::string filePath);
    vsg::ref_ptr<vsg::ShaderStage> readFragmentShader(std::string filePath);
    vsg::ref_ptr<vsg::vec4Array2D> createRGBATexture(std::string filePath);

  private:
    vsg::ref_ptr<vsg::CompileTraversal> _compile;
};
}  // namespace vsg3d
}  // namespace chrono
#endif
