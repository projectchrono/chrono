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
// Radu Serban, Rainer Gericke
// =============================================================================

#ifndef CH_SHAPE_BUILDER_H
#define CH_SHAPE_BUILDER_H

#include <iostream>
#include <string>
#include "chrono_vsg/core/ChApiVSG.h"

#include <vsg/all.h>
#include <vsgXchange/all.h>

#include "chrono/assets/ChVisualSystem.h"

namespace chrono {
namespace vsg3d {
class CH_VSG_API ShapeBuilder {
  public:
    ShapeBuilder(vsg::ref_ptr<vsg::Options> options);
    ~ShapeBuilder();

    vsg::ref_ptr<vsg::Group> createBox(std::shared_ptr<ChVisualMaterial> material, vsg::dmat4 &tf_matrix);

  private:
    vsg::ref_ptr<vsg::Options> m_options;
};
}  // namespace vsg3d
}  // namespace chrono

#endif
