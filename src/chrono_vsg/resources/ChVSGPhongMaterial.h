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
// Header for an helper class defining materials for Phong shading
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

class CH_VSG_API ChVSGPhongMaterial {
  public:
    typedef enum {
        VSG_PHONG_MAT_EMERALD,
        VSG_PHONG_MAT_JADE,
        VSG_PHONG_MAT_OBSIDIAN,
        VSG_PHONG_MAT_PEARL,
        VSG_PHONG_MAT_RUBY,
        VSG_PHONG_MAT_TURQUOISE,
        VSG_PHONG_MAT_BRASS,
        VSG_PHONG_MAT_BRONZE,
        VSG_PHONG_MAT_POLISHED_BRONZE,
        VSG_PHONG_MAT_CHROME,
        VSG_PHONG_MAT_COPPER,
        VSG_PHONG_MAT_POLISHED_COPPER,
        VSG_PHONG_MAT_GOLD,
        VSG_PHONG_MAT_POLISHED_GOLD,
        VSG_PHONG_MAT_PEWTER,
        VSG_PHONG_MAT_SILVER,
        VSG_PHONG_MAT_POLISHED_SILVER,
        VSG_PHONG_MAT_BLACK_PLASTIC,
        VSG_PHONG_MAT_CYAN_PLASTIC,
        VSG_PHONG_MAT_GREEN_PLASTIC,
        VSG_PHONG_MAT_RED_PLASTIC,
        VSG_PHONG_MAT_WHITE_PLASTIC,
        VSG_PHONG_MAT_YELLOW_PLASTIC,
        VSG_PHONG_MAT_BLUE_PLASTIC,
        VSG_PHONG_MAT_BLACK_RUBBER,
        VSG_PHONG_MAT_CYAN_RUBBER,
        VSG_PHONG_MAT_GREEN_RUBBER,
        VSG_PHONG_MAT_RED_RUBBER,
        VSG_PHONG_MAT_WHITE_RUBBER,
        VSG_PHONG_MAT_YELLOW_RUBBER,
        VSG_PHONG_MAT_BLUE_RUBBER
    } VSG_PHONG_MATERIAL;

  public:
    ChVSGPhongMaterial(VSG_PHONG_MATERIAL mat = VSG_PHONG_MAT_EMERALD);

  private:
    vsg::vec3 m_ambientColor;
    vsg::vec3 m_diffuseColor;
    vsg::vec3 m_specularColor;
    float m_shininess;
    float m_opacity;
};
}  // namespace vsg3d
}  // namespace chrono
#endif
