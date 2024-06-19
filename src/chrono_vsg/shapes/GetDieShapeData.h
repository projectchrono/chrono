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
// Rainer Gericke
// =============================================================================

#ifndef CH_DIE_SHAPE_DATA_H
#define CH_DIE_SHAPE_DATA_H

#include <iostream>
#include <string>
#include "chrono_vsg/ChApiVSG.h"

#include <vsg/all.h>
#include <vsgXchange/all.h>

#include "chrono/assets/ChVisualSystem.h"
#include "chrono/assets/ChVisualModel.h"

namespace chrono {
namespace vsg3d {

void GetDieShapeData(vsg::ref_ptr<vsg::vec3Array>& vertices,
                     vsg::ref_ptr<vsg::vec3Array>& normals,
                     vsg::ref_ptr<vsg::vec2Array>& texcoords,
                     vsg::ref_ptr<vsg::ushortArray>& indices);

}  // namespace vsg3d
}  // namespace chrono

#endif
