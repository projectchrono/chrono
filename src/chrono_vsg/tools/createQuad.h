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

#ifndef CREATE_QUAD_H
#define CREATE_QUAD_H

#include <iostream>
#include <vsg/all.h>

#include <vsgXchange/all.h>

namespace chrono {
namespace vsg3d {
vsg::ref_ptr<vsg::Node> createQuad(const vsg::vec3& origin,
                                   const vsg::vec3& horizontal,
                                   const vsg::vec3& vertical,
                                   vsg::ref_ptr<vsg::Data> sourceData = {});
}
}  // namespace chrono
#endif
