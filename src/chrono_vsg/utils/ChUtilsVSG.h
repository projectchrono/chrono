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

#ifndef CH_UTILS_VSG_H
#define CH_UTILS_VSG_H

#include <string>

#include <vsg/all.h>
#include <vsgXchange/all.h>

#include "chrono_vsg/ChApiVSG.h"

namespace chrono {
namespace vsg3d {

CH_VSG_API vsg::ref_ptr<vsg::Node> createQuad(const vsg::vec3& origin,
                                              const vsg::vec3& horizontal,
                                              const vsg::vec3& vertical,
                                              vsg::ref_ptr<vsg::Data> sourceData = {});

CH_VSG_API vsg::ref_ptr<vsg::Node> createSkybox(const vsg::Path& filename,
                                                vsg::ref_ptr<vsg::Options> options,
                                                bool yup);

}  // namespace vsg3d
}  // namespace chrono

#endif
