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
// Rainer Gericke, code taken from https://github.com/vsg-dev/vsgExamples.git
// =============================================================================

#ifndef CREATE_SKYBOX_H
#define CREATE_SKYBOX_H

#include <vsg/all.h>
#include <vsgXchange/all.h>

namespace chrono {
    namespace vsg3d {
        vsg::ref_ptr<vsg::Node> createSkybox(const vsg::Path& filename, vsg::ref_ptr<vsg::Options> options);
    }
}

#endif
