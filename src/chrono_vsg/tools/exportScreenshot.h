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

#ifndef CH_EXPORT_SCREENSHOT_H
#define CH_EXPORT_SCREENSHOT_H

#include <vsg/all.h>

#include <vsgXchange/all.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include <string>

#include "chrono_vsg/ChApiVSG.h"

namespace chrono {
namespace vsg3d {
CH_VSG_API void exportScreenshot(vsg::ref_ptr<vsg::Window> window,
                                 vsg::ref_ptr<vsg::Options> options,
                                 std::string& imageFilename);
}
}  // namespace chrono
#endif