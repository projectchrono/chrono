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

#ifndef CH_VSG_SCREENSHOT_HANDLER_H
#define CH_VSG_SCREENSHOT_HANDLER_H

#include <iostream>
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_vsg/core/ChApiVSG.h"
#include "chrono_vsg/resources/ChVSGSettings.h"

#include <vsg/all.h>

namespace chrono {
namespace vsg3d {

class ChVSGScreenshotHandler : public vsg::Inherit<vsg::Visitor, ChVSGScreenshotHandler> {
  public:
    bool do_image_capture = false;
    bool do_depth_capture = false;
    vsg::ref_ptr<vsg::Event> event;
    bool eventDebugTest = false;

    ChVSGScreenshotHandler(vsg::ref_ptr<vsg::Event> in_event);

    void apply(vsg::KeyPressEvent& keyPress) override {
        #ifdef _WIN32
        if (keyPress.keyBase == 80) {
            do_image_capture = true;
        }
        if (keyPress.keyBase == 84) {
            do_depth_capture = true;
        }
        #else
        if (keyPress.keyBase == 'p') {
            do_image_capture = true;
        }
        if (keyPress.keyBase == 't') {
            do_depth_capture = true;
        }
#endif
    }

    void printInfo(vsg::ref_ptr<vsg::Window> window);
    void screenshot_image(vsg::ref_ptr<vsg::Window> window);
    void screenshot_depth(vsg::ref_ptr<vsg::Window> window);
};
}  // namespace vsg3d
}  // namespace chrono
#endif
