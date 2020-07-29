// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
// Vulkan Scene Graph viewer, this class will hopefully draw the system to the
// screen and handle input some day
// =============================================================================

#ifndef CH_VSG_APP_H
#define CH_VSG_APP_H

#include <iostream>
#include "chrono_vsg/core/ChApiVSG.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/core/ChTimer.h"

#include <vsg/all.h>

namespace chrono {
namespace vsg {

/// @addtogroup vsg_module
/// @{

class CH_VSG_API ChVSGApp {
  public:
    ChVSGApp(ChSystem* system);
    ~ChVSGApp();
    ::vsg::ref_ptr<::vsg::Viewer> GetViewer() { return m_viewer; }

  private:
    ChSystem* m_system;

    ::vsg::ref_ptr<::vsg::WindowTraits> m_windowTraits;

    ::vsg::Paths m_searchPaths;

    std::vector<::vsg::ref_ptr<::vsg::Node>> m_nodes;

    ::vsg::ref_ptr<::vsg::Node> m_scene;

    ::vsg::ref_ptr<::vsg::Viewer> m_viewer;

    ::vsg::ref_ptr<::vsg::Window> m_window;
};

}  // namespace vsg
}  // namespace chrono
#endif
