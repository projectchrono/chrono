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
// Authors: Hammad Mazhar
// =============================================================================
//
// Chrono::OpenGL test program.
//
// A Random Set of Geometries in Space
// The global reference frame has Z up.
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_opengl/ChOpenGLWindow.h"

using namespace chrono;
using namespace geometry;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC sys;

    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    auto bin = chrono_types::make_shared<ChBody>();
    utils::AddSphereGeometry(bin.get(), mat, 1, ChVector<>(0, 0, 0));
    utils::AddEllipsoidGeometry(bin.get(), mat, ChVector<>(.5, 1, 1), ChVector<>(3, 0, 0));
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(1, 1, 1), ChVector<>(6, 0, 0));
    utils::AddCylinderGeometry(bin.get(), mat, 1, 1, ChVector<>(9, 0, 0));
    utils::AddConeGeometry(bin.get(), mat, 1, 3, ChVector<>(12, 0, 0));
    sys.AddBody(bin);

    // Render everything
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "OpenGL Shapes", &sys);
    gl_window.SetCamera(ChVector<>(6, -10, 0), ChVector<>(6, 0, 0), ChVector<>(0, 0, 1));
    gl_window.SetRenderMode(opengl::WIREFRAME);

    while (gl_window.Active()) {
        gl_window.Render();
    }

    return 0;
}
