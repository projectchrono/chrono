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
//
// Chrono::vsg3d test program.
//
// Template for the new chrono graphics representation
//
// The global reference frame has Z up.
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_vsg/VSGApp.h"

using namespace chrono;
using namespace chrono::vsg3d;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2021 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC sys;
    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    auto bin = chrono_types::make_shared<ChBody>();
    sys.AddBody(bin);

    double a = 0.5;
    double b = 0.25;
    double c = 0.1;
    ChVector<> xdir(1.5, 0.0, 0.0);
    ChVector<> ydir(0.0, 1.5, 0.0);
    ChVector<> zdir(0.0, 0.0, 1.5);
    ChQuaternion<> rot(1, 0, 0, 0);
    ////rot = Q_from_AngX(CH_C_PI / 6);

    utils::AddSphereGeometry(bin.get(), mat, 0.05, ChVector<>(0, 0, 0));

    utils::AddSphereGeometry(bin.get(), mat, a, xdir * 1, rot);
    utils::AddSphereGeometry(bin.get(), mat, b, ydir * 1, rot);
    utils::AddSphereGeometry(bin.get(), mat, c, zdir * 1, rot);

    utils::AddEllipsoidGeometry(bin.get(), mat, ChVector<>(a, 2 * a, 2 * a), xdir * 2, rot);
    utils::AddEllipsoidGeometry(bin.get(), mat, ChVector<>(2 * b, b, 2 * b), ydir * 2, rot);
    utils::AddEllipsoidGeometry(bin.get(), mat, ChVector<>(2 * c, 2 * c, c), zdir * 2, rot);

    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(a, 2 * a, 2 * a), xdir * 3, rot);
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(2 * b, b, 2 * b), ydir * 3, rot);
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(2 * c, 2 * c, c), zdir * 3, rot);

    utils::AddCylinderGeometry(bin.get(), mat, a, 0.5, xdir * 4, rot);
    utils::AddCylinderGeometry(bin.get(), mat, b, 0.5, ydir * 4, rot);
    utils::AddCylinderGeometry(bin.get(), mat, c, 0.5, zdir * 4, rot);

    utils::AddConeGeometry(bin.get(), mat, a, 1.5, xdir * 5, rot);
    utils::AddConeGeometry(bin.get(), mat, b, 1.5, ydir * 5, rot);
    utils::AddConeGeometry(bin.get(), mat, c, 1.5, zdir * 5, rot);

    utils::AddCapsuleGeometry(bin.get(), mat, a, 0.5, xdir * 6, rot);
    utils::AddCapsuleGeometry(bin.get(), mat, b, 0.5, ydir * 6, rot);
    utils::AddCapsuleGeometry(bin.get(), mat, c, 0.5, zdir * 6, rot);

    // Render everything
    VSGApp app;

    app.Initialize(1280, 720, "VSG Lab", &sys);


    while (app.GetViewer()->advanceToNextFrame()) {
        app.Render();
    }

    return 0;
}
