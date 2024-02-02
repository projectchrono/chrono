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
// Authors: Hammad Mazhar (OpenGL), Rainer Gericke (VSG)
// =============================================================================
//
// Chrono::VSG test program.
//
// A Random Set of Geometries in Space
// The global reference frame has Z up.
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;
using namespace chrono::vsg3d;
using namespace geometry;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

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
    rot = Q_from_AngX(CH_C_PI / 6);

    utils::AddSphereGeometry(bin.get(), mat, 0.05, ChVector<>(0, 0, 0));

    utils::AddSphereGeometry(bin.get(), mat, a, xdir * 1, rot);
    utils::AddSphereGeometry(bin.get(), mat, b, ydir * 1, rot);
    utils::AddSphereGeometry(bin.get(), mat, c, zdir * 1, rot);

    utils::AddEllipsoidGeometry(bin.get(), mat, ChVector<>(2 * a, 4 * a, 4 * a), xdir * 2, rot);
    utils::AddEllipsoidGeometry(bin.get(), mat, ChVector<>(4 * b, 2 * b, 4 * b), ydir * 2, rot);
    utils::AddEllipsoidGeometry(bin.get(), mat, ChVector<>(4 * c, 4 * c, 2 * c), zdir * 2, rot);

    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(2 * a, 4 * a, 4 * a), xdir * 3, rot);
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(4 * b, 2 * b, 4 * b), ydir * 3, rot);
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(4 * c, 4 * c, 2 * c), zdir * 3, rot);

    utils::AddCylinderGeometry(bin.get(), mat, a, 0.5, xdir * 4, rot);
    utils::AddCylinderGeometry(bin.get(), mat, b, 0.5, ydir * 4, rot);
    utils::AddCylinderGeometry(bin.get(), mat, c, 0.5, zdir * 4, rot);

    utils::AddConeGeometry(bin.get(), mat, a, 1.5, xdir * 5, rot);
    utils::AddConeGeometry(bin.get(), mat, b, 1.5, ydir * 5, rot);
    utils::AddConeGeometry(bin.get(), mat, c, 1.5, zdir * 5, rot);

    utils::AddCapsuleGeometry(bin.get(), mat, a, 1.0, xdir * 6, rot);
    utils::AddCapsuleGeometry(bin.get(), mat, b, 1.0, ydir * 6, rot);
    utils::AddCapsuleGeometry(bin.get(), mat, c, 1.0, zdir * 6, rot);

    int num_divs = 24;  // round primitive shape resolution (corresponding angle = 360/24 = 15 deg)

    auto vis = chrono_types::make_shared<ChVisualSystemVSG>(num_divs);
    vis->AttachSystem(&sys);
    vis->AddCamera(ChVector<>(10, -10, 10), ChVector<>(2, 2, 0));
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowSize(ChVector2<int>(800, 600));
    vis->SetWindowPosition(ChVector2<int>(100, 300));
    vis->SetWindowTitle("Chrono VSG Shapes");
    vis->SetUseSkyBox(true);
    vis->SetLightIntensity(0.9f);
    vis->SetLightDirection(0.5 * CH_C_PI_2, CH_C_PI_4);
    vis->SetWireFrameMode(false);
    vis->Initialize();

    while (vis->Run()) {
        vis->Render();
    }
    return 0;
}
