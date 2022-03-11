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

#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;
using namespace chrono::vsg3d;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2022 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC sys;
    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    auto bin = chrono_types::make_shared<ChBody>();
    sys.AddBody(bin);

    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->SetWindowSize(ChVector2<int>(1000, 800));
    vis->SetWindowTitle("VSG Test Lab");
    vis->SetUseSkyBox(true);
    vis->Initialize();
    sys.SetVisualSystem(vis);

    while(vis->Run()) {
        vis->Render();
    }
    return 0;
}
