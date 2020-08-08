#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_vsg/ChVSGApp.h"

using namespace chrono;
using namespace geometry;
using namespace chrono::vsg3d;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2020 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC sys;

    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    auto bin = chrono_types::make_shared<ChBody>();
    utils::AddSphereGeometry(bin.get(), mat, 1, ChVector<>(0, 0, 0));
    utils::AddEllipsoidGeometry(bin.get(), mat, ChVector<>(.5, 1, 1), ChVector<>(3, 0, 0));
    utils::AddCylinderGeometry(bin.get(), mat, 1, 1, ChVector<>(3.5, 0, 0));
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(1, 1, 1), ChVector<>(6, 0, 0));
    sys.AddBody(bin);

    ChVSGApp app;
    bool ok = app.Initialize(800, 600, "VSG Viewer", &sys);
    while (app.GetViewer()->advanceToNextFrame()) {
        app.Render();
    }
    return 0;
}
