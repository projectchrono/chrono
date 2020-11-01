#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono_vsg/assets/ChTexturedPBR.h"

#include "chrono_vsg/ChVSGApp.h"

using namespace chrono;
using namespace geometry;
using namespace chrono::vsg3d;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2020 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC sys;

    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    /*
        auto bin = chrono_types::make_shared<ChBody>();
        // bin->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("blu.png")));
        bin->AddAsset(chrono_types::make_shared<ChColorAsset>(1.0, 0.0, 0.0, 1.0));
        utils::AddSphereGeometry(bin.get(), mat, 1, ChVector<>(0, 0, 0));
        utils::AddEllipsoidGeometry(bin.get(), mat, ChVector<>(.5, 1, 1), ChVector<>(3, 0, 0));
        utils::AddCylinderGeometry(bin.get(), mat, 1, 1, ChVector<>(6.0, 0, 0));
        utils::AddBoxGeometry(bin.get(), mat, ChVector<>(1, 1, 1), ChVector<>(9.0, 0, 0));
        sys.AddBody(bin);
    */

    auto box1 = chrono_types::make_shared<ChBody>();
    utils::AddBoxGeometry(box1.get(), mat, ChVector<>(1, 1, 1), ChVector<>(0.0, 2.0, 0));
    box1->AddAsset(chrono_types::make_shared<ChColorAsset>(1.0, 0.0, 0.0, 1.0));
    sys.AddBody(box1);

    auto box2 = chrono_types::make_shared<ChBody>();
    utils::AddBoxGeometry(box2.get(), mat, ChVector<>(1, 1, 1), ChVector<>(0.0, -2.0, 0));
    box2->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("vsg/textures/Wood034.jpg")));
    sys.AddBody(box2);

    auto box3 = chrono_types::make_shared<ChBody>();
    utils::AddBoxGeometry(box3.get(), mat, ChVector<>(1, 1, 1), ChVector<>(0.0, 6.0, 0));
    box3->AddAsset(chrono_types::make_shared<ChTexturedPBR>(GetChronoDataFile("vsg/textures/pbr/plastic1")));
    sys.AddBody(box3);

    auto cyl1 = chrono_types::make_shared<ChBody>();
    utils::AddCylinderGeometry(cyl1.get(), mat, 1, 1, ChVector<>(3.0, 2.0, 0));
    cyl1->AddAsset(chrono_types::make_shared<ChColorAsset>(1.0, 0.5, 0.0, 1.0));
    sys.AddBody(cyl1);

    auto cyl2 = chrono_types::make_shared<ChBody>();
    utils::AddCylinderGeometry(cyl2.get(), mat, 1, 1, ChVector<>(3.0, -2.0, 0));
    cyl2->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("vsg/textures/Marble010.jpg")));
    sys.AddBody(cyl2);

    auto cyl3 = chrono_types::make_shared<ChBody>();
    utils::AddCylinderGeometry(cyl3.get(), mat, 1, 1, ChVector<>(3.0, 6.0, 0));
    cyl3->AddAsset(chrono_types::make_shared<ChTexturedPBR>(GetChronoDataFile("vsg/textures/pbr/plastic2")));
    sys.AddBody(cyl3);

    auto ellipsoid1 = chrono_types::make_shared<ChBody>();
    utils::AddEllipsoidGeometry(ellipsoid1.get(), mat, ChVector<>(.5, 1, 1), ChVector<>(6, 2, 0));
    ellipsoid1->AddAsset(chrono_types::make_shared<ChColorAsset>(1.0, 0.5, 0.5, 1.0));
    sys.AddBody(ellipsoid1);

    auto ellipsoid2 = chrono_types::make_shared<ChBody>();
    utils::AddEllipsoidGeometry(ellipsoid2.get(), mat, ChVector<>(.5, 1, 1), ChVector<>(6, -2, 0));
    ellipsoid2->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("vsg/textures/Cork001.jpg")));
    sys.AddBody(ellipsoid2);

    auto ellipsoid3 = chrono_types::make_shared<ChBody>();
    utils::AddEllipsoidGeometry(ellipsoid3.get(), mat, ChVector<>(.5, 1, 1), ChVector<>(6, 6, 0));
    ellipsoid3->AddAsset(chrono_types::make_shared<ChTexturedPBR>(GetChronoDataFile("vsg/textures/pbr/plastic3")));
    sys.AddBody(ellipsoid3);

    auto sphere1 = chrono_types::make_shared<ChBody>();
    utils::AddSphereGeometry(sphere1.get(), mat, 1, ChVector<>(9, 2, 0));
    sphere1->AddAsset(chrono_types::make_shared<ChColorAsset>(0.5, 1.0, 0.5, 1.0));
    sys.AddBody(sphere1);

    auto sphere2 = chrono_types::make_shared<ChBody>();
    utils::AddSphereGeometry(sphere2.get(), mat, 1, ChVector<>(9, -2, 0));
    sphere2->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("vsg/textures/Metal005.jpg")));
    sys.AddBody(sphere2);

    auto sphere3 = chrono_types::make_shared<ChBody>();
    utils::AddSphereGeometry(sphere3.get(), mat, 1, ChVector<>(9, 6, 0));
    sphere3->AddAsset(chrono_types::make_shared<ChTexturedPBR>(GetChronoDataFile("vsg/textures/pbr/plastic4")));
    sys.AddBody(sphere3);

    ChVSGApp app;

    bool ok = app.Initialize(800, 600, "VSG Viewer", &sys);
    while (app.GetViewer()->advanceToNextFrame()) {
        app.Render();
    }
    return 0;
}
