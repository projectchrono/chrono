//
// Basic PBR parameter demo adopted from https://learnopengl.com/
//
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono_vsg/assets/ChPBRSetting.h"

#include "chrono_vsg/ChVSGApp.h"

using namespace chrono;
using namespace geometry;
using namespace chrono::vsg3d;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2020 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC sys;

    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    int nrRows = 7;
    int nrColumns = 7;
    float spacing = 2.5;

    ChVSGApp app;
    app.setClearColor(0.5, 0.5, 0.5);

    ChColor albedo(1.0, 0.0, 0.0, 1.0);
    float ao = 1.0;
    for (int row = 0; row < nrRows; ++row) {
        float metallic = (float)row / (float)nrRows;
        for (int col = 0; col < nrColumns; ++col) {
            float roughness = ChClamp((float)col / (float)nrColumns, 0.05f, 1.0f);
            float posx = spacing * (col - (nrColumns / 2));
            float posy = spacing * (row - (nrRows / 2));
            float posz = 0.0;
            auto sphere = chrono_types::make_shared<ChBody>();
            utils::AddSphereGeometry(sphere.get(), mat, 1, ChVector<>(posx, posy, posz));
            sphere->AddAsset(chrono_types::make_shared<ChPBRSetting>(albedo, metallic, roughness, ao));
            sys.AddBody(sphere);
        }
    }

    bool ok = app.Initialize(800, 600, "VSG PBR Parameter Demo", &sys);
    while (app.GetViewer()->advanceToNextFrame()) {
        app.Render();
    }
    return 0;
}
