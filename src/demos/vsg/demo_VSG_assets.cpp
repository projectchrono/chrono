#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChParticlesClones.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/geometry/ChLineNurbs.h"
#include "chrono/geometry/ChSurfaceNurbs.h"
#include "chrono/assets/ChSurfaceShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChBoxShape.h"

#include "chrono_vsg/ChVSGChronoApp.h"

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vsg3d;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono::Engine physical system
    ChSystemNSC mphysicalSystem;
    std::string windowTitle = "See some assets";

    ChVSGChronoApp application(&mphysicalSystem, windowTitle);

    //
    // EXAMPLE 1:
    //

    // Create a ChBody, and attach some 'assets' that define 3D shapes for visualization purposes.
    // Note: these assets are independent from collision shapes!

    // Create a rigid body as usual, and add it
    // to the physical system:
    auto mfloor = chrono_types::make_shared<ChBody>();
    mfloor->SetBodyFixed(true);

    // Contact material
    auto floor_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    // Define a collision shape
    mfloor->GetCollisionModel()->ClearModel();
    mfloor->GetCollisionModel()->AddBox(floor_mat, 10, 10, 0.5, ChVector<>(0, 0, -1));
    mfloor->GetCollisionModel()->BuildModel();
    mfloor->SetCollide(true);

    // Add body to system
    application.GetSystem()->Add(mfloor);

    // ==Asset== attach a 'box' shape.
    // Note that assets are managed via shared pointer, so they
    // can also be shared). Do not forget AddAsset() at the end!
    auto mboxfloor = chrono_types::make_shared<ChBoxShape>();
    mboxfloor->GetBoxGeometry().Pos = ChVector<>(0, 0, -1);
    mboxfloor->GetBoxGeometry().Size = ChVector<>(10, 10, 0.5);
    mfloor->AddAsset(mboxfloor);

    // ==Asset== attach color asset.
    auto mfloorcolor = chrono_types::make_shared<ChColorAsset>();
    mfloorcolor->SetColor(ChColor(0.3f, 0.3f, 0.6f));
    mfloor->AddAsset(mfloorcolor);

    bool window_ok = application.OpenWindow();
    while (application.GetViewer()->advanceToNextFrame() && window_ok) {
        application.Render();
    }
}
