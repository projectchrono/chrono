#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkTrajectory.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_vehicle/tracked_vehicle/utils/ChSprocketProfiles.h"

using namespace chrono;
using namespace geometry;

using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

int main(int argc, char* argv[]) {
    ChSystem mphysicalSystem;

    ChIrrApp application(&mphysicalSystem, L"Paths", core::dimension2d<u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 0, -1));

    // Show contact forces in Irrlicht application
    application.SetSymbolscale(0.2);
    application.SetContactsDrawMode(ChIrrTools::eCh_ContactsDrawMode::CONTACT_NORMALS);


    // Create the ground body
    ChSharedPtr<ChBody> ground(new ChBody());
    ground->SetBodyFixed(true);
    application.GetSystem()->AddBody(ground);

    // Create the rotating gear body
    ChVector<> pin_loc(0, 0, 0);

    ChSharedPtr<ChBody> gear(new ChBody());
    gear->SetPos(pin_loc);
    gear->SetWvel_loc(ChVector<>(0, 0, 0.0));
    application.GetSystem()->AddBody(gear);

    int n_teeth = 10;
    double R_T = 0.2605;
    double R_C = 0.3;
    double R = 0.089;
    ChSharedPtr<ChLinePath> path = vehicle::ChCircularProfile(n_teeth, R_T, R_C, R);

    // Add the collision shape to the body
    gear->GetCollisionModel()->SetSafeMargin(0.02);
    gear->SetCollide(true);
    gear->GetCollisionModel()->ClearModel();
    gear->GetCollisionModel()->Add2Dpath(*path.get_ptr());
    gear->GetCollisionModel()->BuildModel();

    // Create a ChLineShape, a visualization asset for lines.
    ChSharedPtr<ChLineShape> asset(new ChLineShape);
    asset->SetLineGeometry(path);
    asset->SetColor(ChColor(1, 0, 0));
    gear->AddAsset(asset);

    // Revolute constraint 
    ChSharedPtr<ChLinkLockRevolute> revolute (new ChLinkLockRevolute);
    revolute->Initialize(gear, ground, ChCoordsys<>(pin_loc));
    application.GetSystem()->Add(revolute);

    application.AssetBindAll();
    application.AssetUpdateAll();
    application.SetTimestep(0.01);
    application.SetTryRealtime(true);

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();
    }

    return 0;
}
