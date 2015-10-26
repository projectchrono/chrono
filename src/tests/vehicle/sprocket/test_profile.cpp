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
    ChSystem system;

    ChIrrApp application(&system, L"Paths", core::dimension2d<u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0.5, 0.5, -1));

    // Show contact forces in Irrlicht application
    application.SetSymbolscale(0.2);
    application.SetContactsDrawMode(ChIrrTools::eCh_ContactsDrawMode::CONTACT_NORMALS);

    // ----------------------
    // Create the ground body
    // ----------------------

    ChSharedPtr<ChBody> ground(new ChBody());
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    application.GetSystem()->AddBody(ground);

    // -----------------------------
    // Create the rotating gear body
    // -----------------------------

    ChVector<> gear_loc(0, 0, 0);

    ChSharedPtr<ChBody> gear(new ChBody());
    gear->SetIdentifier(0);
    gear->SetPos(gear_loc);
    //gear->SetWvel_loc(ChVector<>(0, 0, 0.1));
    application.GetSystem()->AddBody(gear);

    int n_teeth = 10;
    double R_T = 0.2605;
    double R_C = 0.3;
    double R = 0.089;
    double separation = 0.25;

    ChSharedPtr<ChLinePath> gear_profile = vehicle::ChCircularProfile(n_teeth, R_T, R_C, R);

    // Add the collision shape to gear
    gear->GetCollisionModel()->SetSafeMargin(0.02);
    gear->SetCollide(true);
    gear->GetCollisionModel()->ClearModel();
    gear->GetCollisionModel()->Add2Dpath(*gear_profile.get_ptr(), ChVector<>(0, 0, separation / 2));
    gear->GetCollisionModel()->Add2Dpath(*gear_profile.get_ptr(), ChVector<>(0, 0, -separation / 2));
    gear->GetCollisionModel()->BuildModel();

    // Add ChLineShape visualization asset to gear
    ChSharedPtr<ChLineShape> gear_profile_plus(new ChLineShape);
    gear_profile_plus->SetLineGeometry(gear_profile);
    gear_profile_plus->Pos = ChVector<>(0, 0, separation / 2);
    gear_profile_plus->SetColor(ChColor(1, 0, 0));
    gear->AddAsset(gear_profile_plus);

    ChSharedPtr<ChLineShape> gear_profile_minus(new ChLineShape);
    gear_profile_minus->SetLineGeometry(gear_profile);
    gear_profile_minus->SetColor(ChColor(0, 1, 0));
    gear_profile_minus->Pos = ChVector<>(0, 0, -separation / 2);
    gear->AddAsset(gear_profile_minus);

    ChSharedPtr<ChCylinderShape> gear_axle(new ChCylinderShape);
    gear_axle->GetCylinderGeometry().p1 = ChVector<>(0, 0, separation / 2);
    gear_axle->GetCylinderGeometry().p2 = ChVector<>(0, 0, -separation / 2);
    gear_axle->GetCylinderGeometry().rad = 0.1 * R;
    gear->AddAsset(gear_axle);

    // Revolute constraint (gear-ground)
    ChSharedPtr<ChLinkLockRevolute> revolute(new ChLinkLockRevolute);
    revolute->Initialize(gear, ground, ChCoordsys<>(gear_loc));
    application.GetSystem()->Add(revolute);

    // ---------------------
    // Create a falling body
    // ---------------------

    double pin_hlen = 0.6 * separation;
    double pin_radius = 0.3 * R;
    ChVector<> pin_loc = gear_loc + ChVector<>(0, R_T, 0);

    ChSharedPtr<ChBody> pin(new ChBody());
    pin->SetIdentifier(1);
    pin->SetPos(pin_loc);
    //pin->SetBodyFixed(true);
    application.GetSystem()->AddBody(pin);

    ChSharedPtr<ChLinePath> pin_profile(new ChLinePath);
    ChLineArc pin_circle(ChCoordsys<>(), pin_radius, CH_C_2PI, 0, false);
    pin_profile->AddSubLine(pin_circle);

    // Add collision shapes to pin
    pin->GetCollisionModel()->SetSafeMargin(0.02);
    pin->SetCollide(true);
    pin->GetCollisionModel()->ClearModel();
    pin->GetCollisionModel()->Add2Dpath(*pin_profile.get_ptr(), ChVector<>(0, 0, separation / 2));
    pin->GetCollisionModel()->Add2Dpath(*pin_profile.get_ptr(), ChVector<>(0, 0, -separation / 2));
    //pin->GetCollisionModel()->AddCylinder(pin_radius, pin_radius, pin_hlen);
    pin->GetCollisionModel()->BuildModel();

    // Add pin visualization
    ChSharedPtr<ChCylinderShape> pin_cyl(new ChCylinderShape);
    pin_cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, pin_hlen);
    pin_cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, -pin_hlen);
    pin_cyl->GetCylinderGeometry().rad = pin_radius;
    pin->AddAsset(pin_cyl);
    ChSharedPtr<ChColorAsset> pin_col(new ChColorAsset);
    pin_col->SetColor(ChColor(0.2f, 0.5f, 0.8f));
    pin->AddAsset(pin_col);

    // ---------------
    // Simulation loop
    // ---------------

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
