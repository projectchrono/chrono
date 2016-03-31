#include <cmath>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"

#include "chrono/geometry/ChCLinePath.h"
#include "chrono/geometry/ChCLineSegment.h"
#include "chrono/geometry/ChCLineArc.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::irrlicht;

using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

// -----------------------------------------------------------------------------
std::shared_ptr<geometry::ChLinePath> CreateProfile(int num_teeth, double R_T, double R_C, double R) {
    auto profile = std::make_shared<geometry::ChLinePath>();

    double beta = CH_C_2PI / num_teeth;
    double sbeta = std::sin(beta / 2);
    double cbeta = std::cos(beta / 2);
    double y = (R_T * R_T + R_C * R_C - R * R) / (2 * R_C);
    double x = std::sqrt(R_T * R_T - y * y);
    double gamma = std::asin(x / R);

    for (int i = 0; i < num_teeth; ++i) {
        double alpha = -i * beta;
        ChVector<> p0(0, R_C, 0);
        ChVector<> p1(-R_T * sbeta, R_T * cbeta, 0);
        ChVector<> p2(-x, y, 0);
        ChVector<> p3(x, y, 0);
        ChVector<> p4(R_T * sbeta, R_T * cbeta, 0);
        ChQuaternion<> quat;
        quat.Q_from_AngZ(alpha);
        ChMatrix33<> rot(quat);
        p0 = rot * p0;
        p1 = rot * p1;
        p2 = rot * p2;
        p3 = rot * p3;
        p4 = rot * p4;
        geometry::ChLineSegment seg1(p1, p2);
        double angle1 = alpha + 1.5 * CH_C_PI - gamma;
        double angle2 = alpha + 1.5 * CH_C_PI + gamma;
        geometry::ChLineArc arc(ChCoordsys<>(p0), R, angle1, angle2, true);
        geometry::ChLineSegment seg2(p3, p4);
        profile->AddSubLine(seg1);
        profile->AddSubLine(arc);
        profile->AddSubLine(seg2);
    }

    return profile;
}

// -----------------------------------------------------------------------------
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

    auto ground = std::make_shared<ChBody>();
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    application.GetSystem()->AddBody(ground);

    // -----------------------------
    // Create the rotating gear body
    // -----------------------------

    ChVector<> gear_loc(0, 0, 0);

    auto gear = std::make_shared<ChBody>();
    gear->SetIdentifier(0);
    gear->SetPos(gear_loc);
    //gear->SetWvel_loc(ChVector<>(0, 0, 0.1));
    application.GetSystem()->AddBody(gear);

    int n_teeth = 10;
    double R_T = 0.2605;
    double R_C = 0.3;
    double R = 0.089;
    double separation = 0.25;

    std::shared_ptr<ChLinePath> gear_profile = CreateProfile(n_teeth, R_T, R_C, R);

    // Add the collision shape to gear
    gear->GetCollisionModel()->SetSafeMargin(0.02);
    gear->SetCollide(true);
    gear->GetCollisionModel()->ClearModel();
    gear->GetCollisionModel()->Add2Dpath(*gear_profile.get(), ChVector<>(0, 0, separation / 2));
    gear->GetCollisionModel()->Add2Dpath(*gear_profile.get(), ChVector<>(0, 0, -separation / 2));
    gear->GetCollisionModel()->BuildModel();

    // Add ChLineShape visualization asset to gear
    auto gear_profile_plus = std::make_shared<ChLineShape>();
    gear_profile_plus->SetLineGeometry(gear_profile);
    gear_profile_plus->Pos = ChVector<>(0, 0, separation / 2);
    gear_profile_plus->SetColor(ChColor(1, 0, 0));
    gear->AddAsset(gear_profile_plus);

    auto gear_profile_minus = std::make_shared<ChLineShape>();
    gear_profile_minus->SetLineGeometry(gear_profile);
    gear_profile_minus->SetColor(ChColor(0, 1, 0));
    gear_profile_minus->Pos = ChVector<>(0, 0, -separation / 2);
    gear->AddAsset(gear_profile_minus);

    auto gear_axle = std::make_shared<ChCylinderShape>();
    gear_axle->GetCylinderGeometry().p1 = ChVector<>(0, 0, separation / 2);
    gear_axle->GetCylinderGeometry().p2 = ChVector<>(0, 0, -separation / 2);
    gear_axle->GetCylinderGeometry().rad = 0.1 * R;
    gear->AddAsset(gear_axle);

    // Revolute constraint (gear-ground)
    auto revolute = std::make_shared<ChLinkLockRevolute>();
    revolute->Initialize(gear, ground, ChCoordsys<>(gear_loc));
    application.GetSystem()->Add(revolute);

    // ---------------------
    // Create a falling body
    // ---------------------

    double pin_hlen = 0.6 * separation;
    double pin_radius = 0.3 * R;
    ChVector<> pin_loc = gear_loc + ChVector<>(0, R_T, 0);

    auto pin = std::make_shared<ChBody>();
    pin->SetIdentifier(1);
    pin->SetPos(pin_loc);
    //pin->SetBodyFixed(true);
    application.GetSystem()->AddBody(pin);

    auto pin_profile = std::make_shared<ChLinePath>();
    ChLineArc pin_circle(ChCoordsys<>(), pin_radius, CH_C_2PI, 0, false);
    pin_profile->AddSubLine(pin_circle);

    // Add collision shapes to pin
    pin->GetCollisionModel()->SetSafeMargin(0.02);
    pin->SetCollide(true);
    pin->GetCollisionModel()->ClearModel();
    pin->GetCollisionModel()->Add2Dpath(*pin_profile.get(), ChVector<>(0, 0, separation / 2));
    pin->GetCollisionModel()->Add2Dpath(*pin_profile.get(), ChVector<>(0, 0, -separation / 2));
    //pin->GetCollisionModel()->AddCylinder(pin_radius, pin_radius, pin_hlen);
    pin->GetCollisionModel()->BuildModel();

    // Add pin visualization
    auto pin_cyl = std::make_shared<ChCylinderShape>();
    pin_cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, pin_hlen);
    pin_cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, -pin_hlen);
    pin_cyl->GetCylinderGeometry().rad = pin_radius;
    pin->AddAsset(pin_cyl);
    auto pin_col = std::make_shared<ChColorAsset>();
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
