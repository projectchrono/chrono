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
// Author: Radu Serban
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/assets/ChVisualSystem.h"
#include "chrono/utils/ChBodyGeometry.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

#include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Output directory
const std::string out_dir = GetChronoOutputPath() + "FSI_BCE/";

// -----------------------------------------------------------------------------

std::shared_ptr<vsg3d::ChVisualSystemVSG> CreateVisulization(ChFsiSystemSPH& sysFSI,
                                                             ChSystem& sysMBS,
                                                             const std::string& title);
void Box();
void Sphere();
void Cylinder1();
void Cylinder2();
void Cone();
void BoxContainer();

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    // Interior and exterior BCEs for box geometry (also with small dimension)
    Box();

    // Interior and exterior BCEs for sphere geometry (Cartesian and polar coordinates)
    Sphere();

    // Interior and exterior BCEs for cylinder geometry (Cartesian and polar coordinates)
    Cylinder1();

    // Interior and exterior BCEs for thin cylinder geometry (Cartesian and polar coordinates)
    Cylinder2();

    // Interior and exterior BCEs for cone geometry (Cartesian and polar coordinates)
    Cone();

    // Box container
    BoxContainer();

    return 0;
}

// -----------------------------------------------------------------------------

class MarkerPositionVisibilityCallback : public ChSphVisualizationVSG::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}
    virtual bool get(unsigned int n) const override { return pos[n].y >= 0; }
};

std::shared_ptr<vsg3d::ChVisualSystemVSG> CreateVisulization(ChFsiSystemSPH& sysFSI,
                                                             ChSystem& sysMBS,
                                                             const std::string& title) {
    auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(&sysFSI);
    visFSI->EnableFluidMarkers(true);
    visFSI->EnableBoundaryMarkers(true);
    visFSI->EnableRigidBodyMarkers(true);
    visFSI->SetBCEVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());

    // VSG visual system (attach visFSI as plugin)
    auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    visVSG->AttachPlugin(visFSI);
    visVSG->AttachSystem(&sysMBS);
    visVSG->SetWindowTitle(title);
    visVSG->SetWindowSize(1280, 800);
    visVSG->SetWindowPosition(100, 100);
    visVSG->AddCamera(ChVector3d(-0.2, -3.0, 0), ChVector3d(-0.2, 0, 0));
    visVSG->SetLightIntensity(0.9f);
    visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);
    visVSG->ToggleBodyLabelVisibility();
    visVSG->SetBodyLabelsScale(0.5);

    visVSG->Initialize();

    return visVSG;
}

// -----------------------------------------------------------------------------

void Box() {
    double spacing = 0.025;

    ChSystemSMC sysMBS;
    ChFsiFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH);

    sysSPH.SetInitialSpacing(spacing);
    sysSPH.SetKernelMultiplier(1.0);
    sysFSI.SetStepSizeCFD(1e-4);

    std::ofstream fbce;
    ChFrame<> frame(ChVector3d(-0.1, 0, -0.1), QuatFromAngleY(CH_PI / 8));

    {
        ChVector3d box_size(0.4, 0.2, 0.4);

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(-0.5, 0, -0.5));
        body->SetName("exterior thick");

        utils::ChBodyGeometry geometry;
        geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(frame.GetPos(), frame.GetRot(), box_size));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        auto bce = sysSPH.CreatePointsBoxExterior(box_size);
        fbce.open((out_dir + "/box_bndry_1.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    {
        ChVector3d box_size(0.1, 0.2, 0.4);

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, -0.5));
        body->SetName("exterior thin");

        utils::ChBodyGeometry geometry;
        geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(frame.GetPos(), frame.GetRot(), box_size));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        auto bce = sysSPH.CreatePointsBoxExterior(box_size);
        fbce.open((out_dir + "/box_bndry_2.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    {
        ChVector3d box_size(0.4, 0.2, 0.4);

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(-0.5, 0, +0.5));
        body->SetName("interior thick");

        utils::ChBodyGeometry geometry;
        geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(frame.GetPos(), frame.GetRot(), box_size));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        auto bce = sysSPH.CreatePointsBoxInterior(box_size);
        fbce.open((out_dir + "/box_solid_1.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    {
        ChVector3d box_size(0.1, 0.2, 0.4);

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, +0.5));
        body->SetName("exterior thin");

        utils::ChBodyGeometry geometry;
        geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(frame.GetPos(), frame.GetRot(), box_size));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        auto bce = sysSPH.CreatePointsBoxInterior(box_size);
        fbce.open((out_dir + "/box_solid_2.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    sysFSI.Initialize();
    auto vis = CreateVisulization(sysFSI, sysMBS, "BCE markers for box geometry");
    while (vis->Run()) {
        vis->Render();
    }
}

// -----------------------------------------------------------------------------

void Sphere() {
    double spacing = 0.025;

    ChSystemSMC sysMBS;
    ChFsiFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH);

    sysSPH.SetInitialSpacing(spacing);
    sysSPH.SetKernelMultiplier(1.0);
    sysFSI.SetStepSizeCFD(1e-4);

    std::ofstream fbce;
    ChFrame<> frame(ChVector3d(-0.1, 0, -0.1), QuatFromAngleY(CH_PI / 8));

    // Spherical container (Cartesian coordinates)
    {
        double radius = 0.25;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(-0.5, 0, -0.5));
        body->SetName("exterior Cartesian");

        utils::ChBodyGeometry geometry;
        geometry.coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(frame.GetPos(), radius));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        auto bce = sysSPH.CreatePointsSphereExterior(radius, false);
        fbce.open((out_dir + "/sphere_bndry_1.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    // Spherical container (Polar coordinates)
    {
        double radius = 0.25;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, -0.5));
        body->SetName("exterior polar");

        utils::ChBodyGeometry geometry;
        geometry.coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(frame.GetPos(), radius));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        auto bce = sysSPH.CreatePointsSphereExterior(radius, true);
        fbce.open((out_dir + "/sphere_bndry_2.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    // Spherical solid (Cartesian coordinates)
    {
        double radius = 0.25;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(-0.5, 0, +0.5));
        body->SetName("interior Cartesian");

        utils::ChBodyGeometry geometry;
        geometry.coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(frame.GetPos(), radius));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        auto bce = sysSPH.CreatePointsSphereInterior(radius, false);
        fbce.open((out_dir + "/sphere_solid_1.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    // Spherical solid (Polar coordinates)
    {
        double radius = 0.25;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, +0.5));
        body->SetName("interior polar");

        utils::ChBodyGeometry geometry;
        geometry.coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(frame.GetPos(), radius));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        auto bce = sysSPH.CreatePointsSphereInterior(radius, true);
        fbce.open((out_dir + "/sphere_solid_2.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    sysFSI.Initialize();
    auto vis = CreateVisulization(sysFSI, sysMBS, "BCE markers for sphere geometry");
    while (vis->Run()) {
        vis->Render();
    }
}

// -----------------------------------------------------------------------------

void Cylinder1() {
    double spacing = 0.025;

    ChSystemSMC sysMBS;
    ChFsiFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH);

    sysSPH.SetInitialSpacing(spacing);
    sysSPH.SetKernelMultiplier(1.0);
    sysFSI.SetStepSizeCFD(1e-4);

    std::ofstream fbce;
    ChFrame<> frame(ChVector3d(-0.1, 0, -0.1), QuatFromAngleY(CH_PI / 8));

    // Cylindrical container (Cartesian coordinates)
    {
        double radius = 0.125;
        double length = 0.4;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(-0.5, 0, -0.5));
        body->SetName("exterior Cartesian");

        utils::ChBodyGeometry geometry;
        geometry.coll_cylinders.push_back(
            utils::ChBodyGeometry::CylinderShape(frame.GetPos(), frame.GetRot(), radius, length));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        auto bce = sysSPH.CreatePointsCylinderExterior(radius, length, false);
        fbce.open((out_dir + "/cylinder_bndry_1.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    // Cylindrical container (Polar coordinates)
    {
        double radius = 0.125;
        double length = 0.4;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, -0.5));
        body->SetName("exterior polar");

        utils::ChBodyGeometry geometry;
        geometry.coll_cylinders.push_back(
            utils::ChBodyGeometry::CylinderShape(frame.GetPos(), frame.GetRot(), radius, length));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        auto bce = sysSPH.CreatePointsCylinderExterior(radius, length, true);
        fbce.open((out_dir + "/cylinder_bndry_2.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    // Cylindrical solid (Cartesian coordinates)
    {
        double radius = 0.125;
        double length = 0.4;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(-0.5, 0, +0.5));
        body->SetName("interior Cartesian");

        utils::ChBodyGeometry geometry;
        geometry.coll_cylinders.push_back(
            utils::ChBodyGeometry::CylinderShape(frame.GetPos(), frame.GetRot(), radius, length));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        auto bce = sysSPH.CreatePointsCylinderInterior(radius, length, false);
        fbce.open((out_dir + "/cylinder_solid_1.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    // Cylindrical solid (Polar coordinates)
    {
        double radius = 0.125;
        double length = 0.4;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, +0.5));
        body->SetName("interior polar");

        utils::ChBodyGeometry geometry;
        geometry.coll_cylinders.push_back(
            utils::ChBodyGeometry::CylinderShape(frame.GetPos(), frame.GetRot(), radius, length));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        auto bce = sysSPH.CreatePointsCylinderInterior(radius, length, true);
        fbce.open((out_dir + "/cylinder_solid_2.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    sysFSI.Initialize();
    auto vis = CreateVisulization(sysFSI, sysMBS, "BCE markers for cylinder geometry");
    while (vis->Run()) {
        vis->Render();
    }
}

void Cylinder2() {
    double spacing = 0.025;

    ChSystemSMC sysMBS;
    ChFsiFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH);

    sysSPH.SetInitialSpacing(spacing);
    sysSPH.SetKernelMultiplier(1.0);
    sysFSI.SetStepSizeCFD(1e-4);

    std::ofstream fbce;
    ChFrame<> frame(ChVector3d(-0.1, 0, -0.1), QuatFromAngleY(CH_PI / 8));

    // Cylindrical solid short (Cartesian coordinates)
    {
        double radius = 0.125;
        double length = 0.1;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(-0.5, 0, -0.5));
        body->SetName("exterior Cartesian");

        utils::ChBodyGeometry geometry;
        geometry.coll_cylinders.push_back(
            utils::ChBodyGeometry::CylinderShape(frame.GetPos(), frame.GetRot(), radius, length));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        auto bce = sysSPH.CreatePointsCylinderInterior(radius, length, false);
        fbce.open((out_dir + "/cylinder_solid_3.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    // Cylindrical solid short (Polar coordinates)
    {
        double radius = 0.125;
        double length = 0.1;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, -0.5));
        body->SetName("exterior polar");

        utils::ChBodyGeometry geometry;
        geometry.coll_cylinders.push_back(
            utils::ChBodyGeometry::CylinderShape(frame.GetPos(), frame.GetRot(), radius, length));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        auto bce = sysSPH.CreatePointsCylinderInterior(radius, length, true);
        fbce.open((out_dir + "/cylinder_solid_4.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    // Cylindrical solid thin (Cartesian coordinates)
    {
        double radius = 0.06;
        double length = 0.4;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(-0.5, 0, +0.5));
        body->SetName("interior Cartesian");

        utils::ChBodyGeometry geometry;
        geometry.coll_cylinders.push_back(
            utils::ChBodyGeometry::CylinderShape(frame.GetPos(), frame.GetRot(), radius, length));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        auto bce = sysSPH.CreatePointsCylinderInterior(radius, length, false);
        fbce.open((out_dir + "/cylinder_solid_5.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    // Cylindrical solid thin (Polar coordinates)
    {
        double radius = 0.05;
        double length = 0.4;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, +0.5));
        body->SetName("interior polar");

        utils::ChBodyGeometry geometry;
        geometry.coll_cylinders.push_back(
            utils::ChBodyGeometry::CylinderShape(frame.GetPos(), frame.GetRot(), radius, length));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        auto bce = sysSPH.CreatePointsCylinderInterior(radius, length, true);
        fbce.open((out_dir + "/cylinder_solid_6.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    sysFSI.Initialize();
    auto vis = CreateVisulization(sysFSI, sysMBS, "BCE markers for thin cylinder geometry");
    while (vis->Run()) {
        vis->Render();
    }
}

// -----------------------------------------------------------------------------

void Cone() {
    double spacing = 0.025;

    ChSystemSMC sysMBS;
    ChFsiFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH);

    sysSPH.SetInitialSpacing(spacing);
    sysSPH.SetKernelMultiplier(1.0);
    sysFSI.SetStepSizeCFD(1e-4);

    ChFrame<> frame(ChVector3d(-0.1, 0, -0.1), QuatFromAngleY(CH_PI / 8));
    std::ofstream fbce;

    double cone_radius = 0.25;
    double cone_height = 0.2;

    {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(-0.5, 0, -0.5));
        body->SetName("exterior Cartesian");

        auto bce = sysSPH.CreatePointsConeExterior(cone_radius, cone_height, false);
        fbce.open((out_dir + "/cone_bndry_cartesian.txt"), std::ios::trunc);
        for (int i = 0; i < bce.size(); i++) {
            fbce << bce[i].x() << " " << bce[i].y() << " " << bce[i].z() << endl;
        }
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, -0.5));
        body->SetName("exterior polar");

        auto bce = sysSPH.CreatePointsConeExterior(cone_radius, cone_height, true);
        fbce.open((out_dir + "/cone_bndry_polar.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, -0.5));
        body->SetName("truncated exterior polar");

        auto bce = sysSPH.CreatePointsTruncatedConeExterior(cone_radius, cone_radius / 2, cone_height, true);
        fbce.open((out_dir + "/cone_bndry_truncated_polar.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, -0.5));
        body->SetName("truncated exterior Cartesian");

        auto bce = sysSPH.CreatePointsTruncatedConeExterior(cone_radius, cone_radius / 2, cone_height, false);
        fbce.open((out_dir + "/cone_bndry_truncated_cartesian.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(-0.5, 0, +0.5));
        body->SetName("interior Cartesian");

        auto bce = sysSPH.CreatePointsConeInterior(cone_radius, cone_height, false);
        fbce.open((out_dir + "/cone_solid_cartesian.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, +0.5));
        body->SetName("interior polar");

        auto bce = sysSPH.CreatePointsConeInterior(cone_radius, cone_height, true);
        fbce.open((out_dir + "/cone_solid_polar.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, +0.5));
        body->SetName("truncated interior polar");

        auto bce = sysSPH.CreatePointsTruncatedConeInterior(cone_radius, cone_radius / 2, cone_height, true);
        fbce.open((out_dir + "/cone_solid_truncated_polar.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, +0.5));
        body->SetName("truncated interior Cartesian");

        auto bce = sysSPH.CreatePointsTruncatedConeInterior(cone_radius, cone_radius / 2, cone_height, false);
        fbce.open((out_dir + "/cone_solid_truncated_cartesian.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    sysFSI.Initialize();
    auto vis = CreateVisulization(sysFSI, sysMBS, "BCE markers for cone geometry");
    while (vis->Run()) {
        vis->Render();
    }
}

// -----------------------------------------------------------------------------

void BoxContainer() {
    double spacing = 0.025;

    ChSystemSMC sysMBS;
    ChFsiFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH);

    sysSPH.SetInitialSpacing(spacing);
    sysSPH.SetKernelMultiplier(1.0);
    sysFSI.SetStepSizeCFD(1e-4);

    std::ofstream fbce;
    ChFrame<> frame(ChVector3d(-0.1, 0, -0.1), QuatFromAngleY(CH_PI / 8));

    ChVector3d size(0.8, 0.6, 0.4);

    {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(-0.6, 0, -0.5));
        body->SetName("2, 2, -1");

        auto bce = sysSPH.CreatePointsBoxContainer(size, {2, 2, -1});
        fbce.open((out_dir + "/container_1.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.6, 0, -0.5));
        body->SetName("0, 2, 2");

        auto bce = sysSPH.CreatePointsBoxContainer(size, {0, 2, 2});
        fbce.open((out_dir + "/container_2.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(-0.6, 0, +0.5));
        body->SetName("2, 0, -1");

        auto bce = sysSPH.CreatePointsBoxContainer(size, {2, 0, -1});
        fbce.open((out_dir + "/container_3.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.6, 0, +0.5));
        body->SetName("-1, +1, -1");

        auto bce = sysSPH.CreatePointsBoxContainer(size, {-1, +1, -1});
        fbce.open((out_dir + "/container_4.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body, bce, frame, false);
    }

    sysFSI.Initialize();
    auto vis = CreateVisulization(sysFSI, sysMBS, "BCE markers for container geometry");
    while (vis->Run()) {
        vis->Render();
    }
}
