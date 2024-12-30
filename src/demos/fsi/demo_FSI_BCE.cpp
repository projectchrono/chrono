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
#include "chrono/utils/ChBodyGeometry.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"
#include "chrono_fsi/sph/utils/ChUtilsTypeConvert.h"

#include "chrono_fsi/sph/visualization/ChFsiVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fsi;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Enable/disable run-time visualization
bool render = true;

// Output directory
const std::string out_dir = GetChronoOutputPath() + "FSI_BCE/";

// -----------------------------------------------------------------------------

std::shared_ptr<ChFsiVisualization> CreateVisulization(ChFsiSystemSPH& sysFSI,
                                                       ChSystem& sysMBS,
                                                       const std::string& title);
void Box();
void Sphere();
void Cylinder1();
void Cylinder2();
void CylindricalAnnulus();
void Cone();

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

#ifndef CHRONO_OPENGL
    if (vis_type == ChVisualSystem::Type::OpenGL)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::OpenGL;
#endif
#if !defined(CHRONO_OPENGL) && !defined(CHRONO_VSG)
    render = false;
#endif

    // Interior and exterior BCEs for box geometry (also with small dimension)
    Box();

    // Interior and exterior BCEs for sphere geometry (Cartesian and polar coordinates)
    Sphere();

    // Interior and exterior BCEs for cylinder geometry (Cartesian and polar coordinates)
    Cylinder1();

    // Interior and exterior BCEs for thin cylinder geometry (Cartesian and polar coordinates)
    Cylinder2();

    Cone();
    CylindricalAnnulus();

    return 0;
}

// -----------------------------------------------------------------------------

class MarkerPositionVisibilityCallback : public ChFsiVisualization::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}

    virtual bool get(unsigned int n) const override { return pos[n].y >= 0; }
};

std::shared_ptr<ChFsiVisualization> CreateVisulization(ChFsiSystemSPH& sysFSI,
                                                       ChSystem& sysMBS,
                                                       const std::string& title) {
    std::shared_ptr<ChFsiVisualization> visFSI;
    if (render) {
        switch (vis_type) {
            case ChVisualSystem::Type::OpenGL:
#ifdef CHRONO_OPENGL
                visFSI = chrono_types::make_shared<ChFsiVisualizationGL>(&sysFSI);
#endif
                break;
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
#endif
                break;
            }
        }

        visFSI->SetTitle(title);
        visFSI->SetSize(1280, 1280);
        visFSI->AddCamera(ChVector3d(-0.2, -3.0, 0), ChVector3d(-0.2, 0, 0));
        visFSI->SetCameraMoveScale(0.1f);
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableRigidBodyMarkers(true);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetBCEVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
        visFSI->AttachSystem(&sysMBS);
        visFSI->Initialize();
    }

    return visFSI;
}

// -----------------------------------------------------------------------------

void Box() {
    double spacing = 0.025;

    ChSystemSMC sysMBS;
    ChFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(sysMBS, sysSPH);

    sysSPH.SetInitialSpacing(spacing);
    sysSPH.SetKernelMultiplier(1.0);
    sysFSI.SetStepSizeCFD(1e-4);

    std::vector<ChVector3d> bce;
    std::ofstream fbce;
    ChFrame<> frame(ChVector3d(-0.1, 0, -0.1), QuatFromAngleY(CH_PI / 8));

    {
        ChVector3d box_size(0.4, 0.2, 0.4);

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(-0.5, 0, -0.5));

        utils::ChBodyGeometry geometry;
        geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(frame.GetPos(), frame.GetRot(), box_size));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        bce.clear();
        sysSPH.CreateBCE_BoxExterior(box_size, bce);
        fbce.open((out_dir + "/box_bndry_1.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body);
        sysSPH.AddPointsBCE(body, bce, frame, false);
    }

    {
        ChVector3d box_size(0.1, 0.2, 0.4);

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, -0.5));

        utils::ChBodyGeometry geometry;
        geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(frame.GetPos(), frame.GetRot(), box_size));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        bce.clear();
        sysSPH.CreateBCE_BoxExterior(box_size, bce);
        fbce.open((out_dir + "/box_bndry_2.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body);
        sysSPH.AddPointsBCE(body, bce, frame, false);
    }

    {
        ChVector3d box_size(0.4, 0.2, 0.4);

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(-0.5, 0, +0.5));

        utils::ChBodyGeometry geometry;
        geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(frame.GetPos(), frame.GetRot(), box_size));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        bce.clear();
        sysSPH.CreateBCE_BoxInterior(box_size, bce);
        fbce.open((out_dir + "/box_solid_1.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body);
        sysSPH.AddPointsBCE(body, bce, frame, true);
    }

    {
        ChVector3d box_size(0.1, 0.2, 0.4);

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, +0.5));

        utils::ChBodyGeometry geometry;
        geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(frame.GetPos(), frame.GetRot(), box_size));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        bce.clear();
        sysSPH.CreateBCE_BoxInterior(box_size, bce);
        fbce.open((out_dir + "/box_solid_2.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body);
        sysSPH.AddPointsBCE(body, bce, frame, true);
    }

    if (render) {
        sysFSI.Initialize();
        auto visFSI = CreateVisulization(sysFSI, sysMBS, "BCE markers for box geometry");
        while (visFSI->Render()) {
        }
    }
}

// -----------------------------------------------------------------------------

void Sphere() {
    double spacing = 0.025;

    ChSystemSMC sysMBS;
    ChFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(sysMBS, sysSPH);

    sysSPH.SetInitialSpacing(spacing);
    sysSPH.SetKernelMultiplier(1.0);
    sysFSI.SetStepSizeCFD(1e-4);

    std::vector<ChVector3d> bce;
    std::ofstream fbce;
    ChFrame<> frame(ChVector3d(-0.1, 0, -0.1), QuatFromAngleY(CH_PI / 8));

    // Spherical container (Cartesian coordinates)
    {
        double radius = 0.25;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(-0.5, 0, -0.5));

        utils::ChBodyGeometry geometry;
        geometry.coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(frame.GetPos(), radius));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        bce.clear();
        sysSPH.CreateBCE_SphereExterior(radius, false, bce);
        fbce.open((out_dir + "/sphere_bndry_1.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body);
        sysSPH.AddPointsBCE(body, bce, frame, false);
    }

    // Spherical container (Polar coordinates)
    {
        double radius = 0.25;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, -0.5));

        utils::ChBodyGeometry geometry;
        geometry.coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(frame.GetPos(), radius));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        bce.clear();
        sysSPH.CreateBCE_SphereExterior(radius, true, bce);
        fbce.open((out_dir + "/sphere_bndry_2.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body);
        sysSPH.AddPointsBCE(body, bce, frame, false);
    }

    // Spherical solid (Cartesian coordinates)
    {
        double radius = 0.25;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(-0.5, 0, +0.5));

        utils::ChBodyGeometry geometry;
        geometry.coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(frame.GetPos(), radius));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        bce.clear();
        sysSPH.CreateBCE_SphereInterior(radius, false, bce);
        fbce.open((out_dir + "/sphere_solid_1.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body);
        sysSPH.AddPointsBCE(body, bce, frame, true);
    }

    // Spherical solid (Polar coordinates)
    {
        double radius = 0.25;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, +0.5));

        utils::ChBodyGeometry geometry;
        geometry.coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(frame.GetPos(), radius));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        bce.clear();
        sysSPH.CreateBCE_SphereInterior(radius, true, bce);
        fbce.open((out_dir + "/sphere_solid_2.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body);
        sysSPH.AddPointsBCE(body, bce, frame, true);
    }

    if (render) {
        sysFSI.Initialize();
        auto visFSI = CreateVisulization(sysFSI, sysMBS, "BCE markers for sphere geometry");
        while (visFSI->Render()) {
        }
    }
}

// -----------------------------------------------------------------------------

void Cylinder1() {
    double spacing = 0.025;

    ChSystemSMC sysMBS;
    ChFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(sysMBS, sysSPH);

    sysSPH.SetInitialSpacing(spacing);
    sysSPH.SetKernelMultiplier(1.0);
    sysFSI.SetStepSizeCFD(1e-4);

    std::vector<ChVector3d> bce;
    std::ofstream fbce;
    ChFrame<> frame(ChVector3d(-0.1, 0, -0.1), QuatFromAngleY(CH_PI / 8));

    // Cylindrical container (Cartesian coordinates)
    {
        double radius = 0.125;
        double length = 0.4;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(-0.5, 0, -0.5));

        utils::ChBodyGeometry geometry;
        geometry.coll_cylinders.push_back(
            utils::ChBodyGeometry::CylinderShape(frame.GetPos(), frame.GetRot(), radius, length));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        bce.clear();
        sysSPH.CreateBCE_CylinderExterior(radius, length, false, bce);
        fbce.open((out_dir + "/cylinder_bndry_1.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body);
        sysSPH.AddPointsBCE(body, bce, frame, false);
    }

    // Cylindrical container (Polar coordinates)
    {
        double radius = 0.125;
        double length = 0.4;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, -0.5));

        utils::ChBodyGeometry geometry;
        geometry.coll_cylinders.push_back(
            utils::ChBodyGeometry::CylinderShape(frame.GetPos(), frame.GetRot(), radius, length));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        bce.clear();
        sysSPH.CreateBCE_CylinderExterior(radius, length, true, bce);
        fbce.open((out_dir + "/cylinder_bndry_2.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body);
        sysSPH.AddPointsBCE(body, bce, frame, false);
    }

    // Cylindrical solid (Cartesian coordinates)
    {
        double radius = 0.125;
        double length = 0.4;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(-0.5, 0, +0.5));

        utils::ChBodyGeometry geometry;
        geometry.coll_cylinders.push_back(
            utils::ChBodyGeometry::CylinderShape(frame.GetPos(), frame.GetRot(), radius, length));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        bce.clear();
        sysSPH.CreateBCE_CylinderInterior(radius, length, false, bce);
        fbce.open((out_dir + "/cylinder_solid_1.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body);
        sysSPH.AddPointsBCE(body, bce, frame, true);
    }

    // Cylindrical solid (Polar coordinates)
    {
        double radius = 0.125;
        double length = 0.4;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, +0.5));

        utils::ChBodyGeometry geometry;
        geometry.coll_cylinders.push_back(
            utils::ChBodyGeometry::CylinderShape(frame.GetPos(), frame.GetRot(), radius, length));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        bce.clear();
        sysSPH.CreateBCE_CylinderInterior(radius, length, true, bce);
        fbce.open((out_dir + "/cylinder_solid_2.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body);
        sysSPH.AddPointsBCE(body, bce, frame, true);
    }

    if (render) {
        sysFSI.Initialize();
        auto visFSI = CreateVisulization(sysFSI, sysMBS, "BCE markers for cylinder geometry");
        while (visFSI->Render()) {
        }
    }
}

void Cylinder2() {
    double spacing = 0.025;

    ChSystemSMC sysMBS;
    ChFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(sysMBS, sysSPH);

    sysSPH.SetInitialSpacing(spacing);
    sysSPH.SetKernelMultiplier(1.0);
    sysFSI.SetStepSizeCFD(1e-4);

    std::vector<ChVector3d> bce;
    std::ofstream fbce;
    ChFrame<> frame(ChVector3d(-0.1, 0, -0.1), QuatFromAngleY(CH_PI / 8));

    // Cylindrical solid short (Cartesian coordinates)
    {
        double radius = 0.125;
        double length = 0.1;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(-0.5, 0, -0.5));

        utils::ChBodyGeometry geometry;
        geometry.coll_cylinders.push_back(
            utils::ChBodyGeometry::CylinderShape(frame.GetPos(), frame.GetRot(), radius, length));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        bce.clear();
        sysSPH.CreateBCE_CylinderInterior(radius, length, false, bce);
        fbce.open((out_dir + "/cylinder_solid_3.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body);
        sysSPH.AddPointsBCE(body, bce, frame, true);
    }

    // Cylindrical solid short (Polar coordinates)
    {
        double radius = 0.125;
        double length = 0.1;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, -0.5));

        utils::ChBodyGeometry geometry;
        geometry.coll_cylinders.push_back(
            utils::ChBodyGeometry::CylinderShape(frame.GetPos(), frame.GetRot(), radius, length));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        bce.clear();
        sysSPH.CreateBCE_CylinderInterior(radius, length, true, bce);
        fbce.open((out_dir + "/cylinder_solid_4.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body);
        sysSPH.AddPointsBCE(body, bce, frame, true);
    }

    // Cylindrical solid thin (Cartesian coordinates)
    {
        double radius = 0.06;
        double length = 0.4;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(-0.5, 0, +0.5));

        utils::ChBodyGeometry geometry;
        geometry.coll_cylinders.push_back(
            utils::ChBodyGeometry::CylinderShape(frame.GetPos(), frame.GetRot(), radius, length));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        bce.clear();
        sysSPH.CreateBCE_CylinderInterior(radius, length, false, bce);
        fbce.open((out_dir + "/cylinder_solid_5.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body);
        sysSPH.AddPointsBCE(body, bce, frame, true);
    }

    // Cylindrical solid thin (Polar coordinates)
    {
        double radius = 0.05;
        double length = 0.4;

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(ChVector3d(+0.5, 0, +0.5));

        utils::ChBodyGeometry geometry;
        geometry.coll_cylinders.push_back(
            utils::ChBodyGeometry::CylinderShape(frame.GetPos(), frame.GetRot(), radius, length));
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

        bce.clear();
        sysSPH.CreateBCE_CylinderInterior(radius, length, true, bce);
        fbce.open((out_dir + "/cylinder_solid_6.txt"), std::ios::trunc);
        for (const auto& p : bce)
            fbce << p << endl;
        fbce.close();

        sysMBS.AddBody(body);
        sysFSI.AddFsiBody(body);
        sysSPH.AddPointsBCE(body, bce, frame, true);
    }

    if (render) {
        sysFSI.Initialize();
        auto visFSI = CreateVisulization(sysFSI, sysMBS, "BCE markers for thin cylinder geometry");
        while (visFSI->Render()) {
        }
    }
}

// -----------------------------------------------------------------------------

void Cone() {
    double spacing = 0.025;

    ChSystemSMC sysMBS;
    ChFluidSystemSPH sysSPH;

    sysSPH.SetInitialSpacing(spacing);
    sysSPH.SetKernelMultiplier(1.0);

    auto body = chrono_types::make_shared<ChBody>();
    body->SetPos(ChVector3d(-1, -2, -3));
    body->SetPosDt(ChVector3d(0.2, 0.3, 0.4));
    body->SetRot(QuatFromAngleY(CH_PI / 4));
    body->SetAngVelLocal(ChVector3d(0.1, -0.1, 0.2));

    ChFrame<> frame(ChVector3d(-0.1, 0, -0.1), QuatFromAngleY(CH_PI / 8));
    std::vector<ChVector3d> bce;
    std::ofstream fbce;

    double cone_radius = 0.25;
    double cone_height = 0.2;

    bce.clear();
    sysSPH.CreateBCE_ConeExterior(cone_radius, cone_height, false, bce);
    cout << "cone bndry cartesian nBCE = " << bce.size() << endl;
    fbce.open((out_dir + "/cone_bndry_cartesian.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x() << " " << bce[i].y() << " " << bce[i].z() << endl;
    }
    fbce.close();
    sysSPH.AddConeBCE(body, frame, cone_radius, cone_height, false, false);

    bce.clear();
    sysSPH.CreateBCE_ConeExterior(cone_radius, cone_height, true, bce);
    cout << "cone bndry polar nBCE = " << bce.size() << endl;
    fbce.open((out_dir + "/cone_bndry_polar.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x() << " " << bce[i].y() << " " << bce[i].z() << endl;
    }
    fbce.close();
    sysSPH.AddConeBCE(body, frame, cone_radius, cone_height, false, true);

    bce.clear();
    sysSPH.CreateBCE_ConeInterior(cone_radius, cone_height, false, bce);
    cout << "cone solid cartesian nBCE = " << bce.size() << endl;
    fbce.open((out_dir + "/cone_solid_cartesian.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x() << " " << bce[i].y() << " " << bce[i].z() << endl;
    }
    fbce.close();
    sysSPH.AddConeBCE(body, frame, cone_radius, cone_height, true, false);

    bce.clear();
    sysSPH.CreateBCE_ConeInterior(cone_radius, cone_height, true, bce);
    cout << "cone solid polar nBCE = " << bce.size() << endl;
    fbce.open((out_dir + "/cone_solid_polar.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x() << " " << bce[i].y() << " " << bce[i].z() << endl;
    }
    fbce.close();
    sysSPH.AddConeBCE(body, frame, cone_radius, cone_height, true, true);
}

// -----------------------------------------------------------------------------

void CylindricalAnnulus() {
    double spacing = 0.025;

    ChSystemSMC sysMBS;
    ChFluidSystemSPH sysSPH;

    sysSPH.SetInitialSpacing(spacing);
    sysSPH.SetKernelMultiplier(1.0);

    auto body = chrono_types::make_shared<ChBody>();
    body->SetPos(ChVector3d(-1, -2, -3));
    body->SetPosDt(ChVector3d(0.2, 0.3, 0.4));
    body->SetRot(QuatFromAngleY(CH_PI / 4));
    body->SetAngVelLocal(ChVector3d(0.1, -0.1, 0.2));

    ChFrame<> frame(ChVector3d(-0.1, 0, -0.1), QuatFromAngleY(CH_PI / 8));
    std::vector<ChVector3d> bce;
    std::ofstream fbce;

    double ca_radius_inner = 0.2;
    double ca_radius_outer = 0.4;
    double ca_height = 0.2;

    bce.clear();
    sysSPH.CreatePoints_CylinderAnnulus(ca_radius_inner, ca_radius_outer, ca_height, false, spacing, bce);
    cout << "cylinder annulus cartesian nBCE = " << bce.size() << endl;
    fbce.open((out_dir + "/cyl_annulus_cartesian.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x() << " " << bce[i].y() << " " << bce[i].z() << endl;
    }
    fbce.close();
    sysSPH.AddCylinderAnnulusBCE(body, frame, ca_radius_inner, ca_radius_outer, ca_height, false);

    bce.clear();
    sysSPH.CreatePoints_CylinderAnnulus(ca_radius_inner, ca_radius_outer, ca_height, true, spacing, bce);
    cout << "cylinder annulus polar nBCE = " << bce.size() << endl;
    fbce.open((out_dir + "/cyl_annulus_polar.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x() << " " << bce[i].y() << " " << bce[i].z() << endl;
    }
    fbce.close();
    sysSPH.AddCylinderAnnulusBCE(body, frame, ca_radius_inner, ca_radius_outer, ca_height, true);
}
