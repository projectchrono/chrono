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
// Demosntration of the Chrono::VSG run-time visualization system
//
// =============================================================================

#include <cmath>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChParticleCloud.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChRandom.h"
#include "chrono/geometry/ChLineNurbs.h"
#include "chrono/geometry/ChSurfaceNurbs.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapePath.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeEllipsoid.h"
#include "chrono/assets/ChVisualShapeCone.h"
#include "chrono/assets/ChVisualShapeCapsule.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeSurface.h"
#include "chrono/assets/ChVisualShapeModelFile.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

#include "chrono_thirdparty/filesystem/path.h"

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::vsg3d;

int main(int argc, char* argv[]) {
    // Create a Chrono system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // EXAMPLE 1:

    // Create a ChBody, and attach assets that define 3D shapes for visualization purposes.
    // Note: these assets are independent from collision shapes!

    // Create a rigid body and add it to the physical system:
    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetFixed(true);

    // Contact material
    auto floor_mat = chrono_types::make_shared<ChContactMaterialNSC>();

    // Define a collision shape
    auto floor_shape = chrono_types::make_shared<ChCollisionShapeBox>(floor_mat, 20, 1, 20);
    floor->AddCollisionShape(floor_shape, ChFrame<>(ChVector3d(0, -1, 0), QUNIT));
    floor->EnableCollision(true);

    // Add body to system
    sys.Add(floor);

    // Attach a box shape
    // Note that assets are managed via shared pointer, so they can also be shared).
    auto boxfloor = chrono_types::make_shared<ChVisualShapeBox>(20, 1, 20);
    boxfloor->SetColor(ChColor(0.2f, 0.3f, 1.0f));
    floor->AddVisualShape(boxfloor, ChFrame<>(ChVector3d(0, -1, 0), QUNIT));

    // Attach a path shape populated with segments and arcs.
    auto pathfloor = chrono_types::make_shared<ChVisualShapePath>();
    ChLineSegment mseg1(ChVector3d(1, 2, 0), ChVector3d(1, 3, 0));
    pathfloor->GetPathGeometry()->AddSubLine(mseg1);
    ChLineSegment mseg2(ChVector3d(1, 3, 0), ChVector3d(2, 3, 0));
    pathfloor->GetPathGeometry()->AddSubLine(mseg2);
    ChLineArc marc1(ChCoordsys<>(ChVector3d(2, 3.5, 0)), 0.5, -CH_PI_2, CH_PI_2);
    pathfloor->GetPathGeometry()->AddSubLine(marc1);
    pathfloor->SetColor(ChColor(0.0f, 0.5f, 0.8f));
    floor->AddVisualShape(pathfloor);

    // Attach a 'nurbs line' shape:
    // First create the ChLineNurbs geometry, then put it inside a ChVisualShapeLine
    auto nurbs = chrono_types::make_shared<ChLineNurbs>();
    std::vector<ChVector3d> controlpoints = {ChVector3d(1, 2, -1), ChVector3d(1, 3, -1), ChVector3d(1, 3, -2),
                                             ChVector3d(1, 4, -2)};
    nurbs->Setup(3, controlpoints);

    auto nurbsasset = chrono_types::make_shared<ChVisualShapeLine>();
    nurbsasset->SetLineGeometry(nurbs);
    nurbsasset->SetColor(ChColor(0.0f, 0.3f, 1.0f));
    floor->AddVisualShape(nurbsasset);

    // Attach a 'nurbs surface' shape:
    // First create the ChSurfaceNurbs geometry, then put it inside a ChVisualShapeSurface
    auto surf = chrono_types::make_shared<ChSurfaceNurbs>();
    ChMatrixDynamic<ChVector3d> surfpoints(4, 2);  // u points, v points
    surfpoints(0, 0) = ChVector3d(1, 2, 3);
    surfpoints(1, 0) = ChVector3d(1, 3, 3);
    surfpoints(2, 0) = ChVector3d(2, 3, 3);
    surfpoints(3, 0) = ChVector3d(2, 4, 3);
    surfpoints(0, 1) = ChVector3d(1, 2, 1);
    surfpoints(1, 1) = ChVector3d(1, 3, 1);
    surfpoints(2, 1) = ChVector3d(3, 3, 1);
    surfpoints(3, 1) = ChVector3d(2, 4, 1);
    surf->Setup(3, 1, surfpoints);

    auto surfasset = chrono_types::make_shared<ChVisualShapeSurface>();
    surfasset->SetSurfaceGeometry(surf);
    surfasset->SetWireframe(true);
    surfasset->SetColor(ChColor(0.2f, 0.8f, 0.3f));
    floor->AddVisualShape(surfasset, ChFrame<>(ChVector3d(3, -1, 3), QUNIT));

    // EXAMPLE 2:

    // Create the rigid body (this won't move, it is only for visualization tests)
    auto body = chrono_types::make_shared<ChBody>();
    body->SetFixed(true);
    sys.Add(body);

    // Create a shared visualization material
    auto orange_mat = chrono_types::make_shared<ChVisualMaterial>();
    orange_mat->SetDiffuseColor(ChColor(0.9f, 0.4f, 0.2f));

    // Attach a 'sphere' shape
    auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(0.5);
    sphere->AddMaterial(orange_mat);
    body->AddVisualShape(sphere, ChFrame<>(ChVector3d(-1, 0, 0), QUNIT));

    // Attach also a 'box' shape
    auto box = chrono_types::make_shared<ChVisualShapeBox>(0.6, 1.0, 0.2);
    box->AddMaterial(orange_mat);
    body->AddVisualShape(box, ChFrame<>(ChVector3d(1, 1, 0), QUNIT));

    // Attach also a 'cylinder' shape
    auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.3, 0.7);
    cyl->AddMaterial(orange_mat);
    body->AddVisualShape(cyl, ChFrame<>(ChVector3d(2, 0.15, 0), QuatFromAngleX(CH_PI_2)));
    body->AddVisualShape(chrono_types::make_shared<ChVisualShapeSphere>(0.03),
                         ChFrame<>(ChVector3d(2, -0.2, 0), QUNIT));
    body->AddVisualShape(chrono_types::make_shared<ChVisualShapeSphere>(0.03),
                         ChFrame<>(ChVector3d(2, +0.5, 0), QUNIT));

    // Attach three instances of the same 'triangle mesh' shape
    auto mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    mesh->GetMesh()->GetCoordsVertices().push_back(ChVector3d(0, 0, 0));
    mesh->GetMesh()->GetCoordsVertices().push_back(ChVector3d(0, 1, 0));
    mesh->GetMesh()->GetCoordsVertices().push_back(ChVector3d(1, 0, 0));
    mesh->GetMesh()->GetIndicesVertexes().push_back(ChVector3i(0, 1, 2));
    mesh->AddMaterial(orange_mat);

    body->AddVisualShape(mesh, ChFrame<>(ChVector3d(2, 0, 2), QUNIT));
    body->AddVisualShape(mesh, ChFrame<>(ChVector3d(3, 0, 2), QUNIT));
    body->AddVisualShape(mesh, ChFrame<>(ChVector3d(2, 1, 2), QUNIT));

    // Attach a 'Wavefront mesh' asset, referencing a .obj file and offset it.
    // Only the first call of a distinct filename loads from disc; uses instancing.
    auto objmesh = chrono_types::make_shared<ChVisualShapeModelFile>();
    objmesh->SetFilename(GetChronoDataFile("models/forklift/body.obj"));

    body->AddVisualShape(objmesh, ChFrame<>(ChVector3d(0, 0.0, 2)));
    body->AddVisualShape(objmesh, ChFrame<>(ChVector3d(3, 0.0, 2.5)));
    body->AddVisualShape(objmesh, ChFrame<>(ChVector3d(5, 0.0, 3)));
    body->AddVisualShape(objmesh, ChFrame<>(ChVector3d(4, 0.0, -3), QuatFromAngleY(0.5 * CH_PI)));
    body->AddVisualShape(objmesh, ChFrame<>(ChVector3d(0, 0.0, -5), QuatFromAngleY(CH_PI)));
    body->AddVisualShape(objmesh, ChFrame<>(ChVector3d(-4, 0.0, -6), QuatFromAngleY(-CH_PI_4)));

    // Attach an array of boxes, each rotated to make a spiral
    for (int j = 0; j < 20; j++) {
        auto smallbox = chrono_types::make_shared<ChVisualShapeBox>(0.2, 0.2, 0.02);
        smallbox->SetColor(ChColor(j * 0.05f, 1 - j * 0.05f, 0.0f));
        ChMatrix33<> rot(QuatFromAngleY(j * 21 * CH_DEG_TO_RAD));
        ChVector3d pos = rot * ChVector3d(0.4, 0, 0) + ChVector3d(0, j * 0.02, 0);
        body->AddVisualShape(smallbox, ChFrame<>(pos, rot));
    }

    // EXAMPLE 3:

    // Create ChParticleCloud clusters, and attach 'assets' that define a single "sample" 3D shape.
    {
        // 1st cloud - moving spheres with collision
        auto particles = chrono_types::make_shared<ChParticleCloud>();
        particles->SetMass(0.1);
        particles->SetInertiaXX(ChVector3d(0.001, 0.001, 0.001));

        double particle_radius = 0.3;
        auto particle_vis = chrono_types::make_shared<ChVisualShapeSphere>(particle_radius);
        particle_vis->SetColor(ChColor(0.7f, 0.3f, 0.3f));
        particles->AddVisualShape(particle_vis);

        auto particle_mat = chrono_types::make_shared<ChContactMaterialNSC>();
        particle_mat->SetFriction(0.9f);

        auto particle_shape = chrono_types::make_shared<ChCollisionShapeSphere>(particle_mat, particle_radius);
        particles->AddCollisionShape(particle_shape);
        particles->EnableCollision(true);

        for (int np = 0; np < 60; ++np)
            particles->AddParticle(ChCoordsys<>(ChVector3d(2 * ChRandom::Get() - 2, 1.5, 2 * ChRandom::Get() + 2)));

        sys.Add(particles);
    }
    {
        // 2nd cloud - moving boxes with collision
        auto particles = chrono_types::make_shared<ChParticleCloud>();
        particles->SetMass(0.1);
        particles->SetInertiaXX(ChVector3d(0.001, 0.001, 0.001));

        double size_x = 0.3;
        double size_y = 0.2;
        double size_z = 0.7;
        auto particle_vis = chrono_types::make_shared<ChVisualShapeBox>(size_x, size_y, size_z);
        particle_vis->SetColor(ChColor(0.3f, 0.7f, 0.3f));
        particles->AddVisualShape(particle_vis);

        auto particle_mat = chrono_types::make_shared<ChContactMaterialNSC>();

        auto particle_shape = chrono_types::make_shared<ChCollisionShapeBox>(particle_mat, size_x, size_y, size_z);
        particles->AddCollisionShape(particle_shape);
        particles->EnableCollision(true);

        for (int np = 0; np < 30; ++np)
            particles->AddParticle(ChCoordsys<>(ChVector3d(2 * ChRandom::Get() + 4, 1.5, 2 * ChRandom::Get() - 4)));

        sys.Add(particles);
    }
    {
        // 3rd cloud - fixed capsules with color coding
        class MyColorCallback : public ChParticleCloud::ColorCallback {
          public:
            MyColorCallback(double hmin, double hmax) : m_hmin(hmin), m_hmax(hmax) {}
            virtual ChColor get(unsigned int n, const ChParticleCloud& cloud) const override {
                double height = cloud.GetParticlePos(n).y();
                double col = (height - m_hmin) / (m_hmax - m_hmin);
                return ChColor((float)col, (float)col, 0);
            }

          private:
            double m_hmin;
            double m_hmax;
        };

        auto particles = chrono_types::make_shared<ChParticleCloud>();
        particles->SetMass(0.1);
        particles->SetInertiaXX(ChVector3d(0.001, 0.001, 0.001));
        particles->SetFixed(true);
        particles->EnableCollision(false);

        auto particle_vis = chrono_types::make_shared<ChVisualShapeCapsule>(0.2, 0.2);
        particle_vis->SetColor(ChColor(0.3f, 0.3f, 0.7f));
        particles->AddVisualShape(particle_vis);

        for (int np = 0; np < 40; ++np)
            particles->AddParticle(
                ChCoordsys<>(ChVector3d(4 * ChRandom::Get() - 6, 3 * ChRandom::Get() + 2, 4 * ChRandom::Get() - 6)));

        particles->RegisterColorCallback(chrono_types::make_shared<MyColorCallback>(2, 5));

        sys.Add(particles);
    }

    // EXAMPLE 4:

    // Create a convex hull shape

    ChVector3d displ(1, 0.0, 0);
    std::vector<ChVector3d> points;
    points.push_back(ChVector3d(0.8, 0.0, 0.0) + displ);
    points.push_back(ChVector3d(0.8, 0.3, 0.0) + displ);
    points.push_back(ChVector3d(0.8, 0.3, 0.3) + displ);
    points.push_back(ChVector3d(0.0, 0.3, 0.3) + displ);
    points.push_back(ChVector3d(0.0, 0.0, 0.3) + displ);
    points.push_back(ChVector3d(0.8, 0.0, 0.3) + displ);
    auto hull = chrono_types::make_shared<ChBodyEasyConvexHullAuxRef>(
        points, 1000, true, true, chrono_types::make_shared<ChContactMaterialNSC>());
    ////hull->SetFrameRefToAbs(ChFrame<>(ChVector3d(2,0.3,0)));
    ////hull->SetPos(ChVector3d(2,0.3,0));
    hull->Move(ChVector3d(2, 0.3, 0));

    // Create a visualization material
    auto cadet_blue = chrono_types::make_shared<ChVisualMaterial>();
    cadet_blue->SetDiffuseColor(ChColor(0.37f, 0.62f, 0.62f));
    hull->GetVisualShape(0)->SetMaterial(0, cadet_blue);
    hull->GetVisualShape(0)->GetMaterial(0)->SetOpacity(0.5);  // DepthSorted???
    sys.Add(hull);

    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetCameraVertical(CameraVerticalDir::Y);
    vis->SetWindowSize(ChVector2i(1200, 800));
    vis->SetWindowPosition(ChVector2i(100, 300));
    vis->SetWindowTitle("Chrono VSG Assets");
    vis->SetUseSkyBox(true);
    vis->AddCamera(ChVector3d(-8, 8, -16));
    vis->SetCameraAngleDeg(40);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->AddGrid(0.5, 0.5, 12, 12, ChCoordsys<>(ChVector3d(0, -0.49, 0), QuatFromAngleX(CH_PI_2)),
                 ChColor(0.31f, 0.43f, 0.43f));

    // add scenery objects, not bound to bodies
    auto Zup = QuatFromAngleX(-CH_PI_2);

    auto sceneMesh1 = chrono_types::make_shared<ChVisualShapeModelFile>();
    sceneMesh1->SetFilename(GetChronoDataFile("models/red_teapot.obj"));
    int teapotId1 = vis->AddVisualModel(sceneMesh1, ChFrame<>(ChVector3d(0, 3.5, 3), Zup));
    if (teapotId1 == -1)
        std::cerr << "Could not get teapot!" << std::endl;
    int teapotId2 = vis->AddVisualModel(sceneMesh1, ChFrame<>(ChVector3d(-5, 3.5, 3), Zup));
    if (teapotId2 == -1)
        std::cerr << "Could not get teapot!" << std::endl;

    auto sceneMesh2 = chrono_types::make_shared<ChVisualShapeModelFile>();
    sceneMesh2->SetFilename(GetChronoDataFile("models/bunny.glb"));
    int bunndyId = vis->AddVisualModel(sceneMesh2, ChFrame<>(ChVector3d(-5, 0, 5)));
    if (bunndyId == -1)
        std::cerr << "Could not get bunny!" << std::endl;

    auto boxShape = chrono_types::make_shared<ChVisualShapeBox>(0.6, 5.0, 0.2);
    boxShape->AddMaterial(orange_mat);
    int boxId = vis->AddVisualModel(boxShape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));

    // Create a shared visualization material
    auto sphere_mat = chrono_types::make_shared<ChVisualMaterial>();
    sphere_mat->SetDiffuseColor(ChColor(0.9f, 0.9f, 0.9f));
    sphere_mat->SetKdTexture(GetChronoDataFile("textures/spheretexture.png"));
    auto sphereShape = chrono_types::make_shared<ChVisualShapeSphere>(0.75);
    sphereShape->SetMaterial(0, sphere_mat);
    int sphereId = vis->AddVisualModel(sphereShape, ChFrame<>(ChVector3d(-6, 2, -6), QUNIT));

    auto ell_mat = chrono_types::make_shared<ChVisualMaterial>();
    ell_mat->SetKdTexture(GetChronoDataFile("textures/concrete.jpg"));
    auto ellShape = chrono_types::make_shared<ChVisualShapeEllipsoid>(0.2, 0.2, 0.6);
    ellShape->SetMaterial(0, ell_mat);
    ChVector3d ellPos(-1, 1, -1);
    int ellId = vis->AddVisualModel(ellShape, ChFrame<>(ellPos, QUNIT));

    auto caps_mat = chrono_types::make_shared<ChVisualMaterial>();
    caps_mat->SetDiffuseColor(ChColor(0.8f, 0.5f, 0.2f));
    auto capsShape = chrono_types::make_shared<ChVisualShapeCapsule>(0.5, 2);
    capsShape->SetMaterial(0, caps_mat);
    vis->AddVisualModel(capsShape, ChFrame<>(ChVector3d(-6, 1, -1), QUNIT));
    body->AddVisualShape(chrono_types::make_shared<ChVisualShapeSphere>(0.03),
                         ChFrame<>(ChVector3d(-6, 1, -2.5), QUNIT));
    body->AddVisualShape(chrono_types::make_shared<ChVisualShapeSphere>(0.03),
                         ChFrame<>(ChVector3d(-6, 1, +0.5), QUNIT));

    auto cone_mat = chrono_types::make_shared<ChVisualMaterial>();
    cone_mat->SetKdTexture(GetChronoDataFile("textures/pinkwhite.png"));
    auto coneShape = chrono_types::make_shared<ChVisualShapeCone>(0.5, 2.0);
    coneShape->SetMaterial(0, cone_mat);
    vis->AddVisualModel(coneShape, ChFrame<>(ChVector3d(-6, 1, 4), QUNIT));
    vis->AddVisualModel(chrono_types::make_shared<ChVisualShapeSphere>(0.03), ChFrame<>(ChVector3d(-6, 1, 4), QUNIT));
    vis->AddVisualModel(chrono_types::make_shared<ChVisualShapeSphere>(0.03), ChFrame<>(ChVector3d(-6, 1, 6), QUNIT));

    auto cyl_mat = chrono_types::make_shared<ChVisualMaterial>();
    cyl_mat->SetKdTexture(GetChronoDataFile("textures/pinkwhite.png"));
    auto cylShape = chrono_types::make_shared<ChVisualShapeCylinder>(0.2, 0.8);
    cylShape->SetMaterial(0, cyl_mat);
    vis->AddVisualModel(cylShape, ChFrame<>(ChVector3d(-6, 1, -5), QUNIT));
    vis->AddVisualModel(chrono_types::make_shared<ChVisualShapeSphere>(0.03),
                        ChFrame<>(ChVector3d(-6, 1, -5 - 0.4), QUNIT));
    vis->AddVisualModel(chrono_types::make_shared<ChVisualShapeSphere>(0.03),
                        ChFrame<>(ChVector3d(-6, 1, -5 + 0.4), QUNIT));
    vis->SetShadows(true);
    vis->SetLogoVisible(true);
    vis->Initialize();

    // Create output directory
    const std::string out_dir = GetChronoOutputPath() + "VSG_ASSETS";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    ChRealtimeStepTimer rt;
    double step_size = 0.01;
    unsigned int frame_number = 0;
    while (vis->Run()) {
        double time = sys.GetChTime();
        if (frame_number > 2) {
            std::string imgName("/assets-");
            imgName.append(std::to_string(frame_number) + ".png");
            vis->WriteImageToFile(out_dir + imgName);  // does not work with frame == 0!
        }

        vis->UpdateVisualModel(teapotId1, ChFrame<>(ChVector3d(0, 3.5 + 0.5 * std::sin(CH_PI * time / 10), 3), Zup));
        vis->UpdateVisualModel(teapotId2, ChFrame<>(ChVector3d(-5, 3.5, 3), Zup * QuatFromAngleY(time / 20)));
        vis->UpdateVisualModel(boxId, ChFrame<>(ChVector3d(0, 0.01 * time, 0), QUNIT));
        vis->UpdateVisualModel(ellId, ChFrame<>(ellPos, Zup * QuatFromAngleY(0.2 * time) * QuatFromAngleZ(0.1 * time)));

        if (time < 10.0)
            vis->UpdateVisualModel(sphereId, ChFrame<>(ChVector3d(6, 2, 6), QUNIT));
        else if (time >= 10.0 && time < 20.0)
            vis->UpdateVisualModel(sphereId, ChFrame<>(ChVector3d(-6, 2, 6), QUNIT));
        else if (time >= 20.0 && time < 30.0)
            vis->UpdateVisualModel(sphereId, ChFrame<>(ChVector3d(-6, 2, -6), QUNIT));
        else if (time >= 30.0)
            vis->UpdateVisualModel(sphereId, ChFrame<>(ChVector3d(6, 2, -6), QUNIT));

        vis->Render();
        sys.DoStepDynamics(step_size);

        rt.Spin(step_size);
        if (frame_number == 100)
            break;
        frame_number++;
    }

    return 0;
}
