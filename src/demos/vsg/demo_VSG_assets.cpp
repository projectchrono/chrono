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
// Demonstration of the Chrono::VSG run-time visualization system
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
#include "chrono/assets/ChVisualShapeRoundedBox.h"
#include "chrono/assets/ChVisualShapeRoundedCylinder.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeSurface.h"
#include "chrono/assets/ChVisualShapeModelFile.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::vsg3d;

bool collision_vis = true;
bool lines_and_surfaces = true;
bool particle_clouds = true;
bool convex_hull = true;
bool meshes = true;
bool primitives = true;
bool moving_assets = true;

int main(int argc, char* argv[]) {
    // Create a Chrono system
    ChSystemNSC sys;
    sys.SetGravityY();
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // -------------------------------------------------------------------------

    // Create a floor body with a box collision shape
    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetFixed(true);
    floor->SetPos(ChVector3d(0, -1.5, 0));
    floor->EnableCollision(true);
    sys.Add(floor);

    auto floor_ct_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto floor_ct_shape = chrono_types::make_shared<ChCollisionShapeBox>(floor_ct_mat, 20, 1, 20);
    floor->AddCollisionShape(floor_ct_shape);

    auto floor_vis_shape = chrono_types::make_shared<ChVisualShapeBox>(20, 1, 20);
    floor_vis_shape->SetColor(ChColor(0.2f, 0.3f, 0.6f));
    floor->AddVisualShape(floor_vis_shape);

    // ---- Attach various visual shapes to the floor body ----
    if (lines_and_surfaces) {
        // Attach a path shape populated with segments and arcs
        auto pathfloor = chrono_types::make_shared<ChVisualShapePath>();
        ChLineSegment mseg1(ChVector3d(1, 2, 0), ChVector3d(1, 3, 0));
        pathfloor->GetPathGeometry()->AddSubLine(mseg1);
        ChLineSegment mseg2(ChVector3d(1, 3, 0), ChVector3d(2, 3, 0));
        pathfloor->GetPathGeometry()->AddSubLine(mseg2);
        ChLineArc marc1(ChCoordsys<>(ChVector3d(2, 3.5, 0)), 0.5, -CH_PI_2, CH_PI_2);
        pathfloor->GetPathGeometry()->AddSubLine(marc1);
        pathfloor->SetColor(ChColor(0.0f, 0.5f, 0.8f));
        floor->AddVisualShape(pathfloor, ChFramed(ChVector3d(3, -1, -8), QUNIT));

        // Attach a 'nurbs line' shape: first create the ChLineNurbs geometry, then put it inside a ChVisualShapeLine
        auto nurbs = chrono_types::make_shared<ChLineNurbs>();
        std::vector<ChVector3d> controlpoints = {ChVector3d(1, 2, -1), ChVector3d(1, 3, -1), ChVector3d(1, 3, -2), ChVector3d(1, 4, -2)};
        nurbs->Setup(3, controlpoints);

        auto nurbsasset = chrono_types::make_shared<ChVisualShapeLine>();
        nurbsasset->SetLineGeometry(nurbs);
        nurbsasset->SetColor(ChColor(0.0f, 0.3f, 1.0f));
        floor->AddVisualShape(nurbsasset, ChFramed(ChVector3d(0, -1, -8), QUNIT));

        // Attach a 'nurbs surface' shape: first create the ChSurfaceNurbs geometry, then put it inside a ChVisualShapeSurface
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
        floor->AddVisualShape(surfasset, ChFramed(ChVector3d(3, -1, -8), QUNIT));
    }

    // -------------------------------------------------------------------------

    // Create a fixed rigid body with visualization and collision models
    auto body = chrono_types::make_shared<ChBody>();
    body->SetFixed(true);
    body->EnableCollision(true);
    sys.Add(body);

    // Create shared visualization and contact materials
    auto mint_mat = chrono_types::make_shared<ChVisualMaterial>();
    mint_mat->SetDiffuseColor(ChColor(0.68f, 0.92f, 0.70f));
    //
    ChContactMaterialData ct_mat_data;
    auto ct_mat = ct_mat_data.CreateMaterial(sys.GetContactMethod());

    // Add primitive visualization and collision shapes
    if (collision_vis) {
        auto sphere_vis = chrono_types::make_shared<ChVisualShapeSphere>(0.5);
        sphere_vis->AddMaterial(mint_mat);
        body->AddVisualShape(sphere_vis, ChFramed(ChVector3d(0, 1, 0), QUNIT));
        auto sphere_ct = chrono_types::make_shared<ChCollisionShapeSphere>(ct_mat, 0.5);
        body->AddCollisionShape(sphere_ct, ChFramed(ChVector3d(0, 1, 0), QUNIT));
        //
        auto box_vis = chrono_types::make_shared<ChVisualShapeBox>(0.75, 2.0, 1.5);
        box_vis->AddMaterial(mint_mat);
        body->AddVisualShape(box_vis, ChFramed(ChVector3d(1.5, 1, 0), QUNIT));
        auto box_ct = chrono_types::make_shared<ChCollisionShapeBox>(ct_mat, 0.75, 2.0, 1.5);
        body->AddCollisionShape(box_ct, ChFramed(ChVector3d(1.5, 1, 0), QUNIT));
        //
        auto cyl_vis = chrono_types::make_shared<ChVisualShapeCylinder>(0.5, 2.0);
        cyl_vis->AddMaterial(mint_mat);
        body->AddVisualShape(cyl_vis, ChFramed(ChVector3d(3.0, 1, 0), Q_ROTATE_Y_TO_Z));
        auto cyl_ct = chrono_types::make_shared<ChCollisionShapeCylinder>(ct_mat, 0.5, 2.0);
        body->AddCollisionShape(cyl_ct, ChFramed(ChVector3d(3.0, 1, 0), Q_ROTATE_Y_TO_Z));
        //
        auto rbox_vis = chrono_types::make_shared<ChVisualShapeRoundedBox>(ChVector3d(0.75, 2.0, 1.5), 0.25);
        rbox_vis->SetMaterial(0, mint_mat);
        body->AddVisualShape(rbox_vis, ChFramed(ChVector3d(-1.5, 1, 0), QUNIT));
        auto rbox_ct = chrono_types::make_shared<ChCollisionShapeRoundedBox>(ct_mat, ChVector3d(0.75, 2.0, 1.5), 0.25);
        body->AddCollisionShape(rbox_ct, ChFramed(ChVector3d(-1.5, 1, 0), QUNIT));
        //
        auto rcyl_vis = chrono_types::make_shared<ChVisualShapeRoundedCylinder>(0.5, 2.0, 0.25);
        rcyl_vis->SetMaterial(0, mint_mat);
        body->AddVisualShape(rcyl_vis, ChFramed(ChVector3d(-3.0, 1, 0), Q_ROTATE_Y_TO_Z));
        auto rcyl_ct = chrono_types::make_shared<ChCollisionShapeRoundedCylinder>(ct_mat, 0.5, 2.0, 0.25);
        body->AddCollisionShape(rcyl_ct, ChFramed(ChVector3d(-3.0, 1, 0), Q_ROTATE_Y_TO_Z));
    }

    if (meshes) {
        // Attach three instances of the same 'triangle mesh' shape
        auto mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        mesh->GetMesh()->GetCoordsVertices().push_back(ChVector3d(0, 0, 0));
        mesh->GetMesh()->GetCoordsVertices().push_back(ChVector3d(0, 1, 0));
        mesh->GetMesh()->GetCoordsVertices().push_back(ChVector3d(1, 0, 0));
        mesh->GetMesh()->GetIndicesVertices().push_back(ChVector3i(0, 1, 2));
        mesh->AddMaterial(mint_mat);

        body->AddVisualShape(mesh, ChFramed(ChVector3d(2, 0, -2), QUNIT));
        body->AddVisualShape(mesh, ChFramed(ChVector3d(3, 0, -2), QUNIT));
        body->AddVisualShape(mesh, ChFramed(ChVector3d(2, 1, -2), QUNIT));

        // Attach a 'Wavefront mesh' asset, referencing a .obj file and offset it.
        // Only the first call of a distinct filename loads from disc; uses instancing.
        auto objmesh = chrono_types::make_shared<ChVisualShapeModelFile>();
        objmesh->SetFilename(GetChronoDataFile("models/forklift/body.obj"));

        body->AddVisualShape(objmesh, ChFramed(ChVector3d(5, 2, -3)));
        body->AddVisualShape(objmesh, ChFramed(ChVector3d(6, 2, -4), QuatFromAngleY(CH_PI_2)));
        body->AddVisualShape(objmesh, ChFramed(ChVector3d(5, 2, -5), QuatFromAngleY(CH_PI)));
    }

    // Attach an array of boxes, each rotated to make a spiral
    for (int j = 0; j < 20; j++) {
        auto smallbox = chrono_types::make_shared<ChVisualShapeBox>(0.2, 0.2, 0.02);
        smallbox->SetColor(ChColor(j * 0.05f, 1 - j * 0.05f, 0.0f));
        ChMatrix33<> rot(QuatFromAngleY(j * 21 * CH_DEG_TO_RAD));
        ChVector3d pos = rot * ChVector3d(0.4, 0, 0) + ChVector3d(0, j * 0.02, 0);
        body->AddVisualShape(smallbox, ChFramed(pos, rot));
    }

    // -------------------------------------------------------------------------

    // Create ChParticleCloud clusters, and attach 'assets' that define a single "sample" 3D shape.
    if (particle_clouds) {
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

            auto particle_vis = chrono_types::make_shared<ChVisualShapeCapsule>(0.1, 0.1);
            particle_vis->SetColor(ChColor(0.3f, 0.3f, 0.7f));
            particles->AddVisualShape(particle_vis);

            for (int np = 0; np < 40; ++np)
                particles->AddParticle(ChCoordsys<>(ChVector3d(4 * ChRandom::Get() - 6, 3 * ChRandom::Get() + 2, 4 * ChRandom::Get() - 6)));

            particles->RegisterColorCallback(chrono_types::make_shared<MyColorCallback>(2, 5));

            sys.Add(particles);
        }
    }

    // -------------------------------------------------------------------------

    // Create a convex hull shape

    if (convex_hull) {
        std::vector<ChVector3d> points;
        points.push_back(ChVector3d(1.6, 0.0, 0.0));
        points.push_back(ChVector3d(1.6, 0.6, 0.0));
        points.push_back(ChVector3d(1.6, 0.6, 0.6));
        points.push_back(ChVector3d(0.0, 0.6, 0.6));
        points.push_back(ChVector3d(0.0, 0.0, 0.6));
        points.push_back(ChVector3d(1.6, 0.0, 0.6));

        auto hull = chrono_types::make_shared<ChBodyEasyConvexHullAuxRef>(points, 1000, true, true, chrono_types::make_shared<ChContactMaterialNSC>());
        hull->SetFrameRefToAbs(ChFramed(ChVector3d(-4, 0, -4)));

        // Set a transparent visualization material
        auto cadet_blue = chrono_types::make_shared<ChVisualMaterial>();
        cadet_blue->SetDiffuseColor(ChColor(0.37f, 0.62f, 0.62f));
        hull->GetVisualShape(0)->SetMaterial(0, cadet_blue);
        hull->GetVisualShape(0)->GetMaterial(0)->SetOpacity(0.5);

        sys.Add(hull);
    }

    // -------------------------------------------------------------------------

    // Set azimuth and elevation of directional light
    double azimuth = 135 * CH_DEG_TO_RAD;
    double elevation = 45 * CH_DEG_TO_RAD;

    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();

    vis->AttachSystem(&sys);

    vis->SetWindowSize(ChVector2i(1200, 800));
    vis->SetWindowPosition(ChVector2i(100, 300));
    vis->SetWindowTitle("Chrono VSG Assets");
    vis->EnableSkyTexture(SkyMode::DOME);

    vis->SetCameraVertical(CameraVerticalDir::Y);
    vis->AddCamera(ChVector3d(-13, 7, -16));
    vis->SetCameraAngleDeg(40);

    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(azimuth, elevation);

    vis->SetCollisionVisibility(true);

    vis->AddGrid(0.5, 0.5, 40, 40, ChCoordsys<>(ChVector3d(0, 0, 0), QuatFromAngleX(CH_PI_2)), ChColor(0.3f, 0.3f, 0.5f));

    // -------------------------------------------------------------------------

    // Add scenery objects, not bound to bodies

    int teapotId1 = -1;
    int teapotId2 = -1;
    int bunnyId = -1;
    auto Zup = QuatFromAngleX(-CH_PI_2);

    if (moving_assets) {
        auto sceneMesh1 = chrono_types::make_shared<ChVisualShapeModelFile>();
        sceneMesh1->SetFilename(GetChronoDataFile("models/red_teapot.obj"));
        teapotId1 = vis->AddVisualModel(sceneMesh1, ChFramed(ChVector3d(0, 3.5, 3), Zup));
        if (teapotId1 == -1)
            std::cerr << "Could not get teapot!" << std::endl;
        teapotId2 = vis->AddVisualModel(sceneMesh1, ChFramed(ChVector3d(-5, 3.5, 3), Zup));
        if (teapotId2 == -1)
            std::cerr << "Could not get teapot!" << std::endl;

        auto sceneMesh2 = chrono_types::make_shared<ChVisualShapeModelFile>();
        sceneMesh2->SetFilename(GetChronoDataFile("models/bunny.glb"));
        bunnyId = vis->AddVisualModel(sceneMesh2, ChFramed(ChVector3d(-2, 0, -8), QuatFromAngleY(CH_PI)));
        if (bunnyId == -1)
            std::cerr << "Could not get bunny!" << std::endl;
    }

    if (primitives) {
        // Primitive shapes with checker textures
        auto marker_model = chrono_types::make_shared<ChVisualModel>();
        auto marker_shape = chrono_types::make_shared<ChVisualShapeSphere>(0.05);
        marker_shape->SetColor({1.0f, 0.5f, 0.0f});
        marker_model->AddShape(marker_shape);

        std::vector<ChVector3d> shape_pos = {ChVector3d(-5.5, 1, -5), ChVector3d(-5.5, 1, 0), ChVector3d(-5.5, 1, +5),  //
                                             ChVector3d(-7.5, 1, -5), ChVector3d(-7.5, 1, 0), ChVector3d(-7.5, 1, +5),  //
                                             ChVector3d(-9.5, 1, -5), ChVector3d(-9.5, 1, 0), ChVector3d(-9.5, 1, +5)};

        auto box_mat = chrono_types::make_shared<ChVisualMaterial>();
        box_mat->SetKdTexture(GetChronoDataFile("textures/checker1.png"));
        auto boxShape = chrono_types::make_shared<ChVisualShapeBox>(1.0, 1.5, 2.0);
        boxShape->AddMaterial(box_mat);
        vis->AddVisualModel(boxShape, ChFramed(shape_pos[0], QUNIT));
        vis->AddVisualModel(marker_model, shape_pos[0] - ChVector3d(0, 0, 1.0));
        vis->AddVisualModel(marker_model, shape_pos[0] + ChVector3d(0, 0, 1.0));

        auto cyl_mat = chrono_types::make_shared<ChVisualMaterial>();
        cyl_mat->SetKdTexture(GetChronoDataFile("textures/checker2.png"));
        auto cylShape = chrono_types::make_shared<ChVisualShapeCylinder>(0.5, 2.0);
        cylShape->SetMaterial(0, cyl_mat);
        vis->AddVisualModel(cylShape, ChFramed(shape_pos[1], QUNIT));
        vis->AddVisualModel(marker_model, shape_pos[1] - ChVector3d(0, 0, 1.0));
        vis->AddVisualModel(marker_model, shape_pos[1] + ChVector3d(0, 0, 1.0));

        auto sphere_mat = chrono_types::make_shared<ChVisualMaterial>();
        sphere_mat->SetKdTexture(GetChronoDataFile("textures/checker1.png"));
        auto sphereShape = chrono_types::make_shared<ChVisualShapeSphere>(0.75);
        sphereShape->SetMaterial(0, sphere_mat);
        vis->AddVisualModel(sphereShape, ChFramed(shape_pos[2], QUNIT));
        vis->AddVisualModel(marker_model, shape_pos[2] - ChVector3d(0, 0, 0.75));
        vis->AddVisualModel(marker_model, shape_pos[2] + ChVector3d(0, 0, 0.75));

        auto rbox_mat = chrono_types::make_shared<ChVisualMaterial>();
        rbox_mat->SetKdTexture(GetChronoDataFile("textures/checker2.png"));
        auto rboxShape = chrono_types::make_shared<ChVisualShapeRoundedBox>(ChVector3d(1.0, 1.5, 2.0), 0.25);
        rboxShape->SetMaterial(0, rbox_mat);
        vis->AddVisualModel(rboxShape, ChFramed(shape_pos[3], QUNIT));
        vis->AddVisualModel(marker_model, shape_pos[3] - ChVector3d(0, 0, 1.0));
        vis->AddVisualModel(marker_model, shape_pos[3] + ChVector3d(0, 0, 1.0));

        auto rcyl_mat = chrono_types::make_shared<ChVisualMaterial>();
        rcyl_mat->SetKdTexture(GetChronoDataFile("textures/checker1.png"));
        auto rcylShape = chrono_types::make_shared<ChVisualShapeRoundedCylinder>(0.5, 2.0, 0.25);
        rcylShape->SetMaterial(0, rcyl_mat);
        vis->AddVisualModel(rcylShape, ChFramed(shape_pos[4], QUNIT));
        vis->AddVisualModel(marker_model, shape_pos[4] - ChVector3d(0, 0, 1.0));
        vis->AddVisualModel(marker_model, shape_pos[4] + ChVector3d(0, 0, 1.0));

        auto ell_mat = chrono_types::make_shared<ChVisualMaterial>();
        ell_mat->SetKdTexture(GetChronoDataFile("textures/checker2.png"));
        auto ellShape = chrono_types::make_shared<ChVisualShapeEllipsoid>(1.0, 1.5, 2.0);
        ellShape->SetMaterial(0, ell_mat);
        vis->AddVisualModel(ellShape, ChFramed(shape_pos[5], QUNIT));
        vis->AddVisualModel(marker_model, shape_pos[5] - ChVector3d(0, 0, 1.0));
        vis->AddVisualModel(marker_model, shape_pos[5] + ChVector3d(0, 0, 1.0));

        auto die_mat = chrono_types::make_shared<ChVisualMaterial>();
        die_mat->SetKdTexture(GetChronoDataFile("textures/cubetexture_wood.png"));
        auto dieShape = chrono_types::make_shared<ChVisualShapeBox>(1.0, 1.5, 2.0);
        dieShape->SetMaterial(0, die_mat);
        vis->AddVisualModel(dieShape, ChFramed(shape_pos[6], QUNIT));
        vis->AddVisualModel(marker_model, shape_pos[6] - ChVector3d(0, 0, 1.0));
        vis->AddVisualModel(marker_model, shape_pos[6] + ChVector3d(0, 0, 1.0));

        auto caps_mat = chrono_types::make_shared<ChVisualMaterial>();
        caps_mat->SetKdTexture(GetChronoDataFile("textures/checker2.png"));
        auto capsShape = chrono_types::make_shared<ChVisualShapeCapsule>(0.5, 1.0);
        capsShape->SetMaterial(0, caps_mat);
        vis->AddVisualModel(capsShape, ChFramed(shape_pos[7], QUNIT));
        vis->AddVisualModel(marker_model, shape_pos[7] - ChVector3d(0, 0, 1.0));
        vis->AddVisualModel(marker_model, shape_pos[7] + ChVector3d(0, 0, 1.0));

        auto cone_mat = chrono_types::make_shared<ChVisualMaterial>();
        cone_mat->SetKdTexture(GetChronoDataFile("textures/checker1.png"));
        auto coneShape = chrono_types::make_shared<ChVisualShapeCone>(0.5, 2.0);
        coneShape->SetMaterial(0, cone_mat);
        vis->AddVisualModel(coneShape, ChFramed(shape_pos[8], QUNIT));
        vis->AddVisualModel(marker_model, shape_pos[8] - ChVector3d(0, 0, 1.0));
        vis->AddVisualModel(marker_model, shape_pos[8] + ChVector3d(0, 0, 1.0));
    }

    // -------------------------------------------------------------------------

    // Initialize run-time visualization system
    vis->EnableShadows();
    vis->Initialize();

    // Create output directory
    const std::string out_dir = GetChronoOutputPath() + "VSG_ASSETS";
    if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    ChRealtimeStepTimer rt;
    double step_size = 0.01;
    unsigned int frame_number = 0;

    while (vis->Run()) {
        ////if (frame_number > 2 && frame_number <= 100) {
        ////    std::string imgName("/assets-");
        ////    imgName.append(std::to_string(frame_number) + ".png");
        ////    vis->WriteImageToFile(out_dir + imgName);  // does not work with frame == 0!
        ////}

        double time = sys.GetChTime();
        if (teapotId1 >= 0)
            vis->UpdateVisualModel(teapotId1, ChFramed(ChVector3d(0, 3.5 + 0.5 * std::sin(CH_PI * time / 10), 3), Zup));
        if (teapotId2 >= 0)
            vis->UpdateVisualModel(teapotId2, ChFramed(ChVector3d(-5, 3.5, 3), Zup * QuatFromAngleY(time / 20)));

        vis->Render();
        sys.DoStepDynamics(step_size);

        rt.Spin(step_size);

        frame_number++;
    }

    return 0;
}
