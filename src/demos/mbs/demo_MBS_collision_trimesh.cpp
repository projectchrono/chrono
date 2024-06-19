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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// Demo code about collisions of triangle meshes
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/core/ChRandom.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// -----------------------------------------------------------------------------

ChCollisionSystem::Type coll_sys_type = ChCollisionSystem::Type::BULLET;
bool draw_coll_shapes = false;

// -----------------------------------------------------------------------------

// Drawer for collision shapes
class DebugDrawer : public ChCollisionSystem::VisualizationCallback {
  public:
    explicit DebugDrawer(ChVisualSystemIrrlicht* vis) : m_vis(vis) {}
    ~DebugDrawer() {}

    virtual void DrawLine(const ChVector3d& from, const ChVector3d& to, const ChColor& color) override {
        m_vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(from), irr::core::vector3dfCH(to),
                                            irr::video::SColor(255, 55, 55, 255));
    }

    virtual double GetNormalScale() const override { return 1.0; }

    void Draw(int flags, bool use_zbuffer = true) {
        m_vis->GetVideoDriver()->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
        irr::video::SMaterial mattransp;
        mattransp.ZBuffer = use_zbuffer;
        mattransp.Lighting = false;
        m_vis->GetVideoDriver()->setMaterial(mattransp);

        m_vis->GetSystem(0).GetCollisionSystem()->Visualize(flags);
    }

  private:
    ChVisualSystemIrrlicht* m_vis;
};

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    ChRandom::SetSeed(0);

    // Create a Chrono physical system
    ChSystemNSC sys;

    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    ChCollisionModel::SetDefaultSuggestedMargin(0.0025);
    sys.SetCollisionSystemType(coll_sys_type);

    // - Create a floor

    auto floor_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto floor = chrono_types::make_shared<ChBodyEasyBox>(5, 2, 5, 1000, true, true, floor_mat);
    floor->SetPos(ChVector3d(0, -1, 0));
    floor->SetFixed(true);
    floor->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/pinkwhite.png"), 50, 50);
    sys.Add(floor);

    // - Create a falling item with triangle mesh shape

    // Shared contact material for all meshes
    auto mesh_mat = chrono_types::make_shared<ChContactMaterialNSC>();

    // Note: one can create easily a colliding shape using the following
    // piece of code:
    //
    // auto falling = chrono_types::make_shared<ChBodyEasyMesh>(
    //	  GetChronoDataFile("models/bulldozer/shoe_view.obj"),  // file name for OBJ Wavefront mesh
    //	  1000,                                                 // density of the body
    //	  true,			                                        // automatic evaluation of mass, COG position, inertia
    // tensor
    //    true,                                                 // attach visualization asset
    //	  true,			                                        // enable the collision detection
    //    mat,                                                  // surface contact material
    //	  0.005			                                        // radius of 'inflating' of mesh (for more robust
    // collision detection)
    //	  );
    // falling->SetFrameRefToAbs(ChFrame<>(ChVector3d(-0.9 + ChRandom::Get() * 1.4, 0.4 + j * 0.12, -0.9 +
    // ChRandom::Get()
    // * 1.4))); sys.Add(falling);
    //
    // but here we want to show a more low-level control of this process, for
    // various reasons, for example: we want to share a single ChTriangleMeshConnected
    // between 15 falling shapes; also we want to call RepairDuplicateVertexes() on the
    // imported mesh; also we want to scale the imported mesh using Transform().

    auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("models/bulldozer/shoe_view.obj"),
                                                                 false, true);
    mesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(1.2));  // scale to a different size
    mesh->RepairDuplicateVertexes(1e-9);                      // if meshes are not watertight

    // compute mass inertia from mesh
    double mass;
    ChVector3d cog;
    ChMatrix33<> inertia;
    double density = 1000;
    mesh->ComputeMassProperties(true, mass, cog, inertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector3d principal_I;
    ChInertiaUtils::PrincipalInertia(inertia, principal_I, principal_inertia_rot);

    // Create a shared visual model containing a visualizatoin mesh
    auto vis_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    vis_shape->SetMesh(mesh);
    vis_shape->SetMutable(false);
    vis_shape->SetColor(ChColor(0.25f, 0.5f, 0.25f));
    vis_shape->SetBackfaceCull(true);
    auto vis_model = chrono_types::make_shared<ChVisualModel>();
    vis_model->AddShape(vis_shape);

    auto coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(mesh_mat, mesh, false, false, 0.005);

    for (int j = 0; j < 15; ++j) {
        auto falling = chrono_types::make_shared<ChBodyAuxRef>();

        // Set the COG coordinates to barycenter, without displacing the REF reference.
        // Make the COG frame a principal frame.
        falling->SetFrameCOMToRef(ChFrame<>(cog, principal_inertia_rot));

        // Set inertia
        falling->SetMass(mass * density);
        falling->SetInertiaXX(density * principal_I);

        // Set the absolute position of the body:
        falling->SetFrameRefToAbs(
            ChFrame<>(ChVector3d(-0.9 + ChRandom::Get() * 1.4, 0.4 + j * 0.12, -0.9 + ChRandom::Get() * 1.4)));
        sys.Add(falling);

        falling->AddVisualModel(vis_model);
        falling->AddCollisionShape(coll_shape);
        falling->EnableCollision(true);
    }

    // Shared contact material for falling objects
    auto obj_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    obj_mat->SetFriction(0.2f);

    // Create a falling rigid bodies
    for (int bi = 0; bi < 20; bi++) {
        auto sphereBody = chrono_types::make_shared<ChBodyEasySphere>(0.05,      // radius size
                                                                      1000,      // density
                                                                      true,      // visualization?
                                                                      true,      // collision?
                                                                      obj_mat);  // contact material
        sphereBody->SetPos(ChVector3d(-0.5 + ChRandom::Get() * 1, 1.4, -0.5 + ChRandom::Get()));
        sphereBody->GetVisualShape(0)->SetColor(ChColor(0.3f, 0.3f, 0.6f));
        sys.Add(sphereBody);
    }

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(1280, 720);
    vis->SetWindowTitle("Collisions between objects");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(0, 1, -1));
    vis->AddLight(ChVector3d(30, 80, +30), 80, ChColor(0.7f, 0.7f, 0.7f));
    vis->AddLight(ChVector3d(30, 80, -30), 80, ChColor(0.7f, 0.7f, 0.7f));
    vis->EnableShadows();

    ////vis->EnableContactDrawing(ContactsDrawMode::CONTACT_NORMALS);

    // Set the debug drawer for collision visualization
    auto drawer = chrono_types::make_shared<DebugDrawer>(vis.get());
    sys.GetCollisionSystem()->RegisterVisualizationCallback(drawer);
    int mode = ChCollisionSystem::VIS_Shapes;
    bool use_zbuffer = true;

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene(true, true, ChColor(0.55f, 0.63f, 0.75f));
        vis->Render();
        if (draw_coll_shapes)
            drawer->Draw(mode, use_zbuffer);
        vis->EndScene();
        sys.DoStepDynamics(0.005);
    }

    return 0;
}
