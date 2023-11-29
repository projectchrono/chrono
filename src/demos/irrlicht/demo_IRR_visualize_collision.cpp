// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.tx
// =============================================================================
// Authors: Kishor Bhalerao, Radu Serban
// =============================================================================
//
// Demonstrate the use of the debug collision visualization callback with the
// Bullet or Chrono collision system.
// =============================================================================

#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/core/ChTimer.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

auto csys_type = ChCollisionSystem::Type::BULLET;
////auto csys_type = ChCollisionSystem::Type::MULTICORE;

class DebugDrawer : public ChCollisionSystem::VisualizationCallback {
  public:
    explicit DebugDrawer(ChVisualSystemIrrlicht* vis) : m_vis(vis) {}
    ~DebugDrawer() {}

    virtual void DrawLine(const ChVector<>& from, const ChVector<>& to, const ChColor& color) override {
        m_vis->GetVideoDriver()->draw3DLine(irr::core::vector3dfCH(from), irr::core::vector3dfCH(to),
                                            irr::video::SColor(255, color.R * 255, color.G * 255, color.B * 255));
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

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create the Chrono system, bodies, and collison shapes
    ChSystemNSC sys;
    sys.SetCollisionSystemType(csys_type);

    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    auto ground = chrono_types::make_shared<ChBodyEasyBox>(10, 3, 10, 100, mat);
    ground->SetBodyFixed(true);
    ground->SetPos(ChVector<>(0.0, 0.0, 0.0));
    ground->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.3f, 0.6f));
    sys.AddBody(ground);

    auto cyl = chrono_types::make_shared<ChBodyEasyCylinder>(geometry::ChAxis::Y, 0.5, 1.0, 100, mat);
    cyl->SetPos(ChVector<>(0.0, 3.0, 0.0));
    cyl->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.3f, 0.6f));
    sys.AddBody(cyl);

    auto box = chrono_types::make_shared<ChBodyEasyBox>(0.5, 0.5, 0.5, 100, mat);
    box->SetPos(ChVector<>(0.2, 2.0, 0.0));
    box->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.3f, 0.6f));
    sys.AddBody(box);

    auto sphere = chrono_types::make_shared<ChBodyEasySphere>(0.25, 100.0, mat);
    sphere->SetPos(ChVector<>(-0.2, 2.0, 0.75));
    sphere->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.3f, 0.6f));
    sys.AddBody(sphere);

    auto ellipse = chrono_types::make_shared<ChBodyEasyEllipsoid>(ChVector<>(0.4, 0.8, 1.2), 100.0, mat);
    ellipse->SetPos(ChVector<>(0.2, 2.0, -1.0));
    ellipse->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.3f, 0.6f));
    sys.AddBody(ellipse);

    auto mesh =
        chrono_types::make_shared<ChBodyEasyMesh>(GetChronoDataFile("models/cube.obj"), 100.0, mat, 0.05);
    mesh->SetPos(ChVector<>(2.0, 3.5, -2.0));
    mesh->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.3f, 0.6f));
    sys.AddBody(mesh);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Collision visualization demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0, 8, 6));
    vis->AddTypicalLights();

    // Set the debug drawer for collision visualization
    auto drawer = chrono_types::make_shared<DebugDrawer>(vis.get());
    sys.GetCollisionSystem()->RegisterVisualizationCallback(drawer);

    // Specify what information is visualized
    int mode = ChCollisionSystem::VIS_Shapes | ChCollisionSystem::VIS_Aabb;
    //////int mode = ChCollisionSystem::VIS_Shapes;
    ////int mode = ChCollisionSystem::VIS_Contacts;

    bool use_zbuffer = true;

    // Simulation loop
    double timestep = 0.005;
    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        drawer->Draw(mode, use_zbuffer);
        vis->EndScene();
        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);
    }

    return 0;
}
