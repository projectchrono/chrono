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
// Authors: Radu Serban
// =============================================================================
//
// Benchmark test for FEA contacts
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/utils/ChBenchmark.h"

#include "chrono/assets/ChTexture.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_irrlicht/ChIrrApp.h"
#endif

using namespace chrono;
using namespace chrono::fea;

class FEAcontactTest : public utils::ChBenchmarkTest {
  public:
    FEAcontactTest();
    ~FEAcontactTest() { delete m_system; }

    ChSystem* GetSystem() override { return m_system; }
    void ExecuteStep() override { m_system->DoStepDynamics(1e-3); }

    void SimulateVis();

  private:
    void CreateFloor(std::shared_ptr<ChMaterialSurfaceSMC> cmat);
    void CreateBeams(std::shared_ptr<ChMaterialSurfaceSMC> cmat);
    void CreateCables(std::shared_ptr<ChMaterialSurfaceSMC> cmat);

    ChSystemSMC* m_system;
};

FEAcontactTest::FEAcontactTest() {
    m_system = new ChSystemSMC();

    m_system->SetSolverType(ChSolver::Type::MINRES);
    m_system->SetSolverWarmStarting(true);
    m_system->SetMaxItersSolverSpeed(40);
    m_system->SetTolForce(1e-10);
    m_system->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    collision::ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(1);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.006);

    auto cmat = std::make_shared<ChMaterialSurfaceSMC>();
    cmat->SetYoungModulus(6e4);
    cmat->SetFriction(0.3f);
    cmat->SetRestitution(0.2f);
    cmat->SetAdhesion(0);

    CreateFloor(cmat);
    CreateBeams(cmat);
    CreateCables(cmat);

    m_system->SetupInitial();
}

void FEAcontactTest::CreateFloor(std::shared_ptr<ChMaterialSurfaceSMC> cmat) {
    auto mfloor = std::make_shared<ChBodyEasyBox>(2, 0.1, 2, 2700, true);
    mfloor->SetBodyFixed(true);
    mfloor->SetMaterialSurface(cmat);
    m_system->Add(mfloor);

    auto masset_texture = std::make_shared<ChTexture>();
    masset_texture->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
    mfloor->AddAsset(masset_texture);
}

void FEAcontactTest::CreateBeams(std::shared_ptr<ChMaterialSurfaceSMC> cmat) {
    auto mesh = std::make_shared<ChMesh>();
    m_system->Add(mesh);

    auto emat = std::make_shared<ChContinuumElastic>();
    emat->Set_E(0.01e9);
    emat->Set_v(0.3);
    emat->Set_RayleighDampingK(0.003);
    emat->Set_density(1000);

    double angles[4] = {0.15, 0.3, 0.0, 0.7};

    for (int i = 0; i < 4; ++i) {
        ChCoordsys<> cdown(ChVector<>(0, -0.4, 0));
        ChCoordsys<> crot(VNULL, Q_from_AngAxis(CH_C_2PI * angles[i], VECT_Y) * Q_from_AngAxis(CH_C_PI_2, VECT_X));
        ChCoordsys<> cydisp(ChVector<>(0.0, 0.1 + i * 0.1, -0.3));
        ChCoordsys<> ctot = cdown >> crot >> cydisp;
        ChMeshFileLoader::FromTetGenFile(mesh, GetChronoDataFile("fea/beam.node").c_str(),
                                         GetChronoDataFile("fea/beam.ele").c_str(), emat, ctot.pos,
                                         ChMatrix33<>(ctot.rot));
    }

    auto surf = std::make_shared<ChContactSurfaceMesh>();
    mesh->AddContactSurface(surf);
    surf->SetMaterialSurface(cmat);
    surf->AddFacesFromBoundary(0.002);

    auto vis_speed = std::make_shared<ChVisualizationFEAmesh>(*(mesh.get()));
    vis_speed->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    vis_speed->SetColorscaleMinMax(0.0, 5.50);
    vis_speed->SetSmoothFaces(true);
    mesh->AddAsset(vis_speed);
}

void FEAcontactTest::CreateCables(std::shared_ptr<ChMaterialSurfaceSMC> cmat) {
    auto mesh = std::make_shared<ChMesh>();
    m_system->Add(mesh);

    auto section = std::make_shared<ChBeamSectionCable>();
    section->SetDiameter(0.05);
    section->SetYoungModulus(0.01e9);
    section->SetBeamRaleyghDamping(0.05);

    ChBuilderBeamANCF builder;

    builder.BuildBeam(mesh, section, 10, ChVector<>(0, 0.1, -0.5), ChVector<>(0.5, 0.5, -0.5));

    auto cloud = std::make_shared<ChContactSurfaceNodeCloud>();
    mesh->AddContactSurface(cloud);
    cloud->SetMaterialSurface(cmat);
    cloud->AddAllNodes(0.025);

    auto vis_speed = std::make_shared<ChVisualizationFEAmesh>(*(mesh.get()));
    vis_speed->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    vis_speed->SetColorscaleMinMax(0.0, 5.50);
    vis_speed->SetSmoothFaces(true);
    vis_speed->SetWireframe(true);
    mesh->AddAsset(vis_speed);

    auto vis_nodes = std::make_shared<ChVisualizationFEAmesh>(*(mesh.get()));
    vis_nodes->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    vis_nodes->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    vis_nodes->SetSymbolsThickness(0.008);
    mesh->AddAsset(vis_nodes);
}

void FEAcontactTest::SimulateVis() {
#ifdef CHRONO_IRRLICHT
    irrlicht::ChIrrApp application(m_system, L"FEA contacts", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(0, (irr::f32)0.6, -1));
    application.AddLightWithShadow(irr::core::vector3df(1.5, 5.5, -2.5), irr::core::vector3df(0, 0, 0), 3, 2.2, 7.2, 40,
                                   512, irr::video::SColorf(1, 1, 1));

    application.AssetBindAll();
    application.AssetUpdateAll();
    application.AddShadowAll();

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        ExecuteStep();
        application.EndScene();
    }
#endif
}

// =============================================================================

#define NUM_SKIP_STEPS 50  // number of steps for hot start
#define NUM_SIM_STEPS 500  // number of simulation steps for each benchmark

CH_BM_SIMULATION_ONCE(FEAcontact, FEAcontactTest, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);

// =============================================================================

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);

#ifdef CHRONO_IRRLICHT
    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        FEAcontactTest test;
        test.SimulateVis();
        return 0;
    }
#endif

    ::benchmark::RunSpecifiedBenchmarks();
}
