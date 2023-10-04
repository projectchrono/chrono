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
// Authors: Mike Taylor and Radu Serban
// =============================================================================
//
// Small Displacement, Small Deformation, Linear Isotropic Benchmark test for
// ANCF beam elements - Square cantilevered beam with a time-dependent tip load
//
// Garcia-Vallejo, D., Mayo, J., Escalona, J. L., & Dominguez, J. (2004).
// Efficient evaluation of the elastic forces and the Jacobian in the absolute
// nodal coordinate formulation. Nonlinear Dynamics, 35(4), 313-329.
//
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono/physics/ChSystemSMC.h"

#include "chrono/fea/ChElementBeamANCF_3333.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/assets/ChVisualShapeFEA.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/fea/ChLoadsBeam.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemSMC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.8));

    // Set up solver
    auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);
    sys.SetSolver(solver);

    // Set up integrator
    auto integrator = chrono_types::make_shared<ChTimestepperHHT>(&sys);
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(100);
    integrator->SetAbsTolerances(1e-5);
    integrator->SetVerbose(false);
    integrator->SetModifiedNewton(true);
    sys.SetTimestepper(integrator);

    // Mesh properties
    double length = 5;       // m
    double width = 0.1;      // m
    double thickness = 0.1;  // m
    double rho = 8000;       // kg/m^3
    double E = 4e8;          // Pa
    double nu = 0;           // Poisson effect neglected for this model
    // Timoshenko shear correction coefficients for a rectangular cross-section
    double k1 = 10 * (1 + nu) / (12 + 11 * nu);
    double k2 = k1;

    auto material = chrono_types::make_shared<ChMaterialBeamANCF>(rho, E, nu, k1, k2);

    // Create mesh container
    auto mesh = chrono_types::make_shared<ChMesh>();
    sys.Add(mesh);

    // Populate the mesh container with a the nodes and elements for the meshed beam
    int num_elements = 8;
    int num_nodes = (2 * num_elements) + 1;
    double dx = length / (num_nodes - 1);

    // Setup beam cross section gradients to initially align with the global y and z directions
    ChVector<> dir1(0, 1, 0);
    ChVector<> dir2(0, 0, 1);

    // Create the first node and fix it completely to ground (Cantilever constraint)
    auto nodeA = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector<>(0, 0, 0.0), dir1, dir2);
    nodeA->SetFixed(true);
    mesh->AddNode(nodeA);

    auto last_element = chrono_types::make_shared<ChElementBeamANCF_3333>();

    for (int i = 1; i <= num_elements; i++) {
        auto nodeB = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector<>(dx * (2 * i), 0, 0), dir1, dir2);
        auto nodeC = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector<>(dx * (2 * i - 1), 0, 0), dir1, dir2);
        mesh->AddNode(nodeB);
        mesh->AddNode(nodeC);

        auto element = chrono_types::make_shared<ChElementBeamANCF_3333>();
        element->SetNodes(nodeA, nodeB, nodeC);
        element->SetDimensions(2 * dx, thickness, width);
        element->SetMaterial(material);
        element->SetAlphaDamp(0.0);
        mesh->AddElement(element);

        nodeA = nodeB;
        last_element = element;
    }

    auto end_point = nodeA;

    mesh->SetAutomaticGravity(false);

    // Define a custom point load with a time-dependent force
    class MyLoaderTimeDependentTipLoad : public ChLoaderUatomic {
      public:
        MyLoaderTimeDependentTipLoad(std::shared_ptr<ChLoadableU> mloadable)
            : ChLoaderUatomic(mloadable), m_sys(nullptr) {}

        // Compute F=F(U). The load is a 6-row vector, i.e. a wrench: forceX, forceY, forceZ, torqueX, torqueY, torqueZ.
        virtual void ComputeF(const double U,              // normalized position along the beam axis [-1...1]
                              ChVectorDynamic<>& F,        // load at U
                              ChVectorDynamic<>* state_x,  // if non-null, first update state (position) to this
                              ChVectorDynamic<>* state_w   // if non-null, fiurst update state (speed) to this
                              ) override {
            double t = m_sys->GetChTime();
            double Fmax = -500;
            double tc = 2;
            double Fz = Fmax;
            if (t < tc) {
                Fz = 0.5 * Fmax * (1 - cos(CH_C_PI * t / tc));
            }

            F.setZero();
            F(2) = Fz;  // Apply the force along the global Z axis
        }

        void SetSystem(ChSystem* sys) { m_sys = sys; }

      private:
        ChSystem* m_sys;
    };

    // Create the load container and add to the current system
    auto loadcontainer = chrono_types::make_shared<ChLoadContainer>();
    sys.Add(loadcontainer);

    // Create a custom load that uses the custom loader above.
    auto load = chrono_types::make_shared<ChLoad<MyLoaderTimeDependentTipLoad>>(last_element);
    load->loader.SetSystem(&sys);      // set containing system
    load->loader.SetApplication(1.0);  // specify application point
    loadcontainer->Add(load);          // add the load to the load container.

    // Set up mesh visualization
    auto vis_surf = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    vis_surf->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    vis_surf->SetWireframe(true);
    vis_surf->SetDrawInUndeformedReference(true);
    mesh->AddVisualShapeFEA(vis_surf);

    auto vis_node = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    vis_node->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    vis_node->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    vis_node->SetSymbolsThickness(0.01);
    mesh->AddVisualShapeFEA(vis_node);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("ANCF beam");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector<>(-0.4, 0.4, 0.4), ChVector<>(0.0, 0.0, 0.0));
    vis->AttachSystem(&sys);

    // Simulation loop
    while (vis->Run()) {
        // std::cout << "t (s): " << sys.GetChTime() << "  Tip (m): " << end_point->GetPos() << std::endl;

        vis->BeginScene();
        vis->Render();
        irrlicht::tools::drawSegment(vis.get(), ChVector<>(0), ChVector<>(1, 0, 0), ChColor(1, 0, 0));
        irrlicht::tools::drawSegment(vis.get(), ChVector<>(0), ChVector<>(0, 1, 0), ChColor(0, 1, 0));
        irrlicht::tools::drawSegment(vis.get(), ChVector<>(0), ChVector<>(0, 0, 1), ChColor(0, 0, 1));
        vis->EndScene();

        sys.DoStepDynamics(1e-2);
    }
}
