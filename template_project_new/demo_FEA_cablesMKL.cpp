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
// Authors: Dario Mangoni, Radu Serban
// =============================================================================
//
// FEA for 3D beams of 'cable' type (ANCF gradient-deficient beams)
//       uses the Chrono MKL module
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/assets/ChVisualShapeFEA.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"


using namespace chrono;
using namespace fea;

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

using namespace irr;

class Model1 {
  public:
    Model1(ChSystem& system, std::shared_ptr<ChMesh> mesh) {
        double beam_L = 0.1;
        double beam_diameter = 0.015;

        // Create a section, i.e. thickness and material properties
        // for beams. This will be shared among some beams.
        auto msection_cable = chrono_types::make_shared<ChBeamSectionCable>();
        msection_cable->SetDiameter(beam_diameter);
        msection_cable->SetYoungModulus(0.01e9);
        msection_cable->SetRayleighDamping(0.000);

        // Create the nodes
        auto hnodeancf1 = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(0, 0, -0.2), ChVector3d(1, 0, 0));
        auto hnodeancf2 = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(beam_L, 0, -0.2), ChVector3d(1, 0, 0));

        mesh->AddNode(hnodeancf1);
        mesh->AddNode(hnodeancf2);

        // Create the element

        auto belementancf1 = chrono_types::make_shared<ChElementCableANCF>();

        belementancf1->SetNodes(hnodeancf1, hnodeancf2);
        belementancf1->SetSection(msection_cable);

        mesh->AddElement(belementancf1);

        // Apply a force or a torque to a node:
        hnodeancf2->SetForce(ChVector3d(0, 3, 0));

        hnodeancf1->SetFixed(true);

        // Add a rigid body connected to the end of the beam:

        body = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.02, 0.02, 1000);
        body->SetPos(hnodeancf2->GetPos() + ChVector3d(0.05, 0, 0));
        system.Add(body);

        auto constraint_pos = chrono_types::make_shared<ChLinkNodeFrame>();
        constraint_pos->Initialize(hnodeancf2, body);
        system.Add(constraint_pos);

        auto constraint_dir = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
        constraint_dir->Initialize(hnodeancf2, body);
        constraint_dir->SetDirectionInAbsoluteCoords(ChVector3d(1, 0, 0));
        system.Add(constraint_dir);
    }

    // Print position of end body
    void PrintBodyPosition() {
        std::cout << "Time: " << body->GetChTime() << std::endl;
        std::cout << "  " << body->GetPos() << std::endl;
    }

  private:
    std::shared_ptr<ChBodyEasyBox> body;
};

// ----------------------------------------------------------------------------
// Model2: A beam composed of 10 ANCF beam element, with one end hinged to
//         ground, moving under gravity alone.
// This model demonstrates the use of the utility class ChBuilderCableANCF.
// ----------------------------------------------------------------------------
class Model2 {
  public:
    Model2(ChSystem& system, std::shared_ptr<ChMesh> mesh) {
        // Create a section, i.e. thickness and material properties
        // for beams. This will be shared among some beams.

        auto msection_cable2 = chrono_types::make_shared<ChBeamSectionCable>();
        msection_cable2->SetDiameter(0.015);
        msection_cable2->SetYoungModulus(0.01e9);
        msection_cable2->SetRayleighDamping(0.000);

        // This ChBuilderCableANCF helper object is very useful because it will
        // subdivide 'beams' into sequences of finite elements of beam type, ex.
        // one 'beam' could be made of 5 FEM elements of ChElementBeamANCF_3333 class.
        // If new nodes are needed, it will create them for you.
        ChBuilderCableANCF builder;

        // Now, simply use BuildBeam to create a beam from a point to another:
        builder.BuildBeam(mesh,             // the mesh where to put the created nodes and elements
                          msection_cable2,  // the ChBeamSectionCable to use for the ChElementBeamANCF_3333 elements
                          10,               // the number of ChElementBeamANCF_3333 to create
                          ChVector3d(0, 0, -0.1),     // the 'A' point in space (beginning of beam)
                          ChVector3d(0.5, 0, -0.1));  // the 'B' point in space (end of beam)

        // After having used BuildBeam(), you can retrieve the nodes used for the beam,
        // For example say you want to fix both pos and dir of A end and apply a force to the B end:
        // builder.GetLastBeamNodes().back()->SetFixed(true);
        builder.GetLastBeamNodes().front()->SetForce(ChVector3d(0, -0.2, 0));

        // For instance, now retrieve the A end and add a constraint to
        // block the position only of that node:
        auto mtruss = chrono_types::make_shared<ChBody>();
        mtruss->SetFixed(true);

        auto constraint_hinge = chrono_types::make_shared<ChLinkNodeFrame>();
        constraint_hinge->Initialize(builder.GetLastBeamNodes().back(), mtruss);
        system.Add(constraint_hinge);
    }
};

// ----------------------------------------------------------------------------
// Model3: A set of beam elements with connected bodies, each with different
//         number of ANCF cable elements.
// ----------------------------------------------------------------------------

class Model3 {
  public:
    Model3(ChSystem& system, std::shared_ptr<ChMesh> mesh, int n_chains = 6) : bodies(n_chains) {
        auto msection_cable2 = chrono_types::make_shared<ChBeamSectionCable>();
        msection_cable2->SetDiameter(0.015);
        msection_cable2->SetYoungModulus(0.01e9);
        msection_cable2->SetRayleighDamping(0.000);

        auto mtruss = chrono_types::make_shared<ChBody>();
        mtruss->SetFixed(true);

        for (int j = 0; j < n_chains; ++j) {
            ChBuilderCableANCF builder;

            // Now, simply use BuildBeam to create a beam from a point to another:
            builder.BuildBeam(mesh,             // the mesh where to put the created nodes and elements
                              msection_cable2,  // ChBeamSectionCable to use for the ChElementBeamANCF_3333 elements
                              1 + j,            // number of ChElementBeamANCF_3333 to create
                              ChVector3d(0, 0, -0.1 * j),             // point A (beginning of beam)
                              ChVector3d(0.1 + 0.1 * j, 0, -0.1 * j)  // point B (end of beam)
            );

            builder.GetLastBeamNodes().back()->SetForce(ChVector3d(0, -0.2, 0));

            auto constraint_hinge = chrono_types::make_shared<ChLinkNodeFrame>();
            constraint_hinge->Initialize(builder.GetLastBeamNodes().front(), mtruss);
            system.Add(constraint_hinge);

            auto msphere = chrono_types::make_shared<ChVisualShapeSphere>(0.02);
            constraint_hinge->AddVisualShape(msphere);

            // make a box and connect it
            auto mbox = chrono_types::make_shared<ChBodyEasyBox>(0.2, 0.04, 0.04, 1000);
            mbox->SetPos(builder.GetLastBeamNodes().back()->GetPos() + ChVector3d(0.1, 0, 0));
            system.Add(mbox);

            auto constraint_pos = chrono_types::make_shared<ChLinkNodeFrame>();
            constraint_pos->Initialize(builder.GetLastBeamNodes().back(), mbox);
            system.Add(constraint_pos);

            auto constraint_dir = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
            constraint_dir->Initialize(builder.GetLastBeamNodes().back(), mbox);
            constraint_dir->SetDirectionInAbsoluteCoords(ChVector3d(1, 0, 0));
            system.Add(constraint_dir);

            // make another beam
            builder.BuildBeam(
                mesh,                // mesh where to put the created nodes and elements
                msection_cable2,     // ChBeamSectionCable to use for the ChElementBeamANCF_3333 elements
                1 + (n_chains - j),  // number of ChElementBeamANCF_3333 to create
                ChVector3d(mbox->GetPos().x() + 0.1, 0, -0.1 * j),                        // point A (beginning of beam)
                ChVector3d(mbox->GetPos().x() + 0.1 + 0.1 * (n_chains - j), 0, -0.1 * j)  // point B (end of beam)
            );

            auto constraint_pos2 = chrono_types::make_shared<ChLinkNodeFrame>();
            constraint_pos2->Initialize(builder.GetLastBeamNodes().front(), mbox);
            system.Add(constraint_pos2);

            auto constraint_dir2 = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
            constraint_dir2->Initialize(builder.GetLastBeamNodes().front(), mbox);
            constraint_dir2->SetDirectionInAbsoluteCoords(ChVector3d(1, 0, 0));
            system.Add(constraint_dir2);

            // make a box and connect it
            bodies[j] = chrono_types::make_shared<ChBodyEasyBox>(0.2, 0.04, 0.04, 1000);
            bodies[j]->SetPos(builder.GetLastBeamNodes().back()->GetPos() + ChVector3d(0.1, 0, 0));
            system.Add(bodies[j]);

            auto constraint_pos3 = chrono_types::make_shared<ChLinkNodeFrame>();
            constraint_pos3->Initialize(builder.GetLastBeamNodes().back(), bodies[j]);
            system.Add(constraint_pos3);

            auto constraint_dir3 = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
            constraint_dir3->Initialize(builder.GetLastBeamNodes().back(), bodies[j]);
            constraint_dir3->SetDirectionInAbsoluteCoords(ChVector3d(1, 0, 0));
            system.Add(constraint_dir3);
        }
    }

    // Print positions of end bodies in each chain
    void PrintBodyPositions() {
        std::cout << "Time: " << bodies[0]->GetChTime() << std::endl;
        for (auto body : bodies) {
            std::cout << "  " << body->GetPos() << std::endl;
        }
    }

  private:
    std::vector<std::shared_ptr<ChBodyEasyBox>> bodies;  // end bodies
};



int main(int argc, char* argv[]) {

    SetChronoDataPath(CHRONO_DATA_DIR);
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create a Chrono::Engine physical system
    ChSystemSMC sys;

    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    // Create a mesh, that is a container for groups of elements and
    // their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create the model (defined in FEAcables.h)
    auto model = Model3(sys, my_mesh);

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    // Visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChVisualShapeTriangleMesh
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colors as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a colored ChVisualShapeTriangleMesh).

    auto mvisualizebeamA = chrono_types::make_shared<ChVisualShapeFEA>();
    mvisualizebeamA->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
    mvisualizebeamA->SetColorscaleMinMax(-0.4, 0.4);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeamA);

    auto mvisualizebeamC = chrono_types::make_shared<ChVisualShapeFEA>();
    mvisualizebeamC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizebeamC->SetSymbolsThickness(0.006);
    mvisualizebeamC->SetSymbolsScale(0.01);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeamC);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Cables FEM (MKL)");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(0.0, 0.6, -1.0));
    vis->AttachSystem(&sys);

    // Configure PardisoMKL solver.
    // For this simple and relatively small problem, use of the sparsity pattern learner may introduce additional
    // overhead (if the sparsity pattern is not locked).
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>(1);
    mkl_solver->UseSparsityPatternLearner(false);
    mkl_solver->LockSparsityPattern(false);
    mkl_solver->SetVerbose(false);
    sys.SetSolver(mkl_solver);

    sys.Update();

    // Set integrator
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(0.01);
        ////model.PrintBodyPositions();
    }

    return 0;
}
