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
// Authors: Alessandro Tasora
// =============================================================================
//
// FEA for 3D beams of 'cable' type (ANCF gradient-deficient beams)
//       Uses the Chrono MATLAB module
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono_matlab/ChMatlabEngine.h"
#include "chrono_matlab/ChSolverMatlab.h"

#include "FEAvisualization.h"
#include "FEAcables.h"

using namespace chrono;
using namespace chrono::fea;

// Select run-time visualization
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create a Chrono physical system
    ChSystemSMC sys;
    sys.SetGravityZ();

    // Create a mesh, that is a container for groups of elements and
    // their referenced nodes.
    auto mesh = chrono_types::make_shared<ChMesh>();

    // Create one of the available models (defined in FEAcables.h)
    ////auto model = Model1(sys, mesh);
    ////auto model = Model2(sys, mesh);
    auto model = Model3(sys, mesh);

    // Remember to add the mesh to the system!
    sys.Add(mesh);

    // Visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChVisualShapeTriangleMesh asset that is internally managed) by setting  proper coordinates and vertex colors as in the FEM
    // elements. Such a triangle mesh can be rendered by any visual system that can handle a colored ChVisualShapeTriangleMesh.

    ChColormap::Type colormap_type = ChColormap::Type::JET;
    ChVector2d colormap_range(-0.01, 0.01);

    auto vis_beam_A = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_beam_A->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
    vis_beam_A->SetColormap(colormap_type);
    vis_beam_A->SetColormapRange(colormap_range);
    vis_beam_A->SetSmoothFaces(true);
    vis_beam_A->SetWireframe(false);
    mesh->AddVisualShapeFEA(vis_beam_A);

    auto vis_beam_B = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_beam_B->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    vis_beam_B->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    vis_beam_B->SetSymbolsThickness(0.006);
    vis_beam_B->SetSymbolsScale(0.01);
    vis_beam_B->SetZbufferHide(false);
    mesh->AddVisualShapeFEA(vis_beam_B);

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Z, sys, "Cables FEM (Matlab)",        //
                                         ChVector3d(-0.8, -1.8, -0.3), ChVector3d(0, -0.3, -0.4),  //
                                         true, "Mz (Nm)", colormap_range, colormap_type);

    // Change solver to Matlab external linear solver.
    ChMatlabEngine matlab_engine;
    auto matlab_solver = chrono_types::make_shared<ChSolverMatlab>(matlab_engine);
    sys.SetSolver(matlab_solver);

    // Change type of integrator:
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // fast, less precise
    ////sys.SetTimestepperType(chrono::ChTimestepper::Type::HHT);  // precise,slower, might iterate each step

    // if later you want to change integrator settings:
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper())) {
        mystepper->SetAlpha(-0.2);
        mystepper->SetMaxIters(2);
        mystepper->SetAbsTolerances(1e-6);
    }

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
