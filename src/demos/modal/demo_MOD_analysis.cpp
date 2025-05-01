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
// Show how to use the ChAssembly to do a basic modal analysis (eigenvalues
// and eigenvector of the ChAssembly, which can also contain constraints.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"

// #include "chrono_irrlicht/ChModalVisualSystemIrrlicht.h"
#include "chrono_modal/ChModalVisualSystemIrrlicht.h"
#include "chrono_modal/ChModalSolverUndamped.h"
#include "chrono_modal/ChModalSolverDamped.h"

#include "chrono/solver/ChDirectSolverLScomplex.h"
#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include <iomanip>
#include <cmath>

using namespace chrono;
using namespace chrono::modal;
using namespace chrono::fea;
using namespace chrono::irrlicht;

// Output directory
const std::string out_dir = GetChronoOutputPath() + "MODAL_ANALYSIS";

int current_example_id = 1;

double beam_Young = 100.e6;
double beam_density = 1000;
double beam_wz = 0.3;
double beam_wy = 0.05;
double beam_L = 6;

int num_modes = 14;
bool USE_GRAVITY = true;

void MakeAndRunDemoCantilever(ChSystem& sys, bool base_fixed) {
    // Clear previous demo, if any:
    sys.Clear();
    sys.SetChTime(0);

    // CREATE THE ASSEMBLY.

    // Now populate the assembly to analyze.
    // In this demo, make a cantilever constrained to a body (a large box
    // acting as a base):

    // BODY: the base:

    auto my_body_A = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 2, 200);
    my_body_A->SetFixed(base_fixed);
    my_body_A->SetPos(ChVector3d(-0.5, 0, 0));
    sys.Add(my_body_A);

    // Create a FEM mesh, that is a container for groups of elements and their referenced nodes.
    auto mesh = chrono_types::make_shared<ChMesh>();
    sys.Add(mesh);

    mesh->SetAutomaticGravity(USE_GRAVITY);

    // BEAMS:

    // Create a simplified section, i.e. thickness and material properties
    // for beams. This will be shared among some beams.
    auto section = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();

    section->SetDensity(beam_density);
    section->SetYoungModulus(beam_Young);
    section->SetShearModulusFromPoisson(0.31);
    section->SetRayleighDampingBeta(0.00001);
    section->SetRayleighDampingAlpha(0.001);
    section->SetAsRectangularSection(beam_wy, beam_wz);

    // This helps creating sequences of nodes and ChElementBeamEuler elements:
    ChBuilderBeamEuler builder;

    builder.BuildBeam(mesh,                      // the mesh where to put the created nodes and elements
                      section,                   // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                      20,                         // the number of ChElementBeamEuler to create
                      ChVector3d(0, 0, 0),       // the 'A' point in space (beginning of beam)
                      ChVector3d(beam_L, 0, 0),  // the 'B' point in space (end of beam)
                      ChVector3d(0, 1, 0)        // the 'Y' up direction of the section for the beam
    );

    // CONSTRAINT: connect root of blade to the base.

    auto my_root = chrono_types::make_shared<ChLinkMateGeneric>();
    my_root->Initialize(builder.GetLastBeamNodes().front(), my_body_A, ChFrame<>(ChVector3d(0, 0, 1), QUNIT));
    sys.Add(my_root);

    // VISUALIZATION ASSETS:

    auto visualizebeamA = chrono_types::make_shared<ChVisualShapeFEA>();
    visualizebeamA->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_TX);
    visualizebeamA->SetColorscaleMinMax(-0.001, 1200);
    visualizebeamA->SetSmoothFaces(true);
    visualizebeamA->SetWireframe(false);
    mesh->AddVisualShapeFEA(visualizebeamA);

    auto visualizebeamC = chrono_types::make_shared<ChVisualShapeFEA>();
    visualizebeamC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    visualizebeamC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    visualizebeamC->SetSymbolsThickness(0.2);
    visualizebeamC->SetSymbolsScale(0.1);
    visualizebeamC->SetZbufferHide(false);
    mesh->AddVisualShapeFEA(visualizebeamC);

    // Here we perform the complex-modal analysis (damped modes) on the ChAssembly.
    // Short way:
    ////assembly->ComputeModesDamped(num_modes);
    // Or, if you need more control on the eigenvalue solver, do this:
    ////assembly->ComputeModesDamped(ChModalSolveDamped( ...parameters...));
}

void MakeAndRunDemoLbeam(ChSystem& sys, bool body1fixed, bool body2fixed) {
    // Clear previous demo, if any:
    sys.Clear();
    sys.SetChTime(0);

    // Create a FEM mesh, that is a container for groups of elements and their referenced nodes.
    auto mesh = chrono_types::make_shared<ChMesh>();
    sys.Add(mesh);

    mesh->SetAutomaticGravity(USE_GRAVITY);

    // BEAMS:

    // Create a simplified section, i.e. thickness and material properties
    // for beams. This will be shared among some beams.
    auto section = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();

    section->SetDensity(beam_density);
    section->SetYoungModulus(beam_Young);
    section->SetShearModulusFromPoisson(0.31);
    section->SetRayleighDampingBeta(0.00001);
    section->SetRayleighDampingAlpha(0.001);
    section->SetAsRectangularSection(beam_wy, beam_wz);

    // This helps creating sequences of nodes and ChElementBeamEuler elements:
    ChBuilderBeamEuler builder;

    builder.BuildBeam(mesh,                      // the mesh where to put the created nodes and elements
                      section,                   // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                      6,                         // the number of ChElementBeamEuler to create
                      ChVector3d(0, 0, 0),       // the 'A' point in space (beginning of beam)
                      ChVector3d(beam_L, 0, 0),  // the 'B' point in space (end of beam)
                      ChVector3d(0, 1, 0)        // the 'Y' up direction of the section for the beam
    );
    auto start_node = builder.GetLastBeamNodes().front();
    builder.BuildBeam(mesh,     // the mesh where to put the created nodes and elements
                      section,  // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                      6,        // the number of ChElementBeamEuler to create
                      builder.GetLastBeamNodes().back(),    // the 'A' point in space (beginning of beam)
                      ChVector3d(beam_L, beam_L * 0.5, 0),  // the 'B' point in space (end of beam)
                      ChVector3d(1, 0, 0)                   // the 'Y' up direction of the section for the beam
    );
    auto end_node = builder.GetLastBeamNodes().back();

    // BODY: 1st end

    auto my_body_A = chrono_types::make_shared<ChBodyEasyBox>(0.5, 0.5, 0.5, 200);
    my_body_A->SetFixed(body1fixed);
    my_body_A->SetPos(ChVector3d(-0.25, 0, 0));
    sys.Add(my_body_A);

    // BODY: 2nd end

    auto my_body_B = chrono_types::make_shared<ChBodyEasyBox>(0.5, 0.5, 0.5, 200);
    my_body_B->SetFixed(body2fixed);
    my_body_B->SetPos(ChVector3d(beam_L, beam_L * 0.5 + 0.25, 0));
    sys.Add(my_body_B);

    // CONSTRAINT: connect beam end to body
    auto my_root1 = chrono_types::make_shared<ChLinkMateGeneric>();
    my_root1->Initialize(start_node, my_body_A, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    sys.Add(my_root1);

    // CONSTRAINT: connect beam end to body
    auto my_root2 = chrono_types::make_shared<ChLinkMateGeneric>();
    my_root2->Initialize(end_node, my_body_B, ChFrame<>(ChVector3d(beam_L, beam_L * 0.5, 0), QUNIT));
    sys.Add(my_root2);

    // VISUALIZATION ASSETS:

    auto visualizebeamA = chrono_types::make_shared<ChVisualShapeFEA>();
    visualizebeamA->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_TX);
    visualizebeamA->SetColorscaleMinMax(-0.001, 1200);
    visualizebeamA->SetSmoothFaces(true);
    visualizebeamA->SetWireframe(false);
    mesh->AddVisualShapeFEA(visualizebeamA);

    auto visualizebeamC = chrono_types::make_shared<ChVisualShapeFEA>();
    visualizebeamC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    visualizebeamC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    visualizebeamC->SetSymbolsThickness(0.2);
    visualizebeamC->SetSymbolsScale(0.1);
    visualizebeamC->SetZbufferHide(false);
    mesh->AddVisualShapeFEA(visualizebeamC);
}

// Custom event manager
class MyEventReceiver : public irr::IEventReceiver {
  public:
    MyEventReceiver() {}

    bool OnEvent(const irr::SEvent& event) {
        // check if user presses keys
        if (event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) {
            switch (event.KeyInput.Key) {
                case irr::KEY_KEY_1:
                    current_example_id = 1;
                    return true;
                case irr::KEY_KEY_2:
                    current_example_id = 2;
                    return true;
                case irr::KEY_KEY_3:
                    current_example_id = 3;
                    return true;
                case irr::KEY_KEY_4:
                    current_example_id = 4;
                    return true;
                case irr::KEY_KEY_5:
                    current_example_id = 5;
                    return true;
                default:
                    break;
            }
        }
        return false;
    }
};

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2021 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Directory for output data
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // CREATE THE MODEL

    // Create a Chrono physical system
    ChSystemNSC sys;

    // no gravity used here
    sys.SetGravitationalAcceleration(VNULL);

    // Set linear solver
#ifdef CHRONO_PARDISO_MKL
    auto factorization = chrono_types::make_shared<ChSolverComplexPardisoMKL>();
    factorization->GetMklEngine().pardisoParameterArray()[12] = 1;  // custom setting for Pardiso
#else
    auto factorization = chrono_types::make_shared<ChSolverSparseComplexQR>();
#endif

    auto eig_solver = chrono_types::make_shared<ChUnsymGenEigenvalueSolverKrylovSchur>(factorization);
    ChModalSolverDamped modal_solver(num_modes, 1e-5, true, false, eig_solver);

    ChMatrixDynamic<std::complex<double>> eigvects;
    ChVectorDynamic<std::complex<double>> eigvals;
    ChVectorDynamic<> freq;
    ChVectorDynamic<> damping_ratios;

    // VISUALIZATION

    MyEventReceiver receiver;

    ChModalVisualSystemIrrlicht<std::complex<double>> vis;
    vis.AttachSystem(&sys);
    vis.Initialize();
    vis.AddLogo();
    vis.AddSkyBox();
    vis.AddCamera(ChVector3d(1, 1.3, 6), ChVector3d(3, 0, 0));
    vis.AddLightWithShadow(ChVector3d(20, 20, 20), ChVector3d(0, 0, 0), 50, 5, 50, 55);
    vis.AddLight(ChVector3d(-20, -20, 0), 6, ChColor(0.6f, 1.0f, 1.0f));
    vis.AddLight(ChVector3d(0, -20, -20), 6, ChColor(0.6f, 1.0f, 1.0f));
    vis.AddUserEventReceiver(&receiver);
    vis.GetGUIEnvironment()->addStaticText(
        L"Press 1: fixed cantilever\n"
        L"Press 2: free-free cantilever\n"
        L"Press 3: L-beam, root fixed\n"
        L"Press 4: L-beam, free-free\n"
        L"Press 5: L-beam, fixed-fixed",
        irr::core::rect<irr::s32>(400, 80, 850, 200), false, true, 0);

    vis.ShowInfoPanel(true);

    vis.AttachAssembly(sys.GetAssembly(), eigvects, freq);

    int previous_current_example_id = -1;

    // Run the sub-demos
    while (true) {
        if (previous_current_example_id != current_example_id) {
            switch (current_example_id) {
                case 1:
                    // fixed-free
                    MakeAndRunDemoCantilever(sys, true);
                    break;
                case 2:
                    // free-free
                    MakeAndRunDemoCantilever(sys, false);
                    break;
                case 3:
                    // fixed-free L-beam
                    MakeAndRunDemoLbeam(sys, true, false);
                    break;
                case 4:
                    // free-free L-beam
                    MakeAndRunDemoLbeam(sys, false, false);
                    break;
                case 5:
                    // fixed-fixed L-beam
                    MakeAndRunDemoLbeam(sys, true, true);
                    break;
                default:
                    break;
            }

            previous_current_example_id = current_example_id;

            sys.Update(true);
            vis.BindAll();

            modal_solver.Solve(sys.GetAssembly(), eigvects, eigvals, freq, damping_ratios);
            vis.ResetInitialState();
            vis.UpdateModes(eigvects, freq);

            // Just for logging the frequencies:
            std::cout << "Assembly eigen solutions: " << std::endl;
            for (int i = 0; i < freq.size(); ++i) {
                std::cout << "Mode " << std::setw(int(std::log10(num_modes) + 1)) << i << " | Freq: " << std::setw(4)
                          << freq(i) << " Hz"
                          << " | Damping Ratio:" << damping_ratios(i) << " | Eigval: {" << eigvals(i).real() << "; "
                          << eigvals(i).imag() << "i}" << std::endl;
            }
        }

        vis.BeginScene();
        vis.Render();
        tools::drawGrid(&vis, 1, 1, 12, 12, ChCoordsys<>(ChVector3d(0, 0, 0), CH_PI_2, VECT_Z),
                        ChColor(0.5f, 0.5f, 0.5f), true);
        vis.EndScene();

        if (!vis.Run())
            break;
    }

    return 0;
}
