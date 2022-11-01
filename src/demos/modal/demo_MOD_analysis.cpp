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
// Show how to use the ChModalAssembly to do a basic modal analysis (eigenvalues
// and eigenvector of the ChModalAssembly, which can also contain constraints.
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

#include "chrono_modal/ChModalAssembly.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::modal;
using namespace chrono::fea;
using namespace chrono::irrlicht;

// Output directory
const std::string out_dir = GetChronoOutputPath() + "MODAL_ANALYSIS";

int ID_current_example = 1;

double beam_Young = 100.e6;
double beam_density = 1000;
double beam_wz = 0.3;
double beam_wy = 0.05;
double beam_L = 6;

void MakeAndRunDemoCantilever(ChSystem& sys, ChVisualSystemIrrlicht& vis, bool base_fixed) {
    // Clear previous demo, if any:
    sys.Clear();
    sys.SetChTime(0);

    // CREATE THE ASSEMBLY.
    //
    // The ChModalAssembly is the most important item when doing modal analysis.
    // You must add finite elements, bodies and constraints into this assembly in order
    // to compute the modal frequencies etc.; objects not added into this won't be counted.
    auto my_assembly = chrono_types::make_shared<ChModalAssembly>();
    sys.Add(my_assembly);

    // Now populate the assembly to analyze.
    // In this demo, make a cantilever constrained to a body (a large box
    // acting as a base):

    // BODY: the base:

    auto my_body_A = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 2, 200);
    my_body_A->SetBodyFixed(base_fixed);
    my_body_A->SetPos(ChVector<>(-0.5, 0, 0));
    my_assembly->Add(my_body_A);

    // MESH:  Create a FEM mesh, that is a container for groups
    //        of elements and their referenced nodes.

    auto my_mesh = chrono_types::make_shared<ChMesh>();
    my_assembly->Add(my_mesh);

    my_mesh->SetAutomaticGravity(false);

    // BEAMS:

    // Create a simplified section, i.e. thickness and material properties
    // for beams. This will be shared among some beams.
    auto msection = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();

    msection->SetDensity(beam_density);
    msection->SetYoungModulus(beam_Young);
    msection->SetGwithPoissonRatio(0.31);
    msection->SetBeamRaleyghDampingBeta(0.00001);
    msection->SetBeamRaleyghDampingAlpha(0.001);
    msection->SetAsRectangularSection(beam_wy, beam_wz);

    // This helps creating sequences of nodes and ChElementBeamEuler elements:
    ChBuilderBeamEuler builder;

    builder.BuildBeam(my_mesh,                   // the mesh where to put the created nodes and elements
                      msection,                  // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                      6,                         // the number of ChElementBeamEuler to create
                      ChVector<>(0, 0, 0),       // the 'A' point in space (beginning of beam)
                      ChVector<>(beam_L, 0, 0),  // the 'B' point in space (end of beam)
                      ChVector<>(0, 1, 0)        // the 'Y' up direction of the section for the beam
    );

    // CONSTRAINT: connect root of blade to the base.

    auto my_root = chrono_types::make_shared<ChLinkMateGeneric>();
    my_root->Initialize(builder.GetLastBeamNodes().front(), my_body_A, ChFrame<>(ChVector<>(0, 0, 1), QUNIT));
    my_assembly->Add(my_root);

    // VISUALIZATION ASSETS:

    auto mvisualizebeamA = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizebeamA->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_TX);
    mvisualizebeamA->SetColorscaleMinMax(-0.001, 1200);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeamA);

    auto mvisualizebeamC = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizebeamC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizebeamC->SetSymbolsThickness(0.2);
    mvisualizebeamC->SetSymbolsScale(0.1);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeamC);

    // Just for later reference, dump M,R,K,Cq matrices. Ex. for comparison with Matlab eigs()
    my_assembly->DumpSubassemblyMatrices(true, true, true, true, (out_dir + "/dump").c_str());

    // Here we perform the modal analysis on the ChModalAssembly.
    // - We compute only the first n modes. This helps dealing with very large
    //   systems with many DOFs.
    // - If the assembly is free floating (ex like an helicopter or an airplane,
    //   i.e. there is no part that is fixed to ground) it will give six modes with 0 frequency,
    //   the so called rigid body modes.
    // - After computing the modes, you can access the eigenmodes, eigenvalues (also scaled as frequencies)
    //   from the ChModalAssembly member data, ex. via my_assembly2->Get_modes_frequencies
    // - For an interactive display of the modes, in Irrlicht view, use application.SetModalShow(true);
    //   this will pause the dynamic simulation and plot the modes of any ChModalAssembly present in the system
    //   as an oscillating animation. Use the GUI of Irrlicht 3D view to change the ID and amplitude of the plotted
    //   mode.
    my_assembly->ComputeModes(14);

    // Just for logging the frequencies:
    for (int i = 0; i < my_assembly->Get_modes_frequencies().rows(); ++i)
        GetLog() << "Mode n." << i << "  frequency [Hz]: " << my_assembly->Get_modes_frequencies()(i)
                 << "  damping ratio:" << my_assembly->Get_modes_damping_ratios()(i)
                 << "    Re=" << my_assembly->Get_modes_eig()(i).real()
                 << "  Im=" << my_assembly->Get_modes_eig()(i).imag() << "\n";

    // Here we perform the complex-modal analysis (damped modes) on the ChModalAssembly.
    // Short way:
    //   my_assembly->ComputeModesDamped(14);
    // Or, if you need more control on the eigenvalue solver, do this:
    //   my_assembly->ComputeModesDamped(ChModalSolveDamped( ...parameters...));

    ChSolverComplexPardisoMKL mfactorization;   // ex: use Pardiso instead than default Eigen::SparseQR for factorization
    mfactorization.GetMklEngine().pardisoParameterArray()[12] = 1; // ex.: custom setting for Pardiso
    my_assembly->ComputeModesDamped(ChModalSolveDamped(
        14,          // n. of requested eigenmodes
        1e-5,        // base frequency, or vector of frequency spans
        500,         // the max. number of iterations
        1e-10,       // the tolerance
        true,        // verbose
        ChQuadraticEigenvalueSolverKrylovSchur(&mfactorization) // the eigenpair solver algorithm
     )
    );

    // Just for logging the frequencies:
    for (int i = 0; i < my_assembly->Get_modes_frequencies().rows(); ++i)
        GetLog() << "Damped mode n." << i << "  frequency [Hz]: " << my_assembly->Get_modes_frequencies()(i)
                 << "  damping ratio:" << my_assembly->Get_modes_damping_ratios()(i)
                 << "    Re=" << my_assembly->Get_modes_eig()(i).real()
                 << "  Im=" << my_assembly->Get_modes_eig()(i).imag() << "\n";

    // This is needed if you want to see things in Irrlicht
    vis.BindAll();

    int current_example = ID_current_example;
    while ((ID_current_example == current_example) && vis.Run()) {
        vis.BeginScene();
        vis.Render();
        tools::drawGrid(&vis, 1, 1, 12, 12, ChCoordsys<>(ChVector<>(0, 0, 0), CH_C_PI_2, VECT_Z),
                        ChColor(0.5f, 0.5f, 0.5f), true);
        vis.EndScene();
    }
}

void MakeAndRunDemoLbeam(ChSystem& sys, ChVisualSystemIrrlicht& vis, bool body1fixed, bool body2fixed) {
    // Clear previous demo, if any:
    sys.Clear();
    sys.SetChTime(0);

    // CREATE THE ASSEMBLY.
    // The ChModalAssembly is the most important item when doing modal analysis.
    // You must add finite elements, bodies and constraints into this assembly in order
    // to compute the modal frequencies etc.; objects not added into this won't be counted.
    auto my_assembly = chrono_types::make_shared<ChModalAssembly>();
    sys.Add(my_assembly);

    // Now populate the assembly to analyze.
    // In this demo, make a L-shaped beam constrained to two bodies at the end

    // MESH:  Create a FEM mesh, that is a container for groups
    //        of elements and their referenced nodes.

    auto my_mesh = chrono_types::make_shared<ChMesh>();
    my_assembly->Add(my_mesh);

    my_mesh->SetAutomaticGravity(false);

    // BEAMS:

    // Create a simplified section, i.e. thickness and material properties
    // for beams. This will be shared among some beams.
    auto msection = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();

    msection->SetDensity(beam_density);
    msection->SetYoungModulus(beam_Young);
    msection->SetGwithPoissonRatio(0.31);
    msection->SetBeamRaleyghDampingBeta(0.00001);
    msection->SetBeamRaleyghDampingAlpha(0.001);
    msection->SetAsRectangularSection(beam_wy, beam_wz);

    // This helps creating sequences of nodes and ChElementBeamEuler elements:
    ChBuilderBeamEuler builder;

    builder.BuildBeam(my_mesh,                   // the mesh where to put the created nodes and elements
                      msection,                  // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                      6,                         // the number of ChElementBeamEuler to create
                      ChVector<>(0, 0, 0),       // the 'A' point in space (beginning of beam)
                      ChVector<>(beam_L, 0, 0),  // the 'B' point in space (end of beam)
                      ChVector<>(0, 1, 0)        // the 'Y' up direction of the section for the beam
    );
    auto start_node = builder.GetLastBeamNodes().front();
    builder.BuildBeam(my_mesh,   // the mesh where to put the created nodes and elements
                      msection,  // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                      6,         // the number of ChElementBeamEuler to create
                      builder.GetLastBeamNodes().back(),    // the 'A' point in space (beginning of beam)
                      ChVector<>(beam_L, beam_L * 0.5, 0),  // the 'B' point in space (end of beam)
                      ChVector<>(1, 0, 0)                   // the 'Y' up direction of the section for the beam
    );
    auto end_node = builder.GetLastBeamNodes().back();

    // BODY: 1st end

    auto my_body_A = chrono_types::make_shared<ChBodyEasyBox>(0.5, 0.5, 0.5, 200);
    my_body_A->SetBodyFixed(body1fixed);
    my_body_A->SetPos(ChVector<>(-0.25, 0, 0));
    my_assembly->Add(my_body_A);

    // BODY: 2nd end

    auto my_body_B = chrono_types::make_shared<ChBodyEasyBox>(0.5, 0.5, 0.5, 200);
    my_body_B->SetBodyFixed(body2fixed);
    my_body_B->SetPos(ChVector<>(beam_L, beam_L * 0.5 + 0.25, 0));
    my_assembly->Add(my_body_B);

    // CONSTRAINT: connect beam end to body
    auto my_root1 = chrono_types::make_shared<ChLinkMateGeneric>();
    my_root1->Initialize(start_node, my_body_A, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));
    my_assembly->Add(my_root1);

    // CONSTRAINT: connect beam end to body
    auto my_root2 = chrono_types::make_shared<ChLinkMateGeneric>();
    my_root2->Initialize(end_node, my_body_B, ChFrame<>(ChVector<>(beam_L, beam_L * 0.5, 0), QUNIT));
    my_assembly->Add(my_root2);

    // VISUALIZATION ASSETS:

    auto mvisualizebeamA = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizebeamA->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_TX);
    mvisualizebeamA->SetColorscaleMinMax(-0.001, 1200);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeamA);

    auto mvisualizebeamC = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizebeamC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizebeamC->SetSymbolsThickness(0.2);
    mvisualizebeamC->SetSymbolsScale(0.1);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeamC);

    // Just for later reference, dump M,R,K,Cq matrices. Ex. for comparison with Matlab eigs()
    my_assembly->DumpSubassemblyMatrices(true, true, true, true, (out_dir + "/dump").c_str());

    // Here we perform the modal analysis on the ChModalAssembly.
    // - We compute only the first n modes. This helps dealing with very large
    //   systems with many DOFs.
    // - If the assembly is free floating (ex like an helicopter or an airplane,
    //   i.e. there is no part that is fixed to ground) it will give six modes with 0 frequency,
    //   the so called rigid body modes.
    // - After computing the modes, you can access the eigenmodes, eigenvalues (also scaled as frequencies)
    //   from the ChModalAssembly member data, ex. via my_assembly2->Get_modes_frequencies
    // - For an interactive display of the modes, in Irrlicht view, use application.SetModalShow(true);
    //   this will pause the dynamic simulation and plot the modes of any ChModalAssembly present in the system
    //   as an oscillating animation. Use the GUI of Irrlicht 3D view to change the ID and amplitude of the plotted
    //   mode.
    my_assembly->ComputeModes(16);

    // If you need to enter more detailed settings for the eigenvalue solver, do this :
    /*
    my_assembly->ComputeModes(ChModalSolveUndamped(
        12,             // n. lowest nodes to search, or modes clusters {{freq1,nnodes2},{freq2,nnodes2},{...,...}}
        1e-5,           // base freq.
        500,            // max iterations
        1e-10,          // tolerance
        false,          // verbose
        ChGeneralizedEigenvalueSolverKrylovSchur()) // solver type
    );
    */

    // Just for logging the frequencies:
    for (int i = 0; i < my_assembly->Get_modes_frequencies().rows(); ++i)
        GetLog() << "Mode n." << i << "  frequency [Hz]: " << my_assembly->Get_modes_frequencies()(i) << "\n";

    // This is needed if you want to see things in Irrlicht 3D view.
    vis.BindAll();

    int current_example = ID_current_example;
    while ((ID_current_example == current_example) && vis.Run()) {
        vis.BeginScene();
        vis.Render();
        tools::drawGrid(&vis, 1, 1, 12, 12, ChCoordsys<>(ChVector<>(0, 0, 0), CH_C_PI_2, VECT_Z),
                        ChColor(0.5f, 0.5f, 0.5f), true);
        vis.EndScene();
    }
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
                    ID_current_example = 1;
                    return true;
                case irr::KEY_KEY_2:
                    ID_current_example = 2;
                    return true;
                case irr::KEY_KEY_3:
                    ID_current_example = 3;
                    return true;
                case irr::KEY_KEY_4:
                    ID_current_example = 4;
                    return true;
                case irr::KEY_KEY_5:
                    ID_current_example = 5;
                    return true;
                default:
                    break;
            }
        }
        return false;
    }
};

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2021 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Directory for output data
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // CREATE THE MODEL

    // Create a Chrono::Engine physical system
    ChSystemNSC sys;

    // no gravity used here
    sys.Set_G_acc(VNULL);

    // VISUALIZATION

    // Create the Irrlicht visualization system
    ChVisualSystemIrrlicht vis;
    vis.AttachSystem(&sys);
    vis.SetWindowSize(1024, 768);
    vis.SetWindowTitle("Modal analysis");
    vis.Initialize();
    vis.AddLogo();
    vis.AddSkyBox();
    vis.AddCamera(ChVector<>(1, 1.3, 6), ChVector<>(3, 0, 0));
    vis.AddLightWithShadow(ChVector<>(20, 20, 20), ChVector<>(0, 0, 0), 50, 5, 50, 55);
    vis.AddLight(ChVector<>(-20, -20, 0), 6, ChColor(0.6f, 1.0f, 1.0f));
    vis.AddLight(ChVector<>(0, -20, -20), 6, ChColor(0.6f, 1.0f, 1.0f));

    // This is for GUI tweaking of system parameters..
    MyEventReceiver receiver;
    // note how to add a custom event receiver to the default interface:
    vis.AddUserEventReceiver(&receiver);

    // Some help on the screen
    vis.GetGUIEnvironment()->addStaticText(
        L"Press 1: fixed cantilever\n"
        L"Press 2: free-free cantilever\n"
        L"Press 3: L-beam, root fixed\n"
        L"Press 4: L-beam, free-free\n"
        L"Press 5: L-beam, fixed-fixed",
        irr::core::rect<irr::s32>(400, 80, 850, 200), false, true, 0);

    // Note, in order to have this modal visualization  working, a ChModalAssembly must have been added to the ChSystem,
    // where some modes must have been already computed.
    vis.EnableModalAnalysis(true);
    vis.SetModalSpeed(15);
    vis.SetModalAmplitude(0.8);
    vis.SetModalModeNumber(0);

    // Optional: open the GUI for changing the N of the mode shape via a slider in the Irrlicht view
    vis.ShowInfoPanel(true);
    vis.SetInfoTab(1);

    // Run the sub-demos
    while (true) {
        switch (ID_current_example) {
            case 1:
                MakeAndRunDemoCantilever(sys, vis,
                                         true);  // root fixed
                break;
            case 2:
                MakeAndRunDemoCantilever(sys, vis,
                                         false);  // root free, for free-free mode
                break;
            case 3:
                MakeAndRunDemoLbeam(sys, vis,
                                    true,    // end 1 fixed
                                    false);  // end 2 free
                break;
            case 4:
                MakeAndRunDemoLbeam(sys, vis,
                                    false,   // end 1 free
                                    false);  // end 2 free
                break;
            case 5:
                MakeAndRunDemoLbeam(sys, vis,
                                    true,   // end 1 fixed
                                    true);  // end 2 fixed
                break;
            default:
                break;
        }

        if (!vis.Run())
            break;
    }

    return 0;
}
