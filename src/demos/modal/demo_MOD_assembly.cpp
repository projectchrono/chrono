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
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
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
const std::string out_dir = GetChronoOutputPath() + "MODAL_ASSEMBLY";

int ID_current_example = 1;
bool modal_analysis = true;

double beam_Young = 100.e6;
double beam_density = 1000;
double beam_wz = 0.3;
double beam_wy = 0.05;
double beam_L = 6;
int n_elements = 8;

double step_size = 0.05;

void MakeAndRunDemoCantilever(ChSystem& sys,
                              bool do_modal_reduction,
                              bool add_internal_body,
                              bool add_boundary_body,
                              bool add_force,
                              bool add_other_assemblies) {
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
    // In this demo, make a cantilever with fixed end

    // BODY: the base:

    auto my_body_A = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 2, 200);
    my_body_A->SetBodyFixed(true);
    my_body_A->SetPos(ChVector<>(-0.5, 0, 0));
    my_assembly->Add(my_body_A);

    // MESH:  Create two FEM meshes: one for nodes that will be removed in modal reduction,
    //        the other for the nodes that will remain after modal reduction.

    auto my_mesh_internal = chrono_types::make_shared<ChMesh>();
    my_assembly->AddInternal(my_mesh_internal);  // NOTE: MESH FOR INTERNAL NODES: USE my_assembly->AddInternal()

    auto my_mesh_boundary = chrono_types::make_shared<ChMesh>();
    my_assembly->Add(my_mesh_boundary);  // NOTE: MESH FOR BOUNDARY NODES: USE my_assembly->Add()

    my_mesh_internal->SetAutomaticGravity(false);
    my_mesh_boundary->SetAutomaticGravity(false);

    // BEAMS:

    // Create a simplified section, i.e. thickness and material properties
    // for beams. This will be shared among some beams.
    auto msection = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();

    msection->SetDensity(beam_density);
    msection->SetYoungModulus(beam_Young);
    msection->SetGwithPoissonRatio(0.31);
    msection->SetBeamRaleyghDampingBeta(0.01);
    msection->SetBeamRaleyghDampingAlpha(0.0001);
    msection->SetAsRectangularSection(beam_wy, beam_wz);

    ChBuilderBeamEuler builder;

    // The first node is a boundary node: add it to my_mesh_boundary
    auto my_node_A_boundary = chrono_types::make_shared<ChNodeFEAxyzrot>();
    my_node_A_boundary->SetMass(0);
    my_node_A_boundary->GetInertia().setZero();
    my_mesh_boundary->AddNode(my_node_A_boundary);

    // The last node is a boundary node: add it to my_mesh_boundary
    auto my_node_B_boundary = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_L, 0, 0)));
    my_node_B_boundary->SetMass(0);
    my_node_B_boundary->GetInertia().setZero();
    my_mesh_boundary->AddNode(my_node_B_boundary);

    // my_node_A_boundary->SetFixed(true); // NO - issues with bookeeping in modal_Hblock ***TO FIX***, for the moment:
    // use constraint:
    // Constraint the boundary node to truss
    auto my_root = chrono_types::make_shared<ChLinkMateGeneric>();
    my_root->Initialize(my_node_A_boundary, my_body_A, ChFrame<>(ChVector<>(0, 0, 1), QUNIT));
    my_assembly->Add(my_root);

    // The other nodes are internal nodes: let the builder.BuildBeam add them to my_mesh_internal
    builder.BuildBeam(my_mesh_internal,    // the mesh where to put the created nodes and elements
                      msection,            // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                      n_elements,          // the number of ChElementBeamEuler to create
                      my_node_A_boundary,  // the 'A' point in space (beginning of beam)
                      my_node_B_boundary,  // ChVector<>(beam_L, 0, 0), // the 'B' point in space (end of beam)
                      ChVector<>(0, 1, 0)  // the 'Y' up direction of the section for the beam
    );

    my_node_B_boundary->SetForce(ChVector<>(0, 0, 90));  // to trigger some vibration at the free end

    if (add_internal_body) {
        // BODY: in the middle, as internal
        auto my_body_B = chrono_types::make_shared<ChBodyEasyBox>(1.8, 1.8, 1.8, 200);
        my_body_B->SetPos(ChVector<>(beam_L * 0.5, 0, 0));
        my_assembly->AddInternal(my_body_B);

        auto my_mid_constr = chrono_types::make_shared<ChLinkMateGeneric>();
        my_mid_constr->Initialize(builder.GetLastBeamNodes()[n_elements / 2], my_body_B,
                                  ChFrame<>(ChVector<>(beam_L * 0.5, 0, 0), QUNIT));
        my_assembly->AddInternal(my_mid_constr);
    }

    if (add_boundary_body) {
        // BODY: in the end, as boundary
        auto my_body_C = chrono_types::make_shared<ChBodyEasyBox>(0.8, 0.8, 0.8, 200);
        my_body_C->SetPos(ChVector<>(beam_L, 0, 0));
        my_assembly->Add(my_body_C);

        auto my_end_constr = chrono_types::make_shared<ChLinkMateGeneric>();
        my_end_constr->Initialize(builder.GetLastBeamNodes().back(), my_body_C,
                                  ChFrame<>(ChVector<>(beam_L, 0, 0), QUNIT));
        my_assembly->Add(my_end_constr);
    }

    if (add_other_assemblies) {
        // Test how to connect the boundary nodes/bodies of a ChModalAssembly to some other ChAssembly
        // or to other items (bodies, etc.) that are added to the ChSystem, like in this way
        //   ChSystem
        //       ChModalAssembly
        //           internal ChBody, ChNode, etc.
        //           boundary ChBody, ChNode, etc.
        //       ChBody
        //       ChAssembly
        //           ChBody
        //       etc.

        // example if putting additional items directly in the ChSystem:
        auto my_body_D = chrono_types::make_shared<ChBodyEasyBox>(0.2, 0.4, 0.4, 200);
        my_body_D->SetPos(ChVector<>(beam_L * 1.1, 0, 0));
        sys.Add(my_body_D);

        auto my_end_constr2 = chrono_types::make_shared<ChLinkMateGeneric>();
        my_end_constr2->Initialize(builder.GetLastBeamNodes().back(), my_body_D,
                                   ChFrame<>(ChVector<>(beam_L, 0, 0), QUNIT));
        sys.Add(my_end_constr2);

        // example if putting additional items in a second assembly (just a simple rotating blade)
        auto my_assembly0 = chrono_types::make_shared<ChAssembly>();
        sys.Add(my_assembly0);

        auto my_body_blade = chrono_types::make_shared<ChBodyEasyBox>(0.2, 0.6, 0.2, 150);
        my_body_blade->SetPos(ChVector<>(beam_L * 1.15, 0.3, 0));
        my_assembly0->Add(my_body_blade);

        auto rotmotor1 = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        rotmotor1->Initialize(
            my_body_blade,                                                     // slave
            my_body_D,                                                         // master
            ChFrame<>(my_body_D->GetPos(), Q_from_AngAxis(CH_C_PI_2, VECT_Y))  // motor frame, in abs. coords
        );
        auto mwspeed =
            chrono_types::make_shared<ChFunction_Const>(CH_C_2PI);  // constant angular speed, in [rad/s], 2PI/s =360°/s
        rotmotor1->SetSpeedFunction(mwspeed);
        my_assembly0->Add(rotmotor1);
    }

    if (add_force) {
        // Add a force (also to internal nodes that will be removed after modal reduction).
        // This can be done using a callback that will be called all times the time integrator needs it.
        // You will provide a custom force writing into computed_custom_F_full vector (note: it is up to you to use the
        // proper indexes)
        class MyCallback : public ChModalAssembly::CustomForceFullCallback {
          public:
            MyCallback(){};
            virtual void evaluate(
                ChVectorDynamic<>&
                    computed_custom_F_full,  //< compute F here, size= n_boundary_coords_w + n_internal_coords_w
                const ChModalAssembly& link  ///< associated modal assembly
            ) {
                computed_custom_F_full
                    .setZero();  // remember! assume F vector is already properly sized, but not zeroed!
                computed_custom_F_full[computed_custom_F_full.size() - 16] =
                    -60;  // just for test, assign a force to a random coordinate of F, here an internal node
            }
        };
        auto my_callback = chrono_types::make_shared<MyCallback>();

        my_assembly->RegisterCallback_CustomForceFull(my_callback);
    }

    // Just for later reference, dump  M,R,K,Cq matrices. Ex. for comparison with Matlab eigs()
    sys.Setup();
    sys.Update();
    my_assembly->DumpSubassemblyMatrices(true, true, true, true, (out_dir + "/dump").c_str());

    if (do_modal_reduction) {
        // HERE PERFORM THE MODAL REDUCTION!

        my_assembly->SwitchModalReductionON(
            6,  // The number of modes to retain from modal reduction
            ChModalDampingRayleigh(0.01,
                                   0.05)  // The damping model - Optional parameter: default is ChModalDampingNone().
        );

        // Other types of damping that you can try, in SwitchModalReductionON:
        //    ChModalDampingNone()                    // no damping (also default)
        //    ChModalDampingReductionR(*my_assembly)  // transforms the original damping matrix of the full subassembly
        //    ChModalDampingReductionR(full_R_ext)    // transforms an externally-provided damping matrix of the full
        //    subassembly ChModalDampingCustom(reduced_R_ext)     // uses an externally-provided damping matrix of the
        //    reduced subassembly ChModalDampingRayleigh(0.01, 0.05)      // generates a damping matrix from reduced M
        //    ad K using Rayleygh alpha-beta ChModalDampingFactorRmm(zetas)          // generates a damping matrix from
        //    damping factors zetas of dynamic modes ChModalDampingFactorRayleigh(zetas,a,b) // generates a damping
        //    matrix from damping factors of dynamic modes and rayleigh a,b for boundary nodes
        //    ChModalDampingFactorAssembly(zetas)     // (not ready) generates a damping matrix from damping factors of
        //    the modes of the subassembly, including masses of boundary
        //      where for example         ChVectorDynamic<> zetas(4);  zetas << 0.7, 0.5, 0.6, 0.7; // damping factors,
        //      other values assumed as last one.

        // OPTIONAL

        // Just for later reference, dump reduced M,R,K,Cq matrices. Ex. for comparison with Matlab eigs()
        my_assembly->DumpSubassemblyMatrices(true, true, true, true, (out_dir + "/dump_reduced").c_str());

        // Use this for high simulation performance (the internal nodes won't be updated for postprocessing)
        // my_assembly->SetInternalNodesUpdate(false);

        // Finally, optional: do an eigenvalue analysis to check if we approximately have the same eigenmodes of the
        // original not reduced assembly:
        my_assembly->ComputeModesDamped(0);

        // Just for logging the frequencies:
        for (int i = 0; i < my_assembly->Get_modes_frequencies().rows(); ++i)
            GetLog() << "Mode n." << i << "  frequency [Hz]: " << my_assembly->Get_modes_frequencies()(i)
                     << "   damping factor z: " << my_assembly->Get_modes_damping_ratios()(i) << "\n";
    } else {
        // Otherwise we perform a conventional modal analysis on the full ChModalAssembly.
        my_assembly->ComputeModes(12);

        // Just for logging the frequencies:
        for (int i = 0; i < my_assembly->Get_modes_frequencies().rows(); ++i)
            GetLog() << "Mode n." << i << "  frequency [Hz]: " << my_assembly->Get_modes_frequencies()(i) << "\n";
    }

    // VISUALIZATION ASSETS:

    auto mvisualizeInternalA = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh_internal);
    mvisualizeInternalA->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MY);
    mvisualizeInternalA->SetColorscaleMinMax(-600, 600);
    mvisualizeInternalA->SetSmoothFaces(true);
    mvisualizeInternalA->SetWireframe(false);
    my_mesh_internal->AddVisualShapeFEA(mvisualizeInternalA);

    auto mvisualizeInternalB = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh_internal);
    mvisualizeInternalB->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    mvisualizeInternalB->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizeInternalB->SetSymbolsThickness(0.2);
    mvisualizeInternalB->SetSymbolsScale(0.1);
    mvisualizeInternalB->SetZbufferHide(false);
    my_mesh_internal->AddVisualShapeFEA(mvisualizeInternalB);

    auto mvisualizeBoundaryB = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh_boundary);
    mvisualizeBoundaryB->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    mvisualizeBoundaryB->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizeBoundaryB->SetSymbolsThickness(0.4);
    mvisualizeBoundaryB->SetSymbolsScale(4);
    mvisualizeBoundaryB->SetZbufferHide(false);
    my_mesh_boundary->AddVisualShapeFEA(mvisualizeBoundaryB);

    // This is needed if you want to see things in Irrlicht
    auto vis = std::dynamic_pointer_cast<ChVisualSystemIrrlicht>(sys.GetVisualSystem());
    vis->BindAll();

    int current_example = ID_current_example;
    while (ID_current_example == current_example && vis->Run()) {
        vis->BeginScene();
        vis->DrawAll();
        tools::drawGrid(vis->GetVideoDriver(), 1, 1, 12, 12, ChCoordsys<>(ChVector<>(0, 0, 0), CH_C_PI_2, VECT_Z),
                        irr::video::SColor(100, 120, 120, 120), true);
        vis->EndScene();

        if (!modal_analysis)
            sys.DoStepDynamics(step_size);
    }
}

// Custom event manager
class MyEventReceiver : public irr::IEventReceiver {
  public:
    MyEventReceiver(ChVisualSystemIrrlicht* vis) : m_vis(vis) {}

    bool OnEvent(const irr::SEvent& event) {
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
                case irr::KEY_KEY_6:
                    ID_current_example = 6;
                    return true;
                case irr::KEY_SPACE:
                    modal_analysis = !modal_analysis;
                    m_vis->EnableModalAnalysis(modal_analysis);
                    m_vis->SetInfoTab(modal_analysis ? 1 : 0);
                    return true;
                default:
                    break;
            }
        }
        return false;
    }

  private:
    ChVisualSystemIrrlicht* m_vis;
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
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    sys.SetVisualSystem(vis);
    vis->SetWindowSize(1024, 768);
    vis->SetWindowTitle("Modal reduction");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(1, 1.3, 6), ChVector<>(3, 0, 0));
    vis->AddLightWithShadow(ChVector<>(20, 20, 20), ChVector<>(0, 0, 0), 50, 5, 50, 55);
    vis->AddLight(ChVector<>(-20, -20, 0), 6, ChColor(0.6f, 1.0f, 1.0f));
    vis->AddLight(ChVector<>(0, -20, -20), 6, ChColor(0.6f, 1.0f, 1.0f));

    // This is for GUI tweaking of system parameters..
    MyEventReceiver receiver(vis.get());
    // note how to add a custom event receiver to the default interface:
    vis->AddUserEventReceiver(&receiver);

    // Some help on the screen
    vis->GetGUIEnvironment()->addStaticText(
        L" Press 1: cantilever - original\n"
        L" Press 2: cantilever - modal reduction\n"
        L" Press 3: cantilever and boxes - original\n"
        L" Press 4: cantilever and boxes - modal reduction\n"
        L" Press 5: cantilever plus other assembly - original\n"
        L" Press 6: cantilever plus other assembly - modal reduction\n\n"
        L"Press space: toggle between dynamic and modal analysis",
        irr::core::rect<irr::s32>(400, 80, 850, 200), false, true, 0);

    // Change solver to PardisoMKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);

    /*
    // Use HHT second order integrator (but slower)
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    if (auto stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper())) {
        stepper->SetStepControl(false);
    }
    */

    // Note, in order to have this modal visualization  working, a ChModalAssembly must have been added to the ChSystem,
    // where some modes must have been already computed.
    vis->EnableModalAnalysis(modal_analysis);
    vis->SetModalSpeed(15);
    vis->SetModalAmplitude(0.8);
    vis->SetModalModeNumber(0);

    // Optional: open the GUI and set the tab to either Dynamics or Modal Analysis
    vis->ShowInfoPanel(true);
    vis->SetInfoTab(modal_analysis ? 1 : 0);

    // Run the sub-demos
    while (true) {
        vis->SetModalModeNumber(0);

        switch (ID_current_example) {
            case 1:
                MakeAndRunDemoCantilever(sys,
                                         false,   // no modal reduction
                                         false,   // no internal body
                                         false,   // no boundary body
                                         true,    // add force
                                         false);  // no ther assemblies
                break;
            case 2:
                MakeAndRunDemoCantilever(sys,
                                         true,    // modal reduction
                                         false,   // no internal body
                                         false,   // no boundary body
                                         true,    // add force
                                         false);  // no ther assemblies
                break;
            case 3:
                MakeAndRunDemoCantilever(sys,
                                         false,   // no modal reduction
                                         true,    // internal body
                                         true,    // boundary body
                                         true,    // add force
                                         false);  // no ther assemblies
                break;
            case 4:
                MakeAndRunDemoCantilever(sys,
                                         true,    // modal reduction
                                         true,    // internal body
                                         true,    // boundary body
                                         true,    // add force
                                         false);  // no ther assemblies
                break;
            case 5:
                MakeAndRunDemoCantilever(sys,
                                         false,  // no modal reduction
                                         true,   //    internal body
                                         true,   //    boundary body
                                         true,   //    add force
                                         true);  //    other assembly/items
                break;
            case 6:
                MakeAndRunDemoCantilever(sys,
                                         true,   //    modal reduction
                                         true,   //    internal body
                                         true,   //    boundary body
                                         true,   //    add force
                                         true);  //    other assembly/items
                break;
            default:
                break;
        }

        if (!vis->Run())
            break;
    }

    return 0;
}
