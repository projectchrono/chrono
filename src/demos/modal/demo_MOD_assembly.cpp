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
#include "chrono/fea/ChVisualizationFEAmesh.h"

#include "chrono_modal/ChModalAssembly.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_postprocess/ChGnuPlot.h"

using namespace chrono;
using namespace chrono::modal;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace chrono::postprocess;
using namespace irr;

// Output directory
const std::string out_dir = GetChronoOutputPath() + "MODAL_ASSEMBLY";

int ID_current_example = 1;

double beam_Young = 100.e6;
double beam_density = 1000;
double beam_wz = 0.3;
double beam_wy = 0.05;
double beam_L = 6;
int n_elements = 8;

void MakeAndRunDemoCantilever(ChIrrApp& myapp, bool do_modal_reduction, bool add_internal_body,  bool add_boundary_body, bool add_force) 
{
    ChSystem* my_system = myapp.GetSystem();
    // Clear previous demo, if any:
    my_system->Clear();
    my_system->SetChTime(0);

    // CREATE THE ASSEMBLY. 
    //
    // The ChModalAssembly is the most important item when doing modal analysis. 
    // You must add finite elements, bodies and constraints into this assembly in order
    // to compute the modal frequencies etc.; objects not added into this won't be counted.

    auto my_assembly = chrono_types::make_shared<ChModalAssembly>();
    my_system->Add(my_assembly);
    
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
    my_assembly->Add(my_mesh_boundary);          // NOTE: MESH FOR BOUNDARY NODES: USE my_assembly->Add()

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
    my_node_A_boundary->SetMass(0); my_node_A_boundary->GetInertia().setZero(); 
    my_mesh_boundary->AddNode(my_node_A_boundary); 

    // The last node is a boundary node: add it to my_mesh_boundary
    auto my_node_B_boundary = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_L, 0, 0)));
    my_node_B_boundary->SetMass(0); my_node_B_boundary->GetInertia().setZero();
    my_mesh_boundary->AddNode(my_node_B_boundary); 

    //my_node_A_boundary->SetFixed(true); // NO - issues with bookeeping in modal_Hblock ***TO FIX***, for the moment: use constraint:
    // Constraint the boundary node to truss
    auto my_root = chrono_types::make_shared<ChLinkMateGeneric>();
    my_root->Initialize(my_node_A_boundary, my_body_A, ChFrame<>(ChVector<>(0, 0, 1), QUNIT));
    my_assembly->Add(my_root);

    // The other nodes are internal nodes: let the builder.BuildBeam add them to my_mesh_internal
    builder.BuildBeam(
        my_mesh_internal,   // the mesh where to put the created nodes and elements
        msection,           // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
        n_elements,         // the number of ChElementBeamEuler to create
        my_node_A_boundary, // the 'A' point in space (beginning of beam)
        my_node_B_boundary, //ChVector<>(beam_L, 0, 0), // the 'B' point in space (end of beam)
        ChVector<>(0, 1, 0) // the 'Y' up direction of the section for the beam
    );

    //my_node_B_boundary->SetForce(ChVector<>(0, 0, 90)); // to trigger some vibration at the free end

    if (add_internal_body) {

        // BODY: in the middle, as internal 
        auto my_body_B = chrono_types::make_shared<ChBodyEasyBox>(1.8, 1.8, 1.8, 200);
        my_body_B->SetPos(ChVector<>(beam_L * 0.5, 0, 0));
        my_assembly->AddInternal(my_body_B);

        auto my_mid_constr = chrono_types::make_shared<ChLinkMateGeneric>();
        my_mid_constr->Initialize(builder.GetLastBeamNodes()[n_elements/2], my_body_B, ChFrame<>(ChVector<>(beam_L * 0.5, 0, 0), QUNIT));
        my_assembly->AddInternal(my_mid_constr);
    }

    if (add_boundary_body) {

        // BODY: in the end, as boundary 
        auto my_body_C = chrono_types::make_shared<ChBodyEasyBox>(0.8, 0.8, 0.8, 200);
        my_body_C->SetPos(ChVector<>(beam_L, 0, 0));
        my_assembly->Add(my_body_C);

        auto my_end_constr = chrono_types::make_shared<ChLinkMateGeneric>();
        my_end_constr->Initialize(builder.GetLastBeamNodes().back(), my_body_C, ChFrame<>(ChVector<>(beam_L, 0, 0), QUNIT));
        my_assembly->Add(my_end_constr);
    }
    
    if (add_force) {
        // Add a force (also to internal nodes that will be removed after modal reduction).
        // This can be done using a callback that will be called all times the time integrator needs it.
        // You will provide a custom force writing into computed_custom_F_full vector (note: it is up to you to use the proper indexes)
        class MyCallback : public ChModalAssembly::CustomForceFullCallback {
        public:
            MyCallback() {};
            virtual void evaluate(ChVectorDynamic<>& computed_custom_F_full, //< compute F here, size= n_boundary_coords_w + n_internal_coords_w
                const ChModalAssembly& link  ///< associated modal assembly
            ) {
                computed_custom_F_full.setZero(); // remember! assume F vector is already properly sized, but not zeroed!
                computed_custom_F_full[computed_custom_F_full.size() - 16] = -60; // just for test, assign a force to a random coordinate of F, here an internal node
            }
        };
        auto my_callback = chrono_types::make_shared<MyCallback>();

        my_assembly->RegisterCallback_CustomForceFull(my_callback);
    }
    

    // Just for later reference, dump  M,R,K,Cq matrices. Ex. for comparison with Matlab eigs()
    my_system->Setup();
    my_system->Update();
    my_assembly->DumpSubassemblyMatrices(true, true, true, true, (out_dir+"/dump").c_str());


    if (do_modal_reduction) {

        // HERE PERFORM THE MODAL REDUCTION!

        my_assembly->SwitchModalReductionON(
            6,                                        // The number of modes to retain from modal reduction 
            ChModalDampingRayleigh(0.01, 0.05)        // The damping model - Optional parameter: default is ChModalDampingNone().
        );

        // Other types of damping that you can try, in SwitchModalReductionON: 
        //    ChModalDampingNone()                    // no damping (also default)
        //    ChModalDampingReductionR(*my_assembly)  // transforms the original damping matrix of the full subassembly
        //    ChModalDampingReductionR(full_R_ext)    // transforms an externally-provided damping matrix of the full subassembly
        //    ChModalDampingCustom(reduced_R_ext)     // uses an externally-provided damping matrix of the reduced subassembly
        //    ChModalDampingRayleigh(0.01, 0.05)      // generates a damping matrix from reduced M ad K using Rayleygh alpha-beta
        //    ChModalDampingFactorRmm(zetas)          // generates a damping matrix from damping factors zetas of dynamic modes
        //    ChModalDampingFactorRayleigh(zetas,a,b) // generates a damping matrix from damping factors of dynamic modes and rayleigh a,b for boundary nodes
        //    ChModalDampingFactorAssembly(zetas)     // (not ready) generates a damping matrix from damping factors of the modes of the subassembly, including masses of boundary
        //      where for example         ChVectorDynamic<> zetas(4);  zetas << 0.7, 0.5, 0.6, 0.7; // damping factors, other values assumed as last one.

        // OPTIONAL STUFF: 

        // Just for later reference, dump reduced M,R,K,Cq matrices. Ex. for comparison with Matlab eigs()
        my_assembly->DumpSubassemblyMatrices(true, true, true, true, (out_dir+"/dump_reduced").c_str());

        // Use this for high simulation performance (the internal nodes won't be updated for postprocessing)
        //my_assembly->SetInternalNodesUpdate(false);

        // Finally, optional: do an eigenvalue analysis to check if we approximately have the same eigenmodes of the original not reduced assembly:
        my_assembly->ComputeModesDamped(0);

        // Just for logging the frequencies:
        for (int i = 0; i < my_assembly->Get_modes_frequencies().rows(); ++i)
            GetLog()<< "Mode n." << i
                    << "  frequency [Hz]: " << my_assembly->Get_modes_frequencies()(i) 
                    << "   damping factor z: " <<  my_assembly->Get_modes_damping_ratios()(i)
                    << "\n";
    }
    else {

        // Otherwise we perform a conventional modal analysis on the full ChModalAssembly.
        my_assembly->ComputeModes(12);

        // Just for logging the frequencies:
        for (int i = 0; i < my_assembly->Get_modes_frequencies().rows(); ++i)
            GetLog()<< "Mode n." << i
                    << "  frequency [Hz]: " << my_assembly->Get_modes_frequencies()(i) 
                    << "\n";
    }



    //
    // VISUALIZATION ASSETS:
    //

    auto mvisualizeInternalA = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh_internal.get()));
    mvisualizeInternalA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MY);
    mvisualizeInternalA->SetColorscaleMinMax(-600, 600);
    mvisualizeInternalA->SetSmoothFaces(true);
    mvisualizeInternalA->SetWireframe(false);
    my_mesh_internal->AddAsset(mvisualizeInternalA);

    auto mvisualizeInternalB = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh_internal.get()));
    mvisualizeInternalB->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    mvisualizeInternalB->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizeInternalB->SetSymbolsThickness(0.2);
    mvisualizeInternalB->SetSymbolsScale(0.1);
    mvisualizeInternalB->SetZbufferHide(false);
    my_mesh_internal->AddAsset(mvisualizeInternalB);

    auto mvisualizeBoundaryB = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh_boundary.get()));
    mvisualizeBoundaryB->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    mvisualizeBoundaryB->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizeBoundaryB->SetSymbolsThickness(0.4);
    mvisualizeBoundaryB->SetSymbolsScale(4);
    mvisualizeBoundaryB->SetZbufferHide(false);
    my_mesh_boundary->AddAsset(mvisualizeBoundaryB);



    // This is needed if you want to see things in Irrlicht 3D view.
    myapp.AssetBindAll();
    myapp.AssetUpdateAll();

    int current_example = ID_current_example;
    while ((ID_current_example == current_example) && myapp.GetDevice()->run()) {
        myapp.BeginScene();
        myapp.DrawAll();
        tools::drawGrid(myapp.GetVideoDriver(), 1, 1, 12, 12,
                             ChCoordsys<>(ChVector<>(0, 0, 0), CH_C_PI_2, VECT_Z),
                             video::SColor(100, 120, 120, 120), true);

        myapp.DoStep();
        myapp.EndScene();
    }
}

/// Following class will be used to manage events from the user interface

class MyEventReceiver : public IEventReceiver {
  public:
    MyEventReceiver(ChIrrApp* myapp) {
        // store pointer to physical system & other stuff so we can tweak them by user keyboard
        app = myapp;
    }

    bool OnEvent(const SEvent& event) {
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
                default:
                    break;
            }
        }

        return false;
    }

  private:
    ChIrrApp* app;
};



int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2021 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Directory for output data
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    //
    // CREATE THE MODEL
    //

    // Create a Chrono::Engine physical system
    ChSystemNSC my_system;

    // no gravity used here
    my_system.Set_G_acc(VNULL);
 

    //
    // VISUALIZATION
    //

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Modal reduction", core::dimension2d<u32>(1024, 768));

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddLogo();
    application.AddSkyBox();
    application.AddLightWithShadow(irr::core::vector3df(20, 20, 20), irr::core::vector3df(0, 0, 0), 50, 5, 50, 55);
    application.AddLight(irr::core::vector3df(-20, -20, 0), 6, irr::video::SColorf(0.6f, 1.0f, 1.0f, 1.0f));
    application.AddLight(irr::core::vector3df(0, -20, -20), 6, irr::video::SColorf(0.6f, 1.0f, 1.0f, 1.0f));
    // application.AddTypicalLights();
    application.AddCamera(core::vector3df(1.f, 1.3f, 6.f), core::vector3df(3.f, 0.f, 0.f));


    // This is for GUI tweaking of system parameters..
    MyEventReceiver receiver(&application);
    // note how to add a custom event receiver to the default interface:
    application.SetUserEventReceiver(&receiver);

    // Some help on the screen
    application.GetIGUIEnvironment()->addStaticText(
        L" Press 1: cantilever - original \n"
        L" Press 2: cantilever - modal reduction \n"
        L" Press 3: cantilever and boxes - original \n"
        L" Press 4: cantilever and boxes - modal reduction \n"
        L" (Switch off 'show modal shape' to start simulating) \n",
        irr::core::rect<irr::s32>(400, 80, 650, 200), false, true, 0);



    // Some general settings for dynamics. NOTE: this demo is not using dynamics, just modal analysis,
    // but settings are here just in case one wants to do other tests.

    // Change solver to PardisoMKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    my_system.SetSolver(mkl_solver);

    application.SetTimestep(0.05);
   
    /*
    // use HHT second order integrator (but slower)
    my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper())) {
        mystepper->SetStepControl(false);
    }
    */
    
    // Need to show modes in 3D? No changes in your while() loop! In fact if you set application.SetModalShow(true), 
    // the application.BeginScene() in the simulation loop will overimpose the nth mode shape, displayed in 3D as a 
    // continuous oscillation; at the same time application.DoStep() won't advance dynamics. 
    // Note, in order to have this modal visualization  working, a ChModalAssembly must have been added to the ChSystem, 
    // where some modes must have been already computed.
    application.SetModalShow(true);
    application.SetModalSpeed(15);
    application.SetModalAmplitude(0.8);
    application.SetModalModeNumber(0);

    // Optional: this opens the GUI for changing the N of the mode shape via a slider in the Irrlicht view:
    application.SetShowInfos(true);
    application.SetInfosTab(1);

    // Run the sub-demos:

    while (true) {

        application.SetModalModeNumber(0);

        switch (ID_current_example) {
            case 1:
                MakeAndRunDemoCantilever(application,
                    false,    // no modal reduction
                    false,    // no internal body
                    false,    // no boundary body
                    true);    // add force
                break;
            case 2:
                MakeAndRunDemoCantilever(application, 
                    true,     //    modal reduction
                    false,    // no internal body
                    false,    // no boundary body
                    true);    // add force
                break;
            case 3:
                MakeAndRunDemoCantilever(application,
                    false,    // no modal reduction
                    true,     //    internal body
                    true,     //    boundary body
                    true);    // add force
                break;
            case 4:
                MakeAndRunDemoCantilever(application, 
                    true,     //    modal reduction
                    true,     //    internal body
                    true,     //    boundary body
                    true);    // add force
                break;
            default:
                break;
        }
       
        if (!application.GetDevice()->run())
            break;
    }



 
    return 0;
}

