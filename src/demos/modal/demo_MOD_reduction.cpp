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
#include "chrono_modal/ChModalSolverUndamped.h"
#include "chrono_modal/ChUnsymGenEigenvalueSolver.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono/solver/ChDirectSolverLS.h"
#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::modal;
using namespace chrono::fea;
using namespace chrono::irrlicht;

// Output directory
const std::string out_dir = GetChronoOutputPath() + "MODAL_REDUCTION";

int ID_current_example = 1;
bool modal_analysis = true;

double beam_Young = 100.e6;
double beam_density = 1000;
double beam_wz = 0.3;
double beam_wy = 0.05;
double beam_L = 6;
int n_elements = 8;

// damping coefficients
double damping_alpha = 0.0001;
double damping_beta = 0.01;

double step_size = 0.05;

unsigned int num_modes = 12;
bool USE_STATIC_CORRECTION = false;
bool UPDATE_INTERNAL_NODES = true;
bool USE_LINEAR_INERTIAL_TERM = true;
bool USE_GRAVITY = false;

// static stuff for GUI:
bool UPDATE_EXAMPLE = false;
bool FIX_SUBASSEMBLY = true;
bool DO_MODAL_REDUCTION = true;
bool ADD_INTERNAL_BODY = false;
bool ADD_BOUNDARY_BODY = false;
bool ADD_FORCE = true;
bool ADD_OTHER_ASSEMBLY = false;

void CreateCantilever(ChSystem& sys,
                      ChVisualSystemIrrlicht& vis,
                      bool do_modal_reduction,
                      bool add_internal_body,
                      bool add_boundary_body,
                      bool add_force,
                      bool add_other_assemblies,
                      bool fix_subassembly) {
    std::cout << "\n\nTest parameters:\n"
              << "- reduced model: " << (do_modal_reduction ? "YES" : "NO") << "\n"
              << "- internal body: " << (add_internal_body ? "YES" : "NO") << "\n"
              << "- boundary body: " << (add_boundary_body ? "YES" : "NO") << "\n"
              << "- force: " << (add_force ? "YES" : "NO") << "\n"
              << "- other subassembly: " << (add_other_assemblies ? "YES" : "NO") << "\n"
              << "- fix_subassembly: " << (fix_subassembly ? "YES" : "NO") << std::endl;

    // Clear previous demo, if any:
    sys.Clear();
    sys.SetChTime(0);

    // CREATE THE ASSEMBLY.
    //
    // The ChModalAssembly is the most important item when doing modal analysis.
    // You must add finite elements, bodies and constraints into this assembly in order
    // to compute the modal frequencies etc.; objects not added into this won't be counted.
    auto modal_assembly = chrono_types::make_shared<ChModalAssembly>();
    modal_assembly->SetInternalNodesUpdate(UPDATE_INTERNAL_NODES);
    modal_assembly->SetUseLinearInertialTerm(USE_LINEAR_INERTIAL_TERM);
    modal_assembly->SetUseStaticCorrection(USE_STATIC_CORRECTION);
    modal_assembly->SetModalAutomaticGravity(USE_GRAVITY);
    sys.Add(modal_assembly);

    // Specify the modal reduction method used:
    // 1. HERTING: free-free modes are used as the modal basis;
    // 2. CRAIG_BAMPTON: clamped-clamped modes are used as the modal basis.
    modal_assembly->SetReductionType(ChModalAssembly::ReductionType::HERTING);

    // Now populate the assembly to analyze.
    // In this demo, make a cantilever with fixed end

    // Create two FEM meshes: one for nodes that will be removed in modal reduction,
    // the other for the nodes that will remain after modal reduction.

    auto mesh_internal = chrono_types::make_shared<ChMesh>();
    modal_assembly->AddInternal(mesh_internal);  // NOTE: MESH FOR INTERNAL NODES: USE assembly->AddInternal()

    auto mesh_boundary = chrono_types::make_shared<ChMesh>();
    modal_assembly->Add(mesh_boundary);  // NOTE: MESH FOR BOUNDARY NODES: USE assembly->Add()

    mesh_internal->SetAutomaticGravity(USE_GRAVITY);
    mesh_boundary->SetAutomaticGravity(USE_GRAVITY);

    // BEAMS:

    // Create a simplified section, i.e. thickness and material properties
    // for beams. This will be shared among some beams.
    auto section = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();

    section->SetDensity(beam_density);
    section->SetYoungModulus(beam_Young);
    section->SetShearModulusFromPoisson(0.31);
    section->SetRayleighDampingBeta(damping_beta);
    section->SetRayleighDampingAlpha(damping_alpha);
    section->SetAsRectangularSection(beam_wy, beam_wz);

    ChBuilderBeamEuler builder;

    // The first node is a boundary node: add it to mesh_boundary
    auto my_node_A_boundary = chrono_types::make_shared<ChNodeFEAxyzrot>();
    my_node_A_boundary->SetMass(0);
    my_node_A_boundary->GetInertia().setZero();
    mesh_boundary->AddNode(my_node_A_boundary);

    // The last node is a boundary node: add it to mesh_boundary
    auto my_node_B_boundary = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(beam_L, 0, 0)));
    my_node_B_boundary->SetMass(0);
    my_node_B_boundary->GetInertia().setZero();
    mesh_boundary->AddNode(my_node_B_boundary);

    // The other nodes are internal nodes: let the builder.BuildBeam add them to mesh_internal
    builder.BuildBeam(mesh_internal,       // the mesh where to put the created nodes and elements
                      section,             // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                      n_elements,          // the number of ChElementBeamEuler to create
                      my_node_A_boundary,  // the 'A' point in space (beginning of beam)
                      my_node_B_boundary,  // ChVector3d(beam_L, 0, 0), // the 'B' point in space (end of beam)
                      ChVector3d(0, 1, 0)  // the 'Y' up direction of the section for the beam
    );

    if (fix_subassembly) {
        // BODY: the base:
        auto my_body_A = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 2, 200);
        my_body_A->SetFixed(true);
        my_body_A->SetPos(ChVector3d(-0.5, 0, 0));
        modal_assembly->Add(my_body_A);

        // my_node_A_boundary->SetFixed(true); // NO - issues with bookeeping in modal_Hblock ***TO FIX***, for the
        // moment: Constraint the boundary node to truss
        auto my_root = chrono_types::make_shared<ChLinkMateGeneric>();
        my_root->Initialize(my_node_A_boundary, my_body_A, ChFrame<>(ChVector3d(0, 0, 1), QUNIT));
        modal_assembly->Add(my_root);
    } else {
        //// BODY: the base:
        // auto my_body_A = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 2, 200);
        // my_body_A->SetFixed(true);
        // my_body_A->SetPos(ChVector3d(-0.5, 0, 0));
        // sys.Add(my_body_A);

        //// Constraint the boundary node to truss
        // auto my_root = chrono_types::make_shared<ChLinkMateGeneric>();
        // my_root->Initialize(my_node_A_boundary, my_body_A, ChFrame<>(ChVector3d(0, 0, 1), QUNIT));
        // sys.Add(my_root);
    }

    if (add_internal_body) {
        // BODY: in the middle, as internal
        auto my_body_B = chrono_types::make_shared<ChBodyEasyBox>(0.9, 1.4, 1.2, 100);
        my_body_B->SetPos(ChVector3d(beam_L * 0.5, 0, 0));
        modal_assembly->AddInternal(my_body_B);

        auto my_mid_constr = chrono_types::make_shared<ChLinkMateGeneric>();
        my_mid_constr->Initialize(builder.GetLastBeamNodes()[n_elements / 2], my_body_B,
                                  ChFrame<>(ChVector3d(beam_L * 0.5, 0, 0), QUNIT));
        modal_assembly->AddInternal(my_mid_constr);
    }

    if (add_boundary_body) {
        // BODY: in the end, as boundary
        auto my_body_C = chrono_types::make_shared<ChBodyEasyBox>(0.8, 0.8, 0.8, 200);
        my_body_C->SetPos(ChVector3d(beam_L, 0, 0));
        modal_assembly->Add(my_body_C);

        auto my_end_constr = chrono_types::make_shared<ChLinkMateGeneric>();
        my_end_constr->Initialize(builder.GetLastBeamNodes().back(), my_body_C,
                                  ChFrame<>(ChVector3d(beam_L, 0, 0), QUNIT));
        modal_assembly->Add(my_end_constr);
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
        my_body_D->SetPos(ChVector3d(beam_L * 1.1, 0, 0));
        sys.Add(my_body_D);

        auto my_end_constr2 = chrono_types::make_shared<ChLinkMateGeneric>();
        my_end_constr2->Initialize(builder.GetLastBeamNodes().back(), my_body_D,
                                   ChFrame<>(ChVector3d(beam_L, 0, 0), QUNIT));
        sys.Add(my_end_constr2);

        // example if putting additional items in a second assembly (just a simple rotating blade)
        auto assembly0 = chrono_types::make_shared<ChAssembly>();
        sys.Add(assembly0);

        auto my_body_blade = chrono_types::make_shared<ChBodyEasyBox>(0.2, 0.6, 0.2, 150);
        my_body_blade->SetPos(ChVector3d(beam_L * 1.15, 0.3, 0));
        assembly0->Add(my_body_blade);

        auto rotmotor1 = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        rotmotor1->Initialize(my_body_blade,                                           // slave
                              my_body_D,                                               // master
                              ChFrame<>(my_body_D->GetPos(), QuatFromAngleY(CH_PI_2))  // motor frame, in abs. coords
        );
        auto mwspeed =
            chrono_types::make_shared<ChFunctionConst>(CH_2PI);  // constant angular speed, in [rad/s], 2PI/s =360�/s
        rotmotor1->SetSpeedFunction(mwspeed);
        assembly0->Add(rotmotor1);
    }

    if (add_force) {
        // Add a force to boundary nodes in the same way as in a full-state assembly
        my_node_B_boundary->SetForce(ChVector3d(0, -3, 0));  // to trigger some vibration at the free end

        // Add a force to internal nodes in the same way as in a full-state assembly
        auto my_node_internal =
            std::dynamic_pointer_cast<ChNodeFEAxyzrot>(modal_assembly->GetMeshesInternal().front()->GetNodes().back());
        my_node_internal->SetForce(ChVector3d(0, -1, 0));
        my_node_internal->SetTorque(ChVector3d(0, 2, 0));
    }

    // set gravity
    if (USE_GRAVITY)
        sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));  // -Z axis
    else
        sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    sys.Setup();
    sys.Update(true);

    // modal_assembly->WriteSubassemblyMatrices(true, true, true, true, out_dir + "/dump");

    if (do_modal_reduction) {
        auto eigen_solver = chrono_types::make_shared<ChUnsymGenEigenvalueSolverKrylovSchur>();
        ChModalSolverUndamped<ChUnsymGenEigenvalueSolverKrylovSchur> modal_solver(num_modes, 1e-5, true, false,
                                                                                  eigen_solver);
        ChModalDampingRayleigh modal_damping(damping_alpha, damping_beta);
        modal_assembly->DoModalReduction(modal_solver, modal_damping);
    }

    // VISUALIZATION ASSETS:

    auto visualizeInternalA = chrono_types::make_shared<ChVisualShapeFEA>();
    visualizeInternalA->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MY);
    visualizeInternalA->SetColormapRange(-600, 600);
    visualizeInternalA->SetSmoothFaces(true);
    visualizeInternalA->SetWireframe(false);
    mesh_internal->AddVisualShapeFEA(visualizeInternalA);

    auto visualizeInternalB = chrono_types::make_shared<ChVisualShapeFEA>();
    visualizeInternalB->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    visualizeInternalB->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    visualizeInternalB->SetSymbolsThickness(0.2);
    visualizeInternalB->SetSymbolsScale(0.1);
    visualizeInternalB->SetZbufferHide(false);
    mesh_internal->AddVisualShapeFEA(visualizeInternalB);

    auto visualizeBoundaryB = chrono_types::make_shared<ChVisualShapeFEA>();
    visualizeBoundaryB->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    visualizeBoundaryB->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    visualizeBoundaryB->SetSymbolsThickness(0.4);
    visualizeBoundaryB->SetSymbolsScale(4);
    visualizeBoundaryB->SetZbufferHide(false);
    mesh_boundary->AddVisualShapeFEA(visualizeBoundaryB);

    // This is needed if you want to see things in Irrlicht
    vis.BindAll();
}

// Custom event manager
class MyEventReceiver : public irr::IEventReceiver {
  public:
    MyEventReceiver(ChVisualSystemIrrlicht& vis) : m_vis(vis) {}

    bool OnEvent(const irr::SEvent& event) {
        if (event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) {
            switch (event.KeyInput.Key) {
                case irr::KEY_KEY_1:
                    UPDATE_EXAMPLE = true;
                    DO_MODAL_REDUCTION = !DO_MODAL_REDUCTION;
                    return true;
                case irr::KEY_KEY_2:
                    UPDATE_EXAMPLE = true;
                    ADD_INTERNAL_BODY = !ADD_INTERNAL_BODY;
                    return true;
                case irr::KEY_KEY_3:
                    UPDATE_EXAMPLE = true;
                    ADD_BOUNDARY_BODY = !ADD_BOUNDARY_BODY;
                    return true;
                case irr::KEY_KEY_4:
                    UPDATE_EXAMPLE = true;
                    ADD_FORCE = !ADD_FORCE;
                    return true;
                case irr::KEY_KEY_5:
                    UPDATE_EXAMPLE = true;
                    ADD_OTHER_ASSEMBLY = !ADD_OTHER_ASSEMBLY;
                    return true;
                case irr::KEY_KEY_6:
                    UPDATE_EXAMPLE = true;
                    FIX_SUBASSEMBLY = !FIX_SUBASSEMBLY;
                    return true;
                default:
                    break;
            }
        }
        return false;
    }

  private:
    ChVisualSystemIrrlicht& m_vis;
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

    // VISUALIZATION

    // Create the Irrlicht visualization system
    ChVisualSystemIrrlicht vis;
    vis.AttachSystem(&sys);
    vis.SetWindowSize(1024, 768);
    vis.SetWindowTitle("Modal Reduction");
    vis.Initialize();
    vis.AddLogo();
    vis.AddSkyBox();
    vis.AddCamera(ChVector3d(1, 1.3, 6), ChVector3d(3, 0, 0));
    vis.AddTypicalLights();

    // This is for GUI tweaking of system parameters..
    MyEventReceiver receiver(vis);
    // note how to add a custom event receiver to the default interface:
    vis.AddUserEventReceiver(&receiver);

    // Some help on the screen
    auto my_gui_info =
        vis.GetGUIEnvironment()->addStaticText(L" ", irr::core::rect<irr::s32>(400, 80, 850, 200), false, true, 0);

    // Set linear solver
#ifdef CHRONO_PARDISO_MKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);
#else
    auto qr_solver = chrono_types::make_shared<ChSolverSparseQR>();
    sys.SetSolver(qr_solver);
#endif

    // Use HHT second order integrator (but slower)
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    if (auto hht_stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper())) {
        hht_stepper->SetVerbose(false);
        hht_stepper->SetStepControl(false);
        hht_stepper->SetAlpha(-0.2);
        hht_stepper->SetJacobianUpdateMethod(ChTimestepperImplicit::JacobianUpdate::EVERY_STEP);
    }

    UPDATE_EXAMPLE = true;

    // Run the sub-demos
    while (vis.Run()) {
        my_gui_info->setText((std::wstring(L"Reduced model (button 1): ") + (DO_MODAL_REDUCTION ? L"ON" : L"OFF") +
                              std::wstring(L"\nInternal body (button 2): ") + (ADD_INTERNAL_BODY ? L"ON" : L"OFF") +
                              std::wstring(L"\nBoundary body (button 3): ") + (ADD_BOUNDARY_BODY ? L"ON" : L"OFF") +
                              std::wstring(L"\nForces (button 4): ") + (ADD_FORCE ? L"ON" : L"OFF") +
                              std::wstring(L"\nOther subassembly (button 5): ") +
                              (ADD_OTHER_ASSEMBLY ? L"ON" : L"OFF") + std::wstring(L"\nFixed assembly (button 6): ") +
                              (FIX_SUBASSEMBLY ? L"ON" : L"OFF"))
                                 .c_str());

        if (UPDATE_EXAMPLE) {
            CreateCantilever(sys, vis, DO_MODAL_REDUCTION, ADD_INTERNAL_BODY, ADD_BOUNDARY_BODY, ADD_FORCE,
                             ADD_OTHER_ASSEMBLY, FIX_SUBASSEMBLY);
            UPDATE_EXAMPLE = false;
        }

        vis.BeginScene();
        vis.Render();
        tools::drawGrid(&vis, 1, 1, 12, 12, ChCoordsys<>(ChVector3d(0, 0, 0), CH_PI_2, VECT_Z),
                        ChColor(0.5f, 0.5f, 0.5f), true);

        sys.DoStepDynamics(step_size);

        vis.EndScene();
    }

    return 0;
}
