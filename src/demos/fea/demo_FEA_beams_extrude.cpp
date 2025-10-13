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
// FEA for 3D beams
//
// =============================================================================

#include <cmath>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"

#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/assets/ChVisualShapeFEA.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

// A helper function that creates a'lobed gear', to quickly create one or two rotating obstacles for the extruding beam
std::shared_ptr<ChBody> CreateLobedGear(ChVector3d gear_center,
                                        int lobe_copies,
                                        double lobe_width,
                                        double lobe_primitive_rad,
                                        double lobe_inner_rad,
                                        double lobe_outer_rad,
                                        double lobe_thickness,
                                        ChSystem& sys,
                                        std::shared_ptr<ChContactMaterial> contact_mat) {
    auto mgear = chrono_types::make_shared<ChBody>();
    mgear->SetPos(gear_center);
    sys.Add(mgear);

    // cylindrical lobes
    for (int i = 0; i < lobe_copies; ++i) {
        double phase = CH_2PI * ((double)i / (double)lobe_copies);
        ChVector3d loc(lobe_primitive_rad * std::sin(phase), lobe_primitive_rad * std::cos(phase), 0);
        // shortcut from ChUtilsCreators.h: adds both collision shape and visualization asset
        chrono::utils::AddCylinderGeometry(mgear.get(), contact_mat,             //
                                           lobe_width * 0.5, lobe_thickness * 0.5,  //
                                           loc,                                     //
                                           QUNIT,                                   // cylinder axis along Z
                                           true);
    }
    // central hub
    chrono::utils::AddCylinderGeometry(mgear.get(), contact_mat, lobe_inner_rad, lobe_thickness * 0.5, VNULL, QUNIT,
                                       true);
    mgear->EnableCollision(true);

    return mgear;
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create a Chrono physical system
    ChSystemSMC sys;

    // Create and set the collision system
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.001);
    ChCollisionModel::SetDefaultSuggestedMargin(0.002);
    ChCollisionSystemBullet::SetContactBreakingThreshold(0.0001);
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create a ground object, useful reference for connecting constraints etc.
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.Add(ground);

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto mesh = chrono_types::make_shared<ChMesh>();
    sys.Add(mesh);

    // Create a section, i.e. thickness and material properties for beams. This will be shared among some beams.
    double wire_diameter = 0.010;

    auto inertia = chrono_types::make_shared<ChInertiaCosseratSimple>();
    inertia->SetAsCircularSection(wire_diameter, 2700);  // automatically sets A etc., from width, height, density

    auto elasticity = chrono_types::make_shared<ChElasticityCosseratSimple>();
    elasticity->SetYoungModulus(0.5e9);
    elasticity->SetShearModulus(0.5e9 * 0.7);
    elasticity->SetAsCircularSection(wire_diameter);

    auto damping = chrono_types::make_shared<ChDampingCosseratLinear>();
    damping->SetDampingCoefficientsRe((1e-3) * ChVector3d(1, 1, 1));
    damping->SetDampingCoefficientsRk((1e-4) * ChVector3d(1, 1, 1));  //// TODO??? -/+

    auto plasticity = chrono_types::make_shared<ChPlasticityCosseratLumped>();
    plasticity->n_yeld_Mx = chrono_types::make_shared<ChFunctionRamp>(1, 0.01);
    plasticity->n_yeld_My = chrono_types::make_shared<ChFunctionRamp>(0.2, 0.001);
    plasticity->n_yeld_Mz = chrono_types::make_shared<ChFunctionRamp>(0.2, 0.001);

    auto section = chrono_types::make_shared<ChBeamSectionCosserat>(inertia, elasticity, plasticity, damping);

    section->SetCircular(true);
    section->SetDrawCircularRadius(wire_diameter / 2.);

    // Create the surface material for the contacts; this contains information about friction etc.
    // It is a SMC (penalty) material: interpenetration might happen for low Young stiffness,
    // but unstable simulation might happen for high stiffness, requiring smaller timesteps.

    /*
    // option A: Hertz contact force model
    sys.SetContactForceModel(ChSystemSMC::ContactForceModel::Hertz);
    auto contact_mat = chrono_types::make_shared<ChContactMaterialSMC>();
    contact_mat->SetYoungModulus(20e3);  // to adjust heuristically..
    contact_mat->SetRestitution(0.1f);
    contact_mat->SetFriction(0.2f);
    */

    // Option B: Hooke force model
    sys.SetContactForceModel(ChSystemSMC::ContactForceModel::Hooke);
    sys.UseMaterialProperties(false);
    auto contact_mat = chrono_types::make_shared<ChContactMaterialSMC>();
    contact_mat->SetKn(350);  // contact normal stiffness
    contact_mat->SetKt(350);  // contact tangential stiffness
    contact_mat->SetGn(25);   // contact normal damping
    contact_mat->SetGt(25);   // contact tangential damping
    contact_mat->SetFriction(0.2f);

    // Add the EXTRUDER
    auto extruder = chrono_types::make_shared<ChExtruderBeamIGA>(
        &sys,      // the physical system
        mesh,   // the mesh where to add the beams
        section,  // section for created beam
        0.015,     // beam element length (size of discretization: the smaller, the more precise)
        ChCoordsys<>(VNULL, Q_ROTATE_Z_TO_X),  // outlet coordinate system (z axis is the extrusion dir)
        0.08,                                  // the extrusion speed
        1                                      // the order of beams
    );

    // Enable collision for extruded beam
    extruder->SetContact(contact_mat,             // the NSC material for contact surfaces
                         1.15 * wire_diameter * 0.5  // the radius of the collision spheres at the nodes, (enlarge 15%)
    );

    // Do we want gravity effect on FEA elements in this demo?
    mesh->SetAutomaticGravity(false);

    // Attach visualization of the FEM mesh.
    auto vis_element = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_element->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
    vis_element->SetColormapRange(-0.4, 0.4);
    vis_element->SetSmoothFaces(true);
    vis_element->SetWireframe(false);
    mesh->AddVisualShapeFEA(vis_element);

    auto vis_node = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_node->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    vis_node->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    vis_node->SetSymbolsThickness(wire_diameter);
    vis_node->SetSymbolsScale(0.01);
    vis_node->SetZbufferHide(false);
    mesh->AddVisualShapeFEA(vis_node);

    // Add some obstacles: two rotating lobed gears.
    //
    // Here create two rotating lobed gears, just for fun, that wil trap the
    // extruded beam. To quickly create them, use the CreateLobedGear() function
    // implemented at the top of this file.
    // Also, create two simple constant speed motors to rotate the lobed gears.
    int lobe_copies = 8;
    double lobe_width = 0.03;
    double lobe_primitive_rad = 0.3;
    double lobe_inner_rad = 0.13;
    double lobe_outer_rad = 0.34;
    double lobe_thickness = 0.08;
    ChVector3d gear_centerLOW(0.3, -lobe_primitive_rad + 0.01, 0);
    ChVector3d gear_centerHI(0.3, lobe_primitive_rad - 0.01, 0);

    auto gearLOW = CreateLobedGear(gear_centerLOW, lobe_copies, lobe_width, lobe_primitive_rad, lobe_inner_rad,
                                   lobe_outer_rad, lobe_thickness, sys, contact_mat);

    auto gear_motorLOW = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    gear_motorLOW->Initialize(gearLOW, ground, ChFrame<>(gear_centerLOW));
    sys.Add(gear_motorLOW);

    auto gear_speedLOW = chrono_types::make_shared<ChFunctionConst>(-0.18);  // [rad/s]
    gear_motorLOW->SetSpeedFunction(gear_speedLOW);

    auto gearHI = CreateLobedGear(gear_centerHI, lobe_copies, lobe_width, lobe_primitive_rad, lobe_inner_rad,
                                  lobe_outer_rad, lobe_thickness, sys, contact_mat);
    gearHI->SetRot(QuatFromAngleZ(0.5 * CH_2PI / lobe_copies));  // to phase half step respect to other gear

    auto gear_motorHI = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    gear_motorHI->Initialize(gearHI, ground, ChFrame<>(gear_centerHI));
    sys.Add(gear_motorHI);

    auto gear_speedHI = chrono_types::make_shared<ChFunctionConst>(0.18);  // [rad/s]
    gear_motorHI->SetSpeedFunction(gear_speedHI);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Beam continuous extrusion and FEA contacts");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(-0.2, 0, 0.3));
    vis->AttachSystem(&sys);

    // SIMULATION LOOP

    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    mkl_solver->LockSparsityPattern(true);
    sys.SetSolver(mkl_solver);

    double step = 0.0002;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        tools::drawGrid(vis.get(), 0.1, 0.1, 20, 20, CSYSNORM, ChColor(0.4f, 0.4f, 0.4f), true);

        sys.DoStepDynamics(step);

        bool modified = extruder->Update();  //// NOTE to do this to update the extrusion
        if (modified) {
            // A system change occurred: if using a sparse direct linear solver and if using the sparsity pattern
            // learner (enabled by default), then we must force a re-evaluation of system matrix sparsity pattern!
            mkl_solver->ForceSparsityPatternUpdate();
        }

        vis->EndScene();
    }

    return 0;
}
