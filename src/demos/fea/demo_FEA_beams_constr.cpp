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
// FEA for 3D beams and constraints
//
// =============================================================================

#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/assets/ChVisualShapeFEA.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

// Output directory
const std::string out_dir = GetChronoOutputPath() + "BEAM_BUCKLING";

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    /*
    //***TEST***
        ChVector<> mFi, mTi;
        ChVector<> mWvel = (1, 2, 3);
        ChVector<> mWacc =  (0.3, -0.2, 0.4);
        ChVector<> mXacc =  (0.5, 0.6, -0.9); // ok
        ChMatrixNM<double,6,6> mMi, mKi, mRi;
        ChMatrixNM<double,6,6> mKi_num, mRi_num;
        auto minertia1 = chrono_types::make_shared<ChInertiaCosseratAdvanced>(270.0, 0.1, 0.2, 5, 8, 0.5);
        minertia1->ComputeQuadraticTerms(mFi, mTi, mWvel);
        GetLog() << "ComputeQuadraticTerms: \n";
        GetLog() << " Fi_v= " << mFi;
        GetLog() << " Ti_v = " << mTi;
        minertia1->ComputeInertialForce(mFi, mTi, mWvel, mWacc, mXacc);
        GetLog() << "ComputeInertialForce: \n";
        GetLog() << " Fi = " << mFi;
        GetLog() << " Ti = " << mTi;
        minertia1->ComputeInertiaMatrix(mMi);
        GetLog() << " Mi: \n" << mMi << "\n";
        minertia1->ComputeInertiaDampingMatrix(mRi,mWvel);
        GetLog() << " Ri: \n" << mRi << "\n";
        minertia1->ComputeInertiaStiffnessMatrix(mKi,mWvel,mWacc,mXacc);
        GetLog() << " Ki: \n" << mKi << "\n";
        minertia1->compute_Ri_Ki_by_num_diff = true;
        minertia1->ComputeInertiaDampingMatrix(mRi_num,mWvel);
        GetLog() << " Ri_num: \n" << mRi_num << "\n";
        minertia1->ComputeInertiaStiffnessMatrix(mKi_num,mWvel,mWacc,mXacc);
        GetLog() << " Ki_num: \n" << mKi_num << "\n";
     return 1;
     */
    // Create a Chrono::Engine physical system
    ChSystemSMC sys;

    double L = 1;
    double H = 0.25;
    double K = 0.05;
    ChVector<> vA(0, 0, 0);
    ChVector<> vC(L, 0, 0);
    ChVector<> vB(L, -H, 0);
    ChVector<> vG(L - K, -H, 0);
    ChVector<> vd(0, 0, 0.0001);

    // Create a truss:
    auto body_truss = chrono_types::make_shared<ChBody>();
    body_truss->SetBodyFixed(true);

    sys.AddBody(body_truss);

    // Attach a 'box' shape asset for visualization.
    auto mboxtruss = chrono_types::make_shared<ChVisualShapeBox>(0.02, 0.2, 0.1);
    body_truss->AddVisualShape(mboxtruss, ChFrame<>(ChVector<>(-0.01, 0, 0)));

    // Create body for crank
    auto body_crank = chrono_types::make_shared<ChBody>();

    body_crank->SetPos((vB + vG) * 0.5);
    sys.AddBody(body_crank);

    // Attach a 'box' shape asset for visualization.
    auto mboxcrank = chrono_types::make_shared<ChVisualShapeBox>(K, 0.02, 0.02);
    body_crank->AddVisualShape(mboxcrank, ChFrame<>());

    // Create a motor between the truss and the crank:
    class ChFunction_myf : public ChFunction {
      public:
        virtual ChFunction_myf* Clone() const override { return new ChFunction_myf(); }

        virtual double Get_y(double x) const override {
            if (x > 0.4)
                return CH_C_PI;
            else
                return -CH_C_PI * (1.0 - cos(CH_C_PI * x / 0.4)) / 2.0;
        }
    };

    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->Initialize(body_truss, body_crank, ChFrame<>(vG));
    motor->SetAngleFunction(chrono_types::make_shared<ChFunction_myf>());
    sys.Add(motor);

    // Create a FEM mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create the horizontal beam (use an IGA-beam finite element type, for example)

    double beam_wy = 0.10;
    double beam_wz = 0.01;

    // Create a section for the IGA beam.
    // IGA beams require ChBeamSectionCosserat sections, containing at least
    // a ChElasticityCosserat and ChInertiaCosserat models, and optional ChDampingCosserat and ChPlasticityCosserat.

    auto minertia = chrono_types::make_shared<ChInertiaCosseratSimple>();
    minertia->SetAsRectangularSection(beam_wy, beam_wz,
                                      2700);  // automatically sets A etc., from width, height, density

    auto melasticity = chrono_types::make_shared<ChElasticityCosseratSimple>();
    melasticity->SetYoungModulus(73.0e9);
    melasticity->SetGwithPoissonRatio(0.3);
    melasticity->SetAsRectangularSection(beam_wy, beam_wz);

    auto msection1 = chrono_types::make_shared<ChBeamSectionCosserat>(minertia, melasticity);

    msection1->SetDrawThickness(beam_wy, beam_wz);

    ChBuilderBeamIGA builder_iga;
    builder_iga.BuildBeam(my_mesh,    // the mesh to put the elements in
                          msection1,  // section of the beam
                          32,         // number of sections (spans)
                          vA,         // start point
                          vC,         // end point
                          VECT_Y,     // suggested Y direction of section
                          3);         // order (3 = cubic, etc)
    builder_iga.GetLastBeamNodes().front()->SetFixed(true);
    auto node_tip = std::shared_ptr<ChNodeFEAxyzrot>(builder_iga.GetLastBeamNodes().back());
    auto node_mid = std::shared_ptr<ChNodeFEAxyzrot>(builder_iga.GetLastBeamNodes()[17]);

    // Create the vertical beam (Here use Euler beams, for example).
    auto msection2 = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();

    double hbeam_d = 0.024;
    msection2->SetDensity(2700);
    msection2->SetYoungModulus(73.0e9);
    msection2->SetGwithPoissonRatio(0.3);
    msection2->SetBeamRaleyghDamping(0.000);
    msection2->SetAsCircularSection(hbeam_d);

    ChBuilderBeamEuler builder;
    builder.BuildBeam(my_mesh,               // the mesh where to put the created nodes and elements
                      msection2,             // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                      3,                     // the number of ChElementBeamEuler to create
                      vC + vd,               // the 'A' point in space (beginning of beam)
                      vB + vd,               // the 'B' point in space (end of beam)
                      ChVector<>(1, 0, 0));  // the 'Y' up direction of the section for the beam
    auto node_top = std::shared_ptr<ChNodeFEAxyzrot>(builder.GetLastBeamNodes().front());
    auto node_down = std::shared_ptr<ChNodeFEAxyzrot>(builder.GetLastBeamNodes().back());

    // Create a constraint between the vertical and horizontal beams:
    auto constr_bb = chrono_types::make_shared<ChLinkMateGeneric>();
    constr_bb->Initialize(node_top, node_tip, false, node_top->Frame(), node_top->Frame());
    sys.Add(constr_bb);

    constr_bb->SetConstrainedCoords(true, true, true,      // x, y, z
                                    false, false, false);  // Rx, Ry, Rz

    // For example, attach small shape to show the constraint
    auto msphereconstr2 = chrono_types::make_shared<ChVisualShapeSphere>(0.01);
    constr_bb->AddVisualShape(msphereconstr2);

    // Create a beam as a crank
    auto msection3 = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();

    double crankbeam_d = 0.048;
    msection3->SetDensity(2700);
    msection3->SetYoungModulus(73.0e9);
    msection3->SetGwithPoissonRatio(0.3);
    msection3->SetBeamRaleyghDamping(0.000);
    msection3->SetAsCircularSection(crankbeam_d);

    builder.BuildBeam(my_mesh,               // the mesh where to put the created nodes and elements
                      msection3,             // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                      3,                     // the number of ChElementBeamEuler to create
                      vG + vd,               // the 'A' point in space (beginning of beam)
                      vB + vd,               // the 'B' point in space (end of beam)
                      ChVector<>(0, 1, 0));  // the 'Y' up direction of the section for the beam

    auto node_crankG = std::shared_ptr<ChNodeFEAxyzrot>(builder.GetLastBeamNodes().front());
    auto node_crankB = std::shared_ptr<ChNodeFEAxyzrot>(builder.GetLastBeamNodes().back());
    // Create a constraint between the crank beam and body crank:
    auto constr_cbd = chrono_types::make_shared<ChLinkMateGeneric>();
    constr_cbd->Initialize(node_crankG, body_crank, false, node_crankG->Frame(), node_crankG->Frame());
    sys.Add(constr_cbd);

    constr_cbd->SetConstrainedCoords(true, true, true,   // x, y, z
                                     true, true, true);  // Rx, Ry, Rz

    // Create a constraint between the vertical beam and the crank beam:
    auto constr_bc = chrono_types::make_shared<ChLinkMateGeneric>();
    constr_bc->Initialize(node_down, node_crankB, false, node_crankB->Frame(), node_crankB->Frame());
    sys.Add(constr_bc);

    constr_bc->SetConstrainedCoords(true, true, true,    // x, y, z
                                    true, true, false);  // Rx, Ry, Rz

    // For example, attach small shape to show the constraint
    auto msphereconstr3 = chrono_types::make_shared<ChVisualShapeSphere>(0.01);
    constr_bc->AddVisualShape(msphereconstr3);

    //
    // Final touches..
    //

    // We do not want gravity effect on FEA elements in this demo
    my_mesh->SetAutomaticGravity(false);

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    // Visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChVisualShapeTriangleMesh
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colors as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a colored ChVisualShapeTriangleMesh).
    auto mvisualizebeamA = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizebeamA->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MX);
    mvisualizebeamA->SetColorscaleMinMax(-500, 500);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeamA);

    auto mvisualizebeamC = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizebeamC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizebeamC->SetSymbolsThickness(0.006);
    mvisualizebeamC->SetSymbolsScale(0.01);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeamC);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Beams and constraints");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector<>(0.0, 0.6, -1.0));
    vis->AttachSystem(&sys);

    // SIMULATION LOOP

    // Use a solver that can handle stiffnss matrices:
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);

    // Use the following for less numerical damping, 2nd order accuracy (but slower)
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper())) {
        // mystepper->SetVerbose(true);
        mystepper->SetStepControl(false);
    }

    // Output data
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    chrono::ChStreamOutAsciiFile file_out1(out_dir + "/buckling_mid.dat");

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();

        tools::drawGrid(vis.get(), 0.05, 0.05, 20, 20, ChCoordsys<>(VNULL, CH_C_PI_2, VECT_Z),
                        ChColor(0.4f, 0.4f, 0.4f), true);

        sys.DoStepDynamics(0.001);

        // Save output for the first 0.4 seconds
        if (sys.GetChTime() <= 0.4) {
            file_out1 << sys.GetChTime() << " " << node_mid->GetPos().z() << " " << node_mid->GetWvel_par().x() << "\n";
        }

        vis->EndScene();
    }

    return 0;
}
