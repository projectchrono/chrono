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

#include <cmath>

#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChMesh.h"

#include "FEAvisualization.h"

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fea;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;
    /*
    //// TEST
        ChVector3d mFi, mTi;
        ChVector3d mWvel = (1, 2, 3);
        ChVector3d mWacc =  (0.3, -0.2, 0.4);
        ChVector3d mXacc =  (0.5, 0.6, -0.9); // ok
        ChMatrixNM<double,6,6> mMi, mKi, mRi;
        ChMatrixNM<double,6,6> mKi_num, mRi_num;
        auto minertia1 = chrono_types::make_shared<ChInertiaCosseratAdvanced>(270.0, 0.1, 0.2, 5, 8, 0.5);
        minertia1->ComputeQuadraticTerms(mFi, mTi, mWvel);
        std::cout << "ComputeQuadraticTerms:" << std::endl;
        std::cout << " Fi_v= " << mFi;
        std::cout << " Ti_v = " << mTi;
        minertia1->ComputeInertialForce(mFi, mTi, mWvel, mWacc, mXacc);
        std::cout << "ComputeInertialForce:" << std::endl;
        std::cout << " Fi = " << mFi;
        std::cout << " Ti = " << mTi;
        minertia1->ComputeInertiaMatrix(mMi);
        std::cout << " Mi:\n"<< mMi << std::endl;
        minertia1->ComputeInertiaDampingMatrix(mRi,mWvel);
        std::cout << " Ri:\n"<< mRi << std::endl;
        minertia1->ComputeInertiaStiffnessMatrix(mKi,mWvel,mWacc,mXacc);
        std::cout << " Ki:\n"<< mKi << std::endl;
        minertia1->compute_Ri_Ki_by_num_diff = true;
        minertia1->ComputeInertiaDampingMatrix(mRi_num,mWvel);
        std::cout << " Ri_num:\n"<< mRi_num << std::endl;
        minertia1->ComputeInertiaStiffnessMatrix(mKi_num,mWvel,mWacc,mXacc);
        std::cout << " Ki_num:\n"<< mKi_num << std::endl;
     return 1;
     */
    // Create a Chrono physical system
    ChSystemSMC sys;

    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    double L = 1;
    double H = 0.25;
    double K = 0.05;
    ChVector3d vA(0, 0, 0);
    ChVector3d vC(L, 0, 0);
    ChVector3d vB(L, -H, 0);
    ChVector3d vG(L - K, -H, 0);
    ChVector3d vd(0, 0, 0.0001);

    // Create a truss:
    auto body_truss = chrono_types::make_shared<ChBody>();
    body_truss->SetFixed(true);

    sys.AddBody(body_truss);

    // Attach a 'box' shape asset for visualization.
    auto mboxtruss = chrono_types::make_shared<ChVisualShapeBox>(0.02, 0.2, 0.1);
    body_truss->AddVisualShape(mboxtruss, ChFrame<>(ChVector3d(-0.01, 0, 0)));

    // Create body for crank
    auto body_crank = chrono_types::make_shared<ChBody>();

    body_crank->SetPos((vB + vG) * 0.5);
    sys.AddBody(body_crank);

    // Attach a 'box' shape asset for visualization.
    auto mboxcrank = chrono_types::make_shared<ChVisualShapeBox>(K, 0.02, 0.02);
    body_crank->AddVisualShape(mboxcrank, ChFrame<>());

    // Create a motor between the truss and the crank:
    class ChFunctionMyFun : public ChFunction {
      public:
        virtual ChFunctionMyFun* Clone() const override { return new ChFunctionMyFun(); }

        virtual double GetVal(double x) const override {
            if (x > 0.4)
                return CH_PI;
            else
                return -CH_PI * (1.0 - std::cos(CH_PI * x / 0.4)) / 2.0;
        }
    };

    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->Initialize(body_truss, body_crank, ChFrame<>(vG));
    motor->SetAngleFunction(chrono_types::make_shared<ChFunctionMyFun>());
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
    melasticity->SetShearModulusFromPoisson(0.3);
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
    msection2->SetShearModulusFromPoisson(0.3);
    msection2->SetRayleighDamping(0.000);
    msection2->SetAsCircularSection(hbeam_d);

    ChBuilderBeamEuler builder;
    builder.BuildBeam(my_mesh,               // the mesh where to put the created nodes and elements
                      msection2,             // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                      3,                     // the number of ChElementBeamEuler to create
                      vC + vd,               // the 'A' point in space (beginning of beam)
                      vB + vd,               // the 'B' point in space (end of beam)
                      ChVector3d(1, 0, 0));  // the 'Y' up direction of the section for the beam
    auto node_top = std::shared_ptr<ChNodeFEAxyzrot>(builder.GetLastBeamNodes().front());
    auto node_down = std::shared_ptr<ChNodeFEAxyzrot>(builder.GetLastBeamNodes().back());

    // Create a constraint between the vertical and horizontal beams:
    auto constr_bb = chrono_types::make_shared<ChLinkMateSpherical>();
    constr_bb->Initialize(node_top, node_tip, false, node_top->Frame(), node_top->Frame());
    sys.Add(constr_bb);

    // For example, attach small shape to show the constraint
    auto msphereconstr2 = chrono_types::make_shared<ChVisualShapeSphere>(0.01);
    constr_bb->AddVisualShape(msphereconstr2);

    // Create a beam as a crank
    auto msection3 = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();

    double crankbeam_d = 0.048;
    msection3->SetDensity(2700);
    msection3->SetYoungModulus(73.0e9);
    msection3->SetShearModulusFromPoisson(0.3);
    msection3->SetRayleighDamping(0.000);
    msection3->SetAsCircularSection(crankbeam_d);

    builder.BuildBeam(my_mesh,               // the mesh where to put the created nodes and elements
                      msection3,             // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                      3,                     // the number of ChElementBeamEuler to create
                      vG + vd,               // the 'A' point in space (beginning of beam)
                      vB + vd,               // the 'B' point in space (end of beam)
                      ChVector3d(0, 1, 0));  // the 'Y' up direction of the section for the beam

    auto node_crankG = std::shared_ptr<ChNodeFEAxyzrot>(builder.GetLastBeamNodes().front());
    auto node_crankB = std::shared_ptr<ChNodeFEAxyzrot>(builder.GetLastBeamNodes().back());
    // Create a constraint between the crank beam and body crank:
    auto constr_cbd = chrono_types::make_shared<ChLinkMateFix>();
    constr_cbd->Initialize(node_crankG, body_crank, false, node_crankG->Frame(), node_crankG->Frame());
    sys.Add(constr_cbd);

    // Create a constraint between the vertical beam and the crank beam:
    auto constr_bc = chrono_types::make_shared<ChLinkMateRevolute>();
    constr_bc->Initialize(node_down, node_crankB, false, node_crankB->Frame(), node_crankB->Frame());
    sys.Add(constr_bc);

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
    auto mvisualizebeamA = chrono_types::make_shared<ChVisualShapeFEA>();
    mvisualizebeamA->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MX);
    mvisualizebeamA->SetColormapRange(-500, 500);
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

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Y, sys, "Beams and constraints",
                                         ChVector3d(0, 0.6, -1.0));

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
    const std::string out_dir = GetChronoOutputPath() + "BEAM_BUCKLING";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    std::ofstream file_out1(out_dir + "/buckling_mid.dat");

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();

        sys.DoStepDynamics(0.001);

        // Save output for the first 0.4 seconds
        if (sys.GetChTime() <= 0.4) {
            file_out1 << sys.GetChTime() << " " << node_mid->GetPos().z() << " " << node_mid->GetAngVelParent().x()
                      << std::endl;
        }

        vis->EndScene();
    }

    return 0;
}
