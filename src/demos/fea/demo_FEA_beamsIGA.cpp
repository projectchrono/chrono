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

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/fea/ChElementBeamIGA.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_thirdparty/filesystem/path.h"

#define USE_MKL

#ifdef USE_MKL
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

using namespace irr;

int ID_current_example = 1;

const std::string out_dir = GetChronoOutputPath() + "FEA_BEAMS_IGA";

//
// Example A: Low  level approach, creating single elements and nodes:
//

void MakeAndRunDemo0(ChIrrApp& myapp) {
    // Clear previous demo, if any:
    myapp.GetSystem()->Clear();
    myapp.GetSystem()->SetChTime(0);

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    // Remember to add it to the system.
    auto my_mesh = chrono_types::make_shared<ChMesh>();
    my_mesh->SetAutomaticGravity(false);
    myapp.GetSystem()->Add(my_mesh);

    // Create a section, i.e. thickness and material properties
    // for beams. This will be shared among some beams.
    // To simplify things, use ChBeamSectionCosseratEasyRectangular:

    double beam_wy = 0.012;
    double beam_wz = 0.025;

    auto msection =
        chrono_types::make_shared<ChBeamSectionCosseratEasyRectangular>(beam_wy,  // width of section in y direction
                                                                        beam_wz,  // width of section in z direction
                                                                        0.02e10,  // Young modulus
                                                                        0.02e10 * 0.3,  // shear modulus
                                                                        1000            // density
        );

    // Create an IGA beam using a low-level approach, i.e.
    // creating all elements and nodes one after the other:

    double beam_L = 0.1;

    auto hnode1 = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_L * 0, 0, 0)));
    auto hnode2 = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_L * 0.5, 0.00, 0)));
    auto hnode3 = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_L * 1.0, 0.00, 0)));
    auto hnode4 = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_L * 1.5, 0.00, 0)));
    auto hnode5 = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_L * 2.0, 0.00, 0)));

    my_mesh->AddNode(hnode1);
    my_mesh->AddNode(hnode2);
    my_mesh->AddNode(hnode3);
    my_mesh->AddNode(hnode4);
    my_mesh->AddNode(hnode5);

    // cubic spline with 2 spans, 5 control points and 9 knots= {0 0 0 0 1/2 1 1 1 1}

    auto belement1 = chrono_types::make_shared<ChElementBeamIGA>();

    belement1->SetNodesCubic(hnode1, hnode2, hnode3, hnode4, 0, 0, 0, 0, 1. / 2., 1, 1, 1);
    belement1->SetSection(msection);

    my_mesh->AddElement(belement1);

    auto belement2 = chrono_types::make_shared<ChElementBeamIGA>();

    belement2->SetNodesCubic(hnode2, hnode3, hnode4, hnode5, 0, 0, 0, 1. / 2., 1, 1, 1, 1);
    belement2->SetSection(msection);

    my_mesh->AddElement(belement2);

    // Attach a visualization of the FEM mesh.

    auto mvisualizebeamA = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizebeamA->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizebeamA);

    auto mvisualizebeamC = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizebeamC->SetSymbolsThickness(0.006);
    mvisualizebeamC->SetSymbolsScale(0.01);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizebeamC);

    // This is needed if you want to see things in Irrlicht 3D view.
    myapp.AssetBindAll();
    myapp.AssetUpdateAll();

    while (ID_current_example == 1 && myapp.GetDevice()->run()) {
        myapp.BeginScene();
        myapp.DrawAll();
        myapp.DoStep();
        myapp.EndScene();
    }
}

//
// Example B: Automatic creation of the nodes and knots
// using the ChBuilderBeamIGA tool for creating a straight
// rod automatically divided in Nel elements:
//

void MakeAndRunDemo1(ChIrrApp& myapp, int nsections = 32, int order = 2) {
    // Clear previous demo, if any:
    myapp.GetSystem()->Clear();
    myapp.GetSystem()->SetChTime(0);

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    // Remember to add it to the system.
    auto my_mesh = chrono_types::make_shared<ChMesh>();
    my_mesh->SetAutomaticGravity(false);
    myapp.GetSystem()->Add(my_mesh);

    double beam_L = 0.4;
    double beam_tip_load = -2;

    // Create a customized section, i.e. assemblying separate constitutive models for elasticity, damping etc.
    // This can be done by creating a ChBeamSectionCosserat section,
    // to whom we will attach sub-models for: inertia, elasticity, damping (optional), plasticity (optional)
    // By the way: a faster way to define a section for basic circular or rectangular beams is to
    // use the ChBeamSectionCosseratEasyCircular or ChBeamSectionCosseratEasyRectangular, see above.

    double beam_wy = 0.012;
    double beam_wz = 0.025;

    auto minertia = chrono_types::make_shared<ChInertiaCosseratSimple>();
    minertia->SetAsRectangularSection(beam_wy, beam_wz,
                                      1000);  // automatically sets A etc., from width, height, density

    auto melasticity = chrono_types::make_shared<ChElasticityCosseratSimple>();
    melasticity->SetYoungModulus(0.02e10);
    melasticity->SetGshearModulus(0.02e10 * 0.38);
    melasticity->SetAsRectangularSection(beam_wy, beam_wz);  // automatically sets A, Ixx, Iyy, Ksy, Ksz and J

    auto msection =
        chrono_types::make_shared<ChBeamSectionCosserat>(minertia,    // the constitutive model for inertia
                                                         melasticity  // the constitutive model for elasticity
        );
    msection->SetDrawThickness(beam_wy, beam_wz);

    // Use the ChBuilderBeamIGA tool for creating a straight rod
    // divided in Nel elements:

    ChBuilderBeamIGA builder;
    builder.BuildBeam(my_mesh,                   // the mesh to put the elements in
                      msection,                  // section of the beam
                      nsections,                 // number of sections (spans)
                      ChVector<>(0, 0, 0),       // start point
                      ChVector<>(beam_L, 0, 0),  // end point
                      VECT_Y,                    // suggested Y direction of section
                      order);                    // order (3 = cubic, etc)
    builder.GetLastBeamNodes().front()->SetFixed(true);
    builder.GetLastBeamNodes().back()->SetForce(ChVector<>(0, beam_tip_load, 0));
    // builder.GetLastBeamNodes().back()->SetTorque(ChVector<>(0,0, 1.2));

    // Attach a visualization of the FEM mesh.

    auto mvisualizebeamA = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizebeamA->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizebeamA);

    auto mvisualizebeamC = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizebeamC->SetSymbolsThickness(0.006);
    mvisualizebeamC->SetSymbolsScale(0.01);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizebeamC);

    // This is needed if you want to see things in Irrlicht 3D view.
    myapp.AssetBindAll();
    myapp.AssetUpdateAll();

    // Do a linear static analysis.
    myapp.GetSystem()->DoStaticLinear();

    // For comparison with analytical results.
    double poisson = melasticity->GetYoungModulus() / (2.0 * melasticity->GetGshearModulus()) - 1.0;
    double Ks_y = 10.0 * (1.0 + poisson) / (12.0 + 11.0 * poisson);
    double analytic_timoshenko_displ =
        (beam_tip_load * pow(beam_L, 3)) /
            (3 * melasticity->GetYoungModulus() * (1. / 12.) * beam_wz * pow(beam_wy, 3)) +
        (beam_tip_load * beam_L) /
            (Ks_y * melasticity->GetGshearModulus() * beam_wz * beam_wy);  // = (P*L^3)/(3*E*I) + (P*L)/(k*A*G)
    double numerical_displ =
        builder.GetLastBeamNodes().back()->GetPos().y() - builder.GetLastBeamNodes().back()->GetX0().GetPos().y();

    GetLog() << "\n LINEAR STATIC cantilever, order= " << order << "  nsections= " << nsections
             << "  rel.error=  " << fabs((numerical_displ - analytic_timoshenko_displ) / analytic_timoshenko_displ)
             << "\n";

    while (ID_current_example == 1 && myapp.GetDevice()->run()) {
        myapp.BeginScene();
        myapp.DrawAll();
        myapp.DoStep();
        myapp.EndScene();
    }
}

//
// Example C: Automatic creation of the nodes and knots using the
// ChBuilderBeamIGA tool for creating a generic curved rod that matches a Bspline.
// Also attach a rigid body to the end of the spline.
//

void MakeAndRunDemo2(ChIrrApp& myapp) {
    // Clear previous demo, if any:
    myapp.GetSystem()->Clear();
    myapp.GetSystem()->SetChTime(0);

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    // Remember to add it to the system.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    myapp.GetSystem()->Add(my_mesh);

    // Create a section, i.e. thickness and material properties
    // for beams. This will be shared among some beams.
    // To simplify things, use ChBeamSectionCosseratEasyRectangular:

    double beam_wy = 0.012;
    double beam_wz = 0.032;

    auto msection =
        chrono_types::make_shared<ChBeamSectionCosseratEasyRectangular>(beam_wy,  // width of section in y direction
                                                                        beam_wz,  // width of section in z direction
                                                                        0.02e10,  // Young modulus
                                                                        0.02e10 * 0.3,  // shear modulus
                                                                        1000            // density
        );

    // Note: we can add a ChBeamShape component to the section, and that can
    // provide the shape for the visualization of the beam. Note that the visualization
    // shape does not have to match to the physical properties (that in this example are those of a rectangular section)
    std::vector<std::vector<ChVector<> > > polyline_points = {
        {{0, 0.00, 0.02}, {0, 0.01, -0.02}, {0, 0.00, -0.025}, {0, -0.01, -0.025}},  // upper profile, with 4 xyz points
        {{0, -0.01, -0.025}, {0, -0.01, 0.00}, {0, 0.00, 0.02}}  // lower profile, with 3 xyz points. Note x=0, always.
    };
    auto msection_drawshape = chrono_types::make_shared<ChBeamSectionShapePolyline>(polyline_points);
    msection->SetDrawShape(msection_drawshape);

    // Use the ChBuilderBeamIGA tool for creating a curved rod
    // from a B-Spline

    ChBuilderBeamIGA builderR;

    std::vector<ChVector<> > my_points = {{0, 0, 0.2}, {0, 0, 0.3}, {0, -0.01, 0.4}, {0, -0.04, 0.5}, {0, -0.1, 0.6}};

    geometry::ChLineBspline my_spline(3,           // order (3 = cubic, etc)
                                      my_points);  // control points, will become the IGA nodes

    builderR.BuildBeam(my_mesh,    // the mesh to put the elements in
                       msection,   // section of the beam
                       my_spline,  // Bspline to match (also order will be matched)
                       VECT_Y);    // suggested Y direction of section

    builderR.GetLastBeamNodes().front()->SetFixed(true);

    auto mbodywing = chrono_types::make_shared<ChBodyEasyBox>(0.01, 0.2, 0.05, 2000);
    mbodywing->SetCoord(builderR.GetLastBeamNodes().back()->GetCoord());
    myapp.GetSystem()->Add(mbodywing);

    auto myjoint = chrono_types::make_shared<ChLinkMateFix>();
    myjoint->Initialize(builderR.GetLastBeamNodes().back(), mbodywing);
    myapp.GetSystem()->Add(myjoint);

    // Attach a visualization of the FEM mesh.

    auto mvisualizebeamA = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizebeamA->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizebeamA);

    auto mvisualizebeamC = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizebeamC->SetSymbolsThickness(0.006);
    mvisualizebeamC->SetSymbolsScale(0.01);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizebeamC);

    // This is needed if you want to see things in Irrlicht 3D view.
    myapp.AssetBindAll();
    myapp.AssetUpdateAll();

    while (ID_current_example == 2 && myapp.GetDevice()->run()) {
        myapp.BeginScene();
        myapp.DrawAll();
        myapp.DoStep();
        myapp.EndScene();
    }
}

//
// Example D:
// Plasticity in IGA beams.
//

void MakeAndRunDemo3(ChIrrApp& myapp) {
    // Clear previous demo, if any:
    myapp.GetSystem()->Clear();
    myapp.GetSystem()->SetChTime(0);

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    // Remember to add it to the system.
    auto my_mesh = chrono_types::make_shared<ChMesh>();
    my_mesh->SetAutomaticGravity(false);
    myapp.GetSystem()->Add(my_mesh);

    // Create a section, i.e. thickness and material properties
    // for beams. This will be shared among some beams.
    // Note that we will define some basic plasticity. One can
    // set hardening curves, both isotropic hardening and/or kinematic hardening.

    double beam_wy = 0.012;
    double beam_wz = 0.025;

    auto minertia = chrono_types::make_shared<ChInertiaCosseratSimple>();
    minertia->SetAsRectangularSection(beam_wy, beam_wz,
                                      1000);  // automatically sets A etc., from width, height, density

    auto melasticity = chrono_types::make_shared<ChElasticityCosseratSimple>();
    melasticity->SetYoungModulus(0.02e10);
    melasticity->SetGshearModulus(0.02e10 * 0.3);
    melasticity->SetAsRectangularSection(beam_wy, beam_wz);  // automatically sets A, Ixx, Iyy, Ksy, Ksz and J

    auto mplasticity = chrono_types::make_shared<ChPlasticityCosseratLumped>();
    // The isotropic hardening curve. The value at zero absyssa is the initial yeld.
    mplasticity->n_yeld_x = chrono_types::make_shared<ChFunction_Const>(3000);
    // mplasticity->n_yeld_x = chrono_types::make_shared<ChFunction_Ramp>(3000, 1e3);
    // The optional kinematic hardening curve:
    mplasticity->n_beta_x = chrono_types::make_shared<ChFunction_Ramp>(0, 1e3);

    // for bending (on y and z): some kinematic hardening
    mplasticity->n_yeld_My = chrono_types::make_shared<ChFunction_Const>(0.3);
    mplasticity->n_beta_My = chrono_types::make_shared<ChFunction_Ramp>(0, 0.001e2);
    mplasticity->n_yeld_Mz = chrono_types::make_shared<ChFunction_Const>(0.3);
    mplasticity->n_beta_Mz = chrono_types::make_shared<ChFunction_Ramp>(0, 0.001e2);

    auto msection = chrono_types::make_shared<ChBeamSectionCosserat>(minertia, melasticity, mplasticity);

    msection->SetDrawThickness(beam_wy, beam_wz);

    ChBuilderBeamIGA builder;
    builder.BuildBeam(my_mesh,                  // the mesh to put the elements in
                      msection,                 // section of the beam
                      5,                        // number of sections (spans)
                      ChVector<>(0, 0, 0),      // start point
                      ChVector<>(0.4, 0.0, 0),  // end point
                      VECT_Y,                   // suggested Y direction of section
                      2);                       // order (3 = cubic, etc)
    builder.GetLastBeamNodes().front()->SetFixed(true);

    // Now create a linear motor that push-pulls the end of the beam
    // up to repeated plasticization.
    auto truss = chrono_types::make_shared<ChBody>();
    myapp.GetSystem()->Add(truss);
    truss->SetBodyFixed(true);

    auto motor = chrono_types::make_shared<ChLinkMotorLinearPosition>();
    myapp.GetSystem()->Add(motor);
    motor->Initialize(
        builder.GetLastBeamNodes().back(), truss,
        ChFrame<>(builder.GetLastBeamNodes().back()->GetPos(), chrono::Q_from_AngAxis(0 * CH_C_PI_2, VECT_Z)));
    motor->SetGuideConstraint(ChLinkMotorLinear::GuideConstraint::SPHERICAL);
    auto rampup = chrono_types::make_shared<ChFunction_Ramp>(0, 0.1);
    auto rampdo = chrono_types::make_shared<ChFunction_Ramp>(0, -0.1);
    auto motfun = chrono_types::make_shared<ChFunction_Sequence>();
    motfun->InsertFunct(rampdo, 1, 0, true);
    motfun->InsertFunct(rampup, 1, 0, true);
    auto motrepeat = chrono_types::make_shared<ChFunction_Repeat>();
    motrepeat->Set_fa(motfun);
    motrepeat->Set_window_length(2);
    auto motfuntot = chrono_types::make_shared<ChFunction_Sequence>();
    motfuntot->InsertFunct(rampup, 0.5, 0, true);
    motfuntot->InsertFunct(motrepeat, 10, 0, true);
    motor->SetMotionFunction(motfuntot);

    // Attach a visualization of the FEM mesh.

    auto mvisualizebeamA = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizebeamA->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizebeamA);

    auto mvisualizebeamC = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizebeamC->SetSymbolsThickness(0.006);
    mvisualizebeamC->SetSymbolsScale(0.01);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizebeamC);

    // This is needed if you want to see things in Irrlicht 3D view.
    myapp.AssetBindAll();
    myapp.AssetUpdateAll();

    std::string filename = out_dir + "/plasticity.dat";
    ChStreamOutAsciiFile my_plasticfile(filename.c_str());

    while (ID_current_example == 3 && myapp.GetDevice()->run()) {
        myapp.BeginScene();
        myapp.DrawAll();
        myapp.DoStep();

        // Save to file: plastic flow of the 1st element, and other data
        ChMatrixDynamic<> mK(builder.GetLastBeamElements()[0]->GetNdofs(),
                             builder.GetLastBeamElements()[0]->GetNdofs());
        builder.GetLastBeamElements()[0]->ComputeKRMmatricesGlobal(mK, 1, 0, 0);
        auto plasticdat = builder.GetLastBeamElements()[0]->GetPlasticData()[0].get();
        auto plasticdata = dynamic_cast<ChInternalDataLumpedCosserat*>(plasticdat);

        my_plasticfile << myapp.GetSystem()->GetChTime() << " " << builder.GetLastBeamElements()[0]->GetStrainE()[0].x()
                       << " " << builder.GetLastBeamElements()[0]->GetStressN()[0].x() << " "
                       << plasticdata->p_strain_acc << " " << plasticdata->p_strain_e.x() << " " << mK(0, 0) << " "
                       << motor->GetMotorForce() << " " << motor->GetMotorPos() << "\n";
        /*
        my_plasticfile << myapp.GetSystem()->GetChTime() << " "
            << builder.GetLastBeamElements()[0]->GetStrainK()[0].z() << " "
            << builder.GetLastBeamElements()[0]->GetStressM()[0].z() << " "
            << plasticdata->p_strain_acc << " "
            << plasticdata->p_strain_k.z() << " "
            << mK(5, 5) << " "
            << motor->GetMotorForce() << " "
            << motor->GetMotorPos() << "\n";
            */
        myapp.EndScene();
    }
}

//
// Example E: Jeffcott rotor.
// Create IGA beams, connect an offset flywheel, and make it rotate
// with a motor, up to resonance.
//

void MakeAndRunDemo4(ChIrrApp& myapp) {
    // Clear previous demo, if any:
    myapp.GetSystem()->Clear();
    myapp.GetSystem()->SetChTime(0);

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    // Remember to add it to the system.
    auto my_mesh = chrono_types::make_shared<ChMesh>();
    myapp.GetSystem()->Add(my_mesh);

    my_mesh->SetAutomaticGravity(
        true, 2);  // for max precision in gravity of FE, at least 2 integration points per element when using cubic IGA
    myapp.GetSystem()->Set_G_acc(ChVector<>(0, -9.81, 0));

    double beam_L = 6;
    double beam_ro = 0.050;
    double beam_ri = 0.045;

    // Create a section, i.e. thickness and material properties
    // for beams. This will be shared among some beams.

    auto minertia = chrono_types::make_shared<ChInertiaCosseratSimple>();
    minertia->SetDensity(7800);
    minertia->SetArea(CH_C_PI * (pow(beam_ro, 2) - pow(beam_ri, 2)));
    minertia->SetIyy((CH_C_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)));
    minertia->SetIzz((CH_C_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)));

    auto melasticity = chrono_types::make_shared<ChElasticityCosseratSimple>();
    melasticity->SetYoungModulus(210e9);
    melasticity->SetGwithPoissonRatio(0.3);
    melasticity->SetArea(CH_C_PI * (pow(beam_ro, 2) - pow(beam_ri, 2)));
    melasticity->SetIyy((CH_C_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)));
    melasticity->SetIzz((CH_C_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)));
    melasticity->SetJ((CH_C_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)));
    // set the Timoshenko shear factors, if needed: melasticity->SetKsy(..) melasticity->SetKsy(..)

    auto msection = chrono_types::make_shared<ChBeamSectionCosserat>(minertia, melasticity);

    msection->SetCircular(true);
    msection->SetDrawCircularRadius(beam_ro);

    // Use the ChBuilderBeamIGA tool for creating a straight rod
    // divided in Nel elements:

    ChBuilderBeamIGA builder;
    builder.BuildBeam(my_mesh,                   // the mesh to put the elements in
                      msection,                  // section of the beam
                      20,                        // number of sections (spans)
                      ChVector<>(0, 0, 0),       // start point
                      ChVector<>(beam_L, 0, 0),  // end point
                      VECT_Y,                    // suggested Y direction of section
                      1);                        // order (3 = cubic, etc)

    auto node_mid = builder.GetLastBeamNodes()[(int)floor(builder.GetLastBeamNodes().size() / 2.0)];

    // Create the flywheel and attach it to the center of the beam

    auto mbodyflywheel = chrono_types::make_shared<ChBodyEasyCylinder>(0.24, 0.05, 7800);  // R, h, density
    mbodyflywheel->SetCoord(ChCoordsys<>(
        node_mid->GetPos() + ChVector<>(0, 0.05, 0),  // flywheel initial center (plus Y offset)
        Q_from_AngAxis(CH_C_PI_2, VECT_Z))  // flywheel initial alignment (rotate 90° so cylinder axis is on X)
    );
    myapp.GetSystem()->Add(mbodyflywheel);

    auto myjoint = chrono_types::make_shared<ChLinkMateFix>();
    myjoint->Initialize(node_mid, mbodyflywheel);
    myapp.GetSystem()->Add(myjoint);

    // Create the truss
    auto truss = chrono_types::make_shared<ChBody>();
    truss->SetBodyFixed(true);
    myapp.GetSystem()->Add(truss);

    // Create the end bearing
    auto bearing = chrono_types::make_shared<ChLinkMateGeneric>(false, true, true, false, true, true);
    bearing->Initialize(builder.GetLastBeamNodes().back(), truss,
                        ChFrame<>(builder.GetLastBeamNodes().back()->GetPos()));
    myapp.GetSystem()->Add(bearing);

    // Create the motor that rotates the beam
    auto rotmotor1 = chrono_types::make_shared<ChLinkMotorRotationSpeed>();

    // Connect the rotor and the stator and add the motor to the system:
    rotmotor1->Initialize(builder.GetLastBeamNodes().front(),  // body A (slave)
                          truss,                               // body B (master)
                          ChFrame<>(builder.GetLastBeamNodes().front()->GetPos(),
                                    Q_from_AngAxis(CH_C_PI_2, VECT_Y))  // motor frame, in abs. coords
    );
    myapp.GetSystem()->Add(rotmotor1);

    // use a custom function for setting the speed of the motor
    class ChFunction_myf : public ChFunction {
      public:
        virtual ChFunction_myf* Clone() const override { return new ChFunction_myf(); }

        virtual double Get_y(double x) const override {
            double A1 = 0.8;
            double A2 = 1.2;
            double T1 = 0.5;
            double T2 = 1.0;
            double T3 = 1.25;
            double w = 60;
            if (x < T1)
                return A1 * w * (1. - cos(CH_C_PI * x / T1)) / 2.0;
            else if (x > T1 && x <= T2)
                return A1 * w;
            else if (x > T2 && x <= T3)
                return A1 * w + (A2 - A1) * w * (1.0 - cos(CH_C_PI * (x - T2) / (T3 - T2))) / 2.0;
            else  // if (x > T3)
                return A2 * w;
        }
    };

    auto f_ramp = chrono_types::make_shared<ChFunction_myf>();
    rotmotor1->SetMotorFunction(f_ramp);

    // Attach a visualization of the FEM mesh.

    auto mvisualizebeamA = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizebeamA->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizebeamA);

    auto mvisualizebeamC = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizebeamC->SetSymbolsThickness(0.006);
    mvisualizebeamC->SetSymbolsScale(0.01);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizebeamC);

    // This is needed if you want to see things in Irrlicht 3D view.
    myapp.AssetBindAll();
    myapp.AssetUpdateAll();

    std::string filename = out_dir + "/rotor_displ.dat";
    chrono::ChStreamOutAsciiFile file_out1(filename.c_str());

    // Set to a more precise HHT timestepper if needed
    // myapp.GetSystem()->SetTimestepperType(ChTimestepper::Type::HHT);
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(myapp.GetSystem()->GetTimestepper())) {
        mystepper->SetStepControl(false);
        mystepper->SetModifiedNewton(false);
    }

    myapp.SetTimestep(0.002);
    myapp.GetSystem()->DoStaticLinear();

    while (ID_current_example == 4 && myapp.GetDevice()->run()) {
        myapp.BeginScene();
        myapp.DrawAll();
        myapp.DoStep();
        file_out1 << myapp.GetSystem()->GetChTime() << " " << node_mid->GetPos().y() << " " << node_mid->GetPos().z()
                  << "\n";
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
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Initialize output
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Create a Chrono::Engine physical system
    ChSystemSMC my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"IGA beams DEMO (SPACE for dynamics, F10 / F11 statics)",
                         core::dimension2d<u32>(800, 600));

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights(irr::core::vector3df(30.f, 100.f, 30.f), irr::core::vector3df(30.f, 80.f, -30.f), 180,
                                 190, irr::video::SColorf(0.5f, 0.5f, 0.5f, 1.0f),
                                 irr::video::SColorf(0.2f, 0.3f, 0.4f, 1.0f));
    application.AddTypicalCamera(core::vector3df(-0.1f, 0.2f, -0.2f));

    // This is for GUI tweaking of system parameters..
    MyEventReceiver receiver(&application);
    // note how to add a custom event receiver to the default interface:
    application.SetUserEventReceiver(&receiver);

    // Some help on the screen
    application.GetIGUIEnvironment()->addStaticText(
        L" Press 1: static analysis \n Press 2: curved beam connected to body \n Press 3: plasticity \n Press 4: "
        L"Jeffcott rotor",
        irr::core::rect<irr::s32>(10, 80, 250, 150), false, true, 0);

    // Solver default settings for all the sub demos:
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    my_system.SetSolver(solver);
    solver->SetMaxIterations(500);
    solver->SetTolerance(1e-15);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);  // IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
    solver->SetVerbose(false);

    my_system.SetSolverForceTolerance(1e-14);

#ifdef USE_MKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    my_system.SetSolver(mkl_solver);
#endif

    application.SetTimestep(0.01);

    // Run the sub-demos:

    while (true) {
        switch (ID_current_example) {
            case 1:
                /*
                MakeAndRunDemo1(application, 4,1);
                MakeAndRunDemo1(application, 8,1);
                MakeAndRunDemo1(application,16,1);
                MakeAndRunDemo1(application,32,1);
                MakeAndRunDemo1(application,64,1);
                MakeAndRunDemo1(application,512, 1);
                MakeAndRunDemo1(application, 4, 2);
                MakeAndRunDemo1(application, 8, 2);
                MakeAndRunDemo1(application, 16, 2);
                MakeAndRunDemo1(application, 32, 2);
                MakeAndRunDemo1(application, 64, 2);
                MakeAndRunDemo1(application, 512, 2);
                MakeAndRunDemo1(application, 4, 3);
                MakeAndRunDemo1(application, 8, 3);
                MakeAndRunDemo1(application, 16, 3);
                MakeAndRunDemo1(application, 32, 3);
                MakeAndRunDemo1(application, 64, 3);
                MakeAndRunDemo1(application, 512, 3);
                MakeAndRunDemo1(application, 4, 4);
                MakeAndRunDemo1(application, 8, 4);
                MakeAndRunDemo1(application, 16, 4);
                MakeAndRunDemo1(application, 32, 4);
                MakeAndRunDemo1(application, 64, 4);
                MakeAndRunDemo1(application, 512, 4);
                MakeAndRunDemo1(application, 4, 5);
                MakeAndRunDemo1(application, 8, 5);
                MakeAndRunDemo1(application, 16, 5);
                MakeAndRunDemo1(application, 32, 5);
                MakeAndRunDemo1(application, 64, 5);
                MakeAndRunDemo1(application, 512, 5);
                system("pause");
                */
                MakeAndRunDemo1(application, 64, 3);
                break;
            case 2:
                MakeAndRunDemo2(application);
                break;
            case 3:
                MakeAndRunDemo3(application);
                break;
            case 4:
                MakeAndRunDemo4(application);
                break;
            default:
                break;
        }
        if (!application.GetDevice()->run())
            break;
    }

    return 0;
}
