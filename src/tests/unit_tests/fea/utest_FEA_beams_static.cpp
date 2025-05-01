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
// Authors: Simone Benatti, Radu Serban, Alessandro Tasora
// =============================================================================
//
// Test for static analysis of FEA 3D beams: IGA and ANCF
//
// =============================================================================

#include <chrono>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

using namespace chrono;
using namespace chrono::fea;

const int num_tests = 5;
const double beam_tip_init_load = -2.0f;  // base tip load (load increased for each test)
const double threshold = 0.05;            // max 5% error allowed

const double beamL = 0.4;
const double density = 1000.0;  // Beam material density
const double E_mod = 20.02e10;  // Beam modulus of elasticity
const double nu_rat = 0.38;     // Beam material Poisson ratio
const double beam_wy = 0.012;
const double beam_wz = 0.025;
const double k1 = 10 * (1 + nu_rat) / (12 + 11 * nu_rat);  // Timoshenko coefficient
const double k2 = k1;                                      // Timoshenko coefficient

bool use_MKL = false;

double AnalyticalSol(double tip_load) {
    double G_mod = E_mod * nu_rat;
    double poisson = E_mod / (2.0 * G_mod) - 1.0;
    double Ks_y = 10.0 * (1.0 + poisson) / (12.0 + 11.0 * poisson);

    // (P*L^3)/(3*E*I) + (P*L)/(k*A*G)
    double analytical_timoshenko_displ =
        (tip_load * std::pow(beamL, 3)) / (3 * E_mod * (1. / 12.) * beam_wz * std::pow(beam_wy, 3)) +
        (tip_load * beamL) / (Ks_y * G_mod * beam_wz * beam_wy);

    return analytical_timoshenko_displ;
}

double ANCF3243_test(ChSystem& sys, double tip_load, int nelements) {
    // Clear previous demo, if any:
    sys.Clear();
    sys.SetChTime(0);

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    // Remember to add it to the system.
    auto mesh = chrono_types::make_shared<ChMesh>();
    mesh->SetAutomaticGravity(false);
    sys.Add(mesh);

    auto material = chrono_types::make_shared<ChMaterialBeamANCF>(density, E_mod, nu_rat, E_mod * nu_rat, k1, k2);

    ChBuilderBeamANCF_3243 builder;
    builder.BuildBeam(mesh, material, nelements, ChVector3d(0, 0, 0), ChVector3d(beamL, 0, 0), beam_wy, beam_wz, VECT_X,
                      VECT_Y, VECT_Z);
    builder.GetLastBeamNodes().front()->SetFixed(true);
    builder.GetLastBeamNodes().back()->SetForce(ChVector3d(0, tip_load, 0));

    double y_init = builder.GetLastBeamNodes().back()->GetPos().y();

    // Do a linear static analysis.
    sys.DoStaticLinear();

    double numerical_displ = builder.GetLastBeamNodes().back()->GetPos().y() - y_init;

    return numerical_displ;
}

double ANCF3333_test(ChSystem& sys, double tip_load, int nelements) {
    // Clear previous demo, if any:
    sys.Clear();
    sys.SetChTime(0);

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    // Remember to add it to the system.
    auto mesh = chrono_types::make_shared<ChMesh>();
    mesh->SetAutomaticGravity(false);
    sys.Add(mesh);

    auto material = chrono_types::make_shared<ChMaterialBeamANCF>(density, E_mod, nu_rat, E_mod * nu_rat, k1, k2);

    ChBuilderBeamANCF_3333 builder;
    builder.BuildBeam(mesh, material, nelements, ChVector3d(0, 0, 0), ChVector3d(beamL, 0, 0), beam_wy, beam_wz, VECT_Y,
                      VECT_Z);
    builder.GetLastBeamNodes().front()->SetFixed(true);
    builder.GetLastBeamNodes().back()->SetForce(ChVector3d(0, tip_load, 0));

    double y_init = builder.GetLastBeamNodes().back()->GetPos().y();

    // Do a linear static analysis.
    sys.DoStaticLinear();

    double numerical_displ = builder.GetLastBeamNodes().back()->GetPos().y() - y_init;

    return numerical_displ;
}

double IGA_test(ChSystem& sys, double tip_load, int nsections, int order) {
    // Clear previous demo, if any:
    sys.Clear();
    sys.SetChTime(0);

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    // Remember to add it to the system.
    auto mesh = chrono_types::make_shared<ChMesh>();
    mesh->SetAutomaticGravity(false);
    sys.Add(mesh);

    auto section =
        chrono_types::make_shared<ChBeamSectionCosseratEasyRectangular>(beam_wy,  // width of section in y direction
                                                                        beam_wz,  // width of section in z direction
                                                                        E_mod,    // Young modulus
                                                                        E_mod * nu_rat,  // shear modulus
                                                                        density          // density
        );

    // Use the ChBuilderBeamIGA tool for creating a straight rod divided in Nel elements
    ChBuilderBeamIGA builder;
    builder.BuildBeam(mesh,                     // the mesh to put the elements in
                      section,                  // section of the beam
                      nsections,                // number of sections (spans)
                      ChVector3d(0, 0, 0),      // start point
                      ChVector3d(beamL, 0, 0),  // end point
                      VECT_Y,                   // suggested Y direction of section
                      order);                   // order (3 = cubic, etc)
    builder.GetLastBeamNodes().front()->SetFixed(true);
    builder.GetLastBeamNodes().back()->SetForce(ChVector3d(0, tip_load, 0));

    double y_init = builder.GetLastBeamNodes().back()->GetX0().GetPos().y();

    // Do a linear static analysis.
    sys.DoStaticLinear();

    double numerical_displ = builder.GetLastBeamNodes().back()->GetPos().y() - y_init;

    /*
    ChVector3d mforce, mtorque;
    builder.GetLastBeamElements().front()->EvaluateSectionForceTorque(0, mforce, mtorque);
    std::cout << "        IGA sect torque = " << mtorque << std::endl;
    */

    return numerical_displ;
}

double EULER_test(ChSystem& sys, double tip_load, int nelements) {
    // Clear previous demo, if any:
    sys.Clear();
    sys.SetChTime(0);

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    // Remember to add it to the system.
    auto mesh = chrono_types::make_shared<ChMesh>();
    mesh->SetAutomaticGravity(false);
    sys.Add(mesh);

    auto material =
        chrono_types::make_shared<ChBeamSectionEulerEasyRectangular>(beam_wy,         // width of section in y direction
                                                                     beam_wz,         // width of section in z direction
                                                                     E_mod,           // Young modulus
                                                                     E_mod * nu_rat,  // Shear modulus
                                                                     density          // density
        );

    ChBuilderBeamEuler builder;
    builder.BuildBeam(mesh, material, nelements, ChVector3d(0, 0, 0), ChVector3d(beamL, 0, 0), VECT_Y);
    builder.GetLastBeamNodes().front()->SetFixed(true);
    builder.GetLastBeamNodes().back()->SetForce(ChVector3d(0, tip_load, 0));

    double y_init = builder.GetLastBeamNodes().back()->GetPos().y();

    // Do a linear static analysis.
    sys.DoStaticLinear();

    double numerical_displ = builder.GetLastBeamNodes().back()->GetPos().y() - y_init;

    /*
    ChVector3d mforce, mtorque;
    builder.GetLastBeamElements().front()->EvaluateSectionForceTorque(0.9999, mforce, mtorque);
    std::cout << "        EULER sect force  = " << mforce  << std::endl;
    std::cout << "        EULER sect torque = " << mtorque << std::endl;
    */

    return numerical_displ;
}

// Same EULER beam, but with offsets in the section, i.e with center of shear and elastic centers being offset.
// Applying a precise torque to the end tip should "cancel" the effect of those offsets and still getting
// the cantilever with same deflection on Y (if the code for section transformations were wrong, this would not
// succeeed)
double EULER_test_offset(ChSystem& sys, double tip_load_y, int nelements) {
    // Clear previous demo, if any:
    sys.Clear();
    sys.SetChTime(0);

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    // Remember to add it to the system.
    auto mesh = chrono_types::make_shared<ChMesh>();
    mesh->SetAutomaticGravity(false);
    sys.Add(mesh);

    double Cy = 0.15;
    double Cz = 0.25;
    double Sy = 0.10;
    double Sz = 0.20;
    double tip_axial = 2000;  // add also axial tension on X: just to test if this is not canceled in Y deflection
    double tip_load_z = 4;    // add also lateral bending on Z: just to test if this is not canceled in Y deflection

    auto material = chrono_types::make_shared<ChBeamSectionEulerAdvancedGeneric>(
        E_mod * beam_wy * beam_wz,  // axial rigidity
        E_mod * nu_rat *
            ((1. / 12) * beam_wz * std::pow(beam_wy, 3) +
             (1. / 12) * beam_wz * std::pow(beam_wy, 3)),    // torsion rigidity
        E_mod * (1. / 12) * beam_wy * std::pow(beam_wz, 3),  // bending regidity about yy
        E_mod * (1. / 12) * beam_wz * std::pow(beam_wy, 3),  // bending regidity about zz
        0,                                                   // section rotation about elastic center [rad]
        Cy,                                                  // elastic center y displacement respect to centerline
        Cz,                                                  // elastic center z displacement respect to centerline
        Sy,                                                  // shear center y displacement respect to centerline
        Sz,                                                  // shear center z displacement respect to centerline
        density * beam_wy * beam_wz,                         // mass per unit length
        density * ((1. / 12) * beam_wz * std::pow(beam_wy, 3) +
                   (1. / 12) * beam_wz * std::pow(beam_wy, 3))  // polar inertia Jxx per unit lenght
    );

    ChBuilderBeamEuler builder;
    builder.BuildBeam(mesh, material, nelements, ChVector3d(0, 0, 0), ChVector3d(beamL, 0, 0), VECT_Y);
    builder.GetLastBeamNodes().front()->SetFixed(true);
    builder.GetLastBeamNodes().back()->SetForce(ChVector3d(tip_axial, tip_load_y, tip_load_z));
    builder.GetLastBeamNodes().back()->SetTorque(ChVector3d(
        -tip_load_y * Sz + tip_load_z * Sy, tip_axial * Cz,
        -tip_axial * Cy));  // to cancel the effect of offsets - now should deflect on Y as normal cantilever.

    double y_init = builder.GetLastBeamNodes().back()->GetPos().y();

    // Do a linear static analysis.
    sys.DoStaticLinear();

    double numerical_displ = builder.GetLastBeamNodes().back()->GetPos().y() - y_init;

    /*
    ChVector3d mforce, mtorque;
    builder.GetLastBeamElements().front()->EvaluateSectionForceTorque(0, mforce, mtorque);
    std::cout << "        EULER off sect force  = " << mforce  << std::endl;
    std::cout << "        EULER off sect torque = " << mtorque << std::endl;
    std::cout << "           tip load = " << tip_load_y << std::endl;
    */

    return numerical_displ;
}

// Same IGA beam, but with offsets in the section, i.e with center of shear and elastic centers being offset.
// Applying a precise torque to the end tip should "cancel" the effect of those offsets and still getting
// the cantilever with same deflection on Y (if the code for section transformations were wrong, this would not
// succeeed)
double IGA_test_offset(ChSystem& sys, double tip_load_y, int nsections, int order) {
    // Clear previous demo, if any:
    sys.Clear();
    sys.SetChTime(0);

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    // Remember to add it to the system.
    auto mesh = chrono_types::make_shared<ChMesh>();
    mesh->SetAutomaticGravity(false);
    sys.Add(mesh);

    double Cy = 0.15;
    double Cz = 0.25;
    double Sy = 0.1;
    double Sz = 0.2;
    double tip_axial = 2000;  // add also axial tension on X: just to test if this is not canceled in Y deflection
    double tip_load_z = 4;    // add also lateral bending on Z: just to test if this is not canceled in Y deflection

    auto section_inertia = chrono_types::make_shared<ChInertiaCosseratSimple>(
        density, beam_wy * beam_wz, density * (1. / 12) * beam_wz * std::pow(beam_wy, 3),
        density * (1. / 12) * beam_wz * std::pow(beam_wy, 3));  // not important

    auto section_elasticity = chrono_types::make_shared<ChElasticityCosseratAdvanced>(
        (1. / 12) * beam_wy * std::pow(beam_wz, 3),  // Iyy
        (1. / 12) * beam_wz * std::pow(beam_wy, 3),  // Izz
        ((1. / 12) * beam_wz * std::pow(beam_wy, 3) +
         (1. / 12) * beam_wz * std::pow(beam_wy, 3)),  // torsion constant, approx.
        E_mod * nu_rat,                                // G shear modulus
        E_mod,                                         // E young modulus
        beam_wy * beam_wz,                             // A area
        0.8,                                           // Timoshenko shear coefficient Ks for y shear
        0.8,                                           // Timoshenko shear coefficient Ks for z shear
        0,                                             // section rotation for which Iyy Izz are computed
        Cy,                                            // Cy offset of elastic center about which Iyy Izz are computed
        Cz,                                            // Cz offset of elastic center about which Iyy Izz are computed
        0,                                             // section rotation for which Ks_y Ks_z are computed
        Sy,                                            // Sy offset of shear center
        Sz                                             // Sz offset of shear center
    );

    auto section = chrono_types::make_shared<ChBeamSectionCosserat>(section_inertia, section_elasticity);

    // Use the ChBuilderBeamIGA tool for creating a straight rod divided in Nel elements
    ChBuilderBeamIGA builder;
    builder.BuildBeam(mesh,                     // the mesh to put the elements in
                      section,                  // section of the beam
                      nsections,                // number of sections (spans)
                      ChVector3d(0, 0, 0),      // start point
                      ChVector3d(beamL, 0, 0),  // end point
                      VECT_Y,                   // suggested Y direction of section
                      order);                   // order (3 = cubic, etc)
    builder.GetLastBeamNodes().front()->SetFixed(true);
    builder.GetLastBeamNodes().back()->SetForce(ChVector3d(tip_axial, tip_load_y, tip_load_z));
    builder.GetLastBeamNodes().back()->SetTorque(ChVector3d(
        -tip_load_y * Sz + tip_load_z * Sy, tip_axial * Cz,
        -tip_axial * Cy));  // to cancel the effect of offsets - now should deflect on Y as normal cantilever.

    double y_init = builder.GetLastBeamNodes().back()->GetPos().y();

    // Do a linear static analysis.
    sys.DoStaticLinear();

    double numerical_displ = builder.GetLastBeamNodes().back()->GetPos().y() - y_init;

    /*
    ChVector3d mforce, mtorque;
    builder.GetLastBeamElements().front()->EvaluateSectionForceTorque(0, mforce, mtorque);
    std::cout << "        IGA sect torque = " << mtorque << std::endl;
    */

    return numerical_displ;
}

// Same IGA beam as in IGA_test_offset, but this time using axial/bending/shear/torsion
// rigidities instead of E,G, A, Iyy, Izz, J
double IGA_test_offset_rigidity(ChSystem& sys, double tip_load_y, int nsections, int order) {
    // Clear previous demo, if any:
    sys.Clear();
    sys.SetChTime(0);

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    // Remember to add it to the system.
    auto mesh = chrono_types::make_shared<ChMesh>();
    mesh->SetAutomaticGravity(false);
    sys.Add(mesh);

    double Cy = 0.15;
    double Cz = 0.25;
    double Sy = 0.1;
    double Sz = 0.2;
    double tip_axial = 2000;  // add also axial tension on X: just to test if this is not canceled in Y deflection
    double tip_load_z = 4;    // add also lateral bending on Z: just to test if this is not canceled in Y deflection

    auto section_inertia = chrono_types::make_shared<ChInertiaCosseratSimple>(
        density, beam_wy * beam_wz, density * (1. / 12) * beam_wz * std::pow(beam_wy, 3),
        density * (1. / 12) * beam_wz * std::pow(beam_wy, 3));  // not important

    auto section_elasticity = chrono_types::make_shared<ChElasticityCosseratAdvancedGeneric>(
        E_mod * beam_wy * beam_wz,  // Axial rigidity
        E_mod * nu_rat *
            ((1. / 12) * beam_wz * std::pow(beam_wy, 3) +
             (1. / 12) * beam_wz * std::pow(beam_wy, 3)),    // torsion rigidity, approx
        E_mod * (1. / 12) * beam_wy * std::pow(beam_wz, 3),  // bending rigidity Byy
        E_mod * (1. / 12) * beam_wz * std::pow(beam_wy, 3),  // bending rigidity Bzz
        E_mod * nu_rat * 0.8 * beam_wy * beam_wz,            // shear rigidity Hyy
        E_mod * nu_rat * 0.8 * beam_wy * beam_wz,            // shear rigidity Hzz
        0,                                                   ///< section rotation for which Iyy Izz are computed
        Cy,  ///< Cy offset of elastic center about which Iyy Izz are computed
        Cz,  ///< Cz offset of elastic center about which Iyy Izz are computed
        0,   ///< section rotation for which Ks_y Ks_z are computed
        Sy,  ///< Sy offset of shear center
        Sz   ///< Sz offset of shear center
    );

    auto section = chrono_types::make_shared<ChBeamSectionCosserat>(section_inertia, section_elasticity);

    // Use the ChBuilderBeamIGA tool for creating a straight rod divided in Nel elements
    ChBuilderBeamIGA builder;
    builder.BuildBeam(mesh,                     // the mesh to put the elements in
                      section,                  // section of the beam
                      nsections,                // number of sections (spans)
                      ChVector3d(0, 0, 0),      // start point
                      ChVector3d(beamL, 0, 0),  // end point
                      VECT_Y,                   // suggested Y direction of section
                      order);                   // order (3 = cubic, etc)
    builder.GetLastBeamNodes().front()->SetFixed(true);
    builder.GetLastBeamNodes().back()->SetForce(ChVector3d(tip_axial, tip_load_y, tip_load_z));
    builder.GetLastBeamNodes().back()->SetTorque(ChVector3d(
        -tip_load_y * Sz + tip_load_z * Sy, tip_axial * Cz,
        -tip_axial * Cy));  // to cancel the effect of offsets - now should deflect on Y as normal cantilever.

    double y_init = builder.GetLastBeamNodes().back()->GetPos().y();

    // Do a linear static analysis.
    sys.DoStaticLinear();

    double numerical_displ = builder.GetLastBeamNodes().back()->GetPos().y() - y_init;

    /*
    ChVector3d mforce, mtorque;
    builder.GetLastBeamElements().front()->EvaluateSectionForceTorque(0, mforce, mtorque);
    std::cout << "        IGA sect torque = " << mtorque << std::endl;
    */

    return numerical_displ;
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create a Chrono physical system
    ChSystemNSC sys;

    // Solver settings
#ifndef CHRONO_PARDISO_MKL
    use_MKL = false;
#endif

    if (use_MKL) {
#ifdef CHRONO_PARDISO_MKL
        auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        solver->SetVerbose(true);
        sys.SetSolver(solver);
#endif
    } else {
        auto solver = chrono_types::make_shared<ChSolverMINRES>();
        sys.SetSolver(solver);
        solver->SetMaxIterations(500);
        solver->SetTolerance(1e-14);
        solver->EnableDiagonalPreconditioner(true);
        solver->SetVerbose(false);
    }

    // Run all tests
    for (int i = 1; i <= num_tests; i++) {
        std::cout << "============================\nTest # " << i << std::endl;
        double load = i * beam_tip_init_load;
        double analytical_displ = AnalyticalSol(load);
        double ancf3243_displ = ANCF3243_test(sys, load, i + 2);
        double ancf3333_displ = ANCF3333_test(sys, load, i + 2);
        double iga_displ = IGA_test(sys, load, i + 2, 3);
        double igaoffset_displ = IGA_test_offset(sys, load, i + 2, 3);
        double igaoffset_displ_rigidity = IGA_test_offset_rigidity(sys, load, i + 2, 3);
        double ancf3243_err = fabs((ancf3243_displ - analytical_displ) / analytical_displ);
        double ancf3333_err = fabs((ancf3333_displ - analytical_displ) / analytical_displ);
        double iga_err = fabs((iga_displ - analytical_displ) / analytical_displ);
        double igaoffset_err = fabs((igaoffset_displ - analytical_displ) / analytical_displ);
        double igaoffsetrigidity_err = fabs((igaoffset_displ_rigidity - analytical_displ) / analytical_displ);
        std::cout << "Kirchhoff beam models" << std::endl;
        std::cout << "  analytical: " << analytical_displ << std::endl;
        std::cout << "   ANCF 3243: " << ancf3243_displ << "  err: " << ancf3243_err << std::endl;
        std::cout << "   ANCF 3333: " << ancf3333_displ << "  err: " << ancf3333_err << std::endl;
        std::cout << "         IGA: " << iga_displ << "  err: " << iga_err << std::endl;
        std::cout << "    IGA offs: " << igaoffset_displ << "  err: " << igaoffset_err << std::endl;
        std::cout << "   IGA offsr: " << igaoffset_displ_rigidity << "  err: " << igaoffsetrigidity_err << std::endl;

        double euler_displ = EULER_test(sys, load, i + 2);
        double euleroffset_displ = EULER_test_offset(sys, load, i + 2);
        double analytical_displ_euler =
            (load * std::pow(beamL, 3)) /
            (3 * E_mod * (1. / 12.) * beam_wz *
             std::pow(beam_wy, 3));  // (P*L^3)/(3*E*I) + (P*L)/(k*A*G) , note no Timoshenko, no shear effect
        double euler_err = fabs((euler_displ - analytical_displ_euler) / analytical_displ);
        double euleroffset_err = fabs((euleroffset_displ - analytical_displ_euler) / analytical_displ);
        std::cout << "Euler-Bernoulli beam models" << std::endl;
        std::cout << "  analytical: " << analytical_displ_euler << std::endl;
        std::cout << "       Euler: " << euler_displ << "  err: " << euler_err << std::endl;
        std::cout << "  Euler offs: " << euleroffset_displ << "  err: " << euleroffset_err << std::endl;

        if (ancf3243_err > threshold || ancf3333_err > threshold || iga_err > threshold || igaoffset_err > threshold ||
            igaoffsetrigidity_err > threshold || euler_err > threshold || euleroffset_err > threshold) {
            std::cout << "\n\nTest failed" << std::endl;
            return 1;
        }
    }

    std::cout << "\n\nAll tests passed" << std::endl;
    return 0;
}
