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
// Authors: Simone Benatti, Radu Serban
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

#ifdef CHRONO_MKL
#include "chrono_mkl/ChSolverMKL.h"
#endif

using namespace chrono;
using namespace chrono::fea;

const int num_tests = 5;
const double beam_tip_init_load = -2.0f;  // base tip load (load increased for each test)
const double threshold = 0.05;            // max 5% error allowed

const double beamL = 0.4;
const double rho = 1000.0;     // Beam material density
const double E_mod = 0.02e10;  // Beam modulus of elasticity
const double nu_rat = 0.38;    // Beam material Poisson ratio
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
        (tip_load * pow(beamL, 3)) / (3 * E_mod * (1. / 12.) * beam_wz * pow(beam_wy, 3)) +
        (tip_load * beamL) / (Ks_y * G_mod * beam_wz * beam_wy);

    return analytical_timoshenko_displ;
}

double ANCF_test(ChSystem& sys, double tip_load, int nelements) {
    // Clear previous demo, if any:
    sys.Clear();
    sys.SetChTime(0);

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    // Remember to add it to the system.
    auto mesh = chrono_types::make_shared<ChMesh>();
    mesh->SetAutomaticGravity(false);
    sys.Add(mesh);

    auto material = chrono_types::make_shared<ChMaterialBeamANCF>(rho, E_mod, nu_rat, E_mod * nu_rat, k1, k2);

    ChBuilderBeamANCF builder;
    builder.BuildBeam(mesh, material, nelements, ChVector<>(0, 0, 0), ChVector<>(beamL, 0, 0), beam_wy, beam_wz, VECT_Y,
                      VECT_Z);
    builder.GetLastBeamNodes().front()->SetFixed(true);
    builder.GetLastBeamNodes().back()->SetForce(ChVector<>(0, tip_load, 0));

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

    auto melasticity = chrono_types::make_shared<ChElasticityCosseratSimple>();
    melasticity->SetYoungModulus(E_mod);
    melasticity->SetGshearModulus(E_mod * nu_rat);
    melasticity->SetBeamRaleyghDamping(0.0000);

    auto section = chrono_types::make_shared<ChBeamSectionCosserat>(melasticity);
    section->SetDensity(rho);
    section->SetAsRectangularSection(beam_wy, beam_wz);

    // Use the ChBuilderBeamIGA tool for creating a straight rod divided in Nel elements
    ChBuilderBeamIGA builder;
    builder.BuildBeam(mesh,                     // the mesh to put the elements in
                      section,                  // section of the beam
                      nsections,                // number of sections (spans)
                      ChVector<>(0, 0, 0),      // start point
                      ChVector<>(beamL, 0, 0),  // end point
                      VECT_Y,                   // suggested Y direction of section
                      order);                   // order (3 = cubic, etc)
    builder.GetLastBeamNodes().front()->SetFixed(true);
    builder.GetLastBeamNodes().back()->SetForce(ChVector<>(0, tip_load, 0));

    double y_init = builder.GetLastBeamNodes().back()->GetX0().GetPos().y();

    // Do a linear static analysis.
    sys.DoStaticLinear();

    double numerical_displ = builder.GetLastBeamNodes().back()->GetPos().y() - y_init;
    return numerical_displ;
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono::Engine physical system
    ChSystemNSC sys;

    // Solver settings
#ifndef CHRONO_MKL
    use_MKL = false;
#endif

    if (use_MKL) {
#ifdef CHRONO_MKL
        auto solver = chrono_types::make_shared<ChSolverMKL>();
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
        double ancf_displ = ANCF_test(sys, load, i + 2);
        double iga_displ = IGA_test(sys, load, i + 2, 3);
        double ancf_err = fabs((ancf_displ - analytical_displ) / analytical_displ);
        double iga_err = fabs((iga_displ - analytical_displ) / analytical_displ);
        std::cout << "analytical: " << analytical_displ << std::endl;
        std::cout << "      ANCF: " << ancf_displ << "  err: " << ancf_err << std::endl;
        std::cout << "       IGA: " << iga_displ << "  err: " << iga_err << std::endl;

        if (ancf_err > threshold || iga_err > threshold) {
            std::cout << "\n\nTest failed" << std::endl;
            return 1;
        }
    }

    std::cout << "\n\nAll tests passed" << std::endl;
    return 0;
}
