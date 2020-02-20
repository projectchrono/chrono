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
#include "chrono/fea/ChElementBeamIGA.h"
#include "chrono/fea/ChMesh.h"

#ifdef CHRONO_MKL
#include "chrono_mkl/ChSolverMKL.h"
#endif

using namespace chrono;
using namespace chrono::fea;

bool use_MKL = false;
int num_tests = 5;
double threshold = 0.05; // max 5% error allowed

const double beam_tip_init_load = -2.0f;
const double beamL = 0.4;
const double rho = 1000.0;     // Beam material density
const double E_mod = 0.02e10;  // Beam modulus of elasticity
const double nu_rat = 0.38;    // Beam material Poisson ratio
const double beam_wy = 0.012;
const double beam_wz = 0.025;
const double k1 = 10 * (1 + nu_rat) / (12 + 11 * nu_rat);  // Timoshenko coefficient
const double k2 = k1;                                      // Timoshenko coefficient

double AnalyticalSol(double beam_tip_load) {
    double G_mod = E_mod * nu_rat;
    double poisson = E_mod / (2.0 * G_mod) - 1.0;
    double Ks_y = 10.0 * (1.0 + poisson) / (12.0 + 11.0 * poisson);

    // (P*L^3)/(3*E*I) + (P*L)/(k*A*G)
    double analytical_timoshenko_displ =
        (beam_tip_load * pow(beamL, 3)) / (3 * E_mod * (1. / 12.) * beam_wz * pow(beam_wy, 3)) +
        (beam_tip_load * beamL) / (Ks_y * G_mod * beam_wz * beam_wy);

    return analytical_timoshenko_displ;
}

double ANCF_test(ChSystem& mysys, double beam_tip_load, int NofEl) {
    // Clear previous demo, if any:
    mysys.Clear();
    mysys.SetChTime(0);

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    // Remember to add it to the system.
    auto my_mesh = chrono_types::make_shared<ChMesh>();
    my_mesh->SetAutomaticGravity(false);
    mysys.GetSystem()->Add(my_mesh);

    auto material = chrono_types::make_shared<ChMaterialBeamANCF>(rho, E_mod, nu_rat, E_mod * nu_rat, k1, k2);

    ChBuilderBeamANCFFullyPar builder;
    builder.BuildBeam(my_mesh, material, NofEl, ChVector<>(0, 0, 0), ChVector<>(beamL, 0, 0), beam_wy, beam_wz, VECT_Y,
                      VECT_Z);
    builder.GetLastBeamNodes().front()->SetFixed(true);
    builder.GetLastBeamNodes().back()->SetForce(ChVector<>(0, beam_tip_load, 0));

    double y_init = builder.GetLastBeamNodes().back()->GetPos().y();

    // Do a linear static analysis.
    mysys.DoStaticLinear();

    double numerical_displ = builder.GetLastBeamNodes().back()->GetPos().y() - y_init;

    return numerical_displ;
}

double IGA_test(ChSystem& mysys, double beam_tip_load, int nsections, int order) {
    // Clear previous demo, if any:
    mysys.GetSystem()->Clear();
    mysys.GetSystem()->SetChTime(0);

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    // Remember to add it to the system.
    auto my_mesh = chrono_types::make_shared<ChMesh>();
    my_mesh->SetAutomaticGravity(false);
    mysys.Add(my_mesh);

    auto melasticity = chrono_types::make_shared<ChElasticityCosseratSimple>();
    melasticity->SetYoungModulus(E_mod);
    melasticity->SetGshearModulus(E_mod * nu_rat);
    melasticity->SetBeamRaleyghDamping(0.0000);

    auto msection = chrono_types::make_shared<ChBeamSectionCosserat>(melasticity);
    msection->SetDensity(rho);
    msection->SetAsRectangularSection(beam_wy, beam_wz);

    // Use the ChBuilderBeamIGA tool for creating a straight rod divided in Nel elements
    ChBuilderBeamIGA builder;
    builder.BuildBeam(my_mesh,                  // the mesh to put the elements in
                      msection,                 // section of the beam
                      nsections,                // number of sections (spans)
                      ChVector<>(0, 0, 0),      // start point
                      ChVector<>(beamL, 0, 0),  // end point
                      VECT_Y,                   // suggested Y direction of section
                      order);                   // order (3 = cubic, etc)
    builder.GetLastBeamNodes().front()->SetFixed(true);
    builder.GetLastBeamNodes().back()->SetForce(ChVector<>(0, beam_tip_load, 0));

    double y_init = builder.GetLastBeamNodes().back()->GetX0().GetPos().y();

    // Do a linear static analysis.
    mysys.DoStaticLinear();

    double numerical_displ = builder.GetLastBeamNodes().back()->GetPos().y() - y_init;
    return numerical_displ;
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono::Engine physical system
    ChSystemNSC my_system;

    // Solver settings
#ifndef CHRONO_MKL
    use_mkl = false;
#endif

    if (use_MKL) {
#ifdef CHRONO_MKL
        auto mkl_solver = chrono_types::make_shared<ChSolverMKL>();
        mkl_solver->SetVerbose(true);
        my_system.SetSolver(mkl_solver);
#endif
    } else {
        auto solver = chrono_types::make_shared<ChSolverMINRES>();
        my_system.SetSolver(solver);
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
        double ancf_displ = ANCF_test(my_system, load, i + 2);
        double iga_displ = IGA_test(my_system, load, i + 2, 3);
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
