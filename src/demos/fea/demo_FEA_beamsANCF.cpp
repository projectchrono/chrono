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
// Authors: Simone Benatti
// =============================================================================
//
// FEA for 3D beams: IGA and ANCF
//
// =============================================================================

#include <chrono>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChElementBeamIGA.h"
#include "chrono/fea/ChMesh.h"

#include "chrono_mkl/ChSolverMKL.h"

using namespace chrono;
using namespace chrono::fea;

bool use_MKL = false;
int num_tests = 5;

const float beam_tip_init_load = -2.0f;
const double beamL = 0.4;
const double rho = 1000.0;     // Beam material density
const double E_mod = 0.02e10;  // Beam modulus of elasticity
const double nu_rat = 0.38;    // Beam material Poisson ratio
const double beam_wy = 0.012;
const double beam_wz = 0.025;
const double k1 = 10 * (1 + nu_rat) / (12 + 11 * nu_rat);  // Timoshenko coefficient
const double k2 = k1;                                      // Timoshenko coefficient

double AnalyticalSol(float beam_tip_load) {
    double G_mod = E_mod * nu_rat;
    double poisson = E_mod / (2.0 * G_mod) - 1.0;
    double Ks_y = 10.0 * (1.0 + poisson) / (12.0 + 11.0 * poisson);

    // (P*L^3)/(3*E*I) + (P*L)/(k*A*G)
    double analytical_timoshenko_displ =
        (beam_tip_load * pow(beamL, 3)) / (3 * E_mod * (1. / 12.) * beam_wz * pow(beam_wy, 3)) +
        (beam_tip_load * beamL) / (Ks_y * G_mod * beam_wz * beam_wy);

    return analytical_timoshenko_displ;
}

void ANCF_test(ChSystem& mysys, float beam_tip_load, int NofEl, double analytical_displ) {
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
    GetLog() << "LINEAR STATIC ANCF cantilever, num. elements = " << NofEl
             << "  rel.error=  " << fabs((numerical_displ - analytical_displ) / analytical_displ) << "\n\n";
}

void IGA_test(ChSystem& mysys, float beam_tip_load, int nsections, int order, double analytical_displ) {
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
    GetLog() << "LINEAR STATIC IGA cantilever, order= " << order << "  nsections= " << nsections
             << "  rel.error=  " << fabs((numerical_displ - analytical_displ) / analytical_displ) << "\n\n";
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono::Engine physical system
    ChSystemNSC my_system;

    // Solver settings
    if (use_MKL) {
        auto mkl_solver = chrono_types::make_shared<ChSolverMKL>();
        mkl_solver->SetVerbose(true);
        my_system.SetSolver(mkl_solver);
    } else {
        auto solver = chrono_types::make_shared<ChSolverMINRES>();
        my_system.SetSolver(solver);
        solver->SetMaxIterations(500);
        solver->SetTolerance(1e-14);
        solver->EnableDiagonalPreconditioner(true);
        solver->SetVerbose(true);
    }

    // Run all tests
    for (int i = 1; i <= num_tests; i++) {
        std::cout << "\n============================\nTest # " << i << "\n" << std::endl;
        double analytical_displ = AnalyticalSol(i * beam_tip_init_load);
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        ANCF_test(my_system, i * beam_tip_init_load, i + 2, analytical_displ);
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        IGA_test(my_system, i * beam_tip_init_load, i + 2, 3, analytical_displ);
        std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();

        auto ANCFduration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        auto IGAduration = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();

        std::cout << "ANCF Elapsed Time: " << ANCFduration << "\n";
        std::cout << "IGA Elapsed Time:  " << IGAduration << "\n\n";
    }

    return 0;
}
