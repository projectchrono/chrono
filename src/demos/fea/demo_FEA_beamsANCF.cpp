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

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/solver/ChSolverPMINRES.h"
#include "chrono/timestepper/ChTimestepper.h"

#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChElementBeamIGA.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#define USE_MKL

#ifdef USE_MKL
#include "chrono_mkl/ChSolverMKL.h"
#endif

using namespace chrono;
using namespace chrono::fea;

int ID_current_example = 1;

const float beam_tip_init_load = -2.0f;
const double beamL = 0.4;
const double rho = 1000.0;     // Beam material density
const double E_mod = 0.02e10;  // Beam modulus of elasticity
const double nu_rat = 0.38;    // Beam material Poisson ratio
const double beam_wy = 0.012;
const double beam_wz = 0.025;
const double k1 = 10 * (1 + nu_rat) / (12 + 11 * nu_rat);  // Timoshenko coefficient
const double k2 = k1;                                      // Timoshenko coefficient

double ANCF_test(ChSystem& mysys, float beam_tip_load, int NofEl) {
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
    double y_init = builder.GetLastBeamNodes().back()->GetPos().y();

    // Mark completion of system construction
    mysys.SetupInitial();
    builder.GetLastBeamNodes().back()->SetForce(ChVector<>(0, beam_tip_load, 0));
    mysys.DoStaticLinear();
    double numerical_displ = builder.GetLastBeamNodes().back()->GetPos().y() - y_init;
    return numerical_displ;
}

//
// Example B: Automatic creation of the nodes and knots
// using the ChBuilderBeamIGA tool for creating a straight
// rod automatically divided in Nel elements:
//

void IGA_test(ChSystem& mysys, float beam_tip_load, double ANCF_res, int nsections = 32, int order = 2) {
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

    // Use the ChBuilderBeamIGA tool for creating a straight rod
    // divided in Nel elements:

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
    // This is needed if you want to see things in Irrlicht 3D view.

    // Mark completion of system construction
    mysys.SetupInitial();

    // Do a linear static analysis.
    mysys.DoStaticLinear();

    // For comparison with analytical results.
    double poisson = melasticity->GetYoungModulus() / (2.0 * melasticity->GetGshearModulus()) - 1.0;
    double Ks_y = 10.0 * (1.0 + poisson) / (12.0 + 11.0 * poisson);
    double analytic_timoshenko_displ =
        (beam_tip_load * pow(beamL, 3)) /
            (3 * melasticity->GetYoungModulus() * (1. / 12.) * beam_wz * pow(beam_wy, 3)) +
        (beam_tip_load * beamL) /
            (Ks_y * melasticity->GetGshearModulus() * beam_wz * beam_wy);  // = (P*L^3)/(3*E*I) + (P*L)/(k*A*G)
    double numerical_displ =
        builder.GetLastBeamNodes().back()->GetPos().y() - builder.GetLastBeamNodes().back()->GetX0().GetPos().y();

    GetLog() << "\n LINEAR STATIC IGA cantilever, order= " << order << "  nsections= " << nsections
             << "  rel.error=  " << fabs((numerical_displ - analytic_timoshenko_displ) / analytic_timoshenko_displ)
             << "\n";

    GetLog() << "\n LINEAR STATIC ANCF cantilever, nsections= " << nsections
             << "  rel.error=  " << fabs((ANCF_res - analytic_timoshenko_displ) / analytic_timoshenko_displ) << "\n";
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    int test_number = 5;
    // Create a Chrono::Engine physical system
    ChSystemNSC my_system;

    // Solver default settings for all the sub demos:
    my_system.SetSolverType(ChSolver::Type::MINRES);
    my_system.SetSolverWarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    my_system.SetMaxItersSolverSpeed(500);
    my_system.SetMaxItersSolverStab(500);
    my_system.SetTolForce(1e-14);

    auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
    msolver->SetVerbose(false);
    msolver->SetDiagonalPreconditioning(true);

#ifdef USE_MKL
    auto mkl_solver = chrono_types::make_shared<ChSolverMKL<>>();
    my_system.SetSolver(mkl_solver);
#endif

    // Run the sub-demos:

    for (int i = 1; i <= test_number; i++) {
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

        double ancfres = ANCF_test(my_system, i * beam_tip_init_load, i + 2);

        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        IGA_test(my_system, i * beam_tip_init_load, ancfres, i + 2, 3);

        std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();

        auto ANCFduration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        auto IGAduration = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();

        /*GetLog() << "\n ANCF Elapsed Time: " << ANCFduration << "\n\n";
        GetLog() << "\n IGA Elapsed Time: " << IGAduration << "\n\n";*/
        std::cout << "\n ANCF Elapsed Time: " << ANCFduration << "\n\n";
        std::cout << "\n IGA Elapsed Time: " << IGAduration << "\n\n";
    }

    return 0;
}
