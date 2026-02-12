// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Benchmark for implicit integrator (HHT) Jacobian update strategies.
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChElementBeamANCF_3243.h"
#include "chrono/utils/ChUtils.h"
#include "chrono/input_output/ChWriterCSV.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#ifdef CHRONO_MUMPS
    #include "chrono_mumps/ChSolverMumps.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fea;

using std::cout;
using std::endl;

// -----------------------------------------------------------------------------

// Define a custom point load with a time-dependent force
class TipLoad : public ChLoaderUatomic {
  public:
    TipLoad(std::shared_ptr<ChLoadableU> loadable) : ChLoaderUatomic(loadable), m_sys(nullptr) {}

    // Compute F=F(U). The load is a 6-row vector, i.e. a wrench: forceX, forceY, forceZ, torqueX, torqueY, torqueZ.
    virtual void ComputeF(double U,                    // normalized position along the beam axis [-1...1]
                          ChVectorDynamic<>& F,        // load at U (set to zero on entry)
                          ChVectorDynamic<>* state_x,  // if non-null, first update state (position) to this
                          ChVectorDynamic<>* state_w   // if non-null, fiurst update state (speed) to this
                          ) override {
        double t = m_sys->GetChTime();
        double Fmax = -500;
        double tc = 2;
        double Fz = Fmax;
        if (t < tc) {
            Fz = 0.5 * Fmax * (1 - std::cos(CH_PI * t / tc));
        }

        F(2) = Fz;  // Apply the force along the global Z axis
    }

    void SetSystem(ChSystem* sys) { m_sys = sys; }

  private:
    ChSystem* m_sys;
};

// -----------------------------------------------------------------------------

void SolveHHT(ChSolver::Type solver_type,
              ChTimestepperImplicit::JacobianUpdate jacobian_update,
              const std::string& out_dir,
              bool verbose) {
    cout << ChSolver::GetTypeAsString(solver_type) << endl;
    cout << ChTimestepperImplicit::GetJacobianUpdateMethodAsString(jacobian_update) << endl;

    // Create containing system
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));

    // Set up solver
    switch (solver_type) {
        case ChSolver::Type::PARDISO_MKL: {
#ifdef CHRONO_PARDISO_MKL
            auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();
            solver->UseSparsityPatternLearner(true);
            solver->LockSparsityPattern(true);
            solver->SetVerbose(false);
            sys.SetSolver(solver);
#endif
            break;
        }
        case ChSolver::Type::MUMPS: {
#ifdef CHRONO_MUMPS
            auto solver = chrono_types::make_shared<ChSolverMumps>();
            solver->UseSparsityPatternLearner(true);
            solver->LockSparsityPattern(true);
            solver->SetVerbose(false);
            sys.SetSolver(solver);
#endif
            break;
        }
        default:
            return;
    }

    // Set up integrator
    auto integrator = chrono_types::make_shared<ChTimestepperHHT>(&sys);
    integrator->SetAlpha(-0.2);
    integrator->SetAbsTolerances(1e-5);
    integrator->SetVerbose(false);
    integrator->SetJacobianUpdateMethod(jacobian_update);
    integrator->SetMaxIters(4);
    integrator->AcceptTerminatedStep(false);
    integrator->SetStepControl(false);
    sys.SetTimestepper(integrator);

    // Mesh properties
    double length = 5;       // m
    double width = 0.1;      // m
    double thickness = 0.1;  // m
    double rho = 8000;       // kg/m^3
    double E = 4e8;          // Pa
    double nu = 0;           // Poisson effect neglected for this model
    // Timoshenko shear correction coefficients for a rectangular cross-section
    double k1 = 10 * (1 + nu) / (12 + 11 * nu);
    double k2 = k1;

    auto material = chrono_types::make_shared<ChMaterialBeamANCF>(rho, E, nu, k1, k2);

    // Create mesh container
    auto mesh = chrono_types::make_shared<ChMesh>();
    mesh->SetAutomaticGravity(false);
    sys.Add(mesh);

    // Populate mesh with beam elements of specified type
    // Populate the mesh container with the nodes and elements for the meshed beam
    int num_elements = 20;
    int num_nodes = num_elements + 1;
    double dx = length / (num_nodes - 1);

    // Create the first node and fix it completely to ground (Cantilever constraint)
    auto nodeA = chrono_types::make_shared<ChNodeFEAxyzDDD>(ChVector3d(0, 0, 0));
    nodeA->SetFixed(true);
    mesh->AddNode(nodeA);

    std::shared_ptr<ChElementBeamANCF_3243> last_element;
    std::shared_ptr<ChNodeFEAxyzDDD> last_node;

    for (int i = 1; i <= num_elements; i++) {
        auto nodeB = chrono_types::make_shared<ChNodeFEAxyzDDD>(ChVector3d(dx * i, 0, 0));
        mesh->AddNode(nodeB);

        auto element = chrono_types::make_shared<ChElementBeamANCF_3243>();
        element->SetNodes(nodeA, nodeB);
        element->SetDimensions(dx, thickness, width);
        element->SetMaterial(material);
        element->SetAlphaDamp(0.0);
        mesh->AddElement(element);

        nodeA = nodeB;
        last_element = element;
        last_node = nodeB;
    }

    // Create the load container and add to the current system
    auto loadcontainer = chrono_types::make_shared<ChLoadContainer>();
    sys.Add(loadcontainer);

    // Create a custom load that uses the custom loader above.
    auto loader = chrono_types::make_shared<TipLoad>(last_element);
    loader->SetSystem(&sys);      // set containing system
    loader->SetApplication(1.0);  // specify application point

    auto load = chrono_types::make_shared<ChLoad>(loader);
    loadcontainer->Add(load);  // add the load to the load container.

    // Simulation loop
    double tend = 1;
    double step = 1e-3;
    double out_fps = 1000;

    double t = 0;
    double sim_time = 0;
    int sim_frame = 0;
    int out_frame = 0;

    ChWriterCSV csv(" ");
    csv << t << last_node->GetPos() << endl;

    while (t < tend) {
        if (verbose)
            cout << "t = " << t << endl;
        else
            cout << "\r" << t;

        try {
            sys.DoStepDynamics(step);
        } catch (const std::exception&) {
            cout << "Integration failed" << endl << endl;
            return;
        }
        sim_time += sys.GetTimerStep();

        t += step;
        sim_frame++;

        if (t > out_frame / out_fps) {
            csv << t << last_node->GetPos() << endl;
            out_frame++;
        }
    }

    cout << "\nLast node pos:   " << last_node->GetPos() << endl;
    cout << "Simulation time: " << sim_time << endl;
    cout << "  Num steps:       " << sim_frame << endl;
    cout << "  Num iterations:  " << integrator->GetNumIterations() << endl;
    cout << "  Num setups:      " << integrator->GetNumSetupCalls() << endl;
    cout << "  Num solves:      " << integrator->GetNumSolveCalls() << endl;
    cout << endl;

    std::string out_file =
        out_dir + "/ANCFbeam_" + ChTimestepperImplicit::GetJacobianUpdateMethodAsString(jacobian_update) + ".out";
    csv.WriteToFile(out_file);

    return;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2023 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create (if needed) output directory
    std::string out_dir = GetChronoOutputPath() + "FEA_TEST_HHT";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

#ifdef CHRONO_PARDISO_MKL
    cout << endl;
    SolveHHT(ChSolver::Type::PARDISO_MKL, ChTimestepperImplicit::JacobianUpdate::NEVER, out_dir, false);
    SolveHHT(ChSolver::Type::PARDISO_MKL, ChTimestepperImplicit::JacobianUpdate::EVERY_STEP, out_dir, false);
    SolveHHT(ChSolver::Type::PARDISO_MKL, ChTimestepperImplicit::JacobianUpdate::EVERY_ITERATION, out_dir, false);
    SolveHHT(ChSolver::Type::PARDISO_MKL, ChTimestepperImplicit::JacobianUpdate::AUTOMATIC, out_dir, false);
#endif

#ifdef CHRONO_MUMPS
    cout << endl;
    SolveHHT(ChSolver::Type::MUMPS, ChTimestepperImplicit::JacobianUpdate::NEVER, out_dir, false);
    SolveHHT(ChSolver::Type::MUMPS, ChTimestepperImplicit::JacobianUpdate::EVERY_STEP, out_dir, false);
    SolveHHT(ChSolver::Type::MUMPS, ChTimestepperImplicit::JacobianUpdate::EVERY_ITERATION, out_dir, false);
    SolveHHT(ChSolver::Type::MUMPS, ChTimestepperImplicit::JacobianUpdate::AUTOMATIC, out_dir, false);
#endif
}
