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

#include <cmath>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChExternalDynamicsODE.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#ifdef CHRONO_MUMPS
    #include "chrono_mumps/ChSolverMumps.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;

using std::cout;
using std::endl;

// -----------------------------------------------------------------------------

// Semilinear 1D wave equation
//    u_tt - c(t) * u_xx + g(u) = 0
//    I.C. u(x, 0) = u0(x)
//         u_t((x,0) = 0
//    B.C. u(0,t) = u(L,t) = 0
//
//  Use:
//    g(u) = u^3
//    c(t) =   1, if t < t_disc
//           100, if t >= t_disc
//    u0(x) = sin(pi * x)
//    0 <= x <= 1
//    t > 0
//
//  [ u | u_t ] = [ y | v ] 
//  y' = v
//  v' = y_xx - g(y)

class WaveEquation : public ChExternalDynamicsODE {
  public:
    WaveEquation(int n, double t_disc) : n(n), t_disc(t_disc), dx(1.0 / (n + 1)) {}

    virtual WaveEquation* Clone() const override { return new WaveEquation(*this); }

    virtual unsigned int GetNumStates() const override { return 2*n; }

    virtual bool IsStiff() const override { return true; }

    virtual void SetInitialConditions(ChVectorDynamic<>& y0) override {
        for (size_t i = 0; i < n; i++) {
            double x = (i + 1) * dx;
            y0(i) = std::sin(CH_PI * x);
            y0(i + n) = 0;
        }
    }

    virtual void CalculateRHS(double time,                 // current time
                              const ChVectorDynamic<>& y,  // current ODE states
                              ChVectorDynamic<>& rhs       // output ODE right-hand side vector
                              ) override {
        double oodx2 = 1 / (dx * dx);
        double c = (time < t_disc) ? 1.0 : 100.0;

        for (size_t i = 0; i < n; i++) {
            double y2 = y(i) * y(i);
            double y_l = (i == 0) ? 0 : y(i - 1);
            double y_r = (i == n - 1) ? 0 : y(i + 1);
            rhs(i) = y(i + n);
            rhs(i + n) = c * (y_l - 2 * y(i) + y_r) * oodx2 - y(i) * y2;
        }
    }

    virtual bool CalculateJac(double time,                   // current time
                              const ChVectorDynamic<>& y,    // current ODE states
                              const ChVectorDynamic<>& rhs,  // current ODE right-hand side vector
                              ChMatrixDynamic<>& J           // output Jacobian matrix
                              ) override {
        // Do not provide Jacobian information if problem not stiff
        if (!IsStiff())
            return false;

        double oodx2 = 1 / (dx * dx);
        double c = (time < t_disc) ? 1.0 : 100.0;

        // Generate analytical Jacobian
        J.setZero();
        J.topRightCorner(n, n).setIdentity();
        for (size_t i = 0; i < n; i++) {
            double y2 = y(i) * y(i);
            J(i + n, i) = -2 * c * oodx2 - 3 * y2;
            if (i > 0)
                J(i + n, i - 1) = c * oodx2;
            if (i < n - 1)
                J(i + n, i + 1) = c * oodx2;
        }

        return true;
    }

  private:
    int n;
    double t_disc;
    double dx;
};

// -----------------------------------------------------------------------------

void SolveHHT(ChSolver::Type solver_type,
              ChTimestepperImplicit::JacobianUpdate jacobian_update,
              const std::string& out_dir,
              bool verbose) {
    cout << ChSolver::GetTypeAsString(solver_type) << endl;
    cout << ChTimestepperImplicit::GetJacobianUpdateMethodAsString(jacobian_update) << endl;

    ChSystemSMC sys;

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
    integrator->SetMaxIters(100);
    integrator->SetAbsTolerances(1e-5);
    integrator->SetVerbose(verbose);
    integrator->SetJacobianUpdateMethod(jacobian_update);
    integrator->SetStepControl(false);
    sys.SetTimestepper(integrator);

    // Create problem
    double tend = 0.6;
    double step = 1e-3;
    double out_fps = 1000;

    int n = 100;
    auto wave = chrono_types::make_shared<WaveEquation>(n, tend / 2);
    wave->Initialize();
    sys.Add(wave);

    double t = 0;
    int sim_frame = 0;
    int out_frame = 0;

    auto num_states = wave->GetNumStates();
    ChVectorDynamic<> y(num_states);
    y = wave->GetInitialStates();

    Eigen::IOFormat rowFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "  ", "  ", "", "", "", "");
    utils::ChWriterCSV csv(" ");
    csv << t << y.format(rowFmt) << endl;

    double sim_time = 0;

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
            y = wave->GetStates();
            csv << t << y.format(rowFmt) << endl;
            out_frame++;
        }
    }

    cout << "\nFinal ||y||:     " << y.segment(0, n).norm() << endl;
    cout << "Simulation time: " << sim_time << endl;
    cout << "  Num steps:       " << sim_frame << endl;
    cout << "  Num iterations:  " << integrator->GetNumIterations() << endl;
    cout << "  Num setups:      " << integrator->GetNumSetupCalls() << endl;
    cout << "  Num solves:      " << integrator->GetNumSolveCalls() << endl;
    cout << endl;

    std::string out_file =
        out_dir + "/wave_" + ChTimestepperImplicit::GetJacobianUpdateMethodAsString(jacobian_update) + ".out";
    csv.WriteToFile(out_file);

    return;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2023 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create (if needed) output directory
    std::string out_dir = GetChronoOutputPath() + "WAVE_TEST_HHT";
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
