// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Demo code for using a physics item with external dynamics described by ODEs.
//
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChExternalDynamics.h"

#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/timestepper/ChTimestepperHHT.h"

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

using namespace chrono;

std::string out_dir = GetChronoOutputPath() + "DEMO_EXTERNAL_DYNAMICS";

class VanDerPolODE : public ChExternalDynamics {
  public:
    VanDerPolODE(double mu) : m_mu(mu) {}

    virtual int GetNumStates() const override { return 2; }

    virtual bool IsStiff() const override { return m_mu > 10; }

    virtual void SetInitialConditions(ChVectorDynamic<>& y0) override {
        y0(0) = 2.0;
        y0(1) = 0.0;
    }

    virtual void CalculateRHS(double time,                 // current time
                              const ChVectorDynamic<>& y,  // current ODE states
                              ChVectorDynamic<>& rhs       // output ODE right-hand side vector
                              ) override {
        rhs(0) = y(1);
        rhs(1) = m_mu * (1 - y(0) * y(0)) * y(1) - y(0);
    }

    virtual bool CalculateJac(double time,                   // current time
                              const ChVectorDynamic<>& y,    // current ODE states
                              const ChVectorDynamic<>& rhs,  // current ODE right-hand side vector
                              ChMatrixDynamic<>& J           // output Jacobian matrix
                              ) override {
        // Do not provide Jacobian information if problem not stiff
        if (!IsStiff())
            return false;

        // Generate analytical Jacobian
        J(0, 0) = 0;
        J(0, 1) = 1;
        J(1, 0) = -2 * m_mu * y(0) * y(1) - 1;
        J(1, 1) = m_mu * (1 - y(0) * y(0));
        return true;
    }

  private:
    double m_mu;
};

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2023 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create (if needed) output directory
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Non-stiff ODE example
    {
        ChSystemSMC sys;

        auto vdp = chrono_types::make_shared<VanDerPolODE>(1.0);
        vdp->Initialize();
        sys.Add(vdp);

        double t_end = 10;
        double t_step = 1e-2;
        double t = 0;
        ChVectorDynamic<> y(2);

        Eigen::IOFormat rowFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "  ", "  ", "", "", "", "");
        utils::CSV_writer csv(" ");
        y = vdp->GetInitialStates();
        csv << t << y.format(rowFmt) << std::endl;

        while (t < t_end) {
            sys.DoStepDynamics(t_step);
            y = vdp->GetStates();
            csv << t << y.format(rowFmt) << std::endl;
            t += t_step;
        }

        std::string out_file = out_dir + "/vanDerPol_nonstiff.out";
        csv.write_to_file(out_file);

#ifdef CHRONO_POSTPROCESS
        postprocess::ChGnuPlot gplot(out_dir + "/vanDerPol_nonstiff.gpl");
        gplot.SetGrid();
        gplot.SetLabelX("time");
        gplot.SetLabelY("Y");
        gplot.SetTitle("van der Pol (nonstiff)");
        gplot.Plot(out_file, 1, 2, "y0", " with lines lt -1 lw 2");
        gplot.Plot(out_file, 1, 3, "y1", " with lines lt 2 lw 2");
#endif
    }

    // Stiff ODE example
    {
        ChSystemSMC sys;

        auto vdp = chrono_types::make_shared<VanDerPolODE>(300.0);
        vdp->Initialize();
        sys.Add(vdp);

        auto solver = chrono_types::make_shared<ChSolverSparseQR>();
        sys.SetSolver(solver);
        solver->UseSparsityPatternLearner(true);
        solver->LockSparsityPattern(true);
        solver->SetVerbose(false);

#ifdef CHRONO_PARDISO_MKL
        ////auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        ////sys.SetSolver(solver);
        ////solver->UseSparsityPatternLearner(false);
        ////solver->LockSparsityPattern(false);
#endif

        sys.SetTimestepperType(ChTimestepper::Type::HHT);
        auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
        mystepper->SetAlpha(-0.2);
        mystepper->SetMaxiters(100);
        mystepper->SetAbsTolerances(1e-5);
        mystepper->SetModifiedNewton(true);
        mystepper->SetVerbose(false);

        double t_end = 300;
        double t_step = 5e-3;
        double t = 0;
        ChVectorDynamic<> y(2);

        Eigen::IOFormat rowFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "  ", "  ", "", "", "", "");
        utils::CSV_writer csv(" ");
        y = vdp->GetInitialStates();
        csv << t << y.format(rowFmt) << std::endl;

        while (t < t_end) {
            sys.DoStepDynamics(t_step);
            y = vdp->GetStates();
            csv << t << y.format(rowFmt) << std::endl;
            t += t_step;
        }

        std::string out_file = out_dir + "/vanDerPol_Stiff.out";
        csv.write_to_file(out_file);

#ifdef CHRONO_POSTPROCESS
        postprocess::ChGnuPlot gplot(out_dir + "/vanDerPol_stiff.gpl");
        gplot.SetGrid();
        gplot.SetLabelX("time");
        gplot.SetLabelY("Y");
        gplot.SetTitle("van der Pol (stiff)");
        gplot.Plot(out_file, 1, 2, "y0", " with lines lt -1 lw 2");
#endif
    }
}
