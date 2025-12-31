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
// Test simulation of IGA rods.
// Compare results with analytical formulation of Timoshenko beam.
//
// =============================================================================

#include <cmath>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "gtest/gtest.h"

using namespace chrono;
using namespace chrono::fea;

const double precision = 1e-4;
const double beamL = 0.4;
const double density = 1000.0;  // Beam material density
const double E = 0.02e10;       // Beam modulus of elasticity
const double nu = 0.38;         // Beam material Poisson ratio
const double wy = 0.012;
const double wz = 0.025;
const double tip_load = -10.0;

class Model {
  public:
    Model(int sec, int ord);
    std::shared_ptr<ChSystemNSC> GetSystem() const { return m_system; }
    std::shared_ptr<ChNodeFEAxyzrot> GetTipNode() const { return tip_node; }
    std::shared_ptr<ChElasticityCosseratSimple> GetElastModel() const { return melasticity; }

  private:
    std::shared_ptr<ChSystemNSC> m_system;
    std::shared_ptr<ChNodeFEAxyzrot> tip_node;
    std::shared_ptr<ChElasticityCosseratSimple> melasticity;
};

Model::Model(int sec, int ord) {
    m_system = chrono_types::make_shared<ChSystemNSC>();

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    m_system->SetSolver(solver);
    solver->SetMaxIterations(200);
    solver->SetTolerance(1e-15);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(false);

    auto my_mesh = chrono_types::make_shared<ChMesh>();
    my_mesh->SetAutomaticGravity(false);
    m_system->Add(my_mesh);

    auto msection =
        chrono_types::make_shared<ChBeamSectionCosseratEasyRectangular>(wy,      // width of section in y direction
                                                                        wz,      // width of section in z direction
                                                                        E,       // Young modulus
                                                                        E * nu,  // shear modulus
                                                                        density  // density
        );
    this->melasticity = std::dynamic_pointer_cast<ChElasticityCosseratSimple>(msection->GetElasticity());

    // Use the ChBuilderBeamIGA tool for creating a straight rod
    // divided in Nel elements:

    ChBuilderBeamIGA builder;
    builder.BuildBeam(my_mesh,                  // the mesh to put the elements in
                      msection,                 // section of the beam
                      sec,                      // number of sections (spans)
                      ChVector3d(0, 0, 0),      // start point
                      ChVector3d(beamL, 0, 0),  // end point
                      VECT_Y,                   // suggested Y direction of section
                      ord);                     // order (3 = cubic, etc)
    builder.GetLastBeamNodes().front()->SetFixed(true);
    builder.GetLastBeamNodes().back()->SetForce(ChVector3d(0, tip_load, 0));

    tip_node = builder.GetLastBeamNodes().back();
}

double AnalyticalTipDisp(std::shared_ptr<ChElasticityCosseratSimple> elast) {
    double poisson = elast->GetYoungModulus() / (2.0 * elast->GetShearModulus()) - 1.0;

    double Ks_y = 10.0 * (1.0 + poisson) / (12.0 + 11.0 * poisson);
    double analytic_timoshenko_displ =
        (tip_load * std::pow(beamL, 3)) / (3 * elast->GetYoungModulus() * (1. / 12.) * wz * std::pow(wy, 3)) +
        (tip_load * beamL) / (Ks_y * elast->GetShearModulus() * wz * wy);  // = (P*L^3)/(3*E*I) + (P*L)/(k*A*G)
    return analytic_timoshenko_displ;
}

TEST(IGA_Beam, Sim_vs_Analytical) {
    // int main() {
    //  Simulate a cantilever with IGA and compare to analytical solution
    int sections[4] = {8, 12, 16, 20};
    int order[4] = {7, 5, 3, 3};
    for (int i = 0; i < 4; i++) {
        // Model model1;
        Model model(sections[i], order[i]);

        // Do a linear static analysis.
        model.GetSystem()->DoStaticLinear();

        double chrono_disp = model.GetTipNode()->GetPos().y() - model.GetTipNode()->GetX0().GetPos().y();

        double ref_disp = AnalyticalTipDisp(model.GetElastModel());

        std::cout << "  Iter" << i << "  Static Sim disp:  " << chrono_disp << "  Analytical disp:  " << ref_disp
                  << "\n";
        ASSERT_NEAR(chrono_disp, ref_disp, precision);
    }
}
