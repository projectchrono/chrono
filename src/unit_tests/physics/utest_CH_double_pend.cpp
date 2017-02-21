// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Tests for timesteppers.
//
// The model consists of a rigid double pendulum moving under gravity.
// The test compares the simulation results (location and velocity of the center
// of mass of the second pendulum) against the solution obtained byintegrating
// the ODEs in minimal coordinates q = (phi1, phi2).
//
// =============================================================================

#include <cmath>
#include <valarray>

#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsValidation.h"

using namespace chrono;

// =============================================================================

double m1 = 1;
double l1 = 1;
double J1 = 1;
double m2 = 1;
double l2 = 1;
double J2 = 1;
double g = 10;

// =============================================================================

class ODEModel {
  public:
    ODEModel();
    void Simulate(double step, int num_steps);
    const utils::Data& GetData() const { return m_data; }
    void WriteData(double step, const std::string& filename);

  private:
    double m_phi1;
    double m_phi2;
    double m_phi1d;
    double m_phi2d;
    
    double m_phi1dd;
    double m_phi2dd;

    int m_num_intermediate_steps;

    utils::Data m_data;

    void CalcAcceleration();
};

ODEModel::ODEModel() {
    m_phi1 = 0;
    m_phi2 = 0;
    m_phi1d = 0;
    m_phi2d = 0;

    m_num_intermediate_steps = 10;

    m_data.resize(5);  // time + x_pend2 + y_pend2 + xd_pend2 + yd_pend2
}

void ODEModel::Simulate(double step, int num_steps) {
    double step1 = step / m_num_intermediate_steps;

    for (size_t col = 0; col < 5; col++)
        m_data[col].resize(num_steps);

    for (int it = 0; it < num_steps; it++) {
        m_data[0][it] = it * step;
        m_data[1][it] = l1 * cos(m_phi1) + 0.5 * l2 * cos(m_phi2);
        m_data[2][it] = l1 * sin(m_phi1) + 0.5 * l2 * sin(m_phi2);
        m_data[3][it] = -l1 * sin(m_phi1) * m_phi1d - 0.5 * l2 * sin(m_phi2) * m_phi2d;
        m_data[4][it] = l1 * cos(m_phi1) * m_phi1d + 0.5 * l2 * cos(m_phi2) * m_phi2d;
        for (int it1 = 0; it1 < m_num_intermediate_steps; it1++) {
            CalcAcceleration();
            m_phi1d += step1 * m_phi1dd;
            m_phi2d += step1 * m_phi2dd;
            m_phi1 += step1 * m_phi1d;
            m_phi2 += step1 * m_phi2d;
        }
    }
}

void ODEModel::CalcAcceleration() {
    double M11 = 0.25 * m1 * l1 * l1 + J1 + m2 * l1 * l1;
    double M12 = 0.5 * m2 * l1 * l2 * cos(m_phi2 - m_phi1);
    double M22 = 0.25 * m2 * l2 * l2 + J2;
    double det = M11 * M22 - M12 * M12;
    double f1 = -0.5 * m1 * l1 * g * cos(m_phi1) - m2 * l1 * g * cos(m_phi1) + 0.5 * m2 * l1 * l2 * m_phi2d * m_phi2d * sin(m_phi2 - m_phi1);
    double f2 = -0.5 * m2 * l2 * g * cos(m_phi2) - 0.5 * m2 * l1 * l2 * m_phi1d * m_phi1d * sin(m_phi2 - m_phi1);
    m_phi1dd = (M22 * f1 - M12 * f2) / det;
    m_phi2dd = (M11 * f2 - M12 * f1) / det;
}

void ODEModel::WriteData(double step, const std::string& filename) {
    assert(m_data.size() == 5);

    utils::CSV_writer csv(" ");

    for (size_t it = 0; it < m_data[0].size(); ++it) {
        csv << m_data[0][it] << m_data[1][it] << m_data[2][it] << m_data[3][it] << m_data[4][it] << std::endl;
    }

    csv.write_to_file(filename);
}

// =============================================================================

class ChronoModel {
  public:
    ChronoModel();
    std::shared_ptr<ChSystem> GetSystem() const { return m_system; }
    void Simulate(double step, int num_steps);
    const utils::Data& GetData() const { return m_data; }
    const utils::Data& GetCnstrData() const { return m_cnstr_data; }
    void WriteData(double step, const std::string& filename);

  private:
    std::shared_ptr<ChSystem> m_system;
    std::shared_ptr<ChBody> m_pend1;
    std::shared_ptr<ChBody> m_pend2;
    std::shared_ptr<ChLinkLockRevolute> m_revolute1;
    std::shared_ptr<ChLinkLockRevolute> m_revolute2;
    utils::Data m_data;
    utils::Data m_cnstr_data;
};

ChronoModel::ChronoModel() {
    // Create the Chrono physical system
    // ---------------------------------
    m_system = std::make_shared<ChSystem>();
    m_system->Set_G_acc(ChVector<>(0, -g, 0));

    // Create the ground body
    // ----------------------
    auto ground = std::make_shared<ChBody>();
    m_system->AddBody(ground);
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);

    // Create the first pendulum body
    // ------------------------------
    m_pend1 = std::make_shared<ChBody>();
    m_system->AddBody(m_pend1);
    m_pend1->SetIdentifier(1);
    m_pend1->SetMass(m1);
    m_pend1->SetInertiaXX(ChVector<>(1, 1, J1));
    m_pend1->SetPos(ChVector<>(l1 / 2, 0, 0));

    // Create the second pendulum body
    // -------------------------------
    m_pend2 = std::make_shared<ChBody>();
    m_system->AddBody(m_pend2);
    m_pend2->SetIdentifier(2);
    m_pend2->SetMass(m2);
    m_pend2->SetInertiaXX(ChVector<>(1, 1, J2));
    m_pend2->SetPos(ChVector<>(l1 + l2 / 2, 0, 0));

    // Revolute joint ground-pendulum
    // ------------------------------
    m_revolute1 = std::make_shared<ChLinkLockRevolute>();
    m_revolute1->Initialize(ground, m_pend1, ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
    m_system->AddLink(m_revolute1);

    // Revolute joint pendulum-pendulum
    // --------------------------------
    m_revolute2 = std::make_shared<ChLinkLockRevolute>();
    m_revolute2->Initialize(m_pend1, m_pend2, ChCoordsys<>(ChVector<>(l1, 0, 0), QUNIT));
    m_system->AddLink(m_revolute2);

    // Size the data
    // -------------
    m_data.resize(5);                // time + x_pend2 + y_pend2 + xd_pend2 + yd_pend2
    m_cnstr_data.resize(1 + 5 + 5);  // time + revolute1 + revolute2
}

void ChronoModel::Simulate(double step, int num_steps) {
    // Size the output data structures.
    for (size_t col = 0; col < 5; col++)
        m_data[col].resize(num_steps);
    for (size_t col = 0; col < 11; col++)
        m_cnstr_data[col].resize(num_steps);

    // Simulate the system for the specified number of steps.
    for (int it = 0; it < num_steps; it++) {
        // Save current state output information.
        m_data[0][it] = m_system->GetChTime();
        m_data[1][it] = m_pend2->GetPos().x();
        m_data[2][it] = m_pend2->GetPos().y();
        m_data[3][it] = m_pend2->GetPos_dt().x();
        m_data[4][it] = m_pend2->GetPos_dt().y();

        // Save current constraint violations.
        m_cnstr_data[0][it] = m_system->GetChTime();
        ChMatrix<>* C_1 = m_revolute1->GetC();
        for (int col = 0; col < 5; col++)
            m_cnstr_data[1 + col][it] = C_1->GetElement(col, 0);
        ChMatrix<>* C_2 = m_revolute2->GetC();
        for (int col = 0; col < 5; col++)
            m_cnstr_data[6 + col][it] = C_2->GetElement(col, 0);

        // Advance system state.
        m_system->DoStepDynamics(step);
    }
}

void ChronoModel::WriteData(double step, const std::string& filename) {
    assert(m_data.size() == 5);

    utils::CSV_writer csv(" ");

    for (size_t it = 0; it < m_data[0].size(); ++it) {
        csv << m_data[0][it] << m_data[1][it] << m_data[2][it] << m_data[3][it] << m_data[4][it] << std::endl;
    }

    csv.write_to_file(filename);
}

// =============================================================================

bool test_EULER_IMPLICIT_LINEARIZED(double step,
    int num_steps,
    const utils::Data& ref_data,
    double tol_state,
    double tol_cnstr) {
    std::cout << "EULER_IMPLICIT_LINEARIZED integrator" << std::endl;

    // Create Chrono model.
    ChronoModel model;
    std::shared_ptr<ChSystem> system = model.GetSystem();

    // Set integrator and modify parameters.
    system->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    // Set verbose solver and integrator (for debugging).
    ////system->GetSolver()->SetVerbose(true);
    ////system->GetTimestepper()->SetVerbose(true);

    // Simulate the model for the specified number of steps.
    model.Simulate(step, num_steps);
    ////model.WriteData(step, "chrono_swing_EULER_IMPL_LIN.txt");

    // Validate states (x and y for pendulum body).
    utils::DataVector norms_state;
    bool check_state = utils::Validate(model.GetData(), ref_data, utils::RMS_NORM, tol_state, norms_state);
    std::cout << "  validate states: " << (check_state ? "Passed" : "Failed") << "  (tolerance = " << tol_state
        << ")" << std::endl;
    for (size_t col = 0; col < norms_state.size(); col++)
        std::cout << "    " << norms_state[col] << std::endl;

    // Validate constraint violations.
    utils::DataVector norms_cnstr;
    bool check_cnstr = utils::Validate(model.GetCnstrData(), utils::RMS_NORM, tol_cnstr, norms_cnstr);
    std::cout << "  validate constraints: " << (check_cnstr ? "Passed" : "Failed") << "  (tolerance = " << tol_cnstr
        << ")" << std::endl;
    for (size_t col = 0; col < norms_cnstr.size(); col++)
        std::cout << "    " << norms_cnstr[col] << std::endl;

    return check_state && check_cnstr;
}

bool test_HHT(double step, int num_steps, const utils::Data& ref_data, double tol_state, double tol_cnstr) {
    std::cout << "HHT integrator" << std::endl;

    // Create Chrono model.
    ChronoModel model;
    std::shared_ptr<ChSystem> system = model.GetSystem();

    // Set solver and modify parameters.
    ////system->SetMaxItersSolverSpeed(200);
    ////system->SetMaxItersSolverStab(200);
    ////system->SetTolForce(1e-5);

    ////system->SetSolverType(ChSolver::Type::MINRES);
    ////auto solver = std::static_pointer_cast<ChSolverMINRES>(system.GetSolver());

    // Set integrator and modify parameters.
    system->SetTimestepperType(ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system->GetTimestepper());
    integrator->SetAlpha(0);
    integrator->SetMaxiters(20);
    integrator->SetAbsTolerances(1e-6);

    // Set verbose solver and integrator (for debugging).
    ////system->GetSolver()->SetVerbose(true);
    ////system->GetTimestepper()->SetVerbose(true);

    // Simulate the model for the specified number of steps.
    model.Simulate(step, num_steps);
    ////model.WriteData(step, "chrono_swing_HHT.txt");

    // Validate states (x and y for pendulum body).
    utils::DataVector norms_state;
    bool check_state = utils::Validate(model.GetData(), ref_data, utils::RMS_NORM, tol_state, norms_state);
    std::cout << "  validate states: " << (check_state ? "Passed" : "Failed") << "  (tolerance = " << tol_state
        << ")" << std::endl;
    for (size_t col = 0; col < norms_state.size(); col++)
        std::cout << "    " << norms_state[col] << std::endl;

    // Validate constraint violations.
    utils::DataVector norms_cnstr;
    bool check_cnstr = utils::Validate(model.GetCnstrData(), utils::RMS_NORM, tol_cnstr, norms_cnstr);
    std::cout << "  validate constraints: " << (check_cnstr ? "Passed" : "Failed") << "  (tolerance = " << tol_cnstr
        << ")" << std::endl;
    for (size_t col = 0; col < norms_cnstr.size(); col++)
        std::cout << "    " << norms_cnstr[col] << std::endl;

    return check_state && check_cnstr;
}

// =============================================================================

int main(int argc, char* argv[]) {
    double step = 2e-4;
    int num_steps = 5000;

    double tol_state = 1e-3;
    double tol_cnstr = 1e-6;

    // Create and simulate the ODE model.
    ODEModel ode_model;
    ode_model.Simulate(step, num_steps);
    const utils::Data& ref_data = ode_model.GetData();
    ode_model.WriteData(step, "ode_dpend.txt");

    // Perform the timestepper tests.
    bool passed = true;

    std::cout << "Validation tests for slider+pend system" << std::endl;
    std::cout << num_steps << " steps, using h = " << step << std::endl << std::endl;
    passed &= test_EULER_IMPLICIT_LINEARIZED(step, num_steps, ref_data, tol_state, tol_cnstr);
    passed &= test_HHT(step, num_steps, ref_data, tol_state, tol_cnstr);

    // Return 0 if all tests passed.
    return !passed;
}
