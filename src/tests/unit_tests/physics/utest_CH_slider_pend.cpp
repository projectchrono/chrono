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
// Authors: Radu Serban
// =============================================================================
//
// Tests for timesteppers.
//
// The model consists of a pendulum attached to a slider. The slider is
// connected to ground through a horizontal prismatic joint, while the pendulum
// is connected to the slider through a revolute joint. A horizontal spring is
// attached to the slider and the mechanism moves under gravity.
// The test compares the simulation results (location of the pendulum center of
// mass) against the solution obtained by integrating the ODEs in minimal
// coordinates q = (x1, phi2).
//
//                y
//                ^
//                |
//    g |         |                   k1, rest_len = 0
//      |         |    m1 _______/\/\/\/\/\/\__
//     \|/        |      |       |
//             ---|------|---o---|--------------------> x
//                |      |____\__|
//                             \
//                              \
//                               \ m2, l2, J2
//                                \
//
// =============================================================================

#include <cmath>
#include <valarray>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsValidation.h"

using namespace chrono;

// =============================================================================

double m1 = 1;   // mass of slider body
double k1 = 10;  // spring constant
double m2 = 1;   // mass of pendulum body
double l2 = 1;   // length of pendulum
double J2 = 1;   // moment of inertia of pendulum (Z component)
double g = 10;   // gravitational acceleration

// =============================================================================

class ODEModel {
  public:
    ODEModel();
    void Simulate(double step, int num_steps);
    const utils::Data& GetData() const { return m_data; }
    void WriteData(double step, const std::string& filename);

  private:
    double m_x1;
    double m_phi2;
    double m_x1d;
    double m_phi2d;

    double m_x1dd;
    double m_phi2dd;

    int m_num_intermediate_steps;

    utils::Data m_data;

    void CalcAcceleration();
};

ODEModel::ODEModel() {
    m_x1 = 0;
    m_phi2 = 0;
    m_x1d = 0;
    m_phi2d = 0;

    m_num_intermediate_steps = 10;

    m_data.resize(3);
}

void ODEModel::Simulate(double step, int num_steps) {
    double step1 = step / m_num_intermediate_steps;

    for (size_t col = 0; col < 3; col++)
        m_data[col].resize(num_steps);

    for (int it = 0; it < num_steps; it++) {
        m_data[0][it] = it * step;
        m_data[1][it] = m_x1 + 0.5 * l2 * cos(m_phi2);
        m_data[2][it] = 0.5 * l2 * sin(m_phi2);
        for (int it1 = 0; it1 < m_num_intermediate_steps; it1++) {
            CalcAcceleration();
            m_x1d += step1 * m_x1dd;
            m_phi2d += step1 * m_phi2dd;
            m_x1 += step1 * m_x1d;
            m_phi2 += step1 * m_phi2d;
        }
    }
}

void ODEModel::CalcAcceleration() {
    double M11 = m1 + m2;
    double M12 = -0.5 * m2 * l2 * sin(m_phi2);
    double M22 = 0.25 * m2 * l2 * l2 + J2;
    double det = M11 * M22 - M12 * M12;
    double f1 = 0.5 * m2 * l2 * cos(m_phi2) * m_phi2d * m_phi2d - k1 * m_x1;
    double f2 = -0.5 * m2 * l2 * g * cos(m_phi2);
    m_x1dd = (M22 * f1 - M12 * f2) / det;
    m_phi2dd = (M11 * f2 - M12 * f1) / det;
}

void ODEModel::WriteData(double step, const std::string& filename) {
    assert(m_data.size() == 3);

    utils::CSV_writer csv(" ");

    for (size_t it = 0; it < m_data[0].size(); ++it) {
        csv << m_data[0][it] << m_data[1][it] << m_data[2][it] << std::endl;
    }

    csv.write_to_file(filename);
}

// =============================================================================

// Base Chrono model.
class ChronoModel {
  public:
    std::shared_ptr<ChSystemNSC> GetSystem() const { return m_system; }
    void Simulate(double step, int num_steps);
    const utils::Data& GetData() const { return m_data; }
    const utils::Data& GetCnstrData() const { return m_cnstr_data; }
    void WriteData(double step, const std::string& filename);

    virtual std::string GetJointType() const = 0;

  protected:
    ChronoModel();
    virtual ChMatrix<>* GetViolation1() = 0;
    virtual ChMatrix<>* GetViolation2() = 0;

    std::shared_ptr<ChSystemNSC> m_system;
    std::shared_ptr<ChBody> m_ground;
    std::shared_ptr<ChBody> m_slider;
    std::shared_ptr<ChBody> m_pend;
    utils::Data m_data;
    utils::Data m_cnstr_data;
};

// Chrono model using LinkLock joints.
class ChronoModelL : public ChronoModel {
  public:
    ChronoModelL();
    virtual std::string GetJointType() const override { return "LinkLock"; }

  private:
    virtual ChMatrix<>* GetViolation1() override { return m_prismatic->GetC(); }
    virtual ChMatrix<>* GetViolation2() override { return m_revolute->GetC(); }

    std::shared_ptr<ChLinkLockPrismatic> m_prismatic;
    std::shared_ptr<ChLinkLockRevolute> m_revolute;
};

// Chrono model using LinkMate joints.
class ChronoModelM : public ChronoModel {
  public:
    ChronoModelM();
    virtual std::string GetJointType() const override { return "LinkMate"; }

  private:
    virtual ChMatrix<>* GetViolation1() override { return m_prismatic->GetC(); }
    virtual ChMatrix<>* GetViolation2() override { return m_revolute->GetC(); }

    std::shared_ptr<ChLinkMateGeneric> m_prismatic;
    std::shared_ptr<ChLinkMateGeneric> m_revolute;
};

ChronoModel::ChronoModel() {
    // Size the data
    // -------------
    m_data.resize(3);                // time + x_pend + y_pend
    m_cnstr_data.resize(1 + 5 + 5);  // time + translational + revolute

    // Create the Chrono physical system
    // ---------------------------------
    m_system = std::make_shared<ChSystemNSC>();
    m_system->Set_G_acc(ChVector<>(0, -g, 0));

    // Create the ground body
    // ----------------------
    m_ground = std::make_shared<ChBody>();
    m_system->AddBody(m_ground);
    m_ground->SetIdentifier(-1);
    m_ground->SetBodyFixed(true);

    // Create the slider body
    // ----------------------
    m_slider = std::make_shared<ChBody>();
    m_system->AddBody(m_slider);
    m_slider->SetIdentifier(1);
    m_slider->SetMass(m1);
    m_slider->SetInertiaXX(ChVector<>(1, 1, 1));
    m_slider->SetPos(ChVector<>(0, 0, 0));

    // Create the pendulum body
    // ------------------------
    m_pend = std::make_shared<ChBody>();
    m_system->AddBody(m_pend);
    m_pend->SetIdentifier(2);
    m_pend->SetMass(m2);
    m_pend->SetInertiaXX(ChVector<>(1, 1, J2));
    m_pend->SetPos(ChVector<>(l2 / 2, 0, 0));

    // Spring ground-cart
    // ------------------
    auto spring = std::make_shared<ChLinkSpring>();
    spring->Initialize(m_slider, m_ground, false, ChVector<>(0, 0, 0), ChVector<>(0, 0, 0), true);
    spring->Set_SpringK(k1);
    spring->Set_SpringR(0);
    m_system->AddLink(spring);
}

ChronoModelL::ChronoModelL() {
    // Translational joint ground-cart
    // -------------------------------
    m_prismatic = std::make_shared<ChLinkLockPrismatic>();
    m_prismatic->Initialize(m_ground, m_slider, ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngY(CH_C_PI_2)));
    m_system->AddLink(m_prismatic);

    // Revolute joint cart-pendulum
    // ----------------------------
    m_revolute = std::make_shared<ChLinkLockRevolute>();
    m_revolute->Initialize(m_slider, m_pend, ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
    m_system->AddLink(m_revolute);
}

ChronoModelM::ChronoModelM() {
    // Translational joint ground-cart
    // -------------------------------
    m_prismatic = std::make_shared<ChLinkMateGeneric>(true, true, false, true, true, true);
    m_prismatic->Initialize(m_ground, m_slider, ChFrame<>(ChVector<>(0, 0, 0), Q_from_AngY(CH_C_PI_2)));
    m_system->AddLink(m_prismatic);

    // Revolute joint cart-pendulum
    // ----------------------------
    m_revolute = std::make_shared<ChLinkMateGeneric>(true, true, true, true, true, false);
    m_revolute->Initialize(m_slider, m_pend, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));
    m_system->AddLink(m_revolute);
}

void ChronoModel::Simulate(double step, int num_steps) {
    // Size the output data structures.
    for (size_t col = 0; col < 3; col++)
        m_data[col].resize(num_steps);
    for (size_t col = 0; col < 11; col++)
        m_cnstr_data[col].resize(num_steps);

    // Simulate the system for the specified number of steps.
    for (size_t it = 0; it < num_steps; it++) {
        // Save current state output information.
        m_data[0][it] = m_system->GetChTime();
        m_data[1][it] = m_pend->GetPos().x();
        m_data[2][it] = m_pend->GetPos().y();

        // Save current constraint violations.
        m_cnstr_data[0][it] = m_system->GetChTime();
        ChMatrix<>* C_p = GetViolation1();
        for (int col = 0; col < 5; col++)
            m_cnstr_data[1 + col][it] = C_p->GetElement(col, 0);
        ChMatrix<>* C_r = GetViolation2();
        for (int col = 0; col < 5; col++)
            m_cnstr_data[6 + col][it] = C_r->GetElement(col, 0);

        // Advance system state.
        m_system->DoStepDynamics(step);
    }
}

void ChronoModel::WriteData(double step, const std::string& filename) {
    assert(m_data.size() == 3);

    utils::CSV_writer csv(" ");

    for (size_t it = 0; it < m_data[0].size(); ++it) {
        csv << m_data[0][it] << m_data[1][it] << m_data[2][it] << std::endl;
    }

    csv.write_to_file(filename);
}

// =============================================================================

template <typename ChronoModelType>
bool test_EULER(double step, int num_steps, const utils::Data& ref_data, double tol_state, double tol_cnstr) {
    // Create Chrono model.
    ChronoModelType model;
    std::shared_ptr<ChSystemNSC> system = model.GetSystem();

    std::cout << "EULER_IMPLICIT_LINEARIZED integrator *** " << model.GetJointType() << std::endl;

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

template <typename ChronoModelType>
bool test_HHT(double step, int num_steps, const utils::Data& ref_data, double tol_state, double tol_cnstr) {
    // Create Chrono model.
    ChronoModelType model;
    std::shared_ptr<ChSystemNSC> system = model.GetSystem();

    std::cout << "HHT integrator *** " << model.GetJointType() << std::endl;

    // Set solver and modify parameters.
    ////system->SetMaxItersSolverSpeed(200);
    ////system->SetMaxItersSolverStab(200);
    ////system->SetTolForce(1e-5);

    ////system->SetSolverType(ChSolver::Type::MINRES);
    ////auto msolver = std::static_pointer_cast<ChSolverMINRES>(system->GetSolver());

    // Set integrator and modify parameters.
    system->SetTimestepperType(ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system->GetTimestepper());
    integrator->SetAlpha(-0.2);
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
    double step = 1e-3;
    int num_steps = 5000;

    double tol_state = 1e-3;
    double tol_cnstr = 1e-6;

    // Create and simulate the ODE model.
    ODEModel ode_model;
    ode_model.Simulate(step, num_steps);
    const utils::Data& ref_data = ode_model.GetData();
    ////ode_model.WriteData(step, "ode_swing.txt");

    // Perform the timestepper tests.
    bool passed = true;

    std::cout << "Validation tests for slider+pend system" << std::endl;
    std::cout << num_steps << " steps, using h = " << step << std::endl << std::endl;
    passed &= test_EULER<ChronoModelL>(step, num_steps, ref_data, tol_state, tol_cnstr);
    passed &= test_EULER<ChronoModelM>(step, num_steps, ref_data, tol_state, tol_cnstr);
    passed &= test_HHT<ChronoModelL>(step, num_steps, ref_data, tol_state, tol_cnstr);
    passed &= test_HHT<ChronoModelM>(step, num_steps, ref_data, tol_state, tol_cnstr);

    // Return 0 if all tests passed.
    return !passed;
}
