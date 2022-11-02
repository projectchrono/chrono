// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
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
// Rig mechanisms for drawbar-pull tests in the vehicle co-simulation framework.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLoadContainer.h"

#include "chrono_vehicle/cosim/ChVehicleCosimDBPRig.h"

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

// =============================================================================

// Utility custom function for ramping up to a prescribed value.
class ConstantFunction : public chrono::ChFunction {
  public:
    ConstantFunction(double max_value, double time_delay = 0, double time_ramp = 1)
        : m_max_value(max_value), m_time_delay(time_delay), m_time_ramp(time_ramp) {}

    virtual ConstantFunction* Clone() const override {
        return new ConstantFunction(m_time_delay, m_time_ramp, m_max_value);
    }

    virtual double Get_y(double t) const override {
        if (t <= m_time_delay)
            return 0;

        double t1 = t - m_time_delay;
        if (t1 <= m_time_ramp)
            return m_max_value * t1 / m_time_ramp;

        return m_max_value;
    }

    virtual double Get_y_dx(double t) const override {
        if (t > m_time_delay && t <= m_time_delay + m_time_ramp)
            return m_max_value / m_time_ramp;

        return 0;
    }

    virtual double Get_y_dxdx(double t) const override { return 0; }

  private:
    double m_time_delay;
    double m_time_ramp;
    double m_max_value;
};

// Utility custom function for ramping with prescribed rate.
class RampFunction : public chrono::ChFunction {
  public:
    RampFunction(double rate, double time_delay = 0) : m_rate(rate), m_time_delay(time_delay) {}
    virtual RampFunction* Clone() const override { return new RampFunction(m_rate, m_time_delay); }

    virtual double Get_y(double t) const override {
        if (t <= m_time_delay)
            return 0;
        return m_rate * (t - m_time_delay);
    }

    virtual double Get_y_dx(double t) const override {
        if (t > m_time_delay)
            return m_rate;
        return 0;
    }

    virtual double Get_y_dxdx(double t) const override { return 0; }

  private:
    double m_rate;
    double m_time_delay;
};

// =============================================================================

ChVehicleCosimDBPRig::ChVehicleCosimDBPRig()
    : m_dbp_filter_window(0.1),
      m_dbp_filtered(0),
      m_slip_filter_window(0.1),
      m_slip_filtered(0),
      m_verbose(false),
      m_delay_time(0) {}

void ChVehicleCosimDBPRig::Initialize(std::shared_ptr<ChBody> chassis,
                                      double wheel_radius,
                                      double step_size) {
    // Initialize filters
    int nw_dbp = static_cast<int>(std::round(m_dbp_filter_window / step_size));
    m_dbp_filter = chrono_types::make_unique<utils::ChRunningAverage>(nw_dbp);
    int nw_slip = static_cast<int>(std::round(m_slip_filter_window / step_size));
    m_slip_filter = chrono_types::make_unique<utils::ChRunningAverage>(nw_slip);

    InitializeRig(chassis, wheel_radius);
}

void ChVehicleCosimDBPRig::OnAdvance(double step_size) {
    m_dbp_filtered = m_dbp_filter->Add(GetDBP());
    m_slip_filtered = m_slip_filter->Add(GetSlip());
}

// =============================================================================

ChVehicleCosimDBPRigImposedSlip::ChVehicleCosimDBPRigImposedSlip(ActuationType act_type, double base_vel, double slip)
    : m_act_type(act_type),
      m_base_vel(base_vel),
      m_slip(slip),
      m_lin_vel(0),
      m_ang_vel(0),
      m_time_delay(0.2),
      m_time_ramp(0.5) {}

void ChVehicleCosimDBPRigImposedSlip::SetRampingIntervals(double delay, double ramp_time) {
    m_time_delay = delay;
    m_time_ramp = ramp_time;
}

std::string ChVehicleCosimDBPRigImposedSlip::GetActuationTypeAsString(ActuationType type) {
    switch (type) {
        case ActuationType::SET_LIN_VEL:
            return "SET_LIN_VEL";
        case ActuationType::SET_ANG_VEL:
            return "SET_ANG_VEL";
        default:
            return "UNKNOWN";
    }
}

ChVehicleCosimDBPRigImposedSlip::ActuationType ChVehicleCosimDBPRigImposedSlip::GetActuationTypeFromString(
    const std::string& type) {
    if (type == "SET_LIN_VEL")
        return ActuationType::SET_LIN_VEL;
    if (type == "SET_ANG_VEL")
        return ActuationType::SET_ANG_VEL;

    return ActuationType::UNKNOWN;
}

void ChVehicleCosimDBPRigImposedSlip::InitializeRig(std::shared_ptr<ChBody> chassis, double wheel_radius) {
    // Calculate rig linear velocity and wheel angular velocity
    switch (m_act_type) {
        case ActuationType::SET_ANG_VEL:
            m_ang_vel = m_base_vel;
            m_lin_vel = (m_ang_vel * wheel_radius) * (1.0 - m_slip);
            break;
        case ActuationType::SET_LIN_VEL:
            m_lin_vel = m_base_vel;
            m_ang_vel = m_lin_vel / (wheel_radius * (1.0 - m_slip));
            break;
        default:
            m_ang_vel = 0;
            m_lin_vel = 0;
            break;
    }

    // Motor functions
    m_lin_motor_func = chrono_types::make_shared<ConstantFunction>(m_lin_vel, m_time_delay, m_time_ramp);
    m_rot_motor_func = chrono_types::make_shared<ConstantFunction>(m_ang_vel, m_time_delay, m_time_ramp);

    if (m_verbose) {
        cout << "[DBP rig     ] actuation = " << GetActuationTypeAsString(m_act_type) << endl;
        cout << "[DBP rig     ] base velocity = " << m_base_vel << endl;
        cout << "[DBP rig     ] slip = " << m_slip << endl;
        cout << "[DBP rig     ] rig linear velocity = " << m_lin_vel << endl;
        cout << "[DBP rig     ] wheel angular velocity = " << m_ang_vel << endl;
        cout << "[DBP rig     ] wheel radius = " << wheel_radius << endl;
    }

    // Create a "ground" body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetBodyFixed(true);
    chassis->GetSystem()->AddBody(ground);

    // Create a "carrier" body.
    // Since the carrier will be connected to ground with a horizontal prismatic joint, its mass does not contribute to
    // vertical load.
    auto carrier = chrono_types::make_shared<ChBody>();
    carrier->SetMass(10);
    carrier->SetInertiaXX(ChVector<>(1, 1, 1));
    carrier->SetPos(chassis->GetPos());
    carrier->SetRot(QUNIT);
    carrier->SetPos_dt(ChVector<>(m_lin_vel, 0, 0));
    chassis->GetSystem()->AddBody(carrier);

    // Connect chassis body to connector using a vertical prismatic joint
    auto prism_vert = chrono_types::make_shared<ChLinkLockPrismatic>();
    prism_vert->Initialize(carrier, chassis, ChCoordsys<>(carrier->GetPos(), QUNIT));
    chassis->GetSystem()->AddLink(prism_vert);

    // Connect carrier to ground with a linear motor
    m_lin_motor = chrono_types::make_shared<ChLinkMotorLinearSpeed>();
    m_lin_motor->SetSpeedFunction(m_lin_motor_func);
    m_lin_motor->Initialize(carrier, ground, ChFrame<>(carrier->GetPos(), QUNIT));
    chassis->GetSystem()->AddLink(m_lin_motor);

    // Set initialization (ramping up) time
    m_delay_time = 0.2 + 0.5;
}

std::shared_ptr<ChFunction> ChVehicleCosimDBPRigImposedSlip::GetMotorFunction() const {
    return m_rot_motor_func;
}

double ChVehicleCosimDBPRigImposedSlip::GetDBP() const {
    return -m_lin_motor->GetMotorForce();
}

// =============================================================================

ChVehicleCosimDBPRigImposedAngVel::ChVehicleCosimDBPRigImposedAngVel(double ang_vel, double force_rate)
    : m_ang_vel(ang_vel), m_tire_radius(0), m_force_rate(force_rate), m_time_delay(0.2), m_time_ramp(0.5) {}

void ChVehicleCosimDBPRigImposedAngVel::SetRampingIntervals(double delay, double ramp_time) {
    m_time_delay = delay;
    m_time_ramp = ramp_time;
}

void ChVehicleCosimDBPRigImposedAngVel::InitializeRig(std::shared_ptr<ChBody> chassis, double wheel_radius) {
    m_rot_motor_func = chrono_types::make_shared<ConstantFunction>(m_ang_vel, m_time_delay, m_time_ramp);

    // Create a "ground" body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetBodyFixed(true);
    chassis->GetSystem()->AddBody(ground);

    // Create a "carrier" body.
    // Since the carrier will be connected to ground with a horizontal prismatic joint, its mass does not contribute to
    // vertical load.
    m_carrier = chrono_types::make_shared<ChBody>();
    m_carrier->SetMass(10);
    m_carrier->SetInertiaXX(ChVector<>(1, 1, 1));
    m_carrier->SetPos(chassis->GetPos());
    m_carrier->SetRot(QUNIT);
    chassis->GetSystem()->AddBody(m_carrier);

    // Connect chassis body to connector using a vertical prismatic joint
    auto prism_vert = chrono_types::make_shared<ChLinkLockPrismatic>();
    prism_vert->Initialize(m_carrier, chassis, ChCoordsys<>(m_carrier->GetPos(), QUNIT));
    chassis->GetSystem()->AddLink(prism_vert);

    // Connect carrier to ground with a horizonal prismatic joint
    auto prism_horiz = chrono_types::make_shared<ChLinkLockPrismatic>();
    prism_horiz->Initialize(m_carrier, ground, ChCoordsys<>(m_carrier->GetPos(), Q_from_AngY(CH_C_PI_2)));
    chassis->GetSystem()->AddLink(prism_horiz);

    // Apply a resistive force to carrier body
    auto ramp = chrono_types::make_shared<RampFunction>(m_force_rate, 0.2 + 0.5);
    m_DBP_force = chrono_types::make_shared<ChLoadBodyForce>(m_carrier, ChVector<>(-1, 0, 0), true, VNULL, true);
    m_DBP_force->SetModulationFunction(ramp);

    auto load_container = chrono_types::make_shared<ChLoadContainer>();
    load_container->Add(m_DBP_force);
    chassis->GetSystem()->Add(load_container);

    // Set initialization (ramping up) time
    m_delay_time = 0.2 + 0.5;
}

std::shared_ptr<ChFunction> ChVehicleCosimDBPRigImposedAngVel::GetMotorFunction() const {
    return m_rot_motor_func;
}

double ChVehicleCosimDBPRigImposedAngVel::GetSlip() const {
    double lin_vel = m_carrier->GetPos_dt().x();
    double slip = 1 - lin_vel / (m_ang_vel * m_tire_radius);
    return slip;
}

double ChVehicleCosimDBPRigImposedAngVel::GetLinVel() const {
    return m_carrier->GetPos_dt().x();
}

double ChVehicleCosimDBPRigImposedAngVel::GetDBP() const {
    return -m_DBP_force->GetForce().x();
}

}  // namespace vehicle
}  // namespace chrono
