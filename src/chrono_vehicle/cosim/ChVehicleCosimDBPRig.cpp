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
#include "chrono/motion_functions/ChFunction_Const.h"

#include "chrono_vehicle/cosim/ChVehicleCosimDBPRig.h"

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

// =============================================================================

// Utility custom function for ramping up to a prescribed value.
class RampFunction : public chrono::ChFunction {
  public:
    RampFunction(double max_value, double time_delay = 0, double time_ramp = 1)
        : m_max_value(max_value), m_time_delay(time_delay), m_time_ramp(time_ramp) {}

    virtual RampFunction* Clone() const override { return new RampFunction(m_time_delay, m_time_ramp, m_max_value); }

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

// =============================================================================

ChVehicleCosimDBPRig::ChVehicleCosimDBPRig() : m_filter_window(0.1), m_dbp_filtered(0), m_verbose(false) {}

void ChVehicleCosimDBPRig::Initialize(std::shared_ptr<ChBody> chassis,
                                      const std::vector<ChVector<>>& tire_info,
                                      double step_size) {
    // Initialize DBP filter
    int nw = static_cast<int>(std::round(m_filter_window / step_size));
    m_filter = chrono_types::make_unique<utils::ChRunningAverage>(nw);

    InitializeRig(chassis, tire_info);
}

void ChVehicleCosimDBPRig::OnAdvance(double step_size) {
    m_dbp_filtered = m_filter->Add(GetDBP());
}

// =============================================================================

ChVehicleCosimDBPRigImposedSlip::ChVehicleCosimDBPRigImposedSlip(ActuationType act_type, double base_vel, double slip)
    : m_act_type(act_type), m_base_vel(base_vel), m_slip(slip), m_lin_vel(0), m_ang_vel(0) {}

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

void ChVehicleCosimDBPRigImposedSlip::InitializeRig(std::shared_ptr<ChBody> chassis,
                                                    const std::vector<ChVector<>>& tire_info) {
    double tire_radius = tire_info[0].y();

    // Calculate rig linear velocity and wheel angular velocity
    switch (m_act_type) {
        case ActuationType::SET_ANG_VEL:
            m_ang_vel = m_base_vel;
            m_lin_vel = (m_ang_vel * tire_radius) * (1.0 - m_slip);
            break;
        case ActuationType::SET_LIN_VEL:
            m_lin_vel = m_base_vel;
            m_ang_vel = m_lin_vel / (tire_radius * (1.0 - m_slip));
            break;
        default:
            m_ang_vel = 0;
            m_lin_vel = 0;
            break;
    }

    // Motor functions
    m_lin_motor_func = chrono_types::make_shared<RampFunction>(m_lin_vel, 0.2, 0.5);
    m_rot_motor_func = chrono_types::make_shared<RampFunction>(m_ang_vel, 0.2, 0.5);

    if (m_verbose) {
        cout << "[DBP rig     ] actuation                " << GetActuationTypeAsString(m_act_type) << endl;
        cout << "[DBP rig     ] base velocity          = " << m_base_vel << endl;
        cout << "[DBP rig     ] slip                   = " << m_slip << endl;
        cout << "[DBP rig     ] rig linear velocity    = " << m_lin_vel << endl;
        cout << "[DBP rig     ] wheel angular velocity = " << m_ang_vel << endl;
        cout << "[DBP rig     ] tire radius            = " << tire_radius << endl;
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
    m_lin_motor->Initialize(carrier, ground, ChFrame<>(chassis->GetPos(), QUNIT));
    chassis->GetSystem()->AddLink(m_lin_motor);
}

std::shared_ptr<ChFunction> ChVehicleCosimDBPRigImposedSlip::GetMotorFunction() const {
    return m_rot_motor_func;
}

/// Return current raw drawbar-pull value.
double ChVehicleCosimDBPRigImposedSlip::GetDBP() const {
    return m_lin_motor->GetMotorForce();
}

}  // namespace vehicle
}  // namespace chrono
