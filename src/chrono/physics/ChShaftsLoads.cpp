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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/physics/ChShaftsLoads.h"

namespace chrono {

// -----------------------------------------------------------------------------
// ChShaftsLoad
// -----------------------------------------------------------------------------

ChShaftsLoad::ChShaftsLoad(std::shared_ptr<ChShaft> shaft1, std::shared_ptr<ChShaft> shaft2)
    : ChLoadCustomMultiple(shaft1, shaft2) {
    this->torque = 0;
}

void ChShaftsLoad::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto shaft1 = std::dynamic_pointer_cast<ChShaft>(this->loadables[0]);
    auto shaft2 = std::dynamic_pointer_cast<ChShaft>(this->loadables[1]);

    double rot1, rot2;
    if (state_x) {
        // the numerical jacobian algo might change state_x
        rot1 = (*state_x)(0);
        rot2 = (*state_x)(1);
    } else {
        rot1 = shaft1->GetPos();
        rot2 = shaft2->GetPos();
    }

    double rot1_dt, rot2_dt;
    if (state_w) {
        // the numerical jacobian algo might change state_w
        rot1_dt = (*state_w)(0);
        rot2_dt = (*state_w)(1);
    } else {
        rot1_dt = shaft1->GetPosDt();
        rot2_dt = shaft2->GetPosDt();
    }

    double rel_rot = rot1 - rot2;
    double rel_rot_dt = rot1_dt - rot2_dt;

    // Compute torque
    ComputeShaftShaftTorque(rel_rot, rel_rot_dt, this->torque);

    // Compute Q
    this->load_Q(0) = this->torque;
    this->load_Q(1) = -this->torque;
}

std::shared_ptr<ChShaft> ChShaftsLoad::GetShaft1() const {
    return std::dynamic_pointer_cast<ChShaft>(this->loadables[0]);
}

std::shared_ptr<ChShaft> ChShaftsLoad::GetShaft2() const {
    return std::dynamic_pointer_cast<ChShaft>(this->loadables[1]);
}

// -----------------------------------------------------------------------------
// ChShaftsTorsionSpringDamper
// -----------------------------------------------------------------------------

chrono::ChShaftsTorsionSpringDamper::ChShaftsTorsionSpringDamper(std::shared_ptr<ChShaft> shaft1,
                                                                 std::shared_ptr<ChShaft> shaft2,
                                                                 const double stiffness,
                                                                 const double damping)
    : ChShaftsLoad(shaft1, shaft2), m_stiffness(stiffness), m_damping(damping), m_rest_phase(0.0) {}

void ChShaftsTorsionSpringDamper::ComputeShaftShaftTorque(const double rel_rot,
                                                          const double rel_rot_dt,
                                                          double& result_torque) {
    result_torque = -(rel_rot - m_rest_phase) * m_stiffness - rel_rot_dt * m_damping;
}

// -----------------------------------------------------------------------------
// ChShaftsElasticGear
// -----------------------------------------------------------------------------

chrono::ChShaftsElasticGear::ChShaftsElasticGear(
    std::shared_ptr<ChShaft> shaft1,  // first shaft
    std::shared_ptr<ChShaft> shaft2,  // second shaft
    const double stiffness,           // normal stiffness at teeth contact, tangent direction to primitive
    const double damping,             // normal damping at teeth contact, tangent direction to primitive
    const double Ra,                  // primitive radius of the gear on shaft A (the radius of B is not needed)
    const double ratio)
    : ChLoadCustomMultiple(shaft1, shaft2),
      m_stiffness(stiffness),
      m_damping(damping),
      m_rest_phase(0.0),
      m_Ra(Ra),
      m_ratio(ratio) {
    m_contact_force = 0.0;
}

void ChShaftsElasticGear::SetTransmissionRatioAndRadiusA(double ratio, double Ra) {
    m_ratio = ratio;
    m_Ra = Ra;
}

void ChShaftsElasticGear::SetTransmissionRatioFromRadii(double Ra, double Rb, bool internal) {
    m_Ra = Ra;
    m_ratio = fabs(Ra / Rb);
    if (internal)
        m_ratio = -m_ratio;
}

void ChShaftsElasticGear::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto shaft1 = std::dynamic_pointer_cast<ChShaft>(this->loadables[0]);
    auto shaft2 = std::dynamic_pointer_cast<ChShaft>(this->loadables[1]);

    double rot1, rot2;
    if (state_x) {
        // the numerical jacobian algo might change state_x
        rot1 = (*state_x)(0);
        rot2 = (*state_x)(1);
    } else {
        rot1 = shaft1->GetPos();
        rot2 = shaft2->GetPos();
    }

    double rot1_dt, rot2_dt;
    if (state_w) {
        // the numerical jacobian algo might change state_w
        rot1_dt = (*state_w)(0);
        rot2_dt = (*state_w)(1);
    } else {
        rot1_dt = shaft1->GetPosDt();
        rot2_dt = shaft2->GetPosDt();
    }
    double invratio = 1.0 / m_ratio;

    double rel_compression = m_Ra * (rot1 - m_rest_phase) - invratio * m_Ra * rot2;
    double rel_compression_dt = m_Ra * rot1_dt - invratio * m_Ra * rot2_dt;

    // Compute contact force
    m_contact_force = -rel_compression * m_stiffness - rel_compression_dt * m_damping;

    // Compute resulting torques on the two shafts and store them in Q

    this->load_Q(0) = m_Ra * m_contact_force;
    this->load_Q(1) = -invratio * m_Ra * m_contact_force;
}

}  // end namespace chrono