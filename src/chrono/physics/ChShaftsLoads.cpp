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

ChShaftsLoad::ChShaftsLoad(std::shared_ptr<ChShaft> shaftA, std::shared_ptr<ChShaft> shaftB)
    : ChLoadCustomMultiple(shaftA, shaftB) {
    this->torque = 0;
}

void ChShaftsLoad::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto mshaftA = std::dynamic_pointer_cast<ChShaft>(this->loadables[0]);
    auto mshaftB = std::dynamic_pointer_cast<ChShaft>(this->loadables[1]);

    double mrotA, mrotB;
    if (state_x) {
        // the numerical jacobian algo might change state_x
        mrotA = (*state_x)(0);
        mrotB = (*state_x)(1);
    } else {
        mrotA = mshaftA->GetPos();
        mrotB = mshaftB->GetPos();
    }

    double mrotA_dt, mrotB_dt;
    if (state_w) {
        // the numerical jacobian algo might change state_w
        mrotA_dt = (*state_w)(0);
        mrotB_dt = (*state_w)(1);
    } else {
        mrotA_dt = mshaftA->GetPosDer();
        mrotB_dt = mshaftB->GetPosDer();
    }

    double rel_rot = mrotA - mrotB;
    double rel_rot_dt = mrotA_dt - mrotB_dt;

    // COMPUTE THE TORQUE

    ComputeShaftShaftTorque(rel_rot, rel_rot_dt, this->torque);

    // Compute Q

    this->load_Q(0) = this->torque;
    this->load_Q(1) = -this->torque;
}

std::shared_ptr<ChShaft> ChShaftsLoad::GetShaftA() const {
    return std::dynamic_pointer_cast<ChShaft>(this->loadables[0]);
}

std::shared_ptr<ChShaft> ChShaftsLoad::GetShaftB() const {
    return std::dynamic_pointer_cast<ChShaft>(this->loadables[1]);
}

// -----------------------------------------------------------------------------
// ChShaftsTorsionSpringDamper
// -----------------------------------------------------------------------------

chrono::ChShaftsTorsionSpringDamper::ChShaftsTorsionSpringDamper(std::shared_ptr<ChShaft> shaftA,
                                                                 std::shared_ptr<ChShaft> shaftB,
                                                                 const double stiffness,
                                                                 const double damping)
    : ChShaftsLoad(shaftA, shaftB), m_stiffness(stiffness), m_damping(damping), m_rest_phase(0.0) {}

void ChShaftsTorsionSpringDamper::ComputeShaftShaftTorque(const double rel_rot,
                                                          const double rel_rot_dt,
                                                          double& result_torque) {
    result_torque = -(rel_rot - m_rest_phase) * m_stiffness - rel_rot_dt * m_damping;
}

// -----------------------------------------------------------------------------
// ChShaftsElasticGear
// -----------------------------------------------------------------------------

chrono::ChShaftsElasticGear::ChShaftsElasticGear(
    std::shared_ptr<ChShaft> shaftA,  // shaft A
    std::shared_ptr<ChShaft> shaftB,  // shaft B
    const double stiffness,           // normal stiffness at teeth contact, tangent direction to primitive
    const double damping,             // normal damping at teeth contact, tangent direction to primitive
    const double Ra,                  // primitive radius of the gear on shaft A (the radius of B is not needed)
    const double ratio)
    : ChLoadCustomMultiple(shaftA, shaftB),
      m_stiffness(stiffness),
      m_damping(damping),
      m_rest_phase(0.0),
      m_Ra(Ra),
      m_ratio(ratio) {
    m_contact_force = 0.0;
}

void ChShaftsElasticGear::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto shaftA = std::dynamic_pointer_cast<ChShaft>(this->loadables[0]);
    auto shaftB = std::dynamic_pointer_cast<ChShaft>(this->loadables[1]);

    double rotA, rotB;
    if (state_x) {
        // the numerical jacobian algo might change state_x
        rotA = (*state_x)(0);
        rotB = (*state_x)(1);
    } else {
        rotA = shaftA->GetPos();
        rotB = shaftB->GetPos();
    }

    double rotA_dt, rotB_dt;
    if (state_w) {
        // the numerical jacobian algo might change state_w
        rotA_dt = (*state_w)(0);
        rotB_dt = (*state_w)(1);
    } else {
        rotA_dt = shaftA->GetPosDer();
        rotB_dt = shaftB->GetPosDer();
    }
    double invratio = 1.0 / m_ratio;

    double rel_compression = m_Ra * (rotA - m_rest_phase) - invratio * m_Ra * rotB;
    double rel_compression_dt = m_Ra * rotA_dt - invratio * m_Ra * rotB_dt;

    // Compute contact force
    m_contact_force = -rel_compression * m_stiffness - rel_compression_dt * m_damping;

    // Compute resulting torques on the two shafts and store them in Q

    this->load_Q(0) = m_Ra * m_contact_force;
    this->load_Q(1) = -invratio * m_Ra * m_contact_force;
}

}  // end namespace chrono