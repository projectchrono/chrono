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
// =============================================================================

#include "chrono/soa/ChSoaForce.h"

namespace chrono {
namespace soa {

ChSoaConstantForce::ChSoaConstantForce(std::shared_ptr<ChSoaMobilizedBody> body,
                                       const ChVector3d& location,
                                       const ChVector3d& force)
    : ChSoaBodyForce(body), m_location(location), m_force(force) {}

void ChSoaConstantForce::apply() {
    const auto& X_GB = m_body->getAbsPos();
    ChVector3 loc_G = X_GB.GetRotMat() * m_location;

    m_body->ApplyBodyForce(ChSpatialVec(loc_G % m_force, m_force));
}

ChSoaConstantTorque::ChSoaConstantTorque(std::shared_ptr<ChSoaMobilizedBody> body, const ChVector3d& torque)
    : ChSoaBodyForce(body), m_torque(torque) {}

void ChSoaConstantTorque::apply() {
    m_body->ApplyBodyForce(ChSpatialVec(m_torque, VNULL));
}

ChSoaSpringDamperForce::ChSoaSpringDamperForce(std::shared_ptr<ChSoaMobilizedBody> body1,
                                               std::shared_ptr<ChSoaMobilizedBody> body2,
                                               const ChVector3d& loc1,
                                               const ChVector3d& loc2,
                                               double l0,
                                               double k,
                                               double c)
    : m_body1(body1), m_body2(body2), m_loc1(loc1), m_loc2(loc2), m_l0(l0), m_k(k), m_c(c), m_l(l0) {}

void ChSoaSpringDamperForce::apply() {
    // Calculate the relative location and velocity, expressed in global frame.
    // Express the local vectors for the two body points in the global frame.
    ChVector3d s1_G;
    ChVector3d s2_G;
    ChVector3d relLoc;
    ChVector3d relVel;

    if (m_body2->isGround()) {
        s2_G = m_loc2;
        relLoc = s2_G;
        relVel = VNULL;
    } else {
        const auto& X_GB = m_body2->getAbsPos();
        const auto& V_GB = m_body2->getAbsVel();

        s2_G = X_GB.GetRotMat() * m_loc2;
        relLoc = X_GB.GetPos() + s2_G;
        relVel = V_GB.lin() + V_GB.ang() % s2_G;
    }

    if (m_body1->isGround()) {
        s1_G = m_loc1;
        relLoc -= s1_G;
    } else {
        const auto& X_GB = m_body1->getAbsPos();
        const auto& V_GB = m_body1->getAbsVel();

        s1_G = X_GB.GetRotMat() * m_loc1;
        relLoc -= X_GB.GetPos() + s1_G;
        relVel -= V_GB.lin() + V_GB.ang() % s1_G;
    }

    // Calculate the distance between the two points, then normalize the
    // relative location vector to a unit vector along that line.
    m_l = relLoc.Length();
    relLoc /= m_l;

    // Calculate the total force acting along the line defined by 'relLoc' as
    // the sum of the elastic and damping components. This force is calculated
    // for body1. Apply spatial forces on the two bodies, reversing direction
    // for the second one.
    ChVector3d frc_G = m_k * (m_l - m_l0) * relLoc + m_c * (relVel * relLoc) * relLoc;

    if (!m_body1->isGround())
        m_body1->ApplyBodyForce(ChSpatialVec(s1_G % frc_G, frc_G));

    if (!m_body2->isGround())
        m_body2->ApplyBodyForce(ChSpatialVec(frc_G % s2_G, -frc_G));
}

}  // namespace soa
}  // namespace chrono
