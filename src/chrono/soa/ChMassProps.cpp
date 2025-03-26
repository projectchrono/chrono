// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include "chrono/soa/ChMassProps.h"

namespace chrono {
namespace soa {

ChMassProps::ChMassProps() : m_mass(1), m_inertia_B(0.0) {}

ChMassProps::ChMassProps(double mass, const ChFramed& X_BC, const ChMatrix33<>& inertia) : m_mass(mass), m_X_BC(X_BC) {
    const auto& R_BC = X_BC.GetRotMat();
    m_inertia_B = R_BC.transpose() * inertia * R_BC;
    calcInverse();
}

ChMassProps::ChMassProps(double mass, const ChVector3d& com, const ChMatrix33<>& inertia)
    : ChMassProps(mass, ChFramed(com, QUNIT), inertia) {}

ChSpatialMat ChMassProps::asSpatialMat() const {
    ChSpatialMat result;

    auto p_x = ChStarMatrix33<>(m_X_BC.GetPos());

    result.A00() = m_inertia_B;
    result.A01() = m_mass * p_x;
    result.A10() = -m_mass * p_x;
    result.A11() = ChMatrix33<>(m_mass);

    return result;
}

void ChMassProps::calcInverse() {
    m_ooMass = 1 / m_mass;

    auto p_x = ChStarMatrix33<>(m_X_BC.GetPos());

    m_invM_B.A00() = m_inertia_B;
    m_invM_B.A00() = m_invM_B.A00().inverse().eval();

    m_invM_B.A10() = p_x * m_invM_B.A00();
    m_invM_B.A01() = -m_invM_B.A00() * p_x;

    m_invM_B.A11() = ChMatrix33<>(m_ooMass) - m_invM_B.A10() * p_x;
}

ChMatrix33<> ChMassProps::calcPointInertia(const ChVector3d& pos, double mass) {
    ChMatrix33<> inertia;

    double mx = mass * pos.x();
    double my = mass * pos.y();
    double mz = mass * pos.z();

    double mxx = mx * pos.x();
    double myy = my * pos.y();
    double mzz = mz * pos.z();

    inertia(0, 0) = myy + mzz;
    inertia(0, 1) = -mx * pos.y();
    inertia(0, 2) = -mx * pos.z();

    inertia(1, 0) = -mx * pos.y();
    inertia(1, 1) = mxx + mzz;
    inertia(1, 2) = -my * pos.z();

    inertia(2, 0) = -mx * pos.z();
    inertia(2, 1) = -my * pos.z();
    inertia(2, 2) = mxx + myy;

    return inertia;
}

}  // namespace soa
}  // namespace chrono
