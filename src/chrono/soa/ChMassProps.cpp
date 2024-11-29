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

ChMassProps::ChMassProps() : m_mass(1), m_com_B(0.0f), m_inertiaOB_B(1) {}

ChMassProps::ChMassProps(double mass) : m_mass(mass), m_com_B(0.0f), m_inertiaOB_B(1) {
    calcInverse();
}

ChMassProps::ChMassProps(double mass, const ChVector3d& com, const ChMatrix33<>& inertia)
    : m_mass(mass), m_com_B(com), m_inertiaOB_B(inertia) {
    calcInverse();
}

ChSpatialMat ChMassProps::asSpatialMat() const {
    ChSpatialMat result;

    auto p_x = ChStarMatrix33<>(m_com_B);

    result.A00() = m_inertiaOB_B;
    result.A01() = m_mass * p_x;
    result.A10() = -m_mass * p_x;
    result.A11() = ChMatrix33<>(m_mass);

    return result;
}

void ChMassProps::calcInverse() {
    m_ooMass = 1 / m_mass;

    auto p_x = ChStarMatrix33<>(m_com_B);

    m_invM_B.A00() = calcCentralInertia();
    m_invM_B.A00() = m_invM_B.A00().inverse();

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

ChMatrix33<> ChMassProps::calcCentralInertia() const {
    return m_inertiaOB_B - calcPointInertia(m_com_B, m_mass);
}

ChMatrix33<> ChMassProps::calcShiftedInertia(const ChVector3d& newOriginB) const {
    return calcCentralInertia() + calcPointInertia(newOriginB - m_com_B, m_mass);
}

ChMatrix33<> ChMassProps::calcTransformedInertia(const ChFramed& X_BC) const {
    const auto& R_BC = X_BC.GetRotMat();
    auto inertia_shift = calcShiftedInertia(X_BC.GetPos());
    return R_BC.transpose() * inertia_shift * R_BC;
}

ChMassProps ChMassProps::calcShiftedMassProps(const ChVector3d& newOriginB) const {
    return ChMassProps(m_mass, m_com_B - newOriginB, calcShiftedInertia(newOriginB));
}

ChMassProps ChMassProps::calcTransformedMassProps(const ChFramed& X_BC) const {
    return ChMassProps(m_mass, X_BC.TransformPointParentToLocal(m_com_B), calcTransformedInertia(X_BC));
}

ChMassProps ChMassProps::reexpress(const ChMatrix33<>& R_BC) const {
    auto inertiaOB_C = R_BC.transpose() * m_inertiaOB_B * R_BC;
    return ChMassProps(m_mass, R_BC.transpose() * m_com_B, inertiaOB_C);
}

}  // namespace soa
}  // namespace chrono
