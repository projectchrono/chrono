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

#include "chrono/soa/ChSoaAssembly.h"
#include "chrono/soa/ChSoaRevoluteBody.h"

namespace chrono {
namespace soa {

ChSoaRevoluteBody::ChSoaRevoluteBody(std::shared_ptr<ChSoaMobilizedBody> parent,
                               const ChSoaMassProperties& mprops,
                               const ChFramed& inbFrame,
                               const ChFramed& outbFrame,
                               const std::string& name)
    : ChSoaMobilizedBodyT<1>(parent, mprops, inbFrame, outbFrame, name), m_q0(0), m_u0(0) {
    m_H_FM.ang().col(0) = ChVector3d(0, 0, 1).eigen();
    m_H_FM.lin().col(0) = ChVector3d(0, 0, 0).eigen();

    m_H_FM_dot.ang().setZero();
    m_H_FM_dot.lin().setZero();
}

ChSoaRevoluteBody::ChSoaRevoluteBody(const ChSoaRevoluteBody& other) : ChSoaMobilizedBodyT<1>(other) {
  //// TODO
}

// Mobilizer-specific setters for generalized coordinates, velocities, and acceleration

void ChSoaRevoluteBody::setRelPos(double rotAngle) {
    if (m_assembly && m_assembly->IsInitialized())
        setQ(0, std::fmod(rotAngle, CH_2PI));
    else
        m_q0 = std::fmod(rotAngle, CH_2PI);
}

void ChSoaRevoluteBody::setRelVel(double rotRate) {
    if (m_assembly && m_assembly->IsInitialized())
        setU(0, rotRate);
    else
        m_u0 = rotRate;
}

void ChSoaRevoluteBody::setRelAcc(double rotAcc) {
    if (m_assembly && m_assembly->IsInitialized())
        setUdot(0, rotAcc);
}

// Virtual overrides for setters for generalized coordinates, velocities, and acceleration

void ChSoaRevoluteBody::setRelRot(const ChMatrix33d& relRot) {
    float a = relRot(0, 0) + relRot(1, 1);
    float b = relRot(0, 1) - relRot(1, 0);
    if (m_assembly && m_assembly->IsInitialized())
        setQ(0, atan2f(b, a));
    else
        m_q0 = atan2f(b, a);
}

void ChSoaRevoluteBody::setRelAngVel(const ChVector3d& relAngVel) {
    if (m_assembly && m_assembly->IsInitialized())
        setU(0, relAngVel.z());
    else
        m_u0 = relAngVel.z();
}

void ChSoaRevoluteBody::setRelAngAcc(const ChVector3d& relAngAcc) {
    if (m_assembly && m_assembly->IsInitialized())
        setUdot(0, relAngAcc.z());
}

// Mobilizer-specific getters for state and derivatives

double ChSoaRevoluteBody::getQ0(int dof) const {
    assert(dof == 0);
    return m_q0;
}

double ChSoaRevoluteBody::getU0(int dof) const {
    assert(dof == 0);
    return m_u0;
}

// ---

ChMatrix33d ChSoaRevoluteBody::calcRelRot(double q) {
    return ChMatrix33d(q, VECT_Z);
}

void ChSoaRevoluteBody::setJointTransform(const ChVectorDynamic<>& y) {
    m_X_FM.SetRot(ChMatrix33d(y(m_qIdx), VECT_Z));
}

void ChSoaRevoluteBody::prepSim() {
//// NEEDED?!?
}

}  // namespace soa
}  // namespace chrono
