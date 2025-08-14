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
#include "chrono/soa/ChSoaSphericalBody.h"

namespace chrono {
namespace soa {

ChSoaSphericalBody::ChSoaSphericalBody(std::shared_ptr<ChSoaMobilizedBody> parent,
                                       const ChSoaMassProperties& mprops,
                                       const ChFramed& inbFrame,
                                       const ChFramed& outbFrame,
                                       const std::string& name)
    : ChSoaMobilizedBodyT<3>(parent, mprops, inbFrame, outbFrame, name), m_q0(QUNIT), m_u0(VNULL) {
    m_H_FM.ang().col(0) = ChVector3d(1, 0, 0).eigen();
    m_H_FM.ang().col(1) = ChVector3d(0, 1, 0).eigen();
    m_H_FM.ang().col(2) = ChVector3d(0, 0, 1).eigen();

    m_H_FM.lin().col(0) = ChVector3d(0, 0, 0).eigen();
    m_H_FM.lin().col(1) = ChVector3d(0, 0, 0).eigen();
    m_H_FM.lin().col(2) = ChVector3d(0, 0, 0).eigen();

    m_H_FM_dot.ang().setZero();
    m_H_FM_dot.lin().setZero();
}

ChSoaSphericalBody::ChSoaSphericalBody(const ChSoaSphericalBody& other) : ChSoaMobilizedBodyT<3>(other) {
    //// TODO
}

// Mobilizer-specific setters for generalized coordinates, velocities, and acceleration

void ChSoaSphericalBody::setRelPos(const ChMatrix33d& rotMat) {
    //// TODO: setQ as quaternion in single shot
    auto quat = rotMat.GetQuaternion();
    if (m_assembly && m_assembly->IsInitialized()) {
        setQ(0, quat.e0());
        setQ(1, quat.e0());
        setQ(2, quat.e0());
        setQ(3, quat.e0());
    } else {
        m_q0 = quat;
    }
}

void ChSoaSphericalBody::setRelVel(const ChVector3d& angVel) {
    //// TODO: setU as vector in single shot
    if (m_assembly && m_assembly->IsInitialized()) {
        setU(0, angVel.x());
        setU(1, angVel.y());
        setU(2, angVel.z());
    } else {
        m_u0 = angVel;
    }
}

void ChSoaSphericalBody::setRelAcc(const ChVector3d& angAcc) {
    if (m_assembly && m_assembly->IsInitialized()) {
        setUdot(0, angAcc.x());
        setUdot(1, angAcc.y());
        setUdot(2, angAcc.z());
    }
}

// Virtual overrides for setters for generalized coordinates, velocities, and acceleration

void ChSoaSphericalBody::setRelRot(const ChMatrix33d& relRot) {
    auto quat = relRot.GetQuaternion();
    if (m_assembly && m_assembly->IsInitialized()) {
        setQ(0, quat.e0());
        setQ(1, quat.e0());
        setQ(2, quat.e0());
        setQ(3, quat.e0());
    } else {
        m_q0 = quat;
    }
}

void ChSoaSphericalBody::setRelAngVel(const ChVector3d& relAngVel) {
    if (m_assembly && m_assembly->IsInitialized()) {
        setU(0, relAngVel.x());
        setU(1, relAngVel.y());
        setU(2, relAngVel.z());
    } else {
        m_u0 = relAngVel;
    }
}

void ChSoaSphericalBody::setRelAngAcc(const ChVector3d& relAngAcc) {
    if (m_assembly && m_assembly->IsInitialized()) {
        setUdot(0, relAngAcc.x());
        setUdot(1, relAngAcc.y());
        setUdot(2, relAngAcc.z());
    }
}

// Mobilizer-specific getters for state and derivatives

double ChSoaSphericalBody::getQ0(int dof) const {
    return m_q0[dof];
}

double ChSoaSphericalBody::getU0(int dof) const {
    return m_u0[dof];
}

// ---

ChMatrix33d ChSoaSphericalBody::calcRelRot(const ChQuaterniond& q) {
    return ChMatrix33d(q);
}

void ChSoaSphericalBody::setJointTransform(const ChVectorDynamic<>& y) {
    ChQuaterniond quat(y(m_qIdx + 0), y(m_qIdx + 1), y(m_qIdx + 2), y(m_qIdx + 3));
    m_X_FM.SetRot(quat);
}

void ChSoaSphericalBody::prepSim() {
    //// NEEDED?!?
}

}  // namespace soa
}  // namespace chrono
