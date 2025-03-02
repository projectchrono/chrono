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

#include "chrono/soa/ChSoaAssembly.h"
#include "chrono/soa/ChRevoluteBody.h"

namespace chrono {
namespace soa {

ChRevoluteBody::ChRevoluteBody(std::shared_ptr<ChMobilizedBody> parent,
                               const ChMassProps& mprops,
                               const ChFramed& inbFrame,
                               const ChFramed& outbFrame,
                               const std::string& name)
    : ChMobilizedBodyT<1>(parent, mprops, inbFrame, outbFrame, name), m_q0(0), m_u0(0) {
    m_H_FM.ang().col(0) = ChVector3d(0, 0, 1).eigen();
    m_H_FM.lin().col(0) = ChVector3d(0, 0, 0).eigen();

    m_H_FM_dot.ang().setZero();
    m_H_FM_dot.lin().setZero();
}

ChRevoluteBody::ChRevoluteBody(const ChRevoluteBody& other) : ChMobilizedBodyT<1>(other) {
  //// TODO
}

void ChRevoluteBody::setRelPos(double rotAngle) {
    if (m_assembly && m_assembly->IsInitialized())
        setQ(0, std::fmod(rotAngle, CH_2PI));
    else
        m_q0 = std::fmod(rotAngle, CH_2PI);
}

void ChRevoluteBody::setRelVel(double rotRate) {
    if (m_assembly && m_assembly->IsInitialized())
        setU(0, rotRate);
    else
        m_u0 = rotRate;
}

void ChRevoluteBody::setRelAcc(double rotAcc) {
    if (m_assembly && m_assembly->IsInitialized())
        setUdot(0, rotAcc);
}

void ChRevoluteBody::setRelRot(const ChMatrix33d& relRot) {
    float a = relRot(0, 0) + relRot(1, 1);
    float b = relRot(0, 1) - relRot(1, 0);
    if (m_assembly && m_assembly->IsInitialized())
        setQ(0, atan2f(b, a));
    else
        m_q0 = atan2f(b, a);
}

void ChRevoluteBody::setRelAngVel(const ChVector3d& relAngVel) {
    if (m_assembly && m_assembly->IsInitialized())
        setU(0, relAngVel.z());
    else
        m_u0 = relAngVel.z();
}

void ChRevoluteBody::setRelAngAcc(const ChVector3d& relAngAcc) {
    if (m_assembly && m_assembly->IsInitialized())
        setUdot(0, relAngAcc.z());
}

double ChRevoluteBody::getQ0(int dof) const {
    assert(dof == 0);
    return m_q0;
}

double ChRevoluteBody::getU0(int dof) const {
    assert(dof == 0);
    return m_u0;
}

ChMatrix33d ChRevoluteBody::calcRelRot(double q) {
    return ChMatrix33d(q, VECT_Z);
}

void ChRevoluteBody::setJointTransform(const ChVectorDynamic<>& y) {
    m_X_FM.SetRot(ChMatrix33d(y(m_qIdx), VECT_Z));
}

void ChRevoluteBody::prepSim() {
//// NEEDED?!?
}

}  // namespace soa
}  // namespace chrono
