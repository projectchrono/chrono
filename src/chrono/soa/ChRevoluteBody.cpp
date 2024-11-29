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
                               const ChFramed& inbFrame,
                               const ChFramed& outbFrame,
                               const ChMassProps& mprops,
                               const std::string& name)
    : ChMobilizedBodyT<1>(parent, mprops, inbFrame, outbFrame, name) {
    m_H_FM.ang().col(0) = ChVector3d(0, 0, 1).eigen();
    m_H_FM.lin().col(0) = ChVector3d(0, 0, 0).eigen();

    m_H_FM_dot.ang().setZero();
    m_H_FM_dot.lin().setZero();
}

void ChRevoluteBody::setRelPos(double rotAngle) const {
    m_assembly->setCurState(m_qIdx, std::fmod(rotAngle, CH_2PI));
}

void ChRevoluteBody::setRelVel(double rotRate) const {
    m_assembly->setCurState(m_uIdx, rotRate);
}

void ChRevoluteBody::setRelAcc(double rotAcc) const {
    m_assembly->setCurStateDeriv(m_uIdx, rotAcc);
}

void ChRevoluteBody::setRelRot(const ChMatrix33d& relRot) const {
    float a = relRot(0, 0) + relRot(1, 1);
    float b = relRot(0, 1) - relRot(1, 0);
    m_assembly->setCurState(m_qIdx, atan2f(b, a));
}

void ChRevoluteBody::setRelAngVel(const ChVector3d& relAngVel) const {
    m_assembly->setCurState(m_uIdx, relAngVel.z());
}

void ChRevoluteBody::setRelAngAcc(const ChVector3d& relAngAcc) const {
    m_assembly->setCurStateDeriv(m_uIdx, relAngAcc.z());
}

ChMatrix33d ChRevoluteBody::calcRelRot(double q) {
    return ChMatrix33d(q, VECT_Z);
}

void ChRevoluteBody::setJointTransform(const ChVectorDynamic<>& y) {
    m_X_FM.SetRot(ChMatrix33d(y(m_qIdx), VECT_Z));
}

void ChRevoluteBody::prepSim() {
    double q = m_assembly->getCurState(m_qIdx);
    m_assembly->setCurState(m_qIdx, std::fmod(q, CH_2PI));
}

}  // namespace soa
}  // namespace chrono
