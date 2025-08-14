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
#include "chrono/soa/ChSoaPrismaticBody.h"

namespace chrono {
namespace soa {

ChSoaPrismaticBody::ChSoaPrismaticBody(std::shared_ptr<ChSoaMobilizedBody> parent,
                                       const ChSoaMassProperties& mprops,
                                       const ChFramed& inbFrame,
                                       const ChFramed& outbFrame,
                                       const std::string& name)
    : ChSoaMobilizedBodyT<1>(parent, mprops, inbFrame, outbFrame, name), m_q0(0), m_u0(0) {
    m_H_FM.ang().col(0) = ChVector3d(0, 0, 0).eigen();
    m_H_FM.lin().col(0) = ChVector3d(1, 0, 0).eigen();

    m_H_FM_dot.ang().setZero();
    m_H_FM_dot.lin().setZero();
}

ChSoaPrismaticBody::ChSoaPrismaticBody(const ChSoaPrismaticBody& other) : ChSoaMobilizedBodyT<1>(other) {
    //// TODO
}

// Mobilizer-specific setters for generalized coordinates, velocities, and acceleration

void ChSoaPrismaticBody::setRelPos(double x) {
    if (m_assembly && m_assembly->IsInitialized())
        setQ(0, x);
    else
        m_q0 = x;
}

void ChSoaPrismaticBody::setRelVel(double xd) {
    if (m_assembly && m_assembly->IsInitialized())
        setU(0, xd);
    else
        m_u0 = xd;
}

void ChSoaPrismaticBody::setRelAcc(double xdd) {
    if (m_assembly && m_assembly->IsInitialized())
        setUdot(0, xdd);
}

void ChSoaPrismaticBody::setRelLoc(const ChVector3d& relLoc) {
    if (m_assembly && m_assembly->IsInitialized())
        setQ(0, relLoc.x());
    else
        m_q0 = relLoc.x();
}

// Virtual overrides for setters for generalized coordinates, velocities, and acceleration

void ChSoaPrismaticBody::setRelLinVel(const ChVector3d& relLinVel) {
    if (m_assembly && m_assembly->IsInitialized())
        setU(0, relLinVel.x());
    else
        m_u0 = relLinVel.x();
}

void ChSoaPrismaticBody::setRelLinAcc(const ChVector3d& relLinAcc) {
    if (m_assembly && m_assembly->IsInitialized())
        setUdot(0, relLinAcc.x());
}

// Mobilizer-specific getters for state and derivatives

double ChSoaPrismaticBody::getQ0(int dof) const {
    assert(dof == 0);
    return m_q0;
}

double ChSoaPrismaticBody::getU0(int dof) const {
    assert(dof == 0);
    return m_u0;
}

// ---

ChVector3d ChSoaPrismaticBody::calcRelLoc(double q) {
    return ChVector3d(q, 0, 0);
}

void ChSoaPrismaticBody::setJointTransform(const ChVectorDynamic<>& y) {
    m_X_FM.SetPos(ChVector3d(y(m_qIdx), 0, 0));
}

void ChSoaPrismaticBody::prepSim() {
    //// NEEDED?!?
}

}  // namespace soa
}  // namespace chrono
