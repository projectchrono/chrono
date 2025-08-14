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

#include <cmath>

#include "chrono/soa/ChSoaAssembly.h"
#include "chrono/soa/ChSoaUniversalBody.h"

namespace chrono {
namespace soa {

ChSoaUniversalBody::ChSoaUniversalBody(std::shared_ptr<ChSoaMobilizedBody> parent,
                                       const ChSoaMassProperties& mprops,
                                       const ChFramed& inbFrame,
                                       const ChFramed& outbFrame,
                                       const std::string& name)
    : ChSoaMobilizedBodyT<2>(parent, mprops, inbFrame, outbFrame, name), m_q0({0, 0}), m_u0({0, 0}) {
    m_H_FM.ang().col(0) = ChVector3d(1, 0, 0).eigen();
    m_H_FM.ang().col(1) = ChVector3d(0, 0, 0).eigen();

    m_H_FM.lin().col(0) = ChVector3d(0, 0, 0).eigen();
    m_H_FM.lin().col(1) = ChVector3d(0, 0, 0).eigen();

    m_H_FM_dot.ang().setZero();
    m_H_FM_dot.lin().setZero();
}

ChSoaUniversalBody::ChSoaUniversalBody(const ChSoaUniversalBody& other) : ChSoaMobilizedBodyT<2>(other) {
    //// TODO
}

// Mobilizer-specific setters for generalized coordinates, velocities, and acceleration

void ChSoaUniversalBody::setRelPos(double rotAngleX, double rotAngleY) {
    if (m_assembly && m_assembly->IsInitialized()) {
        setQ(0, rotAngleX);
        setQ(1, rotAngleY);
    } else {
        m_q0.x() = rotAngleX;
        m_q0.y() = rotAngleY;
    }
}

void ChSoaUniversalBody::setRelVel(double rotRateX, double rotRateY) {
    if (m_assembly && m_assembly->IsInitialized()) {
        setU(0, rotRateX);
        setU(1, rotRateY);
    } else {
        m_u0.x() = rotRateX;
        m_u0.y() = rotRateY;
    }
}

void ChSoaUniversalBody::setRelAcc(double rotAccX, double rotAccY) {
    if (m_assembly && m_assembly->IsInitialized()) {
        setUdot(0, rotAccX);
        setUdot(1, rotAccY);
    }
}

// Virtual overrides for setters for generalized coordinates, velocities, and acceleration

void ChSoaUniversalBody::setRelRot(const ChMatrix33d& relRot) {
    if (m_assembly && m_assembly->IsInitialized()) {
        setQ(0, atan2f(relRot(1, 2), relRot(1, 1)));
        setQ(1, atan2f(relRot(2, 0), relRot(0, 0)));
    } else {
        m_q0.x() = atan2f(relRot(1, 2), relRot(1, 1));
        m_q0.y() = atan2f(relRot(2, 0), relRot(0, 0));
    }
}

void ChSoaUniversalBody::setRelAngVel(const ChVector3d& relAngVel) {
    if (m_assembly && m_assembly->IsInitialized()) {
        double qx = getQ(0);
        double qy = getQ(1);

        double sx = std::sin(qx);
        double cx = std::cos(qx);
        double sy = std::sin(qy);
        double cy = std::cos(qy);

        ChMatrix33d R_FM({cy, sx * sy, -cx * sy},  //
                         {0, cx, sx},              //
                         {sy, -sx * cy, cx * cy});
        ChVector3d wyz_FM_M = R_FM.transpose() * ChVector3d(0, relAngVel[1], relAngVel[2]);

        setU(0, relAngVel.x());
        setU(1, wyz_FM_M.y());
    } else {
        double sx = std::sin(m_q0.x());
        double cx = std::cos(m_q0.x());
        double sy = std::sin(m_q0.y());
        double cy = std::cos(m_q0.y());

        ChMatrix33d R_FM({cy, sx * sy, -cx * sy},  //
                         {0, cx, sx},              //
                         {sy, -sx * cy, cx * cy});
        ChVector3d wyz_FM_M = R_FM.transpose() * ChVector3d(0, relAngVel[1], relAngVel[2]);

        m_u0.x() = relAngVel.x();
        m_u0.y() = wyz_FM_M.y();
    }
}

void ChSoaUniversalBody::setRelAngAcc(const ChVector3d& relAngAcc) {
    //// TODO
}

// Mobilizer-specific getters for state and derivatives

double ChSoaUniversalBody::getQ0(int dof) const {
    return m_q0[dof];
}

double ChSoaUniversalBody::getU0(int dof) const {
    return m_u0[dof];
}

// ---

ChMatrix33d ChSoaUniversalBody::calcRelRot(const ChVector2d& q) {
    double sx = std::sin(q.x());
    double cx = std::cos(q.x());
    double sy = std::sin(q.y());
    double cy = std::cos(q.y());

    return ChMatrix33d({cy, sx * sy, -cx * sy},  //
                       {0, cx, sx},              //
                       {sy, -sx * cy, cx * cy});
}

void ChSoaUniversalBody::setJointTransform(const ChVectorDynamic<>& y) {
    double qx = getQ(0);
    double qy = getQ(1);

    double sx = std::sin(qx);
    double cx = std::cos(qx);
    double sy = std::sin(qy);
    double cy = std::cos(qy);

    m_X_FM.SetRot(ChMatrix33d({cy, sx * sy, -cx * sy},  //
                              {0, cx, sx},              //
                              {sy, -sx * cy, cx * cy})  //
    );

    m_X_FM.SetPos(VNULL);
}

void ChSoaUniversalBody::setJointVelMat(const ChVectorDynamic<>& y) {
    m_H_FM.ang().col(1) = m_X_FM.GetRotMat().GetAxisY().eigen();
}

void ChSoaUniversalBody::setJointVelMatDot(const ChVectorDynamic<>& y) {
    auto cross = Vcross(m_V_FM.ang(), m_X_FM.GetRotMat().GetAxisY());
    m_H_FM_dot.ang().col(1) = cross.eigen();
}

void ChSoaUniversalBody::prepSim() {
    //// NEEDED?!?
}

}  // namespace soa
}  // namespace chrono
