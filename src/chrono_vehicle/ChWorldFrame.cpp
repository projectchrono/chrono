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
// Authors: Radu Serban
// =============================================================================
//
// Definition of the world frame for Chrono::Vehicle simulations.
//
// =============================================================================

#include "chrono_vehicle/ChWorldFrame.h"

namespace chrono {
namespace vehicle {

void ChWorldFrame::Set(const ChMatrix33<>& rot) {
    instance().m_rot = rot;
    instance().m_quat = rot.GetQuaternion();
    instance().m_vertical = rot.transpose() * ChVector3d(0, 0, 1);
    instance().m_forward = rot.transpose() * ChVector3d(1, 0, 0);
    instance().m_ISO = false;
}

void ChWorldFrame::SetYUP() {
    instance().m_rot = ChMatrix33<>(QuatFromAngleX(CH_PI_2));
    instance().m_quat = instance().m_rot.GetQuaternion();
    instance().m_vertical = ChVector3d(0, 1, 0);
    instance().m_forward = ChVector3d(1, 0, 0);
    instance().m_ISO = false;
}

bool ChWorldFrame::IsISO() {
    return instance().m_ISO;
}

const ChMatrix33<>& ChWorldFrame::Rotation() {
    return instance().m_rot;
}

const ChQuaternion<>& ChWorldFrame::Quaternion() {
    return instance().m_quat;
}

const ChVector3d& ChWorldFrame::Vertical() {
    return instance().m_vertical;
}

const ChVector3d& ChWorldFrame::Forward() {
    return instance().m_forward;
}

ChVector3d ChWorldFrame::ToISO(const ChVector3d& v) {
    return instance().m_rot * v;
}

ChVector3d ChWorldFrame::FromISO(const ChVector3d& v) {
    return instance().m_rot.transpose() * v;
}

double ChWorldFrame::Height(const ChVector3d& v) {
    return instance().m_vertical ^ v;  // dot product
}

void ChWorldFrame::Project(ChVector3d& v) {
    v -= Height(v) * Vertical();
}

}  // end namespace vehicle
}  // end namespace chrono
