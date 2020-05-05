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
    instance().m_quat = rot.Get_A_quaternion();
    instance().m_vertical = rot.transpose() * ChVector<>(0, 0, 1);
    instance().m_forward = rot.transpose() * ChVector<>(1, 0, 0);
    instance().m_ISO = false;
}

void ChWorldFrame::SetYUP() {
    instance().m_rot = ChMatrix33<>(Q_from_AngX(CH_C_PI_2));
    instance().m_quat = instance().m_rot.Get_A_quaternion();
    instance().m_vertical = ChVector<>(0, 1, 0);
    instance().m_forward = ChVector<>(1, 0, 0);
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

const ChVector<>& ChWorldFrame::Vertical() {
    return instance().m_vertical;
}

const ChVector<>& ChWorldFrame::Forward() {
    return instance().m_forward;
}

ChVector<> ChWorldFrame::ToISO(const ChVector<>& v) {
    return instance().m_rot * v;
}

ChVector<> ChWorldFrame::FromISO(const ChVector<>& v) {
    return instance().m_rot.transpose() * v;
}

double ChWorldFrame::Height(const ChVector<>& v) {
    return instance().m_vertical ^ v;  // dot product
}

void ChWorldFrame::Project(ChVector<>& v) {
    v -= Height(v) * Vertical();
}

}  // end namespace vehicle
}  // end namespace chrono
