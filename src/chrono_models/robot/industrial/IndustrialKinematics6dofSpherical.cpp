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
// Authors: Dario Fusai
// =============================================================================
//
// Class for analytical solution of 6-DOF articulated robot kinematics.
//
// =============================================================================

#include "IndustrialKinematics6dofSpherical.h"

namespace chrono {
namespace industrial {

IndustrialKinematics6dofSpherical::IndustrialKinematics6dofSpherical(const std::array<ChCoordsysd, 7>& joints_abs_coord,
                                                               const std::array<double, 2>& vert_angs,
                                                               const std::array<double, 4>& lengths)
    : m_vert_angs(vert_angs), m_lengths(lengths) {
    m_num_joints = 6;

    SetupCoords(joints_abs_coord);
}

IndustrialKinematics6dofSpherical::IndustrialKinematics6dofSpherical(const std::array<ChCoordsysd, 7>& joints_abs_coord,
                                                               const std::array<double, 2>& vert_angs)
    : m_vert_angs(vert_angs) {
    m_num_joints = 6;

    SetupCoords(joints_abs_coord);
    m_lengths = {(m_joints_abs_coord[1].pos - m_joints_abs_coord[0].pos).Length(),   // H
                 (m_joints_abs_coord[2].pos - m_joints_abs_coord[1].pos).Length(),   // L1
                 (m_joints_abs_coord[4].pos - m_joints_abs_coord[2].pos).Length(),   // L2
                 (m_joints_abs_coord[6].pos - m_joints_abs_coord[4].pos).Length()};  // L3
}

IndustrialKinematics6dofSpherical::IndustrialKinematics6dofSpherical(const IndustrialKinematics6dofSpherical& other)
    : m_joints_abs_coord(other.m_joints_abs_coord),
      m_joints_rel_coord(other.m_joints_rel_coord),
      m_vert_angs(other.m_vert_angs),
      m_lengths(other.m_lengths) {
    m_num_joints = other.m_num_joints;
}

void IndustrialKinematics6dofSpherical::SetupCoords(const std::array<ChCoordsysd, 7>& joints_abs_coord) {
    m_joints_abs_coord = joints_abs_coord;
    m_joints_rel_coord = m_joints_abs_coord;  // m_joints_rel_coord[0] = m_joints_abs_coord[0]
    for (int i = 1; i < m_joints_rel_coord.size(); ++i)
        m_joints_rel_coord[i] = m_joints_abs_coord[i - 1].TransformParentToLocal(m_joints_abs_coord[i]);
}

void IndustrialKinematics6dofSpherical::SetupGeomData(const std::array<double, 2>& vert_angs,
                                                   const std::array<double, 4>& lengths) {
    m_vert_angs = vert_angs;
    m_lengths = lengths;
}

ChCoordsysd IndustrialKinematics6dofSpherical::GetFK(const ChVectorDynamic<>& u, int Nth) const {
    ChCoordsysd coord = m_joints_rel_coord[0];  // Fw0
    for (auto i = 1; i < Nth + 1; ++i)
        coord = coord * (QuatFromAngleZ(u[i - 1]) * m_joints_rel_coord[i]);  // up to FwN
    return coord;
}

ChCoordsysd IndustrialKinematics6dofSpherical::GetFK(const ChVectorDynamic<>& u) const {
    return GetFK(u, m_num_joints);
}

ChVectorDynamic<> IndustrialKinematics6dofSpherical::GetIK(const ChCoordsysd& targetcoord) const {
    // Inverse kinematics computation for 6dof articulated robot with spherical wrist
    // NB: works only for the following wrist joints starting rotation matrices ->
    // R34 = [1 0 0; 0 0 1; 0 -1 0], R45 = [0 0 -1; 1 0 0; 0 -1 0], R56 = eye(3).
    // Otherwise, recompute symbolic R36 and appropriate th4/th5/th6 expressions,
    // then inherit class and override GetIK() method.

    // 1) Wrist center positioning ------------------------------------------
    ChVector3d T_0 = m_joints_rel_coord[0].TransformPointParentToLocal(
        targetcoord.pos);                                       // demanded TCP target wrt robot base frame (0)
    ChQuaternion<> qw0 = m_joints_rel_coord[0].rot;             // rotation of robot base frame wrt world frame (w)
    ChQuaternion<> q06 = qw0.GetConjugate() * targetcoord.rot;  // demanded TCP rotation wrt robot base frame (0)
    ChVector3d W_0 = T_0 - q06.GetAxisZ() * m_lengths[3];  // demanded wrist center position wrt robot base frame (0)
    double tmp = 0.0;

    // theta 1
    double theta1 = std::atan2(W_0.y(), W_0.x());

    // theta 3 -> NB: use (z-h)^2 if frame1 is not coincident with frame0
    tmp = ChClamp((std::pow(W_0.x(), 2) + std::pow(W_0.y(), 2) + std::pow(W_0.z() - m_lengths[0], 2) -
                   std::pow(m_lengths[1], 2) - std::pow(m_lengths[2], 2)) /
                      (2.0 * m_lengths[1] * m_lengths[2]),
                  -1.0, 1.0);  // clamp in [-1, 1] to filter numerical error
    double theta3 = std::acos(tmp);

    // theta 2
    double theta2 = -std::atan2(W_0.x() * cos(theta1) + W_0.y() * std::sin(theta1), W_0.z() - m_lengths[0]) +
                    std::atan2(m_lengths[2] * std::sin(theta3), m_lengths[1] + m_lengths[2] * cos(theta3));

    // Adjust values: add +/-angles needed to start robot 3dof from theoretical vertical configuration
    theta2 = theta2 + m_vert_angs[0];
    theta3 = -(theta3) + m_vert_angs[1];  // negative theta3 (not adjusted): formula refers to CW rotation with respect
                                          // to L1 (instead of CCW)

    // 2) Wrist orientation -------------------------------------------------
    // Retrieve end effector rotation relative to wrist center frame
    ChQuaternion<> q03 = qw0.GetConjugate() * GetFK(ChVectorN<double, 3>{theta1, theta2, theta3}, 3)
                                                  .rot;  // rotation of forearm frame wrt robot base frame (0)
    ChQuaternion<> q36 = q03.GetConjugate() * q06;       // rotation of TCP wrt forearm frame (3)

    // theta 4, theta 5, theta 6 (see Slabaugh, "Computing Euler angles from a rotation matrix")
    // note: compute components of R36 rotm from quaternion (see Chrono Rotations whitepaper)
    tmp = std::pow(q36.e0(), 2) - std::pow(q36.e1(), 2) - std::pow(q36.e2(), 2) + std::pow(q36.e3(), 2);
    double R36_22 = ChClamp(tmp, -1.0, 1.0);
    double theta4 = 0.0, theta5 = 0.0, theta6 = 0.0;

    // if R36_22 != +/- 1:
    if (std::abs(std::abs(R36_22) - 1.0) > 1e-6) {
        // Normal state
        double R36_12 = ChClamp(2 * (-q36.e1() * q36.e0() + q36.e2() * q36.e3()), -1.0, 1.0);
        double R36_20 = ChClamp(2 * (q36.e1() * q36.e3() - q36.e2() * q36.e0()), -1.0, 1.0);
        double R36_21 = ChClamp(2 * (q36.e1() * q36.e0() + q36.e2() * q36.e3()), -1.0, 1.0);
        double R36_02 = ChClamp(2 * (q36.e1() * q36.e3() + q36.e2() * q36.e0()), -1.0, 1.0);

        theta5 = std::asin(R36_22);
        double cos_theta5 = -std::cos(theta5);  // why negative sign?

        theta4 = std::atan2(R36_12 / cos_theta5, R36_02 / cos_theta5);
        theta6 = std::atan2(-R36_21 / cos_theta5, R36_20 / cos_theta5);
    } else {
        // Gimbal lock
        theta6 = 0.0;  // assumption
        double R36_00 = ChClamp(
            std::pow(q36.e0(), 2) + std::pow(q36.e1(), 2) - std::pow(q36.e2(), 2) - std::pow(q36.e3(), 2), -1.0, 1.0);
        double R36_01 = ChClamp(2 * (q36.e1() * q36.e2() - q36.e3() * q36.e0()), -1.0, 1.0);

        // if R36_22 = -1:
        if (std::abs(R36_22 + 1.0) < 1e-5) {
            theta5 = -CH_PI_2;
            theta4 = -theta6 + std::atan2(-R36_01, R36_00);
        } else {
            theta5 = CH_PI_2;
            theta4 = theta6 + std::atan2(R36_01, -R36_00);
        }
    }

    ChVectorDynamic<> uik(m_num_joints);
    uik << theta1, theta2, theta3, theta4, theta5, theta6;
    return uik;
}

}  // end namespace industrial
}  // end namespace chrono