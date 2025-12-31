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
// Class for analytical solution of SCARA robot kinematics.
//
// =============================================================================

#include "IndustrialKinematicsSCARA.h"

namespace chrono {
namespace industrial {


IndustrialKinematicsSCARA::IndustrialKinematicsSCARA(const std::array<ChCoordsysd, 5>& joints_abs_coord, const std::array<double, 5>& lengths, bool right_elbow)
	: m_lengths(lengths), m_right_elbow(right_elbow)
{
	m_num_joints = 4;

    SetupCoords(joints_abs_coord);
	m_TCP_rot0 = m_joints_abs_coord.back().rot;
	m_TCP_offset0 = m_joints_abs_coord.back().pos - m_joints_abs_coord.front().pos;
}

//// Infer lengths from given m_Xabs positions (?)
//IndustrialKinematicsSCARA::IndustrialKinematicsSCARA(const std::vector<ChCoordsysd>& Xabs, bool right_elbow)
//: m_right_elbow{ right_elbow }
//{
//	m_Nj = 4;
//    SetupCoords(Xabs);
//	m_TCP_rot0 = Xabs.back().rot;
//	m_TCP_offset0 = Xabs.back().pos - Xabs.front().pos;
//	lengths = {
//		(m_Xabs[1].pos - m_Xabs[0].pos).Length(),   // H
//		(m_Xabs[2].pos - m_Xabs[1].pos).Length(),   // L1
//		(m_Xabs[3].pos - m_Xabs[2].pos).Length(),   // L2
//		(m_Xabs[4].pos - m_Xabs[3].pos).Length() }; // L3
//}

IndustrialKinematicsSCARA::IndustrialKinematicsSCARA(const IndustrialKinematicsSCARA& other)
	: m_joints_abs_coord(other.m_joints_abs_coord), m_joints_rel_coord(other.m_joints_rel_coord), m_lengths(other.m_lengths), m_right_elbow(other.m_right_elbow), m_TCP_rot0(other.m_TCP_rot0), m_TCP_offset0(other.m_TCP_offset0)
{
	m_num_joints = other.m_num_joints;
}

void IndustrialKinematicsSCARA::SetupCoords(const std::array<ChCoordsysd, 5>& joints_abs_coord) {
	m_joints_abs_coord = joints_abs_coord;
	m_joints_rel_coord = joints_abs_coord; // m_joints_rel_coord[0] = m_joints_abs_coord[0]
	for (int i = 1; i < m_joints_rel_coord.size(); ++i)
		m_joints_rel_coord[i] = m_joints_abs_coord[i - 1].TransformParentToLocal(m_joints_abs_coord[i]);
}

void IndustrialKinematicsSCARA::SetupGeomData(const std::array<double, 5>& lengths, bool right_elbow) {
	m_lengths = lengths;
	m_right_elbow = right_elbow;
}

ChCoordsysd IndustrialKinematicsSCARA::GetFK(const ChVectorDynamic<>& u) const {
	// u = RRRP
	ChCoordsysd coord = m_joints_rel_coord[0] 
		* (QuatFromAngleZ(u[0]) * m_joints_rel_coord[1]) 
		* (QuatFromAngleZ(u[1]) * m_joints_rel_coord[2]) 
		* m_joints_rel_coord[3] 
		* ChCoordsysd(ChVector3d(0, 0, u[3]), QuatFromAngleZ(u[2])) 
		* m_joints_rel_coord[4];
	return coord;
}

ChVectorDynamic<> IndustrialKinematicsSCARA::GetIK(const ChCoordsysd& targetcoord) const {
	ChVector3d T_0 = m_joints_rel_coord[0].TransformPointParentToLocal(targetcoord.pos); // demanded EE position in robot base frame (0)
	double phi = (m_TCP_rot0.GetConjugate() * targetcoord.rot).GetRotVec().Length(); // demanded EE rotation
	double Wx = T_0.x() - m_lengths[4] * cos(phi); // wrist (screw) center x
	double Wy = T_0.y() - m_lengths[4] * std::sin(phi); // wrist (screw) center y
	
	// theta2
	double tmp = ChClamp(
		(std::pow(Wx, 2) + std::pow(Wy, 2) - std::pow(m_lengths[1], 2) - std::pow(m_lengths[2], 2)) / (2 * m_lengths[1] * m_lengths[2]),
		-1.0, 1.0);
	double theta2 = (m_right_elbow == true) ? std::acos(tmp) : -std::acos(tmp);
	
	// theta1
	double theta1 = std::atan2(Wy, Wx) - std::atan2(m_lengths[2] * std::sin(theta2), m_lengths[1] + m_lengths[2] * cos(theta2));

	// theta3
	double theta3 = phi - theta1 - theta2;

	// theta4 -> NB: vertical displacement!
	double theta4 = T_0.z() - m_joints_rel_coord[0].TransformPointParentToLocal(m_TCP_offset0).z(); // CAN BE REPLACED WITH NEW m_lengths[3]==D ?

	ChVectorDynamic<> uik(m_num_joints);
	uik << theta1, theta2, theta3, theta4;
	return uik;
}


} // end namespace industrial
} // end namespace chrono