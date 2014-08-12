// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Generic chase camera.  Such a camera tracks a s[pecified point on a specified
// body. It supports three different modes:
// - Chase:  camera attached to a flexible cantilever behind the body.
// - Follow: camera attached to a flexible beam articulated on the body
// - Track:  camera is fixed and tracks the body
//
// TODO: 
// - relax current assumption that the body forward direction is in the negative
//   X direction.
// - add 'Inside' mode (driver POV attached to body)
//
// =============================================================================

#include "utils/ChChaseCamera.h"

namespace chrono {
namespace utils {


// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double ChChaseCamera::m_maxTrackDist2 = 100 * 100;
const std::string ChChaseCamera::m_stateNames[] = { "Chase", "Follow", "Track" };

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChChaseCamera::ChChaseCamera(const ChSharedBodyPtr chassis)
: m_chassis(chassis),
  m_mult(1),
  m_maxMult(10),
  m_minMult(0.5f),
  m_horizGain(4.0f),
  m_vertGain(2.0f),
  m_state(Chase)
{
  Initialize(ChVector<>(0, 0, 0), 5, 1);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChChaseCamera::Initialize(const ChVector<>& ptOnChassis,
                               double            chaseDist,
                               double            chaseHeight)
{
  m_ptOnChassis = ptOnChassis;
  m_dist = chaseDist;
  m_height = chaseHeight;
  m_angle = 0;

  ChVector<> localOffset(chaseDist, 0, chaseHeight);
  m_loc = m_chassis->GetCoord().TrasformLocalToParent(localOffset);
  m_lastLoc = m_loc;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChChaseCamera::SetState(State s)
{
  m_lastLoc = m_loc;
  m_state = s;
  if (m_state == Chase)
    m_angle = 0;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChChaseCamera::Zoom(int val)
{
  if (val == 0 || m_state == Track)
    return;

  if (val < 0 && m_mult > m_minMult)
    m_mult /= 1.01;
  else if (val > 0 && m_mult < m_maxMult)
    m_mult *= 1.01;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChChaseCamera::Turn(int val)
{
  if (val == 0 || m_state != Chase)
    return;

  if (val < 0 && m_angle > -0.75 * CH_C_PI)
    m_angle -= CH_C_PI / 100;
  else if (val > 0 && m_angle < 0.75 * CH_C_PI)
    m_angle += CH_C_PI / 100;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChChaseCamera::Update(double step)
{
  // Perform a simple Euler integration step (derivative controller).
  m_loc += step * calcDeriv(m_loc);

  if (m_state != Track)
    return;

  // If in 'Track' mode, force a state change if too far from tracked body.
  double dist2 = (m_chassis->GetPos() - m_lastLoc).Length2();
  if (dist2 > m_maxTrackDist2)
    SetState(Chase);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChVector<> ChChaseCamera::calcDeriv(const ChVector<>& loc)
{
  // Calculate the desired camera location, based on the current state of the
  // chassis.
  ChVector<> targetLoc = GetTargetPos();
  ChVector<> uC2T;
  ChVector<> desCamLoc;

  if (m_state == Follow)
    uC2T = targetLoc - m_loc;
  else {
    ChQuaternion<> rot = Q_from_AngAxis(m_angle, ChVector<>(0, 0, 1));
    uC2T = rot.Rotate(-m_chassis->GetA()->Get_A_Xaxis());
  }

  uC2T.z = 0;
  uC2T.Normalize();

  desCamLoc = targetLoc - m_mult * m_dist * uC2T;
  desCamLoc.z = targetLoc.z + m_mult * m_height;

  // Calculate the derivative vector (RHS of filter ODEs).
  ChVector<> deriv;

  deriv.x = m_horizGain * (desCamLoc.x - m_loc.x);
  deriv.y = m_horizGain * (desCamLoc.y - m_loc.y);
  deriv.z = m_vertGain  * (desCamLoc.z - m_loc.z);

  return deriv;
}


}  // end namespace utils
}  // end namespace chrono
