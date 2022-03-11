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
// Generic chase camera.  Such a camera tracks a specified point on a specified
// body. It supports three different modes:
// - Chase:  camera attached to a flexible cantilever behind the body;
// - Follow: camera attached to a flexible beam articulated on the body;
// - Track:  camera is fixed and tracks the body;
// - Inside: camera is fixed at a given point on the body.
//
// Assumes Z up.
//
// TODO:
// - relax current assumption that the body forward direction is in the positive
//   X direction.
//
// =============================================================================

#include <cmath>

#include "chrono/utils/ChUtilsChaseCamera.h"

namespace chrono {
namespace utils {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double ChChaseCamera::m_maxTrackDist2 = 100 * 100;
const std::string ChChaseCamera::m_stateNames[] = {"Chase", "Follow", "Track", "Inside", "Free"};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChChaseCamera::ChChaseCamera(std::shared_ptr<ChBody> chassis)
    : m_chassis(chassis),
      m_mult(1),
      m_maxMult(10),
      m_minMult(0.5f),
      m_horizGain(4.0f),
      m_vertGain(2.0f),
      m_state(Chase) {
    Initialize(ChVector<>(0, 0, 0), ChCoordsys<>(), 5, 1);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChChaseCamera::Initialize(const ChVector<>& ptOnChassis,
                               const ChCoordsys<>& driverCoordsys,
                               double chaseDist,
                               double chaseHeight,
                               const ChVector<>& up,
                               const ChVector<>& fwd) {
    m_ptOnChassis = ptOnChassis;
    m_driverCsys = driverCoordsys;
    m_dist = chaseDist;
    m_height = chaseHeight;
    m_angle = 0;

    m_up = up;
    m_fwd = fwd;

    if (m_chassis) {
        ChVector<> localOffset(-m_dist, 0, m_height);
        m_loc = m_chassis->GetFrame_REF_to_abs().TransformPointLocalToParent(m_ptOnChassis + localOffset);
        m_lastLoc = m_loc;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChChaseCamera::SetState(State s) {
    if (m_state == Free) {
        m_mult = 1;
        m_angle = 0;
    }

    m_lastLoc = m_loc;
    m_state = s;

    if (m_state == Chase) {
        m_angle = 0;
    }
    if (m_state == Free) {
        m_loc = GetCameraPos();
        m_mult = 0;
        m_angle = 0;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChChaseCamera::Zoom(int val) {
    if (val == 0 || m_state == Track)
        return;

    if (m_state == Inside) {
        if (val > 0)
            SetState(Chase);
        return;
    }
        
    if (m_state == Chase || m_state == Follow) {
        if (val < 0 && m_mult > m_minMult)
            m_mult /= 1.01;
        else if (val > 0 && m_mult < m_maxMult)
            m_mult *= 1.01;

        if (m_mult <= m_minMult)
            SetState(Inside);
    }

    if (m_state == Free) {
        if (val < 0)
            m_mult += 0.01;
        else
            m_mult -= 0.01;
    }

}

void ChChaseCamera::Raise(int val) {
    if (val == 0 || m_state != Free)
        return;

    if (val < 0)
        m_loc += 0.01 * m_up;
    else
        m_loc -= 0.01 * m_up;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChChaseCamera::Turn(int val) {
    if (val == 0 || m_state == Follow || m_state == Track)
        return;

    if (m_state == Free) {
        if (val < 0)
            m_angle += CH_C_PI / 600;
        else
            m_angle -= CH_C_PI / 600;

        return;
    }

    if (val < 0 && m_angle > -CH_C_PI)
        m_angle -= CH_C_PI / 100;
    else if (val > 0 && m_angle < CH_C_PI)
        m_angle += CH_C_PI / 100;
}

// -----------------------------------------------------------------------------
// Set fixed camera position.
// Note that this also forces the state to 'Track'
// -----------------------------------------------------------------------------
void ChChaseCamera::SetCameraPos(const ChVector<>& pos) {
    m_loc = pos;
    m_lastLoc = pos;
    m_state = Track;
}

// -----------------------------------------------------------------------------
// Set the camera angle
// -----------------------------------------------------------------------------
void ChChaseCamera::SetCameraAngle(double angle) {
    m_angle = angle;
}

void ChChaseCamera::SetChassis(std::shared_ptr<ChBody> chassis) {
    m_chassis = chassis;

    ChVector<> localOffset(-m_dist, 0, m_height);
    m_loc = m_chassis->GetFrame_REF_to_abs().TransformPointLocalToParent(m_ptOnChassis + localOffset);
    m_lastLoc = m_loc;
}

// -----------------------------------------------------------------------------
// Return the camera location and the camera target (look at) location,
// respectively.
// Note that in Inside mode, in order to accommodate a narrow field of view, we
// set the target location to be at the current driver location and move back
// the camera position.
// -----------------------------------------------------------------------------
ChVector<> ChChaseCamera::GetCameraPos() const {
    if (m_state == Inside) {
        ChVector<> driverPos = m_chassis->GetFrame_REF_to_abs().TransformPointLocalToParent(m_driverCsys.pos);
        ChVector<> driverViewDir =
            m_chassis->GetFrame_REF_to_abs().TransformDirectionLocalToParent(m_driverCsys.rot.GetXaxis());
        return driverPos - 1.1 * driverViewDir;
    }

    if (m_state == Track)
        return m_lastLoc;

    return m_loc;
}

ChVector<> ChChaseCamera::GetTargetPos() const {
    if (m_state == Inside) {
        return m_chassis->GetFrame_REF_to_abs().TransformPointLocalToParent(m_driverCsys.pos);
    }

    if(m_state == Free) {
        ChMatrix33<> rot(m_angle, m_up);
        return m_loc + rot.Get_A_Xaxis() * 1.0;
    }

    return m_chassis->GetFrame_REF_to_abs().TransformPointLocalToParent(m_ptOnChassis);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChChaseCamera::SetMultLimits(double minMult, double maxMult) {
    m_minMult = minMult;
    m_maxMult = maxMult;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChChaseCamera::Update(double step) {
    // Perform a simple Euler integration step (derivative controller).
    m_loc += step * calcDeriv(m_loc);

    if (m_state != Track)
        return;

    // If in 'Track' mode, force a state change if too far from tracked body.
    double dist2 = (m_chassis->GetFrame_REF_to_abs().GetPos() - m_lastLoc).Length2();
    if (dist2 > m_maxTrackDist2)
        SetState(Chase);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChVector<> ChChaseCamera::calcDeriv(const ChVector<>& loc) {
    if (m_state == Free) {
        ChVector<> targetLoc = GetTargetPos();
        ChVector<> uC2T = targetLoc - m_loc;
        uC2T.Normalize();
        ChVector<> desCamLoc = m_loc + uC2T * m_mult;

        ChVector<> gains(m_horizGain, m_horizGain, m_horizGain);  // collect gains in a 3-vector
        gains += (m_vertGain - m_horizGain) * m_up;               // trick to overwrite the appropriate vertical gain
        ChVector<> deriv = gains * (desCamLoc - m_loc);           // component-wise vector multiplication!

        return deriv;
    }

    // Calculate the desired camera location, based on the current state of the chassis.
    ChVector<> targetLoc = GetTargetPos();
    ChVector<> uC2T;
    ChVector<> desCamLoc;

    if (m_state == Follow)
        uC2T = targetLoc - m_loc;
    else {
        ChQuaternion<> rot = Q_from_AngAxis(m_angle, m_up);
        uC2T = rot.Rotate(m_chassis->GetA().Get_A_Xaxis());
    }

    uC2T -= (uC2T ^ m_up) * m_up;  // zero out component in the vertical direction
    uC2T.Normalize();

    desCamLoc = targetLoc - m_mult * m_dist * uC2T;
    double desCamHeight = (m_up ^ targetLoc) + m_mult * m_height;  // desired camera height
    desCamLoc += (desCamHeight - (desCamLoc ^ m_up)) * m_up;       // overwrite component in the vertical direction

    // Calculate the derivative vector (RHS of filter ODEs).
    ChVector<> gains(m_horizGain, m_horizGain, m_horizGain);  // collect gains in a 3-vector
    gains += (m_vertGain - m_horizGain) * m_up;               // trick to overwrite the appropriate vertical gain
    ChVector<> deriv = gains * (desCamLoc - m_loc);           // component-wise vector multiplication!

    return deriv;
}

}  // end namespace utils
}  // end namespace chrono
