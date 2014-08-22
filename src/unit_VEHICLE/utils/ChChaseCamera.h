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

#ifndef CH_CHASECAMERA_H
#define CH_CHASECAMERA_H

#include "physics/ChBody.h"

#include "utils/ChApiUtils.h"


namespace chrono {
namespace utils {

class CH_UTILS_API ChChaseCamera {
public:

  enum State {
    Chase,
    Follow,
    Track
  };

  ChChaseCamera(const ChSharedBodyPtr chassis);
  ~ChChaseCamera() {}

  void Initialize(const ChVector<>& ptOnChassis,
                  double            chaseDist,
                  double            chaseHeight);

  void Update(double step);

  void Zoom(int val);
  void Turn(int val);
  void SetState(State s);

  State GetState() const                  { return m_state; }
  const std::string& GetStateName() const { return m_stateNames[m_state]; }
  const ChVector<>& GetCameraPos() const  { return (m_state == Track) ? m_lastLoc : m_loc; }
  ChVector<> GetTargetPos() const         { return m_chassis->GetCoord().TransformLocalToParent(m_ptOnChassis); }

  void SetHorizGain(double g)             { m_horizGain = g; }
  void SetVertGain(double g)              { m_vertGain = g; }
  void SetMultLimits(double minMult,
                     double maxMult)      { m_minMult = minMult; m_maxMult = maxMult; }

private:
  ChVector<> calcDeriv(const ChVector<>& loc);

  State m_state;

  ChSharedBodyPtr m_chassis;
  ChVector<> m_ptOnChassis;
  double m_dist;
  double m_height;
  double m_mult;
  double m_angle;

  ChVector<> m_loc;
  ChVector<> m_lastLoc;

  double m_horizGain;
  double m_vertGain;
  double m_minMult;
  double m_maxMult;

  static const double m_maxTrackDist2;
  static const std::string  m_stateNames[3];
};


}  // end namespace utils
}  // end namespace chrono


#endif
