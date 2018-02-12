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

#ifndef CH_UTILS_CHASECAMERA_H
#define CH_UTILS_CHASECAMERA_H

#include "chrono/core/ChApiCE.h"
#include "chrono/physics/ChBody.h"

namespace chrono {
namespace utils {

class ChApi ChChaseCamera {
  public:
    enum State { Chase, Follow, Track, Inside, Free };

    ChChaseCamera(std::shared_ptr<ChBody> chassis);
    ~ChChaseCamera() {}

    void Initialize(const ChVector<>& ptOnChassis,
                    const ChCoordsys<>& driverCoordsys,
                    double chaseDist,
                    double chaseHeight);

    void Update(double step);

    void Zoom(int val);
    void Turn(int val);
    void Raise(int val);
    void SetState(State s);

    void SetCameraPos(const ChVector<>& pos);
    void SetCameraAngle(double angle);

    State GetState() const { return m_state; }
    const std::string& GetStateName() const { return m_stateNames[m_state]; }

    ChVector<> GetCameraPos() const;
    ChVector<> GetTargetPos() const;

    void SetHorizGain(double g) { m_horizGain = g; }
    void SetVertGain(double g) { m_vertGain = g; }

    void SetMultLimits(double minMult, double maxMult);

  private:
    ChVector<> calcDeriv(const ChVector<>& loc);

    State m_state;

    std::shared_ptr<ChBody> m_chassis;
    ChVector<> m_ptOnChassis;
    ChCoordsys<> m_driverCsys;
    double m_dist;
    double m_height;
    double m_mult;
    double m_angle;

    ChVector<> m_loc;
    ChVector<> m_lastLoc;

    double m_locZ;

    double m_horizGain;
    double m_vertGain;
    double m_minMult;
    double m_maxMult;

    static const double m_maxTrackDist2;
    static const std::string m_stateNames[5];
};

}  // end namespace utils
}  // end namespace chrono

#endif
