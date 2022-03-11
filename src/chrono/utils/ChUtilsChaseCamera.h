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

/// Utility class for a generic chase camera which can be associated with any ChBody in a Chrono system.
/// This camera tracks a given point on the associated body (typically the chassis of a vehicle).
class ChApi ChChaseCamera {
  public:
    /// Chase camera operation mode
    enum State {
        Chase,   ///< camera attached to a "flexible cantilever" behind the body
        Follow,  ///< camera attached to a "flexible beam" articulated on the body
        Track,   ///< camera is fixed and tracks the body
        Inside,  ///< camera is rigidly fixed at a given point on the body
        Free     ///< detached camera
    };

    /// Construct a chase camera associated with the given body.
    ChChaseCamera(std::shared_ptr<ChBody> chassis);

    ~ChChaseCamera() {}

    /// Initialize the chase camera.
    void Initialize(const ChVector<>& ptOnChassis,       ///< target point on associated body
                    const ChCoordsys<>& driverCoordsys,  ///< local driver position (for `Inside` mode)
                    double chaseDist,    ///< base value for distance behind target point on associated body
                    double chaseHeight,  ///< base value for height above target point on associated body
                    const ChVector<>& up = ChVector<>(0, 0, 1),  ///< vertical direction of the world frame
                    const ChVector<>& fwd = ChVector<>(1, 0, 0)  ///< forward direction of the world frame
    );

    /// Update internal dynamics.
    void Update(double step);

    /// Control chase distance.
    void Zoom(int val);

    /// Control angle (in horizontal plane) around target point.
    void Turn(int val);

    /// Control camera height (for `Free` mode).
    void Raise(int val);

    /// Set the chase camera operation mode.
    void SetState(State s);

    /// Overwrite chase camera position.
    void SetCameraPos(const ChVector<>& pos);

    /// Overwrite chase camera angle (in horizontal plane).
    void SetCameraAngle(double angle);

    /// Get the current operation mode.
    State GetState() const { return m_state; }

    /// Return a string description of the current operation mode.
    const std::string& GetStateName() const { return m_stateNames[m_state]; }

    /// Get current camera location (expressed in the world frame).
    ChVector<> GetCameraPos() const;

    /// Get current location of the target point (expressed in the world frame).
    ChVector<> GetTargetPos() const;

    /// Set internal dynamics gain for camera movement in the horizontal plane.
    void SetHorizGain(double g) { m_horizGain = g; }

    /// Set internal dynamics gain for camera movement in the vertical direction.
    void SetVertGain(double g) { m_vertGain = g; }

    /// Set zoom limits.
    void SetMultLimits(double minMult, double maxMult);

    /// Change the associated body.
    void SetChassis(std::shared_ptr<ChBody> chassis);

    /// Set the target point on the associated body.
    void SetTargetPoint(const ChVector<>& point) { m_ptOnChassis = point; }

    /// Set the chase distance.
    void SetChaseDistance(double dist) { m_dist = dist; }

    /// Set the chase height.
    void SetChaseHeight(double height) { m_height = height; }

  private:
    /// Calculate derivatives for internal dynamics ODE.
    ChVector<> calcDeriv(const ChVector<>& loc);

    State m_state;  ///< current camera operation mode

    ChVector<> m_up;   ///< world up direction
    ChVector<> m_fwd;  ///< world forward direction

    std::shared_ptr<ChBody> m_chassis;  ///< associated body
    ChVector<> m_ptOnChassis;           ///< target point on associated body (in local coordinates)
    ChCoordsys<> m_driverCsys;          ///< "driver" position on associated body (for `Inside` mode)
    double m_dist;                      ///< chase distance (behind target point)
    double m_height;                    ///< chase height (above target point)
    double m_mult;                      ///< zoom multiplier
    double m_angle;                     ///< camera angle (in horizontal plane)

    ChVector<> m_loc;      ///< current camera location
    ChVector<> m_lastLoc;  ///< last cached camera location (used when switching modes)

    double m_horizGain;  ///< internal dynamics gain for camera movement in horizontal plane
    double m_vertGain;   ///< internal dynamics gain for camera movement in vertical direction
    double m_minMult;    ///< lower zoom limit
    double m_maxMult;    ///< upper zoom limit

    static const double m_maxTrackDist2;
    static const std::string m_stateNames[5];
};

}  // end namespace utils
}  // end namespace chrono

#endif
