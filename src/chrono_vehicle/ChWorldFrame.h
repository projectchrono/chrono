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

#ifndef CH_WORLD_FRAME_H
#define CH_WORLD_FRAME_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChRotation.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Definition of the world frame for Chrono::Vehicle simulations.
/// The default is an ISO frame (Z up, X forward, Y to the left) and vehicle are assumed to be modeled in such a
/// reference frame. However, this class allows a Chrono::Vehicle model to be simulated in a scene specified in a
/// different reference frame (for example, a Y up frame).
/// The world frame is uniquely defined through a rotation matrix (the rotation required to align the ISO frame with the
/// desired world frame). To change the world frame definition from the default ISO convention, the desired world frame
/// must be set **before** any Chrono::Vehicle library call.
class CH_VEHICLE_API ChWorldFrame {
  public:
    ChWorldFrame(ChWorldFrame const&) = delete;
    void operator=(ChWorldFrame const&) = delete;

    /// Set the world frame as a rotation from the base ISO frame.
    static void Set(const ChMatrix33<>& rot);

    /// Set the world frame as a (right-handed) frame with Y up.
    /// This corresponds to a rotation of 90 degrees about the X axis.
    static void SetYUP();

    /// Return `true` if the world frame is an ISO reference frame and `false` otherwise.
    static bool IsISO();

    /// Get the world frame rotation matrix.
    static const ChMatrix33<>& Rotation();

    /// Get the world frame orientation as a quaternion.
    static const ChQuaternion<>& Quaternion();

    /// Get the vertical direction of the world frame.
    static const ChVector3d& Vertical();

    /// Get the forward direction of the world frame.
    static const ChVector3d& Forward();

    /// Re-express a vector from the current world frame into the ISO frame.
    static ChVector3d ToISO(const ChVector3d& v);

    /// Re-express a vector from the ISO frame to the current world frame.
    static ChVector3d FromISO(const ChVector3d& v);

    /// Get the height of a given vector in the current world frame (i.e., the projection of the vector onto the
    /// vertical direction).
    static double Height(const ChVector3d& v);

    /// Project a given vector onto the horizontal plane of the world frame.
    static void Project(ChVector3d& v);

  private:
    /// Default world frame is ISO, corresponding to an identity rotation.
    ChWorldFrame() : m_rot(1), m_quat(1, 0, 0, 0), m_vertical(0, 0, 1), m_forward(1, 0, 0), m_ISO(true) {}

    /// Return the (unique) instance of the world frame.
    static ChWorldFrame& instance() {
        static ChWorldFrame world_frame;
        return world_frame;
    }

    ChMatrix33<> m_rot;     ///< world frame orientation (relative to ISO) as rotation matrix
    ChQuaternion<> m_quat;  ///< world frame orientation (relative to ISO) as quaternion
    ChVector3d m_vertical;  ///< vertical direction of the world frame
    ChVector3d m_forward;   ///< forward direction of the world frame
    bool m_ISO;             ///< flag indicating whether or not the world frame is ISO
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
