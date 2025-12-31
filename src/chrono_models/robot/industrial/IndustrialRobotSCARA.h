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
// Model of industrial SCARA robot.
//
// =============================================================================

#ifndef INDUSTRIAL_ROBOT_SCARA_H
#define INDUSTRIAL_ROBOT_SCARA_H

#include "IndustrialRobot.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"

namespace chrono {
namespace industrial {

/// @addtogroup robot_models_industrial
/// @{

class CH_MODELS_API IndustrialRobotSCARA : public IndustrialRobot {
  public:
    /// Default constructor.
    IndustrialRobotSCARA(){};

    /// Build SCARA R-R-R-P robot model from given arm lengths (as in scheme below) and add it to sys.
    ///    L1  L2
    ///   I---I---I
    ///   |     D |
    /// H |       I--<
    ///   |        L3
    ///  ---
    IndustrialRobotSCARA(
        ChSystem* sys,                           ///< containing sys
        const std::array<double, 5>& lengths,    ///< robot arm lengths: base height (H), biceps length (L1), forearm
                                                 ///< length (L2), screw height (D), end-effector length (L3)
        const ChFramed& base_frame = ChFramed()  ///< place robot base in these coordinates
    );

    /// Get specific robot body.
    std::shared_ptr<ChBody> GetBiceps() { return m_biceps; };
    std::shared_ptr<ChBody> GetForearm() { return m_forearm; };
    std::shared_ptr<ChBody> GetScrew() { return m_screw; };

    /// Get specific robot joint marker.
    std::shared_ptr<ChMarker> GetMarkerGroundBase() const { return m_marker_ground_base; }
    std::shared_ptr<ChMarker> GetMarkerBaseBiceps() const { return m_marker_base_biceps; }
    std::shared_ptr<ChMarker> GetMarkerBicepsForearm() const { return m_marker_biceps_forearm; }
    std::shared_ptr<ChMarker> GetMarkerForearmScrew() const { return m_marker_forearm_screw; }

    /// Get specific robot motor.
    std::shared_ptr<ChLinkMotorRotationAngle> GetLinkBaseBiceps() { return m_link_base_biceps; }
    std::shared_ptr<ChLinkMotorRotationAngle> GetLinkBicepsForearm() { return m_link_biceps_forearm; }
    std::shared_ptr<ChLinkMotorRotationAngle> GetLinkForearmScrewRot() { return m_link_forearm_screw_rot; }
    std::shared_ptr<ChLinkMotorLinearPosition> GetLinkForearmScrewTransl() { return m_link_forearm_screw_transl; }

    /// Get motors diplacement/velocity/acceleration/torque data.
    /// NB: given the R-R-R-P architecture, the first three motors are rotational and the fourth is translational;
    /// hence, the last returned quantities are linear values/forces.
    virtual ChVectorDynamic<> GetMotorsPos(bool wrap_angles = false) const override;
    virtual ChVectorDynamic<> GetMotorsPosDt() const override;
    virtual ChVectorDynamic<> GetMotorsPosDt2() const override;
    virtual ChVectorDynamic<> GetMotorsForce() const override;

    /// Get robot arm lengths.
    std::array<double, 5> GetLengths() const { return m_lengths; }

    /// Add 1D visual shapes to model, representing robot arm as linear segments.
    virtual void Add1dShapes(const ChColor& col = ChColor(0.0f, 0.0f, 0.0f)) override;

    /// Add 3D visual shapes to model, loosely representing robot silhouette from given arm radius characteristic size.
    virtual void Add3dShapes(double rad, const ChColor& col = ChColor(0.2f, 0.2f, 0.2f)) override;

  protected:
    /// Setup robot internal bodies, markers and motors.
    virtual void SetupBodies();
    virtual void SetupMarkers();
    virtual void SetupLinks();

    std::array<double, 5> m_lengths = {0, 0, 0, 0, 0};  ///< robot arm lengths (H, L1, L2, D, L3)
    std::shared_ptr<ChBody> m_biceps, m_forearm, m_screw;
    std::shared_ptr<ChMarker> m_marker_ground_base, m_marker_base_biceps, m_marker_biceps_forearm,
        m_marker_forearm_screw;
    std::shared_ptr<ChLinkMotorRotationAngle> m_link_base_biceps, m_link_biceps_forearm, m_link_forearm_screw_rot;
    std::shared_ptr<ChLinkMotorLinearPosition> m_link_forearm_screw_transl;
};

/// @} robot_models_industrial

}  // end namespace industrial
}  // end namespace chrono

#endif  // end INDUSTRIAL_ROBOT_SCARA_H