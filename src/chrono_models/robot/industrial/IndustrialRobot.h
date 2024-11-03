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
// Base class for industrial robotics models.
//
// =============================================================================

#ifndef INDUSTRIAL_ROBOT_H
#define INDUSTRIAL_ROBOT_H

#include "chrono_models/ChApiModels.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"

namespace chrono {
namespace industrial {

/// @addtogroup robot_models_industrial
/// @{

class CH_MODELS_API IndustrialRobot {
  public:
    /// Default constructor.
    IndustrialRobot(){};

    /// Default destructor.
    virtual ~IndustrialRobot() = default;

    /// Get the containing sys.
    ChSystem* GetSystem() { return m_sys; }

    /// Get the list of bodies in robot model.
    std::vector<std::shared_ptr<ChBody>> GetBodies() { return m_bodylist; }

    /// Get the list of joint markers in robot model.
    std::vector<std::shared_ptr<ChMarker>> GetMarkers() { return m_markerlist; }

    /// Get the list of motor functions in robot model.
    std::vector<std::shared_ptr<ChFunctionSetpoint>> GetMotfuns() { return m_motfunlist; }

    /// Get the list of motors in robot model.
    std::vector<std::shared_ptr<ChLinkMotor>> GetMotors() { return m_motorlist; }

    /// Get robot base body.
    std::shared_ptr<ChBody> GetBase() { return m_base; };

    /// Get robot end-effector body.
    std::shared_ptr<ChBody> GetEndEffector() { return m_end_effector; };

    /// Get robot Tool-Center-Point marker.
    std::shared_ptr<ChMarker> GetMarkerTCP() const { return m_marker_TCP; }

    /// Get motors rotations/positions, depending on specific robot architecture.
    /// Optionally wrap angles in interval [-PI..+PI].
    virtual ChVectorDynamic<> GetMotorsPos(bool wrap_angles = false) const = 0;

    /// Get motors angular velocities/linear velocities, depending on specific robot architecture.
    virtual ChVectorDynamic<> GetMotorsPosDt() const = 0;

    /// Get motors angular accelerations/linear accelerations, depending on specific robot architecture.
    virtual ChVectorDynamic<> GetMotorsPosDt2() const = 0;

    /// Get motors torques/forces, depending on specific robot architecture.
    virtual ChVectorDynamic<> GetMotorsForce() const = 0;

    /// Get motors travelled distance.
    /// NB: needs runtime call to UpdateEncoder() function to be kept updated.
    ChVectorDynamic<> GetEncoderReading() const { return m_encoder; }

    /// Get overall robot mass.
    virtual double GetMass() const;

    /// Disable robot motors (true), or reactivate them (false).
    void SetMotorsDisabled(bool disable);

    /// Get if motors are currently disabled.
    bool GetMotorsDisabled() const { return m_motors_disabled; }

    /// Set robot base coordinates and consequently update other internal frames.
    virtual void SetBaseFrame(const ChFramed& base_frame);

    /// Set individual motors setpoints, at given time.
    void SetSetpoints(const ChVectorDynamic<>& setpoints, double t);

    /// Set motors setpoints all equal to given value, at given time.
    void SetSetpoints(double setpoint, double t);

    /// Move robot TCP to imposed absolute frame.
    /// Useful to get single-shot numerical Inverse Kinematics. Not suitable for trajectory following.
    AssemblyAnalysis::ExitFlag SetPoseTCP(const ChFramed& target_frame, unsigned int max_iters = 10);

    /// Set robot color (if visual shapes are added).
    void SetColor(const ChColor& col);

    /// Kinematically link external body to a specific robot body, in given frame.
    /// Useful for grip and relocation of some object by robot end-effector.
    void AttachBody(std::shared_ptr<ChBody> body_attach, std::shared_ptr<ChBody> robot_body, const ChFrame<>& frame);

    /// Unlink external body from robot (if previously attached) and potentially set it to 'fixed' thereafter.
    void DetachBody(std::shared_ptr<ChBody> body_attach, bool setfix = false);

    /// Update internal encoder to keep track of distance travelled by motors.
    /// NB: call this function at runtime and get reading with GetEncoderReading() function.
    virtual void UpdateEncoder();

    /// Add 1D visual shapes to model, representing robot arm as linear segments.
    virtual void Add1dShapes(const ChColor& col = ChColor(0.0f, 0.0f, 0.0f)) = 0;

    /// Add 3D visual shapes to model, loosely representing robot silhouette from given arm radius characteristic size.
    virtual void Add3dShapes(double rad, const ChColor& col = ChColor(0.2f, 0.2f, 0.2f)) = 0;

    /// Clear all visual shapes added to model.
    virtual void ClearShapes();

  protected:
    /// Helper function to quickly create a ChLinkMotorRotationAngle.
    virtual std::shared_ptr<ChLinkMotorRotationAngle> CreateMotorRotationAngle(
        ChSystem* sys,
        std::shared_ptr<ChBody> body1,
        std::shared_ptr<ChBody> body2,
        const ChFramed& frame,
        std::shared_ptr<ChFunction> motfun = nullptr);

    /// Helper function to quickly create a ChLinkMotorLinearPosition.
    virtual std::shared_ptr<ChLinkMotorLinearPosition> CreateMotorLinearPosition(
        ChSystem* sys,
        std::shared_ptr<ChBody> body1,
        std::shared_ptr<ChBody> body2,
        const ChFramed& frame,
        std::shared_ptr<ChFunction> motfun = nullptr);

    /// Helper function to switch off motors guide constraints and create dedicated passive links to replace them.
    /// Useful to quickly toggle actuation of motors, so that
    /// - robot actively drives motors
    /// - or passively follows externally imposed trajectory (e.g. on the end effector)
    virtual void CreatePassiveLinks();

    ChSystem* m_sys = nullptr;  ///< containing sys
    ChFramed m_base_frame;      ///< coordinates of robot base

    std::shared_ptr<ChBody> m_base, m_end_effector;
    std::vector<std::shared_ptr<ChBody>> m_bodylist;                ///< list of robot bodies
    std::vector<std::shared_ptr<ChMarker>> m_markerlist;            ///< list of robot joint markers
    std::vector<std::shared_ptr<ChFunctionSetpoint>> m_motfunlist;  ///< list of robot motors functions
    std::vector<std::shared_ptr<ChLinkMotor>> m_motorlist;          ///< list of robot motors
    std::vector<ChFramed> m_joint_frames;                           ///< list of starting joint frames
    std::shared_ptr<ChMarker> m_marker_TCP;                         ///< Tool-Center-Point marker
    bool m_motors_disabled = false;                                 ///< activation state of motors

    std::shared_ptr<ChLinkMateFix> m_link_attach = nullptr;  ///< internal link to grip external body
    bool m_body_attached = false;                            ///< flag for external body attached/not attached
    ChVectorDynamic<> m_encoder, m_encoder_prev;             ///< motors encoder internal data
};

/// @} robot_models_industrial

}  // end namespace industrial
}  // end namespace chrono

#endif  // end INDUSTRIAL_ROBOT_H