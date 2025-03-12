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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Base class for all suspension subsystems
//
// =============================================================================

#ifndef CH_SUSPENSION_H
#define CH_SUSPENSION_H

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftBodyConstraint.h"
#include "chrono/assets/ChVisualShapeCylinder.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/wheeled_vehicle/ChSubchassis.h"
#include "chrono_vehicle/wheeled_vehicle/ChSteering.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Base class for a suspension subsystem.
class CH_VEHICLE_API ChSuspension : public ChPart {
  public:
    struct ForceTSDA {
        std::string name;
        double force;
        double length;
        double velocity;
        ForceTSDA() : name(""), force(0), length(0), velocity(0) {}
        ForceTSDA(const std::string& n, double f, double l, double v) : name(n), force(f), length(l), velocity(v) {}
    };

    struct ForceRSDA {
        std::string name;
        double torque;
        double angle;
        double velocity;
        ForceRSDA() : name(""), torque(0), angle(0), velocity(0) {}
        ForceRSDA(const std::string& n, double t, double a, double v) : name(n), torque(t), angle(a), velocity(v) {}
    };

    virtual ~ChSuspension();

    /// Specify whether or not this suspension can be steered.
    virtual bool IsSteerable() const = 0;

    /// Specify whether or not this is an independent suspension.
    virtual bool IsIndependent() const = 0;

    /// Get the location of the suspension subsystem relative to the associated chassis reference frame.
    /// The suspension reference frame is always aligned with the chassis reference frame.
    const ChVector3d& GetRelPosition() const { return m_rel_loc; }

    /// Get a handle to the spindle body on the specified side.
    std::shared_ptr<ChBody> GetSpindle(VehicleSide side) const { return m_spindle[side]; }

    /// Get a handle to the axle shaft on the specified side.
    std::shared_ptr<ChShaft> GetAxle(VehicleSide side) const { return m_axle[side]; }

    /// Get a handle to the revolute joint on the specified side.
    std::shared_ptr<ChLinkLockRevolute> GetRevolute(VehicleSide side) const { return m_revolute[side]; }

    /// Get the global location of the spindle on the specified side.
    const ChVector3d& GetSpindlePos(VehicleSide side) const { return m_spindle[side]->GetPos(); }

    /// Get the orientation of the spindle body on the specified side.
    /// The spindle body orientation is returned as a quaternion representing a
    /// rotation with respect to the global reference frame.
    ChQuaternion<> GetSpindleRot(VehicleSide side) const;

    /// Get the linear velocity of the spindle body on the specified side.
    /// Return the linear velocity of the spindle center, expressed in the global
    /// reference frame.
    const ChVector3d& GetSpindleLinVel(VehicleSide side) const { return m_spindle[side]->GetPosDt(); }

    /// Get the angular velocity of the spindle body on the specified side.
    /// Return the angular velocity of the spindle frame, expressed in the global
    /// reference frame.
    ChVector3d GetSpindleAngVel(VehicleSide side) const { return m_spindle[side]->GetAngVelParent(); }

    /// Get the angular speed of the axle on the specified side.
    double GetAxleSpeed(VehicleSide side) const { return m_axle[side]->GetPosDt(); }

    /// Synchronize this suspension subsystem.
    virtual void Synchronize(double time);

    /// Advance the state of the suspension subsystem by the specified time step.
    virtual void Advance(double step) {}

    /// Apply the provided motor torque.
    /// The given torque is applied to the specified (left or right) axle. This
    /// function provides the interface to the drivetrain subsystem (intermediated
    /// by the vehicle system).
    void ApplyAxleTorque(VehicleSide side,  ///< indicates the axle (left or right) where the torque should be applied
                         double torque      ///< value of applied torque
    );

    /// Initialize this suspension subsystem.
    /// The suspension subsystem is initialized by attaching it to the specified chassis and (if provided) to the
    /// specified subchassis, at the specified location (with respect to and expressed in the reference frame of the
    /// chassis). It is assumed that the suspension reference frame is always aligned with the chassis reference frame.
    /// If a steering subsystem is provided, the suspension tierods are to be attached to the steering's central link
    /// body (steered suspension); otherwise they are to be attached to the chassis (non-steered suspension).
    void Initialize(std::shared_ptr<ChChassis> chassis,        ///< [in] associated chassis subsystem
                    std::shared_ptr<ChSubchassis> subchassis,  ///< [in] associated subchassis subsystem (may be null)
                    std::shared_ptr<ChSteering> steering,      ///< [in] associated steering subsystem (may be null)
                    const ChVector3d& location,                ///< [in] location relative to the chassis frame
                    double left_ang_vel = 0,                   ///< [in] initial angular velocity of left wheel
                    double right_ang_vel = 0                   ///< [in] initial angular velocity of right wheel
    );

    /// Return the radius of the spindle body (visualization only).
    virtual double getSpindleRadius() const = 0;

    /// Return the width of the spindle body (visualization only).
    virtual double getSpindleWidth() const = 0;

    /// Add visualization assets for the suspension subsystem.
    /// This default implementation uses primitives for spindle visualization.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the suspension subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Specify the suspension body on the specified side to attach a possible antirollbar subsystem.
    /// The default implementation returns a NULL pointer.
    virtual std::shared_ptr<ChBody> GetAntirollBody(VehicleSide side) const { return nullptr; }

    /// Specify the body on the specified side for a possible connection to brake subsystem.
    /// The default implementation returns a NULL pointer (indicating that a brake should connect to the chassis).
    virtual std::shared_ptr<ChBody> GetBrakeBody(VehicleSide side) const { return nullptr; }

    /// Get the wheel track for the suspension subsystem.
    virtual double GetTrack() = 0;

    /// Return current suspension TSDA force information on the specified side.
    virtual std::vector<ForceTSDA> ReportSuspensionForce(VehicleSide side) const { return {}; }

    /// Return current RSDA torque information on the specified side.
    virtual std::vector<ForceRSDA> ReportSuspensionTorque(VehicleSide side) const { return {}; }

    /// Log current constraint violations.
    virtual void LogConstraintViolations(VehicleSide side) {}

    /// Simple model of a parking brake.
    void ApplyParkingBrake(bool brake);

  protected:
    /// Construct a suspension subsystem with given name.
    ChSuspension(const std::string& name);

    /// Construct the concrete suspension subsystem.
    /// The suspension subsystem is initialized by attaching it to the specified chassis and (if provided) to the
    /// specified subchassis, at the specified location (with respect to and expressed in the reference frame of the
    /// chassis). It is assumed that the suspension reference frame is always aligned with the chassis reference frame.
    /// If a steering subsystem is provided, the suspension tierods are to be attached to the steering's central link
    /// body (steered suspension); otherwise they are to be attached to the chassis (non-steered suspension).
    virtual void Construct(
        std::shared_ptr<ChChassis> chassis,        ///< [in] associated chassis subsystem
        std::shared_ptr<ChSubchassis> subchassis,  ///< [in] associated subchassis subsystem (may be null)
        std::shared_ptr<ChSteering> steering,      ///< [in] associated steering subsystem (may be null)
        const ChVector3d& location,                ///< [in] location relative to the chassis frame
        double left_ang_vel,                       ///< [in] initial angular velocity of left wheel
        double right_ang_vel                       ///< [in] initial angular velocity of right wheel
        ) = 0;

    ChVector3d m_rel_loc;                                       ///< location relative to chassis
    std::shared_ptr<ChBody> m_spindle[2];                       ///< handles to spindle bodies
    std::shared_ptr<ChShaft> m_axle[2];                         ///< handles to axle shafts
    std::shared_ptr<ChShaftBodyRotation> m_axle_to_spindle[2];  ///< handles to spindle-shaft connectors
    std::shared_ptr<ChLinkLockRevolute> m_revolute[2];          ///< handles to spindle revolute joints

  private:
    std::shared_ptr<ChVisualShape> m_spindle_shapes[2];

    void AddVisualizationSpindle(VehicleSide side, double radius, double width);
};

/// Vector of handles to suspension subsystems.
typedef std::vector<std::shared_ptr<ChSuspension>> ChSuspensionList;

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
