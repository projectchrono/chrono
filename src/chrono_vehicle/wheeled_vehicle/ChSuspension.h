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
#include "chrono/physics/ChShaftsBody.h"
#include "chrono/assets/ChCylinderShape.h"

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
    struct Force {
        double spring_force;
        double shock_force;
        double spring_length;
        double spring_velocity;
        double shock_length;
        double shock_velocity;
    };

    virtual ~ChSuspension();

    /// Specify whether or not this suspension can be steered.
    virtual bool IsSteerable() const = 0;

    /// Specify whether or not this is an independent suspension.
    virtual bool IsIndependent() const = 0;

    /// Get the location of the suspension subsystem relative to the associated chassis reference frame.
    /// The suspension reference frame is always aligned with the chassis reference frame.
    const ChVector<>& GetRelPosition() const { return m_rel_loc; }

    /// Get a handle to the spindle body on the specified side.
    std::shared_ptr<ChBody> GetSpindle(VehicleSide side) const { return m_spindle[side]; }

    /// Get a handle to the axle shaft on the specified side.
    std::shared_ptr<ChShaft> GetAxle(VehicleSide side) const { return m_axle[side]; }

    /// Get a handle to the revolute joint on the specified side.
    std::shared_ptr<ChLinkLockRevolute> GetRevolute(VehicleSide side) const { return m_revolute[side]; }

    /// Get the global location of the spindle on the specified side.
    const ChVector<>& GetSpindlePos(VehicleSide side) const { return m_spindle[side]->GetPos(); }

    /// Get the orientation of the spindle body on the specified side.
    /// The spindle body orientation is returned as a quaternion representing a
    /// rotation with respect to the global reference frame.
    ChQuaternion<> GetSpindleRot(VehicleSide side) const;

    /// Get the linear velocity of the spindle body on the specified side.
    /// Return the linear velocity of the spindle center, expressed in the global
    /// reference frame.
    const ChVector<>& GetSpindleLinVel(VehicleSide side) const { return m_spindle[side]->GetPos_dt(); }

    /// Get the angular velocity of the spindle body on the specified side.
    /// Return the angular velocity of the spindle frame, expressed in the global
    /// reference frame.
    ChVector<> GetSpindleAngVel(VehicleSide side) const { return m_spindle[side]->GetWvel_par(); }

    /// Get the angular speed of the axle on the specified side.
    double GetAxleSpeed(VehicleSide side) const { return m_axle[side]->GetPos_dt(); }

    /// Synchronize this suspension subsystem.
    /// This function must be called before synchronizing any wheels associated with this suspension.
    void Synchronize();

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
    virtual void Initialize(
        std::shared_ptr<ChChassis> chassis,        ///< [in] associated chassis subsystem
        std::shared_ptr<ChSubchassis> subchassis,  ///< [in] associated subchassis subsystem (may be null)
        std::shared_ptr<ChSteering> steering,      ///< [in] associated steering subsystem (may be null)
        const ChVector<>& location,                ///< [in] location relative to the chassis frame
        double left_ang_vel = 0,                   ///< [in] initial angular velocity of left wheel
        double right_ang_vel = 0                   ///< [in] initial angular velocity of right wheel
        ) = 0;

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

    /// Return current suspension forces (spring and shock) on the specified side.
    /// Different derived types (suspension templates) may load different quantities in the output struct.
    virtual Force ReportSuspensionForce(VehicleSide side) const = 0;

    /// Log current constraint violations.
    virtual void LogConstraintViolations(VehicleSide side) {}

    /// Simple model of a parking brake.
    void ApplyParkingBrake(bool brake);

  protected:
    /// Construct a suspension subsystem with given name.
    ChSuspension(const std::string& name);

    ChVector<> m_rel_loc;                                ///< location relative to chassis
    std::shared_ptr<ChBody> m_spindle[2];                ///< handles to spindle bodies
    std::shared_ptr<ChShaft> m_axle[2];                  ///< handles to axle shafts
    std::shared_ptr<ChShaftsBody> m_axle_to_spindle[2];  ///< handles to spindle-shaft connectors
    std::shared_ptr<ChLinkLockRevolute> m_revolute[2];   ///< handles to spindle revolute joints

  private:
    std::shared_ptr<ChCylinderShape> m_spindle_shapes[2];

    void AddVisualizationSpindle(VehicleSide side, double radius, double width);
};

/// Vector of handles to suspension subsystems.
typedef std::vector<std::shared_ptr<ChSuspension> > ChSuspensionList;

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
