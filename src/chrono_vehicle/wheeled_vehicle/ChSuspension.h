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

#include <string>
#include <vector>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsBody.h"
#include "chrono/assets/ChCylinderShape.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Base class for a suspension subsystem.
class CH_VEHICLE_API ChSuspension : public ChPart {
  public:
    ChSuspension(const std::string& name  ///< [in] name of the subsystem
                 );

    virtual ~ChSuspension() {}

    /// Specify whether or not this suspension can be steered.
    virtual bool IsSteerable() const = 0;

    /// Specify whether or not this is an independent suspension.
    virtual bool IsIndependent() const = 0;

    /// Get the location of the suspension subsystem relative to the chassis reference frame.
    /// The suspension reference frame is always aligned with the chassis reference frame.
    const ChVector<>& GetLocation() const { return m_location; }

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
    const ChQuaternion<>& GetSpindleRot(VehicleSide side) const { return m_spindle[side]->GetRot(); }

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

    /// Get the index of the associated steering index (-1 if non-steered).
    int GetSteeringIndex() const { return m_steering_index; }

    /// Update the suspension subsystem: apply the provided tire forces.
    /// The given tire force and moment is applied to the specified (left or
    /// right) spindle body. This function provides the interface to the tire
    /// system (intermediated by the vehicle system).
    void Synchronize(
        VehicleSide side,            ///< indicates the spindle body (left or right) where the forces should be applied
        const TireForce& tire_force  ///< generalized tire forces
        );

    /// Apply the provided motor torque.
    /// The given torque is applied to the specified (left or right) axle. This
    /// function provides the interface to the drivetrain subsystem (intermediated
    /// by the vehicle system).
    void ApplyAxleTorque(VehicleSide side,  ///< indicates the axle (left or right) where the torque should be applied
                         double torque        ///< value of applied torque
                         );

    /// Initialize this suspension subsystem.
    /// The suspension subsystem is initialized by attaching it to the specified
    /// chassis body at the specified location (with respect to and expressed in
    /// the reference frame of the chassis). It is assumed that the suspension
    /// reference frame is always aligned with the chassis reference frame.
    /// 'tierod_body' is a handle to the body to which the suspension tierods
    /// are to be attached. For a steered suspension, this will be the steering
    /// (central) link of a suspension subsystem.  Otherwise, this is the chassis.
    /// If this suspension is steered, 'steering_index' indicates the index of the
    /// associated steering mechanism in the vehicle's list (-1 for a non-steered suspension).
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const ChVector<>& location,             ///< [in] location relative to the chassis frame
                            std::shared_ptr<ChBody> tierod_body,    ///< [in] body to which tireods are connected
                            int steering_index,                     ///< [in] index of the associated steering mechanism
                            double left_ang_vel = 0,                ///< [in] initial angular velocity of left wheel
                            double right_ang_vel = 0                ///< [in] initial angular velocity of right wheel
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

    /// Specify the left body for a possible antirollbar subsystem.
    /// The default implementation returns a NULL pointer.
    virtual std::shared_ptr<ChBody> GetLeftBody() const { return std::shared_ptr<ChBody>(); }

    /// Specify the right body for a possible antirollbar subsystem.
    /// The default implementation returns a NULL pointer.
    virtual std::shared_ptr<ChBody> GetRightBody() const { return std::shared_ptr<ChBody>(); }

    /// Get the total mass of the suspension subsystem.
    virtual double GetMass() const = 0;

    /// Log current constraint violations.
    virtual void LogConstraintViolations(VehicleSide side) {}

  protected:
    ChVector<> m_location;                               ///< location relative to chassis
    int m_steering_index;                                ///< index of associated steering mechanism
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
