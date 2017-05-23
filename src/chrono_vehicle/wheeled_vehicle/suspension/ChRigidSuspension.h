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
// Base class for a rigid suspension, i.e. spindles directly attached to the
// vehicle chassis through revolute joints.
//
// The suspension subsystem is modeled with respect to a right-handed frame,
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
// The suspension reference frame is assumed to be always aligned with that of
// the vehicle.  When attached to a chassis, only an offset is provided.
//
// All point locations are assumed to be given for the left half of the
// supspension and will be mirrored (reflecting the y coordinates) to construct
// the right side.
//
// =============================================================================

#ifndef CH_RIGID_SUSPENSION_H
#define CH_RIGID_SUSPENSION_H

#include <vector>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Base class for a rigid suspension, i.e. spindles directly attached to the
/// vehicle chassis through revolute joints.
///
/// The suspension subsystem is modeled with respect to a right-handed frame,
/// with X pointing towards the front, Y to the left, and Z up (ISO standard).
/// The suspension reference frame is assumed to be always aligned with that of
/// the vehicle.  When attached to a chassis, only an offset is provided.
///
/// All point locations are assumed to be given for the left half of the
/// supspension and will be mirrored (reflecting the y coordinates) to construct
/// the right side.
class CH_VEHICLE_API ChRigidSuspension : public ChSuspension {
  public:
    ChRigidSuspension(const std::string& name  ///< [in] name of the subsystem
                      );

    virtual ~ChRigidSuspension() {}

    /// Specify whether or not this suspension can be steered.
    virtual bool IsSteerable() const final override { return false; }

    /// Specify whether or not this is an independent suspension.
    virtual bool IsIndependent() const final override { return false; }

    /// Initialize this suspension subsystem.
    /// The suspension subsystem is initialized by attaching it to the specified
    /// chassis body at the specified location (with respect to and expressed in
    /// the reference frame of the chassis). It is assumed that the suspension
    /// reference frame is always aligned with the chassis reference frame.
    /// Finally, tierod_body is a handle to the body to which the suspension
    /// tierods are to be attached. For a steerable suspension, this will be the
    /// steering link of a suspension subsystem.  Otherwise, this is the chassis.
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const ChVector<>& location,             ///< [in] location relative to the chassis frame
                            std::shared_ptr<ChBody> tierod_body,    ///< [in] body to which tireods are connected
                            double left_ang_vel = 0,                ///< [in] initial angular velocity of left wheel
                            double right_ang_vel = 0                ///< [in] initial angular velocity of right wheel
                            ) override;

    /// Get the total mass of the suspension subsystem.
    virtual double GetMass() const override;

    /// Log current constraint violations.
    virtual void LogConstraintViolations(VehicleSide side) override;

  protected:
    /// Identifiers for the various hardpoints.
    enum PointId {
        SPINDLE,  ///< spindle location
        NUM_POINTS
    };

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the suspension reference frame.
    virtual const ChVector<> getLocation(PointId which) = 0;

    /// Return the mass of the spindle body.
    virtual double getSpindleMass() const = 0;

    /// Return the moments of inertia of the spindle body.
    virtual const ChVector<>& getSpindleInertia() const = 0;

    /// Return the inertia of the axle shaft.
    virtual double getAxleInertia() const = 0;

  private:
    // Hardpoint absolute locations
    std::vector<ChVector<>> m_pointsL;
    std::vector<ChVector<>> m_pointsR;

    void InitializeSide(VehicleSide side,
                        std::shared_ptr<ChBodyAuxRef> chassis,
                        std::shared_ptr<ChBody> tierod_body,
                        const std::vector<ChVector<>>& points,
                        double ang_vel);
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
