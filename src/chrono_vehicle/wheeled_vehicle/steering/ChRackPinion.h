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
// Base class for a Rack-Pinion steering subsystem.
// Derived from ChSteering, but still an abstract base class.
//
// The steering subsystem is modeled with respect to a right-handed frame with
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
// The steering link translates along the Y axis. We do not explicitly model the
// pinion but instead use the implied rack-pinion constraint to calculate the
// rack displacement from a given pinion rotation angle.
//
// =============================================================================

#ifndef CH_RACKPINION_H
#define CH_RACKPINION_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChSteering.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_steering
/// @{

/// Base class for a Rack-Pinion steering subsystem.
/// Derived from ChSteering, but still an abstract base class.
///
/// The steering subsystem is modeled with respect to a right-handed frame with
/// with X pointing towards the front, Y to the left, and Z up (ISO standard).
/// The steering link translates along the Y axis. We do not explicitly model the
/// pinion but instead use the implied rack-pinion constraint to calculate the
/// rack displacement from a given pinion rotation angle.
class CH_VEHICLE_API ChRackPinion : public ChSteering {
  public:
    /// Construct a rack-pinion steering mechanism with given base name.
    ChRackPinion(const std::string& name  ///< [in] name of the subsystem
                 );

    virtual ~ChRackPinion();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "RackPinion"; }

    /// Initialize this steering subsystem.
    /// The steering subsystem is initialized by attaching it to the specified chassis at the specified location (with
    /// respect to and expressed in the reference frame of the chassis) and with specified orientation (with respect to
    /// the chassis reference frame).
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,  ///< [in] associated chassis subsystem
                            const ChVector<>& location,          ///< [in] location relative to the chassis frame
                            const ChQuaternion<>& rotation       ///< [in] orientation relative to the chassis frame
                            ) override;

    /// Add visualization assets for the steering subsystem.
    /// This default implementation uses primitives.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the steering subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Update the state of this steering subsystem at the current time.
    /// The steering subsystem is provided the current steering driver input (a
    /// value between -1 and +1).  Positive steering input indicates steering
    /// to the left. This function is called during the vehicle update.
    virtual void Synchronize(double time,     ///< [in] current time
                             double steering  ///< [in] current steering input [-1,+1]
                             ) override;

    /// Log current constraint violations.
    virtual void LogConstraintViolations() override;

  protected:
    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

    /// Return the mass of the steering link.
    virtual double GetSteeringLinkMass() const = 0;

    /// Return the moments of inertia of the steering link.
    virtual ChVector<> GetSteeringLinkInertia() const = 0;

    /// Return the steering link COM offset in Y direction (positive to the left).
    virtual double GetSteeringLinkCOM() const = 0;

    /// Return the radius of the steering link (visualization only).
    virtual double GetSteeringLinkRadius() const = 0;

    /// Return the length of the steering link (visualization only).
    virtual double GetSteeringLinkLength() const = 0;

    /// Return the radius of the pinion.
    virtual double GetPinionRadius() const = 0;

    /// Return the maximum rotation angle of the pinion (in either direction).
    virtual double GetMaxAngle() const = 0;

    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;

    virtual void Output(ChVehicleOutput& database) const override;

    std::shared_ptr<ChLinkLockPrismatic> m_prismatic;  ///< handle to the prismatic joint chassis-link
    std::shared_ptr<ChLinkLinActuator> m_actuator;     ///< handle to the linear actuator on steering link
};

/// @} vehicle_wheeled_steering

}  // end namespace vehicle
}  // end namespace chrono

#endif
