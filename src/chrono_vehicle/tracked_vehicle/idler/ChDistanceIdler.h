// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
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
// Base class for an idler subsystem with a fixed distance tensioner.
// An idler consists of the idler wheel and a carrier body. The carrier body is
// connected to the chassis and the idler wheel to the carrier. A linear
// actuator connects the carrier body and a link body (the chassis or a
// supsension arm).
//
// An idler subsystem is defined with respect to a frame centered at the origin
// of the idler wheel.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_DISTANCE_IDLER_H
#define CH_DISTANCE_IDLER_H

#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkLinActuator.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/tracked_vehicle/ChIdler.h"

namespace chrono {
namespace vehicle {

class ChTrackAssembly;

/// @addtogroup vehicle_tracked_idler
/// @{

/// Base class for an idler subsystem with a fixed distance tensioner.
/// An idler consists of the idler wheel and a carrier body. The carrier body is connected to the chassis and the idler
/// wheel to the carrier. A linear actuator connects the carrier body and a link body (the chassis or a supsension arm).
class CH_VEHICLE_API ChDistanceIdler : public ChIdler {
  public:
    ChDistanceIdler(const std::string& name);
    virtual ~ChDistanceIdler();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "DistanceIdler"; }

    /// Return a handle to the carrier body to which the idler wheel is connected.
    virtual std::shared_ptr<ChBody> GetCarrierBody() const override { return m_carrier; }

    /// Get the tensioner motor element.
    std::shared_ptr<ChLinkLinActuator> GetTensioner() const { return m_tensioner; }

    /// Initialize this idler subsystem.
    /// The idler subsystem is initialized by attaching it to the specified chassis at the specified location (with
    /// respect to and expressed in the reference frame of the chassis). It is assumed that the idler subsystem
    /// reference frame is always aligned with the chassis reference frame. A derived idler subsystem template class
    /// must extend this default implementation and specify contact geometry for the idler wheel.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,  ///< [in] associated chassis
                            const ChVector<>& location,          ///< [in] location relative to the chassis frame
                            ChTrackAssembly* track               ///< [in] containing track assembly
    );

    /// Add visualization assets for the idler subsystem.
    /// This default implementation adds assets to the carrier body.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the idler subsystem.
    /// This default implementation removes the assets from the carrier body.
    virtual void RemoveVisualizationAssets() override;

    /// Log current constraint violations.
    void LogConstraintViolations();

  protected:
    /// Identifiers for the various hardpoints.
    enum PointId {
        CARRIER,          ///< carrier location
        CARRIER_WHEEL,    ///< carrier, connection to idler wheel
        CARRIER_CHASSIS,  ///< carrier, connection to chassis (revolute)
        MOTOR_CARRIER,    ///< motor connection to carrier
        MOTOR_ARM,        ///< motor connection to suspension arm
        NUM_POINTS
    };

    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the idler subsystem reference frame.
    virtual const ChVector<> GetLocation(PointId which) = 0;

    /// Return the mass of the carrier body.
    virtual double GetCarrierMass() const = 0;
    /// Return the moments of inertia of the carrier body.
    virtual const ChVector<>& GetCarrierInertia() = 0;
    /// Return a visualization radius for the carrier body.
    virtual double GetCarrierVisRadius() const = 0;

    /// Return the time to extend the tensioner to prescribed distance.
    virtual double GetTensionerExtensionTime() const = 0;

    /// Return the set distance in the tensioner.
    virtual double GetTensionerDistance() const = 0;

    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;

    virtual void Output(ChVehicleOutput& database) const override;

    std::shared_ptr<ChBody> m_carrier;               ///< carrier body
    std::shared_ptr<ChLinkLockRevolute> m_revolute;  ///< carrier-chassis revolute joint
    std::shared_ptr<ChLinkLinActuator> m_tensioner;  ///< linear motor tensioner element

  private:
    // Hardpoints expressed in absolute frame
    std::vector<ChVector<>> m_points;
};

/// @} vehicle_tracked_idler

}  // end namespace vehicle
}  // end namespace chrono

#endif
