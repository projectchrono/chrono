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
// Base class for a wheeled trailer system.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_WHEELED_TRAILER_H
#define CH_WHEELED_TRAILER_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/wheeled_vehicle/ChAxle.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/chassis/ChChassisConnectorHitch.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled
/// @{

/// Base class for chrono wheeled trailer systems.
/// This class provides the interface between the trailer system and other
/// systems (vehicle, tires, etc.).
/// The reference frame for a trailer follows the ISO standard: Z-axis up, X-axis
/// pointing forward, and Y-axis towards the left of the vehicle.

class CH_VEHICLE_API ChWheeledTrailer {
  public:
    /// Destructor.
    virtual ~ChWheeledTrailer() {}

    /// Get the name identifier for this vehicle.
    const std::string& GetName() const { return m_name; }

    /// Set the name identifier for this vehicle.
    void SetName(const std::string& name) { m_name = name; }

    /// Return the number of axles for this trailer.
    virtual int GetNumberAxles() const = 0;

    /// Get the name of the trailer system template.
    virtual std::string GetTemplateName() const { return "WheeledTrailer"; }

    /// Get the trailer chassis subsystem.
    std::shared_ptr<ChChassisRear> GetChassis() const { return m_chassis; }

    /// Get all trailer axle subsystems.
    const chrono::vehicle::ChAxleList& GetAxles() const { return m_axles; }

    /// Get the specified trailer axle subsystem.
    std::shared_ptr<chrono::vehicle::ChAxle> GetAxle(int id) const { return m_axles[id]; }

    /// Set visualization mode for the chassis subsystem.
    /// This function should be called only after trailer initialization.
    void SetChassisVisualizationType(VisualizationType vis);

    /// Set visualization type for the suspension subsystems.
    /// This function should be called only after trailer initialization.
    void SetSuspensionVisualizationType(VisualizationType vis);

    /// Set visualization type for the wheel subsystems.
    /// This function should be called only after trailer initialization.
    void SetWheelVisualizationType(VisualizationType vis);

    /// Set visualization type for the tire subsystems.
    /// This function should be called only after trailer and tire initialization.
    void SetTireVisualizationType(VisualizationType vis);

    /// Initialize this trailer relative to the specified front chassis.
    /// This base class implementation only initializes the trailer chassis and connector subsystems.
    /// Derived classes must extend this function to initialize the axle subsystems.
    virtual void Initialize(std::shared_ptr<ChChassis> frontChassis);

    /// Initialize the given tire and attach it to the specified wheel.
    /// Optionally, specify tire visualization mode and tire-terrain collision detection method.
    /// This function should be called only after trailer initialization.
    void InitializeTire(
        std::shared_ptr<ChTire> tire,
        std::shared_ptr<ChWheel> wheel,
        VisualizationType tire_vis = VisualizationType::PRIMITIVES,
        ChTire::CollisionType tire_coll = ChTire::CollisionType::SINGLE_POINT);

    /// Update the state of this trailer at the current time.
    /// The trailer system is provided the current braking input (between 0 and 1) and a reference to the terrain
    /// system.
    void Synchronize(double time, double braking, const ChTerrain& terrain);

    /// Advance the state of this trailer by the specified time step.
    /// This function advances the states of all associated tires.
    void Advance(double step);

  protected:
    /// Construct a trailer system using the specified ChSystem.
    ChWheeledTrailer(const std::string& name,  ///< [in] trailer system name
                     ChSystem* system          ///< [in] containing mechanical system
    );

    std::string m_name;  ///< trailer system name

    std::shared_ptr<ChChassisRear> m_chassis;              ///< trailer chassis
    std::shared_ptr<ChChassisConnectorHitch> m_connector;  ///< connector to pulling vehicle
    chrono::vehicle::ChAxleList m_axles;                   ///< list of axle subsystems
};

/// @} vehicle_wheeled

}  // end namespace vehicle
}  // end namespace chrono

#endif
