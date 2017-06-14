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
// Base class for a tracked vehicle system.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_TRACKED_VEHICLE_H
#define CH_TRACKED_VEHICLE_H

#include <vector>

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackDriveline.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackContactManager.h"

/**
    @addtogroup vehicle
    @{
        @defgroup vehicle_tracked Tracked vehicles
    @}
*/

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked
/// @{

/// Base class for chrono tracked vehicle systems.
/// This class provides the interface between the vehicle system and other
/// systems (terrain, driver, etc.)
class CH_VEHICLE_API ChTrackedVehicle : public ChVehicle {
  public:
    /// Construct a vehicle system with a default ChSystem.
    ChTrackedVehicle(const std::string& name,                                                  ///< [in] vehicle name
                     ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::NSC  ///< [in] contact method
                     );

    /// Construct a vehicle system using the specified ChSystem.
    ChTrackedVehicle(const std::string& name,  ///< [in] vehicle name
                     ChSystem* system          ///< [in] containing mechanical system
                     );

    /// Destructor.
    virtual ~ChTrackedVehicle();

    /// Get the vehicle total mass.
    /// This includes the mass of the chassis and all vehicle subsystems.
    virtual double GetVehicleMass() const override;

    /// Get the specified suspension subsystem.
    std::shared_ptr<ChTrackAssembly> GetTrackAssembly(VehicleSide side) const { return m_tracks[side]; }

    /// Get a handle to the vehicle's driveline subsystem.
    std::shared_ptr<ChTrackDriveline> GetDriveline() const { return m_driveline; }

    /// Get a handle to the vehicle's driveshaft body.
    virtual std::shared_ptr<ChShaft> GetDriveshaft() const override { return m_driveline->GetDriveshaft(); }

    /// Get the angular speed of the driveshaft.
    /// This function provides the interface between a vehicle system and a
    /// powertrain system.
    virtual double GetDriveshaftSpeed() const override { return m_driveline->GetDriveshaftSpeed(); }

    /// Get the number of suspensions in the specified track assembly.
    size_t GetNumRoadWheelAssemblies(VehicleSide side) const { return m_tracks[side]->GetNumRoadWheelAssemblies(); }

    /// Get the number of shoes in the specified track assembly.
    size_t GetNumTrackShoes(VehicleSide side) const { return m_tracks[side]->GetNumTrackShoes(); }

    /// Get a handle to the specified track shoe.
    std::shared_ptr<ChTrackShoe> GetTrackShoe(VehicleSide side, size_t id) const {
        return m_tracks[side]->GetTrackShoe(id);
    }

    /// Get the complete state for the specified track shoe.
    /// This includes the location, orientation, linear and angular velocities,
    /// all expressed in the global reference frame.
    BodyState GetTrackShoeState(VehicleSide side, size_t shoe_id) const {
        return m_tracks[side]->GetTrackShoeState(shoe_id);
    }

    /// Get the complete states for all track shoes of the specified track assembly.
    /// It is assumed that the vector of body states was properly sized.
    void GetTrackShoeStates(VehicleSide side, BodyStates& states) const {
        m_tracks[side]->GetTrackShoeStates(states);
    }

    /// Set visualization type for the sprocket subsystem.
    void SetSprocketVisualizationType(VisualizationType vis);

    // Set visualization type for the idler subsystem.
    void SetIdlerVisualizationType(VisualizationType vis);

    /// Set visualization type for the suspension subsystems.
    void SetRoadWheelAssemblyVisualizationType(VisualizationType vis);

    /// Set visualization type for the road-wheel subsystems.
    void SetRoadWheelVisualizationType(VisualizationType vis);

    /// Set visualization type for the roller subsystems.
    void SetRollerVisualizationType(VisualizationType vis);

    /// Set visualization type for the track shoe subsystems.
    void SetTrackShoeVisualizationType(VisualizationType vis);

    /// Enable/disable collision for the sprocket subsystem.
    void SetSprocketCollide(bool state);

    /// Enable/disable collision for the idler subsystem.
    void SetIdlerCollide(bool state);

    /// Enable/disable collision for the road-wheel subsystems.
    void SetRoadWheelCollide(bool state);

    /// Enable/disable collision for the roller subsystems.
    void SetRollerCollide(bool state);

    /// Enable/disable collision for the track shoe subsystems.
    void SetTrackShoeCollide(bool state);

    /// Set collision flags for the various subsystems.
    /// By default, collision is enabled for chassis, sprocket, idler, road wheels, and
    /// track shoes. To override these default settings, this function must be called
    /// called after the call to Initialize(). The 'flags' argument can be any of the
    /// TrackedCollisionFlag enums, or a combination thereof (using bit-wise operators).
    void SetCollide(int flags);

    /// Enable/disable collision between the chassis and all other vehicle
    /// subsystems. This only controls collisions between the chassis and the
    /// track shoes.  All other internal collisions involving the chassis are
    /// always ignored.
    virtual void SetChassisVehicleCollide(bool state) override;

    /// Set contacts to be monitored.
    /// Contact information will be tracked for the specified subsystems.
    void MonitorContacts(int flags) { m_contacts->MonitorContacts(flags); }

    /// Turn on/off contact data collection.
    /// If enabled, contact information will be collected for all monitored subsystems.
    void SetContactCollection(bool val) { m_contacts->SetContactCollection(val); }

    /// Return true if the specified vehicle part is currently experiencing a collision.
    bool IsPartInContact(TrackedCollisionFlag::Enum part) const { return m_contacts->InContact(part); }

    /// Write contact information to file.
    /// If data collection was enabled and at least one subsystem is monitored,
    /// contact information is written (in CSV format) to the specified file.
    void WriteContacts(const std::string& filename) { m_contacts->WriteContacts(filename); }

    /// Initialize this vehicle at the specified global location and orientation.
    /// This base class implementation only initializes the chassis subsystem.
    /// Derived classes must extend this function to initialize all other tracked
    /// vehicle subsystems (the two track assemblies and the driveline).
    virtual void Initialize(const ChCoordsys<>& chassisPos,  ///< [in] initial global position and orientation
                            double chassisFwdVel = 0         ///< [in] initial chassis forward velocity
                            ) override;

    /// Update the state of this vehicle at the current time.
    /// The vehicle system is provided the current driver inputs (throttle between
    /// 0 and 1, steering between -1 and +1, braking between 0 and 1), the torque
    /// from the powertrain, and tire forces (expressed in the global reference
    /// frame).
    void Synchronize(double time,                              ///< [in] current time
                     double steering,                          ///< [in] current steering input [-1,+1]
                     double braking,                           ///< [in] current braking input [0,1]
                     double powertrain_torque,                 ///< [in] input torque from powertrain
                     const TrackShoeForces& shoe_forces_left,  ///< [in] vector of track shoe forces (left side)
                     const TrackShoeForces& shoe_forces_right  ///< [in] vector of track shoe forces (left side)
                     );

    /// Advance the state of this vehicle by the specified time step.
    virtual void Advance(double step) final;

    /// Log current constraint violations.
    virtual void LogConstraintViolations() override;

  protected:
    std::shared_ptr<ChTrackAssembly> m_tracks[2];   ///< handles to the track assemblies (left/right)
    std::shared_ptr<ChTrackDriveline> m_driveline;  ///< handle to the driveline subsystem

    ChTrackContactManager* m_contacts;  ///< manager for internal contacts

    friend class ChTrackedVehicleIrrApp;
};

/// @} vehicle_tracked

}  // end namespace vehicle
}  // end namespace chrono

#endif
