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
// Base class for a tracked vehicle system.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_TRACKED_VEHICLE_H
#define CH_TRACKED_VEHICLE_H

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/tracked_vehicle/ChDrivelineTV.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackContactManager.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked
/// @{

/// Base class for chrono tracked vehicle systems.
/// This class provides the interface between the vehicle system and other
/// systems (terrain, driver, etc.)
class CH_VEHICLE_API ChTrackedVehicle : public ChVehicle {
  public:
    /// Destructor.
    virtual ~ChTrackedVehicle();

    /// Get the name of the vehicle system template.
    virtual std::string GetTemplateName() const override { return "TrackedVehicle"; }

    /// Get the specified suspension subsystem.
    std::shared_ptr<ChTrackAssembly> GetTrackAssembly(VehicleSide side) const { return m_tracks[side]; }

    /// Get a handle to the vehicle's driveline subsystem.
    std::shared_ptr<ChDrivelineTV> GetDriveline() const { return m_driveline; }

    /// Get the number of suspensions in the specified track assembly.
    size_t GetNumTrackSuspensions(VehicleSide side) const { return m_tracks[side]->GetNumTrackSuspensions(); }

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

    /// Set visualization type for the idler subsystem.
    void SetIdlerVisualizationType(VisualizationType vis);

    /// Set visualization type for the suspension subsystems.
    void SetSuspensionVisualizationType(VisualizationType vis);

    /// Set visualization type for the idler wheel subsystem.
    void SetIdlerWheelVisualizationType(VisualizationType vis);

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

    /// Enable user-defined contact forces between track shoes and idlers and/or road-wheels and/or ground. By default,
    /// contact forces are generated by the underlying Chrono contact processing. If enabled, no contact forces are
    /// applied automatically for specified collision types. Instead, these collisions are cached and passed to the
    /// user-supplied callback which must compute the contact force for each individual collision.
    void EnableCustomContact(std::shared_ptr<ChTrackCustomContact> callback);

    /// Set contacts to be monitored.
    /// Contact information will be tracked for the specified subsystems.
    void MonitorContacts(int flags) { m_contact_manager->MonitorContacts(flags); }

    /// Render normals of all monitored contacts.
    void SetRenderContactNormals(bool val) { m_contact_manager->SetRenderNormals(val); }

    /// Render forces of all monitored contacts.
    void SetRenderContactForces(bool val, double scale) { m_contact_manager->SetRenderForces(val, scale); }

    /// Turn on/off contact data collection.
    /// If enabled, contact information will be collected for all monitored subsystems.
    void SetContactCollection(bool val) { m_contact_manager->SetContactCollection(val); }

    /// Return true if the specified vehicle part is currently experiencing a collision.
    bool IsPartInContact(TrackedCollisionFlag::Enum part) const { return m_contact_manager->InContact(part); }

    /// Return estimated resistive torque on the specified sprocket.
    /// This torque is available only if monitoring of contacts for that sprocket is enabled.
    ChVector<> GetSprocketResistiveTorque(VehicleSide side) const {
        return m_contact_manager->GetSprocketResistiveTorque(side);
    }

    /// Write contact information to file.
    /// If data collection was enabled and at least one subsystem is monitored,
    /// contact information is written (in CSV format) to the specified file.
    void WriteContacts(const std::string& filename) { m_contact_manager->WriteContacts(filename); }

    /// Enable/disable output for the track assemblies.
    /// See also ChVehicle::SetOuput.
    void SetTrackAssemblyOutput(VehicleSide side, bool state);

    /// Initialize this vehicle at the specified global location and orientation.
    /// This base class implementation only initializes the main chassis subsystem.
    /// Derived classes must extend this function to initialize all other tracked
    /// vehicle subsystems (the two track assemblies and the driveline).
    virtual void Initialize(const ChCoordsys<>& chassisPos,  ///< [in] initial global position and orientation
                            double chassisFwdVel = 0         ///< [in] initial chassis forward velocity
                            ) override;

    /// Calculate total vehicle mass.
    /// This function is called at the end of the vehicle initialization, but can also be called explicitly.
    virtual void InitializeInertiaProperties() override final;

    /// Update the state of this vehicle at the current time.
    /// The vehicle system is provided the current driver inputs (throttle between 0 and 1, steering between -1 and +1,
    /// braking between 0 and 1).
    void Synchronize(double time,                       ///< [in] current time
                     const DriverInputs& driver_inputs  ///< [in] current driver inputs
    );

    /// Update the state of this vehicle at the current time.
    /// This version can be used in a co-simulation framework and it provides the terrain forces on the track shoes
    /// (assumed to be expressed in the global reference frame).
    void Synchronize(double time,                            ///< [in] current time
                     const DriverInputs& driver_inputs,      ///< [in] current driver inputs
                     const TerrainForces& shoe_forces_left,  ///< [in] vector of track shoe forces (left side)
                     const TerrainForces& shoe_forces_right  ///< [in] vector of track shoe forces (left side)
    );

    /// Advance the state of this vehicle by the specified time step.
    /// In addition to advancing the state of the multibody system (if the vehicle owns the underlying system), this
    /// function also advances the state of the associated powertrain.
    virtual void Advance(double step) override final;

    /// Lock/unlock the differential (if available).
    void LockDifferential(bool lock);

    /// Disconnect driveline.
    /// This function has no effect if called before vehicle initialization.
    void DisconnectDriveline();

    /// Log current constraint violations.
    virtual void LogConstraintViolations() override;

    /// Return a JSON string with information on all modeling components in the vehicle system.
    /// These include bodies, shafts, joints, spring-damper elements, markers, etc.
    virtual std::string ExportComponentList() const override;

    /// Write a JSON-format file with information on all modeling components in the vehicle system.
    /// These include bodies, shafts, joints, spring-damper elements, markers, etc.
    virtual void ExportComponentList(const std::string& filename) const override;

  protected:
    /// Construct a vehicle system with a default ChSystem.
    ChTrackedVehicle(const std::string& name,                               ///< [in] vehicle name
                     ChContactMethod contact_method = ChContactMethod::NSC  ///< [in] contact method
    );

    /// Construct a vehicle system using the specified ChSystem.
    ChTrackedVehicle(const std::string& name,  ///< [in] vehicle name
                     ChSystem* system          ///< [in] containing mechanical system
    );

    /// Calculate current vehicle inertia properties.
    /// This function is called at the end of each vehicle state advance.
    virtual void UpdateInertiaProperties() override final;

    /// Output data for all modeling components in the vehicle system.
    virtual void Output(int frame, ChVehicleOutput& database) const override;

    std::shared_ptr<ChTrackAssembly> m_tracks[2];  ///< track assemblies (left/right)
    std::shared_ptr<ChDrivelineTV> m_driveline;    ///< driveline subsystem

    std::shared_ptr<ChTrackCollisionManager> m_collision_manager;  ///< manager for internal collisions
    std::shared_ptr<ChTrackContactManager> m_contact_manager;      ///< manager for internal contacts

    friend class ChTrackedVehicleVisualSystemIrrlicht;
};

/// @} vehicle_tracked

}  // end namespace vehicle
}  // end namespace chrono

#endif
