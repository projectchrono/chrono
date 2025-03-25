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
// Base classes for the chassis vehicle subsystems:
//  ChChassis          - base class for a main chassis
//  ChChassisRear      - base class for a rear chassis
//  ChChassisConnector - base class for rear chassis connectors
//
// =============================================================================

#ifndef CH_CHASSIS_H
#define CH_CHASSIS_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLoadContainer.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"

namespace chrono {
namespace vehicle {

class ChVehicle;

/// @addtogroup vehicle
/// @{

// -----------------------------------------------------------------------------

/// Base class for the chassis vehicle subsystem.
/// ChChassis is the base class for a main vehicle chassis (a single chassis or a front chassis).
class CH_VEHICLE_API ChChassis : public ChPart {
  public:
    virtual ~ChChassis();

    /// Get the tag of the associated vehicle.
    /// This method can only be called after initialization of the chassis subsystem.
    virtual uint16_t GetVehicleTag() const override;

    /// Get the local driver position and orientation.
    /// This is a coordinate system relative to the chassis reference frame.
    virtual ChCoordsys<> GetLocalDriverCoordsys() const = 0;

    /// Get the location (in the local frame of this chassis) of the connection to a rear chassis.
    virtual const ChVector3d GetLocalPosRearConnector() const { return ChVector3d(0); }

    /// Get a handle to the vehicle's chassis body.
    std::shared_ptr<ChBodyAuxRef> GetBody() const { return m_body; }

    /// Get a pointer to the containing system.
    ChSystem* GetSystem() const { return m_body->GetSystem(); }

    /// Get the global location of the chassis reference frame origin.
    const ChVector3d& GetPos() const;

    /// Get the orientation of the chassis reference frame.
    /// Returns a rotation with respect to the global reference frame.
    ChQuaternion<> GetRot() const;

    /// Get the global location of the driver.
    ChVector3d GetDriverPos() const;

    /// Get the vehicle speed.
    /// Return the speed measured at the origin of the chassis reference frame.
    double GetSpeed() const;

    /// Get the speed of the chassis COM.
    /// Return the speed measured at the chassis center of mass.
    double GetCOMSpeed() const;

    /// Get the roll rate of the chassis.
    /// The yaw rate is referenced to the chassis frame.
    double GetRollRate() const;

    /// Get the pitch rate of the chassis.
    /// The yaw rate is referenced to the chassis frame.
    double GetPitchRate() const;

    /// Get the yaw rate of the chassis.
    /// The yaw rate is referenced to the chassis frame.
    double GetYawRate() const;

    /// Get the turn rate of the chassis.
    /// Unlike the yaw rate (referenced to the chassis frame), the turn rate is referenced to the global frame.
    double GetTurnRate() const;

    /// Get the global position of the specified point.
    /// The point is assumed to be given relative to the chassis reference frame.
    /// The returned location is expressed in the global reference frame.
    ChVector3d GetPointLocation(const ChVector3d& locpos) const;

    /// Get the global velocity of the specified point.
    /// The point is assumed to be given relative to the chassis reference frame.
    /// The returned velocity is expressed in the global reference frame.
    ChVector3d GetPointVelocity(const ChVector3d& locpos) const;

    /// Get the acceleration at the specified point.
    /// The point is assumed to be given relative to the chassis reference frame.
    /// The returned acceleration is expressed in the chassis reference frame.
    ChVector3d GetPointAcceleration(const ChVector3d& locpos) const;

    /// Initialize the chassis at the specified global position and orientation.
    /// The initial position and forward velocity are assumed to be given in the current world frame.
    void Initialize(ChVehicle* vehicle,              ///< [in] containing vehicle
                    const ChCoordsys<>& chassisPos,  ///< [in] absolute chassis position
                    double chassisFwdVel,            ///< [in] initial chassis forward velocity
                    int collision_family = 0         ///< [in] chassis collision family
    );

    /// Enable/disable contact for the chassis.
    /// This function controls contact of the chassis with all other collision shapes in the simulation. Must be called
    /// after initialization and has effect only if the derived object has defined some collision shapes.
    virtual void EnableCollision(bool state) = 0;

    /// Set the "fixed to ground" status of the chassis body.
    void SetFixed(bool val) { m_body->SetFixed(val); }

    /// Return true if the chassis body is fixed to ground.
    bool IsFixed() const { return m_body->IsFixed(); }

    /// Return true if the vehicle model contains bushings.
    bool HasBushings() const { return m_container_bushings->GetNumLoads() > 0; }

    /// Add a marker on the chassis body at the specified position (relative to the chassis reference frame).
    /// If called before initialization, this function has no effect.
    void AddMarker(const std::string& name,  ///< [in] marker name
                   const ChFrame<>& frame    ///< [in] marker position relative to chassis reference frame
    );

    const std::vector<std::shared_ptr<ChMarker>>& GetMarkers() const { return m_markers; }

    /// Set parameters and enable aerodynamic drag force calculation.
    /// By default, aerodynamic drag force calculation is disabled.
    void SetAerodynamicDrag(double Cd,          ///< [in] drag coefficient
                            double area,        ///< [in] reference area
                            double air_density  ///< [in] air density
    );

    /// Update the state of the chassis subsystem.
    /// The base class implementation applies all defined external forces to the chassis body.
    virtual void Synchronize(double time);

    /// Utility function to add a joint (kinematic or bushing) to the vehicle system.
    void AddJoint(std::shared_ptr<ChVehicleJoint> joint);

    /// Utility function to remove a joint (kinematic or bushing) from the vehicle system.
    static void RemoveJoint(std::shared_ptr<ChVehicleJoint> joint);

    /// Base class for a user-defined custom force/torque acting on the chassis body.
    class ExternalForceTorque {
      public:
        ExternalForceTorque(const std::string& name = "") { m_name = name; }
        virtual ~ExternalForceTorque() {}

        /// The external load is updated at each vehicle synchronization.
        /// A derived class must load the current values for the external force and its application point on the chassis
        /// body, both assumed to be provided in the chassis body local frame.
        virtual void Update(double time,
                            const ChChassis& chassis,
                            ChVector3d& force,
                            ChVector3d& point,
                            ChVector3d& torque) {}

        std::string m_name;
    };

    /// Utility force to add an external load to the chassis body.
    void AddExternalForceTorque(std::shared_ptr<ExternalForceTorque> load);

    /// Add a terrain load to the vehicle system.
    /// Terrain forces and torques (generated by the vehicle-terrain interface) are managed through a load container
    /// maintained on the vehicle chassis.
    void AddTerrainLoad(std::shared_ptr<ChLoadBase> terrain_load);

    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

  protected:
    /// Construct a chassis subsystem with the specified name.
    ChChassis(const std::string& name,  ///< [in] name of the subsystem
              bool fixed = false        ///< [in] is the chassis body fixed to ground?
    );

    /// Construct the concrete chassis at the specified global position and orientation.
    /// The initial position and forward velocity are assumed to be given in the current world frame.
    virtual void Construct(ChVehicle* vehicle,              ///< [in] containing vehicle
                           const ChCoordsys<>& chassisPos,  ///< [in] absolute chassis position
                           double chassisFwdVel,            ///< [in] initial chassis forward velocity
                           int collision_family             ///< [in] chassis collision family
                           ) = 0;

    virtual double GetBodyMass() const = 0;
    virtual ChFrame<> GetBodyCOMFrame() const = 0;
    virtual ChMatrix33<> GetBodyInertia() const = 0;

    ChVehicle* m_vehicle;                                                ///< associated vehicle
    std::shared_ptr<ChBodyAuxRef> m_body;                                ///< handle to the chassis body
    std::shared_ptr<ChLoadContainer> m_container_bushings;               ///< load container for vehicle bushings
    std::shared_ptr<ChLoadContainer> m_container_external;               ///< load container for external forces
    std::shared_ptr<ChLoadContainer> m_container_terrain;                ///< load container for terrain forces
    std::vector<std::shared_ptr<ExternalForceTorque>> m_external_loads;  ///< external loads
    std::vector<std::shared_ptr<ChMarker>> m_markers;                    ///< list of user-defined markers
    bool m_fixed;                                                        ///< is the chassis body fixed to ground?

    friend class ChChassisRear;
};

// -----------------------------------------------------------------------------

/// Base class for a rear chassis vehicle subsystem.
/// A rear chassis is attached (through one of the provided ChassisConnector templates) to another chassis.
/// As such, a rear chassis is initialized with a position relative to the other chassis.
class CH_VEHICLE_API ChChassisRear : public ChChassis {
  public:
    virtual ~ChChassisRear() {}

    /// Get the location (in the local frame of this chassis) of the connection to the front chassis.
    virtual const ChVector3d& GetLocalPosFrontConnector() const = 0;

    /// Initialize the rear chassis relative to the specified front chassis.
    void Initialize(std::shared_ptr<ChChassis> chassis,  ///< [in] front chassis
                    int collision_family = 0             ///< [in] chassis collision family
    );

  protected:
    /// Construct a rear chassis subsystem with the specified name.
    ChChassisRear(const std::string& name);

    /// Construct the concrete rear chassis relative to the specified front chassis.
    /// The orientation is set to be the same as that of the front chassis while the location is based on the connector
    /// position on the front and rear chassis.
    virtual void Construct(std::shared_ptr<ChChassis> chassis,  ///< [in] front chassis
                           int collision_family = 0             ///< [in] chassis collision family
                           ) = 0;

  private:
    /// No driver in a rear chassis.
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override final { return ChCoordsys<>(); }

    /// A rear chassis cannot be initialized as a root body.
    virtual void Construct(ChVehicle* vehicle,
                           const ChCoordsys<>& chassisPos,
                           double chassisFwdVel,
                           int collision_family = 0) override final {}
};

/// Vector of handles to rear chassis subsystems.
typedef std::vector<std::shared_ptr<ChChassisRear>> ChChassisRearList;

// -----------------------------------------------------------------------------

/// Base class for a chassis connector subsystem.
class CH_VEHICLE_API ChChassisConnector : public ChPart {
  public:
    ChChassisConnector(const std::string& name);

    virtual ~ChChassisConnector() {}

    /// Initialize this chassis connector subsystem.
    /// The subsystem is initialized by attaching it to the specified front and rear chassis bodies at the specified
    /// location (with respect to and expressed in the reference frame of the front chassis).
    virtual void Initialize(std::shared_ptr<ChChassis> front,    ///< [in] front chassis
                            std::shared_ptr<ChChassisRear> rear  ///< [in] rear chassis
    );

    /// Update the state of this connector subsystem at the current time.
    /// The connector subsystem is provided the current steering driver input (a value between -1 and +1).  Positive
    /// steering input indicates steering to the left. This function is called during the vehicle update. The default
    /// implementation is no-op.
    virtual void Synchronize(double time,                       ///< [in] current time
                             const DriverInputs& driver_inputs  ///< [in] current driver inputs
    ) {}

  private:
    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;
};

/// Vector of handles to rear chassis subsystems.
typedef std::vector<std::shared_ptr<ChChassisConnector>> ChChassisConnectorList;

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
