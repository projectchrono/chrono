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
// Base class for a vehicle system.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_VEHICLE_H
#define CH_VEHICLE_H

#include <numeric>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleOutput.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/ChPowertrain.h"

namespace chrono {
namespace vehicle {

// Forward reference
class ChVehicleVisualSystem;

/// @addtogroup vehicle
/// @{

/// Base class for chrono vehicle systems.
/// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
/// pointing forward, and Y-axis towards the left of the vehicle.
class CH_VEHICLE_API ChVehicle {
  public:
    /// Destructor.
    virtual ~ChVehicle();

    /// Get the name identifier for this vehicle.
    const std::string& GetName() const { return m_name; }

    /// Set the name identifier for this vehicle.
    void SetName(const std::string& name) { m_name = name; }

    /// Get the name of the vehicle system template.
    virtual std::string GetTemplateName() const = 0;

    /// Get a pointer to the Chrono ChSystem.
    ChSystem* GetSystem() { return m_system; }

    /// Get the current simulation time of the underlying ChSystem.
    double GetChTime() const { return m_system->GetChTime(); }

    /// Get a handle to the vehicle's main chassis subsystem.
    std::shared_ptr<ChChassis> GetChassis() const { return m_chassis; }

    /// Get the specified specified rear chassis subsystem.
    std::shared_ptr<ChChassisRear> GetChassisRear(int id) const { return m_chassis_rear[id]; }

    /// Get a handle to the specified chassis connector.
    std::shared_ptr<ChChassisConnector> GetChassisConnector(int id) const { return m_chassis_connectors[id]; }

    /// Get a handle to the vehicle's chassis body.
    std::shared_ptr<ChBodyAuxRef> GetChassisBody() const { return m_chassis->GetBody(); }

    /// Get a handle to the specified rear chassis body.
    std::shared_ptr<ChBodyAuxRef> GetChassisRearBody(int id) const { return m_chassis_rear[id]->GetBody(); }

    /// Get the powertrain attached to this vehicle.
    virtual std::shared_ptr<ChPowertrain> GetPowertrain() const { return nullptr; }

    /// Get the vehicle total mass.
    /// This includes the mass of the chassis and all vehicle subsystems.
    double GetMass() const { return m_mass; }

    /// Get the current vehicle COM frame (relative to and expressed in the vehicle reference frame).
    /// This is a frame aligned with the vehicle reference frame and origin at the current vehicle COM.
    const ChFrame<>& GetCOMFrame() const { return m_com; }

    /// Get the current vehicle inertia (relative to the vehicle COM frame).
    const ChMatrix33<>& GetInertia() const { return m_inertia; }

    /// Get the current vehicle transform relative to the global frame.
    /// This is the same as the global transform of the main chassis.
    const ChFrame<>& GetTransform() const { return m_chassis->GetTransform(); }

    /// Get the vehicle global location.
    /// This is the global location of the main chassis reference frame origin.
    const ChVector<>& GetPos() const { return m_chassis->GetPos(); }

    /// Get the vehicle orientation.
    /// This is the main chassis orientation, returned as a quaternion representing a rotation with respect to the
    /// global reference frame.
    ChQuaternion<> GetRot() const { return m_chassis->GetRot(); }

    /// Get the vehicle speed.
    /// Return the speed measured at the origin of the main chassis reference frame.
    double GetSpeed() const { return m_chassis->GetSpeed(); }

    /// Get the global position of the specified point.
    /// The point is assumed to be given relative to the main chassis reference frame.
    /// The returned location is expressed in the global reference frame.
    ChVector<> GetPointLocation(const ChVector<>& locpos) const { return m_chassis->GetPointLocation(locpos); }

    /// Get the global velocity of the specified point.
    /// The point is assumed to be given relative to the main chassis reference frame.
    /// The returned velocity is expressed in the global reference frame.
    ChVector<> GetPointVelocity(const ChVector<>& locpos) const { return m_chassis->GetPointVelocity(locpos); }

    /// Get the acceleration at the specified point.
    /// The point is assumed to be given relative to the main chassis reference frame.
    /// The returned acceleration is expressed in the chassis reference frame.
    ChVector<> GetPointAcceleration(const ChVector<>& locpos) const { return m_chassis->GetPointAcceleration(locpos); }

    /// Get a handle to the vehicle's driveshaft body.
    virtual std::shared_ptr<ChShaft> GetDriveshaft() const = 0;

    /// Get the global location of the driver.
    ChVector<> GetDriverPos() const { return m_chassis->GetDriverPos(); }

    /// Change the default collision detection system.
    /// Note that this function should be called *before* initialization of the vehicle system in order to create
    /// consistent collision models.
    void SetCollisionSystemType(collision::ChCollisionSystemType collsys_type);

    /// Attach a vehicle visualization system.
    void SetVisualSystem(std::shared_ptr<ChVehicleVisualSystem> vsys);

    /// Get the associated visualization system (if any).
    std::shared_ptr<ChVehicleVisualSystem> GetVisualSystem() const;

    /// Enable output for this vehicle system.
    void SetOutput(ChVehicleOutput::Type type,   ///< [int] type of output DB
                   const std::string& out_dir,   ///< [in] output directory name
                   const std::string& out_name,  ///< [in] rootname of output file
                   double output_step            ///< [in] interval between output times
    );

    /// Initialize this vehicle at the specified global location and orientation.
    /// Derived classes must invoke this base class implementation after they initialize all their subsystem.
    virtual void Initialize(const ChCoordsys<>& chassisPos,  ///< [in] initial global position and orientation
                            double chassisFwdVel = 0         ///< [in] initial chassis forward velocity
                            );

    /// Set visualization mode for the chassis subsystem.
    void SetChassisVisualizationType(VisualizationType vis);

    /// Set visualization mode for the rear chassis subsystems.
    void SetChassisRearVisualizationType(VisualizationType vis);

    /// Enable/disable collision for the chassis subsystem. This function controls
    /// contact of the chassis with all other collision shapes in the simulation.
    void SetChassisCollide(bool state);

    /// Enable/disable collision between the chassis and all other vehicle
    /// subsystems. Note that some of these collisions may be always disabled,
    /// as set by the particular derived vehicle class.
    virtual void SetChassisVehicleCollide(bool state) {}

    /// Enable/disable output from the chassis subsystem.
    void SetChassisOutput(bool state);

    /// Return true if the vehicle model contains bushings.
    bool HasBushings() const { return m_chassis->HasBushings(); }

    /// Advance the state of this vehicle by the specified time step.
    /// A call to ChSystem::DoStepDynamics is done only if the vehicle owns the underlying Chrono system.
    /// Otherwise, the caller is responsible for advancing the sate of the entire system.
    virtual void Advance(double step);

    /// Log current constraint violations.
    virtual void LogConstraintViolations() = 0;

    /// Return a JSON string with information on all modeling components in the vehicle system.
    /// These include bodies, shafts, joints, spring-damper elements, markers, etc.
    virtual std::string ExportComponentList() const = 0;

    /// Write a JSON-format file with information on all modeling components in the vehicle system.
    /// These include bodies, shafts, joints, spring-damper elements, markers, etc.
    virtual void ExportComponentList(const std::string& filename) const = 0;

    /// Output data for all modeling components in the vehicle system.
    virtual void Output(int frame, ChVehicleOutput& database) const = 0;

  protected:
    /// Construct a vehicle system with an underlying ChSystem.
    ChVehicle(const std::string& name,                               ///< [in] vehicle name
              ChContactMethod contact_method = ChContactMethod::NSC  ///< [in] contact method
    );

    /// Construct a vehicle system using the specified ChSystem.
    /// All physical components of the vehicle will be added to that system.
    ChVehicle(const std::string& name,  ///< [in] vehicle name
              ChSystem* system          ///< [in] containing mechanical system
    );

    /// Set the associated Chrono system.
    void SetSystem(ChSystem* sys) { m_system = sys; }

    /// Calculate total vehicle mass from subsystems.
    /// This function is called at the end of the vehicle initialization.
    virtual void InitializeInertiaProperties() = 0;

    /// Calculate current vehicle inertia properties from subsystems.
    /// This function is called at the end of each vehicle state advance.
    virtual void UpdateInertiaProperties() = 0;

    /// Utility function for testing if any subsystem in a list generates output.
    template <typename T>
    static bool AnyOutput(const std::vector<std::shared_ptr<T>>& list) {
        bool val = std::accumulate(list.begin(), list.end(), false,
            [](bool a, std::shared_ptr<T> b) {return a || b->OutputEnabled(); });
        return val;
    }

    std::string m_name;  ///< vehicle name
    ChSystem* m_system;  ///< pointer to the Chrono system
    bool m_ownsSystem;   ///< true if system created at construction

    double m_mass;           ///< total vehicle mass
    ChFrame<> m_com;         ///< current vehicle COM (relative to the vehicle reference frame)
    ChMatrix33<> m_inertia;  ///< current total vehicle inertia (Relative to the vehicle COM frame)

    bool m_output;                 ///< generate ouput for this vehicle system
    ChVehicleOutput* m_output_db;  ///< vehicle output database
    double m_output_step;          ///< output time step
    double m_next_output_time;     ///< time for next output
    int m_output_frame;            ///< current output frame

    std::shared_ptr<ChChassis> m_chassis;         ///< handle to the main chassis subsystem
    ChChassisRearList m_chassis_rear;             ///< list of rear chassis subsystems (can be empty)
    ChChassisConnectorList m_chassis_connectors;  ///< list of chassis connector (must match m_chassis_rear)

  private:
    bool m_initialized;

    friend class ChVehicleCosimVehicleNode;
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
