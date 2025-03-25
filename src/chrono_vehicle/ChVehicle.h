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

#include "chrono/core/ChRealtimeStep.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleOutput.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/ChPowertrainAssembly.h"
#include "chrono_vehicle/ChTerrain.h"

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

    /// Get vehicle tag.
    /// This is a unique integral identifier of a vehicle in a Chrono simulation, automatically assigned at construction.
    uint16_t GetVehicleTag() const { return m_tag; }

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
    std::shared_ptr<ChPowertrainAssembly> GetPowertrainAssembly() const { return m_powertrain_assembly; }

    /// Get the engine in the powertrain assembly (if a powertrain is attached).
    std::shared_ptr<ChEngine> GetEngine() const;

    /// Get the transmission in the powertrain assembly (if a powertrain is attached).
    std::shared_ptr<ChTransmission> GetTransmission() const;

    /// Get the vehicle total mass.
    /// This includes the mass of the chassis and all vehicle subsystems.
    double GetMass() const { return m_mass; }

    /// Get the current vehicle COM frame (relative to and expressed in the vehicle reference frame).
    /// This is a frame aligned with the vehicle reference frame and origin at the current vehicle COM.
    const ChFrame<>& GetCOMFrame() const { return m_com; }

    /// Get the current vehicle inertia (relative to the vehicle COM frame).
    const ChMatrix33<>& GetInertia() const { return m_inertia; }

    /// Get the current vehicle reference frame.
    /// This is the same as the reference frame of the chassis.
    const ChFrameMoving<>& GetRefFrame() const { return GetChassisBody()->GetFrameRefToAbs(); }

    /// Get the current vehicle transform relative to the global frame.
    /// This is the same as the global transform of the main chassis.
    const ChFrame<>& GetTransform() const { return m_chassis->GetTransform(); }

    /// Get the vehicle global location.
    /// This is the global location of the main chassis reference frame origin.
    const ChVector3d& GetPos() const { return m_chassis->GetPos(); }

    /// Get the vehicle orientation.
    /// This is the main chassis orientation, returned as a quaternion representing a rotation with respect to the
    /// global reference frame.
    ChQuaternion<> GetRot() const { return m_chassis->GetRot(); }

    /// Get vehicle roll angle.
    /// This version returns the roll angle with respect to the absolte frame; as such, this is a proper representation
    /// of vehicle roll only on flat horizontal terrain. In the ISO frame convention, a positive roll angle corresponds
    /// to the vehicle left side lifting (e.g., in a turn to the left).
    double GetRoll() const;

    /// Get vehicle pitch angle.
    /// This version returns the pitch angle with respect to the absolte frame; as such, this is a proper representation
    /// of vehicle pitch only on flat horizontal terrain. In the ISO frame convention, a positive pitch angle
    /// corresponds to the vehicle front dipping (e.g., during braking).
    double GetPitch() const;

    /// Get vehicle roll angle (relative to local terrain).
    /// This version returns the roll angle relative to the terrain normal at a point below the vehicle position; as
    /// such, this is a reasonable approximation of local vehicle roll only on relatively flat (but not necessarily
    /// horizontal) terrains. In the ISO frame convention, a positive roll angle corresponds to the vehicle left side
    /// lifting above the terrain plane.
    double GetRoll(const ChTerrain& terrain) const;

    /// Get vehicle pitch angle (relative to local terrain).
    /// This version returns the pitch angle relative to the terrain normal at a point below the vehicle position; as
    /// such, this is a reasonable approximation of local vehicle pitch only on relatively flat (but not necessarily
    /// horizontal) terrains. In the ISO frame convention, a positive pitch angle corresponds to the vehicle front
    /// dipping below the terrain plane.
    double GetPitch(const ChTerrain& terrain) const;

    /// Get the vehicle speed (velocity component in the vehicle forward direction).
    /// Return the speed measured at the origin of the main chassis reference frame.
    double GetSpeed() const { return m_chassis->GetSpeed(); }

    /// Get the vehicle slip angle.
    /// This represents the angle betwwen the forward vehicle X axis and the vehicle velocity vector (calculated at the
    /// origin of the vehicle frame). The return value is in radians with a positive sign for a left turn and a negative
    /// sign for a right turn.
    double GetSlipAngle() const;

    /// Get the vehicle roll rate.
    /// The roll rate is referenced to the chassis frame.
    double GetRollRate() const { return m_chassis->GetRollRate(); }

    /// Get the vehicle pitch rate.
    /// The pitch rate is referenced to the chassis frame.
    double GetPitchRate() const { return m_chassis->GetPitchRate(); }

    /// Get the vehicle yaw rate.
    /// The yaw rate is referenced to the chassis frame.
    double GetYawRate() const { return m_chassis->GetYawRate(); }

    /// Get the vehicle turn rate.
    /// Unlike the yaw rate (referenced to the chassis frame), the turn rate is referenced to the global frame.
    double GetTurnRate() const { return m_chassis->GetTurnRate(); }

    /// Get the global position of the specified point.
    /// The point is assumed to be given relative to the main chassis reference frame.
    /// The returned location is expressed in the global reference frame.
    ChVector3d GetPointLocation(const ChVector3d& locpos) const { return m_chassis->GetPointLocation(locpos); }

    /// Get the global velocity of the specified point.
    /// The point is assumed to be given relative to the main chassis reference frame.
    /// The returned velocity is expressed in the global reference frame.
    ChVector3d GetPointVelocity(const ChVector3d& locpos) const { return m_chassis->GetPointVelocity(locpos); }

    /// Get the acceleration at the specified point.
    /// The point is assumed to be given relative to the main chassis reference frame.
    /// The returned acceleration is expressed in the chassis reference frame.
    ChVector3d GetPointAcceleration(const ChVector3d& locpos) const { return m_chassis->GetPointAcceleration(locpos); }

    /// Get the global location of the driver.
    ChVector3d GetDriverPos() const { return m_chassis->GetDriverPos(); }

    /// Enable/disable soft real-time (default: false).
    /// If enabled, a spinning timer is used to maintain simulation time in sync with real time. This function should be
    /// called right before the main simulation loop, since it starts the embedded ChTimer.
    void EnableRealtime(bool val);

    /// Get current estimated RTF (real time factor).
    /// Note that the "true" RTF is returned, even if soft real-time is enforced.
    /// This represents the real time factor for advancing the dynamic state of the system only and as such does not
    /// take into account any other operations performed during a step (e.g., run-time visualization). During each call
    /// to Advance(), this value is calculated as T/step_size, where T includes the time spent in system setup,
    /// collision detection, and integration.
    double GetRTF() const { return m_system->GetRTF(); }

    /// Get current estimated step RTF (real time factor).
    /// Unlike the value returned by GetRTF(), this represents the real time factor for all calculations performed
    /// during a simulation step, including any other operations in addition to advancing the dynamic state of the
    /// system (run-time visualization, I/O, etc.). This RTF value is calculated as T/step_size, where T represents the
    /// time from the previous call to Advance().
    double GetStepRTF() const { return m_RTF; }

    /// Change the default collision detection system.
    /// Note that this function should be called *before* initialization of the vehicle system in order to create
    /// consistent collision models.
    void SetCollisionSystemType(ChCollisionSystem::Type collsys_type);

    /// Enable output for this vehicle system.
    void SetOutput(ChVehicleOutput::Type type,   ///< [int] type of output DB
                   const std::string& out_dir,   ///< [in] output directory name
                   const std::string& out_name,  ///< [in] rootname of output file
                   double output_step            ///< [in] interval between output times
    );

    /// Enable output for this vehicle system using an existing output stream.
    void SetOutput(ChVehicleOutput::Type type,  ///< [int] type of output DB
                   std::ostream& out_stream,    ///< [in] output stream
                   double output_step           ///< [in] interval between output times
    );

    /// Initialize this vehicle at the specified global location and orientation.
    /// Derived classes must invoke this base class implementation after they initialize all their subsystem.
    virtual void Initialize(const ChCoordsys<>& chassisPos,  ///< [in] initial global position and orientation
                            double chassisFwdVel = 0         ///< [in] initial chassis forward velocity
    );

    /// Initialize the given powertrain assembly and associate it to this vehicle.
    /// The powertrain is initialized by connecting it to this vehicle's chassis and driveline shaft.
    void InitializePowertrain(std::shared_ptr<ChPowertrainAssembly> powertrain);

    /// Calculate total vehicle mass from subsystems.
    /// This function is called at the end of the vehicle initialization, but can also be called explicitly.
    virtual void InitializeInertiaProperties() = 0;

    /// Set visualization mode for the chassis subsystem.
    void SetChassisVisualizationType(VisualizationType vis);

    /// Set visualization mode for the rear chassis subsystems.
    void SetChassisRearVisualizationType(VisualizationType vis);

    /// Enable/disable collision for the chassis subsystem.
    /// This function controls contact of the chassis with all other collision shapes in the simulation.
    void SetChassisCollide(bool state);

    /// Enable/disable collision between the chassis and all other vehicle subsystems.
    /// Note that some of these collisions may be always disabled, as set by the particular derived vehicle class.
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
    virtual void LogConstraintViolations() {}

    /// Return a JSON string with information on all modeling components in the vehicle system.
    /// These include bodies, shafts, joints, spring-damper elements, markers, etc.
    virtual std::string ExportComponentList() const { return ""; }

    /// Write a JSON-format file with information on all modeling components in the vehicle system.
    /// These include bodies, shafts, joints, spring-damper elements, markers, etc.
    virtual void ExportComponentList(const std::string& filename) const {}

    /// Output data for all modeling components in the vehicle system.
    virtual void Output(int frame, ChVehicleOutput& database) const {}

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

    /// Calculate current vehicle inertia properties from subsystems.
    /// This function is called at the end of each vehicle state advance.
    virtual void UpdateInertiaProperties() = 0;

    /// Utility function for testing if any subsystem in a list generates output.
    template <typename T>
    static bool AnyOutput(const std::vector<std::shared_ptr<T>>& list) {
        bool val = std::accumulate(list.begin(), list.end(), false,
                                   [](bool a, std::shared_ptr<T> b) { return a || b->OutputEnabled(); });
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

    std::shared_ptr<ChPowertrainAssembly> m_powertrain_assembly;  ///< associated powertrain system

  private:
    uint16_t m_tag;                        ///< unique identifier of a vehicle in a simulation
    bool m_initialized;                    ///< initialization flag
    bool m_realtime_force;                 ///< enforce real-time (using a spinner)
    ChRealtimeStepTimer m_realtime_timer;  ///< real-time spinner
    ChTimer m_sim_timer;                   ///< timer for vehicle simulation
    double m_RTF;                          ///< current RTF value

    void SetVehicleTag();

    friend class ChVehicleCosimWheeledVehicleNode;
    friend class ChVehicleCosimTrackedVehicleNode;
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
