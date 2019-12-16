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
// Base class for a wheeled vehicle system.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_WHEELED_VEHICLE_H
#define CH_WHEELED_VEHICLE_H

#include <vector>

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChAxle.h"
#include "chrono_vehicle/wheeled_vehicle/ChAntirollBar.h"
#include "chrono_vehicle/wheeled_vehicle/ChBrake.h"
#include "chrono_vehicle/wheeled_vehicle/ChDrivelineWV.h"
#include "chrono_vehicle/wheeled_vehicle/ChSteering.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled
/// @{

/// Base class for chrono wheeled vehicle systems.
/// This class provides the interface between the vehicle system and other
/// systems (tires, driver, etc.).
/// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
/// pointing forward, and Y-axis towards the left of the vehicle.
class CH_VEHICLE_API ChWheeledVehicle : public ChVehicle {
  public:
    /// Construct a vehicle system with a default ChSystem.
    ChWheeledVehicle(const std::string& name,                                                  ///< [in] vehicle name
                     ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::NSC  ///< [in] contact method
                     );

    /// Construct a vehicle system using the specified ChSystem.
    ChWheeledVehicle(const std::string& name,  ///< [in] vehicle name
                     ChSystem* system          ///< [in] containing mechanical system
                     );

    /// Destructor.
    virtual ~ChWheeledVehicle() {}

    /// Get the name of the vehicle system template.
    virtual std::string GetTemplateName() const override { return "WheeledVehicle"; }

    /// Get the powertrain attached to this vehicle.
    virtual std::shared_ptr<ChPowertrain> GetPowertrain() const override { return m_powertrain; }

    /// Get all vehicle axle subsystems.
    const ChAxleList& GetAxles() const { return m_axles; }

    /// Get the specified vehicle axle subsystem.
    std::shared_ptr<ChAxle> GetAxle(int id) const { return m_axles[id]; }

    /// Get the specified suspension subsystem.
    std::shared_ptr<ChSuspension> GetSuspension(int id) const { return m_axles[id]->m_suspension; }

    /// Get the specified steering subsystem.
    std::shared_ptr<ChSteering> GetSteering(int id) const { return m_steerings[id]; }

    /// Get the specified vehicle wheel, by specifying the axle, side, and wheel location.
    /// Axles are assumed to be indexed starting from the front of the vehicle. On each axle, wheels are assumed to be
    /// ordered from inner to outer wheels, first left then right: for a single-wheel axle the order is left wheel,
    /// right wheel; for a double-wheel axle, the order is inner left, inner right, outer left, outer right.
    std::shared_ptr<ChWheel> GetWheel(int axle, VehicleSide side, WheelLocation location = SINGLE) const;

    /// Get a handle to the vehicle's driveline subsystem.
    std::shared_ptr<ChDrivelineWV> GetDriveline() const { return m_driveline; }

    /// Get the vehicle total mass.
    /// This includes the mass of the chassis and all vehicle subsystems, but not the mass of tires.
    virtual double GetVehicleMass() const override;

    /// Get the current global vehicle COM location.
    virtual ChVector<> GetVehicleCOMPos() const override;

    /// Get a handle to the vehicle's driveshaft body.
    virtual std::shared_ptr<ChShaft> GetDriveshaft() const override { return m_driveline->GetDriveshaft(); }

    /// Get the angular speed of the driveshaft.
    /// This function provides the interface between a vehicle system and a
    /// powertrain system.
    virtual double GetDriveshaftSpeed() const override;

    /// Return the number of axles for this vehicle.
    virtual int GetNumberAxles() const = 0;

    /// Get the global location of the specified spindle.
    const ChVector<>& GetSpindlePos(int axle, VehicleSide side) const;

    /// Get the orientation of the specified spindle.
    /// Return a quaternion representing a rotation with respect to the global reference frame.
    const ChQuaternion<>& GetSpindleRot(int axle, VehicleSide side) const;

    /// Get the linear velocity of the specified spindle.
    /// Return the linear velocity of the spindle center, expressed in the global reference frame.
    const ChVector<>& GetSpindleLinVel(int axle, VehicleSide side) const;

    /// Get the angular velocity of the specified spindle.
    /// Return the angular velocity of the spindle frame, expressed in the global reference frame.
    ChVector<> GetSpindleAngVel(int axle, VehicleSide side) const;

    /// Get the angular speed of the specified spindle.
    /// This is the angular speed of the spindle shaft.
    double GetSpindleOmega(int axle, VehicleSide side) const;

    /// Return the vehicle wheelbase.
    virtual double GetWheelbase() const  = 0;

    /// Return the vehicle wheel track of the specified suspension subsystem.
    double GetWheeltrack(int id) const { return m_axles[id]->m_suspension->GetTrack(); }

    /// Return the minimum turning radius.
    /// A concrete wheeled vehicle class should override the default value (20 m).
    virtual double GetMinTurningRadius() const { return 20; }

    /// Return the maximum steering angle.
    /// This default implementation estimates the maximum steering angle based on a bicycle model
    /// and the vehicle minimum turning radius.
    virtual double GetMaxSteeringAngle() const;

    /// Initialize the given tire and attach it to the specified wheel.
    /// Optionally, specify tire visualization mode and tire-terrain collision detection method.
    /// This function should be called only after vehicle initialization.
    void InitializeTire(std::shared_ptr<ChTire> tire,
                        std::shared_ptr<ChWheel> wheel,
                        VisualizationType tire_vis = VisualizationType::PRIMITIVES,
                        ChTire::CollisionType tire_coll = ChTire::CollisionType::SINGLE_POINT);

    /// Initialize the given powertrain system and associate it to this vehicle.
    /// The powertrain is initialized by connecting it to this vehicle's chassis and driveline shaft.
    void InitializePowertrain(std::shared_ptr<ChPowertrain> powertrain);

    /// Set visualization type for the suspension subsystems.
    /// This function should be called only after vehicle initialization.
    void SetSuspensionVisualizationType(VisualizationType vis);

    /// Set visualization type for the steering subsystems.
    /// This function should be called only after vehicle initialization.
    void SetSteeringVisualizationType(VisualizationType vis);

    /// Set visualization type for the wheel subsystems.
    /// This function should be called only after vehicle initialization.
    void SetWheelVisualizationType(VisualizationType vis);

    /// Enable/disable collision between the chassis and all other vehicle subsystems.
    /// This only controls collisions between the chassis and the tire systems.
    virtual void SetChassisVehicleCollide(bool state) override;

    /// Enable/disable output from the suspension subsystems.
    /// See also ChVehicle::SetOuput.
    void SetSuspensionOutput(int id, bool state);

    /// Enable/disable output from the steering subsystems.
    /// See also ChVehicle::SetOuput.
    void SetSteeringOutput(int id, bool state);

    /// Enable/disable output from the anti-roll bar subsystems.
    /// See also ChVehicle::SetOuput.
    void SetAntirollbarOutput(int id, bool state);

    /// Enable/disable output from the driveline subsystem.
    /// See also ChVehicle::SetOuput.
    void SetDrivelineOutput(bool state);

    /// Update the state of this vehicle at the current time.
    /// The vehicle system is provided the current driver inputs (throttle between 0 and 1, steering between -1 and +1,
    /// braking between 0 and 1), and a reference to the terrain system.
    virtual void Synchronize(double time,                            ///< [in] current time
                             const ChDriver::Inputs& driver_inputs,  ///< [in] current driver inputs
                             const ChTerrain& terrain                ///< [in] reference to the terrain system
    );

    /// Advance the state of this vehicle by the specified time step.
    /// In addition to advancing the state of the multibody system (if the vehicle owns the underlying system), this
    /// function also advances the state of the associated powertrain and the states of all associated tires.
    virtual void Advance(double step) override final;

    /// Lock/unlock the differential on the specified axle.
    /// By convention, axles are counted front to back, starting with index 0.
    void LockAxleDifferential(int axle, bool lock);

    /// Lock/unlock the specified central differential.
    /// By convention, central differentials are counted from front to back, starting with index 0 for
    /// the central differential between the two front-most axles.
    void LockCentralDifferential(int which, bool lock);

    /// Log current constraint violations.
    virtual void LogConstraintViolations() override;

    /// Return a JSON string with information on all modeling components in the vehicle system.
    /// These include bodies, shafts, joints, spring-damper elements, markers, etc.
    virtual std::string ExportComponentList() const override;

    /// Write a JSON-format file with information on all modeling components in the vehicle system.
    /// These include bodies, shafts, joints, spring-damper elements, markers, etc.
    virtual void ExportComponentList(const std::string& filename) const override;

  protected:
    /// Output data for all modeling components in the vehicle system.
    virtual void Output(int frame, ChVehicleOutput& database) const override;

    ChAxleList m_axles;                          ///< list of axle subsystems
    ChSteeringList m_steerings;                  ///< list of steering subsystems
    std::shared_ptr<ChDrivelineWV> m_driveline;  ///< driveline subsystem
    std::shared_ptr<ChPowertrain> m_powertrain;  ///< associated powertrain system
};

/// @} vehicle_wheeled

}  // end namespace vehicle
}  // end namespace chrono

#endif
