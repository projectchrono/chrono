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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Definition of a suspension testing mechanism (as a vehicle).
// The tested suspension can be specified through a stand-alone JSON file (and
// may or may not include a steering subsystem), or as a specified axle in a
// vehicle JSON specification file.
//
// The reference frame follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_SUSPENSION_TEST_RIG_H
#define CH_SUSPENSION_TEST_RIG_H

#include <string>
#include <vector>

#include "chrono/assets/ChColor.h"

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"
#include "chrono_vehicle/wheeled_vehicle/ChSteering.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_utils
/// @{

/// Definition of a suspension test rig.
class CH_VEHICLE_API ChSuspensionTestRig : public ChVehicle {
  public:
    /// Construct a test rig for a specified axle of a given vehicle.
    ChSuspensionTestRig(
        const std::string& filename,  ///< JSON file with vehicle specification
        int axle_index,               ///< index of the suspension to be tested
        ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI  ///< contact method
        );

    /// Construct a test rig from specified file.
    ChSuspensionTestRig(
        const std::string& filename,  ///< JSON file with test rig specification
        ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI  ///< contact method
        );

    /// Destructor
    ~ChSuspensionTestRig() {}

    /// Set the actuator function on the left wheel
    void SetActuator_func_L(const std::shared_ptr<ChFunction>& funcL) { m_actuator_L = funcL; }

    /// Set the actuator function on the right wheel
    void SetActuator_func_R(const std::shared_ptr<ChFunction>& funcR) { m_actuator_R = funcR; }

    /// Get a handle to the specified wheel body.
    std::shared_ptr<ChBody> GetWheelBody(VehicleSide side) const { return m_suspension->GetSpindle(side); }

    /// Get the global location of the specified wheel.
    const ChVector<>& GetWheelPos(VehicleSide side) const { return m_suspension->GetSpindlePos(side); }

    /// Get the global rotation of the specified wheel.
    const ChQuaternion<>& GetWheelRot(VehicleSide side) const { return m_suspension->GetSpindleRot(side); }

    /// Get the global linear velocity of wheel.
    const ChVector<>& GetWheelLinVel(VehicleSide side) const { return m_suspension->GetSpindleLinVel(side); }

    /// Get the global angular velocity of wheel.
    ChVector<> GetWheelAngVel(VehicleSide side) const { return m_suspension->GetSpindleAngVel(side); }

    /// Get the complete state for the specified wheel.
    WheelState GetWheelState(VehicleSide side) const;

    double GetActuatorDisp(VehicleSide side);
    double GetActuatorForce(VehicleSide side);
    double GetActuatorMarkerDist(VehicleSide side);

    /// Return true if a steering system is attached.
    bool HasSteering() const { return m_steering != 0; }

    /// Get the rig total mass.
    /// This includes the mass of the suspension and wheels, and (if present) the mass of the
    /// steering mechanism.
    virtual double GetVehicleMass() const override;

    /// Get the local driver position and orientation.
    /// This is a coordinate system relative to the chassis reference frame.
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return ChCoordsys<>(); }

    /// Get a handle to the vehicle's driveshaft body.
    virtual std::shared_ptr<ChShaft> GetDriveshaft() const override { return m_dummy_shaft; }

    /// Get the angular speed of the driveshaft.
    virtual double GetDriveshaftSpeed() const override { return 0; }

    /// Initialize this chassis at the specified global location and orientation.
    virtual void Initialize(const ChCoordsys<>& chassisPos  ///< [in] initial global position and orientation
                            ) override;

    /// Update the state at the current time.
    /// steering between -1 and +1, and no force need be applied if using external actuation
    void Synchronize(double time,                   ///< [in] current time
                     double steering,               ///< [in] current steering input [-1,+1]
                     double disp_L,                 ///< [in] left post displacement
                     double disp_R,                 ///< [in] right post displacement
                     const TireForces& tire_forces  ///< [in] tires force to apply to wheel
                     );

    /// Log current constraint violations.
    virtual void LogConstraintViolations() override;

  private:
    void LoadSteering(const std::string& filename);
    void LoadSuspension(const std::string& filename);
    void LoadWheel(const std::string& filename, int side);
    static void AddVisualize_post(std::shared_ptr<ChBody> post_body,
        std::shared_ptr<ChBody> ground_body,
                                  double height,
                                  double rad,
                                  const ChColor& color);

    std::shared_ptr<ChSuspension> m_suspension;  ///< handle to suspension subsystem
    std::shared_ptr<ChSteering> m_steering;      ///< handle to the steering subsystem
    ChWheelList m_wheels;                        ///< list of handles to wheel subsystems

    std::shared_ptr<ChShaft> m_dummy_shaft;  ///< dummy driveshaft

    std::shared_ptr<ChBody> m_post_L;                         ///< left shaker post
    std::shared_ptr<ChBody> m_post_R;                         ///< right shaker post
    std::shared_ptr<ChLinkLockPrismatic> m_post_L_prismatic;  ///< left post prismatic joint
    std::shared_ptr<ChLinkLockPrismatic> m_post_R_prismatic;  ///< right post prismatic joint
    std::shared_ptr<ChLinkLinActuator> m_post_L_linact;       ///< actuate left post
    std::shared_ptr<ChLinkLinActuator> m_post_R_linact;       ///< actuate right post
    std::shared_ptr<ChLinkLockPointPlane> m_post_L_ptPlane;   ///< actuate L suspension to a specified height
    std::shared_ptr<ChLinkLockPointPlane> m_post_R_ptPlane;   ///< actuate R suspension to a specified height

    std::shared_ptr<ChFunction> m_actuator_L;  ///< actuator function applied to left wheel
    std::shared_ptr<ChFunction> m_actuator_R;  ///< actuator function applied to right wheel

    double m_steer;    ///< cached steering driver input
    double m_displ_L;  ///< cached left post displacement
    double m_displ_R;  ///< cached right post displacement

    ChVector<> m_suspLoc;
    ChVector<> m_steeringLoc;
    ChQuaternion<> m_steeringRot;
};

/// @} vehicle_wheeled_utils

}  // end namespace vehicle
}  // end namespace chrono

#endif
