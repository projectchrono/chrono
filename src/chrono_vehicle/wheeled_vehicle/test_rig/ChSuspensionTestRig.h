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
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

/**
    @addtogroup vehicle_wheeled
    @{
        @defgroup vehicle_wheeled_test_rig Suspension test rig classes
    @}
*/

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_test_rig
/// @{

/// Definition of a suspension test rig.
class CH_VEHICLE_API ChSuspensionTestRig : public ChVehicle {
  public:
    /// Construct a test rig for a specified axle of a given vehicle.
    ChSuspensionTestRig(
        const std::string& filename,         ///< JSON file with vehicle specification
        int axle_index,                      ///< index of the suspension to be tested
        double displ_limit,                  ///< limits for post displacement
        std::shared_ptr<ChTire> tire_left,   ///< left tire
        std::shared_ptr<ChTire> tire_right,  ///< right tire
        ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI  ///< contact method
        );

    /// Construct a test rig from specified file.
    ChSuspensionTestRig(
        const std::string& filename,         ///< JSON file with test rig specification
        std::shared_ptr<ChTire> tire_left,   ///< left tire
        std::shared_ptr<ChTire> tire_right,  ///< right tire
        ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI  ///< contact method
        );

    /// Destructor
    ~ChSuspensionTestRig() {}

    /// Set the limits for post displacement.
    /// Each post will move between [-val, +val].
    void SetDisplacementLimit(double val) { m_displ_limit = val; }

    /// Set the actuator function on the specified post (currently NOT USED).
    void SetActuatorFunction(VehicleSide side, const std::shared_ptr<ChFunction>& func) {
        m_actuator_func[side] = func;
    }

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

    /// Get a handle to the specified post body.
    std::shared_ptr<ChBody> GetPostBody(VehicleSide side) const { return m_post[side]; }

    /// Get the global location of the specified post body.
    const ChVector<>& GetPostPos(VehicleSide side) const { return m_post[side]->GetPos(); }

    double GetActuatorDisp(VehicleSide side);
    double GetActuatorForce(VehicleSide side);
    double GetActuatorMarkerDist(VehicleSide side);

    /// Return true if a steering system is attached.
    bool HasSteering() const { return m_steering != nullptr; }

    /// Get the rig total mass.
    /// This includes the mass of the suspension and wheels, and (if present) the mass of the
    /// steering mechanism.
    virtual double GetVehicleMass() const override;

    /// Get a handle to the vehicle's driveshaft body.
    virtual std::shared_ptr<ChShaft> GetDriveshaft() const override { return m_dummy_shaft; }

    /// Get the angular speed of the driveshaft.
    virtual double GetDriveshaftSpeed() const override { return 0; }

    /// Initialize this chassis at the specified global location and orientation.
    virtual void Initialize(const ChCoordsys<>& chassisPos,  ///< [in] initial global position and orientation
                            double chassisFwdVel = 0         ///< [in] initial forward velocity (ignored)
                            ) override;

    /// Set visualization type for the suspension subsystem.
    void SetSuspensionVisualizationType(VisualizationType vis);

    /// Set visualization type for the steering subsystems.
    void SetSteeringVisualizationType(VisualizationType vis);

    /// Set visualization type for the wheel subsystems.
    void SetWheelVisualizationType(VisualizationType vis);

    /// Set visualization type for the tire subsystems.
    void SetTireVisualizationType(VisualizationType vis);

    /// Update the state at the current time.
    /// steering between -1 and +1, and no force need be applied if using external actuation
    void Synchronize(double time,      ///< [in] current time
                     double steering,  ///< [in] current steering input [-1,+1]
                     double disp_L,    ///< [in] left post displacement
                     double disp_R     ///< [in] right post displacement
                     );

    /// Advance the state of the suspension test rig by the specified time step.
    /// Note that this function also advances the tire states.
    virtual void Advance(double step) override;

    /// Log current constraint violations.
    virtual void LogConstraintViolations() override;

  private:
    /// Definition of a terrain object for use by a suspension test rig.
    class Terrain : public ChTerrain {
      public:
        Terrain();
        virtual double GetHeight(double x, double y) const override;
        virtual ChVector<> GetNormal(double x, double y) const override;
        double m_height_L;
        double m_height_R;
    };

    /// Utility functions to load subsystems from JSON files.
    void LoadSteering(const std::string& filename);
    void LoadSuspension(const std::string& filename);
    void LoadWheel(const std::string& filename, int side);

    /// Utility function to add visualization to post bodies.
    void AddVisualize_post(VehicleSide side, const ChColor& color);

    std::shared_ptr<ChSuspension> m_suspension;  ///< handle to suspension subsystem
    std::shared_ptr<ChSteering> m_steering;      ///< handle to the steering subsystem
    std::shared_ptr<ChShaft> m_dummy_shaft;      ///< dummy driveshaft
    std::shared_ptr<ChWheel> m_wheel[2];         ///< handles to wheel subsystems
    std::shared_ptr<ChTire> m_tire[2];           ///< handles to tire subsystems

    std::shared_ptr<ChBody> m_post[2];                         ///< handles to post bodies
    std::shared_ptr<ChLinkLockPrismatic> m_post_prismatic[2];  ///< handles to post prismatic joints
    std::shared_ptr<ChLinkLinActuator> m_post_linact[2];       ///< handles to post linear actuators
    std::shared_ptr<ChFunction> m_actuator_func[2];            ///< actuator functions applied to left/right posts

    double m_displ_limit;  ///< scale factor for post displacement

    Terrain m_terrain;  ///< terrain object to provide height to the tires

    ChVector<> m_suspLoc;
    ChVector<> m_steeringLoc;
    ChQuaternion<> m_steeringRot;

    static const double m_post_radius;  ///< radius of the post cylindrical platform
    static const double m_post_height;  ///< height of the post cylindrical platform
};

/// @} vehicle_wheeled_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
