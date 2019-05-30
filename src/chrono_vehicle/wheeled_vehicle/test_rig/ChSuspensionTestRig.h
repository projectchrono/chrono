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
// Definition of a suspension testing mechanism (as a vehicle).
// The tested suspension can be specified:
// - through a stand-alone JSON file (may or may not include a steering subsystem)
// - as a specified axle in a vehicle JSON specification file
// - as a specified axle in an existing vehicle (which must have been initialized)
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
#include "chrono/physics/ChLinkMotorLinearPosition.h"
//
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChDriverSTR.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_test_rig
/// @{

/// Definition of a suspension test rig.
class CH_VEHICLE_API ChSuspensionTestRig : public ChVehicle {
  public:
    /// Construct a test rig for a specified axle of a given vehicle.
    /// This version uses a concrete vehicle object.
    ChSuspensionTestRig(ChWheeledVehicle& vehicle,           ///< vehicle source
                        int axle_index,                      ///< index of the suspension to be tested
                        double displ_limit,                  ///< limits for post displacement
                        std::shared_ptr<ChTire> tire_left,   ///< left tire
                        std::shared_ptr<ChTire> tire_right,  ///< right tire
                        ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::NSC  ///< contact method
    );

    /// Construct a test rig for a specified axle of a given vehicle.
    /// This version assumes the vehicle is specified through a JSON file.
    ChSuspensionTestRig(const std::string& filename,         ///< JSON file with vehicle specification
                        int axle_index,                      ///< index of the suspension to be tested
                        double displ_limit,                  ///< limits for post displacement
                        std::shared_ptr<ChTire> tire_left,   ///< left tire
                        std::shared_ptr<ChTire> tire_right,  ///< right tire
                        ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::NSC  ///< contact method
    );

    /// Construct a test rig from specified (JSON) file.
    ChSuspensionTestRig(const std::string& filename,         ///< JSON file with test rig specification
                        std::shared_ptr<ChTire> tire_left,   ///< left tire
                        std::shared_ptr<ChTire> tire_right,  ///< right tire
                        ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::NSC  ///< contact method
    );

    /// Destructor
    ~ChSuspensionTestRig() {}

    /// Set driver system.
    void SetDriver(std::unique_ptr<ChDriverSTR> driver);

    /// Set the initial ride height (relative to the chassis reference frame).
    /// If not specified, the reference height is the suspension design configuration.
    void SetInitialRideHeight(double height) { m_ride_height = height; }

    /// Set the limits for post displacement (same for jounce and rebound).
    void SetDisplacementLimit(double limit) { m_displ_limit = limit; }

    /// Set visualization type for the suspension subsystem (default: PRIMITIVES).
    void SetSuspensionVisualizationType(VisualizationType vis) { m_vis_suspension = vis; }

    /// Set visualization type for the steering subsystems (default: PRIMITIVES).
    void SetSteeringVisualizationType(VisualizationType vis) { m_vis_steering = vis; }

    /// Set visualization type for the wheel subsystems (default: NONE).
    void SetWheelVisualizationType(VisualizationType vis) { m_vis_wheel = vis; }

    /// Set visualization type for the tire subsystems (default: PRIMITIVES).
    void SetTireVisualizationType(VisualizationType vis) { m_vis_tire = vis; }

    /// Initialize this suspension test rig.
    void Initialize();

    /// Advance the state of the suspension test rig by the specified time step.
    virtual void Advance(double step) override;

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

    /// Get the angular speed of the specified wheel (about its rotation axis).
    double GetWheelOmega(VehicleSide side) const;

    /// Get the complete state for the specified wheel.
    WheelState GetWheelState(VehicleSide side) const;

    double GetSteeringInput() const { return m_steering_input; }
    double GetDisplacementLeftInput() const { return m_left_input; }
    double GetDisplacementRightInput() const { return m_right_input; }

    double GetActuatorDisp(VehicleSide side);
    double GetActuatorForce(VehicleSide side);
    double GetActuatorMarkerDist(VehicleSide side);

    /// Return true if a steering system is attached.
    bool HasSteering() const { return m_steering != nullptr; }

    /// Return true if an anti-roll bar system is attached.
    bool HasAntirollbar() const { return m_antirollbar != nullptr; }

    /// Get the suspension subsystem.
    std::shared_ptr<ChSuspension> GetSuspension() const { return m_suspension; }

    /// Get the steering subsystem.
    std::shared_ptr<ChSteering> GetSteering() const { return m_steering; }

    /// Get the anti-roll bar subsystem.
    std::shared_ptr<ChAntirollBar> GetAntirollBar() const { return m_antirollbar; }

    /// Get a handle to the specified wheel subsystem.
    std::shared_ptr<ChWheel> GetWheel(VehicleSide side) const { return m_wheel[side]; }

    /// Get the rig total mass.
    /// This includes the mass of the suspension and wheels, and (if present) the mass of the
    /// steering mechanism.
    double GetMass() const;

    /// Get the tire force and moment on the specified side.
    const TerrainForce& GetTireForce(VehicleSide side) const { return m_tireforce[side]; }

    /// Get current ride height (relative to the chassis reference frame).
    /// This estimate uses the average of the left and right posts.
    double GetRideHeight() const;

    /// Log current constraint violations.
    virtual void LogConstraintViolations() override;

  private:
    // Definition of a terrain object for use by a suspension test rig.
    class Terrain : public ChTerrain {
      public:
        Terrain();
        virtual double GetHeight(double x, double y) const override;
        virtual ChVector<> GetNormal(double x, double y) const override;
        virtual float GetCoefficientFriction(double x, double y) const override;
        double m_height_L;
        double m_height_R;
    };

    // Overrides of ChVehicle methods
    virtual std::string GetTemplateName() const override { return "SuspensionTestRig"; }
    virtual std::shared_ptr<ChShaft> GetDriveshaft() const override { return m_dummy_shaft; }
    virtual double GetDriveshaftSpeed() const override { return 0; }
    virtual ChVector<> GetVehicleCOMPos() const override { return ChVector<>(0, 0, 0); }
    virtual void Output(int frame, ChVehicleOutput& database) const override {}
    virtual std::string ExportComponentList() const override { return ""; }
    virtual void ExportComponentList(const std::string& filename) const override {}
    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override { Initialize(); }
    virtual double GetVehicleMass() const override { return GetMass(); }
    
    // Create the rig mechanism
    void Create();

    // Utility function to add visualization to post bodies.
    void AddVisualize_post(VehicleSide side, const ChColor& color);

    std::shared_ptr<ChSuspension> m_suspension;    ///< suspension subsystem
    std::shared_ptr<ChSteering> m_steering;        ///< steering subsystem
    std::shared_ptr<ChAntirollBar> m_antirollbar;  ///< anti-roll bar subsystem
    std::shared_ptr<ChShaft> m_dummy_shaft;        ///< dummy driveshaft
    std::shared_ptr<ChWheel> m_wheel[2];           ///< wheel subsystems
    std::shared_ptr<ChTire> m_tire[2];             ///< tire subsystems

    std::shared_ptr<ChBody> m_post[2];                            ///< post bodies
    std::shared_ptr<ChLinkMotorLinearPosition> m_post_linact[2];  ///< post linear actuators

    std::unique_ptr<ChDriverSTR> m_driver;  ///< driver system
    double m_steering_input;                ///< current driver steering input
    double m_left_input;                    ///< current driver left post displacement input
    double m_right_input;                   ///< current driver right post displacement input

    double m_ride_height;         ///< ride height
    double m_displ_offset;        ///< post displacement offset (to set reference position)
    double m_displ_delay;         ///< time interval for assuming reference position
    double m_displ_limit;         ///< scale factor for post displacement
    Terrain m_terrain;            ///< terrain object to provide height to the tires
    TerrainForce m_tireforce[2];  ///< tire-terrain forces (left / right)

    ChVector<> m_suspLoc;          ///< suspension location (relative to chassis)
    ChVector<> m_steeringLoc;      ///< steering location (relative to chassis)
    ChQuaternion<> m_steeringRot;  ///< steering orientation (relative to chassis)
    ChVector<> m_antirollbarLoc;   ///< antiroll bar location (relative to chassis)

    VisualizationType m_vis_steering;
    VisualizationType m_vis_suspension;
    VisualizationType m_vis_wheel;
    VisualizationType m_vis_tire;

    static const double m_post_radius;   ///< radius of the post cylindrical platform
    static const double m_post_hheight;  ///< half-height of the post cylindrical platform
};

/// @} vehicle_wheeled_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
