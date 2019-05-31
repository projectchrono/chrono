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
// Definition of a track testing mechanism (as a vehicle).
// The tested track can be specified through a stand-alone JSON file or as a
// specified track assembly in a vehicle JSON specification file.
//
// The reference frame follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_TRACK_TEST_RIG_H
#define CH_TRACK_TEST_RIG_H

#include <string>
//
#include "chrono/physics/ChLinkMotorLinearPosition.h"
//
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChDriverTTR.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_test_rig
/// @{

/// Definition of a suspension test rig.
class CH_VEHICLE_API ChTrackTestRig : public ChVehicle {
  public:
    /// Default constructor.
    ChTrackTestRig() : ChVehicle("TrackTestRig") {}

    /// Construct a test rig from specified track assembly JSON file.
    ChTrackTestRig(const std::string& filename,  ///< [in] JSON file with test rig specification
                   bool create_track = true,     ///< [in] include track shoes?
                   ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::NSC  ///< [in] contact method
    );

    /// Construct a test rig using the specified track assembly and subsystem locations.
    ChTrackTestRig(std::shared_ptr<ChTrackAssembly> assembly,  ///< [in] handle to the track assembly
                   bool create_track = true,                   ///< [in] include track shoes?
                   ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::NSC  ///< [in] contact method
    );

    /// Destructor
    ~ChTrackTestRig() {}

    /// Set driver system.
    void SetDriver(std::unique_ptr<ChDriverTTR> driver);


    /// Set the initial ride height (relative to the sprocket reference frame).
    /// If not specified, the reference height is the track assembly design configuration.
    void SetInitialRideHeight(double height) { m_ride_height = height; }

    /// Set the limits for post displacement (same for jounce and rebound).
    void SetDisplacementLimit(double limit) { m_displ_limit = limit; }

    /// Set maximum sprocket torque
    void SetMaxTorque(double val) { m_max_torque = val; }

    /// Set visualization type for the sprocket subsystem (default: PRIMITIVES).
    void SetSprocketVisualizationType(VisualizationType vis) { m_vis_sprocket = vis; }

    /// Set visualization type for the idler subsystem (default: PRIMITIVES).
    void SetIdlerVisualizationType(VisualizationType vis) { m_vis_idler = vis; }

    /// Set visualization type for the road-wheel assembly subsystem (default: PRIMITIVES).
    void SetRoadWheelAssemblyVisualizationType(VisualizationType vis) { m_vis_roadwheel_assembly = vis; }

    /// Set visualization type for the road-wheel subsystem (default: PRIMITIVES).
    void SetRoadWheelVisualizationType(VisualizationType vis) { m_vis_roadwheel = vis; }

    /// Set visualization type for the track shoe subsystems (default: PRIMITIVES).
    void SetTrackShoeVisualizationType(VisualizationType vis) { m_vis_shoe = vis; }

    /// Initialize this track test rig.
    void Initialize();

    /// Advance the state of the track test rig by the specified time step.
    virtual void Advance(double step) override;

    /// Set collision flags for the various subsystems.
    /// By default, collision is enabled for sprocket, idler, road wheels, and track shoes.
    void SetCollide(int flags) { m_collide_flags = flags; }

    double GetThrottleInput() const { return m_throttle_input; }
    double GetDisplacementInput(int index) { return m_displ_input[index]; }
    std::string GetDriverMessage() const { return m_driver->GetInfoMessage(); }

    double GetActuatorDisp(int index);
    double GetActuatorForce(int index);
    double GetActuatorMarkerDist(int index);

    /// Get the rig total mass.
    /// This is simply the mass of the track subsystem.
    double GetMass() const;

    /// Get the track assembly subsystem.
    std::shared_ptr<ChTrackAssembly> GetTrackAssembly() const { return m_track; }

    /// Get current ride height (relative to the chassis reference frame).
    /// This estimate uses the average of the left and right posts.
    double GetRideHeight() const;

    /// Log current constraint violations.
    virtual void LogConstraintViolations() override;

  private:
    // Create the rig mechanism
    void Create();

    // Utility function to add visualization to post bodies.
    void AddPostVisualization(std::shared_ptr<ChBody> post_body,
                              std::shared_ptr<ChBody> chassis_body,
                              const ChColor& color);

    // Overrides of ChVehicle methods
    virtual std::string GetTemplateName() const override { return "TrackTestRig"; }
    virtual std::shared_ptr<ChShaft> GetDriveshaft() const override { return m_dummy_shaft; }
    virtual double GetDriveshaftSpeed() const override { return 0; }
    virtual double GetVehicleMass() const override { return GetMass(); }
    virtual ChVector<> GetVehicleCOMPos() const override { return ChVector<>(0, 0, 0); }
    virtual void Output(int frame, ChVehicleOutput& database) const override {}
    virtual std::string ExportComponentList() const override { return ""; }
    virtual void ExportComponentList(const std::string& filename) const override {}
    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override { Initialize(); }

    std::shared_ptr<ChTrackAssembly> m_track;  ///< track assembly
    std::shared_ptr<ChShaft> m_dummy_shaft;    ///< dummy driveshaft
    bool m_create_track;                       ///< include track shoes?
    int m_collide_flags;                       ///< collision flags

    std::vector<std::shared_ptr<ChBody>> m_post;                            ///< post bodies
    std::vector<std::shared_ptr<ChLinkMotorLinearPosition>> m_post_linact;  ///< post linear actuators

    std::unique_ptr<ChDriverTTR> m_driver;  ///< driver system
    double m_throttle_input;                ///< current driver throttle input
    std::vector<double> m_displ_input;      ///< current post displacement inputs

    double m_ride_height;         ///< ride height
    double m_displ_offset;        ///< post displacement offset (to set reference position)
    double m_displ_delay;         ///< time interval for assuming reference position
    double m_displ_limit;         ///< scale factor for post displacement

    double m_max_torque;  ///< maximum torque applied to sprocket

    VisualizationType m_vis_sprocket;
    VisualizationType m_vis_idler;
    VisualizationType m_vis_roadwheel_assembly;
    VisualizationType m_vis_roadwheel;
    VisualizationType m_vis_shoe;

    double m_post_radius;   ///< radius of the post cylindrical platform
    double m_post_hheight;  ///< half-height of the post cylindrical platform
};

/// @} vehicle_tracked_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
