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

#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/utils/ChUtilsInputOutput.h"

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
    ChTrackTestRig(const std::string& filename,                            ///< [in] JSON file with rig specification
                   bool create_track = true,                               ///< [in] include track shoes?
                   ChContactMethod contact_method = ChContactMethod::NSC,  ///< [in] contact method
                   bool detracking_control = false                         ///< [in] detracking control
    );

    /// Construct a test rig using the specified track assembly and subsystem locations.
    ChTrackTestRig(std::shared_ptr<ChTrackAssembly> assembly,              ///< [in] handle to the track assembly
                   bool create_track = true,                               ///< [in] include track shoes?
                   ChContactMethod contact_method = ChContactMethod::NSC,  ///< [in] contact method
                   bool detracking_control = false                         ///< [in] detracking control
    );

    /// Destructor
    ~ChTrackTestRig();

    /// Set driver system.
    void SetDriver(std::shared_ptr<ChDriverTTR> driver);

    /// Set the initial ride height (relative to the sprocket reference frame).
    /// If not specified, the reference height is the track assembly design configuration.
    void SetInitialRideHeight(double height) { m_ride_height = height; }

    /// Set the limits for post displacement (same for jounce and rebound).
    /// Default value: 0
    void SetDisplacementLimit(double limit) { m_displ_limit = limit; }

    /// Set time delay before applying post displacement.
    /// Default value: 0
    void SetDisplacementDelay(double delay) { m_displ_delay = delay; }

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

    /// Set filename for optional driver log
    void SetDriverLogFilename(const std::string& filename) { m_driver_logfile = filename; }

    /// Enable/disable output for the track assembly.
    /// See also ChVehicle::SetOuput.
    void SetTrackAssemblyOutput(bool state);

    /// Initialize this track test rig.
    void Initialize();

    /// Advance the state of the track test rig by the specified time step.
    virtual void Advance(double step) override;

    /// Set collision flags for the various subsystems.
    /// By default, collision is enabled for sprocket, idler, road wheels, and track shoes.
    void SetCollide(int flags) { m_collide_flags = flags; }

    /// Enable/disable collision for the rig posts (default: true).
    void SetPostCollide(bool flag);

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

    /// Log current driver inputs.
    void LogDriverInputs();

    /// Log current constraint violations.
    virtual void LogConstraintViolations() override;

    /// Enable/disable data collection for plot generation (default: false).
    /// See PlotOutput details on data collected.
    void SetPlotOutput(double output_step);

    /// Plot collected data.
    /// When called (typically at the end of the simulation loop), the collected data is saved in ASCII format in a file
    /// with the specified name (and 'txt' extension) in the specified output directory. If the Chrono::Postprocess
    /// module is available (with gnuplot support), selected plots are generated.\n
    /// Collected data includes:
    ///  - [col 1]      time (s)
    ///  - [col 2-4]    sprocket location (m)
    ///  - [col 5-7]    idler location (m)
    ///  - [col 8-10]   road wheel 1 location (m)
    ///  - [col 11-13]  road wheel 2 location (m)
    ///  - ...          ...
    virtual void PlotOutput(const std::string& out_dir, const std::string& out_name);

  private:
    // Create the rig mechanism
    void Create(bool create_track, bool detracking_control);

    // Utility function to add visualization to post bodies.
    void AddPostVisualization(std::shared_ptr<ChBody> post_body,
                              std::shared_ptr<ChBody> chassis_body,
                              const ChColor& color);

    /// Output data for all modeling components in the track test rig system.
    virtual void Output(int frame, ChVehicleOutput& database) const override;

    /// Collect data for plotting
    void CollectPlotData(double time);

    // Overrides of ChVehicle methods
    virtual void InitializeInertiaProperties() override {}
    virtual void UpdateInertiaProperties() override {}
    virtual std::string GetTemplateName() const override { return "TrackTestRig"; }
    virtual std::shared_ptr<ChShaft> GetDriveshaft() const override { return m_dummy_shaft; }
    virtual std::string ExportComponentList() const override { return ""; }
    virtual void ExportComponentList(const std::string& filename) const override {}
    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override { Initialize(); }

    std::shared_ptr<ChTrackAssembly> m_track;  ///< track assembly
    std::shared_ptr<ChShaft> m_dummy_shaft;    ///< dummy driveshaft
    int m_collide_flags;                       ///< collision flags

    std::vector<std::shared_ptr<ChBody>> m_post;                            ///< post bodies
    std::vector<std::shared_ptr<ChLinkMotorLinearPosition>> m_post_linact;  ///< post linear actuators

    std::shared_ptr<ChDriverTTR> m_driver;  ///< driver system
    double m_throttle_input;                ///< current driver throttle input
    std::vector<double> m_displ_input;      ///< current post displacement inputs
    std::string m_driver_logfile;           ///< name of optioinal driver log file

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

    std::shared_ptr<ChTrackContactManager> m_contact_manager;  ///< manager for internal contacts

    bool m_plot_output;
    double m_plot_output_step;
    double m_next_plot_output_time;
    utils::CSV_writer* m_csv;

    friend class ChTrackTestRigIrrApp;
};

/// @} vehicle_tracked_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
