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
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChDriverSTR.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_test_rig
/// @{

/// Base class for a suspension test rig.
class CH_VEHICLE_API ChSuspensionTestRig : public ChVehicle {
  public:
    /// Destructor
    virtual ~ChSuspensionTestRig();

    /// Set driver system.
    void SetDriver(std::shared_ptr<ChDriverSTR> driver);

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

    /// Enable/disable output from the suspension subsystem.
    /// See also ChVehicle::SetOuput.
    void SetSuspensionOutput(bool state);

    /// Enable/disable output from the steering subsystem (if one exists).
    /// See also ChVehicle::SetOuput.
    void SetSteeringOutput(bool state);

    /// Enable/disable output from the anti-roll bar subsystem (if one exists).
    /// See also ChVehicle::SetOuput.
    void SetAntirollbarOutput(bool state);

    /// Initialize this suspension test rig.
    void Initialize();

    /// Advance the state of the suspension test rig by the specified time step.
    virtual void Advance(double step) override;

    /// Get the global location of the specified spindle.
    const ChVector<>& GetSpindlePos(VehicleSide side) const { return m_suspension->GetSpindlePos(side); }

    /// Get the global rotation of the specified spindle.
    ChQuaternion<> GetSpindleRot(VehicleSide side) const { return m_suspension->GetSpindleRot(side); }

    /// Get the linear velocity of the specified spindle (expressed in the global reference frame).
    const ChVector<>& GetSpindleLinVel(VehicleSide side) const { return m_suspension->GetSpindleLinVel(side); }

    /// Get the angular velocity of the specified spindle (expressed in the global reference frame).
    ChVector<> GetSpindleAngVel(VehicleSide side) const { return m_suspension->GetSpindleAngVel(side); }

    double GetSteeringInput() const { return m_steering_input; }
    double GetLeftInput() const { return m_left_input; }
    double GetRightInput() const { return m_right_input; }

    virtual double GetActuatorDisp(VehicleSide side) = 0;
    virtual double GetActuatorForce(VehicleSide side) = 0;

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
    TerrainForce ReportTireForce(VehicleSide side) const;

    /// Get the suspension forces on the specified side.
    ChSuspension::Force ReportSuspensionForce(VehicleSide side) const;

    /// Get wheel travel, defined here as the vertical displacement of the spindle center from its position at
    /// the initial specified ride height (or initial configuration, if an initial ride height is not specified).
    double GetWheelTravel(VehicleSide side) const;

    /// Get current ride height (relative to the chassis reference frame).
    /// This estimate uses the average of the left and right posts.
    virtual double GetRideHeight() const = 0;

    /// Log current constraint violations.
    virtual void LogConstraintViolations() override;

    /// Enable/disable data collection for plot generation (default: false).
    /// See ChSuspensionTestRigPlatform::plotOutput and ChSuspensionTestRigPushrod::plotOutput for details on data
    /// collected for a specific type of test rig.
    void SetPlotOutput(double output_step);

    /// Plot collected data.
    /// When called (typically at the end of the simulation loop), the collected data is saved in ASCII format in a file
    /// with the specified name (and 'txt' extension) in the specified output directory. If the Chrono::Postprocess
    /// module is available (with gnuplot support), selected plots are generated.
    virtual void PlotOutput(const std::string& out_dir, const std::string& out_name) = 0;

  protected:
    /// Construct a test rig for a specified axle of a given vehicle.
    /// This version uses a concrete vehicle object.
    ChSuspensionTestRig(ChWheeledVehicle& vehicle,                             ///< vehicle source
                        int axle_index,                                        ///< index of the suspension to be tested
                        double displ_limit,                                    ///< limits for post displacement
                        std::shared_ptr<ChTire> tire_left,                     ///< left tire
                        std::shared_ptr<ChTire> tire_right,                    ///< right tire
                        ChContactMethod contact_method = ChContactMethod::NSC  ///< contact method
    );

    /// Construct a test rig for a specified axle of a given vehicle.
    /// This version assumes the vehicle is specified through a JSON file.
    ChSuspensionTestRig(const std::string& filename,                           ///< JSON file with vehicle specification
                        int axle_index,                                        ///< index of the suspension to be tested
                        double displ_limit,                                    ///< limits for post displacement
                        std::shared_ptr<ChTire> tire_left,                     ///< left tire
                        std::shared_ptr<ChTire> tire_right,                    ///< right tire
                        ChContactMethod contact_method = ChContactMethod::NSC  ///< contact method
    );

    /// Construct a test rig from specified (JSON) file.
    ChSuspensionTestRig(const std::string& filename,         ///< JSON file with test rig specification
                        std::shared_ptr<ChTire> tire_left,   ///< left tire
                        std::shared_ptr<ChTire> tire_right,  ///< right tire
                        ChContactMethod contact_method = ChContactMethod::NSC  ///< contact method
    );

    /// Initialize all underlying vehicle subsystems
    void InitializeSubsystems();

    /// Output data for all modeling components in the suspension test rig system.
    virtual void Output(int frame, ChVehicleOutput& database) const override;

    // Calculate required actuator displacement offset to impose specified ride height
    virtual double CalcDisplacementOffset() = 0;

    // Calculate current terrain height
    virtual double CalcTerrainHeight(VehicleSide side) = 0;

    // Update actuators at current time
    virtual void UpdateActuators(double displ_left,
                                 double displ_speed_left,
                                 double displ_right,
                                 double displ_speed_right) = 0;

    // Collect data for plot output.
    virtual void CollectPlotData(double time) = 0;

    std::shared_ptr<ChSuspension> m_suspension;    ///< suspension subsystem
    std::shared_ptr<ChSteering> m_steering;        ///< steering subsystem
    std::shared_ptr<ChAntirollBar> m_antirollbar;  ///< anti-roll bar subsystem
    std::shared_ptr<ChShaft> m_dummy_shaft;        ///< dummy driveshaft
    std::shared_ptr<ChWheel> m_wheel[2];           ///< wheel subsystems
    std::shared_ptr<ChTire> m_tire[2];             ///< tire subsystems

    double m_ride_height;   ///< ride height
    double m_displ_offset;  ///< post displacement offset (to set reference position)
    double m_displ_delay;   ///< time interval for assuming reference position
    double m_displ_limit;   ///< scale factor for post displacement

    bool m_plot_output;
    double m_plot_output_step;
    double m_next_plot_output_time;
    utils::CSV_writer* m_csv;

  private:
    // Overrides of ChVehicle methods
    virtual std::string GetTemplateName() const override { return "SuspensionTestRig"; }
    virtual std::shared_ptr<ChShaft> GetDriveshaft() const override { return m_dummy_shaft; }
    virtual double GetDriveshaftSpeed() const override { return 0; }
    virtual ChVector<> GetVehicleCOMPos() const override { return ChVector<>(0, 0, 0); }
    virtual std::string ExportComponentList() const override { return ""; }
    virtual void ExportComponentList(const std::string& filename) const override {}
    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override { Initialize(); }
    virtual double GetVehicleMass() const override { return GetMass(); }

    std::shared_ptr<ChDriverSTR> m_driver;  ///< driver system
    double m_steering_input;                ///< current driver steering input
    double m_left_input;                    ///< current driver left post displacement input
    double m_right_input;                   ///< current driver right post displacement input

    double m_spindle_ref[2];  ///< reference spindle vertical positions (for wheel travel calculation)

    std::unique_ptr<ChTerrain> m_terrain;  ///< terrain object to provide height to the tires

    ChVector<> m_suspLoc;          ///< suspension location (relative to chassis)
    ChVector<> m_steeringLoc;      ///< steering location (relative to chassis)
    ChQuaternion<> m_steeringRot;  ///< steering orientation (relative to chassis)
    ChVector<> m_antirollbarLoc;   ///< antiroll bar location (relative to chassis)

    VisualizationType m_vis_steering;
    VisualizationType m_vis_suspension;
    VisualizationType m_vis_wheel;
    VisualizationType m_vis_tire;
};

// -----------------------------------------------------------------------------

/// Definition of a suspension test rig using platforms to actuate the tires.
class CH_VEHICLE_API ChSuspensionTestRigPlatform : public ChSuspensionTestRig {
  public:
    /// Construct a test rig for a specified axle of a given vehicle.
    /// This version uses a concrete vehicle object.
    ChSuspensionTestRigPlatform(ChWheeledVehicle& vehicle,           ///< vehicle source
                                int axle_index,                      ///< index of the suspension to be tested
                                double displ_limit,                  ///< limits for post displacement
                                std::shared_ptr<ChTire> tire_left,   ///< left tire
                                std::shared_ptr<ChTire> tire_right,  ///< right tire
                                ChContactMethod contact_method = ChContactMethod::NSC  ///< contact method
    );

    /// Construct a test rig for a specified axle of a given vehicle.
    /// This version assumes the vehicle is specified through a JSON file.
    ChSuspensionTestRigPlatform(const std::string& filename,         ///< JSON file with vehicle specification
                                int axle_index,                      ///< index of the suspension to be tested
                                double displ_limit,                  ///< limits for post displacement
                                std::shared_ptr<ChTire> tire_left,   ///< left tire
                                std::shared_ptr<ChTire> tire_right,  ///< right tire
                                ChContactMethod contact_method = ChContactMethod::NSC  ///< contact method
    );

    /// Construct a test rig from specified (JSON) file.
    ChSuspensionTestRigPlatform(const std::string& filename,         ///< JSON file with test rig specification
                                std::shared_ptr<ChTire> tire_left,   ///< left tire
                                std::shared_ptr<ChTire> tire_right,  ///< right tire
                                ChContactMethod contact_method = ChContactMethod::NSC  ///< contact method
    );

    virtual double GetActuatorDisp(VehicleSide side) override;
    virtual double GetActuatorForce(VehicleSide side) override;

    /// Get current ride height (relative to the chassis reference frame).
    /// This estimate uses the average of the left and right posts.
    virtual double GetRideHeight() const override;

    /// Plot collected data.
    /// When called (typically at the end of the simulation loop), the collected data is saved in ASCII format in a file
    /// with the specified name (and 'txt' extension) in the specified output directory. If the Chrono::Postprocess
    /// module is available (with gnuplot support), selected plots are generated.\n
    /// Collected data includes:
    ///  - [col 1]      time (s)
    ///  - [col 2-5]    left input, left spindle z (m), left spindle z velocity (m/s), left wheel travel (m)
    ///  - [col 6-7]    left spring force (N), left shock force (N)
    ///  - [col 8-11]   right input, right spindle z (m), right spindle z velocity (m/s), right wheel travel (m)
    ///  - [col 12-13]  right spring force (N), right shock force (N)
    ///  - [col 14]     estimated ride height (m)
    ///  - [col 15]     left camber angle, gamma (deg)
    ///  - [col 16]     right camber angle, gamma (deg)
    ///  - [col 17-19]  application point for left tire force (m)
    ///  - [col 20-22]  left tire force (N)
    ///  - [col 23-25]  left tire moment (Nm)
    ///  - [col 26-28]  application point for right tire force (m)
    ///  - [col 29-31]  right tire force (N)
    ///  - [col 32-34]  right tire moment (Nm)
    /// Tire forces are expressed in the global frame, as applied to the center of the associated wheel.
    virtual void PlotOutput(const std::string& out_dir, const std::string& out_name) override;

  private:
    void Create();
    virtual double CalcDisplacementOffset() override;
    virtual double CalcTerrainHeight(VehicleSide side) override;
    virtual void UpdateActuators(double displ_left,
                                 double displ_speed_left,
                                 double displ_right,
                                 double displ_speed_right) override;
    virtual void CollectPlotData(double time) override;

    void AddPostVisualization(VehicleSide side, const ChColor& color);

    std::shared_ptr<ChBody> m_post[2];                            ///< post bodies
    std::shared_ptr<ChLinkMotorLinearPosition> m_post_linact[2];  ///< post linear actuators

    static const double m_post_radius;   ///< radius of the post cylindrical platform
    static const double m_post_hheight;  ///< half-height of the post cylindrical platform
};

// -----------------------------------------------------------------------------

/// Definition of a suspension test rig with direct actuation on the spindle bodies.
class CH_VEHICLE_API ChSuspensionTestRigPushrod : public ChSuspensionTestRig {
  public:
    /// Construct a test rig for a specified axle of a given vehicle.
    /// This version uses a concrete vehicle object.
    ChSuspensionTestRigPushrod(ChWheeledVehicle& vehicle,           ///< vehicle source
                               int axle_index,                      ///< index of the suspension to be tested
                               double displ_limit,                  ///< limits for post displacement
                               std::shared_ptr<ChTire> tire_left,   ///< left tire
                               std::shared_ptr<ChTire> tire_right,  ///< right tire
                               ChContactMethod contact_method = ChContactMethod::NSC  ///< contact method
    );

    /// Construct a test rig for a specified axle of a given vehicle.
    /// This version assumes the vehicle is specified through a JSON file.
    ChSuspensionTestRigPushrod(const std::string& filename,         ///< JSON file with vehicle specification
                               int axle_index,                      ///< index of the suspension to be tested
                               double displ_limit,                  ///< limits for post displacement
                               std::shared_ptr<ChTire> tire_left,   ///< left tire
                               std::shared_ptr<ChTire> tire_right,  ///< right tire
                               ChContactMethod contact_method = ChContactMethod::NSC  ///< contact method
    );

    /// Construct a test rig from specified (JSON) file.
    ChSuspensionTestRigPushrod(const std::string& filename,         ///< JSON file with test rig specification
                               std::shared_ptr<ChTire> tire_left,   ///< left tire
                               std::shared_ptr<ChTire> tire_right,  ///< right tire
                               ChContactMethod contact_method = ChContactMethod::NSC  ///< contact method
    );

    virtual double GetActuatorDisp(VehicleSide side) override;
    virtual double GetActuatorForce(VehicleSide side) override;

    /// Get current ride height (relative to the chassis reference frame).
    /// This estimate uses the average of the left and right posts.
    virtual double GetRideHeight() const override;

    /// Plot collected data.
    /// When called (typically at the end of the simulation loop), the collected data is saved in ASCII format in a file
    /// with the specified name (and 'txt' extension) in the specified output directory. If the Chrono::Postprocess
    /// module is available (with gnuplot support), selected plots are generated.\n
    /// Collected data includes:
    ///  - [col 1]      time (s)
    ///  - [col 2-5]    left input, left spindle z (m), left spindle z velocity (m/s), left wheel travel (m)
    ///  - [col 6-7]    left spring force (N), left shock force (N)
    ///  - [col 8-11]   right input, right spindle z (m), right spindle z velocity (m/s), right wheel travel (m)
    ///  - [col 12-13]  right spring force (N), right shock force (N)
    ///  - [col 14]     estimated ride height (m)
    ///  - [col 15]     left camber angle, gamma (deg)
    ///  - [col 16]     right camber angle, gamma (deg)
    virtual void PlotOutput(const std::string& out_dir, const std::string& out_name) override;

  private:
    void Create();
    virtual double CalcDisplacementOffset() override;
    virtual double CalcTerrainHeight(VehicleSide side) override;
    virtual void UpdateActuators(double displ_left,
                                 double displ_speed_left,
                                 double displ_right,
                                 double displ_speed_right) override;
    virtual void CollectPlotData(double time) override;

    void AddRodVisualization(VehicleSide side, const ChColor& color);

    std::shared_ptr<ChLinkLinActuator> m_rod_linact[2];  ///< rod linear actuators
    std::shared_ptr<ChBody> m_rod[2];                    ///< rod bodies (for visualization only)

    static const double m_rod_length;
    static const double m_rod_radius;
};

/// @} vehicle_wheeled_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
