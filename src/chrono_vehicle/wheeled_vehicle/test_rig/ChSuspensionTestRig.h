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
// Definition of a suspension testing mechanism for a wheeld vehicle.
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
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigDriver.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_test_rig
/// @{

/// Base class for a suspension test rig.
class CH_VEHICLE_API ChSuspensionTestRig {
  public:
    /// Destructor
    virtual ~ChSuspensionTestRig();

    /// Set driver system.
    void SetDriver(std::shared_ptr<ChSuspensionTestRigDriver> driver);

    /// Set the initial ride height (relative to the chassis reference frame).
    /// If not specified, the reference height is the suspension design configuration.
    void SetInitialRideHeight(double height) { m_ride_height = height; }

    /// Set the limits for post displacement (same for jounce and rebound).
    void SetDisplacementLimit(double limit) { m_displ_limit = limit; }

    /// Include the specified steering mechanism in testing.
    void IncludeSteeringMechanism(int index);

    /// Include the specified subchassis in testing.
    void IncludeSubchassis(int index);

    /// Set visualization type for the suspension subsystem (default: PRIMITIVES).
    void SetSuspensionVisualizationType(VisualizationType vis) { m_vis_suspension = vis; }

    /// Set visualization type for the steering subsystems (default: PRIMITIVES).
    void SetSteeringVisualizationType(VisualizationType vis) { m_vis_steering = vis; }

    /// Set visualization type for the subchassis subsystems (default: PRIMITIVES).
    void SetSubchassisVisualizationType(VisualizationType vis) { m_vis_subchassis = vis; }

    /// Set visualization type for the wheel subsystems (default: NONE).
    void SetWheelVisualizationType(VisualizationType vis) { m_vis_wheel = vis; }

    /// Set visualization type for the tire subsystems (default: PRIMITIVES).
    void SetTireVisualizationType(VisualizationType vis) { m_vis_tire = vis; }

    /// Initialize the suspension test rig.
    /// Initialize and prepare the associated vehicle and rig driver, then let derived classes construct the bodies,
    /// joints, and actuators of the rig mechanism.
    void Initialize();

    /// Advance the state of the suspension test rig by the specified time step.
    void Advance(double step);

    /// Get a handle to the underlying vehicle system.
    ChWheeledVehicle& GetVehicle() const { return *m_vehicle; }

    /// Get the global location of the specified spindle.
    /// Note: 'axle' is the index in the set of tested axles.
    const ChVector<>& GetSpindlePos(int axle, VehicleSide side) const;

    /// Get the global rotation of the specified spindle.
    /// Note: 'axle' is the index in the set of tested axles.
    ChQuaternion<> GetSpindleRot(int axle, VehicleSide side) const;

    /// Get the linear velocity of the specified spindle (expressed in the global reference frame).
    /// Note: 'axle' is the index in the set of tested axles.
    const ChVector<>& GetSpindleLinVel(int axle, VehicleSide side) const;

    /// Get the angular velocity of the specified spindle (expressed in the global reference frame).
    /// Note: 'axle' is the index in the set of tested axles.
    ChVector<> GetSpindleAngVel(int axle, VehicleSide side) const;

    /// Return current driver steering input.
    double GetSteeringInput() const { return m_driver->GetSteering(); }

    /// Return the current post displacement for the specified axle and vehicle side.
    virtual double GetActuatorDisp(int axle, VehicleSide side) = 0;

    /// Return the current post actuation force for the specified axle and vehicle side.
    virtual double GetActuatorForce(int axle, VehicleSide side) = 0;

    /// Get wheel travel, defined here as the vertical displacement of the spindle center from its position at
    /// the initial specified ride height (or initial configuration, if an initial ride height is not specified).
    /// Note: 'axle' is the index in the set of tested axles.
    double GetWheelTravel(int axle, VehicleSide side) const;

    /// Get current ride height for the specified axle (relative to the chassis reference frame).
    /// This estimate uses the average of the left and right posts.
    /// Note: 'axle' is the index in the set of tested axles.
    virtual double GetRideHeight(int axle) const = 0;

    /// Return the info message from the STR driver.
    std::string GetDriverMessage() const { return m_driver->GetInfoMessage(); }

    /// Return true when driver stopped producing inputs.
    bool DriverEnded() const { return m_driver->Ended(); }

    /// Log current constraint violations.
    void LogConstraintViolations();

    /// Enable output for this vehicle system (default: false).
    void SetOutput(ChVehicleOutput::Type type,   ///< type of output DB
                   const std::string& out_dir,   ///< output directory name
                   const std::string& out_name,  ///< rootname of output file
                   double output_step            ///< interval between output times
    );

    /// Enable data collection for plot generation (default: false).
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
    void SetPlotOutput(double output_step);

    /// Plot collected data.
    /// When called (typically at the end of the simulation loop), the collected data is saved in ASCII format in a file
    /// with the specified name (and 'txt' extension) in the specified output directory. If the Chrono::Postprocess
    /// module is available (with gnuplot support), selected plots are generated.\n
    void PlotOutput(const std::string& out_dir, const std::string& out_name);

  protected:
    /// Construct a test rig for specified sets of axles and sterring mechanisms of a given vehicle.
    ChSuspensionTestRig(std::shared_ptr<ChWheeledVehicle> vehicle,  ///< test vehicle
                        std::vector<int> axle_index,                ///< list of tested vehicle axles
                        double displ_limit                          ///< displacement limits
    );

    /// Construct a test rig from the given JSON specification file.
    /// An STR specification file includes the JSON specification for the underlying vehicle, the sets of tested vehicle
    /// subsystems, and the displacement limit for the rig posts.
    ChSuspensionTestRig(const std::string& spec_filename);

    /// Create and initialize the bodies, joints, and actuators of the rig mechanism.
    virtual void InitializeRig() = 0;

    /// Calculate required actuator displacement offset to impose specified ride height.
    /// Note: 'axle' is the index in the set of tested axles.
    virtual double CalcDisplacementOffset(int axle) = 0;

    /// Calculate current terrain height under the specified post.
    /// Note: 'axle' is the index in the set of tested axles.
    virtual double CalcTerrainHeight(int axle, VehicleSide side) = 0;

    /// Update actuators at current time.
    virtual void UpdateActuators(std::vector<double> displ_left,
                                 std::vector<double> displ_speed_left,
                                 std::vector<double> displ_right,
                                 std::vector<double> displ_speed_right) = 0;

    /// Collect data for plot output.
    void CollectPlotData(double time);

    std::shared_ptr<ChWheeledVehicle> m_vehicle;  ///< associated vehicle
    int m_naxles;                                 ///< number of tested axles
    std::vector<int> m_axle_index;                ///< list of tested vehicle axles in rig
    std::vector<int> m_steering_index;            ///< list of tested vehicle steering mechanisms in rig
    std::vector<int> m_subchassis_index;          ///< list of tested vehicle subchassis mechanisms in rig

    double m_ride_height;                ///< ride height
    double m_displ_limit;                ///< scale factor for post displacement
    std::vector<double> m_displ_offset;  ///< post displacement offsets (to set reference positions)

  private:
    std::unique_ptr<ChTerrain> m_terrain;   ///< terrain object to provide height to the tires
    std::shared_ptr<ChSuspensionTestRigDriver> m_driver;  ///< driver system
    double m_steering_input;                ///< current driver steering input
    std::vector<double> m_left_inputs;      ///< current driver left post inputs
    std::vector<double> m_right_inputs;     ///< current driver right post inputs

    std::vector<double> m_spindle_ref_L;  ///< reference left spindle vertical positions (for wheel travel calculation)
    std::vector<double> m_spindle_ref_R;  ///< reference right spindle vertical positions (for wheel travel calculation)

    VisualizationType m_vis_steering;
    VisualizationType m_vis_suspension;
    VisualizationType m_vis_subchassis;
    VisualizationType m_vis_wheel;
    VisualizationType m_vis_tire;

    bool m_plot_output;
    double m_plot_output_step;
    double m_next_plot_output_time;
    utils::CSV_writer* m_csv;
    std::vector<int> m_csv_lengths;
};

// -----------------------------------------------------------------------------

/// Definition of a suspension test rig using platforms to actuate the tires.
class CH_VEHICLE_API ChSuspensionTestRigPlatform : public ChSuspensionTestRig {
  public:
    /// Construct a test rig for specified sets of axles and sterring mechanisms of a given vehicle.
    ChSuspensionTestRigPlatform(std::shared_ptr<ChWheeledVehicle> vehicle,  ///< test vehicle
                                std::vector<int> axle_index,                ///< list of tested vehicle axles
                                double displ_limit                          ///< displacement limits
    );

    /// Construct a test rig from the given JSON specification file.
    /// An STR specification file includes the JSON specification for the underlying vehicle, the sets of tested vehicle
    /// subsystems, and the displacement limit for the rig posts.
    ChSuspensionTestRigPlatform(const std::string& spec_filename);

    ~ChSuspensionTestRigPlatform();

    /// Return the current post displacement for the specified axle and vehicle side.
    virtual double GetActuatorDisp(int axle, VehicleSide side) override;

    /// Return the current post actuation force for the specified axle and vehicle side.
    virtual double GetActuatorForce(int axle, VehicleSide side) override;

    /// Get current ride height (relative to the chassis reference frame).
    /// This estimate uses the average of the left and right posts.
    /// Note: 'axle' is the index in the set of tested axles.
    virtual double GetRideHeight(int axle) const override;

  private:
    virtual void InitializeRig() override;
    virtual double CalcDisplacementOffset(int axle) override;
    virtual double CalcTerrainHeight(int axle, VehicleSide side) override;
    virtual void UpdateActuators(std::vector<double> displ_left,
                                 std::vector<double> displ_speed_left,
                                 std::vector<double> displ_right,
                                 std::vector<double> displ_speed_right) override;

    void AddPostVisualization(std::shared_ptr<ChBody> post, const ChColor& color);

    std::vector<std::shared_ptr<ChBody>> m_post_L;                       ///< left post bodies
    std::vector<std::shared_ptr<ChBody>> m_post_R;                       ///< right post bodies
    std::vector<std::shared_ptr<ChLinkMotorLinearPosition>> m_linact_L;  ///< left post linear actuators
    std::vector<std::shared_ptr<ChLinkMotorLinearPosition>> m_linact_R;  ///< right post linear actuators

    static const double m_post_radius;   ///< radius of the post cylindrical platform
    static const double m_post_hheight;  ///< half-height of the post cylindrical platform
};

// -----------------------------------------------------------------------------

/// Definition of a suspension test rig with direct actuation on the spindle bodies.
class CH_VEHICLE_API ChSuspensionTestRigPushrod : public ChSuspensionTestRig {
  public:
    /// Construct a test rig for specified sets of axles and sterring mechanisms of a given vehicle.
    ChSuspensionTestRigPushrod(std::shared_ptr<ChWheeledVehicle> vehicle,  ///< test vehicle
                               std::vector<int> axle_index,                ///< list of tested vehicle axles
                               double displ_limit                          ///< displacement limits
    );

    /// Construct a test rig from the given JSON specification file.
    /// An STR specification file includes the JSON specification for the underlying vehicle, the sets of tested vehicle
    /// subsystems, and the displacement limit for the rig posts.
    ChSuspensionTestRigPushrod(const std::string& spec_filename);

    ~ChSuspensionTestRigPushrod();

    /// Return the current post displacement for the specified axle and vehicle side.
    virtual double GetActuatorDisp(int axle, VehicleSide side) override;

    /// Return the current post actuation force for the specified axle and vehicle side.
    virtual double GetActuatorForce(int axle, VehicleSide side) override;

    /// Get current ride height (relative to the chassis reference frame).
    /// This estimate uses the average of the left and right posts.
    /// Note: 'axle' is the index in the set of tested axles.
    virtual double GetRideHeight(int axle) const override;

  private:
    virtual void InitializeRig() override;
    virtual double CalcDisplacementOffset(int axle) override;
    virtual double CalcTerrainHeight(int axle, VehicleSide side) override;
    virtual void UpdateActuators(std::vector<double> displ_left,
                                 std::vector<double> displ_speed_left,
                                 std::vector<double> displ_right,
                                 std::vector<double> displ_speed_right) override;

    void AddRodVisualization(std::shared_ptr<ChBody> rod, const ChColor& color);

    std::vector<std::shared_ptr<ChBody>> m_rod_L;                ///< left rod bodies (for visualization only)
    std::vector<std::shared_ptr<ChBody>> m_rod_R;                ///< right rod bodies (for visualization only)
    std::vector<std::shared_ptr<ChLinkLinActuator>> m_linact_L;  ///< left rod linear actuators
    std::vector<std::shared_ptr<ChLinkLinActuator>> m_linact_R;  ///< right rod linear actuators

    static const double m_rod_length;
    static const double m_rod_radius;
};

/// @} vehicle_wheeled_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
