// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
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
// Implementation of a single-tire static test rig.
// - Accepts an arbitrary Chrono::Vehicle tire object
//   (and associated ChWheel object)
//
// =============================================================================

#ifndef CH_TIRE_STATIC_TEST_RIG_H
#define CH_TIRE_STATIC_TEST_RIG_H

#include <iomanip>
#include <cmath>

#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_test_rig
/// @{

/// Definition of a single-tire static test rig.
class CH_VEHICLE_API ChTireStaticTestRig {
  public:
    /// Tire test rig operation mode.
    enum class Mode {
        SUSPEND,  ///< suspended tire (locked spindle)
        TEST_R,   ///< vertical compression test
        TEST_X,   ///< longitudinal plate displacement test
        TEST_Y,   ///< lateral plate displacement test
        TEST_Z    ///< torsional plate displacement test
    };

    /// Construct a tire test rig within the specified system.
    ChTireStaticTestRig(std::shared_ptr<ChWheel> wheel,  ///< wheel subsystem
                        std::shared_ptr<ChTire> tire,    ///< tire subsystem
                        ChSystem* system                 ///< containing mechanical system
    );

    ~ChTireStaticTestRig();

    /// Set gravitational acceleration (default: 9.81 m/s2).
    void SetGravitationalAcceleration(double grav) { m_grav = grav; }

    /// Set nominal radial load (default: 5000 N).
    void SetNominalRadialLoad(double load) { m_r_load = load; }

    /// Set radial load speed (default: 1e-3 m/s).
    void SetRadialLoadSpeed(double speed) { m_r_speed = speed; }

    /// Set longitudinal load speed (default: 1e-3 m/s).
    void SetLongitudinalLoadSpeed(double speed) { m_x_speed = speed; }

    /// Set lateral load speed (default: 1e-3 m/s).
    void SetLateralLoadSpeed(double speed) { m_y_speed = speed; }

    /// Set torsional load speed (default: 1e-2 rad/s).
    void SetTorsionalLoadSpeed(double speed) { m_z_speed = speed; }

    /// Set collision type for tire-terrain interaction (default: SINGLE_POINT).
    void SetTireCollisionType(ChTire::CollisionType coll_type);

    /// Set the time step for advancing tire dynamics (default: 1e-3 s).
    void SetTireStepsize(double step) { m_tire_step = step; }

    /// Set visualization type for the tire (default: PRIMITIVES).
    void SetTireVisualizationType(VisualizationType vis) { m_tire_vis = vis; }

    /// Specify contact material properties for rig plate.
    void SetPlateMaterialProperties(double friction,      ///< coefficient of friction
                                    double restitution,   ///< coefficient of restitution
                                    double Young_modulus  ///< contact material Young's modulus [Pa]
    );

    /// Set time delay before switching state (default: 0.1 s).
    void SetStateTransitionDelay(double delay) { m_transition_delay = delay; }

    /// Enable output and specify whether results should be plotted using Gnuplot (default: false).
    void SetOutput(const std::string& out_dir, bool gnuplot = false);

    /// Initialize the rig system.
    /// It is the user's responsibility to set the operation mode and motion functions in a consistent way.
    void Initialize(Mode mode);

    /// Advance system state by the specified time step.
    /// If return value is 'false', testing was completed and simulation should be stopped.
    bool Advance(double step);

    /// Get current wheel position.
    ChVector3d GetWheelPos() const;

    /// Get the current tire forces
    TerrainForce ReportTireForce();

    /// Get the current radial load.
    double GetRadialLoad() const;

  private:
    enum class State {
      FIXED,
      DROPPING,
      COMPRESSING,
      DISPLACING,
      DONE
    };

    class RigTerrain : public ChTerrain {
      public:
        RigTerrain() {}
        virtual double GetHeight(const ChVector3d& loc) const override { return h; }
        virtual ChVector3d GetNormal(const ChVector3d& loc) const override { return ChVector3d(0, 0, 1); }
        virtual float GetCoefficientFriction(const ChVector3d& loc) const override { return mu; }
        double h;
        float mu;
    };

    void UpdateActuators(double time);
    void StateTransition(double time);
    void Output(double time);
    void WriteOutput();

    std::string ModeName(Mode mode) const;
    std::string StateName(State state) const;

    ChSystem* m_system;                ///< pointer to the Chrono system
    RigTerrain m_terrain;              ///< handle to underlying terrain subsystem
    std::shared_ptr<ChWheel> m_wheel;  ///< handle to wheel subsystem
    std::shared_ptr<ChTire> m_tire;    ///< handle to tire subsystem
    VisualizationType m_tire_vis;      ///< visualization type for tire subsystem
    double m_tire_step;                ///< step size for tire integration

    double m_grav;  ///< gravitational acceleration

    double m_r_load;   ///< nominal radial load
    double m_r_speed;  ///< radial load speed
    double m_x_speed;  ///< longitudinal load speed
    double m_y_speed;  ///< lateral load speed
    double m_z_speed;  ///< torsional load speed

    float m_plate_friction;       ///< coefficient of friction
    float m_plate_Young_modulus;  ///< Young's modulus (Pa)
    float m_plate_restitution;    ///< coefficient of restitution

    std::shared_ptr<ChBody> m_ground_body;   ///< ground body
    std::shared_ptr<ChBody> m_spindle_body;  ///< wheel spindle body
    std::shared_ptr<ChBody> m_plate_body;    ///< rig plate body

    std::shared_ptr<ChLinkMotorLinearSpeed> m_motor_r;    ///< motor to apply radial load
    std::shared_ptr<ChLinkMotorLinearSpeed> m_motor_x;    ///< motor to apply longitudinal plate displacement
    std::shared_ptr<ChLinkMotorLinearSpeed> m_motor_y;    ///< motor to apply lateral plate displacement
    std::shared_ptr<ChLinkMotorRotationSpeed> m_motor_z;  ///< motor to apply torsional plate displacement

    Mode m_mode;                ///< test mode
    State m_state;              ///< current state (phase)
    double m_transition_delay;  ///< time delay before switching state
    double m_transition_time;   ///< time of last state transition
    double m_spindle_z_ref;     ///< reference spindle z for calculating radial deflection

    std::string m_outdir;
    bool m_output;
    bool m_gnuplot;
    utils::ChWriterCSV m_csv;
};

/// @} vehicle_wheeled_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
