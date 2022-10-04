// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Rig mechanisms for drawbar-pull tests in the vehicle co-simulation framework.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM_DBPRIG_H
#define CH_VEHCOSIM_DBPRIG_H

#include "chrono_vehicle/ChApiVehicle.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/utils/ChFilters.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_cosim_mbs
/// @{

/// Base class for a drawbar-pull rig mechanism.
/// Derived classes implement different ways for measuring DBP.
class CH_VEHICLE_API ChVehicleCosimDBPRig {
  public:
    /// Type of DBP rig
    enum class Type {
        IMPOSED_SLIP,    ///< rig of type ChVehicleCosimDBPRigImposedSlip
        IMPOSED_ANG_VEL  ///< rig of type ChVehicleCosimDBPRigImposedAngVel
    };

    virtual ~ChVehicleCosimDBPRig() {}

    /// Set window (in seconds) for the running average filter for drawbar pull reporting (default: 0.1 s).
    void SetDBPFilterWindow(double window) { m_dbp_filter_window = window; }

    /// Set window (in seconds) for the running average filter for slip reporting (default: 0.1 s).
    void SetSlipFilterWindow(double window) { m_slip_filter_window = window; }

    /// Get rig type.
    virtual Type GetType() const = 0;

    /// Return current value of longitudinal slip.
    virtual double GetSlip() const = 0;

    /// Return current rig linear speed.
    virtual double GetLinVel() const = 0;

    /// Return current wheel angular speed.
    virtual double GetAngVel() const = 0;

    /// Return current raw drawbar-pull value.
    virtual double GetDBP() const = 0;

    /// Return current filtered drawbar-pull value.
    double GetFilteredDBP() const { return m_dbp_filtered; }

    /// Return current filtered slip value.
    double GetFilteredSlip() const { return m_slip_filtered; }

  protected:
    ChVehicleCosimDBPRig();

    /// Initialize the rig mechanism, by attaching it to the specified chassis body.
    virtual void InitializeRig(std::shared_ptr<ChBody> chassis,  ///< associated chassis body
                               double wheel_radius               ///< radius (tire or sprocket)
                               ) = 0;

    /// Return a function to specify spindle angular speed.
    virtual std::shared_ptr<ChFunction> GetMotorFunction() const = 0;

    bool m_verbose;       ///< verbose messages during simulation?
    double m_delay_time;  ///< initialization (ramping up) time

  private:
    void Initialize(std::shared_ptr<ChBody> chassis, double wheel_radius, double step_size);
    void OnAdvance(double step_size);

    std::unique_ptr<utils::ChRunningAverage> m_dbp_filter;   ///< running average filter for DBP
    double m_dbp_filter_window;                              ///< window (span) for the DBP filter
    double m_dbp_filtered;                                   ///< current value of filtered DBP
    std::unique_ptr<utils::ChRunningAverage> m_slip_filter;  ///< running average filter for slip
    double m_slip_filter_window;                             ///< window (span) for the slip filter
    double m_slip_filtered;                                  ///< current value of filtered slip

    friend class ChVehicleCosimWheeledMBSNode;
    friend class ChVehicleCosimTrackedMBSNode;
};

/// Drawbar-pull rig mechanism with imposed slip.
/// This mechanism allows imposing known (fixed) vehicle forward linear velocity and wheel angular velocity to maintain
/// a prescribed value of the longitudinal slip. The actuation specifies if the linear velocity or angular velocity is
/// considered as "base velocity", with the other one derived from the slip value. The DBP force is extracted as the
/// reaction force required to enforce the vehicle forward linear velocity (at steady state).  Each run of this
/// experiment produces one point on the slip-DBP curve.
class CH_VEHICLE_API ChVehicleCosimDBPRigImposedSlip : public ChVehicleCosimDBPRig {
  public:
    /// Type of drawbar pull actuation type.
    enum class ActuationType {
        SET_LIN_VEL,  ///< fixed carrier linear velocity
        SET_ANG_VEL,  ///< fixed wheel angular velocity
        UNKNOWN       ///< unknown actuation type
    };

    ChVehicleCosimDBPRigImposedSlip(ActuationType act_type,  ///< actuation type
                                    double base_vel,         ///< constant linear or angular velocity
                                    double slip              ///< desired longitudinal slip
    );

    ~ChVehicleCosimDBPRigImposedSlip() {}

    /// Get rig type.
    virtual Type GetType() const override { return Type::IMPOSED_SLIP; }

    /// Return the actuation type.
    ActuationType GetActuationType() const { return m_act_type; }

    /// Return a string describing the actuation type.
    static std::string GetActuationTypeAsString(ActuationType type);

    /// Infer the actuation type from the given string.
    static ActuationType GetActuationTypeFromString(const std::string& type);

    /// Return the current slip value.
    virtual double GetSlip() const override { return m_slip; }

    /// Return current rig linear speed.
    virtual double GetLinVel() const override { return m_lin_vel; }

    /// Return current wheel angular speed.
    virtual double GetAngVel() const override { return m_ang_vel; }

    /// Return current raw drawbar-pull value.
    virtual double GetDBP() const override;

    /// Set the delay and ramping time intervals for reaching the prescribed linear and angular velocities.
    /// By default, the velocities are kept zero for 0.2 s and then increased to their final values in 0.5 s.
    void SetRampingIntervals(double delay, double ramp_time);

  private:
    virtual void InitializeRig(std::shared_ptr<ChBody> chassis, double wheel_radius) override;
    virtual std::shared_ptr<ChFunction> GetMotorFunction() const override;

    ActuationType m_act_type;  ///< actuation type

    std::shared_ptr<ChFunction> m_rot_motor_func;         ///< imposed spindle angular speed
    std::shared_ptr<ChFunction> m_lin_motor_func;         ///< imposed carrier linear speed
    std::shared_ptr<ChLinkMotorLinearSpeed> m_lin_motor;  ///< linear motor for imposed linear velocity

    double m_time_delay;  ///< initial interval with zero velocities
    double m_time_ramp;   ///< time interval to ramp up velocities to final values

    double m_base_vel;  ///< base velocity (linear or angular, depending on actuation type)
    double m_slip;      ///< prescribed longitudinal slip for wheel
    double m_lin_vel;   ///< rig linear velocity (based on actuation type and slip value)
    double m_ang_vel;   ///< wheel angular velocity (based on actuation type and slip value)
};

/// Drawbar-pull rig mechanism with imposed angular velocity.
/// This mechanism enforces a prescribed wheel angular velocity. A linearly increasing resistive force is applied
/// against the forward motion of the vehicle and the experiment is ended when the vehicle stops. At each time, the
/// vehicle forward speed and resulting slip are calculated and stored together with the current resistive force (DBP).
/// This experiment produces the entire slip-DBP curve at once.
class CH_VEHICLE_API ChVehicleCosimDBPRigImposedAngVel : public ChVehicleCosimDBPRig {
  public:
    ChVehicleCosimDBPRigImposedAngVel(double ang_vel,    ///< prescribed wheel angular speed
                                      double force_rate  ///< rate of change of resistive force
    );
    ~ChVehicleCosimDBPRigImposedAngVel() {}

    /// Get rig type.
    virtual Type GetType() const override { return Type::IMPOSED_ANG_VEL; }

    /// Return the current slip value.
    virtual double GetSlip() const override;

    /// Return current rig linear speed.
    virtual double GetLinVel() const override;

    /// Return current wheel angular speed.
    virtual double GetAngVel() const override { return m_ang_vel; }

    /// Return current raw drawbar-pull value.
    virtual double GetDBP() const override;

    /// Set the delay and ramping time intervals for reaching the prescribed angular velocity.
    /// By default, the angular velocity is kept zero for 0.2 s and then increased to the prescribed value in 0.5 s.
    void SetRampingIntervals(double delay, double ramp_time);

  private:
    virtual void InitializeRig(std::shared_ptr<ChBody> chassis, double wheel_radius) override;
    virtual std::shared_ptr<ChFunction> GetMotorFunction() const override;

    std::shared_ptr<ChBody> m_carrier;             ///< rig carrier body
    std::shared_ptr<ChFunction> m_rot_motor_func;  ///< imposed spindle angular speed
    std::shared_ptr<ChLoadBodyForce> m_DBP_force;  ///< resistive (DBP) force

    double m_time_delay;  ///< initial interval with zero angular velocity
    double m_time_ramp;   ///< time interval to ramp up angular velocity to prescribed value

    double m_ang_vel;      ///< wheel angular velocity
    double m_force_rate;   ///< rate of change of resistive force
    double m_tire_radius;  ///< tire radius
};

/// @} vehicle_cosim_mbs

}  // end namespace vehicle
}  // end namespace chrono

#endif
