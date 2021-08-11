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
#include "chrono/utils/ChFilters.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_cosim_mbs
/// @{

/// Base class for a drawbar-pull rig mechanism.
/// Derived classes implement different ways for measuring DBP.
class CH_VEHICLE_API ChVehicleCosimDBPRig {
  public:
    virtual ~ChVehicleCosimDBPRig() {}

    /// Set window (in seconds) for the running average filter for drawbar pull reporting (default: 0.1 s).
    void SetDBPfilterWindow(double window) { m_filter_window = window; }

    /// Return current raw drawbar-pull value.
    virtual double GetDBP() const = 0;

    /// Return current filtered drawbar-pull value.
    double GetFilteredDBP() const { return m_dbp_filtered; }

  protected:
    ChVehicleCosimDBPRig();

    /// Initialize the rig mechanism, by attaching it to the specified chassis body.
    virtual void InitializeRig(std::shared_ptr<ChBody> chassis,          ///< associated chassis body
                               const std::vector<ChVector<>>& tire_info  ///< mass, radius, width for each tire
                               ) = 0;

    /// Return a function to specify spindle angular speed.
    virtual std::shared_ptr<ChFunction> GetMotorFunction() const = 0;

    std::unique_ptr<utils::ChRunningAverage> m_filter;  ///< running average filter for DBP
    double m_filter_window;                             ///< window (span) for the DBP filter
    double m_dbp_filtered;                              ///< current value of filtered DBP

    bool m_verbose;  ///< verbose messages during simulation?

  private:
    void Initialize(std::shared_ptr<ChBody> chassis, const std::vector<ChVector<>>& tire_info, double step_size);
    void OnAdvance(double step_size);

    friend class ChVehicleCosimMBSNode;
};

/// Drawbar-pull rig mechanism with imposed slip.
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

    /// Return the actuation type.
    ActuationType GetActuationType() const { return m_act_type; }

    /// Return a string describing the actuation type.
    static std::string GetActuationTypeAsString(ActuationType type);

    /// Infer the actuation type from the given string.
    static ActuationType GetActuationTypeFromString(const std::string& type);

    /// Return current raw drawbar-pull value.
    virtual double GetDBP() const override;

  private:
    virtual void InitializeRig(std::shared_ptr<ChBody> chassis, const std::vector<ChVector<>>& tire_info) override;
    virtual std::shared_ptr<ChFunction> GetMotorFunction() const override;

    ActuationType m_act_type;  ///< actuation type

    std::shared_ptr<ChFunction> m_rot_motor_func;         ///< imposed spindle angular speed
    std::shared_ptr<ChFunction> m_lin_motor_func;         ///< imposed carrier linear speed
    std::shared_ptr<ChLinkMotorLinearSpeed> m_lin_motor;  ///< linear motor for imposed linear velocity

    double m_base_vel;  ///< base velocity (linear or angular, depending on actuation type)
    double m_slip;      ///< prescribed longitudinal slip for wheel
    double m_lin_vel;   ///< rig linear velocity (based on actuation type and slip value)
    double m_ang_vel;   ///< wheel angular velocity (based on actuation type and slip value)
};

/// @} vehicle_cosim_mbs

}  // end namespace vehicle
}  // end namespace chrono

#endif