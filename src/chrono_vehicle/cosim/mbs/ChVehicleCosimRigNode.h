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
// Mechanism for testing tires over granular terrain.  The mechanism is 
// co-simulated with a tire and a terrain system.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM_RIG_NODE_H
#define CH_VEHCOSIM_RIG_NODE_H

#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

#include "chrono_vehicle/cosim/ChVehicleCosimMBSNode.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_cosim
/// @{

/// Mechanism for a single-wheel testing rig.
/// The mechanism system is co-simulated with a tire and a terrain system.
class CH_VEHICLE_API ChVehicleCosimRigNode : public ChVehicleCosimMBSNode {
  public:
    /// Type of drawbar pull actuation type.
    enum class ActuationType {
        SET_LIN_VEL,  ///< fixed carrier linear velocity
        SET_ANG_VEL,  ///< fixed wheel angular velocity
        UNKNOWN       ///< unknown actuation type
    };

    ChVehicleCosimRigNode(ActuationType act_type,  ///< actuation type (SET_LIN_VEL or SET_ANG_VEL)
                          double base_vel,         ///< constant linear or angular velocity
                          double slip              ///< desired longitudinal slip
    );

    ~ChVehicleCosimRigNode();

    /// Return the actuation type.
    ActuationType GetActuationType() const { return m_act_type; }

    /// Return a string describing the actuation type.
    static std::string GetActuationTypeAsString(ActuationType type);

    /// Infer the actuation type from the given string.
    static ActuationType GetActuationTypeFromString(const std::string& type);

    /// Set total rig system mass (default: 100).
    /// This represents the equivalent load on the soil from all rig bodies and the tire itself. Note that the total
    /// mass must be at least 2 kg more than the tire mass; otherwise, it will be overwritten.
    void SetTotalMass(double mass) { m_total_mass = mass; }

    /// Set (constant) toe angle in radians (default: 0).
    void SetToeAngle(double angle) { m_toe_angle = angle; }

    /// Set window (in seconds) for the running average filter for drawbar pull reporting (default: 0.1 s).
    void SetDBPfilterWindow(double window) { m_dbp_filter_window = window; }

    /// Output logging and debugging data.
    virtual void OutputData(int frame) override final;

  private:
    virtual void InitializeMBS(const std::vector<ChVector2<>>& tire_info,  ///< mass and radius for each tire
                               const ChVector2<>& terrain_size,            ///< terrain length x width
                               double terrain_height                       ///< initial terrain height
                               ) override;

    virtual std::shared_ptr<ChBody> GetSpindleBody(unsigned int i) const override { return m_spindle; }

    virtual double GetSpindleLoad(unsigned int i) const override { return m_total_mass; }

    void ChVehicleCosimRigNode::WriteBodyInformation(utils::CSV_writer& csv);

  private:
    ActuationType m_act_type;  ///< actuation type

    std::shared_ptr<ChBody> m_ground;   ///< ground body
    std::shared_ptr<ChBody> m_chassis;  ///< chassis body
    std::shared_ptr<ChBody> m_set_toe;  ///< set toe body
    std::shared_ptr<ChBody> m_upright;  ///< upright body
    std::shared_ptr<ChBody> m_spindle;  ///< spindle body

    double m_total_mass;  ///< total equivalent wheel mass

    double m_toe_angle;  ///< toe angle (controls tire slip angle)

    std::shared_ptr<ChLinkMotorRotationAngle> m_slip_motor;  ///< angular motor constraint
    std::shared_ptr<ChLinkLockPrismatic> m_prism_vel;        ///< prismatic joint for chassis linear velocity
    std::shared_ptr<ChLinkLinActuator> m_lin_actuator;       ///< actuator imposing linear velocity to system
    std::shared_ptr<ChLinkLockPrismatic> m_prism_axl;        ///< prismatic joint for chassis-axle joint
    std::shared_ptr<ChLinkMotorRotationAngle> m_rev_motor;   ///< motor enforcing prescribed rim angular vel

    double m_base_vel;  ///< base velocity (linear or angular, depending on actuation type)
    double m_slip;      ///< prescribed longitudinal slip for wheel
    double m_lin_vel;   ///< rig linear velocity (based on actuation type and slip value)
    double m_ang_vel;   ///< wheel angular velocity (based on actuation type and slip value)

    utils::ChRunningAverage* m_dbp_filter;  ///< running average filter for DBP
    double m_dbp_filter_window;             ///< window (span) for the DBP filter
    double m_dbp;                           ///< current value of filtered DBP
};

/// @} vehicle_cosim

}  // end namespace vehicle
}  // end namespace chrono

#endif
