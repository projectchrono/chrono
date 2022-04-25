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
// Template for an articulation chassis connector.  This is an active connector,
// modeled with a rotational motor (a revolute joint along the chassis vertical
// axis whose DOF can be actuated based on current steering input).
//
// =============================================================================

#ifndef CH_CHASSIS_CONNECTOR_ARTICULATED_H
#define CH_CHASSIS_CONNECTOR_ARTICULATED_H

#include "chrono_vehicle/ChChassis.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Template for an articulation chassis connector.  This is an active connector,
/// modeled with a rotational motor (a revolute joint along the chassis vertical
/// axis whose DOF can be actuated based on current steering input).
class CH_VEHICLE_API ChChassisConnectorArticulated : public ChChassisConnector {
  public:
    ChChassisConnectorArticulated(const std::string& name);
    ~ChChassisConnectorArticulated();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "ChassisConnectorArticulated"; }

    /// Initialize this chassis connector subsystem.
    /// The subsystem is initialized by attaching it to the specified front and rear
    /// chassis bodies at the specified location (with respect to and expressed in
    /// the reference frame of the front chassis).
    virtual void Initialize(std::shared_ptr<ChChassis> front,    ///< [in] front chassis
                            std::shared_ptr<ChChassisRear> rear  ///< [in] rear chassis
                            ) override;

    /// Update the state of this connector subsystem at the current time.
    /// The connector subsystem is provided the current steering driver input (a value between -1 and +1).
    /// Positive steering input indicates steering to the left.
    /// This connector uses the steering input to control the angle in the underlying rotational motor.
    virtual void Synchronize(double time,     ///< [in] current time
                             double steering  ///< [in] current steering input [-1,+1]
                             ) override;

  protected:
    ///  Return the maximum steering angle.  The steering input is scaled by this value to produce the angle applied to
    ///  the underlying rotational motor.
    virtual double GetMaxSteeringAngle() const = 0;

    std::shared_ptr<ChLinkMotorRotationAngle> m_motor;  ///< steering motor
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
