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
// Template for a torsion chassis connector.  This is a passive connector,
// modeled with a revolute joint (aligned with the vehicle's longitudinal axis)
// and a rotational spring-damper.
//
// =============================================================================

#ifndef CH_CHASSIS_CONNECTOR_TORSION_H
#define CH_CHASSIS_CONNECTOR_TORSION_H

#include "chrono_vehicle/ChChassis.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Template for a torsion chassis connector.  This is a passive connector,
/// modeled with a revolute joint (aligned with the vehicle's longitudinal axis)
/// and a rotational spring-damper.
class CH_VEHICLE_API ChChassisConnectorTorsion : public ChChassisConnector {
  public:
    ChChassisConnectorTorsion(const std::string& name);
    ~ChChassisConnectorTorsion();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "ChassisConnectorTorsion"; }

    /// Initialize this chassis connector subsystem.
    /// The subsystem is initialized by attaching it to the specified front and rear
    /// chassis bodies at the specified location (with respect to and expressed in
    /// the reference frame of the front chassis).
    virtual void Initialize(std::shared_ptr<ChChassis> front,    ///< [in] front chassis
                            std::shared_ptr<ChChassisRear> rear  ///< [in] rear chassis
                            ) override;

  protected:
    /// Return the torsion stiffness of the chassis.
    virtual double GetTorsionStiffness() const = 0;

    std::shared_ptr<ChLinkLockRevolute> m_joint;  ///< revolute joint of the connector
    std::shared_ptr<ChLinkRSDA> m_spring;         ///< rotational spring-damper
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
