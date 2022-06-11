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
// Template for a hitch chassis connector.  This is a passive connector,
// modeled with a spherical joint.
//
// =============================================================================

#ifndef CH_CHASSIS_CONNECTOR_HITCH_H
#define CH_CHASSIS_CONNECTOR_HITCH_H

#include "chrono_vehicle/ChChassis.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Template for a hitch chassis connector.  This is a passive connector, modeled with a spherical joint.
class CH_VEHICLE_API ChChassisConnectorHitch : public ChChassisConnector {
  public:
    ChChassisConnectorHitch(const std::string& name);
    ~ChChassisConnectorHitch();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "ChassisConnectorHitch"; }

    /// Initialize this chassis connector subsystem.
    /// The subsystem is initialized by attaching it to the specified front and rear
    /// chassis bodies at the specified location (with respect to and expressed in
    /// the reference frame of the front chassis).
    virtual void Initialize(std::shared_ptr<ChChassis> front,    ///< [in] front chassis
                            std::shared_ptr<ChChassisRear> rear  ///< [in] rear chassis
                            ) override;

  protected:
    std::shared_ptr<ChLinkLockSpherical> m_joint;  ///< spherical joint of the connector
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
