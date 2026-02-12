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
// Authors: Ian Rust
// =============================================================================
//
// Template for a fifth wheel chassis connector.  This is a passive connector,
// modeled with a universal joint.
//
// =============================================================================

#ifndef CH_CHASSIS_CONNECTOR_FIFTH_WHEEL_H
#define CH_CHASSIS_CONNECTOR_FIFTH_WHEEL_H

#include "chrono_vehicle/ChChassis.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Template for a fifth-wheel chassis connector.
/// This is a passive connector, modeled with a universal joint that allows pitch and yaw relative DOFs.
class CH_VEHICLE_API ChChassisConnectorFifthWheel : public ChChassisConnector {
  public:
    ChChassisConnectorFifthWheel(const std::string& name);
    ~ChChassisConnectorFifthWheel();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "ChassisConnectorFifthWheel"; }

    /// Initialize this chassis connector subsystem.
    /// The subsystem is initialized by attaching it to the specified front and rear chassis bodies at their connection
    /// points.
    virtual void Initialize(std::shared_ptr<ChChassis> front,    ///< [in] front chassis
                            std::shared_ptr<ChChassisRear> rear  ///< [in] rear chassis
                            ) override;

  protected:
    virtual void PopulateComponentList() override;

    std::shared_ptr<ChLinkUniversal> m_joint;  ///< universal joint of the connector
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
