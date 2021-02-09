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
// Base class for segmented track assemblies.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_TRACK_ASSEMBLY_SEGMENTED_H
#define CH_TRACK_ASSEMBLY_SEGMENTED_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

#include "chrono/physics/ChLinkRotSpringCB.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked
/// @{

/// Base class for segmented track assemblies.
/// Track shoes in such an assembly are modeled with one or more rigid bodies connected through joints and/or bushings.
class CH_VEHICLE_API ChTrackAssemblySegmented : public ChTrackAssembly {
  public:
    enum class ConnectionType {
        IDEAL_JOINT,  ///< ideal (kinematic) joints
        RSDA_JOINT    ///< joints with torsional stiffness
    };

    virtual ~ChTrackAssemblySegmented() {}

    ConnectionType GetConnectionType() const { return m_connection_type; }
    std::shared_ptr<ChLinkRotSpringCB::TorqueFunctor> GetTorqueFunctor() const { return m_torque_funct; }

  protected:
    ChTrackAssemblySegmented(const std::string& name,  ///< [in] name of the subsystem
                             VehicleSide side          ///< [in] assembly on left/right vehicle side
    );

    ConnectionType m_connection_type;                                  ///< track shoe connection type
    std::shared_ptr<ChLinkRotSpringCB::TorqueFunctor> m_torque_funct;  ///< torque for RSDA_JOINT connection type
};

/// @} vehicle_tracked

}  // end namespace vehicle
}  // end namespace chrono

#endif
