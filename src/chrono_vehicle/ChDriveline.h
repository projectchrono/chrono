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
// Base class for a vehicle driveline.
//
// =============================================================================

#ifndef CH_DRIVELINE_H
#define CH_DRIVELINE_H

#include "chrono/physics/ChShaft.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Base class for a vehicle driveline subsystem.
class CH_VEHICLE_API ChDriveline : public ChPart {
  public:
    virtual ~ChDriveline();

    /// Get a handle to the driveshaft.
    /// Return a pointer to the shaft that connects this driveline to a powertrain system.
    std::shared_ptr<ChShaft> GetDriveshaft() const { return m_driveshaft; }

    /// Get the angular speed of the driveshaft.
    /// This represents the output from the driveline subsystem that is passed to the powertrain system.
    double GetDriveshaftSpeed() const { return -m_driveshaft->GetPos_dt(); }

  protected:
    ChDriveline(const std::string& name);

    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

    std::shared_ptr<ChShaft> m_driveshaft;  ///< shaft connection to the powertrain
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
