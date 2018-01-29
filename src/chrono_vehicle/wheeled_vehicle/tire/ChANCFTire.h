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
// Template for a deformable ANCF tire.
//
// =============================================================================

#ifndef CH_ANCFTIRE_H
#define CH_ANCFTIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// ANCF tire template.
/// This tire is modeled as a mesh composed of ANCF shell elements.
class CH_VEHICLE_API ChANCFTire : public ChDeformableTire {
  public:
    ChANCFTire(const std::string& name);

    virtual ~ChANCFTire() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "ANCFTire"; }

  protected:
    /// Create the ChLoad for applying pressure to the tire.
    virtual void CreatePressureLoad() override final;

    /// Create the contact surface for the tire mesh.
    virtual void CreateContactSurface() override final;

    /// Create the tire-rim connections.
    virtual void CreateRimConnections(std::shared_ptr<ChBody> wheel) override final;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
