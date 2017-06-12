// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================
//
// Template for a deformable tire based on Reissner-Mindlin 4 nodes shells
//
// =============================================================================

#ifndef CH_REISSNERTIRE_H
#define CH_REISSNERTIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// Tire template for tires based on Reissner shells 
/// This tire is modeled as a mesh composed of Reissner-Mindlin 4 nodes
/// shell elements. 
class CH_VEHICLE_API ChReissnerTire : public ChDeformableTire {
  public:
    ChReissnerTire(const std::string& name);

    virtual ~ChReissnerTire() {}

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
