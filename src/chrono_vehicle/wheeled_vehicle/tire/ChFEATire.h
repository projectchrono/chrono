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
// Template for a deformable co-rotational FEA tire.
//
// =============================================================================

#ifndef CH_FEATIRE_H
#define CH_FEATIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// Co-rotational FEA tire template.
/// This tire is modeled as a mesh composed of co-rotational elements.
class CH_VEHICLE_API ChFEATire : public ChDeformableTire {
  public:
    ChFEATire(const std::string& name);

    virtual ~ChFEATire() {}

  protected:
    /// Return list of internal nodes.
    /// These nodes define the mesh surface over which pressure loads are applied.
    virtual std::vector<std::shared_ptr<fea::ChNodeFEAbase>> GetInternalNodes() const = 0;

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
