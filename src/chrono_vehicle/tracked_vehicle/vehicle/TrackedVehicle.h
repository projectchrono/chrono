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
// Tracked vehicle model constructed from a JSON specification file
//
// =============================================================================

#ifndef TRACKED_VEHICLE_H
#define TRACKED_VEHICLE_H

#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChMaterialSurface.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked
/// @{

/// Tracked vehicle model constructed from a JSON specification file.
class CH_VEHICLE_API TrackedVehicle : public ChTrackedVehicle {
  public:
    TrackedVehicle(const std::string& filename,
                   ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::NSC);

    TrackedVehicle(ChSystem* system, const std::string& filename);

    ~TrackedVehicle() {}

    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

  private:
    void Create(const std::string& filename);

  private:
    double m_track_offset[2];  ///< offsets for the left and right track assemblies
};

/// @} vehicle_tracked

}  // end namespace vehicle
}  // end namespace chrono

#endif
