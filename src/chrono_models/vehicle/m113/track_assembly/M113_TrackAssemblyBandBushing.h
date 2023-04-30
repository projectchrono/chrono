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
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// M113 continuous band track assembly subsystem using rigid-link track shoes.
//
// =============================================================================

#ifndef M113_TRACK_ASSEMBLY_RIGID_CB_H
#define M113_TRACK_ASSEMBLY_RIGID_CB_H

#include <string>

#include "chrono_models/ChApiModels.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyBandBushing.h"

namespace chrono {
namespace vehicle {
namespace m113 {

/// @addtogroup vehicle_models_m113
/// @{

/// M113 continuous band track assembly subsystem using rigid-link track shoes.
class CH_MODELS_API M113_TrackAssemblyBandBushing : public ChTrackAssemblyBandBushing {
  public:
    M113_TrackAssemblyBandBushing(VehicleSide side, BrakeType brake_type, bool use_suspension_bushings);

    virtual const ChVector<> GetSprocketLocation() const override;
    virtual const ChVector<> GetIdlerLocation() const override;
    virtual const ChVector<> GetRoadWhelAssemblyLocation(int which) const override;

  private:
    static const ChVector<> m_sprocket_loc;
    static const ChVector<> m_idler_loc;
    static const ChVector<> m_susp_locs_L[5];
    static const ChVector<> m_susp_locs_R[5];
};

/// @} vehicle_models_m113

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
