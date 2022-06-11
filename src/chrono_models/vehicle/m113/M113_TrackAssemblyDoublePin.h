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
// M113 double-pin track assembly subsystem.
//
// =============================================================================

#ifndef M113_TRACK_ASSEMBLY_DOUBLE_PIN_H
#define M113_TRACK_ASSEMBLY_DOUBLE_PIN_H

#include <string>

#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyDoublePin.h"
#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

/// @addtogroup vehicle_models_m113
/// @{

/// M113 track assembly using double-pin track shoes.
class CH_MODELS_API M113_TrackAssemblyDoublePin : public ChTrackAssemblyDoublePin {
  public:
    M113_TrackAssemblyDoublePin(VehicleSide side,
                                DoublePinTrackShoeType topology,
                                BrakeType brake_type,
                                bool use_track_bushings,
                                bool use_suspension_bushings,
                                bool use_track_RSDA);

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
