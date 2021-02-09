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
// Authors: Rainer Gericke
// =============================================================================
//
// Marder single-pin track assembly subsystem.
//
// =============================================================================

#ifndef MARDER_TRACK_ASSEMBLY_SINGLE_PIN_H
#define MARDER_TRACK_ASSEMBLY_SINGLE_PIN_H

#include <string>

#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblySinglePin.h"
#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace marder {

/// @addtogroup vehicle_models_marder
/// @{

/// Marder track assembly using single-pin track shoes.
class CH_MODELS_API Marder_TrackAssemblySinglePin : public ChTrackAssemblySinglePin {
  public:
    Marder_TrackAssemblySinglePin(VehicleSide side, BrakeType brake_type);

    virtual const ChVector<> GetSprocketLocation() const override;
    virtual const ChVector<> GetIdlerLocation() const override;
    virtual const ChVector<> GetRoadWhelAssemblyLocation(int which) const override;
    virtual const ChVector<> GetRollerLocation(int which) const override;

  private:
    static const ChVector<> m_sprocket_loc;
    static const ChVector<> m_idler_loc;
    static const ChVector<> m_susp_locs_L[6];
    static const ChVector<> m_susp_locs_R[6];
    static const ChVector<> m_supp_locs_L[3];
    static const ChVector<> m_supp_locs_R[3];

    static const double m_right_x_offset;
};

/// @} vehicle_models_marder

}  // namespace marder
}  // end namespace vehicle
}  // end namespace chrono

#endif
