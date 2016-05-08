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
// Authors: Radu Serban
// =============================================================================
//
// M113 track assembly subsystem.
//
// =============================================================================

#ifndef M113_TRACK_ASSEMBLY_H
#define M113_TRACK_ASSEMBLY_H

#include <string>

#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

#include "models/ChApiModels.h"

namespace m113 {

///
///
///
class CH_MODELS_API M113_TrackAssembly : public chrono::vehicle::ChTrackAssembly {
  public:
    M113_TrackAssembly(chrono::vehicle::VehicleSide side);
    ~M113_TrackAssembly() {}

  private:
};

}  // end namespace m113

#endif
