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
// M113 double-pin track assembly subsystem.
//
// =============================================================================

#ifndef M113_TRACK_ASSEMBLY_DOUBLE_PIN_H
#define M113_TRACK_ASSEMBLY_DOUBLE_PIN_H

#include <string>

#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyDoublePin.h"
#include "models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

/// M113 track assembly using double-pin track shoes.
class CH_MODELS_API M113_TrackAssemblyDoublePin : public ChTrackAssemblyDoublePin {
public:
    M113_TrackAssemblyDoublePin(VehicleSide side);

    void SetIdlerVisType(VisualizationType vis);
    void SetRoadWheelVisType(VisualizationType vis);
    void SetSprocketVisType(VisualizationType vis);
    void SetTrackShoeVisType(VisualizationType vis);
};

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
