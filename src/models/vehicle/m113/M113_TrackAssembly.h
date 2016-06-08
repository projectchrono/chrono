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

namespace chrono {
namespace vehicle {
namespace m113 {

/// Base class for the M113 track assembly.
class CH_MODELS_API M113_TrackAssembly : public ChTrackAssembly {
  public:
    ~M113_TrackAssembly() {}

    void SetIdlerVisType(VisualizationType vis);
    void SetRoadWheelVisType(VisualizationType vis);
    virtual void SetSprocketVisType(VisualizationType vis) = 0;
    virtual void SetTrackShoeVisType(VisualizationType vis) = 0;

  protected:
    M113_TrackAssembly(VehicleSide side);
};

/// M113 track assembly using single-pin track shoes.
class CH_MODELS_API M113_TrackAssemblySinglePin : public M113_TrackAssembly {
public:
    M113_TrackAssemblySinglePin(VehicleSide side);

    virtual void SetSprocketVisType(VisualizationType vis) override;
    virtual void SetTrackShoeVisType(VisualizationType vis) override;
};

/// M113 track assembly using double-pin track shoes.
class CH_MODELS_API M113_TrackAssemblyDoublePin : public M113_TrackAssembly {
public:
    M113_TrackAssemblyDoublePin(VehicleSide side);

    virtual void SetSprocketVisType(VisualizationType vis) override;
    virtual void SetTrackShoeVisType(VisualizationType vis) override;
};

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
