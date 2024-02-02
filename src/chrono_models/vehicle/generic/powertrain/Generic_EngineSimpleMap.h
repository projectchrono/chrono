// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Mike Taylor, Marcel Offermans
// =============================================================================
//
// Simple engine model for the generic vehicle based on torque-speed engine maps
//
// =============================================================================

#ifndef GENERIC_ENGINE_SIMPLEMAP_H
#define GENERIC_ENGINE_SIMPLEMAP_H

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/powertrain/ChEngineSimpleMap.h"
#include "chrono/core/ChCubicSpline.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace generic {

/// @addtogroup vehicle_models_generic
/// @{

/// Engine model for a generic vehicle.
class CH_MODELS_API Generic_EngineSimpleMap : public ChEngineSimpleMap {
  public:
    Generic_EngineSimpleMap(const std::string& name);
    double GetMaxEngineSpeed() override;
    void SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) override;
};

/// @} vehicle_models_generic

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
