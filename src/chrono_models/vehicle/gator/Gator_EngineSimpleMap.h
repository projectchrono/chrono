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
// Authors: Radu Serban, Marcel Offermans
// =============================================================================
//
// Simple engine map model for the Gator vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
//
// =============================================================================

#ifndef GATOR_ENGINESIMPLEMAP_H
#define GATOR_ENGINESIMPLEMAP_H

#include "chrono_vehicle/powertrain/ChEngineSimpleMap.h"
#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace gator {

/// @addtogroup vehicle_models_gator
/// @{

/// Simple Gator powertrain subsystem (based on engine speed-torque maps).
class CH_MODELS_API Gator_EngineSimpleMap : public ChEngineSimpleMap {
  public:
    Gator_EngineSimpleMap(const std::string& name);
    double GetMaxEngineSpeed() override;
    void SetEngineTorqueMaps(ChFunction_Recorder& map0,  ///< [out] engine map at zero throttle
                             ChFunction_Recorder& mapF   ///< [out] engine map at full throttle
                             ) override;
};

/// @} vehicle_models_gator

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono

#endif  // GATOR_ENGINESIMPLEMAP_H
