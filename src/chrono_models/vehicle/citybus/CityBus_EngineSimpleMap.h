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
// Authors: Radu Serban, Asher Elmquist, Evan Hoerl, Marcel Offermans
// =============================================================================
//
// Simple engine model for the CityBus vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
//
// =============================================================================

#ifndef CITYBUS_ENGINESIMPLEMAP_H
#define CITYBUS_ENGINESIMPLEMAP_H

#include "chrono_vehicle/powertrain/ChEngineSimpleMap.h"
#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace citybus {

/// @addtogroup vehicle_models_citybus
/// @{

/// Simple CityBus engine subsystem (based on engine speed-torque maps).
class CH_MODELS_API CityBus_EngineSimpleMap : public ChEngineSimpleMap {
  public:
    CityBus_EngineSimpleMap(const std::string& name);
    virtual double GetMaxEngineSpeed() override;
    virtual void SetEngineTorqueMaps(ChFunction_Recorder& map0,  ///< [out] engine map at zero throttle
                                     ChFunction_Recorder& mapF   ///< [out] engine map at full throttle
                                     ) override;
};

/// @} vehicle_models_citybus

}  // end namespace citybus
}  // end namespace vehicle
}  // end namespace chrono

#endif  // CITYBUS_ENGINESIMPLEMAP_H
