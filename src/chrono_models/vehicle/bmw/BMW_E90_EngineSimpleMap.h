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
// Authors: Radu Serban, Asher Elmquist, Marcel Offermans, Rainer Gericke
// =============================================================================
//
// Simple engine placeholder!! model for the BMW E90 (330i 2006) vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
//
// =============================================================================

#ifndef BMW_E90_ENGINESIMPLEMAP_H
#define BMW_E90_ENGINESIMPLEMAP_H

#include "chrono_vehicle/powertrain/ChEngineSimpleMap.h"
#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace bmw {

/// @addtogroup vehicle_models_bmw
/// @{

/// Simple Sedan powertrain subsystem (based on engine speed-torque maps).
class CH_MODELS_API BMW_E90_EngineSimpleMap : public ChEngineSimpleMap {
  public:
    BMW_E90_EngineSimpleMap(const std::string& name);

    /// Specify maximum engine speed.
    double GetMaxEngineSpeed() override;

    /// Set the engine speed-torque maps.
    /// A concrete class must add the speed-torque points to the provided maps,
    /// using the ChFunctionInterp::AddPoint() function.
    void SetEngineTorqueMaps(ChFunctionInterp& map0,  ///< [out] engine map at zero throttle
                             ChFunctionInterp& mapF   ///< [out] engine map at full throttle
                             ) override;
};

/// @} vehicle_models_bmw

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono

#endif
