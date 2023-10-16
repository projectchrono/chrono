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
// Authors: Radu Serban, Jayne Henry
// =============================================================================
//
// Simple engine model for the ARTcar vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
//
// =============================================================================

#ifndef ARTCAR_ENGINESIMPLEMAP_H
#define ARTCAR_ENGINESIMPLEMAP_H

#include "chrono_vehicle/powertrain/ChEngineSimpleMap.h"
#include "chrono_models/ChApiModels.h"

using namespace chrono::vehicle;
using namespace chrono;

namespace chrono {
namespace vehicle {
namespace artcar {

/// @addtogroup vehicle_models_artcar
/// @{

/// Simple ARTcar powertrain subsystem (based on engine speed-torque maps).
class CH_MODELS_API ARTcar_EngineSimpleMap : public ChEngineSimpleMap {
  public:
    ARTcar_EngineSimpleMap(const std::string& name);

    /// Specify maximum engine speed.
    double GetMaxEngineSpeed() override;

    /// Set the engine speed-torque maps.
    /// A concrete class must add the speed-torque points to the provided maps,
    /// using the ChFunction_Recorder::AddPoint() function.
    void SetEngineTorqueMaps(ChFunction_Recorder& map0,  ///< [out] engine map at zero throttle
                             ChFunction_Recorder& mapF   ///< [out] engine map at full throttle
                             ) override;

  private:
    double m_voltage_ratio;
    double m_Kv_rating;
    double m_supply_voltage;
    double m_stall_torque;

    friend class ARTcar;
};

/// @} vehicle_models_artcar

}  // end namespace artcar
}  // namespace vehicle
}  // namespace chrono

#endif
