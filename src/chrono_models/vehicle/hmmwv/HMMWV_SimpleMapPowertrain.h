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
// Simple powertrain model for the HMMWV vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// =============================================================================

#ifndef HMMWV_SIMPLEMAP_POWERTRAIN_H
#define HMMWV_SIMPLEMAP_POWERTRAIN_H

#include "chrono_vehicle/powertrain/ChSimpleMapPowertrain.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// Simple HMMWV powertrain subsystem (based on engine speed-torque maps).
class CH_MODELS_API HMMWV_SimpleMapPowertrain : public ChSimpleMapPowertrain {
  public:
    HMMWV_SimpleMapPowertrain(const std::string& name);

    /// Specify maximum engine speed.
    virtual double GetMaxEngineSpeed() override;

    /// Set the engine speed-torque maps.
    /// A concrete class must add the speed-torque points to the provided maps,
    /// using the ChFunction_Recorder::AddPoint() function.
    virtual void SetEngineTorqueMaps(ChFunction_Recorder& map0,  ///< [out] engine map at zero throttle
                                     ChFunction_Recorder& mapF   ///< [out] engine map at full throttle
                                     ) override;

    /// Set the gears, i.e. the transmission ratios of the various gears.
    /// A concrete class must populate the vector of forward gear ratios, ordered as 1st, 2nd, etc.
    /// and provide a value for the single reverse gear ratio.
    virtual void SetGearRatios(std::vector<double>& fwd_gear_ratios,  ///< [out] list of forward gear ratios
                               double& reverse_gear_ratio             ///< [out] single reverse gear ratio
                               ) override;

    /// Set the ideal shift points for automatic gear shifting.
    /// For each forward gear, specify a pair (min, max) with the minimum and
    /// maximum engine speed for shifting (down and up, respectively).
    virtual void SetShiftPoints(
        std::vector<std::pair<double, double>>& shift_bands  ///< [out] down-shift/up-shift points
        ) override;
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif
