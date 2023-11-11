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
// Authors: Radu Serban
// =============================================================================
//
// Duro engine model based on ChShaft objects.
//
// =============================================================================

#ifndef DURO_ENGINE_SHAFTS_H
#define DURO_ENGINE_SHAFTS_H

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/powertrain/ChEngineShafts.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace duro {

/// @addtogroup vehicle_models_duro
/// @{

/// Shafts-based engine model for the Duro vehicle.
class CH_MODELS_API Duro_EngineShafts : public ChEngineShafts {
  public:
    Duro_EngineShafts(const std::string& name);

    ~Duro_EngineShafts() {}

    virtual double GetMotorBlockInertia() const override { return m_motorblock_inertia; }
    virtual double GetMotorshaftInertia() const override { return m_motorshaft_inertia; }

    virtual void SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) override;
    virtual void SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) override;

  private:
    // Shaft inertias.
    static const double m_motorblock_inertia;
    static const double m_motorshaft_inertia;
};

/// @} vehicle_models_duro

}  // namespace duro
}  // end namespace vehicle
}  // end namespace chrono

#endif
