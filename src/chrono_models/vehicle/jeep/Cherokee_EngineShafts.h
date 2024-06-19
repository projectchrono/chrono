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
// Authors: Radu Serban, Asher Elmquist, Rainer Gericke
// =============================================================================
//
// Jeep Cherokee 4WD engine model based on ChShaft objects.
// Engine data taken from:
// https://www.automobile-catalog.com/curve/2006/1317620/jeep_wrangler_sport_4_0l.html#gsc.tab=0
//
// =============================================================================

#ifndef CHRONO_CHEROKEE_ENGINESHAFTS_H
#define CHRONO_CHEROKEE_ENGINESHAFTS_H

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/powertrain/ChEngineShafts.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace jeep {

/// @addtogroup vehicle_models_cherokee
/// @{

/// Shafts-based engine model for the Jeep Cherokee vehicle.
class CH_MODELS_API Cherokee_EngineShafts : public ChEngineShafts {
  public:
    Cherokee_EngineShafts(const std::string& name);

    ~Cherokee_EngineShafts() {}

    virtual double GetMotorBlockInertia() const override { return m_motorblock_inertia; }
    virtual double GetMotorshaftInertia() const override { return m_motorshaft_inertia; }

    virtual void SetEngineTorqueMap(std::shared_ptr<ChFunctionInterp>& map) override;
    virtual void SetEngineLossesMap(std::shared_ptr<ChFunctionInterp>& map) override;

  private:
    // Shaft inertias.
    static const double m_motorblock_inertia;
    static const double m_motorshaft_inertia;
};

/// @} vehicle_models_cherokee

}  // end namespace jeep
}  // end namespace vehicle
}  // end namespace chrono

#endif  // CHRONO_CHEROKEE_ENGINESHAFTS_H
