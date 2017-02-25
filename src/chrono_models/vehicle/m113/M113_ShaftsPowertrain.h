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
// Authors:
// =============================================================================
//
// M113 powertrain model based on ChShaft objects.
//
// =============================================================================

#ifndef M113_SHAFTS_POWERTRAIN_H
#define M113_SHAFTS_POWERTRAIN_H

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/powertrain/ChShaftsPowertrain.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

class CH_MODELS_API M113_ShaftsPowertrain : public ChShaftsPowertrain {
  public:
    M113_ShaftsPowertrain();

    ~M113_ShaftsPowertrain() {}

    virtual void SetGearRatios(std::vector<double>& gear_ratios) override;

    virtual double GetMotorBlockInertia() const override { return m_motorblock_inertia; }
    virtual double GetCrankshaftInertia() const override { return m_crankshaft_inertia; }
    virtual double GetIngearShaftInertia() const override { return m_ingear_shaft_inertia; }

    virtual void SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) override;
    virtual void SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) override;
    virtual void SetTorqueConverterCapacityFactorMap(std::shared_ptr<ChFunction_Recorder>& map) override;
    virtual void SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunction_Recorder>& map) override;

  private:
    // Shaft inertias.
    static const double m_motorblock_inertia;
    static const double m_crankshaft_inertia;
    static const double m_ingear_shaft_inertia;
};

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
