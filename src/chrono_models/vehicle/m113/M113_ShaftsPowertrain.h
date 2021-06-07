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

/// @addtogroup vehicle_models_m113
/// @{

/// Shafts-based powertrain model for the M113 vehicle.
class CH_MODELS_API M113_ShaftsPowertrain : public ChShaftsPowertrain {
  public:
    M113_ShaftsPowertrain(const std::string& name);

    ~M113_ShaftsPowertrain() {}

    virtual void SetGearRatios(std::vector<double>& fwd, double& rev) override;

    virtual double GetMotorBlockInertia() const override { return m_motorblock_inertia; }
    virtual double GetCrankshaftInertia() const override { return m_crankshaft_inertia; }
    virtual double GetIngearShaftInertia() const override { return m_ingear_shaft_inertia; }

    virtual double GetUpshiftRPM() const override { return m_upshift_RPM; }
    virtual double GetDownshiftRPM() const override { return m_downshift_RPM; }

    virtual void SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) override;
    virtual void SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) override;
    virtual void SetTorqueConverterCapacityFactorMap(std::shared_ptr<ChFunction_Recorder>& map) override;
    virtual void SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunction_Recorder>& map) override;

  private:
    // Shaft inertias.
    static const double m_motorblock_inertia;
    static const double m_crankshaft_inertia;
    static const double m_ingear_shaft_inertia;

    // Gear shifting characteristics
    static const double m_upshift_RPM;
    static const double m_downshift_RPM;
};

/// @} vehicle_models_m113

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
