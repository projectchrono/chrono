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
// Authors: Radu Serban, Asher Elmquist, Rainer Gericke
// =============================================================================
//
// Jeep Cherokee 1997 automatic transmission model based on ChShaft objects.
// Vehicle Parameters taken from SAE Paper 1999-01-0121
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#ifndef CHRONO_CHEROKEE_AUTOMATICTRANSMISSIONSTAFTS_H
#define CHRONO_CHEROKEE_AUTOMATICTRANSMISSIONSTAFTS_H

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/powertrain/ChAutomaticTransmissionShafts.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace jeep {

/// @addtogroup vehicle_models_cherokee
/// @{

/// Shafts-based powertrain model for the Cherokee vehicle.
class CH_MODELS_API Cherokee_AutomaticTransmissionShafts : public ChAutomaticTransmissionShafts {
  public:
    Cherokee_AutomaticTransmissionShafts(const std::string& name);
    ~Cherokee_AutomaticTransmissionShafts() {}

    virtual void SetGearRatios(std::vector<double>& fwd, double& rev) override;

    virtual double GetTransmissionBlockInertia() const override { return m_transmissionblock_inertia; }
    virtual double GetIngearShaftInertia() const override { return m_ingear_shaft_inertia; }
    virtual double GetMotorshaftInertia() const override { return m_motorshaft_inertia; }
    virtual double GetDriveshaftInertia() const override { return m_driveshaft_inertia; }

    virtual double GetUpshiftRPM() const override { return m_upshift_RPM; }
    virtual double GetDownshiftRPM() const override { return m_downshift_RPM; }

    virtual void SetTorqueConverterCapacityFactorMap(std::shared_ptr<ChFunctionInterp>& map) override;
    virtual void SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunctionInterp>& map) override;

  private:
    // Shaft inertias.
    static const double m_transmissionblock_inertia;
    static const double m_motorshaft_inertia;
    static const double m_driveshaft_inertia;
    static const double m_ingear_shaft_inertia;

    // Gear shifting characteristics
    static const double m_upshift_RPM;
    static const double m_downshift_RPM;
};

/// @} vehicle_models_cherokee

}  // end namespace jeep
}  // end namespace vehicle
}  // end namespace chrono

#endif  // CHRONO_CHEROKEE_AUTOMATICTRANSMISSIONSTAFTS_H
