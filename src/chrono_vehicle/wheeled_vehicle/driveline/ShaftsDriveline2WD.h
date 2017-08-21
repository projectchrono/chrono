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
// 2WD driveline model template based on ChShaft objects using data from file
// (JSON format).
//
// =============================================================================

#ifndef SHAFTS_DRIVELINE_2WD_H
#define SHAFTS_DRIVELINE_2WD_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline2WD.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_driveline
/// @{

/// 2WD driveline model template based on ChShaft objects using data from file (JSON format).
class CH_VEHICLE_API ShaftsDriveline2WD : public ChShaftsDriveline2WD {
  public:
    ShaftsDriveline2WD(const std::string& filename);
    ShaftsDriveline2WD(const rapidjson::Document& d);
    ~ShaftsDriveline2WD() {}

    virtual double GetDriveshaftInertia() const override { return m_driveshaft_inertia; }
    virtual double GetDifferentialBoxInertia() const override { return m_differentialbox_inertia; }

    virtual double GetConicalGearRatio() const override { return m_conicalgear_ratio; }
    virtual double GetDifferentialRatio() const override { return m_differential_ratio; }

  private:
    void Create(const rapidjson::Document& d);

    // Shaft inertias.
    double m_driveshaft_inertia;
    double m_differentialbox_inertia;

    // Gear ratios.
    double m_conicalgear_ratio;
    double m_differential_ratio;
};

/// @} vehicle_wheeled_driveline

}  // end namespace vehicle
}  // end namespace chrono

#endif
