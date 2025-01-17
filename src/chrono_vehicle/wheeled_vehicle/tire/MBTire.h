// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
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
// MB tire constructed with data from file (JSON format).
//
// =============================================================================

#ifndef MB_TIRE_H
#define MB_TIRE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChMBTire.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// MB tire constructed with data from file (JSON format).
class CH_VEHICLE_API MBTire : public ChMBTire {
  public:
    MBTire(const std::string& filename);
    MBTire(const rapidjson::Document& d);
    ~MBTire();

    /// Get the default tire pressure.
    virtual double GetDefaultPressure() const override { return m_default_pressure; }

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_default_pressure;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
